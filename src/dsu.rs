// DSU/Cemuhook UDP server (port 26760).
//
// Spec source: v1993/gcemuhook (the protocol library used by evdevhook2). Verified
// against Cemu and Eden clients.
//
// Lazy device control: this server toggles a shared `device_active` flag — true when
// at least one client is subscribed for our slot, false otherwise. The Triton reader
// thread owns the HID handle and opens/closes it in response. With no clients, the
// HID device stays closed and the controller is free to enter standby.

use crate::stats;
use crate::triton::ImuSample;
use std::collections::HashMap;
use std::io::{self, Cursor, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Duration, Instant};

const PROTOCOL_VERSION: u16 = 1001;
const MAGIC_SERVER: &[u8; 4] = b"DSUS";
const MAGIC_CLIENT: &[u8; 4] = b"DSUC";
const HEADER_LEN: usize = 16;
const HEADER_LEN_FULL: usize = 20; // header + message-type field
const CLIENT_TIMEOUT: Duration = Duration::from_secs(5);
const CLEANUP_INTERVAL: Duration = Duration::from_secs(1);
const RECV_TIMEOUT: Duration = Duration::from_millis(2);

#[allow(dead_code)] // EXT_RUMBLE_* defined for completeness; we don't expose rumble yet.
mod msg_type {
    pub const VERSION: u32 = 0x100000;
    pub const PORTS: u32 = 0x100001;
    pub const DATA: u32 = 0x100002;
    pub const EXT_RUMBLE_INFO: u32 = 0x110001;
    pub const EXT_RUMBLE_SET: u32 = 0x110002;
}

mod slot_state {
    pub const CONNECTED: u8 = 2;
}
mod device_type {
    pub const GYRO_FULL: u8 = 2;
}
mod connection_type {
    pub const USB: u8 = 1;
}
const BATTERY_NA: u8 = 0;

const OUR_SLOT: u8 = 0;
/// Stable locally-administered MAC: 02:28:DE:13:04:00 — encodes Valve VID + Triton-family
/// PID base + slot 0. Cemu/Eden don't validate the MAC value, just need it to be stable.
const OUR_MAC: [u8; 6] = [0x02, 0x28, 0xDE, 0x13, 0x04, OUR_SLOT];

/// A subscribed DSU client. Identified by (client_id, addr) — the same client_id can
/// subscribe from multiple addrs (rare), but typically each client uses a unique id.
struct Subscriber {
    addr: SocketAddr,
    last_request: Instant,
    /// Per-client outgoing packet counter. Incremented on every DATA packet we send.
    packet_counter: u32,
}

pub struct Server {
    socket: UdpSocket,
    server_id: u32,
    subscribers: HashMap<u32, Subscriber>,
    /// Set when at least one DSU subscriber is active. The device thread also
    /// honours a parallel `ui_wants_device` flag, so the controller may be open
    /// even when this is false (e.g. the user opened the settings window).
    dsu_wants_device: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    sample_rx: Receiver<ImuSample>,
    /// Last sample received — broadcast on every iteration the controller is awake.
    last_sample: Option<ImuSample>,
    last_cleanup: Instant,
    // Stats
    last_stats: Instant,
    samples_in_window: u32,
    packets_in_window: u32,
    requests_in_window: u32,
    // Orientation integrator
    orientation_q: [f32; 4],
    last_sample_at: Option<Instant>,
}

impl Server {
    pub fn bind(
        port: u16,
        dsu_wants_device: Arc<AtomicBool>,
        shutdown: Arc<AtomicBool>,
        sample_rx: Receiver<ImuSample>,
    ) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", port))?;
        socket.set_read_timeout(Some(RECV_TIMEOUT))?;
        let server_id = rand_u32();
        Ok(Self {
            socket,
            server_id,
            subscribers: HashMap::new(),
            dsu_wants_device,
            shutdown,
            sample_rx,
            last_sample: None,
            last_cleanup: Instant::now(),
            last_stats: Instant::now(),
            samples_in_window: 0,
            packets_in_window: 0,
            requests_in_window: 0,
            orientation_q: [1.0, 0.0, 0.0, 0.0],
            last_sample_at: None,
        })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    pub fn server_id(&self) -> u32 {
        self.server_id
    }

    pub fn run(&mut self) -> io::Result<()> {
        let mut buf = [0u8; 2048];
        while !self.shutdown.load(Ordering::Relaxed) {
            // 1) Drain pending IMU samples and broadcast each one. Emulators expect
            //    one DATA packet per IMU tick (~250 Hz on Triton); deduping makes the
            //    motion feel choppy.
            loop {
                match self.sample_rx.try_recv() {
                    Ok(s) => {
                        self.last_sample = Some(s);
                        self.samples_in_window += 1;
                        self.broadcast_data_packet(&s);
                        // Integrate gyro into the orientation quaternion. Real-time dt
                        // from wall-clock; capped at 100 ms to absorb scheduling gaps
                        // (e.g. the device-thread waking the controller from standby).
                        // Honour any pending recenter request from the UI.
                        if stats::RECENTER_REQUEST.swap(false, Ordering::Relaxed) {
                            self.orientation_q = [1.0, 0.0, 0.0, 0.0];
                            self.last_sample_at = None;
                        }
                        let now = Instant::now();
                        let dt = match self.last_sample_at {
                            Some(t) => now.duration_since(t).as_secs_f32().min(0.1),
                            None => 0.0,
                        };
                        self.last_sample_at = Some(now);
                        if dt > 0.0 {
                            integrate_gyro(&mut self.orientation_q, s.gyro_dps, dt);
                        }
                        stats::publish_motion(s.gyro_dps, s.accel_g, self.orientation_q);
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        eprintln!("dsu: device thread channel closed, shutting down");
                        return Ok(());
                    }
                }
            }

            // 3) Process at most one inbound datagram, then loop. UDP recv errors are
            //    *never* fatal: on Windows the OS forwards ICMP "port unreachable" from
            //    a previous send_to as a ConnectionReset on the *next* recv_from, which
            //    is the canonical UDP gotcha. Log and continue.
            match self.socket.recv_from(&mut buf) {
                Ok((n, src)) => {
                    self.requests_in_window += 1;
                    if let Err(e) = self.handle_request(&buf[..n], src) {
                        eprintln!("dsu: request parse error from {src}: {e}");
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {}
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => {
                    eprintln!("dsu: recv non-fatal error: {} ({:?})", e, e.kind());
                }
            }

            // 4) Periodic subscriber cleanup + device-active flag toggle.
            if self.last_cleanup.elapsed() >= CLEANUP_INTERVAL {
                self.cleanup_subscribers();
                self.last_cleanup = Instant::now();
            }

            // 5) Periodic stats — both to stderr and to the shared LIVE struct
            //    that the UI thread reads at ~30 Hz for live display.
            if self.last_stats.elapsed() >= Duration::from_secs(1) {
                let secs = self.last_stats.elapsed().as_secs_f32();
                let (gyro, accel) = self
                    .last_sample
                    .as_ref()
                    .map(|s| (s.gyro_dps, s.accel_g))
                    .unwrap_or(([0.0; 3], [0.0; 3]));
                let gmag = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
                let device_active_now = self.dsu_wants_device.load(Ordering::Relaxed);
                eprintln!(
                    "stats: subs={:>2} reqs={:>3} ({:>5.1}/s)  imu={:>4} ({:>5.1}/s)  pkt={:>4} ({:>5.1}/s)  |gyro|={:>5.1}dps  active={}",
                    self.subscribers.len(),
                    self.requests_in_window,
                    self.requests_in_window as f32 / secs,
                    self.samples_in_window,
                    self.samples_in_window as f32 / secs,
                    self.packets_in_window,
                    self.packets_in_window as f32 / secs,
                    gmag,
                    device_active_now,
                );
                stats::publish(stats::ServerStats {
                    subscribers: self.subscribers.len(),
                    requests_per_sec: self.requests_in_window as f32 / secs,
                    samples_per_sec: self.samples_in_window as f32 / secs,
                    packets_per_sec: self.packets_in_window as f32 / secs,
                    last_gyro_dps: gyro,
                    last_accel_g: accel,
                    orientation: self.orientation_q,
                    device_active: device_active_now,
                    server_id: self.server_id,
                    bound_port: self.socket.local_addr().map(|a| a.port()).unwrap_or(0),
                });
                self.requests_in_window = 0;
                self.samples_in_window = 0;
                self.packets_in_window = 0;
                self.last_stats = Instant::now();
            }
        }
        Ok(())
    }

    fn handle_request(&mut self, msg: &[u8], src: SocketAddr) -> io::Result<()> {
        if msg.len() < HEADER_LEN_FULL {
            return Ok(());
        }
        let mut c = Cursor::new(msg);
        let mut magic = [0u8; 4];
        c.read_exact(&mut magic)?;
        if &magic != MAGIC_CLIENT {
            return Ok(());
        }
        if read_u16_le(&mut c)? != PROTOCOL_VERSION {
            return Ok(());
        }
        let length = read_u16_le(&mut c)? as usize;
        if msg.len() < HEADER_LEN + length {
            return Ok(());
        }

        // CRC32 of the whole packet with the CRC field zeroed.
        let claimed_crc = read_u32_le(&mut c)?;
        let mut crc_buf = msg[..HEADER_LEN + length].to_vec();
        crc_buf[8..12].fill(0);
        let computed = crc32fast::hash(&crc_buf);
        if claimed_crc != computed {
            return Ok(());
        }

        let client_id = read_u32_le(&mut c)?;
        let mtype = read_u32_le(&mut c)?;

        match mtype {
            msg_type::VERSION => self.send_version(src)?,
            msg_type::PORTS => self.handle_ports(&mut c, src)?,
            msg_type::DATA => self.handle_data_request(&mut c, src, client_id)?,
            _ => {
                // Ignore EXT_RUMBLE_* and unknown types for now.
            }
        }
        Ok(())
    }

    fn handle_ports(&mut self, c: &mut Cursor<&[u8]>, src: SocketAddr) -> io::Result<()> {
        let amount = read_u32_le(c)?.min(4) as usize;
        let mut requested = vec![0u8; amount];
        c.read_exact(&mut requested)?;
        for slot in requested {
            self.send_slot_info(src, slot)?;
        }
        Ok(())
    }

    fn handle_data_request(
        &mut self,
        c: &mut Cursor<&[u8]>,
        src: SocketAddr,
        client_id: u32,
    ) -> io::Result<()> {
        let reg_type = read_u8(c)?;
        let slot = read_u8(c)?;
        let mut mac = [0u8; 6];
        c.read_exact(&mut mac)?;

        // Decide whether this client wants OUR slot.
        // RegistrationType bits: SLOT = 1<<0, MAC = 1<<1, ALL = 0 (no bits = subscribe to all).
        let wants_us = reg_type == 0 // ALL
            || (reg_type & 0b01 != 0 && slot == OUR_SLOT)
            || (reg_type & 0b10 != 0 && mac == OUR_MAC);
        if !wants_us {
            return Ok(());
        }

        let was_empty = self.subscribers.is_empty();
        self.subscribers
            .entry(client_id)
            .and_modify(|s| {
                s.addr = src;
                s.last_request = Instant::now();
            })
            .or_insert_with(|| Subscriber {
                addr: src,
                last_request: Instant::now(),
                packet_counter: 0,
            });
        if was_empty {
            self.dsu_wants_device.store(true, Ordering::Relaxed);
            eprintln!("dsu: first subscriber {client_id:08X} from {src} -> waking controller");
        }
        Ok(())
    }

    fn cleanup_subscribers(&mut self) {
        let was_empty = self.subscribers.is_empty();
        let now = Instant::now();
        self.subscribers
            .retain(|_, s| now.duration_since(s.last_request) < CLIENT_TIMEOUT);
        let is_empty = self.subscribers.is_empty();
        if !was_empty && is_empty {
            self.dsu_wants_device.store(false, Ordering::Relaxed);
            eprintln!("dsu: last subscriber timed out -> releasing controller");
        }
    }

    fn send_version(&self, src: SocketAddr) -> io::Result<()> {
        let mut out = vec![0u8; HEADER_LEN_FULL + 2];
        write_header(&mut out, self.server_id, msg_type::VERSION);
        out[HEADER_LEN_FULL..HEADER_LEN_FULL + 2].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        finalize_crc(&mut out);
        self.socket.send_to(&out, src)?;
        Ok(())
    }

    fn send_slot_info(&self, src: SocketAddr, slot: u8) -> io::Result<()> {
        // Always 12 bytes after the header: controller_header(11) + terminating zero(1).
        let mut out = vec![0u8; HEADER_LEN_FULL + 12];
        write_header(&mut out, self.server_id, msg_type::PORTS);
        write_controller_header(&mut out[HEADER_LEN_FULL..], slot);
        // Trailing byte is left zero by the buffer init.
        finalize_crc(&mut out);
        self.socket.send_to(&out, src)?;
        Ok(())
    }

    fn broadcast_data_packet(&mut self, sample: &ImuSample) {
        if self.subscribers.is_empty() {
            return;
        }
        let mut out = vec![0u8; HEADER_LEN_FULL + 80];
        write_header(&mut out, self.server_id, msg_type::DATA);
        write_controller_header(&mut out[HEADER_LEN_FULL..], OUR_SLOT);
        let body = &mut out[HEADER_LEN_FULL + 11..]; // after controller_header
        body[0] = 1; // connected
        // body[1..5] = packet_counter (per-client, filled in below)
        // Buttons + sticks + analog buttons + 2 touches: all zero, the virtual Xbox
        // surface from Steam Input is providing those; we only carry motion.
        // body[5..9]   = buttons1, buttons2, home, touch (zeros)
        // body[9..13]  = lx, ly, rx, ry (zero, but DSU clients expect 127 = neutral)
        body[9] = 127;
        body[10] = 127;
        body[11] = 127;
        body[12] = 127;
        // body[13..25] = 12 analog buttons (zero)
        // body[25..37] = 2 × touch (zero)
        // body[37..45] = motion timestamp u64 LE (microseconds)
        body[37..45].copy_from_slice(&(sample.timestamp_us as u64).to_le_bytes());
        // body[45..57] = accel x/y/z f32 LE (g)
        body[45..49].copy_from_slice(&sample.accel_g[0].to_le_bytes());
        body[49..53].copy_from_slice(&sample.accel_g[1].to_le_bytes());
        body[53..57].copy_from_slice(&sample.accel_g[2].to_le_bytes());
        // body[57..69] = gyro x/y/z f32 LE (deg/s)
        body[57..61].copy_from_slice(&sample.gyro_dps[0].to_le_bytes());
        body[61..65].copy_from_slice(&sample.gyro_dps[1].to_le_bytes());
        body[65..69].copy_from_slice(&sample.gyro_dps[2].to_le_bytes());
        // body[69..80] = trailing slack (we sized to 80; only 69 used). Per spec the DATA
        // payload is exactly 80 bytes — leave the rest zero.

        for sub in self.subscribers.values_mut() {
            sub.packet_counter = sub.packet_counter.wrapping_add(1);
            // Per-client counter goes at offset HEADER_LEN_FULL + 11 + 1 = 32
            out[32..36].copy_from_slice(&sub.packet_counter.to_le_bytes());
            finalize_crc(&mut out);
            match self.socket.send_to(&out, sub.addr) {
                Ok(_) => self.packets_in_window += 1,
                Err(e) => eprintln!("dsu: send to {} failed: {e}", sub.addr),
            }
        }
    }
}

fn write_header(out: &mut [u8], server_id: u32, mtype: u32) {
    out[0..4].copy_from_slice(MAGIC_SERVER);
    out[4..6].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    let payload_len = (out.len() - HEADER_LEN) as u16;
    out[6..8].copy_from_slice(&payload_len.to_le_bytes());
    out[8..12].fill(0); // CRC placeholder
    out[12..16].copy_from_slice(&server_id.to_le_bytes());
    out[16..20].copy_from_slice(&mtype.to_le_bytes());
}

fn finalize_crc(out: &mut [u8]) {
    out[8..12].fill(0);
    let crc = crc32fast::hash(out);
    out[8..12].copy_from_slice(&crc.to_le_bytes());
}

/// 11 bytes: slot id, slot state, device type, connection type, MAC[6], battery.
fn write_controller_header(buf: &mut [u8], slot: u8) {
    buf[0] = slot;
    if slot == OUR_SLOT {
        buf[1] = slot_state::CONNECTED;
        buf[2] = device_type::GYRO_FULL;
        buf[3] = connection_type::USB;
        buf[4..10].copy_from_slice(&OUR_MAC);
        buf[10] = BATTERY_NA;
    }
    // Other slots remain all-zero (NOT_CONNECTED).
}

fn read_u8(c: &mut Cursor<&[u8]>) -> io::Result<u8> {
    let mut b = [0u8; 1];
    c.read_exact(&mut b)?;
    Ok(b[0])
}

fn read_u16_le(c: &mut Cursor<&[u8]>) -> io::Result<u16> {
    let mut b = [0u8; 2];
    c.read_exact(&mut b)?;
    Ok(u16::from_le_bytes(b))
}

fn read_u32_le(c: &mut Cursor<&[u8]>) -> io::Result<u32> {
    let mut b = [0u8; 4];
    c.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

/// Integrate a gyro sample (deg/sec) into an orientation quaternion (w, x, y, z)
/// over `dt` seconds. Standard quaternion derivative: dQ/dt = ½ * Q * ω, where ω
/// is the angular velocity quaternion (0, gx, gy, gz) in rad/sec.
fn integrate_gyro(q: &mut [f32; 4], gyro_dps: [f32; 3], dt: f32) {
    let to_rad = std::f32::consts::PI / 180.0;
    let gx = gyro_dps[0] * to_rad;
    let gy = gyro_dps[1] * to_rad;
    let gz = gyro_dps[2] * to_rad;
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let dw = -0.5 * (x * gx + y * gy + z * gz) * dt;
    let dx = 0.5 * (w * gx + y * gz - z * gy) * dt;
    let dy = 0.5 * (w * gy - x * gz + z * gx) * dt;
    let dz = 0.5 * (w * gz + x * gy - y * gx) * dt;
    let nw = w + dw;
    let nx = x + dx;
    let ny = y + dy;
    let nz = z + dz;
    let mag = (nw * nw + nx * nx + ny * ny + nz * nz).sqrt();
    if mag > 1e-6 {
        q[0] = nw / mag;
        q[1] = nx / mag;
        q[2] = ny / mag;
        q[3] = nz / mag;
    }
}

fn rand_u32() -> u32 {
    // Not crypto — just needs to differ across server restarts so clients reconcile.
    use std::time::{SystemTime, UNIX_EPOCH};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    use std::hash::{Hash, Hasher};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .hash(&mut h);
    std::process::id().hash(&mut h);
    h.finish() as u32
}

// Suppress unused-import warning when compiled without io::Write usage above.
#[allow(dead_code)]
fn _force_write_use(_w: &mut dyn Write) {}
