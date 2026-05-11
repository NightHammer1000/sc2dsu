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
const HEADER_LEN_FULL: usize = 20;
const CLIENT_TIMEOUT: Duration = Duration::from_secs(5);
const CLEANUP_INTERVAL: Duration = Duration::from_secs(1);
const RECV_TIMEOUT: Duration = Duration::from_millis(2);

#[allow(dead_code)]
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
const OUR_MAC: [u8; 6] = [0x02, 0x28, 0xDE, 0x13, 0x04, OUR_SLOT];

struct Subscriber {
    addr: SocketAddr,
    last_request: Instant,
    packet_counter: u32,
}

pub struct Server {
    socket: UdpSocket,
    server_id: u32,
    subscribers: HashMap<u32, Subscriber>,
    dsu_wants_device: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    sample_rx: Receiver<ImuSample>,
    last_sample: Option<ImuSample>,
    last_cleanup: Instant,
    last_stats: Instant,
    samples_in_window: u32,
    packets_in_window: u32,
    requests_in_window: u32,
    orientation_q: [f32; 4],
    last_sample_at: Option<Instant>,
}

impl Server {
    pub fn bind(
        port: u16,
        expose_to_network: bool,
        dsu_wants_device: Arc<AtomicBool>,
        shutdown: Arc<AtomicBool>,
        sample_rx: Receiver<ImuSample>,
    ) -> io::Result<Self> {
        let host = if expose_to_network {
            "0.0.0.0"
        } else {
            "127.0.0.1"
        };
        let socket = UdpSocket::bind((host, port))?;
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
            loop {
                match self.sample_rx.try_recv() {
                    Ok(s) => {
                        self.last_sample = Some(s);
                        self.samples_in_window += 1;
                        self.broadcast_data_packet(&s);
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

            if self.last_cleanup.elapsed() >= CLEANUP_INTERVAL {
                self.cleanup_subscribers();
                self.last_cleanup = Instant::now();
            }

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
            _ => {}
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

        let wants_us = reg_type == 0
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
        let mut out = vec![0u8; HEADER_LEN_FULL + 12];
        write_header(&mut out, self.server_id, msg_type::PORTS);
        write_controller_header(&mut out[HEADER_LEN_FULL..], slot);
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
        let body = &mut out[HEADER_LEN_FULL + 11..];
        body[0] = 1;
        body[9] = 127;
        body[10] = 127;
        body[11] = 127;
        body[12] = 127;
        body[37..45].copy_from_slice(&(sample.timestamp_us as u64).to_le_bytes());
        body[45..49].copy_from_slice(&sample.accel_g[0].to_le_bytes());
        body[49..53].copy_from_slice(&sample.accel_g[1].to_le_bytes());
        body[53..57].copy_from_slice(&sample.accel_g[2].to_le_bytes());
        body[57..61].copy_from_slice(&sample.gyro_dps[0].to_le_bytes());
        body[61..65].copy_from_slice(&sample.gyro_dps[1].to_le_bytes());
        body[65..69].copy_from_slice(&sample.gyro_dps[2].to_le_bytes());

        for sub in self.subscribers.values_mut() {
            sub.packet_counter = sub.packet_counter.wrapping_add(1);
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
    out[8..12].fill(0);
    out[12..16].copy_from_slice(&server_id.to_le_bytes());
    out[16..20].copy_from_slice(&mtype.to_le_bytes());
}

fn finalize_crc(out: &mut [u8]) {
    out[8..12].fill(0);
    let crc = crc32fast::hash(out);
    out[8..12].copy_from_slice(&crc.to_le_bytes());
}

fn write_controller_header(buf: &mut [u8], slot: u8) {
    buf[0] = slot;
    if slot == OUR_SLOT {
        buf[1] = slot_state::CONNECTED;
        buf[2] = device_type::GYRO_FULL;
        buf[3] = connection_type::USB;
        buf[4..10].copy_from_slice(&OUR_MAC);
        buf[10] = BATTERY_NA;
    }
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

#[allow(dead_code)]
fn _force_write_use(_w: &mut dyn Write) {}
