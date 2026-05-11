use crate::config;
use crate::stats;
use crate::triton::{self, ControllerState};
use std::collections::HashMap;
use std::io::{self, Cursor, Read};
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
const STATS_INTERVAL: Duration = Duration::from_secs(1);
const RECV_TIMEOUT: Duration = Duration::from_millis(2);
const RECV_BUF_LEN: usize = 2048;
const MAX_SUBSCRIBERS: usize = 16;
const CRC_OFFSET: usize = 8;
const CRC_LEN: usize = 4;
const CONTROLLER_HEADER_LEN: usize = 11;

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
    sample_rx: Receiver<ControllerState>,
    last_gyro: [f32; 3],
    last_accel: [f32; 3],
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
        sample_rx: Receiver<ControllerState>,
    ) -> io::Result<Self> {
        let socket = UdpSocket::bind((config::bind_host(expose_to_network), port))?;
        socket.set_read_timeout(Some(RECV_TIMEOUT))?;
        let server_id = rand_u32();
        Ok(Self {
            socket,
            server_id,
            subscribers: HashMap::new(),
            dsu_wants_device,
            shutdown,
            sample_rx,
            last_gyro: [0.0; 3],
            last_accel: [0.0; 3],
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
        let mut buf = [0u8; RECV_BUF_LEN];
        while !self.shutdown.load(Ordering::Relaxed) {
            if !self.pump_samples() {
                return Ok(());
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

            if self.last_stats.elapsed() >= STATS_INTERVAL {
                self.emit_stats();
            }
        }
        Ok(())
    }

    fn pump_samples(&mut self) -> bool {
        loop {
            match self.sample_rx.try_recv() {
                Ok(s) => {
                    self.samples_in_window += 1;
                    self.last_gyro = s.imu.gyro_dps;
                    self.last_accel = s.imu.accel_g;
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
                        integrate_gyro(&mut self.orientation_q, s.imu.gyro_dps, dt);
                    }
                    stats::publish_motion(s.imu.gyro_dps, s.imu.accel_g, self.orientation_q);
                }
                Err(TryRecvError::Empty) => return true,
                Err(TryRecvError::Disconnected) => {
                    eprintln!("dsu: device thread channel closed, shutting down");
                    return false;
                }
            }
        }
    }

    fn emit_stats(&mut self) {
        let secs = self.last_stats.elapsed().as_secs_f32();
        let gyro = self.last_gyro;
        let accel = self.last_accel;
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
        if claimed_crc != crc_over_zeroed(&msg[..HEADER_LEN + length]) {
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
        if self.subscribers.len() >= MAX_SUBSCRIBERS && !self.subscribers.contains_key(&client_id) {
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

    fn broadcast_data_packet(&mut self, sample: &ControllerState) {
        if self.subscribers.is_empty() {
            return;
        }
        const PACKET_NUM_OFFSET: usize = HEADER_LEN_FULL + CONTROLLER_HEADER_LEN + 1;

        let mut out = vec![0u8; HEADER_LEN_FULL + 80];
        write_header(&mut out, self.server_id, msg_type::DATA);
        write_controller_header(&mut out[HEADER_LEN_FULL..], OUR_SLOT);
        write_data_body(&mut out[HEADER_LEN_FULL + CONTROLLER_HEADER_LEN..], sample);

        for sub in self.subscribers.values_mut() {
            sub.packet_counter = sub.packet_counter.wrapping_add(1);
            out[PACKET_NUM_OFFSET..PACKET_NUM_OFFSET + 4]
                .copy_from_slice(&sub.packet_counter.to_le_bytes());
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
    out[CRC_OFFSET..CRC_OFFSET + CRC_LEN].fill(0);
    out[12..16].copy_from_slice(&server_id.to_le_bytes());
    out[16..20].copy_from_slice(&mtype.to_le_bytes());
}

fn crc_over_zeroed(msg: &[u8]) -> u32 {
    let mut h = crc32fast::Hasher::new();
    h.update(&msg[..CRC_OFFSET]);
    h.update(&[0u8; CRC_LEN]);
    h.update(&msg[CRC_OFFSET + CRC_LEN..]);
    h.finalize()
}

fn finalize_crc(out: &mut [u8]) {
    let crc = crc_over_zeroed(out);
    out[CRC_OFFSET..CRC_OFFSET + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
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

const TRIGGER_DIGITAL_THRESHOLD: u8 = 200;

fn write_data_body(body: &mut [u8], state: &ControllerState) {
    const B_CONNECTED: usize = 0;
    const B_BUTTONS: usize = 5;
    const B_STICKS: usize = 9;
    const B_ANALOG: usize = 13;
    const B_TIMESTAMP: usize = 37;
    const B_MOTION: usize = 45;

    body[B_CONNECTED] = 1;

    let l2 = trigger_to_u8(state.trigger_left);
    let r2 = trigger_to_u8(state.trigger_right);
    body[B_BUTTONS..B_BUTTONS + 4].copy_from_slice(&dsu_button_bytes(
        state.buttons,
        l2 >= TRIGGER_DIGITAL_THRESHOLD,
        r2 >= TRIGGER_DIGITAL_THRESHOLD,
    ));

    body[B_STICKS] = stick_to_u8(state.left_stick[0]);
    body[B_STICKS + 1] = stick_to_u8(state.left_stick[1]);
    body[B_STICKS + 2] = stick_to_u8(state.right_stick[0]);
    body[B_STICKS + 3] = stick_to_u8(state.right_stick[1]);

    let pressed = |mask: u32| state.buttons & mask != 0;
    let full = |on: bool| if on { u8::MAX } else { 0 };
    let analog: [u8; 12] = {
        use triton::button as bt;
        [
            full(pressed(bt::DPAD_LEFT)),
            full(pressed(bt::DPAD_DOWN)),
            full(pressed(bt::DPAD_RIGHT)),
            full(pressed(bt::DPAD_UP)),
            full(pressed(bt::Y)),
            full(pressed(bt::B)),
            full(pressed(bt::A)),
            full(pressed(bt::X)),
            full(pressed(bt::R)),
            full(pressed(bt::L)),
            r2,
            l2,
        ]
    };
    body[B_ANALOG..B_ANALOG + analog.len()].copy_from_slice(&analog);

    body[B_TIMESTAMP..B_TIMESTAMP + 8]
        .copy_from_slice(&(state.imu.timestamp_us as u64).to_le_bytes());
    let motion = [
        state.imu.accel_g[0],
        state.imu.accel_g[1],
        state.imu.accel_g[2],
        state.imu.gyro_dps[0],
        state.imu.gyro_dps[1],
        state.imu.gyro_dps[2],
    ];
    for (i, v) in motion.iter().enumerate() {
        let off = B_MOTION + i * 4;
        body[off..off + 4].copy_from_slice(&v.to_le_bytes());
    }
}

fn dsu_button_bytes(buttons: u32, l2_down: bool, r2_down: bool) -> [u8; 4] {
    use triton::button as bt;
    let down = |mask: u32| buttons & mask != 0;
    let buttons1 = pack_bits(&[
        (down(bt::MENU), 0),
        (down(bt::L3), 1),
        (down(bt::R3), 2),
        (down(bt::VIEW), 3),
        (down(bt::DPAD_UP), 4),
        (down(bt::DPAD_RIGHT), 5),
        (down(bt::DPAD_DOWN), 6),
        (down(bt::DPAD_LEFT), 7),
    ]);
    let buttons2 = pack_bits(&[
        (l2_down, 0),
        (r2_down, 1),
        (down(bt::L), 2),
        (down(bt::R), 3),
        (down(bt::X), 4),
        (down(bt::A), 5),
        (down(bt::B), 6),
        (down(bt::Y), 7),
    ]);
    [
        buttons1,
        buttons2,
        u8::from(down(bt::STEAM)),
        u8::from(down(bt::QAM)),
    ]
}

fn pack_bits(bits: &[(bool, u8)]) -> u8 {
    bits.iter()
        .fold(0u8, |acc, &(on, pos)| acc | (u8::from(on) << pos))
}

fn stick_to_u8(v: i16) -> u8 {
    ((i32::from(v) + 32768) >> 8) as u8
}

fn trigger_to_u8(v: u16) -> u8 {
    (v >> 7).min(255) as u8
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_fields_and_crc_are_consistent() {
        let mut out = vec![0u8; HEADER_LEN_FULL + 2];
        write_header(&mut out, 0xDEAD_BEEF, msg_type::VERSION);
        out[HEADER_LEN_FULL..HEADER_LEN_FULL + 2].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        finalize_crc(&mut out);
        assert_eq!(&out[0..4], MAGIC_SERVER);
        assert_eq!(u16::from_le_bytes([out[4], out[5]]), PROTOCOL_VERSION);
        assert_eq!(
            u16::from_le_bytes([out[6], out[7]]),
            (out.len() - HEADER_LEN) as u16
        );
        assert_eq!(
            u32::from_le_bytes(out[12..16].try_into().unwrap()),
            0xDEAD_BEEF
        );
        assert_eq!(
            u32::from_le_bytes(out[16..20].try_into().unwrap()),
            msg_type::VERSION
        );
        let claimed = u32::from_le_bytes(out[CRC_OFFSET..CRC_OFFSET + CRC_LEN].try_into().unwrap());
        assert_eq!(claimed, crc_over_zeroed(&out));
    }

    #[test]
    fn crc_over_zeroed_ignores_crc_field() {
        let a: Vec<u8> = (0..32u8).collect();
        let mut b = a.clone();
        b[CRC_OFFSET..CRC_OFFSET + CRC_LEN].copy_from_slice(&[0xFF; CRC_LEN]);
        assert_eq!(crc_over_zeroed(&a), crc_over_zeroed(&b));
    }

    #[test]
    fn stick_and_trigger_scaling() {
        assert_eq!(stick_to_u8(0), 128);
        assert_eq!(stick_to_u8(i16::MIN), 0);
        assert_eq!(stick_to_u8(i16::MAX), 255);
        assert_eq!(trigger_to_u8(0), 0);
        assert_eq!(trigger_to_u8(0x7FFF), 255);
        assert_eq!(trigger_to_u8(0x8000), 255);
        let mid = trigger_to_u8(0x4000);
        assert!((100..160).contains(&mid), "mid pull = {mid}");
    }

    #[test]
    fn dsu_button_bytes_maps_buttons_to_correct_bits() {
        use crate::triton::button as bt;
        let bit = |n: u8| 1u8 << n;
        let [b1, b2, home, touch] = dsu_button_bytes(bt::A | bt::DPAD_UP | bt::STEAM, false, false);
        assert_eq!(b1, bit(4));
        assert_eq!(b2, bit(5));
        assert_eq!(home, 1);
        assert_eq!(touch, 0);

        let [b1, b2, home, touch] = dsu_button_bytes(
            bt::MENU | bt::VIEW | bt::L | bt::R | bt::Y | bt::QAM,
            true,
            true,
        );
        assert_eq!(b1, bit(0) | bit(3));
        assert_eq!(b2, bit(0) | bit(1) | bit(2) | bit(3) | bit(7));
        assert_eq!(home, 0);
        assert_eq!(touch, 1);

        assert_eq!(
            dsu_button_bytes(bt::L4 | bt::L5 | bt::R4 | bt::R5, false, false),
            [0, 0, 0, 0]
        );
    }

    #[test]
    fn data_body_encodes_buttons_sticks_and_motion() {
        let mut body = vec![0u8; 80];
        let state = ControllerState {
            buttons: triton::button::B | triton::button::DPAD_LEFT,
            trigger_left: 0x7FFF,
            trigger_right: 0,
            left_stick: [i16::MAX, i16::MIN],
            right_stick: [0, 0],
            imu: triton::ImuSample {
                timestamp_us: 0xABCD_1234,
                accel_g: [0.25, -0.5, 1.0],
                gyro_dps: [10.0, -20.0, 30.0],
            },
        };
        write_data_body(&mut body, &state);
        assert_eq!(body[0], 1);
        assert_eq!(body[5], 0b1000_0000);
        assert_eq!(body[6], 0b0100_0001);
        assert_eq!(body[7], 0);
        assert_eq!(body[8], 0);
        assert_eq!(&body[9..13], &[255, 0, 128, 128]);
        assert_eq!(&body[13..25], &[255, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 255]);
        assert_eq!(&body[25..37], &[0u8; 12]);
        assert_eq!(
            u64::from_le_bytes(body[37..45].try_into().unwrap()),
            0xABCD_1234u64
        );
        assert_eq!(f32::from_le_bytes(body[45..49].try_into().unwrap()), 0.25);
        assert_eq!(f32::from_le_bytes(body[49..53].try_into().unwrap()), -0.5);
        assert_eq!(f32::from_le_bytes(body[53..57].try_into().unwrap()), 1.0);
        assert_eq!(f32::from_le_bytes(body[57..61].try_into().unwrap()), 10.0);
        assert_eq!(f32::from_le_bytes(body[61..65].try_into().unwrap()), -20.0);
        assert_eq!(f32::from_le_bytes(body[65..69].try_into().unwrap()), 30.0);
    }

    #[test]
    fn partial_trigger_pull_sets_analog_but_not_digital() {
        let mut body = vec![0u8; 80];
        let state = ControllerState {
            buttons: 0,
            trigger_left: 0x3200,
            trigger_right: 0,
            left_stick: [0, 0],
            right_stick: [0, 0],
            imu: triton::ImuSample {
                timestamp_us: 0,
                accel_g: [0.0; 3],
                gyro_dps: [0.0; 3],
            },
        };
        assert!(trigger_to_u8(state.trigger_left) < TRIGGER_DIGITAL_THRESHOLD);
        write_data_body(&mut body, &state);
        assert_eq!(body[6], 0);
        assert_eq!(body[24], trigger_to_u8(0x3200));
    }

    #[test]
    fn integrate_gyro_keeps_quaternion_normalized() {
        let mut q = [1.0f32, 0.0, 0.0, 0.0];
        for _ in 0..1000 {
            integrate_gyro(&mut q, [45.0, -30.0, 15.0], 0.01);
        }
        let mag = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((mag - 1.0).abs() < 1e-3, "magnitude drifted: {mag}");
    }

    #[test]
    fn integrate_gyro_zero_rate_is_identity() {
        let mut q = [1.0f32, 0.0, 0.0, 0.0];
        integrate_gyro(&mut q, [0.0, 0.0, 0.0], 0.01);
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
    }
}
