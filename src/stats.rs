// ServerStats is split into sections by owning subsystem. Each writer can
// only mutate its own section, which structurally prevents one writer from
// clobbering fields it doesn't own (the bug the old flat `publish()` had).

use std::sync::RwLock;
use std::sync::atomic::AtomicBool;

pub static RECENTER_REQUEST: AtomicBool = AtomicBool::new(false);
pub static RECALIBRATE_REQUEST: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Debug, Default)]
pub struct ServerSection {
    pub subscribers: usize,
    pub requests_per_sec: f32,
    pub samples_per_sec: f32,
    pub packets_per_sec: f32,
    pub device_active: bool,
    pub server_id: u32,
    pub bound_port: u16,
}

#[derive(Clone, Copy, Debug)]
pub struct MotionSection {
    pub last_gyro_dps: [f32; 3],
    pub last_accel_g: [f32; 3],
    pub orientation: [f32; 4],
}

impl Default for MotionSection {
    fn default() -> Self {
        Self {
            last_gyro_dps: [0.0; 3],
            last_accel_g: [0.0; 3],
            orientation: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CalibrationSection {
    pub active: bool,
    pub steady: bool,
    pub confidence: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ServerStats {
    pub server: ServerSection,
    pub motion: MotionSection,
    pub calibration: CalibrationSection,
}

static LIVE: RwLock<ServerStats> = RwLock::new(ServerStats {
    server: ServerSection {
        subscribers: 0,
        requests_per_sec: 0.0,
        samples_per_sec: 0.0,
        packets_per_sec: 0.0,
        device_active: false,
        server_id: 0,
        bound_port: 0,
    },
    motion: MotionSection {
        last_gyro_dps: [0.0; 3],
        last_accel_g: [0.0; 3],
        orientation: [1.0, 0.0, 0.0, 0.0],
    },
    calibration: CalibrationSection {
        active: false,
        steady: false,
        confidence: 0.0,
    },
});

pub fn snapshot() -> ServerStats {
    *LIVE.read().unwrap_or_else(|e| e.into_inner())
}

pub fn publish_server(s: ServerSection) {
    LIVE.write().unwrap_or_else(|e| e.into_inner()).server = s;
}

pub fn publish_motion(m: MotionSection) {
    LIVE.write().unwrap_or_else(|e| e.into_inner()).motion = m;
}

pub fn publish_calibration(c: CalibrationSection) {
    LIVE.write().unwrap_or_else(|e| e.into_inner()).calibration = c;
}
