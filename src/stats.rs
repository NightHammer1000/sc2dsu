// Shared live stats — written by the DSU server thread, read by the UI thread.
// All fields atomic / Mutex-free for the lock-free hot path; we use a single
// RwLock around the whole snapshot for simplicity (UI reads at ~30 Hz).

use std::sync::RwLock;
use std::sync::atomic::AtomicBool;

/// UI → DSU thread signal: when set, the DSU thread will reset its integrator
/// quaternion to identity on the next IMU sample, treating the controller's
/// current pose as "flat / centered".
pub static RECENTER_REQUEST: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Debug, Default)]
pub struct ServerStats {
    pub subscribers: usize,
    pub requests_per_sec: f32,
    pub samples_per_sec: f32,
    pub packets_per_sec: f32,
    pub last_gyro_dps: [f32; 3],
    pub last_accel_g: [f32; 3],
    /// Integrated controller orientation quaternion (w, x, y, z), updated from
    /// gyro samples on the DSU thread. Identity at startup; drifts over time
    /// because we don't blend in accel for correction yet.
    pub orientation: [f32; 4],
    pub device_active: bool,
    pub server_id: u32,
    pub bound_port: u16,
}

pub static LIVE: RwLock<ServerStats> = RwLock::new(ServerStats {
    subscribers: 0,
    requests_per_sec: 0.0,
    samples_per_sec: 0.0,
    packets_per_sec: 0.0,
    last_gyro_dps: [0.0; 3],
    last_accel_g: [0.0; 3],
    orientation: [1.0, 0.0, 0.0, 0.0],
    device_active: false,
    server_id: 0,
    bound_port: 0,
});

pub fn snapshot() -> ServerStats {
    LIVE.read().map(|g| *g).unwrap_or_default()
}

pub fn publish(s: ServerStats) {
    if let Ok(mut g) = LIVE.write() {
        *g = s;
    }
}

/// Hot-path update — called from the DSU server thread on every IMU sample so the
/// UI viz can redraw at 60 Hz with fresh data, without waiting for the per-second
/// stats roll-up. Touches only the live motion + orientation fields.
pub fn publish_motion(gyro_dps: [f32; 3], accel_g: [f32; 3], orientation: [f32; 4]) {
    if let Ok(mut g) = LIVE.write() {
        g.last_gyro_dps = gyro_dps;
        g.last_accel_g = accel_g;
        g.orientation = orientation;
    }
}
