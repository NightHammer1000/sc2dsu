// Steam Controller (2026 / "Triton") HID protocol — discovery, init, IMU reader.
//
// Spec: SDL3 src/joystick/hidapi/SDL_hidapi_steam_triton.c +
//       src/joystick/hidapi/steam/{controller_constants.h, controller_structs.h}.

use crate::config;
use hidapi::{DeviceInfo, HidApi, HidDevice};
use std::time::{Duration, Instant};

pub const VID_VALVE: u16 = 0x28DE;
pub const PID_TRITON_WIRED: u16 = 0x1302;
pub const PID_TRITON_BLE: u16 = 0x1303;
pub const PID_PROTEUS_DONGLE: u16 = 0x1304;
pub const PID_NEREID_DONGLE: u16 = 0x1305;

const FEATURE_REPORT_ID: u8 = 0x01;
const FEATURE_REPORT_BYTES: usize = 64;
const ID_SET_SETTINGS_VALUES: u8 = 0x87;
const SETTING_LIZARD_MODE: u8 = 9;
const SETTING_IMU_MODE: u8 = 48;
const LIZARD_MODE_OFF: u16 = 0;
const GYRO_MODE_RAW_ACCEL_AND_GYRO: u16 = 0x0008 | 0x0010;

pub const TRITON_REPORT_STATE: u8 = 0x42;
pub const TRITON_REPORT_BATTERY: u8 = 0x43;
pub const TRITON_REPORT_STATE_BLE: u8 = 0x45;
pub const TRITON_REPORT_WIRELESS_X: u8 = 0x46;
pub const TRITON_REPORT_WIRELESS: u8 = 0x79;

/// Re-disable lizard mode every 3 s — the controller turns it back on otherwise (matches SDL).
const LIZARD_REFRESH_INTERVAL: Duration = Duration::from_secs(3);
/// Re-enable IMU streaming every 1 s. If Steam Input's config UI claims the
/// device, it can disable IMU mode silently; refreshing keeps our setting alive.
const IMU_REFRESH_INTERVAL: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Microsecond timestamp from the controller's IMU.
    pub timestamp_us: u32,
    /// Accelerometer in g, axis-remapped to SDL convention (X = raw_X, Y = raw_Z, Z = -raw_Y).
    pub accel_g: [f32; 3],
    /// Gyroscope in deg/s, same SDL axis remap as accel.
    pub gyro_dps: [f32; 3],
}

pub fn pid_label(pid: u16) -> &'static str {
    match pid {
        PID_TRITON_WIRED => "Triton wired",
        PID_TRITON_BLE => "Triton BLE",
        PID_PROTEUS_DONGLE => "Proteus Puck",
        PID_NEREID_DONGLE => "Nereid dongle",
        _ => "?",
    }
}

pub fn is_triton_pid(pid: u16) -> bool {
    matches!(
        pid,
        PID_TRITON_WIRED | PID_TRITON_BLE | PID_PROTEUS_DONGLE | PID_NEREID_DONGLE
    )
}

/// All vendor-defined HID interfaces on Triton-family devices we might want to talk to.
pub fn list_candidates(api: &HidApi) -> Vec<DeviceInfo> {
    api.device_list()
        .filter(|d| {
            d.vendor_id() == VID_VALVE && is_triton_pid(d.product_id()) && d.usage_page() >= 0xFF00
        })
        .cloned()
        .collect()
}

fn build_set_setting_report(setting_num: u8, setting_value: u16) -> [u8; FEATURE_REPORT_BYTES] {
    // SDL sends a 64-byte buffer where buffer[0] = 1 IS the HID report ID and the
    // FeatureReportMsg overlay starts at buffer[1]. hidapi-rs uses the same convention
    // (slice[0] = report ID prefix), so the slice we pass is exactly 64 bytes.
    let mut buf = [0u8; FEATURE_REPORT_BYTES];
    buf[0] = FEATURE_REPORT_ID;
    buf[1] = ID_SET_SETTINGS_VALUES;
    buf[2] = 3; // FeatureReportHeader.length = sizeof(ControllerSetting) = 3
    buf[3] = setting_num;
    let v = setting_value.to_le_bytes();
    buf[4] = v[0];
    buf[5] = v[1];
    buf
}

/// Parse the IMU portion of a state report payload (after the 1-byte report ID has
/// already been stripped). Returns None if the buffer is too short.
///
/// TritonMTUFull_t / TritonMTUNoQuat_t (#pragma pack(1)):
///   +0  u8   seq_num
///   +1  u32  buttons
///   +5  i16  triggerLeft, +7 triggerRight
///   +9  i16  leftStickX, +11 leftStickY, +13 rightStickX, +15 rightStickY
///   +17 i16  leftPadX,  +19 leftPadY,    +21 u16 pressureLeft
///   +23 i16  rightPadX, +25 rightPadY,   +27 u16 pressureRight
///   +29 imu:  +0 u32 timestamp, +4..+15 i16 accelXYZ + gyroXYZ, (+16..+23 i16 quat WXYZ on Full)
pub fn parse_imu(payload: &[u8]) -> Option<ImuSample> {
    const IMU_OFFSET: usize = 29;
    const IMU_NOQUAT_LEN: usize = 16;
    if payload.len() < IMU_OFFSET + IMU_NOQUAT_LEN {
        return None;
    }
    let imu = &payload[IMU_OFFSET..];
    let ts = u32::from_le_bytes([imu[0], imu[1], imu[2], imu[3]]);
    let i16le = |o: usize| i16::from_le_bytes([imu[o], imu[o + 1]]);
    let raw_accel = (i16le(4), i16le(6), i16le(8));
    let raw_gyro = (i16le(10), i16le(12), i16le(14));

    // ±2 g full-scale on accel, ±2000 deg/s full-scale on gyro.
    let to_g = |v: i16| (v as f32 / 32768.0) * 2.0;
    let to_dps = |v: i16| (v as f32 / 32768.0) * 2000.0;
    let raw_accel_f = [to_g(raw_accel.0), to_g(raw_accel.1), to_g(raw_accel.2)];
    let raw_gyro_f = [to_dps(raw_gyro.0), to_dps(raw_gyro.1), to_dps(raw_gyro.2)];

    // Pull the current axis mapping from the live config every sample so edits to
    // ~/.config/sc2dsu/config.toml (or whatever the OS's APPDATA path is) take
    // effect without restarting the server.
    let cfg = config::snapshot();
    Some(ImuSample {
        timestamp_us: ts,
        accel_g: [
            cfg.accel.x.apply(raw_accel_f),
            cfg.accel.y.apply(raw_accel_f),
            cfg.accel.z.apply(raw_accel_f),
        ],
        gyro_dps: [
            cfg.gyro.x.apply(raw_gyro_f),
            cfg.gyro.y.apply(raw_gyro_f),
            cfg.gyro.z.apply(raw_gyro_f),
        ],
    })
}

pub struct OpenSlot {
    dev: HidDevice,
    last_lizard_refresh: Instant,
    last_imu_refresh: Instant,
    /// The interface number we successfully opened on (for diagnostics only).
    pub interface_number: i32,
    pub product_id: u16,
}

impl OpenSlot {
    /// Open a specific candidate interface and send the lizard-off + IMU-on init reports.
    /// Returns an OpenSlot only if both feature reports succeeded (proves the slot is
    /// alive and willing to talk; empty Puck slots return Win32 0x1F).
    pub fn open(api: &HidApi, info: &DeviceInfo) -> Result<Self, String> {
        let dev = api
            .open_path(info.path())
            .map_err(|e| format!("open: {e}"))?;
        let lizard = build_set_setting_report(SETTING_LIZARD_MODE, LIZARD_MODE_OFF);
        dev.send_feature_report(&lizard)
            .map_err(|e| format!("lizard-off: {e}"))?;
        let imu = build_set_setting_report(SETTING_IMU_MODE, GYRO_MODE_RAW_ACCEL_AND_GYRO);
        dev.send_feature_report(&imu)
            .map_err(|e| format!("imu-on: {e}"))?;
        let _ = dev.set_blocking_mode(false);
        Ok(Self {
            dev,
            last_lizard_refresh: Instant::now(),
            last_imu_refresh: Instant::now(),
            interface_number: info.interface_number(),
            product_id: info.product_id(),
        })
    }

    /// Read up to one HID report. Returns:
    ///   Ok(Some(sample)) — got a state report carrying IMU data
    ///   Ok(None)         — read returned 0 (no data ready), got a non-IMU report,
    ///                      or got a state report with truncated IMU
    ///   Err(_)           — HID error; caller should drop and re-open
    pub fn read_one(&mut self, timeout_ms: i32) -> Result<Option<ImuSample>, String> {
        if self.last_lizard_refresh.elapsed() >= LIZARD_REFRESH_INTERVAL {
            let lizard = build_set_setting_report(SETTING_LIZARD_MODE, LIZARD_MODE_OFF);
            // A failed refresh isn't fatal — it just means the device blipped; next read will tell us.
            let _ = self.dev.send_feature_report(&lizard);
            self.last_lizard_refresh = Instant::now();
        }
        if self.last_imu_refresh.elapsed() >= IMU_REFRESH_INTERVAL {
            let imu = build_set_setting_report(SETTING_IMU_MODE, GYRO_MODE_RAW_ACCEL_AND_GYRO);
            let _ = self.dev.send_feature_report(&imu);
            self.last_imu_refresh = Instant::now();
        }
        let mut buf = [0u8; 64];
        match self.dev.read_timeout(&mut buf, timeout_ms) {
            Ok(0) => Ok(None),
            Ok(n) => {
                let id = buf[0];
                if id == TRITON_REPORT_STATE || id == TRITON_REPORT_STATE_BLE {
                    Ok(parse_imu(&buf[1..n]))
                } else {
                    Ok(None)
                }
            }
            Err(e) => Err(format!("read: {e}")),
        }
    }
}

/// Find the slot the controller is paired into. On the Proteus Puck only the active
/// slot accepts feature reports; empty slots return Win32 0x1F on SetFeature. So
/// "open + init succeeded" is a sufficient liveness signal.
pub fn find_active_slot(api: &HidApi, candidates: &[DeviceInfo]) -> Option<OpenSlot> {
    candidates
        .iter()
        .find_map(|info| OpenSlot::open(api, info).ok())
}
