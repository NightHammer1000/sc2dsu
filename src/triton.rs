use crate::config;
use crate::gyro_calibration::GyroCalibration;
use crate::stats;
use hidapi::{DeviceInfo, HidApi, HidDevice};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

pub const VID_VALVE: u16 = 0x28DE;
pub const PID_TRITON_WIRED: u16 = 0x1302;
pub const PID_TRITON_BLE: u16 = 0x1303;
pub const PID_PROTEUS_DONGLE: u16 = 0x1304;
pub const PID_NEREID_DONGLE: u16 = 0x1305;

const FEATURE_REPORT_ID: u8 = 0x01;
const FEATURE_REPORT_BYTES: usize = 64;
const INPUT_REPORT_BYTES: usize = 64;
const ID_SET_SETTINGS_VALUES: u8 = 0x87;
const SETTING_LIZARD_MODE: u8 = 9;
const SETTING_IMU_MODE: u8 = 48;
const LIZARD_MODE_OFF: u16 = 0;
const GYRO_MODE_RAW_ACCEL_AND_GYRO: u16 = 0x0008 | 0x0010;

pub const TRITON_REPORT_STATE: u8 = 0x42;
pub const TRITON_REPORT_STATE_BLE: u8 = 0x45;
#[allow(dead_code)]
pub const TRITON_REPORT_BATTERY: u8 = 0x43;
#[allow(dead_code)]
pub const TRITON_REPORT_WIRELESS_X: u8 = 0x46;
#[allow(dead_code)]
pub const TRITON_REPORT_WIRELESS: u8 = 0x79;

const LIZARD_REFRESH_INTERVAL: Duration = Duration::from_secs(3);
const IMU_REFRESH_INTERVAL: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub timestamp_us: u32,
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
}

#[allow(dead_code)]
pub mod button {
    pub const A: u32 = 0x0000_0001;
    pub const B: u32 = 0x0000_0002;
    pub const X: u32 = 0x0000_0004;
    pub const Y: u32 = 0x0000_0008;
    pub const QAM: u32 = 0x0000_0010;
    pub const R3: u32 = 0x0000_0020;
    pub const VIEW: u32 = 0x0000_0040;
    pub const R4: u32 = 0x0000_0080;
    pub const R5: u32 = 0x0000_0100;
    pub const R: u32 = 0x0000_0200;
    pub const DPAD_DOWN: u32 = 0x0000_0400;
    pub const DPAD_RIGHT: u32 = 0x0000_0800;
    pub const DPAD_LEFT: u32 = 0x0000_1000;
    pub const DPAD_UP: u32 = 0x0000_2000;
    pub const MENU: u32 = 0x0000_4000;
    pub const L3: u32 = 0x0000_8000;
    pub const STEAM: u32 = 0x0001_0000;
    pub const L4: u32 = 0x0002_0000;
    pub const L5: u32 = 0x0004_0000;
    pub const L: u32 = 0x0008_0000;
}

#[derive(Debug, Clone, Copy)]
pub struct ControllerState {
    pub buttons: u32,
    pub trigger_left: u16,
    pub trigger_right: u16,
    pub left_stick: [i16; 2],
    pub right_stick: [i16; 2],
    pub imu: ImuSample,
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

pub fn list_candidates(api: &HidApi) -> Vec<DeviceInfo> {
    api.device_list()
        .filter(|d| {
            d.vendor_id() == VID_VALVE && is_triton_pid(d.product_id()) && d.usage_page() >= 0xFF00
        })
        .cloned()
        .collect()
}

fn build_set_setting_report(setting_num: u8, setting_value: u16) -> [u8; FEATURE_REPORT_BYTES] {
    let mut buf = [0u8; FEATURE_REPORT_BYTES];
    buf[0] = FEATURE_REPORT_ID;
    buf[1] = ID_SET_SETTINGS_VALUES;
    buf[2] = 3;
    buf[3] = setting_num;
    let v = setting_value.to_le_bytes();
    buf[4] = v[0];
    buf[5] = v[1];
    buf
}

pub fn parse_imu(
    payload: &[u8],
    gyro_map: &config::AxisMap,
    accel_map: &config::AxisMap,
) -> Option<ImuSample> {
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

    let to_g = |v: i16| (v as f32 / 32768.0) * 2.0;
    let to_dps = |v: i16| (v as f32 / 32768.0) * 2000.0;
    let raw_accel_f = [to_g(raw_accel.0), to_g(raw_accel.1), to_g(raw_accel.2)];
    let raw_gyro_f = [to_dps(raw_gyro.0), to_dps(raw_gyro.1), to_dps(raw_gyro.2)];

    Some(ImuSample {
        timestamp_us: ts,
        accel_g: [
            accel_map.x.apply(raw_accel_f),
            accel_map.y.apply(raw_accel_f),
            accel_map.z.apply(raw_accel_f),
        ],
        gyro_dps: [
            gyro_map.x.apply(raw_gyro_f),
            gyro_map.y.apply(raw_gyro_f),
            gyro_map.z.apply(raw_gyro_f),
        ],
    })
}

pub fn parse_state(
    payload: &[u8],
    gyro_map: &config::AxisMap,
    accel_map: &config::AxisMap,
) -> Option<ControllerState> {
    let imu = parse_imu(payload, gyro_map, accel_map)?;
    let u16le = |o: usize| u16::from_le_bytes([payload[o], payload[o + 1]]);
    let i16le = |o: usize| i16::from_le_bytes([payload[o], payload[o + 1]]);
    Some(ControllerState {
        buttons: u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]),
        trigger_left: u16le(5),
        trigger_right: u16le(7),
        left_stick: [i16le(9), i16le(11)],
        right_stick: [i16le(13), i16le(15)],
        imu,
    })
}

pub struct OpenSlot {
    dev: HidDevice,
    last_lizard_refresh: Instant,
    last_imu_refresh: Instant,
    gyro_map: config::AxisMap,
    accel_map: config::AxisMap,
    gyro_cal: GyroCalibration,
    auto_calibrate: bool,
    last_imu_ts_us: Option<u32>,
    cfg_generation: u64,
    pub interface_number: i32,
    pub product_id: u16,
}

impl OpenSlot {
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
        let cfg_generation = config::generation();
        let cfg = config::snapshot();
        Ok(Self {
            dev,
            last_lizard_refresh: Instant::now(),
            last_imu_refresh: Instant::now(),
            gyro_map: cfg.gyro,
            accel_map: cfg.accel,
            gyro_cal: GyroCalibration::new(),
            auto_calibrate: cfg.auto_calibrate,
            last_imu_ts_us: None,
            cfg_generation,
            interface_number: info.interface_number(),
            product_id: info.product_id(),
        })
    }

    pub fn read_one(&mut self, timeout_ms: i32) -> Result<Option<ControllerState>, String> {
        if self.last_lizard_refresh.elapsed() >= LIZARD_REFRESH_INTERVAL {
            let lizard = build_set_setting_report(SETTING_LIZARD_MODE, LIZARD_MODE_OFF);
            let _ = self.dev.send_feature_report(&lizard);
            self.last_lizard_refresh = Instant::now();
        }
        if self.last_imu_refresh.elapsed() >= IMU_REFRESH_INTERVAL {
            let imu = build_set_setting_report(SETTING_IMU_MODE, GYRO_MODE_RAW_ACCEL_AND_GYRO);
            let _ = self.dev.send_feature_report(&imu);
            self.last_imu_refresh = Instant::now();
        }
        let live_generation = config::generation();
        if live_generation != self.cfg_generation {
            let cfg = config::snapshot();
            self.gyro_map = cfg.gyro;
            self.accel_map = cfg.accel;
            self.auto_calibrate = cfg.auto_calibrate;
            self.cfg_generation = live_generation;
            // Bias is estimated in the post-mapping frame; a remap invalidates it.
            // Toggling auto-calibrate also resets so re-enabling starts fresh.
            self.gyro_cal.reset();
            self.last_imu_ts_us = None;
        }
        if stats::RECALIBRATE_REQUEST.swap(false, Ordering::Relaxed) {
            self.gyro_cal.reset();
            self.last_imu_ts_us = None;
        }
        let mut buf = [0u8; INPUT_REPORT_BYTES];
        match self.dev.read_timeout(&mut buf, timeout_ms) {
            Ok(0) => Ok(None),
            Ok(n) => {
                let id = buf[0];
                if id == TRITON_REPORT_STATE || id == TRITON_REPORT_STATE_BLE {
                    let Some(mut state) = parse_state(&buf[1..n], &self.gyro_map, &self.accel_map)
                    else {
                        return Ok(None);
                    };
                    let dt = match self.last_imu_ts_us {
                        Some(prev) => {
                            let delta = state.imu.timestamp_us.wrapping_sub(prev);
                            // Clamp at 100 ms — a longer gap means we lost the
                            // stream (Steam took the device, reopen, etc.) and
                            // pretending it's one big step is worse than
                            // skipping the update.
                            (delta as f32 / 1_000_000.0).clamp(0.0, 0.1)
                        }
                        None => 0.0,
                    };
                    self.last_imu_ts_us = Some(state.imu.timestamp_us);
                    if self.auto_calibrate {
                        state.imu.gyro_dps =
                            self.gyro_cal
                                .correct(state.imu.gyro_dps, state.imu.accel_g, dt);
                    }
                    stats::publish_calibration(stats::CalibrationSection {
                        active: self.auto_calibrate,
                        steady: self.gyro_cal.is_steady(),
                        confidence: self.gyro_cal.confidence(),
                    });
                    Ok(Some(state))
                } else {
                    Ok(None)
                }
            }
            Err(e) => Err(format!("read: {e}")),
        }
    }
}

pub fn find_active_slot(api: &HidApi, candidates: &[DeviceInfo]) -> Option<OpenSlot> {
    for info in candidates {
        match OpenSlot::open(api, info) {
            Ok(slot) => return Some(slot),
            Err(e) => eprintln!(
                "triton: open iface {} (PID {:04X}) failed: {e}",
                info.interface_number(),
                info.product_id()
            ),
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity_map() -> config::AxisMap {
        config::AxisMap {
            x: config::Axis::new(0, false),
            y: config::Axis::new(1, false),
            z: config::Axis::new(2, false),
        }
    }

    fn build_imu_payload(ts: u32, accel_raw: [i16; 3], gyro_raw: [i16; 3]) -> Vec<u8> {
        let mut p = vec![0u8; 45];
        p[29..33].copy_from_slice(&ts.to_le_bytes());
        for (i, v) in accel_raw.iter().enumerate() {
            p[33 + i * 2..35 + i * 2].copy_from_slice(&v.to_le_bytes());
        }
        for (i, v) in gyro_raw.iter().enumerate() {
            p[39 + i * 2..41 + i * 2].copy_from_slice(&v.to_le_bytes());
        }
        p
    }

    #[test]
    fn parse_imu_rejects_short_payload() {
        assert!(parse_imu(&[0u8; 44], &identity_map(), &identity_map()).is_none());
    }

    #[test]
    fn parse_imu_decodes_full_scale_values() {
        let payload = build_imu_payload(0x1234_5678, [16384, 0, -16384], [16384, -16384, 0]);
        let s = parse_imu(&payload, &identity_map(), &identity_map()).unwrap();
        assert_eq!(s.timestamp_us, 0x1234_5678);
        assert!((s.accel_g[0] - 1.0).abs() < 1e-4);
        assert!(s.accel_g[1].abs() < 1e-4);
        assert!((s.accel_g[2] + 1.0).abs() < 1e-4);
        assert!((s.gyro_dps[0] - 1000.0).abs() < 1e-2);
        assert!((s.gyro_dps[1] + 1000.0).abs() < 1e-2);
        assert!(s.gyro_dps[2].abs() < 1e-2);
    }

    #[test]
    fn parse_imu_applies_axis_mapping() {
        let payload = build_imu_payload(0, [1000, 2000, 3000], [100, 200, 300]);
        let swap_xy = config::AxisMap {
            x: config::Axis::new(1, false),
            y: config::Axis::new(0, true),
            z: config::Axis::new(2, false),
        };
        let direct = parse_imu(&payload, &identity_map(), &identity_map()).unwrap();
        let mapped = parse_imu(&payload, &swap_xy, &identity_map()).unwrap();
        assert!((mapped.gyro_dps[0] - direct.gyro_dps[1]).abs() < 1e-6);
        assert!((mapped.gyro_dps[1] + direct.gyro_dps[0]).abs() < 1e-6);
        assert!((mapped.gyro_dps[2] - direct.gyro_dps[2]).abs() < 1e-6);
    }

    #[test]
    fn parse_state_decodes_buttons_sticks_triggers() {
        let mut p = build_imu_payload(0x1111_2222, [1, 2, 3], [4, 5, 6]);
        p[1..5].copy_from_slice(&(button::A | button::DPAD_LEFT | button::STEAM).to_le_bytes());
        p[5..7].copy_from_slice(&1234u16.to_le_bytes());
        p[7..9].copy_from_slice(&31000u16.to_le_bytes());
        p[9..11].copy_from_slice(&(-100i16).to_le_bytes());
        p[11..13].copy_from_slice(&200i16.to_le_bytes());
        p[13..15].copy_from_slice(&(-300i16).to_le_bytes());
        p[15..17].copy_from_slice(&400i16.to_le_bytes());
        let s = parse_state(&p, &identity_map(), &identity_map()).unwrap();
        assert_eq!(s.buttons, button::A | button::DPAD_LEFT | button::STEAM);
        assert_eq!(s.trigger_left, 1234);
        assert_eq!(s.trigger_right, 31000);
        assert_eq!(s.left_stick, [-100, 200]);
        assert_eq!(s.right_stick, [-300, 400]);
        assert_eq!(s.imu.timestamp_us, 0x1111_2222);
    }

    #[test]
    fn parse_state_rejects_short_payload() {
        assert!(parse_state(&[0u8; 30], &identity_map(), &identity_map()).is_none());
    }
}
