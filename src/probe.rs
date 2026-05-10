// Diagnostic probe mode — enumerate every Valve HID interface, try to open each
// candidate, send the init feature reports, and dump 3 s of decoded gyro/accel.
// Used to verify the SDL spec against the actual device on the wire.

use crate::triton::{
    self, ImuSample, OpenSlot, TRITON_REPORT_BATTERY, TRITON_REPORT_STATE, TRITON_REPORT_STATE_BLE,
    TRITON_REPORT_WIRELESS, TRITON_REPORT_WIRELESS_X,
};
use hidapi::HidApi;
use std::time::{Duration, Instant};

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    let api = HidApi::new()?;

    println!("=== All Valve (VID 0x28DE) HID interfaces ===");
    for d in api
        .device_list()
        .filter(|d| d.vendor_id() == triton::VID_VALVE)
    {
        let cand = triton::is_triton_pid(d.product_id()) && d.usage_page() >= 0xFF00;
        let mark = if cand { "  <-- candidate" } else { "" };
        println!(
            "  PID {:04X} iface={:>2} usage_page=0x{:04X} usage=0x{:04X} serial={:?} product={:?}{}",
            d.product_id(),
            d.interface_number(),
            d.usage_page(),
            d.usage(),
            d.serial_number().unwrap_or(""),
            d.product_string().unwrap_or(""),
            mark
        );
    }

    let candidates = triton::list_candidates(&api);
    println!("\n=== {} candidate interface(s) ===", candidates.len());
    for info in &candidates {
        println!(
            "\n>>> trying iface {} of PID {:04X} ({})",
            info.interface_number(),
            info.product_id(),
            triton::pid_label(info.product_id())
        );
        match OpenSlot::open(&api, info) {
            Ok(slot) => probe_one(slot),
            Err(e) => println!("    open/init failed: {e}"),
        }
    }
    Ok(())
}

fn probe_one(mut slot: OpenSlot) {
    println!(
        "    open + init ok (iface {}, PID {:04X}); reading 3 s ...",
        slot.interface_number, slot.product_id
    );
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut counts = [0u32; 256];
    let mut imu_seen = 0u32;
    let mut last_print = Instant::now();
    while Instant::now() < deadline {
        match slot.read_one(50) {
            Ok(Some(s)) => {
                counts[TRITON_REPORT_STATE as usize] += 1; // approx; some may be BLE
                imu_seen += 1;
                if last_print.elapsed() >= Duration::from_millis(100) {
                    print_sample(&s);
                    last_print = Instant::now();
                }
            }
            Ok(None) => {
                // No state report this poll; could be a battery/wireless report or just idle.
                // We don't currently expose other report IDs through OpenSlot — that's fine for
                // the probe, since the per-ID histogram below would only matter for debugging
                // unknown frames.
            }
            Err(e) => {
                println!("    read error: {e}");
                break;
            }
        }
    }

    println!("    --- 3 s summary ---");
    print_count("STATE/STATE_BLE", counts[TRITON_REPORT_STATE as usize]);
    print_count("BATTERY", counts[TRITON_REPORT_BATTERY as usize]);
    print_count("WIRELESS", counts[TRITON_REPORT_WIRELESS as usize]);
    print_count("WIRELESS_X", counts[TRITON_REPORT_WIRELESS_X as usize]);
    print_count("STATE_BLE", counts[TRITON_REPORT_STATE_BLE as usize]);
    println!("    IMU-bearing frames decoded: {imu_seen}");
}

fn print_count(label: &str, n: u32) {
    if n > 0 {
        println!("      {label:<22} {n:>6}");
    }
}

fn print_sample(s: &ImuSample) {
    println!(
        "      ts={:>10} gyro(dps)=[{:>+8.2} {:>+8.2} {:>+8.2}]  accel(g)=[{:>+6.3} {:>+6.3} {:>+6.3}]",
        s.timestamp_us,
        s.gyro_dps[0],
        s.gyro_dps[1],
        s.gyro_dps[2],
        s.accel_g[0],
        s.accel_g[1],
        s.accel_g[2],
    );
}
