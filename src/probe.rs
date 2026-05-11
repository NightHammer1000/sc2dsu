use crate::triton::{self, ControllerState, OpenSlot};
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
    let mut frames_seen = 0u32;
    let mut last_print = Instant::now();
    while Instant::now() < deadline {
        match slot.read_one(50) {
            Ok(Some(s)) => {
                frames_seen += 1;
                if last_print.elapsed() >= Duration::from_millis(100) {
                    print_sample(&s);
                    last_print = Instant::now();
                }
            }
            Ok(None) => {}
            Err(e) => {
                println!("    read error: {e}");
                break;
            }
        }
    }
    println!("    --- 3 s summary: {frames_seen} STATE frames decoded ---");
}

fn print_sample(s: &ControllerState) {
    println!(
        "      ts={:>10}  buttons=0x{:05X}  L=({:>+6},{:>+6}) R=({:>+6},{:>+6})  trig=({:>5},{:>5})  gyro(dps)=[{:>+8.2} {:>+8.2} {:>+8.2}]  accel(g)=[{:>+6.3} {:>+6.3} {:>+6.3}]",
        s.imu.timestamp_us,
        s.buttons,
        s.left_stick[0],
        s.left_stick[1],
        s.right_stick[0],
        s.right_stick[1],
        s.trigger_left,
        s.trigger_right,
        s.imu.gyro_dps[0],
        s.imu.gyro_dps[1],
        s.imu.gyro_dps[2],
        s.imu.accel_g[0],
        s.imu.accel_g[1],
        s.imu.accel_g[2],
    );
}
