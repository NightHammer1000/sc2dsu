#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod autostart;
mod config;
mod dsu;
mod gyro_calibration;
mod probe;
mod stats;
mod triton;
mod ui;

use hidapi::HidApi;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{SyncSender, sync_channel};
use std::thread;
use std::time::{Duration, Instant};

const SAMPLE_QUEUE_LEN: usize = 64;

#[derive(Debug, PartialEq, Eq)]
enum Mode {
    Gui { start_minimized: bool },
    Headless,
    Probe,
}

fn parse_args() -> Mode {
    parse_args_from(std::env::args().skip(1))
}

fn parse_args_from(args: impl Iterator<Item = String>) -> Mode {
    for arg in args {
        match arg.as_str() {
            "--probe" | "-p" => return Mode::Probe,
            "--headless" | "-H" => return Mode::Headless,
            "--tray" | "--minimized" => {
                return Mode::Gui {
                    start_minimized: true,
                };
            }
            "--gui" => {
                return Mode::Gui {
                    start_minimized: false,
                };
            }
            _ => {}
        }
    }
    Mode::Gui {
        start_minimized: false,
    }
}

fn attach_console() {
    use windows_sys::Win32::System::Console::{ATTACH_PARENT_PROCESS, AllocConsole, AttachConsole};
    // SAFETY: AttachConsole/AllocConsole take no caller-supplied pointers; a failed
    // AttachConsole (no parent console, or already attached) is detected via its return
    // value, and AllocConsole's failure (e.g. console already present) is harmless here.
    unsafe {
        if AttachConsole(ATTACH_PARENT_PROCESS) == 0 {
            AllocConsole();
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    match parse_args() {
        Mode::Probe => {
            attach_console();
            probe::run()
        }
        Mode::Headless => {
            attach_console();
            run_server(None)
        }
        Mode::Gui { start_minimized } => run_server(Some(start_minimized)),
    }
}

fn run_server(gui_start_minimized: Option<bool>) -> Result<(), Box<dyn std::error::Error>> {
    let cfg = config::load_or_create();
    let dsu_port = cfg.port;
    let dsu_expose = cfg.expose_to_network;
    config::install(cfg);

    let dsu_wants_device = Arc::new(AtomicBool::new(false));
    let ui_wants_device = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let (tx, rx) = sync_channel::<triton::ControllerState>(SAMPLE_QUEUE_LEN);

    let device_handle = {
        let dsu_wants = dsu_wants_device.clone();
        let ui_wants = ui_wants_device.clone();
        let shutdown = shutdown.clone();
        thread::Builder::new()
            .name("triton-reader".into())
            .spawn(move || run_device_thread(dsu_wants, ui_wants, shutdown, tx))?
    };

    let server_handle = {
        let dsu_wants = dsu_wants_device.clone();
        let shutdown = shutdown.clone();
        thread::Builder::new()
            .name("dsu-server".into())
            .spawn(move || -> std::io::Result<()> {
                let mut server = dsu::Server::bind(dsu_port, dsu_expose, dsu_wants, shutdown, rx)?;
                eprintln!(
                    "sc2dsu DSU server listening on {}  (server id 0x{:08X})",
                    server.local_addr()?,
                    server.server_id()
                );
                eprintln!("waiting for client subscription before opening the controller ...");
                server.run()
            })?
    };

    match gui_start_minimized {
        Some(start_minimized) => {
            ui::run(shutdown.clone(), ui_wants_device.clone(), start_minimized)
                .map_err(|e| -> Box<dyn std::error::Error> { e.into() })?;
        }
        None => {
            let _ = server_handle.join();
        }
    }

    shutdown.store(true, Ordering::Relaxed);
    let _ = device_handle.join();
    Ok(())
}

fn run_device_thread(
    dsu_wants: Arc<AtomicBool>,
    ui_wants: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    tx: SyncSender<triton::ControllerState>,
) {
    let want_device = || dsu_wants.load(Ordering::Relaxed) || ui_wants.load(Ordering::Relaxed);

    let mut api = match HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("triton: HidApi init failed: {e}");
            return;
        }
    };

    while !shutdown.load(Ordering::Relaxed) {
        if !want_device() {
            thread::sleep(Duration::from_millis(200));
            continue;
        }

        if let Err(e) = api.refresh_devices() {
            eprintln!("triton: refresh_devices failed ({e}); rebuilding HidApi");
            match HidApi::new() {
                Ok(a) => api = a,
                Err(e) => {
                    eprintln!("triton: HidApi re-init failed: {e}; backing off");
                    thread::sleep(Duration::from_millis(1000));
                    continue;
                }
            }
        }

        let candidates = triton::list_candidates(&api);
        if candidates.is_empty() {
            thread::sleep(Duration::from_millis(500));
            continue;
        }

        let mut slot = match triton::find_active_slot(&api, &candidates) {
            Some(s) => {
                eprintln!(
                    "triton: opened slot iface {} (PID {:04X} {})",
                    s.interface_number,
                    s.product_id,
                    triton::pid_label(s.product_id),
                );
                s
            }
            None => {
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };

        run_slot(&mut slot, &tx, &want_device, &shutdown);
        eprintln!(
            "triton: closing slot (dsu_wants={}, ui_wants={})",
            dsu_wants.load(Ordering::Relaxed),
            ui_wants.load(Ordering::Relaxed)
        );
    }
}

fn run_slot(
    slot: &mut triton::OpenSlot,
    tx: &SyncSender<triton::ControllerState>,
    want_device: &impl Fn() -> bool,
    shutdown: &AtomicBool,
) {
    const SILENCE_REOPEN_MS: u128 = 2000;
    const STALE_THRESHOLD: u32 = 100;

    let mut consecutive_errors = 0u32;
    let mut last_sample_at = Instant::now();
    let mut last_imu_ts: u32 = 0;
    let mut stale_count: u32 = 0;
    while want_device() && !shutdown.load(Ordering::Relaxed) {
        match slot.read_one(50) {
            Ok(Some(sample)) => {
                consecutive_errors = 0;
                last_sample_at = Instant::now();
                if sample.imu.timestamp_us == last_imu_ts {
                    stale_count += 1;
                    if stale_count >= STALE_THRESHOLD {
                        eprintln!(
                            "triton: IMU timestamp frozen for {} samples — Steam likely disabled IMU; reopening slot",
                            STALE_THRESHOLD
                        );
                        return;
                    }
                } else {
                    stale_count = 0;
                    last_imu_ts = sample.imu.timestamp_us;
                    let _ = tx.try_send(sample);
                }
            }
            Ok(None) => {
                consecutive_errors = 0;
                if last_sample_at.elapsed().as_millis() >= SILENCE_REOPEN_MS {
                    eprintln!(
                        "triton: no STATE reports for {} ms — Steam likely commandeered the device; reopening slot",
                        SILENCE_REOPEN_MS
                    );
                    return;
                }
            }
            Err(e) => {
                consecutive_errors += 1;
                if consecutive_errors >= 5 {
                    eprintln!("triton: 5 consecutive read errors ({e}); reopening slot");
                    return;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Mode {
        parse_args_from(args.iter().map(|s| (*s).to_string()))
    }

    #[test]
    fn parse_args_defaults_to_visible_gui() {
        assert_eq!(
            parse(&[]),
            Mode::Gui {
                start_minimized: false
            }
        );
        assert_eq!(
            parse(&["some-positional-arg"]),
            Mode::Gui {
                start_minimized: false
            }
        );
    }

    #[test]
    fn parse_args_recognizes_each_mode() {
        assert_eq!(parse(&["--probe"]), Mode::Probe);
        assert_eq!(parse(&["-p"]), Mode::Probe);
        assert_eq!(parse(&["--headless"]), Mode::Headless);
        assert_eq!(parse(&["-H"]), Mode::Headless);
        assert_eq!(
            parse(&["--gui"]),
            Mode::Gui {
                start_minimized: false
            }
        );
        assert_eq!(
            parse(&["--tray"]),
            Mode::Gui {
                start_minimized: true
            }
        );
        assert_eq!(
            parse(&["--minimized"]),
            Mode::Gui {
                start_minimized: true
            }
        );
    }

    #[test]
    fn parse_args_uses_first_recognized_flag() {
        assert_eq!(
            parse(&["--gui", "--probe"]),
            Mode::Gui {
                start_minimized: false
            }
        );
    }
}
