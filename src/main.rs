#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod autostart;
mod config;
mod dsu;
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

enum Mode {
    Gui { start_minimized: bool },
    Headless,
    Probe,
}

fn parse_args() -> Mode {
    for arg in std::env::args().skip(1) {
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mode = parse_args();
    if matches!(mode, Mode::Probe) {
        return probe::run();
    }

    let cfg = config::load_or_create();
    let dsu_port = cfg.port;
    let dsu_expose = cfg.expose_to_network;
    config::install(cfg);

    let dsu_wants_device = Arc::new(AtomicBool::new(false));
    let ui_wants_device = Arc::new(AtomicBool::new(false));
    let shutdown = Arc::new(AtomicBool::new(false));
    let (tx, rx) = sync_channel::<triton::ImuSample>(64);

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

    match mode {
        Mode::Gui { start_minimized } => {
            ui::run(shutdown.clone(), ui_wants_device.clone(), start_minimized)
                .map_err(|e| -> Box<dyn std::error::Error> { e.into() })?;
        }
        Mode::Headless => {
            let _ = server_handle.join();
        }
        Mode::Probe => unreachable!(),
    }

    shutdown.store(true, Ordering::Relaxed);
    let _ = device_handle.join();
    let _ = server_handle;
    Ok(())
}

fn run_device_thread(
    dsu_wants: Arc<AtomicBool>,
    ui_wants: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    tx: SyncSender<triton::ImuSample>,
) {
    const SILENCE_REOPEN_MS: u128 = 2000;

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

        let mut consecutive_errors = 0;
        let mut last_sample_at = Instant::now();
        let mut last_imu_ts: u32 = 0;
        let mut stale_count: u32 = 0;
        const STALE_THRESHOLD: u32 = 100;
        while want_device() && !shutdown.load(Ordering::Relaxed) {
            match slot.read_one(50) {
                Ok(Some(sample)) => {
                    consecutive_errors = 0;
                    last_sample_at = Instant::now();
                    if sample.timestamp_us == last_imu_ts {
                        stale_count += 1;
                        if stale_count >= STALE_THRESHOLD {
                            eprintln!(
                                "triton: IMU timestamp frozen for {} samples — Steam likely disabled IMU; reopening slot",
                                STALE_THRESHOLD
                            );
                            break;
                        }
                    } else {
                        stale_count = 0;
                        last_imu_ts = sample.timestamp_us;
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
                        break;
                    }
                }
                Err(e) => {
                    consecutive_errors += 1;
                    if consecutive_errors >= 5 {
                        eprintln!("triton: 5 consecutive read errors ({e}); reopening slot");
                        break;
                    }
                }
            }
        }
        eprintln!(
            "triton: closing slot (dsu_wants={}, ui_wants={})",
            dsu_wants.load(Ordering::Relaxed),
            ui_wants.load(Ordering::Relaxed)
        );
    }
}
