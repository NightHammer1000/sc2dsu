// Persisted user config — axis mapping, port, etc. Lives at
// %APPDATA%\sc2dsu\config.toml. Re-read on every IMU sample so edits to the
// file take effect without restarting the server.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::RwLock;

/// Which raw IMU axis (X / Y / Z, 0..=2) to source each DSU output axis from,
/// plus a sign flip per output. The default reproduces SDL's mapping for the
/// Triton, which is correct for SDL gyro consumers but Eden displays the axes
/// in a different order — adjust here without rebuilding.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Axis {
    /// Raw IMU axis index (0 = chip X, 1 = chip Y, 2 = chip Z).
    pub source: u8,
    /// If true, multiply by -1 before sending.
    pub invert: bool,
}

impl Axis {
    pub const fn new(source: u8, invert: bool) -> Self {
        Self { source, invert }
    }
    pub fn apply(&self, raw: [f32; 3]) -> f32 {
        let v = raw[(self.source as usize).min(2)];
        if self.invert { -v } else { v }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct AxisMap {
    pub x: Axis,
    pub y: Axis,
    pub z: Axis,
}

impl Default for AxisMap {
    /// SDL's mapping for the Triton: DSU_X = raw_X, DSU_Y = raw_Z, DSU_Z = -raw_Y.
    fn default() -> Self {
        Self {
            x: Axis::new(0, false),
            y: Axis::new(2, false),
            z: Axis::new(1, true),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    /// UDP port for the DSU server.
    pub port: u16,
    /// Axis remap for the gyroscope.
    pub gyro: AxisMap,
    /// Axis remap for the accelerometer (usually the same — they share the chip).
    pub accel: AxisMap,
    /// Hide the settings window at launch and live in the tray. Honored on every
    /// startup; the `--tray` CLI flag forces this on for one launch regardless.
    pub start_minimized: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            port: 26760,
            gyro: AxisMap::default(),
            accel: AxisMap::default(),
            start_minimized: false,
        }
    }
}

pub fn config_path() -> PathBuf {
    let dir = dirs::config_dir().unwrap_or_else(|| PathBuf::from("."));
    dir.join("sc2dsu").join("config.toml")
}

pub fn load_or_create() -> Config {
    let path = config_path();
    if let Ok(s) = std::fs::read_to_string(&path) {
        match toml::from_str::<Config>(&s) {
            Ok(c) => {
                eprintln!("config: loaded {}", path.display());
                return c;
            }
            Err(e) => {
                eprintln!(
                    "config: {} is malformed ({e}); using defaults and not overwriting",
                    path.display()
                );
                return Config::default();
            }
        }
    }
    let c = Config::default();
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    if let Ok(s) = toml::to_string_pretty(&c) {
        match std::fs::write(&path, s) {
            Ok(()) => eprintln!("config: wrote default {}", path.display()),
            Err(e) => eprintln!("config: failed to write {}: {e}", path.display()),
        }
    }
    c
}

/// Process-wide live config. The reader thread re-locks for read on every IMU
/// sample, so edits to the config file picked up by the watcher take effect
/// immediately. Writes are rare (manual edit + reload, or future UI).
pub static LIVE: RwLock<Config> = RwLock::new(Config {
    port: 26760,
    gyro: AxisMap {
        x: Axis::new(0, false),
        y: Axis::new(2, false),
        z: Axis::new(1, true),
    },
    accel: AxisMap {
        x: Axis::new(0, false),
        y: Axis::new(2, false),
        z: Axis::new(1, true),
    },
    start_minimized: false,
});

pub fn install(initial: Config) {
    if let Ok(mut g) = LIVE.write() {
        *g = initial;
    }
}

pub fn snapshot() -> Config {
    LIVE.read()
        .map(|g| g.clone())
        .unwrap_or_else(|_| Config::default())
}

/// Replace the live config and persist it to disk. Returns Err on disk-write failure;
/// the in-memory config is updated either way (so the change takes effect immediately
/// even if persistence fails — better than dropping the user's edit).
pub fn update_and_save(new_cfg: Config) -> std::io::Result<()> {
    if let Ok(mut g) = LIVE.write() {
        *g = new_cfg.clone();
    }
    let path = config_path();
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let s = toml::to_string_pretty(&new_cfg)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    std::fs::write(&path, s)
}
