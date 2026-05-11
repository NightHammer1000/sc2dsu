use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::RwLock;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Axis {
    pub source: u8,
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
    pub port: u16,
    pub gyro: AxisMap,
    pub accel: AxisMap,
    pub start_minimized: bool,
    pub expose_to_network: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            port: 26760,
            gyro: AxisMap::default(),
            accel: AxisMap::default(),
            start_minimized: false,
            expose_to_network: false,
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
    expose_to_network: false,
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
