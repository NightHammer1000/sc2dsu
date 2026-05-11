use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::RwLock;
use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct AxisMap {
    pub x: Axis,
    pub y: Axis,
    pub z: Axis,
}

impl AxisMap {
    pub const DEFAULT: Self = Self {
        x: Axis::new(0, false),
        y: Axis::new(2, false),
        z: Axis::new(1, true),
    };
}

impl Default for AxisMap {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    pub port: u16,
    pub gyro: AxisMap,
    pub accel: AxisMap,
    pub start_minimized: bool,
    pub expose_to_network: bool,
    pub close_to_tray: bool,
}

impl Config {
    pub const DEFAULT: Self = Self {
        port: 26760,
        gyro: AxisMap::DEFAULT,
        accel: AxisMap::DEFAULT,
        start_minimized: false,
        expose_to_network: false,
        close_to_tray: false,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
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

static LIVE: RwLock<Config> = RwLock::new(Config::DEFAULT);

static GENERATION: AtomicU64 = AtomicU64::new(0);

pub fn generation() -> u64 {
    GENERATION.load(Ordering::Acquire)
}

pub fn bind_host(expose_to_network: bool) -> &'static str {
    if expose_to_network {
        "0.0.0.0"
    } else {
        "127.0.0.1"
    }
}

pub fn install(initial: Config) {
    *LIVE.write().unwrap_or_else(|e| e.into_inner()) = initial;
    GENERATION.fetch_add(1, Ordering::Release);
}

pub fn snapshot() -> Config {
    LIVE.read().unwrap_or_else(|e| e.into_inner()).clone()
}

pub fn update_and_save(new_cfg: Config) -> std::io::Result<()> {
    *LIVE.write().unwrap_or_else(|e| e.into_inner()) = new_cfg.clone();
    GENERATION.fetch_add(1, Ordering::Release);
    let path = config_path();
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let s = toml::to_string_pretty(&new_cfg)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    std::fs::write(&path, s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_apply_selects_and_inverts() {
        let raw = [1.0, 2.0, 3.0];
        assert_eq!(Axis::new(0, false).apply(raw), 1.0);
        assert_eq!(Axis::new(2, false).apply(raw), 3.0);
        assert_eq!(Axis::new(1, true).apply(raw), -2.0);
    }

    #[test]
    fn axis_apply_clamps_out_of_range_source() {
        let raw = [1.0, 2.0, 3.0];
        assert_eq!(Axis::new(7, false).apply(raw), 3.0);
    }

    #[test]
    fn config_toml_round_trips() {
        let original = Config::default();
        let text = toml::to_string_pretty(&original).unwrap();
        let parsed: Config = toml::from_str(&text).unwrap();
        assert_eq!(original, parsed);
    }

    #[test]
    fn config_partial_toml_uses_defaults() {
        let parsed: Config = toml::from_str("port = 12345").unwrap();
        assert_eq!(parsed.port, 12345);
        assert_eq!(parsed.gyro, AxisMap::DEFAULT);
        assert_eq!(parsed.accel, AxisMap::DEFAULT);
        assert!(!parsed.expose_to_network);
    }

    #[test]
    fn bind_host_maps_flag() {
        assert_eq!(bind_host(true), "0.0.0.0");
        assert_eq!(bind_host(false), "127.0.0.1");
    }
}
