// Windows "run at logon" autostart toggle, via the per-user
// HKCU\Software\Microsoft\Windows\CurrentVersion\Run registry key.

use std::io;
use winreg::RegKey;
use winreg::enums::*;

const RUN_KEY: &str = r"Software\Microsoft\Windows\CurrentVersion\Run";
const VALUE_NAME: &str = "SC2DSU";

fn current_exe_quoted() -> io::Result<String> {
    let exe = std::env::current_exe()?;
    // Quote so paths with spaces survive parsing by Windows.
    Ok(format!("\"{}\" --tray", exe.display()))
}

pub fn is_enabled() -> bool {
    let hkcu = RegKey::predef(HKEY_CURRENT_USER);
    let key = match hkcu.open_subkey(RUN_KEY) {
        Ok(k) => k,
        Err(_) => return false,
    };
    let val: io::Result<String> = key.get_value(VALUE_NAME);
    val.is_ok()
}

pub fn enable() -> io::Result<()> {
    let hkcu = RegKey::predef(HKEY_CURRENT_USER);
    let (key, _) = hkcu.create_subkey(RUN_KEY)?;
    let cmd = current_exe_quoted()?;
    key.set_value(VALUE_NAME, &cmd)
}

pub fn disable() -> io::Result<()> {
    let hkcu = RegKey::predef(HKEY_CURRENT_USER);
    let key = hkcu.open_subkey_with_flags(RUN_KEY, KEY_SET_VALUE)?;
    match key.delete_value(VALUE_NAME) {
        Ok(()) => Ok(()),
        // Already absent → fine.
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(e),
    }
}
