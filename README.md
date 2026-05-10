# SC2DSU

Cemuhook DSU server for the 2026 Steam Controller's gyro and accelerometer. Runs on `127.0.0.1:26760`. Steam Input handles buttons; this handles motion.

Download `sc2dsu.exe` from [Releases](https://github.com/NightHammer1000/sc2dsu/releases) and run it. Plug in the Puck, point your emulator at `127.0.0.1:26760`, done. Only tested with Eden over the Proteus Puck.

If an axis is wrong, swap the source or flip invert in the settings window. Saved live; takes effect on the next IMU sample. Config lives at `%APPDATA%\sc2dsu\config.toml`.

Run modes: `sc2dsu` (GUI + server), `sc2dsu --tray` (start hidden), `sc2dsu --headless` (server only, log to stderr), `sc2dsu --probe` (enumerate Valve HIDs and dump 3 s of decoded IMU).

Only the Proteus Puck (`0x1304`) was actually plugged in during development. Wired (`0x1302`), BLE (`0x1303`), and Nereid Puck (`0x1305`) are listed in SDL's Triton driver as the same family, so the code path treats them identically — but I have no idea if any of that actually works on real hardware. Reports welcome.

Build with `cargo build --release`. CI runs `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo build --release --locked` on every push.

HID protocol from SDL3 [`SDL_hidapi_steam_triton.c`](https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steam_triton.c) and [steam/](https://github.com/libsdl-org/SDL/tree/main/src/joystick/hidapi/steam) headers. DSU protocol from [v1993/gcemuhook](https://github.com/v1993/gcemuhook). MIT.
