use std::fs;
use std::io::Write;
use std::path::Path;

fn main() {
    let out = Path::new("assets/tray.ico");
    if let Some(parent) = out.parent() {
        fs::create_dir_all(parent).expect("create assets/");
    }
    if !out.exists() {
        write_solid_ico(out, 16, [60, 130, 220, 255]).expect("write tray.ico");
    }

    if std::env::var_os("CARGO_CFG_WINDOWS").is_some() {
        use embed_manifest::manifest::{DpiAwareness, Setting};
        embed_manifest::embed_manifest(
            embed_manifest::new_manifest("sc2dsu")
                .dpi_awareness(DpiAwareness::Unaware)
                .gdi_scaling(Setting::Enabled),
        )
        .expect("embed manifest");
    }

    println!("cargo:rerun-if-changed=build.rs");
}

fn write_solid_ico(path: &Path, size: u32, color: [u8; 4]) -> std::io::Result<()> {
    let mut f = fs::File::create(path)?;
    let xor_size = size * size * 4;
    let row_bytes = size.div_ceil(32) * 4;
    let and_size = row_bytes * size;
    let bmp_size = 40 + xor_size + and_size;

    f.write_all(&[0, 0, 1, 0, 1, 0])?;
    f.write_all(&[size as u8, size as u8, 0, 0, 1, 0, 32, 0])?;
    f.write_all(&bmp_size.to_le_bytes())?;
    f.write_all(&22u32.to_le_bytes())?;

    f.write_all(&40u32.to_le_bytes())?;
    f.write_all(&(size as i32).to_le_bytes())?;
    f.write_all(&((size * 2) as i32).to_le_bytes())?;
    f.write_all(&1u16.to_le_bytes())?;
    f.write_all(&32u16.to_le_bytes())?;
    f.write_all(&0u32.to_le_bytes())?;
    f.write_all(&0u32.to_le_bytes())?;
    f.write_all(&0i32.to_le_bytes())?;
    f.write_all(&0i32.to_le_bytes())?;
    f.write_all(&0u32.to_le_bytes())?;
    f.write_all(&0u32.to_le_bytes())?;

    for _ in 0..(size * size) {
        f.write_all(&[color[2], color[1], color[0], color[3]])?;
    }
    for _ in 0..and_size {
        f.write_all(&[0u8])?;
    }
    Ok(())
}
