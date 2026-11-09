//! The robot's `md1:` setup code: installing its key, and showing it as a QR.
//!
//! The code is `md1:<first name>:<64 hex characters>` — the robot's first name (derived from the
//! SoC serial, so a rename does not change it) and its 32-byte key. The phone app scans it, and
//! `btd` and `duck-gateway` derive the same first name to answer with
//! (`factory-provisioning.md` owns the factory sequence this serves).
//!
//! **The robot never generates its own key.** A provisioning station (or, on a dev board, whoever
//! plays it) generates the 32 bytes and hands them over on stdin, so they appear in no command line
//! and no shell history. `install` is that station's helper on the robot: it checks the key, checks
//! the data partition is really there, and commits the file atomically. It never prints the key —
//! `show` does that, for whoever is printing or scanning the label.

use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
use std::path::Path;

use crate::{Failure, exit};

/// Where the key lives: what `btd`'s and `duck-gateway`'s `LoadCredential=` read.
pub const KEY_PATH: &str = "/var/lib/robot/config/ble-psk";

/// The persistent data partition the key must land on.
const DATA_DIR: &str = "/var/lib/robot";

/// Where an image that mounts a data partition at [`DATA_DIR`] declares it. On such an image a
/// [`DATA_DIR`] that is not a mount point is the rootfs fallback, and a key written there would
/// vanish with the next A/B update. A board with no such unit (a hand-provisioned Zero 3) keeps
/// its data on the rootfs by design, and is not refused.
const DATA_MOUNT_UNITS: [&str; 3] = [
    "/etc/systemd/system/var-lib-robot.mount",
    "/usr/lib/systemd/system/var-lib-robot.mount",
    "/lib/systemd/system/var-lib-robot.mount",
];

/// `robotctl system setup-code`: the code, as text and as a QR to scan off the terminal.
pub fn show() -> Result<(), Failure> {
    let id = first_name()?;
    let key = match fs::read_to_string(KEY_PATH) {
        Ok(text) => parse_key(&text).map_err(|why| {
            Failure::new(
                exit::FAILED,
                format!("{KEY_PATH} is not a valid key: {why}"),
            )
        })?,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(Failure::new(
                exit::REFUSED,
                format!(
                    "no setup code on this robot ({KEY_PATH} is missing). Install one with \
                     `sudo robotctl system setup-code install`, the key on stdin"
                ),
            ));
        }
        Err(e) if e.kind() == std::io::ErrorKind::PermissionDenied => {
            return Err(Failure::new(
                exit::DENIED,
                "the key is root's: sudo robotctl system setup-code".to_owned(),
            ));
        }
        Err(e) => {
            return Err(Failure::new(
                exit::FAILED,
                format!("cannot read {KEY_PATH}: {e}"),
            ));
        }
    };
    let code = code(&id, &key);
    let modules = qr_modules(&code)
        .ok_or_else(|| Failure::new(exit::FAILED, "cannot encode the QR".to_owned()))?;
    print!("{}", terminal_qr(&modules));
    println!("robot  {id}");
    println!("code   {code}");
    println!("Anyone who sees this code can control this robot.");
    Ok(())
}

/// `robotctl system setup-code install`: commit the key read on stdin.
pub fn install(force: bool) -> Result<(), Failure> {
    // Safety: geteuid cannot fail and touches no memory.
    if unsafe { libc::geteuid() } != 0 {
        return Err(Failure::new(
            exit::DENIED,
            "installing the key needs root: … | sudo robotctl system setup-code install".to_owned(),
        ));
    }
    // Before reading anything: a board that cannot answer with an id cannot use the key.
    let id = first_name()?;
    if DATA_MOUNT_UNITS.iter().any(|unit| Path::new(unit).exists())
        && !is_mount_point(Path::new(DATA_DIR))
    {
        return Err(Failure::new(
            exit::REFUSED,
            format!(
                "{DATA_DIR} is not mounted, so the key would land on the root filesystem and be \
                 lost at the next update. Check `systemctl status var-lib-robot.mount`"
            ),
        ));
    }

    let mut text = String::new();
    std::io::stdin()
        .take(1024)
        .read_to_string(&mut text)
        .map_err(|e| Failure::new(exit::USAGE, format!("cannot read the key on stdin: {e}")))?;
    let key = parse_key(&text).map_err(|why| Failure::new(exit::USAGE, why))?;

    write_key(Path::new(KEY_PATH), &key, force).map_err(|e| match e {
        WriteError::Exists => Failure::new(
            exit::REFUSED,
            "this robot already has a key, and replacing it invalidates its printed QR and \
             every phone holding the old code. Re-run with --force if that is what you want"
                .to_owned(),
        ),
        WriteError::Io(e) => Failure::new(exit::FAILED, format!("cannot write {KEY_PATH}: {e}")),
    })?;
    println!("installed the setup code key for {id}");
    println!("show it:  sudo robotctl system setup-code");
    println!("btd and duck-gateway use it from their next start");
    Ok(())
}

fn first_name() -> Result<String, Failure> {
    robotd_params::serial::this_first_name().ok_or_else(|| {
        Failure::new(
            exit::REFUSED,
            "this board has no SoC serial, so it has no first name to put in a setup code"
                .to_owned(),
        )
    })
}

/// The 64 hex characters of a 32-byte key, lowercased. Surrounding whitespace — the newline a
/// shell pipe adds — is tolerated; anything else is not.
fn parse_key(text: &str) -> Result<String, String> {
    let key = text.trim();
    if key.len() != 64 {
        return Err(format!(
            "the key must be 64 hex characters (32 bytes), got {}",
            key.len()
        ));
    }
    if !key.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("the key must be hexadecimal".to_owned());
    }
    Ok(key.to_ascii_lowercase())
}

/// The setup code, as the app reads it and the QR carries it.
fn code(id: &str, key: &str) -> String {
    format!("md1:{id}:{key}")
}

#[derive(Debug)]
enum WriteError {
    Exists,
    Io(std::io::Error),
}

impl From<std::io::Error> for WriteError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Commit `key` at `path`, root-only, atomically: a power cut leaves the old key or the new one,
/// never half of either. Refuses to replace an existing key unless `force`.
fn write_key(path: &Path, key: &str, force: bool) -> Result<(), WriteError> {
    if !force && path.exists() {
        return Err(WriteError::Exists);
    }
    let dir = path
        .parent()
        .ok_or_else(|| WriteError::Io(std::io::Error::other("the key path has no directory")))?;
    fs::create_dir_all(dir)?;
    let staging = dir.join(".ble-psk.new");
    // A leftover from an interrupted run holds nothing anyone relies on.
    let _ = fs::remove_file(&staging);
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(&staging)?;
    file.write_all(key.as_bytes())?;
    file.sync_all()?;
    drop(file);
    fs::rename(&staging, path)?;
    File::open(dir)?.sync_all()?;
    Ok(())
}

/// A mount point sits on a different device from its parent directory.
fn is_mount_point(path: &Path) -> bool {
    let (Ok(here), Some(Ok(parent))) = (fs::metadata(path), path.parent().map(fs::metadata)) else {
        return false;
    };
    here.dev() != parent.dev()
}

/// The modules of `text`'s QR, `true` for dark, with a two-module quiet zone.
///
/// **Error correction L**, which keeps every setup code at version 4: `md1:duck-xxxx:` and 64 hex
/// characters is 78 bytes, exactly what 4-L holds. 33 modules and the quiet zone draw 74 columns
/// wide (see [`terminal_qr`]), inside an 80-column terminal; level M needs version 5 and 82. A
/// terminal is a clean, flat source, which is what the lower correction level gives up nothing
/// on.
fn qr_modules(text: &str) -> Option<Vec<Vec<bool>>> {
    let code =
        qrcode::QrCode::with_error_correction_level(text.as_bytes(), qrcode::EcLevel::L).ok()?;
    let width = code.width();
    let colors = code.to_colors();
    const QUIET: usize = 2;
    let side = width + 2 * QUIET;
    let mut rows = vec![vec![false; side]; side];
    for y in 0..width {
        for x in 0..width {
            rows[y + QUIET][x + QUIET] = colors[y * width + x] == qrcode::Color::Dark;
        }
    }
    Some(rows)
}

/// One module per row, two spaces wide, painted with the **background** colour only.
///
/// Not half blocks (`▀`), which put two modules in one row: how tall that glyph is depends on the
/// font, and on macOS Terminal it stops short of the cell's middle, so every dark module above a
/// light one came out thinner than its neighbours and phones would not read the code. A
/// background fills the whole cell, line spacing included, whatever the font. Black on white
/// whatever the terminal's theme: a camera reads dark-on-light.
fn terminal_qr(modules: &[Vec<bool>]) -> String {
    let mut out = String::new();
    for row in modules {
        for &dark in row {
            out.push_str(if dark {
                "\x1b[48;2;0;0;0m  "
            } else {
                "\x1b[48;2;255;255;255m  "
            });
        }
        out.push_str("\x1b[0m\n");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    const KEY: &str = "71717171717171717171717171717171717171717171717171717171717171aa";

    #[test]
    fn a_piped_key_is_accepted_lowercased_and_anything_else_is_refused() {
        assert_eq!(
            parse_key(&format!("{}\n", KEY.to_uppercase())).unwrap(),
            KEY
        );
        assert!(parse_key(&KEY[..62]).is_err());
        assert!(parse_key(&format!("{KEY}00")).is_err());
        assert!(parse_key(&format!("{}zz", &KEY[..62])).is_err());
        assert!(parse_key(&format!("md1:duck-ccf2:{KEY}")).is_err());
    }

    #[test]
    fn the_code_is_what_the_app_parses() {
        assert_eq!(code("duck-ccf2", KEY), format!("md1:duck-ccf2:{KEY}"));
    }

    #[test]
    fn the_key_is_written_root_only_and_never_silently_replaced() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("config").join("ble-psk");
        write_key(&path, KEY, false).unwrap();
        assert_eq!(fs::read_to_string(&path).unwrap(), KEY);
        assert_eq!(
            fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );

        let other = "00".repeat(32);
        assert!(matches!(
            write_key(&path, &other, false),
            Err(WriteError::Exists)
        ));
        assert_eq!(fs::read_to_string(&path).unwrap(), KEY);

        write_key(&path, &other, true).unwrap();
        assert_eq!(fs::read_to_string(&path).unwrap(), other);
        assert!(!path.parent().unwrap().join(".ble-psk.new").exists());
    }

    #[test]
    fn a_plain_directory_is_not_a_mount_point() {
        let dir = tempfile::tempdir().unwrap();
        let inner = dir.path().join("robot");
        fs::create_dir(&inner).unwrap();
        assert!(!is_mount_point(&inner));
    }

    #[test]
    fn a_setup_code_draws_one_row_per_module_inside_eighty_columns() {
        // Runs of digits in the hex encode more compactly, so the size depends on the key. A key
        // with none is the largest: version 4, 33 modules, and the quiet zone on both sides.
        let worst = qr_modules(&code("duck-ccf2", &"ab".repeat(32))).unwrap();
        assert_eq!(worst.len(), 33 + 4);
        for modules in [worst, qr_modules(&code("duck-ccf2", KEY)).unwrap()] {
            let text = terminal_qr(&modules);
            assert_eq!(text.lines().count(), modules.len());
            let widest = text
                .lines()
                .map(|line| line.matches("  ").count() * 2)
                .max()
                .unwrap();
            assert!(widest <= 80, "{widest} columns");
        }
    }
}
