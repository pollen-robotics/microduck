//! The robot's `md1:` setup code: where it comes from, showing it as a QR, and handing its key to
//! the services that need it.
//!
//! The code is `md1:<first name>:<64 hex characters>` — the robot's first name (derived from the
//! SoC serial, so a rename does not change it) and its 32-byte key. The phone app scans it, and
//! `btd` and `duck-gateway` derive the same first name to answer with.
//!
//! **Two sources, the EEPROM first.** A production robot carries its code in its ID EEPROM, written
//! by the factory with an external programmer and printed as the QR on the robot
//! ([`robotd_params::label`] owns the layout, `factory-provisioning.md` the process). A dev board
//! with nothing there can have a key installed on the data partition with `install`. The id in an
//! EEPROM code must be this robot's first name: one that is not is somebody else's EEPROM image or
//! label, and is refused rather than served.
//!
//! **`export` is how the services get it.** `configd`'s unit runs it as root before starting, and
//! it leaves the key in systemd's credential store in `/run` (memory, root-only). `btd` and
//! `duck-gateway` import it from there and turn secure mode on when it is present; when it is not,
//! `btd` serves the PIN path as before.
//!
//! **The robot never generates its own key.** The factory, or on a dev board whoever plays it,
//! generates the 32 bytes. `install` reads them on stdin, so they appear in no command line and no
//! shell history, and never prints them back.

use std::fs::{self, DirBuilder, File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt};
use std::path::Path;

use robotd_params::label::{self, SetupCode};

use crate::{Failure, exit};

/// Where `install` keeps a dev board's key, on the data partition.
pub const KEY_PATH: &str = "/var/lib/robot/config/ble-psk";

/// Where `export` leaves the key for the services: systemd's credential store, searched by
/// `ImportCredential=ble-psk`. In `/run`, so it is never on disk and is rebuilt every boot.
const CREDSTORE: &str = "/run/credstore";
const CREDENTIAL: &str = "/run/credstore/ble-psk";

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

/// Where this robot's code came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    Eeprom,
    File,
}

impl Source {
    fn describe(self) -> &'static str {
        match self {
            Self::Eeprom => "the ID EEPROM",
            Self::File => KEY_PATH,
        }
    }
}

/// This robot's setup code, or `None` when it has none — the EEPROM's if it carries one, else the
/// one `install` left. Root only: both are root's.
fn current() -> Result<Option<(SetupCode, Source)>, Failure> {
    let first_name = first_name()?;
    match label::setup_code() {
        Ok(Some(code)) => {
            return matches_first_name(code, &first_name).map(|code| Some((code, Source::Eeprom)));
        }
        Ok(None) => {}
        Err(why) => {
            return Err(Failure::new(
                exit::FAILED,
                format!("the ID EEPROM holds no valid setup code: {why}"),
            ));
        }
    }
    match fs::read_to_string(KEY_PATH) {
        Ok(text) => {
            let key = parse_key(&text).map_err(|why| {
                Failure::new(
                    exit::FAILED,
                    format!("{KEY_PATH} is not a valid key: {why}"),
                )
            })?;
            Ok(Some((
                SetupCode {
                    id: first_name,
                    key,
                },
                Source::File,
            )))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(Failure::new(
            exit::FAILED,
            format!("cannot read {KEY_PATH}: {e}"),
        )),
    }
}

/// An EEPROM code is only this robot's if its id is this robot's first name. Anything else means
/// the chip was programmed with another unit's code, and its QR is on that unit.
fn matches_first_name(code: SetupCode, first_name: &str) -> Result<SetupCode, Failure> {
    if code.id == first_name {
        return Ok(code);
    }
    Err(Failure::new(
        exit::REFUSED,
        format!(
            "the ID EEPROM holds the setup code for {}, but this robot is {first_name}: it was \
             programmed with another unit's code. Secure mode stays off until it is reprogrammed",
            code.id
        ),
    ))
}

fn need_root(what: &str) -> Result<(), Failure> {
    // Safety: geteuid cannot fail and touches no memory.
    if unsafe { libc::geteuid() } != 0 {
        return Err(Failure::new(
            exit::DENIED,
            format!("{what} needs root: the key is root's"),
        ));
    }
    Ok(())
}

/// `robotctl system setup-code`: the code, as text and as a QR to scan off the terminal.
pub fn show() -> Result<(), Failure> {
    need_root("sudo robotctl system setup-code: showing the code")?;
    let Some((code, source)) = current()? else {
        return Err(Failure::new(
            exit::REFUSED,
            format!(
                "no setup code on this robot: nothing in the ID EEPROM and no {KEY_PATH}. \
                 On a dev board, install one with `sudo robotctl system setup-code install`, \
                 the key on stdin"
            ),
        ));
    };
    // The whole line, as printed on the robot: label serial, separator, code. A dev board with no
    // label serial in its EEPROM shows the code alone; the app reads both.
    let serial = label::read()
        .ok()
        .flatten()
        .and_then(|raw| label::label_serial_in(&raw));
    let line = label::qr_line(serial.as_deref(), &code);
    let modules = qr_modules(&line)
        .ok_or_else(|| Failure::new(exit::FAILED, "cannot encode the QR".to_owned()))?;
    print!("{}", terminal_qr(&modules));
    println!("robot   {}", code.id);
    println!("from    {}", source.describe());
    println!("code    {line}");
    println!("Anyone who sees this code can control this robot.");
    Ok(())
}

/// `robotctl system setup-code export`: leave the key where `btd` and `duck-gateway` import it.
///
/// Run by `configd.service` before it starts, which both services are ordered after. Always
/// leaves the store matching the robot: the key when there is a valid code, nothing when there is
/// none or it is invalid — so a stale key from a previous boot or source can never be served.
pub fn export() -> Result<(), Failure> {
    need_root("exporting the key")?;
    let found = current();
    match &found {
        Ok(Some((code, source))) => {
            DirBuilder::new()
                .recursive(true)
                .mode(0o700)
                .create(CREDSTORE)
                .map_err(|e| {
                    Failure::new(exit::FAILED, format!("cannot create {CREDSTORE}: {e}"))
                })?;
            write_key(Path::new(CREDENTIAL), &code.key, true).map_err(|e| {
                Failure::new(exit::FAILED, format!("cannot write {CREDENTIAL}: {e:?}"))
            })?;
            println!(
                "setup code for {} from {}: secure mode on",
                code.id,
                source.describe()
            );
        }
        Ok(None) | Err(_) => {
            match fs::remove_file(CREDENTIAL) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => {
                    return Err(Failure::new(
                        exit::FAILED,
                        format!("cannot remove the stale {CREDENTIAL}: {e}"),
                    ));
                }
            }
            if matches!(found, Ok(None)) {
                println!("no setup code on this robot: btd serves the PIN path");
            }
        }
    }
    found.map(|_| ())
}

/// `robotctl system setup-code install`: commit the key read on stdin.
pub fn install(force: bool) -> Result<(), Failure> {
    need_root("installing the key (… | sudo robotctl system setup-code install)")?;
    // Before reading anything: a board that cannot answer with an id cannot use the key.
    let id = first_name()?;
    // Only a blank EEPROM leaves room for an installed key. One holding a code uses that code, and
    // one holding something unreadable makes `export` refuse every key — a key installed here
    // would be reported as installed and never used.
    match label::setup_code() {
        Ok(None) => {}
        Ok(Some(_)) => {
            return Err(Failure::new(
                exit::REFUSED,
                "this robot's ID EEPROM already holds its setup code, which is the one printed on \
                 it and the one it uses. `install` is for dev boards with nothing there"
                    .to_owned(),
            ));
        }
        Err(why) => {
            return Err(Failure::new(
                exit::REFUSED,
                format!(
                    "the ID EEPROM holds no valid setup code ({why}), and while it does the robot \
                     uses no key at all and serves the PIN path. Reprogram the EEPROM; an \
                     installed key would not be used"
                ),
            ));
        }
    }
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
    println!("use it:   sudo systemctl restart configd btd duck-gateway");
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
/// **Error correction L**, the smallest code for a given text: the full line — 18 digits, `:`,
/// `md1:duck-xxxx:` and 64 hex characters, 97 in all — is version 5 at worst, 37 modules and the
/// quiet zone drawing 82 columns wide (see [`terminal_qr`]); level M would need version 6 and 90.
/// A terminal is a clean, flat source, which is what the lower correction level gives up nothing
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

    fn code(id: &str, key: &str) -> String {
        SetupCode {
            id: id.to_owned(),
            key: key.to_owned(),
        }
        .text()
    }

    #[test]
    fn the_code_is_what_the_app_parses() {
        assert_eq!(code("duck-ccf2", KEY), format!("md1:duck-ccf2:{KEY}"));
    }

    #[test]
    fn an_eeprom_code_for_another_unit_is_refused() {
        let ours = SetupCode {
            id: "duck-ccf2".to_owned(),
            key: KEY.to_owned(),
        };
        assert_eq!(matches_first_name(ours.clone(), "duck-ccf2").unwrap(), ours);
        let theirs = SetupCode {
            id: "duck-0042".to_owned(),
            key: KEY.to_owned(),
        };
        let refused = matches_first_name(theirs, "duck-ccf2").unwrap_err();
        assert_eq!(refused.code, exit::REFUSED);
        assert!(refused.message.contains("duck-0042"));
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
    fn a_full_line_draws_one_row_per_module_inside_eighty_two_columns() {
        let line = |key: &str| {
            label::qr_line(
                Some("202004743264300098"),
                &SetupCode {
                    id: "duck-ccf2".to_owned(),
                    key: key.to_owned(),
                },
            )
        };
        // Runs of digits encode more compactly, so the size depends on the key. A key with none is
        // the largest: version 5, 37 modules, and the quiet zone on both sides.
        let worst = qr_modules(&line(&"ab".repeat(32))).unwrap();
        assert_eq!(worst.len(), 37 + 4);
        for modules in [
            worst,
            qr_modules(&line(KEY)).unwrap(),
            qr_modules(&code("duck-ccf2", KEY)).unwrap(),
        ] {
            let text = terminal_qr(&modules);
            assert_eq!(text.lines().count(), modules.len());
            let widest = text
                .lines()
                .map(|line| line.matches("  ").count() * 2)
                .max()
                .unwrap();
            assert!(widest <= 82, "{widest} columns");
        }
    }
}
