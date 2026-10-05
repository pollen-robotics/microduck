//! `robotctl led` — the face board's LEDs, straight through the kernel's LED class.
//!
//! The beta board's face carries twelve LEDs on a PCA9535 expander, which its device tree hands
//! to `gpio-leds` as `face:<place>:<colour>` under `/sys/class/leds` (the Zero 3W has none, and
//! says so). This is the bench tool for them: list, switch, blink, or hand one to a kernel
//! trigger.
//!
//! It writes sysfs itself rather than asking a daemon, because no daemon owns an LED yet. When
//! one does — the camera LED for `mediad`'s streaming indicator (architecture.md §7), a status
//! LED for whoever owns that status — that daemon is the LED's single writer (§1.1, invariant 4)
//! and this stays a way to look and to try things out. The image gives the `robot` group write
//! access to `brightness` and `trigger`; `delay_on`/`delay_off` only appear once the `timer`
//! trigger is set and are root's, so a custom blink rate needs sudo while the default 500/500
//! does not.

use std::fs;
use std::path::Path;

use clap::Subcommand;

use super::{Failure, exit};

type Result<T> = std::result::Result<T, Failure>;

/// Where the kernel puts LED class devices.
pub(super) const LEDS_DIR: &str = "/sys/class/leds";

/// The prefix the beta DTS gives every face LED.
const FACE: &str = "face:";

#[derive(Subcommand, Debug)]
pub(super) enum LedCommand {
    /// Every face LED: brightness and the active trigger.
    List,
    /// Switch LEDs on or off, or to a brightness. `all` means every face LED.
    Set {
        /// An LED's name, whole (`face:rgb:red`) or without the `face:` prefix (`rgb:red`),
        /// or `all`.
        led: String,
        /// `on`, `off`, or a brightness from 0 to the LED's maximum.
        value: String,
    },
    /// Blink an LED with the kernel's `timer` trigger.
    Blink {
        led: String,
        /// Milliseconds on. Anything but the default 500 needs root (see the module doc).
        #[arg(long, default_value_t = 500)]
        on_ms: u32,
        /// Milliseconds off. Same caveat.
        #[arg(long, default_value_t = 500)]
        off_ms: u32,
    },
    /// Hand an LED to a kernel trigger (`none`, `heartbeat`, `timer`, ...). The LED's
    /// `/sys/class/leds/<led>/trigger` lists the choices.
    Trigger { led: String, trigger: String },
}

pub(super) fn run(dir: &Path, command: LedCommand) -> Result<()> {
    let leds = face_leds(dir)?;
    match command {
        LedCommand::List => {
            for led in &leds {
                let path = dir.join(led);
                println!(
                    "{led:<20} {:>3}/{:<3} {}",
                    read(&path.join("brightness")).unwrap_or_default(),
                    read(&path.join("max_brightness")).unwrap_or_default(),
                    active_trigger(&read(&path.join("trigger")).unwrap_or_default()),
                );
            }
        }
        LedCommand::Set { led, value } => {
            for name in select(&leds, &led)? {
                let path = dir.join(name);
                let level = match value.as_str() {
                    "on" => read(&path.join("max_brightness"))?,
                    "off" => "0".to_owned(),
                    n if n.parse::<u32>().is_ok() => n.to_owned(),
                    n => return Err(usage(format!("{n:?} is not on, off or a brightness"))),
                };
                // Writing a brightness drops any trigger, which is what "switch it" means.
                write(&path.join("brightness"), &level)?;
            }
        }
        LedCommand::Blink { led, on_ms, off_ms } => {
            for name in select(&leds, &led)? {
                let path = dir.join(name);
                write(&path.join("trigger"), "timer")?;
                if (on_ms, off_ms) != (500, 500) {
                    write(&path.join("delay_on"), &on_ms.to_string())?;
                    write(&path.join("delay_off"), &off_ms.to_string())?;
                }
            }
        }
        LedCommand::Trigger { led, trigger } => {
            for name in select(&leds, &led)? {
                write(&dir.join(name).join("trigger"), &trigger)?;
            }
        }
    }
    Ok(())
}

fn usage(message: String) -> Failure {
    Failure::new(exit::USAGE, message)
}

/// The face LEDs present, sorted.
fn face_leds(dir: &Path) -> Result<Vec<String>> {
    let entries = fs::read_dir(dir)
        .map_err(|e| Failure::new(exit::FAILED, format!("reading {}: {e}", dir.display())))?;
    let mut leds: Vec<String> = entries
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| n.starts_with(FACE))
        .collect();
    if leds.is_empty() {
        return Err(Failure::new(
            exit::FAILED,
            format!(
                "no face LEDs under {} — this board has no face board, or its device tree \
                 does not describe one (the beta board's does)",
                dir.display()
            ),
        ));
    }
    leds.sort();
    Ok(leds)
}

/// The LEDs a name means: `all`, an exact name, or a name without the `face:` prefix.
fn select<'a>(leds: &'a [String], wanted: &str) -> Result<Vec<&'a String>> {
    if wanted == "all" {
        return Ok(leds.iter().collect());
    }
    let full = if wanted.starts_with(FACE) {
        wanted.to_owned()
    } else {
        format!("{FACE}{wanted}")
    };
    match leds.iter().find(|l| **l == full) {
        Some(l) => Ok(vec![l]),
        None => Err(usage(format!(
            "no LED {wanted:?}; these exist: {}",
            leds.join(", ")
        ))),
    }
}

/// The trigger in `[brackets]` in a `trigger` file, which lists every choice.
fn active_trigger(triggers: &str) -> &str {
    triggers
        .split_whitespace()
        .find_map(|t| t.strip_prefix('[')?.strip_suffix(']'))
        .unwrap_or("?")
}

fn read(path: &Path) -> Result<String> {
    fs::read_to_string(path)
        .map(|s| s.trim().to_owned())
        .map_err(|e| Failure::new(exit::FAILED, format!("reading {}: {e}", path.display())))
}

fn write(path: &Path, value: &str) -> Result<()> {
    fs::write(path, value).map_err(|e| {
        // Permission is the likely failure and has its own exit code: the fix is sudo.
        let code = if e.kind() == std::io::ErrorKind::PermissionDenied {
            exit::DENIED
        } else {
            exit::FAILED
        };
        Failure::new(
            code,
            format!(
                "writing {value:?} to {}: {e} (the robot group can write brightness and \
                 trigger; blink rates need root)",
                path.display()
            ),
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sysfs-shaped tree: two face LEDs and one that is not on the face.
    fn fake_leds() -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        for name in ["face:rgb:red", "face:cam:red", "mmc0::"] {
            let led = dir.path().join(name);
            fs::create_dir(&led).unwrap();
            fs::write(led.join("brightness"), "0\n").unwrap();
            fs::write(led.join("max_brightness"), "1\n").unwrap();
            fs::write(led.join("trigger"), "[none] timer heartbeat\n").unwrap();
        }
        dir
    }

    fn set(led: &str, value: &str) -> LedCommand {
        LedCommand::Set {
            led: led.into(),
            value: value.into(),
        }
    }

    fn blink(led: &str, on_ms: u32, off_ms: u32) -> LedCommand {
        LedCommand::Blink {
            led: led.into(),
            on_ms,
            off_ms,
        }
    }

    #[test]
    fn only_face_leds_are_listed_and_selected() {
        let dir = fake_leds();
        let leds = face_leds(dir.path()).unwrap();
        assert_eq!(leds, ["face:cam:red", "face:rgb:red"]);
        assert_eq!(select(&leds, "rgb:red").unwrap(), [&leds[1]]);
        assert_eq!(select(&leds, "face:cam:red").unwrap(), [&leds[0]]);
        assert_eq!(select(&leds, "all").unwrap().len(), 2);
        assert_eq!(select(&leds, "mmc0::").unwrap_err().code, exit::USAGE);
    }

    #[test]
    fn set_writes_the_maximum_for_on_and_zero_for_off() {
        let dir = fake_leds();
        let led = dir.path().join("face:rgb:red");
        run(dir.path(), set("rgb:red", "on")).unwrap();
        assert_eq!(fs::read_to_string(led.join("brightness")).unwrap(), "1");
        run(dir.path(), set("all", "off")).unwrap();
        assert_eq!(fs::read_to_string(led.join("brightness")).unwrap(), "0");
        let err = run(dir.path(), set("all", "dim")).unwrap_err();
        assert_eq!(err.code, exit::USAGE);
    }

    #[test]
    fn blink_sets_the_timer_and_only_touches_delays_when_asked() {
        let dir = fake_leds();
        let led = dir.path().join("face:cam:red");
        run(dir.path(), blink("cam:red", 500, 500)).unwrap();
        assert_eq!(fs::read_to_string(led.join("trigger")).unwrap(), "timer");
        assert!(!led.join("delay_on").exists());
        run(dir.path(), blink("cam:red", 100, 900)).unwrap();
        assert_eq!(fs::read_to_string(led.join("delay_off")).unwrap(), "900");
    }

    #[test]
    fn the_active_trigger_is_the_bracketed_one() {
        assert_eq!(active_trigger("none [heartbeat] timer"), "heartbeat");
        assert_eq!(active_trigger(""), "?");
    }

    #[test]
    fn a_board_without_a_face_says_so() {
        let dir = tempfile::tempdir().unwrap();
        let err = face_leds(dir.path()).unwrap_err();
        assert_eq!(err.code, exit::FAILED);
        assert!(err.message.contains("no face LEDs"));
    }
}
