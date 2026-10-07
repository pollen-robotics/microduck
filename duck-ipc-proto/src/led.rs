//! The face board's LEDs: their names, and the one way a daemon switches one.
//!
//! **Which daemon owns which LED, and what each one says, is `architecture.md` §3.2.** This is
//! the mechanism they share: an LED is a directory under `/sys/class/leds` that the beta board's
//! device tree hands to `gpio-leds`, and switching it is a write to `brightness` or `trigger`.
//!
//! Here rather than in each daemon because three of them write LEDs and they must agree on two
//! things that are easy to get subtly wrong: that a blinking LED is switched on by dropping its
//! trigger *first* (a non-zero `brightness` written to a blinking LED changes the brightness it
//! blinks at, and it keeps blinking), and that a board without the LED is a board without it —
//! [`Led::open`] answers `None`, and the caller has nothing to switch.
//!
//! std only: every service links this crate, and a sysfs write needs nothing else.

use std::path::{Path, PathBuf};

/// Where the kernel puts LED class devices.
pub const LEDS_DIR: &str = "/sys/class/leds";

/// The beta board's LEDs, by the label its device tree gives them. Owners: `architecture.md` §3.2.
pub mod name {
    /// The flashlight: three channels of one RGB LED.
    pub const FLASHLIGHT_RED: &str = "face:rgb:red";
    pub const FLASHLIGHT_GREEN: &str = "face:rgb:green";
    pub const FLASHLIGHT_BLUE: &str = "face:rgb:blue";
    /// Lit while the camera's picture is leaving the robot.
    pub const CAMERA: &str = "face:cam:red";
    /// The status LED, green/red.
    pub const STATUS_GREEN: &str = "face:gr2:green";
    pub const STATUS_RED: &str = "face:gr2:red";
    /// The network LED, green/blue.
    pub const NETWORK_GREEN: &str = "face:gb:green";
    pub const NETWORK_BLUE: &str = "face:gb:blue";
    /// The battery gauge, bottom to top: a green/red LED, then two greens. The device tree's
    /// numbering is not the column's — `green2` sits in the middle and `green1` on top, as seen
    /// on a beta board.
    pub const BATTERY_LOW_GREEN: &str = "face:gr1:green";
    pub const BATTERY_LOW_RED: &str = "face:gr1:red";
    pub const BATTERY_MID: &str = "face:green2:green";
    pub const BATTERY_HIGH: &str = "face:green1:green";
}

/// What an LED is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Light {
    Off,
    On,
    /// The kernel's `timer` trigger at its default 500/500 ms. Another rate needs root
    /// (`delay_on` is root's), and one rate is all the LEDs need.
    Blink,
}

impl From<bool> for Light {
    fn from(on: bool) -> Self {
        if on { Light::On } else { Light::Off }
    }
}

/// One LED, written only when what it shows changes.
///
/// Writing on change rather than on every tick is what leaves `robotctl led set` usable as the
/// bench override it is documented as: it holds until the owner next has something new to say.
#[derive(Debug)]
pub struct Led {
    path: PathBuf,
    shown: Option<Light>,
}

impl Led {
    /// The LED `name` under [`LEDS_DIR`], or `None` on a board without it.
    pub fn open(name: &str) -> Option<Led> {
        Self::open_in(Path::new(LEDS_DIR), name)
    }

    /// The LED `name` under `dir` — a fake tree, in tests.
    pub fn open_in(dir: &Path, name: &str) -> Option<Led> {
        let path = dir.join(name);
        path.join("brightness")
            .exists()
            .then_some(Led { path, shown: None })
    }

    /// Show `light`. A no-op when it is already showing it.
    ///
    /// A failure is returned once, for the change that failed, and not retried until the next
    /// change: the likely cause is a sandbox or a permission, neither of which a retry fixes,
    /// and an owner that ticks every second would otherwise log it every second.
    pub fn set(&mut self, light: Light) -> Result<(), String> {
        if self.shown == Some(light) {
            return Ok(());
        }
        self.shown = Some(light);
        match light {
            // Zero drops any trigger as well, so this is also how a blink stops.
            Light::Off => self.write("brightness", "0"),
            Light::On => {
                self.write("trigger", "none")?;
                self.write("brightness", "1")
            }
            Light::Blink => self.write("trigger", "timer"),
        }
    }

    fn write(&self, file: &str, value: &str) -> Result<(), String> {
        let path = self.path.join(file);
        std::fs::write(&path, value)
            .map_err(|e| format!("writing {value:?} to {}: {e}", path.display()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake(names: &[&str]) -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        for name in names {
            let led = dir.path().join(name);
            std::fs::create_dir(&led).unwrap();
            std::fs::write(led.join("brightness"), "0").unwrap();
            std::fs::write(led.join("trigger"), "[none] timer").unwrap();
        }
        dir
    }

    fn read(dir: &Path, name: &str, file: &str) -> String {
        std::fs::read_to_string(dir.join(name).join(file)).unwrap()
    }

    #[test]
    fn a_board_without_the_led_has_nothing_to_open() {
        let dir = fake(&[name::CAMERA]);
        assert!(Led::open_in(dir.path(), name::CAMERA).is_some());
        assert!(Led::open_in(dir.path(), name::FLASHLIGHT_RED).is_none());
    }

    #[test]
    fn on_drops_the_trigger_before_lighting_and_off_clears_both() {
        let dir = fake(&[name::CAMERA]);
        let mut led = Led::open_in(dir.path(), name::CAMERA).unwrap();
        led.set(Light::Blink).unwrap();
        assert_eq!(read(dir.path(), name::CAMERA, "trigger"), "timer");
        led.set(Light::On).unwrap();
        assert_eq!(read(dir.path(), name::CAMERA, "trigger"), "none");
        assert_eq!(read(dir.path(), name::CAMERA, "brightness"), "1");
        led.set(Light::Off).unwrap();
        assert_eq!(read(dir.path(), name::CAMERA, "brightness"), "0");
    }

    #[test]
    fn an_unchanged_light_is_not_rewritten() {
        let dir = fake(&[name::CAMERA]);
        let mut led = Led::open_in(dir.path(), name::CAMERA).unwrap();
        led.set(Light::On).unwrap();
        // Someone at the bench switches it off; the owner saying "on" again must not undo that.
        std::fs::write(dir.path().join(name::CAMERA).join("brightness"), "0").unwrap();
        led.set(Light::On).unwrap();
        assert_eq!(read(dir.path(), name::CAMERA, "brightness"), "0");
    }

    #[test]
    fn a_failed_write_is_reported_once_per_change() {
        let dir = fake(&[name::CAMERA]);
        let mut led = Led::open_in(dir.path(), name::CAMERA).unwrap();
        std::fs::remove_dir_all(dir.path().join(name::CAMERA)).unwrap();
        assert!(led.set(Light::On).is_err());
        assert!(led.set(Light::On).is_ok());
        assert!(led.set(Light::Off).is_err());
    }
}
