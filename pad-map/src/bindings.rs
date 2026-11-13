//! The controller sections of `robotd.toml` — `[pad]`, `[pad_imu_head_control]`, `[pad_drive]`,
//! `[pad_axes]`, `[pad_head]`, `[pad_body]` and `[pad_roller]` — and `[autonomous]`, read into a
//! [`Config`].

use std::path::Path;
use std::time::Duration;

use crate::Config;

/// How often to look for a rewritten config. See `padd`'s loop.
pub const BINDINGS_POLL: Duration = Duration::from_secs(1);

/// The command line's say over the file, for bench use: either flag, when given, wins over its
/// key on every read.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Overrides {
    /// `--deadzone`, over `pad_axes.deadzone`.
    pub deadzone: Option<f64>,
    /// `--max-head`, over every `[pad_head]` limit.
    pub max_head: Option<f64>,
}

/// `--deadzone`, refused outside what `pad_axes.deadzone` accepts.
pub fn parse_deadzone(text: &str) -> Result<f64, String> {
    let value: f64 = text.parse().map_err(|_| "expected a number")?;
    if value.is_finite() && (0.0..1.0).contains(&value) {
        Ok(value)
    } else {
        Err("deadzone must be finite and in [0, 1)".to_owned())
    }
}

/// `--max-head`, refused outside what `[pad_head]` accepts.
pub fn parse_max_head(text: &str) -> Result<f64, String> {
    let value: f64 = text.parse().map_err(|_| "expected a number")?;
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err("max-head must be finite and nonnegative".to_owned())
    }
}

impl Config {
    /// The built-in mapping, with the command line's overrides: what a pad drives with before the
    /// file has been read, and when it never can be.
    pub fn defaults(roller: bool, overrides: &Overrides) -> Self {
        let mut config = Self {
            bindings: Default::default(),
            imu_head: Default::default(),
            drive: Default::default(),
            axes: Default::default(),
            head: Default::default(),
            body: Default::default(),
            roller_limits: Default::default(),
            autonomy: Default::default(),
            roller,
        };
        config.apply(overrides);
        config
    }

    fn apply(&mut self, overrides: &Overrides) {
        if let Some(deadzone) = overrides.deadzone {
            self.axes.deadzone = deadzone;
        }
        if let Some(max_head) = overrides.max_head {
            // The limits only: the file's `expo` and `max_speed` are not what `--max-head` says.
            let (expo, max_speed) = (self.head.expo, self.head.max_speed);
            self.head = robotd_params::PadHeadParams {
                expo,
                max_speed,
                ..robotd_params::PadHeadParams::all(max_head)
            };
        }
    }

    /// Re-read the controller sections. `true` when they were read.
    ///
    /// A file that will not parse is never a reason to leave somebody without a pad, and never a
    /// reason to change what a held stick means: the mapping already in use stays — the built-in
    /// one at startup — and the reason is logged. That matters more here than elsewhere because
    /// this is re-read while running, and a half-saved file caught mid-write is a bad file for a
    /// second; the next read gets the finished one.
    pub fn reload(&mut self, path: &Path, overrides: &Overrides) -> bool {
        let params = match robotd_params::Params::load(path, false) {
            Ok(params) => params,
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    path = %path.display(),
                    "cannot read the controller settings; keeping the mapping in use"
                );
                return false;
            }
        };
        self.bindings = params.pad;
        self.imu_head = params.pad_imu_head_control;
        self.drive = params.pad_drive;
        self.axes = params.pad_axes;
        self.head = params.pad_head;
        self.body = params.pad_body;
        self.roller_limits = params.pad_roller;
        self.autonomy = params.autonomous;
        self.apply(overrides);
        let (b, d) = (&self.bindings, &self.drive);
        tracing::info!(
            a = %b.a, b = %b.b, x = %b.x, y = %b.y, lb = %b.lb, rb = %b.rb,
            pad_imu_head_control = self.imu_head.enabled, pad_imu_head_gain = self.imu_head.gain,
            vx = ?(d.vx_min, d.vx_max), vy = ?(d.vy_min, d.vy_max),
            vyaw = ?(d.vyaw_min, d.vyaw_max),
            deadzone = self.axes.deadzone,
            autonomous = self.autonomy.enabled, pad_idle_s = self.autonomy.pad_idle_s,
            "controller settings"
        );
        // Every binding is a lot of line for the common case of none changed.
        tracing::debug!(axes = ?self.axes, head = ?self.head, body = ?self.body,
            roller = ?self.roller_limits, "stick bindings");
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A good file is taken whole, the command line still wins over it, and a bad one changes
    /// nothing — not even back to the defaults.
    #[test]
    fn a_bad_reload_keeps_the_mapping_and_overrides_win() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        let overrides = Overrides {
            deadzone: Some(0.2),
            max_head: Some(0.8),
        };
        let mut config = Config::defaults(false, &overrides);
        assert_eq!(config.axes.deadzone, 0.2);

        std::fs::write(
            &path,
            "[pad_axes]\ndeadzone = 0.05\n[pad_axes.drive.vx]\nsource = 'right_y'\n",
        )
        .unwrap();
        assert!(config.reload(&path, &overrides));
        assert_eq!(config.axes.deadzone, 0.2, "--deadzone over the file");
        assert_eq!(config.head.head_yaw_max, 0.8, "--max-head over every joint");
        assert_eq!(
            config.axes.drive.vx.source,
            robotd_params::pad_axes::AxisSource::RightY
        );

        let before = config.axes.clone();
        std::fs::write(&path, "[pad_axes]\ndeadzone = 1.0\n").unwrap();
        assert!(!config.reload(&path, &overrides));
        assert_eq!(config.axes, before);

        for bad in ["nan", "-1", "1"] {
            assert!(parse_deadzone(bad).is_err(), "{bad}");
        }
        assert!(parse_max_head("-0.1").is_err());
    }
}
