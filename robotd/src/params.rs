//! Startup parameters.
//!
//! A file rather than a wall of CLI flags — the prototype grew 142 of them and most were
//! variants, dead skills and dead sensors, all of which are gone. **Read once at startup,
//! not watched**; live reload is deferred (`docs/robotd-design.md` §7.2).
//!
//! It lives outside `releases/<ver>/` so it survives an update *and* a rollback: this is
//! per-robot configuration, not shipped defaults (`architecture.md` §3).

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// Where a release is mounted. Policy paths default under here, so an ordinary update
/// carries the policy with the binaries that were trained against it.
pub const RELEASE_DIR: &str = "/opt/robot/daemon/current";

/// Where a provisioned robot keeps it, alongside the updater's own config.
pub const DEFAULT_PATH: &str = "/etc/robot/robotd.toml";

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Params {
    pub bus: Bus,
    pub control: Control,
    pub update_gate: UpdateGate,
    pub policy: PolicyParams,
    pub safety: SafetyParams,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PolicyParams {
    /// Whether to load a policy at all.
    ///
    /// False means slice 1's behaviour: run the loop, hold the pose, stay healthy. That is a
    /// legitimate configuration — it is the safest thing to be doing while hammering
    /// install/rollback cycles at a bench — and it is distinct from a policy that was wanted
    /// and could not be loaded, which is unhealthy.
    pub enabled: bool,
    /// Walking policy. Defaults inside the release directory so a normal update ships it;
    /// point this elsewhere to try a build without cutting a release, which is the loop
    /// anyone iterating on gait actually runs.
    pub walk: PathBuf,
    /// Standing policy. Without one the walking policy runs at every velocity.
    pub stand: Option<PathBuf>,
    /// Scales raw policy output into a joint offset.
    pub action_scale: f64,
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of `gain`.
    pub standing_gain_ratio: f64,
    /// Position P gain while running.
    pub gain: u16,
    /// First-order low-pass on the head joints. Absent means none, which is what the
    /// prototype ships — there it is behind `--head-low-pass`, off by default.
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SafetyParams {
    /// Projected-gravity z above which the robot counts as going down. Upright is about
    /// -1.0; on its side is near 0.
    pub fall_gravity_z: f64,
    /// How long that has to hold. Debounced so a firm footfall is not a fall.
    pub fall_debounce_ms: u64,
    /// Intent age past which the velocity is zeroed. Stop, not limp.
    pub deadman_ms: u64,
    /// Gain once fallen — low enough to yield rather than fight the floor.
    pub gain_limp: u16,
}

impl Default for PolicyParams {
    fn default() -> Self {
        Self {
            enabled: true,
            walk: PathBuf::from(RELEASE_DIR).join("policies/alpha_walking.onnx"),
            stand: Some(PathBuf::from(RELEASE_DIR).join("policies/alpha_stand.onnx")),
            // The prototype's numbers.
            action_scale: 0.7,
            standing_action_scale: 1.0,
            standing_gain_ratio: 0.6,
            gain: 200,
            head_lowpass: None,
            legs_lowpass: None,
        }
    }
}

impl Default for SafetyParams {
    fn default() -> Self {
        Self {
            fall_gravity_z: -0.5,
            fall_debounce_ms: 200,
            deadman_ms: 500,
            gain_limp: 50,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Bus {
    /// Serial port the servos and the IMU board share. The Radxa Zero 3W wires them to
    /// `/dev/ttyS2`.
    pub port: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Control {
    /// Control loop rate. 50 Hz is inherited from the prototype, where it was chosen on a
    /// Pi Zero 2W — re-derive it on the Radxa rather than trusting it.
    pub hz: u32,
}

/// Thresholds that decide `healthy` — and therefore whether an update is kept.
///
/// **Not** the thresholds for everything `robot.health` reports. That answer also describes the
/// battery, the motor temperatures and the loop counters, and none of those may reach a verdict
/// (`docs/robotd-design.md` §4.5) — so none of them has a setting here. Naming this section
/// `[health]` invited exactly that mistake: it reads like "how the robot is doing", when what it
/// configures is the one question auto-rollback turns on.
///
/// Everything here is a property of the *software*. A future `[thermal]` section for a motor
/// temperature that should throttle the robot would be a different thing, and belongs under a
/// different name.
///
/// The section was called `[health]`. Renamed outright rather than aliased: a board carrying
/// the old name gets a parse error naming the section, which is a better outcome than a robot
/// quietly running on default thresholds nobody chose.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct UpdateGate {
    /// Below this achieved rate the robot reports unhealthy, which is what makes the
    /// updater's auto-rollback mean something. A loop running at 60% of target is alive,
    /// answers every request, and is badly broken.
    pub min_achieved_hz: f64,
    /// How many periods may pass with no tick before the loop counts as **wedged**.
    ///
    /// This detects a dead loop, not a slow one — `min_achieved_hz` owns degradation. Keep
    /// the two apart: set this near the period and it fires on ordinary scheduler jitter,
    /// which on a loaded board would report a perfectly good release unhealthy and roll it
    /// back. A loop that has not ticked in half a second is genuinely gone; one that
    /// ticked 80 ms late is just late.
    pub stall_periods: u32,
    /// Consecutive bus read failures tolerated before reporting unhealthy. One dropped
    /// transaction is ordinary; a run of them means the bus is gone.
    pub max_consecutive_errors: u32,
}

impl Default for Bus {
    fn default() -> Self {
        Self {
            port: "/dev/ttyS2".into(),
        }
    }
}

impl Default for Control {
    fn default() -> Self {
        Self { hz: 50 }
    }
}

impl Default for UpdateGate {
    fn default() -> Self {
        Self {
            // 90% of the default rate. Generous enough not to trip on a slow tick, tight
            // enough that a loop losing every tenth cycle is not called healthy.
            min_achieved_hz: 45.0,
            // 500 ms at the default rate. Deliberately far from the period: three periods
            // is 60 ms, which ordinary scheduler jitter exceeds on a busy machine, and a
            // health check that trips on jitter rolls back good releases.
            stall_periods: 25,
            max_consecutive_errors: 10,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParamsError {
    #[error("reading {path}: {source}")]
    Read {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("parsing {path}: {source}")]
    Parse {
        path: String,
        #[source]
        source: toml::de::Error,
    },
    #[error("{path}: control.hz must be between 1 and 1000, got {got}")]
    Rate { path: String, got: u32 },
}

impl Params {
    /// Load from `path`. A missing file at the *default* location is not an error — an
    /// unprovisioned board should still come up on defaults rather than refuse to start,
    /// and a daemon that will not start is much harder to diagnose remotely than one
    /// running on known defaults. A file explicitly named on the command line must exist.
    pub fn load(path: &Path, explicit: bool) -> Result<Self, ParamsError> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound && !explicit => {
                tracing::warn!(path = %path.display(), "no params file; using defaults");
                return Ok(Self::default());
            }
            Err(source) => {
                return Err(ParamsError::Read {
                    path: path.display().to_string(),
                    source,
                });
            }
        };

        let params: Params = toml::from_str(&text).map_err(|source| ParamsError::Parse {
            path: path.display().to_string(),
            source,
        })?;
        params.validate(path)?;
        Ok(params)
    }

    /// Reject values that would produce a loop that cannot work, at startup rather than as
    /// a division by zero three seconds later.
    fn validate(&self, path: &Path) -> Result<(), ParamsError> {
        if self.control.hz == 0 || self.control.hz > 1000 {
            return Err(ParamsError::Rate {
                path: path.display().to_string(),
                got: self.control.hz,
            });
        }
        Ok(())
    }

    pub fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(1.0 / self.control.hz as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(dir: &std::path::Path, body: &str) -> std::path::PathBuf {
        let path = dir.join("robotd.toml");
        std::fs::write(&path, body).unwrap();
        path
    }

    /// An unprovisioned board must still come up. A daemon that refuses to start because a
    /// config file is absent is far harder to diagnose on a robot than one running on
    /// documented defaults.
    #[test]
    fn a_missing_default_file_falls_back_to_defaults() {
        let dir = tempfile::tempdir().unwrap();
        let p = Params::load(&dir.path().join("absent.toml"), false).unwrap();
        assert_eq!(p.control.hz, 50);
    }

    /// But a file named explicitly on the command line must exist — silently ignoring
    /// `--params /path/typo.toml` would run the robot on settings nobody chose.
    #[test]
    fn an_explicitly_named_missing_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(Params::load(&dir.path().join("absent.toml"), true).is_err());
    }

    /// Partial files are the normal case — a board overrides the port and nothing else.
    #[test]
    fn absent_sections_take_their_defaults() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[bus]\nport = \"/dev/ttyUSB0\"\n");
        let p = Params::load(&path, true).unwrap();
        assert_eq!(p.bus.port, "/dev/ttyUSB0");
        assert_eq!(p.control.hz, 50);
        assert_eq!(p.update_gate.stall_periods, 25);
    }

    /// The shipped example must agree with the built-in defaults, or the file documents a
    /// robot that does not exist — and an operator reading it would draw wrong conclusions
    /// about what their board is actually doing.
    #[test]
    fn the_shipped_example_matches_the_defaults() {
        let shipped = include_str!("../../deploy/robotd.toml");
        let from_file: Params = toml::from_str(shipped).expect("deploy/robotd.toml must parse");
        let built_in = Params::default();

        assert_eq!(from_file.bus.port, built_in.bus.port);
        assert_eq!(from_file.control.hz, built_in.control.hz);
        assert_eq!(
            from_file.update_gate.min_achieved_hz,
            built_in.update_gate.min_achieved_hz
        );
        assert_eq!(
            from_file.update_gate.stall_periods,
            built_in.update_gate.stall_periods
        );
        assert_eq!(
            from_file.update_gate.max_consecutive_errors,
            built_in.update_gate.max_consecutive_errors
        );
    }

    /// A typo in a key must fail loudly. Silently ignoring `min_acheived_hz` would leave
    /// the update gate at a threshold the operator believes they changed.
    #[test]
    fn an_unknown_key_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[update_gate]\nmin_acheived_hz = 10.0\n");
        assert!(Params::load(&path, true).is_err());
    }

    /// The old section name must be *rejected*, not silently ignored.
    ///
    /// A board still carrying `[health]` gets a `robotd` that refuses to start and says why,
    /// which is the honest outcome: `deny_unknown_fields` means the operator hears about the
    /// file rather than running on defaults they did not choose while believing otherwise.
    /// `install.sh` never overwrites `robotd.toml`, so the fix is to edit the section name —
    /// and the parse error names it.
    #[test]
    fn the_old_health_section_name_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[health]\nmin_achieved_hz = 40.0\n");
        assert!(Params::load(&path, true).is_err());
    }

    /// Zero would divide by zero when computing the period; absurdly high would spin.
    #[test]
    fn an_impossible_rate_is_rejected_at_startup() {
        let dir = tempfile::tempdir().unwrap();
        for hz in ["0", "5000"] {
            let path = write(dir.path(), &format!("[control]\nhz = {hz}\n"));
            assert!(Params::load(&path, true).is_err(), "hz = {hz} was accepted");
        }
    }
}
