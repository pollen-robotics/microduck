//! `[head_imu]` in `/etc/robot/robotd.toml`, which is where this daemon's one switch lives.
//!
//! Read out of `robotd`'s file rather than a file of its own, exactly as `mediad` reads
//! `[media]`: the schema, the defaults and the validation are `robotd_params`'s, so the value
//! `robotctl configure` writes is the value this daemon understands, and a key cannot drift
//! between an editor and a reader.
//!
//! Its own module rather than four lines in `main`, for the reason `mediad`'s config is: `main`
//! here is largely Linux-only, so anything living in it is not compiled — let alone tested — on
//! the machine it is written on.

use std::path::{Path, PathBuf};

use robotd_params::Params;

/// The file, when `--config` said nothing.
pub fn default_path() -> PathBuf {
    PathBuf::from(robotd_params::DEFAULT_PATH)
}

/// Read the file, or fall back to the built-in defaults.
///
/// **A file this daemon cannot read is not a reason to stop ranging.** `robotd` refuses to start
/// on a broken params file, which is the loud signal and belongs to the daemon whose control loop
/// the file configures; depth is what somebody looks at while sorting that out. So this warns,
/// names the file, and carries on with the defaults — which for `[head_imu]` means off, the same
/// answer an unprovisioned board gets.
pub fn load(path: &Path, explicit: bool) -> Params {
    match Params::load(path, explicit) {
        Ok(params) => params,
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "unusable params file; carrying on with the built-in defaults"
            );
            Params::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The default is off, and it is the answer on a board with no file at all — which is what
    /// makes "no head IMU samples" the normal state rather than a fault to chase.
    #[test]
    fn the_head_imu_is_off_by_default_and_on_an_unprovisioned_board() {
        assert!(!Params::default().head_imu.enabled);

        let dir = tempfile::tempdir().expect("tempdir");
        let missing = dir.path().join("robotd.toml");
        assert!(!load(&missing, false).head_imu.enabled);
    }

    /// And on it when somebody has said so. The point of reading `robotd`'s file at all: this is
    /// the key `robotctl configure` writes.
    #[test]
    fn the_file_turns_it_on() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "[head_imu]\nenabled = true\n").expect("write");
        assert!(load(&path, true).head_imu.enabled);
    }
}
