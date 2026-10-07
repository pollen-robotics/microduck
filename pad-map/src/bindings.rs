//! `[pad]`, `[pad_imu_head_control]` and `[pad_drive]`, read from `robotd.toml`.

use std::path::Path;
use std::time::Duration;

/// How often to look for a rewritten config. See `padd`'s loop.
pub const BINDINGS_POLL: Duration = Duration::from_secs(1);

/// The button bindings, the IMU head switch and the walking speeds, or the defaults.
///
/// A file that will not parse is never a reason to leave somebody without a pad: the defaults
/// are a working robot, and the reason is logged. That matters more here than elsewhere because
/// this is re-read while running — a half-saved file caught mid-write must not take the buttons
/// away, and the next read a second later gets the finished one.
pub fn read_bindings(
    path: &Path,
) -> (
    robotd_params::PadParams,
    robotd_params::PadImuHeadControlParams,
    robotd_params::PadDriveParams,
) {
    match robotd_params::Params::load(path, false) {
        Ok(params) => {
            let pad = params.pad;
            let imu_head = params.pad_imu_head_control;
            let drive = params.pad_drive;
            tracing::info!(
                a = %pad.a, b = %pad.b, x = %pad.x, y = %pad.y, lb = %pad.lb, rb = %pad.rb,
                pad_imu_head_control = imu_head.enabled, pad_imu_head_gain = imu_head.gain,
                vx = ?(drive.vx_min, drive.vx_max), vy = ?(drive.vy_min, drive.vy_max),
                vyaw = ?(drive.vyaw_min, drive.vyaw_max),
                "button bindings"
            );
            (pad, imu_head, drive)
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "cannot read the button bindings; using the default mapping"
            );
            (
                robotd_params::PadParams::default(),
                robotd_params::PadImuHeadControlParams::default(),
                robotd_params::PadDriveParams::default(),
            )
        }
    }
}
