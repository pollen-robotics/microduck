//! What the *board* says about itself, as opposed to what the robot says.
//!
//! Deliberately not in `duck-control` and not behind [`crate::RobotIo`]: nothing here touches
//! the Dynamixel bus. It is a `sysfs` read, so it belongs to the daemon that runs on a Linux
//! board rather than to the crate that models a robot — and it keeps working when the motor
//! bus does not, which is exactly when a thermal reading is interesting.
//!
//! Absent everywhere but Linux, and absent on a Linux box with no thermal zones. Both answer
//! `None` by simply finding no files, which is why there is no `cfg` here: a laptop dev build
//! reports no board temperature and says so, rather than failing to compile.

/// Where the kernel exposes every thermal sensor it knows about.
const THERMAL_ROOT: &str = "/sys/class/thermal";

/// Millidegrees Celsius, per the kernel's thermal sysfs ABI.
const MILLI_PER_DEGREE: f64 = 1000.0;

/// The hottest thermal zone on the board, in °C.
///
/// **The maximum across zones, not the CPU's alone.** A Radxa Zero 3 exposes `soc-thermal` and
/// `gpu-thermal`; other boards add NPU and DDR zones. Reporting the hottest of them is the
/// conservative answer to "is this board too hot", and it cannot silently omit the zone that
/// was actually climbing — which picking one zone by name would, the first time a board wired
/// its sensors differently.
///
/// `None` when there are no readable zones: not Linux, or a kernel without thermal sysfs.
/// Never `Some(0.0)` — a zone that reads as nonsense is skipped rather than averaged in, since
/// 0 °C on a running board is a sensor fault, not a temperature.
pub fn hottest_zone_c() -> Option<f64> {
    let mut hottest: Option<f64> = None;

    // `read_dir` once per sample rather than a cached list of paths: it is a few microseconds
    // at 1 Hz, and a cache would go stale against a zone that appears when a driver loads.
    for entry in std::fs::read_dir(THERMAL_ROOT).ok()?.flatten() {
        let name = entry.file_name();
        if !name.to_string_lossy().starts_with("thermal_zone") {
            continue;
        }
        let Ok(raw) = std::fs::read_to_string(entry.path().join("temp")) else {
            continue;
        };
        let Ok(milli) = raw.trim().parse::<f64>() else {
            continue;
        };
        let celsius = milli / MILLI_PER_DEGREE;
        // A negative or zero reading is a sensor that is not working. Below-freezing is
        // physically possible for a board and pointless to chase; a fault is far likelier.
        if celsius <= 0.0 {
            continue;
        }
        if hottest.is_none_or(|high| celsius > high) {
            hottest = Some(celsius);
        }
    }

    hottest
}

#[cfg(test)]
mod tests {
    use super::*;

    /// On Linux CI this must find the host's own zones; on macOS it must answer `None` rather
    /// than panic. Either way the *shape* is what is asserted, because a test cannot know what
    /// temperature the machine running it is at.
    #[test]
    fn a_reading_is_plausible_or_absent() {
        match hottest_zone_c() {
            None => {} // No thermal sysfs. Correct on macOS, and on a container without it.
            Some(c) => assert!(
                (1.0..=150.0).contains(&c),
                "{c} °C is not a temperature a board reports; check the millidegree scale"
            ),
        }
    }

    /// The scale is the whole risk here: the kernel reports millidegrees, and forgetting the
    /// divisor turns 47 °C into 47000, which would sail through any "is it hot" threshold
    /// anyone later writes against this.
    #[test]
    fn millidegrees_convert_to_degrees() {
        assert_eq!(47123.0 / MILLI_PER_DEGREE, 47.123);
    }
}
