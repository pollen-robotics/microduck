//! Which camera sensor this robot has, and the handful of facts about it that `mediad` cannot ask
//! the driver for.
//!
//! **The board decides which sensor is allowed; the media graph says which one is there.** A
//! camera and its board go together — the Zero 3W robots carry an IMX219, the beta board a GC2093
//! (`robotd_params::board::Board::camera_sensor`) — so the head camera must be the declared
//! board's, and a media graph holding another sensor is refused: that is a robot assembled with
//! the wrong module, or one declaring the wrong board, and both are worth stopping on rather than
//! streaming through. `[media] sensor` forces a named sensor instead, for a board fitted with
//! another camera on purpose. That is the exception, and it is logged as one.
//!
//! The sensor names itself in the topology (`m00_b_imx219 2-0010`, `m00_b_gc2093 2-0037`), which
//! is how the check is made — and why a sensor this daemon has no profile for is refused by name
//! rather than driven with another sensor's numbers. Exposure in the wrong units is a picture that
//! is black or white, and a field of view borrowed from another lens is a geometry that is quietly
//! wrong — and a consumer has no way to tell.
//!
//! Everything else about a sensor is either read from the driver at run time or is the same for
//! every sensor here — the pinned mode is 1920×1080 raw 10-bit on both. What is in a [`Sensor`] is
//! what is left: the units its controls are in, the exposure the auto-exposure loop may spend, and
//! the optics, when anyone has measured them.

use robotd_params::CameraSensor;
use robotd_params::board::Board;

use crate::camera::SensorMode;

/// One camera sensor `mediad` knows how to drive.
#[derive(Debug)]
pub struct Sensor {
    pub id: CameraSensor,
    /// The readout mode `pipeline` pins, as a media bus format and size.
    pub mode: SensorMode,
    pub bus_format: &'static str,
    pub exposure: Exposure,
    /// Who steers the exposure while the camera streams.
    pub metering: Metering,
    /// Horizontal field of view across a frame in [`Sensor::mode`], degrees — what the nominal
    /// geometry rests on. `None` when nobody has measured it, and then nominal intrinsics are not
    /// published at all.
    pub hfov_deg: Option<f64>,
    /// A solve of this sensor behind its lens, shared by every robot built with that part. `None`
    /// for a part nobody has calibrated.
    pub family: Option<fn() -> robotd_params::CameraIntrinsics>,
}

impl Sensor {
    /// What the sensor's entity name contains in the media graph, and what the logs call it.
    pub fn name(&self) -> &'static str {
        self.id.label()
    }
}

/// Who steers this sensor's exposure. One writer, never two.
///
/// **Two loops on one sensor do not average, they fight.** Measured on a beta (2026-10-20) with
/// both running: rkaiq held the picture at mediad's own setpoint, so mediad, seeing every darker
/// step it wrote overwritten a frame later, stepped its belief down to its floor — 4 lines at 1x —
/// and then re-asserted that floor on its ten-second heartbeat. Each re-assert was a black frame,
/// rkaiq's recovery overshot to 1688 lines at 16x, and under strong lights that is a picture that
/// flashes to white and hunts back every ten seconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Metering {
    /// `mediad::exposure`, because the board's 3A engine converges once at stream start and then
    /// stops. `scripts/setup-rkaiq.sh` turns that engine's AE off to match.
    Mediad,
    /// The 3A engine, whose AE runs every frame on this board: Seeed's tuning file ships it on
    /// (`ae_calib.CommCtrl.Enable = 1`, `AecRunInterval = 0`), and it tracks the scene.
    Rkaiq,
}

/// What the auto-exposure loop may spend, in this sensor's own units.
///
/// The two shutter caps are kept the same in **time** across sensors — 11.4 ms soft, 22.9 ms hard —
/// because that is what they are about: a walking robot's picture is not smeared at the first, and
/// the frame time is not stretched at the second. Only the line count that buys that time differs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Exposure {
    /// Shutter spent before any gain, in lines.
    pub soft_lines: f64,
    /// The longest shutter asked for, in lines. Below the frame length on every sensor here, since
    /// a driver answers a longer exposure by stretching the frame rather than clamping.
    pub hard_lines: f64,
    /// The analogue gain register value that means 1x.
    pub unity_gain: u32,
    /// Analogue gain ceiling, in multiples of 1x.
    pub max_analogue: f64,
    /// Where the sensor starts, before the loop has metered anything: a boot value leaves the
    /// picture black rather than merely dark.
    pub start_lines: u32,
    pub start_analogue: f64,
}

impl Exposure {
    /// The starting analogue gain, as the register value.
    pub fn start_gain(&self) -> u32 {
        (self.start_analogue * f64::from(self.unity_gain)) as u32
    }
}

/// The alpha robots' head camera: the IMX219 behind a ~3.05 mm M12 lens. The numbers are the
/// prototype's, which ran for months; `crate::camera` has the optics.
pub const IMX219: Sensor = Sensor {
    id: CameraSensor::Imx219,
    mode: SensorMode {
        width: 1920,
        height: 1080,
    },
    bus_format: "SRGGB10_1X10",
    exposure: Exposure {
        // One line is ~19.05 µs in the pinned mode, which is 1766 lines long.
        soft_lines: 600.0,
        hard_lines: 1200.0,
        unity_gain: 256,
        max_analogue: 11.0,
        start_lines: 600,
        start_analogue: 4.0,
    },
    metering: Metering::Mediad,
    hfov_deg: Some(62.0),
    family: Some(robotd_params::CameraIntrinsics::alpha),
};

/// The beta board's head camera: GalaxyCore's GC2093 on Seeed's main board.
///
/// The mode is the driver's only one: 1920×1080 raw 10-bit at 30 fps, 1125 lines a frame (1080 plus
/// a vertical blanking of 45), so a line is ~29.6 µs and the exposure control tops out at 1121.
/// Analogue gain is 64 for 1x — read off the driver, `min=64 max=8192`. The ceiling is kept at the
/// IMX219's 11x rather than the driver's, so the loop spends noise the same way on both.
///
/// The geometry is the family solve, [`robotd_params::CameraIntrinsics::beta`] (unit *sky-gtc*): ~85°
/// across the 16:9 frame.
pub const GC2093: Sensor = Sensor {
    id: CameraSensor::Gc2093,
    mode: SensorMode {
        width: 1920,
        height: 1080,
    },
    bus_format: "SRGGB10_1X10",
    exposure: Exposure {
        soft_lines: 385.0,
        hard_lines: 773.0,
        unity_gain: 64,
        max_analogue: 11.0,
        start_lines: 385,
        start_analogue: 4.0,
    },
    // The beta image installs rkaiq with Seeed's IQ file, whose AE keeps converging — the one
    // thing the alpha's engine does not do.
    metering: Metering::Rkaiq,
    // 2·atan(640 / fx) on the 1280-wide solve.
    hfov_deg: Some(85.2),
    family: Some(robotd_params::CameraIntrinsics::beta),
};

// The GC2093's exposure control stops at 1121 lines. A hard cap above it would be refused by the
// driver, or answered with a longer frame — so it does not build.
const _: () = assert!(GC2093.exposure.hard_lines < 1121.0);

/// The profile of a sensor `robotd-params` names. Exhaustive, so a sensor added there does not
/// build until it has one here.
pub fn profile(id: CameraSensor) -> &'static Sensor {
    match id {
        CameraSensor::Imx219 => &IMX219,
        CameraSensor::Gc2093 => &GC2093,
    }
}

/// Every sensor this daemon can drive.
pub const SENSORS: &[&Sensor] = &[&IMX219, &GC2093];

/// The sensor an entity name belongs to, if it is one of ours.
pub fn identify(entity: &str) -> Option<&'static Sensor> {
    SENSORS
        .iter()
        .copied()
        .find(|sensor| entity.contains(sensor.name()))
}

/// The sensor the head camera must be, and why — the why is what the refusal has to say.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Expected {
    pub sensor: CameraSensor,
    pub why: Why,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Why {
    /// The board's camera. `declared` is false when `[board] version` is unset and the board is
    /// the default, which is the likeliest cause of a mismatch on a beta.
    Board { board: Board, declared: bool },
    /// `[media] sensor` names it.
    Forced,
}

impl Expected {
    /// From `[media] sensor`, the board in effect, and whether the file declared it.
    pub fn new(choice: robotd_params::MediaSensor, board: Board, declared: bool) -> Self {
        match choice.forced() {
            Some(sensor) => Self {
                sensor,
                why: Why::Forced,
            },
            None => Self {
                sensor: board.camera_sensor(),
                why: Why::Board { board, declared },
            },
        }
    }

    /// Choose the expected sensor among the ones a media graph holds, or say why none will do.
    ///
    /// `found` is every sensor with a profile, as entity name and profile; `others` the sensors
    /// without one. Pure, so every refusal is a test rather than a board.
    pub fn pick(
        &self,
        found: &[(String, &'static Sensor)],
        others: &[String],
    ) -> Result<(String, &'static Sensor), String> {
        if let Some((entity, sensor)) = found.iter().find(|(_, s)| s.id == self.sensor) {
            return Ok((entity.clone(), sensor));
        }
        let want = self.sensor.label();
        let there: Vec<&str> = found
            .iter()
            .map(|(entity, _)| entity.as_str())
            .chain(others.iter().map(String::as_str))
            .collect();
        if there.is_empty() {
            return Err(format!("no sensor in the media graph; expected {want}"));
        }
        let there = there.join(", ");
        Err(match self.why {
            Why::Forced => format!(
                "the media graph has {there}, and `[media] sensor` forces {want}. Set it to the \
                 sensor that is fitted, or back to \"board\"."
            ),
            Why::Board {
                board,
                declared: true,
            } => format!(
                "the media graph has {there}, and this robot is declared a {board} board, which \
                 is built with {want}. A camera and its board go together, so either the board \
                 is declared wrong (`[board] version`, `robotctl configure`) or the camera module \
                 is. To drive this camera on this board anyway, set `[media] sensor` to it."
            ),
            Why::Board {
                board,
                declared: false,
            } => format!(
                "the media graph has {there}, and this robot declares no board, which reads as \
                 {board} — built with {want}. If it is another board, declare it in \
                 `[board] version` (`robotctl health` offers to); to drive this camera on a \
                 {board} anyway, set `[media] sensor` to it."
            ),
        })
    }
}

/// What one media graph holds, read off `media-ctl -p`.
#[derive(Debug, Default)]
pub struct Topology {
    /// Every sensor this daemon has a profile for, as its entity name and profile.
    pub ours: Vec<(String, &'static Sensor)>,
    /// Entities the driver calls a sensor and that are not in [`SENSORS`] — named in the error, so
    /// a board with a camera nobody wrote a profile for says which camera that is.
    pub others: Vec<String>,
}

impl Topology {
    /// Parse `media-ctl -p` output. An entity is a header line followed by its type:
    ///
    /// ```text
    /// - entity 76: m00_b_gc2093 2-0037 (1 pad, 1 link)
    ///              type V4L2 subdev subtype Sensor flags 0
    /// ```
    pub fn read(printed: &str) -> Self {
        let mut topology = Self::default();
        let mut entity: Option<&str> = None;
        for line in printed.lines() {
            let line = line.trim_start();
            if let Some(header) = line.strip_prefix("- entity") {
                entity = header
                    .split_once(": ")
                    .map(|(_, rest)| rest.split(" (").next().unwrap_or(rest).trim())
                    .filter(|name| !name.is_empty());
                if let Some(name) = entity
                    && let Some(sensor) = identify(name)
                {
                    topology.ours.push((name.to_string(), sensor));
                }
                continue;
            }
            if line.starts_with("type ")
                && line.contains("subtype Sensor")
                && let Some(name) = entity.take()
                && identify(name).is_none()
            {
                topology.others.push(name.to_string());
            }
        }
        topology
    }
}

/// The names in [`SENSORS`], for a message that says what would have been accepted.
pub fn known() -> String {
    SENSORS
        .iter()
        .map(|sensor| sensor.name())
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotd_params::MediaSensor;

    #[test]
    fn a_sensor_is_found_by_its_entity_name() {
        assert_eq!(identify("m00_b_imx219 2-0010").unwrap().name(), "imx219");
        assert_eq!(identify("m00_b_gc2093 2-0037").unwrap().name(), "gc2093");
        assert!(
            identify("m00_b_gc2613 2-0031").is_none(),
            "not one we drive"
        );
        assert!(identify("rkisp-isp-subdev").is_none());
    }

    #[test]
    fn every_named_sensor_has_its_own_profile() {
        for sensor in SENSORS {
            assert_eq!(profile(sensor.id).id, sensor.id);
        }
    }

    /// What `media-ctl -p` printed on a beta board, cut to the entities that matter.
    const BETA: &str = "\
- entity 73: rockchip-csi2-dphy0 (2 pads, 2 links)
             type V4L2 subdev subtype Unknown flags 0
- entity 76: m00_b_gc2093 2-0037 (1 pad, 1 link)
             type V4L2 subdev subtype Sensor flags 0
             device node name /dev/v4l-subdev3
";

    #[test]
    fn the_beta_boards_camera_is_found_in_its_topology() {
        let topology = Topology::read(BETA);
        assert_eq!(topology.ours.len(), 1);
        let (entity, sensor) = &topology.ours[0];
        assert_eq!(entity, "m00_b_gc2093 2-0037");
        assert_eq!(sensor.name(), "gc2093");
        assert!(topology.others.is_empty());
    }

    /// A camera with no profile is named, rather than reported as "no camera".
    #[test]
    fn a_sensor_with_no_profile_is_named() {
        let topology = Topology::read(
            "- entity 80: m00_b_ov5647 2-0036 (1 pad, 1 link)\n\
             \x20            type V4L2 subdev subtype Sensor flags 0\n",
        );
        assert!(topology.ours.is_empty());
        assert_eq!(topology.others, vec!["m00_b_ov5647 2-0036"]);
    }

    /// A declared beta drives its GC2093.
    #[test]
    fn the_boards_own_camera_is_accepted() {
        let topology = Topology::read(BETA);
        let beta = Expected::new(MediaSensor::Board, Board::Beta, true);
        let (entity, sensor) = beta.pick(&topology.ours, &topology.others).unwrap();
        assert_eq!(entity, "m00_b_gc2093 2-0037");
        assert_eq!(sensor.id, CameraSensor::Gc2093);
    }

    /// **The mismatch this exists for.** The camera and the board go together, so the beta's
    /// GC2093 on a board declared zero3 is refused — and the refusal says which half to fix.
    #[test]
    fn another_boards_camera_is_refused_and_says_why() {
        let topology = Topology::read(BETA);

        let declared = Expected::new(MediaSensor::Board, Board::Zero3, true);
        let why = declared.pick(&topology.ours, &topology.others).unwrap_err();
        assert!(why.contains("m00_b_gc2093 2-0037"), "{why}");
        assert!(why.contains("declared a zero3"), "{why}");
        assert!(
            why.contains("[media] sensor"),
            "the way out is named: {why}"
        );

        // Undeclared is the likelier cause on a beta, and the message says to declare it.
        let undeclared = Expected::new(MediaSensor::Board, Board::Zero3, false);
        let why = undeclared
            .pick(&topology.ours, &topology.others)
            .unwrap_err();
        assert!(why.contains("declares no board"), "{why}");
        assert!(why.contains("[board] version"), "{why}");
    }

    /// The exception: forced, the named sensor is driven whatever the board.
    #[test]
    fn a_forced_sensor_overrides_the_board() {
        let topology = Topology::read(BETA);
        let forced = Expected::new(MediaSensor::Gc2093, Board::Zero3, true);
        assert_eq!(forced.why, Why::Forced);
        let (_, sensor) = forced.pick(&topology.ours, &topology.others).unwrap();
        assert_eq!(sensor.id, CameraSensor::Gc2093);

        // And forcing a sensor that is not fitted is refused like any mismatch.
        let wrong = Expected::new(MediaSensor::Imx219, Board::Beta, true);
        let why = wrong.pick(&topology.ours, &topology.others).unwrap_err();
        assert!(why.contains("forces imx219"), "{why}");
    }

    /// The caps are one decision in milliseconds, written out per sensor in lines. A sensor whose
    /// line count drifts from the time it is meant to buy is a sensor that smears or stretches its
    /// frame where the other does not.
    #[test]
    fn the_shutter_caps_buy_the_same_time_on_every_sensor() {
        let line_us = |sensor: &Sensor| match sensor.id {
            CameraSensor::Imx219 => 19.05,
            CameraSensor::Gc2093 => 1e6 / 30.0 / 1125.0,
        };
        for sensor in SENSORS {
            let soft_ms = sensor.exposure.soft_lines * line_us(sensor) / 1000.0;
            let hard_ms = sensor.exposure.hard_lines * line_us(sensor) / 1000.0;
            assert!((soft_ms - 11.4).abs() < 0.1, "{}: {soft_ms}", sensor.name());
            assert!((hard_ms - 22.9).abs() < 0.1, "{}: {hard_ms}", sensor.name());
        }
    }

    #[test]
    fn the_starting_gain_is_in_the_sensors_own_units() {
        assert_eq!(IMX219.exposure.start_gain(), 1024);
        assert_eq!(GC2093.exposure.start_gain(), 256);
    }
}
