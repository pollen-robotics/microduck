//! What the camera's geometry is, so a consumer can do more than look at the picture.
//!
//! A frame is pixels. Turning pixels into directions — which is what SLAM, visual odometry, or
//! "how far away is that duck" all need — takes the **intrinsics**: the focal length in pixels and
//! where the optical axis crosses the image. Without them a monocular reconstruction is
//! scale-free and its angles are wrong; with them it maps a room.
//!
//! # Derived from two physical numbers, so they can be checked rather than trusted
//!
//! The sensor is an IMX219 with a 1.12 µm pixel pitch behind a 3.04 mm lens (Pi Camera v2), and
//! those two give the focal length in pixels at the sensor's native scale:
//!
//! ```text
//! 3.04 mm / 1.12 µm = 2714 px
//! ```
//!
//! That number is a property of the optics and the sensor, not of any mode: it holds for every
//! frame the sensor reads out at native pixel pitch. What changes per mode is **how much of the
//! sensor is read** and **what the ISP scales it to**, and both move the intrinsics:
//!
//! - `pipeline::pin_sensor_mode` puts the sensor in **1920×1080**, which on this sensor is a
//!   *crop* rather than a binning — so pixels stay native-pitch, `fx` stays 2714, and the centre
//!   is at (960, 540).
//! - The ISP then scales that to whatever `[media] quality` asks for. A uniform scale multiplies
//!   `fx`, `fy`, `cx` and `cy` by the same factor: 1280/1920 gives `fx` ≈ 1810 and a centre at
//!   (640, 360).
//!
//! # And when the mode is not the one we asked for, this publishes nothing
//!
//! `pin_sensor_mode` shells out to `media-ctl` and can fail — a board without `v4l-utils`, an
//! entity name that moved. Capture still works: the sensor stays in its 3280×2464 boot mode and
//! the ISP scales from there. But that is the **full sensor** rather than a crop of it, so the
//! field of view is wider and every intrinsic is wrong by about 1.7× — and 4:3 scaled into 16:9
//! means the ISP is also cropping or squashing vertically, which changes `fy` independently.
//!
//! So intrinsics are published for a mode this code put the sensor in and not otherwise.
//! **Numbers that are quietly wrong are worse than none**: a consumer told nothing knows it must
//! calibrate, and a consumer told 2714 when the truth is 1590 builds a confident, wrong map.
//!
//! # Nominal is not calibrated, and the wire says which
//!
//! Everything above is the *design* of the camera, not a measurement of the one on this robot:
//! lens focal lengths vary by a few percent unit to unit, the principal point is never exactly the
//! centre, and nothing here models distortion at all. That is enough for a room-scale map and not
//! enough for photogrammetry, so the published record carries `calibrated: false` until somebody
//! measures a particular robot and puts the result in `[media.intrinsics]`. A consumer that needs
//! better can then tell that it needs to ask.

/// The pixel pitch of the IMX219, from its datasheet.
const PIXEL_PITCH_UM: f64 = 1.12;

/// The focal length of the lens the module ships with (Pi Camera v2).
const FOCAL_LENGTH_MM: f64 = 3.04;

/// Focal length in pixels at the sensor's native pixel pitch — see the module header.
fn native_focal_px() -> f64 {
    FOCAL_LENGTH_MM * 1000.0 / PIXEL_PITCH_UM
}

/// A sensor readout mode: how many pixels come off the sensor, and how they were read.
///
/// # The mode decides the field of view, and the numbers are not intuitive
///
/// The IMX219's array is 3280×2464 at a 1.12 µm pitch behind a 3.04 mm lens, which is
/// `2·atan(3280·1.12µm / 2 / 3.04mm)` = **62°** horizontally when the whole array is read. Every
/// other mode reads less of it, and reading less of an array behind a fixed lens is *zoom*:
///
/// | mode | how | horizontal FOV | note |
/// |---|---|---|---|
/// | 3280×2464 | full, native pitch | **62°** | the boot mode, capped at 21 fps |
/// | 1640×1232 | full, 2×2 binned | **62°** | same view, half the pixels, less noise |
/// | 1920×1080 | crop, native pitch | **39°** | what this daemon pins, for 30 fps |
/// | 1280×720 | crop, native pitch | **27°** | a *narrower* view, not a cheaper 720p |
///
/// So "read 720p off the sensor instead of scaling 1080p down" costs a third of the remaining
/// field of view and gains nothing but bandwidth — and the 1080p→720p scale it replaces is
/// averaging 2.25 pixels into one, which is free noise reduction the crop also gives up. For
/// anything that looks at the picture rather than at a face in the middle of it — SLAM most of
/// all — the mode to want is **1640×1232**: the whole 62°, binned, at 30 fps.
///
/// That is not a change this module makes; it is `pipeline::pin_sensor_mode`'s to make and a
/// `[media] quality` to expose. It is written down here because this is the file where the
/// consequences are arithmetic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorMode {
    pub width: u32,
    pub height: u32,
    /// How many sensor pixels were averaged per output pixel along one axis: 1 for a native
    /// readout, 2 for a 2×2 binning. It multiplies the effective pixel pitch and therefore
    /// divides the focal length in pixels.
    pub binning: u32,
}

impl SensorMode {
    /// The mode `pipeline::pin_sensor_mode` asks for, and the only one whose geometry this knows.
    pub const PINNED: Self = Self {
        width: 1920,
        height: 1080,
        binning: 1,
    };

    /// The full field of view, binned — the mode a perception consumer wants. Not pinned by
    /// anything yet; here so the arithmetic above is checkable rather than a claim.
    pub const FULL_BINNED: Self = Self {
        width: 1640,
        height: 1232,
        binning: 2,
    };
}

/// Where the optical axis is and how long the focal length is, in pixels of a delivered frame.
///
/// **For the frame as it is sent**, which is not rotated: the camera is mounted a quarter turn off
/// and nothing on the robot turns the pixels back (`pipeline`'s header says why). A consumer that
/// rotates the image has to rotate these too — `cx` and `cy` swap, and so do `fx` and `fy` — and
/// the `rotate` field alongside these in `media.video` is what tells it by how much.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// **Whether these were measured on this robot.** `false` means they are the module's design
    /// figures: good to a few percent, with no distortion model, and enough for a room-scale map
    /// rather than for metrology.
    pub calibrated: bool,
    /// Radial and tangential terms in OpenCV's order — `k1 k2 p1 p2 k3` — or empty for "no model
    /// of the distortion", which is what a nominal record has.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub distortion: Vec<f64>,
}

impl Intrinsics {
    /// The design figures for a delivered frame, or `None` when the geometry is not known.
    ///
    /// `None` has one cause and it is worth surfacing rather than papering over: the sensor is not
    /// in the mode this code pins, so how much of the sensor a frame covers is unknown.
    pub fn nominal(mode: Option<SensorMode>, width: u32, height: u32) -> Option<Self> {
        let mode = mode?;
        if width == 0 || height == 0 || mode.width == 0 || mode.height == 0 {
            return None;
        }

        // A uniform scale is the only one this can reason about. The ISP *can* scale the axes
        // independently, and then the sensor's aspect ratio has been changed on the way out —
        // which means part of the frame was cropped or squashed, and neither is derivable from
        // the output size alone.
        let scale_x = f64::from(width) / f64::from(mode.width);
        let scale_y = f64::from(height) / f64::from(mode.height);
        if (scale_x - scale_y).abs() > 0.01 {
            return None;
        }

        // Binning makes each output pixel cover `binning` sensor pixels, so the focal length in
        // pixels shrinks by the same factor before the ISP's scale is applied.
        let focal = native_focal_px() / f64::from(mode.binning.max(1)) * scale_x;
        Some(Self {
            fx: focal,
            fy: focal,
            // The principal point is *assumed* central, which is what makes this nominal: on a
            // real module it is a few pixels off, in a direction only a calibration knows.
            cx: f64::from(width) / 2.0,
            cy: f64::from(height) / 2.0,
            calibrated: false,
            distortion: Vec::new(),
        })
    }

    /// A calibration, scaled from the resolution it was measured at to the one being delivered.
    ///
    /// Scaling a calibration is exact for a uniform resize — every intrinsic is in pixels and
    /// pixels all change size together — and wrong for a crop, which is why the record carries the
    /// resolution it was taken at rather than assuming one. An aspect change between the two means
    /// the second image is not the first one resized, so this refuses it.
    pub fn scaled_from(
        measured: &robotd_params::CameraIntrinsics,
        width: u32,
        height: u32,
    ) -> Option<Self> {
        if measured.width == 0 || measured.height == 0 || width == 0 || height == 0 {
            return None;
        }
        let scale_x = f64::from(width) / f64::from(measured.width);
        let scale_y = f64::from(height) / f64::from(measured.height);
        if (scale_x - scale_y).abs() > 0.01 {
            tracing::warn!(
                measured = format!("{}x{}", measured.width, measured.height),
                delivered = format!("{width}x{height}"),
                "the calibration was measured at a different aspect ratio than the stream, so it \
                 cannot be scaled to it; publishing nominal intrinsics instead"
            );
            return None;
        }
        Some(Self {
            fx: measured.fx * scale_x,
            fy: measured.fy * scale_y,
            cx: measured.cx * scale_x,
            cy: measured.cy * scale_y,
            calibrated: true,
            // Distortion coefficients are dimensionless in normalised image coordinates, so a
            // uniform resize leaves them alone.
            distortion: measured.distortion.clone(),
        })
    }

    /// What to publish: a calibration if this robot has one, the design figures otherwise.
    pub fn published(
        configured: Option<&robotd_params::CameraIntrinsics>,
        mode: Option<SensorMode>,
        width: u32,
        height: u32,
    ) -> Option<Self> {
        configured
            .and_then(|measured| Self::scaled_from(measured, width, height))
            .or_else(|| Self::nominal(mode, width, height))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn measured(width: u32, height: u32) -> robotd_params::CameraIntrinsics {
        robotd_params::CameraIntrinsics {
            width,
            height,
            fx: 1800.0,
            fy: 1802.0,
            cx: 646.0,
            cy: 358.0,
            distortion: vec![-0.31, 0.12, 0.0, 0.0, -0.02],
        }
    }

    /// The two physical numbers, and the one they produce.
    #[test]
    fn the_focal_length_in_pixels_comes_from_the_datasheet() {
        // 3.04 mm over a 1.12 µm pitch. Written out so a wrong constant is a failing test rather
        // than a plausible-looking number in an SDP nobody checks.
        assert!(
            (native_focal_px() - 2714.29).abs() < 0.01,
            "{}",
            native_focal_px()
        );
    }

    /// 1280x720 out of the pinned 1920x1080 mode: a two-thirds scale, uniformly.
    #[test]
    fn the_pinned_mode_scales_to_what_is_streamed() {
        let at_720p = Intrinsics::nominal(Some(SensorMode::PINNED), 1280, 720).expect("known");
        assert!((at_720p.fx - 1809.52).abs() < 0.01, "{}", at_720p.fx);
        assert_eq!(at_720p.fy, at_720p.fx, "square pixels, uniform scale");
        assert_eq!((at_720p.cx, at_720p.cy), (640.0, 360.0));
        assert!(
            !at_720p.calibrated,
            "these are the module's, not this robot's"
        );
        assert!(
            at_720p.distortion.is_empty(),
            "nominal models no distortion"
        );

        // And at the mode's own resolution, the native figure survives untouched.
        let at_1080p = Intrinsics::nominal(Some(SensorMode::PINNED), 1920, 1080).expect("known");
        assert!((at_1080p.fx - native_focal_px()).abs() < 0.01);
    }

    /// Binning divides the focal length in pixels, and the full-FOV mode is the case that matters.
    ///
    /// 1640×1232 read 2×2-binned sees the same 62° as the full array, and its `fx` is half the
    /// native figure — which is the arithmetic behind the mode table: a wider view is a shorter
    /// focal length in pixels, for the same lens.
    #[test]
    fn a_binned_mode_has_a_shorter_focal_length_in_pixels() {
        let binned = Intrinsics::nominal(Some(SensorMode::FULL_BINNED), 1640, 1232).expect("known");
        assert!(
            (binned.fx - native_focal_px() / 2.0).abs() < 0.01,
            "{}",
            binned.fx
        );

        // And the field of view it implies is the whole sensor's 62°, which is the check that the
        // two numbers in the table agree with each other.
        let hfov = 2.0 * (f64::from(1640_u32) / 2.0 / binned.fx).atan().to_degrees();
        assert!((hfov - 62.2).abs() < 0.5, "{hfov}");

        // Against which the pinned 1080p crop is a 39° view: same lens, less sensor.
        let pinned = Intrinsics::nominal(Some(SensorMode::PINNED), 1920, 1080).expect("known");
        let pinned_hfov = 2.0 * (f64::from(1920_u32) / 2.0 / pinned.fx).atan().to_degrees();
        assert!((pinned_hfov - 38.9).abs() < 0.5, "{pinned_hfov}");
    }

    /// **A sensor in an unknown mode publishes nothing.**
    ///
    /// `pin_sensor_mode` can fail, and then the sensor is in its 3280x2464 boot mode: the full
    /// frame rather than a crop of it, about 1.7× wider, and 4:3 scaled into 16:9 on top. Numbers
    /// that are quietly wrong are worse than none — a consumer told nothing calibrates.
    #[test]
    fn an_unknown_sensor_mode_yields_no_intrinsics() {
        assert!(Intrinsics::nominal(None, 1280, 720).is_none());
        assert!(Intrinsics::published(None, None, 1280, 720).is_none());
    }

    /// A frame whose aspect ratio is not the mode's has been cropped or squashed on the way out,
    /// and which of those cannot be told from the size.
    #[test]
    fn a_changed_aspect_ratio_is_refused_rather_than_guessed() {
        assert!(
            Intrinsics::nominal(Some(SensorMode::PINNED), 640, 480).is_none(),
            "4:3 out of a 16:9 mode is not a resize"
        );
        assert!(Intrinsics::nominal(Some(SensorMode::PINNED), 1280, 0).is_none());
    }

    /// A calibration wins, and scales.
    #[test]
    fn a_calibration_is_preferred_and_carried_to_the_streamed_size() {
        // Measured at 1280x720, delivered at 640x360: everything halves.
        let published = Intrinsics::published(
            Some(&measured(1280, 720)),
            Some(SensorMode::PINNED),
            640,
            360,
        )
        .expect("a calibration");
        assert!(published.calibrated);
        assert!((published.fx - 900.0).abs() < 0.01);
        assert!((published.cx - 323.0).abs() < 0.01);
        assert_eq!(
            published.distortion,
            vec![-0.31, 0.12, 0.0, 0.0, -0.02],
            "distortion is dimensionless, so a resize leaves it alone"
        );

        // And at the resolution it was measured at, it is published as measured.
        let same = Intrinsics::published(Some(&measured(1280, 720)), None, 1280, 720)
            .expect("as measured");
        assert_eq!(
            (same.fx, same.fy, same.cx, same.cy),
            (1800.0, 1802.0, 646.0, 358.0)
        );
    }

    /// A calibration that cannot be scaled falls back to nominal rather than to nothing — and
    /// says so, because a robot that was calibrated and is publishing design figures is a
    /// mismatch somebody should fix.
    #[test]
    fn a_calibration_at_the_wrong_aspect_falls_back_to_nominal() {
        let published = Intrinsics::published(
            Some(&measured(640, 480)),
            Some(SensorMode::PINNED),
            1280,
            720,
        )
        .expect("nominal");
        assert!(
            !published.calibrated,
            "the fallback must not claim to be a measurement"
        );
        assert!((published.fx - 1809.52).abs() < 0.01);
    }

    /// The shape a consumer reads. `calibrated` is not optional in the JSON: a consumer that has
    /// to guess whether numbers were measured will guess that they were.
    #[test]
    fn the_wire_shape_names_what_it_is() {
        let json =
            serde_json::to_value(Intrinsics::nominal(Some(SensorMode::PINNED), 1280, 720).unwrap())
                .unwrap();
        assert_eq!(json["calibrated"], false);
        assert!((json["fx"].as_f64().unwrap() - 1809.52).abs() < 0.01);
        assert_eq!(json["cx"], 640.0);
        assert!(
            json.get("distortion").is_none(),
            "an empty distortion model is absent rather than an empty list: a consumer reading \
             `[]` has to know that means `unknown` rather than `none`"
        );
    }
}
