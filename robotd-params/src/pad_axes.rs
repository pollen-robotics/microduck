//! `[pad_axes]`, `[pad_head]`, `[pad_body]` and `[pad_roller]`: which stick drives which command,
//! and how far full deflection asks it to go. `pad-map` evaluates them for `padd` and `netpadd`.
//!
//! The defaults are the shipped mapping, signs included. A stick's sign is gilrs's: up and right
//! read positive on every pad it normalises. The robot's frame has `vy` positive to the left and
//! `vyaw` counter-clockwise, which is why strafe and turn are inverted.

use serde::{Deserialize, Serialize};

use crate::PadDriveParams;

/// A logical stick axis, as gilrs normalises it, or nothing.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AxisSource {
    #[default]
    None,
    LeftX,
    LeftY,
    RightX,
    RightY,
}

/// [`AxisSource`]'s spellings, for the editor's choice list.
pub const AXIS_SOURCES: &[&str] = &["none", "left_x", "left_y", "right_x", "right_y"];

/// One command's stick: which axis, which way round, and how sensitive.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct AxisBinding {
    pub source: AxisSource,
    pub invert: bool,
    /// Applied before the clamp to full deflection, so above 1 it reaches the limit early.
    pub gain: f64,
}

impl AxisBinding {
    const fn new(source: AxisSource, invert: bool) -> Self {
        Self {
            source,
            invert,
            gain: 1.0,
        }
    }

    /// The command's share of its limit, in `[-1, 1]`: deadzone, then inversion, gain and the
    /// clamp. The deadzone zeroes small deflections without rescaling what is left, as the
    /// prototype did.
    pub fn evaluate(&self, sticks: Sticks, deadzone: f64) -> f64 {
        let value = match self.source {
            AxisSource::None => return 0.0,
            AxisSource::LeftX => sticks.left_x,
            AxisSource::LeftY => sticks.left_y,
            AxisSource::RightX => sticks.right_x,
            AxisSource::RightY => sticks.right_y,
        };
        if !value.is_finite() || value.abs() < deadzone {
            return 0.0;
        }
        let sign = if self.invert { -1.0 } else { 1.0 };
        (value * sign * self.gain).clamp(-1.0, 1.0)
    }
}

/// The four stick axes of one frame, raw.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sticks {
    pub left_x: f64,
    pub left_y: f64,
    pub right_x: f64,
    pub right_y: f64,
}

/// A binding as the file writes it: every field optional, so a gain-only edit keeps that mode's
/// source and sign. Deserialising [`AxisBinding`] directly would default the rest and silently
/// switch the binding off.
#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields, default)]
struct BindingOverride {
    source: Option<AxisSource>,
    invert: Option<bool>,
    gain: Option<f64>,
}

impl BindingOverride {
    fn apply(self, binding: &mut AxisBinding) {
        if let Some(source) = self.source {
            binding.source = source;
        }
        if let Some(invert) = self.invert {
            binding.invert = invert;
        }
        if let Some(gain) = self.gain {
            binding.gain = gain;
        }
    }
}

/// One mode's bindings: the struct, its shipped defaults, a deserialiser that lays the file's
/// overrides on top of them, and the list of its registry keys.
macro_rules! mode_axes {
    (
        $(#[$doc:meta])*
        $name:ident, $mode:literal {
            $( $field:ident: $source:ident $(, $invert:ident)? ; )*
        }
    ) => {
        $(#[$doc])*
        #[derive(Debug, Clone, PartialEq, Serialize)]
        pub struct $name {
            $( pub $field: AxisBinding, )*
        }

        impl Default for $name {
            fn default() -> Self {
                Self {
                    $( $field: AxisBinding::new(AxisSource::$source, mode_axes!(@invert $($invert)?)), )*
                }
            }
        }

        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                #[derive(Default, Deserialize)]
                #[serde(deny_unknown_fields, default)]
                struct Overrides {
                    $( $field: Option<BindingOverride>, )*
                }
                let overrides = Overrides::deserialize(deserializer)?;
                let mut axes = Self::default();
                $(
                    if let Some(o) = overrides.$field {
                        o.apply(&mut axes.$field);
                    }
                )*
                Ok(axes)
            }
        }

        impl $name {
            fn bindings(&self) -> impl Iterator<Item = (&'static str, &AxisBinding)> {
                [ $( (concat!("pad_axes.", $mode, ".", stringify!($field)), &self.$field), )* ]
                    .into_iter()
            }
        }
    };
    (@invert inverted) => { true };
    (@invert) => { false };
}

mode_axes! {
    /// D-pad left: the left stick walks and strafes, the right stick turns. Also the sticks of
    /// head + move while the pad's IMU has the head.
    DriveAxes, "drive" {
        vx: LeftY;
        vy: LeftX, inverted;
        vyaw: RightX, inverted;
    }
}

mode_axes! {
    /// D-pad up: all four axes pose the head, and the body holds still. The prototype's alpha
    /// mapping, signs included — its head_pitch and head_yaw joint axes run opposite to the
    /// stick, verified on hardware there and kept verbatim.
    HeadAxes, "head" {
        neck_pitch: RightY;
        head_pitch: LeftY, inverted;
        head_yaw: LeftX, inverted;
        head_roll: RightX;
    }
}

mode_axes! {
    /// D-pad right, when the right stick has the head: the left stick walks and turns — no
    /// strafe, the right stick is busy — and the right stick looks around with the signs head
    /// mode gives its left stick.
    HeadDriveAxes, "head_drive" {
        vx: LeftY;
        vy: None;
        vyaw: LeftX, inverted;
        neck_pitch: None;
        head_pitch: RightY, inverted;
        head_yaw: RightX, inverted;
        head_roll: None;
    }
}

mode_axes! {
    /// D-pad down: the left stick crouches and leans the standing robot sideways, and the right
    /// stick looks around as in head + move, so it means one thing wherever it poses the head.
    /// No forward/back tilt: the right stick has the head. The lean keeps the old body mode's
    /// sign, moved from the right stick to the left.
    BodyPoseAxes, "body_pose" {
        z: LeftY;
        pitch: None;
        roll: LeftX;
        neck_pitch: None;
        head_pitch: RightY, inverted;
        head_yaw: RightX, inverted;
        head_roll: None;
    }
}

/// `[pad_axes]`: the deadzone and every mode's bindings. Only overrides are written; a field left
/// out keeps the mode's shipped binding.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadAxesParams {
    /// Deflection below this counts as centre. Analogue sticks rarely rest at exactly zero, and
    /// without it the robot creeps. The prototype's value.
    pub deadzone: f64,
    pub drive: DriveAxes,
    pub head: HeadAxes,
    pub head_drive: HeadDriveAxes,
    pub body_pose: BodyPoseAxes,
}

impl Default for PadAxesParams {
    fn default() -> Self {
        Self {
            deadzone: 0.1,
            drive: DriveAxes::default(),
            head: HeadAxes::default(),
            head_drive: HeadDriveAxes::default(),
            body_pose: BodyPoseAxes::default(),
        }
    }
}

impl PadAxesParams {
    /// Every binding with its registry key prefix, for validation.
    pub fn bindings(&self) -> impl Iterator<Item = (&'static str, &AxisBinding)> {
        self.drive
            .bindings()
            .chain(self.head.bindings())
            .chain(self.head_drive.bindings())
            .chain(self.body_pose.bindings())
    }
}

/// `[pad_head]`: full-deflection head travel per joint, radians. The head command feeds the
/// policy's observation rather than a servo directly, so this is the prototype's generous 2.5 —
/// the network itself decides how far the head actually goes. Also the clamp on the pad-IMU head.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadHeadParams {
    pub neck_pitch_max: f64,
    pub head_pitch_max: f64,
    pub head_yaw_max: f64,
    pub head_roll_max: f64,
}

impl Default for PadHeadParams {
    fn default() -> Self {
        Self::all(2.5)
    }
}

impl PadHeadParams {
    /// One limit for every joint — `--max-head`.
    pub fn all(max: f64) -> Self {
        Self {
            neck_pitch_max: max,
            head_pitch_max: max,
            head_yaw_max: max,
            head_roll_max: max,
        }
    }
}

/// `[pad_body]`: body-pose stick ranges, from the training env via the prototype. Height in
/// metres is asymmetric — little headroom up at the standing height, more crouch down — and the
/// tilts are radians, capped at ~15°.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadBodyParams {
    pub z_min: f64,
    pub z_max: f64,
    pub pitch_max: f64,
    pub roll_max: f64,
}

impl Default for PadBodyParams {
    fn default() -> Self {
        Self {
            z_min: -0.025,
            z_max: 0.010,
            pitch_max: 0.2618,
            roll_max: 0.2618,
        }
    }
}

/// `[pad_roller]`: `[pad_drive]`'s counterpart on wheels. The prototype's roller shaping: push
/// harder than you can brake, no strafe, and heading capped at 0.3 rad/s regardless of the walking
/// limits — the roller launch line's `--max-angular-vel 0.3`, unchanged across both of its eras.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadRollerParams {
    pub vx_min: f64,
    pub vx_max: f64,
    pub vy_min: f64,
    pub vy_max: f64,
    pub vyaw_min: f64,
    pub vyaw_max: f64,
}

impl Default for PadRollerParams {
    fn default() -> Self {
        Self {
            vx_min: -0.5,
            vx_max: 0.6,
            vy_min: 0.0,
            vy_max: 0.0,
            vyaw_min: -0.3,
            vyaw_max: 0.3,
        }
    }
}

impl PadRollerParams {
    /// The same bounds as walking ones, so both are scaled by one function.
    pub fn limits(&self) -> PadDriveParams {
        PadDriveParams {
            vx_min: self.vx_min,
            vx_max: self.vx_max,
            vy_min: self.vy_min,
            vy_max: self.vy_max,
            vyaw_min: self.vyaw_min,
            vyaw_max: self.vyaw_max,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadzone_is_not_rescaled_and_gain_is_clamped() {
        let x = |left_x| Sticks {
            left_x,
            ..Default::default()
        };
        let mut axis = AxisBinding::new(AxisSource::LeftX, false);
        assert_eq!(axis.evaluate(x(0.09), 0.1), 0.0);
        assert_eq!(axis.evaluate(x(0.1), 0.1), 0.1);
        assert_eq!(axis.evaluate(x(f64::NAN), 0.1), 0.0);
        axis.gain = 3.0;
        axis.invert = true;
        assert_eq!(axis.evaluate(x(0.5), 0.1), -1.0);
        axis.source = AxisSource::None;
        assert_eq!(axis.evaluate(x(1.0), 0.1), 0.0);
    }

    /// Every binding is listed once under its own key — what validation walks.
    #[test]
    fn every_binding_has_its_own_key() {
        let keys: Vec<_> = PadAxesParams::default()
            .bindings()
            .map(|(k, _)| k)
            .collect();
        assert_eq!(keys.len(), 3 + 4 + 7 + 7);
        assert!(keys.contains(&"pad_axes.drive.vyaw"));
        assert!(keys.contains(&"pad_axes.body_pose.head_roll"));
        let mut unique = keys.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(unique.len(), keys.len());
    }
}
