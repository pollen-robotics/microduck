//! Pad axes in robot coordinates. Defaults preserve the shipped controller mapping.
use serde::{Deserialize, Serialize};

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
pub const AXIS_SOURCES: &[&str] = &["none", "left_x", "left_y", "right_x", "right_y"];

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct AxisBinding {
    pub source: AxisSource,
    pub invert: bool,
    pub gain: f64,
}
impl Default for AxisBinding {
    fn default() -> Self {
        Self::new(AxisSource::None, false)
    }
}
impl AxisBinding {
    const fn new(source: AxisSource, invert: bool) -> Self {
        Self {
            source,
            invert,
            gain: 1.0,
        }
    }
    /// Normalize the selected stick before scaling to a command's physical limits.
    pub fn evaluate(&self, sticks: [f64; 4], deadzone: f64) -> f64 {
        let value = match self.source {
            AxisSource::None => 0.0,
            AxisSource::LeftX => sticks[0],
            AxisSource::LeftY => sticks[1],
            AxisSource::RightX => sticks[2],
            AxisSource::RightY => sticks[3],
        };
        if !value.is_finite() || value.abs() < deadzone {
            return 0.0;
        }
        let sign = if self.invert { -1.0 } else { 1.0 };
        (value * sign * self.gain).clamp(-1.0, 1.0)
    }
}

// A gain-only edit must keep that mode's source and sign. Deserializing each binding
// independently with AxisBinding::default() would silently disable it instead.
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

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DriveAxes {
    pub vx: AxisBinding,
    pub vy: AxisBinding,
    pub vyaw: AxisBinding,
}
impl Default for DriveAxes {
    fn default() -> Self {
        Self {
            vx: AxisBinding::new(AxisSource::LeftY, false),
            vy: AxisBinding::new(AxisSource::LeftX, true),
            vyaw: AxisBinding::new(AxisSource::RightX, true),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct HeadAxes {
    pub neck_pitch: AxisBinding,
    pub head_pitch: AxisBinding,
    pub head_yaw: AxisBinding,
    pub head_roll: AxisBinding,
}
impl Default for HeadAxes {
    fn default() -> Self {
        Self {
            neck_pitch: AxisBinding::new(AxisSource::RightY, false),
            head_pitch: AxisBinding::new(AxisSource::LeftY, true),
            head_yaw: AxisBinding::new(AxisSource::LeftX, true),
            head_roll: AxisBinding::new(AxisSource::RightX, false),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct HeadDriveAxes {
    pub vx: AxisBinding,
    pub vy: AxisBinding,
    pub vyaw: AxisBinding,
    pub neck_pitch: AxisBinding,
    pub head_pitch: AxisBinding,
    pub head_yaw: AxisBinding,
    pub head_roll: AxisBinding,
}
impl Default for HeadDriveAxes {
    fn default() -> Self {
        Self {
            vx: AxisBinding::new(AxisSource::LeftY, false),
            vy: AxisBinding::new(AxisSource::None, false),
            vyaw: AxisBinding::new(AxisSource::LeftX, true),
            neck_pitch: AxisBinding::new(AxisSource::None, false),
            head_pitch: AxisBinding::new(AxisSource::RightY, true),
            head_yaw: AxisBinding::new(AxisSource::RightX, true),
            head_roll: AxisBinding::new(AxisSource::None, false),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BodyPoseAxes {
    pub z: AxisBinding,
    pub pitch: AxisBinding,
    pub roll: AxisBinding,
    pub neck_pitch: AxisBinding,
    pub head_pitch: AxisBinding,
    pub head_yaw: AxisBinding,
    pub head_roll: AxisBinding,
}
impl Default for BodyPoseAxes {
    fn default() -> Self {
        Self {
            z: AxisBinding::new(AxisSource::LeftY, false),
            pitch: AxisBinding::new(AxisSource::None, false),
            roll: AxisBinding::new(AxisSource::LeftX, false),
            neck_pitch: AxisBinding::new(AxisSource::None, false),
            head_pitch: AxisBinding::new(AxisSource::RightY, true),
            head_yaw: AxisBinding::new(AxisSource::RightX, true),
            head_roll: AxisBinding::new(AxisSource::None, false),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadAxesParams {
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
    pub fn bindings(&self) -> Vec<(&'static str, &AxisBinding)> {
        vec![
            ("pad_axes.drive.vx", &self.drive.vx),
            ("pad_axes.drive.vy", &self.drive.vy),
            ("pad_axes.drive.vyaw", &self.drive.vyaw),
            ("pad_axes.head.neck_pitch", &self.head.neck_pitch),
            ("pad_axes.head.head_pitch", &self.head.head_pitch),
            ("pad_axes.head.head_yaw", &self.head.head_yaw),
            ("pad_axes.head.head_roll", &self.head.head_roll),
            ("pad_axes.head_drive.vx", &self.head_drive.vx),
            ("pad_axes.head_drive.vy", &self.head_drive.vy),
            ("pad_axes.head_drive.vyaw", &self.head_drive.vyaw),
            (
                "pad_axes.head_drive.neck_pitch",
                &self.head_drive.neck_pitch,
            ),
            (
                "pad_axes.head_drive.head_pitch",
                &self.head_drive.head_pitch,
            ),
            ("pad_axes.head_drive.head_yaw", &self.head_drive.head_yaw),
            ("pad_axes.head_drive.head_roll", &self.head_drive.head_roll),
            ("pad_axes.body_pose.z", &self.body_pose.z),
            ("pad_axes.body_pose.pitch", &self.body_pose.pitch),
            ("pad_axes.body_pose.roll", &self.body_pose.roll),
            ("pad_axes.body_pose.neck_pitch", &self.body_pose.neck_pitch),
            ("pad_axes.body_pose.head_pitch", &self.body_pose.head_pitch),
            ("pad_axes.body_pose.head_yaw", &self.body_pose.head_yaw),
            ("pad_axes.body_pose.head_roll", &self.body_pose.head_roll),
        ]
    }
}

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
        Self {
            neck_pitch_max: 2.5,
            head_pitch_max: 2.5,
            head_yaw_max: 2.5,
            head_roll_max: 2.5,
        }
    }
}

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

impl<'de> Deserialize<'de> for DriveAxes {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Default, Deserialize)]
        #[serde(deny_unknown_fields, default)]
        struct Overrides {
            vx: Option<BindingOverride>,
            vy: Option<BindingOverride>,
            vyaw: Option<BindingOverride>,
        }
        let overrides = Overrides::deserialize(deserializer)?;
        let mut result = Self::default();
        if let Some(binding) = overrides.vx {
            binding.apply(&mut result.vx);
        }
        if let Some(binding) = overrides.vy {
            binding.apply(&mut result.vy);
        }
        if let Some(binding) = overrides.vyaw {
            binding.apply(&mut result.vyaw);
        }
        Ok(result)
    }
}

impl<'de> Deserialize<'de> for HeadAxes {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Default, Deserialize)]
        #[serde(deny_unknown_fields, default)]
        struct Overrides {
            neck_pitch: Option<BindingOverride>,
            head_pitch: Option<BindingOverride>,
            head_yaw: Option<BindingOverride>,
            head_roll: Option<BindingOverride>,
        }
        let overrides = Overrides::deserialize(deserializer)?;
        let mut result = Self::default();
        if let Some(binding) = overrides.neck_pitch {
            binding.apply(&mut result.neck_pitch);
        }
        if let Some(binding) = overrides.head_pitch {
            binding.apply(&mut result.head_pitch);
        }
        if let Some(binding) = overrides.head_yaw {
            binding.apply(&mut result.head_yaw);
        }
        if let Some(binding) = overrides.head_roll {
            binding.apply(&mut result.head_roll);
        }
        Ok(result)
    }
}

impl<'de> Deserialize<'de> for HeadDriveAxes {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Default, Deserialize)]
        #[serde(deny_unknown_fields, default)]
        struct Overrides {
            vx: Option<BindingOverride>,
            vy: Option<BindingOverride>,
            vyaw: Option<BindingOverride>,
            neck_pitch: Option<BindingOverride>,
            head_pitch: Option<BindingOverride>,
            head_yaw: Option<BindingOverride>,
            head_roll: Option<BindingOverride>,
        }
        let overrides = Overrides::deserialize(deserializer)?;
        let mut result = Self::default();
        if let Some(binding) = overrides.vx {
            binding.apply(&mut result.vx);
        }
        if let Some(binding) = overrides.vy {
            binding.apply(&mut result.vy);
        }
        if let Some(binding) = overrides.vyaw {
            binding.apply(&mut result.vyaw);
        }
        if let Some(binding) = overrides.neck_pitch {
            binding.apply(&mut result.neck_pitch);
        }
        if let Some(binding) = overrides.head_pitch {
            binding.apply(&mut result.head_pitch);
        }
        if let Some(binding) = overrides.head_yaw {
            binding.apply(&mut result.head_yaw);
        }
        if let Some(binding) = overrides.head_roll {
            binding.apply(&mut result.head_roll);
        }
        Ok(result)
    }
}

impl<'de> Deserialize<'de> for BodyPoseAxes {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Default, Deserialize)]
        #[serde(deny_unknown_fields, default)]
        struct Overrides {
            z: Option<BindingOverride>,
            pitch: Option<BindingOverride>,
            roll: Option<BindingOverride>,
            neck_pitch: Option<BindingOverride>,
            head_pitch: Option<BindingOverride>,
            head_yaw: Option<BindingOverride>,
            head_roll: Option<BindingOverride>,
        }
        let overrides = Overrides::deserialize(deserializer)?;
        let mut result = Self::default();
        if let Some(binding) = overrides.z {
            binding.apply(&mut result.z);
        }
        if let Some(binding) = overrides.pitch {
            binding.apply(&mut result.pitch);
        }
        if let Some(binding) = overrides.roll {
            binding.apply(&mut result.roll);
        }
        if let Some(binding) = overrides.neck_pitch {
            binding.apply(&mut result.neck_pitch);
        }
        if let Some(binding) = overrides.head_pitch {
            binding.apply(&mut result.head_pitch);
        }
        if let Some(binding) = overrides.head_yaw {
            binding.apply(&mut result.head_yaw);
        }
        if let Some(binding) = overrides.head_roll {
            binding.apply(&mut result.head_roll);
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn deadzone_is_not_rescaled_and_gain_is_clamped() {
        let mut axis = AxisBinding::new(AxisSource::LeftX, false);
        assert_eq!(axis.evaluate([0.09, 0.0, 0.0, 0.0], 0.1), 0.0);
        assert_eq!(axis.evaluate([0.1, 0.0, 0.0, 0.0], 0.1), 0.1);
        assert_eq!(axis.evaluate([f64::NAN, 0.0, 0.0, 0.0], 0.1), 0.0);
        axis.gain = 3.0;
        axis.invert = true;
        assert_eq!(axis.evaluate([0.5, 0.0, 0.0, 0.0], 0.1), -1.0);
        axis.source = AxisSource::None;
        assert_eq!(axis.evaluate([1.0; 4], 0.1), 0.0);
    }
}
