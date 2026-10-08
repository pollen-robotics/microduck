//! Named controller profiles. Selection is independent of the channels a profile controls.
use crate::pad_axes::{AxisBinding, BodyPoseAxes, DriveAxes, HeadAxes, PadAxesParams};
use serde::{Deserialize, Serialize};

pub const MODE_BUTTONS: &[&str] = &[
    "none",
    "a",
    "b",
    "x",
    "y",
    "lb",
    "rb",
    "dpad_up",
    "dpad_right",
    "dpad_down",
    "dpad_left",
    "left_stick",
    "right_stick",
];

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PadModesParams {
    /// An empty list inherits the shipped profiles and the existing pad_axes overrides.
    pub profiles: Vec<PadMode>,
    #[serde(deserialize_with = "deserialize_button")]
    pub next_button: String,
    #[serde(deserialize_with = "deserialize_button")]
    pub previous_button: String,
}

impl Default for PadModesParams {
    fn default() -> Self {
        Self {
            profiles: vec![],
            next_button: "none".into(),
            previous_button: "none".into(),
        }
    }
}
fn deserialize_button<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<String, D::Error> {
    let button = String::deserialize(deserializer)?;
    if MODE_BUTTONS.contains(&button.as_str()) {
        Ok(button)
    } else {
        Err(serde::de::Error::custom("unknown mode-switch button"))
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PadMode {
    pub id: String,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub button: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drive: Option<DriveAxes>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub head: Option<HeadAxes>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<BodyAxes>,
    #[serde(default)]
    pub imu_head: bool,
    /// Optional stick mapping used while tilt is controlling the head.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imu_drive: Option<DriveAxes>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BodyAxes {
    pub z: AxisBinding,
    pub pitch: AxisBinding,
    pub roll: AxisBinding,
}
impl From<&BodyPoseAxes> for BodyAxes {
    fn from(a: &BodyPoseAxes) -> Self {
        Self {
            z: a.z,
            pitch: a.pitch,
            roll: a.roll,
        }
    }
}
impl PadMode {
    pub fn bindings(&self) -> Vec<&AxisBinding> {
        let mut bindings = Vec::new();
        for a in [&self.drive, &self.imu_drive].into_iter().flatten() {
            bindings.extend([&a.vx, &a.vy, &a.vyaw]);
        }
        if let Some(a) = &self.head {
            bindings.extend([&a.neck_pitch, &a.head_pitch, &a.head_yaw, &a.head_roll]);
        }
        if let Some(a) = &self.body {
            bindings.extend([&a.z, &a.pitch, &a.roll]);
        }
        bindings
    }
}
impl PadModesParams {
    pub fn effective(&self, a: &PadAxesParams) -> Vec<PadMode> {
        if !self.profiles.is_empty() {
            return self.profiles.clone();
        }
        let mode =
            |id: &str, name: &str, button: &str, drive, head, body, imu_head, imu_drive| PadMode {
                id: id.into(),
                name: name.into(),
                button: Some(button.into()),
                drive,
                head,
                body,
                imu_head,
                imu_drive,
            };
        let h = &a.head_drive;
        let b = &a.body_pose;
        vec![
            mode(
                "drive",
                "Move",
                "dpad_left",
                Some(a.drive.clone()),
                None,
                None,
                false,
                None,
            ),
            mode(
                "head",
                "Head",
                "dpad_up",
                None,
                Some(a.head.clone()),
                None,
                false,
                None,
            ),
            mode(
                "head_drive",
                "Head + move",
                "dpad_right",
                Some(DriveAxes {
                    vx: h.vx,
                    vy: h.vy,
                    vyaw: h.vyaw,
                }),
                Some(HeadAxes {
                    neck_pitch: h.neck_pitch,
                    head_pitch: h.head_pitch,
                    head_yaw: h.head_yaw,
                    head_roll: h.head_roll,
                }),
                None,
                true,
                Some(a.drive.clone()),
            ),
            mode(
                "body_pose",
                "Body + head",
                "dpad_down",
                None,
                Some(HeadAxes {
                    neck_pitch: b.neck_pitch,
                    head_pitch: b.head_pitch,
                    head_yaw: b.head_yaw,
                    head_roll: b.head_roll,
                }),
                Some(BodyAxes::from(b)),
                false,
                None,
            ),
        ]
    }
    /// Mode selection consumes the press, so a face button cannot also launch a skill.
    pub fn select(&self, profiles: &[PadMode], current: usize, button: &str) -> Option<usize> {
        if profiles.is_empty() || button.is_empty() {
            return None;
        }
        if button == self.next_button {
            return Some((current + 1) % profiles.len());
        }
        if button == self.previous_button {
            return Some((current + profiles.len() - 1) % profiles.len());
        }
        profiles
            .iter()
            .position(|m| m.button.as_deref() == Some(button))
    }
    pub fn validate(&self, axes: &PadAxesParams) -> Result<(), &'static str> {
        let profiles = self.effective(axes);
        let mut ids = std::collections::BTreeSet::new();
        let mut buttons = std::collections::BTreeSet::new();
        for button in [&self.next_button, &self.previous_button] {
            if !MODE_BUTTONS.contains(&button.as_str()) {
                return Err("unknown mode-switch button");
            }
            if button.as_str() != "none" && !buttons.insert(button.as_str()) {
                return Err("a button can have only one mode-switch action");
            }
        }
        for m in &profiles {
            if m.id.is_empty()
                || !m
                    .id
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
                || !ids.insert(m.id.as_str())
            {
                return Err("mode ids must be unique, nonempty and use letters, numbers, _ or -");
            }
            if m.name.trim().is_empty() {
                return Err("mode names must not be blank");
            }
            if let Some(b) = &m.button {
                if b.is_empty() || !MODE_BUTTONS.contains(&b.as_str()) {
                    return Err("unknown mode-selection button");
                }
                if b.as_str() != "none" && !buttons.insert(b.as_str()) {
                    return Err("a button can have only one mode-switch action");
                }
            }
            if m.imu_head && m.head.is_none() {
                return Err("controller tilt requires head control in that mode");
            }
            if m.imu_drive.is_some() && (!m.imu_head || m.drive.is_none()) {
                return Err("tilt driving mappings require tilt and movement control");
            }
            if m.bindings()
                .iter()
                .any(|b| !b.gain.is_finite() || b.gain < 0.0)
            {
                return Err("mode gains must be finite and nonnegative");
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn x_toggles_two_named_modes_and_cycles_any_number_on_press_edges() {
        let mut config = PadModesParams {
            next_button: "x".into(),
            ..Default::default()
        };
        config.profiles = config
            .effective(&PadAxesParams::default())
            .into_iter()
            .take(2)
            .collect();
        for m in &mut config.profiles {
            m.button = None;
        }
        config.profiles[0].name = "Slow walking".into();
        config.profiles[1].name = "Look around".into();
        assert_eq!(config.validate(&PadAxesParams::default()), Ok(()));
        assert_eq!(config.select(&config.profiles, 0, "x"), Some(1));
        assert_eq!(config.select(&config.profiles, 1, "x"), Some(0));
        assert_eq!(config.select(&config.profiles, 0, "a"), None);
        let mut extra = config.profiles[0].clone();
        extra.id = "third".into();
        config.profiles.push(extra);
        assert_eq!(config.select(&config.profiles, 1, "x"), Some(2));
        assert_eq!(config.select(&config.profiles, 2, "x"), Some(0));
        config.previous_button = "y".into();
        assert_eq!(config.select(&config.profiles, 0, "y"), Some(2));
    }
    #[test]
    fn ambiguous_buttons_duplicate_ids_and_invalid_gains_are_refused() {
        let axes = PadAxesParams::default();
        let mut config = PadModesParams {
            profiles: PadModesParams::default().effective(&axes),
            ..Default::default()
        };
        config.next_button = "dpad_left".into();
        assert!(config.validate(&axes).is_err());
        config.next_button = "none".into();
        config.profiles[1].id = "drive".into();
        assert!(config.validate(&axes).is_err());
        config.profiles[1].id = "head".into();
        config.profiles[0].drive.as_mut().unwrap().vx.gain = f64::NAN;
        assert!(config.validate(&axes).is_err());
    }
}
