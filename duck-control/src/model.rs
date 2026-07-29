//! The robot, as data.
//!
//! One variant — **alpha** — because that is the only robot that exists. Every shipped
//! policy is `alpha_*`; v1/v1.5/v1.6 are history. A second revision becomes a second set
//! of tables, which is honest until there is a second robot to generalise from.
//!
//! The numeric values here are lifted from `microduck_runtime`'s `motor.rs`, where they
//! were measured against hardware rather than derived. Re-deriving them from a datasheet
//! is exactly the kind of change that looks right and walks wrong.

/// Left leg (5) · neck/head/mouth (5) · right leg (5).
pub const NUM_JOINTS: usize = 15;

/// Dynamixel IDs, indexed as [`JOINT_NAMES`].
pub const JOINT_IDS: [u8; NUM_JOINTS] = [
    20, 21, 22, 23, 24, // left leg
    30, 31, 32, 33, 34, // neck, head, mouth
    10, 11, 12, 13, 14, // right leg
];

pub const JOINT_NAMES: [&str; NUM_JOINTS] = [
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "mouth",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
];

/// The mouth is absent from every alpha policy — they are all 61-D observation, 14-action,
/// and the action vector skips this index. Named so that omission is deliberate rather
/// than an off-by-one someone has to rediscover.
pub const MOUTH_INDEX: usize = 9;

/// Home pose. The trunk sits ~5 mm further forward than the v1.5 pose so the CoM is over
/// the ankle axis; the old pose biased the robot backwards.
///
/// Must match `HOME_FRAME` in the training env — a policy is trained against these angles
/// and observes joint positions *relative* to them, so a discrepancy here is a constant
/// offset on 14 observation slots.
pub const DEFAULT_POSITION: [f64; NUM_JOINTS] = [
    0.0,     // left_hip_yaw
    -0.0873, // left_hip_roll
    -0.4579, // left_hip_pitch
    -0.0049, // left_knee
    0.4530,  // left_ankle
    0.3491,  // neck_pitch
    0.3491,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // mouth
    0.0,     // right_hip_yaw
    0.0873,  // right_hip_roll
    0.4579,  // right_hip_pitch
    0.0049,  // right_knee
    -0.4530, // right_ankle
];

/// The `imu_to_dxl` v2 board's Dynamixel ID. It rides the motor bus and is read in the
/// same transaction as the servos ([`crate::bus`]).
pub const IMU_DXL_ID: u8 = 200;

pub const BAUD_RATE: u32 = 1_000_000;

/// EEPROM registers asserted (and corrected) at startup.
///
/// `return_delay_time` is the load-bearing one: the XL330 ships at 250, which is 500 µs of
/// turnaround *per device*. Across 16 devices that is 8 ms per tick — 40% of a 20 ms budget
/// — spent waiting for servos to get around to answering. The rest are here because the
/// runtime found them worth pinning; `shutdown = 52` is the error mask that latches on
/// overload, overheating and input-voltage faults.
pub const EXPECTED_REGISTERS: &[(&str, u8)] = &[
    ("return_delay_time", 0),
    ("baud_rate", 3), // 3 = 1 Mbps, must agree with BAUD_RATE
    ("pwm_slope", 255),
    ("shutdown", 52),
];

/// Index of a joint by name. Linear scan over 15 entries, used at startup and in tests.
pub fn joint_index(name: &str) -> Option<usize> {
    JOINT_NAMES.iter().position(|n| *n == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three tables are indexed by the same integer everywhere in the crate. If they
    /// ever diverge in length, every lookup silently reads the wrong joint.
    #[test]
    fn tables_agree_on_length() {
        assert_eq!(JOINT_IDS.len(), NUM_JOINTS);
        assert_eq!(JOINT_NAMES.len(), NUM_JOINTS);
        assert_eq!(DEFAULT_POSITION.len(), NUM_JOINTS);
    }

    /// A duplicated Dynamixel ID makes a `sync_read` return blocks that cannot be matched
    /// back to joints, and a `sync_write` command two joints at once. Both fail in ways
    /// that look like a wiring fault.
    #[test]
    fn ids_are_unique() {
        let mut seen = JOINT_IDS;
        seen.sort_unstable();
        seen.windows(2)
            .for_each(|w| assert_ne!(w[0], w[1], "duplicate Dynamixel ID {}", w[0]));
    }

    /// The IMU board shares the bus with the servos, so its ID must not collide with one.
    #[test]
    fn imu_id_does_not_collide_with_a_joint() {
        assert!(!JOINT_IDS.contains(&IMU_DXL_ID));
    }

    /// `MOUTH_INDEX` is used to skip a slot when mapping 14 policy actions onto 15 joints.
    /// Pointing it at the wrong joint would shift every action after it by one.
    #[test]
    fn mouth_index_names_the_mouth() {
        assert_eq!(JOINT_NAMES[MOUTH_INDEX], "mouth");
        assert_eq!(joint_index("mouth"), Some(MOUTH_INDEX));
    }

    /// The legs are mirrored: the roll/pitch/ankle pairs are equal and opposite. A sign
    /// typo in the home pose is invisible by inspection and makes the robot stand crooked.
    #[test]
    fn home_pose_legs_are_mirrored() {
        for (left, right) in [
            ("left_hip_roll", "right_hip_roll"),
            ("left_hip_pitch", "right_hip_pitch"),
            ("left_knee", "right_knee"),
            ("left_ankle", "right_ankle"),
        ] {
            let l = DEFAULT_POSITION[joint_index(left).unwrap()];
            let r = DEFAULT_POSITION[joint_index(right).unwrap()];
            assert!(
                (l + r).abs() < 1e-9,
                "{left} ({l}) and {right} ({r}) should be equal and opposite"
            );
        }
    }
}
