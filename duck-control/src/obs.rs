//! The observation vector the policy sees.
//!
//! **This is the highest-risk code in the crate.** It is a flat array of 61 floats whose
//! every index must match what the policy was trained against. A wrong offset does not fail
//! loudly — it produces a plausible-looking robot that falls over, and the symptom looks
//! like a tuning or timing problem rather than an indexing one.
//!
//! Every alpha policy is `obs[1,61] → actions[1,14]`, verified across walking, standing,
//! ground pick, ball kick and sit. So there is exactly one layout, not the five the
//! prototype carried (51/54-D legacy, 49-D wheeled, 85-D tracking are all v1/v1.5 history).
//!
//! ```text
//! index   width  contents
//! 0..3        3  gyro, trunk frame, rad/s
//! 3..6        3  projected gravity, trunk frame, unit vector
//! 6..20      14  joint position minus home pose, mouth excluded
//! 20..34     14  joint velocity, mouth excluded
//! 34..48     14  previous action, mouth excluded
//! 48..61     13  command (below)
//! ```
//!
//! The command block, which is the part with no second source of truth:
//!
//! ```text
//! 48..51      3  vx, vy, vyaw
//! 51..55      4  neck_pitch, head_pitch, head_yaw, head_roll
//! 55..57      2  body x, y      — always zero, unbound in training
//! 57          1  body z
//! 58          1  body roll
//! 59          1  body pitch
//! 60          1  body yaw       — always zero, unbound in training
//! ```
//!
//! Two things about that block are easy to get wrong and were confirmed against
//! `microduck_runtime`'s `control_step` rather than assumed:
//!
//!  1. **Body x, y and yaw are hardcoded zero.** They are unbound in the training
//!     environment, so an all-zero body command is the *nominal* encoding, not a
//!     placeholder standing in for something better.
//!  2. **Head targets ride in the command, and are not added on top of the policy output.**
//!     The prototype does both, in different modes, and gates the post-hoc addition behind
//!     `if !new_cmd_obs` with the note "head_offsets are a COMMAND fed via the obs vector
//!     instead — don't double-add it here". Doing both would bend the head twice.
//!
//! Note also the order inside the body block: `z, roll, pitch`, not `z, pitch, roll`.

use crate::imu::ImuData;
use crate::model::{MOUTH_INDEX, NUM_JOINTS};

/// Total width of the observation.
pub const OBS_LEN: usize = 61;

/// Actions a policy returns — the 15 joints minus the mouth.
pub const ACTION_LEN: usize = 14;

/// These two are also a published contract, in `duck_ipc_proto` — a policy manifest declares
/// them, and `updaterd` refuses a mismatched policy on them before it downloads 800 KB. Two
/// copies of a number that decides whether a robot walks is exactly the drift this repository
/// keeps writing tests about, so here is the test.
const _: () = {
    assert!(OBS_LEN == duck_ipc_proto::POLICY_OBS_LEN);
    assert!(ACTION_LEN == duck_ipc_proto::POLICY_ACTION_LEN);
};

/// Joints that appear in the observation: all but the mouth.
pub const OBS_JOINTS: usize = NUM_JOINTS - 1;

/// Width of the trailing command block.
pub const COMMAND_LEN: usize = 13;

/// What a client is asking the robot to do, in the form the policy consumes.
///
/// Held as physical units in the trunk frame — the conversion to the flat command block
/// happens in [`Observation::build`] and nowhere else.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    /// Forward, left, yaw-rate.
    pub twist: [f64; 3],
    /// neck_pitch, head_pitch, head_yaw, head_roll.
    pub head: [f64; 4],
    /// Standing body pose: z, roll, pitch. Zero is the nominal stance.
    pub body: BodyPose,
}

impl Command {
    /// Magnitude of the velocity command, which is what selects walking versus standing.
    pub fn twist_magnitude(&self) -> f64 {
        self.twist.iter().map(|v| v * v).sum::<f64>().sqrt()
    }
}

/// Standing body pose offsets. Not commandable in slice 2 — carried so the layout is
/// complete and so the field exists when a `pose` intent lands.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BodyPose {
    pub z: f64,
    pub roll: f64,
    pub pitch: f64,
}

/// The joints a policy sees, in order, with the mouth skipped.
///
/// One definition, used for positions, velocities and the home pose alike — so those three
/// blocks cannot disagree about which joints they cover or what order they are in.
///
/// Returns an array rather than an iterator so its width is part of its type. A filtered
/// iterator is not `ExactSizeIterator` — `Filter` cannot know how many elements pass — and
/// that is what would force [`fill`] to count at runtime instead of being checked here.
fn policy_joints(values: &[f64; NUM_JOINTS]) -> [f64; OBS_JOINTS] {
    // Skipping one valid index leaves exactly one fewer. Stated as a compile-time check so
    // the reasoning is enforced rather than merely believed.
    const { assert!(OBS_JOINTS == NUM_JOINTS - 1) };
    std::array::from_fn(|slot| values[joint_of(slot)])
}

/// The joint a policy slot refers to.
///
/// Slots below the mouth map straight through; at and above, everything shifts up by one.
/// Written once and used in both directions — [`policy_joints`] reads through it,
/// [`Observation::scatter_action`] writes through it — so the two cannot disagree about
/// where the policy's n-th value belongs.
#[inline]
const fn joint_of(slot: usize) -> usize {
    if slot < MOUTH_INDEX { slot } else { slot + 1 }
}

/// Write one block, narrowing to `f32` on the way in.
///
/// Both sides are fixed-size arrays of the same `N`, so a block and its source cannot
/// disagree about width — the compiler rejects it. That matters more than it sounds: `zip`
/// stops at the shorter side, so a mismatch would otherwise leave the tail silently at
/// zero, and a zero in the observation is a joint sitting at its home pose. The policy
/// would act on a plausible robot that does not exist.
fn fill<const N: usize>(block: &mut [f32; N], values: [f64; N]) {
    *block = values.map(|value| value as f32);
}

/// A built observation, ready to hand to the policy.
///
/// A fixed array rather than a `Vec`: it is rebuilt 50 times a second on a thread that
/// should not be visiting the allocator.
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    data: [f32; OBS_LEN],
}

impl Observation {
    pub fn as_slice(&self) -> &[f32] {
        &self.data
    }

    /// An all-zero observation, for warming a session up before the control loop starts.
    ///
    /// Not a valid robot state — it is only ever fed to an inference whose output is
    /// discarded, to pay the first-call cost off the hot path.
    pub fn zeroed() -> Self {
        Self {
            data: [0.0; OBS_LEN],
        }
    }

    /// Assemble the observation.
    ///
    /// `joint_positions` are absolute; the policy sees them relative to the home pose,
    /// because that is what it was trained on. `last_action` is the previous *policy
    /// output* — raw, before action scaling — in 14-wide policy order.
    pub fn build(
        imu: &ImuData,
        joint_positions: &[f64; NUM_JOINTS],
        joint_velocities: &[f64; NUM_JOINTS],
        home_pose: &[f64; NUM_JOINTS],
        last_action: &[f32; ACTION_LEN],
        command: &Command,
    ) -> Self {
        let mut data = [0.0f32; OBS_LEN];

        // Carve the buffer into the blocks of the layout table above, by name — as
        // *arrays*, so each block's width is in its type and [`fill`] can only be handed a
        // source of matching length. `expect` cannot fire: the widths are constants that
        // sum to `OBS_LEN`, which `the_layout_widths_sum_to_the_declared_input` pins.
        const LAYOUT: &str = "block widths must sum to OBS_LEN";
        let (gyro, rest) = data.split_first_chunk_mut::<3>().expect(LAYOUT);
        let (gravity, rest) = rest.split_first_chunk_mut::<3>().expect(LAYOUT);
        let (positions, rest) = rest.split_first_chunk_mut::<OBS_JOINTS>().expect(LAYOUT);
        let (velocities, rest) = rest.split_first_chunk_mut::<OBS_JOINTS>().expect(LAYOUT);
        let (previous_action, rest) = rest.split_first_chunk_mut::<OBS_JOINTS>().expect(LAYOUT);
        let (command_block, rest) = rest.split_first_chunk_mut::<COMMAND_LEN>().expect(LAYOUT);
        debug_assert!(rest.is_empty(), "{LAYOUT}");

        fill(gyro, imu.gyro);
        fill(gravity, imu.gravity);
        let angles = policy_joints(joint_positions);
        let home = policy_joints(home_pose);
        fill(positions, std::array::from_fn(|i| angles[i] - home[i]));
        fill(velocities, policy_joints(joint_velocities));

        // Already `f32`, and the same width, so this is a plain copy rather than a
        // conversion — the previous action is the policy's own output fed back.
        *previous_action = *last_action;

        // Reads in the same order as the table, which is the point: this block is the one
        // with no second source of truth, so it should be checkable against the docs by eye.
        fill(
            command_block,
            [
                command.twist[0],
                command.twist[1],
                command.twist[2],
                command.head[0],
                command.head[1],
                command.head[2],
                command.head[3],
                0.0, // body x — unbound in training
                0.0, // body y — unbound
                command.body.z,
                command.body.roll,
                command.body.pitch,
                0.0, // body yaw — unbound
            ],
        );

        Self { data }
    }

    /// Map a policy's 14 outputs onto the 15 joints, leaving the mouth untouched.
    ///
    /// The mouth is absent from every alpha policy, so its slot stays at whatever the
    /// caller had. Getting this wrong shifts every joint after index 9 by one, which is
    /// both catastrophic and completely silent.
    pub fn scatter_action(action: &[f32; ACTION_LEN]) -> [f64; NUM_JOINTS] {
        let mut out = [0.0f64; NUM_JOINTS];
        // The mirror of `policy_joints`, through the same `joint_of` mapping: that one
        // reads the mouth out, this writes around it.
        for (slot, value) in action.iter().enumerate() {
            out[joint_of(slot)] = *value as f64;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DEFAULT_POSITION;

    fn imu() -> ImuData {
        ImuData {
            gyro: [1.0, 2.0, 3.0],
            gravity: [4.0, 5.0, 6.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }

    fn command() -> Command {
        Command {
            twist: [0.1, 0.2, 0.3],
            head: [0.4, 0.5, 0.6, 0.7],
            body: BodyPose {
                z: 0.8,
                roll: 0.9,
                pitch: 1.0,
            },
        }
    }

    fn build_with(positions: [f64; NUM_JOINTS], last_action: [f32; ACTION_LEN]) -> Observation {
        Observation::build(
            &imu(),
            &positions,
            &[0.0; NUM_JOINTS],
            &DEFAULT_POSITION,
            &last_action,
            &command(),
        )
    }

    /// The widths must sum to exactly what the ONNX graph declares. Every alpha policy is
    /// `obs[1,61]`, and a mismatch is rejected by the runtime rather than misread — but it
    /// is far better to fail here than at session run time on a robot.
    #[test]
    fn the_layout_widths_sum_to_the_declared_input() {
        assert_eq!(3 + 3 + OBS_JOINTS * 3 + COMMAND_LEN, OBS_LEN);
        assert_eq!(OBS_JOINTS, ACTION_LEN);
    }

    /// Each block must land at the offset the policy expects. This pins all six boundaries
    /// with distinguishable values, so a block that moves shows up as a specific index
    /// rather than as a robot that walks badly.
    #[test]
    fn every_block_lands_at_its_documented_offset() {
        let mut positions = DEFAULT_POSITION;
        positions[0] += 0.25; // first non-mouth joint
        let mut last_action = [0.0f32; ACTION_LEN];
        last_action[0] = -0.5;
        last_action[ACTION_LEN - 1] = 0.75;

        let obs = build_with(positions, last_action);
        let d = obs.as_slice();

        assert_eq!(&d[0..3], &[1.0, 2.0, 3.0], "gyro");
        assert_eq!(&d[3..6], &[4.0, 5.0, 6.0], "gravity");
        assert!((d[6] - 0.25).abs() < 1e-6, "joint_pos relative to home");
        assert_eq!(d[20], 0.0, "joint_vel");
        assert_eq!(d[34], -0.5, "last_action first");
        assert_eq!(d[47], 0.75, "last_action last");
        assert_eq!(&d[48..51], &[0.1, 0.2, 0.3], "twist");
        assert_eq!(&d[51..55], &[0.4, 0.5, 0.6, 0.7], "head");
    }

    /// Body x, y and yaw are unbound in training. They must be zero regardless of what the
    /// caller supplies, or the policy sees a signal it was never trained on.
    #[test]
    fn unbound_body_axes_are_always_zero() {
        let obs = build_with(DEFAULT_POSITION, [0.0; ACTION_LEN]);
        let d = obs.as_slice();
        assert_eq!(d[55], 0.0, "body x");
        assert_eq!(d[56], 0.0, "body y");
        assert_eq!(d[60], 0.0, "body yaw");
    }

    /// The body block is ordered z, roll, pitch — *not* z, pitch, roll. The prototype maps
    /// its 3-wide body command into the 6-wide slot in exactly this order, and swapping the
    /// last two would tilt the robot sideways when asked to lean forward.
    #[test]
    fn the_body_block_is_z_roll_pitch() {
        let obs = build_with(DEFAULT_POSITION, [0.0; ACTION_LEN]);
        let d = obs.as_slice();
        assert_eq!(d[57], 0.8, "body z");
        assert_eq!(d[58], 0.9, "body roll");
        assert_eq!(d[59], 1.0, "body pitch");
    }

    /// Joint positions are observed relative to the home pose. Feeding absolute angles
    /// would offset fourteen inputs by a constant the policy never saw in training.
    #[test]
    fn joint_positions_are_relative_to_the_home_pose() {
        let obs = build_with(DEFAULT_POSITION, [0.0; ACTION_LEN]);
        for (i, value) in obs.as_slice()[6..20].iter().enumerate() {
            assert!(
                value.abs() < 1e-9,
                "joint {i} should read zero at the home pose, got {value}"
            );
        }
    }

    /// The mouth is absent from the policy on both sides. If the observation included it,
    /// every joint after index 9 would shift by one — silently.
    #[test]
    fn the_mouth_is_excluded_from_the_observation() {
        let mut positions = DEFAULT_POSITION;
        positions[MOUTH_INDEX] += 1.0; // a large, unmistakable deviation

        let obs = build_with(positions, [0.0; ACTION_LEN]);
        for (i, value) in obs.as_slice()[6..20].iter().enumerate() {
            assert!(
                value.abs() < 1e-9,
                "moving the mouth changed joint slot {i} to {value}"
            );
        }
    }

    /// Scattering 14 actions back over 15 joints must skip the mouth, not shift past it.
    #[test]
    fn scattering_an_action_skips_the_mouth() {
        let mut action = [0.0f32; ACTION_LEN];
        // Distinct values so a shift is visible rather than merely nonzero.
        for (i, a) in action.iter_mut().enumerate() {
            *a = (i + 1) as f32;
        }
        let scattered = Observation::scatter_action(&action);

        assert_eq!(scattered[MOUTH_INDEX], 0.0, "mouth must be left alone");
        // Joints before the mouth line up one-to-one...
        assert_eq!(scattered[0], 1.0);
        assert_eq!(scattered[8], 9.0);
        // ...and those after it are offset by exactly one policy slot.
        assert_eq!(scattered[10], 10.0);
        assert_eq!(scattered[NUM_JOINTS - 1], ACTION_LEN as f64);
    }

    /// Walking versus standing is chosen on command magnitude, so the magnitude has to be
    /// the twist alone — head and body movement must not make the robot think it is walking.
    #[test]
    fn twist_magnitude_ignores_head_and_body() {
        let mut c = Command::default();
        assert_eq!(c.twist_magnitude(), 0.0);

        c.head = [1.0, 1.0, 1.0, 1.0];
        c.body = BodyPose {
            z: 1.0,
            roll: 1.0,
            pitch: 1.0,
        };
        assert_eq!(c.twist_magnitude(), 0.0, "only the twist counts");

        c.twist = [3.0, 4.0, 0.0];
        assert!((c.twist_magnitude() - 5.0).abs() < 1e-12);
    }
}
