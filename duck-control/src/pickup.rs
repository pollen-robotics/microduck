//! Pick-up detection: is somebody holding the robot?
//!
//! A policy keeps stepping when the duck is lifted — it has no idea its feet left the floor —
//! so a robot picked up mid-walk thrashes in the hand until somebody presses Start. This is the
//! classifier that notices instead: every tick it sees the last second of what the loop already
//! reads, and says how likely it is that a hand is carrying the robot. `robotd` pauses the
//! policy on a confident "held" (ramping to [`PAUSE_POSE`]) and resumes it on a confident "on
//! the floor".
//!
//! The model is trained entirely in simulation (`microduck_rl`, `pickup/` and
//! `scripts/pickup_*.py`): the deployed walking policy runs under the training randomisation
//! while a simulated hand lifts, carries, tilts, shakes, sets down and drops the robot, and a
//! small temporal CNN learns p(held) from the window below. Everything in this file is the
//! contract with that training code — the feature layout, the pause pose and the hysteresis
//! were all fixed there, and changing one here without retraining is the silent kind of wrong.
//!
//! ```text
//! index   width  contents (one row per tick, the model sees the last WINDOW rows)
//! 0..3        3  gyro, trunk frame, rad/s
//! 3..6        3  projected gravity, trunk frame, unit vector
//! 6..20      14  joint position minus home pose, mouth excluded
//! 20..34     14  joint velocity, mouth excluded
//! 34..48     14  the target commanded on the PREVIOUS tick, minus home pose, mouth excluded
//! 48..62     14  |present current|, amperes, mouth excluded
//! 62          1  1.0 if that previous target was a pause target, else 0.0
//! ```
//!
//! The target and the pause flag are the previous tick's because the training rows pair each
//! sensor reading with the command that produced it: a row is "what the robot did with what it
//! was told". Current is in the contract but the shipped model does not read it — the graph drops
//! those columns before its first layer. In simulation current separated held from standing far
//! better than it does on a real robot, so the model was trained without it.
//!
//! v2 (2026-10-03): the first model missed a robot lifted by the HEAD and resumed whenever the robot
//! was turned far from upright (180° about either axis) — the simulated hand had only ever gripped
//! the trunk and only passed through large tilts. v2 is trained with head grips, sustained holds at
//! any orientation and quick 180° turns; it is also a third of the size (8k parameters, current
//! dropped from the graph), because v1 cost ~0.5 ms a tick on the board — 1.4× a policy inference.

use std::path::{Path, PathBuf};
use std::time::Duration;

use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use ort::value::Tensor;

use crate::imu::ImuData;
use crate::model::{DEFAULT_POSITION, MOUTH_INDEX, NUM_JOINTS};
use crate::obs::{ACTION_LEN, Observation, policy_joints};
use crate::policy::{PolicyError, catching_ort_panics, ensure_runtime, tensor_shape};

/// Width of one feature row.
pub const FEAT_LEN: usize = 63;
/// Rows the model sees: one second at 50 Hz.
pub const WINDOW: usize = 50;

/// The pose a paused robot holds, as offsets from home in policy order (mouth excluded).
///
/// Not the home pose. In simulation, a robot holding home at the walking gain on the floor
/// topples — half of them past 40° within a second — because home is a balanced pose only for
/// a policy that keeps balancing it. This is the walking policy's own average stance at zero
/// command, mirrored left/right: it tips far more slowly, and it is the stance the policy
/// expects to wake up in when the robot is put back down.
pub const PAUSE_POSE: [f64; ACTION_LEN] = [
    0.0, -0.100, 0.038, -0.145, 0.044, // left hip yaw, hip roll, hip pitch, knee, ankle
    -0.096, 0.008, 0.0, 0.0, // neck pitch, head pitch, head yaw, head roll
    0.0, 0.100, -0.038, 0.145, -0.044, // right hip yaw, hip roll, hip pitch, knee, ankle
];

/// How long the move from the last policy target to [`PAUSE_POSE`] takes.
pub const PAUSE_RAMP: Duration = Duration::from_millis(300);

/// One feature row. See the module table.
pub fn features(
    imu: &ImuData,
    positions: &[f64; NUM_JOINTS],
    velocities: &[f64; NUM_JOINTS],
    currents_ma: &[f64; NUM_JOINTS],
    previous_target: &[f64; NUM_JOINTS],
    previous_paused: bool,
) -> [f32; FEAT_LEN] {
    let mut row = [0.0f32; FEAT_LEN];
    let home = policy_joints(&DEFAULT_POSITION);
    let angles = policy_joints(positions);
    let target = policy_joints(previous_target);
    let rates = policy_joints(velocities);
    let amps = policy_joints(currents_ma);
    for i in 0..3 {
        row[i] = imu.gyro[i] as f32;
        row[3 + i] = imu.gravity[i] as f32;
    }
    for j in 0..ACTION_LEN {
        row[6 + j] = (angles[j] - home[j]) as f32;
        row[20 + j] = rates[j] as f32;
        row[34 + j] = (target[j] - home[j]) as f32;
        row[48 + j] = (amps[j].abs() / 1000.0) as f32;
    }
    row[62] = if previous_paused { 1.0 } else { 0.0 };
    row
}

/// The joint targets of a robot `elapsed` into a pause that began at `from`.
///
/// A linear ramp from `from` to [`PAUSE_POSE`], then the pose itself. The mouth is not part of
/// the pose and keeps whatever `from` had.
pub fn pause_target(from: &[f64; NUM_JOINTS], elapsed: Duration) -> [f64; NUM_JOINTS] {
    let t = (elapsed.as_secs_f64() / PAUSE_RAMP.as_secs_f64()).clamp(0.0, 1.0);
    let offsets = Observation::scatter_action(&PAUSE_POSE.map(|v| v as f32));
    std::array::from_fn(|i| {
        if i == MOUTH_INDEX {
            from[i]
        } else {
            let pose = DEFAULT_POSITION[i] + offsets[i];
            from[i] + (pose - from[i]) * t
        }
    })
}

/// The classifier and the window it reads.
pub struct Detector {
    session: Session,
    path: PathBuf,
    /// Ring buffer of rows; `next` is where the next row goes, which is also the oldest row
    /// once the window is full.
    rows: Box<[[f32; FEAT_LEN]; WINDOW]>,
    next: usize,
    filled: usize,
    /// The model input, oldest row first — rebuilt each tick from the ring.
    input: Vec<f32>,
}

impl std::fmt::Debug for Detector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Detector")
            .field("path", &self.path)
            .field("filled", &self.filled)
            .finish_non_exhaustive()
    }
}

impl Detector {
    /// Load and validate the model: `features[1, WINDOW, FEAT_LEN] → p_held[1]`.
    ///
    /// Validated here, at startup, for the same reason the policies are: a wrong file must be
    /// reported while the robot is standing still, not discovered on the first tick.
    pub fn load(path: &Path) -> Result<Self, PolicyError> {
        ensure_runtime()?;
        let session = catching_ort_panics(|| {
            Session::builder()
                .and_then(|b| b.with_optimization_level(GraphOptimizationLevel::Level3))
                // One thread, like every other network on the control thread.
                .and_then(|b| b.with_intra_threads(1))
                .and_then(|b| b.commit_from_file(path))
                .map_err(|source| PolicyError::Load {
                    path: path.to_owned(),
                    source,
                })
        })?;
        let shape_error = |what: &'static str, expected: &str, got: String| PolicyError::Shape {
            path: path.to_owned(),
            what,
            expected: expected.to_owned(),
            got,
        };
        let (inputs, outputs) = (session.inputs(), session.outputs());
        if inputs.len() != 1 || outputs.len() != 1 {
            return Err(shape_error(
                "input/output contract",
                "features -> p_held",
                format!("{} inputs, {} outputs", inputs.len(), outputs.len()),
            ));
        }
        let input = &inputs[0];
        if input.name() != "features" {
            return Err(shape_error(
                "input name",
                "features",
                input.name().to_owned(),
            ));
        }
        let shape = tensor_shape(path, input)?;
        if shape != [1, WINDOW as i64, FEAT_LEN as i64] {
            return Err(shape_error(
                "features shape",
                &format!("[1, {WINDOW}, {FEAT_LEN}]"),
                format!("{shape:?}"),
            ));
        }
        let mut detector = Self {
            session,
            path: path.to_owned(),
            rows: Box::new([[0.0; FEAT_LEN]; WINDOW]),
            next: 0,
            filled: 0,
            input: vec![0.0; WINDOW * FEAT_LEN],
        };
        // Pay the first-inference cost now rather than on the first tick that counts.
        detector.infer()?;
        Ok(detector)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Forget the window. The next decision then waits for a full second of fresh rows.
    pub fn reset(&mut self) {
        self.next = 0;
        self.filled = 0;
    }

    /// Add one tick's row and score the window, or `None` while it is still filling.
    ///
    /// The model was only ever shown complete windows of real history, so a window padded with
    /// copies of its first row is out of distribution — padding it this way at startup made
    /// the simulated robot pause itself a tenth of a second after boot.
    pub fn observe(&mut self, row: [f32; FEAT_LEN]) -> Result<Option<f32>, PolicyError> {
        self.rows[self.next] = row;
        self.next = (self.next + 1) % WINDOW;
        self.filled = (self.filled + 1).min(WINDOW);
        if self.filled < WINDOW {
            return Ok(None);
        }
        self.infer().map(Some)
    }

    fn infer(&mut self) -> Result<f32, PolicyError> {
        for k in 0..WINDOW {
            let row = &self.rows[(self.next + k) % WINDOW];
            self.input[k * FEAT_LEN..(k + 1) * FEAT_LEN].copy_from_slice(row);
        }
        let fail = |e: String| PolicyError::Inference(format!("pickup detector: {e}"));
        let tensor = Tensor::from_array(([1usize, WINDOW, FEAT_LEN], self.input.clone()))
            .map_err(|e| fail(e.to_string()))?;
        let outputs = self
            .session
            .run(ort::inputs!["features" => tensor])
            .map_err(|e| fail(e.to_string()))?;
        let (_, p) = outputs[0]
            .try_extract_tensor::<f32>()
            .map_err(|e| fail(e.to_string()))?;
        match p.first() {
            Some(&p) if p.is_finite() => Ok(p),
            _ => Err(fail(format!("expected one finite probability, got {p:?}"))),
        }
    }
}

/// When a probability becomes a decision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thresholds {
    /// Pause once p(held) has stayed above this for [`Self::pause_ticks`] ticks.
    pub pause_above: f32,
    pub pause_ticks: u32,
    /// Resume once p(held) has stayed below this for [`Self::resume_ticks`] ticks…
    pub resume_below: f32,
    pub resume_ticks: u32,
    /// …and the pause has lasted at least this long, so one confident tick cannot flicker it.
    pub min_pause: Duration,
}

impl Default for Thresholds {
    /// The values the model was evaluated with, closed loop, in simulation.
    ///
    /// Resume is the fast side on purpose. A paused robot set down on the floor is holding a
    /// fixed pose, and a fixed pose tips: in simulation 6 % of set-down robots went past 40°
    /// before a resume under 0.25 s, and 36 % before one over 0.5 s. Resuming at
    /// 0.35 × 4 ticks gave a 0.18 s median and 3 % tipped, against 0.30 s and 12 % at
    /// 0.2 × 10 ticks — for 1.5 mid-carry false resumes an hour, each a moment of legs moving
    /// in the hand before the next pause.
    fn default() -> Self {
        Self {
            pause_above: 0.8,
            pause_ticks: 5,
            resume_below: 0.35,
            resume_ticks: 4,
            min_pause: Duration::from_millis(300),
        }
    }
}

/// What [`Latch::update`] decided this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    Pause,
    Resume,
}

/// Hysteresis over p(held): the robot's paused-because-held state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Latch {
    thresholds: Thresholds,
    paused: bool,
    run: u32,
    paused_for: Duration,
}

impl Latch {
    pub fn new(thresholds: Thresholds) -> Self {
        Self {
            thresholds,
            paused: false,
            run: 0,
            paused_for: Duration::ZERO,
        }
    }

    pub fn paused(&self) -> bool {
        self.paused
    }

    /// How long the current pause has lasted; zero when not paused.
    pub fn paused_for(&self) -> Duration {
        self.paused_for
    }

    /// Back to not paused, with no evidence either way.
    pub fn reset(&mut self) {
        self.paused = false;
        self.run = 0;
        self.paused_for = Duration::ZERO;
    }

    /// One tick's probability.
    pub fn update(&mut self, p_held: f32, period: Duration) -> Option<Edge> {
        let t = &self.thresholds;
        if self.paused {
            self.paused_for += period;
        }
        let toward_flip = if self.paused {
            p_held < t.resume_below && self.paused_for >= t.min_pause
        } else {
            p_held > t.pause_above
        };
        self.run = if toward_flip { self.run + 1 } else { 0 };
        let needed = if self.paused {
            t.resume_ticks
        } else {
            t.pause_ticks
        };
        if self.run < needed {
            return None;
        }
        self.run = 0;
        self.paused = !self.paused;
        self.paused_for = Duration::ZERO;
        Some(if self.paused {
            Edge::Pause
        } else {
            Edge::Resume
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK: Duration = Duration::from_millis(20);

    fn run(latch: &mut Latch, p: f32, ticks: u32) -> Vec<Edge> {
        (0..ticks).filter_map(|_| latch.update(p, TICK)).collect()
    }

    /// A single confident spike — a stumble, a hard footfall — must not pause a walking robot.
    #[test]
    fn a_short_spike_does_not_pause() {
        let mut latch = Latch::new(Thresholds::default());
        assert!(run(&mut latch, 0.99, 4).is_empty());
        assert!(run(&mut latch, 0.1, 1).is_empty());
        assert!(run(&mut latch, 0.99, 4).is_empty());
        assert!(!latch.paused());
        assert_eq!(run(&mut latch, 0.99, 1), vec![Edge::Pause]);
    }

    /// The pause holds through a confident "on the floor" until it is `min_pause` old, or the
    /// first tick after a pause — when the window still reads the hand's approach — could flip it
    /// straight back.
    #[test]
    fn resume_waits_for_the_minimum_pause_then_needs_its_own_run() {
        let t = Thresholds::default();
        let mut latch = Latch::new(t);
        run(&mut latch, 0.99, t.pause_ticks);
        assert!(latch.paused());
        let min_ticks = (t.min_pause.as_millis() / TICK.as_millis()) as u32;
        // `min_pause` reached on tick `min_ticks`, then `resume_ticks` in a row.
        assert!(run(&mut latch, 0.0, min_ticks + t.resume_ticks - 2).is_empty());
        assert_eq!(run(&mut latch, 0.0, 1), vec![Edge::Resume]);
        assert!(!latch.paused());
    }

    /// The thresholds are a band: a probability inside it moves neither way.
    #[test]
    fn a_probability_between_the_thresholds_holds_the_state() {
        let mut latch = Latch::new(Thresholds::default());
        assert!(run(&mut latch, 0.6, 500).is_empty());
        run(&mut latch, 0.99, 5);
        assert!(run(&mut latch, 0.6, 500).is_empty());
        assert!(latch.paused());
    }

    #[test]
    fn the_pause_pose_is_mirror_symmetric() {
        // Home is mirrored for every leg joint, so a symmetric posture has opposite offsets.
        for j in 0..5 {
            assert_eq!(PAUSE_POSE[j], -PAUSE_POSE[9 + j], "joint {j}");
        }
        assert_eq!(PAUSE_POSE[7], 0.0, "no head yaw");
        assert_eq!(PAUSE_POSE[8], 0.0, "no head roll");
    }

    #[test]
    fn the_pause_ramp_starts_where_the_robot_was_and_ends_on_the_pose() {
        let mut from = DEFAULT_POSITION;
        from[0] = 0.5;
        from[MOUTH_INDEX] = 0.3;
        assert_eq!(pause_target(&from, Duration::ZERO), from);
        let end = pause_target(&from, PAUSE_RAMP * 2);
        let offsets = Observation::scatter_action(&PAUSE_POSE.map(|v| v as f32));
        for i in 0..NUM_JOINTS {
            let want = if i == MOUTH_INDEX {
                0.3
            } else {
                DEFAULT_POSITION[i] + offsets[i]
            };
            assert!((end[i] - want).abs() < 1e-12, "joint {i}");
        }
    }

    /// The row layout is the training contract: every block where the table says, the mouth
    /// skipped, currents in amperes, and the previous tick's target relative to home.
    #[test]
    fn the_feature_row_follows_the_training_layout() {
        let imu = ImuData {
            gyro: [0.1, 0.2, 0.3],
            gravity: [0.0, 0.0, -1.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        };
        let mut positions = DEFAULT_POSITION;
        positions[10] += 0.25; // right hip yaw = policy slot 9
        let mut velocities = [0.0; NUM_JOINTS];
        velocities[MOUTH_INDEX] = 9.0; // must not appear anywhere
        velocities[14] = 1.5; // right ankle = policy slot 13
        let mut currents = [0.0; NUM_JOINTS];
        currents[3] = 420.0; // left knee, mA
        let mut target = DEFAULT_POSITION;
        target[0] += 0.05;
        let row = features(&imu, &positions, &velocities, &currents, &target, true);
        assert_eq!(&row[0..6], &[0.1, 0.2, 0.3, 0.0, 0.0, -1.0]);
        assert!((row[6 + 9] - 0.25).abs() < 1e-6);
        assert_eq!(row[20 + 13], 1.5);
        assert!(!row[20..34].contains(&9.0));
        assert!((row[34] - 0.05).abs() < 1e-6);
        assert!((row[48 + 3] - 0.42).abs() < 1e-6);
        assert_eq!(row[62], 1.0);
    }
}
