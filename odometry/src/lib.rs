//! Where the robot is, estimated from its own legs and IMU.
//!
//! A port of the prototype runtime's contact-based odometry, itself derived
//! from Rhoban's humanoid `model_service`. The idea: at any moment one point of
//! one sole is the robot's contact with the ground. Anchor that point to the
//! world (it is on flat ground, so its world Z is 0 and its world X/Y are
//! wherever it was when it became the anchor), orient the trunk by the IMU, and
//! the trunk's world position follows by forward kinematics. When some other
//! sole corner drops below the anchor — a step — the anchor moves there, at
//! that corner's current world X/Y, so the estimate never jumps.
//!
//! Two changes from the prototype:
//!   - The foot chains come from the `kinematics` crate's MJCF model instead of
//!     hand-transcribed segment tables — the geometry has one source of truth.
//!   - Each foot's chain is evaluated once per tick and its four corners are
//!     transformed through the result; the prototype re-walked a full leg chain
//!     per corner (9 chain evaluations per tick, now 2).
//!
//! Heading is whatever the IMU's integrated yaw says — there is no
//! magnetometer, so the world frame is "wherever the robot was looking at
//! boot", which is all a relative-motion consumer needs.
//!
//! Alpha only, like the daemon: v1/v1.5 geometry stayed in the prototype.
//!
//! The candidate contact points — the "sole corners" — are an [`AnchorSet`],
//! sampled on the sole mesh by `microduck_rl/scripts/odom_anchor_points.py`
//! (see [`anchors`]). [`Odometry::alpha`] uses [`ALPHA16`], a 4x4 grid over
//! the whole sole on the mesh: in the MuJoCo twin over a 3.2 m walk it drifted
//! 9 mm where the legacy v1.5 bbox ([`V15`]) drifted 17 mm and the flat
//! patch's four corners ([`ALPHA4`]) 16 mm, and it stands at the true height
//! (V15 sat 3.7 mm high). The scan costs about half a microsecond more.

mod anchors;

pub use anchors::{ALPHA4, ALPHA16, AnchorSet, V15};
use duck_ipc_proto::JOINT_NAMES;
use kinematics::{Model, Pose, Quat, SiteId};

/// A candidate corner must sit below world Z = `-SWITCH_MARGIN` to bid for the
/// anchor. The anchor itself sits at Z = 0, so the margin is slack for FK and
/// IMU noise, not a physical depth.
const SWITCH_MARGIN: f64 = -0.010;

/// Ticks a candidate must stay the lowest point before the anchor moves. At
/// the control loop's 50 Hz this is 40 ms — well inside a stance phase, well
/// past a one-tick glitch.
const SWITCH_CONFIRM_TICKS: u32 = 2;

const LEFT: usize = 0;
const RIGHT: usize = 1;

/// The sole centre in the `*_foot` site frame (x forward): the frame footsteps are planned and tracked in
/// (`microduck_rl/src/mjlab_microduck/footsteps.py`, `SOLE_CENTER_IN_SITE`).
pub const SOLE_CENTRE_IN_SITE: [f64; 3] = [0.007, 0.0, 0.0];

pub struct Odometry {
    model: &'static Model,
    feet: [SiteId; 2],
    /// The candidate contact points on each sole.
    anchors: &'static AnchorSet,
    /// `JOINT_NAMES` position → model joint index. `None` for the mouth, which
    /// moves no leg.
    joint_map: [Option<usize>; JOINT_NAMES.len()],
    /// Scratch angle slice in model order, reused every tick.
    angles: Vec<f64>,
    /// Which foot the anchor is on.
    anchor_foot: usize,
    /// The contact point, in that foot's site frame.
    anchor_local: [f64; 3],
    /// The contact point's world X/Y. Its Z is 0 by definition: flat ground.
    anchor_xy: [f64; 2],
    position: [f64; 3],
    yaw: f64,
    /// A corner bidding to become the anchor: (foot, local point, world X/Y).
    pending: Option<(usize, [f64; 3], [f64; 2])>,
    pending_ticks: u32,
    /// True until the first update seeds `anchor_xy` from the actual startup
    /// pose, so the trunk starts at (0, 0) instead of offset by the foot.
    needs_init: bool,
    /// Planar sole-centre poses `[x, y, yaw]` of the left and right foot in the world frame.
    soles: [[f64; 3]; 2],
}

impl Odometry {
    pub fn new(model: &'static Model, anchors: &'static AnchorSet) -> Self {
        let feet = [
            model.site("left_foot").expect("model has a left_foot site"),
            model
                .site("right_foot")
                .expect("model has a right_foot site"),
        ];
        Self {
            feet,
            anchors,
            joint_map: JOINT_NAMES.map(|name| model.joint_index(name)),
            angles: vec![0.0; model.num_joints()],
            model,
            anchor_foot: LEFT,
            anchor_local: [0.0; 3],
            anchor_xy: [0.0; 2],
            position: [0.0; 3],
            yaw: 0.0,
            pending: None,
            pending_ticks: 0,
            needs_init: true,
            soles: [[0.0; 3]; 2],
        }
    }

    /// The alpha robot with the [`ALPHA16`] grid — the production estimator.
    pub fn alpha() -> Self {
        Self::new(Model::alpha(), &ALPHA16)
    }

    /// The alpha robot with a chosen anchor set.
    pub fn alpha_with(anchors: &'static AnchorSet) -> Self {
        Self::new(Model::alpha(), anchors)
    }

    /// The anchor set this estimator picks its contact point from.
    pub fn anchors(&self) -> &'static AnchorSet {
        self.anchors
    }

    /// Advance the estimate by one sensor sample.
    ///
    /// `joints` are measured angles in [`JOINT_NAMES`] order (the mouth is
    /// ignored); `quat_wxyz` is the IMU's trunk-in-world orientation. Call it
    /// on fresh samples only — a coasted tick has nothing new to say.
    pub fn update(&mut self, joints: &[f64; JOINT_NAMES.len()], quat_wxyz: [f64; 4]) {
        for (angle, idx) in joints.iter().zip(self.joint_map) {
            if let Some(idx) = idx {
                self.angles[idx] = *angle;
            }
        }
        let [w, x, y, z] = quat_wxyz;
        let rot = Quat::new(w, x, y, z).normalized();

        // Both foot poses, once — everything below reads these.
        let feet = [
            self.model.site_pose(self.feet[LEFT], &self.angles),
            self.model.site_pose(self.feet[RIGHT], &self.angles),
        ];

        if self.needs_init {
            let anchor_world = rot.rotate(feet[self.anchor_foot].pos);
            self.anchor_xy = [anchor_world[0], anchor_world[1]];
            self.needs_init = false;
        }

        self.reproject(rot, &feet);

        // A corner below the anchor is a step landing — but only after it holds
        // the claim for SWITCH_CONFIRM_TICKS, so FK jitter cannot walk the
        // anchor around mid-stance.
        match self.lowest_corner(rot, &feet) {
            None => {
                self.pending = None;
                self.pending_ticks = 0;
            }
            Some((foot, local, world_xy)) => {
                if self.pending.is_some_and(|(pf, _, _)| pf == foot) {
                    self.pending_ticks += 1;
                } else {
                    self.pending = Some((foot, local, world_xy));
                    self.pending_ticks = 1;
                }
                if self.pending_ticks >= SWITCH_CONFIRM_TICKS {
                    let (foot, local, world_xy) = self.pending.take().expect("just matched");
                    self.anchor_foot = foot;
                    self.anchor_local = local;
                    // The corner keeps the world X/Y it already has, so the
                    // estimate is continuous across the switch.
                    self.anchor_xy = world_xy;
                    self.reproject(rot, &feet);
                    self.pending_ticks = 0;
                }
            }
        }

        self.yaw = rot.yaw();
        for (sole, pose) in self.soles.iter_mut().zip(&feet) {
            let centre = rot.rotate(pose.transform_point(SOLE_CENTRE_IN_SITE));
            let yaw = (rot * pose.quat).yaw();
            *sole = [
                self.position[0] + centre[0],
                self.position[1] + centre[1],
                yaw,
            ];
        }
    }

    /// Trunk position in the world frame, metres. Z is height above the ground
    /// plane the anchor defines.
    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    /// Trunk heading, radians, in the IMU's boot-relative world frame.
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    /// The contact anchor's world X/Y — where the estimator believes the stance
    /// foot touches the ground. For telemetry; `position` is the answer.
    pub fn anchor_xy(&self) -> [f64; 2] {
        self.anchor_xy
    }

    /// Which foot carries the anchor: 0 = left, 1 = right.
    pub fn anchor_foot(&self) -> usize {
        self.anchor_foot
    }

    /// Planar pose `[x, y, yaw]` of each sole centre ([`SOLE_CENTRE_IN_SITE`]) in the world frame, left then
    /// right — what the footstep planner plans from and the footstep policy's targets are expressed against.
    pub fn sole_poses(&self) -> [[f64; 3]; 2] {
        self.soles
    }

    /// Trunk position from the anchor: the contact point sits at
    /// (`anchor_xy`, 0), the trunk is minus the world-rotated trunk→contact
    /// vector away from it.
    fn reproject(&mut self, rot: Quat, feet: &[Pose; 2]) {
        let contact_in_trunk = feet[self.anchor_foot].transform_point(self.anchor_local);
        let contact = rot.rotate(contact_in_trunk);
        self.position = [
            self.anchor_xy[0] - contact[0],
            self.anchor_xy[1] - contact[1],
            -contact[2],
        ];
    }

    /// The lowest sole corner below the switch threshold, if any, with its
    /// current world X/Y.
    fn lowest_corner(&self, rot: Quat, feet: &[Pose; 2]) -> Option<(usize, [f64; 3], [f64; 2])> {
        let mut lowest = -SWITCH_MARGIN;
        let mut best = None;
        for foot in [LEFT, RIGHT] {
            for &corner in self.anchors.foot(foot) {
                let in_world = rot.rotate(feet[foot].transform_point(corner));
                let world = [
                    self.position[0] + in_world[0],
                    self.position[1] + in_world[1],
                    self.position[2] + in_world[2],
                ];
                if world[2] < lowest {
                    lowest = world[2];
                    best = Some((foot, corner, [world[0], world[1]]));
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_quat(yaw: f64) -> [f64; 4] {
        [(yaw / 2.0).cos(), 0.0, 0.0, (yaw / 2.0).sin()]
    }

    fn roll_quat(roll: f64) -> [f64; 4] {
        [(roll / 2.0).cos(), (roll / 2.0).sin(), 0.0, 0.0]
    }

    /// The generated alpha sets must be sole points: inside the sole's
    /// footprint in the site frame and within its thickness, with the right
    /// foot the mirror image of the left.
    #[test]
    fn alpha_anchor_sets_lie_on_the_sole() {
        for set in [&ALPHA4, &ALPHA16] {
            assert_eq!(set.left.len(), set.right.len(), "{}", set.name);
            for p in set.left.iter().chain(set.right) {
                assert!((-0.021..=0.035).contains(&p[0]), "{} x {p:?}", set.name);
                assert!((-0.022..=0.022).contains(&p[1]), "{} y {p:?}", set.name);
                assert!((-0.002..=0.012).contains(&p[2]), "{} z {p:?}", set.name);
            }
            // Each foot is sampled on its own mesh, so the order differs; the
            // right sole is the left one mirrored in Y (to the sampling step).
            for l in set.left {
                let twin = set.right.iter().any(|r| {
                    (l[0] - r[0]).abs() < 3e-4
                        && (l[1] + r[1]).abs() < 3e-4
                        && (l[2] - r[2]).abs() < 3e-4
                });
                assert!(twin, "{}: no mirror of {l:?} on the right sole", set.name);
            }
        }
        assert_eq!(ALPHA4.left.len(), 4);
        assert_eq!(ALPHA16.left.len(), 16);
    }

    /// Reference trajectory for the Python replica in
    /// `microduck_rl/scripts/infer_policy.py` (`PyOdometry`): the same joint
    /// and IMU sequence fed to both must give the same positions.
    /// `cargo test -p odometry -- --ignored --nocapture dump_reference_trajectory`
    #[test]
    #[ignore = "reference dump for the Python replica, run by hand"]
    fn dump_reference_trajectory() {
        for set in [&V15, &ALPHA4, &ALPHA16] {
            let mut odo = Odometry::alpha_with(set);
            let mut joints = [0.0; JOINT_NAMES.len()];
            for i in 0..400 {
                let t = i as f64;
                joints[1] = 0.15 * (0.05 * t).sin(); // left_hip_roll
                joints[2] = 0.30 * (0.07 * t).sin(); // left_hip_pitch
                joints[3] = 0.20 * (0.07 * t).cos(); // left_knee
                joints[12] = 0.30 * (0.07 * t).cos(); // right_hip_pitch
                joints[13] = 0.20 * (0.07 * t).sin(); // right_knee
                odo.update(&joints, roll_quat(0.10 * (0.05 * t).sin()));
                let [x, y, z] = odo.position();
                println!(
                    "REF {} {i} {x:.9} {y:.9} {z:.9} {}",
                    set.name,
                    odo.anchor_foot()
                );
            }
        }
    }

    /// Cost of one `update` per anchor set, for sizing the set the robot
    /// runs. `cargo test -p odometry --release -- --ignored --nocapture`.
    #[test]
    #[ignore = "timing, run by hand"]
    fn update_cost_per_anchor_set() {
        let mut joints = [0.0; JOINT_NAMES.len()];
        for set in [&V15, &ALPHA4, &ALPHA16] {
            let mut odo = Odometry::alpha_with(set);
            let iters = 200_000;
            let start = std::time::Instant::now();
            for i in 0..iters {
                // Wiggle the legs so the FK and corner scan see real data.
                let t = i as f64 * 0.02;
                joints[0] = 0.3 * t.sin();
                joints[5] = 0.3 * t.cos();
                odo.update(&joints, roll_quat(0.05 * t.sin()));
            }
            let per = start.elapsed().as_nanos() as f64 / iters as f64;
            println!(
                "{:>8}: {} points/foot, {per:7.0} ns per update",
                set.name,
                set.left.len()
            );
        }
    }

    /// A robot standing still is at the origin and stays there — the first
    /// update seeds the anchor so the trunk starts at (0, 0), and constant
    /// inputs must not integrate into drift.
    /// The footstep follower plans from these: at the zero pose the soles sit side by side (left on +y) under
    /// the trunk, heading where the trunk heads; a yawed IMU turns them with it about the trunk. A wrong sign or
    /// frame here sends every footstep target sideways.
    #[test]
    fn sole_poses_follow_the_trunk_heading() {
        for yaw in [0.0, 0.5, -2.0] {
            let mut odo = Odometry::alpha();
            odo.update(&[0.0; JOINT_NAMES.len()], yaw_quat(yaw));
            let [l, r] = odo.sole_poses();
            let [px, py, _] = odo.position();
            for s in [l, r] {
                assert!(
                    (s[2] - yaw).abs() < 1e-9,
                    "sole yaw {} vs trunk {yaw}",
                    s[2]
                );
            }
            // Left minus right, in the trunk frame: all lateral, positive.
            let (dx, dy) = (l[0] - r[0], l[1] - r[1]);
            let (fwd, lat) = (
                dx * yaw.cos() + dy * yaw.sin(),
                -dx * yaw.sin() + dy * yaw.cos(),
            );
            assert!(
                fwd.abs() < 1e-6 && (0.05..0.15).contains(&lat),
                "fwd {fwd} lat {lat}"
            );
            // Midpoint below the trunk, offset forward by the sole-centre shift only (to FK precision).
            let (mx, my) = ((l[0] + r[0]) / 2.0 - px, (l[1] + r[1]) / 2.0 - py);
            assert!(
                (mx * mx + my * my).sqrt() < 0.03,
                "soles {mx}, {my} from the trunk"
            );
        }
    }

    #[test]
    fn standing_still_stays_at_the_origin() {
        let mut odo = Odometry::alpha();
        let joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..200 {
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        }
        let [x, y, z] = odo.position();
        assert!(x.abs() < 1e-9 && y.abs() < 1e-9, "drifted to ({x}, {y})");
        assert!(z > 0.02, "trunk should stand above the ground, z = {z}");
        assert_eq!(odo.yaw(), 0.0);
    }

    #[test]
    fn yaw_follows_the_imu() {
        let mut odo = Odometry::alpha();
        let joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..10 {
            odo.update(&joints, yaw_quat(0.3));
        }
        assert!((odo.yaw() - 0.3).abs() < 1e-9);
    }

    /// The mouth is a face, not a leg: index 9 must not reach the FK.
    #[test]
    fn the_mouth_moves_no_odometry() {
        let mut still = Odometry::alpha();
        let mut chatting = Odometry::alpha();
        let quiet = [0.0; JOINT_NAMES.len()];
        let mut open = quiet;
        open[9] = 42.0;
        for _ in 0..10 {
            still.update(&quiet, [1.0, 0.0, 0.0, 0.0]);
            chatting.update(&open, [1.0, 0.0, 0.0, 0.0]);
        }
        assert_eq!(still.position(), chatting.position());
    }

    /// A one-tick disturbance must not move the anchor; a held one must. This
    /// is the temporal confirmation doing its job against FK/IMU glitches.
    #[test]
    fn the_anchor_switches_only_after_the_claim_holds() {
        let mut odo = Odometry::alpha();
        let joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..10 {
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        }
        let settled_foot = odo.anchor_foot();

        // Roll the trunk so the OTHER foot is clearly lower — for one tick.
        // Positive roll about +x pushes -y (the right foot) down.
        let away = if settled_foot == LEFT { 0.35 } else { -0.35 };
        odo.update(&joints, roll_quat(away));
        odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            odo.anchor_foot(),
            settled_foot,
            "one glitchy tick stole the anchor"
        );

        // Held for several ticks, the other foot takes it.
        for _ in 0..5 {
            odo.update(&joints, roll_quat(away));
        }
        assert_ne!(odo.anchor_foot(), settled_foot, "a held claim must win");
    }

    /// Sweeping the stance hip moves the foot under the trunk, so the trunk
    /// must travel over the anchored foot — displacement with no teleporting:
    /// every tick's step stays small.
    #[test]
    fn stance_leg_motion_translates_the_trunk_continuously() {
        let mut odo = Odometry::alpha();
        let mut joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..10 {
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        }

        let hips = [
            JOINT_NAMES
                .iter()
                .position(|n| *n == "left_hip_pitch")
                .expect("exists"),
            JOINT_NAMES
                .iter()
                .position(|n| *n == "right_hip_pitch")
                .expect("exists"),
        ];
        let start = odo.position();
        let mut last = start;
        for i in 1..=50 {
            let angle = 0.2 * (i as f64 / 50.0);
            joints[hips[0]] = angle;
            joints[hips[1]] = angle;
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
            let now = odo.position();
            let step: f64 = (0..3)
                .map(|k| (now[k] - last[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(step < 0.02, "tick {i} jumped {step} m");
            last = now;
        }
        let moved = ((last[0] - start[0]).powi(2) + (last[1] - start[1]).powi(2)).sqrt();
        assert!(
            moved > 0.005,
            "a 0.2 rad hip sweep moved the trunk only {moved} m"
        );
        assert!(last.iter().all(|v| v.is_finite()));
    }
}
