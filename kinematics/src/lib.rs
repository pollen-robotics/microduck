//! MJCF-driven forward kinematics, compiled for the control loop.
//!
//! The prototype's `microduck_kinematics` crate proved the approach — parse the
//! training MJCF, walk the tree — but its query path was built for convenience:
//! joint angles travelled in a `HashMap<String, f64>`, and asking for one site
//! recomputed every body in the robot into a freshly allocated `Vec`. Fine at
//! bench cadence; wasteful inside a 50 Hz loop that asks for both feet every
//! tick.
//!
//! Here the model is *compiled* once at load: names are resolved to indices,
//! and each site gets its own flattened root→site chain of links. A query is
//! then a fold over that chain — no hashing, no allocation, no bodies the site
//! does not hang from. Angles are a plain `&[f64]` indexed by [`Model`]'s joint
//! order; resolve names to indices once with [`Model::joint_index`], not per
//! query.
//!
//! Correctness is pinned two ways: `tests/fk_against_mujoco.rs` compares every
//! site against MuJoCo's own `mj_kinematics` on 64 random poses, and
//! [`head`] pins the sign conventions the rest of the system steers by.

mod math;
mod mjcf;

pub mod hand;
pub mod head;
pub mod tof;

pub use math::{Pose, Quat};
pub use mjcf::ParseError;

use std::sync::LazyLock;

/// The alpha kinematic tree, stripped from the same `mjlab_microduck` scene the
/// walking policies are trained in. Updating the mechanics means replacing this
/// file and rerunning the fixture generator — no Rust changes.
const ALPHA_MJCF: &str = include_str!("../assets/alpha/robot_walk.xml");

/// A site, resolved. Cheap to copy, only meaningful with the model that
/// produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SiteId(usize);

/// One hop of a site's chain: the fixed transform into a body's frame, then —
/// if the body articulates — a rotation about its hinge.
#[derive(Clone, Copy)]
struct Link {
    rest: Pose,
    /// Joint index into the angle slice, and the unit hinge axis.
    joint: Option<(usize, [f64; 3])>,
}

/// A parsed and compiled kinematic model. Build one per MJCF and keep it — all
/// queries take `&self`.
pub struct Model {
    joint_names: Vec<String>,
    /// `[lo, hi]` travel limits per joint, radians, straight from the MJCF.
    joint_ranges: Vec<Option<(f64, f64)>>,
    site_names: Vec<String>,
    /// Per site, the flattened root→site chain (the site's own rest pose is the
    /// final, joint-less link).
    chains: Vec<Box<[Link]>>,
    /// The body tree, in MJCF order (a body's parent always precedes it), for
    /// whole-skeleton FK. Parallel arrays: name, parent index (`None` = root),
    /// rest pose in the parent frame, and the hinge joint to the parent.
    body_names: Vec<String>,
    body_parents: Vec<Option<usize>>,
    body_rest: Vec<Pose>,
    body_joint: Vec<Option<(usize, [f64; 3])>>,
    /// The trunk's standing height above the floor, metres — the scene's drop
    /// height for `trunk_base`, which is where the training world puts the
    /// floor relative to the trunk frame.
    trunk_height: f64,
}

impl Model {
    /// Compile a model from MJCF text.
    pub fn parse(xml: &str) -> Result<Self, ParseError> {
        let tree = mjcf::parse(xml)?;

        // Joint indices in tree order: deterministic, and a body's joint is
        // resolvable the moment its body is visited.
        let mut joint_names = Vec::new();
        let mut joint_ranges = Vec::new();
        let mut body_joint: Vec<Option<(usize, [f64; 3])>> = Vec::with_capacity(tree.bodies.len());
        let mut body_names = Vec::with_capacity(tree.bodies.len());
        let mut body_parents = Vec::with_capacity(tree.bodies.len());
        let mut body_rest = Vec::with_capacity(tree.bodies.len());
        for body in &tree.bodies {
            body_joint.push(body.joint.as_ref().map(|j| {
                joint_names.push(j.name.clone());
                joint_ranges.push(j.range);
                (joint_names.len() - 1, j.axis)
            }));
            body_names.push(body.name.clone());
            body_parents.push(body.parent);
            body_rest.push(body.rest);
        }

        let mut site_names = Vec::with_capacity(tree.sites.len());
        let mut chains = Vec::with_capacity(tree.sites.len());
        for site in &tree.sites {
            // Ancestors tip-to-root, then reversed — the root itself is
            // identity with no joint, so it contributes nothing and is skipped.
            let mut chain = Vec::new();
            let mut at = Some(site.body);
            while let Some(idx) = at {
                let body = &tree.bodies[idx];
                if body.parent.is_some() {
                    chain.push(Link {
                        rest: body.rest,
                        joint: body_joint[idx],
                    });
                }
                at = body.parent;
            }
            chain.reverse();
            chain.push(Link {
                rest: site.rest,
                joint: None,
            });

            site_names.push(site.name.clone());
            chains.push(chain.into_boxed_slice());
        }

        Ok(Self {
            joint_names,
            joint_ranges,
            site_names,
            chains,
            body_names,
            body_parents,
            body_rest,
            body_joint,
            trunk_height: tree.trunk_pos[2],
        })
    }

    /// The alpha robot's model, parsed once for the process. The embedded asset
    /// is covered by the MuJoCo fixture test, so this cannot fail at runtime
    /// without failing CI first.
    pub fn alpha() -> &'static Model {
        static ALPHA: LazyLock<Model> =
            LazyLock::new(|| Model::parse(ALPHA_MJCF).expect("embedded alpha MJCF parses"));
        &ALPHA
    }

    /// How long the angle slice for [`Model::site_pose`] must be.
    pub fn num_joints(&self) -> usize {
        self.joint_names.len()
    }

    /// Joint names in angle-slice order.
    pub fn joint_names(&self) -> impl Iterator<Item = &str> {
        self.joint_names.iter().map(String::as_str)
    }

    /// Where a named joint lives in the angle slice. Resolve once, at setup.
    pub fn joint_index(&self, name: &str) -> Option<usize> {
        self.joint_names.iter().position(|n| n == name)
    }

    /// The joint's `[lo, hi]` travel limits, radians — `None` when the MJCF
    /// declares none. What an IK must clamp against: the servos enforce these
    /// mechanically, so a target beyond them is a target the robot cannot hold.
    pub fn joint_range(&self, joint: usize) -> Option<(f64, f64)> {
        self.joint_ranges.get(joint).copied().flatten()
    }

    /// The trunk frame's standing height above the floor, metres, as the
    /// training scene declares it (`trunk_base`'s world drop height). A floor
    /// filter's offset, from the asset instead of a constant.
    pub fn trunk_height_m(&self) -> f64 {
        self.trunk_height
    }

    pub fn site_names(&self) -> impl Iterator<Item = &str> {
        self.site_names.iter().map(String::as_str)
    }

    /// Resolve a site by name. Resolve once, at setup.
    pub fn site(&self, name: &str) -> Option<SiteId> {
        self.site_names.iter().position(|n| n == name).map(SiteId)
    }

    /// The site's pose in the trunk frame, at the given joint angles.
    ///
    /// `angles` is indexed by [`Model::joint_index`] and must cover every
    /// joint — a short slice is a call-site bug, not a robot state, so it
    /// panics rather than silently reading zeros.
    pub fn site_pose(&self, site: SiteId, angles: &[f64]) -> Pose {
        assert_eq!(
            angles.len(),
            self.joint_names.len(),
            "angle slice must cover every joint"
        );
        let mut t = Pose::IDENTITY;
        for link in &self.chains[site.0] {
            t = t * link.rest;
            if let Some((idx, axis)) = link.joint {
                // A pure rotation in place: composing a full Pose would rotate
                // a zero translation for nothing.
                t.quat = t.quat * Quat::from_axis_angle(axis, angles[idx]);
            }
        }
        t
    }

    /// Every body's pose in the trunk frame, in [`Model::body_names`] order, at
    /// these angles — the whole skeleton, of which the sites are a few leaves.
    ///
    /// One pass in tree order (a parent always precedes its children, so its
    /// pose is already known): a body is its parent's pose, moved by its rest
    /// offset, then turned by its own hinge. `angles` covers every joint or it
    /// panics, the same contract as [`Model::site_pose`].
    pub fn body_poses(&self, angles: &[f64]) -> Vec<Pose> {
        assert_eq!(
            angles.len(),
            self.joint_names.len(),
            "angle slice must cover every joint"
        );
        let mut poses = vec![Pose::IDENTITY; self.body_names.len()];
        for i in 0..self.body_names.len() {
            let mut t = match self.body_parents[i] {
                Some(p) => poses[p] * self.body_rest[i],
                None => self.body_rest[i], // the root's world drop is stripped: it rests at identity
            };
            if let Some((idx, axis)) = self.body_joint[i] {
                t.quat = t.quat * Quat::from_axis_angle(axis, angles[idx]);
            }
            poses[i] = t;
        }
        poses
    }

    /// Body names in [`Model::body_poses`] order.
    pub fn body_names(&self) -> impl Iterator<Item = &str> {
        self.body_names.iter().map(String::as_str)
    }

    /// Each body's parent index in [`Model::body_poses`] order (`None` = root) —
    /// the skeleton's edges, for drawing it or matching visual meshes to links.
    pub fn body_parents(&self) -> &[Option<usize>] {
        &self.body_parents
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A three-body arm with sites at each level — enough structure to catch a
    /// chain built in the wrong order or rooted at the wrong body.
    const ARM: &str = r#"
        <mujoco>
          <worldbody>
            <body name="trunk_base" pos="9 9 9">
              <site name="origin"/>
              <body name="upper" pos="1 0 0">
                <joint name="shoulder" axis="0 0 1"/>
                <body name="lower" pos="1 0 0">
                  <joint name="elbow" axis="0 0 1"/>
                  <site name="tip" pos="1 0 0"/>
                </body>
              </body>
            </body>
          </worldbody>
        </mujoco>
    "#;

    #[test]
    fn a_planar_arm_folds_like_high_school_trig() {
        let model = Model::parse(ARM).expect("parses");
        assert_eq!(model.num_joints(), 2);
        let tip = model.site("tip").expect("tip exists");

        // Straight out: 1 (base offset) + 1 (lower offset) + 1 (site offset).
        let straight = model.site_pose(tip, &[0.0, 0.0]);
        assert!((straight.pos[0] - 3.0).abs() < 1e-12);

        // Elbow at 90°: the site offset swings to +y, x loses that reach.
        let bent = model.site_pose(tip, &[0.0, std::f64::consts::FRAC_PI_2]);
        assert!((bent.pos[0] - 2.0).abs() < 1e-12, "x: {}", bent.pos[0]);
        assert!((bent.pos[1] - 1.0).abs() < 1e-12, "y: {}", bent.pos[1]);
    }

    /// The root's world placement (`pos="9 9 9"`) must not leak into
    /// trunk-frame FK.
    #[test]
    fn the_trunk_frame_ignores_where_mujoco_drops_the_robot() {
        let model = Model::parse(ARM).expect("parses");
        let origin = model.site("origin").expect("origin exists");
        assert_eq!(model.site_pose(origin, &[0.0, 0.0]).pos, [0.0; 3]);
    }

    #[test]
    fn body_poses_place_the_whole_skeleton() {
        let model = Model::parse(ARM).expect("parses");
        let names: Vec<&str> = model.body_names().collect();
        assert_eq!(names, ["trunk_base", "upper", "lower"]);
        assert_eq!(model.body_parents(), &[None, Some(0), Some(1)]);

        // Straight: upper sits one out from the root, lower one further.
        let p = model.body_poses(&[0.0, 0.0]);
        assert!((p[1].pos[0] - 1.0).abs() < 1e-12 && p[1].pos[1].abs() < 1e-12);
        assert!((p[2].pos[0] - 2.0).abs() < 1e-12 && p[2].pos[1].abs() < 1e-12);

        // Shoulder at 90°: upper's origin does not move (a hinge turns the frame
        // in place), but lower swings out along +y.
        let p = model.body_poses(&[std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(
            (p[1].pos[0] - 1.0).abs() < 1e-12,
            "upper x: {}",
            p[1].pos[0]
        );
        assert!(
            (p[2].pos[0] - 1.0).abs() < 1e-12,
            "lower x: {}",
            p[2].pos[0]
        );
        assert!(
            (p[2].pos[1] - 1.0).abs() < 1e-12,
            "lower y: {}",
            p[2].pos[1]
        );

        // The body under a site agrees with the site FK, minus the site offset.
        let tip = model.site("tip").expect("tip");
        let s = model.site_pose(tip, &[0.3, -0.4]);
        let lower = model.body_poses(&[0.3, -0.4])[2];
        // The tip site sits [1,0,0] out in `lower`; walking it back lands on lower's origin.
        let back = lower
            * Pose {
                pos: [1.0, 0.0, 0.0],
                quat: Quat::IDENTITY,
            };
        assert!((s.pos[0] - back.pos[0]).abs() < 1e-12 && (s.pos[1] - back.pos[1]).abs() < 1e-12);
    }

    #[test]
    fn the_embedded_alpha_model_has_what_the_daemon_asks_for() {
        let model = Model::alpha();
        for site in ["left_foot", "right_foot", "head_camera", "tof"] {
            assert!(model.site(site).is_some(), "alpha MJCF lost site {site:?}");
        }
        for joint in ["left_hip_yaw", "right_ankle", "neck_pitch", "head_roll"] {
            assert!(
                model.joint_index(joint).is_some(),
                "alpha MJCF lost joint {joint:?}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "angle slice must cover every joint")]
    fn a_short_angle_slice_is_a_bug_not_a_zero() {
        let model = Model::parse(ARM).expect("parses");
        let tip = model.site("tip").expect("tip exists");
        model.site_pose(tip, &[0.0]);
    }

    #[test]
    fn a_model_without_a_trunk_is_refused_with_a_name() {
        let Err(err) = Model::parse("<mujoco><worldbody/></mujoco>") else {
            panic!("a trunk-less model must not parse");
        };
        assert!(err.to_string().contains("trunk_base"), "{err}");
    }
}
