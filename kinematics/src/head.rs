//! Head-chain FK in the frames the vision stack actually speaks.
//!
//! The model hands out MJCF site poses; the consumers — camera reprojection,
//! ToF point clouds — want cv2 camera axes (+x right, +y down, +z forward) or
//! the ToF sensor's own axes (+x forward along the optical axis, +y left,
//! +z up). The two constant quaternions here are that translation layer,
//! carried over from the prototype runtime where they were tuned against real
//! images; the sign-pin test at the bottom is what keeps an MJCF update from
//! silently making the robot nod the wrong way.

use crate::{Model, Pose, Quat, SiteId};

/// MJCF `head_camera` site → cv2 camera frame. Constant across robot versions:
/// the per-version site quats differ only by an absolute frame choice, and
/// `q_site⁻¹ * q_cv2` comes out the same.
pub const SITE_TO_CV2: Quat = Quat::new(0.5, -0.5, 0.5, -0.5);

/// Sensor frame (+x forward, +y left, +z up — the VL53L5CX/L8CX integration
/// convention) → cv2 camera frame.
pub const SENSOR_IN_CV2_Q: Quat = Quat::new(0.5, 0.5, -0.5, 0.5);

/// The head joints, in the order [`HeadFk`] takes its angles.
const HEAD_JOINTS: [&str; 4] = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"];

/// The head chain with its names resolved once, so per-frame FK does no string
/// work at all.
pub struct HeadFk {
    model: &'static Model,
    camera: SiteId,
    /// The MJCF's own `tof` site, when the asset carries one — the sensor's
    /// true position, a couple of centimetres from the camera it would
    /// otherwise borrow.
    tof: Option<SiteId>,
    joints: [usize; 4],
}

impl HeadFk {
    /// Panics if the model lacks the head chain — that is a broken asset, and
    /// the embedded one is covered by tests.
    pub fn new(model: &'static Model) -> Self {
        Self {
            model,
            camera: model
                .site("head_camera")
                .expect("model has a head_camera site"),
            tof: model.site("tof"),
            joints: HEAD_JOINTS
                .map(|name| model.joint_index(name).expect("model has the head joints")),
        }
    }

    pub fn alpha() -> Self {
        Self::new(Model::alpha())
    }

    /// Camera pose in the trunk frame, cv2 axes.
    ///
    /// `joints` = `[neck_pitch, head_pitch, head_yaw, head_roll]`, radians.
    /// Every other joint is taken at zero — the head chain hangs from the
    /// trunk, so the legs cannot move it *within the trunk frame*.
    pub fn camera_in_trunk_cv2(&self, joints: [f64; 4]) -> Pose {
        let site = self.site_in_trunk(self.camera, joints);
        Pose::new(site.pos, site.quat * SITE_TO_CV2)
    }

    /// ToF sensor pose in the trunk frame: rotating a sensor-frame vector
    /// (forward, left, up) by the result's quat expresses it in the trunk.
    ///
    /// Anchored on the MJCF's own `tof` site when the asset has one — the
    /// true mount, a couple of centimetres from the camera. The prototype
    /// borrowed the camera's position because its assets predated the site;
    /// that stays as the fallback, and the orientation convention is the same
    /// either way (the pin test below holds the two mounts parallel).
    pub fn tof_in_trunk(&self, joints: [f64; 4]) -> Pose {
        match self.tof {
            Some(site) => {
                let site = self.site_in_trunk(site, joints);
                Pose::new(site.pos, site.quat * SITE_TO_CV2 * SENSOR_IN_CV2_Q)
            }
            None => {
                let cam = self.camera_in_trunk_cv2(joints);
                Pose::new(cam.pos, cam.quat * SENSOR_IN_CV2_Q)
            }
        }
    }

    /// A head-chain site posed by the four head joints, everything else at
    /// zero.
    fn site_in_trunk(&self, site: SiteId, joints: [f64; 4]) -> Pose {
        // The alpha model has 14 joints; 32 leaves room for any future duck
        // without ever touching the allocator.
        let mut angles = [0.0f64; 32];
        assert!(self.model.num_joints() <= angles.len());
        for (idx, angle) in self.joints.into_iter().zip(joints) {
            angles[idx] = angle;
        }
        self.model
            .site_pose(site, &angles[..self.model.num_joints()])
    }

    /// Gaze IK: the head joints that point the camera at a trunk-frame point.
    ///
    /// Two of the four joints do the aiming — `head_pitch` and `head_yaw` —
    /// while `neck_pitch` is posture the caller chooses and `head_roll` stays
    /// level. The two are genuinely coupled: `head_pitch` sits *upstream* of
    /// `head_yaw` in the chain, so pitching tilts the plane yaw pans in, and
    /// near ±90° of yaw the pitch joint loses elevation authority entirely (its
    /// axis aligns with the camera's forward). A per-axis update stalls there,
    /// so this solves the 2×2 system properly: damped Gauss-Newton against the
    /// real FK, Jacobian by finite differences — FK is ~50 ns, so the whole
    /// solve is a couple of microseconds.
    ///
    /// Joints are clamped to the MJCF's travel limits every step — the servos
    /// enforce those mechanically, so an unclamped answer would be a pose the
    /// robot cannot hold. `clamped` reports that the *result* still misses the
    /// target, whether the miss is a travel limit or the gimbal geometry: it is
    /// the caller's "the robot is looking as close as it can".
    pub fn look_at(&self, target_in_trunk: [f64; 3], neck_pitch: f64) -> Gaze {
        // Tighter than any servo tracks, loose enough to converge quickly.
        const TOLERANCE: f64 = 1e-4;
        const STEP_H: f64 = 1e-5;
        /// Levenberg damping: keeps the step finite through the yaw-90°
        /// singularity, costs one extra iteration elsewhere.
        const LAMBDA: f64 = 1e-3;
        const MAX_STEP: f64 = 0.7;

        let range = |i: usize| {
            self.model
                .joint_range(self.joints[i])
                .unwrap_or((-std::f64::consts::PI, std::f64::consts::PI))
        };
        let clamp = |v: f64, (lo, hi): (f64, f64)| v.clamp(lo, hi);

        // The pointing error (yaw, pitch) of the current pose, in camera axes.
        let error = |joints: [f64; 4]| -> [f64; 2] {
            let cam = self.camera_in_trunk_cv2(joints);
            let v = [
                target_in_trunk[0] - cam.pos[0],
                target_in_trunk[1] - cam.pos[1],
                target_in_trunk[2] - cam.pos[2],
            ];
            // cv2 camera axes: +x right, +y down, +z forward.
            let v = cam.quat.conjugate().rotate(v);
            let flat = v[0].hypot(v[2]);
            [v[0].atan2(v[2]), v[1].atan2(flat)]
        };

        let mut joints = [clamp(neck_pitch, range(0)), 0.0, 0.0, 0.0];
        let mut residual = f64::INFINITY;
        for _ in 0..30 {
            let e = error(joints);
            residual = e[0].abs().max(e[1].abs());
            if residual < TOLERANCE {
                break;
            }

            // Jacobian de/d(head_pitch, head_yaw) by forward differences.
            let mut j = [[0.0f64; 2]; 2];
            for (col, joint) in [1usize, 2].into_iter().enumerate() {
                let mut probe = joints;
                probe[joint] += STEP_H;
                let ep = error(probe);
                j[0][col] = (ep[0] - e[0]) / STEP_H;
                j[1][col] = (ep[1] - e[1]) / STEP_H;
            }

            // Solve (JᵀJ + λI) Δ = -Jᵀe, the 2×2 case written out.
            let (a, b) = (
                j[1][0].mul_add(j[1][0], j[0][0] * j[0][0]) + LAMBDA,
                j[1][0].mul_add(j[1][1], j[0][0] * j[0][1]),
            );
            let d = j[1][1].mul_add(j[1][1], j[0][1] * j[0][1]) + LAMBDA;
            let g = [
                j[1][0].mul_add(e[1], j[0][0] * e[0]),
                j[1][1].mul_add(e[1], j[0][1] * e[0]),
            ];
            let det = b.mul_add(-b, a * d);
            if det.abs() < 1e-12 {
                break; // fully singular and damped-out: nothing left to gain
            }
            let step = [
                b.mul_add(g[1], -d * g[0]) / det,
                a.mul_add(-g[1], b * g[0]) / det,
            ];
            let scale = (MAX_STEP / step[0].hypot(step[1])).min(1.0);
            joints[1] = clamp(joints[1] + scale * step[0], range(1));
            joints[2] = clamp(joints[2] + scale * step[1], range(2));
        }
        Gaze {
            joints,
            clamped: residual >= TOLERANCE,
        }
    }
}

/// What [`HeadFk::look_at`] chose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gaze {
    /// `[neck_pitch, head_pitch, head_yaw, head_roll]`, radians — ready for
    /// the `robot.head` intent.
    pub joints: [f64; 4],
    /// The target lies beyond the head's travel; `joints` is the closest gaze
    /// the limits allow, not a lock on the target.
    pub clamped: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poses_are_finite_and_unit() {
        let fk = HeadFk::alpha();
        let pose = fk.camera_in_trunk_cv2([0.0; 4]);
        assert!(pose.pos.iter().all(|v| v.is_finite()));
        let n: f64 = pose.quat.wxyz().iter().map(|v| v * v).sum();
        assert!((n - 1.0).abs() < 1e-9, "quaternion not unit: {n}");
    }

    /// Tilting head_pitch must move the camera — guards against a chain that
    /// silently lost a joint name in an MJCF update.
    #[test]
    fn head_pitch_moves_the_camera() {
        let fk = HeadFk::alpha();
        let level = fk.camera_in_trunk_cv2([0.0; 4]).pos;
        let tilted = fk.camera_in_trunk_cv2([0.0, 0.5, 0.0, 0.0]).pos;
        let moved = (level[0] - tilted[0])
            .abs()
            .max((level[2] - tilted[2]).abs());
        assert!(
            moved > 1e-3,
            "head_pitch had no effect: {level:?} vs {tilted:?}"
        );
    }

    /// Pin the alpha head sign conventions (camera-forward response to a
    /// positive joint angle, trunk frame: +y left, +z up). The gaze and
    /// laser-tracking sign flips are chosen against these — if an MJCF update
    /// changes them, this fails instead of the robot nodding the wrong way.
    #[test]
    fn alpha_head_axis_conventions() {
        let fk = HeadFk::alpha();
        let forward = |joints: [f64; 4]| {
            // cv2 +z is the optical axis.
            fk.camera_in_trunk_cv2(joints).quat.rotate([0.0, 0.0, 1.0])
        };
        // +head_yaw looks left (+y) on alpha.
        assert!(forward([0.0, 0.0, 0.3, 0.0])[1] > 0.1);
        // +head_pitch looks DOWN (-z) on alpha.
        assert!(forward([0.0, 0.3, 0.0, 0.0])[2] < -0.1);
    }

    /// The IK's contract, checked by its own FK: after `look_at`, the camera's
    /// optical axis passes through the target — for targets all around the
    /// reachable envelope, converged and unclamped.
    #[test]
    fn look_at_actually_looks_at_the_target() {
        let fk = HeadFk::alpha();
        let targets = [
            [1.0, 0.0, 0.0],   // dead ahead
            [0.5, 0.5, 0.1],   // up-left
            [0.4, -0.6, -0.2], // down-right
            [0.2, 0.6, 0.05],  // hard left
            [0.3, 0.0, -0.4],  // steeply down
        ];
        for target in targets {
            let gaze = fk.look_at(target, 0.0);
            assert!(
                !gaze.clamped,
                "reachable target {target:?} reported clamped"
            );

            let cam = fk.camera_in_trunk_cv2(gaze.joints);
            let v = [
                target[0] - cam.pos[0],
                target[1] - cam.pos[1],
                target[2] - cam.pos[2],
            ];
            let forward = cam.quat.rotate([0.0, 0.0, 1.0]);
            let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            let along = (v[0] * forward[0] + v[1] * forward[1] + v[2] * forward[2]) / norm;
            assert!(
                along > 1.0 - 1e-6,
                "camera misses {target:?}: forward {forward:?}, to-target {v:?}"
            );
        }
    }

    /// The posture parameter is held, not solved: the aim must come from the
    /// head joints around whatever neck the caller asked for.
    #[test]
    fn look_at_keeps_the_neck_it_was_given() {
        let fk = HeadFk::alpha();
        let gaze = fk.look_at([0.8, 0.2, -0.1], 0.3);
        assert!((gaze.joints[0] - 0.3).abs() < 1e-12);
        assert_eq!(gaze.joints[3], 0.0, "head_roll stays level");
        assert!(!gaze.clamped);
    }

    /// Straight behind needs a 180° yaw; the MJCF allows ±170°. The answer is
    /// the closest gaze the limits allow, said out loud via `clamped`.
    #[test]
    fn a_target_behind_the_robot_clamps_at_the_yaw_limit() {
        let fk = HeadFk::alpha();
        let gaze = fk.look_at([-1.0, 0.0, 0.0], 0.0);
        assert!(gaze.clamped, "a 180° target must not claim a lock");
        let yaw_limit = Model::alpha()
            .joint_range(Model::alpha().joint_index("head_yaw").expect("exists"))
            .expect("head_yaw has a range")
            .1;
        assert!(
            (gaze.joints[2].abs() - yaw_limit).abs() < 1e-9,
            "the head should be pinned at its yaw limit: {} vs {yaw_limit}",
            gaze.joints[2]
        );
    }

    /// The sensor pose sits on the MJCF's own `tof` site — exactly — and its
    /// orientation convention matches the camera-borrowed path the prototype
    /// used, which is only true while the asset mounts the two parallel. If a
    /// future MJCF tilts the ToF, the orientation half of this is what fails,
    /// and the conventions need a fresh look rather than a looser tolerance.
    #[test]
    fn the_tof_pose_sits_on_the_mjcf_tof_site() {
        let model = Model::alpha();
        let fk = HeadFk::alpha();
        let joints = [0.1, 0.2, -0.1, 0.05];
        let pose = fk.tof_in_trunk(joints);

        let mut angles = vec![0.0; model.num_joints()];
        for (name, angle) in [
            ("neck_pitch", 0.1),
            ("head_pitch", 0.2),
            ("head_yaw", -0.1),
            ("head_roll", 0.05),
        ] {
            angles[model.joint_index(name).expect("joint exists")] = angle;
        }
        let site = model.site_pose(model.site("tof").expect("tof site"), &angles);
        for (a, b) in pose.pos.iter().zip(site.pos) {
            assert!((a - b).abs() < 1e-12, "{:?} vs {:?}", pose.pos, site.pos);
        }

        // Same forward axis as the camera-borrowed convention.
        let cam = fk.camera_in_trunk_cv2(joints);
        let borrowed = (cam.quat * SENSOR_IN_CV2_Q).rotate([1.0, 0.0, 0.0]);
        let anchored = pose.quat.rotate([1.0, 0.0, 0.0]);
        for (a, b) in anchored.iter().zip(borrowed) {
            assert!(
                (a - b).abs() < 1e-9,
                "the tof site no longer parallels the camera: {anchored:?} vs {borrowed:?}"
            );
        }
    }
}
