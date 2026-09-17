//! The little rigid-body algebra FK actually needs: a quaternion and a pose.
//!
//! Hand-rolled rather than pulled from nalgebra deliberately. The whole
//! requirement is "compose a dozen rigid transforms and rotate a few vectors" —
//! about a hundred lines — and the MuJoCo fixtures in `tests/` pin every one of
//! them to 1e-6, which is a stronger correctness argument than a dependency's
//! name. What nalgebra would add is compile time, not confidence.
//!
//! Conventions, shared with MJCF and the prototype runtime:
//!   - Quaternions are Hamilton, scalar first: `[w, x, y, z]`.
//!   - `a * b` applies `b` first, then `a` — the parent-times-child order, so a
//!     chain reads root-to-tip left to right.

use std::ops::Mul;

/// A rotation. Unit by construction everywhere this crate makes one; `new` is
/// for spelling out constants and trusts the caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// The same rotation at unit length. A degenerate (near-zero) input becomes
    /// the identity rather than NaN — it can only come from a hand-written XML
    /// attribute, and a model that ignores a broken quat is diagnosable where a
    /// model full of NaN is not.
    pub fn normalized(self) -> Self {
        let n = self
            .z
            .mul_add(
                self.z,
                self.y
                    .mul_add(self.y, self.x.mul_add(self.x, self.w * self.w)),
            )
            .sqrt();
        if n < 1e-12 {
            return Self::IDENTITY;
        }
        Self::new(self.w / n, self.x / n, self.y / n, self.z / n)
    }

    /// Rotation of `angle` radians about a **unit** axis.
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Self {
        let (s, c) = (0.5 * angle).sin_cos();
        Self::new(c, axis[0] * s, axis[1] * s, axis[2] * s)
    }

    /// Rotate a vector: `q v q⁻¹`, in the cross-product form that skips the
    /// full quaternion sandwich.
    pub fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        let [vx, vy, vz] = v;
        let tx = 2.0 * self.z.mul_add(-vy, self.y * vz);
        let ty = 2.0 * self.x.mul_add(-vz, self.z * vx);
        let tz = 2.0 * self.y.mul_add(-vx, self.x * vy);
        [
            self.z
                .mul_add(-ty, self.y.mul_add(tz, self.w.mul_add(tx, vx))),
            self.x
                .mul_add(-tz, self.z.mul_add(tx, self.w.mul_add(ty, vy))),
            self.y
                .mul_add(-tx, self.x.mul_add(ty, self.w.mul_add(tz, vz))),
        ]
    }

    /// The inverse rotation — for a unit quaternion, the conjugate is it.
    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// As `[w, x, y, z]`, the order MJCF and every wire format here use.
    pub fn wxyz(self) -> [f64; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Yaw about world +z, for an estimator that reports heading as one angle.
    pub fn yaw(self) -> f64 {
        // atan2 of the rotation matrix's (1,0) over (0,0) elements, expanded.
        let siny = 2.0 * self.x.mul_add(self.y, self.w * self.z);
        let cosy = 2.0f64.mul_add(-self.z.mul_add(self.z, self.y * self.y), 1.0);
        siny.atan2(cosy)
    }
}

impl Mul for Quat {
    type Output = Quat;

    fn mul(self, b: Quat) -> Quat {
        let a = self;
        Quat::new(
            a.z.mul_add(-b.z, a.y.mul_add(-b.y, a.x.mul_add(-b.x, a.w * b.w))),
            a.z.mul_add(-b.y, a.y.mul_add(b.z, a.x.mul_add(b.w, a.w * b.x))),
            a.z.mul_add(b.x, a.y.mul_add(b.w, a.x.mul_add(-b.z, a.w * b.y))),
            a.z.mul_add(b.w, a.y.mul_add(-b.x, a.x.mul_add(b.y, a.w * b.z))),
        )
    }
}

/// A rigid transform: rotate by `quat`, then translate by `pos`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub pos: [f64; 3],
    pub quat: Quat,
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        pos: [0.0; 3],
        quat: Quat::IDENTITY,
    };

    pub const fn new(pos: [f64; 3], quat: Quat) -> Self {
        Self { pos, quat }
    }

    /// A point in this pose's frame, expressed in the parent frame.
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        let r = self.quat.rotate(p);
        [self.pos[0] + r[0], self.pos[1] + r[1], self.pos[2] + r[2]]
    }
}

impl Mul for Pose {
    type Output = Pose;

    fn mul(self, b: Pose) -> Pose {
        Pose {
            pos: self.transform_point(b.pos),
            quat: self.quat * b.quat,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12)
    }

    /// A quarter turn about +z takes +x to +y — the one orientation everyone
    /// can check in their head, so a sign error in `mul`/`rotate` fails here
    /// before it fails against a fixture.
    #[test]
    fn quarter_turn_about_z_sends_x_to_y() {
        let q = Quat::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        assert!(close(q.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]));
        assert!((q.yaw() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);

        // Two quarter turns composed = one half turn: +x to -x.
        assert!(close((q * q).rotate([1.0, 0.0, 0.0]), [-1.0, 0.0, 0.0]));
    }

    /// `a * b` must mean "b first, then a": translate-then-rotate differs from
    /// rotate-then-translate, and this pins which one the chain fold does.
    #[test]
    fn pose_composition_applies_the_right_operand_first() {
        let turn = Pose::new(
            [0.0; 3],
            Quat::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2),
        );
        let step = Pose::new([1.0, 0.0, 0.0], Quat::IDENTITY);

        // Step in the child frame, seen from a turned parent: the step lands on +y.
        assert!(close((turn * step).pos, [0.0, 1.0, 0.0]));
        // Turn in the child frame of a stepped parent: the origin stays stepped.
        assert!(close((step * turn).pos, [1.0, 0.0, 0.0]));
    }

    #[test]
    fn a_broken_quat_normalizes_to_identity_not_nan() {
        let q = Quat::new(0.0, 0.0, 0.0, 0.0).normalized();
        assert_eq!(q, Quat::IDENTITY);
    }
}
