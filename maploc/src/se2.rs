//! Planar rigid transforms: the only geometry the map and the pose graph speak.
//!
//! The duck walks on a floor, so everything the map stores is `(x, y, yaw)`. The 3D part of the
//! problem — a pitched head on a swaying trunk — is resolved once per depth frame in
//! [`crate::project`], before anything reaches this module.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// `(x, y, yaw)` — metres and radians. As a transform it maps a point from the frame it describes
/// into its parent: `parent_p = pose.apply(child_p)`.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// `self ∘ other`: first `other`, then `self`.
    pub fn compose(self, other: Self) -> Self {
        let (s, c) = self.yaw.sin_cos();
        Self {
            x: self.x + c * other.x - s * other.y,
            y: self.y + s * other.x + c * other.y,
            yaw: wrap(self.yaw + other.yaw),
        }
    }

    pub fn inverse(self) -> Self {
        let (s, c) = self.yaw.sin_cos();
        Self {
            x: -(c * self.x + s * self.y),
            y: -(-s * self.x + c * self.y),
            yaw: wrap(-self.yaw),
        }
    }

    /// The pose of `to` expressed in `self`'s frame: `self⁻¹ ∘ to`.
    pub fn between(self, to: Self) -> Self {
        self.inverse().compose(to)
    }

    pub fn apply(self, p: [f64; 2]) -> [f64; 2] {
        let (s, c) = self.yaw.sin_cos();
        [self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1]]
    }

    pub fn dist(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
}

/// An angle folded into `(-π, π]`.
pub fn wrap(a: f64) -> f64 {
    let r = (a + PI).rem_euclid(2.0 * PI) - PI;
    if r <= -PI { r + 2.0 * PI } else { r }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Pose2, b: Pose2) -> bool {
        (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12 && wrap(a.yaw - b.yaw).abs() < 1e-12
    }

    #[test]
    fn compose_inverse_between_agree() {
        let a = Pose2::new(1.0, -2.0, 2.9);
        let b = Pose2::new(-0.3, 0.7, -2.8);
        assert!(close(a.compose(a.inverse()), Pose2::IDENTITY));
        assert!(close(a.compose(a.between(b)), b));
        let p = [0.4, -1.1];
        let q = a.compose(b).apply(p);
        let r = a.apply(b.apply(p));
        assert!((q[0] - r[0]).abs() < 1e-12 && (q[1] - r[1]).abs() < 1e-12);
    }

    #[test]
    fn wrap_stays_in_the_half_open_interval() {
        // The pose graph differences yaws every iteration; a residual of 2π must read as zero,
        // or a node that turned a full circle drags the whole graph round with it.
        assert!(wrap(2.0 * PI).abs() < 1e-12);
        assert!((wrap(PI) - PI).abs() < 1e-12);
        assert!((wrap(-PI) - PI).abs() < 1e-12);
        assert!((wrap(3.0 * PI + 0.1) - (-PI + 0.1)).abs() < 1e-9);
    }
}
