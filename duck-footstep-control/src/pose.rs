//! Planar poses and the two feet.

use std::f64::consts::PI;

/// `[x, y, yaw]`: metres, metres, radians.
pub type Pose2 = [f64; 3];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Foot {
    Left,
    Right,
}

impl Foot {
    /// The odometry's numbering: 0 = left, 1 = right.
    pub fn from_index(index: usize) -> Self {
        if index == 0 { Foot::Left } else { Foot::Right }
    }

    pub fn index(self) -> usize {
        match self {
            Foot::Left => 0,
            Foot::Right => 1,
        }
    }

    pub fn other(self) -> Self {
        match self {
            Foot::Left => Foot::Right,
            Foot::Right => Foot::Left,
        }
    }

    /// Which side of the other foot this one stands on: +1 left, −1 right.
    pub fn side(self) -> f64 {
        match self {
            Foot::Left => 1.0,
            Foot::Right => -1.0,
        }
    }
}

/// Angle wrapped to [−π, π).
pub fn wrap(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// `a ∘ b`: the pose `b`, given in `a`'s frame, expressed in `a`'s parent frame.
pub fn compose(a: Pose2, b: Pose2) -> Pose2 {
    let (s, c) = a[2].sin_cos();
    [
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        wrap(a[2] + b[2]),
    ]
}

/// `a⁻¹ ∘ b`: the pose `b` expressed in `a`'s frame.
pub fn relative(a: Pose2, b: Pose2) -> Pose2 {
    let (s, c) = a[2].sin_cos();
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    [c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn relative_undoes_compose() {
        let a = [0.3, -0.2, 2.9];
        let b = [0.05, 0.095, 0.4];
        let back = relative(a, compose(a, b));
        for i in 0..3 {
            assert!((back[i] - b[i]).abs() < 1e-12, "{back:?} vs {b:?}");
        }
    }

    #[test]
    fn wrap_stays_in_range() {
        for a in [-7.0, -PI, 0.0, PI, 4.0, 13.0] {
            let w = wrap(a);
            assert!((-PI..PI).contains(&w), "{a} → {w}");
            let turns = (a - w) / (2.0 * PI);
            assert!((turns - turns.round()).abs() < 1e-9, "{a} → {w}");
        }
    }
}
