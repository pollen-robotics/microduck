//! SE(2) pose graph over submap anchor poses.
//!
//! Nodes hold a 2D pose `(x, y, yaw)` and a back-pointer to the submap
//! they describe. Edges are relative-pose constraints (`T_from^-1 * T_to`)
//! with a 3×3 information matrix.

use crate::submap::Pose2;

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct PoseNode {
    pub pose: Pose2,
    /// Index of the submap this node corresponds to in the submap list.
    pub submap_idx: usize,
    /// True iff this node is fixed during optimization (typically the
    /// first node, anchoring the graph in world frame).
    pub fixed: bool,
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct PoseEdge {
    pub from: usize,
    pub to: usize,
    /// Measured relative pose from `from` to `to`: `T_from^-1 ⊕ T_to`.
    pub measurement: Pose2,
    /// 3×3 information matrix. Larger entries = tighter constraint.
    pub information: [[f32; 3]; 3],
}

#[derive(Default, serde::Serialize, serde::Deserialize)]
pub struct PoseGraph {
    nodes: Vec<PoseNode>,
    edges: Vec<PoseEdge>,
}

impl PoseGraph {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a node with the given world pose. Returns its index.
    pub fn add_node(&mut self, pose: Pose2, submap_idx: usize) -> usize {
        let idx = self.nodes.len();
        self.nodes.push(PoseNode {
            pose,
            submap_idx,
            fixed: idx == 0,
        });
        idx
    }

    pub fn add_edge(&mut self, edge: PoseEdge) {
        self.edges.push(edge);
    }

    pub fn nodes(&self) -> &[PoseNode] {
        &self.nodes
    }
    pub fn edges(&self) -> &[PoseEdge] {
        &self.edges
    }
    pub fn edges_mut(&mut self) -> &mut [PoseEdge] {
        &mut self.edges
    }
    pub fn nodes_mut(&mut self) -> &mut [PoseNode] {
        &mut self.nodes
    }

    pub fn fix_node(&mut self, idx: usize) {
        if let Some(n) = self.nodes.get_mut(idx) {
            n.fixed = true;
        }
    }
}

/// Compose: world-frame pose of point expressed in body frame of `a`,
/// such that `composed.pose = a ⊕ b_local`. Used to convert relative
/// edges into world poses (e.g. when initializing a node from a parent
/// + a measurement).
pub fn compose(a: Pose2, b_local: Pose2) -> Pose2 {
    let (ax, ay, ayaw) = a;
    let (bx, by, byaw) = b_local;
    let ca = ayaw.cos();
    let sa = ayaw.sin();
    let x = ax + ca * bx - sa * by;
    let y = ay + sa * bx + ca * by;
    let yaw = wrap_pi(ayaw + byaw);
    (x, y, yaw)
}

/// SE(2) inverse: if `c = compose(a, b)`, then `a = compose(c, inverse(b))`.
pub fn inverse(p: Pose2) -> Pose2 {
    let (x, y, yaw) = p;
    let c = yaw.cos();
    let s = yaw.sin();
    (-(c * x + s * y), s * x - c * y, -yaw)
}

/// Express `b` in the local frame of `a`: `T_a^-1 ⊕ T_b`. Used to
/// produce the measurement for an odometry-style edge from two anchor
/// poses, or to evaluate the "predicted" measurement during optimization.
pub fn between(a: Pose2, b: Pose2) -> Pose2 {
    let (ax, ay, ayaw) = a;
    let (bx, by, byaw) = b;
    let dx = bx - ax;
    let dy = by - ay;
    let ca = ayaw.cos();
    let sa = ayaw.sin();
    (ca * dx + sa * dy, -sa * dx + ca * dy, wrap_pi(byaw - ayaw))
}

#[inline]
pub fn wrap_pi(a: f32) -> f32 {
    use std::f32::consts::PI;
    let two_pi = 2.0 * PI;
    let mut y = (a + PI).rem_euclid(two_pi) - PI;
    if y == PI {
        y = -PI;
    }
    y
}

/// Diagonal information matrix from per-axis sigmas. Lower sigmas =
/// tighter constraint = larger information.
pub fn information_from_sigmas(sigma_xy: f32, sigma_yaw: f32) -> [[f32; 3]; 3] {
    let inv_xy2 = 1.0 / (sigma_xy * sigma_xy);
    let inv_yaw2 = 1.0 / (sigma_yaw * sigma_yaw);
    [
        [inv_xy2, 0.0, 0.0],
        [0.0, inv_xy2, 0.0],
        [0.0, 0.0, inv_yaw2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn between_then_compose_is_identity() {
        let a = (1.0, -0.5, 0.6);
        let b = (2.5, 0.3, 0.2);
        let rel = between(a, b);
        let recovered = compose(a, rel);
        assert!((recovered.0 - b.0).abs() < 1e-5);
        assert!((recovered.1 - b.1).abs() < 1e-5);
        assert!((recovered.2 - b.2).abs() < 1e-5);
    }

    #[test]
    fn first_added_node_is_fixed_by_default() {
        let mut g = PoseGraph::new();
        let i0 = g.add_node((0.0, 0.0, 0.0), 0);
        let i1 = g.add_node((1.0, 0.0, 0.0), 1);
        assert_eq!(i0, 0);
        assert_eq!(i1, 1);
        assert!(g.nodes()[0].fixed);
        assert!(!g.nodes()[1].fixed);
    }
}
