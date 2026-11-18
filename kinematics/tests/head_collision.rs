//! Issue #480: the head must be a measurable solid and the beak must hinge.
//!
//! The MJCF parser drops geoms (FK needs none), so collision geometry can
//! silently vanish from the asset — these assertions read the raw XML for the
//! geoms and the parsed model for the beak joint.

use kinematics::Model;

const MJCF: &str = include_str!("../assets/alpha/robot_walk.xml");

/// The whole head subtree: from the head shell body to the next sibling body.
fn head_subtree() -> &'static str {
    let start = MJCF
        .find("<body name=\"bottom_head_shell\"")
        .expect("asset has bottom_head_shell");
    let end = MJCF[start..]
        .find("<body name=\"bearing_roll\"")
        .map(|i| start + i)
        .expect("asset has bearing_roll after the head");
    &MJCF[start..end]
}

#[test]
fn head_shell_carries_collision_geometry() {
    let geoms = head_subtree().matches("<geom").count();
    assert!(geoms >= 2, "head subtree has {geoms} geoms, want shell + beak");
}

#[test]
fn beak_has_a_hinge_joint_and_moves() {
    let model = Model::alpha();

    let jaw = model
        .joint_index("jaw")
        .expect("model has a jaw joint for the lower beak");
    assert!(model.joint_range(jaw).is_some(), "jaw joint has a range");

    let names: Vec<&str> = model.body_names().collect();
    assert!(
        names.contains(&"lower_beak"),
        "model has a lower_beak body, bodies = {names:?}"
    );

    let tip = model.site("lower_beak_tip").expect("model has lower_beak_tip");

    // A joint count worth of zeros, then half-open vs closed.
    let closed = vec![0.0; model.num_joints()];
    let mut open = closed.clone();
    open[jaw] = 0.35;

    let a = model.site_pose(tip, &closed);
    let b = model.site_pose(tip, &open);
    let d = (a.pos[0] - b.pos[0]).abs()
        + (a.pos[1] - b.pos[1]).abs()
        + (a.pos[2] - b.pos[2]).abs();
    assert!(d > 1e-4, "lower_beak_tip does not move with the jaw (Δ={d})");
}
