//! Requires ONNX Runtime >= 1.23: cargo test -p duck-control --test footstep_planner -- --ignored
//!
//! The planner as the robot runs it — `load_planner` on the published FootstepNet
//! (`fixtures/footstepnet.onnx`, Hub `apirrone/microduck-footstepnet`) — against the Python reference's rollouts
//! of the same file (`duck-footstep-control/tests/fixtures/planner_parity.json`). The pure-logic parity test pins
//! the glue with a replaying stub; this one adds the session: input/output names, metadata options, f32 I/O.
use duck_control::footsteps::load_planner;
use duck_footstep_control::{Foot, Obstacle};
use serde_json::Value;
use std::path::PathBuf;

fn pose(v: &Value) -> [f64; 3] {
    let a: Vec<f64> = serde_json::from_value(v.clone()).unwrap();
    [a[0], a[1], a[2]]
}

fn foot(v: &Value) -> Foot {
    if v.as_str() == Some("left") {
        Foot::Left
    } else {
        Foot::Right
    }
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn onnx_planner_reproduces_the_python_rollouts() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut planner =
        load_planner(&dir.join("tests/fixtures/footstepnet.onnx")).expect("planner loads");
    let fixture: Value = serde_json::from_str(
        &std::fs::read_to_string(
            dir.join("../duck-footstep-control/tests/fixtures/planner_parity.json"),
        )
        .unwrap(),
    )
    .unwrap();
    assert!(
        (planner.options().max_dx_forward - fixture["options"]["max_dx_forward"].as_f64().unwrap())
            .abs()
            < 1e-12
    );
    for (i, case) in fixture["cases"].as_array().unwrap().iter().enumerate() {
        let obstacle = case["obstacle"].as_array().map(|o| Obstacle {
            x: o[0].as_f64().unwrap(),
            y: o[1].as_f64().unwrap(),
            radius: o[2].as_f64().unwrap(),
        });
        let plan = planner
            .plan(
                pose(&case["support"]),
                foot(&case["support_foot"]),
                pose(&case["goal"]),
                foot(&case["goal_foot"]),
                obstacle,
                50,
            )
            .unwrap();
        let want = case["steps"].as_array().unwrap();
        assert_eq!(plan.steps.len(), want.len(), "case {i}");
        assert_eq!(plan.reached, case["reached"].as_bool().unwrap(), "case {i}");
        for (k, ((got, _), w)) in plan.steps.iter().zip(want).enumerate() {
            let w = pose(&w["landing"]);
            for d in 0..3 {
                // Same graph, same f32 math, different runtime build: rounding only, compounded over a rollout.
                assert!(
                    (got[d] - w[d]).abs() < 1e-4,
                    "case {i} step {k}: {got:?} vs {w:?}"
                );
            }
        }
    }
}

/// `robotctl robot goto`: a walker that lands every target exactly reaches goals in the robot's frame — by a
/// named foot, or by the stance centre — within FootstepNet's 2 cm / 5° tolerance, squares up and stops. The
/// frame is the start stance's, so a goal straight ahead of a robot facing +y is up the y axis.
#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn goto_reaches_goals_in_the_robot_frame() {
    use duck_footstep_control::{Follower, GoalOutcome, GoalRun, compose, relative};
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut planner = load_planner(&dir.join("tests/fixtures/footstepnet.onnx")).unwrap();
    let start = [0.3, -0.2, std::f64::consts::FRAC_PI_2];
    let half = 0.042;
    for (goal, foot) in [
        ([0.3, 0.0, 0.0], None),
        ([0.2, 0.1, 0.8], Some(Foot::Left)),
        ([-0.1, -0.05, -1.2], Some(Foot::Right)),
        ([0.0, 0.0, std::f64::consts::PI], None),
    ] {
        let mut soles = [
            compose(start, [0.0, half, 0.0]),
            compose(start, [0.0, -half, 0.0]),
        ];
        let mut run = GoalRun::new(goal, foot);
        let mut follower = Follower::new();
        let mut support = Foot::Right;
        let mut steps = 0;
        while let Some(command) = run
            .tick(&mut follower, &mut planner, support, soles)
            .unwrap()
        {
            let swing = support.other();
            soles[swing.index()] = compose(soles[support.index()], command);
            support = swing;
            steps += 1;
            assert!(steps < 100, "goal {goal:?} never ended");
        }
        let (world_goal, goal_foot) = run.world_goal().unwrap();
        let expected = match foot {
            Some(_) => compose(start, goal),
            None => compose(compose(start, goal), [0.0, 0.095 / 2.0, 0.0]),
        };
        let off = relative(expected, world_goal);
        assert!(
            off.iter().all(|v| v.abs() < 1e-9),
            "goal anchored at {world_goal:?}, not {expected:?}"
        );
        match run.outcome() {
            Some(GoalOutcome::Reached {
                distance, heading, ..
            }) => {
                assert!(
                    distance < 0.02 && heading < 0.0873,
                    "goal {goal:?}: missed by {distance} m, {heading} rad"
                );
            }
            other => panic!("goal {goal:?}: {other:?}"),
        }
        // Squared up: the other foot ends beside the goal foot.
        let other = relative(soles[goal_foot.index()], soles[goal_foot.other().index()]);
        assert!(
            other[0].abs() < 0.02 && (other[1].abs() - 0.095).abs() < 0.025,
            "not squared: {other:?}"
        );
    }
}
