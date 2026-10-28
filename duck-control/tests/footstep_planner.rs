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
