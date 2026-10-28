//! The Rust planner loop against the Python reference, step by step.
//!
//! `fixtures/planner_parity.json` holds the reference planner's rollouts (microduck_rl `footstep_planner.py`,
//! itself checked against FootstepNet's gym env) with the observation and raw network output of every step;
//! `generate_planner_parity.py` rebuilds it. A stub network checks each observation it is given and replays the
//! recorded output, so this pins everything around the network — observation, mirroring, clipping, the neutral
//! step, the rollout and the reached test — without ONNX Runtime.

use duck_footstep_control::{Foot, Obstacle, Planner, PlannerError, PlannerOptions, StepNet};
use serde_json::Value;

struct Replay {
    steps: Vec<(Vec<f64>, [f32; 3])>,
    next: usize,
    worst_obs: f64,
}

impl StepNet for Replay {
    fn step(&mut self, observation: &[f32; 8]) -> Result<[f32; 3], PlannerError> {
        let (expected, out) = self
            .steps
            .get(self.next)
            .ok_or_else(|| PlannerError::Net("more steps than the reference".into()))?;
        for (got, want) in observation.iter().zip(expected) {
            self.worst_obs = self.worst_obs.max((f64::from(*got) - want).abs());
        }
        self.next += 1;
        Ok(*out)
    }
}

fn foot(v: &Value) -> Foot {
    match v.as_str().unwrap() {
        "left" => Foot::Left,
        "right" => Foot::Right,
        other => panic!("foot {other}"),
    }
}

fn pose(v: &Value) -> [f64; 3] {
    let a: Vec<f64> = serde_json::from_value(v.clone()).unwrap();
    [a[0], a[1], a[2]]
}

#[test]
fn planner_matches_the_python_reference() {
    let fixture: Value =
        serde_json::from_str(include_str!("fixtures/planner_parity.json")).expect("fixture parses");
    let options = PlannerOptions::from_json(&fixture["options"].to_string()).unwrap();
    let mut total = 0;
    for (i, case) in fixture["cases"].as_array().unwrap().iter().enumerate() {
        let steps: Vec<_> = case["steps"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| {
                let obs: Vec<f64> = serde_json::from_value(s["obs"].clone()).unwrap();
                let net: Vec<f32> = serde_json::from_value(s["net"].clone()).unwrap();
                (obs, [net[0], net[1], net[2]])
            })
            .collect();
        let n = steps.len();
        let mut planner = Planner::new(
            Replay {
                steps,
                next: 0,
                worst_obs: 0.0,
            },
            options.clone(),
        );
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
            .unwrap_or_else(|e| panic!("case {i}: {e}"));
        assert_eq!(plan.steps.len(), n, "case {i}: step count");
        assert_eq!(
            plan.reached,
            case["reached"].as_bool().unwrap(),
            "case {i}: reached"
        );
        for (k, ((got, got_foot), want)) in plan
            .steps
            .iter()
            .zip(case["steps"].as_array().unwrap())
            .enumerate()
        {
            let want_pose = pose(&want["landing"]);
            for d in 0..3 {
                assert!(
                    (got[d] - want_pose[d]).abs() < 1e-6,
                    "case {i} step {k}: {got:?} vs {want_pose:?}"
                );
            }
            assert_eq!(*got_foot, foot(&want["foot"]), "case {i} step {k}: foot");
        }
        // The observation goes through f32 on both sides; anything beyond rounding is a convention bug.
        assert!(
            planner_worst(&planner) < 1e-5,
            "case {i}: observation off by {}",
            planner_worst(&planner)
        );
        total += n;
    }
    assert!(total > 100, "fixture too small: {total} steps");
}

fn planner_worst(planner: &Planner<Replay>) -> f64 {
    planner.net().worst_obs
}
