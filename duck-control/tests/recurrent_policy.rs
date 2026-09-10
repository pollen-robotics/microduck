//! Requires ONNX Runtime >= 1.23: cargo test -p duck-control --test recurrent_policy -- --ignored
//! Kept explicit so hosts without the dynamically loaded runtime still run the normal suite.
use duck_control::obs::Observation;
use duck_control::policy::{Net, Policy, PolicyPaths, validate};
use std::path::PathBuf;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(format!("{name}.onnx"))
}
fn load(walk: &str, stand: Option<&str>) -> Policy {
    Policy::load(
        &PolicyPaths {
            walk: fixture(walk),
            stand: stand.map(fixture),
            ..Default::default()
        },
        0.05,
    )
    .unwrap()
}
fn obs() -> Observation {
    Observation::from([0.2; 61])
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn feedforward_outputs_are_unchanged_by_selection_and_reset() {
    let mut p = load("feedforward", None);
    for net in [Net::Walk, Net::Stand, Net::Skill(999), Net::Walk] {
        assert_eq!(p.infer(&obs(), net).unwrap(), [0.2; 14]);
        p.reset();
    }
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn lstm_carries_state_and_reset_reproduces_first_action() {
    let mut p = load("lstm", None);
    let first = p.infer(&obs(), Net::Walk).unwrap();
    let second = p.infer(&obs(), Net::Walk).unwrap();
    assert_ne!(first, second, "memory must affect the next step");
    p.reset();
    assert_eq!(first, p.infer(&obs(), Net::Walk).unwrap());
    assert_eq!(second, p.infer(&obs(), Net::Walk).unwrap());
    // Missing slots resolve to the currently active walking network, without resetting it.
    let third = p.infer(&obs(), Net::Walk).unwrap();
    p.reset();
    p.infer(&obs(), Net::Walk).unwrap();
    p.infer(&obs(), Net::Stand).unwrap();
    assert_eq!(third, p.infer(&obs(), Net::Skill(5)).unwrap());
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn switching_networks_does_not_leak_or_resume_old_state() {
    let mut p = load("lstm", Some("lstm"));
    let first = p.infer(&obs(), Net::Walk).unwrap();
    assert_ne!(first, p.infer(&obs(), Net::Walk).unwrap());
    assert_eq!(first, p.infer(&obs(), Net::Stand).unwrap());
    assert_ne!(first, p.infer(&obs(), Net::Stand).unwrap());
    assert_eq!(first, p.infer(&obs(), Net::Walk).unwrap());
    let mut mixed = load("lstm", Some("feedforward"));
    assert_eq!(first, mixed.infer(&obs(), Net::Walk).unwrap());
    assert_eq!([0.2; 14], mixed.infer(&obs(), Net::Stand).unwrap());
    assert_eq!(first, mixed.infer(&obs(), Net::Walk).unwrap());
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn warmup_does_not_become_episode_history_and_dynamic_batch_works() {
    let mut p = load("lstm", None);
    let first = p.infer(&obs(), Net::Walk).unwrap();
    p.reset();
    assert_eq!(first, p.infer(&obs(), Net::Walk).unwrap());
    let mut dynamic = load("dynamic_batch", None);
    assert_eq!(first, dynamic.infer(&obs(), Net::Walk).unwrap());
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn unsupported_contracts_fail_before_the_control_loop() {
    for name in [
        "bad_width",
        "bad_batch",
        "bad_state_shape",
        "dynamic_hidden",
        "missing_state",
        "extra_input",
        "wrong_type",
        "bad_rank",
        "bad_action_count",
    ] {
        assert!(validate(&fixture(name)).is_err(), "accepted {name}");
    }
    assert!(
        Policy::load(
            &PolicyPaths {
                walk: fixture("nan_state"),
                ..Default::default()
            },
            0.05
        )
        .is_err()
    );
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn bad_inference_does_not_poison_the_next_episode() {
    let mut p = load("lstm", None);
    let first = p.infer(&obs(), Net::Walk).unwrap();
    assert!(
        p.infer(&Observation::from([f32::NAN; 61]), Net::Walk)
            .is_err()
    );
    assert_eq!(first, p.infer(&obs(), Net::Walk).unwrap());
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn replacing_another_slot_preserves_only_the_unchanged_active_network() {
    let mut old = load("lstm", Some("feedforward"));
    old.infer(&obs(), Net::Walk).unwrap();
    let mut replacement = load("lstm", Some("lstm"));
    replacement.carry_over(&old);
    assert_eq!(
        old.infer(&obs(), Net::Walk).unwrap(),
        replacement.infer(&obs(), Net::Walk).unwrap()
    );
    let mut changed = load("feedforward", None);
    changed.carry_over(&old);
    assert_eq!([0.2; 14], changed.infer(&obs(), Net::Walk).unwrap());
    // The inactive standing slot must still start a new episode.
    let mut fresh = load("lstm", None);
    assert_eq!(
        fresh.infer(&obs(), Net::Walk).unwrap(),
        replacement.infer(&obs(), Net::Stand).unwrap()
    );
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn changed_weights_at_the_same_path_start_with_fresh_memory() {
    let path =
        std::env::temp_dir().join(format!("microduck-recurrent-{}.onnx", std::process::id()));
    std::fs::copy(fixture("lstm"), &path).unwrap();
    let paths = PolicyPaths {
        walk: path.clone(),
        ..Default::default()
    };
    let mut old = Policy::load(&paths, 0.05).unwrap();
    old.infer(&obs(), Net::Walk).unwrap();
    std::fs::copy(fixture("lstm_changed"), &path).unwrap();
    let mut replacement = Policy::load(&paths, 0.05).unwrap();
    replacement.carry_over(&old);
    let mut fresh = load("lstm_changed", None);
    assert_eq!(
        fresh.infer(&obs(), Net::Walk).unwrap(),
        replacement.infer(&obs(), Net::Walk).unwrap()
    );
    std::fs::remove_file(path).unwrap();
}
