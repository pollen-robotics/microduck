//! The shipped pick-up model, through the Rust loader.
//!
//! Requires ONNX Runtime >= 1.23: cargo test -p duck-control --test pickup_detector -- --ignored
//!
//! `fixtures/pickup_windows.json` holds two one-second windows from the simulator the model was
//! trained in — a robot walking, and one held in the hand — and the probability Python's
//! onnxruntime gave each. Matching them here proves the ring buffer hands the model its rows
//! oldest-first and that the file the release ships is the one that was evaluated.
use duck_control::pickup::{Detector, FEAT_LEN, WINDOW};
use std::path::PathBuf;

fn model() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("models/pickup_detector.onnx")
}

fn window(name: &str) -> (Vec<[f32; FEAT_LEN]>, f32) {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/pickup_windows.json");
    let all: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let case = &all[name];
    let rows = case["window"]
        .as_array()
        .unwrap()
        .iter()
        .map(|row| {
            let v: Vec<f32> = row
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap() as f32)
                .collect();
            <[f32; FEAT_LEN]>::try_from(v).unwrap()
        })
        .collect::<Vec<_>>();
    assert_eq!(rows.len(), WINDOW);
    (rows, case["p_held"].as_f64().unwrap() as f32)
}

#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn the_shipped_model_reproduces_the_training_side_scores() {
    let mut detector = Detector::load(&model()).unwrap();
    for case in ["walking", "held"] {
        let (rows, want) = window(case);
        detector.reset();
        let mut last = None;
        for (i, row) in rows.into_iter().enumerate() {
            last = detector.observe(row).unwrap();
            // No verdict from a partial window.
            assert_eq!(last.is_some(), i == WINDOW - 1, "{case}: tick {i}");
        }
        let got = last.unwrap();
        assert!(
            (got - want).abs() < 2e-3,
            "{case}: got {got}, python said {want}"
        );
    }
}

/// The ring must keep sliding: a window that wraps is still oldest-first.
#[test]
#[ignore = "requires ONNX Runtime >= 1.23"]
fn a_wrapped_window_scores_the_same_as_a_fresh_one() {
    let mut detector = Detector::load(&model()).unwrap();
    let (walking, _) = window("walking");
    let (held, want) = window("held");
    for row in walking.into_iter().take(17) {
        detector.observe(row).unwrap();
    }
    let mut last = None;
    for row in held {
        last = detector.observe(row).unwrap();
    }
    assert!((last.unwrap() - want).abs() < 2e-3);
}
