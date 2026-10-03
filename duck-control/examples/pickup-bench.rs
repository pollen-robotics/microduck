//! What `[pickup]` costs per control tick, on whatever this runs on.
//! cargo run --release -p duck-control --example pickup-bench -- pickup_detector.onnx [walk_policy.onnx] [ticks]
//!
//! Times `Detector::observe` — the per-tick work robotd adds: one row into the window, the window
//! copied out oldest-first, one inference — and, when a policy is given, one policy inference the
//! same way, so the two can be compared on the board itself. Single-threaded, like the loop.
use duck_control::{
    obs::Observation,
    pickup::{Detector, FEAT_LEN, WINDOW},
    policy::{Net, Policy, PolicyPaths},
};
use std::{error::Error, path::PathBuf, time::Instant};

fn report(name: &str, mut us: Vec<f64>) {
    us.sort_by(f64::total_cmp);
    let q = |p: f64| us[((us.len() - 1) as f64 * p) as usize];
    let mean = us.iter().sum::<f64>() / us.len() as f64;
    println!(
        "{name:>8}: mean {mean:7.1} us   p50 {:7.1}   p90 {:7.1}   p99 {:7.1}   max {:7.1}   ({:.2} % of a 20 ms tick at p99)",
        q(0.5),
        q(0.9),
        q(0.99),
        us[us.len() - 1],
        q(0.99) / 200.0
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let model = PathBuf::from(
        args.next()
            .ok_or("usage: pickup-bench DETECTOR.onnx [POLICY.onnx] [TICKS]")?,
    );
    let policy = args
        .next()
        .filter(|a| a.ends_with(".onnx"))
        .map(PathBuf::from);
    let ticks: usize = args.last().and_then(|a| a.parse().ok()).unwrap_or(3000);

    let mut detector = Detector::load(&model)?;
    // A plausible, changing row: what matters for timing is that every tick is a real inference
    // on a full window, which it is from the WINDOW-th row on.
    let row = |k: usize| -> [f32; FEAT_LEN] {
        std::array::from_fn(|i| ((k * 31 + i * 7) % 97) as f32 / 97.0 - 0.5)
    };
    for k in 0..WINDOW + 50 {
        detector.observe(row(k))?;
    }
    let mut times = Vec::with_capacity(ticks);
    for k in 0..ticks {
        let r = row(k);
        let t = Instant::now();
        let p = detector.observe(r)?;
        times.push(t.elapsed().as_secs_f64() * 1e6);
        assert!(p.is_some());
    }
    report("pickup", times);

    if let Some(path) = policy {
        let mut policy = Policy::load(
            &PolicyPaths {
                walk: path,
                ..Default::default()
            },
            0.05,
        )?;
        for _ in 0..50 {
            policy.infer(&Observation::zeroed(), Net::Walk)?;
        }
        let mut times = Vec::with_capacity(ticks);
        for k in 0..ticks {
            let obs = Observation::from(std::array::from_fn(|i| ((k + i) % 13) as f32 / 13.0));
            let t = Instant::now();
            policy.infer(&obs, Net::Walk)?;
            times.push(t.elapsed().as_secs_f64() * 1e6);
        }
        report("policy", times);
    }
    Ok(())
}
