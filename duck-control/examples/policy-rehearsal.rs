//! Offline inference only: never opens a motor bus.
//! cargo run --release -p duck-control --example policy-rehearsal -- policy.onnx [trace.json]
//! Trace: [{"obs": [61 floats], "reset": false}, ...]. Outputs actions and latency JSON.
use duck_control::{
    obs::Observation,
    policy::{Net, Policy, PolicyPaths},
};
use std::{error::Error, path::PathBuf, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let path = PathBuf::from(
        args.next()
            .ok_or("usage: policy-rehearsal MODEL.onnx [TRACE.json]")?,
    );
    let trace: Vec<serde_json::Value> = if let Some(path) = args.next() {
        serde_json::from_slice(&std::fs::read(path)?)?
    } else {
        // Benchmark with nominal gravity, zero commands and previous-action feedback.
        vec![serde_json::Value::Null; 1000]
    };
    if trace.is_empty() || args.next().is_some() {
        return Err("expected a non-empty trace and at most two arguments".into());
    }
    let mut policy = Policy::load(
        &PolicyPaths {
            walk: path,
            ..Default::default()
        },
        0.05,
    )?;
    // Warm CPU/kernel caches, then discard all memory from these synthetic steps.
    for _ in 0..50 {
        policy.infer(&Observation::zeroed(), Net::Walk)?;
    }
    policy.reset();
    let mut actions = Vec::with_capacity(trace.len());
    let mut times = Vec::with_capacity(trace.len());
    let mut previous = [0.0; 14];
    for entry in trace {
        let mut x = [0.0f32; 61];
        if entry.is_null() {
            x[5] = -1.0;
            x[34..48].copy_from_slice(&previous);
        } else {
            let values: Vec<f32> =
                serde_json::from_value(entry.get("obs").ok_or("trace entry missing obs")?.clone())?;
            x = values
                .try_into()
                .map_err(|_| "trace observations must contain exactly 61 numbers")?;
            if entry
                .get("reset")
                .and_then(|v| v.as_bool())
                .unwrap_or(false)
            {
                policy.reset();
            }
        }
        let observation = Observation::from(x);
        let start = Instant::now();
        previous = policy.infer(&observation, Net::Walk)?;
        times.push(start.elapsed().as_secs_f64() * 1000.0);
        actions.push(previous);
    }
    times.sort_by(f64::total_cmp);
    let percentile = |p: f64| times[((times.len() as f64 * p).ceil() as usize).saturating_sub(1)];
    println!(
        "{}",
        serde_json::json!({
            "steps": times.len(), "latency_ms": {"p50": percentile(0.50), "p95": percentile(0.95),
            "p99": percentile(0.99), "max": times[times.len()-1]},
            "over_20_ms": times.iter().filter(|&&t| t > 20.0).count(), "actions": actions,
            "scope": "actor inference only; excludes sensor and motor I/O"
        })
    );
    Ok(())
}
