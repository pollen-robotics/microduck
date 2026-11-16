//! How the mapper's cost grows with the map: lap the synthetic living room for hundreds of stops
//! and time the work each stop costs. Ignored by default (it takes a while):
//!
//!     cargo test --release -p maploc --test scale -- --ignored --nocapture

use std::time::Instant;

use maploc::mapper::{Event, Mapper, MapperConfig};
use maploc::se2::Pose2;
use maploc::sim::{Drift, Out, Prism, Robot, Step, World};

fn living_room() -> World {
    World::room(4.0, 3.0)
        .with_table(2.4, 1.8, 3.4, 2.6)
        .with(Prism::rect(0.0, 1.2, 0.5, 1.9, 0.0, 0.45))
        .with(Prism::rect(1.6, 0.0, 2.3, 0.35, 0.0, 0.5))
        .with(Prism::rect(1.9, 1.3, 2.1, 1.5, 0.0, 0.15))
}

fn lap() -> Vec<Step> {
    let pts = [
        [1.0, 0.8],
        [1.6, 0.8],
        [2.2, 0.8],
        [2.8, 0.8],
        [3.3, 0.9],
        [3.3, 1.5],
        [2.8, 1.5],
        [2.2, 1.1],
        [1.5, 1.4],
        [1.0, 2.0],
        [1.0, 2.6],
        [1.6, 2.5],
        [2.0, 2.0],
        [1.4, 1.2],
        [1.0, 0.8],
    ];
    pts.iter()
        .flat_map(|&p| {
            [
                Step::Walk { to: p },
                Step::Stop {
                    secs: 4.0,
                    sweep: true,
                },
            ]
        })
        .collect()
}

#[test]
#[ignore]
fn cost_per_stop_as_the_map_grows() {
    let laps: usize = std::env::var("LAPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(40);
    let drift = Drift {
        scale: 0.06,
        slip_per_m: 0.04,
        yaw_bias: 1f64.to_radians() / 60.0,
        yaw_scale: 0.015,
    };
    let mut robot = Robot::new(living_room(), Pose2::new(1.0, 0.8, 0.0), drift);
    let mut mapper = Mapper::new(MapperConfig::default());
    let steps: Vec<Step> = (0..laps).flat_map(|_| lap()).collect();
    // The slowest single call between two keyframes is what a stop costs the worker thread.
    let mut slowest = 0f64;
    let mut rows: Vec<(usize, f64)> = Vec::new();
    let mut kfs = 0usize;
    let mut covered = 0usize;
    robot.run(&steps, &mut |o| {
        let t = Instant::now();
        match o {
            Out::State(s) => mapper.on_state(s),
            Out::Depth(f) => mapper.on_depth(f),
        }
        slowest = slowest.max(t.elapsed().as_secs_f64());
        for e in mapper.take_events() {
            if let Event::Covered { .. } = e {
                covered += 1;
                rows.push((kfs, slowest));
                slowest = 0.0;
            }
            if let Event::Keyframe { .. } = e {
                kfs += 1;
                rows.push((kfs, slowest));
                slowest = 0.0;
            }
        }
    });
    let t = Instant::now();
    let grid = mapper.grid(0.025);
    let grid_s = t.elapsed().as_secs_f64();
    println!(
        "{} stops: {kfs} keyframes, {covered} covered",
        kfs + covered
    );
    println!("keyframes  stop cost (ms, laptop, slowest call), per 50 stops");
    for chunk in rows.chunks(50) {
        let n = chunk.last().unwrap().0;
        let mut v: Vec<f64> = chunk.iter().map(|r| r.1 * 1e3).collect();
        v.sort_by(f64::total_cmp);
        println!(
            "{n:6}     median {:7.2}  max {:7.2}",
            v[v.len() / 2],
            v[v.len() - 1]
        );
    }
    println!(
        "grid at 2.5 cm over {kfs} keyframes: {:.1} ms ({:?} cells)",
        grid_s * 1e3,
        grid.map(|g| g.cells.len())
    );
}
