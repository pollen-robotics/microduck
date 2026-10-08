//! The mapper against a synthetic world with ground truth: does it actually know where it is?

use maploc::mapper::{Event, Mapper, MapperConfig};
use maploc::se2::{Pose2, wrap};
use maploc::sim::{Drift, Out, Prism, Robot, Step, World};

fn living_room() -> World {
    World::room(4.0, 3.0)
        .with_table(2.4, 1.8, 3.4, 2.6)
        .with(Prism::rect(0.0, 1.2, 0.5, 1.9, 0.0, 0.45)) // a sofa end against the wall
        .with(Prism::rect(1.6, 0.0, 2.3, 0.35, 0.0, 0.5)) // a low cabinet
        .with(Prism::rect(1.9, 1.3, 2.1, 1.5, 0.0, 0.15)) // a low box in the middle
}

fn realistic_drift() -> Drift {
    Drift {
        scale: 0.06,
        slip_per_m: 0.04,
        yaw_bias: 1f64.to_radians() / 60.0,
        yaw_scale: 0.015,
    }
}

/// A lap of the room with a sweeping stop every ~0.6 m.
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
    let mut v = Vec::new();
    for p in pts {
        v.push(Step::Walk { to: p });
        v.push(Step::Stop {
            secs: 4.0,
            sweep: true,
        });
    }
    v
}

struct Run {
    mapper: Mapper,
    robot: Robot,
    events: Vec<Event>,
}

fn run(mapper: Mapper, robot: Robot, steps: &[Step]) -> Run {
    let mut r = Run {
        mapper,
        robot,
        events: Vec::new(),
    };
    let Run {
        mapper,
        robot,
        events,
    } = &mut r;
    robot.run(steps, &mut |o| {
        match o {
            Out::State(s) => mapper.on_state(s),
            Out::Depth(f) => mapper.on_depth(f),
        }
        events.extend(mapper.take_events());
    });
    r
}

fn err(a: Pose2, b: Pose2) -> (f64, f64) {
    (a.dist(b), wrap(a.yaw - b.yaw).abs().to_degrees())
}

fn start() -> Robot {
    Robot::new(living_room(), Pose2::new(1.0, 0.8, 0.0), realistic_drift())
}

fn first_stop() -> Vec<Step> {
    vec![Step::Stop {
        secs: 4.0,
        sweep: true,
    }]
}

#[test]
fn tracking_beats_odometry_over_two_laps() {
    let mut steps = first_stop();
    steps.extend(lap());
    steps.extend(lap());
    let r = run(Mapper::new(MapperConfig::default()), start(), &steps);
    let truth = r.robot.truth;
    let pose = r.mapper.pose().expect("tracking at the end");
    let (e_map, a_map) = err(pose, truth);
    let (e_odo, a_odo) = err(r.robot.odom, truth);
    let n_kf = r
        .events
        .iter()
        .filter(|e| matches!(e, Event::Keyframe { .. }))
        .count();
    let n_matched = r
        .events
        .iter()
        .filter(|e| matches!(e, Event::Keyframe { matched: Some(m), .. } if m.accepted))
        .count();
    let loops = r
        .events
        .iter()
        .filter(|e| matches!(e, Event::LoopClosed { .. }))
        .count();
    eprintln!("keyframes {n_kf} matched {n_matched} loops {loops}");
    eprintln!("mapper error {e_map:.3} m {a_map:.1}°, odometry error {e_odo:.3} m {a_odo:.1}°");
    assert!(r.mapper.lost().is_none());
    assert!(e_map < 0.10, "mapper ended {e_map:.3} m off");
    assert!(
        e_map < 0.5 * e_odo,
        "no better than odometry: {e_map:.3} vs {e_odo:.3}"
    );
}

#[test]
#[ignore]
fn debug_per_keyframe() {
    let mut steps = first_stop();
    steps.extend(lap());
    steps.extend(lap());
    let r = run(Mapper::new(MapperConfig::default()), start(), &steps);
    for e in &r.events {
        if let Event::Keyframe {
            id, matched, occ, ..
        } = e
        {
            let kf = &r.mapper.keyframes()[*id as usize];
            let t = r.robot.truth_at(kf.t_start_ns).unwrap();
            let p = r.mapper.graph().nodes[*id as usize];
            let (e, a) = err(p, t);
            eprintln!("kf {id:2} occ {occ:4} span {:3.0}° err {e:.3} m {a:4.1}°  {}", kf.span_rad.to_degrees(), matched.as_ref().map_or("-".into(), |m| format!("{} r{} corr {:.3} {:.3} {:.1}° hit {:.2} known {:.2} confl {:.2} n {} σ {:.3} {:.3} {:.2}°", if m.accepted {"OK "} else {"no "}, m.rank, m.correction.x, m.correction.y, m.correction.yaw.to_degrees(), m.hit_frac, m.known_frac, m.conflict_frac, m.n_points, m.sigma[0], m.sigma[1], m.sigma[2])));
        } else {
            eprintln!("{e:?}");
        }
    }
}

fn mapped_room() -> Run {
    let mut steps = first_stop();
    steps.extend(lap());
    run(Mapper::new(MapperConfig::default()), start(), &steps)
}

/// A short wander with sweeping stops from wherever the robot is.
fn wander(from: [f64; 2]) -> Vec<Step> {
    let mut v = vec![Step::Stop {
        secs: 4.0,
        sweep: true,
    }];
    for d in [[0.3, 0.0], [0.3, 0.3], [0.0, 0.3]] {
        v.push(Step::Walk {
            to: [from[0] + d[0], from[1] + d[1]],
        });
        v.push(Step::Stop {
            secs: 4.0,
            sweep: true,
        });
    }
    v
}

/// How close a relocalization must land to the truth. The heading allowance is not the search's
/// error — without drift it lands within 4 cm and 1° — but the map's: one lap under
/// [`realistic_drift`] leaves ~2° of heading error baked into the first lap's keyframes, and a
/// relocalized pose is right *in the map*, not in the world.
const RELOC_M: f64 = 0.15;
const RELOC_DEG: f64 = 6.0;

fn relocalized_after(events: &[Event]) -> Option<usize> {
    events.iter().find_map(|e| match e {
        Event::Relocalized {
            after_keyframes, ..
        } => Some(*after_keyframes),
        _ => None,
    })
}

#[test]
fn a_carried_robot_finds_itself_again() {
    let mut r = mapped_room();
    let n_before = r.events.len();
    let to = Pose2::new(2.6, 1.0, 2.2);
    let mut steps = vec![Step::Carry { to }];
    steps.extend(wander([to.x, to.y]));
    let Run {
        mapper,
        robot,
        events,
    } = &mut r;
    robot.run(&steps, &mut |o| {
        match o {
            Out::State(s) => mapper.on_state(s),
            Out::Depth(f) => mapper.on_depth(f),
        }
        events.extend(mapper.take_events());
    });
    let after = &r.events[n_before..];
    assert!(
        after.iter().any(|e| matches!(e, Event::Lost { .. })),
        "the carry went unnoticed"
    );
    let n = relocalized_after(after).expect("never relocalized");
    let (e, a) = err(r.mapper.pose().unwrap(), r.robot.truth);
    eprintln!("relocalized after {n} stops, error {e:.3} m {a:.1}°");
    assert!(e < RELOC_M && a < RELOC_DEG, "{e:.3} m {a:.1}°");
}

fn reboot(r: &Run, truth: Pose2) -> (Mapper, Robot) {
    let session: maploc::mapper::Session =
        serde_json::from_str(&serde_json::to_string(&r.mapper.session()).unwrap()).unwrap();
    let mapper = Mapper::resume(MapperConfig::default(), session);
    let mut robot = Robot::new(living_room(), truth, realistic_drift());
    // A fresh odometry frame: wherever it boots is the origin, facing an arbitrary way.
    robot.odom = Pose2::IDENTITY;
    (mapper, robot)
}

#[test]
fn after_a_power_cycle_in_place_it_resumes_where_it_was() {
    let r = mapped_room();
    let parked = r.robot.truth;
    let (mapper, robot) = reboot(&r, parked);
    assert!(
        mapper.lost().is_some(),
        "a rebooted mapper cannot know where it is yet"
    );
    let r2 = run(mapper, robot, &wander([parked.x, parked.y]));
    let n = relocalized_after(&r2.events).expect("never relocalized after reboot");
    let (e, a) = err(r2.mapper.pose().unwrap(), r2.robot.truth);
    eprintln!("relocalized after {n} stops, error {e:.3} m {a:.1}°");
    assert!(n <= 3);
    assert!(e < RELOC_M && a < RELOC_DEG, "{e:.3} m {a:.1}°");
}

#[test]
fn after_a_power_cycle_somewhere_else_it_searches_the_whole_map() {
    let r = mapped_room();
    let moved = Pose2::new(2.8, 1.2, -2.5);
    let (mapper, robot) = reboot(&r, moved);
    let r2 = run(mapper, robot, &wander([moved.x, moved.y]));
    let n = relocalized_after(&r2.events).expect("never relocalized after being moved while off");
    let (e, a) = err(r2.mapper.pose().unwrap(), r2.robot.truth);
    eprintln!("relocalized after {n} stops, error {e:.3} m {a:.1}°");
    assert!(e < RELOC_M && a < RELOC_DEG, "{e:.3} m {a:.1}°");
}

#[test]
fn a_low_box_is_on_the_map_and_a_table_top_is_not() {
    let r = mapped_room();
    let g = r.mapper.grid(0.05).unwrap();
    let at = |x: f64, y: f64| {
        let i = ((x - g.origin[0]) / g.res) as usize;
        let j = ((y - g.origin[1]) / g.res) as usize;
        g.cells[j * g.w + i]
    };
    let occupied_near = |x0: f64, y0: f64, x1: f64, y1: f64| {
        let mut n = 0;
        let mut x = x0;
        while x <= x1 {
            let mut y = y0;
            while y <= y1 {
                n += usize::from(at(x, y) == maploc::grid::Cell::Occupied);
                y += 0.05;
            }
            x += 0.05;
        }
        n
    };
    // The 15 cm box at (1.9–2.1, 1.3–1.5), with 10 cm of slack for the map's own error.
    let box_cells = occupied_near(1.8, 1.2, 2.2, 1.6);
    // Under the table top (2.4–3.4, 1.8–2.6), away from its legs: nothing the duck hits.
    let under_table = occupied_near(2.6, 2.0, 3.2, 2.4);
    eprintln!("box cells {box_cells}, under the table {under_table}");
    assert!(box_cells >= 2, "the low box was erased");
    assert_eq!(under_table, 0, "the table top was drawn as a wall");
}

#[test]
#[ignore]
fn debug_reloc_without_drift() {
    let mut steps = first_stop();
    steps.extend(lap());
    let r = run(
        Mapper::new(MapperConfig::default()),
        Robot::new(living_room(), Pose2::new(1.0, 0.8, 0.0), Drift::NONE),
        &steps,
    );
    for truth in [r.robot.truth, Pose2::new(2.8, 1.2, -2.5)] {
        let session = r.mapper.session();
        let mapper = Mapper::resume(MapperConfig::default(), session);
        let mut robot = Robot::new(living_room(), truth, Drift::NONE);
        robot.odom = Pose2::IDENTITY;
        let r2 = run(mapper, robot, &wander([truth.x, truth.y]));
        for e in &r2.events {
            if !matches!(e, Event::Keyframe { .. }) {
                eprintln!("{e:?}");
            }
        }
        let (e, a) = err(r2.mapper.pose().unwrap(), r2.robot.truth);
        eprintln!("no drift: error {e:.3} m {a:.1}°");
    }
}

/// The real robot's fall, as its journal told it: the pick-up detector fires for a third of a
/// second as the stumble begins, then the robot lands a little way off and stands back up. That is
/// not a carry — nobody takes a duck anywhere in 0.36 s — and searching the whole map for it, as
/// the first cut did, left the robot lost for good in a thin map. It must look nearby.
#[test]
fn a_stumble_is_searched_for_nearby_and_found() {
    let mut r = mapped_room();
    let n_before = r.events.len();
    let here = r.robot.truth;
    let mut steps = vec![Step::Stumble {
        by: Pose2::new(0.15, -0.10, 0.5),
    }];
    let after = here.compose(Pose2::new(0.15, -0.10, 0.5));
    steps.extend(wander([after.x, after.y]));
    let Run {
        mapper,
        robot,
        events,
    } = &mut r;
    robot.run(&steps, &mut |o| {
        match o {
            Out::State(s) => mapper.on_state(s),
            Out::Depth(f) => mapper.on_depth(f),
        }
        events.extend(mapper.take_events());
    });
    let after = &r.events[n_before..];
    assert!(
        after.iter().any(|e| matches!(
            e,
            Event::Lost {
                cause: maploc::mapper::LostCause::Bumped
            }
        )),
        "a third of a second in the air was taken for a carry: {:?}",
        after
            .iter()
            .filter(|e| matches!(e, Event::Lost { .. }))
            .collect::<Vec<_>>()
    );
    for e in after {
        if let Event::Searched { scope, outcome, .. } = e {
            eprintln!("search ({scope}): {outcome}");
        }
    }
    let n = relocalized_after(after).expect("never relocalized after a stumble");
    // Judged at the stop that relocalized, against the map where it landed: the nearest stop of
    // the first lap carries the map's own error there, and a relocalization can only be as right
    // as the map it lands in.
    let node = after
        .iter()
        .find_map(|e| match e {
            Event::Relocalized { node, .. } => Some(*node),
            _ => None,
        })
        .unwrap();
    let truth_at = |id: usize| {
        r.robot
            .truth_at(r.mapper.keyframes()[id].t_start_ns)
            .unwrap()
    };
    let (pose, truth) = (r.mapper.graph().nodes[node], truth_at(node));
    let nearest = (0..16)
        .min_by(|&a, &b| truth_at(a).dist(truth).total_cmp(&truth_at(b).dist(truth)))
        .unwrap();
    let map_err = wrap(r.mapper.graph().nodes[nearest].yaw - truth_at(nearest).yaw);
    let a = wrap(wrap(pose.yaw - truth.yaw) - map_err)
        .abs()
        .to_degrees();
    let e = pose.dist(truth);
    eprintln!(
        "relocalized after {n} stops: {e:.3} m off, {a:.1}° beyond the map's own heading error"
    );
    assert!(n <= 3);
    // Two first-lap stops overlap here — the map's origin, fixed, and the lap's end, ~3° off it —
    // and the match may land on either, so the heading allowance is that disagreement plus a degree.
    assert!(e < RELOC_M && a < 4.5, "{e:.3} m {a:.1}°");
}

#[test]
#[ignore]
fn debug_stumble_without_drift() {
    let drift = if std::env::var("DRIFT").is_ok() {
        realistic_drift()
    } else {
        Drift::NONE
    };
    let mut steps = first_stop();
    steps.extend(lap());
    let mut r = run(
        Mapper::new(MapperConfig::default()),
        Robot::new(living_room(), Pose2::new(1.0, 0.8, 0.0), drift),
        &steps,
    );
    let n0 = r.events.len();
    let after = r.robot.truth.compose(Pose2::new(0.15, -0.10, 0.5));
    let mut steps = vec![Step::Stumble {
        by: Pose2::new(0.15, -0.10, 0.5),
    }];
    steps.extend(wander([after.x, after.y]));
    let Run {
        mapper,
        robot,
        events,
    } = &mut r;
    robot.run(&steps, &mut |o| {
        match o {
            Out::State(s) => mapper.on_state(s),
            Out::Depth(f) => mapper.on_depth(f),
        }
        events.extend(mapper.take_events());
    });
    for e in &r.events[n0..] {
        match e {
            Event::Keyframe { id, .. } => {
                let kf = &r.mapper.keyframes()[*id as usize];
                let t = r.robot.truth_at(kf.t_start_ns).unwrap();
                let p = r.mapper.graph().nodes[*id as usize];
                eprintln!(
                    "kf {id} truth {:.2} {:.2} {:.1}° node {:.2} {:.2} {:.1}°",
                    t.x,
                    t.y,
                    t.yaw.to_degrees(),
                    p.x,
                    p.y,
                    p.yaw.to_degrees()
                );
            }
            other => eprintln!("{other:?}"),
        }
    }
    let (e, a) = err(r.mapper.pose().unwrap(), r.robot.truth);
    eprintln!("no drift: {e:.3} m {a:.1}°");
}
