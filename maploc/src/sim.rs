//! A synthetic world with ground truth, for testing the mapper without a robot.
//!
//! Rooms are vertical prisms — walls, boxes, table legs and table tops — and the robot is a
//! script of walks, stops and carries. It produces the two streams the service consumes, built
//! the way the real ones are: state at 50 Hz carrying odometry that drifts (stride scale, gyro
//! bias, lateral slip), depth at 15 Hz stamped when read rather than when captured, the head
//! sweeping during stops, the trunk bobbing while it walks. The truth is kept alongside, so a
//! test can ask how wrong the mapper is, not merely whether it ran.
//!
//! The MuJoCo twin is the higher-fidelity bench; this one is deterministic, runs in
//! milliseconds, and lives next to the code it tests.

use kinematics::{Pose, Quat};

use crate::input::{Activity, DepthFrame, StateSample};
use crate::se2::{Pose2, wrap};

/// A convex polygon (counter-clockwise) extruded between two heights.
#[derive(Debug, Clone)]
pub struct Prism {
    pub poly: Vec<[f64; 2]>,
    pub z0: f64,
    pub z1: f64,
}

impl Prism {
    pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64, z0: f64, z1: f64) -> Self {
        Self {
            poly: vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
            z0,
            z1,
        }
    }

    /// Distance along the unit ray `o + t·d` to where it enters, if it does, by clipping the ray
    /// against every face's half-space.
    fn hit(&self, o: [f64; 3], d: [f64; 3]) -> Option<f64> {
        let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
        let mut clip = |num: f64, den: f64| -> bool {
            // Inside where num + t·den <= 0.
            if den.abs() < 1e-12 {
                return num <= 0.0;
            }
            let t = -num / den;
            if den < 0.0 {
                t0 = t0.max(t);
            } else {
                t1 = t1.min(t);
            }
            true
        };
        let n = self.poly.len();
        for i in 0..n {
            let a = self.poly[i];
            let b = self.poly[(i + 1) % n];
            let nrm = [b[1] - a[1], a[0] - b[0]];
            if !clip(
                nrm[0] * (o[0] - a[0]) + nrm[1] * (o[1] - a[1]),
                nrm[0] * d[0] + nrm[1] * d[1],
            ) {
                return None;
            }
        }
        if !clip(self.z0 - o[2], -d[2]) || !clip(o[2] - self.z1, d[2]) {
            return None;
        }
        (t0 <= t1 && t0 > 0.0).then_some(t0)
    }
}

#[derive(Debug, Clone, Default)]
pub struct World {
    pub prisms: Vec<Prism>,
}

impl World {
    /// Nearest surface along a unit ray, the floor included.
    pub fn cast(&self, o: [f64; 3], d: [f64; 3]) -> Option<f64> {
        let mut best = (d[2] < -1e-9).then(|| -o[2] / d[2]);
        for p in &self.prisms {
            if let Some(t) = p.hit(o, d) {
                best = Some(best.map_or(t, |b: f64| b.min(t)));
            }
        }
        best
    }

    /// A walled room with one thin wall per side, `w × h`, its low corner at the origin.
    pub fn room(w: f64, h: f64) -> Self {
        let t = 0.05;
        Self {
            prisms: vec![
                Prism::rect(-t, -t, w + t, 0.0, 0.0, 1.0),
                Prism::rect(-t, h, w + t, h + t, 0.0, 1.0),
                Prism::rect(-t, 0.0, 0.0, h, 0.0, 1.0),
                Prism::rect(w, 0.0, w + t, h, 0.0, 1.0),
            ],
        }
    }

    pub fn with(mut self, p: Prism) -> Self {
        self.prisms.push(p);
        self
    }

    /// A table: four legs and a top at 0.7 m — open underneath for a duck.
    pub fn with_table(self, x0: f64, y0: f64, x1: f64, y1: f64) -> Self {
        let l = 0.04;
        self.with(Prism::rect(x0, y0, x0 + l, y0 + l, 0.0, 0.7))
            .with(Prism::rect(x1 - l, y0, x1, y0 + l, 0.0, 0.7))
            .with(Prism::rect(x0, y1 - l, x0 + l, y1, 0.0, 0.7))
            .with(Prism::rect(x1 - l, y1 - l, x1, y1, 0.0, 0.7))
            .with(Prism::rect(x0, y0, x1, y1, 0.66, 0.7))
    }
}

/// How the robot's odometry lies.
#[derive(Debug, Clone, Copy)]
pub struct Drift {
    /// Stride scale error: odometry reads `1 + scale` times the distance walked.
    pub scale: f64,
    /// Sideways slip per metre walked, metres (a constant crab).
    pub slip_per_m: f64,
    /// Gyro bias left after fusion, rad/s.
    pub yaw_bias: f64,
    /// Gyro scale error on turns.
    pub yaw_scale: f64,
}

impl Drift {
    pub const NONE: Self = Self {
        scale: 0.0,
        slip_per_m: 0.0,
        yaw_bias: 0.0,
        yaw_scale: 0.0,
    };
}

#[derive(Debug, Clone, Copy)]
pub enum Step {
    /// Turn to face the target, then walk straight to it.
    Walk { to: [f64; 2] },
    /// Turn in place to a heading.
    Turn { yaw: f64 },
    /// Stand still for `secs`, sweeping the head if `sweep`.
    Stop { secs: f64, sweep: bool },
    /// Picked up, carried to a pose, put down. Odometry position freezes while carried (its feet
    /// are in the air); the gyro keeps the heading.
    Carry { to: Pose2 },
}

pub struct Robot {
    pub world: World,
    pub drift: Drift,
    pub truth: Pose2,
    pub odom: Pose2,
    pub t_ns: u64,
    /// Read latency of a depth frame after its capture.
    pub tof_latency_ns: u64,
    /// Range noise, metres, deterministic.
    pub range_noise_m: f64,
    /// The sensor stops reporting beyond this.
    pub max_range_m: f64,
    pub head_pitch: f64,
    next_frame_ns: u64,
    rng: u64,
    head_yaw: f64,
    /// The truth at each state tick, for tests.
    pub truth_log: Vec<(u64, Pose2)>,
}

/// Something the robot emitted, in time order.
pub enum Out {
    State(StateSample),
    Depth(DepthFrame),
}

const TICK_NS: u64 = 20_000_000;
const FRAME_NS: u64 = 66_666_667;
const WALK_SPEED: f64 = 0.15;
const TURN_SPEED: f64 = 0.6;

impl Robot {
    pub fn new(world: World, start: Pose2, drift: Drift) -> Self {
        Self {
            world,
            drift,
            truth: start,
            odom: start,
            t_ns: 1_000_000_000,
            tof_latency_ns: 10_000_000,
            range_noise_m: 0.01,
            max_range_m: 1.6,
            head_pitch: 8f64.to_radians(),
            next_frame_ns: 1_000_000_000,
            rng: 0x2545_f491_4f6c_dd1d,
            head_yaw: 0.0,
            truth_log: Vec::new(),
        }
    }

    fn noise(&mut self) -> f64 {
        // xorshift: deterministic, so a failing test fails the same way twice.
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    }

    /// Run a script, handing every emitted sample to `sink` in time order.
    pub fn run(&mut self, steps: &[Step], sink: &mut dyn FnMut(Out)) {
        for &s in steps {
            match s {
                Step::Walk { to } => {
                    let yaw = (to[1] - self.truth.y).atan2(to[0] - self.truth.x);
                    self.turn_to(yaw, sink);
                    let dist = (to[0] - self.truth.x).hypot(to[1] - self.truth.y);
                    let n = (dist / (WALK_SPEED * TICK_NS as f64 * 1e-9))
                        .ceil()
                        .max(1.0) as usize;
                    let step = dist / n as f64;
                    for k in 0..n {
                        let bob = 0.03 * (k as f64 * 0.8).sin();
                        self.tick(Pose2::new(step, 0.0, 0.0), true, false, bob, sink);
                    }
                }
                Step::Turn { yaw } => self.turn_to(yaw, sink),
                Step::Stop { secs, sweep } => {
                    let n = (secs / (TICK_NS as f64 * 1e-9)) as usize;
                    for k in 0..n {
                        // A head sweep of ±0.9 rad over 6 s, starting forward.
                        self.head_yaw = if sweep {
                            0.9 * (2.0 * std::f64::consts::PI * k as f64 * TICK_NS as f64 * 1e-9
                                / 6.0)
                                .sin()
                        } else {
                            0.0
                        };
                        self.tick(Pose2::IDENTITY, false, false, 0.0, sink);
                    }
                    self.head_yaw = 0.0;
                }
                Step::Carry { to } => {
                    for _ in 0..100 {
                        self.tick(Pose2::IDENTITY, false, true, 0.0, sink);
                    }
                    let dyaw = wrap(to.yaw - self.truth.yaw);
                    self.truth = to;
                    // The gyro sees the turn; the legs see nothing.
                    self.odom.yaw = wrap(self.odom.yaw + dyaw * (1.0 + self.drift.yaw_scale));
                    for _ in 0..50 {
                        self.tick(Pose2::IDENTITY, false, true, 0.0, sink);
                    }
                }
            }
        }
    }

    fn turn_to(&mut self, yaw: f64, sink: &mut dyn FnMut(Out)) {
        let d = wrap(yaw - self.truth.yaw);
        let n = (d.abs() / (TURN_SPEED * TICK_NS as f64 * 1e-9)).ceil() as usize;
        for _ in 0..n {
            self.tick(Pose2::new(0.0, 0.0, d / n as f64), true, false, 0.0, sink);
        }
    }

    /// Advance one control tick by `delta` in the body frame, emitting state and any depth frame
    /// captured during it.
    fn tick(
        &mut self,
        delta: Pose2,
        commanded: bool,
        carried: bool,
        pitch: f64,
        sink: &mut dyn FnMut(Out),
    ) {
        let dt = TICK_NS as f64 * 1e-9;
        self.truth = self.truth.compose(delta);
        if !carried {
            let d = self.drift;
            let lin = delta.x.hypot(delta.y);
            let od = Pose2::new(
                delta.x * (1.0 + d.scale),
                delta.y * (1.0 + d.scale) + d.slip_per_m * lin,
                delta.yaw * (1.0 + d.yaw_scale),
            );
            self.odom = self.odom.compose(od);
        }
        self.odom.yaw = wrap(self.odom.yaw + self.drift.yaw_bias * dt);
        self.t_ns += TICK_NS;
        self.truth_log.push((self.t_ns, self.truth));

        // Depth frames captured during this tick, emitted after it (they are read later).
        while self.next_frame_ns <= self.t_ns {
            let cap = self.next_frame_ns;
            self.next_frame_ns += FRAME_NS;
            let f = self.depth(cap, pitch);
            // The reading lags the capture; the service sees it after this tick's state, which is
            // what a mapper waiting on state history must cope with.
            sink(Out::Depth(f));
        }
        sink(Out::State(self.state(commanded, carried, pitch)));
    }

    fn trunk_quat(&self, yaw: f64, pitch: f64) -> Quat {
        Quat::from_axis_angle([0.0, 0.0, 1.0], yaw) * Quat::from_axis_angle([0.0, 1.0, 0.0], pitch)
    }

    fn tof_in_trunk(&self) -> Pose {
        let q = Quat::from_axis_angle([0.0, 0.0, 1.0], self.head_yaw)
            * Quat::from_axis_angle([0.0, 1.0, 0.0], self.head_pitch);
        let (s, c) = self.head_yaw.sin_cos();
        // The sensor sits ahead of the neck's yaw axis; turning the head swings it round.
        Pose::new([0.05 + 0.03 * c, 0.03 * s, 0.13], q)
    }

    fn state(&self, commanded: bool, carried: bool, pitch: f64) -> StateSample {
        StateSample {
            t_ns: self.t_ns,
            trunk_pos: [self.odom.x, self.odom.y, 0.12],
            odom_yaw: self.odom.yaw,
            trunk_quat: self.trunk_quat(self.odom.yaw, pitch),
            tof_in_trunk: self.tof_in_trunk(),
            activity: Activity {
                commanded,
                picked_up: carried,
                ..Activity::default()
            },
        }
    }

    fn depth(&mut self, capture_ns: u64, pitch: f64) -> DepthFrame {
        let trunk = Pose::new(
            [self.truth.x, self.truth.y, 0.12],
            self.trunk_quat(self.truth.yaw, pitch),
        );
        let sensor = trunk * self.tof_in_trunk();
        let beams = *kinematics::tof::Reprojector::alpha().beams();
        let mut distance_mm = vec![0i16; 64];
        let mut status = vec![0u8; 64];
        for (i, b) in beams.iter().enumerate() {
            let d = sensor.quat.rotate(*b);
            if let Some(t) = self.world.cast(sensor.pos, d)
                && t <= self.max_range_m
            {
                let r = t + self.range_noise_m * 2.0 * self.noise();
                distance_mm[i] = (r * 1000.0) as i16;
                status[i] = 5;
            }
        }
        DepthFrame {
            t_ns: capture_ns + self.tof_latency_ns,
            distance_mm,
            status,
        }
    }

    /// The true pose at a time.
    pub fn truth_at(&self, t_ns: u64) -> Option<Pose2> {
        let i = self.truth_log.partition_point(|(t, _)| *t <= t_ns);
        self.truth_log.get(i.checked_sub(1)?).map(|(_, p)| *p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rays_hit_walls_tables_and_floor_where_they_should() {
        let w = World::room(4.0, 3.0).with_table(2.0, 1.0, 3.0, 2.0);
        // Level ray along +x from (1, 1.5, 0.25): passes under the table top (0.66 m) between
        // the legs, reaches the far wall at x = 4.
        let t = w.cast([1.0, 1.5, 0.25], [1.0, 0.0, 0.0]).unwrap();
        assert!((t - 3.0).abs() < 1e-9, "{t}");
        // Down at 45°: the floor at 0.25·√2.
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let t = w.cast([1.0, 1.5, 0.25], [s, 0.0, -s]).unwrap();
        assert!((t - 0.25 * 2f64.sqrt()).abs() < 1e-9);
        // Through a leg at y = 1.02.
        let t = w.cast([1.0, 1.02, 0.25], [1.0, 0.0, 0.0]).unwrap();
        assert!((t - 1.0).abs() < 1e-9, "{t}");
    }
}
