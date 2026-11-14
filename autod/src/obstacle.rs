//! What the head ToF says is in the way. `docs/design/autonomous.md` §5 is the reasoning; this
//! is the mechanism.
//!
//! The runtime's avoidance was built for a sensor arriving at about 1 Hz and was unreliable. Its
//! failures, each answered here:
//!
//! - **The floor read as a wall when the trunk pitched.** Its floor rule had no IMU and a constant
//!   trunk height. Here [`kinematics::tof::Reprojector`] levels every beam by the measured gravity,
//!   at odometry's trunk height.
//! - **Phantom stalls.** Only statuses 5 and 9 counted as *observed*, so a dark or moving target
//!   past ~30 cm (4, 13) left the path unobserved and the duck stood waiting for data. Here 255 is
//!   *observed clear*, the theremin's wider set is a range, and everything else is unknown.
//! - **"Clear" overwrote "remembered".** One frame that failed to see a chair leg again dropped it.
//!   Here a confirmed hit lives in the odometry frame for [`STALE_S`] whatever later frames say,
//!   and it is re-expressed in the current body frame every tick, so it keeps getting closer as the
//!   duck walks at it.
//! - **Single noisy zones.** A hit counts only with a second hit within [`CONFIRM_M`] of it, in
//!   the same frame or the one before. At 15 Hz that costs one frame at most.
//!
//! Drops, beams that went over an edge, are hazards too: §5 step 3.
//!
//! Pure: frames and states in, a summary out, no sockets — the tests drive it with synthetic
//! frames.

use std::collections::VecDeque;

use duck_ipc_proto as proto;
use kinematics::tof::{COLS, Posture, ROWS, Reprojector, Zone};

/// Walking at a remembered obstacle closer than this, measured from the beak, stops the duck and
/// turns it away. Practically beak distance: bolder reads better.
pub const STOP_M: f64 = 0.15;
/// Closer than this, slow down and steer away. At the pad's 0.3 m/s that is a second and a half
/// of warning, and the room still has to let the duck walk right up to things before it turns —
/// a wider envelope is what pens a duck in a furnished room.
pub const REACT_M: f64 = 0.45;
/// The path ahead counts as freshly seen for this long after the last frame that covered it.
/// Sized so an ordinary wander glance (the head away for up to ~0.8 s) does not de-rate the walk.
pub const FRESH_S: f64 = 0.9;
/// Past this, the path is unobserved: the duck may turn but not advance. Also how long a
/// confirmed hit is remembered.
pub const STALE_S: f64 = 1.5;

/// Trunk origin to the tip of the beak, metres. Avoidance thresholds mean the gap to the *front*
/// of the robot: without this a 0.15 m stop is behind the beak, and contact happens before it can
/// fire (the runtime's walked-into-the-wall bug).
pub const BEAK_M: f64 = 0.18;
/// Half the width of the corridor the body sweeps: the duck's half-width plus a margin.
pub const HALF_WIDTH_M: f64 = 0.12;
/// Returns higher than this above the floor are not in the way: the duck walks under tables.
pub const MAX_HEIGHT_M: f64 = 0.35;
/// Returns farther than this, horizontally from the sensor, are dropped. Avoidance never needs
/// them, and the long ranges are where an 8×8 sensor is noisiest.
pub const MAX_RANGE_M: f64 = 1.5;
/// A hit needs a neighbour this close (in the same frame or the previous one) to be believed.
pub const CONFIRM_M: f64 = 0.08;
/// Beams within this azimuth of straight ahead, in the trunk frame, observe the walking path.
const FORWARD_AZ_RAD: f64 = 0.3;
/// The path observation is discarded once the duck has turned or moved this much since it was
/// made: "ahead" no longer means what was seen. The prototype's numbers.
const TURNED_RAD: f64 = 0.45;
const MOVED_M: f64 = 0.5;
/// A floor seen earlier places a drop only if it lies on the drop's bearing from the sensor,
/// within this: floor seen during a glance says nothing about the path.
const ALONG_RAD: f64 = 0.1;
/// And only if the duck has walked less than this since it was seen.
const FLOOR_KEPT_M: f64 = 0.5;
const ZONES: usize = ROWS * COLS;

/// ST status bytes whose distance is believed: the theremin's set (`[theremin] statuses`).
/// ST documents only 5 and 9 as valid, and past ~30 cm a dark or moving target comes back 4 or 13
/// with a distance that is fine. The confirmation rule is what keeps the extra noise out.
pub const TRUSTED: [u8; 7] = [4, 5, 6, 9, 10, 12, 13];
/// "No target": the beam saw nothing in range. That is *observed clear*, not unknown.
const NO_TARGET: u8 = 255;

/// The brain's view of the path ahead, in the current body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    /// Gap from the beak to the nearest remembered hit in the walking corridor, metres;
    /// infinity when the corridor is clear.
    pub ahead_m: f64,
    /// Which way to turn to get away: +1 left, −1 right. Toward the clearer side.
    pub away: f64,
    /// Seconds since the walking path was last observed.
    pub age_s: f64,
}

/// What a frame is paired with: the robot at (nearly) the frame's instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// `CLOCK_MONOTONIC`, nanoseconds — the same clock as `TofFrame::t_ns`.
    pub t_ns: u64,
    /// `[neck_pitch, head_pitch, head_yaw, head_roll]`, measured.
    pub head: [f64; 4],
    pub gravity: [f64; 3],
    pub trunk_height_m: Option<f64>,
    /// Odometry `[x, y, yaw]`.
    pub pose: [f64; 3],
}

impl Sample {
    pub fn of(state: &proto::RobotState) -> Self {
        let joint = |name: &str| {
            proto::JOINT_NAMES
                .iter()
                .position(|n| *n == name)
                .and_then(|i| state.joints.get(i).copied())
                .unwrap_or(0.0)
        };
        Self {
            t_ns: state.t_ns,
            head: [
                joint("neck_pitch"),
                joint("head_pitch"),
                joint("head_yaw"),
                joint("head_roll"),
            ],
            gravity: state.safety.gravity,
            // Zero is "no estimate" (an unconverged estimator), not a buried trunk.
            trunk_height_m: (state.odom.position[2] > 0.02).then_some(state.odom.position[2]),
            pose: [
                state.odom.position[0],
                state.odom.position[1],
                state.odom.yaw,
            ],
        }
    }
}

/// The last second of samples, to pair each frame with the one nearest its capture. The latest
/// state is up to 66 ms off a 15 Hz frame, and during a head glance that is a lot of yaw.
pub struct History(VecDeque<Sample>);

impl Default for History {
    fn default() -> Self {
        Self(VecDeque::with_capacity(HISTORY))
    }
}

/// A second at 50 Hz.
const HISTORY: usize = 50;

impl History {
    pub fn push(&mut self, sample: Sample) {
        if self.0.len() == HISTORY {
            self.0.pop_front();
        }
        self.0.push_back(sample);
    }

    pub fn nearest(&self, t_ns: u64) -> Option<Sample> {
        self.0.iter().min_by_key(|s| s.t_ns.abs_diff(t_ns)).copied()
    }
}

/// Something in the way, in the odometry frame.
#[derive(Debug, Clone, Copy)]
struct Hazard {
    xy: [f64; 2],
    t_s: f64,
    drop: bool,
    /// Counts toward which way to turn. Not a drop put at the sensor, which is on one side of
    /// the trunk whichever way the edge runs.
    steer: bool,
}

/// Where the path was last observed from.
#[derive(Debug, Clone, Copy)]
struct Seen {
    t_s: f64,
    pose: [f64; 3],
}

pub struct Model {
    reprojector: Reprojector,
    hazards: Vec<Hazard>,
    /// The previous frame's hits, `[x, y, height]` in the odometry frame, for confirmation.
    previous: Vec<[f64; 3]>,
    /// Each zone's last floor from a judged beam, in the odometry frame, and where the duck was.
    floor_seen: [Option<([f64; 2], [f64; 3])>; ZONES],
    /// Which zones were drops last frame: a drop counts once it is seen twice.
    previous_drop: [bool; ZONES],
    seen: Option<Seen>,
}

impl Default for Model {
    fn default() -> Self {
        Self {
            reprojector: Reprojector::alpha(),
            hazards: Vec::new(),
            previous: Vec::new(),
            floor_seen: [None; ZONES],
            previous_drop: [false; ZONES],
            seen: None,
        }
    }
}

fn secs(t_ns: u64) -> f64 {
    t_ns as f64 * 1e-9
}

/// Body frame → odometry frame.
fn to_world(pose: [f64; 3], body: [f64; 2]) -> [f64; 2] {
    let (s, c) = pose[2].sin_cos();
    [
        pose[0] + c * body[0] - s * body[1],
        pose[1] + s * body[0] + c * body[1],
    ]
}

/// Odometry frame → body frame.
fn to_body(pose: [f64; 3], world: [f64; 2]) -> [f64; 2] {
    let (s, c) = pose[2].sin_cos();
    let (dx, dy) = (world[0] - pose[0], world[1] - pose[1]);
    [c * dx + s * dy, -s * dx + c * dy]
}

impl Model {
    /// Fold one frame in, seen from `at`.
    pub fn observe(&mut self, frame: &proto::TofFrame, at: &Sample) {
        let n = frame.distance_mm.len().min(frame.status.len());
        if n != kinematics::tof::ROWS * kinematics::tof::COLS {
            return;
        }
        let mut ranges = [None; kinematics::tof::ROWS * kinematics::tof::COLS];
        for (i, slot) in ranges.iter_mut().enumerate() {
            let mm = frame.distance_mm[i];
            if TRUSTED.contains(&frame.status[i]) && mm > 0 {
                *slot = Some(f64::from(mm) / 1000.0);
            }
        }
        let posture = Posture {
            gravity: at.gravity,
            trunk_height_m: at.trunk_height_m,
        };
        let zones = self.reprojector.project(&ranges, at.head, &posture);
        let sensor = self.reprojector.sensor_in_trunk(at.head);
        let t_s = secs(frame.t_ns);

        let mut forward = 0usize;
        let mut hits: Vec<[f64; 3]> = Vec::new();
        let mut drops: Vec<bool> = Vec::new();
        for (i, zone) in zones.iter().enumerate() {
            let dir = sensor.quat.rotate(self.reprojector.beams()[i]);
            let ahead = dir[0] > 0.0 && dir[1].atan2(dir[0]).abs() < FORWARD_AZ_RAD;
            // Observed means the beam came back with something to say: a range, the floor, or
            // a confident "nothing there". A failed status says nothing either way.
            let observed = !matches!(zone, Zone::Empty) || frame.status[i] == NO_TARGET;
            if ahead && observed {
                forward += 1;
            }
            match *zone {
                Zone::Hit {
                    point,
                    range,
                    height,
                } if height <= MAX_HEIGHT_M && range <= MAX_RANGE_M => {
                    let xy = to_world(at.pose, [point[0], point[1]]);
                    hits.push([xy[0], xy[1], height]);
                    drops.push(false);
                }
                _ => {}
            }
        }
        // Two beams, so one stray zone sweeping past the axis during a glance does not refresh a
        // path the head is not really looking at.
        if forward >= 2 {
            self.seen = Some(Seen { t_s, pose: at.pose });
        }

        // Drops: autonomous.md §5 step 3.
        let judged = self.reprojector.judged(at.head, &posture);
        let bearing = |p: [f64; 2]| (p[1] - sensor.pos[1]).atan2(p[0] - sensor.pos[0]);
        let mut drop_now = [false; ZONES];
        let mut unplaced: Vec<[f64; 3]> = Vec::new();
        for col in 0..COLS {
            let mut floor_here: Option<[f64; 2]> = None;
            for row in (0..ROWS).rev() {
                let i = row * COLS + col;
                match zones[i] {
                    Zone::Floor { point } if judged[i] => {
                        let xy = to_world(at.pose, [point[0], point[1]]);
                        floor_here = Some(xy);
                        self.floor_seen[i] = Some((xy, at.pose));
                    }
                    Zone::Drop { edge, .. }
                        if (edge[0] - sensor.pos[0]).hypot(edge[1] - sensor.pos[1])
                            <= MAX_RANGE_M =>
                    {
                        drop_now[i] = true;
                        if !(row..ROWS).any(|r| self.previous_drop[r * COLS + col]) {
                            continue;
                        }
                        let earlier = self.floor_seen[i]
                            .filter(|&(xy, from)| {
                                let b = to_body(at.pose, xy);
                                (from[0] - at.pose[0]).hypot(from[1] - at.pose[1]) < FLOOR_KEPT_M
                                    && b[0] <= edge[0]
                                    && crate::brain::wrap_angle(
                                        bearing(b) - bearing([edge[0], edge[1]]),
                                    )
                                    .abs()
                                        < ALONG_RAD
                            })
                            .map(|(xy, _)| xy);
                        match floor_here.or(earlier) {
                            Some(xy) => {
                                hits.push([xy[0], xy[1], 0.0]);
                                drops.push(true);
                            }
                            None => unplaced.push(edge),
                        }
                    }
                    _ => {}
                }
            }
        }
        self.previous_drop = drop_now;

        let close = |a: &[f64; 3], b: &[f64; 3]| {
            let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() <= CONFIRM_M
        };
        let confirmed: Vec<bool> = (0..hits.len())
            .map(|i| {
                (0..hits.len()).any(|j| j != i && close(&hits[i], &hits[j]))
                    || self.previous.iter().any(|other| close(&hits[i], other))
            })
            .collect();

        let mut stop_here = false;
        for edge in unplaced {
            if edge[0] <= 0.0 || edge[1].abs() >= HALF_WIDTH_M {
                continue;
            }
            let known = |xy: [f64; 2]| {
                let [x, y] = to_body(at.pose, xy);
                x > 0.0 && y.abs() < HALF_WIDTH_M && x <= edge[0]
            };
            let placed = (0..hits.len())
                .any(|i| drops[i] && confirmed[i] && known([hits[i][0], hits[i][1]]));
            // A drop still in the path keeps the one already known for it, or the stop lapses.
            let mut remembered = false;
            for h in self.hazards.iter_mut().filter(|h| h.drop && known(h.xy)) {
                h.t_s = t_s;
                remembered = true;
            }
            stop_here |= !placed && !remembered;
        }
        // Seen twice already, so it needs no second hit.
        if stop_here {
            self.hazards.push(Hazard {
                xy: to_world(at.pose, [sensor.pos[0], sensor.pos[1]]),
                t_s,
                drop: true,
                steer: false,
            });
        }

        for (i, hit) in hits.iter().enumerate() {
            if !confirmed[i] {
                continue;
            }
            let xy = [hit[0], hit[1]];
            // A drop seen again where one is kept refreshes it rather than piling up.
            if drops[i]
                && let Some(h) = self
                    .hazards
                    .iter_mut()
                    .find(|h| h.drop && (h.xy[0] - xy[0]).hypot(h.xy[1] - xy[1]) <= CONFIRM_M)
            {
                h.t_s = t_s;
                continue;
            }
            self.hazards.push(Hazard {
                xy,
                t_s,
                drop: drops[i],
                steer: true,
            });
        }
        self.previous = hits;
        self.hazards.retain(|h| t_s - h.t_s <= STALE_S);
    }

    /// The path ahead from `at`, now. `None` until the path has been observed at all.
    pub fn summary(&self, at: &Sample) -> Option<Obstacle> {
        let seen = self.seen?;
        let now = secs(at.t_ns);
        let turned = crate::brain::wrap_angle(at.pose[2] - seen.pose[2]).abs() > TURNED_RAD;
        let moved = (at.pose[0] - seen.pose[0]).hypot(at.pose[1] - seen.pose[1]) > MOVED_M;
        let age_s = if turned || moved {
            f64::INFINITY
        } else {
            (now - seen.t_s).max(0.0)
        };

        let mut ahead_m = f64::INFINITY;
        // The nearest hazard on each side within a wider fan, to know which way is clearer.
        let (mut left, mut right) = (f64::INFINITY, f64::INFINITY);
        for h in self.hazards.iter().filter(|h| now - h.t_s <= STALE_S) {
            let [x, y] = to_body(at.pose, h.xy);
            if x <= 0.0 {
                continue;
            }
            if y.abs() < HALF_WIDTH_M {
                ahead_m = ahead_m.min((x - BEAK_M).max(0.0));
            }
            if !h.steer {
                continue;
            }
            let d = x.hypot(y);
            if y >= 0.0 {
                left = left.min(d);
            } else {
                right = right.min(d);
            }
        }
        Some(Obstacle {
            ahead_m,
            away: if left >= right { 1.0 } else { -1.0 },
            age_s,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn upright(t_ns: u64, pose: [f64; 3]) -> Sample {
        Sample {
            t_ns,
            head: [0.0; 4],
            gravity: [0.0, 0.0, -1.0],
            trunk_height_m: None,
            pose,
        }
    }

    /// A frame where every zone says `status` at `mm`.
    fn frame(t_ns: u64, mm: i16, status: u8) -> proto::TofFrame {
        proto::TofFrame {
            seq: 0,
            at_us: 0,
            t_ns,
            rows: 8,
            cols: 8,
            distance_mm: vec![mm; 64],
            status: vec![status; 64],
        }
    }

    /// A frame of "no target" everywhere except a block of zones on the optical axis returning
    /// `mm` with a valid status — a wall patch or a chair back straight ahead.
    fn wall_ahead(t_ns: u64, mm: i16) -> proto::TofFrame {
        let mut f = frame(t_ns, 0, NO_TARGET);
        for row in 2..5 {
            for col in 3..5 {
                f.distance_mm[row * 8 + col] = mm;
                f.status[row * 8 + col] = 5;
            }
        }
        f
    }

    const MS: u64 = 1_000_000;

    /// Two frames a 15th of a second apart, from the same place: a drop counts once seen twice.
    fn twice(model: &mut Model, at: &Sample, frame: impl Fn(u64) -> proto::TofFrame) -> Sample {
        model.observe(&frame(at.t_ns), at);
        let next = Sample {
            t_ns: at.t_ns + 66 * MS,
            ..*at
        };
        model.observe(&frame(next.t_ns), &next);
        next
    }

    #[test]
    fn nothing_seen_is_no_summary() {
        let model = Model::default();
        assert_eq!(model.summary(&upright(0, [0.0; 3])), None);
    }

    /// 255 everywhere is a clear path, freshly seen — not "unknown", which is what stalled the
    /// runtime's duck in front of nothing.
    #[test]
    fn no_target_is_a_clear_path() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        model.observe(&frame(at.t_ns, 0, NO_TARGET), &at);
        let o = model.summary(&at).unwrap();
        assert_eq!(o.ahead_m, f64::INFINITY);
        assert!(o.age_s < 1e-9);
    }

    /// Failed statuses say nothing: they must neither refresh the path nor put a wall in it.
    #[test]
    fn failed_statuses_are_unknown_not_clear() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        model.observe(&frame(at.t_ns, 500, 2), &at);
        assert_eq!(model.summary(&at), None);
    }

    /// A wall half a metre ahead is in the corridor, measured from the beak.
    #[test]
    fn a_wall_ahead_is_measured_from_the_beak() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        model.observe(&wall_ahead(at.t_ns, 500), &at);
        let o = model.summary(&at).unwrap();
        assert!(
            (0.15..0.45).contains(&o.ahead_m),
            "ahead {} for a wall ~0.5 m from the head",
            o.ahead_m
        );
    }

    /// Steep forward zones of a level head, returning from `below` metres under the floor.
    fn floor_ahead(t_ns: u64, below: f64) -> proto::TofFrame {
        let rp = Reprojector::alpha();
        let sensor = rp.sensor_in_trunk([0.0; 4]);
        let above_floor = sensor.pos[2] + kinematics::Model::alpha().trunk_height_m();
        let mut f = frame(t_ns, 0, NO_TARGET);
        let mut steep = 0;
        for row in 0..8 {
            for col in 2..6 {
                let down = -sensor.quat.rotate(rp.beams()[row * 8 + col])[2];
                if down >= Reprojector::DROP_MIN_DOWNWARD {
                    f.distance_mm[row * 8 + col] = ((above_floor + below) / down * 1000.0) as i16;
                    f.status[row * 8 + col] = 5;
                    steep += 1;
                }
            }
        }
        assert!(steep >= 4, "a level head has steep forward zones");
        f
    }

    /// Steep forward zones of a level head at `pose`, with a step of `depth` where the world's x
    /// passes `edge_x`.
    fn edge_at(t_ns: u64, pose: [f64; 3], edge_x: f64, depth: f64) -> proto::TofFrame {
        edge_ahead(t_ns, pose, [0.0; 4], edge_x, depth)
    }

    /// [`edge_at`] across every column, not only the forward ones.
    fn edge_all(t_ns: u64, pose: [f64; 3], edge_x: f64, depth: f64) -> proto::TofFrame {
        edge_cols(t_ns, pose, [0.0; 4], 0..8, edge_x, depth)
    }

    /// [`edge_at`] for a head at `head`.
    fn edge_ahead(
        t_ns: u64,
        pose: [f64; 3],
        head: [f64; 4],
        edge_x: f64,
        depth: f64,
    ) -> proto::TofFrame {
        edge_cols(t_ns, pose, head, 2..6, edge_x, depth)
    }

    fn edge_cols(
        t_ns: u64,
        pose: [f64; 3],
        head: [f64; 4],
        cols: std::ops::Range<usize>,
        edge_x: f64,
        depth: f64,
    ) -> proto::TofFrame {
        let rp = Reprojector::alpha();
        let sensor = rp.sensor_in_trunk(head);
        let above_floor = sensor.pos[2] + kinematics::Model::alpha().trunk_height_m();
        let mut f = frame(t_ns, 0, NO_TARGET);
        for row in 0..8 {
            for col in cols.clone() {
                let dir = sensor.quat.rotate(rp.beams()[row * 8 + col]);
                let down = -dir[2];
                if down < Reprojector::DROP_MIN_DOWNWARD {
                    continue;
                }
                let at_floor = above_floor / down;
                let lands = to_world(
                    pose,
                    [
                        sensor.pos[0] + at_floor * dir[0],
                        sensor.pos[1] + at_floor * dir[1],
                    ],
                );
                let range = if lands[0] > edge_x {
                    (above_floor + depth) / down
                } else {
                    at_floor
                };
                f.distance_mm[row * 8 + col] = (range * 1000.0) as i16;
                f.status[row * 8 + col] = 5;
            }
        }
        f
    }

    /// Walks at a 10 cm step at `speed` until the gate stops; how far short of it the beak is.
    fn walk_at_an_edge(speed: f64) -> f64 {
        let mut model = Model::default();
        let edge_x = 1.5;
        let dt = 1.0 / 15.0;
        let mut x = 0.0;
        for k in 0..((2.0 / speed / dt) as u64) {
            let at = upright(1000 * MS + k * (dt * 1e9) as u64, [x, 0.0, 0.0]);
            model.observe(&edge_at(at.t_ns, at.pose, edge_x, 0.10), &at);
            if model.summary(&at).unwrap().ahead_m <= STOP_M {
                return edge_x - (x + BEAK_M);
            }
            x += speed * dt;
        }
        panic!("never stopped, at x {x:.2} with the edge at {edge_x}");
    }

    /// Walking at an edge at the pad's top speed, the duck stops short of it, two frames and all.
    #[test]
    fn a_duck_walking_at_an_edge_stops_before_it() {
        let short = walk_at_an_edge(0.3);
        assert!(
            (0.0..0.25).contains(&short),
            "{short:.3} m short of the edge"
        );
    }

    /// A slow approach still stops short of the edge.
    #[test]
    fn a_slow_approach_keeps_the_edge() {
        let short = walk_at_an_edge(0.01);
        assert!(
            (0.0..0.25).contains(&short),
            "{short:.3} m short of the edge"
        );
    }

    /// Setting off with every beam already past an edge, the duck stops at once.
    #[test]
    fn a_duck_setting_off_close_to_an_edge_stops_at_once() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [1.0, 0.0, 0.0]);
        let edge_x = 1.35;
        let at = twice(&mut model, &at, |t| edge_at(t, at.pose, edge_x, 0.10));
        let ahead = model.summary(&at).unwrap().ahead_m;
        assert!(
            ahead <= STOP_M,
            "ahead {ahead:.3} m with the edge {:.3} m past the beak",
            edge_x - 1.0 - BEAK_M
        );
    }

    /// Looking down, the edge goes past the floor the steeper rows saw, not at the sensor.
    #[test]
    fn the_floor_seen_short_of_an_edge_places_it() {
        let head = [0.0, 0.3, 0.0, 0.0];
        let at = Sample {
            head,
            ..upright(1000 * MS, [0.0; 3])
        };
        let rp = Reprojector::alpha();
        let sensor = rp.sensor_in_trunk(head);
        let above_floor = sensor.pos[2] + kinematics::Model::alpha().trunk_height_m();
        // An edge between the nearest and furthest landings, so some rows see floor.
        let lands: Vec<f64> = (0..64)
            .filter(|i| (2..6).contains(&(i % 8)))
            .map(|i| sensor.quat.rotate(rp.beams()[i]))
            .filter(|dir| -dir[2] >= Reprojector::DROP_MIN_DOWNWARD)
            .map(|dir| sensor.pos[0] + above_floor / -dir[2] * dir[0])
            .collect();
        let (near, far) = lands
            .iter()
            .fold((f64::MAX, f64::MIN), |(a, b), &x| (a.min(x), b.max(x)));
        assert!(far - near > 0.1, "rows to tell apart: {near:.2}..{far:.2}");
        let edge_x = (near + far) / 2.0;
        let mut model = Model::default();
        let at = twice(&mut model, &at, |t| {
            edge_ahead(t, at.pose, head, edge_x, 0.10)
        });
        let ahead = model.summary(&at).unwrap().ahead_m;
        assert!(
            ahead > 0.05 && ahead <= edge_x - BEAK_M + 0.01,
            "ahead {ahead:.3} m, the edge {:.3} m past the beak",
            edge_x - BEAK_M
        );
    }

    /// Every steep zone of a level head at `pose`, with a step of `depth` along the floor at
    /// `edge_y` to the left: a beam landing further left comes back from the lower floor.
    fn edge_beside(t_ns: u64, pose: [f64; 3], edge_y: f64, depth: f64) -> proto::TofFrame {
        let rp = Reprojector::alpha();
        let sensor = rp.sensor_in_trunk([0.0; 4]);
        let above_floor = sensor.pos[2] + kinematics::Model::alpha().trunk_height_m();
        let mut f = frame(t_ns, 0, NO_TARGET);
        for i in 0..64 {
            let dir = sensor.quat.rotate(rp.beams()[i]);
            let down = -dir[2];
            if down < Reprojector::DROP_MIN_DOWNWARD {
                continue;
            }
            let at_floor = above_floor / down;
            let lands_y = pose[1] + sensor.pos[1] + at_floor * dir[1];
            let range = if lands_y > edge_y {
                (above_floor + depth) / down
            } else {
                at_floor
            };
            f.distance_mm[i] = (range * 1000.0) as i16;
            f.status[i] = 5;
        }
        f
    }

    /// An edge beside the path does not stop a duck walking along it.
    #[test]
    fn an_edge_beside_the_path_does_not_stop_the_duck() {
        let mut model = Model::default();
        let (dt, speed) = (1.0 / 15.0, 0.2);
        let mut x = 0.0;
        for k in 0..60u64 {
            let at = upright(1000 * MS + k * (dt * 1e9) as u64, [x, 0.0, 0.0]);
            model.observe(&edge_beside(at.t_ns, at.pose, 0.2, 0.10), &at);
            let ahead = model.summary(&at).unwrap().ahead_m;
            assert!(
                ahead > STOP_M,
                "stopped at x {x:.2} with the edge to the side"
            );
            x += speed * dt;
        }
    }

    /// Standing at an edge keeps a handful of drops, not one more every frame.
    #[test]
    fn standing_at_an_edge_keeps_a_handful_of_drops() {
        let head = [0.0, 0.3, 0.0, 0.0];
        let mut model = Model::default();
        for k in 0..225u64 {
            let at = Sample {
                head,
                ..upright(1000 * MS + k * 66 * MS, [0.0; 3])
            };
            model.observe(&edge_ahead(at.t_ns, at.pose, head, 0.45, 0.10), &at);
        }
        assert!(model.hazards.len() <= 16, "{} kept", model.hazards.len());
    }

    /// One frame that reads a drop, say from a pitch jolt, stops nothing.
    #[test]
    fn a_drop_seen_once_stops_nothing() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        model.observe(&floor_ahead(at.t_ns, 0.10), &at);
        let next = upright(1066 * MS, [0.0; 3]);
        model.observe(&floor_ahead(next.t_ns, 0.0), &next);
        assert_eq!(model.summary(&next).unwrap().ahead_m, f64::INFINITY);
    }

    /// Stopped at an edge with a wall on its right, the duck turns left, away from the wall.
    #[test]
    fn a_stop_at_an_edge_turns_away_from_the_wall_beside_it() {
        let rp = Reprojector::alpha();
        let sensor = rp.sensor_in_trunk([0.0; 4]);
        let right: Vec<usize> = (0..64)
            .filter(|&i| {
                let dir = sensor.quat.rotate(rp.beams()[i]);
                dir[1] < -0.25 && dir[2].abs() < 0.2
            })
            .collect();
        assert!(right.len() >= 2, "a level head has zones to the right");
        let mut model = Model::default();
        let at = twice(&mut model, &upright(1000 * MS, [1.1, 0.0, 0.0]), |t| {
            let mut f = edge_at(t, [1.1, 0.0, 0.0], 1.3, 0.10);
            for &i in &right {
                f.distance_mm[i] = 400;
                f.status[i] = 5;
            }
            f
        });
        let o = model.summary(&at).unwrap();
        assert!(o.ahead_m <= STOP_M, "stopped: {}", o.ahead_m);
        assert_eq!(o.away, 1.0, "away from the wall on the right");
    }

    /// Floor seen during a glance does not vouch for the path: a lip ahead stops the duck at once.
    #[test]
    fn floor_seen_in_a_glance_does_not_place_an_edge_ahead() {
        let mut model = Model::default();
        let pose = [1.0, 0.0, 0.0];
        let glance = [0.0, 0.0, 0.9, 0.0];
        let at = Sample {
            head: glance,
            ..upright(1000 * MS, pose)
        };
        model.observe(&edge_ahead(at.t_ns, pose, glance, 5.0, 0.10), &at);
        let at = twice(&mut model, &upright(1066 * MS, pose), |t| {
            edge_at(t, pose, 1.0 + BEAK_M + 0.07, 0.10)
        });
        let ahead = model.summary(&at).unwrap().ahead_m;
        assert!(
            ahead <= STOP_M,
            "ahead {ahead:.3} m with a lip 7 cm past the beak"
        );
    }

    /// Turned away from an edge, the stop it set lets go: an edge beside the path keeps nothing.
    #[test]
    fn an_edge_left_beside_the_path_lets_the_duck_go() {
        let mut model = Model::default();
        let pose = [1.1, 0.0, 0.0];
        let at = twice(&mut model, &upright(1000 * MS, pose), |t| {
            edge_all(t, pose, 1.35, 0.10)
        });
        assert!(
            model.summary(&at).unwrap().ahead_m <= STOP_M,
            "stopped facing the edge"
        );
        let turned = [1.1, 0.0, 80f64.to_radians()];
        let mut last = at;
        for k in 1..=40u64 {
            last = upright(at.t_ns + k * 66 * MS, turned);
            model.observe(&edge_all(last.t_ns, turned, 1.35, 0.10), &last);
        }
        assert_eq!(model.summary(&last).unwrap().ahead_m, f64::INFINITY);
    }

    /// Standing at a lip, the stop holds for as long as the edge is in the path, with no gap a
    /// creeping turn or a brace could step through.
    #[test]
    fn a_stop_at_an_edge_holds_while_the_edge_is_in_the_path() {
        let mut model = Model::default();
        let pose = [1.0, 0.0, 0.0];
        let edge_x = 1.0 + BEAK_M + 0.05;
        let at = twice(&mut model, &upright(1000 * MS, pose), |t| {
            edge_at(t, pose, edge_x, 0.10)
        });
        for k in 1..=450u64 {
            let now = upright(at.t_ns + k * 66 * MS, pose);
            model.observe(&edge_at(now.t_ns, pose, edge_x, 0.10), &now);
            let ahead = model.summary(&now).unwrap().ahead_m;
            assert!(
                ahead <= STOP_M,
                "the stop lapsed after {k} frames: ahead {ahead}"
            );
        }
    }

    /// Flat floor seen by the same zones is clear.
    #[test]
    fn the_flat_floor_is_not_a_drop() {
        let mut model = Model::default();
        let at = twice(&mut model, &upright(1000 * MS, [0.0; 3]), |t| {
            floor_ahead(t, 0.0)
        });
        assert_eq!(model.summary(&at).unwrap().ahead_m, f64::INFINITY);
    }

    /// One noisy zone on its own stops nothing.
    #[test]
    fn a_lone_zone_is_not_believed() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        let mut f = frame(at.t_ns, 0, NO_TARGET);
        f.distance_mm[3 * 8 + 3] = 400;
        f.status[3 * 8 + 3] = 5;
        model.observe(&f, &at);
        assert_eq!(model.summary(&at).unwrap().ahead_m, f64::INFINITY);
    }

    /// A remembered wall gets closer as the duck walks at it, with no new frame — and a later
    /// frame that fails to see it again does not erase it.
    #[test]
    fn a_remembered_wall_gets_closer_and_survives_a_miss() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        model.observe(&wall_ahead(at.t_ns, 600), &at);
        let first = model.summary(&at).unwrap().ahead_m;

        let later = upright(1300 * MS, [0.2, 0.0, 0.0]);
        model.observe(&frame(later.t_ns, 0, NO_TARGET), &later);
        let closer = model.summary(&later).unwrap().ahead_m;
        assert!(
            (first - closer - 0.2).abs() < 0.02,
            "first {first}, after 0.2 m {closer}"
        );

        // And forgotten once it has gone unseen for longer than STALE_S.
        let much_later = upright(1000 * MS + 1600 * MS, [0.2, 0.0, 0.0]);
        model.observe(&frame(much_later.t_ns, 0, NO_TARGET), &much_later);
        assert_eq!(model.summary(&much_later).unwrap().ahead_m, f64::INFINITY);
    }

    /// Turning away invalidates what "ahead" was observed to be.
    #[test]
    fn turning_makes_the_path_unobserved() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        model.observe(&frame(at.t_ns, 0, NO_TARGET), &at);
        let turned = upright(1100 * MS, [0.0, 0.0, 0.8]);
        assert_eq!(model.summary(&turned).unwrap().age_s, f64::INFINITY);
    }

    /// The head looking hard to one side is not looking at the path.
    #[test]
    fn a_head_turned_away_does_not_refresh_the_path() {
        let mut model = Model::default();
        let mut at = upright(1000 * MS, [0.0; 3]);
        at.head[2] = 1.0;
        model.observe(&frame(at.t_ns, 0, NO_TARGET), &at);
        assert_eq!(model.summary(&at), None);
    }

    /// Something to the right means turn left.
    #[test]
    fn the_clear_side_is_away_from_the_obstacle() {
        let mut model = Model::default();
        let at = upright(1000 * MS, [0.0; 3]);
        let mut f = frame(at.t_ns, 0, NO_TARGET);
        // Columns 6 and 7 are the sensor's right (column 0 is its left).
        for row in 3..5 {
            for col in 6..8 {
                f.distance_mm[row * 8 + col] = 400;
                f.status[row * 8 + col] = 5;
            }
        }
        model.observe(&f, &at);
        assert_eq!(model.summary(&at).unwrap().away, 1.0);
    }

    #[test]
    fn frames_pair_with_the_nearest_sample() {
        let mut history = History::default();
        for k in 0..60u64 {
            history.push(upright(k * 20 * MS, [k as f64, 0.0, 0.0]));
        }
        let nearest = history.nearest(41 * 20 * MS + 3 * MS).unwrap();
        assert_eq!(nearest.pose[0], 41.0);
    }
}
