//! Laser tracking: a red dot on the floor, followed with the head and walked to.
//!
//! The camera is `mediad`'s, so the dot is found there and arrives here as a *direction* — a ray
//! in the upright camera's frame (`mediad::laser` says why that is the right thing to cross a
//! socket). This module is the rest of it, and it is all geometry the loop already has: the head
//! FK puts the ray in the trunk, the IMU turns the trunk into the world, odometry says where the
//! trunk is above the floor, and the ray meets the floor at the dot.
//!
//! **The target is a place, not a pixel.** The prototype's first version servoed the head on the
//! pixel error and either crawled or oscillated, depending on the gain; servoing on where the dot
//! *is* — then asking the head IK to look there — has no gain to tune at all, because the answer
//! is the head angle. The same place, carried by odometry, is what the legs walk to, and it is
//! what keeps being followed for `memory_ms` after the dot leaves the picture.
//!
//! **Three rates meet here**, as in the theremin: `mediad` looks at `search_hz` or `track_hz`, the
//! loop ticks at 50 Hz, and the reader thread sits between them on the socket. The loop never
//! waits: it loads the newest dot from a slot and projects it once, with the joints and attitude
//! of the tick that read it — 50 to 100 ms later than the frame, which at the head's speed is a
//! degree or two and not worth a history buffer yet.
//!
//! **It shares the robot.** What it produces is a head pose and a twist; the loop gives each to the
//! dot only while no client is asking for something else (`main.rs` says how), so a pad's stick
//! always wins, and with no dot in sight this module asks for nothing.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use arc_swap::{ArcSwap, ArcSwapOption};
use duck_ipc_proto as proto;
use kinematics::Quat;
use kinematics::head::HeadFk;
use robotd_params::LaserParams;

/// How fast the reader thread retries a `mediad` that is not answering.
const RECONNECT: Duration = Duration::from_secs(2);

/// How often the reader wakes when nothing arrives — to notice tracking being switched off, or
/// the params changing, without waiting on a socket that may stay quiet.
const WAKE: Duration = Duration::from_millis(300);

/// How often `[laser]` is checked for edits.
const RELOAD: Duration = Duration::from_secs(1);

/// A dot older than this is not news: `mediad` says something every look, even "nothing", so a
/// silence this long is a connection that has stalled. Two and a half looks at the default
/// `search_hz` of 1 — a look can wait up to half a second for its frame — not one.
const DOT_STALE: Duration = Duration::from_millis(2500);

/// Request id for the subscription. Any number; `mediad` echoes it.
const SUBSCRIBE_ID: u64 = 1;

/// One answer from `mediad`, stamped on arrival.
struct Sighting {
    dot: proto::LaserDot,
    at: Instant,
    /// Bumped per arrival, so the loop projects each dot once.
    seq: u64,
}

/// What the reader thread and the loop share.
struct Shared {
    active: AtomicBool,
    latest: ArcSwapOption<Sighting>,
    /// The camera side, as a line for `robot.state`.
    camera: ArcSwap<String>,
    params: ArcSwap<LaserParams>,
    seq: AtomicU64,
}

/// What the loop knows this tick that the projection needs.
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    /// `[neck_pitch, head_pitch, head_yaw, head_roll]`, measured.
    pub head: [f64; 4],
    /// Trunk → world, `[w, x, y, z]`.
    pub imu: [f64; 4],
    /// Trunk position in the world, metres; z is height above the floor.
    pub odom: [f64; 3],
    /// Trunk heading in the world, radians.
    pub yaw: f64,
}

/// What tracking asks of the loop this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Drive {
    /// A head pose that looks at the dot, when `[laser] head` is on and there is a dot.
    pub head: Option<[f64; 4]>,
    /// A twist toward the dot, when `[laser] walk` is on and there is a dot to walk to.
    pub twist: Option<[f64; 3]>,
    /// The block for `robot.state`. The loop sets `head`/`walk` once it knows which of the two
    /// it actually took.
    pub state: proto::LaserTrackState,
}

pub struct Laser {
    shared: Arc<Shared>,
    fk: HeadFk,
    /// The last projected dot, world x/y, and when it was seen.
    target: Option<([f64; 2], Instant)>,
    /// The last sighting projected, by [`Sighting::seq`].
    projected: u64,
    /// The floor's height in the world, from the trunk height when odometry has none yet.
    trunk_height: f64,
}

impl Laser {
    /// Start the reader thread. It holds no connection while tracking is off.
    pub fn spawn(params: LaserParams, config: PathBuf) -> Self {
        let laser = Self::new(params);
        let shared = laser.shared.clone();
        if std::thread::Builder::new()
            .name("laser-reader".into())
            .spawn(move || read(&shared, &config))
            .is_err()
        {
            tracing::warn!("cannot spawn the laser reader; no laser tracking");
        }
        laser
    }

    /// No reader behind it: dots are whatever is put in the slot. For the tests, for the
    /// theremin's reason — a reader dialling a socket nobody answers would race them.
    fn new(params: LaserParams) -> Self {
        let active = params.enabled;
        Self {
            shared: Arc::new(Shared {
                active: AtomicBool::new(active),
                latest: ArcSwapOption::empty(),
                camera: ArcSwap::from_pointee("starting".to_owned()),
                params: ArcSwap::from_pointee(params),
                seq: AtomicU64::new(0),
            }),
            fk: HeadFk::alpha(),
            target: None,
            projected: 0,
            trunk_height: kinematics::Model::alpha().trunk_height_m(),
        }
    }

    pub fn set_active(&mut self, active: bool) {
        if active != self.shared.active.swap(active, Ordering::Relaxed) {
            tracing::warn!(active, "laser tracking");
        }
        if !active {
            self.target = None;
        }
    }

    pub fn active(&self) -> bool {
        self.shared.active.load(Ordering::Relaxed)
    }

    /// One tick. Never blocks; `None` while tracking is off.
    pub fn tick(&mut self, pose: &Pose, neck_pitch: f64, now: Instant) -> Option<Drive> {
        if !self.active() {
            return None;
        }
        let params = self.shared.params.load_full();
        let sighting = self.shared.latest.load_full();
        let fresh = sighting
            .as_ref()
            .filter(|s| now.saturating_duration_since(s.at) < DOT_STALE);

        if let Some(s) = fresh
            && s.seq != self.projected
        {
            self.projected = s.seq;
            if let Some(ray) = s.dot.ray
                && let Some(at) = self.project(ray, pose, &params)
            {
                self.target = Some((at, s.at));
            }
        }

        let memory = Duration::from_millis(params.memory_ms);
        let target = self
            .target
            .filter(|(_, seen)| now.saturating_duration_since(*seen) < memory);
        if target.is_none() {
            self.target = None;
        }

        let mut state = proto::LaserTrackState {
            camera: self.shared.camera.load().as_ref().clone(),
            dot: fresh.map(|s| s.dot.clone()),
            age_ms: self
                .target
                .map(|(_, seen)| now.saturating_duration_since(seen).as_millis() as u64),
            ..Default::default()
        };
        let Some((world, _)) = target else {
            return Some(Drive {
                head: None,
                twist: None,
                state,
            });
        };

        // The dot in the robot's heading frame: the world offset turned by the heading alone,
        // because walking is planar and the trunk's pitch is not a direction to walk in.
        let (wx, wy) = (world[0] - pose.odom[0], world[1] - pose.odom[1]);
        let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
        let ahead = [wx * c + wy * s, -wx * s + wy * c];
        state.target_m = Some(ahead);

        let head = params.head.then(|| {
            // Back into the trunk, tilt and all, for the IK: the floor point relative to the
            // trunk origin in the world, rotated by the inverse attitude.
            let floor = self.floor(pose, &params);
            let rel = [wx, wy, floor - pose.odom[2]];
            let in_trunk = imu_quat(pose.imu).conjugate().rotate(rel);
            self.fk.look_at(in_trunk, neck_pitch).joints
        });
        let twist = params.walk.then(|| walk_to(ahead, &params));
        state.twist = twist;
        Some(Drive { head, twist, state })
    }

    /// Where the floor is, in the world's z.
    ///
    /// Odometry's z is the trunk's height above the plane its contact anchor defines, so the floor
    /// is at zero — once odometry has a fix. Before it does (the IMU still converging) the trunk is
    /// assumed at its trained standing height.
    fn floor(&self, pose: &Pose, params: &LaserParams) -> f64 {
        let base = if pose.odom[2] > 0.05 {
            0.0
        } else {
            pose.odom[2] - self.trunk_height
        };
        base + params.floor_offset_m
    }

    /// The ray `[x, y, 1]` from the camera, through the head FK and the attitude, onto the
    /// floor — world x/y, or `None` when it does not meet the floor in front of the robot within
    /// `max_range_m`.
    fn project(&self, ray: [f64; 2], pose: &Pose, params: &LaserParams) -> Option<[f64; 2]> {
        let camera = self.fk.camera_in_trunk_cv2(pose.head);
        let attitude = imu_quat(pose.imu);
        let dir = attitude.rotate(camera.quat.rotate([ray[0], ray[1], 1.0]));
        let offset = attitude.rotate(camera.pos);
        let origin = [
            pose.odom[0] + offset[0],
            pose.odom[1] + offset[1],
            pose.odom[2] + offset[2],
        ];
        let drop = self.floor(pose, params) - origin[2];
        // Looking down at all, and the floor below the lens: anything else is a dot on a wall, or
        // a ray so shallow that a pixel of error is metres of floor.
        if dir[2] > -1e-3 || drop >= 0.0 {
            return None;
        }
        let t = drop / dir[2];
        let at = [origin[0] + t * dir[0], origin[1] + t * dir[1]];
        let range = (at[0] - pose.odom[0]).hypot(at[1] - pose.odom[1]);
        (range <= params.max_range_m).then_some(at)
    }
}

/// The twist that takes the robot to a dot at `ahead` (+x ahead, +y left, metres).
///
/// Turn toward it in proportion to the bearing; walk in proportion to the distance left past
/// `stop_m`, scaled by how squarely it is faced — so a dot behind the shoulder is turned to
/// before it is walked to, rather than walked past.
fn walk_to(ahead: [f64; 2], p: &LaserParams) -> [f64; 3] {
    let distance = ahead[0].hypot(ahead[1]);
    let bearing = ahead[1].atan2(ahead[0]);
    let vyaw = (p.k_yaw * bearing).clamp(-p.vyaw_max, p.vyaw_max);
    let facing = bearing.cos().max(0.0);
    let vx = (p.k_dist * (distance - p.stop_m) * facing).clamp(0.0, p.vx_max);
    // Parked: close enough that the dot is under the beak, where its bearing is mostly noise.
    if distance < p.stop_m {
        return [0.0, 0.0, 0.0];
    }
    [vx, 0.0, vyaw]
}

fn imu_quat([w, x, y, z]: [f64; 4]) -> Quat {
    Quat::new(w, x, y, z).normalized()
}

/// The reader: while tracking is on, hold a `media.laser` subscription and leave the newest dot
/// in the slot. Also where `[laser]` is re-read, since this thread is awake anyway.
fn read(shared: &Shared, config: &Path) {
    let mut stamp = robotd_params::config_stamp(config);
    let mut checked = Instant::now();
    loop {
        reload(shared, config, &mut stamp, &mut checked);
        if !shared.active.load(Ordering::Relaxed) {
            shared.latest.store(None);
            shared.camera.store(Arc::new("off".to_owned()));
            std::thread::sleep(WAKE);
            continue;
        }
        let why = subscribe(shared, config, &mut stamp, &mut checked);
        shared.latest.store(None);
        tracing::info!(why, "laser: the subscription ended");
        // A switch-off or a retune reconnects at once; a broken mediad is retried slowly.
        let retune = why.starts_with("retuning");
        shared.camera.store(Arc::new(why));
        if shared.active.load(Ordering::Relaxed) && !retune {
            std::thread::sleep(RECONNECT);
        }
    }
}

/// Pick up edits to `[laser]`. A file that will not load keeps the params running and says so.
fn reload(
    shared: &Shared,
    config: &Path,
    stamp: &mut robotd_params::ConfigStamp,
    checked: &mut Instant,
) -> bool {
    if checked.elapsed() < RELOAD {
        return false;
    }
    *checked = Instant::now();
    let now = robotd_params::config_stamp(config);
    if now == *stamp {
        return false;
    }
    *stamp = now;
    match robotd_params::Params::load(config, false) {
        Ok(params) if params.laser != **shared.params.load() => {
            tracing::warn!(laser = ?params.laser, "laser: [laser] changed, applying");
            // `enabled` is the boot state — and an edit to it is somebody switching tracking, so
            // it switches now rather than at the next restart. `robotctl laser-track` switches
            // without writing, so the two only meet when the file is the one that moved.
            if params.laser.enabled != shared.params.load().enabled {
                shared.active.store(params.laser.enabled, Ordering::Relaxed);
            }
            shared.params.store(Arc::new(params.laser));
            true
        }
        Ok(_) => false,
        Err(error) => {
            tracing::warn!(error = %error, "laser: the config will not load; keeping the running [laser]");
            false
        }
    }
}

/// One subscription, from the request to whatever ended it — returned as the line to show.
fn subscribe(
    shared: &Shared,
    config: &Path,
    stamp: &mut robotd_params::ConfigStamp,
    checked: &mut Instant,
) -> String {
    let params = shared.params.load_full();
    let detect = params.detect();
    let socket = &params.socket;
    let stream = match UnixStream::connect(socket) {
        Ok(stream) => stream,
        Err(e) => return format!("mediad is not answering on {}: {e}", socket.display()),
    };
    if let Err(e) = stream.set_read_timeout(Some(WAKE)) {
        return format!("socket: {e}");
    }
    let mut writer = match stream.try_clone() {
        Ok(w) => w,
        Err(e) => return format!("socket: {e}"),
    };
    let request = serde_json::json!({
        "jsonrpc": "2.0",
        "id": SUBSCRIBE_ID,
        "method": proto::method::MEDIA_LASER,
        "params": detect,
    });
    if let Err(e) = writer.write_all(format!("{request}\n").as_bytes()) {
        return format!("subscribe: {e}");
    }

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    let mut answered = false;
    loop {
        if !shared.active.load(Ordering::Relaxed) {
            return "off".to_owned();
        }
        if reload(shared, config, stamp, checked) && shared.params.load().detect() != detect {
            return "retuning: [laser] changed".to_owned();
        }
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => return "mediad closed the subscription".to_owned(),
            Ok(_) => {}
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                // A partial line stays in `line` only if read_line was mid-line, which a
                // timeout between notifications is not; nothing to keep.
                continue;
            }
            Err(e) => return format!("read: {e}"),
        }
        if !answered {
            answered = true;
            if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
                && let Some(error) = response.error
            {
                // An older mediad does not know the method: that is the install consequence to
                // name, not a reason to keep retrying in silence.
                return if error.code == proto::code::METHOD_NOT_FOUND {
                    "this mediad has no laser detector — update the robot".to_owned()
                } else {
                    format!("mediad refused: {}", error.message)
                };
            }
            shared.camera.store(Arc::new(format!(
                "looking at {:.0}–{:.0} Hz",
                detect.search_hz, detect.track_hz
            )));
            continue;
        }
        let Ok(note) = serde_json::from_str::<proto::Request>(&line) else {
            continue;
        };
        if note.method != proto::method::MEDIA_LASER_DOT {
            continue;
        }
        let Some(Ok(dot)) = note.params.map(serde_json::from_value::<proto::LaserDot>) else {
            continue;
        };
        shared.camera.store(Arc::new(format!(
            "looking at {:.0} Hz, {:.1} ms a look",
            dot.hz, dot.took_ms
        )));
        let seq = shared.seq.fetch_add(1, Ordering::Relaxed) + 1;
        shared.latest.store(Some(Arc::new(Sighting {
            dot,
            at: Instant::now(),
            seq,
        })));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn laser() -> Laser {
        Laser::new(LaserParams {
            enabled: true,
            ..Default::default()
        })
    }

    fn push(l: &Laser, ray: [f64; 2]) {
        let seq = l.shared.seq.fetch_add(1, Ordering::Relaxed) + 1;
        l.shared.latest.store(Some(Arc::new(Sighting {
            dot: proto::LaserDot {
                seen: true,
                ray: Some(ray),
                ..Default::default()
            },
            at: Instant::now(),
            seq,
        })));
    }

    /// Standing upright at the origin, heading +x, odometry at the trained trunk height.
    fn standing() -> Pose {
        Pose {
            head: [0.0; 4],
            imu: [1.0, 0.0, 0.0, 0.0],
            odom: [0.0, 0.0, kinematics::Model::alpha().trunk_height_m()],
            yaw: 0.0,
        }
    }

    /// The ray through the image centre with the head level looks at the horizon and never meets
    /// the floor — and a ray tipped down does, ahead of the robot.
    #[test]
    fn a_downward_ray_lands_on_the_floor_ahead() {
        let mut l = laser();
        push(&l, [0.0, 0.0]);
        let drive = l.tick(&standing(), 0.0, Instant::now()).expect("active");
        assert_eq!(
            drive.state.target_m, None,
            "the horizon is not on the floor"
        );

        // 0.3 down in cv2 (+y is down in the upright picture).
        push(&l, [0.0, 0.3]);
        let drive = l.tick(&standing(), 0.0, Instant::now()).expect("active");
        let [x, y] = drive.state.target_m.expect("on the floor");
        assert!(x > 0.2 && x < 2.5, "ahead: {x}");
        assert!(y.abs() < 0.05, "straight ahead: {y}");
    }

    /// **The sign that matters on a robot.** A dot to the right of the upright picture (+x in
    /// cv2) is to the robot's right (−y), and the twist turns that way.
    #[test]
    fn a_dot_on_the_right_is_turned_to_on_the_right() {
        let mut l = laser();
        push(&l, [0.3, 0.3]);
        let drive = l.tick(&standing(), 0.0, Instant::now()).expect("active");
        let [_, y] = drive.state.target_m.expect("on the floor");
        assert!(y < -0.05, "right is -y: {y}");
        let twist = drive.twist.expect("walking");
        assert!(twist[2] < 0.0, "turn right: {twist:?}");
        // And the head looks right too: +head_yaw looks left on this robot.
        let head = drive.head.expect("looking");
        assert!(head[2] < 0.0, "head yaw right: {head:?}");
    }

    /// The target is a place: once seen, the robot turning does not move it in the world, only
    /// where it is relative to the robot.
    #[test]
    fn the_target_stays_put_while_the_robot_turns() {
        let mut l = laser();
        push(&l, [0.0, 0.3]);
        let before = l.tick(&standing(), 0.0, Instant::now()).unwrap();
        let [x, _] = before.state.target_m.unwrap();
        // The robot turned 90° left; nothing new seen.
        let turned = Pose {
            yaw: std::f64::consts::FRAC_PI_2,
            ..standing()
        };
        let after = l.tick(&turned, 0.0, Instant::now()).unwrap();
        let [ax, ay] = after.state.target_m.unwrap();
        assert!(ax.abs() < 1e-4, "{ax}");
        assert!((ay + x).abs() < 1e-4, "now on the right: {ay} vs {x}");
    }

    /// A dot forgotten is not followed: after `memory_ms` with nothing seen, the robot is given
    /// back to whoever else is driving it.
    #[test]
    fn a_lost_dot_is_let_go_after_the_memory() {
        let mut l = Laser::new(LaserParams {
            enabled: true,
            memory_ms: 50,
            ..Default::default()
        });
        push(&l, [0.0, 0.3]);
        assert!(
            l.tick(&standing(), 0.0, Instant::now())
                .unwrap()
                .twist
                .is_some()
        );
        let later = Instant::now() + Duration::from_millis(60);
        let drive = l.tick(&standing(), 0.0, later).unwrap();
        assert_eq!(drive.twist, None);
        assert_eq!(drive.head, None);
    }

    /// Off is off: nothing asked of the loop at all.
    #[test]
    fn switched_off_asks_for_nothing() {
        let mut l = laser();
        push(&l, [0.0, 0.3]);
        l.set_active(false);
        assert_eq!(l.tick(&standing(), 0.0, Instant::now()), None);
    }

    /// Parked on the dot: close in, the twist is zero rather than a turn chasing noise.
    #[test]
    fn close_enough_is_a_stop() {
        let p = LaserParams::default();
        assert_eq!(walk_to([0.1, 0.05], &p), [0.0, 0.0, 0.0]);
        // Far and dead ahead walks, at most vx_max, without turning.
        let [vx, _, vyaw] = walk_to([2.0, 0.0], &p);
        assert!((vx - p.vx_max).abs() < 1e-9 && vyaw.abs() < 1e-9);
        // Behind the shoulder turns first and does not walk.
        let [vx, _, vyaw] = walk_to([-0.5, 0.5], &p);
        assert_eq!(vx, 0.0);
        assert!(vyaw > 0.0);
    }
}
