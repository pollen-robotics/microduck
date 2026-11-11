//! Autonomous soccer: find the purple ball, walk to it, line a foot up, kick it, look again.
//!
//! Soccer mode is two switches, both [`crate::intents::Intents`]' levels, set by `robot.soccer`:
//! *soccer mode*, which moves nothing here — it is what a pad daemon reads to put the kicks on the
//! bumpers — and *autonomous*, which is this module. The switches are levels rather than the
//! laser's edges because nothing in the config file starts soccer: it is a game somebody starts,
//! with an NFC tag or a pad.
//!
//! **It is laser tracking with a different target.** The ball arrives from `mediad`'s
//! `media.ball` as a direction, exactly as the laser dot does, and becomes a place by the same
//! projection — onto the plane a ball's radius above the floor, where its centre is, rather than
//! onto the floor ([`crate::laser::project`]). The place is carried by odometry, which is what
//! makes the last steps work: the ball goes under the beak and out of the picture while the robot
//! is lining up, and it is the remembered place, not a pixel, that the foot is put beside.
//!
//! **What it does, tick by tick:**
//!
//! | | twist | head |
//! |---|---|---|
//! | no ball known | turn in place at `search_vyaw` | the loop's own |
//! | ball beyond `approach_m` | turn to it and walk at it | on the ball |
//! | nearer | walk forwards and sideways until it sits at (`kick_x_m`, ±`kick_y_m`) | on the ball |
//! | there, for `settle_ms` | still, then `kick_left`/`kick_right` through `robot.do`'s queue | on the ball |
//! | after a kick, for `cooldown_ms` | nothing asked | the loop's own |
//!
//! The foot is chosen once, on entering `approach_m` — the side the ball is on — and kept until
//! the kick, so a ball near the centre line does not make the robot dither between feet.
//!
//! **It shares the robot as the laser does** (`main.rs` arbitrates): a deflected client stick has
//! the legs, a client head pose has the head. A kick is only queued while the policy is driving.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use arc_swap::{ArcSwap, ArcSwapOption};
use duck_ipc_proto as proto;
use kinematics::head::HeadFk;
use robotd_params::SoccerParams;

use crate::laser::{self, Pose};

/// How fast the reader retries a `mediad` that is not answering.
const RECONNECT: Duration = Duration::from_secs(2);
/// How often the reader wakes when nothing arrives, to notice autonomous being switched off.
const WAKE: Duration = Duration::from_millis(300);
/// How often `[soccer]` is checked for edits.
const RELOAD: Duration = Duration::from_secs(1);
/// A look older than this is a stalled connection, not news — as the laser's `DOT_STALE`.
const SEEN_STALE: Duration = Duration::from_millis(2500);
const SUBSCRIBE_ID: u64 = 1;

/// The ball's diameter over its apparent width is its distance; this is the diameter.
fn diameter(params: &SoccerParams) -> f64 {
    2.0 * params.ball_radius_m
}

struct Sighting {
    seen: proto::BallSeen,
    at: Instant,
    seq: u64,
}

struct Shared {
    autonomous: AtomicBool,
    latest: ArcSwapOption<Sighting>,
    camera: ArcSwap<String>,
    params: ArcSwap<SoccerParams>,
    seq: AtomicU64,
}

/// The foot a kick is lined up for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Foot {
    Left,
    Right,
}

impl Foot {
    /// The skill that kicks with it — the names `robot.do` takes.
    pub fn skill(self) -> &'static str {
        match self {
            Foot::Left => "kick_left",
            Foot::Right => "kick_right",
        }
    }

    fn name(self) -> &'static str {
        match self {
            Foot::Left => "left",
            Foot::Right => "right",
        }
    }

    /// Which side of the trunk the ball sits on for this foot's kick: `kick_y_m` for the left,
    /// its mirror for the right.
    fn side(self) -> f64 {
        match self {
            Foot::Left => 1.0,
            Foot::Right => -1.0,
        }
    }
}

/// What autonomous soccer asks of the loop this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Drive {
    pub head: Option<[f64; 4]>,
    pub twist: Option<[f64; 3]>,
    /// A kick to queue this tick, by skill name.
    pub kick: Option<&'static str>,
    /// The block for `robot.state`; the loop sets `head`/`walk` once it knows what it took.
    pub state: proto::SoccerState,
}

pub struct Soccer {
    shared: Arc<Shared>,
    fk: HeadFk,
    trunk_height: f64,
    /// The ball's centre, world x/y, and when it was last seen.
    target: Option<([f64; 2], Instant)>,
    projected: u64,
    foot: Option<Foot>,
    /// Since when the ball has sat in the kick spot.
    settled: Option<Instant>,
    /// When the last kick was queued.
    kicked: Option<Instant>,
    kicks: u32,
}

impl Soccer {
    /// Start the reader thread. It holds no connection while autonomous soccer is off.
    pub fn spawn(params: SoccerParams, config: PathBuf) -> Self {
        let soccer = Self::new(params);
        let shared = soccer.shared.clone();
        if std::thread::Builder::new()
            .name("ball-reader".into())
            .spawn(move || read(&shared, &config))
            .is_err()
        {
            tracing::warn!("cannot spawn the ball reader; no autonomous soccer");
        }
        soccer
    }

    fn new(params: SoccerParams) -> Self {
        Self {
            shared: Arc::new(Shared {
                autonomous: AtomicBool::new(false),
                latest: ArcSwapOption::empty(),
                camera: ArcSwap::from_pointee("off".to_owned()),
                params: ArcSwap::from_pointee(params),
                seq: AtomicU64::new(0),
            }),
            fk: HeadFk::alpha(),
            trunk_height: kinematics::Model::alpha().trunk_height_m(),
            target: None,
            projected: 0,
            foot: None,
            settled: None,
            kicked: None,
            kicks: 0,
        }
    }

    /// Switch autonomous soccer. Switching it on starts from nothing: no remembered ball, no foot,
    /// no kicks counted.
    pub fn set_autonomous(&mut self, on: bool) {
        if on != self.shared.autonomous.swap(on, Ordering::Relaxed) {
            tracing::warn!(autonomous = on, "soccer");
            self.target = None;
            self.foot = None;
            self.settled = None;
            self.kicked = None;
            if on {
                self.kicks = 0;
            }
        }
    }

    pub fn autonomous(&self) -> bool {
        self.shared.autonomous.load(Ordering::Relaxed)
    }

    /// One tick. Never blocks; `None` while autonomous soccer is off. `driving` is whether the
    /// policy is walking the robot — a kick is queued only then.
    pub fn tick(
        &mut self,
        pose: &Pose,
        neck_pitch: f64,
        driving: bool,
        now: Instant,
    ) -> Option<Drive> {
        if !self.autonomous() {
            return None;
        }
        let params = self.shared.params.load_full();
        let sighting = self.shared.latest.load_full();
        let fresh = sighting
            .as_ref()
            .filter(|s| now.saturating_duration_since(s.at) < SEEN_STALE);
        let plane =
            laser::floor_z(pose, self.trunk_height) + params.floor_offset_m + params.ball_radius_m;

        // A kick in flight: ask for nothing, and forget the ball — it is about to be somewhere
        // else. Sightings during it are not projected, so the ball found after is a fresh one.
        let kicking = self.kicked.is_some_and(|at| {
            now.saturating_duration_since(at) < Duration::from_millis(params.cooldown_ms)
        });
        if let Some(s) = fresh
            && s.seq != self.projected
        {
            self.projected = s.seq;
            if !kicking
                && let Some(ray) = s.seen.ray
                && let Some(at) = laser::project(&self.fk, ray, pose, plane, params.max_range_m)
            {
                self.target = Some((at, s.at));
            }
        }
        let memory = Duration::from_millis(params.memory_ms);
        if self
            .target
            .is_some_and(|(_, seen)| now.saturating_duration_since(seen) >= memory)
        {
            self.target = None;
            self.foot = None;
            self.settled = None;
        }

        let mut state = proto::SoccerState {
            autonomous: true,
            camera: self.shared.camera.load().as_ref().clone(),
            ball: fresh.map(|s| s.seen.clone()),
            size_range_m: fresh
                .and_then(|s| s.seen.width)
                .filter(|w| *w > 1e-6)
                .map(|w| diameter(&params) / w),
            age_ms: self
                .target
                .map(|(_, seen)| now.saturating_duration_since(seen).as_millis() as u64),
            kicks: self.kicks,
            ..Default::default()
        };

        if kicking {
            state.phase = "kicking".into();
            return Some(Drive {
                head: None,
                twist: None,
                kick: None,
                state,
            });
        }

        let Some((world, _)) = self.target else {
            state.phase = "searching".into();
            let twist = (params.walk && params.search_vyaw != 0.0).then_some([
                0.0,
                0.0,
                params.search_vyaw,
            ]);
            state.twist = twist;
            return Some(Drive {
                head: None,
                twist,
                kick: None,
                state,
            });
        };

        // The ball in the robot's heading frame, as the laser's dot.
        let (wx, wy) = (world[0] - pose.odom[0], world[1] - pose.odom[1]);
        let (c, s) = (pose.yaw.cos(), pose.yaw.sin());
        let ahead = [wx * c + wy * s, -wx * s + wy * c];
        state.target_m = Some(ahead);

        let head = params.head.then(|| {
            let rel = [wx, wy, plane - pose.odom[2]];
            self.fk
                .look_at(laser::in_trunk(pose, rel), neck_pitch)
                .joints
        });

        let (twist, kick) = self.plan(ahead, &params, driving, now, &mut state);
        let twist = twist.filter(|_| params.walk);
        state.twist = twist;
        state.foot = self.foot.map(|f| f.name().to_owned());
        Some(Drive {
            head,
            twist,
            kick,
            state,
        })
    }

    /// The twist toward a ball at `ahead`, and the kick when it is time for one.
    fn plan(
        &mut self,
        ahead: [f64; 2],
        p: &SoccerParams,
        driving: bool,
        now: Instant,
        state: &mut proto::SoccerState,
    ) -> (Option<[f64; 3]>, Option<&'static str>) {
        let distance = ahead[0].hypot(ahead[1]);
        if distance > p.approach_m {
            state.phase = "approaching".into();
            self.foot = None;
            self.settled = None;
            let bearing = ahead[1].atan2(ahead[0]);
            let vyaw = (p.k_yaw * bearing).clamp(-p.vyaw_max, p.vyaw_max);
            let facing = bearing.cos().max(0.0);
            let vx = (p.k_x * (distance - p.kick_x_m) * facing).clamp(0.0, p.vx_max);
            return (Some([vx, 0.0, vyaw]), None);
        }

        let foot = *self.foot.get_or_insert(if ahead[1] >= 0.0 {
            Foot::Left
        } else {
            Foot::Right
        });
        let spot = [p.kick_x_m, foot.side() * p.kick_y_m];
        let err = [ahead[0] - spot[0], ahead[1] - spot[1]];
        if err[0].abs() <= p.tol_x_m && err[1].abs() <= p.tol_y_m {
            let since = *self.settled.get_or_insert(now);
            if !driving {
                state.phase = "waiting for the policy".into();
                return (Some([0.0; 3]), None);
            }
            if p.kick && now.saturating_duration_since(since) >= Duration::from_millis(p.settle_ms)
            {
                state.phase = "kicking".into();
                self.kicked = Some(now);
                self.kicks += 1;
                state.kicks = self.kicks;
                self.target = None;
                self.foot = None;
                self.settled = None;
                tracing::warn!(foot = foot.name(), kicks = self.kicks, "soccer: kick");
                return (Some([0.0; 3]), Some(foot.skill()));
            }
            state.phase = "lining up".into();
            return (Some([0.0; 3]), None);
        }
        self.settled = None;
        state.phase = "lining up".into();
        // Turn so the ball sits square ahead of the kicking foot, and close the rest with small
        // forward and sideways steps — backwards too, a little, for a ball already under the beak.
        let vyaw =
            (p.k_yaw * err[1].atan2(ahead[0].max(p.kick_x_m))).clamp(-p.vyaw_max, p.vyaw_max);
        let vx = (p.k_x * err[0]).clamp(-p.vx_max / 2.0, p.vx_max);
        let vy = (p.k_y * err[1]).clamp(-p.vy_max, p.vy_max);
        (Some([vx, vy, vyaw]), None)
    }
}

/// The reader: while autonomous soccer is on, hold a `media.ball` subscription and leave the
/// newest look in the slot — `laser::read`'s shape, for the ball.
fn read(shared: &Shared, config: &Path) {
    let mut stamp = robotd_params::config_stamp(config);
    let mut checked = Instant::now();
    loop {
        reload(shared, config, &mut stamp, &mut checked);
        if !shared.autonomous.load(Ordering::Relaxed) {
            shared.latest.store(None);
            shared.camera.store(Arc::new("off".to_owned()));
            std::thread::sleep(WAKE);
            continue;
        }
        let why = subscribe(shared, config, &mut stamp, &mut checked);
        shared.latest.store(None);
        tracing::info!(why, "soccer: the ball subscription ended");
        let retune = why.starts_with("retuning");
        shared.camera.store(Arc::new(why));
        if shared.autonomous.load(Ordering::Relaxed) && !retune {
            std::thread::sleep(RECONNECT);
        }
    }
}

/// Pick up edits to `[soccer]`. A file that will not load keeps the running params and says so.
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
        Ok(params) if params.soccer != **shared.params.load() => {
            tracing::warn!(soccer = ?params.soccer, "soccer: [soccer] changed, applying");
            shared.params.store(Arc::new(params.soccer));
            true
        }
        Ok(_) => false,
        Err(error) => {
            tracing::warn!(error = %error, "soccer: the config will not load; keeping the running [soccer]");
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
        "method": proto::method::MEDIA_BALL,
        "params": detect,
    });
    if let Err(e) = writer.write_all(format!("{request}\n").as_bytes()) {
        return format!("subscribe: {e}");
    }

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    let mut answered = false;
    loop {
        if !shared.autonomous.load(Ordering::Relaxed) {
            return "off".to_owned();
        }
        if reload(shared, config, stamp, checked) && shared.params.load().detect() != detect {
            return "retuning: [soccer] changed".to_owned();
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
                continue;
            }
            Err(e) => return format!("read: {e}"),
        }
        if !answered {
            answered = true;
            if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
                && let Some(error) = response.error
            {
                return if error.code == proto::code::METHOD_NOT_FOUND {
                    "this mediad has no ball detector — update the robot".to_owned()
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
        if note.method != proto::method::MEDIA_BALL_SEEN {
            continue;
        }
        let Some(Ok(seen)) = note.params.map(serde_json::from_value::<proto::BallSeen>) else {
            continue;
        };
        shared.camera.store(Arc::new(format!(
            "looking at {:.0} Hz, {:.1} ms a look",
            seen.hz, seen.took_ms
        )));
        let seq = shared.seq.fetch_add(1, Ordering::Relaxed) + 1;
        shared.latest.store(Some(Arc::new(Sighting {
            seen,
            at: Instant::now(),
            seq,
        })));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn soccer() -> Soccer {
        let mut s = Soccer::new(SoccerParams::default());
        s.set_autonomous(true);
        s
    }

    fn push(s: &Soccer, ray: [f64; 2]) {
        let seq = s.shared.seq.fetch_add(1, Ordering::Relaxed) + 1;
        s.shared.latest.store(Some(Arc::new(Sighting {
            seen: proto::BallSeen {
                seen: true,
                ray: Some(ray),
                ..Default::default()
            },
            at: Instant::now(),
            seq,
        })));
    }

    fn standing() -> Pose {
        Pose {
            head: [0.0; 4],
            imu: [1.0, 0.0, 0.0, 0.0],
            odom: [0.0, 0.0, kinematics::Model::alpha().trunk_height_m()],
            yaw: 0.0,
        }
    }

    /// Put the remembered ball at `ahead` (+x ahead, +y left) of a robot standing at the origin.
    fn place(s: &mut Soccer, ahead: [f64; 2], seen: Instant) {
        s.target = Some((ahead, seen));
    }

    /// Off is off: nothing asked of the loop at all.
    #[test]
    fn switched_off_asks_for_nothing() {
        let mut s = soccer();
        s.set_autonomous(false);
        assert_eq!(s.tick(&standing(), 0.0, true, Instant::now()), None);
    }

    /// No ball known: turn in place, look wherever the loop looks.
    #[test]
    fn no_ball_searches_by_turning() {
        let mut s = soccer();
        let drive = s.tick(&standing(), 0.0, true, Instant::now()).unwrap();
        assert_eq!(drive.state.phase, "searching");
        assert_eq!(drive.twist, Some([0.0, 0.0, 0.5]));
        assert_eq!(drive.head, None);
    }

    /// A ball seen down and to the right is a place ahead and to the right, which the robot turns
    /// toward and walks at — and it is projected a radius above the floor, so it lands nearer
    /// than the same ray would on the floor.
    #[test]
    fn a_far_ball_is_approached() {
        let mut s = soccer();
        push(&s, [0.2, 0.3]);
        let drive = s.tick(&standing(), 0.0, true, Instant::now()).unwrap();
        let [x, y] = drive.state.target_m.expect("on the floor");
        assert!(x > 0.45, "far enough to approach: {x}");
        assert!(y < 0.0, "right is -y: {y}");
        assert_eq!(drive.state.phase, "approaching");
        let [vx, _, vyaw] = drive.twist.unwrap();
        assert!(vx > 0.0 && vyaw < 0.0, "{:?}", drive.twist);
        assert!(drive.head.unwrap()[2] < 0.0, "head turns right");

        let pose = standing();
        let floor = laser::floor_z(&pose, s.trunk_height);
        let [floor_x, _] =
            laser::project(&s.fk, [0.2, 0.3], &pose, floor, 10.0).expect("on the floor");
        assert!(x < floor_x, "{x} vs the floor's {floor_x}");
    }

    /// Near, on the left: the left foot, and sideways steps toward the kick spot.
    #[test]
    fn a_near_ball_picks_the_foot_on_its_side() {
        let mut s = soccer();
        let now = Instant::now();
        place(&mut s, [0.3, 0.15], now);
        let drive = s.tick(&standing(), 0.0, true, now).unwrap();
        assert_eq!(drive.state.phase, "lining up");
        assert_eq!(drive.state.foot.as_deref(), Some("left"));
        let [vx, vy, _] = drive.twist.unwrap();
        assert!(vx > 0.0 && vy > 0.0, "{:?}", drive.twist);

        // The ball drifting across the centre line does not swap feet mid-approach.
        place(&mut s, [0.3, -0.01], now);
        let drive = s.tick(&standing(), 0.0, true, now).unwrap();
        assert_eq!(drive.state.foot.as_deref(), Some("left"));
    }

    /// In the spot: still for `settle_ms`, then one kick with that foot, then hands off for the
    /// cooldown — and the ball is forgotten, since it is about to move.
    #[test]
    fn lined_up_settles_then_kicks_once() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let t0 = Instant::now();
        let spot = [p.kick_x_m, -p.kick_y_m];
        place(&mut s, spot, t0);
        let drive = s.tick(&standing(), 0.0, true, t0).unwrap();
        assert_eq!(drive.kick, None, "settling first");
        assert_eq!(drive.twist, Some([0.0; 3]));

        let t1 = t0 + Duration::from_millis(p.settle_ms + 10);
        place(&mut s, spot, t1);
        let drive = s.tick(&standing(), 0.0, true, t1).unwrap();
        assert_eq!(drive.kick, Some("kick_right"));
        assert_eq!(drive.state.kicks, 1);

        let t2 = t1 + Duration::from_millis(100);
        let drive = s.tick(&standing(), 0.0, true, t2).unwrap();
        assert_eq!(drive.state.phase, "kicking");
        assert_eq!((drive.twist, drive.kick, drive.head), (None, None, None));

        let t3 = t1 + Duration::from_millis(p.cooldown_ms + 10);
        let drive = s.tick(&standing(), 0.0, true, t3).unwrap();
        assert_eq!(
            drive.state.phase, "searching",
            "the ball is looked for again"
        );
    }

    /// No kick while the policy is not driving: it would be refused, and counted as done.
    #[test]
    fn no_kick_without_the_policy() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let t0 = Instant::now();
        place(&mut s, [p.kick_x_m, p.kick_y_m], t0);
        s.tick(&standing(), 0.0, false, t0);
        let t1 = t0 + Duration::from_millis(p.settle_ms + 10);
        place(&mut s, [p.kick_x_m, p.kick_y_m], t1);
        let drive = s.tick(&standing(), 0.0, false, t1).unwrap();
        assert_eq!(drive.kick, None);
        assert_eq!(drive.state.phase, "waiting for the policy");
    }
}
