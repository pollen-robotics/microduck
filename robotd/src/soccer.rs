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
//! | ball beyond `approach_m` | turn in place to face it, then walk straight at it | on the ball |
//! | nearer | turn in place to aim the kicking foot at it, then step until it sits at (`kick_x_m`, ±`kick_y_m`) | on the ball |
//! | there, for `settle_ms` | still | on the ball |
//! | getting ready, `ready_ms` | still | neutral — then `kick_left`/`kick_right` through `robot.do`'s queue |
//! | the kick running, `cooldown_ms` | nothing asked | where the ball was |
//!
//! **Turn, then walk.** Walking and turning at once draws a curve the ball is not on; so the legs
//! only walk once the ball (or, near, the kicking foot's line to it) is within `align_rad` of
//! straight ahead, and stop walking to turn again past twice that. The hysteresis is what keeps it
//! from stuttering between the two at the threshold.
//!
//! The foot is chosen once, on entering `approach_m` — the side the ball is on — and kept until
//! the kick, so a ball near the centre line does not make the robot dither between feet.
//!
//! **A kick is only a kick once it has started.** Asking for one is a request the loop may refuse
//! — a move still running, the policy not driving — and the loop tells this module which with
//! [`Soccer::kick_outcome`] in the same tick. Refused, the ball is still where it was: it stays
//! tracked, and the kick is asked again after another `settle_ms`. Only a kick that started forgets
//! the ball, which is about to be somewhere else.
//!
//! **Close, the ball is out of sight.** The last steps are taken with it under the beak, where the
//! camera may not see it at all, so within `approach_m` it is remembered for `close_memory_ms`
//! rather than `memory_ms` — odometry over a few short steps is good to a centimetre or two.
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
    /// Facing the ball well enough to walk — see the module header on the hysteresis.
    aligned: bool,
    /// Since when the ball has sat in the kick spot.
    settled: Option<Instant>,
    /// In the kick spot, with the hysteresis: entered within the tolerances, left past twice
    /// them — the gait sways the trunk a centimetre or two a step even standing, and a spot that
    /// is left on every sway never settles.
    in_spot: bool,
    /// Since when the ball has been within twice the tolerances — near enough that a robot still
    /// short of the spot after `stall_ms` has stalled, not missed.
    near: Option<Instant>,
    /// Lined up, and getting ready to kick since then: standing still with the head neutral for
    /// `ready_ms`, because the kick network was trained from a robot standing still and looking
    /// ahead, and started from one mid-step with its head pitched down at the ball it barely moves.
    ready: Option<Instant>,
    /// A kick asked of the loop this tick, waiting for [`Soccer::kick_outcome`]: the foot, and
    /// where the ball was.
    asked: Option<(Foot, [f64; 2])>,
    /// When the last kick started, and where the ball was then — the head keeps looking there
    /// while the kick runs.
    kicked: Option<(Instant, [f64; 2])>,
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
            aligned: false,
            settled: None,
            in_spot: false,
            near: None,
            ready: None,
            asked: None,
            kicked: None,
            kicks: 0,
        }
    }

    /// What became of the kick [`Drive::kick`] asked for this tick: `Ok` it started, `Err` why it
    /// did not. Started, the ball is forgotten for the cooldown; refused, it stays tracked and the
    /// kick is asked again once it has settled again.
    pub fn kick_outcome(&mut self, outcome: Result<(), String>, now: Instant) {
        let Some((foot, at)) = self.asked.take() else {
            return;
        };
        match outcome {
            Ok(()) => {
                self.kicks += 1;
                tracing::warn!(foot = foot.name(), kicks = self.kicks, "soccer: kick");
                self.kicked = Some((now, at));
                self.target = None;
                self.foot = None;
                self.aligned = false;
            }
            Err(reason) => {
                tracing::warn!(foot = foot.name(), %reason, "soccer: kick refused; trying again");
            }
        }
        self.settled = None;
        self.in_spot = false;
        self.near = None;
        self.ready = None;
    }

    /// Switch autonomous soccer. Switching it on starts from nothing: no remembered ball, no foot,
    /// no kicks counted.
    pub fn set_autonomous(&mut self, on: bool) {
        if on != self.shared.autonomous.swap(on, Ordering::Relaxed) {
            tracing::warn!(autonomous = on, "soccer");
            self.target = None;
            self.foot = None;
            self.aligned = false;
            self.settled = None;
            self.in_spot = false;
            self.near = None;
            self.ready = None;
            self.asked = None;
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
        let kicking = self.kicked.filter(|(at, _)| {
            now.saturating_duration_since(*at) < Duration::from_millis(params.cooldown_ms)
        });
        if let Some(s) = fresh
            && s.seq != self.projected
        {
            self.projected = s.seq;
            if kicking.is_none()
                && let Some(ray) = s.seen.ray
                && let Some(at) = laser::project(&self.fk, ray, pose, plane, params.max_range_m)
            {
                self.target = Some((at, s.at));
            }
        }
        if let Some((world, seen)) = self.target {
            let near =
                (world[0] - pose.odom[0]).hypot(world[1] - pose.odom[1]) <= params.approach_m;
            let memory = Duration::from_millis(if near {
                params.close_memory_ms.max(params.memory_ms)
            } else {
                params.memory_ms
            });
            if now.saturating_duration_since(seen) >= memory {
                self.target = None;
                self.foot = None;
                self.aligned = false;
                self.settled = None;
                self.in_spot = false;
                self.near = None;
                self.ready = None;
            }
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

        // The look at a world point `[x, y]` on the ball's plane.
        let look = |this: &Self, at: [f64; 2]| {
            params
                .head
                .then(|| this.look_at(at, pose, plane, neck_pitch))
        };

        if let Some((_, at)) = kicking {
            state.phase = "kicking".into();
            return Some(Drive {
                head: look(self, at),
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

        let (twist, kick) = self.plan(ahead, &params, driving, now, &mut state);
        // Getting ready, the head goes neutral — where the kick network expects it — rather
        // than following the ball under the beak.
        let head = if self.ready.is_some() {
            params.head.then_some([0.0; 4])
        } else {
            look(self, world)
        };
        if let Some(skill) = kick {
            let foot = if skill == Foot::Left.skill() {
                Foot::Left
            } else {
                Foot::Right
            };
            self.asked = Some((foot, world));
        }
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

    /// The head joints that look at world point `at`, on the plane at height `plane`.
    fn look_at(&self, at: [f64; 2], pose: &Pose, plane: f64, neck_pitch: f64) -> [f64; 4] {
        let rel = [
            at[0] - pose.odom[0],
            at[1] - pose.odom[1],
            plane - pose.odom[2],
        ];
        self.fk
            .look_at(laser::in_trunk(pose, rel), neck_pitch)
            .joints
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
        // Committed to a kick: stand still, head neutral, then kick — whatever the remembered
        // ball does meanwhile, since the robot is not moving and the head going up is not news.
        if let (Some(at), Some(foot)) = (self.ready, self.foot) {
            if !driving {
                self.ready = None;
                state.phase = "waiting for the policy".into();
                return (Some([0.0; 3]), None);
            }
            if now.saturating_duration_since(at) >= Duration::from_millis(p.ready_ms) {
                state.phase = "kicking".into();
                return (Some([0.0; 3]), Some(foot.skill()));
            }
            state.phase = "getting ready".into();
            return (Some([0.0; 3]), None);
        }
        let distance = ahead[0].hypot(ahead[1]);
        if distance > p.approach_m {
            state.phase = "approaching".into();
            self.foot = None;
            self.settled = None;
            self.in_spot = false;
            self.near = None;
            self.ready = None;
            let bearing = ahead[1].atan2(ahead[0]);
            let vyaw = (p.k_yaw * bearing).clamp(-p.vyaw_max, p.vyaw_max);
            let vx = if self.align(bearing, p) {
                (p.k_x * (distance - p.kick_x_m)).clamp(0.0, p.vx_max)
            } else {
                0.0
            };
            return (Some([vx, 0.0, vyaw]), None);
        }

        let foot = *self.foot.get_or_insert(if ahead[1] >= 0.0 {
            Foot::Left
        } else {
            Foot::Right
        });
        let spot = [p.kick_x_m, foot.side() * p.kick_y_m];
        let err = [ahead[0] - spot[0], ahead[1] - spot[1]];
        state.spot_m = Some(err);
        let within = |k: f64| err[0].abs() <= k * p.tol_x_m && err[1].abs() <= k * p.tol_y_m;
        self.in_spot = within(if self.in_spot { 2.0 } else { 1.0 });
        if within(2.0) {
            self.near.get_or_insert(now);
        } else {
            self.near = None;
        }
        // Near the spot for `stall_ms` and still not in it: the last centimetres ask for a speed
        // too small for the gait to walk, and waiting longer will not get it there.
        let stalled = self.near.is_some_and(|at| {
            now.saturating_duration_since(at) >= Duration::from_millis(p.stall_ms)
        });
        if self.in_spot || stalled {
            let since = *self.settled.get_or_insert(now);
            if !driving {
                state.phase = "waiting for the policy".into();
                return (Some([0.0; 3]), None);
            }
            let settled =
                now.saturating_duration_since(since) >= Duration::from_millis(p.settle_ms);
            if p.kick && (settled || stalled) {
                tracing::info!(
                    foot = foot.name(),
                    stalled,
                    "soccer: lined up, getting ready to kick"
                );
                self.ready = Some(now);
                state.phase = "getting ready".into();
                return (Some([0.0; 3]), None);
            }
            state.phase = "lining up".into();
            return (Some([0.0; 3]), None);
        }
        self.settled = None;
        state.phase = "lining up".into();
        // Aim the kicking foot's line at the ball — the bearing of the ball from a point
        // `kick_y_m` to that side — and only then step: forward to close the distance (back a
        // little, for a ball already under the beak), sideways for the last centimetres.
        let aim = err[1].atan2(ahead[0].max(p.kick_x_m));
        let vyaw = (p.k_yaw * aim).clamp(-p.vyaw_max, p.vyaw_max);
        if !self.align(aim, p) {
            return (Some([0.0, 0.0, vyaw]), None);
        }
        let vx = (p.k_x * err[0]).clamp(-p.vx_max / 2.0, p.vx_max);
        let vy = (p.k_y * err[1]).clamp(-p.vy_max, p.vy_max);
        (Some([vx, vy, vyaw]), None)
    }

    /// Whether the robot faces `bearing` well enough to walk, with the hysteresis: aligned below
    /// `align_rad`, and stays aligned until past twice that.
    fn align(&mut self, bearing: f64, p: &SoccerParams) -> bool {
        let off = bearing.abs();
        self.aligned = if self.aligned {
            off <= 2.0 * p.align_rad
        } else {
            off <= p.align_rad
        };
        self.aligned
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

    /// Lined up at `t`: the robot gets ready first — still, head neutral — and the kick comes
    /// `ready_ms` later. Returns that kick, and when it came.
    fn ready_then_kick(
        s: &mut Soccer,
        ball: [f64; 2],
        t: Instant,
    ) -> (Option<&'static str>, Instant) {
        let p = SoccerParams::default();
        place(s, ball, t);
        let drive = s.tick(&standing(), 0.0, true, t).unwrap();
        assert_eq!(drive.state.phase, "getting ready");
        assert_eq!(drive.kick, None);
        assert_eq!(drive.twist, Some([0.0; 3]), "standing still");
        assert_eq!(drive.head, Some([0.0; 4]), "head neutral");
        let later = t + Duration::from_millis(p.ready_ms + 10);
        place(s, ball, later);
        (s.tick(&standing(), 0.0, true, later).unwrap().kick, later)
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
        let [_, _, vyaw] = drive.twist.unwrap();
        assert!(vyaw < 0.0, "turns right: {:?}", drive.twist);
        assert!(drive.head.unwrap()[2] < 0.0, "head turns right");

        let pose = standing();
        let floor = laser::floor_z(&pose, s.trunk_height);
        let [floor_x, _] =
            laser::project(&s.fk, [0.2, 0.3], &pose, floor, 10.0).expect("on the floor");
        assert!(x < floor_x, "{x} vs the floor's {floor_x}");
    }

    /// **Turn, then walk.** A ball well off to the side is turned to in place; once it is within
    /// `align_rad` the robot walks at it, and keeps walking through a small wobble past that —
    /// it only stops to turn again past twice `align_rad`.
    #[test]
    fn a_far_ball_is_faced_before_it_is_walked_to() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let now = Instant::now();
        let at = |bearing: f64| [1.0 * bearing.cos(), 1.0 * bearing.sin()];

        place(&mut s, at(0.6), now);
        let [vx, _, vyaw] = s.tick(&standing(), 0.0, true, now).unwrap().twist.unwrap();
        assert_eq!(vx, 0.0, "turning in place");
        assert!(vyaw > 0.0);

        place(&mut s, at(0.5 * p.align_rad), now);
        let [vx, _, _] = s.tick(&standing(), 0.0, true, now).unwrap().twist.unwrap();
        assert!(vx > 0.0, "faced: walking");

        place(&mut s, at(1.5 * p.align_rad), now);
        let [vx, _, _] = s.tick(&standing(), 0.0, true, now).unwrap().twist.unwrap();
        assert!(vx > 0.0, "a wobble does not stop it");

        place(&mut s, at(2.5 * p.align_rad), now);
        let [vx, _, _] = s.tick(&standing(), 0.0, true, now).unwrap().twist.unwrap();
        assert_eq!(vx, 0.0, "too far off: turning again");
    }

    /// Near, on the left: the left foot; turn to aim it, then step toward the kick spot.
    #[test]
    fn a_near_ball_picks_the_foot_on_its_side() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let now = Instant::now();
        place(&mut s, [0.3, 0.25], now);
        let drive = s.tick(&standing(), 0.0, true, now).unwrap();
        assert_eq!(drive.state.phase, "lining up");
        assert_eq!(drive.state.foot.as_deref(), Some("left"));
        let [vx, vy, vyaw] = drive.twist.unwrap();
        assert!(
            vx == 0.0 && vy == 0.0 && vyaw > 0.0,
            "aiming first: {:?}",
            drive.twist
        );

        // Aimed — the ball on the foot's line — it steps in.
        place(&mut s, [0.3, p.kick_y_m + 0.01], now);
        let [vx, _, _] = s.tick(&standing(), 0.0, true, now).unwrap().twist.unwrap();
        assert!(vx > 0.0, "stepping in");

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
        let (kick, t1) = ready_then_kick(&mut s, spot, t1);
        assert_eq!(kick, Some("kick_right"));
        s.kick_outcome(Ok(()), t1);

        // The kick running: the legs are the skill's, and the head stays where the ball was.
        let t2 = t1 + Duration::from_millis(100);
        let drive = s.tick(&standing(), 0.0, true, t2).unwrap();
        assert_eq!(drive.state.phase, "kicking");
        assert_eq!(drive.state.kicks, 1);
        assert_eq!((drive.twist, drive.kick), (None, None));
        let head = drive.head.expect("still looking at the ball");
        assert!(head[2] < 0.0, "down and to the right: {head:?}");

        let t3 = t1 + Duration::from_millis(p.cooldown_ms + 10);
        let drive = s.tick(&standing(), 0.0, true, t3).unwrap();
        assert_eq!(
            drive.state.phase, "searching",
            "the ball is looked for again"
        );
    }

    /// **A refused kick is not a kick.** The loop saying no — a move still running — leaves the
    /// ball tracked and the head on it, and the kick is asked again once it has settled again.
    #[test]
    fn a_refused_kick_keeps_the_ball_and_tries_again() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let t0 = Instant::now();
        let spot = [p.kick_x_m, p.kick_y_m];
        place(&mut s, spot, t0);
        s.tick(&standing(), 0.0, true, t0);
        let t1 = t0 + Duration::from_millis(p.settle_ms + 10);
        let (kick, t1) = ready_then_kick(&mut s, spot, t1);
        assert_eq!(kick, Some("kick_left"));
        s.kick_outcome(Err("a scripted move is already running".into()), t1);

        let t2 = t1 + Duration::from_millis(20);
        let drive = s.tick(&standing(), 0.0, true, t2).unwrap();
        assert_eq!(drive.state.phase, "lining up", "still on the ball");
        assert_eq!(drive.state.kicks, 0);
        assert!(drive.head.is_some());
        assert_eq!(drive.kick, None, "settling again first");

        let t3 = t2 + Duration::from_millis(p.settle_ms + 10);
        assert_eq!(ready_then_kick(&mut s, spot, t3).0, Some("kick_left"));
    }

    /// **Sway does not reset the settle.** Once in the spot, a centimetre or two of the trunk's
    /// sway — past the tolerance, inside twice it — keeps it in, and the kick comes on time.
    #[test]
    fn sway_inside_twice_the_tolerance_still_kicks() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let t0 = Instant::now();
        let spot = [p.kick_x_m, p.kick_y_m];
        place(&mut s, spot, t0);
        s.tick(&standing(), 0.0, true, t0);
        let t1 = t0 + Duration::from_millis(p.settle_ms / 2);
        let swayed = [spot[0], spot[1] + 1.5 * p.tol_y_m];
        place(&mut s, swayed, t1);
        let drive = s.tick(&standing(), 0.0, true, t1).unwrap();
        assert_eq!(drive.twist, Some([0.0; 3]), "still in the spot");
        let t2 = t0 + Duration::from_millis(p.settle_ms + 10);
        assert_eq!(ready_then_kick(&mut s, spot, t2).0, Some("kick_left"));
    }

    /// **Stalled short.** Near the spot but never in it — the gait will not walk the last
    /// centimetre — the robot kicks from there after `stall_ms`, and says how far off it was.
    #[test]
    fn stalled_near_the_spot_kicks_anyway() {
        let p = SoccerParams::default();
        let mut s = soccer();
        let t0 = Instant::now();
        let short = [p.kick_x_m + 1.5 * p.tol_x_m, p.kick_y_m];
        place(&mut s, short, t0);
        let drive = s.tick(&standing(), 0.0, true, t0).unwrap();
        assert_eq!(drive.kick, None);
        let [dx, dy] = drive.state.spot_m.expect("the error to the spot");
        assert!((dx - 1.5 * p.tol_x_m).abs() < 1e-9 && dy.abs() < 1e-9);

        let t1 = t0 + Duration::from_millis(p.stall_ms + 10);
        assert_eq!(ready_then_kick(&mut s, short, t1).0, Some("kick_left"));

        // Farther than twice the tolerance is not stalled, it is still walking.
        let mut s = soccer();
        let far = [p.kick_x_m + 3.0 * p.tol_x_m, p.kick_y_m];
        place(&mut s, far, t0);
        s.tick(&standing(), 0.0, true, t0);
        place(&mut s, far, t1);
        let drive = s.tick(&standing(), 0.0, true, t1).unwrap();
        assert_eq!(drive.state.phase, "lining up", "not getting ready");
        let later = t1 + Duration::from_millis(p.ready_ms + 10);
        place(&mut s, far, later);
        assert_eq!(s.tick(&standing(), 0.0, true, later).unwrap().kick, None);
    }

    /// Under the beak, out of sight, the ball is not forgotten after `memory_ms` — only after
    /// `close_memory_ms`. Far away it is.
    #[test]
    fn a_close_ball_is_remembered_longer() {
        let p = SoccerParams::default();
        let t0 = Instant::now();
        let later = t0 + Duration::from_millis(p.memory_ms + 100);

        let mut s = soccer();
        place(&mut s, [0.3, 0.05], t0);
        let drive = s.tick(&standing(), 0.0, true, later).unwrap();
        assert_eq!(drive.state.phase, "lining up");

        let mut s = soccer();
        place(&mut s, [1.5, 0.0], t0);
        let drive = s.tick(&standing(), 0.0, true, later).unwrap();
        assert_eq!(drive.state.phase, "searching");

        let mut s = soccer();
        place(&mut s, [0.3, 0.05], t0);
        let gone = t0 + Duration::from_millis(p.close_memory_ms + 100);
        assert_eq!(
            s.tick(&standing(), 0.0, true, gone).unwrap().state.phase,
            "searching"
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
