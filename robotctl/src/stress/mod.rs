//! `robotctl stress` — a fixed lap of walks, provoked falls and sit/stands, run until the battery
//! is empty, timing every get-up. The design is
//! `docs/superpowers/specs/2026-10-27-stress-test-design.md`.
//!
//! **It is the same test on every robot.** `robotctl endurance` draws its activities at random,
//! which is right for a battery curve and wrong for comparing two robots: one detour and the
//! runs no longer line up. Here the lap ([`lap`]) is data with a version, the steps start on a
//! schedule that only a get-up moves, and anything a robot does differently — a guard, a fall
//! nobody asked for, a slow get-up — is logged rather than compensated.
//!
//! **A fall is a trip** (`robot.trip`): the legs held mid-stride for `--trip-ms` while the policy
//! keeps running, so it tries to recover and, failing, goes over with robotd's limp-fall live —
//! the way a real stumble ends. `--fall cut` is the first version's fall, the pad's Start tapped
//! mid-stride: the policy off for 0.5 s and back while the robot is still going down, which also
//! switches the limp-fall off for exactly that moment, so the robot lands stiff. Each fall logs
//! a `limp` event when the limp-fall caught it. The get-up is the policy's own; [`getup`] times
//! it and decides when to give up.
//!
//! **It stays off the walls by the map** ([`room`]): `mapd`'s map of the test room, built once
//! and copied to every robot under test (`robotctl map load`). Before each walk it faces a way the
//! whole walk keeps `--wall` off every wall, back towards the room's most open place when it has
//! strayed `--square` from it, and stops a walk that would come closer. Every fall and sit leaves
//! `mapd` lost; the robot then stands and turns on the spot, which is what lets `mapd` find it
//! again, for up to `--relocalize` seconds.
//!
//! **The map is frozen for the run** (`map.mode`, re-asked every lap in case `mapd` restarted):
//! `mapd` finds the robot in it and adds nothing, so every robot keeps off the walls of the same
//! map from first lap to last, and a fall it misreads cannot move a wall. `--map-learns` lets
//! the map grow as usual. The run hands mapping back when it stops.
//!
//! **Without the map it falls back to the first guards** ([`guard`]) for that walk: an odometry
//! square around the start, the head ToF ahead, and a look behind before backing up. A duck with
//! no map, or one still lost after `--relocalize`, runs on those and says so.
//!
//! The background unit, the run directory and the sampler are `endurance`'s ([`runlog`]); the
//! columns are a superset of what `scripts/endurance-plot.py` reads, plus `lap` and `step`.

mod getup;
pub(crate) mod guard;
mod lap;
mod room;

use std::collections::VecDeque;
use std::io::BufRead;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use clap::{Args, Subcommand};
use duck_ipc_proto as proto;
use serde::Serialize;

use crate::runlog::{
    self, Halt, Run, Sampler, StateFeed, Unit, brief, json, signal_name, signalled, wait_for_signal,
};
use crate::{Client, Failure, exit};
use getup::{Getup, Timing, Verdict};
use lap::{Action, Schedule, Step};
use room::{Pose, Room};

const UNIT: Unit = Unit {
    name: "robotctl-stress",
    runs_dir: "/var/lib/stress",
    command: "stress",
    description: "robotctl stress: falls and get-ups until the battery is empty",
    stopped: "the policy is off, the robot holding its home pose",
};

/// `robotd`'s deadman zeroes the twist 500 ms after the last intent; 20 Hz is well inside it.
const MOVE_PERIOD: Duration = Duration::from_millis(50);
/// A depth frame older than this is no depth at all: at 15 Hz it is several frames missed.
const TOF_MAX_AGE: Duration = Duration::from_millis(500);
/// Heading error above which the walk back to the centre turns on the spot, rad.
const TURN_THRESHOLD: f64 = 0.6;
/// The walk back to the centre ends this far inside the square, as a share of its half-side, so
/// the robot is not on the edge again at the next step.
const SQUARE_RETURN: f64 = 0.8;
/// Looking behind (`look_back`): how long the head gets to arrive after the body's quarter turn,
/// and how long the frames are then read for, s.
const LOOK_SETTLE: f64 = 0.4;
const LOOK_READ: f64 = 0.3;
/// A turn on the spot is done within this of its heading, rad, or after this long, s.
const TURN_TOLERANCE: f64 = 0.08;
const TURN_TIMEOUT: f64 = 4.0;
/// A wall guard turns until the nearest return ahead is this much further than `--wall`.
const WALL_CLEAR: f64 = 1.5;
/// A map pose older than this is no pose: `mapd` publishes at 4 Hz.
const MAP_MAX_AGE: Duration = Duration::from_secs(1);
/// The map's cell for the room, m. `mapd`'s keyframes are 2.5 cm; this is plenty for 30 cm.
const ROOM_RES: f64 = 0.05;
/// How far ahead of the robot the live map guard looks, m: a tick at full stick, and some.
const MAP_LOOKAHEAD: f64 = 0.05;
/// Relocalizing: how long each stop stands, s. `mapd` opens a keyframe after 0.4 s of stillness
/// and wants ten frames; `robotd`'s idle glance, which widens what the stop sees, starts at 2 s.
const RELOC_STOP: f64 = 3.0;
/// Odometry kept to line a map pose up with, s at 20 Hz.
const ODOM_HISTORY: usize = 60;
/// A map pose is lined up with the odometry sample nearest its stamp, if one is this close, ns.
const ODOM_MATCH_NS: u64 = 60_000_000;

const DRIVING: [&str; 2] = ["walk", "stand"];

#[derive(Subcommand, Debug)]
pub enum StressCommand {
    /// Whether a run is going, its lap and step, and its latest sample and events.
    Status,
    /// Stop the background run. The policy goes off, as with the pad's Start.
    Stop,
    /// Start the default run if `[stress] on_boot` is set (`robotctl configure`), and do nothing
    /// otherwise. What `robotctl-stress-boot.timer` runs once per boot.
    Boot {
        /// The config to read. The default is where a provisioned robot keeps it.
        #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
        config: PathBuf,
    },
}

#[derive(Args, Debug, Clone, Serialize)]
pub struct StressArgs {
    /// Run directory. Default: /var/lib/stress/<hostname>-<UTC time>. An existing one is
    /// appended to.
    #[arg(long)]
    out: Option<PathBuf>,
    /// Seconds between samples.
    #[arg(long, default_value_t = 10.0)]
    interval: f64,
    /// Run here instead of in the background. What the background unit itself runs.
    #[arg(long)]
    foreground: bool,

    /// Walking speed, m/s. Full stick.
    #[arg(long, default_value_t = 0.3, help_heading = "Lap")]
    speed: f64,
    /// Turn rate, rad/s.
    #[arg(long, default_value_t = 1.5, help_heading = "Lap")]
    vyaw: f64,
    /// How a fall is made: `trip` holds the legs mid-stride while the policy keeps running, so
    /// it tries to recover and robotd's limp-fall catches the fall as it would a real one;
    /// `cut` switches the policy off for --cut, the pad's Start, which takes both away.
    #[arg(long, default_value = "trip", value_parser = ["trip", "cut"], help_heading = "Lap")]
    fall: String,
    /// How long a trip holds the legs, ms. Default: `[stress] trip_ms` (`robotctl configure`,
    /// 500), re-read at every fall so it can be tuned mid-run; this flag pins it for the run.
    /// Find it by hand with `robotctl robot trip MS` while walking with the pad.
    #[arg(long, help_heading = "Lap")]
    trip_ms: Option<u32>,
    /// Which legs a trip holds: both, or one foot caught. Default: `[stress] trip_legs`, re-read
    /// at every fall like `trip_ms`.
    #[arg(long, value_parser = ["both", "left", "right"], help_heading = "Lap")]
    trip_legs: Option<String>,
    /// The config the trip is read from.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,
    /// With --fall cut: how long the policy is cut, s. Longer lets the robot settle and fall
    /// softly.
    #[arg(long, default_value_t = 0.5, help_heading = "Lap")]
    cut: f64,
    /// Down this long after the policy came back: cut it for --reset and try once more, s.
    #[arg(long, default_value_t = 7.0, help_heading = "Get-up")]
    getup_timeout: f64,
    /// The cut between the two get-up tries, s.
    #[arg(long, default_value_t = 2.0, help_heading = "Get-up")]
    reset: f64,
    /// Half the side of the square it stays in, around where it started, m.
    #[arg(long, default_value_t = 0.5, help_heading = "Guards")]
    square: f64,
    /// Stop walking forward at an obstacle this close, m (head ToF). 30 cm, not 20: a forward
    /// fall from 20 cm lands against the wall.
    #[arg(long, default_value_t = 0.30, help_heading = "Guards")]
    wall: f64,
    /// Ignore `mapd`'s map and keep off the walls with the odometry square and the head ToF,
    /// as the first stress runs did.
    #[arg(long, help_heading = "Guards")]
    no_map: bool,
    /// Lost in the map before a walk (every fall and sit does it): stand and turn on the spot for
    /// up to this long, s, so `mapd` can find the robot again. Still lost after it, that walk
    /// falls back to the odometry square and the ToF.
    #[arg(long, default_value_t = 15.0, help_heading = "Guards")]
    relocalize: f64,
    /// Let `mapd` add to the map during the run. By default the map is frozen for it: the robot
    /// is found in it and nothing is added.
    #[arg(long, help_heading = "Guards")]
    map_learns: bool,
    /// At or below this battery %, stop driving and let robotd's own empty-battery shutdown run
    /// undisturbed.
    #[arg(long, default_value_t = 2.0, help_heading = "Guards")]
    hands_off_pct: f64,
    /// After giving up, call for help this often, s: two quacks and a wheee, loud enough to hear
    /// from the next room. 0 for silence.
    #[arg(long, default_value_t = 15.0, help_heading = "Guards")]
    call_every: f64,
}

/// The daemons a run talks to.
pub struct Sockets<'a> {
    pub robot: &'a Path,
    pub tof: &'a Path,
    pub map: &'a Path,
}

pub fn run(
    sockets: Sockets,
    command: Option<StressCommand>,
    args: StressArgs,
) -> Result<(), Failure> {
    let robot_socket = sockets.robot;
    match command {
        Some(StressCommand::Status) => UNIT.status(),
        Some(StressCommand::Stop) => UNIT.stop(),
        Some(StressCommand::Boot { config }) => boot(&sockets, &config),
        None if args.foreground => foreground(&sockets, args),
        None => UNIT.start(robot_socket, args.out.as_deref()),
    }
}

// ── at boot ─────────────────────────────────────────────────────────────────────────────────

/// How long the boot run waits for `robotd` to be publishing state. The timer fires seconds into
/// the boot and `robotd` is usually up by then; a board still without it after this has a problem
/// a stress test should not start on top of.
const BOOT_ROBOTD_WAIT: Duration = Duration::from_secs(120);

/// `[stress] on_boot`: start the default run, as `sudo robotctl stress` with no flags would.
///
/// Off is the normal answer and exits 0 saying so, because the timer runs this on every boot of
/// every robot that ships it. A config that does not load counts as off — the boot is the wrong
/// moment to guess what a broken file meant, and `robotd` reports the file itself. A run already
/// going is not a failure either: there is nothing to start, and the one going is the one wanted.
fn boot(sockets: &Sockets, config: &Path) -> Result<(), Failure> {
    let robot_socket = sockets.robot;
    let on = match robotd_params::Params::load(config, false) {
        Ok(params) => params.stress.on_boot,
        Err(e) => {
            println!("{}: {e}; not starting a stress run", config.display());
            return Ok(());
        }
    };
    if !on {
        println!("[stress] on_boot is off; nothing to start");
        return Ok(());
    }
    if UNIT.active() {
        println!("a stress run is already going ({}); leaving it", UNIT.name);
        return Ok(());
    }
    // Not just the socket: robotd answers on it before it publishes any `robot.state`, and a run
    // started in that window spends its first 35 s finding no state and stops `fatal`. That was
    // three boots in seven on the first beta runs.
    let deadline = Instant::now() + BOOT_ROBOTD_WAIT;
    let not_starting = |why: String| {
        Failure::new(
            exit::UNREACHABLE,
            format!(
                "[stress] on_boot is on, but {why} within {} s: is robotd running? Not starting",
                BOOT_ROBOTD_WAIT.as_secs()
            ),
        )
    };
    while !robot_socket.exists() {
        if Instant::now() >= deadline {
            return Err(not_starting(format!(
                "{} did not appear",
                robot_socket.display()
            )));
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    let feed = StateFeed::spawn(robot_socket.to_owned());
    while feed.frame().is_none() {
        if Instant::now() >= deadline {
            return Err(not_starting("robotd published no robot.state".into()));
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    println!("robotd is publishing state");
    println!("[stress] on_boot is on: starting the default run");
    // The run's own command line, not this one: `stress boot --config …` re-run with
    // `--foreground` is not a run. The sockets are passed through so the run talks to the same
    // daemons this checked.
    let argv = [
        "--robot-socket".into(),
        robot_socket.as_os_str().to_owned(),
        "--tof-socket".into(),
        sockets.tof.as_os_str().to_owned(),
        "--map-socket".into(),
        sockets.map.as_os_str().to_owned(),
        "stress".into(),
        "--config".into(),
        config.as_os_str().to_owned(),
    ]
    .to_vec();
    UNIT.start_with(robot_socket, None, argv)
}

// ── the depth stream ────────────────────────────────────────────────────────────────────────

#[derive(Default)]
struct TofInner {
    latest: Option<(Instant, proto::TofFrame)>,
    /// Why there is no depth, as `tofd` or the socket said it. Cleared by the next frame.
    unavailable: Option<String>,
}

/// The latest depth frame from `tofd`, reconnecting forever — `monitor`'s reader, keeping only
/// the last frame.
#[derive(Clone)]
struct TofFeed(Arc<Mutex<TofInner>>);

impl TofFeed {
    fn spawn(socket: PathBuf) -> Self {
        let feed = Self(Arc::default());
        let inner = feed.0.clone();
        std::thread::spawn(move || {
            loop {
                let why = Self::stream(&socket, &inner);
                inner.lock().expect("tof lock").unavailable = Some(why);
                std::thread::sleep(Duration::from_secs(2));
            }
        });
        feed
    }

    /// One subscription, from its call to whatever ended it.
    fn stream(socket: &Path, inner: &Mutex<TofInner>) -> String {
        let mut client = match Client::connect_to("tofd", socket) {
            Ok(c) => c,
            Err(f) => return brief(&f),
        };
        let _ = client.writer.set_read_timeout(Some(Duration::from_secs(5)));
        if let Err(f) = client.send(&proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::TofStream,
        )) {
            return brief(&f);
        }
        let mut line = String::new();
        loop {
            line.clear();
            match client.reader.read_line(&mut line) {
                Ok(0) => return "tofd closed the depth stream".into(),
                Err(e) => return format!("the depth stream stopped: {e}"),
                Ok(_) => {}
            }
            if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
                && let Ok(status) = response.result_as::<proto::TofStreamResult>()
            {
                if status.sensor.is_none() {
                    inner.lock().expect("tof lock").unavailable = Some(
                        status
                            .unavailable
                            .unwrap_or_else(|| "tofd has no sensor".into()),
                    );
                }
                continue;
            }
            if let Some(frame) = serde_json::from_str::<proto::Request>(&line)
                .ok()
                .and_then(|r| r.as_tof_frame())
            {
                let mut inner = inner.lock().expect("tof lock");
                inner.latest = Some((Instant::now(), frame));
                inner.unavailable = None;
            }
        }
    }

    fn frame(&self) -> Option<proto::TofFrame> {
        let inner = self.0.lock().expect("tof lock");
        let (at, frame) = inner.latest.as_ref()?;
        (at.elapsed() <= TOF_MAX_AGE).then(|| frame.clone())
    }

    fn unavailable(&self) -> Option<String> {
        self.0.lock().expect("tof lock").unavailable.clone()
    }
}

// ── the map ─────────────────────────────────────────────────────────────────────────────────

#[derive(Default)]
struct MapInner {
    latest: Option<(Instant, proto::MapState)>,
    /// Why there is no pose at all — `mapd` unreachable, or not mapping. Cleared by the next state.
    unavailable: Option<String>,
}

/// The latest `map.state` from `mapd`, reconnecting forever.
#[derive(Clone)]
struct MapFeed(Arc<Mutex<MapInner>>);

impl MapFeed {
    fn spawn(socket: PathBuf) -> Self {
        let feed = Self(Arc::default());
        let inner = feed.0.clone();
        std::thread::spawn(move || {
            loop {
                let why = Self::stream(&socket, &inner);
                let mut inner = inner.lock().expect("map lock");
                inner.unavailable = Some(why);
                inner.latest = None;
                drop(inner);
                std::thread::sleep(Duration::from_secs(2));
            }
        });
        feed
    }

    fn stream(socket: &Path, inner: &Mutex<MapInner>) -> String {
        let mut client = match Client::connect_to("mapd", socket) {
            Ok(c) => c,
            Err(f) => return brief(&f),
        };
        let _ = client.writer.set_read_timeout(Some(Duration::from_secs(5)));
        if let Err(f) = client.send(&proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::MapStream,
        )) {
            return brief(&f);
        }
        let mut line = String::new();
        loop {
            line.clear();
            match client.reader.read_line(&mut line) {
                Ok(0) => return "mapd closed the map stream".into(),
                Err(e) => return format!("the map stream stopped: {e}"),
                Ok(_) => {}
            }
            if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
                && let Ok(status) = response.result_as::<proto::MapStatusResult>()
            {
                let mut inner = inner.lock().expect("map lock");
                inner.unavailable = status.unavailable;
                inner.latest = Some((Instant::now(), status.state));
                continue;
            }
            if let Some(state) = serde_json::from_str::<proto::Request>(&line)
                .ok()
                .and_then(|r| r.as_map_state())
            {
                let mut inner = inner.lock().expect("map lock");
                inner.latest = Some((Instant::now(), state));
            }
        }
    }

    /// The latest state, if it is recent and `mapd` is mapping.
    fn state(&self) -> Option<proto::MapState> {
        let inner = self.0.lock().expect("map lock");
        if inner.unavailable.is_some() {
            return None;
        }
        let (at, state) = inner.latest.as_ref()?;
        (at.elapsed() <= MAP_MAX_AGE).then(|| state.clone())
    }

    fn unavailable(&self) -> Option<String> {
        let inner = self.0.lock().expect("map lock");
        if let Some(why) = &inner.unavailable {
            return Some(why.clone());
        }
        match &inner.latest {
            None => Some("no map state from mapd yet".into()),
            Some((at, _)) if at.elapsed() > MAP_MAX_AGE => Some(format!(
                "no map state for {:.0} s",
                at.elapsed().as_secs_f64()
            )),
            Some(_) => None,
        }
    }
}

/// The room, fetched from `mapd` at the start of every lap: the map moves a little as `mapd`
/// re-solves it, and a lap is short enough for one grid to hold.
fn fetch_room(socket: &Path) -> Result<Room, String> {
    let mut client = Client::connect_to("mapd", socket).map_err(|f| brief(&f))?;
    let _ = client
        .writer
        .set_read_timeout(Some(Duration::from_secs(10)));
    let response = client
        .call(&proto::Call::MapGrid(proto::MapGridParams {
            res_m: Some(ROOM_RES),
        }))
        .map_err(|f| brief(&f))?;
    let grid: proto::MapGridResult =
        crate::decode(&crate::result_of(response).map_err(|f| brief(&f))?)
            .map_err(|f| brief(&f))?;
    Room::from_grid(&grid).ok_or_else(|| "the map has no floor in it yet".into())
}

// ── the driver ──────────────────────────────────────────────────────────────────────────────

type Drive = Result<(), Halt>;

/// What `HandsOff` carries after a give-up: the activity is already `gave_up`.
const GAVE_UP: &str = "gave up";

/// What a walking step is doing once a guard has cut it short.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Guard {
    /// The step's own twist.
    Off,
    /// Turning on the spot until the way ahead is clear.
    Wall,
    /// Walking back towards the centre of the square.
    Square,
    /// Recovered: standing still for the rest of the slot.
    Done,
}

struct Driver {
    run: Arc<Run>,
    feed: StateFeed,
    tof: TofFeed,
    reprojector: kinematics::tof::Reprojector,
    args: StressArgs,
    timing: Timing,
    socket: PathBuf,
    client: Option<Client>,
    schedule: Schedule,
    home: (f64, f64),
    /// Whether the robot is in a sit this driver asked for (or found it in after a get-up). One
    /// it did not is robotd's battery shutdown, and must not be undone.
    our_sit: bool,
    /// Said once: the run goes on without the wall guard.
    no_tof_logged: bool,
    /// The last odometry read, to see a `robotd` restart zero it.
    last_xy: Option<(f64, f64)>,
    map: MapFeed,
    map_socket: PathBuf,
    /// The room this lap keeps off the walls of, when there is a map.
    room: Option<Room>,
    /// The room's most open place, from the first map seen: where the walks are steered back to.
    map_home: Option<(f64, f64)>,
    /// Why the last lap had no room, said once per change.
    no_room: Option<String>,
    /// Odometry at each tick, stamped with `robotd`'s clock, to line `mapd`'s pose up with now.
    odom: VecDeque<(u64, Pose)>,
    /// Whether `mapd` last said the map was frozen; `None` before asking.
    map_frozen: Option<bool>,
}

impl Driver {
    // The connection is reopened whenever it drops: a robotd restart mid-test should cost a few
    // seconds of behaviour, not the run.
    fn conn(&mut self) -> Result<&mut Client, Halt> {
        while self.client.is_none() {
            match Client::connect_to("robotd", &self.socket) {
                Ok(c) => {
                    let _ = c.writer.set_read_timeout(Some(Duration::from_secs(5)));
                    self.client = Some(c);
                }
                Err(f) => {
                    self.run.event("robotd_unreachable", &brief(&f));
                    self.wait(5.0)?;
                }
            }
        }
        Ok(self.client.as_mut().expect("just connected"))
    }

    fn drop_conn(&mut self, why: &str) {
        self.run.event("connection_lost", why);
        self.client = None;
    }

    fn call(&mut self, call: &proto::Call) -> Result<Option<proto::IntentResult>, Halt> {
        let result = self.conn()?.call(call);
        match result {
            Ok(response) => Ok(response.result.and_then(|r| serde_json::from_value(r).ok())),
            Err(f) => {
                self.drop_conn(&brief(&f));
                Ok(None)
            }
        }
    }

    fn move_(&mut self, vx: f64, vyaw: f64) -> Drive {
        let request = proto::Request::notify(&proto::Call::RobotMove(proto::MoveParams {
            vx,
            vy: 0.0,
            vyaw,
        }));
        if let Err(f) = self.conn()?.send(&request) {
            self.drop_conn(&brief(&f));
        }
        Ok(())
    }

    /// The policy on or off — the pad's Start, without its toggle.
    fn enable(&mut self, on: bool) -> Drive {
        let result = self.call(&proto::Call::RobotEnable(proto::EnableParams {
            on,
            toggle: false,
        }))?;
        if !result.as_ref().is_some_and(|r| r.accepted) {
            self.run.event(
                if on {
                    "enable_refused"
                } else {
                    "disable_refused"
                },
                &json(&result),
            );
        }
        Ok(())
    }

    fn sit_toggle(&mut self, refused: &str) -> Drive {
        let result = self.call(&proto::Call::RobotDo(proto::DoParams {
            skill: "sit_toggle".into(),
        }))?;
        if !result.as_ref().is_some_and(|r| r.accepted) {
            self.run.event(refused, &json(&result));
        }
        Ok(())
    }

    fn wait(&self, seconds: f64) -> Drive {
        let end = Instant::now() + Duration::from_secs_f64(seconds.max(0.0));
        loop {
            if signalled() {
                return Err(Halt::Stop);
            }
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(());
            }
            std::thread::sleep(left.min(Duration::from_millis(100)));
        }
    }

    /// One tick of the 20 Hz loop: this twist, then the rest of the period.
    fn tick(&mut self, vx: f64, vyaw: f64) -> Drive {
        if let Some(f) = self.feed.frame()
            && self.odom.back().is_none_or(|(t, _)| *t != f.t_ns)
        {
            if self.odom.len() == ODOM_HISTORY {
                self.odom.pop_front();
            }
            self.odom
                .push_back((f.t_ns, (f.odom.position[0], f.odom.position[1], f.odom.yaw)));
        }
        self.move_(vx, vyaw)?;
        self.wait(MOVE_PERIOD.as_secs_f64())
    }

    fn now(&self) -> f64 {
        self.run.elapsed()
    }

    fn policy(&self) -> String {
        self.feed.frame().map(|f| f.policy).unwrap_or_default()
    }

    fn driving(&self) -> bool {
        DRIVING.contains(&self.policy().as_str())
    }

    fn pose(&self) -> Option<(f64, f64, f64)> {
        let f = self.feed.frame()?;
        Some((f.odom.position[0], f.odom.position[1], f.odom.yaw))
    }

    /// `safety.limp` from the latest frame: robotd's limp-fall has the robot.
    fn limp(&self) -> bool {
        self.feed.frame().is_some_and(|f| f.safety.limp)
    }

    /// `safety.fallen` from the latest frame, `None` when there is none.
    fn fallen(&self) -> Option<bool> {
        self.feed.frame().map(|f| f.safety.fallen)
    }

    /// The nearest obstacle ahead, when there is depth to say so.
    fn nearest_ahead(&mut self) -> Option<f64> {
        let Some(frame) = self.tof.frame() else {
            if !self.no_tof_logged
                && let Some(why) = self.tof.unavailable()
            {
                self.run
                    .event("no_tof", &format!("{why}; walls are not watched"));
                self.no_tof_logged = true;
            }
            return None;
        };
        let state = self.feed.frame()?;
        let zones = guard::classify(&frame, &state, &self.reprojector)?;
        guard::nearest_ahead(&zones, guard::CORRIDOR_HALF_WIDTH)
    }

    /// Where the robot is in the map now: `mapd`'s pose carried forward by the odometry since
    /// it was computed. `None` while lost, or with no map.
    fn map_pose(&self) -> Option<Pose> {
        let state = self.map.state()?;
        let p = state.pose?;
        let at_stamp = (p.x, p.y, p.yaw);
        let then = self
            .odom
            .iter()
            .min_by_key(|(t, _)| t.abs_diff(state.t_ns))
            .filter(|(t, _)| t.abs_diff(state.t_ns) <= ODOM_MATCH_NS);
        match (then, self.pose()) {
            (Some((_, then)), Some(now)) => Some(room::compose(
                room::compose(at_stamp, room::inverse(*then)),
                now,
            )),
            _ => Some(at_stamp),
        }
    }

    /// The room for this lap, or the first guards, said when it changes.
    fn refresh_room(&mut self) {
        if self.args.no_map {
            if self.no_room.is_none() {
                self.no_room = Some("--no-map".into());
                self.run
                    .event("no_map", "--no-map: odometry square and ToF");
            }
            return;
        }
        if !self.args.map_learns {
            self.freeze_map(true);
        }
        match fetch_room(&self.map_socket) {
            Ok(room) => {
                if self.no_room.take().is_some() || self.map_home.is_none() {
                    let home = *self.map_home.get_or_insert(room.centre());
                    let state = self.map.state();
                    self.run.event(
                        "map",
                        &format!(
                            "keeping off the walls by the map: {} stops, home at ({:.2}, {:.2}), {:.2} m from the nearest wall",
                            state.map_or(0, |s| s.keyframes),
                            home.0,
                            home.1,
                            room.clearance(home.0, home.1)
                        ),
                    );
                }
                self.room = Some(room);
            }
            Err(why) => {
                self.room = None;
                if self.no_room.as_deref() != Some(why.as_str()) {
                    self.run
                        .event("no_map", &format!("{why}; odometry square and ToF"));
                    self.no_room = Some(why);
                }
            }
        }
    }

    /// Freeze the map for the run, or hand mapping back. Said when the answer changes; a `mapd`
    /// that cannot is the run's to note, not to stop over.
    fn freeze_map(&mut self, on: bool) {
        let result = Client::connect_to("mapd", &self.map_socket).and_then(|mut c| {
            let _ = c.writer.set_read_timeout(Some(Duration::from_secs(5)));
            let response = c.call(&proto::Call::MapMode(proto::MapModeParams {
                localize_only: on,
            }))?;
            crate::decode::<proto::MapModeResult>(&crate::result_of(response)?)
        });
        let now = match result {
            Ok(r) => r.localize_only,
            Err(f) => {
                if self.map_frozen != Some(false) {
                    self.run
                        .event("map_mode", &format!("{}; the map may grow", brief(&f)));
                }
                self.map_frozen = Some(false);
                return;
            }
        };
        if self.map_frozen != Some(now) {
            let what = match (on, now) {
                (true, true) => "frozen: localizing only",
                (true, false) => "nothing mapped to freeze: mapd is mapping",
                (false, _) => "mapping again",
            };
            self.run.event("map_mode", what);
        }
        self.map_frozen = Some(now);
    }

    /// Found in the map before a walk, standing and turning on the spot to let `mapd` find the
    /// robot if it has to. Outside the step's slot, like a look behind.
    fn localize(&mut self) -> Result<bool, Halt> {
        if self.room.is_none() {
            return Ok(false);
        }
        if self.map_pose().is_some() {
            return Ok(true);
        }
        // mapd gone, or not mapping: standing still would find nothing.
        if let Some(why) = self.map.unavailable() {
            self.run.event(
                "still_lost",
                &format!("{why}: this walk on the odometry square and ToF"),
            );
            return Ok(false);
        }
        let why = self
            .map
            .state()
            .and_then(|s| s.lost)
            .unwrap_or_else(|| "lost".into());
        let t0 = self.now();
        let end = t0 + self.args.relocalize;
        let mut stops = 0;
        while self.now() < end {
            self.check_battery()?;
            if self.caught_falling()? {
                continue;
            }
            // A stop: still long enough for mapd to open a keyframe and the head to glance round.
            let stop_end = (self.now() + RELOC_STOP).min(end);
            while self.now() < stop_end && self.map_pose().is_none() {
                self.tick(0.0, 0.0)?;
            }
            stops += 1;
            if self.map_pose().is_some() {
                self.run.event(
                    "relocalized",
                    &format!("{why}; found after {:.1} s, {stops} stops", self.now() - t0),
                );
                self.schedule.unanchor();
                return Ok(true);
            }
            // A new view, and the move that closes the stop: a keyframe is placed when the robot
            // moves again, and two that agree are what a relocalization needs.
            if let Some((_, _, yaw)) = self.pose() {
                self.turn_to(yaw + std::f64::consts::FRAC_PI_2)?;
            }
        }
        self.run.event(
            "still_lost",
            &format!(
                "{why}; not found after {:.0} s, {stops} stops: this walk on the odometry square and ToF",
                self.now() - t0
            ),
        );
        self.schedule.unanchor();
        Ok(false)
    }

    /// Face the way the map says this walk can go. Outside the step's slot, like a look behind.
    fn face_for(&mut self, step: &Step, vx: f64) -> Drive {
        let (Some(room), Some(pose), Some(home)) = (&self.room, self.map_pose(), self.map_home)
        else {
            return Ok(());
        };
        let travel = vx * step.seconds;
        let heading = room.heading_for(pose, travel, self.args.wall, home, self.args.square);
        let turn = guard::wrap(heading.yaw - pose.2);
        if heading.run < travel.abs() {
            self.run.event(
                "map_short",
                &format!(
                    "no way has {:.2} m free; the longest has {:.2} m, {:.0}°",
                    travel.abs(),
                    heading.run,
                    turn.to_degrees()
                ),
            );
        }
        if turn.abs() > TURN_TOLERANCE
            && let Some((_, _, yaw)) = self.pose()
        {
            let why = if heading.homeward {
                "towards home"
            } else {
                "off the wall"
            };
            self.run.event(
                "map_turn",
                &format!(
                    "{:.0}° {why}, at ({:.2}, {:.2}), {:.2} m from the nearest wall",
                    turn.to_degrees(),
                    pose.0,
                    pose.1,
                    room.clearance(pose.0, pose.1)
                ),
            );
            self.turn_to(yaw + turn)?;
            self.schedule.unanchor();
        }
        Ok(())
    }

    /// Whether the next tick of a walk at `vx` would bring the robot within `--wall` of a wall,
    /// by the map. `None` when the map has no pose to say.
    fn map_blocked(&self, vx: f64) -> Option<bool> {
        let room = self.room.as_ref()?;
        let pose = self.map_pose()?;
        let direction = if vx < 0.0 {
            pose.2 + std::f64::consts::PI
        } else {
            pose.2
        };
        Some(room.free_run(pose, direction, MAP_LOOKAHEAD, self.args.wall) < MAP_LOOKAHEAD)
    }

    fn check_battery(&self) -> Drive {
        let pct = self.run.shared.lock().expect("shared lock").battery_pct;
        if let Some(pct) = pct.filter(|p| *p <= self.args.hands_off_pct) {
            return Err(Halt::HandsOff(format!("battery at {pct:.1}%")));
        }
        let label = self.policy();
        if label == "rest" || (label == "sit" && !self.our_sit) {
            return Err(Halt::HandsOff(format!(
                "robot went to {label:?} on its own"
            )));
        }
        Ok(())
    }

    /// The policy on, from whatever state the robot is in, and wait until it drives.
    fn ensure_driving(&mut self) -> Drive {
        self.check_battery()?;
        if self.driving() {
            return Ok(());
        }
        if self.policy() == "sit" {
            return self.rise(None);
        }
        self.enable(true)?;
        let deadline = Instant::now() + Duration::from_secs(30);
        while !self.driving() {
            if Instant::now() > deadline {
                self.run.event(
                    "enable_timeout",
                    &format!("policy still {:?}", self.policy()),
                );
                return Ok(());
            }
            self.tick(0.0, 0.0)?;
        }
        Ok(())
    }

    /// Out of the seat, waiting until it drives or `until` (lap time) — or 15 s.
    fn rise(&mut self, until: Option<f64>) -> Drive {
        self.check_battery()?;
        self.sit_toggle("rise_refused")?;
        self.our_sit = false;
        let until = until.unwrap_or(self.now() + 15.0);
        while !self.driving() && self.now() < until {
            self.wait(0.1)?;
        }
        if !self.driving() {
            self.run
                .event("rise_timeout", &format!("policy {:?}", self.policy()));
        }
        Ok(())
    }

    // ── the steps ──

    fn run_forever(&mut self) -> Drive {
        if self.policy() == "sit" {
            // Found seated at the start: that seat is someone's before this run, not robotd's
            // battery shutdown, and standing it up is the first thing to do.
            self.our_sit = true;
        }
        self.ensure_driving()?;
        let deadline = Instant::now() + Duration::from_secs(5);
        let (x, y) = loop {
            if let Some((x, y, _)) = self.pose() {
                break (x, y);
            }
            if Instant::now() > deadline {
                return Err(Halt::Fatal("no robot.state from robotd".into()));
            }
            self.wait(0.1)?;
        };
        self.home = (x, y);
        self.run.shared.lock().expect("shared lock").home = Some((x, y));
        self.run
            .event("home", &format!("odometry start ({x:.3}, {y:.3})"));

        let steps = lap::lap(self.args.speed, self.args.vyaw);
        for n in 1.. {
            self.run.shared.lock().expect("shared lock").lap = Some(n);
            self.run
                .event("lap_start", &format!("lap {n} ({})", lap::VERSION));
            self.refresh_room();
            for step in &steps {
                self.run.shared.lock().expect("shared lock").step = step.name.into();
                self.run.set_activity(activity(step.action), step.name);
                match step.action {
                    Action::Walk { vx, vyaw } => self.walk(step, vx, vyaw)?,
                    Action::Fall { vx } => self.fall(vx)?,
                    Action::Sit => self.sit(step)?,
                    Action::Stand => self.stand(step)?,
                }
            }
        }
        Ok(())
    }

    /// A fall in the middle of a step that did not ask for one: the same get-up, and the step is
    /// over. Returns whether it happened.
    fn caught_falling(&mut self) -> Result<bool, Halt> {
        if self.fallen() != Some(true) {
            return Ok(false);
        }
        self.run.event("fallen_unprovoked", "");
        self.get_up(Getup::unprovoked(self.timing))?;
        Ok(true)
    }

    fn walk(&mut self, step: &Step, vx: f64, vyaw: f64) -> Drive {
        if !self.driving() {
            self.ensure_driving()?;
        }
        // A turn on the spot goes nowhere: with the map, no guard has anything to say about it.
        // Without, the odometry square still watches it, as it always did.
        if vx == 0.0 && self.room.is_some() {
            return self.turn_step(step, vyaw);
        }
        if self.localize()? {
            self.face_for(step, vx)?;
            return self.walk_by_map(step, vx, vyaw);
        }
        if vx < 0.0 {
            self.look_back(vx * step.seconds)?;
        }
        let end = self.schedule.deadline(step.seconds, self.now());
        let half = self.args.square;
        let mut guard = Guard::Off;
        while self.now() < end {
            self.check_battery()?;
            if self.caught_falling()? {
                return self.hold_until(end);
            }
            let pose = self.pose();
            self.follow_odometry(pose);
            let walking_forward = match guard {
                Guard::Off => vx > 0.0,
                Guard::Square => true,
                Guard::Wall | Guard::Done => false,
            };
            if walking_forward && let Some(d) = self.nearest_ahead().filter(|d| *d < self.args.wall)
            {
                self.run.event("wall_guard", &format!("{d:.2} m ahead"));
                // A wall on the way back to the start means the start is wrong: the robot never
                // walked through a wall, so odometry drifted it behind one. Walking back to it
                // again would pin the robot to that wall for every step to come.
                if guard == Guard::Square
                    && let Some((x, y, _)) = pose
                {
                    let (dx, dy) = (x - self.home.0, y - self.home.1);
                    self.home = (x, y);
                    self.run.shared.lock().expect("shared lock").home = Some(self.home);
                    self.run.event(
                        "home_moved",
                        &format!("a wall on the way back; the start is here now, ({dx:.2}, {dy:.2}) m from the old one"),
                    );
                }
                guard = Guard::Wall;
            }
            if guard == Guard::Off
                && let Some((x, y, _)) = pose
                && guard::outside_square((x, y), self.home, half)
            {
                let (dx, dy) = (x - self.home.0, y - self.home.1);
                self.run.event(
                    "square_guard",
                    &format!("({dx:.2}, {dy:.2}) m from start; walking back"),
                );
                guard = Guard::Square;
            }
            let (cmd_vx, cmd_vyaw) = match guard {
                Guard::Off => (vx, vyaw),
                Guard::Wall => {
                    let clear = self
                        .nearest_ahead()
                        .is_none_or(|d| d >= self.args.wall * WALL_CLEAR);
                    if clear {
                        guard = Guard::Done;
                        (0.0, 0.0)
                    } else {
                        (0.0, self.args.vyaw)
                    }
                }
                Guard::Square => match pose {
                    Some((x, y, yaw))
                        if guard::outside_square((x, y), self.home, half * SQUARE_RETURN) =>
                    {
                        guard::toward(
                            (x, y, yaw),
                            self.home,
                            self.args.speed,
                            self.args.vyaw,
                            TURN_THRESHOLD,
                        )
                    }
                    Some(_) => {
                        guard = Guard::Done;
                        (0.0, 0.0)
                    }
                    None => (0.0, 0.0),
                },
                Guard::Done => (0.0, 0.0),
            };
            self.tick(cmd_vx, cmd_vyaw)?;
        }
        Ok(())
    }

    /// A walk with the map: the step's own twist until the map says the next tick would come
    /// within `--wall` of a wall, then standing for the rest of the slot. Lost on the way — a
    /// bump, a contradiction — it stands too: the walk was chosen for a pose it no longer has.
    fn walk_by_map(&mut self, step: &Step, vx: f64, vyaw: f64) -> Drive {
        let end = self.schedule.deadline(step.seconds, self.now());
        while self.now() < end {
            self.check_battery()?;
            if self.caught_falling()? {
                return self.hold_until(end);
            }
            let pose = self.pose();
            self.follow_odometry(pose);
            match self.map_blocked(vx) {
                Some(false) => {}
                Some(true) => {
                    let at = self.map_pose().unwrap_or_default();
                    let near = self.room.as_ref().map_or(0.0, |r| r.clearance(at.0, at.1));
                    self.run.event(
                        "map_guard",
                        &format!(
                            "{} at ({:.2}, {:.2}), {near:.2} m from the nearest wall; standing",
                            step.name, at.0, at.1
                        ),
                    );
                    return self.hold_until(end);
                }
                None => {
                    let why = self
                        .map
                        .state()
                        .and_then(|s| s.lost)
                        .or_else(|| self.map.unavailable())
                        .unwrap_or_else(|| "no pose".into());
                    self.run.event(
                        "map_lost",
                        &format!("{} mid-walk: {why}; standing", step.name),
                    );
                    return self.hold_until(end);
                }
            }
            self.tick(vx, vyaw)?;
        }
        Ok(())
    }

    /// A turn on the spot for the step's slot.
    fn turn_step(&mut self, step: &Step, vyaw: f64) -> Drive {
        let end = self.schedule.deadline(step.seconds, self.now());
        while self.now() < end {
            self.check_battery()?;
            if self.caught_falling()? {
                return self.hold_until(end);
            }
            let pose = self.pose();
            self.follow_odometry(pose);
            self.tick(0.0, vyaw)?;
        }
        Ok(())
    }

    /// Before backing up `travel` m (negative), see what is behind: the ToF only looks forward,
    /// so turn the body a quarter left and the head a quarter left — the sensor then faces where
    /// the robot is about to back into. Clear for the whole backing up plus `--wall`: turn back
    /// and go. A wall nearer than that: carry on to a half turn, so the robot faces the wall and
    /// backs away from it, and the forward walks after have the live guard.
    ///
    /// Outside the step's slot, like a get-up: the look takes a couple of seconds, and the step
    /// keeps its full length after it.
    fn look_back(&mut self, travel: f64) -> Drive {
        let Some((_, _, yaw0)) = self.pose() else {
            return Ok(());
        };
        let need = travel.abs() + self.args.wall;
        self.call(&proto::Call::RobotLook(proto::LookParams {
            x: 0.0,
            y: 1.0,
            z: 0.0,
            neck_pitch: 0.0,
        }))?;
        self.turn_to(yaw0 + std::f64::consts::FRAC_PI_2)?;
        // The head has to have arrived, and the frames to be from where it arrived.
        self.hold(LOOK_SETTLE)?;
        let mut behind: Option<f64> = None;
        let end = self.now() + LOOK_READ;
        while self.now() < end {
            if let Some(d) = self.nearest_left() {
                behind = Some(behind.map_or(d, |b| b.min(d)));
            }
            self.tick(0.0, 0.0)?;
        }
        self.call(&proto::Call::RobotLook(proto::LookParams {
            x: 1.0,
            y: 0.0,
            z: 0.0,
            neck_pitch: 0.0,
        }))?;
        match behind.filter(|d| *d < need) {
            Some(d) => {
                self.run.event(
                    "look_back",
                    &format!("wall {d:.2} m behind, {need:.2} m needed; turned round"),
                );
                self.turn_to(yaw0 + std::f64::consts::PI)?;
            }
            None => {
                let seen = behind.map_or("nothing".into(), |d| format!("{d:.2} m"));
                self.run
                    .event("look_back", &format!("clear: {seen} behind"));
                self.turn_to(yaw0)?;
            }
        }
        self.schedule.unanchor();
        Ok(())
    }

    /// Turn on the spot to `target` yaw (odometry), the short way round. Gives up after
    /// [`TURN_TIMEOUT`]: a robot that cannot find its heading should still get on with the lap.
    fn turn_to(&mut self, target: f64) -> Drive {
        let end = self.now() + TURN_TIMEOUT;
        while self.now() < end {
            self.check_battery()?;
            let Some((_, _, yaw)) = self.pose() else {
                break;
            };
            let err = guard::wrap(target - yaw);
            if err.abs() < TURN_TOLERANCE {
                break;
            }
            self.tick(0.0, self.args.vyaw.abs().copysign(err))?;
        }
        Ok(())
    }

    /// Stand still for `seconds`, keeping the twist alive.
    fn hold(&mut self, seconds: f64) -> Drive {
        let end = self.now() + seconds;
        while self.now() < end {
            self.tick(0.0, 0.0)?;
        }
        Ok(())
    }

    /// The nearest obstacle on the robot's left, with the head turned to look there.
    fn nearest_left(&mut self) -> Option<f64> {
        let frame = self.tof.frame()?;
        let state = self.feed.frame()?;
        let zones = guard::classify(&frame, &state, &self.reprojector)?;
        guard::nearest_left(&zones, guard::CORRIDOR_HALF_WIDTH)
    }

    /// Cut the policy mid-stride, give it back, and see the robot down and up again.
    fn fall(&mut self, vx: f64) -> Drive {
        self.check_battery()?;
        // Down already (the walk into it fell on its own): get up first, then cut anyway, so
        // the lap keeps its six cuts.
        self.caught_falling()?;
        let way = if vx >= 0.0 { "forward" } else { "backward" };
        if self.args.fall == "cut" {
            self.run.event(
                "fall_triggered",
                &format!("{way}, cut {:.2} s", self.args.cut),
            );
            self.cut_policy(self.args.cut)?;
        } else {
            self.trip(vx, way)?;
        }
        self.get_up(Getup::provoked(self.timing))?;
        self.schedule.unanchor();
        Ok(())
    }

    /// Hold the legs mid-stride and keep asking for the walk through it: the policy is still
    /// driving, and a foot caught while walking is caught while the walk goes on.
    fn trip(&mut self, vx: f64, way: &str) -> Drive {
        let (ms, legs) = self.trip_setting();
        self.run.event(
            "fall_triggered",
            &format!("{way}, trip {ms} ms {}", legs.as_str()),
        );
        let result = self.call(&proto::Call::RobotTrip(proto::TripParams { ms, legs }))?;
        if !result.as_ref().is_some_and(|r| r.accepted) {
            self.run.event("trip_refused", &json(&result));
        }
        let end = self.now() + f64::from(ms) / 1000.0;
        while self.now() < end {
            self.tick(vx, 0.0)?;
        }
        Ok(())
    }

    /// The trip for this fall: a flag wins, else `[stress]` as the file says now — read per fall
    /// so `robotctl configure` tunes a running test. A file that does not load keeps the
    /// built-in values and says so, rather than stopping a run over an edit.
    fn trip_setting(&self) -> (u32, proto::TripLegs) {
        let file = match robotd_params::Params::load(&self.args.config, false) {
            Ok(params) => params.stress,
            Err(e) => {
                self.run
                    .event("config_unreadable", &format!("{e}; built-in trip"));
                robotd_params::StressParams::default()
            }
        };
        (
            self.args.trip_ms.unwrap_or(file.trip_ms),
            self.args
                .trip_legs
                .as_deref()
                .map_or(file.trip_legs, crate::trip_legs),
        )
    }

    /// Watch a get-up from the policy coming back, at 20 Hz, until a verdict ends it.
    fn get_up(&mut self, mut g: Getup) -> Drive {
        let t0 = self.now();
        let mut seen_down = false;
        let mut was_limp = false;
        loop {
            // robotd's limp-fall: whether this fall went through it, and when. A fall that never
            // did landed stiff, and its get-up is not the same measurement.
            let limp = self.limp();
            if limp && !was_limp {
                self.run.event("limp", &format!("{:.2} s", self.now() - t0));
            }
            was_limp = limp;
            let fallen = self.fallen();
            if fallen == Some(true) && !seen_down {
                seen_down = true;
                self.run.event("fallen", "");
            }
            if self.policy() == "sit" {
                // A robot that reads as seated is handed to the sit/stand network: that seat is
                // this test's, not robotd's battery shutdown.
                self.our_sit = true;
            }
            match g.tick(self.now() - t0, fallen) {
                Verdict::Waiting => {}
                Verdict::NoFall => {
                    self.run.event("no_fall", "the policy caught it");
                    return Ok(());
                }
                Verdict::GotUp {
                    seconds,
                    after_reset,
                } => {
                    let how = if after_reset { " after the reset" } else { "" };
                    self.run.event("got_up", &format!("{seconds:.2} s{how}"));
                    if self.policy() == "sit" {
                        self.run.event("seated", "up on its seat; standing it");
                        self.rise(None)?;
                    }
                    return Ok(());
                }
                Verdict::Reset => {
                    let down = self.timing.timeout;
                    self.run
                        .event("getup_failed", &format!("still down after {down:.0} s"));
                    self.check_battery_level()?;
                    self.cut_policy(self.args.reset)?;
                    g.resumed(self.now() - t0);
                    self.run.event(
                        "long_reset",
                        &format!("{:.0} s off, policy back", self.args.reset),
                    );
                }
                Verdict::GaveUp => {
                    self.run
                        .event("gave_up", "still down after the reset; torque off");
                    let result = self.call(&proto::Call::RobotRelax)?;
                    self.run.event("relax", &json(&result));
                    self.run.set_activity("gave_up", "");
                    return Err(Halt::HandsOff(GAVE_UP.into()));
                }
            }
            // Zero twist: command magnitude picks the standing network.
            self.tick(0.0, 0.0)?;
        }
    }

    /// The battery half of [`Self::check_battery`]: mid-get-up, a seat is the get-up's.
    fn check_battery_level(&self) -> Drive {
        let pct = self.run.shared.lock().expect("shared lock").battery_pct;
        match pct.filter(|p| *p <= self.args.hands_off_pct) {
            Some(pct) => Err(Halt::HandsOff(format!("battery at {pct:.1}%"))),
            None => Ok(()),
        }
    }

    fn sit(&mut self, step: &Step) -> Drive {
        let end = self.schedule.deadline(step.seconds, self.now());
        self.check_battery()?;
        if self.caught_falling()? {
            return self.hold_until(end);
        }
        self.sit_toggle("sit_refused")?;
        self.our_sit = true;

        // Seated: nothing to send. The seat is latched, and a twist would not move a sitting
        // duck. Checked every tenth of a second, so a shutdown that starts while seated is seen.
        while self.now() < end {
            self.wait(0.1)?;
            self.check_battery()?;
        }
        if self.policy() != "sit" {
            self.run
                .event("sit_timeout", &format!("policy {:?}", self.policy()));
        }
        Ok(())
    }

    fn stand(&mut self, step: &Step) -> Drive {
        let end = self.schedule.deadline(step.seconds, self.now());
        if self.policy() == "sit" || self.our_sit {
            self.rise(Some(end))?;
        }
        while self.now() < end {
            self.check_battery()?;
            if self.caught_falling()? {
                return self.hold_until(end);
            }
            self.tick(0.0, 0.0)?;
        }
        Ok(())
    }

    /// The rest of an abandoned step's slot, standing still, so the next step starts when the
    /// lap says. A get-up that ran past the slot leaves nothing to wait.
    fn hold_until(&mut self, end: f64) -> Drive {
        while self.now() < end {
            self.check_battery()?;
            self.tick(0.0, 0.0)?;
        }
        Ok(())
    }

    /// A `robotd` restart zeroes odometry. Move the start with it, so the square stays where it
    /// was on the floor rather than sending the robot off to where the old origin now reads.
    fn follow_odometry(&mut self, pose: Option<(f64, f64, f64)>) {
        let Some((x, y, _)) = pose else {
            return;
        };
        if let Some(before) = self.last_xy
            && guard::odometry_jumped(before, (x, y))
        {
            let (dx, dy) = (x - before.0, y - before.1);
            self.home = (self.home.0 + dx, self.home.1 + dy);
            self.run.shared.lock().expect("shared lock").home = Some(self.home);
            self.run.event(
                "odometry_jump",
                &format!("moved ({dx:.2}, {dy:.2}) m between reads; start moved with it"),
            );
        }
        self.last_xy = Some((x, y));
    }

    /// One call for help: two quacks and a wheee of about a second.
    fn call_for_help(&mut self) -> Drive {
        for _ in 0..2 {
            self.call(&proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Chirp,
                hold: None,
            }))?;
            self.wait(0.4)?;
        }
        // The wheee is a hold that decays unless renewed, as the pad's D-pad up renews it.
        let end = Instant::now() + Duration::from_millis(1200);
        while Instant::now() < end {
            self.sound_hold(true)?;
            self.wait(0.1)?;
        }
        self.sound_hold(false)
    }

    fn sound_hold(&mut self, hold: bool) -> Drive {
        let request = proto::Request::notify(&proto::Call::RobotSound(proto::SoundParams {
            tag: proto::SoundTag::Wheee,
            hold: Some(hold),
        }));
        if let Err(f) = self.conn()?.send(&request) {
            self.drop_conn(&brief(&f));
        }
        Ok(())
    }

    /// Given up: call for help every `--call-every` until the run is stopped. Nothing moves —
    /// the robot is limp on the floor — so a robotd that is gone just means a silent call.
    fn call_for_help_until_stopped(&mut self) {
        let every = self.args.call_every;
        if every <= 0.0 {
            return wait_for_signal(&self.run);
        }
        self.run.event(
            "calling_for_help",
            &format!("two quacks and a wheee every {every:.0} s"),
        );
        loop {
            if self.call_for_help().is_err() || self.wait(every).is_err() {
                break;
            }
        }
        self.run.event("signal", signal_name());
    }

    /// The policy off for `seconds` and back. A stop in the middle leaves it off, which is
    /// where [`Self::park`] puts it anyway.
    fn cut_policy(&mut self, seconds: f64) -> Drive {
        self.enable(false)?;
        self.wait(seconds)?;
        self.enable(true)
    }

    /// On the way out: stop, and the policy off — the pad's Start, holding the home pose. No
    /// waits: with robotd gone, a stop must not hang reconnecting.
    fn park(&mut self) {
        let _ = self.call(&proto::Call::RobotStop);
        let _ = self.enable(false);
        self.release_map();
    }

    /// Mapping handed back, if this run froze the map.
    fn release_map(&mut self) {
        if self.map_frozen == Some(true) {
            self.freeze_map(false);
        }
    }
}

/// The `activity` column for a step.
fn activity(action: Action) -> &'static str {
    match action {
        Action::Walk { .. } => "walk",
        Action::Fall { .. } => "fall",
        Action::Sit => "sit",
        Action::Stand => "stand",
    }
}

fn foreground(sockets: &Sockets, args: StressArgs) -> Result<(), Failure> {
    let (robot_socket, tof_socket) = (sockets.robot, sockets.tof);
    runlog::need_robotd(robot_socket)?;
    let out = args.out.clone().unwrap_or_else(|| UNIT.default_out());
    let run = runlog::open_run(
        &out,
        serde_json::json!({ "lap": lap::VERSION, "args": &args }),
    )?;
    run.event("start", &format!("logging to {}", out.display()));

    runlog::install_signal_handlers();

    let feed = StateFeed::spawn(robot_socket.to_owned());
    Sampler::new(run.clone(), feed.clone(), robot_socket.to_owned(), &out)
        .map_err(|e| Failure::new(exit::FAILED, format!("opening samples.csv: {e}")))?
        .spawn(Duration::from_secs_f64(args.interval.max(0.5)));

    let timing = Timing {
        timeout: args.getup_timeout,
        reset: args.reset,
        ..Timing::default()
    };
    let mut driver = Driver {
        run: run.clone(),
        feed,
        tof: TofFeed::spawn(tof_socket.to_owned()),
        reprojector: kinematics::tof::Reprojector::alpha(),
        args,
        timing,
        socket: robot_socket.to_owned(),
        client: None,
        schedule: Schedule::default(),
        home: (0.0, 0.0),
        our_sit: false,
        no_tof_logged: false,
        last_xy: None,
        map: MapFeed::spawn(sockets.map.to_owned()),
        map_socket: sockets.map.to_owned(),
        room: None,
        map_home: None,
        no_room: None,
        odom: VecDeque::with_capacity(ODOM_HISTORY),
        map_frozen: None,
    };
    match driver.run_forever() {
        Err(Halt::HandsOff(why)) => {
            // Keep logging, touch nothing: after a give-up the robot is limp on the floor, and
            // near empty the rows to the power-off are the end of the curve.
            if why == GAVE_UP {
                driver.call_for_help_until_stopped();
            } else {
                run.set_activity("hands_off", &why);
                wait_for_signal(&run);
            }
            driver.release_map();
            run.event("stop", "stopped while hands-off");
            Ok(())
        }
        Err(Halt::Fatal(why)) => {
            run.event("fatal", &why);
            driver.park();
            Err(Failure::new(exit::FAILED, why))
        }
        // `run_forever` only returns through a halt; `Ok` is here for the type.
        Err(Halt::Stop) | Ok(()) => {
            run.event("signal", signal_name());
            driver.park();
            run.event("stop", "robot stopped, policy off");
            Ok(())
        }
    }
}
