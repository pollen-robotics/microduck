//! `robotctl endurance` — run a duck through ordinary use until the battery is empty, and log
//! what that costs: charge, board heat, throttling.
//!
//! **The driver is an ordinary client.** It asks `robotd` for what a person with a pad would —
//! `robot.enable`, `robot.move` at 20 Hz, `robot.do sit_toggle` — and cycles through walking,
//! standing and sitting in random order and for random lengths, so the load is the mix a duck
//! sees in use rather than one steady state. The camera is not streamed.
//!
//! **It stays home.** Walking is goal-to-goal on odometry: goals are drawn inside
//! `--goal-radius` of where it started, and past `--fence` it abandons the goal and walks back.
//! Commands are full stick — `[pad_drive]`'s defaults — because a gentler one does not always
//! get the gait going from a stand.
//!
//! **It runs until the board dies.** `robotd` sits the robot down and powers off at 0%
//! (`[safety] battery_empty_shutdown`). Below `--hands-off-pct` this stops driving, so it never
//! sends a `sit_toggle` into the middle of that shutdown, and only logs. Every row is `fsync`ed
//! as it is written, so the log is whole up to the last sample after the reboot.
//!
//! **It runs in the background.** `robotctl endurance` re-runs itself as a transient systemd
//! unit and returns, so the ssh session can go; `status` and `stop` find it again. The columns
//! are what `scripts/endurance-plot.py` reads.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::{Args, Subcommand};
use duck_ipc_proto as proto;
use serde::Serialize;

use crate::runlog::{
    self, Halt, Run, Sampler, StateFeed, Unit, brief, json, signal_name, signalled, wait_for_signal,
};
use crate::{Client, Failure, exit};

const UNIT: Unit = Unit {
    name: "robotctl-endurance",
    runs_dir: "/var/lib/endurance",
    command: "endurance",
    description: "robotctl endurance: battery and heat test",
    stopped: "the robot is left standing",
};

/// `robotd`'s deadman zeroes the twist 500 ms after the last intent; 20 Hz is well inside it.
const MOVE_PERIOD: Duration = Duration::from_millis(50);

#[derive(Subcommand, Debug)]
pub enum EnduranceCommand {
    /// Whether a run is going, and its latest sample and events.
    Status,
    /// Stop the background run. The robot is stopped and left standing.
    Stop,
}

#[derive(Args, Debug, Clone, Serialize)]
pub struct EnduranceArgs {
    /// Run directory. Default: /var/lib/endurance/<hostname>-<UTC time>. An existing one is
    /// appended to, so a restarted run stays one battery curve.
    #[arg(long)]
    out: Option<PathBuf>,
    /// Seconds between samples.
    #[arg(long, default_value_t = 10.0)]
    interval: f64,
    /// Seed for the activity schedule; random when absent.
    #[arg(long)]
    seed: Option<u64>,
    /// Log only, never move the robot — the idle-drain baseline.
    #[arg(long)]
    dry_run: bool,
    /// Run here instead of in the background. What the background unit itself runs.
    #[arg(long)]
    foreground: bool,

    /// Relative share of activities that are walks.
    #[arg(long, default_value_t = 0.40, help_heading = "Behaviour")]
    w_walk: f64,
    /// Relative share that are stands.
    #[arg(long, default_value_t = 0.35, help_heading = "Behaviour")]
    w_stand: f64,
    /// Relative share that are sits.
    #[arg(long, default_value_t = 0.25, help_heading = "Behaviour")]
    w_sit: f64,
    /// Walk length, MIN:MAX seconds.
    #[arg(long, default_value = "20:60", value_parser = span, help_heading = "Behaviour")]
    walk_s: (f64, f64),
    /// Stand length, MIN:MAX seconds.
    #[arg(long, default_value = "20:90", value_parser = span, help_heading = "Behaviour")]
    stand_s: (f64, f64),
    /// Sit length, MIN:MAX seconds.
    #[arg(long, default_value = "30:120", value_parser = span, help_heading = "Behaviour")]
    sit_s: (f64, f64),
    /// Walking speed, m/s. Full stick: a gentler command does not always start the gait.
    #[arg(long, default_value_t = 0.3, help_heading = "Behaviour")]
    speed: f64,
    /// Turn rate, rad/s.
    #[arg(long, default_value_t = 1.5, help_heading = "Behaviour")]
    vyaw: f64,
    /// Goals are drawn inside this many metres of the start.
    #[arg(long, default_value_t = 0.30, help_heading = "Behaviour")]
    goal_radius: f64,
    /// Past this many metres from the start, walk straight back.
    #[arg(long, default_value_t = 0.45, help_heading = "Behaviour")]
    fence: f64,
    /// A goal is reached within this many metres.
    #[arg(long, default_value_t = 0.08, help_heading = "Behaviour")]
    arrive: f64,
    /// Heading error above which it turns on the spot, rad.
    #[arg(long, default_value_t = 0.6, help_heading = "Behaviour")]
    turn_threshold: f64,
    /// At or below this battery %, stop driving and let robotd's own empty-battery shutdown run
    /// undisturbed.
    #[arg(long, default_value_t = 2.0, help_heading = "Behaviour")]
    hands_off_pct: f64,
}

fn span(text: &str) -> Result<(f64, f64), String> {
    let (lo, hi) = text.split_once(':').unwrap_or((text, text));
    let parse = |s: &str| {
        s.trim()
            .parse::<f64>()
            .map_err(|e| format!("{text:?}: {e}"))
    };
    let (lo, hi) = (parse(lo)?, parse(hi)?);
    if !(lo > 0.0 && lo <= hi) {
        return Err(format!("{text:?}: want MIN:MAX seconds, 0 < MIN <= MAX"));
    }
    Ok((lo, hi))
}

pub fn run(
    robot_socket: &Path,
    command: Option<EnduranceCommand>,
    args: EnduranceArgs,
) -> Result<(), Failure> {
    match command {
        Some(EnduranceCommand::Status) => UNIT.status(),
        Some(EnduranceCommand::Stop) => UNIT.stop(),
        None if args.foreground => foreground(robot_socket, args),
        None => UNIT.start(robot_socket, args.out.as_deref()),
    }
}

// ── the behaviour ───────────────────────────────────────────────────────────────────────────

/// SplitMix64. The schedule needs to be random and, with `--seed`, repeatable — not good.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn weighted<'a>(&mut self, choices: &[(&'a str, f64)]) -> &'a str {
        let total: f64 = choices.iter().map(|(_, w)| w).sum();
        let mut pick = self.unit() * total;
        for (name, w) in choices {
            if pick < *w {
                return name;
            }
            pick -= w;
        }
        choices.last().expect("at least one choice").0
    }
}

fn steer(pose: (f64, f64, f64), goal: (f64, f64), args: &EnduranceArgs) -> (f64, f64) {
    crate::stress::guard::toward(pose, goal, args.speed, args.vyaw, args.turn_threshold)
}

fn new_goal(rng: &mut Rng, here: (f64, f64), home: (f64, f64), radius: f64) -> (f64, f64) {
    for _ in 0..50 {
        let r = radius * rng.unit().sqrt();
        let a = rng.uniform(-std::f64::consts::PI, std::f64::consts::PI);
        let goal = (home.0 + r * a.cos(), home.1 + r * a.sin());
        if (goal.0 - here.0).hypot(goal.1 - here.1) >= 0.6 * radius {
            return goal;
        }
    }
    home
}

struct Driver {
    run: Arc<Run>,
    feed: StateFeed,
    args: EnduranceArgs,
    socket: PathBuf,
    rng: Rng,
    client: Option<Client>,
    /// Whether the robot is in a sit this driver asked for. One it did not ask for is robotd's
    /// battery shutdown, and must not be undone.
    our_sit: bool,
    /// Set while parking, so the waits that parking needs are not cut short by the signal that
    /// asked for it.
    parking: bool,
}

type Step = Result<(), Halt>;

const DRIVING: [&str; 2] = ["walk", "stand"];

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

    fn move_(&mut self, vx: f64, vyaw: f64) -> Step {
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

    fn wait(&self, seconds: f64) -> Step {
        let end = Instant::now() + Duration::from_secs_f64(seconds.max(0.0));
        loop {
            if signalled() && !self.parking {
                return Err(Halt::Stop);
            }
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(());
            }
            std::thread::sleep(left.min(Duration::from_millis(100)));
        }
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

    /// Why the robot should not be driven right now, or None.
    fn down(&self) -> Option<&'static str> {
        let Some(frame) = self.feed.frame() else {
            return Some("no state from robotd");
        };
        if frame.safety.fallen {
            Some("fallen")
        } else if frame.safety.picked_up {
            Some("picked up")
        } else {
            None
        }
    }

    fn check_battery(&self) -> Step {
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

    /// Zero twist at the intent rate. Standing is a policy driving, not an absence of intents.
    fn hold_still(&mut self, seconds: f64) -> Step {
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            self.move_(0.0, 0.0)?;
            self.wait(MOVE_PERIOD.as_secs_f64())?;
        }
        Ok(())
    }

    fn wait_until_upright(&mut self) -> Step {
        let Some(reason) = self.down() else {
            return Ok(());
        };
        self.run.set_activity("paused", reason);
        while self.down().is_some() {
            self.move_(0.0, 0.0)?;
            self.wait(0.5)?;
        }
        self.run.event("resumed", "upright and on the floor again");
        self.hold_still(2.0)
    }

    fn ensure_driving(&mut self) -> Step {
        self.check_battery()?;
        if self.driving() {
            return Ok(());
        }
        if self.policy() == "sit" {
            return self.rise();
        }
        let result = self.call(&proto::Call::RobotEnable(proto::EnableParams {
            on: true,
            toggle: false,
        }))?;
        self.run.event("enable", &json(&result));
        let deadline = Instant::now() + Duration::from_secs(30);
        while !self.driving() {
            if Instant::now() > deadline {
                self.run.event(
                    "enable_timeout",
                    &format!("policy still {:?}", self.policy()),
                );
                return Ok(());
            }
            self.move_(0.0, 0.0)?;
            self.wait(0.2)?;
        }
        self.hold_still(2.0)
    }

    fn stand(&mut self, seconds: f64) -> Step {
        self.run.set_activity("stand", &format!("{seconds:.0}s"));
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            if self.down().is_some() {
                return Ok(());
            }
            self.check_battery()?;
            self.move_(0.0, 0.0)?;
            self.wait(MOVE_PERIOD.as_secs_f64())?;
        }
        Ok(())
    }

    fn sit(&mut self, seconds: f64) -> Step {
        self.run.set_activity("sit", &format!("{seconds:.0}s"));
        self.hold_still(0.5)?;
        let result = self.call(&proto::Call::RobotDo(proto::DoParams {
            skill: "sit_toggle".into(),
        }))?;
        if !result.as_ref().is_some_and(|r| r.accepted) {
            self.run.event("sit_refused", &json(&result));
            return self.hold_still(seconds);
        }
        self.our_sit = true;
        let deadline = Instant::now() + Duration::from_secs(15);
        while self.policy() != "sit" && Instant::now() < deadline {
            self.wait(0.1)?;
        }
        if self.policy() != "sit" {
            self.run
                .event("sit_timeout", &format!("policy {:?}", self.policy()));
        }
        // Seated: nothing to send. The seat is latched, and a twist would not move a sitting
        // duck. Checked once a second, so a shutdown that starts while seated is not stood up.
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            self.wait(1.0)?;
            self.check_battery()?;
        }
        self.rise()
    }

    fn rise(&mut self) -> Step {
        self.check_battery()?;
        self.our_sit = false;
        let result = self.call(&proto::Call::RobotDo(proto::DoParams {
            skill: "sit_toggle".into(),
        }))?;
        if !result.as_ref().is_some_and(|r| r.accepted) {
            self.run.event("rise_refused", &json(&result));
        }
        let deadline = Instant::now() + Duration::from_secs(15);
        while !self.driving() && Instant::now() < deadline {
            self.wait(0.1)?;
        }
        if !self.driving() {
            self.run
                .event("rise_timeout", &format!("policy {:?}", self.policy()));
        }
        self.hold_still(1.0)
    }

    fn walk(&mut self, seconds: f64) -> Step {
        self.run.set_activity("walk", &format!("{seconds:.0}s"));
        let home = self
            .run
            .shared
            .lock()
            .expect("shared lock")
            .home
            .expect("home is set before the first activity");
        let mut goal: Option<(f64, f64)> = None;
        let mut returning = false;
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            if self.down().is_some() {
                return Ok(());
            }
            self.check_battery()?;
            let Some((x, y, yaw)) = self.pose() else {
                self.move_(0.0, 0.0)?;
                self.wait(MOVE_PERIOD.as_secs_f64())?;
                continue;
            };
            let from_home = (x - home.0).hypot(y - home.1);
            if from_home > self.args.fence && !returning {
                self.run.event(
                    "fence",
                    &format!("{from_home:.2} m from start; walking back"),
                );
                goal = Some(home);
                returning = true;
            }
            let target = *goal.get_or_insert_with(|| {
                new_goal(&mut self.rng, (x, y), home, self.args.goal_radius)
            });
            if (target.0 - x).hypot(target.1 - y) < self.args.arrive {
                goal = None;
                returning = false;
                // A short pause at each goal, as a duck being steered around does.
                let pause = self.rng.uniform(0.5, 2.0);
                self.hold_still(pause)?;
                continue;
            }
            let (vx, vyaw) = steer((x, y, yaw), target, &self.args);
            self.move_(vx, vyaw)?;
            self.wait(MOVE_PERIOD.as_secs_f64())?;
        }
        self.hold_still(0.5)
    }

    fn run_forever(&mut self) -> Step {
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
        self.run.shared.lock().expect("shared lock").home = Some((x, y));
        self.run
            .event("home", &format!("odometry start ({x:.3}, {y:.3})"));

        let a = &self.args;
        let all = [
            ("walk", a.w_walk, a.walk_s),
            ("stand", a.w_stand, a.stand_s),
            ("sit", a.w_sit, a.sit_s),
        ];
        let mut last = "";
        loop {
            self.wait_until_upright()?;
            self.ensure_driving()?;
            // Never the same activity twice in a row: three stands in a row is one long stand.
            let mut choices: Vec<(&str, f64)> = all
                .iter()
                .filter(|(k, w, _)| *w > 0.0 && *k != last)
                .map(|(k, w, _)| (*k, *w))
                .collect();
            if choices.is_empty() {
                choices = all.iter().map(|(k, w, _)| (*k, *w)).collect();
            }
            let kind = self.rng.weighted(&choices);
            let (lo, hi) = all.iter().find(|(k, _, _)| *k == kind).expect("a choice").2;
            let seconds = self.rng.uniform(lo, hi);
            match kind {
                "walk" => self.walk(seconds)?,
                "stand" => self.stand(seconds)?,
                _ => self.sit(seconds)?,
            }
            last = kind;
        }
    }

    /// On the way out: stop, and leave the robot standing (rise if this driver sat it).
    fn park(&mut self) {
        self.parking = true;
        if self.policy() == "sit" && self.our_sit {
            let _ = self.rise();
        }
        let _ = self.call(&proto::Call::RobotStop);
    }
}

fn foreground(robot_socket: &Path, args: EnduranceArgs) -> Result<(), Failure> {
    runlog::need_robotd(robot_socket)?;
    let out = args.out.clone().unwrap_or_else(|| UNIT.default_out());
    let run = runlog::open_run(&out, serde_json::json!({ "args": &args }))?;
    let mut detail = format!("logging to {}", out.display());
    if args.dry_run {
        detail.push_str(" (dry run)");
    }
    run.event("start", &detail);

    runlog::install_signal_handlers();

    let feed = StateFeed::spawn(robot_socket.to_owned());
    Sampler::new(run.clone(), feed.clone(), robot_socket.to_owned(), &out)
        .map_err(|e| Failure::new(exit::FAILED, format!("opening samples.csv: {e}")))?
        .spawn(Duration::from_secs_f64(args.interval.max(0.5)));

    if args.dry_run {
        run.set_activity("dry_run", "");
        wait_for_signal(&run);
        run.event("stop", "dry run ended");
        return Ok(());
    }

    let seed = args.seed.unwrap_or_else(|| {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos() as u64)
    });
    let mut driver = Driver {
        run: run.clone(),
        feed,
        socket: robot_socket.to_owned(),
        rng: Rng(seed),
        args,
        client: None,
        our_sit: false,
        parking: false,
    };
    match driver.run_forever() {
        Err(Halt::HandsOff(why)) => {
            // Keep logging, touch nothing: the rows from here to the power-off are the end of
            // the curve, which is the part this test exists for.
            run.set_activity("hands_off", &why);
            wait_for_signal(&run);
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
            run.event("stop", "robot stopped and left standing");
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[derive(Parser)]
    struct Wrap {
        #[command(flatten)]
        args: EnduranceArgs,
    }

    fn defaults() -> EnduranceArgs {
        Wrap::parse_from(["x"]).args
    }

    #[test]
    fn spans_parse_and_refuse_nonsense() {
        assert_eq!(span("20:60"), Ok((20.0, 60.0)));
        assert_eq!(span("30"), Ok((30.0, 30.0)));
        assert!(span("60:20").is_err());
        assert!(span("0:5").is_err());
        assert!(span("a:b").is_err());
    }

    #[test]
    fn goals_stay_inside_the_disc() {
        let mut rng = Rng(7);
        for _ in 0..1000 {
            let g = new_goal(&mut rng, (0.1, -0.1), (0.0, 0.0), 0.3);
            assert!(g.0.hypot(g.1) <= 0.3 + 1e-9, "{g:?}");
        }
    }

    #[test]
    fn steering_turns_on_the_spot_then_walks_at_full_stick() {
        let a = defaults();
        // Goal behind: turn on the spot, full rate, the short way round.
        assert_eq!(steer((0.0, 0.0, 0.0), (-1.0, 0.1), &a), (0.0, 1.5));
        assert_eq!(steer((0.0, 0.0, 0.0), (-1.0, -0.1), &a), (0.0, -1.5));
        // Goal ahead: full speed, small correction.
        let (vx, vyaw) = steer((0.0, 0.0, 0.0), (1.0, 0.1), &a);
        assert_eq!(vx, a.speed);
        assert!(vyaw > 0.0 && vyaw < a.vyaw);
    }

    #[test]
    fn weighted_choice_follows_the_weights() {
        let mut rng = Rng(1);
        let n = 20_000;
        let walks = (0..n)
            .filter(|_| rng.weighted(&[("walk", 3.0), ("sit", 1.0)]) == "walk")
            .count();
        let share = walks as f64 / n as f64;
        assert!((share - 0.75).abs() < 0.02, "{share}");
    }
}
