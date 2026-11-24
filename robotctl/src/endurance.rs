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

use std::fs::{File, OpenOptions};
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::{Args, Subcommand};
use duck_ipc_proto as proto;
use serde::Serialize;

use crate::{Client, Failure, exit};

/// The transient unit the background run lives in. One at a time: two drivers would fight.
const UNIT: &str = "robotctl-endurance";
/// Where runs go unless `--out` says otherwise. `/var/lib` because it survives the reboot that
/// ends every run; `/var/log` is a tmpfs on the Yocto image.
const RUNS_DIR: &str = "/var/lib/endurance";
const THERMAL_ROOT: &str = "/sys/class/thermal";
const CPUFREQ_ROOT: &str = "/sys/devices/system/cpu/cpufreq";

/// `robotd`'s deadman zeroes the twist 500 ms after the last intent; 20 Hz is well inside it.
const MOVE_PERIOD: Duration = Duration::from_millis(50);
/// A state frame older than this is no state at all.
const FRAME_MAX_AGE: Duration = Duration::from_secs(1);

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
        Some(EnduranceCommand::Status) => status(),
        Some(EnduranceCommand::Stop) => stop(),
        None if args.foreground => foreground(robot_socket, args),
        None => start(robot_socket, args),
    }
}

// ── the background unit ─────────────────────────────────────────────────────────────────────

fn need_root(what: &str) -> Result<(), Failure> {
    // Safety: geteuid cannot fail and touches no memory.
    if unsafe { libc::geteuid() } != 0 {
        return Err(Failure::new(
            exit::DENIED,
            format!("{what} needs root: sudo robotctl endurance …"),
        ));
    }
    Ok(())
}

fn unit_active() -> bool {
    std::process::Command::new("systemctl")
        .args(["is-active", "--quiet", UNIT])
        .status()
        .is_ok_and(|s| s.success())
}

fn start(robot_socket: &Path, args: EnduranceArgs) -> Result<(), Failure> {
    need_root("starting a run")?;
    if !robot_socket.exists() {
        return Err(Failure::new(
            exit::UNREACHABLE,
            format!("no {}: is robotd running?", robot_socket.display()),
        ));
    }
    if unit_active() {
        return Err(Failure::new(
            exit::BUSY,
            format!(
                "a run is already going (unit {UNIT}): `robotctl endurance status`, or \
                 `sudo robotctl endurance stop` first"
            ),
        ));
    }
    let exe = std::env::current_exe()
        .map_err(|e| Failure::new(exit::FAILED, format!("cannot find this binary: {e}")))?;

    // This invocation's own arguments, plus the two that make the unit's copy the worker: the
    // run directory fixed now, so it can be printed, and `--foreground`.
    let out = args.out.clone().unwrap_or_else(default_out);
    let mut argv: Vec<std::ffi::OsString> = std::env::args_os().skip(1).collect();
    if args.out.is_none() {
        argv.push("--out".into());
        argv.push(out.clone().into());
    }
    argv.push("--foreground".into());

    let status = std::process::Command::new("systemd-run")
        .arg(format!("--unit={UNIT}"))
        .arg("--collect")
        .arg("--description=robotctl endurance: battery and heat test")
        .arg("--quiet")
        .arg(&exe)
        .args(&argv)
        .status()
        .map_err(|e| Failure::new(exit::FAILED, format!("could not run systemd-run: {e}")))?;
    if !status.success() {
        return Err(Failure::new(
            exit::FAILED,
            format!("systemd-run failed ({status})"),
        ));
    }
    println!("started in the background as {UNIT}; safe to disconnect");
    println!("  logging to   {}", out.display());
    println!("  progress     journalctl -u {UNIT} -f    (or: robotctl endurance status)");
    println!("  abort        sudo robotctl endurance stop");
    Ok(())
}

fn stop() -> Result<(), Failure> {
    need_root("stopping a run")?;
    if !unit_active() {
        println!("no run is going");
        return Ok(());
    }
    let ok = std::process::Command::new("systemctl")
        .args(["stop", UNIT])
        .status()
        .is_ok_and(|s| s.success());
    if !ok {
        return Err(Failure::new(
            exit::FAILED,
            format!("systemctl stop {UNIT} failed"),
        ));
    }
    println!("stopped; the robot is left standing");
    Ok(())
}

fn status() -> Result<(), Failure> {
    let running = unit_active();
    println!("{}", if running { "running" } else { "not running" });
    let latest = std::fs::read_dir(RUNS_DIR)
        .ok()
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.join("samples.csv").exists())
        .max_by_key(|p| {
            p.join("samples.csv")
                .metadata()
                .and_then(|m| m.modified())
                .unwrap_or(UNIX_EPOCH)
        });
    let Some(dir) = latest else {
        println!("no runs in {RUNS_DIR}");
        return Ok(());
    };
    println!("latest run   {}", dir.display());
    if let Some(row) = last_row(&dir.join("samples.csv")) {
        let get = |k: &str| {
            row.iter()
                .find(|(h, _)| h == k)
                .map_or("", |(_, v)| v.as_str())
        };
        let hours = get("t_s").parse::<f64>().unwrap_or(0.0) / 3600.0;
        println!(
            "last sample  {hours:.2} h in: battery {}% ({} V), board {} °C, throttled {}, {}",
            or_dash(get("battery_pct")),
            or_dash(get("battery_v")),
            or_dash(get("cpu_temp_c")),
            or_dash(get("throttled")),
            or_dash(get("activity")),
        );
    }
    if let Ok(text) = std::fs::read_to_string(dir.join("events.csv")) {
        println!("recent events:");
        for line in text
            .lines()
            .skip(1)
            .collect::<Vec<_>>()
            .iter()
            .rev()
            .take(5)
            .rev()
        {
            println!("  {line}");
        }
    }
    Ok(())
}

fn or_dash(s: &str) -> &str {
    if s.is_empty() { "–" } else { s }
}

fn last_row(path: &Path) -> Option<Vec<(String, String)>> {
    let text = std::fs::read_to_string(path).ok()?;
    let mut lines = text.lines();
    let header = lines.next()?;
    let last = lines.last()?;
    // Only the sample columns are read here, and none of them is ever quoted.
    Some(
        header
            .split(',')
            .map(str::to_owned)
            .zip(last.split(',').map(str::to_owned))
            .collect(),
    )
}

fn default_out() -> PathBuf {
    let host = read_text(Path::new("/proc/sys/kernel/hostname")).unwrap_or_else(|| "duck".into());
    let (y, mo, d, h, mi, s) = crate::show::civil(unix_secs() as i64);
    Path::new(RUNS_DIR).join(format!("{host}-{y:04}{mo:02}{d:02}T{h:02}{mi:02}{s:02}Z"))
}

// ── the log ─────────────────────────────────────────────────────────────────────────────────

fn unix_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// One CSV field, quoted when it has to be.
fn csv_field(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

/// A CSV appended a row at a time and `fsync`ed after each, so a power cut keeps it.
struct DurableCsv {
    file: File,
    fields: Vec<String>,
}

impl DurableCsv {
    fn open(path: &Path, fields: Vec<String>) -> std::io::Result<Self> {
        let fresh = path.metadata().map_or(true, |m| m.len() == 0);
        let mut file = OpenOptions::new().create(true).append(true).open(path)?;
        if fresh {
            let header: Vec<String> = fields.iter().map(|f| csv_field(f)).collect();
            writeln!(file, "{}", header.join(","))?;
            file.sync_all()?;
        }
        Ok(Self { file, fields })
    }

    /// Columns absent from `row` are written empty.
    fn write(&mut self, row: &[(String, String)]) {
        let line: Vec<String> = self
            .fields
            .iter()
            .map(|f| {
                row.iter()
                    .find(|(k, _)| k == f)
                    .map_or(String::new(), |(_, v)| csv_field(v))
            })
            .collect();
        // A failed write is reported on the journal and the run goes on: losing one row is
        // better than losing the rest.
        if let Err(e) =
            writeln!(self.file, "{}", line.join(",")).and_then(|()| self.file.sync_data())
        {
            eprintln!("could not write a row: {e}");
        }
    }
}

fn fsync_dir(path: &Path) {
    if let Ok(dir) = File::open(path) {
        let _ = dir.sync_all();
    }
}

/// What the driver and the sampler share.
struct Shared {
    activity: String,
    home: Option<(f64, f64)>,
    battery_pct: Option<f64>,
}

struct Run {
    t0: Instant,
    shared: Mutex<Shared>,
    events: Mutex<DurableCsv>,
}

impl Run {
    fn elapsed(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }

    fn event(&self, kind: &str, detail: &str) {
        let row = [
            ("wall".into(), format!("{:.3}", unix_secs())),
            ("t_s".into(), format!("{:.1}", self.elapsed())),
            ("kind".into(), kind.into()),
            ("detail".into(), detail.into()),
        ];
        self.events.lock().expect("events lock").write(&row);
        println!("[{:8.1}s] {kind}: {detail}", self.elapsed());
    }

    fn set_activity(&self, activity: &str, detail: &str) {
        self.shared.lock().expect("shared lock").activity = activity.to_owned();
        self.event("activity", format!("{activity} {detail}").trim());
    }
}

// ── the state stream ────────────────────────────────────────────────────────────────────────

#[derive(Default)]
struct FeedInner {
    latest: Option<(Instant, proto::RobotState)>,
    current_sum: f64,
    current_n: u64,
}

/// The latest `robot.state` frame, plus motor current averaged since the last sample.
#[derive(Clone)]
struct StateFeed(Arc<Mutex<FeedInner>>);

impl StateFeed {
    fn spawn(socket: PathBuf) -> Self {
        let feed = Self(Arc::default());
        let inner = feed.0.clone();
        std::thread::spawn(move || {
            loop {
                // Reconnects forever: a robotd restart mid-run costs seconds, not the run.
                let _ = Self::stream(&socket, &inner);
                std::thread::sleep(Duration::from_secs(2));
            }
        });
        feed
    }

    fn stream(socket: &Path, inner: &Mutex<FeedInner>) -> Result<(), Failure> {
        let mut client = Client::connect_to("robotd", socket)?;
        let _ = client.writer.set_read_timeout(Some(Duration::from_secs(5)));
        client.send(&proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(20) }),
        ))?;
        let mut line = String::new();
        loop {
            line.clear();
            match client.reader.read_line(&mut line) {
                Ok(0) | Err(_) => return Ok(()),
                Ok(_) => {}
            }
            let Some(state) = serde_json::from_str::<proto::Request>(line.trim())
                .ok()
                .and_then(|r| r.as_state())
            else {
                continue;
            };
            let mut inner = inner.lock().expect("feed lock");
            if !state.currents_ma.is_empty() {
                inner.current_sum += state.currents_ma.iter().map(|c| c.abs()).sum::<f64>();
                inner.current_n += 1;
            }
            inner.latest = Some((Instant::now(), state));
        }
    }

    fn frame(&self) -> Option<proto::RobotState> {
        let inner = self.0.lock().expect("feed lock");
        let (at, state) = inner.latest.as_ref()?;
        (at.elapsed() <= FRAME_MAX_AGE).then(|| state.clone())
    }

    fn take_mean_current_ma(&self) -> Option<f64> {
        let mut inner = self.0.lock().expect("feed lock");
        let mean = (inner.current_n > 0).then(|| inner.current_sum / inner.current_n as f64);
        inner.current_sum = 0.0;
        inner.current_n = 0;
        mean
    }
}

// ── what the board says about itself, read directly ─────────────────────────────────────────
//
// `robot.health` carries the hottest zone and the worst throttle already. These are read here as
// well because they keep working when robotd does not, and per-zone is what a heat test wants.

fn read_text(path: &Path) -> Option<String> {
    std::fs::read_to_string(path)
        .ok()
        .map(|s| s.trim().to_owned())
}

fn read_int(path: &Path) -> Option<i64> {
    read_text(path)?.parse().ok()
}

/// `policy10` after `policy2`: sysfs numbers its entries, and the sort should agree.
fn by_index(name: &str) -> i64 {
    let digits: String = name.chars().filter(char::is_ascii_digit).collect();
    digits.parse().unwrap_or(-1)
}

fn entries(root: &str, prefix: &str) -> Vec<(String, PathBuf)> {
    let mut found: Vec<(String, PathBuf)> = std::fs::read_dir(root)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            name.starts_with(prefix).then(|| (name, e.path()))
        })
        .collect();
    found.sort_by_key(|(name, _)| by_index(name));
    found
}

fn sysfs_kind(dir: &Path, fallback: &str) -> String {
    read_text(&dir.join("type"))
        .unwrap_or_else(|| fallback.to_owned())
        .replace('-', "_")
}

struct Sampler {
    run: Arc<Run>,
    feed: StateFeed,
    socket: PathBuf,
    /// `(column, temp file)`. The zone index is in the name because types repeat.
    zones: Vec<(String, PathBuf)>,
    /// `(column, cur_state file)`.
    cooling: Vec<(String, PathBuf)>,
    policies: Vec<(String, PathBuf)>,
    csv: DurableCsv,
    robotd_up: bool,
}

impl Sampler {
    fn new(run: Arc<Run>, feed: StateFeed, socket: PathBuf, out: &Path) -> std::io::Result<Self> {
        let zones: Vec<_> = entries(THERMAL_ROOT, "thermal_zone")
            .into_iter()
            .map(|(name, dir)| {
                let zone = name.trim_start_matches("thermal_");
                (
                    format!("temp_{zone}_{}_c", sysfs_kind(&dir, "zone")),
                    dir.join("temp"),
                )
            })
            .collect();
        let cooling: Vec<_> = entries(THERMAL_ROOT, "cooling_device")
            .into_iter()
            .map(|(name, dir)| {
                let kind = sysfs_kind(&dir, &name);
                (format!("{name}_{kind}"), dir.join("cur_state"))
            })
            .collect();
        let policies = entries(CPUFREQ_ROOT, "policy");

        let mut fields: Vec<String> = [
            "wall",
            "t_s",
            "uptime_s",
            "robotd",
            "battery_v",
            "battery_pct",
            "cpu_temp_c",
        ]
        .map(String::from)
        .to_vec();
        fields.extend(zones.iter().map(|(c, _)| c.clone()));
        fields.extend(
            [
                "throttled",
                "throttle_level",
                "throttle_max_level",
                "cpu_khz_ceiling",
                "cpu_khz_max",
            ]
            .map(String::from),
        );
        fields.extend(policies.iter().map(|(n, _)| format!("{n}_cur_khz")));
        fields.extend(policies.iter().map(|(n, _)| format!("{n}_scaling_max_khz")));
        fields.extend(cooling.iter().map(|(c, _)| c.clone()));
        fields.extend(
            [
                "load1",
                "motor_max_c",
                "motor_mean_c",
                "motor_hottest",
                "motor_current_ma",
                "loop_hz",
                "activity",
                "policy",
                "fallen",
                "picked_up",
                "odom_x",
                "odom_y",
                "odom_yaw",
                "odom_dist",
                "healthy",
                "reason",
            ]
            .map(String::from),
        );
        let csv = DurableCsv::open(&out.join("samples.csv"), fields)?;
        Ok(Self {
            run,
            feed,
            socket,
            zones,
            cooling,
            policies,
            csv,
            robotd_up: true,
        })
    }

    fn spawn(mut self, interval: Duration) {
        std::thread::spawn(move || {
            let mut next = Instant::now();
            loop {
                self.sample();
                next += interval;
                std::thread::sleep(next.saturating_duration_since(Instant::now()));
            }
        });
    }

    fn health(&self) -> Result<proto::HealthResult, String> {
        let mut client = Client::connect_to("robotd", &self.socket).map_err(|f| brief(&f))?;
        let _ = client.writer.set_read_timeout(Some(Duration::from_secs(3)));
        let response = client
            .call(&proto::Call::RobotHealth)
            .map_err(|f| brief(&f))?;
        if let Some(error) = response.error {
            return Err(error.message);
        }
        serde_json::from_value(response.result.unwrap_or_default()).map_err(|e| e.to_string())
    }

    fn sample(&mut self) {
        let mut row: Vec<(String, String)> = Vec::new();
        let mut put = |k: &str, v: String| row.push((k.to_owned(), v));
        put("wall", format!("{:.3}", unix_secs()));
        put("t_s", format!("{:.1}", self.run.elapsed()));
        let first = |p: &str| {
            read_text(Path::new(p))
                .and_then(|s| s.split_whitespace().next().map(str::to_owned))
                .unwrap_or_default()
        };
        put("uptime_s", first("/proc/uptime"));
        put("load1", first("/proc/loadavg"));

        for (column, path) in &self.zones {
            let value = read_int(path)
                .filter(|m| *m != 0)
                .map_or(String::new(), |m| format!("{:.1}", m as f64 / 1000.0));
            put(column, value);
        }
        let opt = |v: Option<i64>| v.map_or(String::new(), |v| v.to_string());
        for (name, dir) in &self.policies {
            put(
                &format!("{name}_cur_khz"),
                opt(read_int(&dir.join("scaling_cur_freq"))),
            );
            put(
                &format!("{name}_scaling_max_khz"),
                opt(read_int(&dir.join("scaling_max_freq"))),
            );
        }
        for (column, path) in &self.cooling {
            put(column, opt(read_int(path)));
        }

        let health = match self.health() {
            Ok(h) => {
                put("robotd", "up".into());
                if !self.robotd_up {
                    self.run.event("health_available", "robotd answers again");
                }
                self.robotd_up = true;
                Some(h)
            }
            Err(e) => {
                put("robotd", "down".into());
                // Once per outage: the last minute before a power-off is all outage.
                if self.robotd_up {
                    self.run.event("health_unavailable", &e);
                }
                self.robotd_up = false;
                None
            }
        };
        if let Some(h) = &health {
            if let Some(b) = &h.battery {
                put("battery_v", format!("{:.3}", b.volts));
                put("battery_pct", format!("{:.1}", b.percent));
                self.run.shared.lock().expect("shared lock").battery_pct = Some(b.percent);
            }
            if let Some(c) = h.cpu_temp_c {
                put("cpu_temp_c", format!("{c:.1}"));
            }
            if let Some(t) = &h.cpu_throttle {
                put("throttle_level", t.level.to_string());
                put("throttle_max_level", t.max_level.to_string());
                put("cpu_khz_ceiling", t.khz.to_string());
                put("cpu_khz_max", t.max_khz.to_string());
                put("throttled", u8::from(t.throttled()).to_string());
            }
            if let Some(m) = &h.motors {
                put("motor_max_c", m.max_c.to_string());
                put("motor_mean_c", format!("{:.1}", m.mean_c));
                put("motor_hottest", m.hottest.clone());
            }
            if let Some(hz) = h.control_loop.as_ref().and_then(|l| l.achieved_hz) {
                put("loop_hz", format!("{hz:.1}"));
            }
            put("healthy", u8::from(h.healthy).to_string());
            put("reason", h.reason.clone().unwrap_or_default());
        }

        if let Some(ma) = self.feed.take_mean_current_ma() {
            put("motor_current_ma", format!("{ma:.0}"));
        }
        let (activity, home) = {
            let shared = self.run.shared.lock().expect("shared lock");
            (shared.activity.clone(), shared.home)
        };
        put("activity", activity.clone());
        if let Some(frame) = self.feed.frame() {
            put("policy", frame.policy.clone());
            put("fallen", u8::from(frame.safety.fallen).to_string());
            put("picked_up", u8::from(frame.safety.picked_up).to_string());
            let [x, y, _] = frame.odom.position;
            put("odom_x", format!("{x:.3}"));
            put("odom_y", format!("{y:.3}"));
            put("odom_yaw", format!("{:.3}", frame.odom.yaw));
            if let Some((hx, hy)) = home {
                put("odom_dist", format!("{:.3}", (x - hx).hypot(y - hy)));
            }
        }

        let get = |k: &str| {
            row.iter()
                .find(|(c, _)| c == k)
                .map_or("?", |(_, v)| v.as_str())
                .to_owned()
        };
        println!(
            "[{:8.1}s] batt {}% ({} V)  cpu {} °C  throttled={}  {activity}",
            self.run.elapsed(),
            get("battery_pct"),
            get("battery_v"),
            get("cpu_temp_c"),
            get("throttled"),
        );
        self.csv.write(&row);
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

fn wrap(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

/// Forward speed and turn rate toward `goal` from `(x, y, yaw)`: on the spot when the goal is
/// well off the nose, otherwise forward with a saturating heading correction.
fn steer(pose: (f64, f64, f64), goal: (f64, f64), args: &EnduranceArgs) -> (f64, f64) {
    let (x, y, yaw) = pose;
    let err = wrap((goal.1 - y).atan2(goal.0 - x) - yaw);
    if err.abs() > args.turn_threshold {
        (0.0, args.vyaw.copysign(err))
    } else {
        (args.speed, (2.5 * err).clamp(-args.vyaw, args.vyaw))
    }
}

/// A point inside the goal disc, far enough from here that getting there is a walk.
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

/// Why the driver stopped driving.
enum Halt {
    /// SIGTERM or SIGINT: stop the robot, leave it standing.
    Stop,
    /// The battery is nearly empty, or robotd started its own shutdown. A `sit_toggle` or an
    /// enable sent into the middle of that is a client fighting the daemon, so let go and log.
    HandsOff(String),
    Fatal(String),
}

static SIGNAL: AtomicI32 = AtomicI32::new(0);

extern "C" fn on_signal(signum: libc::c_int) {
    SIGNAL.store(signum, Ordering::Relaxed);
}

fn signalled() -> bool {
    SIGNAL.load(Ordering::Relaxed) != 0
}

fn signal_name() -> &'static str {
    match SIGNAL.load(Ordering::Relaxed) {
        libc::SIGTERM => "SIGTERM",
        libc::SIGINT => "SIGINT",
        _ => "signal",
    }
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

/// A failure's first line. `Client`'s messages carry a remedy on the lines after it, which is
/// right for a person at a terminal and noise in an events log.
fn brief(failure: &Failure) -> String {
    failure
        .message
        .lines()
        .next()
        .unwrap_or_default()
        .to_owned()
}

fn json<T: Serialize>(value: &T) -> String {
    serde_json::to_string(value).unwrap_or_default()
}

fn foreground(robot_socket: &Path, args: EnduranceArgs) -> Result<(), Failure> {
    if !robot_socket.exists() {
        return Err(Failure::new(
            exit::UNREACHABLE,
            format!("no {}: is robotd running?", robot_socket.display()),
        ));
    }
    let io = |what: &str, e: std::io::Error| Failure::new(exit::FAILED, format!("{what}: {e}"));
    let out = args.out.clone().unwrap_or_else(default_out);
    std::fs::create_dir_all(&out).map_err(|e| io(&format!("creating {}", out.display()), e))?;
    if let Some(parent) = out.parent() {
        fsync_dir(parent);
    }

    // One segment per start: a run restarted into the same directory says so.
    let meta_path = out.join("meta.json");
    let mut meta: serde_json::Value = std::fs::read_to_string(&meta_path)
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or_else(|| serde_json::json!({ "segments": [] }));
    let now = unix_secs();
    let (y, mo, d, h, mi, s) = crate::show::civil(now as i64);
    let segment = serde_json::json!({
        "started_wall": now,
        "started_utc": format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}Z"),
        "boot_id": read_text(Path::new("/proc/sys/kernel/random/boot_id")),
        "host": read_text(Path::new("/proc/sys/kernel/hostname")),
        "robotctl": env!("CARGO_PKG_VERSION"),
        "args": args,
    });
    if let Some(segments) = meta["segments"].as_array_mut() {
        segments.push(segment);
    }
    std::fs::write(
        &meta_path,
        serde_json::to_string_pretty(&meta).unwrap_or_default(),
    )
    .map_err(|e| io("writing meta.json", e))?;
    fsync_dir(&out);

    let events = DurableCsv::open(
        &out.join("events.csv"),
        ["wall", "t_s", "kind", "detail"].map(String::from).to_vec(),
    )
    .map_err(|e| io("opening events.csv", e))?;
    let run = Arc::new(Run {
        t0: Instant::now(),
        shared: Mutex::new(Shared {
            activity: "starting".into(),
            home: None,
            battery_pct: None,
        }),
        events: Mutex::new(events),
    });
    let mut detail = format!("logging to {}", out.display());
    if args.dry_run {
        detail.push_str(" (dry run)");
    }
    run.event("start", &detail);

    // Safety: the handler only stores to an atomic, which is async-signal-safe.
    unsafe {
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
    }

    let feed = StateFeed::spawn(robot_socket.to_owned());
    Sampler::new(run.clone(), feed.clone(), robot_socket.to_owned(), &out)
        .map_err(|e| io("opening samples.csv", e))?
        .spawn(Duration::from_secs_f64(args.interval.max(0.5)));

    let wait_for_signal = || {
        while !signalled() {
            std::thread::sleep(Duration::from_millis(200));
        }
        run.event("signal", signal_name());
    };

    if args.dry_run {
        run.set_activity("dry_run", "");
        wait_for_signal();
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
            wait_for_signal();
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
    fn csv_quotes_only_what_needs_it() {
        assert_eq!(csv_field("walk 12s"), "walk 12s");
        assert_eq!(
            csv_field(r#"{"accepted":true,"x":1}"#),
            r#""{""accepted"":true,""x"":1}""#
        );
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

    #[test]
    fn sysfs_entries_sort_numerically() {
        let mut names = vec!["policy10", "policy2", "policy0"];
        names.sort_by_key(|n| by_index(n));
        assert_eq!(names, ["policy0", "policy2", "policy10"]);
    }
}
