//! What a long unattended test on the robot needs whatever it does: a background unit that
//! outlives the ssh session, a run directory whose every row is on disk as it is written, the
//! latest `robot.state`, and a sampler logging the battery, the board's heat and the motors.
//!
//! `robotctl endurance` and `robotctl stress` are two behaviours on top of this. Each keeps its
//! own driver; nothing here moves the robot.

use std::fs::{File, OpenOptions};
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use duck_ipc_proto as proto;
use serde::Serialize;

use crate::{Client, Failure, exit};

const THERMAL_ROOT: &str = "/sys/class/thermal";
const CPUFREQ_ROOT: &str = "/sys/devices/system/cpu/cpufreq";

/// A state frame older than this is no state at all.
pub(crate) const FRAME_MAX_AGE: Duration = Duration::from_secs(1);

// ── the background unit ─────────────────────────────────────────────────────────────────────

/// One kind of run: its transient unit, where its runs go, and how it is named to a person.
pub(crate) struct Unit {
    /// The transient unit the background run lives in. One at a time: two drivers would fight.
    pub(crate) name: &'static str,
    /// Where runs go unless `--out` says otherwise. It has to survive the reboot that ends every
    /// run, and `/var/log` is a tmpfs on the Yocto image.
    pub(crate) runs_dir: &'static str,
    /// The `robotctl` subcommand, for the hints printed to a person.
    pub(crate) command: &'static str,
    /// The unit's description in `systemctl status`.
    pub(crate) description: &'static str,
    /// What a stop leaves the robot in, for the person who asked for it.
    pub(crate) stopped: &'static str,
}

impl Unit {
    pub(crate) fn need_root(&self, what: &str) -> Result<(), Failure> {
        // Safety: geteuid cannot fail and touches no memory.
        if unsafe { libc::geteuid() } != 0 {
            return Err(Failure::new(
                exit::DENIED,
                format!("{what} needs root: sudo robotctl {} …", self.command),
            ));
        }
        Ok(())
    }

    pub(crate) fn active(&self) -> bool {
        std::process::Command::new("systemctl")
            .args(["is-active", "--quiet", self.name])
            .status()
            .is_ok_and(|s| s.success())
    }

    /// Re-run this invocation as the background unit, with `--out` fixed now (so it can be
    /// printed) and `--foreground`, which makes the unit's copy the worker.
    pub(crate) fn start(&self, robot_socket: &Path, out: Option<&Path>) -> Result<(), Failure> {
        self.start_with(robot_socket, out, std::env::args_os().skip(1).collect())
    }

    /// [`Self::start`] with the invocation given rather than read from this process — for a
    /// caller whose own command line is not the run's (`robotctl stress boot`).
    pub(crate) fn start_with(
        &self,
        robot_socket: &Path,
        out: Option<&Path>,
        mut argv: Vec<std::ffi::OsString>,
    ) -> Result<(), Failure> {
        let (name, command) = (self.name, self.command);
        self.need_root("starting a run")?;
        need_robotd(robot_socket)?;
        if self.active() {
            return Err(Failure::new(
                exit::BUSY,
                format!(
                    "a run is already going (unit {name}): `robotctl {command} status`, or \
                     `sudo robotctl {command} stop` first"
                ),
            ));
        }
        let exe = std::env::current_exe()
            .map_err(|e| Failure::new(exit::FAILED, format!("cannot find this binary: {e}")))?;

        let dir = out.map_or_else(|| self.default_out(), Path::to_path_buf);
        if out.is_none() {
            argv.push("--out".into());
            argv.push(dir.clone().into());
        }
        argv.push("--foreground".into());

        let status = std::process::Command::new("systemd-run")
            .arg(format!("--unit={name}"))
            .arg("--collect")
            .arg(format!("--description={}", self.description))
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
        println!("started in the background as {name}; safe to disconnect");
        println!("  logging to   {}", dir.display());
        println!("  progress     journalctl -u {name} -f    (or: robotctl {command} status)");
        println!("  abort        sudo robotctl {command} stop");
        Ok(())
    }

    pub(crate) fn stop(&self) -> Result<(), Failure> {
        self.need_root("stopping a run")?;
        if !self.active() {
            println!("no run is going");
            return Ok(());
        }
        let ok = std::process::Command::new("systemctl")
            .args(["stop", self.name])
            .status()
            .is_ok_and(|s| s.success());
        if !ok {
            return Err(Failure::new(
                exit::FAILED,
                format!("systemctl stop {} failed", self.name),
            ));
        }
        println!("stopped; {}", self.stopped);
        Ok(())
    }

    pub(crate) fn status(&self) -> Result<(), Failure> {
        let running = self.active();
        println!("{}", if running { "running" } else { "not running" });
        let latest = std::fs::read_dir(self.runs_dir)
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
            println!("no runs in {}", self.runs_dir);
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
            if !get("lap").is_empty() {
                println!("lap          {} at {}", get("lap"), or_dash(get("step")));
            }
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

    pub(crate) fn default_out(&self) -> PathBuf {
        let host =
            read_text(Path::new("/proc/sys/kernel/hostname")).unwrap_or_else(|| "duck".into());
        let (y, mo, d, h, mi, s) = crate::show::civil(unix_secs() as i64);
        Path::new(self.runs_dir).join(format!("{host}-{y:04}{mo:02}{d:02}T{h:02}{mi:02}{s:02}Z"))
    }
}

pub(crate) fn need_robotd(robot_socket: &Path) -> Result<(), Failure> {
    if robot_socket.exists() {
        return Ok(());
    }
    Err(Failure::new(
        exit::UNREACHABLE,
        format!("no {}: is robotd running?", robot_socket.display()),
    ))
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

// ── the log ─────────────────────────────────────────────────────────────────────────────────

pub(crate) fn unix_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// One CSV field, quoted when it has to be.
pub(crate) fn csv_field(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

/// A CSV appended a row at a time and `fsync`ed after each, so a power cut keeps it.
pub(crate) struct DurableCsv {
    file: File,
    fields: Vec<String>,
}

impl DurableCsv {
    /// Appending to a file that has a header keeps that header's columns, so a run resumed by a
    /// build that logs more (or fewer) columns stays one aligned table: what the old header has
    /// no column for is dropped.
    pub(crate) fn open(path: &Path, fields: Vec<String>) -> std::io::Result<Self> {
        // The first line alone: damage further down must not read as a file with no header, and
        // get a second one written into the middle of the table.
        let mut existing = String::new();
        if let Ok(file) = File::open(path) {
            let _ = std::io::BufReader::new(file).read_line(&mut existing);
        }
        let existing = Some(existing.trim_end().to_owned()).filter(|h| !h.is_empty());
        let mut file = OpenOptions::new().create(true).append(true).open(path)?;
        // Column names are never quoted, so a plain split reads the header back.
        let fields = match existing {
            Some(header) => header.split(',').map(str::to_owned).collect(),
            None => {
                let header: Vec<String> = fields.iter().map(|f| csv_field(f)).collect();
                writeln!(file, "{}", header.join(","))?;
                file.sync_all()?;
                fields
            }
        };
        Ok(Self { file, fields })
    }

    /// Columns absent from `row` are written empty.
    pub(crate) fn write(&mut self, row: &[(String, String)]) {
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
        let row = format!("{}\n", line.join(","));
        if let Err(e) = self
            .file
            .write_all(row.as_bytes())
            .and_then(|()| self.file.sync_data())
        {
            eprintln!("could not write a row: {e}");
        }
    }
}

pub(crate) fn fsync_dir(path: &Path) {
    if let Ok(dir) = File::open(path) {
        let _ = dir.sync_all();
    }
}

/// Replace `path` in one step: a power cut leaves the old file or the new one, never an empty
/// one. Written in place, `meta.json` was left at 0 bytes on real runs.
pub(crate) fn write_atomic(path: &Path, contents: &[u8]) -> std::io::Result<()> {
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    let mut file = File::create(&tmp)?;
    file.write_all(contents)?;
    file.sync_all()?;
    drop(file);
    std::fs::rename(&tmp, path)?;
    if let Some(parent) = path.parent() {
        fsync_dir(parent);
    }
    Ok(())
}

/// Cut a CSV back to its last whole row, returning how many bytes went. A power cut in the middle
/// of a write leaves half a row with no newline, and the next start would append to it, gluing
/// two rows into one that fits neither.
pub(crate) fn repair_tail(path: &Path) -> std::io::Result<u64> {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(0),
        Err(e) => return Err(e),
    };
    let keep = bytes
        .iter()
        .rposition(|b| *b == b'\n')
        .map_or(0, |newline| newline + 1);
    let cut = bytes.len() - keep;
    if cut > 0 {
        let file = OpenOptions::new().write(true).open(path)?;
        file.set_len(keep as u64)?;
        file.sync_all()?;
    }
    Ok(cut as u64)
}

// ── a clock worth naming a run by ───────────────────────────────────────────────────────────

/// How often the wait for the clock says it is still waiting.
const CLOCK_REPORT: Duration = Duration::from_secs(30);

/// `timedatectl`'s answer, as `timedatectl show -p NTPSynchronized --value` prints it.
fn says_synchronized(output: &str) -> bool {
    output.trim() == "yes"
}

fn clock_synchronized() -> bool {
    std::process::Command::new("timedatectl")
        .args(["show", "-p", "NTPSynchronized", "--value"])
        .output()
        .is_ok_and(|o| o.status.success() && says_synchronized(&String::from_utf8_lossy(&o.stdout)))
}

/// Hold until a time server has set the clock, however long that takes.
///
/// The board has no battery-backed clock: it boots on the last time it saved, and stays there for
/// as long as it has no network. Runs started like that were named after the day before, with
/// every wall time wrong by an amount nothing recorded. Nothing is created until this returns.
pub(crate) fn wait_for_clock() {
    let t0 = Instant::now();
    let mut next_report = Duration::ZERO;
    while !clock_synchronized() {
        if t0.elapsed() >= next_report {
            println!(
                "waiting for the network time before starting ({} s so far): no run until the \
                 clock is right, so check the wifi",
                t0.elapsed().as_secs()
            );
            next_report += CLOCK_REPORT;
        }
        std::thread::sleep(Duration::from_secs(2));
    }
}

// ── what a run ran on ───────────────────────────────────────────────────────────────────────

/// Where the release's policies live: what `robotd` logs loading.
const POLICIES: &str = "/opt/robot/policies/current";

/// `rauc.slot=` on the kernel command line.
fn boot_slot(cmdline: &str) -> Option<&str> {
    cmdline
        .split_whitespace()
        .find_map(|arg| arg.strip_prefix("rauc.slot="))
}

/// `VERSION=` in os-release, unquoted.
fn os_version(os_release: &str) -> Option<&str> {
    os_release
        .lines()
        .find_map(|line| line.strip_prefix("VERSION="))
        .map(|v| v.trim_matches('"'))
}

/// Each policy's SHA-256, by file name, in name order.
fn policy_hashes(dir: &Path) -> serde_json::Map<String, serde_json::Value> {
    use sha2::{Digest, Sha256};
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|e| e == "onnx"))
        .collect();
    paths.sort();
    paths
        .iter()
        .filter_map(|path| {
            let digest = Sha256::digest(std::fs::read(path).ok()?);
            let hex: String = digest.iter().map(|b| format!("{b:02x}")).collect();
            Some((path.file_name()?.to_string_lossy().into_owned(), hex.into()))
        })
        .collect()
}

/// Exactly what this segment ran on. Every build of a release says `0.18.4`, which could not tell
/// two different stress tests apart; the revision and the release can.
fn provenance() -> serde_json::Value {
    let build = proto::build_info!();
    let cmdline = read_text(Path::new("/proc/cmdline")).unwrap_or_default();
    let os_release = read_text(Path::new("/etc/os-release")).unwrap_or_default();
    serde_json::json!({
        "revision": build.revision,
        "built_at": build.built_at,
        "release": std::env::current_exe()
            .ok()
            .and_then(|exe| proto::release_from_path(&exe.to_string_lossy()))
            .map(|version| version.to_string()),
        "slot": boot_slot(&cmdline),
        "os": os_version(&os_release),
        "policies": policy_hashes(Path::new(POLICIES)),
    })
}

// ── the journal of the boots before this one ────────────────────────────────────────────────

/// The units whose logs explain a run: what happened to the robot, why the run did what it did,
/// and whether the boot itself was well.
const JOURNAL_UNITS: &[&str] = &[
    "robotd",
    "NetworkManager",
    "systemd-timesyncd",
    "microduck-slot-health",
    "rauc-mark-good",
];

/// The boot ids in `journalctl --list-boots`, whatever else its columns say.
fn listed_boots(list: &str) -> Vec<String> {
    list.lines()
        .filter_map(|line| line.split_whitespace().nth(1))
        .filter(|id| id.len() == 32 && id.chars().all(|c| c.is_ascii_hexdigit()))
        .map(str::to_owned)
        .collect()
}

/// Which run directory under `parent` each boot wrote to, by journal-style boot id. A boot with
/// two runs answers the later one.
fn runs_by_boot(parent: &Path) -> std::collections::HashMap<String, PathBuf> {
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(parent)
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .collect();
    dirs.sort();
    let mut runs = std::collections::HashMap::new();
    for dir in dirs {
        let Some(meta) = std::fs::read_to_string(dir.join("meta.json"))
            .ok()
            .and_then(|text| serde_json::from_str::<serde_json::Value>(&text).ok())
        else {
            continue;
        };
        for segment in meta["segments"].as_array().into_iter().flatten() {
            if let Some(id) = segment["boot_id"].as_str() {
                runs.insert(id.trim().replace('-', ""), dir.clone());
            }
        }
    }
    runs
}

fn save_journal(boot: &str, units: &[&str], to: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(to.parent().unwrap_or(Path::new(".")))?;
    let mut tmp = to.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    let file = File::create(&tmp)?;
    let mut journal = std::process::Command::new("journalctl");
    journal.args(["-b", boot, "--no-pager", "-o", "short-iso"]);
    for unit in units {
        journal.args(["-u", unit]);
    }
    // To the file, not through memory: a boot's log can be tens of megabytes on a 1 GB board.
    let status = journal.stdout(file.try_clone()?).status()?;
    if !status.success() {
        return Err(std::io::Error::other(format!("journalctl: {status}")));
    }
    file.sync_all()?;
    std::fs::rename(&tmp, to)?;
    if let Some(parent) = to.parent() {
        fsync_dir(parent);
    }
    Ok(())
}

/// Keep the log of every earlier boot that has a run under `out`'s parent, in that run's
/// `journal/<boot id>.txt`, in the background. Those logs are how a run gets explained, and the
/// journal rotates them away within days. A boot's log is only whole once the boot is over, so
/// each is taken from the boot after; one that cannot be read costs an event, never the run.
pub(crate) fn save_earlier_journals(run: Arc<Run>, out: &Path, unit: &'static str) {
    let Some(parent) = out.parent().map(Path::to_path_buf) else {
        return;
    };
    let current = read_text(Path::new("/proc/sys/kernel/random/boot_id"))
        .unwrap_or_default()
        .replace('-', "");
    std::thread::spawn(move || {
        let Ok(list) = std::process::Command::new("journalctl")
            .args(["--list-boots", "--no-pager"])
            .output()
        else {
            return run.event("journal_failed", "could not run journalctl");
        };
        let runs = runs_by_boot(&parent);
        let mut units = JOURNAL_UNITS.to_vec();
        units.push(unit);
        for boot in listed_boots(&String::from_utf8_lossy(&list.stdout)) {
            let Some(dir) = runs.get(&boot).filter(|_| boot != current) else {
                continue;
            };
            let to = dir.join("journal").join(format!("{boot}.txt"));
            if to.exists() {
                continue;
            }
            match save_journal(&boot, &units, &to) {
                Ok(()) => run.event("journal_saved", &to.display().to_string()),
                Err(e) => run.event("journal_failed", &format!("{boot}: {e}")),
            }
        }
    });
}

/// What the driver and the sampler share.
#[derive(Default)]
pub(crate) struct Shared {
    pub(crate) activity: String,
    pub(crate) home: Option<(f64, f64)>,
    pub(crate) battery_pct: Option<f64>,
    /// The lap a scripted run is on, and the step within it. Empty columns for a run that has
    /// no laps.
    pub(crate) lap: Option<u32>,
    pub(crate) step: String,
}

pub(crate) struct Run {
    pub(crate) t0: Instant,
    pub(crate) shared: Mutex<Shared>,
    pub(crate) events: Mutex<DurableCsv>,
}

impl Run {
    pub(crate) fn elapsed(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }

    pub(crate) fn event(&self, kind: &str, detail: &str) {
        let row = [
            ("wall".into(), format!("{:.3}", unix_secs())),
            ("t_s".into(), format!("{:.1}", self.elapsed())),
            ("kind".into(), kind.into()),
            ("detail".into(), detail.into()),
        ];
        self.events.lock().expect("events lock").write(&row);
        println!("[{:8.1}s] {kind}: {detail}", self.elapsed());
    }

    pub(crate) fn set_activity(&self, activity: &str, detail: &str) {
        self.shared.lock().expect("shared lock").activity = activity.to_owned();
        self.event("activity", format!("{activity} {detail}").trim());
    }
}

// ── the state stream ────────────────────────────────────────────────────────────────────────

#[derive(Default)]
struct FeedInner {
    latest: Option<(Instant, proto::RobotState)>,
    current: CurrentWindow,
}

/// Motor current since the last sample: per joint and summed, the mean and the peak.
#[derive(Default, Clone)]
pub(crate) struct CurrentWindow {
    pub(crate) n: u64,
    /// Indexed as [`proto::JOINT_NAMES`].
    pub(crate) sum_ma: [f64; proto::JOINT_NAMES.len()],
    pub(crate) peak_ma: [f64; proto::JOINT_NAMES.len()],
    pub(crate) total_sum_ma: f64,
    pub(crate) total_peak_ma: f64,
}

impl CurrentWindow {
    fn add(&mut self, currents_ma: &[f64]) {
        let mut total = 0.0;
        for (i, c) in currents_ma
            .iter()
            .take(proto::JOINT_NAMES.len())
            .enumerate()
        {
            let c = c.abs();
            self.sum_ma[i] += c;
            self.peak_ma[i] = self.peak_ma[i].max(c);
            total += c;
        }
        self.total_sum_ma += total;
        self.total_peak_ma = self.total_peak_ma.max(total);
        self.n += 1;
    }
}

/// The latest `robot.state` frame, plus motor current since the last sample.
#[derive(Clone)]
pub(crate) struct StateFeed(Arc<Mutex<FeedInner>>);

impl StateFeed {
    pub(crate) fn spawn(socket: PathBuf) -> Self {
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
        // Every tick, not decimated: a current peak is a few ticks long, and a 20 Hz stream
        // misses most of them.
        client.send(&proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: None }),
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
                inner.current.add(&state.currents_ma);
            }
            inner.latest = Some((Instant::now(), state));
        }
    }

    pub(crate) fn frame(&self) -> Option<proto::RobotState> {
        let inner = self.0.lock().expect("feed lock");
        let (at, state) = inner.latest.as_ref()?;
        (at.elapsed() <= FRAME_MAX_AGE).then(|| state.clone())
    }

    /// `None` when no frame since the last call carried current.
    fn take_current(&self) -> Option<CurrentWindow> {
        let mut inner = self.0.lock().expect("feed lock");
        let window = std::mem::take(&mut inner.current);
        (window.n > 0).then_some(window)
    }
}

// ── what the board says about itself, read directly ─────────────────────────────────────────
//
// `robot.health` carries the hottest zone and the worst throttle already. These are read here as
// well because they keep working when robotd does not, and per-zone is what a heat test wants.

pub(crate) fn read_text(path: &Path) -> Option<String> {
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

pub(crate) struct Sampler {
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
    pub(crate) fn new(
        run: Arc<Run>,
        feed: StateFeed,
        socket: PathBuf,
        out: &Path,
    ) -> std::io::Result<Self> {
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
                "motor_current_peak_ma",
            ]
            .map(String::from),
        );
        for joint in proto::JOINT_NAMES {
            fields.push(format!("{joint}_current_ma"));
            fields.push(format!("{joint}_current_peak_ma"));
        }
        fields.extend(
            [
                "loop_hz",
                "activity",
                "lap",
                "step",
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

    pub(crate) fn spawn(mut self, interval: Duration) {
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

        if let Some(w) = self.feed.take_current() {
            let n = w.n as f64;
            put("motor_current_ma", format!("{:.0}", w.total_sum_ma / n));
            put("motor_current_peak_ma", format!("{:.0}", w.total_peak_ma));
            for (i, joint) in proto::JOINT_NAMES.iter().enumerate() {
                put(
                    &format!("{joint}_current_ma"),
                    format!("{:.0}", w.sum_ma[i] / n),
                );
                put(
                    &format!("{joint}_current_peak_ma"),
                    format!("{:.0}", w.peak_ma[i]),
                );
            }
        }
        let (activity, home, lap, step) = {
            let shared = self.run.shared.lock().expect("shared lock");
            (
                shared.activity.clone(),
                shared.home,
                shared.lap,
                shared.step.clone(),
            )
        };
        put("activity", activity.clone());
        put("lap", lap.map_or(String::new(), |l| l.to_string()));
        put("step", step);
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

/// Why the driver stopped driving.
pub(crate) enum Halt {
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

pub(crate) fn signalled() -> bool {
    SIGNAL.load(Ordering::Relaxed) != 0
}

pub(crate) fn signal_name() -> &'static str {
    match SIGNAL.load(Ordering::Relaxed) {
        libc::SIGTERM => "SIGTERM",
        libc::SIGINT => "SIGINT",
        _ => "signal",
    }
}

/// SIGTERM (`stop`, the unit going away) and SIGINT (ctrl-C in `--foreground`) end the drive.
pub(crate) fn install_signal_handlers() {
    // Safety: the handler only stores to an atomic, which is async-signal-safe.
    unsafe {
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
    }
}

/// Create (or reopen) the run directory: a `meta.json` segment for this start, carrying
/// `segment`'s fields beside the start time, boot and host, and `events.csv` behind a [`Run`].
/// An existing directory is appended to, so a restarted run stays one battery curve.
pub(crate) fn open_run(out: &Path, segment: serde_json::Value) -> Result<Arc<Run>, Failure> {
    let io = |what: &str, e: std::io::Error| Failure::new(exit::FAILED, format!("{what}: {e}"));
    std::fs::create_dir_all(out).map_err(|e| io(&format!("creating {}", out.display()), e))?;
    if let Some(parent) = out.parent() {
        fsync_dir(parent);
    }

    let meta_path = out.join("meta.json");
    let mut notes = Vec::new();
    let mut meta = serde_json::json!({ "segments": [] });
    if let Ok(bytes) = std::fs::read(&meta_path) {
        match serde_json::from_slice(&bytes) {
            Ok(read) => meta = read,
            // Starting over must not be the end of what it says; set it aside for a person.
            Err(e) => {
                let aside = out.join(format!("meta.json.unreadable-{}", unix_secs() as u64));
                std::fs::rename(&meta_path, &aside)
                    .map_err(|e| io("setting aside meta.json", e))?;
                notes.push(format!(
                    "meta.json unreadable ({e}), moved to {}",
                    aside.display()
                ));
            }
        }
    }
    let now = unix_secs();
    let (y, mo, d, h, mi, s) = crate::show::civil(now as i64);
    let mut entry = serde_json::json!({
        "started_wall": now,
        "started_utc": format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}Z"),
        "boot_id": read_text(Path::new("/proc/sys/kernel/random/boot_id")),
        "host": read_text(Path::new("/proc/sys/kernel/hostname")),
        "robotctl": env!("CARGO_PKG_VERSION"),
    });
    for extra in [provenance(), segment] {
        if let (Some(entry), serde_json::Value::Object(extra)) = (entry.as_object_mut(), extra) {
            entry.extend(extra);
        }
    }
    if let Some(segments) = meta["segments"].as_array_mut() {
        segments.push(entry);
    }
    write_atomic(
        &meta_path,
        serde_json::to_string_pretty(&meta)
            .unwrap_or_default()
            .as_bytes(),
    )
    .map_err(|e| io("writing meta.json", e))?;

    for name in ["events.csv", "samples.csv"] {
        match repair_tail(&out.join(name)) {
            Ok(0) => {}
            Ok(cut) => notes.push(format!("{name}: cut {cut} bytes of an unfinished row")),
            Err(e) => return Err(io("repairing a csv", e)),
        }
    }

    let events = DurableCsv::open(
        &out.join("events.csv"),
        ["wall", "t_s", "kind", "detail"].map(String::from).to_vec(),
    )
    .map_err(|e| io("opening events.csv", e))?;
    let run = Arc::new(Run {
        t0: Instant::now(),
        shared: Mutex::new(Shared {
            activity: "starting".into(),
            ..Shared::default()
        }),
        events: Mutex::new(events),
    });
    for note in notes {
        run.event("repaired", &note);
    }
    Ok(run)
}

/// Log until a signal, touching nothing — the tail of a run that has stopped driving.
pub(crate) fn wait_for_signal(run: &Run) {
    while !signalled() {
        std::thread::sleep(Duration::from_millis(200));
    }
    run.event("signal", signal_name());
}

/// A failure's first line. `Client`'s messages carry a remedy on the lines after it, which is
/// right for a person at a terminal and noise in an events log.
pub(crate) fn brief(failure: &Failure) -> String {
    failure
        .message
        .lines()
        .next()
        .unwrap_or_default()
        .to_owned()
}

pub(crate) fn json<T: Serialize>(value: &T) -> String {
    serde_json::to_string(value).unwrap_or_default()
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csv_quotes_only_what_needs_it() {
        assert_eq!(csv_field("walk 12s"), "walk 12s");
        assert_eq!(
            csv_field(r#"{"accepted":true,"x":1}"#),
            r#""{""accepted"":true,""x"":1}""#
        );
    }

    #[test]
    fn sysfs_entries_sort_numerically() {
        let mut names = vec!["policy10", "policy2", "policy0"];
        names.sort_by_key(|n| by_index(n));
        assert_eq!(names, ["policy0", "policy2", "policy10"]);
    }

    #[test]
    fn current_window_keeps_mean_and_peak_per_joint_and_summed() {
        let mut w = CurrentWindow::default();
        let mut a = [0.0; 15];
        a[3] = 100.0;
        a[4] = -50.0;
        let mut b = [0.0; 15];
        b[3] = 300.0;
        w.add(&a);
        w.add(&b);
        assert_eq!(w.n, 2);
        assert_eq!(w.sum_ma[3] / 2.0, 200.0);
        assert_eq!(w.peak_ma[3], 300.0);
        assert_eq!(w.peak_ma[4], 50.0, "magnitude, whatever the sign");
        assert_eq!(w.total_sum_ma / 2.0, 225.0);
        assert_eq!(
            w.total_peak_ma, 300.0,
            "the peak of the sum, not the sum of the peaks"
        );
    }

    #[test]
    fn appending_keeps_the_existing_header() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("samples.csv");
        let row = |k: &str, v: &str| (k.to_owned(), v.to_owned());
        DurableCsv::open(&path, vec!["a".into(), "b".into()])
            .unwrap()
            .write(&[row("a", "1"), row("b", "2")]);
        DurableCsv::open(&path, vec!["a".into(), "new".into(), "b".into()])
            .unwrap()
            .write(&[row("a", "3"), row("new", "x"), row("b", "4")]);
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "a,b\n1,2\n3,4\n");
    }

    #[test]
    fn a_torn_row_is_cut_before_the_next_start_appends() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("samples.csv");
        std::fs::write(&path, "a,b\n1,2\n3,").unwrap();
        assert_eq!(repair_tail(&path).unwrap(), 2);
        let row = |k: &str, v: &str| (k.to_owned(), v.to_owned());
        DurableCsv::open(&path, vec!["a".into(), "b".into()])
            .unwrap()
            .write(&[row("a", "5"), row("b", "6")]);
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "a,b\n1,2\n5,6\n");
    }

    #[test]
    fn a_whole_file_and_a_missing_one_need_no_repair() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("events.csv");
        assert_eq!(repair_tail(&path).unwrap(), 0);
        std::fs::write(&path, "a,b\n1,2\n").unwrap();
        assert_eq!(repair_tail(&path).unwrap(), 0);
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "a,b\n1,2\n");
    }

    #[test]
    fn a_torn_header_is_rewritten_whole() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("samples.csv");
        std::fs::write(&path, "a,").unwrap();
        repair_tail(&path).unwrap();
        DurableCsv::open(&path, vec!["a".into(), "b".into()]).unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "a,b\n");
    }

    #[test]
    fn damage_after_the_header_does_not_cost_the_header() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("samples.csv");
        let mut bytes = b"a,b\n".to_vec();
        bytes.extend([0xff, 0xfe, b'\n']);
        std::fs::write(&path, &bytes).unwrap();
        DurableCsv::open(&path, vec!["x".into()]).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), bytes, "no second header");
    }

    #[test]
    fn an_atomic_write_replaces_and_leaves_no_temporary() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("meta.json");
        write_atomic(&path, b"one").unwrap();
        write_atomic(&path, b"two").unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"two");
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
    }

    fn open(out: &Path) -> Arc<Run> {
        open_run(out, serde_json::json!({ "lap": "x" })).unwrap_or_else(|_| panic!("open_run"))
    }

    #[test]
    fn reopening_a_run_adds_a_segment() {
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("run");
        open(&out);
        open(&out);
        let meta: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(out.join("meta.json")).unwrap()).unwrap();
        let segments = meta["segments"].as_array().unwrap();
        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0]["lap"], "x");
        assert!(segments[0].get("policies").is_some(), "{}", segments[0]);
    }

    #[test]
    fn an_unreadable_meta_is_set_aside_and_reported() {
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("run");
        std::fs::create_dir(&out).unwrap();
        std::fs::write(out.join("meta.json"), "{ cut off").unwrap();
        std::fs::write(out.join("events.csv"), "wall,t_s,kind,detail\n1.0,0.0,st").unwrap();
        open(&out);
        let kept: Vec<String> = std::fs::read_dir(&out)
            .unwrap()
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.starts_with("meta.json.unreadable-"))
            .collect();
        assert_eq!(kept.len(), 1);
        assert_eq!(
            std::fs::read_to_string(out.join(&kept[0])).unwrap(),
            "{ cut off"
        );
        let events = std::fs::read_to_string(out.join("events.csv")).unwrap();
        assert!(events.contains("repaired,"), "{events}");
        assert!(events.contains("meta.json unreadable"), "{events}");
        assert!(events.contains("events.csv: cut 10 bytes"), "{events}");
        assert_eq!(events.matches("wall,t_s").count(), 1, "{events}");
    }

    #[test]
    fn only_a_yes_from_timedatectl_is_a_synced_clock() {
        assert!(says_synchronized("yes\n"));
        assert!(!says_synchronized("no\n"));
        assert!(!says_synchronized(""));
    }

    #[test]
    fn provenance_reads_the_slot_and_the_os() {
        let cmdline = "root=PARTLABEL=rootfs_a rauc.slot=A rw rootwait";
        assert_eq!(boot_slot(cmdline), Some("A"));
        assert_eq!(boot_slot("root=/dev/mmcblk0p2 rw"), None);
        let release = "ID=microduck\nVERSION=\"1.0.1 (wrynose)\"\nVERSION_ID=1.0.1\n";
        assert_eq!(os_version(release), Some("1.0.1 (wrynose)"));
    }

    #[test]
    fn policies_are_hashed_by_name() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("walk.onnx"), b"abc").unwrap();
        std::fs::write(dir.path().join("notes.txt"), b"abc").unwrap();
        let hashes = policy_hashes(dir.path());
        assert_eq!(hashes.len(), 1);
        assert_eq!(
            hashes["walk.onnx"],
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn boots_are_read_from_either_list_format() {
        let list = "IDX BOOT ID                          FIRST ENTRY                 LAST ENTRY\n \
                    -1 d788cdfe1f6b4561a8d785fb87c8d1c0 Tue 2026-11-03 11:55:59 UTC Tue\n  \
                    0 f71ac0d1ac394adc82069b679c737783 Wed 2026-11-04 02:43:12 UTC Wed\n\
                    -- No entries --\n";
        assert_eq!(
            listed_boots(list),
            [
                "d788cdfe1f6b4561a8d785fb87c8d1c0",
                "f71ac0d1ac394adc82069b679c737783"
            ]
        );
    }

    #[test]
    fn a_boot_finds_the_run_it_wrote_to() {
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("run");
        std::fs::create_dir(&out).unwrap();
        std::fs::write(
            out.join("meta.json"),
            r#"{"segments":[{"boot_id":"f437bd57-c8e1-4248-aacf-031045e0c4ca"}]}"#,
        )
        .unwrap();
        std::fs::create_dir(dir.path().join("broken")).unwrap();
        std::fs::write(dir.path().join("broken/meta.json"), "").unwrap();
        let runs = runs_by_boot(dir.path());
        assert_eq!(runs.len(), 1);
        assert_eq!(runs["f437bd57c8e14248aacf031045e0c4ca"], out);
    }
}
