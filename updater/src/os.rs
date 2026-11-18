//! `os.*`: the operating system's A/B slots, through the image's `microduck-osupdate`.
//!
//! **This module is a wrapper, on purpose.** The script is what knows about RAUC, the channel
//! and the bucket, and it is also what serves the LAN page on port 8090 and what someone with a
//! shell runs by hand. Re-implementing it here would give the robot two OS updaters that could
//! disagree about the one thing that matters — which slot boots next — so this module runs the
//! script and reads back what it says, and adds only what a socket needs: a refusal on a board
//! without it, a busy answer, the preflight every update passes, and an answer that waits long
//! enough to carry the script's own early refusals.
//!
//! The work runs as a transient unit, `microduck-osupdate-apply`, the same one the 8090 page
//! starts, so either path sees the other's update as running. Not as a child of this process:
//! it ends in `systemctl reboot`, it must outlive the connection that asked for it, and it needs
//! none of the sandboxing `updaterd.service` applies to itself.
//!
//! Every answer is the script's `status`, as an [`OsStatus`].

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use serde::Deserialize;

use crate::proto::{self, OsAvailable, OsBuild, OsSlot, OsStatus};

/// Where the image installs the script. Absent on any board not built from `microduck_yocto`.
pub const SCRIPT: &str = "/usr/bin/microduck-osupdate";
/// The script's own state, on the data partition. Read here only for the phase file's age.
pub const STATE_DIR: &str = "/var/lib/robot/osupdate";
/// The transient unit an update runs in. The 8090 page starts the same one.
const APPLY_UNIT: &str = "microduck-osupdate-apply";

/// `status` runs `rauc status` three times over and reads a few files.
const STATUS_TIMEOUT: Duration = Duration::from_secs(20);
/// `check` makes two downloads, each capped at 30 s by the script. Under `duckctl`'s 60 s idle
/// budget, so a hung mirror is an error from here rather than silence there.
const CHECK_TIMEOUT: Duration = Duration::from_secs(55);
/// How long `os.apply` waits for the update to get past its checks before answering.
///
/// Long enough for the script's own refusals — a build already running, a channel with no
/// manifest, a bad build id — to come back as an error rather than as a phase that reads
/// `idle` a second later; short enough that a client is not left waiting on a slow mirror.
const SETTLE: Duration = Duration::from_secs(30);
const POLL: Duration = Duration::from_millis(250);

/// The programs this module runs. Fields so a test can substitute stubs for every one of them.
#[derive(Debug, Clone)]
pub struct Tools {
    pub script: PathBuf,
    pub state_dir: PathBuf,
    pub systemd_run: PathBuf,
    pub systemctl: PathBuf,
}

impl Tools {
    /// What a robot runs.
    pub fn system() -> Self {
        Self {
            script: SCRIPT.into(),
            state_dir: STATE_DIR.into(),
            systemd_run: "systemd-run".into(),
            systemctl: "systemctl".into(),
        }
    }

    /// Whether this board can update its OS at all.
    fn supported(&self) -> Result<(), proto::Error> {
        if self.script.exists() {
            return Ok(());
        }
        Err(proto::Error::new(
            proto::code::NOT_SUPPORTED,
            format!(
                "this robot's OS cannot be updated over the air: it was not built with A/B \
                 slots ({} is missing). Reflash it with a microduck_yocto image.",
                self.script.display()
            ),
        ))
    }
}

/// What an `os.apply` or `os.switch` starts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// The channel's newest build, or the one named.
    Apply(Option<String>),
    /// Boot the other slot as it is.
    Switch,
}

impl Action {
    fn args(&self) -> Vec<&str> {
        match self {
            Action::Apply(None) => vec!["apply"],
            Action::Apply(Some(build)) => vec!["apply", build],
            Action::Switch => vec!["switch"],
        }
    }
}

/// A build id as the channel names it: the build's UTC time, `YYYYMMDDhhmmss`.
///
/// The script checks this too. Checked here as well so a bad one is `INVALID_PARAMS` at once,
/// rather than a transient unit that starts, refuses and leaves its refusal in a file.
pub fn valid_build(build: &str) -> bool {
    build.len() == 14 && build.bytes().all(|b| b.is_ascii_digit())
}

/// The slots, the channel as last checked, and the phase.
pub async fn status(tools: &Tools) -> Result<OsStatus, proto::Error> {
    tools.supported()?;
    let out = run(&tools.script, &["status"], STATUS_TIMEOUT).await?;
    parse_status(&out)
}

/// Ask the channel what is newest, then answer with the status.
///
/// Refused while an update runs. The script's `check` resets the phase to `idle` when it fails,
/// so a check that hit a flaky mirror mid-download would make a running update read as stopped.
pub async fn check(tools: &Tools) -> Result<OsStatus, proto::Error> {
    tools.supported()?;
    if running(tools).await {
        return Err(busy());
    }
    if let Err(e) = run(&tools.script, &["check"], CHECK_TIMEOUT).await {
        return Err(proto::Error::new(proto::code::NETWORK, e.message));
    }
    status(tools).await
}

/// Start an update or a slot switch, and answer once it is under way.
///
/// "Under way" is a phase past the script's checks — `downloading`, `installing` or
/// `rebooting` — and the phase file must have been written by this run rather than left by one
/// that died. If the unit ends first, the script refused, and what it said is the error.
pub async fn start(tools: &Tools, action: &Action) -> Result<OsStatus, proto::Error> {
    start_with(tools, action, SETTLE).await
}

async fn start_with(
    tools: &Tools,
    action: &Action,
    settle: Duration,
) -> Result<OsStatus, proto::Error> {
    tools.supported()?;
    if let Action::Apply(Some(build)) = action
        && !valid_build(build)
    {
        return Err(proto::Error::new(
            proto::code::INVALID_PARAMS,
            format!("build must be 14 digits, the build's UTC time: got {build:?}"),
        ));
    }
    if running(tools).await {
        return Err(busy());
    }

    // A second back, not now: the kernel stamps files from a coarse clock that can trail
    // `SystemTime::now()` by a tick, and a phase written by this run must not read as stale.
    // Nothing a second old can be this run's predecessor's — that one finished, or the unit
    // would still be running.
    let since = SystemTime::now() - Duration::from_secs(1);

    let unit = format!("--unit={APPLY_UNIT}");
    let mut args = vec!["-q", unit.as_str(), "--collect"];
    let script = tools.script.to_string_lossy();
    args.push(&script);
    args.extend(action.args());
    run(&tools.systemd_run, &args, STATUS_TIMEOUT).await?;

    let deadline = tokio::time::Instant::now() + settle;
    loop {
        let fresh = phase_written_since(&tools.state_dir, since);
        let phase = std::fs::read_to_string(tools.state_dir.join("phase")).unwrap_or_default();
        if fresh && matches!(phase.trim(), "downloading" | "installing" | "rebooting") {
            return status(tools).await;
        }
        if !running(tools).await {
            let status = status(tools).await?;
            // Every path through the script writes the phase before anything can go wrong, so a
            // unit that ended without writing one died before it started.
            if !fresh {
                return Err(proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "the OS updater exited without starting; `journalctl -u \
                     microduck-osupdate-apply` on the robot says why",
                ));
            }
            // Only `fail` leaves a finished unit idle; success ends in a reboot.
            if status.phase == "idle" {
                return Err(proto::Error::new(
                    proto::code::PREFLIGHT_FAILED,
                    status
                        .message
                        .unwrap_or_else(|| "the OS updater stopped without saying why".into()),
                ));
            }
            return Ok(status);
        }
        if tokio::time::Instant::now() >= deadline {
            // Still checking: a slow mirror. The update carries on, and `os.status` follows it.
            return status(tools).await;
        }
        tokio::time::sleep(POLL).await;
    }
}

fn busy() -> proto::Error {
    proto::Error::new(proto::code::BUSY, "an OS update is already running")
}

/// Whether the update unit exists and has not finished. `--collect` removes it once it has,
/// failed or not, so anything but these two states is "not running".
pub async fn running(tools: &Tools) -> bool {
    let unit = format!("{APPLY_UNIT}.service");
    match run(
        &tools.systemctl,
        &["show", "-p", "ActiveState", "--value", &unit],
        STATUS_TIMEOUT,
    )
    .await
    {
        Ok(state) => matches!(state.trim(), "active" | "activating" | "deactivating"),
        // A systemctl that cannot answer is not evidence of an update; the script's own lock
        // still refuses a second one.
        Err(_) => false,
    }
}

fn phase_written_since(state_dir: &Path, since: SystemTime) -> bool {
    std::fs::metadata(state_dir.join("phase"))
        .and_then(|m| m.modified())
        .is_ok_and(|modified| modified >= since)
}

/// Run a program and return its stdout, or its last line of stderr as the error.
async fn run(program: &Path, args: &[&str], timeout: Duration) -> Result<String, proto::Error> {
    let could_not = |e: std::io::Error| {
        proto::Error::new(
            proto::code::INTERNAL_ERROR,
            format!("could not run {}: {e}", program.display()),
        )
    };
    let mut command = tokio::process::Command::new(program);
    command
        .args(args)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .kill_on_drop(true);
    let child = crate::spawn::retrying_busy(&mut command)
        .await
        .map_err(could_not)?;
    let output = tokio::time::timeout(timeout, child.wait_with_output())
        .await
        .map_err(|_| {
            proto::Error::new(
                proto::code::INTERNAL_ERROR,
                format!(
                    "{} {} did not finish in {timeout:?}",
                    program.display(),
                    args.join(" ")
                ),
            )
        })?
        .map_err(could_not)?;

    if output.status.success() {
        return Ok(String::from_utf8_lossy(&output.stdout).into_owned());
    }
    let stderr = String::from_utf8_lossy(&output.stderr);
    let why = stderr
        .lines()
        .rev()
        .find(|l| !l.trim().is_empty())
        .unwrap_or("no output")
        .trim()
        .to_owned();
    Err(proto::Error::new(
        proto::code::INTERNAL_ERROR,
        format!("{} {} failed: {why}", program.display(), args.join(" ")),
    ))
}

/// The script's `status`, exactly as it prints it: unknowns are empty strings.
#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct RawStatus {
    machine: String,
    channel: String,
    slot: String,
    installed_build: String,
    available: Option<RawAvailable>,
    other: Option<RawSlot>,
    builds: Vec<OsBuild>,
    phase: String,
    progress: u64,
    message: String,
    last_result: String,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct RawAvailable {
    version: String,
    build: String,
    size: u64,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct RawSlot {
    slot: String,
    build: String,
    good: bool,
    installed: String,
}

fn known(s: String) -> Option<String> {
    let s = s.trim();
    (!s.is_empty()).then(|| s.to_owned())
}

fn parse_status(json: &str) -> Result<OsStatus, proto::Error> {
    let raw: RawStatus = serde_json::from_str(json.trim()).map_err(|e| {
        proto::Error::new(
            proto::code::INTERNAL_ERROR,
            format!("microduck-osupdate status printed something unreadable: {e}"),
        )
    })?;
    Ok(OsStatus {
        machine: known(raw.machine),
        channel: known(raw.channel),
        slot: known(raw.slot),
        installed_build: known(raw.installed_build),
        available: raw.available.map(|a| OsAvailable {
            version: a.version,
            build: a.build,
            size: a.size,
        }),
        // The script always prints `other`; a board with one slot prints it empty.
        other: raw.other.and_then(|o| {
            Some(OsSlot {
                slot: Some(known(o.slot)?),
                build: known(o.build),
                good: o.good,
                installed: known(o.installed),
            })
        }),
        builds: raw.builds,
        phase: known(raw.phase).unwrap_or_else(|| "idle".into()),
        progress: raw.progress.min(100) as u8,
        message: known(raw.message),
        last_result: known(raw.last_result),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What the script printed on a beta board mid-download, after an earlier rollback.
    const DOWNLOADING: &str = r#"{"machine":"microduck-beta","channel":"dev","slot":"A","installed_build":"","available":{"version":"0.4.0-dev","build":"20261028093000","size":412000000},"other":{"slot":"B","build":"20261020101500","good":true,"installed":"2026-10-20T10:40:02Z"},"builds":[{"channel":"dev","build":"20261020101500","version":"0.3.9-dev"},{"channel":"dev","build":"20261028093000","version":"0.4.0-dev"}],"phase":"downloading","progress":37,"message":"downloading 0.4.0-dev","last_result":""}"#;

    #[test]
    fn empty_strings_are_read_as_unknown() {
        let s = parse_status(DOWNLOADING).unwrap();
        assert_eq!(s.machine.as_deref(), Some("microduck-beta"));
        assert_eq!(s.slot.as_deref(), Some("A"));
        // A flashed slot has no build, and must not read as a build called "".
        assert_eq!(s.installed_build, None);
        assert_eq!(s.last_result, None);
        assert_eq!(s.phase, "downloading");
        assert_eq!(s.progress, 37);
        let other = s.other.unwrap();
        assert_eq!(other.slot.as_deref(), Some("B"));
        assert!(other.good);
        assert_eq!(s.builds.len(), 2);
        assert_eq!(s.available.unwrap().size, 412_000_000);
    }

    #[test]
    fn an_idle_board_that_never_checked_has_nothing_available() {
        let s = parse_status(
            r#"{"machine":"microduck-beta","channel":"dev","slot":"A","installed_build":"","available":null,"other":{"slot":"","build":"","good":false,"installed":""},"builds":[],"phase":"idle","progress":0,"message":"","last_result":""}"#,
        )
        .unwrap();
        assert_eq!(s.available, None);
        assert_eq!(s.other, None);
        assert_eq!(s.message, None);
    }

    #[test]
    fn build_ids_are_fourteen_digits() {
        assert!(valid_build("20261028093000"));
        assert!(!valid_build("2026102809300"));
        assert!(!valid_build("2026102809300a"));
        assert!(!valid_build("../../etc/x.env"));
    }

    fn write_exec(path: &Path, body: &str) {
        std::fs::write(path, body).unwrap();
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    /// Stubs for all three programs. `systemd-run` records its arguments and runs the command in
    /// the foreground, so by the time it returns the "unit" has finished and `systemctl` can say
    /// so. `apply` behaves as `apply_body` says, with `$STATE` the state directory.
    fn stubs(apply_body: &str) -> (tempfile::TempDir, Tools) {
        let dir = tempfile::tempdir().unwrap();
        let state = dir.path().join("state");
        std::fs::create_dir(&state).unwrap();
        let script = dir.path().join("microduck-osupdate");
        write_exec(
            &script,
            &format!(
                r#"#!/bin/sh
STATE={state}
case "$1" in
status)
	printf '{{"machine":"microduck-beta","channel":"dev","slot":"A","installed_build":"","available":null,"other":{{"slot":"B","build":"","good":true,"installed":""}},"builds":[],"phase":"%s","progress":0,"message":"%s","last_result":""}}\n' "$(cat $STATE/phase 2>/dev/null || echo idle)" "$(cat $STATE/message 2>/dev/null)" ;;
apply|switch)
{apply_body} ;;
esac
"#,
                state = state.display(),
            ),
        );
        let systemd_run = dir.path().join("systemd-run");
        write_exec(
            &systemd_run,
            r#"#!/bin/sh
echo "$@" >> "$(dirname "$0")/calls"
shift 3
"$@"
exit 0
"#,
        );
        let systemctl = dir.path().join("systemctl");
        write_exec(&systemctl, "#!/bin/sh\necho inactive\n");
        let tools = Tools {
            script,
            state_dir: state,
            systemd_run,
            systemctl,
        };
        (dir, tools)
    }

    #[tokio::test]
    async fn a_board_without_the_script_says_so() {
        let tools = Tools {
            script: "/nonexistent/microduck-osupdate".into(),
            ..Tools::system()
        };
        for e in [
            status(&tools).await.unwrap_err(),
            check(&tools).await.unwrap_err(),
            start(&tools, &Action::Switch).await.unwrap_err(),
        ] {
            assert_eq!(e.code, proto::code::NOT_SUPPORTED, "{}", e.message);
        }
    }

    #[tokio::test]
    async fn an_update_under_way_answers_with_its_phase() {
        let (dir, tools) = stubs(
            r#"	echo "downloading 0.4.0-dev" > $STATE/message; echo downloading > $STATE/phase"#,
        );
        let s = start_with(&tools, &Action::Apply(Some("20261028093000".into())), POLL)
            .await
            .unwrap();
        assert_eq!(s.phase, "downloading");
        let calls = std::fs::read_to_string(dir.path().join("calls")).unwrap();
        assert_eq!(
            calls.trim(),
            format!(
                "-q --unit=microduck-osupdate-apply --collect {} apply 20261028093000",
                tools.script.display()
            )
        );
    }

    #[tokio::test]
    async fn a_refusal_by_the_script_is_the_error() {
        let (_dir, tools) = stubs(
            r#"	echo "build 20261028093000 is already running" > $STATE/message; echo idle > $STATE/phase"#,
        );
        let e = start_with(&tools, &Action::Apply(None), POLL)
            .await
            .unwrap_err();
        assert_eq!(e.code, proto::code::PREFLIGHT_FAILED);
        assert_eq!(e.message, "build 20261028093000 is already running");
    }

    #[tokio::test]
    async fn a_phase_left_by_an_earlier_run_is_not_this_one() {
        // A run that died mid-download left `downloading` behind, an hour ago. This one exits
        // without writing anything: it must not be reported as downloading.
        let (_dir, tools) = stubs("	exit 1");
        let phase = tools.state_dir.join("phase");
        std::fs::write(&phase, "downloading\n").unwrap();
        let hour_ago = SystemTime::now() - Duration::from_secs(3600);
        std::fs::File::options()
            .write(true)
            .open(&phase)
            .unwrap()
            .set_modified(hour_ago)
            .unwrap();
        let e = start_with(&tools, &Action::Switch, POLL).await.unwrap_err();
        assert_eq!(e.code, proto::code::INTERNAL_ERROR, "{}", e.message);
    }

    #[tokio::test]
    async fn a_bad_build_id_never_starts_a_unit() {
        let (dir, tools) = stubs("	exit 0");
        let e = start_with(&tools, &Action::Apply(Some("latest".into())), POLL)
            .await
            .unwrap_err();
        assert_eq!(e.code, proto::code::INVALID_PARAMS);
        assert!(!dir.path().join("calls").exists());
    }

    #[tokio::test]
    async fn a_running_update_refuses_a_second() {
        let (dir, tools) = stubs("	exit 0");
        write_exec(&tools.systemctl, "#!/bin/sh\necho active\n");
        assert_eq!(
            start_with(&tools, &Action::Switch, POLL)
                .await
                .unwrap_err()
                .code,
            proto::code::BUSY
        );
        assert_eq!(check(&tools).await.unwrap_err().code, proto::code::BUSY);
        assert!(!dir.path().join("calls").exists());
    }
}
