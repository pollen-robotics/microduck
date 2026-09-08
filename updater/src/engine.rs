//! The update state machine.
//!
//! ```text
//! preflight → fetch manifest → verify sig → compatibility
//!   → download → verify hash → verify sig
//!   → extract → [pre hook] → ATOMIC SWAP → [post hook] → apply
//!   → HEALTH GATE → healthy ? commit+prune : ROLLBACK
//! ```
//!
//! Full description in `docs/design/updater-design.md` §7. Three rules shape everything
//! here:
//!
//!  - **Any failure at or after the swap rolls back.** Hook failure, health
//!    failure and timeout are all the same outcome — there is no "mostly applied".
//!  - **Nothing is extracted to a live path before signature and hash both pass.**
//!  - **The boot counter is armed before the swap**, so a crash between swap and
//!    health check is still recoverable. The reverse order would leave an
//!    unrecorded bad release live.

use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::config::{ApplyAction, ComponentConfig, Config, HealthCheck};
use crate::faults::Faults;
use crate::journal::{BootCounter, Journal, PendingUpdate, Pins, UpdateLock, now_unix};
use crate::manifest::{Capabilities, Compatibility, Manifest};
use crate::proto::{
    ApplyResult, CheckResult, ComponentId, ComponentStatus, InstalledRelease, LogEntry, Outcome,
    Phase, Progress, RunEvent,
};
use crate::robot::RobotClient;
use crate::store::Store;
use crate::transcript::Transcript;
use crate::verify::KeyRing;
use crate::{Error, hooks, preflight, source, verify};

/// Boots a pending update gets to prove itself before unconditional revert.
pub const MAX_BOOT_ATTEMPTS: u32 = 2;

/// How long to wait on any single `robotd` query.
pub const ROBOT_QUERY_TIMEOUT: Duration = Duration::from_secs(2);

/// Cap on how long an apply action (`systemctl restart`, signal) may take.
const APPLY_ACTION_TIMEOUT: Duration = Duration::from_secs(30);

/// Hooks get their own generous ceiling — migrations can be slow — but not
/// unbounded.
const HOOK_TIMEOUT: Duration = Duration::from_secs(120);

/// The pre-install hook gets much longer, because it is the one that installs what the release
/// needs and cannot have: ONNX Runtime, and the GStreamer stack for `mediad` — around 100 MB of
/// apt on a board that has never had it, over whatever wifi the robot is on.
///
/// **Affordable precisely because of where it runs.** Nothing has been swapped yet, the old
/// release is still live and serving, and a hook that runs long is a slow update rather than a
/// robot at risk — where the same minutes spent in the post-install hook would sit between the
/// swap and the restart, with the board running neither release properly.
///
/// Ten minutes is the point past which a stuck apt is more likely wedged than slow. Bounded, not
/// unbounded, for the reason the ceiling exists at all.
///
/// The number lives in `duck-ipc-proto` because it is a contract with every client, not a private
/// budget: the phase notification arrives before the hook, so this is the longest an apply can go
/// silent, and a client with a shorter idle budget calls a working update a dead robot.
const PRE_INSTALL_HOOK_TIMEOUT: Duration =
    Duration::from_secs(crate::proto::UPDATE_MAX_SILENCE_SECONDS);

/// Interval between health probes while the gate is open.
const HEALTH_POLL_INTERVAL: Duration = Duration::from_millis(500);

/// Shortest gap between two download-progress notifications.
///
/// The source reports every HTTP chunk it writes, which for a release artifact is thousands of
/// events, and every subscriber pays for all of them. Over BLE that is fatal rather than
/// wasteful: a progress line is around a hundred bytes, which is five or six notifications at the
/// 20-byte floor `btd` frames to, and `btd` drops lines when the client falls behind — so a phone
/// saw an arbitrary subset of the percentages and a bar that jumped 12 → 61 → 34. Four a second,
/// each a different whole percent, is a bar that moves smoothly and a stream a 20-byte pipe can
/// carry.
const PROGRESS_MIN_GAP: Duration = Duration::from_millis(250);

/// Which of a download's progress reports are worth publishing.
///
/// [`PROGRESS_MIN_GAP`] is why this exists at all. Two rules, and the third method is why it is a
/// type rather than three variables: a percent held back by the gap has to be published when the
/// download ends, or a download whose last change lands inside the gap visibly finishes at 97%.
struct DownloadProgress {
    /// The last percent published, so an unchanged one is dropped. Starts at 0 because the caller
    /// publishes that before the first chunk arrives.
    sent: Option<u8>,
    last: tokio::time::Instant,
    /// Computed, suppressed by the gap, and not yet superseded.
    held: Option<u8>,
}

impl DownloadProgress {
    fn started(now: tokio::time::Instant) -> Self {
        Self {
            sent: Some(0),
            last: now,
            held: None,
        }
    }

    /// Whether this report should be published.
    fn admit(&mut self, percent: Option<u8>, now: tokio::time::Instant) -> bool {
        let due = now.duration_since(self.last) >= PROGRESS_MIN_GAP;

        // With no total there is no percent to coalesce on, so the gap is the only thing
        // rationing "still downloading" — and there is no number worth holding back either.
        if percent.is_none() {
            if due {
                self.last = now;
                return true;
            }
            return false;
        }

        if percent == self.sent {
            return false;
        }
        if due {
            self.sent = percent;
            self.held = None;
            self.last = now;
            return true;
        }
        self.held = percent;
        false
    }

    /// The percent held back by the gap, once there are no more reports coming.
    fn flush(&mut self) -> Option<u8> {
        self.held.take()
    }
}

/// Headroom multiplier over the artifact size: download + extracted copy + slack.
const SPACE_MULTIPLIER: u64 = 3;

/// Space demanded when a manifest omits `size`. Not a real estimate — just enough
/// that the check cannot silently become a no-op.
const MIN_REQUIRED_BYTES: u64 = 32 * 1024 * 1024;

/// Entries retained in the update log.
const LOG_CAPACITY: usize = 200;

/// Run transcripts retained beside it.
///
/// Far fewer than [`LOG_CAPACITY`], because they are not the same kind of record: a log entry is
/// one line and answers "what has this board been through", while a transcript is kilobytes and
/// answers "what did *that* one do". Nobody reads the ninetieth-most-recent transcript, and the
/// log still names every attempt that far back.
const TRANSCRIPTS_KEPT: usize = 20;

/// Highest on-disk/config schema this build understands.
///
/// A release declaring a higher `schema_version` expects migrations this engine has
/// never heard of, so it is refused rather than installed and hoped for. Bump this
/// in the same change that teaches the engine the new layout.
pub const SUPPORTED_SCHEMA_VERSION: u32 = 1;

/// Manifest copy kept inside each installed release, so `select` can re-check
/// compatibility and `list_installed` can report provenance without a network
/// round-trip.
const EMBEDDED_MANIFEST: &str = ".updater-manifest.json";

/// Run a blocking closure on the blocking pool.
///
/// Used for hashing, signature verification, extraction and recursive deletes:
/// all of them run for seconds on a Pi-class board, and leaving them on an async
/// worker would stall the IPC tasks that must keep serving `status`/`subscribe`
/// during an update (`docs/design/architecture.md` §2.3).
async fn blocking<T, F>(f: F) -> Result<T, Error>
where
    F: FnOnce() -> Result<T, Error> + Send + 'static,
    T: Send + 'static,
{
    tokio::task::spawn_blocking(f)
        .await
        .map_err(|e| Error::Internal(format!("blocking task failed: {e}")))?
}

/// Where the engine publishes progress. Unbounded and non-blocking: progress is
/// advisory, and the update must never be slowed by whoever is watching it.
pub type ProgressTx = tokio::sync::mpsc::UnboundedSender<Progress>;

pub struct Engine {
    config: Config,
    /// Behind an `Arc` so verification can be handed to `spawn_blocking` without
    /// borrowing `self` across an await.
    keys: std::sync::Arc<KeyRing>,
    robot: Box<dyn RobotClient>,
    journal: Journal,
    boot_counter: BootCounter,
    pins: Pins,
    faults: Faults,
    /// Whether an applied release schedules the deferred `updaterd`/`btd` restarts.
    ///
    /// On a robot, always. Off only in the test binaries, and for a reason that is about processes
    /// rather than about restarts — see [`Engine::without_deferred_restarts`].
    deferred_restarts: bool,
    /// Where systemd's installed unit files live, for the orphan check ([`crate::orphan`]).
    ///
    /// A field so a test can point it at a directory it owns, and **not** a config key, for the
    /// reason `NEVER_RESTART` is not one: `/etc/systemd/system` is a property of the system rather
    /// than a choice, and a board that got it wrong would silently stop checking.
    unit_dir: PathBuf,
}

/// Where an operation says what it is doing — to whoever is watching, and to the record that
/// outlives them.
///
/// **One object rather than a `ProgressTx` and an `emit` closure side by side.** They were always
/// the same event seen twice, and two parameters made it possible to tell one and not the other —
/// which is exactly what had happened: every phase reached the subscriber and none of them reached
/// the disk, so the phase timeline existed only for as long as somebody held a socket open.
pub struct Recorder {
    component: ComponentId,
    /// `None` for the paths with no subscriber — `rollback`, `select`, `reset-to-golden`, and the
    /// sideload CLI. They still get a transcript; what they do not get is a live stream, which is
    /// unchanged from before this existed.
    progress: Option<ProgressTx>,
    /// `None` when the transcript could not be opened. An update whose diary cannot be written
    /// still runs — see `crate::transcript`.
    transcript: Option<Transcript>,
}

impl Recorder {
    fn begin(
        component: &str,
        progress: Option<ProgressTx>,
        state_dir: &Path,
        opening: RunEvent,
    ) -> Self {
        let transcript = match Transcript::begin(state_dir, TRANSCRIPTS_KEPT) {
            Ok(transcript) => Some(transcript),
            Err(e) => {
                tracing::warn!(error = %e, "could not open a transcript for this run; it will not be in `update show`");
                None
            }
        };
        let rec = Self {
            component: ComponentId::new(component),
            progress,
            transcript,
        };
        rec.note(opening);
        rec
    }

    /// The run this is recording, for the log entry that points at it.
    fn run(&self) -> Option<u64> {
        self.transcript.as_ref().map(Transcript::id)
    }

    /// A phase boundary: to the subscriber and to the transcript both.
    fn phase(&self, phase: Phase, detail: Option<String>) {
        if let Some(progress) = &self.progress {
            let _ = progress.send(Progress {
                component: self.component.clone(),
                phase,
                percent: None,
                detail: detail.clone(),
            });
        }
        self.note(RunEvent::Phase { phase, detail });
    }

    /// Transcript only — too big for a notification, or too dull for one.
    fn note(&self, event: RunEvent) {
        if let Some(transcript) = &self.transcript {
            transcript.record(event);
        }
    }

    /// One sentence, where there is no shape worth inventing.
    fn say(&self, text: impl Into<String>) {
        self.note(RunEvent::Note { text: text.into() });
    }

    /// The channel the download pump writes percentages to.
    ///
    /// Deliberately not routed through [`Self::phase`]: a download reports hundreds of times and
    /// the transcript wants the phase, not the percentages. What it records instead is how big the
    /// artifact turned out to be, once.
    fn progress_tx(&self) -> Option<ProgressTx> {
        self.progress.clone()
    }

    /// A recorder that records nowhere, for tests about something else.
    #[cfg(test)]
    fn silent() -> Self {
        Self {
            component: ComponentId::new("test"),
            progress: None,
            transcript: None,
        }
    }

    /// Close the run with the outcome the update log is about to record.
    fn finish(&self, outcome: &Result<ApplyResult, Error>) {
        self.note(RunEvent::Ended {
            outcome: journal_outcome(outcome).map(|(_, outcome)| outcome),
            summary: summarise(outcome),
        });
    }
}

/// How the health gate passed.
///
/// **Two outcomes, not one.** `HealthCheck::Socket` commits a release onto a robot reporting
/// *degraded* whenever the degradation is something the release cannot have caused and a rollback
/// cannot fix — servo power off, a sensor unplugged. That is the right call and it has been logged
/// at `warn` since it was written, precisely so nobody has to guess afterwards that it is what
/// happened. Collapsing it into `Ok(())` meant the transcript then said "the robot reported
/// healthy" on a board whose own journal, at the same second, said it was degraded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GatePassed {
    Healthy,
    /// Committed anyway, with the reason the robot gave.
    Degraded(String),
}

#[derive(Debug, Clone, Default)]
pub struct ApplyOptions {
    pub dry_run: bool,
    pub interrupt_sessions: bool,
    /// Read the release from this directory instead of the component's configured source.
    ///
    /// The laptop-to-board path (`scripts/dev-push.sh`): build, sign with the dev key, copy
    /// the directory over, apply. Served by [`Engine::apply`] and not by
    /// `updaterd install --from`, which is the reason it exists — `install` has to force
    /// `on_apply` and the health gate off, so it can only be used on a board with no live
    /// release, and every use of it on a working robot silently gives up auto-rollback.
    /// A dev board is exactly where a release most needs to be gated and rolled back.
    ///
    /// It changes where the bytes come from and nothing else: same `LocalDir` source as the
    /// tests and the offline installer, so signature, hash and compatibility are checked
    /// identically. A locally built release installs because the dev key is trusted on that
    /// board, not because a check was skipped.
    pub from_dir: Option<std::path::PathBuf>,

    /// Who asked, as `uid=1000 gid=1000 pid=2317`, for the transcript's opening line.
    ///
    /// `None` on the paths with no peer — the unattended timer, and the sideload CLI. A string
    /// because nothing branches on it: it exists so that "who ran this?" is answerable months
    /// later, and `SO_PEERCRED`'s three numbers are what there is to answer it with.
    pub requested_by: Option<String>,
}

impl Engine {
    pub fn new(
        config: Config,
        keys: KeyRing,
        robot: Box<dyn RobotClient>,
        faults: Faults,
    ) -> Result<Self, Error> {
        let journal = Journal::open(&config.state_dir, LOG_CAPACITY)?;
        let boot_counter = BootCounter::open(&config.state_dir);
        let pins = Pins::open(&config.state_dir);
        Ok(Self {
            config,
            keys: std::sync::Arc::new(keys),
            robot,
            journal,
            boot_counter,
            pins,
            faults,
            deferred_restarts: true,
            unit_dir: PathBuf::from(UNIT_DIR),
        })
    }

    /// Look for orphaned units somewhere other than `/etc/systemd/system`.
    ///
    /// For tests, which cannot write to the real one — and must not, since the check reads whatever
    /// the machine running them happens to have installed.
    #[doc(hidden)]
    pub fn with_unit_dir(mut self, dir: PathBuf) -> Self {
        self.unit_dir = dir;
        self
    }

    /// Stop this engine spawning anything when a release is applied.
    ///
    /// **For test binaries, and what it removes is a `fork`, not a behaviour.** `flock` belongs to an
    /// open file description, so a forked child inherits a *copy* of every lock the process holds and
    /// keeps it alive until it execs. `apply` already drops its own update lock before spawning for
    /// exactly this reason — but in a binary running engines in parallel, the child also inherits
    /// copies of every *other* engine's lock, and those it cannot drop.
    ///
    /// So one test scheduling restarts made a concurrent test's `try_acquire` answer `Busy`, and the
    /// test that noticed it was whichever one happened to be in a lock at that moment:
    ///
    /// ```text
    /// ref "sub/dir" must be refused, got Busy
    /// ```
    ///
    /// It needs a runner with systemd for the spawn to take long enough to matter, so it failed in CI
    /// and never on a laptop — the worst shape a flake can have. Suppressing the spawn removes the
    /// race rather than widening a window, and the restart path itself stays covered by
    /// `restart_tests`, which drives it against a stub `systemctl` and asserts what it was asked.
    #[doc(hidden)]
    pub fn without_deferred_restarts(mut self) -> Self {
        self.deferred_restarts = false;
        self
    }

    // ── queries ──────────────────────────────────────────────────────────────

    /// Is an update available? Changes nothing.
    pub async fn check(&self, component: &str) -> Result<CheckResult, Error> {
        let cfg = self.config.component(component)?;
        let store = self.store(component)?;
        let installed = store.current()?;

        let signed = source::from_config(&cfg.source).latest_manifest().await?;
        self.verify_manifest(&signed)?;
        let manifest = signed.parsed;
        Self::check_channel(&manifest, component)?;

        if Some(&manifest.version) == installed.as_ref() {
            return Ok(CheckResult::UpToDate {
                installed: manifest.version,
            });
        }

        if let Some(pinned) = self.effective_pin(component)
            && pinned != manifest.version
        {
            return Ok(CheckResult::Incompatible {
                candidate: manifest.version,
                reason: format!("component is pinned to {pinned}"),
            });
        }

        // Same rollback-attack guard as `apply`: report it rather than offer it.
        if let Some(current) = &installed
            && manifest.version < *current
        {
            return Ok(CheckResult::Incompatible {
                candidate: manifest.version.clone(),
                reason: format!(
                    "source advertises {} but {current} is installed; refusing to \
                         downgrade (stale mirror, or a withdrawn release?)",
                    manifest.version
                ),
            });
        }

        match manifest.compatibility(&self.capabilities().await) {
            Compatibility::Refused(reason) => Ok(CheckResult::Incompatible {
                candidate: manifest.version,
                reason,
            }),
            // Unknown is not a refusal: see `manifest::Compatibility`.
            Compatibility::Ok | Compatibility::Unknown(_) => Ok(CheckResult::Available {
                mandatory: manifest.is_mandatory_for(installed.as_ref()),
                installed,
                candidate: manifest.version,
                changelog: manifest.changelog,
            }),
        }
    }

    pub async fn status(&self) -> Result<Vec<ComponentStatus>, Error> {
        let mut out = Vec::new();
        for (name, cfg) in &self.config.components {
            let store = Store::new(cfg.install_dir.clone());
            let healthy = match cfg.health {
                HealthCheck::None => None,
                // Only a socket probe means "ask robotd". A command probe is a
                // different question entirely; reporting robotd's health for it would
                // be plainly wrong, so run the probe we were configured with.
                HealthCheck::Socket { .. } => {
                    Some(self.robot.health(ROBOT_QUERY_TIMEOUT).await.is_healthy())
                }
                HealthCheck::Command { .. } => Some(self.health_gate(cfg).await.is_ok()),
            };
            out.push(ComponentStatus {
                component: ComponentId::new(name.clone()),
                installed: store.current()?,
                // Always Idle: `status` is served by a fresh borrow of the engine,
                // while an in-flight update holds it. A caller wanting live phase
                // should subscribe to progress notifications instead.
                phase: Phase::Idle,
                healthy,
                pinned: self.effective_pin(name),
                last_attempt: self.journal.last_for(name)?,
            });
        }
        Ok(out)
    }

    pub fn list_installed(&self, component: &str) -> Result<Vec<InstalledRelease>, Error> {
        let cfg = self.config.component(component)?;
        let store = self.store(component)?;
        let active = store.current()?;

        Ok(store
            .list()?
            .into_iter()
            .map(|version| InstalledRelease {
                active: Some(&version) == active.as_ref(),
                golden: Some(&version) == cfg.golden.as_ref(),
                source_revision: Self::embedded_manifest(&store, &version)
                    .and_then(|m| m.source_revision),
                version,
            })
            .collect())
    }

    /// Components the scheduler should poll.
    pub fn component_names(&self) -> Vec<String> {
        self.config.components.keys().cloned().collect()
    }

    /// Replace the robot client. Tests only.
    ///
    /// Exists so a test can give the engine a robot whose health changes between updates
    /// without rebuilding the whole engine (and losing its journal, which is the state the
    /// interesting cases depend on).
    #[doc(hidden)]
    pub fn replace_robot_for_test(&mut self, robot: Box<dyn RobotClient>) {
        self.robot = robot;
    }

    pub fn log(&self, limit: usize) -> Result<Vec<LogEntry>, Error> {
        self.journal.recent(limit)
    }

    /// One run's transcript, or the most recent when none is named.
    ///
    /// Defaulting to the latest is the whole ergonomics of the command: the question is almost
    /// always about the update that just happened, and making someone look its number up first
    /// would put a step between them and the answer at the exact moment they are least patient.
    pub fn show(&self, run: Option<u64>) -> Result<crate::proto::RunTranscript, Error> {
        let mut available = Transcript::ids(&self.config.state_dir);
        let id = match run {
            Some(id) => id,
            None => match available.last() {
                Some(id) => *id,
                None => {
                    return Err(Error::NoSuchRun {
                        run: None,
                        available: Vec::new(),
                        // What the log holds with nothing behind it, so the message can tell
                        // "nothing has happened" apart from "this is the first release that
                        // records what happened".
                        earlier: self.log(LOG_CAPACITY).map(|log| log.len()).unwrap_or(0),
                    });
                }
            },
        };

        let events = Transcript::read(&self.config.state_dir, id)?;
        available.reverse();
        Ok(crate::proto::RunTranscript {
            run: id,
            component: Transcript::component_of(&events),
            events,
            available,
        })
    }

    /// Versions whose most recent recorded outcome was a rollback.
    ///
    /// Derived from the journal rather than stored, so it self-heals: a version that
    /// failed once and later succeeded drops off the list. Used to keep `rollback` off a
    /// release that already failed, to keep the boot counter from reverting onto one, and
    /// to stop the unattended path retrying one forever ([`crate::ipc`]).
    ///
    /// An unreadable journal yields an empty list. That is the deliberate direction to
    /// fail: treating every version as bad because the log could not be read would block
    /// updates on a robot whose state directory is damaged — exactly when updating is the
    /// repair.
    pub fn known_bad(&self, component: &str) -> Vec<semver::Version> {
        self.journal.known_bad(component).unwrap_or_default()
    }

    // ── the main path ────────────────────────────────────────────────────────

    /// Install `target`, gate it, and roll back if it doesn't come up healthy.
    ///
    /// Emits [`Progress`] on each phase transition.
    ///
    /// **Cancellation:** dropping this future before the swap leaves only staging
    /// garbage, which the next startup cleans. Dropping it *after* the swap leaves the
    /// new release live, armed, and ungated — a half-applied release whose recovery is
    /// deferred to the boot counter on the next `updaterd` start. That is deliberate
    /// (the alternative, rolling back from a cancelled task, is less predictable) but
    /// it is not "never half-applied".
    pub async fn apply(
        &mut self,
        component: &str,
        target: crate::proto::Target,
        options: ApplyOptions,
        progress: ProgressTx,
    ) -> Result<ApplyResult, Error> {
        // Single-flight. Busy is a normal answer, not a failure.
        let lock = UpdateLock::try_acquire(&self.config.state_dir)?.ok_or(Error::Busy)?;

        let cfg = self.config.component(component)?.clone();
        let store = self.store(component)?;
        let installed = store.current()?;

        let rec = Recorder::begin(
            component,
            Some(progress),
            &self.config.state_dir,
            RunEvent::Began {
                component: ComponentId::new(component),
                target: describe_target(&target),
                installed: installed.clone(),
                source: describe_source(&cfg.source, options.from_dir.as_deref()),
                requested_by: options.requested_by.clone(),
            },
        );

        let outcome = self
            .apply_inner(component, &cfg, &store, target, &options, &rec)
            .await;

        // Every operation logs through `record`, which owns what `to` means per outcome.
        self.record(component, installed, &outcome, rec.run());

        // The lock is released before anything is spawned, and that ordering is load-bearing: a
        // fork duplicates every open descriptor in the process, so spawning while holding the
        // update lock hands a copy of it to the child — and in a test binary running engines in
        // parallel, copies of *other* engines' locks too. It surfaced as unrelated operations
        // failing with `Busy`. Nothing below this point touches the store.
        drop(lock);
        schedule_restarts_if_needed(self.deferred_restarts, &outcome, &rec).await;

        // Closed *after* the restarts, not after the outcome, so the transcript ends where the run
        // does. The deferred units are the last thing an apply causes, and a reader who saw
        // `ended` above them would reasonably conclude they belonged to something else.
        rec.finish(&outcome);
        outcome
    }

    #[allow(clippy::too_many_arguments)]
    async fn apply_inner(
        &self,
        component: &str,
        cfg: &ComponentConfig,
        store: &Store,
        target: crate::proto::Target,
        options: &ApplyOptions,
        rec: &Recorder,
    ) -> Result<ApplyResult, Error> {
        let installed = store.current()?;
        // A per-call source override, so the configured one stays in place: a dev board keeps
        // reaching GitHub for `--ref <branch>`, `--staging` and a return to the release stream,
        // and a laptop build is one flag rather than a config edit to undo afterwards.
        let source = match &options.from_dir {
            Some(dir) => Box::new(source::LocalDir::new(dir.clone())) as Box<dyn source::Source>,
            None => source::from_config(&cfg.source),
        };

        // 0. Environment preflight, *before* touching the network. The manifest
        //    fetch is HTTPS, and on a board with no battery-backed RTC it fails
        //    certificate-date validation with an opaque TLS error — the clock check
        //    exists precisely to diagnose that, so it has to run first.
        rec.phase(Phase::Preflight, None);
        self.preflight(None, options, store).await?;

        // 1. Manifest, and its signature. Nothing else happens until this passes.
        rec.phase(Phase::Checking, None);
        let signed = match &target {
            crate::proto::Target::Latest => source.latest_manifest().await?,
            crate::proto::Target::Exact(v) => source.manifest_for(v).await?,
            crate::proto::Target::Ref(git_ref) => source.manifest_at_ref(git_ref).await?,
            crate::proto::Target::Staging => source.staging_manifest().await?,
            crate::proto::Target::StagingExact(v) => source.staging_manifest_for(v).await?,
        };
        let signed_by = self.verify_manifest(&signed)?;
        let manifest = signed.parsed.clone();

        // Recorded here rather than at the end, and before any of the refusals below: a run that
        // was *refused* is one of the two runs anyone reads, and "which release, from where,
        // signed by which key" is what the refusal has to be read against.
        rec.note(RunEvent::Manifest {
            version: manifest.version.clone(),
            sha256: manifest.sha256.clone(),
            bytes: manifest.size,
            url: Some(manifest.url.clone()),
            signed_by: Some(signed_by),
            source_revision: manifest.source_revision.clone(),
        });

        Self::check_channel(&manifest, component)?;

        if let Some(pinned) = self.effective_pin(component)
            && pinned != manifest.version
        {
            return Err(Error::Incompatible(format!(
                "component is pinned to {pinned}, refusing {}",
                manifest.version
            )));
        }

        if Some(&manifest.version) == installed.as_ref() {
            // Correct, and for years the whole answer. It is the wrong *question* in one case: the
            // release is installed and a daemon is serving from a different one. That is what an
            // operator reaching for `apply` is usually trying to fix, and answering "already current"
            // told them there was nothing to fix. The units are named here and restarted after the
            // reply — see `restarts_owed`.
            let stale = self.stale_units(&manifest.version, cfg, store);
            return Ok(ApplyResult::AlreadyCurrent {
                version: manifest.version,
                stale,
            });
        }

        // Rollback-attack guard. A signature proves an artifact is *ours*; it says
        // nothing about it being *current*. A stale or reverted mirror can serve an
        // old, still-validly-signed manifest, which would silently walk the fleet
        // backwards onto a version we withdrew — the classic downgrade attack on a
        // signed-artifact scheme.
        //
        // Only `Latest` is guarded. `Exact` is a deliberate operator action (that is how a
        // targeted revert works), and `Ref` *always* looks like a downgrade — a dev build is
        // a semver prerelease, so it sorts below the release it precedes — so guarding it
        // would reject every branch install. Rollback and reset-to-golden move backwards on
        // purpose without passing through here.
        //
        // `StagingExact` is unguarded for the `Exact` reason: naming a candidate is how someone
        // reinstalls the one a board just rolled back from, which is exactly the move they reach
        // for while investigating that rollback. Bare `Staging` *is* guarded, but by
        // [`staging_has_nothing_newer`] below rather than by this — the refusals are not the same
        // claim, and this one's message would be actively misleading there.
        //
        // `from_dir` is exempt for the `Exact` reason. The guard defends against a *mirror*
        // that has gone backwards, and a directory named on the command line by somebody with
        // root is not a mirror — it is the same explicit statement of intent as naming a
        // version. It also would not leave a usable flag: a locally built release is a
        // prerelease of the version it precedes, so it sorts below whatever the board is on
        // and every single laptop push would be refused as a downgrade.
        if matches!(target, crate::proto::Target::Latest)
            && options.from_dir.is_none()
            && let Some(installed) = &installed
            && manifest.version < *installed
        {
            return Err(Error::WouldDowngrade {
                installed: installed.clone(),
                candidate: manifest.version,
            });
        }

        // `--staging` on a board that is ahead of the candidate channel. Reachable the moment a
        // release is promoted straight to stable — which is a supported path, `release.yml` calls
        // it "build → STABLE directly (no staging release exists; NOT canaried)" — because the
        // staging scan then keeps answering with the last version that did publish a candidate.
        //
        // Refused rather than installed. Every layer below here behaved correctly when it was
        // not: the artifact verified, the swap happened, a unit that the older release does not
        // contain failed to start, and the update reverted. But the operator asked for "the one
        // being tested" and there is no such thing, so the answer is a sentence rather than a
        // rollback — and `orphan` cannot be that sentence, since it only sees a stale unit, not a
        // stale channel.
        if let Some(installed) =
            staging_has_nothing_newer(&target, installed.as_ref(), &manifest.version)
        {
            return Err(Error::StagingBehind {
                component: component.to_owned(),
                installed: installed.clone(),
                candidate: manifest.version,
            });
        }

        match manifest.compatibility(&self.capabilities().await) {
            Compatibility::Ok => {}
            Compatibility::Refused(reason) => return Err(Error::Incompatible(reason)),
            // For the daemon channel this is fine to proceed through — that update
            // is how a dead robotd gets fixed. A model manifest declaring a
            // model_api is the case that reaches here, and waiting is correct.
            Compatibility::Unknown(reason) => {
                if manifest.model_api.is_some() {
                    return Err(Error::Incompatible(format!(
                        "cannot confirm compatibility: {reason}"
                    )));
                }
            }
        }

        // 2. Space preflight. Deferred to here because the requirement comes from
        //    the manifest, which we now have and have verified.
        rec.phase(Phase::Preflight, None);
        self.preflight(Some(&manifest), options, store).await?;

        // 3. Download into staging. Staging lives beside the release tree so the
        //    later rename stays on one filesystem.
        let staging = store.staging_dir(&manifest.version);
        let _ = std::fs::remove_dir_all(&staging);
        let download_dir = staging.join("dl");
        let extract_dir = staging.join("root");

        let result = self
            .stage_and_swap(
                component,
                cfg,
                store,
                &manifest,
                &signed.bytes,
                &*source,
                &staging,
                &download_dir,
                &extract_dir,
                options,
                rec,
            )
            .await;

        // Staging is always disposable; leaving it behind only wastes disk. Removing
        // an extracted tree is many syscalls, so it too goes off the async worker.
        let doomed = staging.clone();
        let _ = tokio::task::spawn_blocking(move || std::fs::remove_dir_all(doomed)).await;
        result
    }

    #[allow(clippy::too_many_arguments)]
    async fn stage_and_swap(
        &self,
        component: &str,
        cfg: &ComponentConfig,
        store: &Store,
        manifest: &Manifest,
        manifest_bytes: &[u8],
        source: &dyn source::Source,
        _staging: &Path,
        download_dir: &Path,
        extract_dir: &Path,
        options: &ApplyOptions,
        rec: &Recorder,
    ) -> Result<ApplyResult, Error> {
        let previous = store.current()?;

        rec.phase(Phase::Downloading, None);
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<(u64, Option<u64>)>();
        // The source takes a clone; this handle exists so the channel closes exactly
        // when we say so, letting the pump drain rather than be aborted.
        let tx_keepalive = tx.clone();
        let pump = {
            let progress = rec.progress_tx();
            let component = component.to_owned();
            tokio::spawn(async move {
                let send = |percent| {
                    let Some(progress) = &progress else {
                        return;
                    };
                    let _ = progress.send(Progress {
                        component: ComponentId::new(component.clone()),
                        phase: Phase::Downloading,
                        percent,
                        detail: None,
                    });
                };

                // Coalesced rather than forwarded one-for-one: the source speaks once per HTTP
                // chunk and nobody watching needs that. See `DownloadProgress`.
                let mut gate = DownloadProgress::started(tokio::time::Instant::now());
                while let Some((done, total)) = rx.recv().await {
                    let percent = total
                        .filter(|t| *t > 0)
                        .map(|t| ((done.min(t) * 100) / t) as u8);
                    if gate.admit(percent, tokio::time::Instant::now()) {
                        send(percent);
                    }
                }
                if let Some(percent) = gate.flush() {
                    send(Some(percent));
                }
            })
        };
        let fetched = source.fetch_artifact(manifest, download_dir, tx).await?;
        // Drop the sender so the pump sees end-of-stream and forwards everything it
        // has; `abort()` here would discard the last few updates, so a download could
        // visibly stall at 97%.
        drop(tx_keepalive);
        let _ = pump.await;

        if self.faults.corrupt_artifact {
            // Append a byte so the hash no longer matches — the same observable
            // condition as a truncated download or a tampered mirror.
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new()
                .append(true)
                .open(&fetched.artifact)
                .map_err(|e| Error::Io {
                    path: fetched.artifact.clone(),
                    source: e,
                })?;
            let _ = f.write_all(b"x");
        }

        // 4. Integrity, then authenticity. Both before anything is extracted.
        //
        // Hashing and signature verification stream hundreds of megabytes and take
        // seconds on this class of board. Run on the async worker they would stall
        // the IPC tasks that are meant to keep answering `status`/`subscribe` while
        // the update runs, so both go to `spawn_blocking`.
        // The size the download actually turned out to be, which a manifest need not have
        // declared and a resumed transfer makes worth stating outright.
        if let Ok(meta) = std::fs::metadata(&fetched.artifact) {
            rec.say(format!(
                "downloaded {} to {}",
                describe_bytes(meta.len()),
                fetched.artifact.display()
            ));
        }

        rec.phase(Phase::Verifying, None);
        let artifact = fetched.artifact.clone();
        let expected = manifest.sha256.clone();
        blocking(move || verify::verify_sha256(&artifact, &expected)).await?;

        let signature = std::fs::read(&fetched.signature).map_err(|e| Error::Io {
            path: fetched.signature.clone(),
            source: e,
        })?;
        let keys = std::sync::Arc::clone(&self.keys);
        let artifact = fetched.artifact.clone();
        let signed_by = blocking(move || {
            keys.verify_file(&artifact, &signature)
                .map(|key| key.id.clone())
        })
        .await?;
        rec.say(format!(
            "hash matches; signature verifies against {signed_by}"
        ));

        // 5. Extract to the side, never over a live path. Also CPU-bound (zstd).
        rec.phase(Phase::Extracting, None);
        let artifact = fetched.artifact.clone();
        let dest = extract_dir.to_path_buf();
        let limits = self.config.archive_limits();
        blocking(move || verify::extract_artifact(&artifact, &dest, limits)).await?;

        // Keep the verified manifest with the release, for `select` and provenance.
        std::fs::write(extract_dir.join(EMBEDDED_MANIFEST), manifest_bytes).map_err(|e| {
            Error::Io {
                path: extract_dir.join(EMBEDDED_MANIFEST),
                source: e,
            }
        })?;

        // 5b. Would this release leave an installed unit with nothing to exec? See
        //     [`crate::orphan`] — a downgrade past the release that introduced a daemon leaves
        //     that daemon's unit behind, and it then fails with `203/EXEC`, which fails the
        //     restart, which reverts the update.
        //
        //     Here rather than in `preflight` because the candidate's file list does not exist
        //     until now, and before the dry run returns because "will this downgrade work?" is
        //     exactly what a dry run is asked. Nothing has moved yet: staging is disposable, the
        //     boot counter is unarmed, `current` still points where it did.
        //
        //     No target is exempt. `WouldDowngrade` above guards only `Latest`, because it is
        //     about a mirror serving a stale manifest; this is about a unit that will not start,
        //     which does not care how the target was named — and `Ref` is how the case was
        //     observed.
        let orphans = crate::orphan::would_orphan(&self.unit_dir, &store.link_path(), extract_dir);
        if !orphans.is_empty() {
            return Err(Error::WouldOrphanUnit(crate::orphan::refusal(
                &manifest.version,
                &orphans,
            )));
        }

        if options.dry_run {
            return Ok(ApplyResult::DryRunPassed {
                candidate: manifest.version.clone(),
            });
        }

        // 6. Pre-install hook, before the release becomes live.
        rec.phase(Phase::RunningPreHook, None);
        let ctx = hooks::HookContext {
            component: component.to_owned(),
            old_version: previous.clone(),
            new_version: manifest.version.clone(),
            install_dir: cfg.install_dir.clone(),
            release_dir: store.release_dir(&manifest.version),
            old_schema_version: previous
                .as_ref()
                .and_then(|v| Self::embedded_manifest(store, v))
                .map(|m| m.schema_version),
            new_schema_version: manifest.schema_version,
        };
        let hook = hooks::run(
            extract_dir,
            hooks::HookKind::PreInstall,
            &ctx,
            PRE_INSTALL_HOOK_TIMEOUT,
        )
        .await;
        record_hook(rec, hooks::HookKind::PreInstall.relative_path(), &hook);
        hook?;

        // 7. Publish the release directory with one rename, then arm the boot
        //    counter *before* the symlink swap so a crash in between is
        //    recoverable.
        let release_dir = store.release_dir(&manifest.version);
        let _ = std::fs::remove_dir_all(&release_dir);
        if let Some(parent) = release_dir.parent() {
            std::fs::create_dir_all(parent).map_err(|e| Error::Io {
                path: parent.to_path_buf(),
                source: e,
            })?;
        }
        std::fs::rename(extract_dir, &release_dir).map_err(|e| Error::Io {
            path: release_dir.clone(),
            source: e,
        })?;

        self.boot_counter.arm(&PendingUpdate {
            component: component.to_owned(),
            version: manifest.version.clone(),
            previous: previous.clone(),
            boots: 0,
        })?;

        rec.phase(
            Phase::Swapping,
            Some(format!(
                "{} → {}",
                previous
                    .as_ref()
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "nothing".into()),
                manifest.version
            )),
        );
        store.swap_to(&manifest.version)?;

        if self.faults.abort_after_swap {
            // Simulates `kill -9` immediately after the swap: the symlink points at
            // the new release and the boot counter is still armed. Recovery is
            // `recover_on_start`'s job, which a test then exercises.
            return Err(Error::Internal("simulated abort after swap".into()));
        }

        // 8. Everything from here rolls back on failure.
        let gate = self
            .post_swap(component, cfg, store, &ctx, &release_dir, rec)
            .await;

        match gate {
            Ok(()) => {
                rec.phase(Phase::Committing, None);
                self.boot_counter.confirm(component)?;
                // Pruning is best-effort — the update has already succeeded — but a
                // failure must be visible, or a robot slowly filling its eMMC looks
                // perfectly healthy.
                match store.prune(cfg.keep_previous, cfg.golden.as_ref()) {
                    Ok(removed) if !removed.is_empty() => {
                        tracing::info!(?removed, "pruned old releases");
                        rec.say(format!(
                            "pruned {}",
                            removed
                                .iter()
                                .map(|v| v.to_string())
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                    }
                    Ok(_) => {}
                    Err(e) => tracing::error!(error = %e, "could not prune old releases"),
                }
                Ok(ApplyResult::Applied {
                    from: previous,
                    to: manifest.version.clone(),
                })
            }
            Err(reason) => {
                rec.phase(Phase::RollingBack, Some(reason.to_string()));
                match self
                    .rollback_to(component, cfg, store, previous.as_ref(), rec)
                    .await?
                {
                    Some(reverted) => Ok(ApplyResult::RolledBack {
                        attempted: manifest.version.clone(),
                        reason: reverted.describe(&reason.to_string()),
                        reverted_to: Some(reverted.version),
                    }),
                    // Nothing was reverted, so saying "rolled back" would be a lie.
                    None => Ok(ApplyResult::Stuck {
                        version: manifest.version.clone(),
                        reason: format!(
                            "{reason}; no previous release and no golden configured, so there \
                             was nothing to revert to"
                        ),
                    }),
                }
            }
        }
    }

    /// Post-install hook, apply action, health gate — the three things that can
    /// fail after the swap and therefore trigger a rollback.
    async fn post_swap(
        &self,
        _component: &str,
        cfg: &ComponentConfig,
        _store: &Store,
        ctx: &hooks::HookContext,
        release_dir: &Path,
        rec: &Recorder,
    ) -> Result<(), Error> {
        rec.phase(Phase::RunningPostHook, None);
        if self.faults.fail_post_hook {
            return Err(Error::Hook {
                hook: hooks::POST_INSTALL.into(),
                detail: "injected failure".into(),
            });
        }
        let hook = hooks::run(release_dir, hooks::HookKind::PostInstall, ctx, HOOK_TIMEOUT).await;
        record_hook(rec, hooks::HookKind::PostInstall.relative_path(), &hook);
        hook?;

        rec.phase(Phase::Applying, None);
        self.run_apply_action(&cfg.on_apply, release_dir, rec)
            .await?;

        // Only where daemons are actually being replaced. `ApplyAction::None` means this component
        // has none to restart — a model, or the bootstrap install, which forces it precisely
        // because nothing is installed yet and there is no running daemon to make stale.
        if matches!(cfg.on_apply, ApplyAction::Restart { .. }) {
            self_test_updaterd(release_dir, self.config.loaded_from.as_deref()).await?;
            rec.say("the new updaterd passed its self-test");
        }

        rec.phase(Phase::HealthGate, None);
        let verdict = self.health_gate(cfg).await;
        record_gate(rec, &verdict);
        verdict.map(|_| ())
    }

    /// Swap back to `previous` and re-run the apply action.
    ///
    /// A failure here is [`Error::RollbackFailed`] — the most serious outcome, kept
    /// distinct so support sees it immediately rather than reading it as an
    /// ordinary failure.
    /// Highest installed release strictly *older* than `current`, skipping any whose
    /// most recent recorded outcome was a rollback.
    ///
    /// Two constraints, both learned the hard way:
    ///
    ///  - **Strictly older.** A plain "newest that isn't current" walks *forward*
    ///    after an auto-rollback, because the release that just failed is still on
    ///    disk (the failure path deliberately doesn't prune). That would make
    ///    `rollback` — the one command a support engineer reaches for after a bad
    ///    update — reinstall the bad update.
    ///  - **Not known-bad.** A release the journal recorded as rolled back is not a
    ///    safe landing spot, even if it is the newest older one.
    fn rollback_target(
        &self,
        component: &str,
        store: &Store,
        current: Option<&semver::Version>,
    ) -> Result<semver::Version, Error> {
        let installed = store.list()?;
        let known_bad = self.journal.known_bad(component)?;

        let mut candidates: Vec<_> = installed
            .into_iter()
            .filter(|v| match current {
                Some(current) => v < current,
                // Nothing linked: any installed release is a step forward.
                None => true,
            })
            .filter(|v| !known_bad.contains(v))
            .collect();
        candidates.sort();

        candidates.pop().ok_or_else(|| {
            Error::Corrupt(format!(
                "no older, known-good release installed to roll back to (current: {})",
                current
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "none".into())
            ))
        })
    }

    /// What a revert achieved: the release now live, and whatever went wrong after it was.
    ///
    /// Two facts rather than one, because they have different urgencies and used to be collapsed into
    /// a single `RollbackFailed`. The version is the recovery; `apply_error` is a unit that would not
    /// restart on command, on a robot which is otherwise back where it was — not, despite what this
    /// once said, a daemon that is down. [`Reverted::describe`] has the difference and why it bit.
    async fn rollback_to(
        &self,
        component: &str,
        cfg: &ComponentConfig,
        store: &Store,
        previous: Option<&semver::Version>,
        rec: &Recorder,
    ) -> Result<Option<Reverted>, Error> {
        if self.faults.fail_rollback {
            return Err(Error::RollbackFailed("injected rollback failure".into()));
        }

        let Some(previous) = previous else {
            // Nothing to go back to: a first install that failed its gate, with no
            // golden configured. The bad release stays linked.
            //
            // The trial is cleared anyway. Leaving it armed would make every
            // subsequent boot "recover" the same unrecoverable update, appending a
            // bogus rollback entry each time and never converging. The caller
            // reports `Stuck`, which says truthfully that nothing was reverted.
            self.boot_counter
                .confirm(component)
                .map_err(|e| Error::RollbackFailed(e.to_string()))?;
            return Ok(None);
        };

        // The swap and the trial are the recovery: past here the robot *is* back on the release it
        // came from, and only these two failing mean it could not be put back.
        store
            .swap_to(previous)
            .map_err(|e| Error::RollbackFailed(e.to_string()))?;
        self.boot_counter
            .confirm(component)
            .map_err(|e| Error::RollbackFailed(e.to_string()))?;

        // The apply action is not, and this used to be `RollbackFailed` too.
        //
        // It is reachable by an ordinary bad release: `hooks/postinstall` overwrites unit files and
        // deliberately does not put them back, so a release that fails *because* one of its units
        // cannot start leaves that file behind — and if the release being reverted to ships a unit of
        // the same name, this restart inherits the same failure. `scripts/systemd-test.sh` reproduces
        // exactly that.
        //
        // Reporting it as a failed rollback asserted the one thing that was not true — the swap had
        // already happened — and buried the one thing that was: a daemon is down. So it is carried
        // into the outcome instead, which says both. `RollbackFailed` keeps its meaning: the robot
        // could not be put back at all.
        let apply_error = if self.faults.fail_rollback_apply {
            Some("injected apply-action failure during rollback".to_owned())
        } else {
            match self
                .run_apply_action(&cfg.on_apply, &store.release_dir(previous), rec)
                .await
            {
                Ok(()) => None,
                Err(e) => Some(e.to_string()),
            }
        };
        if let Some(detail) = &apply_error {
            // At warn rather than error, and the wording changed with it. This is a restart that
            // did not take on command, which `Restart=always` may already have undone; asserting an
            // outage here is what made the reverted-board report contradict the health line next to
            // it. See [`Reverted::describe`].
            tracing::warn!(
                component,
                reverted_to = %previous,
                error = %detail,
                "reverted, but a unit refused to restart on command — check whether it came back"
            );
        }

        Ok(Some(Reverted {
            version: previous.clone(),
            apply_error,
        }))
    }

    // ── explicit transitions ─────────────────────────────────────────────────

    /// Install a policy set from the Hub and tell `robotd` to pick it up.
    ///
    /// Not an update in the component sense — no manifest, no signature, no health gate, no
    /// rollback — and [`crate::policy`] says why. It is here because it needs this process's two
    /// privileges, a network stack and root, and because `robotd` must be told afterwards: the
    /// swap moves a symlink underneath unchanged paths, so nothing about the slots looks
    /// different from the loop's side until something says otherwise.
    ///
    /// A robot that does not pick it up is reported rather than treated as a failure. The files
    /// are installed and correct; the running loop is one restart behind, which is a true thing
    /// to say and not a reason to undo a download.
    pub async fn install_policies(
        &self,
        version: Option<&str>,
    ) -> Result<crate::proto::PolicyInstallResult, Error> {
        let root = std::path::Path::new(crate::policy::POLICY_ROOT);
        let (installed, previous) = crate::policy::install(root, version).await?;
        let reloaded = match &previous {
            // Nothing moved, so there is nothing for the robot to re-read and asking would only
            // make it go home and rebuild for no reason.
            None => true,
            Some(_) => self.robot.reload_policies(ROBOT_QUERY_TIMEOUT).await,
        };
        Ok(crate::proto::PolicyInstallResult {
            installed,
            previous,
            reloaded,
        })
    }

    /// Fetch one policy from any Hub repo into this robot's library.
    ///
    /// The model API comes from the running `robotd` rather than from a constant here, because it
    /// is the daemon that implements the contract and this process only carries it. An
    /// unreachable robot is not a refusal — see [`crate::policy::Expectations::here`].
    pub async fn fetch_policy(
        &self,
        repo: &str,
        revision: Option<&str>,
        file: Option<&str>,
    ) -> Result<crate::proto::PolicyFetchResult, Error> {
        let model_api = self.robot.model_api(ROBOT_QUERY_TIMEOUT).await;
        // What the robot is pointed at, so the prune that follows the fetch cannot take a gait
        // out from under it. `None` — a robot that did not answer — prunes nothing at all.
        let in_use = self.robot.policy_paths(ROBOT_QUERY_TIMEOUT).await;
        crate::policy::fetch(
            std::path::Path::new(crate::policy::LIBRARY_ROOT),
            repo,
            revision,
            file,
            crate::policy::Expectations::here(model_api),
            in_use.as_deref(),
        )
        .await
    }

    /// Revert to the previously installed release.
    ///
    /// Reachable when `robotd` is dead — that is the case it exists for
    /// (`docs/design/architecture.md` §1.1).
    pub async fn rollback(&mut self, component: &str) -> Result<ApplyResult, Error> {
        let _lock = UpdateLock::try_acquire(&self.config.state_dir)?.ok_or(Error::Busy)?;
        let cfg = self.config.component(component)?.clone();
        let store = self.store(component)?;

        let current = store.current()?;
        let previous = self.rollback_target(component, &store, current.as_ref())?;

        let rec = self.begin_transition(component, "rollback", current.clone());
        self.transition_to(component, &cfg, &store, &previous, current, &rec)
            .await
    }

    /// Revert to the never-pruned known-good release
    /// (`docs/design/updater-design.md` §8.2).
    pub async fn reset_to_golden(&mut self, component: &str) -> Result<ApplyResult, Error> {
        let _lock = UpdateLock::try_acquire(&self.config.state_dir)?.ok_or(Error::Busy)?;
        let cfg = self.config.component(component)?.clone();
        let store = self.store(component)?;

        let golden = cfg
            .golden
            .clone()
            .ok_or_else(|| Error::Config(format!("component {component} has no golden release")))?;
        let current = store.current()?;

        let rec = self.begin_transition(component, "reset to golden", current.clone());
        self.transition_to(component, &cfg, &store, &golden, current, &rec)
            .await
    }

    /// Point the symlink at an already-installed release without downloading.
    ///
    /// Model-library switching, and a targeted revert for the daemon. Gated and
    /// rolled back like an update: a bad selection must be as recoverable as a bad
    /// install.
    pub async fn select(
        &mut self,
        component: &str,
        version: &semver::Version,
    ) -> Result<ApplyResult, Error> {
        let lock = UpdateLock::try_acquire(&self.config.state_dir)?.ok_or(Error::Busy)?;
        let cfg = self.config.component(component)?.clone();
        let store = self.store(component)?;

        if !store.release_dir(version).is_dir() {
            return Err(Error::NotInstalled {
                component: component.to_owned(),
                version: version.clone(),
            });
        }

        // Re-check compatibility from the manifest kept with the release: the
        // daemon may have changed since it was installed.
        if let Some(manifest) = Self::embedded_manifest(&store, version) {
            match manifest.compatibility(&self.capabilities().await) {
                Compatibility::Ok => {}
                Compatibility::Refused(reason) => return Err(Error::Incompatible(reason)),
                Compatibility::Unknown(reason) if manifest.model_api.is_some() => {
                    return Err(Error::Incompatible(format!(
                        "cannot confirm compatibility: {reason}"
                    )));
                }
                Compatibility::Unknown(_) => {}
            }
        }

        let current = store.current()?;
        if current.as_ref() == Some(version) {
            // Same repair as `apply`'s, for the same reason: selecting the release a board already has
            // active is the other command an operator reaches for when a daemon looks wrong.
            return Ok(ApplyResult::AlreadyCurrent {
                version: version.clone(),
                stale: self.stale_units(version, &cfg, &store),
            });
        }

        // `select` can move to a release carrying newer binaries, so the deferred units are as
        // stale afterwards as they are after an `apply`. Lock released first, as above.
        let rec = self.begin_transition(component, &format!("select {version}"), current.clone());
        let outcome = self
            .transition_to(component, &cfg, &store, version, current, &rec)
            .await;
        drop(lock);
        schedule_restarts_if_needed(self.deferred_restarts, &outcome, &rec).await;
        outcome
    }

    /// Open a transcript for one of the three transitions that move between releases already on
    /// the board.
    ///
    /// No progress channel: these paths have never streamed phases to a subscriber and this does
    /// not change that. They are recorded all the same, because "the robot is on an old release
    /// and nobody knows why" is answered by a rollback's transcript at least as often as by an
    /// apply's.
    fn begin_transition(
        &self,
        component: &str,
        asked_as: &str,
        installed: Option<semver::Version>,
    ) -> Recorder {
        Recorder::begin(
            component,
            None,
            &self.config.state_dir,
            RunEvent::Began {
                component: ComponentId::new(component),
                target: asked_as.to_owned(),
                installed,
                source: "already installed on this board".into(),
                requested_by: None,
            },
        )
    }

    /// Shared tail of rollback / reset-to-golden / select: swap, apply, gate, and
    /// revert on failure.
    async fn transition_to(
        &self,
        component: &str,
        cfg: &ComponentConfig,
        store: &Store,
        to: &semver::Version,
        from: Option<semver::Version>,
        rec: &Recorder,
    ) -> Result<ApplyResult, Error> {
        // Validate *before* arming. Arming for a version that then fails to link
        // would leave a trial referring to something never live, which a later boot
        // would "recover" from with a spurious rollback and a bogus log entry.
        if !store.release_dir(to).is_dir() {
            let failed = Err(Error::NotInstalled {
                component: component.to_owned(),
                version: to.clone(),
            });
            rec.finish(&failed);
            return failed;
        }

        // Armed before the swap, so a crash in between is still recoverable.
        self.boot_counter.arm(&PendingUpdate {
            component: component.to_owned(),
            version: to.clone(),
            previous: from.clone(),
            boots: 0,
        })?;

        rec.phase(
            Phase::Swapping,
            Some(format!(
                "{} → {to}",
                from.as_ref()
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "nothing".into())
            )),
        );

        // From here the trial is armed, so any early return must disarm it —
        // otherwise a later boot reverts an update that never went live.
        if let Err(e) = store.swap_to(to) {
            let _ = self.boot_counter.confirm(component);
            let failed = Err(e);
            rec.finish(&failed);
            return failed;
        }
        rec.phase(Phase::Applying, None);
        if let Err(e) = self
            .run_apply_action(&cfg.on_apply, &store.release_dir(to), rec)
            .await
        {
            let _ = self.boot_counter.confirm(component);
            let failed = Err(e);
            rec.finish(&failed);
            return failed;
        }

        rec.phase(Phase::HealthGate, None);
        let gate = self.health_gate(cfg).await;
        record_gate(rec, &gate);

        let outcome = match gate {
            Ok(_) => {
                self.boot_counter.confirm(component)?;
                Ok(ApplyResult::Applied {
                    from: from.clone(),
                    to: to.clone(),
                })
            }
            Err(reason) => {
                rec.phase(Phase::RollingBack, Some(reason.to_string()));
                match self
                    .rollback_to(component, cfg, store, from.as_ref(), rec)
                    .await?
                {
                    Some(reverted) => Ok(ApplyResult::RolledBack {
                        attempted: to.clone(),
                        reason: reverted.describe(&reason.to_string()),
                        reverted_to: Some(reverted.version),
                    }),
                    None => Ok(ApplyResult::Stuck {
                        version: to.clone(),
                        reason: format!("{reason}; nothing to revert to"),
                    }),
                }
            }
        };

        // Every class of outcome is journalled, matching `apply`. Logging only
        // successes here meant support could see a rollback that happened via an
        // update but not one via `rollback`/`select`/`reset-to-golden`.
        self.record(component, from, &outcome, rec.run());
        rec.finish(&outcome);
        outcome
    }

    /// Append the outcome of an operation to the update log.
    ///
    /// The single place any operation writes an entry, so `to` cannot mean different things
    /// in different paths.
    ///
    /// `to` names **the version the entry is about**:
    ///  - `Success` → the version now running,
    ///  - `RolledBack` → the version that *failed* (not the one reverted to),
    ///  - `Stuck` → the version that failed and could not be reverted.
    ///
    /// That definition is load-bearing: [`crate::journal::Journal::known_bad`] reads
    /// this field to avoid choosing a failed release as a rollback target. Recording
    /// the reverted-to version here would blacklist the release the robot is
    /// successfully running.
    ///
    /// Best-effort: the log is advisory and must never change what the client is
    /// told (`docs/design/updater-design.md` §8.3).
    fn record(
        &self,
        component: &str,
        from: Option<semver::Version>,
        outcome: &Result<ApplyResult, Error>,
        run: Option<u64>,
    ) {
        let Some((to, outcome)) = journal_outcome(outcome) else {
            return;
        };

        let entry = LogEntry {
            at: now_unix(),
            component: ComponentId::new(component),
            from,
            to,
            outcome,
            run,
        };
        if let Err(e) = self.journal.append(&entry) {
            tracing::error!(error = %e, "could not write the update log");
        }
    }

    /// Pin a component to a version, or unpin with `None`.
    ///
    /// Written to `state_dir`, not back into `updater.toml`: a pin is device state
    /// that must survive updates, and rewriting a human-edited config would destroy
    /// its comments.
    ///
    /// Refuses a version that is neither installed nor obtainable — a pin nothing can
    /// satisfy is an update freeze that looks like a working robot.
    pub async fn pin(
        &mut self,
        component: &str,
        version: Option<semver::Version>,
    ) -> Result<(), Error> {
        let cfg = self.config.component(component)?.clone();

        if let Some(version) = &version {
            let store = Store::new(cfg.install_dir.clone());
            if !store.release_dir(version).is_dir()
                && source::from_config(&cfg.source)
                    .manifest_for(version)
                    .await
                    .is_err()
            {
                return Err(Error::NotInstalled {
                    component: component.to_owned(),
                    version: version.clone(),
                });
            }
        }

        self.pins.set(component, version.as_ref())
    }

    /// The pin in force for a component: runtime state overrides the config default.
    fn effective_pin(&self, component: &str) -> Option<semver::Version> {
        match self.pins.get(component) {
            Ok(Some(pinned)) => Some(pinned),
            Ok(None) => self
                .config
                .component(component)
                .ok()
                .and_then(|c| c.pinned.clone()),
            Err(e) => {
                // Failing open here would silently ignore a pin. Log and fall back to
                // the config value, which is at least explicit.
                tracing::error!(error = %e, "could not read pins; using the config default");
                self.config
                    .component(component)
                    .ok()
                    .and_then(|c| c.pinned.clone())
            }
        }
    }

    // ── startup recovery ─────────────────────────────────────────────────────

    /// Recover from an interrupted run. **Call once at startup, before serving.**
    ///
    /// Two jobs: revert a pending update that never confirmed healthy across
    /// [`MAX_BOOT_ATTEMPTS`] boots, and delete staging leftovers. This is the path
    /// that catches a release which doesn't start at all — the in-process health
    /// gate can't, because it died with it.
    /// The units of one component that are not running the release named, and so are owed a restart.
    ///
    /// The same reading [`Self::reconcile_running_units`] does at startup, for one component and
    /// without acting: the acting happens after the reply is on the wire, because the units this most
    /// often names are the two that cannot be restarted before it. Called on the already-current
    /// paths only, which is where the answer changes what the operation reports.
    fn stale_units(
        &self,
        version: &semver::Version,
        cfg: &ComponentConfig,
        store: &Store,
    ) -> Vec<String> {
        let units = units_shipped(&store.release_dir(version), &configured_units(cfg));
        crate::reconcile::stale_units(version, &units)
    }

    /// Check that the restarts an update scheduled actually happened, and fix what did not.
    ///
    /// Runs at startup, beside [`Self::recover_on_start`], because that is the first moment the
    /// answer exists: `updaterd` cannot watch its own replacement land, so the check belongs to the
    /// successor. See [`crate::reconcile`] for why scheduling a restart is not evidence one
    /// happened.
    ///
    /// Per component, since each has its own active release and ships its own units. Failures are
    /// logged rather than returned, for the same reason the scheduling is: this must never be a
    /// reason to refuse to serve.
    pub async fn reconcile_running_units(&self) -> Vec<crate::reconcile::Finding> {
        let mut findings = Vec::new();

        for name in self.config.components.keys() {
            let Ok(store) = self.store(name) else {
                continue;
            };
            let Ok(Some(active)) = store.current() else {
                // No active release: a component that has never been installed. Nothing to compare
                // against, and nothing to fix.
                continue;
            };

            // The *shipped* set rather than the restart set: `updaterd` and `btd` are excluded from
            // an update's own restarts, which makes them the two this check exists for.
            let units = units_shipped(
                &store.release_dir(&active),
                &configured_units(&self.config.components[name]),
            );
            if units.is_empty() {
                continue;
            }

            findings.extend(crate::reconcile::check(SYSTEMCTL, &active, SELF_UNIT, units).await);
        }

        findings
    }

    pub async fn recover_on_start(&mut self) -> Result<Vec<ApplyResult>, Error> {
        for name in self.config.components.keys().cloned().collect::<Vec<_>>() {
            if let Ok(store) = self.store(&name) {
                let _ = store.clean_staging();
            }
        }

        // Before the boot counter, and that ordering is load-bearing: a rescue has already made the
        // decision an armed trial was going to make, and further than the trial would have gone.
        // Left in place, `record_boot` below would advance that trial and eventually revert to
        // `previous` — moving `current` off the golden release the rescue just chose.
        self.record_rescue();

        self.refresh_golden_links();

        // Every component's trial advances, independently. A model transition must
        // not consume or clear a daemon update's budget.
        let mut outcomes = Vec::new();
        for pending in self.boot_counter.record_boot()? {
            if !BootCounter::exhausted(&pending, MAX_BOOT_ATTEMPTS) {
                tracing::info!(
                    component = %pending.component,
                    version = %pending.version,
                    boots = pending.boots,
                    "update still on trial"
                );
                continue;
            }

            // A trial for a component that has since been removed from config must
            // not wedge startup; clear it and move on.
            let Ok(cfg) = self.config.component(&pending.component).cloned() else {
                tracing::warn!(
                    component = %pending.component,
                    "pending trial for an unconfigured component; clearing it"
                );
                self.boot_counter.confirm(&pending.component)?;
                continue;
            };

            // **The budget decides when to ask; the robot decides whether to revert.**
            //
            // A trial reaching this point means no apply ever confirmed it — the usual cause being
            // an apply that was killed before its gate ran, which is what happens when the
            // release's own `hooks/postinstall` restarts `updaterd` mid-apply. It does *not* mean
            // the release is bad, and reverting one that is working replaces the code under
            // whoever is looking at the robot, having told them nothing.
            //
            // So the same three-way question `health_gate` asks, in the same words, because the
            // reasoning is identical:
            //
            //   - healthy: the release works. Confirm it. The budget was spent counting boots on a
            //     release that was fine all along.
            //   - degraded: the robot is not working *for a reason a rollback cannot fix* — no
            //     servo power, a loose bus, absent hardware. Reverting hides a hardware fault
            //     behind a software change, and the next release will be reverted too.
            //   - anything else: revert, as before. `robotd` not answering, or answering
            //     unhealthy, is the case this budget exists for.
            //
            // Only for a socket gate. `HealthCheck::None` has nothing to ask, and `Command` answers
            // a two-way question — there is no "degraded" in an exit status.
            if let HealthCheck::Socket { .. } = cfg.health {
                match self.robot.health(ROBOT_QUERY_TIMEOUT).await {
                    crate::robot::Health::Healthy => {
                        tracing::warn!(
                            component = %pending.component,
                            version = %pending.version,
                            boots = pending.boots,
                            "trial was never confirmed, but the robot is healthy on it; \
                             committing instead of reverting"
                        );
                        self.boot_counter.confirm(&pending.component)?;
                        continue;
                    }
                    crate::robot::Health::Degraded(reason) => {
                        tracing::warn!(
                            component = %pending.component,
                            version = %pending.version,
                            boots = pending.boots,
                            reason = %reason,
                            "trial was never confirmed and the robot is degraded for a reason this \
                             release cannot have caused and a rollback cannot fix; committing"
                        );
                        self.boot_counter.confirm(&pending.component)?;
                        continue;
                    }
                    // Fall through to the revert below, which logs why.
                    _ => {}
                }
            }

            tracing::warn!(
                component = %pending.component,
                version = %pending.version,
                boots = pending.boots,
                "pending update never confirmed healthy; reverting"
            );

            let store = self.store(&pending.component)?;

            // Its own run, and not a continuation of the apply that armed the trial — because it
            // is one. That apply ended when `updaterd` restarted itself, minutes and a reboot ago,
            // and its transcript says so by having no ending. This is the verdict arriving, and it
            // is what someone reading `update show` after a robot came back on an older release
            // needs to find.
            let rec = Recorder::begin(
                &pending.component,
                None,
                &self.config.state_dir,
                RunEvent::Began {
                    component: ComponentId::new(pending.component.clone()),
                    target: format!("boot-counter revert from {}", pending.version),
                    installed: Some(pending.version.clone()),
                    source: "already installed on this board".into(),
                    requested_by: None,
                },
            );
            rec.say(format!(
                "this release was armed for trial and never reported healthy across {} boots",
                pending.boots
            ));

            // §8.2's chain: previous → golden. Escalate past `previous` when it is
            // absent, gone from disk, or itself recorded as bad — otherwise a robot
            // whose previous release is also broken reverts onto a second failure and
            // never reaches golden.
            let known_bad = self
                .journal
                .known_bad(&pending.component)
                .unwrap_or_default();
            let previous_is_usable = pending
                .previous
                .as_ref()
                .is_some_and(|v| store.release_dir(v).is_dir() && !known_bad.contains(v));

            let target = if previous_is_usable {
                pending.previous.clone()
            } else {
                if pending.previous.is_some() {
                    tracing::warn!(
                        component = %pending.component,
                        "recorded previous release is missing or known-bad; escalating to golden"
                    );
                }
                cfg.golden.clone().filter(|g| store.release_dir(g).is_dir())
            };
            let reverted = self
                .rollback_to(&pending.component, &cfg, &store, target.as_ref(), &rec)
                .await?;

            let reason = format!("never reported healthy across {} boots", pending.boots);

            let outcome = match reverted {
                Some(reverted) => ApplyResult::RolledBack {
                    attempted: pending.version.clone(),
                    reason: reverted.describe(&reason),
                    reverted_to: Some(reverted.version),
                },
                // Nothing to revert to. `rollback_to` has cleared the trial, so this
                // is reported exactly once rather than on every subsequent boot.
                None => ApplyResult::Stuck {
                    version: pending.version.clone(),
                    reason: format!(
                        "{reason}; no previous release and no golden configured, so there was \
                         nothing to revert to — needs operator intervention"
                    ),
                },
            };

            rec.finish(&Ok(outcome.clone()));

            let logged = LogEntry {
                at: now_unix(),
                component: ComponentId::new(pending.component.clone()),
                from: Some(pending.version.clone()),
                to: match &outcome {
                    ApplyResult::RolledBack { reverted_to, .. } => reverted_to.clone(),
                    _ => None,
                },
                outcome: match &outcome {
                    ApplyResult::Stuck { reason, .. } => Outcome::Aborted {
                        reason: reason.clone(),
                    },
                    _ => Outcome::RolledBack {
                        reason: reason.clone(),
                    },
                },
                run: rec.run(),
            };
            if let Err(e) = self.journal.append(&logged) {
                tracing::error!(error = %e, "could not write the update log");
            }

            outcomes.push(outcome);
        }

        Ok(outcomes)
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    /// Facts a manifest is checked against.
    ///
    /// `model_api` is `None` when `robotd` is unreachable, which the compatibility
    /// check treats as *unknown* rather than incompatible — see
    /// [`crate::manifest::Compatibility`].
    async fn capabilities(&self) -> Capabilities {
        Capabilities {
            hw_rev: self.config.hw_rev,
            model_api: self.robot.model_api(ROBOT_QUERY_TIMEOUT).await,
            schema_version: SUPPORTED_SCHEMA_VERSION,
        }
    }

    fn store(&self, component: &str) -> Result<Store, Error> {
        let cfg = self.config.component(component)?;
        Ok(Store::new(cfg.install_dir.clone()))
    }

    /// Turn a `robot-rescue` breadcrumb into a permanent record, and clear it.
    ///
    /// The rescue swaps `current` to golden and reboots with no daemon running, so this start is the
    /// first moment anything can write that down where it will be found. Three things happen, and
    /// each is one of the constraints the design names:
    ///
    /// - **the update log gets an entry**, because the breadcrumb is deleted below and a rollback
    ///   nobody can see afterwards is how a day goes to "it works on my board";
    /// - **any armed trial for that component is cleared**, since the rescue already decided it;
    /// - **the breadcrumb is removed**, which is what releases the rescue's loop guard. It refuses to
    ///   act while one is on record, so the guard opens exactly when the board proves it can run its
    ///   update daemon again, and stays shut when it cannot.
    ///
    /// Never fatal. A board that has just been rescued must not be a board whose `updaterd` refuses
    /// to serve.
    fn record_rescue(&mut self) {
        let path = self
            .config
            .state_dir
            .join(crate::journal::RESCUE_BREADCRUMB);
        let Ok(text) = std::fs::read_to_string(&path) else {
            return;
        };

        let crumb = crate::journal::Breadcrumb::parse(&text);
        tracing::warn!(
            from = crumb.from.as_deref().unwrap_or("(none)"),
            to = crumb.to.as_deref().unwrap_or("(unknown)"),
            because = crumb.because.as_deref().unwrap_or("(unrecorded)"),
            "this board was rescued to golden since the last start"
        );

        // Matched on `install_dir` rather than on the name `daemon`: the rescue knows which tree it
        // swapped and nothing else about the config, and a hardcoded component name would be a
        // second place that has to agree with `updater.toml`.
        let component = crumb.install_dir.as_deref().and_then(|dir| {
            self.config
                .components
                .iter()
                .find(|(_, cfg)| cfg.install_dir == std::path::Path::new(dir))
                .map(|(name, _)| name.clone())
        });

        match component {
            Some(name) => {
                let because = format!(
                    "boot recovery swapped to golden without updaterd ({})",
                    crumb.because.as_deref().unwrap_or("no reason recorded")
                );

                // A transcript for a run this process did not perform. Thin by necessity — the
                // rescue is a shell script that ran before `updaterd` existed on this boot, and
                // the breadcrumb is everything it left behind — but a run someone can find, which
                // beats a rollback in the log with nothing behind it.
                let rec = Recorder::begin(
                    &name,
                    None,
                    &self.config.state_dir,
                    RunEvent::Began {
                        component: crate::proto::ComponentId(name.clone()),
                        target: "reset to golden, by robot-rescue".into(),
                        installed: crumb.from.as_deref().and_then(|v| v.parse().ok()),
                        source: "already installed on this board".into(),
                        requested_by: None,
                    },
                );
                rec.say(
                    "performed outside updaterd, before it started; this transcript is what the \
                     breadcrumb it left recorded, not a first-hand account",
                );
                rec.note(RunEvent::Ended {
                    outcome: Some(crate::proto::Outcome::RolledBack {
                        reason: because.clone(),
                    }),
                    summary: because.clone(),
                });

                let entry = crate::proto::LogEntry {
                    at: crate::journal::now_unix(),
                    component: crate::proto::ComponentId(name.clone()),
                    from: crumb.from.as_deref().and_then(|v| v.parse().ok()),
                    to: crumb.to.as_deref().and_then(|v| v.parse().ok()),
                    outcome: crate::proto::Outcome::RolledBack { reason: because },
                    run: rec.run(),
                };
                if let Err(e) = self.journal.append(&entry) {
                    tracing::error!(error = %e, "could not record the rescue in the update log");
                }
                if let Err(e) = self.boot_counter.confirm(&name) {
                    tracing::error!(error = %e, "could not clear the trial the rescue superseded");
                }
            }
            // Not attributed to a component we guessed. The journal warning above is the record in
            // this case, and the breadcrumb is still cleared: a guard that never opens would leave
            // the board unable to rescue itself a second time, which is worse than a missing log
            // line for a state only a moved `install_dir` produces.
            None => tracing::warn!(
                install_dir = crumb.install_dir.as_deref().unwrap_or("(unrecorded)"),
                "the rescued tree matches no configured component; not recording it in the log"
            ),
        }

        // Durable, because the guard depends on this being gone: a delete that did not reach disk
        // means the next start declines to act on a board that has already been rescued once.
        if let Err(e) = std::fs::remove_file(&path) {
            tracing::error!(error = %e, "could not clear the rescue breadcrumb; robot-rescue will decline until it is removed by hand");
        } else {
            let _ = crate::fsutil::fsync_parent(&path);
        }
    }

    /// Publish each component's configured golden release as a `golden` symlink.
    ///
    /// `scripts/robot-rescue` runs when `updaterd` does not, so it cannot ask this process for
    /// golden and must not parse `updater.toml` to find it — a release whose `updaterd` rejects
    /// that file is the likeliest thing the rescue exists for. The link is how the answer survives
    /// the daemon.
    ///
    /// Never fatal. Failing to publish golden loses the rescue path; refusing to start over it
    /// loses the update path as well, which is strictly worse.
    fn refresh_golden_links(&self) {
        for (name, cfg) in &self.config.components {
            let Some(golden) = &cfg.golden else {
                continue;
            };
            let store = Store::new(cfg.install_dir.clone());

            // A configured golden that is not installed is not a rollback target, and a dangling
            // link would make the rescue believe otherwise. Loud, because it means the never-brick
            // guarantee is currently void on this board — `prune` protects golden once it is here,
            // but nothing installs it retroactively.
            if !store.release_dir(golden).is_dir() {
                tracing::warn!(
                    component = %name,
                    version = %golden,
                    "golden is configured but not installed; no rollback target for the recovery path"
                );
                continue;
            }

            match store.mark_golden(golden) {
                Ok(()) => tracing::debug!(component = %name, version = %golden, "golden published"),
                Err(e) => tracing::warn!(
                    component = %name,
                    version = %golden,
                    error = %e,
                    "could not publish the golden symlink; robot-rescue will decline to act"
                ),
            }
        }
    }

    /// The manifest kept inside an installed release, if it's readable.
    fn embedded_manifest(store: &Store, version: &semver::Version) -> Option<Manifest> {
        let path = store.release_dir(version).join(EMBEDDED_MANIFEST);
        let bytes = std::fs::read(path).ok()?;
        serde_json::from_slice(&bytes).ok()
    }

    /// Restart or signal, per config.
    ///
    /// Models use `Reload` so a weights swap doesn't interrupt motor control
    /// (`docs/design/updater-design.md` §5.5). Note what is *absent* from a daemon
    /// restart list: `updaterd` never restarts itself, and shouldn't restart
    /// `btd` either — see `docs/design/updater-design.md` §4.
    ///
    /// **One unit per invocation**, and a unit that does not exist on this board is skipped
    /// rather than failing the update. Both halves of that were learned the hard way.
    ///
    /// `systemctl restart a b` fails as a whole if *either* unit is unknown — and it fails
    /// without restarting the one that does exist. So the release which first introduces a
    /// daemon could not be installed at all: its unit file arrives *with* that release and is
    /// therefore not installed when `on_apply` runs, systemd refuses the whole command, and
    /// `robotd` is left running from a release directory the swap has already moved. The health
    /// gate then reports it unreachable and reverts, with nothing in the outcome naming the
    /// unit that was missing.
    ///
    /// A missing unit is tolerated; a unit that exists and *fails to restart* is not. That
    /// distinction is the point, and it is why this asks systemd for `LoadState` rather than
    /// reading an exit code: `systemctl` does not reliably distinguish the two, and swallowing
    /// both would turn "the daemon is broken" into a silent success.
    async fn run_apply_action(
        &self,
        action: &ApplyAction,
        release_dir: &Path,
        rec: &Recorder,
    ) -> Result<(), Error> {
        match action {
            ApplyAction::None => {
                rec.say("this component restarts nothing");
                Ok(())
            }
            ApplyAction::Restart { units } => {
                for unit in units_to_restart(release_dir, units) {
                    let result = restart_one(SYSTEMCTL, &unit).await;
                    rec.note(RunEvent::Unit {
                        unit: unit.clone(),
                        action: "restart".into(),
                        detail: match &result {
                            Ok(()) => None,
                            Err(e) => Some(e.to_string()),
                        },
                    });
                    result?;
                }
                Ok(())
            }
            ApplyAction::Reload { unit, signal } => {
                let mut c = tokio::process::Command::new(SYSTEMCTL);
                c.arg("kill").arg(format!("--signal={signal}")).arg(unit);
                let result = run_systemctl(c, "apply action").await;
                rec.note(RunEvent::Unit {
                    unit: unit.clone(),
                    action: format!("reload ({signal})"),
                    detail: match &result {
                        Ok(()) => None,
                        Err(e) => Some(e.to_string()),
                    },
                });
                result
            }
        }
    }

    /// Wait for the new release to report healthy.
    ///
    /// A timeout is a **failure**: unproven is not healthy, or auto-rollback would
    /// never fire on a release that hangs.
    async fn health_gate(&self, cfg: &ComponentConfig) -> Result<GatePassed, Error> {
        if self.faults.fail_health {
            return Err(Error::Health("injected health failure".into()));
        }

        let Some(timeout) = cfg.health.timeout() else {
            // HealthCheck::None — nothing to gate on.
            return Ok(GatePassed::Healthy);
        };

        if self.faults.hang_health {
            tokio::time::sleep(timeout).await;
            return Err(Error::Health(format!(
                "health probe did not answer within {}s",
                timeout.as_secs()
            )));
        }

        match &cfg.health {
            HealthCheck::None => Ok(GatePassed::Healthy),
            HealthCheck::Socket { .. } => {
                // The socket path lives in `Config::robot_socket` and is used to build
                // the RobotClient in `main`; here we just ask the client.
                let deadline = tokio::time::Instant::now() + timeout;
                let mut last = String::from("no answer");
                while tokio::time::Instant::now() < deadline {
                    match self.robot.health(ROBOT_QUERY_TIMEOUT).await {
                        crate::robot::Health::Healthy => return Ok(GatePassed::Healthy),
                        // Passes. Logged at warn, not swallowed: committing a release onto a
                        // robot that cannot move is the right call, but nobody should have to
                        // guess afterwards that that is what happened.
                        crate::robot::Health::Degraded(reason) => {
                            tracing::warn!(
                                reason = %reason,
                                "committing: the robot is degraded for a reason this release \
                                 cannot have caused and a rollback cannot fix"
                            );
                            return Ok(GatePassed::Degraded(reason));
                        }
                        crate::robot::Health::Unhealthy(reason) => last = reason,
                        // Fails, like `Unreachable`, and reads nothing like it. "unreachable"
                        // about a robot that is serving its socket sends the reader to the wrong
                        // half of the system for an hour; see `docs/project/install-path-gap.md`.
                        crate::robot::Health::Incompatible(reason) => {
                            last = format!(
                                "answered in a shape this updaterd cannot read ({reason}) — \
                                 the robot may be fine and the contract is what disagrees"
                            );
                        }
                        crate::robot::Health::Unreachable => {
                            last = "unreachable".into();
                        }
                    }
                    tokio::time::sleep(HEALTH_POLL_INTERVAL).await;
                }
                Err(Error::Health(format!(
                    "not healthy within {}s: {last}",
                    timeout.as_secs()
                )))
            }
            HealthCheck::Command { program, args, .. } => {
                let mut command = tokio::process::Command::new(program);
                command
                    .args(args)
                    .kill_on_drop(true)
                    .stdin(std::process::Stdio::null())
                    .stdout(std::process::Stdio::piped())
                    .stderr(std::process::Stdio::piped());
                let child = crate::spawn::retrying_busy(&mut command)
                    .await
                    .map_err(|e| Error::Health(format!("could not run probe: {e}")))?;
                let output = tokio::time::timeout(timeout, child.wait_with_output())
                    .await
                    .map_err(|_| {
                        Error::Health(format!("probe timed out after {}s", timeout.as_secs()))
                    })?
                    .map_err(|e| Error::Health(format!("could not run probe: {e}")))?;
                if output.status.success() {
                    // An exec probe reports pass or fail and has no way to say "degraded but
                    // not my fault" — that distinction is `robot.health`'s, over the socket.
                    Ok(GatePassed::Healthy)
                } else {
                    Err(Error::Health(format!(
                        "probe exited {}: {}",
                        output.status.code().unwrap_or(-1),
                        String::from_utf8_lossy(&output.stderr).trim()
                    )))
                }
            }
        }
    }

    /// Run preconditions.
    ///
    /// Called twice per apply: once with `None` before any network access (clock,
    /// robot stopped, no live session), then again with the verified manifest for
    /// the disk-space check, whose requirement is only knowable from `size`.
    /// Splitting it is what keeps an unsynced clock from surfacing as an opaque TLS
    /// error instead of the diagnostic written for it.
    async fn preflight(
        &self,
        manifest: Option<&Manifest>,
        options: &ApplyOptions,
        store: &Store,
    ) -> Result<(), Error> {
        // Without a manifest there is no size to check against, so the space check
        // is trivially satisfied on the first pass.
        let (required, available) = match manifest {
            None => (0, u64::MAX),
            Some(manifest) => {
                // A publisher that omits `size` would otherwise make the whole check
                // vacuous (0 needed, always satisfied) — silently disabling it in
                // exactly the case a first install most needs it. Fall back to a
                // floor so "we have essentially no space" is still caught.
                let required = match manifest.size {
                    Some(size) => size.saturating_mul(SPACE_MULTIPLIER),
                    None => {
                        tracing::warn!(
                            version = %manifest.version,
                            "manifest omits `size`; using a minimum space requirement"
                        );
                        MIN_REQUIRED_BYTES
                    }
                };

                if self.faults.simulate_disk_full {
                    (required.max(1), 0)
                } else {
                    (required, self.available_space(store)?)
                }
            }
        };

        let report = preflight::Preflight {
            robot: &*self.robot,
            required_bytes: required,
            available_bytes: available,
            interrupt_sessions: options.interrupt_sessions,
            robot_query_timeout: ROBOT_QUERY_TIMEOUT,
            from_dir: options.from_dir.as_deref(),
        }
        .run()
        .await?;

        if let Some(failure) = report.first_failure() {
            return Err(Error::Preflight(format!(
                "{:?}: {}",
                failure.check,
                failure.detail.clone().unwrap_or_default()
            )));
        }
        Ok(())
    }

    /// Free space for the release tree.
    ///
    /// On a fresh robot `releases/` does not exist yet, and `statvfs` on a missing
    /// path fails — which `unwrap_or(u64::MAX)` would turn into "infinite space",
    /// disabling the check on first install. Walk up to the nearest existing
    /// ancestor instead, which is on the same filesystem.
    fn available_space(&self, store: &Store) -> Result<u64, Error> {
        let mut dir = store.releases_dir();
        loop {
            if dir.exists() {
                return store.available_space_at(&dir);
            }
            match dir.parent() {
                Some(parent) => dir = parent.to_path_buf(),
                None => {
                    return Err(Error::Internal(
                        "could not find an existing directory to measure free space".into(),
                    ));
                }
            }
        }
    }

    /// Verify a manifest's signature, naming what failed.
    ///
    /// The bare "did not verify against any trusted key" is true but unhelpful: the
    /// usual causes are a rotated signing key or a stale release left in the
    /// source, and both are diagnosable only if the message says *which* version and
    /// channel it was. The parsed fields are untrusted here — they are used for the
    /// message only, never for a decision.
    /// Returns the id of the trusted key that admitted it. A *set* of keys is allowed, so which
    /// one signed a release is a fact about that release — and [`crate::verify::TrustedKey::id`]
    /// has said it was for the log since the day it was written.
    fn verify_manifest(&self, signed: &source::SignedBytes<Manifest>) -> Result<String, Error> {
        self.keys
            .verify_bytes(&signed.bytes, &signed.signature)
            .map(|key| key.id.clone())
            .map_err(|e| {
                Error::Verification(format!(
                    "manifest for {} {} (unverified): {e}. \
                     Was the signing key rotated, or is this a stale release?",
                    signed.parsed.channel, signed.parsed.version
                ))
            })
    }

    /// Guard against a manifest that belongs to a different channel, so a
    /// misconfigured URL can't install a model as the daemon.
    fn check_channel(manifest: &Manifest, expected: &str) -> Result<(), Error> {
        if manifest.channel == expected {
            Ok(())
        } else {
            Err(Error::Incompatible(format!(
                "manifest is for channel {:?}, expected {expected:?}",
                manifest.channel
            )))
        }
    }
}

/// The outcome of a revert: where the robot ended up, and what did not come back with it.
struct Reverted {
    version: semver::Version,
    /// The apply action's failure, when the swap succeeded and it did not. Names the unit — see
    /// [`try_restart`].
    apply_error: Option<String>,
}

impl Reverted {
    /// The reason to report, which must carry both facts when there are two.
    ///
    /// Appended rather than replacing: why the update failed is what someone is looking for, and a
    /// unit that then failed to restart is a second thing to act on, not a correction of the first.
    ///
    /// **What this deliberately no longer claims is that something is down.** It used to, and on a
    /// bench board that sentence was false by the time anyone could read it: every daemon here runs
    /// `Restart=always`, so a unit that refuses a restart *on command* is usually back seconds
    /// later on its own. Reporting the refusal as an outage sent a reader looking for a dead daemon
    /// on a robot whose own health line, printed directly above this one, said `robot healthy`.
    ///
    /// So it reports what it observed — a restart that did not take — and names the command that
    /// answers the question it cannot. [`restart_one`] has already cleared the start-rate counter
    /// and tried a second time by the time this runs, so reaching here means two refusals, not one.
    fn describe(&self, why: &str) -> String {
        match &self.apply_error {
            None => why.to_owned(),
            Some(detail) => format!(
                "{why}; the release was reverted, but {detail}. `Restart=` may have brought it back \
                 since — `robotctl health` says whether it did."
            ),
        }
    }
}

/// Is this a bare `--staging` whose newest candidate is *older* than what the board runs?
///
/// Returns the installed version when so, purely so the caller can build the refusal without
/// unwrapping the option a second time.
///
/// A predicate rather than a let-chain at the call site because the interesting content is which
/// targets it answers for, and that is worth a test. `Target::Staging` only:
///
/// - `StagingExact` names an older candidate on purpose — see the call site.
/// - `Latest` has its own guard, which makes a different claim.
/// - `Ref` and `Exact` are exempt from both, for reasons the call site states.
///
/// Equality is not "behind": that a candidate has been promoted to exactly what is installed is
/// what `AlreadyCurrent` reports, above this and more usefully.
fn staging_has_nothing_newer<'a>(
    target: &crate::proto::Target,
    installed: Option<&'a semver::Version>,
    candidate: &semver::Version,
) -> Option<&'a semver::Version> {
    if !matches!(target, crate::proto::Target::Staging) {
        return None;
    }
    installed.filter(|installed| candidate < *installed)
}

/// The program that drives units. A constant so tests can substitute a stub for it.
const SYSTEMCTL: &str = "systemctl";

/// Where `hooks/postinstall` installs unit files, and so where the orphan check reads them.
const UNIT_DIR: &str = "/etc/systemd/system";

/// This process's own unit, which the reconciliation must recognise and never restart.
///
/// The bare name rather than `updaterd.service`, matching what [`units_shipped`] yields — it takes
/// the file stem, and `systemctl` accepts either.
const SELF_UNIT: &str = "updaterd";

/// Restart one unit, skipping it if this board does not have it installed.
///
/// `systemctl` is a parameter rather than hardcoded so a test can hand it a stub. That is the
/// only reason: nothing should ever pass anything else in production.
/// How long the replacement `updaterd` gets to load its config and exit.
///
/// It parses a file and builds an engine; a second would do. Ten leaves room for a board under load
/// mid-update without making a hung binary cost the health gate's whole budget.
const SELF_TEST_TIMEOUT: Duration = Duration::from_secs(10);

/// Units restarted *after* the outcome has been reported, not during the update.
///
/// The pair from [`NEVER_RESTART`]. Excluding them from the in-flight restart is right and
/// insufficient: it left both running the old binary until someone rebooted, which is how a resident
/// `updaterd` came to reject a newer `robotctl` with "client speaks API v4, daemon speaks v3", and
/// how `btd` fixes were tested against binaries that were never running.
///
/// Deferred rather than skipped, because the reason each is excluded expires the moment the answer
/// is out: `updaterd` has finished performing the update, and the reply `btd` was carrying has been
/// delivered. A client sees its outcome and then a dropped connection, which for BLE is an ordinary
/// reconnect.
const RESTART_AFTER_REPLYING: [&str; 2] = ["updaterd", "btd"];

/// How long to wait before those restarts, so the reply is on the wire first.
///
/// The engine runs *inside* the `update.apply` call, so restarting `updaterd` synchronously would
/// hand the client a broken pipe instead of the outcome it waited minutes for. A response is a
/// single write; five seconds is far more than it needs and still faster than any human reaction.
const DEFERRED_RESTART_DELAY: &str = "5s";

/// Prove the release's `updaterd` can start, before committing to it.
///
/// `updaterd` does not restart itself during an update, so without this a replacement binary that
/// cannot start is discovered at the *next boot* — after the commit, with nobody watching, and with
/// recovery living inside the very process failing to start. systemd retries it a few times and
/// gives up, leaving a robot that cannot update its way out. Here, a failure is just a rollback: the
/// old release is still on disk and nothing has rebooted.
///
/// `--self-test` and not `--check-only`: the latter runs boot recovery for real, which would have a
/// second engine advancing this update's trial against the same store.
///
/// A release with no `updaterd` — a model component, or one predating the flag — passes. The point
/// is to catch a broken replacement, not to require every artifact to contain one.
async fn self_test_updaterd(release_dir: &Path, config: Option<&Path>) -> Result<(), Error> {
    let binary = release_dir.join("bin").join("updaterd");
    if !binary.is_file() {
        return Ok(());
    }

    let mut command = tokio::process::Command::new(&binary);
    command.arg("--self-test");
    // The config *this* engine was loaded from, not the flag's default. Without it the probe reads
    // `/etc/robot/updater.toml` whatever the running daemon was started with, so on any board using
    // `--config` it validates a file that is not in use — and reports the release as broken when
    // that file does not exist. Found by `scripts/systemd-test.sh` on its first run.
    if let Some(config) = config {
        command.arg("--config").arg(config);
    }

    // Through the retry, and this is the call that most needs it: the binary being exec'd was
    // written by *this update*, moments ago, while hooks and `systemctl` were spawning around it —
    // the exact conditions `spawn::retrying_busy` documents. An `ETXTBSY` here is an `Error::SelfTest`,
    // which rolls back a release that was fine.
    command
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());

    let spawned = crate::spawn::retrying_busy(&mut command).await;
    let output = match spawned {
        Ok(child) => {
            match tokio::time::timeout(SELF_TEST_TIMEOUT, child.wait_with_output()).await {
                Ok(Ok(output)) => output,
                Ok(Err(e)) => {
                    return Err(Error::SelfTest(format!(
                        "{} could not be waited for: {e}",
                        binary.display()
                    )));
                }
                Err(_) => {
                    return Err(Error::SelfTest(format!(
                        "{} did not finish within {SELF_TEST_TIMEOUT:?}",
                        binary.display()
                    )));
                }
            }
        }
        // Could not be executed at all: the wrong architecture, a missing interpreter, a corrupt
        // file. Exactly what this exists to catch.
        Err(e) => {
            return Err(Error::SelfTest(format!(
                "could not run {}: {e}",
                binary.display()
            )));
        }
    };

    if output.status.success() {
        tracing::info!("the new updaterd passed its self-test");
        return Ok(());
    }

    // stderr, trimmed, because that is where the reason is — "config error: unknown field
    // `foo`" is the difference between a fixable release and a mystery.
    let reason = String::from_utf8_lossy(&output.stderr);
    let reason = reason.lines().last().unwrap_or("no output").trim();
    Err(Error::SelfTest(format!("{}: {reason}", output.status)))
}

/// Which units an outcome leaves owing a restart.
///
/// Two cases, and they are one act for two reasons.
///
/// **`Applied`** owes the pair a running update cannot restart in place: itself, and the transport the
/// reply may be travelling over.
///
/// **`AlreadyCurrent` with stale units** owes exactly those. Nothing was installed because nothing
/// needed to be, and a daemon is still running something else — the state
/// [`crate::reconcile`] repairs at every start, except for the one unit it refuses to repair. A stale
/// `updaterd` will not restart itself from its own startup path, so the only thing that ever looks at
/// it is an operator running `apply`, who until now got `already_current`, no restart, and a robot
/// still on the old binary. Repairing it here is not the loop the startup check guards against: it
/// fires once, on a request, rather than on every start.
///
/// A rollback owes nothing, and that is not an omission: it leaves the resident `updaterd` already
/// matching `current` — it was never restarted, so it is still the binary belonging to the release
/// being returned to — and restarting there would be churn with nothing to fix.
///
/// Pure, and separated from the scheduling below for the reason `reconcile::verdict_for` is: which
/// outcomes owe what is the part that can be wrong, and arranging each of them on a board costs an
/// afternoon apiece.
/// The update log's verdict for an outcome, or `None` for the outcomes it deliberately does not
/// record — a dry run, and an apply that found nothing to do.
///
/// Extracted from `Engine::record` so that the transcript's closing line and the log entry cannot
/// disagree about how a run ended. They are the same judgement written in two places, which is
/// exactly the shape that drifts.
///
/// The version returned names **the version the entry is about**: the one now running for a
/// success, and the one that *failed* for a rollback — see `Engine::record`, which explains why
/// [`crate::journal::Journal::known_bad`] depends on that.
/// Put the gate's verdict in the transcript, both ways it can pass.
fn record_gate(rec: &Recorder, verdict: &Result<GatePassed, Error>) {
    rec.note(RunEvent::Health {
        passed: verdict.is_ok(),
        detail: match verdict {
            Ok(GatePassed::Healthy) => None,
            Ok(GatePassed::Degraded(reason)) => Some(format!(
                "degraded, and committed anyway — this release cannot have caused it and a \
                 rollback cannot fix it: {reason}"
            )),
            Err(e) => Some(e.to_string()),
        },
    });
}

fn journal_outcome(
    outcome: &Result<ApplyResult, Error>,
) -> Option<(Option<semver::Version>, Outcome)> {
    Some(match outcome {
        Ok(ApplyResult::Applied { to, .. }) => (Some(to.clone()), Outcome::Success),
        Ok(ApplyResult::RolledBack {
            attempted, reason, ..
        }) => (
            Some(attempted.clone()),
            Outcome::RolledBack {
                reason: reason.clone(),
            },
        ),
        Ok(ApplyResult::Stuck { version, reason }) => (
            Some(version.clone()),
            Outcome::Aborted {
                reason: format!("stuck on {version}: {reason}"),
            },
        ),
        Ok(ApplyResult::AlreadyCurrent { .. } | ApplyResult::DryRunPassed { .. }) => return None,
        Err(e) => (
            None,
            Outcome::Aborted {
                reason: e.to_string(),
            },
        ),
    })
}

/// One sentence for how a run ended, including the two the update log does not keep.
///
/// Always present, unlike [`journal_outcome`]: a dry run that verified a release and stopped is a
/// perfectly good thing to have a transcript of, and "no outcome recorded" would be a poor way to
/// describe the run that told you the release was fine.
fn summarise(outcome: &Result<ApplyResult, Error>) -> String {
    match outcome {
        Ok(ApplyResult::Applied { from, to }) => match from {
            Some(from) => format!("applied {from} → {to}"),
            None => format!("installed {to}"),
        },
        Ok(ApplyResult::AlreadyCurrent { version, stale }) if stale.is_empty() => {
            format!("already on {version}; nothing to do")
        }
        Ok(ApplyResult::AlreadyCurrent { version, stale }) => format!(
            "already on {version}, but {} was not running it; restarting",
            stale.join(", ")
        ),
        Ok(ApplyResult::DryRunPassed { candidate }) => {
            format!("dry run: {candidate} verified, and nothing was changed")
        }
        Ok(ApplyResult::RolledBack {
            attempted,
            reverted_to,
            reason,
        }) => format!(
            "{attempted} failed and was rolled back to {}: {reason}",
            reverted_to
                .as_ref()
                .map(|v| v.to_string())
                .unwrap_or_else(|| "nothing".into())
        ),
        Ok(ApplyResult::Stuck { version, reason }) => {
            format!("stuck on {version}: {reason}")
        }
        Err(e) => format!("refused: {e}"),
    }
}

/// The target as the caller named it, for the transcript's opening line.
///
/// Rendered rather than structured, because it exists to be read back to whoever is asking what
/// they actually ran — and `Target` already carries the structure for anything that branches.
fn describe_target(target: &crate::proto::Target) -> String {
    match target {
        crate::proto::Target::Latest => "latest".to_owned(),
        crate::proto::Target::Exact(v) => v.to_string(),
        crate::proto::Target::Ref(git_ref) => format!("branch {git_ref}"),
        crate::proto::Target::Staging => "latest release candidate".to_owned(),
        crate::proto::Target::StagingExact(v) => format!("release candidate {v}"),
    }
}

/// Where the bytes were going to come from.
///
/// `from_dir` shadows the configured source for one call, so it is what this run's source *was* —
/// reporting the configured one would be a lie in exactly the case (a laptop push that went wrong)
/// where somebody is reading the transcript to find out where a release came from.
fn describe_source(source: &crate::config::SourceConfig, from_dir: Option<&Path>) -> String {
    use crate::config::SourceConfig;
    if let Some(dir) = from_dir {
        return format!(
            "{} (--from, overriding the configured source)",
            dir.display()
        );
    }
    match source {
        SourceConfig::GithubReleases { repo, .. } => format!("github.com/{repo}"),
        SourceConfig::HfHub { repo, revision, .. } => {
            format!("huggingface.co/{repo} at {revision}")
        }
        SourceConfig::LocalDir { path } => path.display().to_string(),
    }
}

/// Bytes as a person reads them. Two significant figures is all anyone wants from a download size.
fn describe_bytes(bytes: u64) -> String {
    const UNITS: [&str; 4] = ["B", "kB", "MB", "GB"];
    let mut value = bytes as f64;
    let mut unit = 0;
    while value >= 1000.0 && unit + 1 < UNITS.len() {
        value /= 1000.0;
        unit += 1;
    }
    if unit == 0 {
        format!("{bytes} B")
    } else {
        format!("{value:.1} {}", UNITS[unit])
    }
}

/// Put a hook's output in the transcript whether it passed or failed.
///
/// **Both, and that is the point.** `hooks::run` logs its output to the journal either way, but a
/// caller takes the outcome with `?` and drops it — so on the failure path the output survives only
/// inside the error message, and on the success path it survived nowhere the update itself could
/// point at. This is the hook that installs ONNX Runtime and the GStreamer stack; its output is the
/// answer to "can this board encode H.264, at this release".
fn record_hook(rec: &Recorder, hook: &str, outcome: &Result<hooks::HookOutcome, Error>) {
    match outcome {
        Ok(run) if !run.ran => rec.say(format!("this release ships no {hook}")),
        Ok(run) => rec.note(RunEvent::Hook {
            hook: hook.to_owned(),
            exit_code: run.exit_code,
            output: run.output.clone(),
        }),
        // The failure carries the output in its message, which is where `hooks::run` puts it.
        Err(Error::Hook { detail, .. }) => rec.note(RunEvent::Hook {
            hook: hook.to_owned(),
            exit_code: None,
            output: detail.clone(),
        }),
        Err(e) => rec.say(format!("{hook} could not be run: {e}")),
    }
}

fn restarts_owed(outcome: &Result<ApplyResult, Error>) -> Vec<&str> {
    match outcome {
        Ok(ApplyResult::Applied { .. }) => RESTART_AFTER_REPLYING.to_vec(),
        Ok(ApplyResult::AlreadyCurrent { stale, .. }) => stale.iter().map(String::as_str).collect(),
        _ => Vec::new(),
    }
}

/// Schedule what [`restarts_owed`] says the outcome owes.
async fn schedule_restarts_if_needed(
    enabled: bool,
    outcome: &Result<ApplyResult, Error>,
    rec: &Recorder,
) {
    if !enabled {
        // A test binary. See `Engine::without_deferred_restarts` for why this is about forking.
        tracing::debug!("deferred restarts suppressed");
        return;
    }
    let units = restarts_owed(outcome);
    if units.is_empty() {
        return;
    }
    if let Ok(ApplyResult::AlreadyCurrent { version, .. }) = outcome {
        tracing::warn!(
            %version,
            units = units.join(","),
            "this release is already installed, and these are not running it; restarting them"
        );
    }
    schedule_deferred_restarts(SYSTEMD_RUN, &units, rec).await;
}

/// The program that schedules the deferred restarts.
///
/// A constant taken as a parameter below, for exactly the reason `SYSTEMCTL` is: a test needs to
/// hand it a stub and assert what was asked. Hardcoded at the call site, this was the one command in
/// the update path that no test could observe — `--on-active` could have been misspelled and every
/// test would still have passed, while on a board the only symptom is a restart that silently never
/// happens.
const SYSTEMD_RUN: &str = "systemd-run";

/// Restart the deferred units, detached, a few seconds from now.
///
/// `systemd-run` rather than a child process, and this is the load-bearing detail: `systemctl
/// restart updaterd` kills `updaterd`'s whole cgroup, and a child of `updaterd` is *in* that cgroup —
/// so it would be killed partway through restarting its own parent. A transient unit lives outside
/// it and survives.
///
/// Failures are logged, never returned. This runs after the update is committed and journalled; an
/// update that succeeded must not be reported as failed because a restart could not be scheduled.
/// Swallowing them is affordable because they are not the last word: [`crate::reconcile`] checks at
/// the next start that each unit is on the active release and restarts what is not.
///
/// The units are a parameter because two callers owe different ones — see [`restarts_owed`] — and
/// because every unit that reaches here goes through the transient timer, including the ones an update
/// restarts in place. A second, immediate path for `robotd` and `configd` would buy five seconds and
/// cost a mechanism. Nor does this need [`restart_one`]'s absent-unit handling: a unit is named here
/// either because the release ships it or because it published an identity, and a process that
/// published one is running.
async fn schedule_deferred_restarts(systemd_run: &str, units: &[&str], rec: &Recorder) {
    for unit in units.iter().copied() {
        let mut command = tokio::process::Command::new(systemd_run);
        command
            .arg(format!("--on-active={DEFERRED_RESTART_DELAY}"))
            .arg("--timer-property=AccuracySec=100ms")
            .arg("--")
            .arg(SYSTEMCTL)
            .arg("restart")
            .arg(unit);

        // `tokio::process`, not `std::process`: this runs inside the async engine, and a blocking
        // `status()` here stalls the runtime — which showed up as unrelated operations later
        // failing with `Busy`, because the update lock had not been released yet.
        let spawned = crate::spawn::retrying_busy(&mut command).await;
        let waited = match spawned {
            Ok(mut child) => child.wait().await,
            Err(e) => Err(e),
        };
        let detail = match waited {
            Ok(status) if status.success() => {
                tracing::info!(unit, delay = DEFERRED_RESTART_DELAY, "restart scheduled");
                None
            }
            Ok(status) => {
                tracing::warn!(
                    unit,
                    %status,
                    "could not schedule the restart; it keeps the old binary until the next updaterd \
                     start notices"
                );
                Some(format!(
                    "systemd-run exited {status}; the unit keeps the old binary until the next updaterd start notices"
                ))
            }
            Err(e) => {
                tracing::warn!(
                    unit,
                    error = %e,
                    "could not run systemd-run; the unit keeps the old binary until the next updaterd \
                     start notices"
                );
                Some(format!(
                    "could not run systemd-run: {e}; the unit keeps the old binary until the next updaterd start notices"
                ))
            }
        };
        rec.note(RunEvent::Unit {
            unit: unit.to_owned(),
            action: format!("restart in {DEFERRED_RESTART_DELAY}"),
            detail,
        });
    }
}

/// Units this update must not touch, whatever a release ships.
///
/// `updaterd` is the process performing the update: restarting it kills the operation mid-flight.
/// `btd` may be the *transport the update was requested over* — restarting it drops the BLE
/// connection carrying `update.subscribe`, so the phone that started the update never learns the
/// outcome, which is the entire app-driven flow (`docs/design/updater-design.md` §4).
///
/// In code rather than in configuration, deliberately. These two are properties of what the daemons
/// *are*, not choices an operator should be able to get wrong — and a board that got them wrong
/// would break in a way nobody could see until an update was already running.
const NEVER_RESTART: [&str; 2] = ["updaterd", "btd"];

/// The units to restart: what the release ships, plus anything the config names.
///
/// **Derived from the release rather than read from the board**, which is the whole point.
/// `on_apply`'s list lives in the operator's `/etc/robot/updater.toml`, and `install.sh` preserves
/// that file — so a board provisioned before a daemon existed keeps a list that does not mention it,
/// and every release swaps that daemon's binary while leaving the old process running. The update
/// reports success, the daemon answers on stale code, and `apply` — as it then was — said
/// `already_current` and did nothing, so the obvious recovery command confirmed there was nothing to
/// recover. Four correct fixes were diagnosed as broken that way in one afternoon; see
/// `docs/project/install-path-gap.md` §4. Both halves are closed now: this function is the first, and
/// [`restarts_owed`] is the second.
///
/// A release already states which units it provides — it ships them in `systemd/` — so it can say
/// which to restart. Same realisation that made `hooks/postinstall` the right place to *install*
/// them.
///
/// The configured list is kept as an addition rather than replaced, for a unit that is not shipped
/// by the release but should still be restarted with it. It is no longer load-bearing: a board whose
/// config is years out of date now gets the right behaviour anyway.
///
/// An unreadable or absent `systemd/` directory yields the configured list unchanged. Older releases
/// predate the directory, and a rollback to one must still work.
/// The units a board's config names for this component, if it names any.
///
/// Shared by the three callers that need it rather than matched inline at each, so a component whose
/// `on_apply` is not a restart cannot be read as naming units in one place and not another.
fn configured_units(cfg: &ComponentConfig) -> Vec<String> {
    match &cfg.on_apply {
        ApplyAction::Restart { units } => units.clone(),
        _ => Vec::new(),
    }
}

fn units_shipped(release_dir: &Path, configured: &[String]) -> Vec<String> {
    let mut units: Vec<String> = Vec::new();

    match std::fs::read_dir(release_dir.join("systemd")) {
        Ok(entries) => {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().and_then(|e| e.to_str()) != Some("service") {
                    continue;
                }
                // A unit with no `[Install]` section is started by something else — a timer, or
                // another unit pulling it in — so its lifecycle is not this engine's to drive.
                //
                // The recovery net's oneshot is exactly that, and deliberately: it asks whether the
                // release that booted came up, and hands over to `robot-rescue` if not.
                // `hooks/postinstall` already skips `enable --now` on it for that reason, in as many
                // words — "`enable --now` on it would run a rollback check in the middle of the
                // update that installed it, with daemons legitimately mid-restart". Reading every
                // `*.service` then restarted it a moment later anyway, which is the same mistake
                // one function further on.
                //
                // The rule rather than a name in `NEVER_RESTART`: `postinstall` and this now agree
                // by construction, and the next unit like it needs nobody to remember.
                if !has_install_section(&path) {
                    tracing::debug!(
                        unit = %path.display(),
                        "no [Install] section, so something other than this update starts it"
                    );
                    continue;
                }
                if let Some(name) = path.file_stem().and_then(|n| n.to_str()) {
                    units.push(name.to_owned());
                }
            }
        }
        Err(e) => {
            // Not fatal. A release without a `systemd/` directory is simply an older one, and the
            // configured list is what boards have always used.
            tracing::debug!(
                dir = %release_dir.display(),
                error = %e,
                "no units in the release; using the configured list"
            );
        }
    }

    units.extend(configured.iter().cloned());

    // Sorted so the restart order is the same on every board and in every test, and deduplicated
    // because the config and the release will usually name the same daemons.
    units.sort();
    units.dedup();
    units
}

/// Is this unit one systemd is asked to enable, or one something else triggers?
///
/// An unreadable file answers "yes", which errs towards restarting: a unit whose contents cannot be
/// read is more likely a permissions or IO problem than a deliberately triggerless unit, and the
/// restart failing loudly beats it being skipped quietly.
fn has_install_section(path: &Path) -> bool {
    match std::fs::read_to_string(path) {
        Ok(text) => text.lines().any(|line| line.trim() == "[Install]"),
        Err(e) => {
            tracing::warn!(unit = %path.display(), error = %e, "cannot read this unit; treating it as one to restart");
            true
        }
    }
}

/// The subset an update restarts in flight: everything shipped, less the two it cannot touch while
/// it is running.
fn units_to_restart(release_dir: &Path, configured: &[String]) -> Vec<String> {
    let mut units = units_shipped(release_dir, configured);
    units.retain(|unit| !NEVER_RESTART.contains(&unit.as_str()));
    units
}

/// Restart one unit, with the two failures that are not what they look like handled here.
///
/// **The rate limit is the interesting one, and it is the common case rather than a race.** systemd
/// defaults to `StartLimitBurst=5` per `StartLimitIntervalSec=10s`; `robotd` sets `RestartSec=2s`.
/// So a daemon that exits immediately — a bad config file, a missing library — burns its five
/// starts in ten seconds and systemd refuses further ones with `Start request repeated too
/// quickly`. The health gate then waits another twenty, and the rollback's restart lands squarely
/// inside that refusal. It fails, and the robot is reported as having a daemon down when the
/// release it is now on would start perfectly.
///
/// That is not hypothetical: it is what a board did on the bench, reporting "so something on this
/// robot is down" while `robotctl health` two lines above said `robot healthy`. The counter is
/// cleared and the restart tried once more, which is what `reset-failed` is for.
///
/// Unconditional rather than matched on the error text: `reset-failed` is a no-op on a unit that is
/// not failed, and the alternative is grepping systemd's prose in whatever locale it was built
/// with. The cost is one extra pair of calls on a path that has already failed; a unit that
/// genuinely cannot start fails the second time too and is reported then.
async fn restart_one(systemctl: &str, unit: &str) -> Result<(), Error> {
    match try_restart(systemctl, unit).await {
        Ok(()) => Ok(()),
        Err(e) => {
            if unit_is_absent(systemctl, unit).await {
                // Expected exactly once per new daemon: the first update that carries it.
                // Whatever installs unit files picks it up, and the next update restarts it.
                tracing::warn!(
                    unit,
                    "not installed on this board, so it was not restarted. This is normal for a \
                     release that introduces a new daemon; install its unit file and it will \
                     restart on the next update."
                );
                return Ok(());
            }

            tracing::warn!(
                unit,
                error = %e,
                "restart refused; clearing the start-rate counter and trying once more"
            );
            let mut c = tokio::process::Command::new(systemctl);
            c.arg("reset-failed").arg(unit);
            // Ignored: a unit that was never failed makes this exit non-zero on some systemd
            // versions, and that must not replace the restart's own error with a worse one.
            let _ = run_systemctl(c, "reset-failed").await;

            try_restart(systemctl, unit).await
        }
    }
}

/// One `systemctl restart`, with the unit named in the error.
///
/// Named, because the caller restarts up to six of them and the bare message — systemd's own
/// "Job for X.service failed because the control process exited" wrapped in "restart failed" —
/// reached the update log without ever saying which unit the job was for.
async fn try_restart(systemctl: &str, unit: &str) -> Result<(), Error> {
    let mut c = tokio::process::Command::new(systemctl);
    c.arg("restart").arg(unit);
    run_systemctl(c, &format!("restarting {unit}")).await
}

/// Does systemd know this unit at all?
///
/// `LoadState=not-found` is the authoritative answer, and deliberately not inferred from an exit
/// code — `systemctl` does not reliably distinguish "no such unit" from "that unit would not
/// start". If the query itself fails we answer "present", so an unrelated systemd problem cannot
/// silently excuse a restart that should have worked.
async fn unit_is_absent(systemctl: &str, unit: &str) -> bool {
    let mut c = tokio::process::Command::new(systemctl);
    c.arg("show")
        .arg("--property=LoadState")
        .arg("--value")
        .arg(unit);
    c.kill_on_drop(true);

    c.stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());

    let Ok(child) = crate::spawn::retrying_busy(&mut c).await else {
        return false;
    };
    match tokio::time::timeout(APPLY_ACTION_TIMEOUT, child.wait_with_output()).await {
        Ok(Ok(output)) => String::from_utf8_lossy(&output.stdout).trim() == "not-found",
        _ => false,
    }
}

async fn run_systemctl(mut command: tokio::process::Command, what: &str) -> Result<(), Error> {
    command.kill_on_drop(true);

    // `spawn` + `wait_with_output` rather than `output()`, so the spawn can go through the
    // `ETXTBSY` retry — `output()` spawns internally and gives nothing to retry. `output()` also
    // pipes both streams for you, and this does not, so they are set explicitly: without them the
    // child inherits ours and the stderr this reports back would be empty.
    let child = crate::spawn::retrying_busy(
        command
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped()),
    )
    .await
    .map_err(|e| Error::Internal(format!("running systemctl: {e}")))?;

    let output = tokio::time::timeout(APPLY_ACTION_TIMEOUT, child.wait_with_output())
        .await
        .map_err(|_| Error::Internal(format!("{what} timed out")))?
        .map_err(|e| Error::Internal(format!("running systemctl: {e}")))?;

    if !output.status.success() {
        return Err(Error::Internal(format!(
            "{what} failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        )));
    }
    Ok(())
}

#[cfg(test)]
mod staging_channel_tests {
    use super::*;
    use crate::proto::Target;

    fn v(s: &str) -> semver::Version {
        semver::Version::parse(s).expect("a version")
    }

    /// The incident, in one assertion. A board on stable `0.4.0` asking for the newest candidate
    /// when the last candidate published was `0.2.0`: 0.3.0 and 0.4.0 were promoted straight to
    /// stable, so the staging scan still answers `0.2.0`.
    #[test]
    fn a_candidate_older_than_the_board_is_reported() {
        assert_eq!(
            staging_has_nothing_newer(&Target::Staging, Some(&v("0.4.0")), &v("0.2.0")),
            Some(&v("0.4.0"))
        );
    }

    /// The ordinary case, which must stay silent: a candidate ahead of the board is the entire
    /// point of the channel.
    #[test]
    fn a_candidate_ahead_of_the_board_is_not() {
        assert_eq!(
            staging_has_nothing_newer(&Target::Staging, Some(&v("0.4.0")), &v("0.5.0")),
            None
        );
    }

    /// A candidate equal to what is installed is `AlreadyCurrent`, which the caller answers
    /// before reaching here. Reporting it as a stale channel would be both wrong and worse.
    #[test]
    fn a_candidate_equal_to_the_board_is_not_behind() {
        assert_eq!(
            staging_has_nothing_newer(&Target::Staging, Some(&v("0.4.0")), &v("0.4.0")),
            None
        );
    }

    /// A first install has nothing to be behind. Guarding it would make `--staging` unusable on a
    /// freshly flashed board, which is one of the two boards that ever uses the flag.
    #[test]
    fn a_board_with_nothing_installed_is_never_behind() {
        assert_eq!(
            staging_has_nothing_newer(&Target::Staging, None, &v("0.2.0")),
            None
        );
    }

    /// Every other target, at a version that *would* trip this if the variant were not checked.
    ///
    /// The load-bearing one is `StagingExact`: it is the way past the refusal this function
    /// produces, so guarding it too would leave a board that can neither install the candidate
    /// nor be told why. The rest have their own guards or their own exemptions.
    #[test]
    fn no_other_target_is_this_functions_business() {
        for target in [
            Target::StagingExact(v("0.2.0")),
            Target::Latest,
            Target::Exact(v("0.2.0")),
            Target::Ref("my-branch".into()),
        ] {
            assert_eq!(
                staging_has_nothing_newer(&target, Some(&v("0.4.0")), &v("0.2.0")),
                None,
                "{target:?} must not be refused as a stale staging channel"
            );
        }
    }

    /// The message is the deliverable — the rollback it replaces was already correct, and what
    /// was missing was a sentence saying the channel had nothing newer. So: the two versions, the
    /// reason there is nothing there, and a command that can be pasted.
    #[test]
    fn the_refusal_says_the_channel_is_behind_and_what_to_type() {
        let text = Error::StagingBehind {
            component: "daemon".into(),
            installed: v("0.4.0"),
            candidate: v("0.2.0"),
        }
        .to_string();

        assert!(text.contains("newest release candidate is 0.2.0"), "{text}");
        assert!(text.contains("already on 0.4.0"), "{text}");
        assert!(
            text.contains("nothing more recent is available on the staging channel"),
            "{text}"
        );
        assert!(
            text.contains("robotctl update apply daemon --staging --version 0.2.0"),
            "{text}"
        );
        // Not the other refusal's words. Someone told "refusing to downgrade" goes looking for a
        // mirror that has gone backwards, and there isn't one.
        assert!(!text.contains("downgrade"), "{text}");
    }

    /// Both refusals answer the same JSON-RPC code, so a client that already handles one handles
    /// this. Pinned because the reasoning is a choice — see [`Error::code`].
    #[test]
    fn it_answers_the_downgrade_code() {
        assert_eq!(
            Error::StagingBehind {
                component: "daemon".into(),
                installed: v("0.4.0"),
                candidate: v("0.2.0"),
            }
            .code(),
            Error::WouldDowngrade {
                installed: v("0.4.0"),
                candidate: v("0.2.0"),
            }
            .code()
        );
    }
}

#[cfg(test)]
mod restart_tests {
    use super::*;

    /// A stub `systemctl` that knows about some units and not others, and records what it was
    /// asked to do. A script rather than a mock because the thing under test is a subprocess
    /// invocation — the bug this exists for was a shell command shape, not a Rust one.
    fn stub_systemctl(dir: &std::path::Path, known: &[&str]) -> std::path::PathBuf {
        let path = dir.join("systemctl");
        let cases = known
            .iter()
            .map(|u| format!("    {u}) exit 0 ;;"))
            .collect::<Vec<_>>()
            .join("\n");
        let script = format!(
            r#"#!/bin/sh
# $1 is the verb. Record every call so the test can assert one invocation per unit.
echo "$@" >> "$(dirname "$0")/calls"
if [ "$1" = show ]; then
    case "$4" in
{cases}
    *) echo not-found; exit 0 ;;
    esac
    echo loaded
    exit 0
fi
case "$2" in
{cases}
    *) echo "Unit $2 not found." >&2; exit 5 ;;
esac
"#
        );
        std::fs::write(&path, script).expect("write stub");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        }
        path
    }

    fn calls(dir: &std::path::Path) -> String {
        std::fs::read_to_string(dir.join("calls")).unwrap_or_default()
    }

    /// A release directory whose `bin/updaterd` is a script behaving as given.
    fn release_with_updaterd(script: &str) -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        let bin = dir.path().join("bin");
        std::fs::create_dir_all(&bin).unwrap();
        let path = bin.join("updaterd");
        std::fs::write(&path, script).unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        }
        dir
    }

    /// The failure this exists to catch, and the reason it must be reported rather than counted:
    /// a rollback reason of "unreachable" sent three investigations down the wrong path.
    #[tokio::test]
    async fn a_replacement_updaterd_that_cannot_start_fails_the_update() {
        let release = release_with_updaterd(
            "#!/bin/sh\necho 'config error: unknown field `nope`' >&2\nexit 1\n",
        );

        let err = self_test_updaterd(release.path(), None).await.unwrap_err();

        assert!(
            matches!(err, Error::SelfTest(_)),
            "expected a self-test failure, got {err:?}"
        );
        assert!(
            err.to_string().contains("unknown field"),
            "the reason must survive into the message: {err}"
        );
    }

    #[tokio::test]
    async fn a_replacement_updaterd_that_starts_passes() {
        let release = release_with_updaterd("#!/bin/sh\nexit 0\n");
        assert!(self_test_updaterd(release.path(), None).await.is_ok());
    }

    /// Model components ship no `updaterd`, and neither do releases predating the flag. Requiring
    /// one would make this a compatibility break rather than a safety net.
    #[tokio::test]
    async fn a_release_without_an_updaterd_passes() {
        let dir = tempfile::tempdir().unwrap();
        assert!(self_test_updaterd(dir.path(), None).await.is_ok());
    }

    /// The two lists are one decision, and splitting them is how a daemon gets excluded from the
    /// restart and then forgotten. Everything held back during the update is restarted after it.
    #[test]
    fn every_unit_held_back_is_restarted_once_the_answer_is_out() {
        let mut held: Vec<&str> = NEVER_RESTART.to_vec();
        let mut deferred: Vec<&str> = RESTART_AFTER_REPLYING.to_vec();
        held.sort_unstable();
        deferred.sort_unstable();
        assert_eq!(held, deferred);
    }

    /// Write a release directory that ships these units.
    fn release_shipping(units: &[&str]) -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dir.path().join("systemd")).unwrap();
        for unit in units {
            // With an `[Install]` section, because that is what makes a unit one an update starts.
            // The unit without one has its own test.
            std::fs::write(
                dir.path().join("systemd").join(format!("{unit}.service")),
                "[Unit]\n\n[Install]\nWantedBy=multi-user.target\n",
            )
            .unwrap();
        }
        dir
    }

    /// The point of the change: a board whose config predates a daemon still restarts it.
    ///
    /// `units = ["robotd"]` is what a board provisioned before `configd` existed still says, and
    /// `install.sh` preserves that file forever. Every release then swapped configd's binary and
    /// left the old process serving, which is how four correct fixes were diagnosed as broken.
    #[test]
    fn a_daemon_the_board_never_heard_of_is_restarted_anyway() {
        let release = release_shipping(&["robotd", "configd"]);
        let stale_config = vec!["robotd".to_owned()];

        assert_eq!(
            units_to_restart(release.path(), &stale_config),
            vec!["configd".to_owned(), "robotd".to_owned()]
        );
    }

    /// `updaterd` is performing the update and `btd` may be the transport it arrived over. Neither
    /// may be restarted however they reach the list — shipped by the release, named in the config,
    /// or both.
    #[test]
    fn updaterd_and_btd_are_never_restarted() {
        let release = release_shipping(&["robotd", "configd", "updaterd", "btd"]);
        let reckless = vec!["updaterd".to_owned(), "btd".to_owned()];

        assert_eq!(
            units_to_restart(release.path(), &reckless),
            vec!["configd".to_owned(), "robotd".to_owned()],
            "the exclusions must hold against both sources"
        );
    }

    /// A rollback to a release predating the `systemd/` directory has to keep working, and the
    /// configured list is what boards used before any of this existed.
    #[test]
    fn a_release_shipping_no_units_falls_back_to_the_config() {
        let dir = tempfile::tempdir().unwrap();
        let configured = vec!["robotd".to_owned(), "configd".to_owned()];

        assert_eq!(
            units_to_restart(dir.path(), &configured),
            vec!["configd".to_owned(), "robotd".to_owned()]
        );
    }

    /// Named in both places is one restart, not two — and the order is the same everywhere, so a
    /// failure is reproducible rather than depending on directory iteration order.
    #[test]
    fn the_set_is_deduplicated_and_ordered() {
        let release = release_shipping(&["robotd", "configd"]);
        let overlapping = vec!["configd".to_owned(), "robotd".to_owned()];

        assert_eq!(
            units_to_restart(release.path(), &overlapping),
            vec!["configd".to_owned(), "robotd".to_owned()]
        );
    }

    /// **The recovery net's oneshot must not be restarted by an update.** It asks whether the release
    /// that booted came up and hands over to `robot-rescue` if not, so running it mid-update points a
    /// rollback check at daemons that are legitimately mid-restart — and `robot-rescue` can swap to
    /// golden and reboot.
    ///
    /// `hooks/postinstall` already declines to `enable --now` it, for that reason and in those words.
    /// Reading every `*.service` then restarted it a moment later anyway: two places applying one rule
    /// to one unit and disagreeing. Keyed on `[Install]` rather than on the name, so the next unit
    /// like it needs nobody to remember.
    #[test]
    fn a_unit_something_else_triggers_is_not_restarted_by_an_update() {
        let release = release_shipping(&["robotd", "configd"]);
        // As the real one is: a oneshot with no `[Install]`, armed by a timer.
        std::fs::write(
            release
                .path()
                .join("systemd")
                .join("robot-boot-check.service"),
            "[Unit]\nDescription=Did the release that booted come up?\n\n\
             [Service]\nType=oneshot\nExecStart=/usr/local/sbin/robot-boot-check\n",
        )
        .unwrap();

        assert_eq!(
            units_to_restart(release.path(), &[]),
            vec!["configd".to_owned(), "robotd".to_owned()],
            "a unit with no [Install] is triggered by something else and is left to it"
        );
    }

    /// The other half, so the rule cannot be satisfied by skipping everything: a unit that *is*
    /// enabled stays in the set, and the timer beside the oneshot changes nothing — only `.service`
    /// files were ever read.
    #[test]
    fn a_unit_with_an_install_section_is_still_restarted() {
        let release = release_shipping(&["robotd"]);
        std::fs::write(
            release
                .path()
                .join("systemd")
                .join("robot-boot-check.timer"),
            "[Timer]\nOnBootSec=180\n\n[Install]\nWantedBy=timers.target\n",
        )
        .unwrap();

        assert_eq!(
            units_to_restart(release.path(), &[]),
            vec!["robotd".to_owned()]
        );
    }

    /// Only `.service` files. A release ships `sysusers.d/` too, and `postinstall` reads it —
    /// nothing there is a unit to restart.
    #[test]
    fn only_service_files_count() {
        let release = release_shipping(&["robotd"]);
        std::fs::write(release.path().join("systemd").join("robot.target"), "x").unwrap();
        std::fs::write(release.path().join("systemd").join("notes.txt"), "x").unwrap();

        assert_eq!(
            units_to_restart(release.path(), &[]),
            vec!["robotd".to_owned()]
        );
    }

    /// The bug this whole change exists for: the release that first carries a new daemon has no
    /// unit file for it yet, and failing the update over that left `robotd` unrestarted from a
    /// swapped-away directory — reported only as "not healthy within 30s: unreachable".
    #[tokio::test]
    async fn a_unit_this_board_does_not_have_is_skipped() {
        let dir = tempfile::tempdir().unwrap();
        let systemctl = stub_systemctl(dir.path(), &["robotd"]);

        assert!(
            restart_one(systemctl.to_str().unwrap(), "configd")
                .await
                .is_ok(),
            "a missing unit must not fail the update"
        );
    }

    /// A `systemctl` that is briefly busy for exec is retried, not reported as a failed restart.
    ///
    /// This is the flake that prompted the change, reproduced deterministically. These tests write a
    /// stub `systemctl` and exec it from parallel threads of one process; a fork in another test
    /// duplicates the write handle to *this* stub into its child, and for a few microseconds the
    /// kernel refuses to exec it. Holding a write handle here is the same condition, on purpose.
    ///
    /// On a board the consequence is worse than a red CI job: the same race reaches
    /// `self_test_updaterd`, which execs a binary the update wrote moments earlier, and an `ETXTBSY`
    /// there rolls back a release that was fine.
    ///
    /// Linux-only, and deliberately not made portable: macOS permits the exec, so on a Mac this
    /// would pass without ever provoking the condition. `hooks.rs`'s
    /// `a_hook_busy_forever_still_fails` is the control that keeps the pair honest — if the platform
    /// stopped producing `ETXTBSY`, that test fails and this one becomes vacuous.
    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn a_systemctl_busy_for_exec_is_retried_rather_than_failed() {
        let dir = tempfile::tempdir().unwrap();
        let path = stub_recorder(dir.path(), "systemctl");

        let holder = std::fs::OpenOptions::new().write(true).open(&path).unwrap();
        // Well inside the ~100 ms budget, and long enough that the first attempts do fail.
        let releaser = tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(25)).await;
            drop(holder);
        });

        restart_one(path.to_str().unwrap(), "robotd")
            .await
            .expect("a transiently busy systemctl must be retried, not reported as a failure");

        releaser.await.unwrap();
    }

    /// The other half, and the reason this is not just "ignore failures": a unit that exists and
    /// will not start is a real problem, and swallowing it would turn a broken daemon into a
    /// silent success.
    #[tokio::test]
    async fn a_unit_that_exists_but_fails_is_still_an_error() {
        let dir = tempfile::tempdir().unwrap();
        // `broken` is known to `show` (so LoadState is not not-found) but restart fails, which is
        // what a daemon that cannot start looks like.
        let path = dir.path().join("systemctl");
        std::fs::write(
            &path,
            "#!/bin/sh\nif [ \"$1\" = show ]; then echo loaded; exit 0; fi\necho 'job failed' >&2\nexit 1\n",
        )
        .unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        }

        let err = restart_one(path.to_str().unwrap(), "robotd")
            .await
            .unwrap_err();
        // Named, so a reader of the update log knows which of six units the job was for.
        assert!(
            format!("{err:?}").contains("restarting robotd failed"),
            "{err:?}"
        );
    }

    /// The bench incident, reproduced. A daemon that exits immediately burns systemd's five starts
    /// in ten seconds (`StartLimitBurst=5`, `robotd` at `RestartSec=2s`), and every restart after
    /// that is refused with `Start request repeated too quickly` — including the rollback's, which
    /// arrives while the gate's thirty seconds are still running down. The board then reported a
    /// daemon down while `robotctl health` said `robot healthy`.
    ///
    /// The stub refuses the first restart the way systemd does and accepts one after `reset-failed`,
    /// which is the sequence being asserted.
    #[tokio::test]
    async fn a_rate_limited_unit_is_reset_and_restarted_rather_than_reported_down() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("systemctl");
        std::fs::write(
            &path,
            r#"#!/bin/sh
echo "$@" >> "$(dirname "$0")/calls"
if [ "$1" = show ]; then echo loaded; exit 0; fi
if [ "$1" = reset-failed ]; then : > "$(dirname "$0")/cleared"; exit 0; fi
[ -f "$(dirname "$0")/cleared" ] && exit 0
echo 'Failed to restart robotd.service: Start request repeated too quickly.' >&2
exit 1
"#,
        )
        .unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        }

        restart_one(path.to_str().unwrap(), "robotd")
            .await
            .expect("a rate-limited unit must be reset and retried, not reported as a failure");

        let calls = calls(dir.path());
        assert!(
            calls.contains("reset-failed robotd"),
            "the start-rate counter was never cleared: {calls}"
        );
    }

    /// The other half, so the retry cannot become "ignore the first failure". A unit that is
    /// genuinely broken refuses both times and is reported, with `reset-failed` having been tried.
    #[tokio::test]
    async fn a_unit_that_refuses_twice_is_still_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("systemctl");
        std::fs::write(
            &path,
            "#!/bin/sh\necho \"$@\" >> \"$(dirname \"$0\")/calls\"\nif [ \"$1\" = show ]; then echo loaded; exit 0; fi\nif [ \"$1\" = reset-failed ]; then exit 0; fi\necho 'job failed' >&2\nexit 1\n",
        )
        .unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        }

        let err = restart_one(path.to_str().unwrap(), "robotd")
            .await
            .unwrap_err();
        assert!(
            format!("{err:?}").contains("restarting robotd failed"),
            "{err:?}"
        );
        assert_eq!(
            calls(dir.path()).matches("restart robotd").count(),
            2,
            "the second attempt is the whole point of clearing the counter"
        );
    }

    /// A stub that records its whole argument list and nothing else. `stub_systemctl` above answers
    /// `show` and branches on units; here the argv *is* the thing under test, so recording it is all
    /// this needs to do.
    fn stub_recorder(dir: &std::path::Path, name: &str) -> std::path::PathBuf {
        let path = dir.join(name);
        std::fs::write(
            &path,
            "#!/bin/sh\necho \"$@\" >> \"$(dirname \"$0\")/calls\"\n",
        )
        .expect("write stub");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
        }
        path
    }

    /// The deferred restarts, as an actual command line. Until this existed nothing in the repository
    /// could observe that call: the program name was hardcoded, so `--on-active` could have been
    /// wrong and every test would still pass — while on a board the only symptom is `btd` quietly
    /// never restarting, which is the exact failure `RESTART_AFTER_REPLYING` was added to fix.
    ///
    /// Four claims, and each one is load-bearing rather than incidental:
    #[tokio::test]
    async fn the_deferred_restarts_are_scheduled_as_transient_timers() {
        let dir = tempfile::tempdir().unwrap();
        let systemd_run = stub_recorder(dir.path(), "systemd-run");

        schedule_deferred_restarts(
            systemd_run.to_str().unwrap(),
            &RESTART_AFTER_REPLYING,
            &Recorder::silent(),
        )
        .await;

        let log = calls(dir.path());
        let lines: Vec<&str> = log.lines().collect();

        // One transient unit per deferred unit, not one command naming both — the same lesson as
        // `units_are_restarted_one_at_a_time`.
        assert_eq!(lines.len(), RESTART_AFTER_REPLYING.len(), "{log}");

        for (line, unit) in lines.iter().zip(RESTART_AFTER_REPLYING) {
            // The delay is what lets the reply reach the client first. Without it the engine hands
            // whoever asked a broken pipe instead of the outcome they waited minutes for.
            assert!(
                line.contains(&format!("--on-active={DEFERRED_RESTART_DELAY}")),
                "{line}"
            );
            // `systemd-run … -- systemctl restart <unit>`: the transient unit *wraps* systemctl, so
            // the restart runs outside updaterd's cgroup and survives its own parent being killed.
            assert!(
                line.ends_with(&format!("-- systemctl restart {unit}")),
                "{line}"
            );
        }

        // Both, and by name. A list that quietly lost one would leave that daemon on the old binary
        // until the next `updaterd` start noticed.
        assert!(log.contains("restart updaterd"), "{log}");
        assert!(log.contains("restart btd"), "{log}");
    }

    /// A stale unit reported by an already-current apply is scheduled by name, and only it.
    ///
    /// The pair an `Applied` owes is fixed; this list is not, so the two cases cannot share one
    /// assertion. `configd` here rather than `updaterd` for a reason that is not arbitrary: it proves
    /// the scheduler is driven by the outcome's list and not by `RESTART_AFTER_REPLYING`, which is the
    /// mistake this generalisation makes possible.
    #[tokio::test]
    async fn the_stale_units_are_the_ones_scheduled() {
        let dir = tempfile::tempdir().unwrap();
        let systemd_run = stub_recorder(dir.path(), "systemd-run");

        schedule_deferred_restarts(
            systemd_run.to_str().unwrap(),
            &["configd"],
            &Recorder::silent(),
        )
        .await;

        let log = calls(dir.path());
        assert!(log.ends_with("-- systemctl restart configd\n"), "{log}");
        assert_eq!(log.lines().count(), 1, "{log}");
    }

    fn already_current(stale: &[&str]) -> Result<ApplyResult, Error> {
        Ok(ApplyResult::AlreadyCurrent {
            version: semver::Version::parse("0.4.0").expect("a test version"),
            stale: stale.iter().map(|u| (*u).to_owned()).collect(),
        })
    }

    /// What each outcome owes, which is the part a board cannot be made to demonstrate.
    ///
    /// The third case is the one this exists for, and it is the whole reason `apply` stopped being
    /// inert: a stale `updaterd` is the skew `reconcile` refuses to repair, so an operator running
    /// `apply` is the only thing that ever reaches it.
    #[test]
    fn which_outcomes_owe_a_restart() {
        let applied = Ok(ApplyResult::Applied {
            from: None,
            to: semver::Version::parse("0.4.0").expect("a test version"),
        });
        assert_eq!(restarts_owed(&applied), RESTART_AFTER_REPLYING.to_vec());

        // Nothing installed, nothing skewed: the ordinary already-current, and it must stay silent.
        // Restarting a healthy robot's daemons because someone asked for a release it already has
        // would be a worse command than the inert one.
        assert!(restarts_owed(&already_current(&[])).is_empty());

        assert_eq!(restarts_owed(&already_current(&["updaterd"])), ["updaterd"]);
        assert_eq!(
            restarts_owed(&already_current(&["configd", "updaterd"])),
            ["configd", "updaterd"]
        );

        // A rollback leaves the resident `updaterd` already matching `current`, so there is nothing
        // to fix and a restart would be churn.
        let rolled_back = Ok(ApplyResult::RolledBack {
            attempted: semver::Version::parse("0.5.0").expect("a test version"),
            reverted_to: None,
            reason: "the gate".to_owned(),
        });
        assert!(restarts_owed(&rolled_back).is_empty());
        assert!(restarts_owed(&Err(Error::Busy)).is_empty());
    }

    /// Scheduling that fails must not propagate. The update is already committed and journalled by
    /// this point, so an unschedulable restart cannot be allowed to report a good update as failed —
    /// `reconcile` picks it up at the next start instead.
    #[tokio::test]
    async fn a_scheduler_that_cannot_be_run_is_not_an_error() {
        let dir = tempfile::tempdir().unwrap();
        // Nothing at this path, so the spawn fails outright.
        schedule_deferred_restarts(
            dir.path().join("absent").to_str().unwrap(),
            &RESTART_AFTER_REPLYING,
            &Recorder::silent(),
        )
        .await;
    }

    /// One invocation per unit, which is the fix. `systemctl restart a b` fails as a whole when
    /// either unit is unknown — and fails *without* restarting the one that exists.
    #[tokio::test]
    async fn units_are_restarted_one_at_a_time() {
        let dir = tempfile::tempdir().unwrap();
        let systemctl = stub_systemctl(dir.path(), &["robotd", "configd"]);

        for unit in ["robotd", "configd"] {
            restart_one(systemctl.to_str().unwrap(), unit)
                .await
                .unwrap();
        }

        let log = calls(dir.path());
        assert!(log.contains("restart robotd"), "{log}");
        assert!(log.contains("restart configd"), "{log}");
        // Never both in one command, which is what broke.
        assert!(!log.contains("restart robotd configd"), "{log}");
    }
}

/// How much of a download's chatter reaches whoever is watching.
///
/// Unit tests with an injected clock rather than a real download: the property under test is a
/// count of notifications per second, and sleeping to observe it would make the suite slower and
/// the assertion flakier.
#[cfg(test)]
mod download_progress {
    use super::*;

    /// The headline: thousands of chunk reports become at most a hundred and one notifications,
    /// which is what makes the stream carryable over a 20-byte BLE pipe. Before this, a phone
    /// received an arbitrary subset of them — `btd` drops progress when the client falls behind —
    /// and showed a bar that jumped 12 → 61 → 34.
    #[test]
    fn a_whole_download_publishes_at_most_one_notification_per_percent() {
        let start = tokio::time::Instant::now();
        let mut gate = DownloadProgress::started(start);

        // 5000 chunks over 20 seconds: a release artifact on a decent connection.
        let total = 5000u64;
        let mut published = 0;
        for chunk in 1..=total {
            let now = start + Duration::from_millis(chunk * 4);
            let percent = Some(((chunk * 100) / total) as u8);
            if gate.admit(percent, now) {
                published += 1;
            }
        }

        assert!(
            (1..=101).contains(&published),
            "published {published} of {total} reports"
        );
    }

    /// And the rate is bounded, not only the count. A small artifact that downloads in a second
    /// would otherwise send a hundred notifications into that second, which over BLE is the same
    /// flood in less time.
    #[test]
    fn nothing_is_published_more_often_than_four_times_a_second() {
        let start = tokio::time::Instant::now();
        let mut gate = DownloadProgress::started(start);

        let mut published = 0;
        for chunk in 1..=1000u64 {
            let now = start + Duration::from_millis(chunk);
            if gate.admit(Some((chunk / 10) as u8), now) {
                published += 1;
            }
        }

        assert!(published <= 4, "{published} notifications in one second");
    }

    /// A percent that does not move is not news. This is what collapses the tail of a download,
    /// where many chunks land on the same whole percent.
    #[test]
    fn an_unchanged_percent_is_never_republished() {
        let start = tokio::time::Instant::now();
        let mut gate = DownloadProgress::started(start);

        assert!(
            !gate.admit(Some(0), start + Duration::from_secs(1)),
            "0% was already published by the caller"
        );
        assert!(gate.admit(Some(1), start + Duration::from_secs(2)));
        assert!(!gate.admit(Some(1), start + Duration::from_secs(3)));
        assert!(gate.admit(Some(2), start + Duration::from_secs(4)));
    }

    /// The last number always gets out.
    ///
    /// Without this a download whose final percent change lands inside the gap — every download
    /// that finishes quickly — visibly stops at 97% and then jumps to the next phase. The same
    /// concern made the pump `drop` its sender rather than `abort` the task.
    #[test]
    fn a_percent_held_back_by_the_gap_is_published_when_the_download_ends() {
        let start = tokio::time::Instant::now();
        let mut gate = DownloadProgress::started(start);

        assert!(gate.admit(Some(96), start + Duration::from_secs(1)));
        assert!(!gate.admit(Some(97), start + Duration::from_millis(1050)));
        assert!(!gate.admit(Some(100), start + Duration::from_millis(1100)));

        assert_eq!(gate.flush(), Some(100), "the download ended at 100%");
        assert_eq!(gate.flush(), None, "and only once");
    }

    /// A published percent leaves nothing to flush, so the end of a download does not repeat it.
    #[test]
    fn nothing_is_held_when_the_last_report_was_published() {
        let start = tokio::time::Instant::now();
        let mut gate = DownloadProgress::started(start);

        assert!(gate.admit(Some(100), start + Duration::from_secs(1)));
        assert_eq!(gate.flush(), None);
    }

    /// A mirror that sends no `Content-Length` gives every report the same content — "still
    /// downloading" — so the gap is the only thing rationing them, and there is no number to
    /// hold back at the end.
    #[test]
    fn with_no_total_the_gap_alone_rations_the_stream() {
        let start = tokio::time::Instant::now();
        let mut gate = DownloadProgress::started(start);

        assert!(!gate.admit(None, start + Duration::from_millis(10)));
        assert!(gate.admit(None, start + Duration::from_millis(300)));
        assert!(!gate.admit(None, start + Duration::from_millis(400)));
        assert!(gate.admit(None, start + Duration::from_millis(600)));
        assert_eq!(gate.flush(), None);
    }
}
