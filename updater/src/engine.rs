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

use std::path::Path;
use std::time::Duration;

use crate::config::{ApplyAction, ComponentConfig, Config, HealthCheck};
use crate::faults::Faults;
use crate::journal::{BootCounter, Journal, PendingUpdate, Pins, UpdateLock, now_unix};
use crate::manifest::{Capabilities, Compatibility, Manifest};
use crate::proto::{
    ApplyResult, CheckResult, ComponentId, ComponentStatus, InstalledRelease, LogEntry, Outcome,
    Phase, Progress,
};
use crate::robot::RobotClient;
use crate::store::Store;
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

/// Interval between health probes while the gate is open.
const HEALTH_POLL_INTERVAL: Duration = Duration::from_millis(500);

/// Headroom multiplier over the artifact size: download + extracted copy + slack.
const SPACE_MULTIPLIER: u64 = 3;

/// Space demanded when a manifest omits `size`. Not a real estimate — just enough
/// that the check cannot silently become a no-op.
const MIN_REQUIRED_BYTES: u64 = 32 * 1024 * 1024;

/// Entries retained in the update log.
const LOG_CAPACITY: usize = 200;

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
}

#[derive(Debug, Clone, Default)]
pub struct ApplyOptions {
    pub dry_run: bool,
    pub interrupt_sessions: bool,
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
        })
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

        let emit = |phase: Phase, percent: Option<u8>| {
            let _ = progress.send(Progress {
                component: ComponentId::new(component),
                phase,
                percent,
                detail: None,
            });
        };

        let outcome = self
            .apply_inner(component, &cfg, &store, target, &options, &progress, &emit)
            .await;

        // Every operation logs through `record`, which owns what `to` means per outcome.
        self.record(component, installed, &outcome);

        // The lock is released before anything is spawned, and that ordering is load-bearing: a
        // fork duplicates every open descriptor in the process, so spawning while holding the
        // update lock hands a copy of it to the child — and in a test binary running engines in
        // parallel, copies of *other* engines' locks too. It surfaced as unrelated operations
        // failing with `Busy`. Nothing below this point touches the store.
        drop(lock);
        schedule_restarts_if_applied(&outcome).await;
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
        progress: &ProgressTx,
        emit: &impl Fn(Phase, Option<u8>),
    ) -> Result<ApplyResult, Error> {
        let installed = store.current()?;
        let source = source::from_config(&cfg.source);

        // 0. Environment preflight, *before* touching the network. The manifest
        //    fetch is HTTPS, and on a board with no battery-backed RTC it fails
        //    certificate-date validation with an opaque TLS error — the clock check
        //    exists precisely to diagnose that, so it has to run first.
        emit(Phase::Preflight, None);
        self.preflight(None, options, store).await?;

        // 1. Manifest, and its signature. Nothing else happens until this passes.
        emit(Phase::Checking, None);
        let signed = match &target {
            crate::proto::Target::Latest => source.latest_manifest().await?,
            crate::proto::Target::Exact(v) => source.manifest_for(v).await?,
            crate::proto::Target::Ref(git_ref) => source.manifest_at_ref(git_ref).await?,
            crate::proto::Target::Staging => source.staging_manifest().await?,
            crate::proto::Target::StagingExact(v) => source.staging_manifest_for(v).await?,
        };
        self.verify_manifest(&signed)?;
        let manifest = signed.parsed.clone();

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
            return Ok(ApplyResult::AlreadyCurrent {
                version: manifest.version,
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
        // The two `Staging` targets are unguarded for the `Exact` reason and not the `Ref`
        // one: a candidate carries a plain version that normally sorts *above* the installed
        // release, so the guard would rarely fire — but when it did, it would be refusing a
        // reinstall of the candidate a board had just rolled back from, which is exactly the
        // move someone reaches for while investigating that rollback.
        if matches!(target, crate::proto::Target::Latest)
            && let Some(installed) = &installed
            && manifest.version < *installed
        {
            return Err(Error::WouldDowngrade {
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
        emit(Phase::Preflight, None);
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
                progress,
                emit,
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
        progress: &ProgressTx,
        emit: &impl Fn(Phase, Option<u8>),
    ) -> Result<ApplyResult, Error> {
        let previous = store.current()?;

        emit(Phase::Downloading, Some(0));
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<(u64, Option<u64>)>();
        // The source takes a clone; this handle exists so the channel closes exactly
        // when we say so, letting the pump drain rather than be aborted.
        let tx_keepalive = tx.clone();
        let pump = {
            let progress = progress.clone();
            let component = component.to_owned();
            tokio::spawn(async move {
                while let Some((done, total)) = rx.recv().await {
                    let percent = total
                        .filter(|t| *t > 0)
                        .map(|t| ((done.min(t) * 100) / t) as u8);
                    let _ = progress.send(Progress {
                        component: ComponentId::new(component.clone()),
                        phase: Phase::Downloading,
                        percent,
                        detail: None,
                    });
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
        emit(Phase::Verifying, None);
        let artifact = fetched.artifact.clone();
        let expected = manifest.sha256.clone();
        blocking(move || verify::verify_sha256(&artifact, &expected)).await?;

        let signature = std::fs::read(&fetched.signature).map_err(|e| Error::Io {
            path: fetched.signature.clone(),
            source: e,
        })?;
        let keys = std::sync::Arc::clone(&self.keys);
        let artifact = fetched.artifact.clone();
        blocking(move || keys.verify_file(&artifact, &signature).map(|_| ())).await?;

        // 5. Extract to the side, never over a live path. Also CPU-bound (zstd).
        emit(Phase::Extracting, None);
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

        if options.dry_run {
            return Ok(ApplyResult::DryRunPassed {
                candidate: manifest.version.clone(),
            });
        }

        // 6. Pre-install hook, before the release becomes live.
        emit(Phase::RunningPreHook, None);
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
        hooks::run(extract_dir, hooks::HookKind::PreInstall, &ctx, HOOK_TIMEOUT).await?;

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

        emit(Phase::Swapping, None);
        store.swap_to(&manifest.version)?;

        if self.faults.abort_after_swap {
            // Simulates `kill -9` immediately after the swap: the symlink points at
            // the new release and the boot counter is still armed. Recovery is
            // `recover_on_start`'s job, which a test then exercises.
            return Err(Error::Internal("simulated abort after swap".into()));
        }

        // 8. Everything from here rolls back on failure.
        let gate = self
            .post_swap(component, cfg, store, &ctx, &release_dir, emit)
            .await;

        match gate {
            Ok(()) => {
                emit(Phase::Committing, None);
                self.boot_counter.confirm(component)?;
                // Pruning is best-effort — the update has already succeeded — but a
                // failure must be visible, or a robot slowly filling its eMMC looks
                // perfectly healthy.
                match store.prune(cfg.keep_previous, cfg.golden.as_ref()) {
                    Ok(removed) if !removed.is_empty() => {
                        tracing::info!(?removed, "pruned old releases");
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
                emit(Phase::RollingBack, None);
                match self
                    .rollback_to(component, cfg, store, previous.as_ref())
                    .await?
                {
                    Some(reverted) => Ok(ApplyResult::RolledBack {
                        attempted: manifest.version.clone(),
                        reverted_to: Some(reverted),
                        reason: reason.to_string(),
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
        emit: &impl Fn(Phase, Option<u8>),
    ) -> Result<(), Error> {
        emit(Phase::RunningPostHook, None);
        if self.faults.fail_post_hook {
            return Err(Error::Hook {
                hook: hooks::POST_INSTALL.into(),
                detail: "injected failure".into(),
            });
        }
        hooks::run(release_dir, hooks::HookKind::PostInstall, ctx, HOOK_TIMEOUT).await?;

        emit(Phase::Applying, None);
        self.run_apply_action(&cfg.on_apply, release_dir).await?;

        // Only where daemons are actually being replaced. `ApplyAction::None` means this component
        // has none to restart — a model, or the bootstrap install, which forces it precisely
        // because nothing is installed yet and there is no running daemon to make stale.
        if matches!(cfg.on_apply, ApplyAction::Restart { .. }) {
            self_test_updaterd(release_dir).await?;
        }

        emit(Phase::HealthGate, None);
        self.health_gate(cfg).await
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

    async fn rollback_to(
        &self,
        component: &str,
        cfg: &ComponentConfig,
        store: &Store,
        previous: Option<&semver::Version>,
    ) -> Result<Option<semver::Version>, Error> {
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

        store
            .swap_to(previous)
            .map_err(|e| Error::RollbackFailed(e.to_string()))?;
        self.boot_counter
            .confirm(component)
            .map_err(|e| Error::RollbackFailed(e.to_string()))?;
        self.run_apply_action(&cfg.on_apply, &store.release_dir(previous))
            .await
            .map_err(|e| Error::RollbackFailed(e.to_string()))?;

        Ok(Some(previous.clone()))
    }

    // ── explicit transitions ─────────────────────────────────────────────────

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

        self.transition_to(component, &cfg, &store, &previous, current)
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

        self.transition_to(component, &cfg, &store, &golden, current)
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
            return Ok(ApplyResult::AlreadyCurrent {
                version: version.clone(),
            });
        }

        // `select` can move to a release carrying newer binaries, so the deferred units are as
        // stale afterwards as they are after an `apply`. Lock released first, as above.
        let outcome = self
            .transition_to(component, &cfg, &store, version, current)
            .await;
        drop(lock);
        schedule_restarts_if_applied(&outcome).await;
        outcome
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
    ) -> Result<ApplyResult, Error> {
        // Validate *before* arming. Arming for a version that then fails to link
        // would leave a trial referring to something never live, which a later boot
        // would "recover" from with a spurious rollback and a bogus log entry.
        if !store.release_dir(to).is_dir() {
            return Err(Error::NotInstalled {
                component: component.to_owned(),
                version: to.clone(),
            });
        }

        // Armed before the swap, so a crash in between is still recoverable.
        self.boot_counter.arm(&PendingUpdate {
            component: component.to_owned(),
            version: to.clone(),
            previous: from.clone(),
            boots: 0,
        })?;

        // From here the trial is armed, so any early return must disarm it —
        // otherwise a later boot reverts an update that never went live.
        if let Err(e) = store.swap_to(to) {
            let _ = self.boot_counter.confirm(component);
            return Err(e);
        }
        if let Err(e) = self
            .run_apply_action(&cfg.on_apply, &store.release_dir(to))
            .await
        {
            let _ = self.boot_counter.confirm(component);
            return Err(e);
        }

        let outcome = match self.health_gate(cfg).await {
            Ok(()) => {
                self.boot_counter.confirm(component)?;
                Ok(ApplyResult::Applied {
                    from: from.clone(),
                    to: to.clone(),
                })
            }
            Err(reason) => {
                match self
                    .rollback_to(component, cfg, store, from.as_ref())
                    .await?
                {
                    Some(reverted) => Ok(ApplyResult::RolledBack {
                        attempted: to.clone(),
                        reverted_to: Some(reverted),
                        reason: reason.to_string(),
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
        self.record(component, from, &outcome);
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
    ) {
        let (to, outcome) = match outcome {
            Ok(ApplyResult::Applied { to, .. }) => (Some(to.clone()), Outcome::Success),
            Ok(ApplyResult::RolledBack {
                attempted, reason, ..
            }) => (
                // The version that failed — see the doc comment.
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
            Ok(ApplyResult::AlreadyCurrent { .. } | ApplyResult::DryRunPassed { .. }) => return,
            Err(e) => (
                None,
                Outcome::Aborted {
                    reason: e.to_string(),
                },
            ),
        };

        let entry = LogEntry {
            at: now_unix(),
            component: ComponentId::new(component),
            from,
            to,
            outcome,
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
    pub async fn recover_on_start(&mut self) -> Result<Vec<ApplyResult>, Error> {
        for name in self.config.components.keys().cloned().collect::<Vec<_>>() {
            if let Ok(store) = self.store(&name) {
                let _ = store.clean_staging();
            }
        }

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

            tracing::warn!(
                component = %pending.component,
                version = %pending.version,
                boots = pending.boots,
                "pending update never confirmed healthy; reverting"
            );

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
            let store = self.store(&pending.component)?;

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
                .rollback_to(&pending.component, &cfg, &store, target.as_ref())
                .await?;

            let reason = format!("never reported healthy across {} boots", pending.boots);

            let outcome = match reverted {
                Some(reverted) => ApplyResult::RolledBack {
                    attempted: pending.version.clone(),
                    reverted_to: Some(reverted),
                    reason: reason.clone(),
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
    ) -> Result<(), Error> {
        match action {
            ApplyAction::None => Ok(()),
            ApplyAction::Restart { units } => {
                for unit in units_to_restart(release_dir, units) {
                    restart_one(SYSTEMCTL, &unit).await?;
                }
                Ok(())
            }
            ApplyAction::Reload { unit, signal } => {
                let mut c = tokio::process::Command::new(SYSTEMCTL);
                c.arg("kill").arg(format!("--signal={signal}")).arg(unit);
                run_systemctl(c, "apply action").await
            }
        }
    }

    /// Wait for the new release to report healthy.
    ///
    /// A timeout is a **failure**: unproven is not healthy, or auto-rollback would
    /// never fire on a release that hangs.
    async fn health_gate(&self, cfg: &ComponentConfig) -> Result<(), Error> {
        if self.faults.fail_health {
            return Err(Error::Health("injected health failure".into()));
        }

        let Some(timeout) = cfg.health.timeout() else {
            // HealthCheck::None — nothing to gate on.
            return Ok(());
        };

        if self.faults.hang_health {
            tokio::time::sleep(timeout).await;
            return Err(Error::Health(format!(
                "health probe did not answer within {}s",
                timeout.as_secs()
            )));
        }

        match &cfg.health {
            HealthCheck::None => Ok(()),
            HealthCheck::Socket { .. } => {
                // The socket path lives in `Config::robot_socket` and is used to build
                // the RobotClient in `main`; here we just ask the client.
                let deadline = tokio::time::Instant::now() + timeout;
                let mut last = String::from("no answer");
                while tokio::time::Instant::now() < deadline {
                    match self.robot.health(ROBOT_QUERY_TIMEOUT).await {
                        crate::robot::Health::Healthy => return Ok(()),
                        // Passes. Logged at warn, not swallowed: committing a release onto a
                        // robot that cannot move is the right call, but nobody should have to
                        // guess afterwards that that is what happened.
                        crate::robot::Health::Degraded(reason) => {
                            tracing::warn!(
                                reason = %reason,
                                "committing: the robot is degraded for a reason this release \
                                 cannot have caused and a rollback cannot fix"
                            );
                            return Ok(());
                        }
                        crate::robot::Health::Unhealthy(reason) => last = reason,
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
                command.args(args).kill_on_drop(true);
                let output = tokio::time::timeout(timeout, command.output())
                    .await
                    .map_err(|_| {
                        Error::Health(format!("probe timed out after {}s", timeout.as_secs()))
                    })?
                    .map_err(|e| Error::Health(format!("could not run probe: {e}")))?;
                if output.status.success() {
                    Ok(())
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
    fn verify_manifest(&self, signed: &source::SignedBytes<Manifest>) -> Result<(), Error> {
        self.keys
            .verify_bytes(&signed.bytes, &signed.signature)
            .map(|_| ())
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

/// The program that drives units. A constant so tests can substitute a stub for it.
const SYSTEMCTL: &str = "systemctl";

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
async fn self_test_updaterd(release_dir: &Path) -> Result<(), Error> {
    let binary = release_dir.join("bin").join("updaterd");
    if !binary.is_file() {
        return Ok(());
    }

    let mut command = tokio::process::Command::new(&binary);
    command.arg("--self-test");

    let output = match tokio::time::timeout(SELF_TEST_TIMEOUT, command.output()).await {
        Ok(Ok(output)) => output,
        // Could not be executed at all: the wrong architecture, a missing interpreter, a corrupt
        // file. Exactly what this exists to catch.
        Ok(Err(e)) => {
            return Err(Error::SelfTest(format!(
                "could not run {}: {e}",
                binary.display()
            )));
        }
        Err(_) => {
            return Err(Error::SelfTest(format!(
                "{} did not finish within {SELF_TEST_TIMEOUT:?}",
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

/// Schedule the deferred restarts when an operation actually moved to a new release.
///
/// Only on `Applied`. A rollback leaves the resident `updaterd` already matching `current` — it was
/// never restarted, so it is still the binary belonging to the release being returned to — and
/// restarting there would be churn with nothing to fix.
async fn schedule_restarts_if_applied(outcome: &Result<ApplyResult, Error>) {
    if matches!(outcome, Ok(ApplyResult::Applied { .. })) {
        schedule_deferred_restarts().await;
    }
}

/// Restart the deferred units, detached, a few seconds from now.
///
/// `systemd-run` rather than a child process, and this is the load-bearing detail: `systemctl
/// restart updaterd` kills `updaterd`'s whole cgroup, and a child of `updaterd` is *in* that cgroup —
/// so it would be killed partway through restarting its own parent. A transient unit lives outside
/// it and survives.
///
/// Failures are logged, never returned. This runs after the update is committed and journalled; an
/// update that succeeded must not be reported as failed because a restart could not be scheduled.
/// The cost of that is the situation we already have today — a daemon running an old binary until
/// the next boot — which `robotctl version` reports.
async fn schedule_deferred_restarts() {
    for unit in RESTART_AFTER_REPLYING {
        let mut command = tokio::process::Command::new("systemd-run");
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
        match command.status().await {
            Ok(status) if status.success() => {
                tracing::info!(unit, delay = DEFERRED_RESTART_DELAY, "restart scheduled");
            }
            Ok(status) => tracing::warn!(
                unit,
                %status,
                "could not schedule the restart; it keeps the old binary until the next boot"
            ),
            Err(e) => tracing::warn!(
                unit,
                error = %e,
                "could not run systemd-run; the unit keeps the old binary until the next boot"
            ),
        }
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
/// reports success, the daemon answers on stale code, and `apply` then says `already_current` and
/// does nothing. Four correct fixes were diagnosed as broken that way in one afternoon; see
/// `docs/project/install-path-gap.md` §4.
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
fn units_to_restart(release_dir: &Path, configured: &[String]) -> Vec<String> {
    let mut units: Vec<String> = Vec::new();

    match std::fs::read_dir(release_dir.join("systemd")) {
        Ok(entries) => {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().and_then(|e| e.to_str()) != Some("service") {
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
    units.retain(|unit| !NEVER_RESTART.contains(&unit.as_str()));
    units
}

async fn restart_one(systemctl: &str, unit: &str) -> Result<(), Error> {
    let mut c = tokio::process::Command::new(systemctl);
    c.arg("restart").arg(unit);

    match run_systemctl(c, "restart").await {
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
                Ok(())
            } else {
                Err(e)
            }
        }
    }
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

    match tokio::time::timeout(APPLY_ACTION_TIMEOUT, c.output()).await {
        Ok(Ok(output)) => String::from_utf8_lossy(&output.stdout).trim() == "not-found",
        _ => false,
    }
}

async fn run_systemctl(mut command: tokio::process::Command, what: &str) -> Result<(), Error> {
    command.kill_on_drop(true);

    let output = tokio::time::timeout(APPLY_ACTION_TIMEOUT, command.output())
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

        let err = self_test_updaterd(release.path()).await.unwrap_err();

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
        assert!(self_test_updaterd(release.path()).await.is_ok());
    }

    /// Model components ship no `updaterd`, and neither do releases predating the flag. Requiring
    /// one would make this a compatibility break rather than a safety net.
    #[tokio::test]
    async fn a_release_without_an_updaterd_passes() {
        let dir = tempfile::tempdir().unwrap();
        assert!(self_test_updaterd(dir.path()).await.is_ok());
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
            std::fs::write(
                dir.path().join("systemd").join(format!("{unit}.service")),
                "[Unit]\n",
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
        assert!(format!("{err:?}").contains("restart failed"), "{err:?}");
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
