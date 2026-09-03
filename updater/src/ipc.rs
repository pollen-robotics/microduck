//! JSON-RPC 2.0 server over a unix socket, for `robotctl` and later `btd`.
//!
//! Framing is NDJSON via `tokio_util::codec::LinesCodec`; message shapes live in
//! [`crate::proto`].
//!
//! Requirements that follow from `docs/design/architecture.md` §1.1:
//!  - Serving never depends on `robotd` being alive.
//!  - A slow or vanished client must not delay an in-flight update.
//!  - An update runs to completion even if every client disconnects — the robot
//!    pulls, so BLE dropping mid-update is normal, not an abort.
//!
//! Structure: one task per connection, and the [`Engine`] behind a mutex. A long
//! operation holds that mutex, so read-only requests use `try_lock` and fall back to
//! a cached snapshot rather than blocking — that is what keeps `status`/`subscribe`
//! answerable *during* an update.
//!
//! **Access control is the socket's file mode.** Anyone who can write to it can
//! trigger an update or a rollback, so it is created `0o660`, group-owned, and every
//! mutating request is logged with the caller's uid/pid from `SO_PEERCRED`.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};
use tokio::sync::{Mutex, broadcast, mpsc};

use crate::config::AutoApply;
use crate::engine::{ApplyOptions, Engine};
use crate::proto::{self, Call, ComponentStatus, Id, Progress, Request, Response};

/// How far a lagging subscriber may fall behind before it's dropped from the
/// broadcast. Progress is advisory: a client that can't keep up gets a gap, never
/// backpressure onto the update.
const PROGRESS_BUFFER: usize = 256;

/// Socket mode: owner and group read/write, nothing for others.
const SOCKET_MODE: u32 = 0o660;

/// Refuse absurdly long lines rather than buffering them.
const MAX_LINE: usize = 1024 * 1024;

/// Delay before the first scheduled check.
///
/// The network is often not up at boot, and a fleet restarting together would arrive
/// as a thundering herd.
const INITIAL_CHECK_DELAY: Duration = Duration::from_secs(60);

/// Who may perform mutating operations.
///
/// Two tiers. Reaching the socket at all requires its group (mode 0660); *mutating*
/// additionally requires being listed here. So support can inspect a robot without being
/// able to change it, and a BLE-facing client's group membership does not amount to "may
/// replace the firmware".
#[derive(Debug, Clone)]
pub struct PeerPolicy {
    /// The uid `updaterd` runs as. Always permitted — it can stop or replace the
    /// daemon regardless, so refusing it would protect nothing.
    owner_uid: u32,
    allow_uids: Vec<u32>,
    allow_gids: Vec<u32>,
}

impl PeerPolicy {
    pub fn new(owner_uid: u32, allow_uids: Vec<u32>, allow_gids: Vec<u32>) -> Self {
        Self {
            owner_uid,
            allow_uids,
            allow_gids,
        }
    }

    /// May this peer mutate?
    ///
    /// An *unknown* peer is denied. `peer_cred` failing is not something to shrug at
    /// when the decision is "may this trigger a firmware change".
    fn may_mutate(&self, peer: Option<&tokio::net::unix::UCred>) -> Result<(), String> {
        let Some(peer) = peer else {
            return Err("peer credentials unavailable; refusing a mutating request".into());
        };

        if peer.uid() == self.owner_uid
            || self.allow_uids.contains(&peer.uid())
            || self.allow_gids.contains(&peer.gid())
        {
            return Ok(());
        }

        Err(format!(
            "uid {} / gid {} is not permitted to change this robot's software; add it to \
             allow_uids or allow_gids in updater.toml, or run as uid {}",
            peer.uid(),
            peer.gid(),
            self.owner_uid
        ))
    }
}

pub struct Server {
    /// Held by whichever task is running a mutating operation. Read-only requests
    /// use `try_lock` so they stay answerable while that happens.
    engine: Arc<Mutex<Engine>>,

    /// Last known component status, refreshed whenever the engine is obtainable, so
    /// `status` can answer during an update instead of blocking on it.
    cached_status: Arc<Mutex<Vec<ComponentStatus>>>,

    /// Latest progress per component, replayed to a client that connects
    /// mid-update.
    latest: Arc<Mutex<Vec<Progress>>>,

    progress_tx: broadcast::Sender<Progress>,

    /// Set once the socket exists, since the owning uid is read from it.
    policy: Arc<Mutex<Option<PeerPolicy>>>,

    allow_uids: Vec<u32>,
    allow_gids: Vec<u32>,

    /// Test-only override of the owning uid; `None` means "read it from the socket".
    forced_owner_uid: Option<u32>,

    /// Which Hugging Face account this robot belongs to.
    ///
    /// Not part of the engine: it shares nothing with an update but the network stack, and it
    /// must stay answerable *during* one — `account.status` is polled while a login is in
    /// flight, and queueing that behind a download would make a wizard look stuck.
    account: Arc<crate::account::Account>,
}

impl Server {
    pub fn new(engine: Engine) -> Self {
        Self::with_policy(engine, Vec::new(), Vec::new())
    }

    /// As [`Self::new`], with uids/gids permitted to mutate beyond the owning uid.
    pub fn with_policy(engine: Engine, allow_uids: Vec<u32>, allow_gids: Vec<u32>) -> Self {
        let (progress_tx, _) = broadcast::channel(PROGRESS_BUFFER);
        Self {
            engine: Arc::new(Mutex::new(engine)),
            cached_status: Arc::new(Mutex::new(Vec::new())),
            latest: Arc::new(Mutex::new(Vec::new())),
            progress_tx,
            policy: Arc::new(Mutex::new(None)),
            allow_uids,
            allow_gids,
            forced_owner_uid: None,
            account: Arc::new(crate::account::Account::new(crate::account::Store::new(
                crate::account::DEFAULT_PATH,
            ))),
        }
    }

    /// Point the account at a token file and a Hugging Face of your choosing.
    ///
    /// Only for tests, and it is not optional there: [`Self::with_policy`] uses
    /// [`crate::account::DEFAULT_PATH`], and a test that ran against it would read — or on a
    /// developer's machine try to write — the real robot's credential.
    #[doc(hidden)]
    pub fn with_account_for_test(mut self, token_path: PathBuf, endpoint: String) -> Self {
        self.account = Arc::new(crate::account::Account::with_endpoint(
            crate::account::Store::new(token_path),
            endpoint,
        ));
        self
    }

    /// Build a server with an explicit owning uid.
    ///
    /// Only for tests: normally the owning uid is read back from the socket, and a
    /// test process cannot easily *not* be the socket's owner.
    #[doc(hidden)]
    pub fn with_policy_for_test(
        engine: Engine,
        owner_uid: u32,
        allow_uids: Vec<u32>,
        allow_gids: Vec<u32>,
    ) -> Self {
        let mut server = Self::with_policy(engine, allow_uids.clone(), allow_gids.clone());
        server.forced_owner_uid = Some(owner_uid);
        server
    }

    /// Bind and serve until the process is asked to stop.
    pub async fn serve(self: Arc<Self>, socket_path: &Path) -> std::io::Result<()> {
        // A leftover socket from a killed process must never stop the recovery path
        // from coming up, so remove it rather than failing to bind.
        if socket_path.exists() {
            tracing::warn!(path = %socket_path.display(), "removing stale socket");
            let _ = std::fs::remove_file(socket_path);
        }
        if let Some(parent) = socket_path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }

        let listener = UnixListener::bind(socket_path)?;

        // Permissions are the whole access-control story here (see module docs), so
        // a failure to tighten them is fatal rather than a warning: serving a
        // world-writable update socket is worse than not serving.
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(socket_path, std::fs::Permissions::from_mode(SOCKET_MODE))?;

        // The socket's owner is our own effective uid; reading it back avoids a libc
        // dependency just to call getuid().
        let owner_uid = self.forced_owner_uid.unwrap_or_else(|| {
            std::fs::metadata(socket_path)
                .map(|m| {
                    use std::os::unix::fs::MetadataExt;
                    m.uid()
                })
                .unwrap_or(0)
        });
        *self.policy.lock().await = Some(PeerPolicy::new(
            owner_uid,
            self.allow_uids.clone(),
            self.allow_gids.clone(),
        ));

        tracing::info!(
            path = %socket_path.display(),
            mode = format!("{SOCKET_MODE:o}"),
            owner_uid,
            allow_uids = ?self.allow_uids,
            allow_gids = ?self.allow_gids,
            "serving update IPC"
        );

        loop {
            let (stream, _addr) = match listener.accept().await {
                Ok(pair) => pair,
                Err(e) => {
                    tracing::warn!(error = %e, "accept failed");
                    continue;
                }
            };
            let server = Arc::clone(&self);
            tokio::spawn(async move {
                if let Err(e) = server.handle_connection(stream).await {
                    // A client hanging up mid-request is routine, not an error worth
                    // shouting about.
                    tracing::debug!(error = %e, "connection ended");
                }
            });
        }
    }

    /// Poll every component's source on a timer, and apply what `policy` allows with no
    /// client attached.
    ///
    /// At the default [`AutoApply::Mandatory`] this is what makes `min_supported`
    /// (`docs/design/updater-design.md` §8.1) actually work. Without it the floor is inert: a
    /// robot only learns it exists when someone opens the app, which is precisely what
    /// you cannot rely on when remediating a bad release.
    ///
    /// Runs inside `updaterd` rather than as a systemd timer or cron job calling
    /// `robotctl`, and that is a correctness matter rather than a preference. An external
    /// timer would go through `update apply`, which **deliberately bypasses the
    /// `known_bad` guard** below — an operator retrying a release may have fixed the
    /// cause, so refusing them would remove the obvious way to test that. On a timer that
    /// same bypass is the bricking loop the guard exists to prevent. A cron job would
    /// therefore inherit the bypass and lose the protection: exactly the wrong half of
    /// each. It also needs the same engine mutex and progress plumbing as a
    /// client-triggered update.
    pub fn spawn_periodic_checks(
        self: &Arc<Self>,
        interval: Duration,
        policy: AutoApply,
    ) -> tokio::task::JoinHandle<()> {
        let server = Arc::clone(self);
        tokio::spawn(async move {
            // Don't check the instant we boot: the network is often not up yet, and a
            // fleet restarting together would arrive as a thundering herd.
            tokio::time::sleep(INITIAL_CHECK_DELAY).await;

            loop {
                server.check_all(policy).await;
                tokio::time::sleep(interval).await;
            }
        })
    }

    /// Keep the account token from expiring under a robot that is simply left on.
    ///
    /// Separate from [`Self::spawn_periodic_checks`] because it is unconditional: that one is
    /// off unless a `check_interval` is configured, and a robot with update checks disabled
    /// still has an account that stops working after thirty days.
    pub fn spawn_account_maintenance(self: &Arc<Self>) -> tokio::task::JoinHandle<()> {
        tokio::spawn(crate::account::maintain(Arc::clone(&self.account)))
    }

    /// One pass of the scheduler. Exposed so tests can drive it without waiting for
    /// a timer.
    #[doc(hidden)]
    pub async fn check_all_for_test(&self, policy: AutoApply) {
        self.check_all(policy).await;
    }

    async fn check_all(&self, policy: AutoApply) {
        // An update in flight already supersedes a scheduled check; queueing behind it
        // would only apply something the operator may not want.
        let components = match self.engine.try_lock() {
            Ok(engine) => engine.component_names(),
            Err(_) => {
                tracing::debug!("skipping scheduled check: an update is in progress");
                return;
            }
        };

        for component in components {
            let result = {
                let Ok(engine) = self.engine.try_lock() else {
                    return;
                };
                engine.check(&component).await
            };

            match result {
                Ok(proto::CheckResult::UpToDate { installed }) => {
                    tracing::debug!(%component, %installed, "up to date");
                }
                Ok(proto::CheckResult::Incompatible { candidate, reason }) => {
                    tracing::info!(%component, %candidate, %reason, "update not applicable");
                }
                Ok(proto::CheckResult::Available {
                    candidate,
                    mandatory,
                    ..
                }) => {
                    if !policy.permits(mandatory) {
                        if mandatory {
                            // Visible but not acted on: the operator opted out, and a
                            // silently-ignored mandatory update is worth shouting about.
                            tracing::warn!(
                                %component,
                                %candidate,
                                ?policy,
                                "a MANDATORY update is available but auto_apply does not \
                                 cover it"
                            );
                        } else {
                            tracing::info!(%component, %candidate, "update available");
                        }
                        continue;
                    }

                    // Never reapply a release this robot already rolled back from —
                    // otherwise a bad one loops forever, re-downloading and restarting every
                    // interval. An explicit `update apply` still retries, which is why a cron
                    // job driving `robotctl` is not a substitute for this scheduler
                    // (`updater-design.md` §8.1.1).
                    let known_bad = match self.engine.try_lock() {
                        Ok(engine) => engine.known_bad(&component),
                        Err(_) => return,
                    };
                    if known_bad.contains(&candidate) {
                        tracing::error!(
                            %component,
                            %candidate,
                            mandatory,
                            "release already failed its health gate on this robot; refusing to \
                             reapply it unattended. Needs a fixed release, or an explicit \
                             `robotctl update apply`."
                        );
                        continue;
                    }

                    if mandatory {
                        tracing::warn!(
                            %component,
                            %candidate,
                            "release is below the minimum supported version; applying without \
                             waiting for a client"
                        );
                    } else {
                        tracing::warn!(
                            %component,
                            %candidate,
                            "auto_apply = all; applying without waiting for a client"
                        );
                    }
                    match self.apply_unattended(&component).await {
                        Ok(outcome) => {
                            tracing::warn!(%component, ?outcome, "unattended update finished")
                        }
                        Err(e) => {
                            tracing::error!(%component, error = %e, "unattended update failed")
                        }
                    }
                }
                Err(e) => {
                    // A source being unreachable is routine on domestic wifi; it must
                    // not look like a fault.
                    tracing::info!(%component, error = %e, "scheduled check failed");
                }
            }
        }
    }

    /// Apply the latest release with no client attached.
    ///
    /// Progress still reaches `subscribe`rs and `latest`, so the app sees an
    /// unattended update in progress rather than an unexplained restart.
    async fn apply_unattended(&self, component: &str) -> Result<proto::ApplyResult, crate::Error> {
        let mut engine = self.engine.try_lock().map_err(|_| crate::Error::Busy)?;

        let (tx, rx) = mpsc::unbounded_channel::<Progress>();
        let pump = self.spawn_progress_pump(rx);

        let result = engine
            .apply(
                component,
                proto::Target::Latest,
                ApplyOptions::default(),
                tx,
            )
            .await;

        pump.abort();
        result
    }

    /// Forward engine progress to `latest` and the broadcast.
    fn spawn_progress_pump(
        &self,
        mut rx: mpsc::UnboundedReceiver<Progress>,
    ) -> tokio::task::JoinHandle<()> {
        let broadcast_tx = self.progress_tx.clone();
        let latest = Arc::clone(&self.latest);
        tokio::spawn(async move {
            while let Some(progress) = rx.recv().await {
                {
                    let mut latest = latest.lock().await;
                    match latest
                        .iter_mut()
                        .find(|e| e.component == progress.component)
                    {
                        Some(slot) => *slot = progress.clone(),
                        None => latest.push(progress.clone()),
                    }
                }
                let _ = broadcast_tx.send(progress);
            }
        })
    }

    /// Read requests, dispatch, write responses, until the peer disconnects.
    ///
    /// A disconnect mid-operation does **not** cancel the operation: the engine call
    /// is awaited here, but the update's effects are committed to disk as it goes,
    /// and boot recovery covers an interruption. See `docs/design/updater-design.md` §7.
    async fn handle_connection(self: Arc<Self>, stream: UnixStream) -> std::io::Result<()> {
        let peer = stream.peer_cred().ok();
        let (read_half, mut write_half) = stream.into_split();
        let mut lines = BufReader::new(read_half).lines();

        while let Some(line) = lines.next_line().await? {
            if line.trim().is_empty() {
                continue;
            }
            if line.len() > MAX_LINE {
                let response = Response::err(
                    None,
                    proto::Error::new(proto::code::INVALID_REQUEST, "request too large"),
                );
                write_line(&mut write_half, &response).await?;
                continue;
            }

            let request: Request = match serde_json::from_str(&line) {
                Ok(request) => request,
                Err(e) => {
                    let response = Response::err(
                        None,
                        proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
                    );
                    write_line(&mut write_half, &response).await?;
                    continue;
                }
            };

            // Notifications get no reply, per the spec.
            let Some(id) = request.id.clone() else {
                continue;
            };

            let call = match request.as_call() {
                Ok(call) => call,
                Err(e) => {
                    write_line(&mut write_half, &Response::err(Some(id), e)).await?;
                    continue;
                }
            };

            if matches!(call, Call::Subscribe) {
                // Streams until the peer goes away, so it owns the connection.
                self.stream_progress(id, &mut write_half).await?;
                continue;
            }

            let response = self.dispatch(id, call, peer, &mut write_half).await;
            write_line(&mut write_half, &response).await?;
        }
        Ok(())
    }

    async fn dispatch(
        &self,
        id: Id,
        call: Call,
        peer: Option<tokio::net::unix::UCred>,
        out: &mut tokio::net::unix::OwnedWriteHalf,
    ) -> Response {
        // One check covering every mutating call, rather than one per arm: a method added
        // to `Call::is_mutating` is authorised by construction, and a new arm here cannot
        // forget to ask.
        if call.is_mutating()
            && let Err(denied) = self.authorise(&id, &call, peer).await
        {
            return denied;
        }

        match call {
            Call::Hello(params) => {
                if params.api_version != proto::API_VERSION {
                    note_version_skew(params.api_version);
                }
                Response::ok(
                    Some(id),
                    &proto::HelloResult {
                        api_version: proto::API_VERSION,
                        daemon_version: semver::Version::parse(env!("CARGO_PKG_VERSION")).ok(),
                        revision: proto::build_info!().revision.map(str::to_owned),
                    },
                )
            }

            // ── read-only ────────────────────────────────────────────────────
            Call::Status => match self.status().await {
                Ok(status) => Response::ok(Some(id), &status),
                Err(e) => Response::err(Some(id), e.to_rpc_error()),
            },
            Call::Log(params) => self
                .with_engine(id.clone(), |engine| engine.log(params.limit))
                .await
                .map_or_else(|e| e, |v| Response::ok(Some(id), &v)),
            Call::Show(params) => self
                .with_engine(id.clone(), |engine| engine.show(params.run))
                .await
                .map_or_else(|e| e, |v| Response::ok(Some(id), &v)),
            Call::ListInstalled(params) => self
                .with_engine(id.clone(), |engine| {
                    engine.list_installed(params.component.as_str())
                })
                .await
                .map_or_else(|e| e, |v| Response::ok(Some(id), &v)),
            // `try_lock`, like every other read here, and for a reason the header states:
            // a request that blocks on the engine mutex is answered whenever the update
            // finishes, which is minutes for a daemon release. A client asking "is there an
            // update?" during one has an immediate answer — there is one running — and getting
            // `BUSY` back at once is what lets it say so. Blocking instead produced a spinner
            // indistinguishable from a robot that had stopped answering.
            Call::Check(params) => match self.engine.try_lock() {
                Ok(engine) => match engine.check(params.component.as_str()).await {
                    Ok(result) => Response::ok(Some(id), &result),
                    Err(e) => Response::err(Some(id), e.to_rpc_error()),
                },
                Err(_) => Response::err(
                    Some(id),
                    proto::Error::new(proto::code::BUSY, "an update is in progress; retry shortly"),
                ),
            },

            // ── mutating ─────────────────────────────────────────────────────
            Call::Apply(params) => {
                let component = params.component.0.clone();
                // The same three numbers `authorise` has already logged, kept for the transcript.
                // A month later "who ran this?" has no other answer: the journal line that carried
                // them may be gone, and the transcript that outlived it is where the question gets
                // asked.
                let requested_by = peer.map(|p| {
                    format!(
                        "uid={} gid={} pid={}",
                        p.uid(),
                        p.gid(),
                        p.pid().map(|pid| pid.to_string()).unwrap_or_else(|| "?".into())
                    )
                });
                self.run_mutating(id, out, move |engine, tx| {
                    Box::pin(async move {
                        engine
                            .apply(
                                &component,
                                params.target,
                                ApplyOptions {
                                    dry_run: params.options.dry_run,
                                    interrupt_sessions: params.options.interrupt_sessions,
                                    from_dir: params.options.from_dir.map(std::path::PathBuf::from),
                                    requested_by,
                                },
                                tx,
                            )
                            .await
                    })
                })
                .await
            }
            // No engine lock: it reads a directory and makes one HTTP request, touches no engine
            // state, and a question about whether a newer gait exists should stay answerable
            // while an unrelated update runs.
            Call::PolicyCheck => {
                Response::ok(Some(id), &crate::policy::check(
                    std::path::Path::new(crate::policy::POLICY_ROOT),
                ).await)
            }
            // `try_lock` and the same `BUSY` every other mutation answers with. Swapping the
            // policy set while a release is being installed would have two things rewriting what
            // the robot runs at once, and the second one to finish would win by accident.
            Call::PolicyInstall(params) => {
                let engine = match self.engine.try_lock() {
                    Ok(engine) => engine,
                    Err(_) => {
                        return Response::err(
                            Some(id),
                            proto::Error::new(
                                proto::code::BUSY,
                                "an update is in progress; retry shortly",
                            ),
                        );
                    }
                };
                match engine.install_policies(params.version.as_deref()).await {
                    Ok(result) => Response::ok(Some(id), &result),
                    Err(e) => Response::err(Some(id), e.to_rpc_error()),
                }
            }
            // ── account.* ───────────────────────────────────────────────────────
            //
            // None of the three touches the engine, so none takes its lock: a login is an HTTP
            // round trip and a file, and both are outside every release directory. That is also
            // what makes `status` answerable during an update, which it has to be.
            Call::AccountLogin(params) => {
                match Arc::clone(&self.account).login(params.force).await {
                    Ok(login) => Response::ok(Some(id), &login),
                    Err(e) => Response::err(Some(id), e.to_rpc_error()),
                }
            }
            Call::AccountStatus => Response::ok(Some(id), &self.account.status().await),
            Call::AccountLogout => match self.account.logout().await {
                Ok(result) => Response::ok(Some(id), &result),
                Err(e) => Response::err(Some(id), e.to_rpc_error()),
            },

            // Read-only and no engine lock: asking the Hub what exists changes nothing here.
            Call::PolicySearch(params) => match crate::policy::search(&params.query).await {
                Ok(result) => Response::ok(Some(id), &result),
                Err(e) => Response::err(Some(id), e.to_rpc_error()),
            },
            // `try_lock` like the other mutations. Nothing it writes collides with an update —
            // the library is outside every release directory — but it asks `robotd` for the
            // model API, and doing that mid-swap gets an answer about whichever daemon happens
            // to be running at that instant.
            Call::PolicyFetch(params) => {
                let engine = match self.engine.try_lock() {
                    Ok(engine) => engine,
                    Err(_) => {
                        return Response::err(
                            Some(id),
                            proto::Error::new(
                                proto::code::BUSY,
                                "an update is in progress; retry shortly",
                            ),
                        );
                    }
                };
                match engine
                    .fetch_policy(
                        &params.repo,
                        params.revision.as_deref(),
                        params.file.as_deref(),
                    )
                    .await
                {
                    Ok(result) => Response::ok(Some(id), &result),
                    Err(e) => Response::err(Some(id), e.to_rpc_error()),
                }
            }
            Call::Rollback(params) => {
                let component = params.component.0;
                self.run_mutating(id, out, move |engine, _tx| {
                    Box::pin(async move { engine.rollback(&component).await })
                })
                .await
            }
            Call::ResetToGolden(params) => {
                let component = params.component.0;
                self.run_mutating(id, out, move |engine, _tx| {
                    Box::pin(async move { engine.reset_to_golden(&component).await })
                })
                .await
            }
            Call::Select(params) => {
                let component = params.component.0;
                let version = params.version;
                self.run_mutating(id, out, move |engine, _tx| {
                    Box::pin(async move { engine.select(&component, &version).await })
                })
                .await
            }
            // Also `try_lock`, and the same `BUSY` every other mutation answers with. Pinning
            // during an update is a request about which version may be installed, made while one
            // is being installed: waiting for the answer to become moot is worse than saying so.
            Call::Pin(params) => match self.engine.try_lock() {
                Ok(mut engine) => match engine.pin(params.component.as_str(), params.version).await {
                    Ok(()) => Response::ok(Some(id), &serde_json::json!({})),
                    Err(e) => Response::err(Some(id), e.to_rpc_error()),
                },
                Err(_) => Response::err(
                    Some(id),
                    proto::Error::new(proto::code::BUSY, "another update is already in progress"),
                ),
            },

            // Owned by `handle_connection`, which hands the whole connection to
            // `stream_progress` instead of answering once.
            Call::Subscribe => Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "subscribe is served on the connection, not dispatched",
                ),
            ),

            // `robot.*` belongs to `robotd`. Reaching this means a client aimed the wrong
            // socket, so name that rather than reporting a generic failure.
            Call::RobotSafeToRestart
            | Call::RobotHealth
            | Call::RobotModelApi
            | Call::RobotRemoteSessionActive
            | Call::RobotMove(_)
            | Call::RobotHead(_)
            | Call::RobotLook(_)
            | Call::RobotStop
            | Call::RobotEnable(_)
            | Call::RobotInit
            | Call::RobotRelax
            | Call::RobotDo(_)
            | Call::RobotSound(_)
            | Call::RobotPose(_)
            | Call::RobotMouth(_)
            | Call::RobotTheremin(_)
            | Call::RobotChorale(_)
            | Call::ChoraleSubscribe
            | Call::ChoraleBeaconSet(_)
            | Call::ChoraleHeard(_)
            | Call::RobotShutdown
            | Call::RobotMode
            | Call::RobotSetMode(_)
            | Call::RobotPolicies
            | Call::RobotLoadPolicy(_)
            | Call::RobotReloadPolicies
            | Call::RobotSubscribe(_) => Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::METHOD_NOT_FOUND,
                    "robot.* is served by robotd, not updaterd",
                ),
            ),

            // `net.*` and `system.*` belong to `configd`, for the reason that service exists:
            // config must be reachable when the robot is dead, and wiring it into the update
            // engine would put provisioning behind the engine's lock.
            Call::NetStatus
            | Call::NetScan
            | Call::NetConnect(_)
            | Call::NetForget(_)
            | Call::SystemInfo
            | Call::SystemServices
            | Call::SystemSetName(_)
            | Call::SystemReboot
            | Call::SystemPairingPin
            | Call::SystemSetPairingPin(_)
            // `pad.*` is `configd`'s for the same reason: pairing a gamepad is a root-only question
            // about the radio's configuration, and it must be answerable when the robot is not
            // working.
            | Call::PadStatus
            | Call::PadPair(_)
            | Call::PadForget(_)
            // The two exceptions in that namespace go to `robotd` rather than `configd` — a
            // binding needs the skill list to check a name against — but from here the answer is
            // the same either way: not this daemon.
            | Call::PadBindings
            | Call::PadBind(_)
            | Call::RobotSkills
            | Call::RobotSetSkill(_)
            | Call::RobotRemoveSkill(_) => Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::METHOD_NOT_FOUND,
                    "net.*, system.* and pad.* are served by configd and robotd, not updaterd",
                ),
            ),

            // `pad.input` is the one call in that namespace `configd` does *not* answer, so it
            // cannot share the arm above: sending someone to `configd` for it would cost them the
            // same wrong-socket round trip this arm exists to save.
            Call::PadInput => Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::METHOD_NOT_FOUND,
                    "pad.input is served by padd itself, on /run/padd/pad.sock — unlike the rest \
                     of pad.*, which configd answers",
                ),
            ),

            // Same story one namespace over: `tofd` owns the sensor and answers for it.
            Call::TofStream => Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::METHOD_NOT_FOUND,
                    "tof.stream is served by tofd itself, on /run/tofd/tof.sock",
                ),
            ),

            // Answered by the transport that received it — `btd` checks the PIN itself and never
            // forwards this. Reaching updaterd means a client sent it to the wrong socket, or over
            // a transport that has no PIN gate: on a unix socket, `SO_PEERCRED` already decided who
            // may talk, so there is nothing here for a PIN to add.
            Call::SystemAuthenticate(_) => Response::err(
                Some(id),
                proto::Error::new(
                    proto::code::METHOD_NOT_FOUND,
                    "system.authenticate is answered by the BLE transport, not by updaterd; \
                     access over a unix socket is decided by the socket's own peer credentials",
                ),
            ),
        }
    }

    /// Run a read-only engine call, falling back to nothing if the engine is busy.
    ///
    /// The `Err` here IS the wire answer, ready to send — that is the point of the shape,
    /// not an accident of it, so clippy's size advice (box the error) would trade one heap
    /// allocation per refusal for nothing: both variants are consumed immediately.
    #[allow(clippy::result_large_err)]
    async fn with_engine<T, F>(&self, id: Id, f: F) -> Result<T, Response>
    where
        F: FnOnce(&Engine) -> Result<T, crate::Error>,
    {
        match self.engine.try_lock() {
            Ok(engine) => f(&engine).map_err(|e| Response::err(Some(id), e.to_rpc_error())),
            Err(_) => Err(Response::err(
                Some(id),
                proto::Error::new(proto::code::BUSY, "an update is in progress; retry shortly"),
            )),
        }
    }

    /// Component status, answerable *during* an update.
    ///
    /// Uses `try_lock` and serves the cached snapshot on contention, with the live
    /// phase filled in from progress notifications. Blocking here would make the
    /// app go blank for the whole duration of an update — exactly when a user is most
    /// likely to be looking at it.
    async fn status(&self) -> Result<Vec<ComponentStatus>, crate::Error> {
        if let Ok(engine) = self.engine.try_lock() {
            let fresh = engine.status().await?;
            *self.cached_status.lock().await = fresh.clone();
            return Ok(fresh);
        }

        let mut cached = self.cached_status.lock().await.clone();
        let latest = self.latest.lock().await.clone();
        for status in &mut cached {
            if let Some(progress) = latest.iter().find(|p| p.component == status.component) {
                status.phase = progress.phase;
            }
        }
        Ok(cached)
    }

    /// Drive a mutating operation, streaming progress notifications on this
    /// connection until it finishes.
    async fn run_mutating<F>(
        &self,
        id: Id,
        out: &mut tokio::net::unix::OwnedWriteHalf,
        op: F,
    ) -> Response
    where
        F: for<'a> FnOnce(
            &'a mut Engine,
            crate::engine::ProgressTx,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<Output = Result<proto::ApplyResult, crate::Error>>
                    + Send
                    + 'a,
            >,
        >,
    {
        let mut engine = match self.engine.try_lock() {
            Ok(engine) => engine,
            Err(_) => {
                return Response::err(
                    Some(id),
                    proto::Error::new(proto::code::BUSY, "another update is already in progress"),
                );
            }
        };

        let (tx, mut rx) = mpsc::unbounded_channel::<Progress>();

        // Fan progress out to subscribers and remember the latest, so a client that
        // reconnects mid-update still sees where things are.
        let broadcast_tx = self.progress_tx.clone();
        let latest = Arc::clone(&self.latest);
        let (local_tx, mut local_rx) = mpsc::unbounded_channel::<Progress>();
        let pump = tokio::spawn(async move {
            while let Some(progress) = rx.recv().await {
                {
                    let mut latest = latest.lock().await;
                    match latest
                        .iter_mut()
                        .find(|e| e.component == progress.component)
                    {
                        Some(slot) => *slot = progress.clone(),
                        None => latest.push(progress.clone()),
                    }
                }
                let _ = broadcast_tx.send(progress.clone());
                let _ = local_tx.send(progress);
            }
        });

        let mut operation = op(&mut engine, tx);
        // Once the client has gone we stop writing but keep awaiting: the update runs
        // to completion regardless of who is watching (`architecture.md` §1.1).
        let mut client_gone = false;
        let result = loop {
            tokio::select! {
                // Prefer draining progress so the client sees ordered phases.
                biased;
                Some(progress) = local_rx.recv(), if !client_gone => {
                    let note = Request::notify_progress(&progress);
                    if write_line(out, &note).await.is_err() {
                        client_gone = true;
                    }
                }
                outcome = &mut operation => break outcome,
            }
        };

        pump.abort();
        // Anything the pump already queued is still worth sending.
        while let Ok(progress) = local_rx.try_recv() {
            let _ = write_line(out, &Request::notify_progress(&progress)).await;
        }

        match result {
            Ok(outcome) => Response::ok(Some(id), &outcome),
            Err(e) => Response::err(Some(id), e.to_rpc_error()),
        }
    }

    /// Replay the latest progress, then forward notifications until the peer closes.
    async fn stream_progress(
        &self,
        _id: Id,
        out: &mut tokio::net::unix::OwnedWriteHalf,
    ) -> std::io::Result<()> {
        let mut rx = self.progress_tx.subscribe();

        for progress in self.latest.lock().await.iter() {
            write_line(out, &Request::notify_progress(progress)).await?;
        }

        loop {
            match rx.recv().await {
                Ok(progress) => write_line(out, &Request::notify_progress(&progress)).await?,
                // A slow subscriber gets a gap, never backpressure onto the update.
                Err(broadcast::error::RecvError::Lagged(skipped)) => {
                    tracing::debug!(skipped, "subscriber lagged");
                }
                Err(broadcast::error::RecvError::Closed) => return Ok(()),
            }
        }
    }

    /// Authorise a mutating request, and record who asked.
    ///
    /// Both halves matter: the check is the security boundary beyond the socket's
    /// mode, and the log is how support answers "who triggered this rollback" — which
    /// a unix socket is the only transport able to answer (`architecture.md` §2.2).
    ///
    /// Returns the refusal as a ready-made response, so a caller cannot forget to act
    /// on a denial. Which is also why the `Err` is a full `Response` and clippy's
    /// large-error advice is declined — see `with_engine`.
    #[allow(clippy::result_large_err)]
    async fn authorise(
        &self,
        id: &Id,
        call: &Call,
        peer: Option<tokio::net::unix::UCred>,
    ) -> Result<(), Response> {
        let method = call.method();
        let component = call.component().map(proto::ComponentId::as_str);
        let verdict = {
            let policy = self.policy.lock().await;
            match policy.as_ref() {
                Some(policy) => policy.may_mutate(peer.as_ref()),
                // Not serving yet, so nothing legitimate can be calling.
                None => Err("policy not established".into()),
            }
        };

        match verdict {
            Ok(()) => {
                tracing::info!(
                    method,
                    component,
                    uid = peer.map(|p| p.uid()),
                    gid = peer.map(|p| p.gid()),
                    pid = ?peer.and_then(|p| p.pid()),
                    "mutating request"
                );
                Ok(())
            }
            Err(reason) => {
                // Denials are warnings, not debug: someone reaching the socket without
                // authorisation is worth seeing in the journal.
                tracing::warn!(
                    method,
                    component,
                    uid = peer.map(|p| p.uid()),
                    gid = peer.map(|p| p.gid()),
                    %reason,
                    "refused a mutating request"
                );
                Err(Response::err(
                    Some(id.clone()),
                    proto::Error::new(proto::code::PERMISSION_DENIED, reason),
                ))
            }
        }
    }
}

/// A client built against another `API_VERSION` is written down, not turned away.
///
/// **This was a refusal, and the refusal was aimed at the wrong thing.** It fired on two numbers
/// differing, while what it existed to prevent is a *call this release cannot serve* — and those are
/// not the same event. Most bumps here add a namespace (v5's `pad.*`, v8's `pad.input`) and cost an
/// older client nothing, so most of what the gate stopped were calls this daemon would have answered
/// correctly. `hello` precedes every `robotctl` command, so the price of one differing digit was
/// every command at once: `update apply`, which is how a skew ends, and `version`, which is how it
/// gets diagnosed. On a board where the client is a symlink into `current` and the daemon is a
/// resident process mid-restart of itself, that window opens on the ordinary path, not a broken one.
///
/// **What refuses now is the call itself.** A method this release does not have comes back
/// `METHOD_NOT_FOUND` naming it, and a `params` member it does not know comes back `INVALID_PARAMS`
/// naming that — both from `proto::Request::as_call`, both narrower than a handshake can be, and both
/// there already. The gate was a blunter second copy of them.
///
/// So the pair of versions goes to the journal, where it turns a later shape error from a puzzle into
/// a diagnosis, and `HelloResult::api_version` hands the client the same fact so it can say so
/// itself. `duckctl` reached this conclusion from the far end of the link first — see
/// `duckctl/src/main.rs`.
fn note_version_skew(client: u32) {
    tracing::warn!(
        client,
        daemon = proto::API_VERSION,
        "a client was built against a different API version — serving it anyway; a method this \
         release does not have, or a parameter it does not know, will be refused by name"
    );
}

async fn write_line<T: serde::Serialize>(
    out: &mut tokio::net::unix::OwnedWriteHalf,
    message: &T,
) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    line.push(b'\n');
    out.write_all(&line).await?;
    out.flush().await
}
