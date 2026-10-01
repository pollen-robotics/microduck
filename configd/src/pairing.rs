//! A pairing session: what `pad.pair` and `pad.reset` start, and what every trigger shares.
//!
//! ## One session, held here, and triggers hold nothing
//!
//! Whatever asks for a pad — `robotctl pad pair`, the phone over `btd`, a button, an NFC tag read
//! by the phone — is a caller of the same two methods, and none of them keeps any state of its own.
//! The session lives where the work does, so a trigger is one call and cannot get pairing wrong:
//!
//!  - **Pressing again while a session runs joins it.** The answer says `joined`, the session is
//!    not restarted and its window does not move, and the robot chirps so the person knows it
//!    heard. Nothing a trigger can do twice breaks a pairing in flight.
//!  - **A reset while pairing replaces the session.** The one running ends `cancelled`, and the
//!    reset — then a fresh session — takes over in the same task, so there is never a moment with
//!    two things driving the adapter.
//!  - **Callers follow a session by number**, through `pad.status`, and the outcome carries the
//!    number it belongs to. See [`proto::PadPairing`].
//!
//! The call answers at once and the session runs after it. That is also what lets the phone pair a
//! pad on a `weird-ble` board at all: pausing `btd` drops the phone's connection, and a call that
//! waited for the outcome would have been waiting on the connection it had just cut.
//!
//! ## What a session does
//!
//! Chirp. On a `weird-ble` board, pause `btd`. If it is a reset, remove every pad from the robot's
//! side and power-cycle the adapter, then peck. Then up to `[pad_pairing] attempts` bonds inside one
//! window of `window_seconds`, each retry starting from a clean slate: the device the last attempt
//! left half-made is removed, and on a `weird-ble` board the adapter is power-cycled again. A bond
//! counts only once an **input device** exists for the pad — `Paired` and `Connected` are both true
//! for a classic pad connected in the wrong order, which has no input device and never will, so
//! that bond is removed and retried too. Greet on success, honk on failure, and start `btd` again.
//!
//! Two failures are not retried, because retrying cannot change them: two pads in pairing mode
//! (the answer is to name one), and no adapter. Nothing found ends the session too — the window is
//! the retry — and after an earlier failed bond it says the pad has probably left pairing mode,
//! because a rejected classic connection is enough to make a pad stop listening.
//!
//! ## What a reset cannot do
//!
//! Clear the pad's half. A robot can only remove its own record of a bond; the pad keeps its key,
//! and an Xbox pad holds one host bond, so a pad that bonded here before still arrives with a key
//! this robot no longer has until it is put back in pairing mode — and sometimes only pairing it to
//! a laptop once and removing it there releases the slot. A reset is a known-clean robot, not a
//! guaranteed pairing; `docs/robot/pair-a-gamepad.md` says what to do with the pad.
//!
//! ## Why `configd` stops `btd`
//!
//! On the aic8800 radio a pad cannot form a new bond while `btd` advertises — the bisect is
//! `docs/project/pad-minimal-pairing.md` — and stopping it leaves its advertisement and its agent's
//! IO capability on the controller, so the adapter is power-cycled as well. That lived in
//! `robotctl` for as long as `robotctl` was the only way to pair, which kept `configd` out of
//! `btd`'s lifecycle. A trigger that is not a terminal has no `robotctl` to do it, so it moved here:
//! gated on the marker, and `btd` is started again on every path out, as it was. **Delete
//! [`Board::stop_btd`] and its callers when the radio changes.**

use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use duck_ipc_proto as proto;
use robotd_params::PadPairingParams;
use tokio::sync::oneshot;
use tokio::time::Instant;

use crate::pad::{self, Pads};

/// How long a freshly bonded pad gets to produce an input device.
///
/// An Xbox pad's appears within a second or two of the bond — HID over GATT, relayed by
/// bluetoothd through uhid — and a classic pad's as soon as the kernel driver binds. Ten is margin;
/// the cost of being too short is a good bond removed and retried, which the pad survives only if
/// it is still in pairing mode.
pub const INPUT_WAIT: Duration = Duration::from_secs(10);

const INPUT_POLL: Duration = Duration::from_millis(250);

/// How long a session waits before pausing `btd`, so the answer to whoever started it has left.
///
/// That answer goes back through `btd` when the phone asked, and stopping `btd` under it loses it.
/// A second is several round trips over BLE; the cost is a second of a sixty-second window.
pub const REPLY_GRACE: Duration = Duration::from_secs(1);

/// Where a board provisioned with `--weird-ble` or `--pause-btd-on-pair` says so. Written by
/// `scripts/setup-board.sh`.
///
/// A marker rather than re-deriving the answer from `Privacy = device` in `main.conf`: an explicit
/// record of the decision someone made cannot be confused with a setting that arrived some other
/// way. Under /var/lib rather than in a release directory: it is a fact about the board, and it
/// has to survive an update and a rollback.
pub const WEIRD_BLE_MARKER: &str = "/var/lib/robot/weird-ble";

const BTD_UNIT: &str = "btd.service";

/// The world a session acts on besides the radio: its settings, `btd`, and the robot's voice.
///
/// A trait for the tests, which is also why the radio is [`Pads`] and not here.
#[async_trait]
pub trait Board: Send + Sync {
    /// `[pad_pairing]`, read afresh for each session.
    fn settings(&self) -> PadPairingParams;
    /// Whether this board needs `btd` paused and the adapter power-cycled to bond a pad.
    fn needs_the_ble_workaround(&self) -> bool;
    /// Stop `btd`, and say whether it was running and now is not — the only case where starting it
    /// again afterwards is right.
    async fn stop_btd(&self) -> bool;
    async fn start_btd(&self);
    /// Best effort: a robot whose `robotd` is down still pairs, silently.
    async fn quack(&self, tag: proto::SoundTag);
}

/// The pairing session, shared by every caller of `pad.pair` and `pad.reset`.
pub struct Pairing {
    inner: Arc<Inner>,
}

struct Inner {
    pads: Arc<dyn Pads>,
    board: Arc<dyn Board>,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    /// The number of the session running, or of the last one to run.
    session: u64,
    running: Option<Running>,
    last: Option<proto::PadPairingOutcome>,
}

struct Running {
    phase: proto::PadPairingPhase,
    attempt: u32,
    attempts: u32,
    deadline: Option<Instant>,
    /// This session began as a reset, or became one. Another reset joins it for its whole
    /// length, pairing half included: a second press of "reset" is the same wish, not a new one.
    reset: bool,
    /// A reset arrived while this session paired. Checked when a session ends as well as raced
    /// against it, so a reset landing just as a session finished on its own is not lost.
    reset_requested: bool,
    /// Ends the session in flight. Taken by the reset that sends it.
    cancel: Option<oneshot::Sender<()>>,
}

impl Running {
    fn new(phase: proto::PadPairingPhase) -> Self {
        Self {
            phase,
            attempt: 0,
            attempts: 0,
            deadline: None,
            reset: phase == proto::PadPairingPhase::Resetting,
            reset_requested: false,
            cancel: None,
        }
    }
}

impl Pairing {
    pub fn new(pads: Arc<dyn Pads>, board: Arc<dyn Board>) -> Self {
        Self {
            inner: Arc::new(Inner {
                pads,
                board,
                state: Mutex::new(State::default()),
            }),
        }
    }

    /// Start a session, or join the one running. Answers at once.
    ///
    /// `mac` and `timeout` apply only to a session this starts — see [`proto::PadPairParams`].
    pub fn start(&self, mac: Option<String>, timeout: Option<u32>) -> proto::PadPairing {
        let mut state = self.inner.lock();
        if state.running.is_some() {
            let joined = snapshot(&state, true);
            drop(state);
            tracing::info!(session = joined.session, "already pairing; joined it");
            self.inner.acknowledge();
            return joined;
        }
        state.session += 1;
        let session = state.session;
        state.running = Some(Running::new(proto::PadPairingPhase::Pairing));
        let answer = snapshot(&state, false);
        drop(state);
        tracing::info!(session, ?mac, ?timeout, "pairing session started");
        tokio::spawn(Arc::clone(&self.inner).drive(session, false, mac, timeout));
        answer
    }

    /// Forget every pad on the robot's side, then start a session. Answers at once.
    ///
    /// Joins a reset already under way. Replaces a session that is only pairing: that one ends
    /// `cancelled`, and this one — with its own number — takes over.
    pub fn reset(&self) -> proto::PadPairing {
        let mut state = self.inner.lock();
        let replace = match &state.running {
            None => None,
            Some(running) if running.reset || running.reset_requested => {
                let joined = snapshot(&state, true);
                drop(state);
                tracing::info!(session = joined.session, "already resetting; joined it");
                self.inner.acknowledge();
                return joined;
            }
            Some(_) => Some(()),
        };

        state.session += 1;
        let session = state.session;
        match replace {
            Some(()) => {
                let running = state.running.as_mut().expect("matched above");
                running.reset_requested = true;
                running.reset = true;
                running.phase = proto::PadPairingPhase::Resetting;
                running.attempt = 0;
                running.attempts = 0;
                running.deadline = None;
                if let Some(cancel) = running.cancel.take() {
                    let _ = cancel.send(());
                }
                tracing::info!(session, "reset: replacing the session that was pairing");
                snapshot(&state, false)
            }
            None => {
                state.running = Some(Running::new(proto::PadPairingPhase::Resetting));
                let answer = snapshot(&state, false);
                drop(state);
                tracing::info!(session, "reset started");
                tokio::spawn(Arc::clone(&self.inner).drive(session, true, None, None));
                answer
            }
        }
    }

    /// Where pairing is now, for `pad.status`.
    pub fn status(&self) -> proto::PadPairing {
        snapshot(&self.inner.lock(), false)
    }
}

fn snapshot(state: &State, joined: bool) -> proto::PadPairing {
    let last = state.last.clone();
    match &state.running {
        None => proto::PadPairing {
            session: state.session,
            last,
            ..Default::default()
        },
        Some(running) => proto::PadPairing {
            session: state.session,
            phase: running.phase,
            attempt: running.attempt,
            attempts: running.attempts,
            remaining_seconds: running
                .deadline
                .map(|d| {
                    d.saturating_duration_since(Instant::now())
                        .as_secs_f64()
                        .ceil() as u32
                })
                .unwrap_or(0),
            joined,
            last,
        },
    }
}

fn failed(
    reason: proto::PadPairFailure,
    detail: impl Into<String>,
    mac: Option<String>,
) -> proto::PadPairResult {
    proto::PadPairResult::Failed {
        reason,
        detail: Some(detail.into()),
        mac,
    }
}

impl Inner {
    fn lock(&self) -> std::sync::MutexGuard<'_, State> {
        // A panic while holding this is a bug in a few lines of bookkeeping, and refusing every
        // later pairing over it would be worse than carrying on with what was written.
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn update(&self, change: impl FnOnce(&mut Running)) {
        if let Some(running) = self.lock().running.as_mut() {
            change(running);
        }
    }

    /// A chirp for a press that joined a session: "I heard you, still on it".
    fn acknowledge(self: &Arc<Self>) {
        if self.board.settings().sounds {
            let inner = Arc::clone(self);
            tokio::spawn(async move { inner.board.quack(proto::SoundTag::Chirp).await });
        }
    }

    /// Run sessions until one ends with no reset waiting behind it.
    async fn drive(
        self: Arc<Self>,
        mut session: u64,
        mut reset: bool,
        mut mac: Option<String>,
        mut timeout: Option<u32>,
    ) {
        let settings = self.board.settings();
        let workaround = self.board.needs_the_ble_workaround();
        if settings.sounds {
            self.board.quack(proto::SoundTag::Chirp).await;
        }

        let mut btd_stopped = false;
        if workaround {
            tokio::time::sleep(REPLY_GRACE).await;
            btd_stopped = self.board.stop_btd().await;
            if btd_stopped {
                tracing::info!("paused btd while the pad bonds; it comes back afterwards");
            }
        }

        loop {
            let (cancel, cancelled) = oneshot::channel();
            self.update(|running| {
                // A reset that landed before this channel existed had nothing to send on, so it is
                // sent here: the session it meant to replace ends before it begins.
                if running.reset_requested {
                    let _ = cancel.send(());
                } else {
                    running.cancel = Some(cancel);
                }
            });

            let result = tokio::select! {
                biased;
                Ok(()) = cancelled => failed(
                    proto::PadPairFailure::Cancelled,
                    "replaced by a reset",
                    None,
                ),
                result = self.session(reset, mac.as_deref(), timeout, &settings, workaround) => result,
            };

            let next = {
                let mut state = self.lock();
                state.last = Some(proto::PadPairingOutcome {
                    session,
                    result: result.clone(),
                });
                match state.running.as_mut() {
                    Some(running) if running.reset_requested => {
                        running.reset_requested = false;
                        running.cancel = None;
                        running.phase = proto::PadPairingPhase::Resetting;
                        Some(state.session)
                    }
                    _ => {
                        state.running = None;
                        None
                    }
                }
            };

            match &result {
                proto::PadPairResult::Paired { pad } => {
                    tracing::warn!(session, mac = %pad.mac, name = %pad.name, "pairing session: paired");
                    if settings.sounds {
                        self.board.quack(proto::SoundTag::Greet).await;
                    }
                }
                proto::PadPairResult::Failed {
                    reason: proto::PadPairFailure::Cancelled,
                    ..
                } => tracing::info!(session, "pairing session replaced by a reset"),
                proto::PadPairResult::Failed { reason, detail, .. } => {
                    tracing::warn!(session, ?reason, ?detail, "pairing session failed");
                    if settings.sounds {
                        self.board.quack(proto::SoundTag::Alarm).await;
                    }
                }
            }

            match next {
                Some(following) => {
                    session = following;
                    reset = true;
                    mac = None;
                    timeout = None;
                }
                None => break,
            }
        }

        if btd_stopped {
            self.board.start_btd().await;
        }
    }

    /// One session: the reset if asked for, then the attempts.
    async fn session(
        &self,
        reset: bool,
        mac: Option<&str>,
        timeout: Option<u32>,
        settings: &PadPairingParams,
        workaround: bool,
    ) -> proto::PadPairResult {
        if reset {
            self.update(|running| running.phase = proto::PadPairingPhase::Resetting);
            match self.pads.reset().await {
                Ok(removed) => tracing::warn!(removed, "every pad forgotten on the robot's side"),
                Err(e) => tracing::warn!(error = %e, "could not forget the pads"),
            }
            if let Err(e) = self.pads.cycle_adapter().await {
                tracing::warn!(error = %e, "could not power-cycle the adapter");
            }
            if settings.sounds {
                self.board.quack(proto::SoundTag::Peck).await;
            }
        }

        let window = pad::pair_timeout(
            timeout,
            Duration::from_secs(u64::from(settings.window_seconds)),
        );
        let deadline = Instant::now() + window;
        let attempts = settings.attempts.max(1);
        self.update(|running| {
            running.phase = proto::PadPairingPhase::Pairing;
            running.attempts = attempts;
            running.deadline = Some(deadline);
        });

        // The pads this robot already had. One of them turning up is an idempotent re-run that
        // re-asserts trust — not a fresh bond to verify, and never one to remove.
        let before: HashSet<String> = match self.pads.status().await {
            Ok(pads) => pads.iter().map(|p| p.mac.to_ascii_lowercase()).collect(),
            Err(_) => HashSet::new(),
        };
        let known = |mac: &str| before.contains(&mac.to_ascii_lowercase());

        let mut last: Option<proto::PadPairResult> = None;
        for attempt in 1..=attempts {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if attempt > 1 && remaining.is_zero() {
                break;
            }
            self.update(|running| running.attempt = attempt);
            if workaround && let Err(e) = self.pads.cycle_adapter().await {
                tracing::warn!(error = %e, "could not power-cycle the adapter; this bond may fail");
            }
            tracing::info!(
                attempt,
                attempts,
                ?remaining,
                "looking for a gamepad in pairing mode"
            );

            let outcome = self
                .pads
                .pair(mac, remaining)
                .await
                .unwrap_or_else(|e| failed(proto::PadPairFailure::Other, e, None));

            match outcome {
                proto::PadPairResult::Paired { pad } => {
                    if known(&pad.mac) || self.wait_for_input(&pad.mac).await {
                        return proto::PadPairResult::Paired { pad };
                    }
                    tracing::warn!(
                        attempt,
                        mac = %pad.mac,
                        "bonded, and no input device appeared; removing the bond to try again"
                    );
                    let _ = self.pads.forget(&pad.mac).await;
                    last = Some(failed(
                        proto::PadPairFailure::NoInput,
                        "it bonded, and no input device appeared for it",
                        Some(pad.mac),
                    ));
                }
                proto::PadPairResult::Failed {
                    reason:
                        proto::PadPairFailure::NotFound
                        | proto::PadPairFailure::Ambiguous
                        | proto::PadPairFailure::NoAdapter,
                    ..
                } => return after(outcome, last),
                proto::PadPairResult::Failed {
                    reason,
                    ref detail,
                    mac: ref tried,
                } => {
                    tracing::warn!(attempt, ?reason, ?detail, "bond failed");
                    // A clean slate for the next attempt: a bond this one left half-made is
                    // removed, unless it is a pad this robot had before the session began. Only a
                    // *bond*: an unbonded device in BlueZ's cache is harmless, and removing it
                    // makes the next sweep rediscover it, out of a pairing window the pad is
                    // already spending.
                    if let Some(tried) = tried
                        && !known(tried)
                        && self.is_bonded(tried).await
                    {
                        tracing::info!(mac = %tried, "removing the half-made bond");
                        let _ = self.pads.forget(tried).await;
                    }
                    last = Some(outcome);
                }
            }
        }
        last.unwrap_or_else(|| {
            failed(
                proto::PadPairFailure::NotFound,
                "the window closed before a pad turned up",
                None,
            )
        })
    }

    async fn is_bonded(&self, mac: &str) -> bool {
        self.pads
            .status()
            .await
            .is_ok_and(|pads| pads.iter().any(|p| p.mac.eq_ignore_ascii_case(mac)))
    }

    async fn wait_for_input(&self, mac: &str) -> bool {
        let deadline = Instant::now() + INPUT_WAIT;
        loop {
            if self.pads.has_input(mac).await {
                return true;
            }
            if Instant::now() >= deadline {
                return false;
            }
            tokio::time::sleep(INPUT_POLL).await;
        }
    }
}

/// A search that found nothing, said in the light of the attempt before it.
///
/// Nothing found straight after a failed bond is not "the pad is not in pairing mode" in the sense
/// the bare refusal means: it *was*, and the failed attempt is what took it out. Saying so is the
/// difference between someone checking the pad and someone checking the robot.
fn after(
    outcome: proto::PadPairResult,
    earlier: Option<proto::PadPairResult>,
) -> proto::PadPairResult {
    match (outcome, earlier) {
        (
            proto::PadPairResult::Failed {
                reason: proto::PadPairFailure::NotFound,
                ..
            },
            Some(proto::PadPairResult::Failed {
                reason,
                detail,
                mac,
            }),
        ) => failed(
            proto::PadPairFailure::NotFound,
            format!(
                "an earlier attempt ended {reason:?}{}, and the pad did not come back — it has \
                 probably left pairing mode. Put it back in pairing mode and pair again.",
                detail.map(|d| format!(" ({d})")).unwrap_or_default()
            ),
            mac,
        ),
        (outcome, _) => outcome,
    }
}

/// The board, for real: `robotd.toml`, `systemctl`, and `robotd`'s socket.
pub struct SystemBoard {
    /// `robotd.toml`, for `[pad_pairing]`.
    pub params: PathBuf,
    /// Whether that path was given explicitly, so a missing file is worth a warning.
    pub params_explicit: bool,
    /// `robotd`'s socket, for the quacks.
    pub robotd: PathBuf,
}

#[async_trait]
impl Board for SystemBoard {
    fn settings(&self) -> PadPairingParams {
        match robotd_params::Params::load(&self.params, self.params_explicit) {
            Ok(params) => params.pad_pairing,
            // Not a reason to refuse a pairing: the defaults pair a pad, and `robotd` is the
            // daemon that refuses loudly over this file.
            Err(e) => {
                tracing::warn!(error = %e, "unusable params file; pairing on the defaults");
                PadPairingParams::default()
            }
        }
    }

    fn needs_the_ble_workaround(&self) -> bool {
        std::path::Path::new(WEIRD_BLE_MARKER).exists()
    }

    async fn stop_btd(&self) -> bool {
        // Only restarted if it was running: a board with `btd` deliberately disabled must not
        // have it switched on by pairing a gamepad.
        if !systemctl(&["is-active", "--quiet", BTD_UNIT]).await {
            return false;
        }
        let stopped = systemctl(&["stop", BTD_UNIT]).await;
        if !stopped {
            // Not fatal: the pairing may still work, and refusing to try would be worse.
            tracing::warn!("could not stop btd, so this pairing may fail on this board");
        }
        stopped
    }

    async fn start_btd(&self) {
        if !systemctl(&["start", BTD_UNIT]).await {
            tracing::error!(
                "could not start btd again; the phone path is down until `systemctl start btd`"
            );
        }
    }

    async fn quack(&self, tag: proto::SoundTag) {
        // Bounded, because a wedged `robotd` must not hold up a pairing to make a noise.
        match tokio::time::timeout(Duration::from_secs(2), sound(&self.robotd, tag)).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => tracing::debug!(error = %e, ?tag, "no quack: robotd did not take it"),
            Err(_) => tracing::debug!(?tag, "no quack: robotd did not answer in time"),
        }
    }
}

/// `systemctl`, with its output discarded: every call here has something better to say than
/// systemd does. `stop` waits for the job, which is what makes the power cycle after it land on an
/// adapter `btd` has let go of.
async fn systemctl(args: &[&str]) -> bool {
    tokio::process::Command::new("systemctl")
        .args(args)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .await
        .is_ok_and(|status| status.success())
}

/// One `robot.sound` over `robotd`'s socket.
async fn sound(socket: &std::path::Path, tag: proto::SoundTag) -> std::io::Result<()> {
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    let stream = tokio::net::UnixStream::connect(socket).await?;
    let (read, mut write) = stream.into_split();
    let call = proto::Call::RobotSound(proto::SoundParams { tag, hold: None });
    let mut line = serde_json::to_vec(&proto::Request::call(proto::Id::Number(1), &call))?;
    line.push(b'\n');
    write.write_all(&line).await?;
    // Read the answer so the call has landed before the connection closes under it.
    let mut answer = String::new();
    BufReader::new(read).read_line(&mut answer).await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pad::{FakePads, bonded, unpaired};

    const XBOX: &str = "78:86:2E:BB:13:28";

    #[derive(Default)]
    struct FakeBoard {
        settings: Mutex<PadPairingParams>,
        workaround: bool,
        quacks: Mutex<Vec<proto::SoundTag>>,
        btd: Mutex<Vec<&'static str>>,
    }

    impl FakeBoard {
        fn quacks(&self) -> Vec<proto::SoundTag> {
            self.quacks.lock().unwrap().clone()
        }
    }

    #[async_trait]
    impl Board for FakeBoard {
        fn settings(&self) -> PadPairingParams {
            self.settings.lock().unwrap().clone()
        }
        fn needs_the_ble_workaround(&self) -> bool {
            self.workaround
        }
        async fn stop_btd(&self) -> bool {
            self.btd.lock().unwrap().push("stop");
            true
        }
        async fn start_btd(&self) {
            self.btd.lock().unwrap().push("start");
        }
        async fn quack(&self, tag: proto::SoundTag) {
            self.quacks.lock().unwrap().push(tag);
        }
    }

    fn setup(pads: FakePads, board: FakeBoard) -> (Pairing, Arc<FakePads>, Arc<FakeBoard>) {
        let pads = Arc::new(pads);
        let board = Arc::new(board);
        let pairing = Pairing::new(pads.clone(), board.clone());
        (pairing, pads, board)
    }

    /// Follow a session the way a caller does: by number, until its outcome is recorded.
    async fn outcome(pairing: &Pairing, session: u64) -> proto::PadPairResult {
        loop {
            if let Some(last) = pairing.status().last
                && last.session >= session
            {
                assert_eq!(
                    last.session, session,
                    "a later session's outcome replaced this one"
                );
                return last.result;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    async fn idle(pairing: &Pairing) {
        while pairing.status().phase != proto::PadPairingPhase::Idle {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    fn reason(result: &proto::PadPairResult) -> Option<proto::PadPairFailure> {
        match result {
            proto::PadPairResult::Failed { reason, .. } => Some(*reason),
            proto::PadPairResult::Paired { .. } => None,
        }
    }

    /// The everyday case, and the feedback that is the whole interface of a trigger with no
    /// screen: a chirp when it starts, a greet when the pad drives.
    #[tokio::test(start_paused = true)]
    async fn a_session_pairs_and_says_so() {
        let (pairing, _, board) = setup(FakePads::new(), FakeBoard::default());
        let started = pairing.start(None, None);
        assert_eq!(started.session, 1);
        assert_eq!(started.phase, proto::PadPairingPhase::Pairing);
        assert!(!started.joined);

        let result = outcome(&pairing, 1).await;
        assert!(
            matches!(result, proto::PadPairResult::Paired { .. }),
            "{result:?}"
        );
        idle(&pairing).await;
        assert_eq!(
            board.quacks(),
            [proto::SoundTag::Chirp, proto::SoundTag::Greet]
        );
    }

    /// Pressing the trigger again while a session runs must not break it: it joins, keeps its
    /// number, does not start a second search, and chirps so the person knows it was heard.
    #[tokio::test(start_paused = true)]
    async fn pressing_again_joins_the_session_running() {
        let pads = FakePads::new();
        pads.bonds_take(Duration::from_secs(5)).await;
        let (pairing, _, board) = setup(pads, FakeBoard::default());

        let first = pairing.start(None, None);
        tokio::time::sleep(Duration::from_secs(1)).await;
        let second = pairing.start(Some("AA:BB:CC:DD:EE:FF".into()), Some(5));
        assert!(second.joined);
        assert_eq!(second.session, first.session);

        let result = outcome(&pairing, first.session).await;
        assert!(
            matches!(result, proto::PadPairResult::Paired { .. }),
            "{result:?}"
        );
        idle(&pairing).await;
        assert_eq!(pairing.status().session, 1, "only one session ran");
        assert_eq!(
            board
                .quacks()
                .iter()
                .filter(|t| **t == proto::SoundTag::Chirp)
                .count(),
            2,
            "the press that joined was acknowledged"
        );
    }

    /// A bond that fails is retried inside the window, from a clean slate — which is what turns
    /// "the first pair times out, the second works" into one press.
    #[tokio::test(start_paused = true)]
    async fn a_failed_bond_is_retried() {
        let pads = FakePads::new();
        pads.fail_next(proto::PadPairFailure::Timeout).await;
        let (pairing, pads, board) = setup(pads, FakeBoard::default());

        let session = pairing.start(None, None).session;
        let result = outcome(&pairing, session).await;
        assert!(
            matches!(result, proto::PadPairResult::Paired { .. }),
            "{result:?}"
        );
        idle(&pairing).await;
        assert_eq!(
            pads.cycles().await,
            0,
            "no power cycle on a board that does not need one"
        );
        assert_eq!(board.quacks().last(), Some(&proto::SoundTag::Greet));
    }

    /// A pad that bonds and gets no input device is the trap that looks like success: a solid
    /// light, `Paired`, `Connected`, and nothing to drive from. It must not greet.
    #[tokio::test(start_paused = true)]
    async fn a_bond_with_no_input_device_is_not_a_success() {
        let pads = FakePads::new();
        pads.without_input(XBOX).await;
        let (pairing, pads, board) = setup(pads, FakeBoard::default());

        let session = pairing.start(None, None).session;
        let result = outcome(&pairing, session).await;
        assert_eq!(
            reason(&result),
            Some(proto::PadPairFailure::NotFound),
            "{result:?}"
        );
        // The retry after the first bond found nothing: the bond was removed, and the fake — like
        // the pad — does not come back into pairing mode by itself. The answer says so.
        let proto::PadPairResult::Failed { detail, .. } = &result else {
            unreachable!()
        };
        assert!(
            detail.as_deref().unwrap_or("").contains("pairing mode"),
            "{detail:?}"
        );
        assert!(
            pads.status().await.unwrap().is_empty(),
            "the useless bond was removed"
        );
        idle(&pairing).await;
        assert_eq!(board.quacks().last(), Some(&proto::SoundTag::Alarm));
    }

    /// Two pads in pairing mode are refused, not retried: retrying cannot pick one, and the
    /// answer — name one — is the caller's.
    #[tokio::test(start_paused = true)]
    async fn ambiguity_is_not_retried() {
        let pads = FakePads::with(vec![
            unpaired(XBOX, "Xbox Wireless Controller"),
            unpaired("98:B6:E9:28:06:09", "Pro Controller"),
        ]);
        let (pairing, _, _) = setup(pads, FakeBoard::default());
        let session = pairing.start(None, None).session;
        let result = outcome(&pairing, session).await;
        assert_eq!(reason(&result), Some(proto::PadPairFailure::Ambiguous));
    }

    /// Every attempt failing ends the session with the last failure, after exactly `attempts`.
    #[tokio::test(start_paused = true)]
    async fn attempts_are_bounded() {
        let pads = FakePads::new();
        for _ in 0..5 {
            pads.fail_next(proto::PadPairFailure::Rejected).await;
        }
        let board = FakeBoard::default();
        board.settings.lock().unwrap().attempts = 2;
        let (pairing, pads, _) = setup(pads, board);

        let session = pairing.start(None, None).session;
        let result = outcome(&pairing, session).await;
        assert_eq!(reason(&result), Some(proto::PadPairFailure::Rejected));
        // Three scripted failures left: two attempts consumed two.
        for _ in 0..3 {
            let next = pads.pair(None, Duration::ZERO).await.unwrap();
            assert_eq!(reason(&next), Some(proto::PadPairFailure::Rejected));
        }
        assert!(matches!(
            pads.pair(None, Duration::ZERO).await.unwrap(),
            proto::PadPairResult::Paired { .. }
        ));
    }

    /// On a `weird-ble` board `btd` is paused once for the session and started again at the end,
    /// and the adapter is power-cycled before every attempt.
    #[tokio::test(start_paused = true)]
    async fn the_btd_pause_brackets_the_session() {
        let pads = FakePads::new();
        pads.fail_next(proto::PadPairFailure::Timeout).await;
        let board = FakeBoard {
            workaround: true,
            ..Default::default()
        };
        let (pairing, pads, board) = setup(pads, board);

        let session = pairing.start(None, None).session;
        outcome(&pairing, session).await;
        idle(&pairing).await;
        // `start` is the last thing a session does, after its outcome is recorded.
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert_eq!(*board.btd.lock().unwrap(), ["stop", "start"]);
        assert_eq!(pads.cycles().await, 2, "one power cycle per attempt");
    }

    /// A reset forgets every pad on the robot's side, power-cycles the adapter, then pairs.
    #[tokio::test(start_paused = true)]
    async fn a_reset_forgets_everything_then_pairs() {
        let pads = FakePads::with(vec![bonded(XBOX, "Xbox Wireless Controller")]);
        let (pairing, pads, board) = setup(pads, FakeBoard::default());

        let started = pairing.reset();
        assert_eq!(started.phase, proto::PadPairingPhase::Resetting);
        let result = outcome(&pairing, started.session).await;
        assert!(
            matches!(result, proto::PadPairResult::Paired { .. }),
            "{result:?}"
        );
        assert_eq!(pads.cycles().await, 1);
        idle(&pairing).await;
        assert_eq!(
            board.quacks(),
            [
                proto::SoundTag::Chirp,
                proto::SoundTag::Peck,
                proto::SoundTag::Greet
            ]
        );
    }

    /// A reset pressed while a session is pairing replaces it: that session ends `cancelled`
    /// under its own number, and the reset runs as the next one. Nothing runs twice at once.
    #[tokio::test(start_paused = true)]
    async fn a_reset_while_pairing_replaces_the_session() {
        let pads = FakePads::with(vec![]);
        let board = FakeBoard::default();
        let (pairing, pads, _) = setup(pads, board);
        pads.bonds_take(Duration::from_secs(30)).await;

        let first = pairing.start(None, None).session;
        tokio::time::sleep(Duration::from_secs(2)).await;
        let reset = pairing.reset();
        assert!(!reset.joined);
        assert_eq!(reset.session, first + 1);
        assert_eq!(reset.phase, proto::PadPairingPhase::Resetting);

        let replaced = outcome(&pairing, first).await;
        assert_eq!(reason(&replaced), Some(proto::PadPairFailure::Cancelled));

        // And a second reset while that one runs joins it rather than starting a third.
        let again = pairing.reset();
        assert!(again.joined);
        assert_eq!(again.session, reset.session);

        pads.bonds_take(Duration::ZERO).await;
        pads.appears(unpaired(XBOX, "Xbox Wireless Controller"))
            .await;
        let result = outcome(&pairing, reset.session).await;
        assert!(
            matches!(result, proto::PadPairResult::Paired { .. }),
            "{result:?}"
        );
        idle(&pairing).await;
        assert_eq!(pairing.status().session, reset.session);
    }

    /// A pad this robot already had is a re-run that re-asserts trust, not a fresh bond: it is
    /// not held to the input-device check, and a switched-off one is never removed for failing it.
    #[tokio::test(start_paused = true)]
    async fn a_pad_already_bonded_is_never_removed() {
        let pads = FakePads::with(vec![bonded(XBOX, "Xbox Wireless Controller")]);
        pads.without_input(XBOX).await;
        let (pairing, pads, _) = setup(pads, FakeBoard::default());

        let session = pairing.start(None, Some(5)).session;
        let result = outcome(&pairing, session).await;
        assert!(
            matches!(result, proto::PadPairResult::Paired { .. }),
            "{result:?}"
        );
        assert_eq!(pads.status().await.unwrap().len(), 1);
    }

    /// No sounds means none, including the chirp for a press that joined.
    #[tokio::test(start_paused = true)]
    async fn a_silent_robot_stays_silent() {
        let board = FakeBoard::default();
        board.settings.lock().unwrap().sounds = false;
        let (pairing, _, board) = setup(FakePads::new(), board);
        let session = pairing.start(None, None).session;
        pairing.start(None, None);
        outcome(&pairing, session).await;
        idle(&pairing).await;
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert!(board.quacks().is_empty());
    }
}
