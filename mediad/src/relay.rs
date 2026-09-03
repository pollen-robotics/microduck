//! The robot's half of the bridge to the rendezvous service.
//!
//! A LAN client reaches `webrtcsink`'s signalling server directly and nothing here is involved.
//! This is what makes a duck reachable from *outside* its network: it connects **outward** to
//! `reachy_mini_central` holding the account token, registers as a **producer**, and the service
//! shows a client only the robots its own account owns. `docs/design/remote-access-design.md` §3
//! owns the argument; this module owns the connection.
//!
//! # What this slice does, and what it deliberately refuses
//!
//! Registration and liveness only (§8, slice 2): the robot appears in the service's listing and
//! stays there. Translating a session — a remote peer's SDP and ICE onto the local signalling
//! server at `ws://127.0.0.1:8443` — is the next slice, and until it exists a `startSession` is
//! answered with `endSession` rather than ignored. That is the honest failure: a client that
//! clicks connect gets a refusal now instead of a robot that shows as busy forever, and the
//! service's own state stays clean.
//!
//! # Why the transport is HTTP, which is not what `remote-webrtc.md` §7 assumed
//!
//! The envelopes are the gst signalling protocol's — the same messages a LAN client exchanges —
//! but they arrive over **SSE** and are sent with **`POST /send`**, with per-hop peer and session
//! ids. So the payload stays opaque and the envelope does not: this is a translator with an
//! opaque payload rather than a relay. §3.2 has the two sides side by side.
//!
//! # Three things read out of their source that shape the code below
//!
//! - **`POST /send` before `GET /events` is a 400.** The peer does not exist until the stream
//!   does — identity comes from the bearer token, and the token is bound to a peer by the
//!   `/events` connection. So the stream is opened *first* and registration follows the welcome,
//!   which happens to be the order §3.4 wanted anyway for a different reason.
//! - **The lease is refreshed by inbound `POST`, not by a healthy stream.** Thirty seconds, and a
//!   half-open TCP connection absorbs server-pushed keepalives silently for minutes — during
//!   which the robot believes it is reachable and is not. Hence [`heartbeat`-cadence] re-posts of
//!   `setPeerStatus`, and hence the split-brain poll.
//! - **Only producers carrying `meta.hardware_id` are swept.** A producer without it is never
//!   evicted, so a crashed daemon would leave a ghost in somebody's robot list forever. This
//!   always sends one — the SoC serial, or `/etc/machine-id` where there is no serial to read.
//!
//! [`heartbeat`-cadence]: Welcome::heartbeat
//!
//! # It is a task, not a daemon
//!
//! In `mediad` rather than a `relayd` for §3.5's reasons: a separate unit would need its own copy
//! of the producer identity, its own config and its own restart story, and would still be useless
//! without `mediad` running. Nothing here touches GStreamer — the boundary `pipeline.rs`'s
//! no-panic rule lives on — and nothing here is `cfg(target_os)`-gated, so the whole of it is
//! testable on a laptop against a fake service.

use std::path::{Path, PathBuf};
use std::time::Duration;

use eventsource_stream::Eventsource as _;
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};

/// The Space the mini's fleet already registers against. §4.
pub const DEFAULT_RENDEZVOUS: &str = "https://pollen-robotics-reachy-mini-central.hf.space";

/// Where `updaterd` keeps the account credential.
///
/// **A cross-daemon file format, and `updaterd` owns it.** `updater::account::Store` writes it and
/// a test there pins the one key this reads, because the writer is what can break the contract.
/// Read on every connect attempt rather than cached: a login that happens while this task is
/// waiting has to take effect without a restart, and re-reading a small file on a path that
/// already sleeps for thirty seconds costs nothing.
pub const DEFAULT_TOKEN_PATH: &str = "/etc/robot/hf-token";

/// How long to wait between looks at a token file that is not there yet.
///
/// This is the `waiting for token` state, and it is the ordinary state of a robot nobody has
/// signed in — so it must be quiet in the journal and cheap on the board.
const NO_TOKEN_POLL: Duration = Duration::from_secs(30);

/// How long a read from the event stream may go quiet before the connection is presumed dead.
///
/// The service emits `event: ping` after 30 s of idle, whose only job is to keep the proxy in
/// front of the Space from killing the connection. Sixty seconds is two missed pings, which is
/// what `reachy_mini`'s relay uses. §3.3.
const READ_TIMEOUT: Duration = Duration::from_secs(60);

/// How long the welcome gets to arrive before the connection is abandoned.
const WELCOME_TIMEOUT: Duration = Duration::from_secs(20);

/// The fallback heartbeat cadence, when the welcome names none.
///
/// The service publishes `recommended_heartbeat_interval_seconds: 10.0` and **no `lease_seconds`**
/// — so `reachy_mini`'s middle rung, `lease_seconds / 3`, is unreachable here and is not
/// reproduced. Five seconds is a sixth of the lease, which survives a missed post.
const HEARTBEAT_FALLBACK: Duration = Duration::from_secs(5);

/// The cadence is clamped, so a misconfigured service can neither ask for a request storm nor
/// talk us into a cadence slower than our own eviction.
const HEARTBEAT_BOUNDS: (Duration, Duration) = (Duration::from_secs(1), Duration::from_secs(60));

/// How often to ask the service whether it still lists this robot. §3.4, split-brain.
const STATUS_POLL: Duration = Duration::from_secs(30);

/// How many consecutive times the service may fail to list this robot before reconnecting.
///
/// Two rather than one: `/api/robot-status` is a separate request from the stream, and one lost
/// answer is not evidence of anything.
const MISSES_BEFORE_RECONNECT: u32 = 2;

/// Reconnect backoff: where it starts, where it stops, and how much noise goes on top.
const BACKOFF_START: Duration = Duration::from_secs(5);
const BACKOFF_MAX: Duration = Duration::from_secs(60);
const BACKOFF_JITTER: f64 = 0.10;

/// How long a request that is not the event stream gets.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(20);

/// Every interval this task runs on, in one place.
///
/// A value rather than the constants directly, and it exists for the tests: §3.4's four failure
/// modes are all *timing* failures — a lease that stops being refreshed, a service that goes on
/// answering while it has forgotten us, a fleet reconnecting in lockstep — and none of them can be
/// reproduced on demand by hand on a board. With the intervals injectable, each one is a test that
/// runs in under a second. Production always uses [`Timings::default`], which is the constants
/// above.
#[derive(Debug, Clone, Copy)]
pub struct Timings {
    pub no_token_poll: Duration,
    pub read_timeout: Duration,
    pub welcome_timeout: Duration,
    pub heartbeat_fallback: Duration,
    pub heartbeat_bounds: (Duration, Duration),
    pub status_poll: Duration,
    pub backoff_start: Duration,
    pub backoff_max: Duration,
}

impl Default for Timings {
    fn default() -> Self {
        Self {
            no_token_poll: NO_TOKEN_POLL,
            read_timeout: READ_TIMEOUT,
            welcome_timeout: WELCOME_TIMEOUT,
            heartbeat_fallback: HEARTBEAT_FALLBACK,
            heartbeat_bounds: HEARTBEAT_BOUNDS,
            status_poll: STATUS_POLL,
            backoff_start: BACKOFF_START,
            backoff_max: BACKOFF_MAX,
        }
    }
}

// ── what a client sees in the listing ────────────────────────────────────────

/// What this robot calls itself to the service.
///
/// Free-form to the protocol and **not to the server**, which reads three of these keys — see the
/// module header on `hardware_id`. The rest are for whoever is looking at a list of robots.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Meta {
    /// The stable-identity key: same physical robot across reinstalls, renames and new tokens.
    ///
    /// The server evicts an older producer of the same user carrying the same value, which is how
    /// a re-flashed board or a restarted daemon stops showing up as a second robot. It is also
    /// what makes this robot sweepable at all.
    pub hardware_id: String,
    /// What a person sees. Absent when `configd` did not answer in time, as elsewhere.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// `microduck`, which is what lets one client list two families of robot without opening a
    /// session to ask what it found.
    pub kind: &'static str,
    /// The release this robot is running, for the reason the local `meta` carries it.
    pub release: String,
    pub api_version: u32,
}

impl Meta {
    /// This robot's `meta`, from what the local producer already learned.
    ///
    /// `hardware_id` falls back to `/etc/machine-id` when there is no SoC serial to read — a
    /// developer's laptop, or a board whose device tree has no `serial-number`. Stable per
    /// install rather than per robot, which is weaker and still correct for the purpose: it keeps
    /// one machine from being listed twice, and it keeps the producer sweepable. `sounds` makes
    /// exactly this substitution for exactly this reason.
    pub fn of(producer: &crate::producer::Producer, machine_id: Option<String>) -> Option<Self> {
        let hardware_id = producer
            .serial
            .clone()
            .or(machine_id)
            .or_else(|| read_machine_id(Path::new("/etc/machine-id")))?;
        Some(Self {
            hardware_id,
            name: producer.name.clone(),
            kind: "microduck",
            release: producer.release.clone(),
            api_version: producer.api_version,
        })
    }
}

fn read_machine_id(path: &Path) -> Option<String> {
    let id = std::fs::read_to_string(path).ok()?.trim().to_owned();
    (!id.is_empty()).then_some(id)
}

// ── the wire ─────────────────────────────────────────────────────────────────

/// What the service sends down the event stream.
///
/// Unknown types are a variant rather than an error: this is somebody else's service and it is
/// allowed to grow messages we do not handle. The ones named here are the ones acted on.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "type", rename_all = "camelCase")]
enum Inbound {
    Welcome(Welcome),
    /// A consumer wants a session. Refused in this slice; translated in the next.
    #[serde(rename_all = "camelCase")]
    StartSession {
        session_id: String,
    },
    /// The other side gave up, or the service ended it.
    #[serde(rename_all = "camelCase")]
    EndSession {
        session_id: Option<String>,
    },
    #[serde(other)]
    Other,
}

/// The first message on a healthy stream, and the only one that has to arrive.
///
/// **Two casings in one object**, which is the service's and not a mistake here: `peerId` is
/// camelCase like every other envelope field, and `recommended_heartbeat_interval_seconds` is
/// snake_case like every `meta` key. A blanket `rename_all` silently reads the cadence as absent
/// and falls back to five seconds — a robot that works while posting twice as often as asked, and
/// nothing anywhere says why. So that one field is named outright.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Welcome {
    /// The id this connection is known by, which `/api/robot-status` reports back. Kept so the
    /// split-brain poll can look for *this* robot rather than for any robot.
    peer_id: String,
    /// The account the token belongs to, as the service resolved it. Logged once: it is the
    /// answer to "whose robot does the service think this is".
    #[serde(default)]
    username: Option<String>,
    /// What the service asks for, in seconds. Absent on a service that does not say.
    #[serde(default, rename = "recommended_heartbeat_interval_seconds")]
    recommended_heartbeat_interval_seconds: Option<f64>,
}

impl Welcome {
    /// The cadence to post at: what was asked for, clamped, or the fallback.
    fn heartbeat(&self, timings: &Timings) -> Duration {
        let (min, max) = timings.heartbeat_bounds;
        match self.recommended_heartbeat_interval_seconds {
            Some(seconds) if seconds.is_finite() && seconds > 0.0 => {
                Duration::from_secs_f64(seconds).clamp(min, max)
            }
            _ => timings.heartbeat_fallback,
        }
    }
}

/// What this robot sends. `POST /send`, one object per request.
#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "camelCase")]
enum Outbound<'a> {
    /// Registration, and every heartbeat after it: the same message, which is why the lease is
    /// keyed on the request rather than on its contents.
    SetPeerStatus {
        roles: [&'static str; 1],
        meta: &'a Meta,
    },
    /// How this slice refuses a session it cannot serve yet.
    #[serde(rename_all = "camelCase")]
    EndSession {
        session_id: &'a str,
        reason: &'a str,
    },
}

/// What `/api/robot-status` answers. Only the ids are read.
#[derive(Debug, Deserialize)]
struct RobotStatus {
    #[serde(default)]
    robots: Vec<RobotStatusEntry>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct RobotStatusEntry {
    peer_id: String,
}

// ── the task ─────────────────────────────────────────────────────────────────

/// Why a connection ended, which is what decides how long to wait before the next one.
#[derive(Debug)]
enum Ended {
    /// The stream closed or went quiet, or a post failed. Ordinary; back off and reconnect.
    Reconnect(String),
    /// The service accepted the token and stopped listing this robot anyway. §3.4.
    SplitBrain,
    /// The service refused the token. Backing off does not fix this — a login does — so this
    /// waits on the token file instead of on a timer.
    Unauthorised,
}

/// The relay, as the task that owns the outward connection.
pub struct Relay {
    base: String,
    token_path: PathBuf,
    meta: Meta,
    client: reqwest::Client,
    timings: Timings,
}

impl Relay {
    /// Build one. Fails only if the HTTP client will not build, which means no TLS stack.
    pub fn new(
        base: impl Into<String>,
        token_path: impl Into<PathBuf>,
        meta: Meta,
    ) -> Option<Self> {
        let client = reqwest::Client::builder()
            .connect_timeout(Duration::from_secs(10))
            .user_agent(concat!("mediad/", env!("CARGO_PKG_VERSION")))
            .build()
            .map_err(|e| {
                tracing::error!(
                    error = %e,
                    "no HTTP client, so this robot cannot be reached from outside its network"
                );
            })
            .ok()?;
        Some(Self {
            base: base.into().trim_end_matches('/').to_owned(),
            token_path: token_path.into(),
            meta,
            client,
            timings: Timings::default(),
        })
    }

    /// Run on intervals other than the shipped ones. See [`Timings`]; tests only.
    #[doc(hidden)]
    pub fn with_timings(mut self, timings: Timings) -> Self {
        self.timings = timings;
        self
    }

    /// Stay registered for as long as this process runs.
    ///
    /// Never returns. Every failure is a reconnect, because there is no state here worth keeping
    /// across one: the service's view of this robot is rebuilt by the next `setPeerStatus`.
    pub async fn run(self) {
        let mut backoff = self.timings.backoff_start;
        loop {
            let Some(token) = self.token() else {
                // At `debug`: a robot nobody has signed in is not a robot with a problem, and
                // this is every thirty seconds forever.
                tracing::debug!(
                    path = %self.token_path.display(),
                    "no account token yet; this robot is reachable on its own network only"
                );
                tokio::time::sleep(self.timings.no_token_poll).await;
                continue;
            };

            match self.session(&token).await {
                Ended::Unauthorised => {
                    tracing::warn!(
                        "the rendezvous service refused this robot's account token; a new login \
                         is what fixes it"
                    );
                    tokio::time::sleep(self.timings.no_token_poll).await;
                }
                Ended::SplitBrain => {
                    // Reconnect immediately rather than backing off: the connection looked
                    // healthy, so there is nothing to wait for, and every second here is a
                    // second the robot is not reachable while believing it is.
                    tracing::warn!(
                        "the service no longer lists this robot although the stream was healthy; \
                         reconnecting"
                    );
                    backoff = self.timings.backoff_start;
                }
                Ended::Reconnect(why) => {
                    tracing::info!(%why, retry_in = ?backoff, "remote access is off; will retry");
                    tokio::time::sleep(jittered(backoff)).await;
                    backoff = (backoff * 2).min(self.timings.backoff_max);
                }
            }
        }
    }

    /// The access token, or `None` when this robot belongs to nobody.
    ///
    /// Reads the one field it needs and ignores the rest: the refresh token and the expiry are
    /// `updaterd`'s business, and a reader that deserialised the whole record would break on a
    /// field added there.
    fn token(&self) -> Option<String> {
        #[derive(Deserialize)]
        struct Credential {
            access_token: String,
        }
        let bytes = std::fs::read(&self.token_path).ok()?;
        match serde_json::from_slice::<Credential>(&bytes) {
            Ok(credential) if !credential.access_token.is_empty() => Some(credential.access_token),
            Ok(_) => None,
            Err(e) => {
                tracing::warn!(
                    path = %self.token_path.display(),
                    error = %e,
                    "the account credential does not parse; treating this robot as signed out"
                );
                None
            }
        }
    }

    /// One connection: open the stream, register, then hold the lease until something breaks.
    async fn session(&self, token: &str) -> Ended {
        let mut events = match self.open_stream(token).await {
            Ok(events) => events,
            Err(ended) => return ended,
        };

        let welcome = match self.await_welcome(&mut events).await {
            Ok(welcome) => welcome,
            Err(ended) => return ended,
        };

        // Registered *before* anything reports this robot as reachable, so no observer can see
        // "remote access enabled" while the service does not yet know the robot exists. §3.4.
        if let Err(ended) = self.set_peer_status(token).await {
            return ended;
        }
        let heartbeat = welcome.heartbeat(&self.timings);
        tracing::info!(
            peer_id = %welcome.peer_id,
            account = welcome.username.as_deref().unwrap_or("unknown"),
            ?heartbeat,
            "registered with the rendezvous service; this robot is reachable from outside its \
             network"
        );

        let mut heartbeats = tokio::time::interval(heartbeat);
        heartbeats.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        heartbeats.tick().await; // the first tick is immediate, and registration just happened
        let mut polls = tokio::time::interval(self.timings.status_poll);
        polls.tick().await;
        let mut misses = 0;

        loop {
            tokio::select! {
                _ = heartbeats.tick() => {
                    if let Err(ended) = self.set_peer_status(token).await {
                        return ended;
                    }
                }
                _ = polls.tick() => {
                    match self.lists_us(token, &welcome.peer_id).await {
                        Ok(true) => misses = 0,
                        Ok(false) => {
                            misses += 1;
                            tracing::warn!(
                                misses,
                                peer_id = %welcome.peer_id,
                                "the service did not list this robot"
                            );
                            if misses >= MISSES_BEFORE_RECONNECT {
                                return Ended::SplitBrain;
                            }
                        }
                        // A failed poll is not a miss: it says nothing about whether the service
                        // lists us, and treating it as one would reconnect a healthy stream
                        // every time the network hiccuped twice.
                        Err(why) => tracing::debug!(%why, "could not ask whether we are listed"),
                    }
                }
                event = tokio::time::timeout(self.timings.read_timeout, events.next()) => {
                    match event {
                        Err(_) => return Ended::Reconnect(format!(
                            "nothing arrived on the event stream for {:?}, which is two missed \
                             pings",
                            self.timings.read_timeout
                        )),
                        Ok(None) => return Ended::Reconnect(
                            "the service closed the event stream".to_owned(),
                        ),
                        Ok(Some(Err(e))) => return Ended::Reconnect(
                            format!("the event stream failed: {e}"),
                        ),
                        Ok(Some(Ok(message))) => {
                            if let Some(ended) = self.handle(token, message).await {
                                return ended;
                            }
                        }
                    }
                }
            }
        }
    }

    /// `GET /events`, as a stream of parsed messages.
    async fn open_stream(
        &self,
        token: &str,
    ) -> Result<impl futures_util::Stream<Item = Result<Inbound, String>> + Unpin, Ended> {
        let url = format!("{}/events", self.base);
        let response = self
            .client
            .get(&url)
            .bearer_auth(token)
            .header("accept", "text/event-stream")
            .send()
            .await
            .map_err(|e| Ended::Reconnect(format!("GET {url}: {e}")))?;

        if response.status() == reqwest::StatusCode::UNAUTHORIZED {
            return Err(Ended::Unauthorised);
        }
        if !response.status().is_success() {
            return Err(Ended::Reconnect(format!(
                "GET {url}: HTTP {}",
                response.status()
            )));
        }

        // `eventsource-stream` owns the framing: `data:` split across TCP reads, multi-line
        // payloads, comments and the fields we do not use. What is left here is JSON.
        let events = response
            .bytes_stream()
            .eventsource()
            .filter_map(|event| async move {
                match event {
                    Err(e) => Some(Err(format!("{e}"))),
                    // The service's keepalive carries no data and means only "still here".
                    Ok(event) if event.data.trim().is_empty() => None,
                    Ok(event) => Some(
                        serde_json::from_str::<Inbound>(&event.data)
                            .map_err(|e| format!("unparseable message: {e}")),
                    ),
                }
            });
        Ok(Box::pin(events))
    }

    /// Read until the welcome, which is the message that says the peer now exists.
    async fn await_welcome(
        &self,
        events: &mut (impl futures_util::Stream<Item = Result<Inbound, String>> + Unpin),
    ) -> Result<Welcome, Ended> {
        let deadline = tokio::time::Instant::now() + self.timings.welcome_timeout;
        loop {
            let event = tokio::time::timeout_at(deadline, events.next())
                .await
                .map_err(|_| {
                    Ended::Reconnect(format!(
                        "no welcome within {:?}",
                        self.timings.welcome_timeout
                    ))
                })?;
            match event {
                None => {
                    return Err(Ended::Reconnect(
                        "the stream closed before the welcome".to_owned(),
                    ));
                }
                Some(Err(why)) => {
                    // A message we cannot read is not a reason to drop a stream that is otherwise
                    // working; the welcome may be the next one.
                    tracing::debug!(%why, "skipping a message while waiting for the welcome");
                }
                Some(Ok(Inbound::Welcome(welcome))) => return Ok(welcome),
                Some(Ok(_)) => {}
            }
        }
    }

    /// Register, and refresh the lease. The same request does both.
    async fn set_peer_status(&self, token: &str) -> Result<(), Ended> {
        self.send(
            token,
            &Outbound::SetPeerStatus {
                roles: ["producer"],
                meta: &self.meta,
            },
        )
        .await
    }

    /// One `POST /send`.
    async fn send(&self, token: &str, message: &Outbound<'_>) -> Result<(), Ended> {
        let url = format!("{}/send", self.base);
        let response = self
            .client
            .post(&url)
            .bearer_auth(token)
            .timeout(REQUEST_TIMEOUT)
            .json(message)
            .send()
            .await
            .map_err(|e| Ended::Reconnect(format!("POST {url}: {e}")))?;

        match response.status() {
            status if status.is_success() => Ok(()),
            reqwest::StatusCode::UNAUTHORIZED => Err(Ended::Unauthorised),
            // 400 here means the peer does not exist — the stream this token was bound to is
            // gone. Reconnecting is what rebuilds it, and it is the whole reason the stream is
            // opened before anything is posted.
            status => Err(Ended::Reconnect(format!("POST {url}: HTTP {status}"))),
        }
    }

    /// Whether the service still lists this robot. §3.4, split-brain.
    async fn lists_us(&self, token: &str, peer_id: &str) -> Result<bool, String> {
        let url = format!("{}/api/robot-status", self.base);
        let response = self
            .client
            .get(&url)
            .bearer_auth(token)
            .timeout(REQUEST_TIMEOUT)
            .send()
            .await
            .map_err(|e| format!("GET {url}: {e}"))?;
        if !response.status().is_success() {
            return Err(format!("GET {url}: HTTP {}", response.status()));
        }
        let status: RobotStatus = response
            .json()
            .await
            .map_err(|e| format!("GET {url}: {e}"))?;
        Ok(status.robots.iter().any(|robot| robot.peer_id == peer_id))
    }

    /// One message from the service. `Some` ends the connection.
    async fn handle(&self, token: &str, message: Inbound) -> Option<Ended> {
        match message {
            // A second welcome on one stream would mean the service rebound this token, which is
            // what happens when another process registers with it. Reconnecting is how we find
            // out whose peer we are now.
            Inbound::Welcome(_) => Some(Ended::Reconnect(
                "the service sent a second welcome on the same stream".to_owned(),
            )),
            Inbound::StartSession { session_id } => {
                // Refused rather than dropped: an unanswered `startSession` leaves this robot
                // showing as busy to its owner with nothing on the other end. §8's slice 3 is
                // what replaces this with a translated session.
                tracing::info!(
                    %session_id,
                    "a remote session was requested; refusing it — the bridge is not built yet"
                );
                self.send(
                    token,
                    &Outbound::EndSession {
                        session_id: &session_id,
                        reason: "this robot cannot serve a remote session yet",
                    },
                )
                .await
                .err()
            }
            Inbound::EndSession { session_id } => {
                tracing::debug!(?session_id, "the service ended a session");
                None
            }
            Inbound::Other => None,
        }
    }
}

/// A duration plus up to [`BACKOFF_JITTER`] of itself, so a fleet does not reconnect in lockstep.
fn jittered(base: Duration) -> Duration {
    let spread = base.mul_f64(BACKOFF_JITTER);
    base + spread.mul_f64(rand::random::<f64>())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meta() -> Meta {
        Meta {
            hardware_id: "3fa1c51b".to_owned(),
            name: Some("olducky".to_owned()),
            kind: "microduck",
            release: "0.10.0".to_owned(),
            api_version: duck_ipc_proto::API_VERSION,
        }
    }

    /// The cadence the welcome asks for is used, and a service that asks for something absurd is
    /// clamped rather than obeyed.
    #[test]
    fn the_heartbeat_cadence_is_the_services_within_reason() {
        let welcome = |seconds: Option<f64>| Welcome {
            peer_id: "peer-1".to_owned(),
            username: None,
            recommended_heartbeat_interval_seconds: seconds,
        };

        let timings = Timings::default();
        assert_eq!(
            welcome(Some(10.0)).heartbeat(&timings),
            Duration::from_secs(10),
            "what this service actually publishes"
        );
        assert_eq!(
            welcome(None).heartbeat(&timings),
            HEARTBEAT_FALLBACK,
            "a service that says nothing gets a sixth of the lease"
        );
        assert_eq!(
            welcome(Some(0.001)).heartbeat(&timings),
            HEARTBEAT_BOUNDS.0,
            "a request storm is refused"
        );
        assert_eq!(
            welcome(Some(600.0)).heartbeat(&timings),
            HEARTBEAT_BOUNDS.1,
            "and so is a cadence slower than the lease it is meant to refresh"
        );
        assert_eq!(
            welcome(Some(f64::NAN)).heartbeat(&timings),
            HEARTBEAT_FALLBACK
        );
        assert_eq!(welcome(Some(-1.0)).heartbeat(&timings), HEARTBEAT_FALLBACK);
    }

    /// The messages this robot has to recognise, as the service spells them.
    #[test]
    fn the_wire_is_read_as_the_service_writes_it() {
        let welcome: Inbound = serde_json::from_str(
            r#"{"type":"welcome","peerId":"p-1","username":"PierreRouanet",
                "recommended_heartbeat_interval_seconds":10.0}"#,
        )
        .unwrap();
        let Inbound::Welcome(welcome) = welcome else {
            panic!("{welcome:?}");
        };
        assert_eq!(welcome.peer_id, "p-1");
        assert_eq!(welcome.username.as_deref(), Some("PierreRouanet"));
        assert_eq!(
            welcome.heartbeat(&Timings::default()),
            Duration::from_secs(10)
        );

        assert_eq!(
            serde_json::from_str::<Inbound>(
                r#"{"type":"startSession","peerId":"p-2","sessionId":"s-1"}"#
            )
            .unwrap(),
            Inbound::StartSession {
                session_id: "s-1".to_owned()
            },
        );
        // Messages this slice does not act on must not be errors: it is somebody else's service
        // and it is allowed to grow.
        for other in [
            r#"{"type":"list","producers":[]}"#,
            r#"{"type":"peerStatusChanged","peerId":"p-1","roles":["producer"],"meta":{}}"#,
            r#"{"type":"sessionRejected","reason":"robot_busy","activeApp":"whatever"}"#,
            r#"{"type":"somethingAddedNextYear"}"#,
        ] {
            assert_eq!(
                serde_json::from_str::<Inbound>(other).unwrap(),
                Inbound::Other,
                "{other}"
            );
        }
    }

    /// What is posted, spelled the way the server reads it.
    #[test]
    fn registration_says_producer_and_carries_the_stable_id() {
        let meta = meta();
        let json = serde_json::to_value(Outbound::SetPeerStatus {
            roles: ["producer"],
            meta: &meta,
        })
        .unwrap();

        assert_eq!(json["type"], "setPeerStatus");
        assert_eq!(json["roles"][0], "producer");
        assert_eq!(
            json["meta"]["hardware_id"], "3fa1c51b",
            "the key the server sweeps and evicts on — snake_case, as it reads it"
        );
        assert_eq!(json["meta"]["kind"], "microduck");
        assert_eq!(json["meta"]["name"], "olducky");

        let json = serde_json::to_value(Outbound::EndSession {
            session_id: "s-1",
            reason: "nope",
        })
        .unwrap();
        assert_eq!(json["type"], "endSession");
        assert_eq!(
            json["sessionId"], "s-1",
            "camelCase, as the server sends it"
        );
    }

    /// A robot with no serial still gets a stable id, because a producer without one is never
    /// swept — it would haunt its owner's robot list after a crash.
    #[test]
    fn a_board_with_no_serial_falls_back_to_the_machine_id() {
        let producer = crate::producer::Producer {
            name: Some("olducky".to_owned()),
            serial: None,
            release: "0.10.0".to_owned(),
            api_version: duck_ipc_proto::API_VERSION,
        };

        let meta = Meta::of(&producer, Some("machine-1".to_owned())).expect("a stable id");
        assert_eq!(meta.hardware_id, "machine-1");

        let with_serial = crate::producer::Producer {
            serial: Some("3fa1c51b".to_owned()),
            ..producer
        };
        assert_eq!(
            Meta::of(&with_serial, Some("machine-1".to_owned()))
                .unwrap()
                .hardware_id,
            "3fa1c51b",
            "the serial wins: it survives a reinstall, and the machine id does not"
        );
    }

    #[test]
    fn an_empty_machine_id_file_is_no_machine_id() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("machine-id");
        std::fs::write(&path, "\n").unwrap();
        assert_eq!(read_machine_id(&path), None);
        std::fs::write(&path, "  abc123 \n").unwrap();
        assert_eq!(read_machine_id(&path), Some("abc123".to_owned()));
    }

    /// The token file is `updaterd`'s, and this reads one field out of it — including when the
    /// record grows fields this does not know.
    #[test]
    fn the_credential_is_read_for_the_one_field_that_matters() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("hf-token");
        let relay = Relay::new("http://127.0.0.1:1", &path, meta()).unwrap();

        assert_eq!(
            relay.token(),
            None,
            "no file is a robot signed in to nobody"
        );

        std::fs::write(
            &path,
            r#"{"access_token":"hf_abc","refresh_token":"r","expires_at":1,"username":"x",
                "something_added_later":true}"#,
        )
        .unwrap();
        assert_eq!(relay.token().as_deref(), Some("hf_abc"));

        std::fs::write(&path, r#"{"refresh_token":"only"}"#).unwrap();
        assert_eq!(
            relay.token(),
            None,
            "a record with no access token is no use"
        );

        std::fs::write(&path, "{ not json").unwrap();
        assert_eq!(
            relay.token(),
            None,
            "and a corrupt one is signed out, not fatal"
        );
    }

    // ── against a fake rendezvous service ───────────────────────────────────
    //
    // The four failure modes §3.4 names are all timing failures, and this is where they are
    // reproduced on demand: a service that stops listing a robot whose stream is fine, a lease
    // that has to be refreshed by traffic rather than by a socket looking healthy, a session
    // arriving that this slice cannot serve, and a token that is not there yet.

    /// A stand-in for `reachy_mini_central`, holding what it was told and what it will say next.
    struct FakeService {
        base: String,
        state: std::sync::Arc<Fake>,
        _task: tokio::task::JoinHandle<()>,
    }

    #[derive(Default)]
    struct Fake {
        /// Every `POST /send` body, in order.
        posts: std::sync::Mutex<Vec<serde_json::Value>>,
        /// How many times the event stream has been opened.
        streams: std::sync::atomic::AtomicUsize,
        /// Whether `/api/robot-status` admits this robot exists.
        lists_us: std::sync::atomic::AtomicBool,
        /// What the welcome asks for, and messages to push after it.
        heartbeat_seconds: std::sync::Mutex<Option<f64>>,
        push: std::sync::Mutex<Option<tokio::sync::mpsc::UnboundedSender<String>>>,
        /// A status to answer `POST /send` with instead of 200.
        refuse_posts: std::sync::Mutex<Option<u16>>,
    }

    impl Fake {
        fn posts(&self) -> Vec<serde_json::Value> {
            self.posts.lock().unwrap().clone()
        }

        fn of_type(&self, kind: &str) -> Vec<serde_json::Value> {
            self.posts()
                .into_iter()
                .filter(|post| post["type"] == kind)
                .collect()
        }

        /// Push a message down the open stream, as the service would.
        fn push(&self, message: serde_json::Value) {
            let sender = self.push.lock().unwrap().clone().expect("a stream is open");
            sender
                .send(message.to_string())
                .expect("the stream is live");
        }
    }

    const PEER_ID: &str = "peer-under-test";

    async fn fake_service() -> FakeService {
        use axum::extract::State;
        use axum::routing::{get, post};

        let state = std::sync::Arc::new(Fake::default());
        state
            .lists_us
            .store(true, std::sync::atomic::Ordering::SeqCst);

        let app = axum::Router::new()
            .route(
                "/events",
                get(|State(fake): State<std::sync::Arc<Fake>>| async move {
                    fake.streams
                        .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<String>();
                    *fake.push.lock().unwrap() = Some(tx);

                    let cadence = *fake.heartbeat_seconds.lock().unwrap();
                    let welcome = match cadence {
                        Some(seconds) => serde_json::json!({
                            "type": "welcome",
                            "peerId": PEER_ID,
                            "username": "PierreRouanet",
                            "recommended_heartbeat_interval_seconds": seconds,
                        }),
                        None => serde_json::json!({
                            "type": "welcome", "peerId": PEER_ID, "username": "PierreRouanet",
                        }),
                    };

                    // The framing the real service uses: `data:` lines, and a comment-only ping
                    // when there is nothing to say.
                    let events = async_stream::stream! {
                        yield Ok::<_, std::io::Error>(format!("data: {welcome}\n\n"));
                        while let Some(message) = rx.recv().await {
                            yield Ok(format!("data: {message}\n\n"));
                        }
                    };
                    (
                        [("content-type", "text/event-stream")],
                        axum::body::Body::from_stream(events),
                    )
                }),
            )
            .route(
                "/send",
                post(
                    |State(fake): State<std::sync::Arc<Fake>>, body: String| async move {
                        let message: serde_json::Value =
                            serde_json::from_str(&body).expect("the relay posts JSON");
                        fake.posts.lock().unwrap().push(message);
                        match *fake.refuse_posts.lock().unwrap() {
                            None => axum::http::StatusCode::OK,
                            Some(status) => axum::http::StatusCode::from_u16(status).unwrap(),
                        }
                    },
                ),
            )
            .route(
                "/api/robot-status",
                get(|State(fake): State<std::sync::Arc<Fake>>| async move {
                    let robots = if fake.lists_us.load(std::sync::atomic::Ordering::SeqCst) {
                        serde_json::json!([{ "peerId": PEER_ID, "busy": false }])
                    } else {
                        serde_json::json!([])
                    };
                    axum::Json(serde_json::json!({ "robots": robots }))
                }),
            )
            .with_state(std::sync::Arc::clone(&state));

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        let task = tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });
        FakeService {
            base,
            state,
            _task: task,
        }
    }

    /// Intervals small enough that a test does not wait for a robot's afternoon.
    fn brisk() -> Timings {
        Timings {
            no_token_poll: Duration::from_millis(50),
            read_timeout: Duration::from_secs(5),
            welcome_timeout: Duration::from_secs(2),
            heartbeat_fallback: Duration::from_millis(50),
            heartbeat_bounds: (Duration::from_millis(20), Duration::from_secs(60)),
            status_poll: Duration::from_millis(50),
            backoff_start: Duration::from_millis(20),
            backoff_max: Duration::from_millis(50),
        }
    }

    fn signed_in(dir: &tempfile::TempDir) -> PathBuf {
        let path = dir.path().join("hf-token");
        std::fs::write(
            &path,
            r#"{"access_token":"hf_abc","username":"PierreRouanet"}"#,
        )
        .unwrap();
        path
    }

    /// Wait for something to become true, or fail saying what never happened.
    async fn until(what: &str, mut ready: impl FnMut() -> bool) {
        for _ in 0..200 {
            if ready() {
                return;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        panic!("{what} did not happen");
    }

    /// The whole of slice 2: the stream opens, the welcome arrives, the robot registers as a
    /// producer, and the lease keeps being refreshed after that.
    ///
    /// The ordering assertion is the one worth having: **nothing is posted before the welcome**.
    /// `POST /send` on this service is a 400 until the token has been bound to a peer by the
    /// event stream, so a relay that registered first would work only by accident of scheduling.
    #[tokio::test]
    async fn it_registers_as_a_producer_and_holds_the_lease() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        *service.state.heartbeat_seconds.lock().unwrap() = Some(0.05);

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk());
        let task = tokio::spawn(relay.run());

        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        let first = service.state.of_type("setPeerStatus")[0].clone();
        assert_eq!(first["roles"][0], "producer");
        assert_eq!(first["meta"]["hardware_id"], "3fa1c51b");
        assert_eq!(first["meta"]["kind"], "microduck");
        assert_eq!(
            service
                .state
                .streams
                .load(std::sync::atomic::Ordering::SeqCst),
            1,
            "one stream, and it was opened before anything was posted"
        );

        // The lease is refreshed by traffic, not by a socket that looks healthy: a service that
        // saw one post and then silence would evict this robot after thirty seconds.
        until("a second heartbeat", || {
            service.state.of_type("setPeerStatus").len() >= 3
        })
        .await;
        task.abort();
    }

    /// A session this slice cannot serve is refused by name, not ignored.
    ///
    /// An unanswered `startSession` leaves the robot showing as busy to its owner with nothing on
    /// the other end — the failure that looks like a broken robot rather than a missing feature.
    #[tokio::test]
    async fn a_session_it_cannot_serve_yet_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk());
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "startSession", "peerId": "a-consumer", "sessionId": "session-1",
        }));

        until("a refusal", || {
            !service.state.of_type("endSession").is_empty()
        })
        .await;
        let refusal = service.state.of_type("endSession")[0].clone();
        assert_eq!(refusal["sessionId"], "session-1");
        assert!(
            refusal["reason"].as_str().unwrap().contains("yet"),
            "the reason should say this is unbuilt rather than broken: {refusal}"
        );
        task.abort();
    }

    /// Split-brain: the stream is healthy and the service has forgotten us.
    ///
    /// Nothing in the connection notices, which is the entire problem — so the poll is what
    /// notices, and two consecutive misses force a reconnect. One miss must not: a single lost
    /// answer says nothing about whether we are listed.
    #[tokio::test]
    async fn a_service_that_stops_listing_this_robot_is_reconnected() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk());
        let task = tokio::spawn(relay.run());
        until("the first stream", || {
            service
                .state
                .streams
                .load(std::sync::atomic::Ordering::SeqCst)
                >= 1
        })
        .await;

        service
            .state
            .lists_us
            .store(false, std::sync::atomic::Ordering::SeqCst);

        until("a reconnect", || {
            service
                .state
                .streams
                .load(std::sync::atomic::Ordering::SeqCst)
                >= 2
        })
        .await;
        task.abort();
    }

    /// A robot nobody has signed in does not talk to anybody, and starts as soon as it is.
    ///
    /// This is the ordinary state of a duck out of a box, so it must be quiet — and the token has
    /// to be picked up without a restart, because the login that writes it happens over BLE while
    /// this task is already running.
    #[tokio::test]
    async fn nothing_happens_until_the_robot_belongs_to_somebody() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("hf-token");
        let service = fake_service().await;

        let relay = Relay::new(&service.base, &path, meta())
            .unwrap()
            .with_timings(brisk());
        let task = tokio::spawn(relay.run());

        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(
            service
                .state
                .streams
                .load(std::sync::atomic::Ordering::SeqCst),
            0,
            "a robot with no account must not reach the service at all"
        );

        // The login lands, over BLE, while this is running.
        std::fs::write(&path, r#"{"access_token":"hf_abc"}"#).unwrap();
        until("registration after a login", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;
        task.abort();
    }

    /// A token the service refuses is not something backing off can fix.
    ///
    /// It waits on the file instead: the remedy is a login, and hammering a 401 every five
    /// seconds until somebody performs one is a request storm against somebody else's Space.
    #[tokio::test]
    async fn a_refused_token_waits_for_a_new_one_rather_than_hammering() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        *service.state.refuse_posts.lock().unwrap() = Some(401);

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(Timings {
                no_token_poll: Duration::from_secs(30),
                ..brisk()
            });
        let task = tokio::spawn(relay.run());

        until("the first attempt", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert_eq!(
            service.state.of_type("setPeerStatus").len(),
            1,
            "a 401 must not be retried on the reconnect timer"
        );
        task.abort();
    }

    /// Jitter is added, and it never shortens the wait.
    #[test]
    fn backoff_jitter_only_ever_adds() {
        for _ in 0..100 {
            let waited = jittered(Duration::from_secs(10));
            assert!(waited >= Duration::from_secs(10), "{waited:?}");
            assert!(waited <= Duration::from_secs(11), "{waited:?}");
        }
    }
}
