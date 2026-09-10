//! The robot's half of the bridge to the rendezvous service.
//!
//! A LAN client reaches `webrtcsink`'s signalling server directly and nothing here is involved.
//! This is what makes a duck reachable from *outside* its network: it connects **outward** to
//! `reachy_mini_central` holding the account token, registers as a **producer**, and the service
//! shows a client only the robots its own account owns. `docs/design/remote-access-design.md` §3
//! owns the argument; this module owns the connection.
//!
//! # What it does
//!
//! **Registration and liveness** (§8, slice 2): the robot appears in the service's listing and
//! stays there, for as long as the account token on disk is the one it connected with.
//!
//! **And it carries a session** (slice 4). A remote consumer cannot reach `webrtcsink`'s
//! signalling server — it is on a loopback address behind somebody's router — so when one asks
//! the rendezvous for a session, [`bridge`] opens that socket *as the consumer on its behalf* and
//! carries envelopes between the two, rewriting the `sessionId` each hop names and reading none
//! of the payload. One session at a time; a second is refused by name.
//!
//! # A control lane that needs no candidate pair, and why there is one
//!
//! The bridge above carries a *negotiation*: SDP and ICE, so that a consumer and the robot can
//! find a path between them and speak WebRTC over it. When they cannot find one, everything built
//! on it is gone — and right now they frequently cannot, because a relay candidate needs
//! `turn.fastrtc.org`, which has no A record and whose zone has no NS records (§6). Signalling
//! crosses, media does not, and the control channel goes with it because SCTP rides the same
//! candidate pair.
//!
//! So the JSON-RPC a consumer wants to send does not have to go through WebRTC at all, and the
//! rendezvous turns out to already carry it: `handle_peer_message` in their `app.py` relays
//! **every key of a `peer` envelope except `type` and `sessionId`** verbatim to the session
//! partner, without looking at `sdp` or `ice`. A `peer` message carrying an `rpc` key is therefore
//! a control call, relayed opaquely, with no change to a service the mini fleet also depends on.
//!
//! ```text
//!   consumer ──POST /send {type:peer, sessionId, rpc:{…}}──► rendezvous ──SSE──► this relay
//!            ◄─────────── SSE {type:peer, sessionId, rpc:{…}} ◄──POST /send────────┘
//! ```
//!
//! `session::run` is what answers it, unchanged: its own header says it is transport-agnostic so
//! that "a WebSocket surface could reuse it unchanged", and this is that surface. Same routing
//! table, same per-lane sockets, same refusal to parse a reply. No ICE, no DTLS, no TURN.
//!
//! **What it is not is a teleop lane.** The rendezvous allows 1200 requests per 60 s per peer, so
//! roughly twenty a second shared with the heartbeat — four calls to install and run a policy is
//! nothing, and a 50 Hz intent stream is over budget in a second. Worse, exceeding it earns a
//! `429` on the *whole peer*, which would take the robot's own lease down with it: a client could
//! knock a robot off the rendezvous by subscribing to telemetry. Hence [`Budget`], which bounds
//! notifications and never a reply — replies are one-per-request and so already bounded by
//! whatever the client itself can afford to send.
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
/// **A cross-daemon file format, and `hf_robot_account` owns it.** `updaterd` performs the login
/// and that crate writes the file; a test there pins the one key `read_access_token` takes out of
/// it, because the writer is what can break the contract.
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

/// How long the robot's own signalling server gets to hand over a session.
///
/// It is in this process — `webrtcsink` runs it — so this is generous rather than tuned. What it
/// guards against is a pipeline that never reached PLAYING, which produces a server with no
/// producer and a handshake that would otherwise wait for one forever.
const LOCAL_HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(10);

/// Where the robot's own signalling server listens.
///
/// `webrtcsink` runs it in this process with `signalling-server-host` and `-port`, so this is the
/// same 8443 `mediad`'s `--port` defaults to. Loopback, because the bridge and the server are one
/// process — a remote peer reaches this robot through the rendezvous, never through this socket.
pub const DEFAULT_LOCAL_SIGNALLING: &str = "ws://127.0.0.1:8443";

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
///
/// **`Peer` carries the whole envelope rather than its fields**, and that is the design rather
/// than laziness: an SDP or an ICE candidate passes through this process untouched, and the only
/// thing rewritten is the `sessionId` around it. Deserialising the payload would mean owning a
/// copy of a schema that belongs to WebRTC, and re-serialising it would mean re-encoding SDP that
/// arrived perfectly good. §3.2 — a translator with an opaque payload.
#[derive(Debug, Clone, PartialEq)]
enum Inbound {
    Welcome(Welcome),
    /// A consumer wants a session, which is what the bridge exists to serve.
    StartSession {
        session_id: String,
    },
    /// The other side gave up, or the service ended it.
    EndSession {
        session_id: Option<String>,
    },
    /// SDP or ICE for a session in flight.
    Peer(serde_json::Value),
    Other,
}

/// Read a message off the wire far enough to route it, and no further.
fn classify(raw: &str) -> Result<Inbound, String> {
    let value: serde_json::Value =
        serde_json::from_str(raw).map_err(|e| format!("unparseable message: {e}"))?;
    let session_id = |value: &serde_json::Value| value["sessionId"].as_str().map(str::to_owned);
    Ok(match value["type"].as_str().unwrap_or_default() {
        "welcome" => Inbound::Welcome(
            serde_json::from_value(value)
                .map_err(|e| format!("a welcome this page cannot read: {e}"))?,
        ),
        "startSession" => match session_id(&value) {
            Some(session_id) => Inbound::StartSession { session_id },
            // A `startSession` with no id is not something to answer: there is nothing to answer
            // *about*, and inventing one would open a session the service cannot route.
            None => return Err("a startSession with no sessionId".to_owned()),
        },
        "endSession" => Inbound::EndSession {
            session_id: session_id(&value),
        },
        "peer" => Inbound::Peer(value),
        _ => Inbound::Other,
    })
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
    /// The credential on disk is not the one this connection is using: a `logout`, or a
    /// `login --force` onto another account.
    ///
    /// **This is what makes `account.logout` mean anything here.** The token is read once per
    /// connection, so without this the relay would go on refreshing the lease with a credential
    /// its owner had deleted — a robot signed out of an account and still listed under it, which
    /// is precisely the claim `remote-access-design.md` §2.6 makes about being revocable.
    /// Dropping the stream is the deregistration: a clean disconnect evicts the peer at once, and
    /// the 30 s sweep is only there for sockets that never report closing.
    CredentialChanged,
}

/// The relay, as the task that owns the outward connection.
pub struct Relay {
    base: String,
    token_path: PathBuf,
    meta: Meta,
    client: reqwest::Client,
    timings: Timings,
    local_signalling: String,
    /// What the pipeline knows, for the two calls [`crate::session::run`] answers itself.
    ///
    /// **A watch rather than a value, because the relay starts before the answer exists.** It is
    /// spawned deliberately early — a robot that appears in its owner's list and cannot stream is
    /// still a robot somebody can reach to find out why — while the frame geometry is only
    /// truthful *after* the pipeline is up, since which sensor mode is in force is not known until
    /// something has tried to set it. So `main` hands over a receiver and fills it in later, and a
    /// lane reads whatever is current when it opens. Empty means `media.video` is refused rather
    /// than answered with zeros.
    video: Option<tokio::sync::watch::Receiver<Option<crate::session::Media>>>,
    /// Where each service listens, for the control lane's pool. Overridable for the same reason
    /// `--rendezvous-url` is: the whole of this module is meant to be exercisable on a laptop,
    /// and a lane whose sockets were hardcoded to `/run/robot` could only be tested on a board.
    sockets: crate::upstream::Sockets,
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
            local_signalling: DEFAULT_LOCAL_SIGNALLING.to_owned(),
            video: None,
            sockets: Default::default(),
        })
    }

    /// Where the services this lane routes to are listening.
    pub fn with_sockets(mut self, sockets: crate::upstream::Sockets) -> Self {
        self.sockets = sockets;
        self
    }

    /// Where to read the video's geometry when a control lane opens.
    ///
    /// A builder rather than a fourth argument to [`Relay::new`] because the video is discovered
    /// later than the relay is built, and because every test here is about the wire rather than
    /// about the camera.
    pub fn with_video(
        mut self,
        video: tokio::sync::watch::Receiver<Option<crate::session::Media>>,
    ) -> Self {
        self.video = Some(video);
        self
    }

    /// Point the bridge at a signalling server other than `webrtcsink`'s own.
    ///
    /// For tests, and for a `mediad` whose `--port` is not the default: the bridge connects to
    /// the server this same process is running, so the two numbers have to agree.
    pub fn with_local_signalling(mut self, url: impl Into<String>) -> Self {
        self.local_signalling = url.into();
        self
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
                Ended::CredentialChanged => {
                    // No backoff: either there is a new token to use immediately, or there is
                    // none and the loop above is about to wait on the file anyway.
                    tracing::info!(
                        "this robot's account credential changed; the rendezvous connection is \
                         dropped, which takes the robot out of the service's listing"
                    );
                    backoff = self.timings.backoff_start;
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
    /// `hf_robot_account` owns the reading of it: it is the crate that writes the file, and the
    /// TURN credentials need the same token out of the same place.
    fn token(&self) -> Option<String> {
        hf_robot_account::read_access_token(&self.token_path)
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

        // At most one at a time. The service gates this too (`sessionRejected` with `robot_busy`),
        // so this is belt-and-braces — and it stays, because two remote peers writing into one
        // intent slot is `remote-webrtc.md` §9's interleaving bug with the pad replaced by a
        // second continent. §3.4.
        let mut bridged: Option<Bridged> = None;

        // The control lane, which is independent of the one above: a consumer may hold one, the
        // other, or both. Built on the first `rpc` envelope of a session and dropped with it.
        let mut control: Option<Control> = None;

        let mut heartbeats = tokio::time::interval(heartbeat);
        heartbeats.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        heartbeats.tick().await; // the first tick is immediate, and registration just happened
        let mut polls = tokio::time::interval(self.timings.status_poll);
        polls.tick().await;
        let mut misses = 0;

        loop {
            tokio::select! {
                _ = heartbeats.tick() => {
                    // Checked here rather than on its own timer: the heartbeat is already the
                    // fastest thing in this loop, so a `logout` takes effect within one cadence
                    // — ten seconds — and re-reading a small file that often costs nothing.
                    if self.token().as_deref() != Some(token) {
                        return Ended::CredentialChanged;
                    }
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
                // A session that ended on its own — the peer left, the pipeline stopped — is
                // reaped here rather than being noticed the next time one is asked for, so
                // `account.status` and the service agree about whether this robot is busy.
                _ = async {
                    match bridged.as_mut() {
                        Some(session) => (&mut session.task).await.ok(),
                        // Nothing to wait for; this branch must never be the one that fires.
                        None => std::future::pending().await,
                    }
                }, if bridged.is_some() => {
                    tracing::debug!("the bridged session finished");
                    bridged = None;
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
                            if let Some(ended) =
                                self.handle(token, message, &mut bridged, &mut control).await
                            {
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
                    Ok(event) => Some(classify(&event.data)),
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
    async fn handle(
        &self,
        token: &str,
        message: Inbound,
        bridged: &mut Option<Bridged>,
        control: &mut Option<Control>,
    ) -> Option<Ended> {
        match message {
            // A second welcome on one stream would mean the service rebound this token, which is
            // what happens when another process registers with it. Reconnecting is how we find
            // out whose peer we are now.
            Inbound::Welcome(_) => Some(Ended::Reconnect(
                "the service sent a second welcome on the same stream".to_owned(),
            )),
            Inbound::StartSession { session_id } => {
                if bridged.as_ref().is_some_and(Bridged::live) {
                    // Refused by name rather than dropped: an unanswered `startSession` leaves a
                    // peer waiting on a robot that is never going to answer, and the owner
                    // looking at a robot that reads as broken rather than busy.
                    tracing::info!(%session_id, "refusing a second remote session");
                    return self
                        .send(
                            token,
                            &Outbound::EndSession {
                                session_id: &session_id,
                                reason: "this robot is already in a remote session",
                            },
                        )
                        .await
                        .err();
                }

                let (to_local, from_remote) = tokio::sync::mpsc::channel(64);
                let hub = Hub {
                    client: self.client.clone(),
                    base: self.base.clone(),
                    token: token.to_owned(),
                };
                let local_url = self.local_signalling.clone();
                let remote_id = session_id.clone();
                let task = tokio::spawn(async move {
                    if let Err(why) = bridge(hub, local_url, remote_id, from_remote).await {
                        tracing::warn!(%why, "a remote session could not be bridged");
                    }
                });
                *bridged = Some(Bridged {
                    remote_id: session_id,
                    to_local,
                    task,
                });
                None
            }
            Inbound::Peer(envelope) => {
                // **`rpc` first, because it needs no session to have been negotiated.** A
                // consumer that never intends to speak WebRTC still had to `startSession` to get
                // a session id — `handle_peer_message` drops an envelope naming a session the
                // service does not know — but it has no offer to send and none to answer. So a
                // control call is dispatched before the bridge is consulted, and a robot whose
                // media never connected still answers.
                if !envelope["rpc"].is_null() {
                    return self.control(token, &envelope, control).await;
                }

                // The envelope is forwarded whole and the payload is never read. What decides
                // where it goes is the session it names — and a `peer` for a session this robot
                // is not in is dropped rather than guessed at.
                let Some(session) = bridged.as_ref() else {
                    tracing::debug!("a peer message arrived with no session to carry it");
                    return None;
                };
                let names_it = envelope["sessionId"].as_str() == Some(session.remote_id.as_str());
                if !names_it {
                    tracing::debug!(
                        session = ?envelope["sessionId"].as_str(),
                        "a peer message for another session; dropping it"
                    );
                    return None;
                }
                if session.to_local.send(envelope).await.is_err() {
                    tracing::debug!("the bridged session went away before its message arrived");
                    *bridged = None;
                }
                None
            }
            Inbound::EndSession { session_id } => {
                // Dropping `Bridged` aborts the task, which closes the local socket — that is
                // what tells `webrtcsink` the consumer is gone.
                if control
                    .as_ref()
                    .is_some_and(|c| session_id.as_deref().is_none_or(|id| id == c.remote_id))
                {
                    tracing::info!(?session_id, "the service ended the control lane");
                    *control = None;
                }
                if bridged
                    .as_ref()
                    .is_some_and(|s| session_id.as_deref().is_none_or(|id| id == s.remote_id))
                {
                    tracing::info!(?session_id, "the service ended the bridged session");
                    *bridged = None;
                } else if control.is_none() {
                    tracing::debug!(?session_id, "the service ended a session we do not have");
                }
                None
            }
            Inbound::Other => None,
        }
    }
}

// ── the control lane: JSON-RPC over the rendezvous, with no candidate pair ───

/// How many notifications a control lane may post per window.
///
/// The rendezvous allows 1200 requests per 60 s **per peer**, and exceeding it earns a `429` on
/// everything that token does — including the heartbeat that holds this robot's lease. So a
/// consumer subscribing to 50 Hz telemetry would not merely get a slow stream, it would take the
/// robot off its owner's listing. Half the allowance is left for the heartbeat, the status poll
/// and whatever a person is actually doing.
const NOTIFICATIONS_PER_WINDOW: u32 = 400;
const NOTIFICATION_WINDOW: Duration = Duration::from_secs(60);

/// A sliding allowance for lines nobody asked for.
///
/// **Replies are deliberately not subject to it.** One reply answers one request, and a request
/// cost the consumer a `POST` of its own, so replies are already bounded by whatever the consumer
/// can afford — throttling them would only break callers. Notifications have no such bound: a
/// subscription is one request and then an unbounded stream.
struct Budget {
    allowance: u32,
    window: Duration,
    spent: u32,
    dropped: u64,
    since: tokio::time::Instant,
}

impl Budget {
    fn new(allowance: u32, window: Duration) -> Self {
        Self {
            allowance,
            window,
            spent: 0,
            dropped: 0,
            since: tokio::time::Instant::now(),
        }
    }

    /// Whether one notification may go out now.
    fn take(&mut self) -> bool {
        let now = tokio::time::Instant::now();
        if now.duration_since(self.since) >= self.window {
            if self.dropped > 0 {
                tracing::info!(
                    dropped = self.dropped,
                    "notifications dropped on the control lane: it is not a telemetry transport, \
                     and the alternative is a rate limit that would end this robot's lease"
                );
                self.dropped = 0;
            }
            self.spent = 0;
            self.since = now;
        }
        if self.spent < self.allowance {
            self.spent += 1;
            true
        } else {
            self.dropped += 1;
            false
        }
    }
}

/// One consumer's control lane, and the task that owns both its halves.
struct Control {
    /// The id the *service* knows this session by, which is what every envelope has to name.
    remote_id: String,
    /// Lines on their way into [`crate::session::run`].
    to_session: tokio::sync::mpsc::Sender<String>,
    task: tokio::task::JoinHandle<()>,
}

impl Control {
    fn live(&self) -> bool {
        !self.task.is_finished()
    }
}

impl Drop for Control {
    fn drop(&mut self) {
        // The task owns the session and its pool of unix sockets; aborting it closes every one,
        // which is what ends a subscription a consumer walked away from.
        self.task.abort();
    }
}

impl Relay {
    /// One `peer` envelope carrying `rpc`.
    ///
    /// Opens the lane if this session has none yet, which is what makes a control-only consumer
    /// need no cooperation from the media path: it says `startSession`, the service gives it an
    /// id, and its first call is what builds everything on this side.
    async fn control(
        &self,
        token: &str,
        envelope: &serde_json::Value,
        control: &mut Option<Control>,
    ) -> Option<Ended> {
        let Some(session_id) = envelope["sessionId"].as_str() else {
            tracing::debug!("an rpc envelope with no sessionId; dropping it");
            return None;
        };

        // A new session supersedes an old lane rather than being refused. Unlike a media session
        // — where two consumers writing into one intent slot is `remote-webrtc.md` §9's
        // interleaving bug — a lane holds no hardware: it is a socket per service, and the
        // service's own concurrency gate already means one consumer at a time.
        let reusable = control
            .as_ref()
            .is_some_and(|c| c.remote_id == session_id && c.live());
        if !reusable {
            *control = Some(self.open_control(token, session_id));
        }

        // The payload goes in as the line `session::run` expects, which is the object itself
        // rather than a string containing one: this transport is JSON all the way down, so
        // encoding a JSON-RPC object *inside* a JSON string would be an escaping bug waiting for
        // its first apostrophe.
        let line = envelope["rpc"].to_string();
        let lane = control.as_ref().expect("just built");
        if lane.to_session.send(line).await.is_err() {
            tracing::debug!("the control lane went away before its call arrived");
            *control = None;
        }
        None
    }

    /// Build a lane: a session, a pool of upstream sockets, and the task that posts its answers.
    fn open_control(&self, token: &str, session_id: &str) -> Control {
        let (to_session, from_consumer) = tokio::sync::mpsc::channel::<String>(64);
        // Deeper than the inbound half on purpose: one call can answer with a stream, and the
        // budget below would rather drop a notification than have a service block writing it.
        let (to_consumer, mut from_session) = tokio::sync::mpsc::channel::<String>(256);

        let hub = Hub {
            client: self.client.clone(),
            base: self.base.clone(),
            token: token.to_owned(),
        };
        let pool = crate::upstream::Pool::new(self.sockets.clone(), to_consumer.clone());
        // Read now rather than held as a receiver: a lane's answer to `media.video` should be
        // whatever was true when the consumer connected, and a picture that changed shape
        // mid-session is a `media.video` notification's job on the media path.
        let video = self.video.as_ref().and_then(|watch| watch.borrow().clone());
        let remote_id = session_id.to_owned();
        let session_id = session_id.to_owned();

        let task = tokio::spawn(async move {
            let session = tokio::spawn(crate::session::run(
                from_consumer,
                to_consumer,
                pool,
                // `None` on a board with no camera, and on a lane opened before the pipeline has
                // said what the video is — which is possible because the relay starts first, on
                // purpose. `session::run` refuses `media.video` rather than answering with zeros.
                video,
            ));

            let mut budget = Budget::new(NOTIFICATIONS_PER_WINDOW, NOTIFICATION_WINDOW);
            while let Some(line) = from_session.recv().await {
                let Ok(payload) = serde_json::from_str::<serde_json::Value>(&line) else {
                    // `session::run` builds every line through `duck-ipc-proto`, and a service's
                    // own output is forwarded verbatim — so this is a daemon emitting something
                    // that is not JSON, which is worth a line rather than a silent drop.
                    tracing::warn!(line = %line.chars().take(120).collect::<String>(),
                        "a control lane answer that is not JSON; dropping it");
                    continue;
                };
                // An answer has an id because the call it answers had one. Everything else is a
                // notification, and only notifications are rationed.
                let answers = !payload["id"].is_null();
                if !answers && !budget.take() {
                    continue;
                }
                let envelope = serde_json::json!({
                    "type": "peer",
                    "sessionId": session_id,
                    "rpc": payload,
                });
                if let Err(why) = hub.send(&envelope).await {
                    // The stream this token is bound to has gone, or the service refused. Either
                    // way the connection loop is about to find out on its own; this lane just
                    // stops.
                    tracing::info!(%why, "a control lane could not reach the service");
                    break;
                }
            }
            session.abort();
        });

        tracing::info!(%remote_id, "a control lane is open: JSON-RPC without a candidate pair");
        Control {
            remote_id,
            to_session,
            task,
        }
    }
}

// ── the local half: one bridged session ──────────────────────────────────────
//
// A remote consumer wants a session with this robot. On the LAN that consumer would open a
// WebSocket to `webrtcsink`'s own signalling server and ask it for one; from off the LAN it cannot
// reach that socket at all, so **this task plays the consumer on its behalf**: it opens the local
// WebSocket, asks for a session with the local producer, and then carries envelopes between the
// two sides, rewriting the one field whose value differs per hop.
//
// The roles are inverted on the two sides, and that is the whole shape of it (§3.1): to the
// rendezvous this process *is* the robot, and to `webrtcsink` it is a peer asking for a session.
//
// **One local connection per bridged session**, opened when the session starts and dropped when it
// ends. A long-lived local socket multiplexing several sessions would need the session table §3.2
// describes; one connection per session makes that table a single pair of ids, and the concurrent
// session the table would have existed for is refused anyway — the service gates it, and §3.4 says
// to keep gating it here too.

/// What the bridge needs to talk to the rendezvous while a session is in flight.
#[derive(Clone)]
struct Hub {
    client: reqwest::Client,
    base: String,
    token: String,
}

impl Hub {
    /// `POST /send`, for a task that has no `Relay` to hand.
    async fn send(&self, message: &serde_json::Value) -> Result<(), String> {
        let url = format!("{}/send", self.base);
        let response = self
            .client
            .post(&url)
            .bearer_auth(&self.token)
            .timeout(REQUEST_TIMEOUT)
            .json(message)
            .send()
            .await
            .map_err(|e| format!("POST {url}: {e}"))?;
        if !response.status().is_success() {
            return Err(format!("POST {url}: HTTP {}", response.status()));
        }
        Ok(())
    }

    /// Tell the service a session is over. Best effort: a failure here is a session that is
    /// already gone, and the peer finds out when the media stops either way.
    async fn end(&self, session_id: &str, reason: &str) {
        let message = serde_json::json!({
            "type": "endSession", "sessionId": session_id, "reason": reason,
        });
        if let Err(why) = self.send(&message).await {
            tracing::debug!(%why, "could not tell the service the session ended");
        }
    }
}

/// A bridged session, from the rendezvous side's point of view.
struct Bridged {
    /// The id the *service* knows this session by.
    remote_id: String,
    /// Envelopes from the service, on their way to the local signalling server.
    to_local: tokio::sync::mpsc::Sender<serde_json::Value>,
    task: tokio::task::JoinHandle<()>,
}

impl Bridged {
    /// Whether the task carrying this session is still running.
    fn live(&self) -> bool {
        !self.task.is_finished()
    }
}

impl Drop for Bridged {
    fn drop(&mut self) {
        // The task owns a WebSocket and a channel; aborting it closes both, which is what tells
        // `webrtcsink` the consumer went away.
        self.task.abort();
    }
}

/// Carry one session, and **tell the service when it is over however that happens**.
///
/// The wrapper exists for that second half. A consumer whose session ends without being told sits
/// looking at a robot it believes is connecting, forever — and the failure that produces it is
/// never the ordinary path, it is an early return from somewhere in the middle. So there is one
/// place that posts `endSession` and every exit goes through it.
async fn bridge(
    hub: Hub,
    local_url: String,
    remote_id: String,
    from_remote: tokio::sync::mpsc::Receiver<serde_json::Value>,
) -> Result<(), String> {
    let outcome = carry(&hub, &local_url, &remote_id, from_remote).await;
    match &outcome {
        Ok(why) => {
            hub.end(&remote_id, why).await;
            tracing::info!(remote = %remote_id, %why, "the bridged session ended");
        }
        Err(why) => {
            tracing::warn!(remote = %remote_id, %why, "the bridged session failed");
            hub.end(&remote_id, why).await;
        }
    }
    outcome.map(|_| ())
}

/// One session, from the local handshake to whichever side stops first.
///
/// `Ok` carries why it ended, which is what the consumer is told.
async fn carry(
    hub: &Hub,
    local_url: &str,
    remote_id: &str,
    mut from_remote: tokio::sync::mpsc::Receiver<serde_json::Value>,
) -> Result<String, String> {
    use futures_util::SinkExt;
    use tokio_tungstenite::tungstenite::Message;

    let (mut socket, _) = tokio_tungstenite::connect_async(local_url)
        .await
        .map_err(|e| format!("{local_url}: {e}"))?;

    // The consumer's own handshake against `webrtcsink`'s server: a welcome, then ask what is
    // producing, then ask that producer for a session. Exactly what the console page does over a
    // LAN, which is why the page was the reference for this rather than the protocol document.
    let mut producer = None;
    let mut local_id = None;
    let deadline = tokio::time::Instant::now() + LOCAL_HANDSHAKE_TIMEOUT;
    while local_id.is_none() {
        let message = tokio::time::timeout_at(deadline, socket.next())
            .await
            .map_err(|_| {
                format!("{local_url} did not start a session within {LOCAL_HANDSHAKE_TIMEOUT:?}")
            })?
            .ok_or_else(|| format!("{local_url} closed during the handshake"))?
            .map_err(|e| format!("{local_url}: {e}"))?;
        let Message::Text(text) = message else {
            continue;
        };

        // Read as the envelope it is rather than through `classify`: two of these three types
        // exist only on this side of the bridge, and giving them variants in the rendezvous's
        // vocabulary would put local-only messages in a remote-only enum.
        let value: serde_json::Value =
            serde_json::from_str(&text).map_err(|e| format!("{local_url}: {e}"))?;
        match value["type"].as_str().unwrap_or_default() {
            "welcome" => {
                socket
                    .send(Message::text(r#"{"type":"list"}"#))
                    .await
                    .map_err(|e| format!("{local_url}: {e}"))?;
            }
            "list" => {
                let first = value["producers"].get(0).and_then(|p| p["id"].as_str());
                let Some(id) = first else {
                    return Err(
                        "the robot's own signalling server lists no producer, so there is \
                         nothing to bridge — its pipeline has not reached PLAYING"
                            .to_owned(),
                    );
                };
                producer = Some(id.to_owned());
                let request = serde_json::json!({ "type": "startSession", "peerId": id });
                socket
                    .send(Message::text(request.to_string()))
                    .await
                    .map_err(|e| format!("{local_url}: {e}"))?;
            }
            "sessionStarted" => {
                local_id = value["sessionId"].as_str().map(str::to_owned);
            }
            _ => {}
        }
    }

    let local_id = local_id.expect("the loop above does not end until it is set");
    tracing::info!(
        remote = %remote_id,
        local = %local_id,
        producer = producer.as_deref().unwrap_or("unknown"),
        "a remote session is bridged to this robot's own signalling server"
    );

    // From here it is two directions and one rewritten field.
    let outcome = loop {
        tokio::select! {
            // The local producer: SDP, ICE, or the end of the session.
            message = socket.next() => {
                let Some(message) = message else {
                    break Err("the robot's signalling server closed the session".to_owned());
                };
                let message = message.map_err(|e| format!("{local_url}: {e}"))?;
                let Message::Text(text) = message else { continue };
                match classify(&text)? {
                    Inbound::Peer(mut envelope) => {
                        envelope["sessionId"] = serde_json::Value::String(remote_id.to_owned());
                        hub.send(&envelope).await?;
                    }
                    Inbound::EndSession { .. } => {
                        break Ok("the robot ended the session".to_owned());
                    }
                    _ => {}
                }
            }
            // The remote consumer, by way of the event stream this session arrived on.
            envelope = from_remote.recv() => {
                let Some(mut envelope) = envelope else {
                    break Ok("the rendezvous connection went away".to_owned());
                };
                envelope["sessionId"] = serde_json::Value::String(local_id.clone());
                socket
                    .send(Message::text(envelope.to_string()))
                    .await
                    .map_err(|e| format!("{local_url}: {e}"))?;
            }
        }
    };

    // The local side is told with a message rather than a dropped socket, so `webrtcsink` frees
    // its consumer immediately instead of on a timeout. The remote side is told by the caller,
    // which is the only place that does it.
    let farewell = serde_json::json!({ "type": "endSession", "sessionId": local_id });
    let _ = socket.send(Message::text(farewell.to_string())).await;
    let _ = socket.close(None).await;
    outcome
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
        let welcome = classify(
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
            classify(r#"{"type":"startSession","peerId":"p-2","sessionId":"s-1"}"#).unwrap(),
            Inbound::StartSession {
                session_id: "s-1".to_owned()
            },
        );
        // A `peer` keeps its whole envelope, payload included — that is what makes this a
        // translator rather than a parser.
        let peer =
            classify(r#"{"type":"peer","sessionId":"s-1","sdp":{"type":"offer","sdp":"v=0\r\n"}}"#)
                .unwrap();
        let Inbound::Peer(envelope) = &peer else {
            panic!("{peer:?}")
        };
        assert_eq!(envelope["sdp"]["sdp"], "v=0\r\n");

        // Messages this slice does not act on must not be errors: it is somebody else's service
        // and it is allowed to grow.
        for other in [
            r#"{"type":"list","producers":[]}"#,
            r#"{"type":"peerStatusChanged","peerId":"p-1","roles":["producer"],"meta":{}}"#,
            r#"{"type":"sessionRejected","reason":"robot_busy","activeApp":"whatever"}"#,
            r#"{"type":"somethingAddedNextYear"}"#,
        ] {
            assert_eq!(classify(other).unwrap(), Inbound::Other, "{other}");
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
        /// How many times an event stream has been dropped by the client, which is what a clean
        /// disconnect looks like from here — and what evicts a producer on the real service.
        closed: std::sync::atomic::AtomicUsize,
        /// The bearer token each stream was opened with, in order.
        bearers: std::sync::Mutex<Vec<String>>,
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

        fn bearers(&self) -> Vec<String> {
            self.bearers.lock().unwrap().clone()
        }

        fn closed(&self) -> usize {
            self.closed.load(std::sync::atomic::Ordering::SeqCst)
        }

        fn streams(&self) -> usize {
            self.streams.load(std::sync::atomic::Ordering::SeqCst)
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

    /// Increments the fake's `closed` count when the stream it lives in is dropped.
    struct Closing(std::sync::Arc<Fake>);

    impl Drop for Closing {
        fn drop(&mut self) {
            self.0
                .closed
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
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
                get(|State(fake): State<std::sync::Arc<Fake>>,
                     headers: axum::http::HeaderMap| async move {
                    fake.streams
                        .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    fake.bearers.lock().unwrap().push(
                        headers
                            .get("authorization")
                            .and_then(|value| value.to_str().ok())
                            .unwrap_or("")
                            .to_owned(),
                    );
                    let closing = Closing(std::sync::Arc::clone(&fake));
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
                        // Moved in, so it is dropped when the client hangs up — which is how the
                        // real service learns to evict a producer.
                        let _closing = closing;
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

    /// Signing the robot out takes it out of the service's listing, promptly.
    ///
    /// The token is read once per connection, so nothing about a `logout` reaches this task on its
    /// own: without the check on the heartbeat tick the relay would go on refreshing the lease
    /// with a credential its owner had deleted — a robot signed out of an account and still
    /// listed under it. §2.6 claims `account.logout` is one of the three things that make a LAN
    /// peer's login acceptable, so it has to be true here and not only in `updaterd`.
    ///
    /// Dropping the stream *is* the deregistration: a clean disconnect evicts the peer at once on
    /// the real service, and the 30 s sweep exists only for sockets that never report closing.
    #[tokio::test]
    async fn signing_the_robot_out_drops_the_connection() {
        let dir = tempfile::tempdir().unwrap();
        let path = signed_in(&dir);
        let service = fake_service().await;
        *service.state.heartbeat_seconds.lock().unwrap() = Some(0.05);

        let relay = Relay::new(&service.base, &path, meta())
            .unwrap()
            .with_timings(brisk());
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        // `account logout` on the robot is exactly this.
        std::fs::remove_file(&path).unwrap();

        until("the stream to be dropped", || service.state.closed() >= 1).await;
        let posts_when_signed_out = service.state.of_type("setPeerStatus").len();
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert_eq!(
            service.state.of_type("setPeerStatus").len(),
            posts_when_signed_out,
            "a robot signed out must stop refreshing its lease"
        );
        assert_eq!(
            service.state.streams(),
            1,
            "and must not reconnect: there is nothing to connect with"
        );
        task.abort();
    }

    /// Signing in to a *different* account moves the robot, rather than leaving it where it was.
    ///
    /// `login --force` is how a robot changes hands, and the connection it changes hands over is
    /// authenticated by the old owner's token. So the same check that notices a logout has to
    /// notice a replacement, and reconnect with the new credential.
    #[tokio::test]
    async fn a_relogin_reconnects_with_the_new_token() {
        let dir = tempfile::tempdir().unwrap();
        let path = signed_in(&dir);
        let service = fake_service().await;
        *service.state.heartbeat_seconds.lock().unwrap() = Some(0.05);

        let relay = Relay::new(&service.base, &path, meta())
            .unwrap()
            .with_timings(brisk());
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;
        assert_eq!(service.state.bearers()[0], "Bearer hf_abc");

        std::fs::write(&path, r#"{"access_token":"hf_somebody_else"}"#).unwrap();

        until("a reconnect on the new token", || {
            service.state.bearers().len() >= 2
        })
        .await;
        assert_eq!(
            service.state.bearers()[1],
            "Bearer hf_somebody_else",
            "the second connection belongs to whoever the robot belongs to now"
        );
        assert!(
            service.state.closed() >= 1,
            "and the first one was closed, which is what un-lists the robot for the old account"
        );
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

    // ── the local half ──────────────────────────────────────────────────────
    //
    // A stand-in for `webrtcsink`'s own signalling server, which is the one thing about this
    // module that cannot be a channel: the bridge is a WebSocket client, and the framing is part
    // of what it gets wrong if it gets anything wrong.

    // ── the control lane ────────────────────────────────────────────────────
    //
    // The property under test is the one the lane exists for: **a call crosses with no candidate
    // pair, no offer and no answer.** Every test here pushes an `rpc` envelope at a relay whose
    // media path was never negotiated, which is exactly the state a consumer behind a NAT is left
    // in while §6's TURN endpoint has no DNS.

    /// A daemon that reads one line and replies with the next canned one, remembering what it saw.
    ///
    /// Lifted from `session.rs`'s tests rather than shared, for now: two fakes of five lines are
    /// cheaper than a test-support surface that both have to agree on.
    fn fake_daemon(
        path: &Path,
        replies: Vec<String>,
    ) -> tokio::sync::mpsc::UnboundedReceiver<String> {
        use tokio::io::{AsyncBufReadExt as _, AsyncWriteExt as _, BufReader};

        let listener = std::os::unix::net::UnixListener::bind(path).unwrap();
        listener.set_nonblocking(true).unwrap();
        let listener = tokio::net::UnixListener::from_std(listener).unwrap();
        let (seen_tx, seen_rx) = tokio::sync::mpsc::unbounded_channel();

        tokio::spawn(async move {
            while let Ok((stream, _)) = listener.accept().await {
                let seen = seen_tx.clone();
                let mut replies = replies.clone().into_iter();
                tokio::spawn(async move {
                    let (read, mut write) = stream.into_split();
                    let mut lines = BufReader::new(read).lines();
                    while let Ok(Some(line)) = lines.next_line().await {
                        let _ = seen.send(line);
                        if let Some(reply) = replies.next() {
                            let _ = write.write_all(format!("{reply}\n").as_bytes()).await;
                            let _ = write.flush().await;
                        }
                    }
                });
            }
        });
        seen_rx
    }

    /// Sockets pointing at a directory, so a lane can be driven on a laptop.
    fn sockets_in(dir: &Path) -> crate::upstream::Sockets {
        crate::upstream::Sockets {
            updater: dir.join("updater.sock"),
            robot: dir.join("robot.sock"),
            config: dir.join("config.sock"),
            pad: dir.join("pad.sock"),
            tof: dir.join("tof.sock"),
        }
    }

    /// **A call crosses the rendezvous and its answer comes back, with no WebRTC anywhere.**
    ///
    /// This is the whole of the lane: `POST /send {type:peer, rpc}` in, a JSON-RPC line out to
    /// the service that owns the answer, and the answer back out as another `peer` envelope. No
    /// `startSession` is bridged, no offer is exchanged, and nothing in the path can be defeated
    /// by a NAT — which is the point, because the media path currently can be.
    #[tokio::test]
    async fn a_call_crosses_the_rendezvous_with_no_candidate_pair() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        let sockets = sockets_in(dir.path());

        // `robot.policies` is routed to `robotd`, and this is what a duck answers with.
        let answer = serde_json::json!({
            "jsonrpc": "2.0", "id": 1,
            "result": { "mode": "walk", "enabled": true, "slots": [] },
        })
        .to_string();
        let mut robotd = fake_daemon(&sockets.robot, vec![answer.clone()]);

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk())
            .with_sockets(sockets);
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "peer",
            "sessionId": "remote-session-1",
            "rpc": { "jsonrpc": "2.0", "id": 1, "method": "robot.policies", "params": {} },
        }));

        // The daemon sees the line the consumer wrote, verbatim: this transport routes and does
        // not rewrite.
        let asked = tokio::time::timeout(Duration::from_secs(5), robotd.recv())
            .await
            .expect("robotd was asked within five seconds")
            .expect("a line");
        let asked: serde_json::Value = serde_json::from_str(&asked).unwrap();
        assert_eq!(asked["method"], "robot.policies");
        assert_eq!(asked["id"], 1);

        // And the answer comes back as a `peer` envelope naming the same session.
        until("the answer to reach the service", || {
            service
                .state
                .of_type("peer")
                .iter()
                .any(|posted| !posted["rpc"].is_null())
        })
        .await;
        let posted = service
            .state
            .of_type("peer")
            .into_iter()
            .find(|posted| !posted["rpc"].is_null())
            .unwrap();
        assert_eq!(
            posted["sessionId"], "remote-session-1",
            "the answer names the session the service routes on"
        );
        assert_eq!(
            posted["rpc"],
            serde_json::from_str::<serde_json::Value>(&answer).unwrap(),
            "and the payload is the daemon's own line, unparsed and unwrapped"
        );
        task.abort();
    }

    /// A method this transport refuses is refused *here*, without reaching a daemon.
    ///
    /// `route::permits` is what says so, and it is the same table the datachannel uses — so this
    /// asserts the lane consults it rather than that the answer is what it is. A lane that
    /// forwarded everything would make the rendezvous a way around a per-transport rule, and the
    /// pairing PIN is the one that would matter: a peer that can rewrite it locks a phone out of
    /// the recovery path.
    #[tokio::test]
    async fn the_lane_refuses_what_the_route_table_refuses() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        let sockets = sockets_in(dir.path());
        // Deliberately no daemon at all: a refusal must not depend on one being there.

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk())
            .with_sockets(sockets);
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "peer",
            "sessionId": "remote-session-1",
            "rpc": { "jsonrpc": "2.0", "id": 7, "method": "system.pairingPin", "params": {} },
        }));

        until("the refusal", || {
            service
                .state
                .of_type("peer")
                .iter()
                .any(|posted| !posted["rpc"]["error"].is_null())
        })
        .await;
        let posted = service
            .state
            .of_type("peer")
            .into_iter()
            .find(|posted| !posted["rpc"]["error"].is_null())
            .unwrap();
        assert_eq!(
            posted["rpc"]["id"], 7,
            "a refusal answers the call that earned it"
        );
        task.abort();
    }

    /// `media.video` is refused rather than answered with zeros when there is no picture.
    ///
    /// The lane can open before the pipeline has said what the video is — the relay starts first,
    /// on purpose — and it carries no media of its own in any case. A consumer told the frame is
    /// 0×0 and upright has been handed a wrong number that looks like a right one; told there is
    /// no video, it can act.
    #[tokio::test]
    async fn a_lane_with_no_picture_says_so() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk())
            .with_sockets(sockets_in(dir.path()));
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "peer",
            "sessionId": "remote-session-1",
            "rpc": { "jsonrpc": "2.0", "id": 3, "method": "media.video", "params": {} },
        }));

        until("the answer", || {
            service
                .state
                .of_type("peer")
                .iter()
                .any(|posted| !posted["rpc"].is_null())
        })
        .await;
        let posted = service
            .state
            .of_type("peer")
            .into_iter()
            .find(|posted| !posted["rpc"].is_null())
            .unwrap();
        assert!(
            posted["rpc"]["error"]["message"]
                .as_str()
                .unwrap_or_default()
                .contains("not publishing video"),
            "said {:?}",
            posted["rpc"]
        );
        task.abort();
    }

    /// What the fake local server saw and said.
    #[derive(Default)]
    struct Local {
        /// Everything the bridge sent it, in order.
        seen: std::sync::Mutex<Vec<serde_json::Value>>,
        /// A sender for pushing messages *to* the bridge, once it has connected.
        push: std::sync::Mutex<Option<tokio::sync::mpsc::UnboundedSender<String>>>,
    }

    impl Local {
        fn seen(&self) -> Vec<serde_json::Value> {
            self.seen.lock().unwrap().clone()
        }

        fn of_type(&self, kind: &str) -> Vec<serde_json::Value> {
            self.seen()
                .into_iter()
                .filter(|m| m["type"] == kind)
                .collect()
        }

        fn push(&self, message: serde_json::Value) {
            let sender = self
                .push
                .lock()
                .unwrap()
                .clone()
                .expect("the bridge has connected");
            sender
                .send(message.to_string())
                .expect("the bridge is listening");
        }
    }

    const LOCAL_PRODUCER: &str = "the-robots-own-producer";
    const LOCAL_SESSION: &str = "local-session-1";

    /// A signalling server that answers the consumer handshake and then relays.
    ///
    /// `producers` controls the one interesting failure: a server with none, which is what a
    /// pipeline that never reached PLAYING looks like from here.
    async fn fake_signalling(producers: bool) -> (String, std::sync::Arc<Local>) {
        use axum::extract::State;
        use axum::extract::ws::{Message, WebSocketUpgrade};
        use axum::routing::any;

        let local = std::sync::Arc::new(Local::default());

        let app = axum::Router::new()
            .route(
                "/",
                any(move |upgrade: WebSocketUpgrade,
                     State(local): State<std::sync::Arc<Local>>| async move {
                    upgrade.on_upgrade(move |mut socket| async move {
                        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<String>();
                        *local.push.lock().unwrap() = Some(tx);

                        // The welcome, unprompted, as the real server sends it.
                        let welcome = serde_json::json!({ "type": "welcome", "peerId": "the-bridge" });
                        if socket.send(Message::text(welcome.to_string())).await.is_err() {
                            return;
                        }

                        loop {
                            tokio::select! {
                                incoming = socket.recv() => {
                                    let Some(Ok(Message::Text(text))) = incoming else { return };
                                    let message: serde_json::Value =
                                        serde_json::from_str(&text).expect("the bridge sends JSON");
                                    let kind = message["type"].as_str().unwrap_or("").to_owned();
                                    local.seen.lock().unwrap().push(message);

                                    let answer = match kind.as_str() {
                                        "list" => Some(serde_json::json!({
                                            "type": "list",
                                            "producers": if producers {
                                                serde_json::json!([{ "id": LOCAL_PRODUCER, "meta": {} }])
                                            } else {
                                                serde_json::json!([])
                                            },
                                        })),
                                        "startSession" => Some(serde_json::json!({
                                            "type": "sessionStarted",
                                            "peerId": LOCAL_PRODUCER,
                                            "sessionId": LOCAL_SESSION,
                                        })),
                                        _ => None,
                                    };
                                    if let Some(answer) = answer
                                        && socket.send(Message::text(answer.to_string())).await.is_err()
                                    {
                                        return;
                                    }
                                }
                                pushed = rx.recv() => {
                                    let Some(pushed) = pushed else { return };
                                    if socket.send(Message::text(pushed)).await.is_err() {
                                        return;
                                    }
                                }
                            }
                        }
                    })
                }),
            )
            .with_state(std::sync::Arc::clone(&local));

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let url = format!("ws://{}/", listener.local_addr().unwrap());
        tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });
        (url, local)
    }

    /// **The whole of slice 4: a session's envelopes cross, and their ids are rewritten.**
    ///
    /// The two assertions that matter are the two rewrites. An offer from the robot's own
    /// producer must reach the service carrying the *remote* session id, because that is the id
    /// the service routes on — and the consumer's answer must reach the local server carrying the
    /// *local* one, for the same reason on the other side. Getting either backwards produces a
    /// session where signalling is exchanged and no media ever flows, with nothing in any log to
    /// say why, which is why this is pinned rather than tried by hand.
    #[tokio::test]
    async fn a_remote_session_is_bridged_to_the_local_signalling_server() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        let (local_url, local) = fake_signalling(true).await;

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk())
            .with_local_signalling(&local_url);
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        // A consumer asks the service for a session with this robot.
        service.state.push(serde_json::json!({
            "type": "startSession", "peerId": "a-consumer", "sessionId": "remote-session-1",
        }));

        // The bridge plays the consumer on the local side: welcome, list, startSession.
        until("the local handshake", || {
            !local.of_type("startSession").is_empty()
        })
        .await;
        assert_eq!(
            local.of_type("startSession")[0]["peerId"],
            LOCAL_PRODUCER,
            "the session is asked of the robot's own producer"
        );

        // The producer offers. `webrtcsink` knows what it is sending, so the offer comes from it.
        local.push(serde_json::json!({
            "type": "peer",
            "sessionId": LOCAL_SESSION,
            "sdp": { "type": "offer", "sdp": "v=0\r\no=- 1 2 IN IP4 127.0.0.1\r\n" },
        }));

        until("the offer to reach the service", || {
            !service.state.of_type("peer").is_empty()
        })
        .await;
        let forwarded = service.state.of_type("peer")[0].clone();
        assert_eq!(
            forwarded["sessionId"], "remote-session-1",
            "outbound envelopes carry the id the *service* routes on"
        );
        assert_eq!(
            forwarded["sdp"]["sdp"], "v=0\r\no=- 1 2 IN IP4 127.0.0.1\r\n",
            "and the payload is untouched — this process never parses SDP"
        );

        // The consumer answers, by way of the service.
        service.state.push(serde_json::json!({
            "type": "peer",
            "sessionId": "remote-session-1",
            "sdp": { "type": "answer", "sdp": "v=0\r\nanswer\r\n" },
        }));

        until("the answer to reach the robot", || {
            !local.of_type("peer").is_empty()
        })
        .await;
        let delivered = local.of_type("peer")[0].clone();
        assert_eq!(
            delivered["sessionId"], LOCAL_SESSION,
            "inbound envelopes carry the id the robot's own server routes on"
        );
        assert_eq!(delivered["sdp"]["sdp"], "v=0\r\nanswer\r\n");

        task.abort();
    }

    /// A second consumer is refused by name while one is being carried.
    ///
    /// The service gates this itself, so this is the belt-and-braces §3.4 asks for — and the
    /// reason it stays is that two remote peers writing into one intent slot is the interleaving
    /// bug `remote-webrtc.md` §9 defers, with a second continent instead of a gamepad.
    #[tokio::test]
    async fn a_second_remote_session_is_refused_while_one_is_live() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        let (local_url, local) = fake_signalling(true).await;

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk())
            .with_local_signalling(&local_url);
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "startSession", "peerId": "first", "sessionId": "remote-1",
        }));
        until("the first session", || {
            !local.of_type("startSession").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "startSession", "peerId": "second", "sessionId": "remote-2",
        }));
        until("a refusal", || {
            !service.state.of_type("endSession").is_empty()
        })
        .await;

        let refusal = service.state.of_type("endSession")[0].clone();
        assert_eq!(
            refusal["sessionId"], "remote-2",
            "the second one is what is refused"
        );
        assert!(
            refusal["reason"].as_str().unwrap().contains("already"),
            "and the reason says the robot is busy rather than broken: {refusal}"
        );
        assert_eq!(
            local.of_type("startSession").len(),
            1,
            "the robot's own server was asked for one session, not two"
        );
        task.abort();
    }

    /// A robot whose pipeline never started has no producer to bridge to, and says so.
    ///
    /// The peer is told the session is over rather than left waiting on media that is never
    /// coming — the failure this shape has to avoid is a client that looks connected forever.
    #[tokio::test]
    async fn a_robot_with_no_producer_ends_the_session_rather_than_hanging() {
        let dir = tempfile::tempdir().unwrap();
        let service = fake_service().await;
        let (local_url, _local) = fake_signalling(false).await;

        let relay = Relay::new(&service.base, signed_in(&dir), meta())
            .unwrap()
            .with_timings(brisk())
            .with_local_signalling(&local_url);
        let task = tokio::spawn(relay.run());
        until("registration", || {
            !service.state.of_type("setPeerStatus").is_empty()
        })
        .await;

        service.state.push(serde_json::json!({
            "type": "startSession", "peerId": "a-consumer", "sessionId": "remote-session-1",
        }));

        until("the session to be ended", || {
            !service.state.of_type("endSession").is_empty()
        })
        .await;
        assert_eq!(
            service.state.of_type("endSession")[0]["sessionId"],
            "remote-session-1"
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
