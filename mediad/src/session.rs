//! One peer's control channel, as a pipe to the services that own the answers.
//!
//! Transport-agnostic on purpose: this takes lines in and gives lines out, and knows nothing about
//! datachannels. That is what makes it testable without a WebRTC peer — the tests below drive it
//! over channels against fake daemons on real unix sockets — and it is also what lets the
//! rendezvous control lane (`remote-access-design.md` §3.8) reuse it unchanged.
//!
//! ## What it does not do
//!
//! **It never parses a reply.** Requests are read far enough to route them and no further;
//! everything a service emits is forwarded verbatim. Two things follow, and both matter:
//!
//! - A subscription works with no special case. It is a stream of notifications on an open
//!   connection, and every one has to reach the peer — which correlating replies to requests would
//!   break, keeping the first and dropping the rest.
//! - Adding a method to the API costs nothing here. `duck-ipc-proto` stays the only place a method
//!   is defined, and this file does not grow a case for it.
//!
//! **It does not authenticate.** `remote-webrtc.md` §4: there is no gate on the robot. A LAN peer
//! may drive it, and a bridged peer authenticated to the rendezvous service — on both sides —
//! before arriving. `route::permits` refuses `system.authenticate` by name rather than answering
//! it, so a client that asks gets a clear no instead of a lie.

use duck_ipc_proto as proto;
use tokio::sync::mpsc;

use crate::route::{self, Route};
use crate::upstream::Pool;

/// Drive one peer's control channel until its inbound stream ends.
///
/// `inbound` carries one JSON-RPC object per item, as the peer sent it. `outbound` is where replies
/// and notifications go, merged from every service — the peer sorts them out by `id`, which is its
/// business rather than ours.
/// What the video is: the frame the encoder sends, and how far the camera is mounted from upright.
///
/// Held by the session because a peer has to be able to *ask*. Pushing it when the channel appears
/// races the browser: `mediad` writes the line the moment `webrtcsink` hands over the channel, and
/// if the peer's datachannel is not open yet the line is dropped — which is exactly what happened,
/// and the console showed a sideways picture with nothing in the log to say why. The push is kept
/// as a courtesy for a client that only listens; `media.video` as a *call* is what the console uses,
/// because a question it asks when it is ready cannot arrive too early.
#[derive(Debug, Clone, PartialEq)]
pub struct Video {
    pub width: u32,
    pub height: u32,
    /// Degrees clockwise the camera is mounted from upright.
    pub rotate: u32,
    /// The camera's geometry, for a consumer that has to turn pixels into directions — SLAM,
    /// visual odometry, "how far is that". `None` when there is no camera or its mode is not one
    /// `crate::camera` knows the field of view of, which is a robot that should not be believed
    /// rather than one to guess about.
    pub intrinsics: Option<crate::camera::Intrinsics>,
}

/// The methods a peer asks with. Answered here rather than routed: no service owns them.
const VIDEO_METHOD: &str = proto::method::MEDIA_VIDEO;
const SET_VIDEO_METHOD: &str = proto::method::MEDIA_SET_VIDEO;
const STREAM_METHOD: &str = "media.stream";

/// What this daemon can answer about its own media, when it has a pipeline to answer from.
///
/// One handle rather than two Options, because the two arrive together: both are known only once
/// the pipeline is up — `pinned_sensor()` is not truthful before then, and there are no frames to
/// encode either — and both are absent for the same reasons, a board with no camera or a lane
/// opened before the pipeline started. A peer asking either gets a refusal that says which.
#[derive(Clone)]
pub struct Media {
    pub video: Video,
    pub streamer: std::sync::Arc<crate::stream::Streamer>,
    /// The video track's valve, which `media.setVideo` votes on. `None` for a pipeline built
    /// without one, which answers every vote `accepted: false`.
    pub gate: Option<std::sync::Arc<Gate>>,
}

/// Whether the WebRTC video track is encoded and sent, decided by the sessions watching it.
///
/// **Video flows while any session wants it**, and a session wants it from the moment it opens
/// until it says otherwise — so a console that knows nothing of this method never finds the
/// picture paused by somebody else's bridge, and a bridge that pauses it gets it paused only for as
/// long as it is alone. A session's vote ends with the session.
///
/// **With no session at all the valve is open**, which is how the pipeline was built and what
/// `webrtcsink` needs: it learns the video's caps from the first buffer it is handed and offers
/// nothing to a consumer until it has. Nobody is encoding then anyway — `webrtcsink` builds an
/// encoder per consumer — so open costs nothing that closed would save.
///
/// The decision is here, portable and tested; what it switches is `pipeline::VideoValve`'s, handed
/// in as `apply` and called only on a change, under the lock, so two sessions voting at once
/// cannot apply their answers out of order. `remote-webrtc.md` §2.2.
pub struct Gate {
    votes: std::sync::Mutex<Votes>,
    apply: Box<dyn Fn(bool) + Send + Sync>,
}

#[derive(Debug)]
struct Votes {
    sessions: usize,
    off: usize,
    /// What `apply` was last told, which starts as the pipeline was built: open.
    flowing: bool,
}

impl Gate {
    pub fn new(apply: impl Fn(bool) + Send + Sync + 'static) -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self {
            votes: std::sync::Mutex::new(Votes {
                sessions: 0,
                off: 0,
                flowing: true,
            }),
            apply: Box::new(apply),
        })
    }

    /// Whether the video is being sent now.
    pub fn flowing(&self) -> bool {
        self.lock().flowing
    }

    // A panic while holding this leaves two counters, which are as good as they were.
    fn lock(&self) -> std::sync::MutexGuard<'_, Votes> {
        self.votes
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Change the counts, and tell the valve if the answer moved.
    fn tally(&self, change: impl FnOnce(&mut Votes)) -> bool {
        let mut votes = self.lock();
        change(&mut votes);
        let flowing = votes.sessions == 0 || votes.off < votes.sessions;
        if flowing != votes.flowing {
            votes.flowing = flowing;
            tracing::info!(
                sessions = votes.sessions,
                "{}",
                if flowing {
                    "video resumed: a session wants the picture"
                } else {
                    "video paused: every session asked for it off"
                }
            );
            (self.apply)(flowing);
        }
        votes.flowing
    }
}

/// One session's say in [`Gate`]: wanting the video when cast, and gone when dropped.
pub struct Vote {
    gate: std::sync::Arc<Gate>,
    on: bool,
}

impl Vote {
    pub fn cast(gate: std::sync::Arc<Gate>) -> Self {
        gate.tally(|votes| votes.sessions += 1);
        Self { gate, on: true }
    }

    /// Change this session's mind, and answer whether the video is now flowing.
    pub fn set(&mut self, on: bool) -> bool {
        let was = std::mem::replace(&mut self.on, on);
        self.gate.tally(|votes| match (was, on) {
            (true, false) => votes.off += 1,
            (false, true) => votes.off -= 1,
            _ => {}
        })
    }
}

impl Drop for Vote {
    fn drop(&mut self) {
        let off = !self.on;
        self.gate.tally(|votes| {
            votes.sessions -= 1;
            if off {
                votes.off -= 1;
            }
        });
    }
}

/// Where this session sits among the WebRTC peers `mediad` is serving, for the one decision that
/// depends on who else is here: whether an OS update this session asks for may drop the others.
///
/// `updaterd` refuses an update while any WebRTC peer is connected (`updater-design.md` §7.2),
/// counting from the same [`proto::read_webrtc_peers`] read here. A session asking for one is
/// itself a peer, and must not be refused for its own presence — so when nobody *else* is here,
/// [`handle`] answers the check for it by setting `interrupt_sessions`. With company it forwards
/// the call as sent, and the check refuses on the company's behalf.
#[derive(Clone)]
pub struct Seat {
    /// Whether this session's own client is one of the peers counted. Always for a datachannel,
    /// which exists because `webrtcsink` added a consumer. For the rendezvous control lane, only
    /// while the same remote client also holds the bridged video session: the lane carries no
    /// media itself, but its client may be watching over WebRTC at the same time, and that video
    /// is the client's own presence, not company.
    counted: std::sync::Arc<dyn Fn() -> bool + Send + Sync>,
    peers: fn() -> Option<u32>,
    /// Whether this session has a say in the video track ([`Gate`]). A datachannel does: it rides
    /// the session that carries the track. A rendezvous lane does not — it has no track, and a
    /// lane that left a vote on would hold the video open for a client watching nothing.
    votes: bool,
}

impl Seat {
    /// A datachannel: one of the peers `webrtcsink` counts.
    pub fn peer() -> Seat {
        Seat {
            counted: std::sync::Arc::new(|| true),
            peers: proto::read_webrtc_peers,
            votes: true,
        }
    }

    /// The rendezvous control lane, counted while `counted` says its client holds the video.
    pub fn lane(counted: impl Fn() -> bool + Send + Sync + 'static) -> Seat {
        Seat {
            counted: std::sync::Arc::new(counted),
            peers: proto::read_webrtc_peers,
            votes: false,
        }
    }

    /// Whether this session is the only one an update would drop. `None` from the count is
    /// `updaterd`'s "not saying", which it lets through, so it is alone here too.
    fn alone(&self) -> bool {
        let peers = (self.peers)().unwrap_or(0);
        peers.saturating_sub(u32::from((self.counted)())) == 0
    }
}

/// The line to forward for `call`: unchanged, except an OS update from a session that is
/// [alone](Seat::alone), which goes with `interrupt_sessions` set.
///
/// The rest of the request is kept as the peer sent it. Rebuilding it from `call` would drop
/// anything this release does not model, and the one thing `handle` must never do is change a
/// request it only meant to route.
fn exempt_if_alone(call: &proto::Call, line: &str, seat: &Seat) -> Option<String> {
    let asked = match call {
        proto::Call::OsApply(p) => p.interrupt_sessions,
        proto::Call::OsSwitch(p) => p.interrupt_sessions,
        _ => return None,
    };
    if asked || !seat.alone() {
        return None;
    }
    let mut request: serde_json::Value = serde_json::from_str(line).ok()?;
    let object = request.as_object_mut()?;
    // `params` is optional for both — `{"method":"os.apply"}` is the channel's newest.
    let params = object
        .entry("params")
        .or_insert_with(|| serde_json::Value::Object(Default::default()));
    if params.is_null() {
        *params = serde_json::Value::Object(Default::default());
    }
    params
        .as_object_mut()?
        .insert("interrupt_sessions".into(), true.into());
    tracing::info!(
        method = call.method(),
        "no other WebRTC session to drop; asking updaterd to go ahead"
    );
    Some(request.to_string())
}

/// What the video is, told to a peer once when its channel opens.
///
/// **The rotation is the whole point.** Nothing on the robot rotates pixels any more — a `videoflip`
/// in the pipeline cost the encoder its zero-copy path and the board 22 fps — so the stream a
/// browser receives is the picture the camera took, sideways on a robot whose camera is mounted a
/// quarter turn off. The page cannot work out by how much: a 180° mount is indistinguishable from an
/// upright one, and even a quarter turn is only a guess from the aspect ratio. So it is told.
///
/// A notification, with no id, because the page already treats an id-less line as something that
/// streams (`robot.state` is the other one) — no new mechanism at either end.
pub fn video_notification(video: &Video) -> String {
    // Built with `serde_json` rather than `format!` since it gained a nested object: one escaping
    // mistake in a hand-written line is a peer that cannot parse anything this daemon says.
    serde_json::json!({
        "jsonrpc": "2.0",
        "method": VIDEO_METHOD,
        "params": video_params(video),
    })
    .to_string()
}

/// The video's description, which the notification pushes and the call answers with.
///
/// One function for both, because a peer that asks and a peer that listens must be told the same
/// thing — and the console does both, a push when the channel opens and a call when it is ready.
fn video_params(video: &Video) -> serde_json::Value {
    let mut params = serde_json::json!({
        "width": video.width,
        "height": video.height,
        "rotate": video.rotate,
        // The two clocks at one instant: RTCP sender reports state RTP time in wall-clock
        // (`real_ns`), `robot.state`/`tof.frame` stamp with `mono_ns`'s clock. A peer that has both
        // can put the picture on the robot's axis.
        "mono_ns": proto::clock::monotonic_ns(),
        "real_ns": proto::clock::realtime_ns(),
    });
    // Absent rather than null when the geometry is unknown: a consumer reading a missing key knows
    // it must calibrate, where one reading `null` has to be told what that meant.
    if let Some(intrinsics) = &video.intrinsics {
        params["intrinsics"] = serde_json::to_value(intrinsics).expect("intrinsics serialise");
    }
    params
}

/// `media` is `None` when there is no pipeline to answer for.
///
/// A datachannel always has one — it exists because `webrtcsink` handed over a consumer — but the
/// rendezvous control lane (`relay::Relay::control`) is JSON-RPC with no media beside it, and it
/// can open before the pipeline has started. Answering `media.video` with zeros in that case would
/// be a consumer told the picture is 0×0 and upright, which is worse than being told there is no
/// picture: one is actionable, the other is a wrong number that looks like a right one.
pub async fn run(
    mut inbound: mpsc::Receiver<String>,
    outbound: mpsc::Sender<String>,
    mut pool: Pool,
    media: Option<Media>,
    seat: Seat,
) {
    // A peer that came with the video is a visit, and how long it stays is video streamed.
    let _visit = media.is_some().then(crate::vetbook::visit);
    // Wanting the video from the first line, and dropped — with whatever it last said — when the
    // channel ends.
    let mut vote = media
        .as_ref()
        .and_then(|media| media.gate.clone())
        .filter(|_| seat.votes)
        .map(Vote::cast);
    while let Some(line) = inbound.recv().await {
        let line = line.trim().to_string();
        if line.is_empty() {
            continue;
        }
        if let Some(reply) = handle(&line, &mut pool, media.as_ref(), &seat, vote.as_mut()).await {
            // A closed outbound means the peer is gone; there is nothing left to do for it.
            if outbound.send(reply).await.is_err() {
                break;
            }
        }
    }
    tracing::info!("control channel ended");
}

/// Route one line. Returns a reply to send back only when this transport answers it itself —
/// which is to say, only when it refuses.
async fn handle(
    line: &str,
    pool: &mut Pool,
    media: Option<&Media>,
    seat: &Seat,
    vote: Option<&mut Vote>,
) -> Option<String> {
    let request: proto::Request = match serde_json::from_str(line) {
        Ok(request) => request,
        Err(e) => {
            // Unparseable, so there is no `id` to answer against and no method to name. A
            // JSON-RPC parse error with a null id is the honest reply.
            tracing::debug!(error = %e, "unparseable frame on the control channel");
            return Some(error_line(
                None,
                proto::Error::new(proto::code::PARSE_ERROR, "not a JSON-RPC object"),
            ));
        }
    };

    // The id, kept before the request is consumed. A notification has none, and `Response`
    // serialises that as `null` — which is right: a refusal to a notification is still worth
    // sending, because silence would look like acceptance.
    let id = request.id.clone();

    // Answered here, before anything tries to make a `Call` of it: this is `mediad`'s own question
    // about `mediad`'s own pipeline, and there is no service to route it to.
    if request.method == VIDEO_METHOD {
        return Some(match media {
            Some(media) => {
                let mut answer = video_params(&media.video);
                // Whether the picture is being sent, which only this call can say: the
                // notification is pushed once, before anybody has voted.
                answer["on"] = media.gate.as_deref().is_none_or(Gate::flowing).into();
                serde_json::to_string(&proto::Response::ok(id, &answer))
                    .expect("Response serialises")
            }
            None => error_line(
                id,
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "this robot is not publishing video, so there is no geometry to describe",
                ),
            ),
        });
    }

    // This session's vote on the video track — `Gate` has the rule. Answered here for
    // `media.video`'s reason.
    if request.method == SET_VIDEO_METHOD {
        let Some(media) = media else {
            return Some(error_line(
                id,
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "this robot is not publishing video, so there is none to pause",
                ),
            ));
        };
        let params = request
            .params
            .clone()
            .map(serde_json::from_value::<proto::MediaSetVideoParams>);
        let on = match params {
            Some(Ok(params)) => params.on,
            Some(Err(e)) => {
                return Some(error_line(
                    id,
                    proto::Error::new(
                        proto::code::INVALID_PARAMS,
                        format!("media.setVideo takes {{\"on\": true|false}}: {e}"),
                    ),
                ));
            }
            None => {
                return Some(error_line(
                    id,
                    proto::Error::new(
                        proto::code::INVALID_PARAMS,
                        "media.setVideo takes {\"on\": true|false}",
                    ),
                ));
            }
        };
        let switch = switch(on, media.gate.as_deref(), vote);
        return Some(
            serde_json::to_string(&proto::Response::ok(id, &switch)).expect("Response serialises"),
        );
    }

    // **The one call that makes this robot send its camera somewhere it was told about.**
    //
    // Answered here rather than routed for the same reason `media.video` is — the pipeline is
    // `mediad`'s and no service owns it — and it is the reason a frame stream needs no relay
    // candidate: the robot dials out, so a NAT is not a participant. `stream.rs`'s header has the
    // argument; this is where a peer asks for it.
    if request.method == STREAM_METHOD {
        let Some(media) = media else {
            return Some(error_line(
                id,
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "this robot has no camera to stream",
                ),
            ));
        };
        return Some(match stream_request(&request.params) {
            // No `url` key at all is a question rather than an instruction, which is what lets a
            // client show what is streaming without having to remember what it asked for.
            Ask::Status => {
                serde_json::to_string(&proto::Response::ok(id, &media.streamer.status()))
                    .expect("Response serialises")
            }
            Ask::Stop => serde_json::to_string(&proto::Response::ok(id, &media.streamer.stop()))
                .expect("Response serialises"),
            Ask::Start(config) => match media.streamer.start(config) {
                Ok(answer) => serde_json::to_string(&proto::Response::ok(id, &answer))
                    .expect("Response serialises"),
                Err(why) => error_line(id, proto::Error::new(proto::code::INVALID_PARAMS, why)),
            },
        });
    }

    let call = match request.as_call() {
        Ok(call) => call,
        Err(e) => {
            // An unknown method or a params shape this release does not know. `as_call` names
            // which, and that message is the whole value — a version skew fails on the one call
            // that cannot be served rather than on the handshake.
            return Some(error_line(id, e));
        }
    };

    match route::route_for(&call) {
        Route::Refused => {
            tracing::debug!(method = call.method(), "refused over WebRTC");
            Some(error_line(id, route::refusal(&call)))
        }
        Route::To(service, lane) => {
            let exempted = exempt_if_alone(&call, line, seat);
            match pool
                .send(service, lane, exempted.as_deref().unwrap_or(line))
                .await
            {
                Ok(()) => None,
                Err(e) => {
                    // The service is not there, or stopped reading. Answering is important: a
                    // client that gets silence cannot tell "the robot is thinking" from "nothing
                    // will ever come back", and `robotd` is the service most likely to be missing
                    // because it is the one an update restarts.
                    tracing::debug!(method = call.method(), error = %e, "upstream unreachable");
                    Some(error_line(
                        id,
                        proto::Error::new(
                            proto::code::INTERNAL_ERROR,
                            format!("{:?} is not answering: {e}", service),
                        ),
                    ))
                }
            }
        }
    }
}

/// What a `media.setVideo` call does to the video track, and what it is now.
///
/// A caller with no vote — a rendezvous lane, or a pipeline with no valve — is answered with the
/// state it could not change rather than refused, so a client that sends it everywhere reads one
/// field to learn whether it counted.
fn switch(on: bool, gate: Option<&Gate>, vote: Option<&mut Vote>) -> proto::MediaVideoSwitch {
    match (gate, vote) {
        (_, Some(vote)) => proto::MediaVideoSwitch {
            accepted: true,
            on: vote.set(on),
            reason: None,
        },
        (Some(gate), None) => proto::MediaVideoSwitch {
            accepted: false,
            on: gate.flowing(),
            reason: Some(
                "a rendezvous control lane carries no video; vote over the WebRTC session's \
                 control channel"
                    .into(),
            ),
        },
        (None, None) => proto::MediaVideoSwitch {
            accepted: false,
            on: true,
            reason: Some("this pipeline was built without a video valve".into()),
        },
    }
}

/// One refusal, as a line. Built through [`proto::Response`] rather than by hand so the envelope
/// has exactly one definition — the same reason `duck-ipc-proto` exists.
/// What a `media.stream` call is asking for.
///
/// `url` present is start, `url: null` is stop, no `url` key is a question. The same three-way
/// reading `robot.loadPolicy` gives `slot` and `path`, and for the same reason: one method that a
/// client can use to look, to change and to put back is one method to route and one to permit.
enum Ask {
    Status,
    Stop,
    Start(crate::stream::Config),
}

fn stream_request(params: &Option<serde_json::Value>) -> Ask {
    use crate::stream::Config;

    // Absent params is the same question as params with no `url`: a client asking what is
    // streaming should not have to send `{}` to be understood.
    let params = match params {
        None => return Ask::Status,
        Some(params) => params,
    };
    match params.get("url") {
        None => Ask::Status,
        Some(serde_json::Value::Null) => Ask::Stop,
        Some(url) => Ask::Start(Config {
            url: url.as_str().unwrap_or_default().to_owned(),
            // Every number is optional: what a caller almost always wants is "stream to here",
            // and a default that is cheap enough to ignore is better than four required fields.
            fps: params
                .get("fps")
                .and_then(serde_json::Value::as_f64)
                .unwrap_or(Config::DEFAULT_FPS),
            longest: params
                .get("longest")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(Config::DEFAULT_LONGEST as u64) as u32,
            quality: params
                .get("quality")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(Config::DEFAULT_QUALITY as u64)
                .min(100) as u8,
            // H.264 unless asked otherwise: the encode is the VPU's rather than a core's, and
            // inter-frame prediction is worth five to fifteen times the bytes over the same
            // wifi. JPEG stays reachable because it needs no keyframe to start and no decoder
            // state to keep, which is what a receiver that reconnects constantly wants.
            encoding: params
                .get("encoding")
                .and_then(serde_json::Value::as_str)
                .and_then(crate::stream::Encoding::parse)
                .unwrap_or_default(),
        }),
    }
}

fn error_line(id: Option<proto::Id>, error: proto::Error) -> String {
    // A `Response` cannot fail to serialise: every field is a `String`, an `Id` or an `Error`.
    serde_json::to_string(&proto::Response::err(id, error)).expect("Response serialises")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::upstream::Sockets;
    use std::path::Path;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    /// A daemon that reads one line, replies with the next canned line, and remembers what it saw.
    fn fake_daemon(
        dir: &Path,
        name: &str,
        replies: Vec<String>,
    ) -> mpsc::UnboundedReceiver<String> {
        let path = dir.join(name);
        let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
        listener.set_nonblocking(true).unwrap();
        let listener = tokio::net::UnixListener::from_std(listener).unwrap();
        let (seen_tx, seen_rx) = mpsc::unbounded_channel();

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

    struct Harness {
        to_peer: mpsc::Receiver<String>,
        from_peer: mpsc::Sender<String>,
        _dir: tempfile::TempDir,
    }

    /// A peer with nobody else connected, which is how every test but the ones about company sees
    /// the robot. `None`, not `Some(1)`: no test should depend on a `/run/mediad` it cannot see.
    fn alone() -> Seat {
        Seat {
            counted: std::sync::Arc::new(|| true),
            peers: || None,
            votes: true,
        }
    }

    fn harness(sockets: Sockets, dir: tempfile::TempDir) -> Harness {
        seated(sockets, dir, alone())
    }

    fn seated(sockets: Sockets, dir: tempfile::TempDir, seat: Seat) -> Harness {
        gated(sockets, dir, seat, None)
    }

    fn gated(
        sockets: Sockets,
        dir: tempfile::TempDir,
        seat: Seat,
        gate: Option<std::sync::Arc<Gate>>,
    ) -> Harness {
        let (from_peer, inbound) = mpsc::channel(16);
        let (outbound, to_peer) = mpsc::channel(64);
        let (replies_tx, mut replies_rx) = mpsc::channel::<String>(64);
        let pool = Pool::new(sockets, replies_tx);

        // Everything a service emits goes to the peer, exactly as the datachannel writer will.
        let peer_writer = outbound.clone();
        tokio::spawn(async move {
            while let Some(line) = replies_rx.recv().await {
                if peer_writer.send(line).await.is_err() {
                    break;
                }
            }
        });
        tokio::spawn(run(
            inbound,
            outbound,
            pool,
            Some(Media {
                video: Video {
                    width: 1280,
                    height: 720,
                    rotate: 90,
                    intrinsics: crate::camera::Intrinsics::nominal(
                        Some(&crate::sensor::IMX219),
                        1280,
                        720,
                    ),
                },
                // A streamer whose encoder produces nothing, which is all this needs: the tests
                // here are about which method is answered by whom, and an encoder that touched a
                // pipeline would make the whole file unbuildable off a board.
                streamer: std::sync::Arc::new(crate::stream::Streamer::new(
                    crate::stream::Encoders {
                        jpeg: std::sync::Arc::new(|_| None),
                        h264: Some(std::sync::Arc::new(|_| None)),
                        gate: None,
                    },
                    crate::producer::Producer {
                        name: Some("olducky".to_owned()),
                        serial: Some("3fa1c51b".to_owned()),
                        release: "0.10.0".to_owned(),
                        api_version: proto::API_VERSION,
                        simulated: false,
                    },
                    90,
                    dir.path().join("hf-token"),
                )),
                gate,
            }),
            seat,
        ));
        Harness {
            to_peer,
            from_peer,
            _dir: dir,
        }
    }

    fn sockets_in(dir: &Path) -> Sockets {
        Sockets {
            updater: dir.join("updaterd.sock"),
            robot: dir.join("robotd.sock"),
            config: dir.join("configd.sock"),
            pad: dir.join("pad.sock"),
            tof: dir.join("tof.sock"),
        }
    }

    /// A permitted call reaches the service that owns it, and its reply comes back.
    #[tokio::test]
    async fn a_permitted_call_reaches_its_service() {
        let dir = tempfile::tempdir().unwrap();
        let mut robot_seen = fake_daemon(
            dir.path(),
            "robotd.sock",
            vec![r#"{"jsonrpc":"2.0","id":1,"result":{"healthy":true}}"#.into()],
        );
        let mut updater_seen = fake_daemon(dir.path(), "updaterd.sock", vec![]);
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer
            .send(r#"{"jsonrpc":"2.0","id":1,"method":"robot.health"}"#.into())
            .await
            .unwrap();

        let seen = robot_seen.recv().await.unwrap();
        assert!(seen.contains("robot.health"), "{seen}");
        let reply = h.to_peer.recv().await.unwrap();
        assert!(reply.contains(r#""healthy":true"#), "{reply}");
        // And it did not go to the wrong service.
        assert!(updater_seen.try_recv().is_err());
    }

    fn os_apply(params: &str) -> (proto::Call, String) {
        let line = format!(r#"{{"jsonrpc":"2.0","id":3,"method":"os.apply"{params}}}"#);
        let call = serde_json::from_str::<proto::Request>(&line)
            .unwrap()
            .as_call()
            .unwrap();
        (call, line)
    }

    fn interrupts(line: &str) -> bool {
        let request: serde_json::Value = serde_json::from_str(line).unwrap();
        request["params"]["interrupt_sessions"] == true
    }

    /// Who counts as company: the other datachannels, and every datachannel for the lane.
    #[test]
    fn a_session_is_alone_when_nobody_else_would_be_dropped() {
        let peer = |n: fn() -> Option<u32>| Seat {
            counted: std::sync::Arc::new(|| true),
            peers: n,
            votes: true,
        };
        let lane = |n: fn() -> Option<u32>| Seat {
            counted: std::sync::Arc::new(|| false),
            peers: n,
            votes: false,
        };
        // A lane whose client is also watching: its own video is not company.
        let watching = |n: fn() -> Option<u32>| Seat {
            counted: std::sync::Arc::new(|| true),
            peers: n,
            votes: false,
        };
        assert!(peer(|| Some(1)).alone(), "only itself");
        assert!(!peer(|| Some(2)).alone(), "one other peer");
        assert!(lane(|| Some(0)).alone());
        assert!(!lane(|| Some(1)).alone(), "a peer the lane is not");
        assert!(
            watching(|| Some(1)).alone(),
            "the lane's own client's video"
        );
        assert!(!watching(|| Some(2)).alone(), "and someone else's");
        // `updaterd` lets an unknown count through, so treating it as company here would refuse
        // nothing and only log a lie.
        assert!(peer(|| None).alone());
        assert!(lane(|| None).alone());
    }

    /// Alone, an OS update goes with the check answered; the rest of the request is untouched.
    #[test]
    fn an_os_update_from_a_session_alone_skips_the_session_check() {
        let (call, line) = os_apply(r#","params":{"build":"20261028093000"}"#);
        let sent = exempt_if_alone(&call, &line, &alone()).expect("rewritten");
        assert!(interrupts(&sent), "{sent}");
        assert!(sent.contains("20261028093000"), "{sent}");
        assert!(sent.contains(r#""id":3"#), "{sent}");

        // No `params` at all is the channel's newest, and still gets the flag.
        let (call, line) = os_apply("");
        assert!(interrupts(
            &exempt_if_alone(&call, &line, &alone()).unwrap()
        ));
        let switch = r#"{"jsonrpc":"2.0","id":4,"method":"os.switch","params":null}"#;
        let call = serde_json::from_str::<proto::Request>(switch)
            .unwrap()
            .as_call()
            .unwrap();
        assert!(interrupts(
            &exempt_if_alone(&call, switch, &alone()).unwrap()
        ));
    }

    /// With company, the call goes as sent, so `updaterd` refuses on the company's behalf.
    #[test]
    fn an_os_update_with_company_is_left_for_updaterd_to_refuse() {
        let crowded = Seat {
            counted: std::sync::Arc::new(|| true),
            peers: || Some(2),
            votes: true,
        };
        let (call, line) = os_apply("");
        assert_eq!(exempt_if_alone(&call, &line, &crowded), None);

        // A client that asked to interrupt has nothing to rewrite.
        let (call, line) = os_apply(r#","params":{"interrupt_sessions":true}"#);
        assert_eq!(exempt_if_alone(&call, &line, &alone()), None);

        // And nothing else is touched: a daemon release is refused here anyway, and every other
        // call is the peer's own.
        let line = r#"{"jsonrpc":"2.0","id":5,"method":"robot.health"}"#;
        let call = serde_json::from_str::<proto::Request>(line)
            .unwrap()
            .as_call()
            .unwrap();
        assert_eq!(exempt_if_alone(&call, line, &alone()), None);
    }

    /// End to end: what reaches `updaterd` is the rewritten line.
    #[tokio::test]
    async fn updaterd_receives_the_exempted_os_update() {
        let dir = tempfile::tempdir().unwrap();
        let mut updater_seen = fake_daemon(dir.path(), "updaterd.sock", vec![]);
        let h = harness(sockets_in(dir.path()), dir);

        h.from_peer.send(os_apply("").1).await.unwrap();

        let seen = updater_seen.recv().await.unwrap();
        assert!(interrupts(&seen), "{seen}");
    }

    /// A refused call is answered by `mediad` and never reaches a socket.
    ///
    /// `net.connect` is the interesting one: it is *permitted over BLE* and refused here, because
    /// reconfiguring wifi would take this session with it.
    #[tokio::test]
    async fn a_refused_call_is_answered_here_and_forwarded_nowhere() {
        let dir = tempfile::tempdir().unwrap();
        let mut config_seen = fake_daemon(dir.path(), "configd.sock", vec![]);
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer
            .send(
                r#"{"jsonrpc":"2.0","id":7,"method":"net.connect","params":{"ssid":"x","psk":"y"}}"#
                    .into(),
            )
            .await
            .unwrap();

        let reply = h.to_peer.recv().await.unwrap();
        assert!(reply.contains("not available over WebRTC"), "{reply}");
        assert!(reply.contains(r#""id":7"#), "{reply}");
        assert!(
            config_seen.try_recv().is_err(),
            "a refused call reached configd"
        );
    }

    /// The pad tap reaches `padd`, which is a socket `btd` deliberately does not hold.
    ///
    /// The concrete difference between the two transports, so it is worth a test of its own rather
    /// than only a route-table assertion.
    #[tokio::test]
    async fn the_pad_tap_reaches_padd() {
        let dir = tempfile::tempdir().unwrap();
        let mut pad_seen = fake_daemon(dir.path(), "pad.sock", vec![]);
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer
            .send(r#"{"jsonrpc":"2.0","id":2,"method":"pad.input"}"#.into())
            .await
            .unwrap();

        let seen = pad_seen.recv().await.unwrap();
        assert!(seen.contains("pad.input"), "{seen}");
        // Nothing came back to the peer: `padd` answers with a stream, and this transport does not
        // invent a reply for it.
        assert!(h.to_peer.try_recv().is_err());
    }

    /// Every notification in a stream reaches the peer.
    ///
    /// This is the case that breaks if replies are ever correlated to requests, which is why it is
    /// pinned here as well as in `btd`.
    #[tokio::test]
    async fn every_notification_in_a_stream_reaches_the_peer() {
        let dir = tempfile::tempdir().unwrap();
        let progress: Vec<String> = (0..3)
            .map(|i| {
                format!(
                    r#"{{"jsonrpc":"2.0","method":"update.progress","params":{{"percent":{}}}}}"#,
                    i * 50
                )
            })
            .collect();
        let _updater_seen = fake_daemon(dir.path(), "updaterd.sock", progress);
        let mut h = harness(sockets_in(dir.path()), dir);

        // One request, then two more lines to make the fake emit its remaining replies.
        for _ in 0..3 {
            h.from_peer
                .send(r#"{"jsonrpc":"2.0","id":3,"method":"update.subscribe"}"#.into())
                .await
                .unwrap();
        }

        let mut seen = 0;
        for _ in 0..3 {
            let line = h.to_peer.recv().await.unwrap();
            assert!(line.contains("update.progress"), "{line}");
            seen += 1;
        }
        assert_eq!(seen, 3);
    }

    /// A missing service is reported, not met with silence.
    ///
    /// `robotd` is the one most likely to be absent, because it is the one an update restarts — and
    /// a peer that gets nothing back cannot tell "thinking" from "never coming".
    #[tokio::test]
    async fn an_absent_service_is_reported() {
        let dir = tempfile::tempdir().unwrap();
        // No fake daemon at all: the socket does not exist.
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer
            .send(r#"{"jsonrpc":"2.0","id":9,"method":"robot.health"}"#.into())
            .await
            .unwrap();

        let reply = h.to_peer.recv().await.unwrap();
        assert!(reply.contains("is not answering"), "{reply}");
        assert!(reply.contains(r#""id":9"#), "{reply}");
    }

    /// An unknown method is refused by name rather than forwarded, which is what makes a version
    /// skew fail on the one call that cannot be served.
    #[tokio::test]
    async fn an_unknown_method_names_itself() {
        let dir = tempfile::tempdir().unwrap();
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer
            .send(r#"{"jsonrpc":"2.0","id":4,"method":"robot.teleport"}"#.into())
            .await
            .unwrap();

        let reply = h.to_peer.recv().await.unwrap();
        assert!(reply.contains("robot.teleport"), "{reply}");
    }

    /// The line that tells a page how the camera is mounted, and a consumer what its geometry is.
    ///
    /// This is the only thing between a console that rotates the picture and one that shows it
    /// sideways and says nothing — and now also between a perception consumer that can turn a
    /// pixel into a direction and one that has to guess a focal length.
    #[test]
    fn the_video_notification_carries_the_mount_rotation_and_the_geometry() {
        let line = video_notification(&Video {
            width: 1280,
            height: 720,
            rotate: 90,
            intrinsics: crate::camera::Intrinsics::nominal(Some(&crate::sensor::IMX219), 1280, 720),
        });
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("valid json");
        assert_eq!(parsed["method"], "media.video");
        assert!(parsed.get("id").is_none(), "a notification carries no id");
        assert_eq!(parsed["params"]["width"], 1280);
        assert_eq!(parsed["params"]["height"], 720);
        assert_eq!(parsed["params"]["rotate"], 90);

        let intrinsics = &parsed["params"]["intrinsics"];
        assert!(
            (intrinsics["fx"].as_f64().unwrap() - 1065.14).abs() < 0.1,
            "{intrinsics}"
        );
        assert_eq!(intrinsics["cx"], 640.0);
        assert_eq!(
            intrinsics["calibrated"], false,
            "the module's design figures, and a consumer has to be able to tell"
        );
    }

    /// A robot whose camera geometry is unknown omits the key rather than sending a null.
    ///
    /// Which is every robot streaming a test pattern, and any board where `media-ctl` would not
    /// set the sensor mode. A consumer reading a missing key knows it has to calibrate; one
    /// reading `null` has to be told what that meant.
    #[test]
    fn an_unknown_geometry_is_absent_from_the_line() {
        let line = video_notification(&Video {
            width: 1280,
            height: 720,
            rotate: 0,
            intrinsics: None,
        });
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("valid json");
        assert!(parsed["params"].get("intrinsics").is_none(), "{line}");
        assert_eq!(parsed["params"]["width"], 1280);
    }

    /// `media.video` is answered by the session, not routed to a service.
    ///
    /// This is the path the console uses, and it exists because pushing the same information when
    /// the channel appears races the browser's datachannel and loses — a sideways picture with
    /// nothing in the log. A question the page asks when it is ready cannot arrive too early.
    /// `media.stream` reads three ways off one key, and the validation is here rather than later.
    ///
    /// No `url` is a question, so a client can show what is streaming without remembering what it
    /// asked for. And a url that is not a WebSocket is refused *before* a thread and a socket are
    /// spawned for it — a robot that accepted `http://` would sit in a reconnect loop against
    /// something that can never upgrade, having told the caller yes.
    #[tokio::test]
    async fn media_stream_answers_the_question_and_refuses_a_bad_url() {
        let dir = tempfile::tempdir().unwrap();
        let mut h = harness(sockets_in(dir.path()), dir);

        async fn ask(h: &mut Harness, line: &str) -> serde_json::Value {
            h.from_peer.send(line.into()).await.unwrap();
            serde_json::from_str(&h.to_peer.recv().await.unwrap()).expect("valid json")
        }

        let answer = ask(
            &mut h,
            r#"{"jsonrpc":"2.0","id":1,"method":"media.stream"}"#,
        )
        .await;
        assert_eq!(answer["result"]["streaming"], false, "{answer}");
        assert_eq!(answer["result"]["sent"], 0);

        let answer = ask(
            &mut h,
            r#"{"jsonrpc":"2.0","id":2,"method":"media.stream","params":{"url":"http://a.b"}}"#,
        )
        .await;
        assert_eq!(
            answer["error"]["code"],
            proto::code::INVALID_PARAMS,
            "{answer}"
        );
        assert!(
            answer["error"]["message"]
                .as_str()
                .unwrap()
                .contains("ws://"),
            "the refusal says what a url has to be: {answer}"
        );

        // Stopping something that was never started is not an error: a client tidying up after
        // itself should not have to know whether there was anything to tidy.
        let answer = ask(
            &mut h,
            r#"{"jsonrpc":"2.0","id":3,"method":"media.stream","params":{"url":null}}"#,
        )
        .await;
        assert_eq!(answer["result"]["streaming"], false, "{answer}");
        assert_eq!(answer["result"]["was"], serde_json::Value::Null);
    }

    #[tokio::test]
    async fn the_page_can_ask_what_the_video_is() {
        let dir = tempfile::tempdir().unwrap();
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer
            .send(r#"{"jsonrpc":"2.0","id":7,"method":"media.video","params":{}}"#.into())
            .await
            .unwrap();

        let reply = h.to_peer.recv().await.unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&reply).expect("valid json");
        assert_eq!(parsed["id"], 7, "{reply}");
        assert_eq!(parsed["result"]["width"], 1280);
        assert_eq!(parsed["result"]["height"], 720);
        assert_eq!(parsed["result"]["rotate"], 90);
        // No service was involved: it is answered here, and nothing was forwarded.
        assert!(parsed.get("error").is_none(), "{reply}");
    }

    /// A gate whose valve is a log of what it was told.
    fn recorded() -> (
        std::sync::Arc<Gate>,
        std::sync::Arc<std::sync::Mutex<Vec<bool>>>,
    ) {
        let told = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = std::sync::Arc::clone(&told);
        (Gate::new(move |on| log.lock().unwrap().push(on)), told)
    }

    /// **The console is never surprised.** Video flows while any session wants it, a session wants
    /// it until it says otherwise, and a vote leaves with its session — so a bridge alone can
    /// pause the picture, a console arriving resumes it, and the bridge is paused again only once
    /// the console has gone. The valve is told only on a change, because each telling asks the
    /// encoders for a keyframe.
    #[test]
    fn video_flows_while_any_session_wants_it() {
        let (gate, told) = recorded();
        assert!(gate.flowing(), "built open, with nobody here");

        let mut bridge = Vote::cast(gate.clone());
        assert!(gate.flowing(), "a new session wants the video");
        assert!(!bridge.set(false), "alone, it can pause it");
        assert!(!bridge.set(false), "and saying so twice changes nothing");

        let console = Vote::cast(gate.clone());
        assert!(gate.flowing(), "a session that arrives wants the picture");
        drop(console);
        assert!(!gate.flowing(), "and its vote leaves with it");

        assert!(bridge.set(true));
        assert!(!bridge.set(false));
        drop(bridge);
        assert!(
            gate.flowing(),
            "nobody left: open again, so the next consumer's negotiation has caps"
        );
        assert_eq!(
            *told.lock().unwrap(),
            [false, true, false, true, false, true]
        );
    }

    /// A session ending with its vote still on takes nothing off with it.
    #[test]
    fn a_session_leaving_with_the_video_wanted_changes_nothing() {
        let (gate, told) = recorded();
        let mut bridge = Vote::cast(gate.clone());
        let console = Vote::cast(gate.clone());
        assert!(bridge.set(false), "the console still wants it");
        drop(console);
        assert!(!gate.flowing());
        drop(bridge);
        assert_eq!(*told.lock().unwrap(), [false, true]);
    }

    async fn ask(h: &mut Harness, line: &str) -> serde_json::Value {
        h.from_peer.send(line.into()).await.unwrap();
        serde_json::from_str(&h.to_peer.recv().await.unwrap()).expect("valid json")
    }

    /// `media.setVideo` over a datachannel is this session's vote, answered with `accepted` and
    /// what the track is doing now — which `media.video` reports too. And the session's end takes
    /// the vote with it: the channel closing is what reopens the valve.
    #[tokio::test]
    async fn media_set_video_is_this_sessions_vote() {
        let dir = tempfile::tempdir().unwrap();
        let (gate, told) = recorded();
        let mut h = gated(sockets_in(dir.path()), dir, alone(), Some(gate.clone()));
        let set = |id: u32, params: &str| {
            format!(r#"{{"jsonrpc":"2.0","id":{id},"method":"media.setVideo","params":{params}}}"#)
        };
        let read = r#"{"jsonrpc":"2.0","id":9,"method":"media.video","params":{}}"#;

        let video = ask(&mut h, read).await;
        assert_eq!(video["result"]["width"], 1280, "{video}");
        assert_eq!(video["result"]["on"], true, "a new session's video is on");

        let off = ask(&mut h, &set(2, r#"{"on":false}"#)).await;
        assert_eq!(
            off["result"],
            serde_json::json!({"accepted": true, "on": false}),
            "{off}"
        );
        assert!(!gate.flowing());
        let video = ask(&mut h, read).await;
        assert_eq!(video["result"]["on"], false, "the read says so too");

        let on = ask(&mut h, &set(3, r#"{"on":true}"#)).await;
        assert_eq!(on["result"]["on"], true, "{on}");

        for bad in [r#"{"on":"off"}"#, "{}", "null"] {
            let bad = ask(&mut h, &set(4, bad)).await;
            assert_eq!(bad["error"]["code"], proto::code::INVALID_PARAMS, "{bad}");
        }
        let bare = ask(
            &mut h,
            r#"{"jsonrpc":"2.0","id":5,"method":"media.setVideo"}"#,
        )
        .await;
        assert_eq!(bare["error"]["code"], proto::code::INVALID_PARAMS, "{bare}");

        let off = ask(&mut h, &set(6, r#"{"on":false}"#)).await;
        assert_eq!(off["result"]["on"], false, "{off}");

        // The peer goes; `run` ends, and the vote with it.
        drop(h.from_peer);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while !gate.flowing() {
            assert!(
                std::time::Instant::now() < deadline,
                "the vote outlived its session"
            );
            tokio::time::sleep(std::time::Duration::from_millis(5)).await;
        }
        assert_eq!(*told.lock().unwrap(), [false, true, false, true]);
    }

    /// A rendezvous lane has no video track and so no vote: its `on` is answered, not counted,
    /// and the answer says why.
    #[tokio::test]
    async fn a_lane_cannot_pause_the_video() {
        let dir = tempfile::tempdir().unwrap();
        let (gate, told) = recorded();
        let lane = Seat {
            counted: std::sync::Arc::new(|| false),
            peers: || None,
            votes: false,
        };
        let mut h = gated(sockets_in(dir.path()), dir, lane, Some(gate.clone()));

        let off = ask(
            &mut h,
            r#"{"jsonrpc":"2.0","id":1,"method":"media.setVideo","params":{"on":false}}"#,
        )
        .await;
        assert_eq!(off["result"]["accepted"], false, "{off}");
        assert_eq!(off["result"]["on"], true);
        assert!(off["result"]["reason"].as_str().unwrap().contains("lane"));
        assert!(gate.flowing());
        assert!(told.lock().unwrap().is_empty());
    }

    /// Garbage is answered rather than dropped, with a null id because there is none to echo.
    #[tokio::test]
    async fn garbage_gets_a_parse_error() {
        let dir = tempfile::tempdir().unwrap();
        let mut h = harness(sockets_in(dir.path()), dir);

        h.from_peer.send("not json at all".into()).await.unwrap();

        let reply = h.to_peer.recv().await.unwrap();
        assert!(reply.contains("not a JSON-RPC object"), "{reply}");
        assert!(reply.contains(r#""id":null"#), "{reply}");
    }
}
