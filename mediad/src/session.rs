//! One peer's control channel, as a pipe to the services that own the answers.
//!
//! Transport-agnostic on purpose: this takes lines in and gives lines out, and knows nothing about
//! datachannels. That is what makes it testable without a WebRTC peer — the tests below drive it
//! over channels against fake daemons on real unix sockets — and it is also what would let a
//! WebSocket surface (`remote-webrtc.md` §11) reuse it unchanged.
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

/// The method a peer asks with. Answered here rather than routed: no service owns it.
const VIDEO_METHOD: &str = "media.video";

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
    });
    // Absent rather than null when the geometry is unknown: a consumer reading a missing key knows
    // it must calibrate, where one reading `null` has to be told what that meant.
    if let Some(intrinsics) = &video.intrinsics {
        params["intrinsics"] = serde_json::to_value(intrinsics).expect("intrinsics serialise");
    }
    params
}

pub async fn run(
    mut inbound: mpsc::Receiver<String>,
    outbound: mpsc::Sender<String>,
    mut pool: Pool,
    video: Video,
) {
    while let Some(line) = inbound.recv().await {
        let line = line.trim().to_string();
        if line.is_empty() {
            continue;
        }
        if let Some(reply) = handle(&line, &mut pool, &video).await {
            // A closed outbound means the peer is gone; there is nothing left to do for it.
            if outbound.send(reply).await.is_err() {
                break;
            }
        }
    }
    tracing::debug!("control channel ended");
}

/// Route one line. Returns a reply to send back only when this transport answers it itself —
/// which is to say, only when it refuses.
async fn handle(line: &str, pool: &mut Pool, video: &Video) -> Option<String> {
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
        return Some(
            serde_json::to_string(&proto::Response::ok(id, &video_params(video)))
                .expect("Response serialises"),
        );
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
            match pool.send(service, lane, line).await {
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

/// One refusal, as a line. Built through [`proto::Response`] rather than by hand so the envelope
/// has exactly one definition — the same reason `duck-ipc-proto` exists.
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

    fn harness(sockets: Sockets, dir: tempfile::TempDir) -> Harness {
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
            Video {
                width: 1280,
                height: 720,
                rotate: 90,
                intrinsics: crate::camera::Intrinsics::nominal(
                    Some(crate::camera::SensorMode::PINNED),
                    1280,
                    720,
                ),
            },
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
            intrinsics: crate::camera::Intrinsics::nominal(
                Some(crate::camera::SensorMode::PINNED),
                1280,
                720,
            ),
        });
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("valid json");
        assert_eq!(parsed["method"], "media.video");
        assert!(parsed.get("id").is_none(), "a notification carries no id");
        assert_eq!(parsed["params"]["width"], 1280);
        assert_eq!(parsed["params"]["height"], 720);
        assert_eq!(parsed["params"]["rotate"], 90);

        let intrinsics = &parsed["params"]["intrinsics"];
        assert!(
            (intrinsics["fx"].as_f64().unwrap() - 1809.52).abs() < 0.01,
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
