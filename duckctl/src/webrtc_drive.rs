//! `duckctl webrtc-drive` — the pad plugged into this machine drives the robot over WebRTC.
//!
//! The same session the console opens, from Rust instead of a browser: signalling on `mediad`'s
//! WebSocket (`ws://<robot>:8443`), the robot's offer answered here, and the `control`
//! datachannel the robot creates for every consumer (`remote-webrtc.md` §5). Nothing changes on
//! the robot — this is one more consumer of the pipeline it already runs.
//!
//! The driving is `padd`'s, unchanged: a Unix socket here stands in for `robotd`'s, and every
//! line `padd` writes to it goes down `control` as it is. `mediad` routes each one to the service
//! that owns it, with the WebRTC route table deciding what may pass — and every call `padd` makes
//! is on it.
//!
//! **Only replies come back to `padd`.** It reads "the next line" as the answer to the request it
//! just made, and `control` also carries lines nobody asked for: `media.video` describing the
//! picture, a refusal of a notification (id `null`). One of those in the middle would shift every
//! later answer by one, silently. `padd` subscribes to nothing, so a line with no id — or a null
//! one — is never for it and is dropped here; see [`is_reply`].
//!
//! The robot's button bindings come over the channel too (`pad.bindings`), so the pad on this desk
//! runs the robot's `[pad]`. The walking speeds (`[pad_drive]`) have no call to read them by, so
//! `padd` uses its defaults for those.
//!
//! The robot's own `padd` is left running: there is no way to stop it without a shell, and with
//! no pad connected to the robot it sends nothing.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use duck_ipc_proto as proto;
use futures::{SinkExt, StreamExt};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;
use webrtc::data_channel::{DataChannel, DataChannelEvent};
use webrtc::media_stream::track_remote::TrackRemote;
use webrtc::peer_connection::{
    MediaEngine, PeerConnection, PeerConnectionBuilder, PeerConnectionEventHandler,
    RTCConfigurationBuilder, RTCIceCandidateInit, RTCPeerConnectionIceEvent,
    RTCPeerConnectionState, RTCSessionDescription, Registry, register_default_interceptors,
};
use webrtc::rtp_transceiver::RTCRtpTransceiverDirection;

/// `mediad`'s signalling port (`--port`'s default there).
pub const SIGNALLING_PORT: u16 = 8443;

/// How long to wait for the session to reach an open `control` channel.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);

/// How long to wait for the robot to answer `pad.bindings`.
const BINDINGS_TIMEOUT: Duration = Duration::from_secs(5);

/// The id the bindings request goes out under — a string, so it cannot collide with `padd`'s
/// numbered requests.
const BINDINGS_ID: &str = "duckctl-pad-bindings";

/// Run a whole session against the robot at `address`, signalling on `port`.
pub async fn run(address: &str, port: u16) -> Result<(), Box<dyn std::error::Error>> {
    let padd = find_padd();
    let dir = std::env::temp_dir().join(format!("duckctl-webrtc-{}", std::process::id()));
    std::fs::create_dir_all(&dir)?;
    let result = session(address, port, &padd, &dir).await;
    let _ = std::fs::remove_dir_all(&dir);
    result
}

async fn session(
    address: &str,
    port: u16,
    padd: &std::path::Path,
    dir: &std::path::Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let url = format!("ws://{address}:{port}");
    eprintln!("signalling on {url}…");
    let (ws, _) = tokio_tungstenite::connect_async(&url)
        .await
        .map_err(|e| format!("could not reach the robot's signalling server at {url}: {e}"))?;

    let (events, mut event_rx) = mpsc::unbounded_channel::<Event>();
    let (pc, ice_rx) = peer_connection(events.clone()).await?;
    let pc = Arc::new(pc);
    tokio::spawn(signalling(ws, Arc::clone(&pc), ice_rx));

    // The channel the robot creates, once the session has got that far.
    let control = tokio::time::timeout(CONNECT_TIMEOUT, async {
        match event_rx.recv().await {
            Some(Event::DataChannel(channel)) => Ok(channel),
            Some(Event::Failed(why)) => Err(why),
            None => Err("the session ended before the robot opened its control channel".to_owned()),
        }
    })
    .await
    .map_err(|_| "no control channel from the robot within 20 s".to_owned())??;

    // One task owns the channel's events: `poll` is not to be shared.
    let (opened_tx, opened_rx) = tokio::sync::oneshot::channel();
    let (from_robot, mut from_robot_rx) = mpsc::unbounded_channel::<String>();
    {
        let control = Arc::clone(&control);
        tokio::spawn(async move {
            let mut opened = Some(opened_tx);
            while let Some(event) = control.poll().await {
                match event {
                    DataChannelEvent::OnOpen => {
                        if let Some(opened) = opened.take() {
                            let _ = opened.send(());
                        }
                    }
                    DataChannelEvent::OnMessage(message) => {
                        let text = String::from_utf8_lossy(&message.data).into_owned();
                        if from_robot.send(text).is_err() {
                            break;
                        }
                    }
                    DataChannelEvent::OnClose => break,
                    _ => {}
                }
            }
        });
    }
    tokio::time::timeout(CONNECT_TIMEOUT, opened_rx)
        .await
        .map_err(|_| "the control channel did not open within 20 s".to_owned())?
        .map_err(|_| "the control channel closed before it opened".to_owned())?;
    eprintln!("control channel open");

    // The robot's bindings, so the pad here runs its `[pad]`.
    let config = dir.join("robotd.toml");
    match fetch_bindings(&control, &mut from_robot_rx).await {
        Ok(toml) => {
            std::fs::write(&config, toml)?;
            eprintln!("using the robot's button bindings");
        }
        Err(e) => eprintln!("note: could not read the robot's button bindings ({e}); defaults"),
    }

    let socket = dir.join("robotd.sock");
    let listener = tokio::net::UnixListener::bind(&socket)?;
    let mut padd = tokio::process::Command::new(padd)
        .arg("--socket")
        .arg(&socket)
        .arg("--config")
        .arg(&config)
        .arg("--tap-socket")
        .arg(dir.join("pad.sock"))
        .kill_on_drop(true)
        .spawn()
        .map_err(|e| {
            format!(
                "could not run {}: {e}\n`padd` has to be installed on this machine: `cargo install \
                 --path padd` from the repository, beside `duckctl`.",
                padd.display()
            )
        })?;

    let (stream, _) = tokio::select! {
        accepted = listener.accept() => accepted?,
        status = padd.wait() => return Err(format!("padd ended before connecting: {status:?}").into()),
    };
    eprintln!("driving {address} over WebRTC from this machine's pad — Ctrl-C to stop");
    let (read, mut write) = stream.into_split();
    let mut from_padd = BufReader::new(read).lines();

    let outcome = loop {
        tokio::select! {
            line = from_padd.next_line() => match line {
                Ok(Some(line)) => {
                    if let Err(e) = control.send_text(&line).await {
                        break Err(format!("the control channel refused a line: {e}"));
                    }
                }
                // padd hung up; its exit is reported below.
                Ok(None) | Err(_) => break Ok(()),
            },
            message = from_robot_rx.recv() => match message {
                Some(message) => {
                    for line in message.lines().filter(|line| is_reply(line)) {
                        if write.write_all(format!("{line}\n").as_bytes()).await.is_err() {
                            break;
                        }
                    }
                }
                None => break Err("the robot closed the control channel".to_owned()),
            },
            event = event_rx.recv() => {
                if let Some(Event::Failed(why)) = event {
                    break Err(why);
                }
            }
            // The interrupt is padd's: the terminal sends it to both, padd stops, and its
            // hang-up ends this loop. Caught here only so this process outlives it and closes
            // the session properly.
            _ = tokio::signal::ctrl_c() => {}
            status = padd.wait() => {
                if let Ok(status) = status
                    && !status.success()
                {
                    eprintln!("padd ended: {status}");
                }
                break Ok(());
            }
        }
    };

    drop(write);
    let _ = tokio::time::timeout(Duration::from_secs(2), padd.wait()).await;
    let _ = pc.close().await;
    outcome.map_err(Into::into)
}

/// Whether a line from `control` is an answer to a request — the only thing `padd` waits for.
///
/// A line with no id, or a null one, is a notification or a refusal of one: never an answer to
/// anything `padd` asked, and passed to it would be read as the answer to its next request.
fn is_reply(line: &str) -> bool {
    serde_json::from_str::<serde_json::Value>(line)
        .ok()
        .and_then(|value| value.get("id").cloned())
        .is_some_and(|id| !id.is_null())
}

/// What the peer connection reports that the session cares about.
enum Event {
    DataChannel(Arc<dyn DataChannel>),
    Failed(String),
}

/// The local ICE candidates, for the signalling task to send.
type IceRx = mpsc::UnboundedReceiver<RTCIceCandidateInit>;

struct Handler {
    events: mpsc::UnboundedSender<Event>,
    ice: mpsc::UnboundedSender<RTCIceCandidateInit>,
}

#[async_trait::async_trait]
impl PeerConnectionEventHandler for Handler {
    async fn on_ice_candidate(&self, event: RTCPeerConnectionIceEvent) {
        if let Ok(init) = event.candidate.to_json() {
            let _ = self.ice.send(init);
        }
    }

    async fn on_connection_state_change(&self, state: RTCPeerConnectionState) {
        if state == RTCPeerConnectionState::Failed {
            let _ = self.events.send(Event::Failed(
                "the WebRTC connection failed — the robot and this machine could not reach each \
                 other directly"
                    .to_owned(),
            ));
        }
    }

    async fn on_data_channel(&self, channel: Arc<dyn DataChannel>) {
        if channel.label().await.is_ok_and(|label| label == "control") {
            let _ = self.events.send(Event::DataChannel(channel));
        }
    }

    /// The camera arrives in every session; nothing here shows it, so it is read and dropped
    /// rather than left to back up.
    async fn on_track(&self, track: Arc<dyn TrackRemote>) {
        tokio::spawn(async move { while track.poll().await.is_some() {} });
    }
}

/// The peer connection, and the local ICE candidates it gathers for the signalling task to send.
async fn peer_connection(
    events: mpsc::UnboundedSender<Event>,
) -> Result<(impl PeerConnection, IceRx), Box<dyn std::error::Error>> {
    let (ice, ice_rx) = mpsc::unbounded_channel();

    let mut media_engine = MediaEngine::default();
    // The robot offers H.264 video, and an answer has to say something about it.
    media_engine.register_default_codecs()?;
    let registry = register_default_interceptors(Registry::new(), &mut media_engine)?;
    let runtime = webrtc::runtime::default_runtime().ok_or("no WebRTC runtime compiled in")?;
    let peer = PeerConnectionBuilder::new()
        // On a LAN both ends have host candidates; no STUN round trip to anybody's server.
        .with_configuration(RTCConfigurationBuilder::new().build())
        .with_media_engine(media_engine)
        .with_interceptor_registry(registry)
        .with_handler(Arc::new(Handler { events, ice }))
        .with_runtime(runtime)
        .with_udp_addrs(vec!["0.0.0.0:0".to_owned()])
        .build()
        .await?;
    Ok((peer, ice_rx))
}

/// The signalling half: list the producers, start a session with the robot, answer its offer,
/// and trade ICE candidates both ways for as long as the socket lasts. The shapes are
/// `remote-webrtc.md` §7's, the same ones the console speaks.
async fn signalling<P: PeerConnection>(
    ws: tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    pc: Arc<P>,
    mut ice_rx: IceRx,
) {
    let (mut tx, mut rx) = ws.split();
    let mut session_id: Option<String> = None;
    let mut pending_ice: Vec<RTCIceCandidateInit> = Vec::new();
    let send = |value: serde_json::Value| Message::Text(value.to_string().into());

    loop {
        tokio::select! {
            candidate = ice_rx.recv() => {
                let Some(candidate) = candidate else { continue };
                match &session_id {
                    Some(id) => {
                        let _ = tx.send(send(ice_message(id, &candidate))).await;
                    }
                    None => pending_ice.push(candidate),
                }
            }
            message = rx.next() => {
                let Some(Ok(Message::Text(text))) = message else {
                    if message.is_none() { return; }
                    continue;
                };
                let Ok(msg) = serde_json::from_str::<serde_json::Value>(&text) else { continue };
                match msg.get("type").and_then(|t| t.as_str()) {
                    Some("welcome") => {
                        let _ = tx.send(send(serde_json::json!({ "type": "list" }))).await;
                    }
                    Some("list") if session_id.is_none() => {
                        let producer = msg
                            .get("producers")
                            .and_then(|p| p.as_array())
                            .and_then(|p| p.first())
                            .and_then(|p| p.get("id"))
                            .and_then(|id| id.as_str());
                        match producer {
                            Some(id) => {
                                let _ = tx
                                    .send(send(serde_json::json!({ "type": "startSession", "peerId": id })))
                                    .await;
                            }
                            None => eprintln!(
                                "the robot is not producing — mediad registers when its pipeline \
                                 is playing; its journal says why it is not"
                            ),
                        }
                    }
                    Some("sessionStarted") => {
                        let id = msg.get("sessionId").and_then(|s| s.as_str()).map(str::to_owned);
                        if let Some(id) = &id {
                            for candidate in pending_ice.drain(..) {
                                let _ = tx.send(send(ice_message(id, &candidate))).await;
                            }
                        }
                        session_id = id;
                    }
                    Some("peer") => {
                        let Some(id) = session_id.clone() else { continue };
                        if let Some(sdp) = msg.get("sdp").and_then(|s| s.get("sdp")).and_then(|s| s.as_str()) {
                            match answer(&*pc, sdp).await {
                                Ok(answer) => {
                                    let _ = tx
                                        .send(send(serde_json::json!({
                                            "type": "peer",
                                            "sessionId": id,
                                            "sdp": { "type": "answer", "sdp": answer },
                                        })))
                                        .await;
                                }
                                Err(e) => eprintln!("could not answer the robot's offer: {e}"),
                            }
                        } else if let Some(ice) = msg.get("ice") {
                            let candidate = RTCIceCandidateInit {
                                candidate: ice
                                    .get("candidate")
                                    .and_then(|c| c.as_str())
                                    .unwrap_or_default()
                                    .to_owned(),
                                sdp_mline_index: ice
                                    .get("sdpMLineIndex")
                                    .and_then(|i| i.as_u64())
                                    .map(|i| i as u16),
                                ..Default::default()
                            };
                            if !candidate.candidate.is_empty() {
                                let _ = pc.add_ice_candidate(candidate).await;
                            }
                        }
                    }
                    Some("endSession") => return,
                    Some("error") => eprintln!("signalling error: {}", msg.get("details").unwrap_or(&msg)),
                    _ => {}
                }
            }
        }
    }
}

/// Answer the robot's offer, returning the answer's SDP.
async fn answer<P: PeerConnection>(pc: &P, offer: &str) -> Result<String, String> {
    let offer = RTCSessionDescription::offer(offer.to_owned()).map_err(|e| e.to_string())?;
    pc.set_remote_description(offer)
        .await
        .map_err(|e| e.to_string())?;
    // Decline every media section the robot offered. The camera is not wanted here and it is not
    // free: the robot sends ~3.5 Mbit/s of H.264 to every consumer that accepts it, on the same
    // transport as `control` — and `control` is reliable and ordered, so on a marginal link the
    // video's congestion is a stall in the commands behind it. Answered `inactive`, `webrtcsink`
    // sends this peer nothing but the datachannel.
    for transceiver in pc.get_transceivers().await {
        transceiver
            .set_direction(RTCRtpTransceiverDirection::Inactive)
            .await
            .map_err(|e| e.to_string())?;
    }
    let answer = pc.create_answer(None).await.map_err(|e| e.to_string())?;
    let sdp = answer.sdp.clone();
    pc.set_local_description(answer)
        .await
        .map_err(|e| e.to_string())?;
    Ok(sdp)
}

/// A local ICE candidate as the signalling server carries it.
fn ice_message(session_id: &str, candidate: &RTCIceCandidateInit) -> serde_json::Value {
    serde_json::json!({
        "type": "peer",
        "sessionId": session_id,
        "ice": {
            "candidate": candidate.candidate,
            "sdpMLineIndex": candidate.sdp_mline_index.unwrap_or(0),
        },
    })
}

/// Ask the robot for its button bindings and render them as a `[pad]` section.
async fn fetch_bindings(
    control: &Arc<dyn DataChannel>,
    from_robot: &mut mpsc::UnboundedReceiver<String>,
) -> Result<String, String> {
    let request = proto::Request::call(
        proto::Id::Text(BINDINGS_ID.to_owned()),
        &proto::Call::PadBindings,
    );
    let line = serde_json::to_string(&request).map_err(|e| e.to_string())?;
    control.send_text(&line).await.map_err(|e| e.to_string())?;

    tokio::time::timeout(BINDINGS_TIMEOUT, async {
        while let Some(message) = from_robot.recv().await {
            for line in message.lines() {
                let Ok(response) = serde_json::from_str::<proto::Response>(line) else {
                    continue;
                };
                if response.id != Some(proto::Id::Text(BINDINGS_ID.to_owned())) {
                    continue;
                }
                if let Some(error) = response.error {
                    return Err(error.message);
                }
                let result = response
                    .result_as::<proto::PadBindingsResult>()
                    .map_err(|e| e.to_string())?;
                return Ok(pad_section(&result));
            }
        }
        Err("the control channel closed".to_owned())
    })
    .await
    .map_err(|_| "no answer".to_owned())?
}

/// The bindings as the `[pad]` section `padd` reads.
fn pad_section(result: &proto::PadBindingsResult) -> String {
    let mut toml = String::from("[pad]\n");
    for binding in &result.bindings {
        toml.push_str(&format!(
            "{} = {}\n",
            binding.button,
            serde_json::to_string(&binding.skill).unwrap_or_default()
        ));
    }
    toml
}

/// `padd` next to this binary — where `cargo install` puts both — else whatever `PATH` finds.
fn find_padd() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(|dir| dir.join("padd")))
        .filter(|candidate| candidate.is_file())
        .unwrap_or_else(|| PathBuf::from("padd"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers reach `padd`; notifications and refusals of notifications do not — one of those
    /// read as an answer would shift every later one by a line.
    #[test]
    fn only_replies_reach_padd() {
        assert!(is_reply(
            r#"{"jsonrpc":"2.0","id":7,"result":{"accepted":true}}"#
        ));
        assert!(is_reply(
            r#"{"jsonrpc":"2.0","id":"x","error":{"code":-1,"message":"no"}}"#
        ));
        assert!(!is_reply(
            r#"{"jsonrpc":"2.0","method":"media.video","params":{}}"#
        ));
        assert!(!is_reply(
            r#"{"jsonrpc":"2.0","id":null,"error":{"code":-32601,"message":"refused"}}"#
        ));
        assert!(!is_reply("not json"));
    }

    /// The bindings come out as a `[pad]` section that parses as the robot's config does, empty
    /// buttons included.
    #[test]
    fn the_bindings_become_a_pad_section() {
        let result = proto::PadBindingsResult {
            bindings: vec![
                proto::PadBinding {
                    button: "a".to_owned(),
                    skill: "sit_toggle".to_owned(),
                    overridden: false,
                    error: None,
                },
                proto::PadBinding {
                    button: "x".to_owned(),
                    skill: String::new(),
                    overridden: false,
                    error: None,
                },
            ],
        };
        let toml = pad_section(&result);
        let parsed: toml_check::Doc = toml_check::parse(&toml);
        assert_eq!(parsed.get("a"), Some("sit_toggle"));
        assert_eq!(parsed.get("x"), Some(""));
    }

    /// A tiny reader for the one shape `pad_section` writes, so this test needs no TOML crate.
    mod toml_check {
        pub struct Doc(Vec<(String, String)>);
        impl Doc {
            pub fn get(&self, key: &str) -> Option<&str> {
                self.0
                    .iter()
                    .find(|(k, _)| k == key)
                    .map(|(_, v)| v.as_str())
            }
        }
        pub fn parse(text: &str) -> Doc {
            let mut lines = text.lines();
            assert_eq!(lines.next(), Some("[pad]"));
            Doc(lines
                .map(|line| {
                    let (key, value) = line.split_once(" = ").expect("key = value");
                    (
                        key.to_owned(),
                        serde_json::from_str::<String>(value).expect("a string"),
                    )
                })
                .collect())
        }
    }

    #[test]
    fn ice_carries_the_session_and_a_line_index() {
        let candidate = RTCIceCandidateInit {
            candidate: "candidate:1 1 udp 2130706431 192.168.1.2 50000 typ host".to_owned(),
            sdp_mline_index: None,
            ..Default::default()
        };
        let message = ice_message("s1", &candidate);
        assert_eq!(message["sessionId"], "s1");
        assert_eq!(message["ice"]["sdpMLineIndex"], 0);
        assert_eq!(message["ice"]["candidate"], candidate.candidate);
    }
}
