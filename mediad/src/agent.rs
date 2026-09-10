//! A WebSocket for programs, next to the WebRTC console for people.
//!
//! `architecture.md` §5.3 argues that an LLM-driven controller should not be pushed through
//! WebRTC: an agent does not want a 30 fps H.264 track to decode, it wants a frame every second or
//! two and a state blob, and making it do ICE and DTLS and a decode pipeline first is a poor
//! trade. What it wants is "open a socket, poll a frame, send intents". That is this.
//!
//! **It is the same API, not a second one.** A line arriving here goes through
//! [`crate::session::run`] — the same dispatcher the datachannel uses, against the same
//! [`crate::route`] table — so what an agent may call is what a console peer may call, decided in
//! one place. Adding a method here is not a thing anybody can do; a method is routed once, for
//! every transport, or it is refused everywhere.
//!
//! That is also why this is small. `session::run` takes a pair of `String` channels and knows
//! nothing about what carries them, so the whole of the transport is pumping text frames into one
//! and out of the other.

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::{mpsc, watch};

use crate::session::{self, Media};
use crate::upstream::{Pool, Sockets};

/// Lines buffered in each direction before a slow peer starts costing something.
///
/// An agent's own calls are request-response and never queue, so this is really the size of a
/// subscription's backlog — `robot.state` at the control rate is the fast one. Deep enough to ride
/// out a garbage collection at the other end, shallow enough that a peer which has stopped reading
/// is noticed rather than buffered for ever.
const QUEUE: usize = 256;

/// What an agent connection needs to serve: where the daemons are, and the media that may not
/// exist yet.
///
/// The media arrives late by construction — the console is served before the pipeline is built,
/// because a page that cannot bind must not cost the video — so it comes as a watch rather than a
/// value. A connection reads it once, at accept, which is also when a `None` is honest: no
/// pipeline, no frames to hand out.
#[derive(Clone)]
pub struct State {
    pub sockets: Sockets,
    pub video: watch::Receiver<Option<Media>>,
}

/// The upgrade handler, for the router to hang on a path.
pub async fn upgrade(ws: WebSocketUpgrade, state: State) -> Response {
    ws.on_upgrade(move |socket| serve(socket, state))
}

/// One agent, until it goes away.
async fn serve(socket: WebSocket, state: State) {
    let (mut sink, mut stream) = socket.split();
    let (inbound, inbound_rx) = mpsc::channel::<String>(QUEUE);
    let (outbound, mut outbound_rx) = mpsc::channel::<String>(QUEUE);

    // Its own connections to the daemons, per peer rather than shared, for the reason the
    // datachannel's are: one peer's minutes-long update must not silence another's telemetry.
    let pool = Pool::new(state.sockets.clone(), outbound.clone());
    let media = state.video.borrow().clone();
    if media.is_none() {
        // Worth saying once. Every control call still works; `media.*` is what will refuse, and
        // "the camera answers nothing" is otherwise a silent property of having connected early.
        tracing::debug!("an agent connected before the pipeline; frames are not available yet");
    }

    let writer = tokio::spawn(async move {
        while let Some(line) = outbound_rx.recv().await {
            if sink.send(Message::Text(line.into())).await.is_err() {
                break;
            }
        }
    });

    let session = tokio::spawn(session::run(inbound_rx, outbound, pool, media));

    while let Some(message) = stream.next().await {
        match message {
            // One JSON-RPC object per message, which is what a WebSocket already frames for us —
            // so unlike every other transport here there is no newline to reassemble.
            Ok(Message::Text(text)) => {
                if inbound.send(text.to_string()).await.is_err() {
                    break;
                }
            }
            Ok(Message::Close(_)) => break,
            // Ping and pong are the runtime's; binary is not something this speaks.
            Ok(_) => {}
            Err(e) => {
                tracing::debug!(error = %e, "an agent's socket ended");
                break;
            }
        }
    }

    // Dropping `inbound` ends `session::run`, which drops the last `outbound` and ends the writer.
    drop(inbound);
    let _ = session.await;
    writer.abort();
    tracing::debug!("agent disconnected");
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_ipc_proto as proto;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    /// A daemon that reads one line and answers with a canned one, so a call that is *routed* can
    /// be told from one that is merely accepted.
    fn fake_daemon(path: std::path::PathBuf, reply: String) {
        let listener = std::os::unix::net::UnixListener::bind(&path).unwrap();
        listener.set_nonblocking(true).unwrap();
        let listener = tokio::net::UnixListener::from_std(listener).unwrap();
        tokio::spawn(async move {
            while let Ok((stream, _)) = listener.accept().await {
                let reply = reply.clone();
                tokio::spawn(async move {
                    let (read, mut write) = stream.into_split();
                    let mut lines = BufReader::new(read).lines();
                    while let Ok(Some(_)) = lines.next_line().await {
                        let _ = write.write_all(format!("{reply}\n").as_bytes()).await;
                        let _ = write.flush().await;
                    }
                });
            }
        });
    }

    /// Serve the real router on an ephemeral port and open a real WebSocket to it.
    async fn connect(
        dir: &std::path::Path,
    ) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>
    {
        let sockets = Sockets {
            robot: dir.join("robotd.sock"),
            updater: dir.join("updaterd.sock"),
            config: dir.join("configd.sock"),
            ..Sockets::default()
        };

        let (_video_tx, video) = watch::channel(None);
        let state = State { sockets, video };

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let router = axum::Router::new().route(
            "/agent",
            axum::routing::get(move |ws| upgrade(ws, state.clone())),
        );
        tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let (socket, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/agent"))
            .await
            .expect("the agent socket accepts a websocket");
        socket
    }

    async fn ask(
        socket: &mut tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
        call: &proto::Call,
    ) -> serde_json::Value {
        let request = proto::Request::call(proto::Id::Number(1), call);
        socket
            .send(tokio_tungstenite::tungstenite::Message::Text(
                serde_json::to_string(&request).unwrap().into(),
            ))
            .await
            .unwrap();
        let answer = socket.next().await.unwrap().unwrap();
        serde_json::from_str(answer.to_text().unwrap()).unwrap()
    }

    /// **The point of the whole module: a program gets the surface a person gets.**
    ///
    /// `hello` is answered, a call `route` permits reaches the daemon that owns it, and one it
    /// refuses is refused here too — all through `session::run`, which is what makes "same API
    /// behind all of them" true rather than a thing to keep in step by hand.
    #[tokio::test]
    async fn an_agent_gets_the_same_surface_a_console_peer_gets() {
        let dir = tempfile::tempdir().unwrap();
        fake_daemon(
            dir.path().join("robotd.sock"),
            r#"{"jsonrpc":"2.0","id":1,"result":{"healthy":true}}"#.to_owned(),
        );
        // `hello` belongs to updaterd, like it does on every other transport.
        fake_daemon(
            dir.path().join("updaterd.sock"),
            format!(
                r#"{{"jsonrpc":"2.0","id":1,"result":{{"api_version":{}}}}}"#,
                proto::API_VERSION
            ),
        );
        let mut socket = connect(dir.path()).await;

        let hello = ask(
            &mut socket,
            &proto::Call::Hello(proto::HelloParams {
                api_version: proto::API_VERSION,
            }),
        )
        .await;
        assert_eq!(
            hello["result"]["api_version"],
            proto::API_VERSION,
            "a version handshake must work before anything else: {hello}"
        );

        let health = ask(&mut socket, &proto::Call::RobotHealth).await;
        assert_eq!(
            health["result"]["healthy"], true,
            "a permitted call must reach robotd: {health}"
        );

        // Refused over WebRTC, so refused here: the PIN authorises a phone, and a transport that
        // hands it out is a transport that hands out the robot.
        let pin = ask(&mut socket, &proto::Call::SystemPairingPin).await;
        assert_eq!(
            pin["error"]["code"],
            proto::code::METHOD_NOT_FOUND,
            "the agent socket must not widen what a transport may call: {pin}"
        );
    }
}
