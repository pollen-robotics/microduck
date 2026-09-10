//! One connected central, from `hello` to disconnect.
//!
//! This is the whole of `btd`'s behaviour, and it holds no state about the robot — only about
//! the conversation: a reassembly buffer and whichever upstream sockets this session has had
//! reason to open.
//!
//! A request is forwarded **verbatim**. `btd` parses each line only far enough to answer two
//! questions — is this method allowed here, and which socket owns it — and then passes the
//! original bytes on. It never rewrites `id`, never re-serialises params, and never invents a
//! result. That is what keeps it a transport rather than a second implementation of the API,
//! and it is why adding a protocol method costs one line in [`crate::route`] and nothing here.

use duck_ipc_proto as proto;
use tokio::sync::mpsc;

use crate::framing::{self, Reassembler};
use crate::link::{Link, QUEUE};
use crate::pairing;
use crate::route::{self, Route};
use crate::upstream::{Pool, Sockets};

/// How many wrong PINs a session may offer before it is closed.
///
/// A six-digit PIN is a million guesses, and the link is encrypted but not authenticated, so
/// rationing attempts is the only thing standing between a peer in radio range and brute force.
/// Three, then the session ends: reconnecting costs a full BLE connect and bond, which turns an
/// afternoon of guessing into something far longer while staying invisible to a legitimate client
/// that mistypes twice.
const PIN_ATTEMPTS: u32 = 3;

/// Serve one central until it disconnects or breaks framing.
pub async fn run(mut link: Link, sockets: Sockets) {
    let peer = link.peer.clone();
    tracing::info!(peer = %peer, mtu = link.mtu(), "session opened; mtu is the floor \
     until the first write reports the negotiated one");

    let (replies_tx, mut replies) = mpsc::channel::<String>(QUEUE);
    let config_socket = sockets.config.clone();
    let mut pool = Pool::new(sockets, replies_tx);
    let mut inbound = Reassembler::new();

    // Nothing but `hello` and `system.authenticate` is served until the client proves the PIN.
    // See `crate::pairing` for why this is here rather than in the bond.
    let mut authenticated = false;
    let mut attempts_left = PIN_ATTEMPTS;

    loop {
        tokio::select! {
            // Bytes from the radio.
            chunk = link.inbound.recv() => {
                let Some(chunk) = chunk else { break };

                let lines = match inbound.push(&chunk) {
                    Ok(lines) => lines,
                    Err(e) => {
                        // Framing failures end the session rather than being answered. There
                        // is no id to answer *to* — we never saw a complete request — and a
                        // peer that cannot frame will not be helped by a JSON error it also
                        // cannot parse.
                        tracing::warn!(peer = %peer, error = ?e, "framing failed; closing session");
                        break;
                    }
                };

                for line in lines {
                    let outcome = dispatch(
                        &mut pool,
                        &config_socket,
                        &line,
                        &mut authenticated,
                        &mut attempts_left,
                    )
                    .await;

                    if let Some(response) = outcome.response
                        && send_line(&link, &response).await.is_err()
                    {
                        return;
                    }
                    if outcome.close {
                        tracing::warn!(peer = %peer, "closing the session: too many bad PINs");
                        return;
                    }
                }
            }

            // A reply or a notification from a service.
            line = replies.recv() => {
                // The channel is held by `pool`, which lives as long as this loop, so `None`
                // is unreachable — but treat it as end-of-session rather than panicking.
                let Some(line) = line else { break };
                if send_line(&link, &line).await.is_err() {
                    return;
                }
            }
        }
    }

    if inbound.pending() > 0 {
        // Worth a line: it distinguishes "client finished and left" from "client vanished
        // mid-message", which is the difference between a normal disconnect and a bug.
        tracing::debug!(peer = %peer, pending = inbound.pending(), "session ended mid-line");
    }
    tracing::info!(peer = %peer, "session closed");
}

/// What handling one line produced.
struct Outcome {
    /// A response `btd` must send itself. `None` means an upstream will answer — the ordinary path.
    response: Option<String>,
    /// End the session after sending. Only ever set by exhausting the PIN attempts.
    close: bool,
}

impl Outcome {
    fn nothing() -> Self {
        Self {
            response: None,
            close: false,
        }
    }
    fn reply(response: String) -> Self {
        Self {
            response: Some(response),
            close: false,
        }
    }
}

/// Handle one complete line.
async fn dispatch(
    pool: &mut Pool,
    config_socket: &std::path::Path,
    line: &str,
    authenticated: &mut bool,
    attempts_left: &mut u32,
) -> Outcome {
    let request: proto::Request = match serde_json::from_str(line) {
        Ok(request) => request,
        Err(e) => {
            // No id is recoverable from an unparseable line, so the response carries `null` —
            // which the spec requires and every client already handles.
            return Outcome::reply(encode(&proto::Response::err(
                None,
                proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
            )));
        }
    };

    // A notification — no id — expects no reply, so a refused one is dropped silently. It is
    // still not forwarded: the allowlist is not advisory.
    let id = request.id.clone();

    let call = match request.as_call() {
        Ok(call) => call,
        Err(e) => {
            return match id.map(|id| encode(&proto::Response::err(Some(id), e))) {
                Some(r) => Outcome::reply(r),
                None => Outcome::nothing(),
            };
        }
    };

    // The PIN gate. `hello` is allowed through because it only reports versions — the same
    // information the GATT read already gives an unauthenticated client — and refusing it would
    // leave a mismatched client unable to learn why nothing works.
    if !*authenticated
        && !matches!(
            call,
            proto::Call::SystemAuthenticate(_) | proto::Call::Hello(_)
        )
    {
        tracing::info!(method = call.method(), "refused: not authenticated");
        let error = proto::Error::new(
            proto::code::PERMISSION_DENIED,
            format!(
                "{} needs authentication first: send system.authenticate with the robot's PIN \
                 (`robotctl system pin` on the robot)",
                call.method()
            ),
        );
        return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
            Some(r) => Outcome::reply(r),
            None => Outcome::nothing(),
        };
    }

    let (upstream, lane) = match route::route_for(&call) {
        Route::To(upstream, lane) => (upstream, lane),
        Route::Local => {
            let proto::Call::SystemAuthenticate(params) = &call else {
                // `route_for` returns `Local` for exactly one variant; anything else here is a
                // routing table that grew a local method without teaching this function about it.
                tracing::error!(method = call.method(), "routed locally with no handler");
                let error = proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "routed locally with no handler",
                );
                return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
                    Some(r) => Outcome::reply(r),
                    None => Outcome::nothing(),
                };
            };
            return authenticate(config_socket, params, authenticated, attempts_left, id).await;
        }
        Route::Refused => {
            tracing::info!(method = call.method(), "refused over BLE");
            return match id.map(|id| encode(&proto::Response::err(Some(id), route::refusal(&call))))
            {
                Some(r) => Outcome::reply(r),
                None => Outcome::nothing(),
            };
        }
    };

    if let Err(e) = pool.send(upstream, lane, line).await {
        tracing::warn!(method = call.method(), upstream = ?upstream, error = %e, "upstream unreachable");
        // Naming the service is what makes this diagnosable from a phone screenshot: "robotd
        // is not answering" is a different problem from "the robot refused".
        let error = proto::Error::new(
            proto::code::INTERNAL_ERROR,
            format!("{upstream:?} is not answering: {e}"),
        );
        return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
            Some(r) => Outcome::reply(r),
            None => Outcome::nothing(),
        };
    }
    Outcome::nothing()
}

/// Check the PIN, and ration the attempts.
///
/// Fetched from `configd` per attempt rather than cached, so `robotctl system set-pin` takes effect
/// on the next try rather than the next reboot — and so a `configd` that cannot answer means the
/// session is refused rather than admitted.
///
/// The comparison is on the string, not a number: `000042` and `42` are different PINs, and a
/// numeric parse would make them the same.
async fn authenticate(
    config_socket: &std::path::Path,
    params: &proto::AuthenticateParams,
    authenticated: &mut bool,
    attempts_left: &mut u32,
    id: Option<proto::Id>,
) -> Outcome {
    let expected = match pairing::pin(config_socket).await {
        Ok(expected) => expected,
        Err(e) => {
            tracing::warn!(error = %e, "cannot read the PIN; refusing to authenticate");
            let error = proto::Error::new(
                proto::code::INTERNAL_ERROR,
                "cannot check the PIN: configd is not answering",
            );
            return match id.map(|id| encode(&proto::Response::err(Some(id), error))) {
                Some(r) => Outcome::reply(r),
                None => Outcome::nothing(),
            };
        }
    };

    // Constant-time-ish: compare whole strings of equal length rather than returning early on the
    // first differing digit. Over BLE the timing signal is buried in milliseconds of radio, so this
    // is hygiene rather than a defence — but it costs nothing.
    let ok = expected.pin.len() == params.pin.len()
        && expected
            .pin
            .bytes()
            .zip(params.pin.bytes())
            .fold(0u8, |acc, (a, b)| acc | (a ^ b))
            == 0;

    if ok {
        *authenticated = true;
        tracing::info!(default_pin = expected.is_default, "authenticated");
        if expected.is_default {
            // Worth saying every time: a factory PIN authenticates anyone who read this repository.
            tracing::warn!(
                "authenticated with the FACTORY PIN, which is public. Set a per-robot one: \
                 robotctl system set-pin <6 digits>"
            );
        }
        let result = proto::AuthenticateResult {
            authenticated: true,
            attempts_remaining: PIN_ATTEMPTS,
        };
        return match id.map(|id| encode(&proto::Response::ok(Some(id), &result))) {
            Some(r) => Outcome::reply(r),
            None => Outcome::nothing(),
        };
    }

    *attempts_left = attempts_left.saturating_sub(1);
    tracing::warn!(attempts_left = *attempts_left, "wrong PIN");

    let result = proto::AuthenticateResult {
        authenticated: false,
        attempts_remaining: *attempts_left,
    };
    let response = id.map(|id| encode(&proto::Response::ok(Some(id), &result)));
    Outcome {
        response,
        close: *attempts_left == 0,
    }
}

/// Chunk one line out to the central.
async fn send_line(link: &Link, line: &str) -> Result<(), ()> {
    // Read once here, so one line is chunked one way even if a write reports a new MTU
    // meanwhile. See `Link::mtu`.
    for chunk in framing::chunks(line, link.mtu()) {
        if link.outbound.send(chunk).await.is_err() {
            // The backend dropped its half: the central is gone.
            return Err(());
        }
    }
    Ok(())
}

fn encode(response: &proto::Response) -> String {
    // A Response is plain strings, ints and enums; this cannot fail. If it somehow did,
    // sending nothing would hang the client, so send something it can parse as an error.
    serde_json::to_string(response).unwrap_or_else(|_| {
        r#"{"jsonrpc":"2.0","id":null,"error":{"code":-32603,"message":"internal error"}}"#
            .to_owned()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
    use tokio::net::UnixListener;
    use tokio::sync::mpsc::{Receiver, Sender};

    /// A stand-in for one daemon: accepts a connection, records the lines it receives, and
    /// replies with whatever the test queued.
    ///
    /// A real unix socket rather than a mock, because the framing between `btd` and a daemon is
    /// part of what is under test — the same reason `robotd`'s own tests speak over a socket.
    struct FakeDaemon {
        path: PathBuf,
        seen: Sender<String>,
        replies: Vec<String>,
    }

    impl FakeDaemon {
        fn spawn(
            dir: &std::path::Path,
            name: &str,
            replies: Vec<String>,
        ) -> (PathBuf, Receiver<String>) {
            let path = dir.join(name);
            let (seen, seen_rx) = mpsc::channel(16);
            let daemon = FakeDaemon {
                path: path.clone(),
                seen,
                replies,
            };

            let listener = UnixListener::bind(&daemon.path).expect("bind");
            tokio::spawn(async move {
                while let Ok((stream, _)) = listener.accept().await {
                    let seen = daemon.seen.clone();
                    let replies = daemon.replies.clone();
                    tokio::spawn(async move {
                        let (read, mut write) = stream.into_split();
                        let mut lines = BufReader::new(read).lines();
                        while let Ok(Some(line)) = lines.next_line().await {
                            let _ = seen.send(line).await;
                            for reply in &replies {
                                let _ = write.write_all(format!("{reply}\n").as_bytes()).await;
                            }
                            let _ = write.flush().await;
                        }
                    });
                }
            });
            (path, seen_rx)
        }
    }

    /// Collect notified chunks and reassemble them the way a client would.
    async fn read_reply(from_robot: &mut Receiver<Vec<u8>>) -> String {
        let mut r = Reassembler::new();
        loop {
            let chunk = tokio::time::timeout(std::time::Duration::from_secs(2), from_robot.recv())
                .await
                .expect("client saw no reply")
                .expect("link closed");
            if let Some(line) = r.push(&chunk).expect("framing").into_iter().next() {
                return line;
            }
        }
    }

    /// Like [`read_reply`], and also says how the line was cut up on the way out.
    async fn read_reply_in_chunks(from_robot: &mut Receiver<Vec<u8>>) -> (String, Vec<usize>) {
        let mut r = Reassembler::new();
        let mut sizes = Vec::new();
        loop {
            let chunk = tokio::time::timeout(std::time::Duration::from_secs(2), from_robot.recv())
                .await
                .expect("client saw no reply")
                .expect("link closed");
            sizes.push(chunk.len());
            if let Some(line) = r.push(&chunk).expect("framing").into_iter().next() {
                return (line, sizes);
            }
        }
    }

    /// The reply a fake `configd` gives to `system.pairingPin`.
    fn pin_reply(pin: &str) -> String {
        serde_json::to_string(&proto::Response::ok(
            Some(proto::Id::Number(1)),
            &proto::PairingPinResult {
                pin: pin.to_owned(),
                is_default: false,
            },
        ))
        .unwrap()
    }

    /// Do what a real client must now do first: prove the PIN.
    ///
    /// Every test goes through this, which means every test also exercises the gate — a session
    /// that stopped authenticating would fail all of them rather than silently serve everything.
    async fn authenticate(to_robot: &Sender<Vec<u8>>, from_robot: &mut Receiver<Vec<u8>>) {
        let request =
            r#"{"jsonrpc":"2.0","id":99,"method":"system.authenticate","params":{"pin":"424242"}}"#;
        to_robot
            .send(format!("{request}\n").into_bytes())
            .await
            .unwrap();
        let reply = read_reply(from_robot).await;
        assert!(
            reply.contains(r#""authenticated":true"#),
            "the handshake failed: {reply}"
        );
    }

    fn sockets(dir: &std::path::Path, updater: &str, robot: &str) -> Sockets {
        Sockets {
            updater: dir.join(updater),
            robot: dir.join(robot),
            // Not exercised by these tests; `configd` gets its own once it has a fake.
            config: dir.join("configd.sock"),
        }
    }

    /// The ordinary path: an allowed call reaches the right daemon **byte for byte**, and its
    /// reply comes back. Verbatim forwarding is the property that keeps btd a transport.
    #[tokio::test]
    async fn an_allowed_call_is_forwarded_verbatim_and_answered() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock",
            vec![r#"{"jsonrpc":"2.0","id":1,"result":{"api_version":2,"daemon_version":"0.1.4","revision":null}}"#.into()]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        let request = r#"{"jsonrpc":"2.0","id":1,"method":"hello","params":{"api_version":2}}"#;
        to_robot.send(request.as_bytes().to_vec()).await.unwrap();
        to_robot.send(b"\n".to_vec()).await.unwrap();

        assert_eq!(
            seen.recv().await.unwrap(),
            request,
            "not forwarded byte for byte"
        );
        assert!(
            read_reply(&mut from_robot)
                .await
                .contains(r#""api_version":2"#)
        );
    }

    /// A refused call must never touch the upstream. Answering correctly is not enough — the
    /// point of the allowlist is that the daemon never sees it.
    #[tokio::test]
    async fn a_refused_call_never_reaches_the_daemon() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        to_robot.send(
            format!("{}\n", r#"{"jsonrpc":"2.0","id":9,"method":"update.resetToGolden","params":{"component":"daemon"}}"#)
                .into_bytes(),
        ).await.unwrap();

        let reply = read_reply(&mut from_robot).await;
        assert!(
            reply.contains(&proto::code::PERMISSION_DENIED.to_string()),
            "{reply}"
        );

        // Nothing arrived at the daemon, and "nothing" needs a moment to be provable.
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        assert!(seen.try_recv().is_err(), "a refused call was forwarded");
    }

    /// `robot.*` goes to `robotd` and not to `updaterd`. One table drives routing and
    /// permission, so a mistake here would send an update trigger to the control daemon.
    #[tokio::test]
    async fn robot_calls_go_to_robotd() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut updater_seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, mut robot_seen) = FakeDaemon::spawn(
            dir.path(),
            "robotd.sock",
            vec![r#"{"jsonrpc":"2.0","id":2,"result":{"healthy":true}}"#.into()],
        );

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        to_robot
            .send(b"{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"robot.health\"}\n".to_vec())
            .await
            .unwrap();

        assert!(robot_seen.recv().await.unwrap().contains("robot.health"));
        assert!(
            read_reply(&mut from_robot)
                .await
                .contains(r#""healthy":true"#)
        );
        assert!(
            updater_seen.try_recv().is_err(),
            "robotd's call went to updaterd"
        );
    }

    /// **A reply is sized for the MTU the link has learned, and a session starts knowing only
    /// the floor.**
    ///
    /// This is the mechanism behind `bluez`'s shared MTU cell, tested here because that file needs
    /// a radio and this one does not. What it pins is the ordering that makes the trick sound: a
    /// central subscribes before it writes, so a session opens at the 20-byte floor and learns the
    /// real payload from the first write — which is always `system.authenticate`, before any reply
    /// worth chunking exists.
    ///
    /// Sizing every reply for the floor was not merely slow. Ten times the notifications is ten
    /// times the queue depth in BlueZ, and past roughly 5 KiB the notification session was torn
    /// down mid-reply — a `system.logs` tail was the first reply big enough to find it.
    #[tokio::test]
    async fn a_reply_is_chunked_for_the_mtu_the_link_has_learned() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(
            dir.path(),
            "robotd.sock",
            vec![r#"{"jsonrpc":"2.0","id":2,"result":{"healthy":true}}"#.into()],
        );

        let mtu = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(20));
        let (link, to_robot, mut from_robot) = Link::pair_sharing_mtu(mtu.clone(), "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));

        // At the floor, so the authentication answer goes out in 20-byte pieces.
        to_robot
            .send(
                b"{\"jsonrpc\":\"2.0\",\"id\":0,\"method\":\"system.authenticate\",\"params\":{\"pin\":\"424242\"}}\n"
                    .to_vec(),
            )
            .await
            .unwrap();
        let (reply, sizes) = read_reply_in_chunks(&mut from_robot).await;
        assert!(reply.contains(r#""authenticated":true"#), "{reply}");
        assert!(
            sizes.len() > 1,
            "a floor-sized reply arrived whole: {sizes:?}"
        );
        assert!(
            sizes.iter().all(|&n| n <= 20),
            "a chunk exceeded the floor: {sizes:?}"
        );

        // What the write callback does on a real link, once BlueZ has reported the MTU.
        mtu.store(182, std::sync::atomic::Ordering::Relaxed);

        to_robot
            .send(b"{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"robot.health\"}\n".to_vec())
            .await
            .unwrap();
        let (reply, sizes) = read_reply_in_chunks(&mut from_robot).await;
        assert!(reply.contains(r#""healthy":true"#), "{reply}");
        assert_eq!(
            sizes.len(),
            1,
            "the reply was still cut up for the floor: {sizes:?}"
        );
    }

    /// A subscription is a stream of notifications on an open connection, and every one has to
    /// reach the central. This is the case that would break if replies were correlated to
    /// requests rather than forwarded as they arrive.
    #[tokio::test]
    async fn every_notification_in_a_stream_reaches_the_client() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let progress: Vec<String> = (0..3)
            .map(|i| format!(
                r#"{{"jsonrpc":"2.0","method":"update.progress","params":{{"component":"daemon","phase":"downloading","percent":{},"detail":null}}}}"#,
                i * 50
            ))
            .collect();
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", progress);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        to_robot
            .send(b"{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"update.subscribe\"}\n".to_vec())
            .await
            .unwrap();

        // A 23-byte MTU means each of these arrives in several chunks, so this also proves
        // reassembly survives back-to-back messages.
        for expected in [0, 50, 100] {
            let line = read_reply(&mut from_robot).await;
            assert!(line.contains(&format!(r#""percent":{expected}"#)), "{line}");
        }
    }

    /// Garbage gets an error with a null id, not a dropped session: a client that sent one bad
    /// line should be able to carry on.
    #[tokio::test]
    async fn an_unparseable_line_is_answered_and_the_session_survives() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock",
            vec![r#"{"jsonrpc":"2.0","id":1,"result":{"api_version":2,"daemon_version":null,"revision":null}}"#.into()]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        to_robot.send(b"not json at all\n".to_vec()).await.unwrap();
        let reply = read_reply(&mut from_robot).await;
        assert!(
            reply.contains(&proto::code::PARSE_ERROR.to_string()),
            "{reply}"
        );
        assert!(reply.contains(r#""id":null"#), "{reply}");

        // Still usable.
        to_robot.send(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\",\"params\":{\"api_version\":2}}\n".to_vec()).await.unwrap();
        assert!(
            read_reply(&mut from_robot)
                .await
                .contains(r#""api_version":2"#)
        );
    }

    /// A daemon that is not running must produce a diagnosable error naming it, rather than a
    /// hang. `robotd` is missing precisely when an update has just restarted it, which is when
    /// someone is most likely to be looking at a phone.
    #[tokio::test]
    async fn a_dead_daemon_is_reported_rather_than_hanging() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        // No robotd socket at all.

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "absent.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        to_robot
            .send(b"{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"robot.health\"}\n".to_vec())
            .await
            .unwrap();

        let reply = read_reply(&mut from_robot).await;
        assert!(reply.contains("Robot is not answering"), "{reply}");
    }

    /// A notification (no id) gets no reply even when refused — the spec says so, and a client
    /// waiting for one would wait forever.
    #[tokio::test]
    async fn a_refused_notification_is_answered_with_silence() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        to_robot.send(
            format!("{}\n", r#"{"jsonrpc":"2.0","method":"update.resetToGolden","params":{"component":"daemon"}}"#)
                .into_bytes(),
        ).await.unwrap();

        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        assert!(
            from_robot.try_recv().is_err(),
            "a notification was answered"
        );
        assert!(
            seen.try_recv().is_err(),
            "a refused notification was forwarded"
        );
    }

    /// The gate. An unauthenticated call is refused, and the message says what to do about it —
    /// this is the first thing a phone app author will hit.
    #[tokio::test]
    async fn nothing_is_served_before_the_pin() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut seen) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        // Deliberately NOT authenticated.

        to_robot
            .send(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"update.status\"}\n".to_vec())
            .await
            .unwrap();

        let reply = read_reply(&mut from_robot).await;
        assert!(
            reply.contains(&proto::code::PERMISSION_DENIED.to_string()),
            "{reply}"
        );
        assert!(
            reply.contains("system.authenticate"),
            "the refusal must say how to proceed: {reply}"
        );

        // And it never reached the daemon: the gate is not advisory.
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        assert!(
            seen.try_recv().is_err(),
            "an unauthenticated call was forwarded"
        );
    }

    /// `hello` is the one exception, because it reports only versions — the same thing the GATT
    /// read already tells an unauthenticated client — and refusing it would leave a mismatched
    /// client unable to learn why nothing works.
    #[tokio::test]
    async fn hello_is_allowed_before_the_pin() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, _) = FakeDaemon::spawn(
            dir.path(),
            "updaterd.sock",
            vec![r#"{"jsonrpc":"2.0","id":1,"result":{"api_version":4,"daemon_version":null,"revision":null}}"#.into()],
        );
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));

        to_robot
            .send(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\",\"params\":{\"api_version\":4}}\n".to_vec())
            .await
            .unwrap();
        assert!(read_reply(&mut from_robot).await.contains("api_version"));
    }

    /// A wrong PIN counts down and then closes the session. A six-digit PIN is a million guesses
    /// over a link that is encrypted but not authenticated, so rationing the attempts is the only
    /// thing making brute force expensive.
    #[tokio::test]
    async fn wrong_pins_are_rationed_and_then_the_session_closes() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));

        let wrong =
            r#"{"jsonrpc":"2.0","id":1,"method":"system.authenticate","params":{"pin":"000000"}}"#;
        for expected_left in [2, 1, 0] {
            to_robot
                .send(format!("{wrong}\n").into_bytes())
                .await
                .unwrap();
            let reply = read_reply(&mut from_robot).await;
            assert!(reply.contains(r#""authenticated":false"#), "{reply}");
            assert!(
                reply.contains(&format!(r#""attempts_remaining":{expected_left}"#)),
                "a client must be able to say how many tries are left: {reply}"
            );
        }

        // The third failure ends the session, so the link closes rather than accepting a fourth.
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        assert!(
            to_robot
                .send(format!("{wrong}\n").into_bytes())
                .await
                .is_err()
                || from_robot.try_recv().is_err(),
            "the session should have closed after exhausting the attempts"
        );
    }

    /// A PIN differing only in a leading zero must not authenticate. The stored form is a string
    /// precisely so that `042042` and `42042` are different secrets.
    #[tokio::test]
    async fn a_leading_zero_is_part_of_the_pin() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("042042")]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "updaterd.sock", vec![]);
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(185, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));

        let without =
            r#"{"jsonrpc":"2.0","id":1,"method":"system.authenticate","params":{"pin":"42042"}}"#;
        to_robot
            .send(format!("{without}\n").into_bytes())
            .await
            .unwrap();
        assert!(
            read_reply(&mut from_robot)
                .await
                .contains(r#""authenticated":false"#),
            "a PIN missing its leading zero must not authenticate"
        );
    }

    /// A stand-in with `updaterd`'s connection model, which is the whole reason lanes exist:
    /// **one request at a time per connection**, and a connection handed to `update.subscribe`
    /// never reads another line (`Server::stream_progress` owns it until the peer goes away).
    ///
    /// Every line it receives is reported as `<connection number> <line>`, so a test can assert
    /// which calls travelled together rather than only that they arrived.
    fn spawn_serial_updaterd(dir: &std::path::Path) -> (PathBuf, Receiver<String>) {
        let path = dir.join("updaterd.sock");
        let (seen, seen_rx) = mpsc::channel(16);
        let listener = UnixListener::bind(&path).expect("bind");

        tokio::spawn(async move {
            let mut connection = 0u32;
            while let Ok((stream, _)) = listener.accept().await {
                connection += 1;
                let seen = seen.clone();
                let n = connection;
                tokio::spawn(async move {
                    // Held for the task's life: dropping the stream would close the socket, and a
                    // client that learns of a swallowed request by disconnection has not
                    // reproduced the bug — it would reconnect and recover.
                    let (read, _write) = stream.into_split();
                    let mut lines = BufReader::new(read).lines();
                    while let Ok(Some(line)) = lines.next_line().await {
                        let subscribe = line.contains(proto::method::SUBSCRIBE);
                        let _ = seen.send(format!("{n} {line}")).await;
                        if subscribe {
                            std::future::pending::<()>().await;
                        }
                    }
                });
            }
        });
        (path, seen_rx)
    }

    async fn next_line(seen: &mut Receiver<String>, what: &str) -> String {
        tokio::time::timeout(std::time::Duration::from_secs(2), seen.recv())
            .await
            .unwrap_or_else(|_| panic!("updaterd never saw the {what}"))
            .expect("daemon gone")
    }

    /// Subscribing must not swallow the update that follows it.
    ///
    /// This is the failure the lanes were added for, and it is the worst one in the transport:
    /// with a single connection per service, the apply was written into a socket that
    /// `stream_progress` had stopped reading. No reply, no error, no update — an owner tapping
    /// "update" and a robot doing nothing at all. Nothing else in this file would have caught it,
    /// because every other test makes one call.
    #[tokio::test]
    async fn an_apply_still_reaches_updaterd_while_a_progress_stream_is_open() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut seen) = spawn_serial_updaterd(dir.path());
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        let subscribe = r#"{"jsonrpc":"2.0","id":1,"method":"update.subscribe","params":{}}"#;
        to_robot
            .send(format!("{subscribe}\n").into_bytes())
            .await
            .unwrap();
        let streaming = next_line(&mut seen, "subscribe").await;

        let apply = r#"{"jsonrpc":"2.0","id":2,"method":"update.apply","params":{"component":"daemon","target":"latest"}}"#;
        to_robot
            .send(format!("{apply}\n").into_bytes())
            .await
            .unwrap();
        let applying = next_line(&mut seen, "apply").await;

        assert!(
            applying.contains(apply),
            "not forwarded verbatim: {applying}"
        );
        // And on its own connection, which is *why* it arrived.
        let connection = |line: &str| line.split(' ').next().unwrap().to_owned();
        assert_ne!(connection(&streaming), connection(&applying));
    }

    /// A status poll during an update must not queue behind it.
    ///
    /// `updaterd` goes to some trouble to answer `update.status` while the engine is busy — a
    /// cached snapshot with the live phase patched in — and all of it is wasted if the request
    /// sits unread in a socket for the minutes an update takes. The assertion is that the two
    /// calls travel on different connections, because that is the property the daemon's effort
    /// depends on.
    #[tokio::test]
    async fn a_status_poll_does_not_travel_behind_an_apply() {
        let dir = tempdir();
        let (_, _) = FakeDaemon::spawn(dir.path(), "configd.sock", vec![pin_reply("424242")]);
        let (_, mut seen) = spawn_serial_updaterd(dir.path());
        let (_, _) = FakeDaemon::spawn(dir.path(), "robotd.sock", vec![]);

        let (link, to_robot, mut from_robot) = Link::pair(23, "AA:BB");
        tokio::spawn(run(
            link,
            sockets(dir.path(), "updaterd.sock", "robotd.sock"),
        ));
        authenticate(&to_robot, &mut from_robot).await;

        for request in [
            r#"{"jsonrpc":"2.0","id":1,"method":"update.apply","params":{"component":"daemon","target":"latest"}}"#,
            r#"{"jsonrpc":"2.0","id":2,"method":"update.status","params":{}}"#,
        ] {
            to_robot
                .send(format!("{request}\n").into_bytes())
                .await
                .unwrap();
        }

        let applying = next_line(&mut seen, "apply").await;
        let polling = next_line(&mut seen, "status poll").await;
        let connection = |line: &str| line.split(' ').next().unwrap().to_owned();
        assert_ne!(
            connection(&applying),
            connection(&polling),
            "the status poll shares the apply's queue"
        );
    }

    /// Sockets live in a temp directory, and unix socket paths are short by necessity — a
    /// long temp path would exceed `sun_path` and fail to bind for reasons unrelated to btd.
    fn tempdir() -> tempfile::TempDir {
        tempfile::Builder::new()
            .prefix("btd")
            .tempdir()
            .expect("tempdir")
    }
}
