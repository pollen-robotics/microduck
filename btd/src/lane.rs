//! The wifi lane: **Bluetooth's rules, over wifi.**
//!
//! A phone on the robot's network can reach it faster than the radio carries — a photo is tens of
//! 20-byte-MTU notifications over BLE and one burst over wifi — and from anywhere on that
//! network rather than within a few metres. Until this lane, the only wifi path was the video
//! connection, which comes with a video stream and with `mediad`'s permissions: steering, and
//! power to the joints, on the grounds that a peer holding the video is watching the robot
//! (`mediad/src/route.rs`). A phone that only wants to throw a switch or fetch a photo is neither
//! watching nor asking to steer, and should not be given either.
//!
//! So this lane is **a `btd` session that arrived by another wire**. The same [`session::dispatch`]
//! as the radio — the same [`crate::route`] table, the same upstream sockets, the same refusals,
//! word for word — and nothing about permissions written here. Adding a method to the BLE table
//! adds it here; refusing one refuses it here. There is no third table to drift.
//!
//! **Who may open it.** Nobody on the network reaches this socket directly. It is a unix socket in
//! `btd`'s own `RuntimeDirectory=`, mode 0600, so only processes running as `btd` can connect —
//! and the one that does is `duck-gateway`, which accepts a TCP connection on the wifi only after
//! the phone has proved the robot's setup code through the same Noise handshake as the media
//! gateway (`duck-auth`). The bytes that cross the network are Noise-encrypted; the plaintext
//! exists only on this socket. That is the same credential the secure BLE session checks, so this
//! is no wider than Bluetooth: it is Bluetooth's door, reached from further away by the same key.
//!
//! **Framing** is the one every unix-socket client already speaks: one JSON-RPC object per line,
//! `\n`-terminated, replies and notifications interleaved, sorted out by `id`. No chunking, no
//! MTU, no reassembly — a stream transport needs none.
//!
//! **A lease, like the secure BLE session's.** A phone that goes to the background can leave a TCP
//! connection open for a long time with nothing behind it; [`IDLE`] without an inbound line closes
//! the lane, and only inbound lines renew it.

use std::path::Path;
use std::time::Duration;

use tokio::io::{AsyncBufReadExt, AsyncRead, AsyncWrite, AsyncWriteExt, BufReader};
use tokio::net::UnixListener;
use tokio::sync::mpsc;

use crate::link::QUEUE;
use crate::session;
use crate::upstream::{Pool, Sockets};
use duck_ble::framing::MAX_LINE;

/// How long a lane may sit without a request before it is closed. The secure BLE session's.
pub const IDLE: Duration = Duration::from_secs(60);

/// Listen on `path` and serve each connection as a lane, until the listener fails.
///
/// A stale socket left by a crashed `btd` is removed first; `RuntimeDirectory=` normally does
/// that for us, but a socket that outlived its daemon would otherwise make every start fail.
pub async fn serve(path: &Path, sockets: Sockets) -> std::io::Result<()> {
    let _ = std::fs::remove_file(path);
    let listener = UnixListener::bind(path)?;
    restrict(path)?;
    tracing::info!(path = %path.display(), "wifi lane listening (reached through duck-gateway)");
    loop {
        let (stream, _) = listener.accept().await?;
        let sockets = sockets.clone();
        tokio::spawn(async move {
            tracing::info!("wifi lane opened");
            run(stream, sockets).await;
            tracing::info!("wifi lane closed");
        });
    }
}

/// Owner only: the gateway runs as `btd`, and nothing else on the robot should get a
/// pre-authenticated door by connecting here.
#[cfg(unix)]
fn restrict(path: &Path) -> std::io::Result<()> {
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))
}

/// Serve one lane until its peer leaves, sends a line too long to be a request, or goes quiet.
pub async fn run<S>(stream: S, sockets: Sockets)
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let (read, mut write) = tokio::io::split(stream);
    let mut reader = BufReader::new(read);
    let (replies_tx, mut replies) = mpsc::channel::<String>(QUEUE);
    let config_socket = sockets.config.clone();
    let mut pool = Pool::new(sockets, replies_tx);
    // The gateway has already proved the setup code: the PIN gate is the secure BLE session's
    // `true`, for the same reason — a PIN would be a weaker proof of what is already proven.
    let mut authenticated = true;
    let mut unused_attempts = 0;
    let deadline = tokio::time::sleep(IDLE);
    tokio::pin!(deadline);
    let mut buffer = Vec::new();

    loop {
        tokio::select! {
            () = &mut deadline => {
                tracing::info!("wifi lane idle; closing");
                return;
            }
            read = next_line(&mut reader, &mut buffer) => {
                let line = match read {
                    Ok(Some(line)) => line,
                    Ok(None) | Err(_) => return,
                };
                if line.len() > MAX_LINE {
                    tracing::warn!(bytes = line.len(), "wifi lane line too long; closing");
                    return;
                }
                let Ok(line) = String::from_utf8(line) else { return };
                let line = line.trim_end_matches('\r').to_owned();
                if line.is_empty() {
                    continue;
                }
                deadline.as_mut().reset(tokio::time::Instant::now() + IDLE);
                let outcome = session::dispatch(
                    &mut pool, &config_socket, &line, &mut authenticated, &mut unused_attempts,
                )
                .await;
                if let Some(reply) = outcome.response
                    && write_line(&mut write, &reply).await.is_err()
                {
                    return;
                }
                if outcome.close {
                    return;
                }
            }
            line = replies.recv() => {
                let Some(line) = line else { return };
                if write_line(&mut write, &line).await.is_err() {
                    return;
                }
            }
        }
    }
}

async fn write_line<W: AsyncWrite + Unpin>(write: &mut W, line: &str) -> std::io::Result<()> {
    write
        .write_all(line.trim_end_matches('\n').as_bytes())
        .await?;
    write.write_all(b"\n").await?;
    write.flush().await
}

/// The next line, without its `\n` — or, once more than [`MAX_LINE`] bytes have arrived with no
/// newline, those bytes, so the caller sees a line too long to be a request and closes. Never
/// buffers more than that plus one read, whatever the peer sends. `None` at end of stream.
///
/// Cancel-safe for `select!`: what has been read so far lives in `pending`, not in this future.
async fn next_line<R: AsyncRead + Unpin>(
    reader: &mut BufReader<R>,
    pending: &mut Vec<u8>,
) -> std::io::Result<Option<Vec<u8>>> {
    loop {
        let available = reader.fill_buf().await?;
        if available.is_empty() {
            return Ok(None);
        }
        if let Some(at) = available.iter().position(|&b| b == b'\n') {
            pending.extend_from_slice(&available[..at]);
            reader.consume(at + 1);
            return Ok(Some(std::mem::take(pending)));
        }
        let n = available.len();
        pending.extend_from_slice(available);
        reader.consume(n);
        if pending.len() > MAX_LINE {
            return Ok(Some(std::mem::take(pending)));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_ipc_proto as proto;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
    use tokio::net::UnixListener;

    /// One daemon that answers every line with `reply`, and says what it saw.
    fn fake_daemon(path: &Path, reply: &'static str) -> mpsc::Receiver<String> {
        let listener = UnixListener::bind(path).unwrap();
        let (seen, seen_rx) = mpsc::channel(16);
        tokio::spawn(async move {
            while let Ok((stream, _)) = listener.accept().await {
                let seen = seen.clone();
                tokio::spawn(async move {
                    let (read, mut write) = stream.into_split();
                    let mut lines = BufReader::new(read).lines();
                    while let Ok(Some(line)) = lines.next_line().await {
                        let _ = seen.send(line).await;
                        let _ = write.write_all(format!("{reply}\n").as_bytes()).await;
                    }
                });
            }
        });
        seen_rx
    }

    fn sockets(dir: &Path) -> Sockets {
        Sockets {
            config: dir.join("configd.sock"),
            robot: dir.join("robotd.sock"),
            updater: dir.join("updaterd.sock"),
            nfc: dir.join("nfcd.sock"),
        }
    }

    async fn reply(
        lines: &mut tokio::io::Lines<BufReader<tokio::io::ReadHalf<tokio::io::DuplexStream>>>,
    ) -> String {
        tokio::time::timeout(Duration::from_secs(2), lines.next_line())
            .await
            .expect("no reply")
            .unwrap()
            .expect("lane closed")
    }

    /// **A request goes where it would over Bluetooth, verbatim, and its answer comes back as one
    /// line** — the round trip, over real unix sockets.
    #[tokio::test]
    async fn a_permitted_call_reaches_its_daemon_and_the_answer_comes_back() {
        let dir = tempfile::tempdir().unwrap();
        let sockets = sockets(dir.path());
        let mut seen = fake_daemon(
            &sockets.robot,
            r#"{"jsonrpc":"2.0","id":7,"result":{"accepted":true}}"#,
        );
        let (phone, robot) = tokio::io::duplex(64 * 1024);
        tokio::spawn(run(robot, sockets));
        let (read, mut write) = tokio::io::split(phone);
        let mut lines = BufReader::new(read).lines();
        let request = r#"{"jsonrpc":"2.0","id":7,"method":"robot.sound","params":{"tag":"chirp"}}"#;
        write
            .write_all(format!("{request}\n").as_bytes())
            .await
            .unwrap();
        assert_eq!(seen.recv().await.unwrap(), request);
        assert!(reply(&mut lines).await.contains(r#""accepted":true"#));
    }

    /// **Steering is refused, as over Bluetooth** — answered here, never forwarded, with the
    /// radio's own sentence.
    #[tokio::test]
    async fn what_bluetooth_refuses_the_lane_refuses() {
        let dir = tempfile::tempdir().unwrap();
        let sockets = sockets(dir.path());
        let mut seen = fake_daemon(&sockets.robot, r#"{"jsonrpc":"2.0","id":1,"result":{}}"#);
        let (phone, robot) = tokio::io::duplex(64 * 1024);
        tokio::spawn(run(robot, sockets));
        let (read, mut write) = tokio::io::split(phone);
        let mut lines = BufReader::new(read).lines();
        write
            .write_all(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"robot.move\",\"params\":{\"vx\":0.3}}\n")
            .await
            .unwrap();
        let answer: serde_json::Value = serde_json::from_str(&reply(&mut lines).await).unwrap();
        assert_eq!(answer["error"]["code"], proto::code::PERMISSION_DENIED);
        assert!(
            seen.try_recv().is_err(),
            "a refused call must not reach robotd"
        );
    }

    /// **The table is btd's, so the two cannot disagree**: every call the protocol has is
    /// refused over the lane exactly when `route` refuses it over the radio.
    #[tokio::test]
    async fn the_lane_refuses_exactly_what_the_radio_refuses() {
        let dir = tempfile::tempdir().unwrap();
        let sockets = sockets(dir.path());
        for call in duck_ipc_proto::test_support::every_call() {
            let (phone, robot) = tokio::io::duplex(64 * 1024);
            tokio::spawn(run(robot, sockets.clone()));
            let (read, mut write) = tokio::io::split(phone);
            let mut lines = BufReader::new(read).lines();
            let line =
                serde_json::to_string(&proto::Request::call(proto::Id::Number(1), &call)).unwrap();
            write
                .write_all(format!("{line}\n").as_bytes())
                .await
                .unwrap();
            let answer: serde_json::Value = serde_json::from_str(&reply(&mut lines).await).unwrap();
            let refused = answer["error"]["code"] == proto::code::PERMISSION_DENIED;
            assert_eq!(
                refused,
                crate::route::route_for(&call) == crate::route::Route::Refused,
                "{}: {answer}",
                call.method()
            );
        }
    }

    /// A line longer than a request may be ends the lane rather than being buffered.
    #[tokio::test]
    async fn an_endless_line_closes_the_lane() {
        let dir = tempfile::tempdir().unwrap();
        let (phone, robot) = tokio::io::duplex(64 * 1024);
        let task = tokio::spawn(run(robot, sockets(dir.path())));
        let (_read, mut write) = tokio::io::split(phone);
        let flood = vec![b'x'; MAX_LINE + 64];
        let _ = write.write_all(&flood).await;
        tokio::time::timeout(Duration::from_secs(2), task)
            .await
            .unwrap()
            .unwrap();
    }

    /// A phone that went to the background does not hold a lane open forever.
    #[tokio::test(start_paused = true)]
    async fn a_quiet_lane_closes() {
        let dir = tempfile::tempdir().unwrap();
        let (_phone, robot) = tokio::io::duplex(1024);
        let task = tokio::spawn(run(robot, sockets(dir.path())));
        // Paused time advances by itself once every task is waiting: the lane's lease is the
        // only timer, so it is what fires — well inside this test's own bound.
        tokio::time::timeout(IDLE + Duration::from_secs(5), task)
            .await
            .unwrap()
            .unwrap();
    }

    /// Only the owner can connect: the socket is the inside of the gateway's door.
    #[tokio::test]
    async fn the_socket_is_the_owners_alone() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("lane.sock");
        let sockets = sockets(dir.path());
        let server = tokio::spawn({
            let path = path.clone();
            async move { serve(&path, sockets).await }
        });
        for _ in 0..50 {
            if path.exists() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600);
        server.abort();
    }
}
