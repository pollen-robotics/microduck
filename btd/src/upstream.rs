//! Connections to the services that actually own the answers.
//!
//! One socket per service, connected directly — with four services there is no case for a
//! broker, and a bus would be another component that can fail (`architecture.md` §2.2).
//!
//! Every operation here is timeout-bounded, without exception. Any peer may be dead, and a
//! closed or silent socket is a normal answer rather than an error worth retrying forever —
//! `robotd` in particular is the service most likely to be missing, since it is the one an
//! update restarts.

use std::collections::HashMap;
use std::io;
use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tokio::sync::mpsc;

use crate::route::{Lane, Upstream};

/// Long enough for a loaded board, short enough that a phone gets an answer rather than a
/// spinner. A unix socket connect either succeeds immediately or the daemon is not there.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Cap on a single write. A blocked write means the daemon has stopped reading, which is a
/// dead peer rather than a slow one.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

/// What to advertise the robot as.
///
/// The name belongs to `configd` (`architecture.md` §4.1) — an SDK should not have to go through
/// Bluetooth to set it — so `btd` asks rather than decides. Here rather than in `bluez` so the
/// crate's entry point has the same shape off-Linux, where there is no radio to advertise on.
#[derive(Debug, Clone)]
pub struct NameChoice {
    /// `--name`, which pins the advertised name and turns reconciliation off. Bench use: it exists
    /// so a board can be given a known name without touching its stored config.
    pub pinned: Option<String>,
    /// Used when `configd` cannot be reached. `btd` is on the recovery path and must answer when
    /// the rest of the robot does not (`systemd/btd.service`), so an unreachable `configd` costs
    /// the derived name and nothing more.
    pub fallback: String,
}

/// Where each service listens.
#[derive(Debug, Clone)]
pub struct Sockets {
    pub updater: PathBuf,
    pub robot: PathBuf,
    pub config: PathBuf,
}

impl Sockets {
    pub fn path(&self, upstream: Upstream) -> &Path {
        match upstream {
            Upstream::Updater => &self.updater,
            Upstream::Robot => &self.robot,
            Upstream::Config => &self.config,
        }
    }
}

/// Ask one service one question, on `btd`'s own behalf.
///
/// A one-shot connection rather than a [`Pool`] entry, because this is not forwarding: nothing
/// here belongs to a client's session. `btd` asks two questions of its own — the PIN during a
/// pairing exchange, and the robot's name to advertise — and both want a single answer now rather
/// than a merged stream of lines. With exactly one reply in flight there is nothing to correlate,
/// so the `id` is a constant.
///
/// Timeout-bounded like everything else in this module. The caller picks the timeout because the
/// deadlines differ by an order of magnitude: BlueZ holds a pairing exchange open while a phone
/// shows a spinner, whereas nothing is waiting on a name.
///
/// Returns the response with its error already turned into `Err`, so a caller only has to
/// deserialise the result it expected.
pub async fn ask(
    service: &str,
    socket: &Path,
    call: &proto::Call,
    timeout: Duration,
) -> Result<proto::Response, String> {
    tokio::time::timeout(timeout, ask_now(service, socket, call))
        .await
        .map_err(|_| format!("{service} did not answer in time"))?
}

async fn ask_now(
    service: &str,
    socket: &Path,
    call: &proto::Call,
) -> Result<proto::Response, String> {
    let stream = UnixStream::connect(socket)
        .await
        .map_err(|e| format!("cannot reach {service} at {}: {e}", socket.display()))?;
    let (read, mut write) = stream.into_split();
    let mut lines = BufReader::new(read).lines();

    let request = proto::Request::call(proto::Id::Number(1), call);
    let mut line = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
    line.push(b'\n');
    write.write_all(&line).await.map_err(|e| e.to_string())?;
    write.flush().await.map_err(|e| e.to_string())?;

    let reply = lines
        .next_line()
        .await
        .map_err(|e| e.to_string())?
        .ok_or_else(|| format!("{service} closed the connection without answering"))?;

    let response: proto::Response = serde_json::from_str(&reply).map_err(|e| e.to_string())?;
    if let Some(error) = response.error {
        return Err(format!("{service} refused: {error}"));
    }
    Ok(response)
}

/// The write half of a live connection. The read half lives in a spawned task.
struct Conn {
    write: tokio::net::unix::OwnedWriteHalf,
}

/// Connections opened so far in one BLE session, made on demand.
///
/// Lazy rather than eager because most sessions touch one service: a phone asking for the
/// version has no reason to make `robotd` accept a connection it will never use. And because
/// connecting eagerly would mean a dead `robotd` delayed or failed a session that did not need
/// it.
///
/// **Keyed on the lane as well as the service**, which is what keeps a minutes-long update from
/// silencing everything else the client asks. Every daemon here serves one connection one request
/// at a time, so calls that share a connection share a queue — see [`Lane`] for the two orderings
/// a single connection per service broke. At most four sockets per service per session, and in
/// practice two.
pub struct Pool {
    sockets: Sockets,
    conns: HashMap<(Upstream, Lane), Conn>,
    /// Every reply and notification from every upstream, merged. Merging is safe because
    /// JSON-RPC correlates by `id`, which is the client's business — `btd` forwards lines
    /// without reading them.
    replies: mpsc::Sender<String>,
}

impl Pool {
    pub fn new(sockets: Sockets, replies: mpsc::Sender<String>) -> Self {
        Self {
            sockets,
            conns: HashMap::new(),
            replies,
        }
    }

    /// Send one line to `upstream` on `lane`'s connection, connecting first if needed.
    ///
    /// A daemon that restarted between two requests is ordinary here: `robotd` is the one an
    /// update restarts, and `updaterd` restarts itself from a release's postinstall hook. The
    /// connection from before the restart is still in the pool, its reader has already seen the
    /// socket close, and the first write into it fails. Those bytes never left, so they are written
    /// again on a fresh connection rather than reported. Reporting them told a phone that a daemon
    /// listening on a fresh socket was not answering, once per lane, after every update. Once and
    /// not in a loop: a daemon that is genuinely gone fails the reconnect, and that is the error
    /// worth reporting.
    pub async fn send(&mut self, upstream: Upstream, lane: Lane, line: &str) -> io::Result<()> {
        let key = (upstream, lane);
        let mut bytes = line.as_bytes().to_vec();
        bytes.push(b'\n');

        if !self.conns.contains_key(&key) {
            let conn = self.open(upstream, lane).await?;
            self.conns.insert(key, conn);
        }
        match self.write(key, &bytes).await {
            Err(e) if peer_is_gone(&e) => {
                let conn = self.open(upstream, lane).await?;
                self.conns.insert(key, conn);
                self.write(key, &bytes).await
            }
            done => done,
        }
    }

    /// One bounded write on the connection for `key`. Any failure drops that connection, so
    /// nothing keeps writing into a dead socket. This lane's only: the others may be perfectly
    /// alive, and a restart that broke one breaks the next write to each of them anyway.
    async fn write(&mut self, key: (Upstream, Lane), bytes: &[u8]) -> io::Result<()> {
        // Unwrap is sound: the caller inserted it, or `contains_key` held.
        let conn = self.conns.get_mut(&key).expect("connection present");
        let write = async {
            conn.write.write_all(bytes).await?;
            conn.write.flush().await
        };
        match tokio::time::timeout(WRITE_TIMEOUT, write).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => {
                self.conns.remove(&key);
                Err(e)
            }
            Err(_) => {
                self.conns.remove(&key);
                Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "upstream write timed out",
                ))
            }
        }
    }

    async fn open(&self, upstream: Upstream, lane: Lane) -> io::Result<Conn> {
        let path = self.sockets.path(upstream);
        let stream = tokio::time::timeout(CONNECT_TIMEOUT, UnixStream::connect(path))
            .await
            .map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "connect timed out"))??;

        let (read, write) = stream.into_split();
        let replies = self.replies.clone();
        // The lane is in the label because there are now several connections to each service,
        // and "Updater closed" without it names four possible sockets — including the progress
        // stream, whose closing is ordinary, and the operation lane, whose closing is not.
        let label = format!("{upstream:?}/{lane:?}");

        // The read half is pumped for the session's lifetime. Responses and notifications are
        // the same thing to us: a line to forward. That is what makes `update.subscribe`'s
        // progress stream work without any special case.
        tokio::spawn(async move {
            let mut lines = BufReader::new(read).lines();
            loop {
                match lines.next_line().await {
                    Ok(Some(line)) => {
                        // A full queue means the central cannot keep up. Give up on the line
                        // rather than the session: progress is advisory, and blocking here
                        // would stall every other upstream too.
                        if replies.try_send(line).is_err() {
                            tracing::debug!(upstream = %label, "dropped a line; client is behind");
                        }
                    }
                    Ok(None) => break,
                    Err(e) => {
                        tracing::debug!(upstream = %label, error = %e, "upstream read failed");
                        break;
                    }
                }
            }
            tracing::debug!(upstream = %label, "upstream closed");
        });

        tracing::debug!(upstream = ?upstream, lane = ?lane, path = %path.display(), "connected");
        Ok(Conn { write })
    }
}

/// Whether a write failed because the peer went away, rather than being slow or refusing. Only
/// these are worth one more try, because only these mean the bytes went to a daemon that is no
/// longer there. A timeout is a daemon that is there and stuck, and that is not retried.
fn peer_is_gone(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::BrokenPipe
            | io::ErrorKind::ConnectionReset
            | io::ErrorKind::ConnectionAborted
            | io::ErrorKind::NotConnected
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    const TIMEOUT: Duration = Duration::from_secs(5);

    /// A fake `configd` that answers one request with `response` and hangs up.
    fn serve_once(path: &Path, response: proto::Response) -> tokio::task::JoinHandle<String> {
        let listener = tokio::net::UnixListener::bind(path).unwrap();
        tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let (read, mut write) = stream.into_split();
            let request = BufReader::new(read)
                .lines()
                .next_line()
                .await
                .unwrap()
                .unwrap();

            let mut line = serde_json::to_vec(&response).unwrap();
            line.push(b'\n');
            write.write_all(&line).await.unwrap();
            write.flush().await.unwrap();
            request
        })
    }

    /// The whole path `bluez` uses to learn what to advertise: ask `configd`, get a name back.
    #[tokio::test]
    async fn a_question_is_asked_and_the_answer_deserialised() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("configd.sock");
        let fake = serve_once(
            &path,
            proto::Response::ok(
                Some(proto::Id::Number(1)),
                &proto::SystemInfoResult {
                    name: "duck-7f3a".into(),
                    serial: Some("bb7b734a7717ac41".into()),
                    uptime_seconds: 12,
                },
            ),
        );

        let info: proto::SystemInfoResult =
            ask("configd", &path, &proto::Call::SystemInfo, TIMEOUT)
                .await
                .unwrap()
                .result_as()
                .unwrap();
        assert_eq!(info.name, "duck-7f3a");

        // And the method on the wire was the one asked for, not merely something that parsed.
        let request = fake.await.unwrap();
        assert!(request.contains(proto::method::SYSTEM_INFO), "{request}");
    }

    /// `btd` is on the recovery path and must come up when the rest of the robot has not, so an
    /// absent `configd` is a reported error rather than a hang or a panic.
    #[tokio::test]
    async fn an_absent_service_is_an_error_naming_it() {
        let dir = tempfile::tempdir().unwrap();
        let err = ask(
            "configd",
            &dir.path().join("absent.sock"),
            &proto::Call::SystemInfo,
            TIMEOUT,
        )
        .await
        .unwrap_err();
        assert!(err.contains("cannot reach configd"), "{err}");
    }

    /// A service that refuses must not read as an answer: the caller would otherwise advertise
    /// whatever `result_as` makes of a null result.
    #[tokio::test]
    async fn a_refusal_is_an_error_rather_than_an_empty_answer() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("configd.sock");
        let _fake = serve_once(
            &path,
            proto::Response::err(
                Some(proto::Id::Number(1)),
                proto::Error::new(proto::code::INTERNAL_ERROR, "no"),
            ),
        );

        let err = ask("configd", &path, &proto::Call::SystemInfo, TIMEOUT)
            .await
            .unwrap_err();
        assert!(err.contains("configd refused"), "{err}");
    }
    /// The daemon restarted between two requests.
    ///
    /// The comment on the write path says a broken pipe is ordinary, the daemon restarted, and
    /// the next call reconnects. It is the call *after* the next: the first write after a restart
    /// went into the socket the old daemon closed, failed, and the phone was told the service was
    /// not answering while it was listening on a fresh socket the whole time. The pool's own
    /// reader had already seen the socket close and told nobody.
    ///
    /// On the BLE update path this is `updaterd` restarting itself from the release's postinstall
    /// hook, and the phone's next `update.status` failing for it.
    #[tokio::test]
    async fn a_restarted_daemon_gets_the_next_request_not_the_one_after() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("updaterd.sock");
        let sockets = Sockets {
            updater: path.clone(),
            robot: dir.path().join("robotd.sock"),
            config: dir.path().join("configd.sock"),
        };
        let (replies, mut forwarded) = mpsc::channel(8);
        let mut pool = Pool::new(sockets, replies);

        let before = serve_once(
            &path,
            proto::Response::ok(Some(proto::Id::Number(1)), &serde_json::json!({})),
        );
        pool.send(
            Upstream::Updater,
            Lane::Prompt,
            r#"{"jsonrpc":"2.0","id":1,"method":"update.status"}"#,
        )
        .await
        .unwrap();
        assert!(
            forwarded.recv().await.is_some(),
            "the first answer is forwarded"
        );
        // The daemon has answered and hung up: it is restarting.
        before.await.unwrap();

        // And it is back, listening on a fresh socket at the same path.
        std::fs::remove_file(&path).unwrap();
        let after = serve_once(
            &path,
            proto::Response::ok(Some(proto::Id::Number(2)), &serde_json::json!({})),
        );
        pool.send(
            Upstream::Updater,
            Lane::Prompt,
            r#"{"jsonrpc":"2.0","id":2,"method":"update.status"}"#,
        )
        .await
        .expect("a daemon that is back must get the request, not a broken pipe");
        let request = after.await.unwrap();
        assert!(request.contains(r#""id":2"#), "{request}");
        assert!(
            forwarded.recv().await.is_some(),
            "and its answer is forwarded"
        );
    }
}
