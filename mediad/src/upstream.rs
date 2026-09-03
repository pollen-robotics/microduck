//! Connections to the services that own the answers.
//!
//! One socket per service, connected directly — with five services there is still no case for a
//! broker, and a bus would be another component that can fail (`architecture.md` §2.2).
//!
//! **Five, where `btd` holds three.** That is the concrete difference between the transports:
//! `mediad` carries the pad tap and the depth stream, which BLE refuses on capacity grounds and
//! which `btd` deliberately holds no socket for. `mediad::route` is where that decision lives.
//!
//! Every operation here is timeout-bounded, without exception. Any peer may be dead, and a closed
//! or silent socket is a normal answer rather than an error worth retrying forever — `robotd` in
//! particular is the service most likely to be missing, since it is the one an update restarts.
//!
//! ## This is close to `btd::upstream`, and not shared yet
//!
//! Deliberately. The two differ in more than the service count: `btd`'s carries `NameChoice`,
//! which is about what to advertise over Bluetooth and has no meaning here. Extracting a shared
//! pool from one example would be guessing at the shape; extracting it from two, once this one has
//! run against a real peer, is a refactor with evidence behind it. The duplication is named here
//! so it is a decision rather than an oversight.

use std::collections::HashMap;
use std::io;
use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tokio::sync::mpsc;

/// Long enough for a loaded board, short enough that a peer gets an answer rather than a spinner.
/// A unix socket connect either succeeds immediately or the daemon is not there.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Cap on a single write. A blocked write means the daemon has stopped reading, which is a dead
/// peer rather than a slow one.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

/// Where each service listens. Defaults are `proto::socket`, which is where they live so that a
/// path duplicated per client cannot drift per client.
#[derive(Debug, Clone)]
pub struct Sockets {
    pub updater: PathBuf,
    pub robot: PathBuf,
    pub config: PathBuf,
    pub pad: PathBuf,
    pub tof: PathBuf,
}

impl Default for Sockets {
    fn default() -> Self {
        Self {
            updater: proto::socket::UPDATER.into(),
            robot: proto::socket::ROBOT.into(),
            config: proto::socket::CONFIG.into(),
            pad: proto::socket::PAD.into(),
            tof: proto::socket::TOF.into(),
        }
    }
}

impl Sockets {
    pub fn path(&self, service: proto::Service) -> &Path {
        match service {
            proto::Service::Updater => &self.updater,
            proto::Service::Robot => &self.robot,
            proto::Service::Config => &self.config,
            proto::Service::Pad => &self.pad,
            proto::Service::Tof => &self.tof,
        }
    }
}

struct Conn {
    write: tokio::net::unix::OwnedWriteHalf,
}

/// One connection per (service, lane), opened on demand and kept for the session.
///
/// **Keyed on the lane as well as the service**, which is what keeps a minutes-long update from
/// silencing everything else a peer asks. Every daemon serves one connection one request at a
/// time, so calls that share a connection share a queue — [`proto::Lane`] records the two
/// orderings that a single connection per service breaks. At most four sockets per service per
/// session, and in practice two.
pub struct Pool {
    sockets: Sockets,
    conns: HashMap<(proto::Service, proto::Lane), Conn>,
    /// Every reply and notification from every service, merged. Merging is safe because JSON-RPC
    /// correlates by `id`, which is the peer's business — `mediad` forwards lines without reading
    /// them, for the same reason `btd` does not: a subscription is a stream of notifications on an
    /// open connection, and correlating replies to requests would drop all but the first.
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

    /// Send one line to `service` on `lane`'s connection, connecting first if needed.
    ///
    /// A daemon that restarted between two requests is ordinary here: `robotd` is the one an
    /// update restarts, and `updaterd` restarts itself from a release's postinstall hook. The
    /// connection from before the restart is still in the pool, its reader has already seen the
    /// socket close, and the first write into it fails. Those bytes never left, so they are written
    /// again on a fresh connection rather than reported. Reporting them told the console that a
    /// daemon listening on a fresh socket was not answering, once per lane, after every restart.
    /// Once and not in a loop: a daemon that is genuinely gone fails the reconnect, and that is
    /// the error worth reporting.
    pub async fn send(
        &mut self,
        service: proto::Service,
        lane: proto::Lane,
        line: &str,
    ) -> io::Result<()> {
        let key = (service, lane);
        let mut bytes = line.as_bytes().to_vec();
        bytes.push(b'\n');

        if !self.conns.contains_key(&key) {
            let conn = self.open(service, lane).await?;
            self.conns.insert(key, conn);
        }
        match self.write(key, &bytes).await {
            Err(e) if peer_is_gone(&e) => {
                let conn = self.open(service, lane).await?;
                self.conns.insert(key, conn);
                self.write(key, &bytes).await
            }
            done => done,
        }
    }

    /// One bounded write on the connection for `key`. Any failure drops that connection, so
    /// nothing keeps writing into a dead socket. Only this lane's: the others may be perfectly
    /// alive.
    async fn write(&mut self, key: (proto::Service, proto::Lane), bytes: &[u8]) -> io::Result<()> {
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

    async fn open(&self, service: proto::Service, lane: proto::Lane) -> io::Result<Conn> {
        let path = self.sockets.path(service);
        let stream = tokio::time::timeout(CONNECT_TIMEOUT, UnixStream::connect(path))
            .await
            .map_err(|_| io::Error::new(io::ErrorKind::TimedOut, "connect timed out"))??;

        let (read, write) = stream.into_split();
        let replies = self.replies.clone();
        // The lane is in the label because there are several connections per service, and
        // "Updater closed" without it names four possible sockets — including the progress stream,
        // whose closing is ordinary, and the operation lane, whose closing is not.
        let label = format!("{service:?}/{lane:?}");

        // Pumped for the session's lifetime. Responses and notifications are the same thing here:
        // a line to forward. That is what makes a subscription's stream work with no special case.
        tokio::spawn(async move {
            let mut lines = BufReader::new(read).lines();
            loop {
                match lines.next_line().await {
                    Ok(Some(line)) => {
                        // A full queue means the peer cannot keep up. Give up on the line rather
                        // than the session: telemetry is advisory, and blocking here would stall
                        // every other service too.
                        if replies.try_send(line).is_err() {
                            tracing::debug!(upstream = %label, "dropped a line; peer is behind");
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

        tracing::debug!(service = ?service, lane = ?lane, path = %path.display(), "connected");
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

    /// A fake daemon that answers one request with `response` and hangs up.
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

    /// The daemon restarted between two requests.
    ///
    /// The same pool `btd` has, with the same hole it had: the first write after a restart went
    /// into the socket the old daemon closed, failed, and the console was told the service was
    /// not answering while it was listening on a fresh socket the whole time.
    #[tokio::test]
    async fn a_restarted_daemon_gets_the_next_request_not_the_one_after() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.sock");
        let sockets = Sockets {
            robot: path.clone(),
            updater: dir.path().join("updaterd.sock"),
            config: dir.path().join("configd.sock"),
            pad: dir.path().join("padd.sock"),
            tof: dir.path().join("tofd.sock"),
        };
        let (replies, mut forwarded) = mpsc::channel(8);
        let mut pool = Pool::new(sockets, replies);

        let before = serve_once(
            &path,
            proto::Response::ok(Some(proto::Id::Number(1)), &serde_json::json!({})),
        );
        pool.send(
            proto::Service::Robot,
            proto::Lane::Prompt,
            r#"{"jsonrpc":"2.0","id":1,"method":"robot.health"}"#,
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
            proto::Service::Robot,
            proto::Lane::Prompt,
            r#"{"jsonrpc":"2.0","id":2,"method":"robot.health"}"#,
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
