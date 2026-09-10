//! A local `media.frame` endpoint for a recorder or perception process on the robot.
//!
//! A frame stays out of the WebRTC control channel: at the default geometry the UYVY payload is
//! about 1.8 MiB, so JSON/base64 would make a control request several MiB and let a slow peer tie
//! camera data to the network. This socket sends one JSON-RPC response header, then precisely
//! `bytes` raw bytes, which keeps the metadata inspectable without copying pixels through a text
//! encoding.
//!
//! **It asks for a frame rather than taking the last one.** [`Frames`] is a rendezvous, not a
//! cache: the capture branch copies a buffer only when a reader has asked for one
//! ([`crate::pipeline::Frames`] explains why — 1.84 MiB thirty times a second for readers that
//! want two). So a caller here waits for the capture that answers it, bounded by
//! [`pipeline::FRAME_TIMEOUT`], and a camera that has stopped is reported as a timeout rather than
//! answered with the frame it stopped on.
//!
//! The socket is group-readable like the other observation sockets: whoever may watch
//! `robot.state` may ask for a picture.

use std::os::unix::fs::PermissionsExt;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, UNIX_EPOCH};

use anyhow::{Context, Result};
use duck_ipc_proto as proto;

use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

use crate::pipeline::Frames;

const SOCKET_MODE: u32 = 0o660;

/// The group that may ask for a frame. Deliberately the same one as `robotd`'s socket, `padd`'s
/// tap and `tof`'s stream: whoever may watch the robot may watch what it sees.
const GROUP: &str = "robot";

/// The longest request this endpoint will read. `media.frame` takes no parameters worth naming, so
/// anything approaching this is a client that has lost the plot.
const MAX_REQUEST_BYTES: usize = 4096;

/// Claim the socket without replacing a live listener or a non-socket file.
pub async fn bind(socket: &Path) -> Result<(std::fs::File, UnixListener)> {
    use std::os::unix::fs::FileTypeExt;
    if let Some(parent) = socket.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    // Keep the lock inode for the listener's lifetime, including the stale-socket probe.
    let mut lock_path = socket.as_os_str().to_os_string();
    lock_path.push(".lock");
    let lock = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(lock_path)?;
    lock.try_lock()
        .context("another mediad owns the frame socket")?;
    let listener = match UnixListener::bind(socket) {
        Ok(listener) => listener,
        Err(error) if error.kind() == std::io::ErrorKind::AddrInUse => {
            if !std::fs::symlink_metadata(socket)?.file_type().is_socket() {
                return Err(error.into());
            }
            match tokio::time::timeout(Duration::from_secs(1), UnixStream::connect(socket)).await {
                Ok(Err(probe)) if probe.kind() == std::io::ErrorKind::ConnectionRefused => {
                    std::fs::remove_file(socket)?;
                    UnixListener::bind(socket)?
                }
                _ => return Err(error.into()),
            }
        }
        Err(error) => return Err(error.into()),
    };
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))
        .with_context(|| format!("setting permissions on {}", socket.display()))?;
    if let Err(error) = give_to_group(socket, GROUP) {
        // Not fatal, and said out loud with what it means: the socket exists, and only `mediad`
        // and root can reach it. On a board that is a broken install; on a laptop it is a machine
        // with no `robot` group, which is ordinary.
        tracing::warn!(
            error = %error, group = GROUP, socket = %socket.display(),
            "media.frame stays private to mediad — nothing else can ask for a picture"
        );
    }
    tracing::info!(
        path = %socket.display(),
        mode = format!("{SOCKET_MODE:o}"),
        "serving media.frame locally"
    );

    Ok((lock, listener))
}

/// Bound both simultaneous clients and the lifetime of silent or slow clients.
pub async fn serve(listener: UnixListener, frames: Frames) -> Result<()> {
    let slots = Arc::new(tokio::sync::Semaphore::new(16));
    loop {
        match listener.accept().await {
            Ok((stream, _)) => {
                let Ok(permit) = slots.clone().try_acquire_owned() else {
                    continue;
                };
                let frames = frames.clone();
                tokio::spawn(async move {
                    let _permit = permit;
                    let _ =
                        tokio::time::timeout(Duration::from_secs(5), handle(stream, frames)).await;
                });
            }
            Err(error) => tracing::warn!(error = %error, "media.frame accept failed"),
        }
    }
}

async fn handle(stream: UnixStream, frames: Frames) -> Result<()> {
    let (read, mut write) = stream.into_split();
    // Bounded *before* the line is buffered. Checking the length afterwards would mean a client
    // could make this process hold an arbitrarily long line first, which is the thing the cap is
    // for. One byte over the cap is read so that "too large" stays distinguishable from a request
    // that exactly fills it.
    let mut reader = BufReader::new(read);
    loop {
        let mut line = Vec::new();
        let read = (&mut reader)
            .take(MAX_REQUEST_BYTES as u64 + 1)
            .read_until(b'\n', &mut line)
            .await?;
        if read == 0 {
            return Ok(());
        }
        if line.len() > MAX_REQUEST_BYTES {
            write_response(
                &mut write,
                proto::Response::err(
                    None,
                    proto::Error::new(proto::code::INVALID_PARAMS, "request is too large"),
                ),
            )
            .await?;
            return Ok(());
        }
        let request: proto::Request = match serde_json::from_slice(&line) {
            Ok(request) => request,
            Err(error) => {
                write_response(
                    &mut write,
                    proto::Response::err(
                        None,
                        proto::Error::new(proto::code::PARSE_ERROR, error.to_string()),
                    ),
                )
                .await?;
                return Ok(());
            }
        };
        if request.method == proto::method::HELLO {
            write_response(
                &mut write,
                proto::Response::ok(
                    request.id,
                    &proto::HelloResult {
                        api_version: proto::API_VERSION,
                        daemon_version: proto::semver::Version::parse(env!("CARGO_PKG_VERSION"))
                            .ok(),
                        revision: proto::build_info!().revision.map(str::to_owned),
                    },
                ),
            )
            .await?;
            continue;
        }
        if request.method != proto::method::MEDIA_FRAME {
            write_response(
                &mut write,
                proto::Response::err(
                    request.id,
                    proto::Error::new(
                        proto::code::METHOD_NOT_FOUND,
                        format!("{} is not served by mediad", request.method),
                    ),
                ),
            )
            .await?;
            continue;
        }
        // `next_frame` registers the demand and parks on a condvar until the capture that answers it
        // lands, so it cannot run on the runtime's thread.
        let frame = tokio::task::spawn_blocking(move || frames.next_frame()).await?;
        let Some(frame) = frame else {
            write_response(
                &mut write,
                proto::Response::err(
                    request.id,
                    proto::Error::new(
                        proto::code::INTERNAL_ERROR,
                        "no frame arrived within the capture timeout",
                    ),
                ),
            )
            .await?;
            return Ok(());
        };
        let captured_at_unix_us = frame
            .captured_at
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros();
        let header = proto::MediaFrameHeader {
            width: frame.width,
            height: frame.height,
            format: frame.format.to_owned(),
            bytes: frame.data.len(),
            captured_at_unix_us,
        };
        write_response(&mut write, proto::Response::ok(request.id, &header)).await?;
        write.write_all(&frame.data).await?;
        write.flush().await?;
        return Ok(());
    }
}

/// Hand the socket to `GROUP`. Mirrors `tof`'s stream and `padd`'s tap, including that a missing
/// group is a warning rather than a failure.
fn give_to_group(socket: &Path, group: &str) -> std::io::Result<()> {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;

    let name = CString::new(group).map_err(std::io::Error::other)?;
    // SAFETY: `getgrnam` reads the group database and returns a pointer into storage it owns. The
    // name is a valid C string for the length of the call, and nothing else in this process calls
    // into the group database.
    let entry = unsafe { libc::getgrnam(name.as_ptr()) };
    if entry.is_null() {
        return Err(std::io::Error::other(format!(
            "no {group} group on this system"
        )));
    }
    // SAFETY: checked non-null immediately above, and `struct group` is fully initialised by
    // `getgrnam` when it returns a pointer at all.
    let gid = unsafe { (*entry).gr_gid };

    let path = CString::new(socket.as_os_str().as_bytes()).map_err(std::io::Error::other)?;
    // SAFETY: a valid C string path; `-1` for the owner is the documented "leave it alone".
    if unsafe { libc::chown(path.as_ptr(), u32::MAX, gid) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

async fn write_response(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    response: proto::Response,
) -> Result<()> {
    let mut line = serde_json::to_vec(&response)?;
    line.push(b'\n');
    write.write_all(&line).await?;
    write.flush().await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, UNIX_EPOCH};

    use super::*;
    use crate::pipeline::Frame;
    use tokio::io::AsyncReadExt;

    #[tokio::test]
    async fn hello_and_unknown_method_keep_the_connection_for_a_frame() {
        let frames = Frames::default();
        let producer = answer_once(
            frames.clone(),
            Frame {
                width: 2,
                height: 1,
                format: "UYVY",
                captured_at: UNIX_EPOCH,
                data: vec![128, 16, 128, 235],
            },
        );
        let (mut client, server) = UnixStream::pair().unwrap();
        let task = tokio::spawn(handle(server, frames));
        client.write_all(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\"}\n{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"unknown\"}\n{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"media.frame\"}\n").await.unwrap();
        let mut reader = BufReader::new(client);
        for id in 1..=3 {
            let mut line = String::new();
            reader.read_line(&mut line).await.unwrap();
            let response: proto::Response = serde_json::from_str(&line).unwrap();
            assert_eq!(response.id, Some(proto::Id::Number(id)));
            if id == 1 {
                assert_eq!(response.result.unwrap()["api_version"], proto::API_VERSION);
            }
            if id == 2 {
                assert_eq!(response.error.unwrap().code, proto::code::METHOD_NOT_FOUND);
            }
        }
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await.unwrap();
        assert_eq!(bytes, [128, 16, 128, 235]);
        task.await.unwrap().unwrap();
        producer.join().unwrap();
    }

    #[tokio::test]
    async fn invalid_utf8_gets_a_parse_error() {
        let (mut client, server) = UnixStream::pair().unwrap();
        let task = tokio::spawn(handle(server, Frames::default()));
        client.write_all(&[255, b'\n']).await.unwrap();
        let mut line = String::new();
        BufReader::new(client).read_line(&mut line).await.unwrap();
        let response: proto::Response = serde_json::from_str(&line).unwrap();
        assert_eq!(response.error.unwrap().code, proto::code::PARSE_ERROR);
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn socket_claim_preserves_files_and_live_listeners_but_recovers_stale_sockets() {
        let dir = tempfile::tempdir().unwrap();
        let regular = dir.path().join("regular");
        std::fs::write(&regular, b"keep").unwrap();
        assert!(bind(&regular).await.is_err());
        assert_eq!(std::fs::read(&regular).unwrap(), b"keep");
        let live = dir.path().join("live");
        let listener = UnixListener::bind(&live).unwrap();
        assert!(bind(&live).await.is_err());
        assert!(UnixStream::connect(&live).await.is_ok());
        drop(listener);
        let (lock, listener) = bind(&live).await.unwrap();
        assert!(bind(&live).await.is_err());
        drop(listener);
        drop(lock);
        assert!(bind(&live).await.is_ok());
    }

    #[tokio::test]
    async fn idle_clients_are_bounded_and_expire() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("media.sock");
        let listener = UnixListener::bind(&socket).unwrap();
        let server = tokio::spawn(serve(listener, Frames::default()));
        let mut clients = Vec::new();
        for _ in 0..16 {
            let mut client = UnixStream::connect(&socket).await.unwrap();
            client
                .write_all(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\"}\n")
                .await
                .unwrap();
            let mut reader = BufReader::new(client);
            let mut line = String::new();
            reader.read_line(&mut line).await.unwrap();
            assert!(!line.is_empty());
            clients.push(reader);
        }
        let mut excess = UnixStream::connect(&socket).await.unwrap();
        let mut byte = [0];
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(1), excess.read(&mut byte))
                .await
                .unwrap()
                .unwrap(),
            0
        );
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(6), clients[0].read(&mut byte))
                .await
                .unwrap()
                .unwrap(),
            0
        );
        server.abort();
    }

    /// Stand in for the capture branch: wait for the demand this endpoint registers, then answer
    /// it once. Mirrors what `wire_frames` does on a buffer somebody asked for.
    fn answer_once(frames: Frames, frame: Frame) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            while !frames.take_request() {
                assert!(
                    std::time::Instant::now() < deadline,
                    "the endpoint never asked for a frame"
                );
                std::thread::yield_now();
            }
            frames.deliver(frame);
        })
    }

    async fn reply(frames: Frames, request: &str) -> proto::Response {
        let (mut client, server) = UnixStream::pair().unwrap();
        let task = tokio::spawn(handle(server, frames));
        client.write_all(request.as_bytes()).await.unwrap();
        client.shutdown().await.unwrap();
        let mut text = String::new();
        BufReader::new(client).read_line(&mut text).await.unwrap();
        task.await.unwrap().unwrap();
        serde_json::from_str(text.trim()).unwrap()
    }

    /// A camera that never delivers is a timeout, not a silent hang and not a stale frame.
    #[tokio::test]
    async fn a_capture_that_never_comes_is_an_explicit_error() {
        let response = reply(
            Frames::default(),
            "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"media.frame\",\"params\":{}}\n",
        )
        .await;
        assert_eq!(response.id, Some(proto::Id::Number(7)));
        assert_eq!(response.error.unwrap().code, proto::code::INTERNAL_ERROR);
    }

    /// Refused before any demand is registered, so an unknown method cannot make the capture
    /// branch copy 1.8 MiB for nothing.
    #[tokio::test]
    async fn an_unknown_method_is_refused_without_asking_for_a_frame() {
        let frames = Frames::default();
        let response = reply(
            frames.clone(),
            "{\"jsonrpc\":\"2.0\",\"id\":\"request\",\"method\":\"media.other\"}\n",
        )
        .await;
        assert_eq!(response.id, Some(proto::Id::Text("request".into())));
        assert_eq!(response.error.unwrap().code, proto::code::METHOD_NOT_FOUND);
        assert!(
            !frames.take_request(),
            "a refused method must not leave demand behind"
        );
    }

    #[tokio::test]
    async fn an_oversized_request_is_rejected_before_it_is_parsed() {
        let request = format!("{}\n", "x".repeat(MAX_REQUEST_BYTES + 1));
        let response = reply(Frames::default(), &request).await;
        assert_eq!(response.id, None);
        assert_eq!(response.error.unwrap().code, proto::code::INVALID_PARAMS);
    }

    /// The read is bounded before the line is buffered, so a client that never sends a newline
    /// cannot make this process hold an unbounded string.
    #[tokio::test]
    async fn a_request_without_a_newline_is_still_bounded() {
        let (mut client, server) = UnixStream::pair().unwrap();
        let task = tokio::spawn(handle(server, Frames::default()));
        client
            .write_all("y".repeat(MAX_REQUEST_BYTES * 4).as_bytes())
            .await
            .unwrap();
        client.shutdown().await.unwrap();
        let mut text = String::new();
        BufReader::new(client).read_line(&mut text).await.unwrap();
        task.await.unwrap().unwrap();
        let response: proto::Response = serde_json::from_str(text.trim()).unwrap();
        assert_eq!(response.error.unwrap().code, proto::code::INVALID_PARAMS);
    }

    #[tokio::test]
    async fn a_frame_reply_names_and_follows_with_exactly_its_pixels() {
        let frames = Frames::default();
        let producer = answer_once(
            frames.clone(),
            Frame {
                width: 2,
                height: 1,
                format: "UYVY",
                captured_at: UNIX_EPOCH + Duration::from_secs(1),
                data: vec![128, 32, 128, 64],
            },
        );
        let (mut client, server) = UnixStream::pair().unwrap();
        let task = tokio::spawn(handle(server, frames));
        client
            .write_all(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"media.frame\"}\n")
            .await
            .unwrap();
        let mut reader = BufReader::new(client);
        let mut line = String::new();
        reader.read_line(&mut line).await.unwrap();
        let response: proto::Response = serde_json::from_str(line.trim()).unwrap();
        let result = response.result.unwrap();
        assert_eq!(result["width"], 2);
        assert_eq!(result["height"], 1);
        assert_eq!(result["format"], "UYVY");
        assert_eq!(result["bytes"], 4);
        assert_eq!(result["captured_at_unix_us"], 1_000_000);
        let mut pixels = [0; 4];
        reader.read_exact(&mut pixels).await.unwrap();
        assert_eq!(pixels, [128, 32, 128, 64]);
        task.await.unwrap().unwrap();
        producer.join().unwrap();
    }
}
