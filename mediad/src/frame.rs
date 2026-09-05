//! A local `media.frame` endpoint for a recorder or perception process on the robot.
//!
//! A frame stays out of the WebRTC control channel: at the default geometry the UYVY payload is
//! about 1.8 MiB, so JSON/base64 would make a control request several MiB and let a slow peer tie
//! camera data to the network. The Unix socket is group-readable like the other observation
//! sockets. It sends one JSON-RPC response header, then precisely `bytes` raw bytes; that keeps
//! the metadata inspectable without copying pixels through a text encoding.

use std::os::unix::fs::PermissionsExt;
use std::path::Path;
use std::time::UNIX_EPOCH;

use anyhow::{Context, Result};
use duck_ipc_proto as proto;
use serde::Serialize;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

use crate::pipeline::Frames;

const SOCKET_MODE: u32 = 0o660;
const MAX_REQUEST_BYTES: usize = 4096;

#[derive(Debug, Serialize)]
struct Header {
    width: u32,
    height: u32,
    format: &'static str,
    bytes: usize,
    /// Wall time makes the snapshot joinable to a separately sampled robot state. It is not used
    /// to pace capture, so an NTP adjustment cannot affect the pipeline.
    captured_at_unix_us: u128,
}

/// Serve snapshots until the daemon exits. A client gets the latest frame immediately, or an
/// explicit error before the first camera buffer arrives; it never waits in a queue for a future
/// frame.
pub async fn serve(socket: &Path, frames: Frames) -> Result<()> {
    if let Some(parent) = socket.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating {}", parent.display()))?;
    }
    if socket.exists() {
        std::fs::remove_file(socket)
            .with_context(|| format!("removing stale {}", socket.display()))?;
    }
    let listener =
        UnixListener::bind(socket).with_context(|| format!("binding {}", socket.display()))?;
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))
        .with_context(|| format!("setting permissions on {}", socket.display()))?;
    tracing::info!(path = %socket.display(), mode = format!("{SOCKET_MODE:o}"), "serving media.frame locally");

    loop {
        match listener.accept().await {
            Ok((stream, _)) => {
                let frames = frames.clone();
                tokio::spawn(async move {
                    if let Err(error) = handle(stream, frames).await {
                        tracing::debug!(error = %error, "media.frame client ended");
                    }
                });
            }
            Err(error) => tracing::warn!(error = %error, "media.frame accept failed"),
        }
    }
}

async fn handle(stream: UnixStream, frames: Frames) -> Result<()> {
    let (read, mut write) = stream.into_split();
    let mut reader = BufReader::new(read);
    let mut line = String::new();
    reader.read_line(&mut line).await?;
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
    let request: proto::Request = match serde_json::from_str(line.trim()) {
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
        return Ok(());
    }
    let Some(frame) = frames.latest() else {
        write_response(
            &mut write,
            proto::Response::err(
                request.id,
                proto::Error::new(
                    proto::code::INTERNAL_ERROR,
                    "camera has not produced a frame yet",
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
    let header = Header {
        width: frame.width,
        height: frame.height,
        format: frame.format,
        bytes: frame.data.len(),
        captured_at_unix_us,
    };
    write_response(&mut write, proto::Response::ok(request.id, &header)).await?;
    write.write_all(&frame.data).await?;
    write.flush().await?;
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

    #[tokio::test]
    async fn no_camera_frame_is_an_explicit_error() {
        let response = reply(
            Frames::default(),
            "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"media.frame\",\"params\":{}}\n",
        )
        .await;
        assert_eq!(response.id, Some(proto::Id::Number(7)));
        assert_eq!(response.error.unwrap().code, proto::code::INTERNAL_ERROR);
    }

    #[tokio::test]
    async fn an_unknown_method_is_refused_without_reading_a_frame() {
        let response = reply(
            Frames::default(),
            "{\"jsonrpc\":\"2.0\",\"id\":\"request\",\"method\":\"media.other\"}\n",
        )
        .await;
        assert_eq!(response.id, Some(proto::Id::String("request".into())));
        assert_eq!(response.error.unwrap().code, proto::code::METHOD_NOT_FOUND);
    }

    #[tokio::test]
    async fn an_oversized_request_is_rejected_before_it_is_parsed() {
        let request = format!("{}\n", "x".repeat(MAX_REQUEST_BYTES + 1));
        let response = reply(Frames::default(), &request).await;
        assert_eq!(response.id, None);
        assert_eq!(response.error.unwrap().code, proto::code::INVALID_PARAMS);
    }

    #[tokio::test]
    async fn a_frame_reply_names_and_follows_with_exactly_its_pixels() {
        let frames = Frames::default();
        frames.publish(Frame {
            width: 2,
            height: 1,
            format: "UYVY",
            captured_at: UNIX_EPOCH + Duration::from_secs(1),
            data: vec![128, 32, 128, 64],
        });
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
    }
}
