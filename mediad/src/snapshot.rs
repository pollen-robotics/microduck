//! A bounded local snapshot client for the HTTP console; camera bytes never enter control routing.
use anyhow::{Context, Result, ensure};
use duck_ipc_proto as proto;
use std::path::Path;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

pub(crate) async fn fetch(socket: &Path) -> Result<(proto::MediaFrameHeader, Vec<u8>)> {
    tokio::time::timeout(std::time::Duration::from_secs(3), async {
        let stream = UnixStream::connect(socket).await?;
        let (read, mut write) = stream.into_split();
        let mut reader = BufReader::new(read);
        // A normal version handshake, followed by the deliberately unrouted binary method.
        let request = proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::Hello(proto::HelloParams {
                api_version: proto::API_VERSION,
            }),
        );
        let mut line = serde_json::to_vec(&request)?;
        line.push(b'\n');
        write.write_all(&line).await?;
        let hello = response(&mut reader, 1).await?;
        let hello: proto::HelloResult = serde_json::from_value(hello)?;
        ensure!(
            hello.api_version == proto::API_VERSION,
            "camera API version mismatch"
        );
        let request = proto::Request {
            jsonrpc: "2.0".into(),
            id: Some(proto::Id::Number(2)),
            method: proto::method::MEDIA_FRAME.into(),
            params: None,
        };
        let mut line = serde_json::to_vec(&request)?;
        line.push(b'\n');
        write.write_all(&line).await?;
        let header: proto::MediaFrameHeader =
            serde_json::from_value(response(&mut reader, 2).await?)?;
        ensure!(
            header.valid_uyvy(),
            "invalid camera geometry or payload size"
        );
        let mut data = vec![0; header.bytes];
        reader.read_exact(&mut data).await?;
        Ok((header, data))
    })
    .await
    .context("camera snapshot timed out")?
}

async fn response(
    reader: &mut (impl tokio::io::AsyncBufRead + Unpin),
    id: u64,
) -> Result<serde_json::Value> {
    let mut line = Vec::new();
    reader.take(4097).read_until(b'\n', &mut line).await?;
    ensure!(
        line.len() <= 4096 && line.ends_with(b"\n"),
        "invalid camera response length"
    );
    let response: proto::Response = serde_json::from_slice(&line)?;
    ensure!(
        response.jsonrpc == "2.0" && response.id == Some(proto::Id::Number(id)),
        "camera response ID mismatch"
    );
    if let Some(error) = response.error {
        anyhow::bail!("camera: {}", error.message);
    }
    response.result.context("camera response has no result")
}

/// Encode the frame as a PNG, upright.
///
/// **This is the one consumer that has nowhere to put the rotation.** A PNG opened in a browser
/// or piped into a viewer carries no `rotate` alongside it, so a snapshot route that returned the
/// sensor's own orientation would hand every human a sideways picture and no way to know why. The
/// raw UYVY path keeps its "told, not applied" contract — the header names the angle and the
/// recorder turns it — but here the header *is* the thing being thrown away.
///
/// It costs nothing the hot path would notice: this runs once per request on a blocking thread,
/// not thirty times a second in front of the encoder, which is what made `videoflip` expensive.
pub(crate) fn png(header: proto::MediaFrameHeader, data: Vec<u8>) -> Result<Vec<u8>> {
    use image::ImageEncoder;
    let turn = uyvy::Turn::from_degrees(header.rotate)
        .context("camera reported a mount that is not a multiple of 90°")?;
    let mut rgb = Vec::new();
    // The turned dimensions, not the header's: a quarter turn swaps the axes, and encoding the
    // capture geometry over rotated pixels is a diagonally sheared image rather than an error.
    let (width, height) = uyvy::rgb_from_uyvy(
        &data,
        header.width as usize,
        header.height as usize,
        // No downscale. `max` is the same either side of a quarter turn, so this stays the
        // longest edge whichever way the frame is about to go.
        header.width.max(header.height) as usize,
        turn,
        &mut rgb,
    );
    let mut encoded = Vec::new();
    image::codecs::png::PngEncoder::new(&mut encoded).write_image(
        &rgb,
        width as u32,
        height as u32,
        image::ExtendedColorType::Rgb8,
    )?;
    Ok(encoded)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[tokio::test]
    async fn malformed_or_unmatched_headers_fail_before_pixels_are_allocated() {
        for bytes in [
            vec![b'x'; 4097],
            vec![255, b'\n'],
            b"{\"jsonrpc\":\"2.0\",\"id\":99,\"result\":{}}\n".to_vec(),
        ] {
            let mut reader = BufReader::new(bytes.as_slice());
            assert!(response(&mut reader, 2).await.is_err());
        }
    }
    #[tokio::test]
    async fn silent_camera_has_a_deadline() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("media.sock");
        let _listener = tokio::net::UnixListener::bind(&socket).unwrap();
        let result = tokio::time::timeout(std::time::Duration::from_secs(5), fetch(&socket))
            .await
            .unwrap();
        assert!(result.unwrap_err().to_string().contains("timed out"));
    }
}
