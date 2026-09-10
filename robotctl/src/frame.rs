//! Local binary snapshots deliberately bypass the JSON-only service router.
use super::{Client, Failure, exit, proto};
use std::io::{BufRead, Read};
use std::path::Path;
use std::time::Duration;

pub(super) fn run(socket: &Path, output: &Path) -> Result<(), Failure> {
    let mut client = Client::connect_to("mediad", socket)?;
    let timeout = Some(Duration::from_secs(3));
    client
        .reader
        .get_ref()
        .set_read_timeout(timeout)
        .map_err(failed)?;
    client.writer.set_write_timeout(timeout).map_err(failed)?;
    client.hello()?;
    let id = proto::Id::Number(client.next_id);
    client.send(&proto::Request {
        jsonrpc: "2.0".into(),
        id: Some(id.clone()),
        method: proto::method::MEDIA_FRAME.into(),
        params: None,
    })?;
    let (header, bytes) = read_frame(&mut client.reader, id).map_err(failed)?;
    // Never leave a success-looking file behind after an incomplete camera reply.
    std::fs::write(output, bytes).map_err(failed)?;
    eprintln!("{}", serde_json::to_string(&header).map_err(failed)?);
    Ok(())
}

fn failed(error: impl std::fmt::Display) -> Failure {
    Failure::new(exit::FAILED, format!("camera snapshot: {error}"))
}

fn read_frame(
    reader: &mut impl BufRead,
    id: proto::Id,
) -> Result<(proto::MediaFrameHeader, Vec<u8>), Box<dyn std::error::Error>> {
    let mut line = Vec::new();
    reader.by_ref().take(4097).read_until(b'\n', &mut line)?;
    if line.len() > 4096 || !line.ends_with(b"\n") {
        return Err("invalid snapshot header length".into());
    }
    let response: proto::Response = serde_json::from_slice(&line)?;
    if response.id != Some(id) || response.jsonrpc != "2.0" {
        return Err("snapshot response does not match request".into());
    }
    if let Some(error) = response.error {
        return Err(format!("{}: {}", error.code, error.message).into());
    }
    let header: proto::MediaFrameHeader =
        serde_json::from_value(response.result.ok_or("missing snapshot result")?)?;
    if !header.valid_uyvy() {
        return Err("invalid UYVY geometry or size".into());
    }
    let mut bytes = vec![0; header.bytes];
    // Reuse the header reader: it may already have buffered the beginning of the binary tail.
    reader.read_exact(&mut bytes)?;
    Ok((header, bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufReader, Cursor};
    fn reply(bytes: usize, tail: &[u8]) -> Vec<u8> {
        let header = proto::MediaFrameHeader {
            width: 2,
            height: 1,
            format: "UYVY".into(),
            bytes,
            captured_at_unix_us: 1,
        };
        let mut wire =
            serde_json::to_vec(&proto::Response::ok(Some(proto::Id::Number(2)), &header)).unwrap();
        wire.push(b'\n');
        wire.extend(tail);
        wire
    }
    #[test]
    fn command_handshakes_and_only_saves_complete_frames() {
        use std::io::Write;
        use std::os::unix::net::UnixListener;
        for complete in [true, false] {
            let dir = tempfile::tempdir().unwrap();
            let socket = dir.path().join("media.sock");
            let output = dir.path().join("frame.uyvy");
            let listener = UnixListener::bind(&socket).unwrap();
            let server = std::thread::spawn(move || {
                let (mut stream, _) = listener.accept().unwrap();
                let mut reader = BufReader::new(stream.try_clone().unwrap());
                let mut line = String::new();
                reader.read_line(&mut line).unwrap();
                let hello: proto::Request = serde_json::from_str(&line).unwrap();
                assert_eq!(hello.method, proto::method::HELLO);
                let hello = proto::Response::ok(
                    hello.id,
                    &proto::HelloResult {
                        api_version: proto::API_VERSION,
                        daemon_version: None,
                        revision: None,
                    },
                );
                writeln!(stream, "{}", serde_json::to_string(&hello).unwrap()).unwrap();
                line.clear();
                reader.read_line(&mut line).unwrap();
                let frame: proto::Request = serde_json::from_str(&line).unwrap();
                assert_eq!(frame.method, proto::method::MEDIA_FRAME);
                assert_eq!(frame.id, Some(proto::Id::Number(2)));
                stream
                    .write_all(&reply(
                        4,
                        if complete {
                            &[128, 16, 128, 235]
                        } else {
                            &[128]
                        },
                    ))
                    .unwrap();
            });
            let result = run(&socket, &output);
            server.join().unwrap();
            assert_eq!(result.is_ok(), complete);
            assert_eq!(output.exists(), complete);
            if complete {
                assert_eq!(std::fs::read(output).unwrap(), [128, 16, 128, 235]);
            }
        }
    }

    #[test]
    fn coalesced_header_and_binary_are_preserved() {
        let mut reader = BufReader::new(Cursor::new(reply(4, &[128, 16, 128, 235])));
        assert_eq!(
            read_frame(&mut reader, proto::Id::Number(2)).unwrap().1,
            [128, 16, 128, 235]
        );
    }
    #[test]
    fn truncated_mismatched_and_oversized_replies_fail() {
        for (wire, id) in [
            (reply(4, &[1]), 2),
            (reply(4, &[1; 4]), 3),
            (reply(usize::MAX, &[]), 2),
            (vec![b'x'; 4097], 2),
        ] {
            assert!(
                read_frame(
                    &mut BufReader::new(Cursor::new(wire)),
                    proto::Id::Number(id)
                )
                .is_err()
            );
        }
    }
}
