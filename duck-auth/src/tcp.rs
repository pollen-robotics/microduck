//! Noise-authenticated byte tunnel for the local media WebSocket.
//!
//! The webview connects to an app-owned loopback listener. The robot's gateway connects only
//! to mediad's loopback signaller after this handshake succeeds. Each direction carries
//! length-prefixed Noise messages; the WebSocket bytes remain opaque to the tunnel.

use std::io;
use std::sync::{Arc, Mutex};

use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

use crate::{ClientStart, RobotConfirm, Session};

const MAX_RECORD: usize = 16 * 1024;

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

/// Send one bounded Noise ciphertext or handshake message.
pub async fn write_record<W: AsyncWrite + Unpin>(writer: &mut W, bytes: &[u8]) -> io::Result<()> {
    let size = u16::try_from(bytes.len()).map_err(|_| invalid("Noise record too large"))?;
    if bytes.is_empty() || bytes.len() > MAX_RECORD {
        return Err(invalid("Noise record size is invalid"));
    }
    writer.write_u16(size).await?;
    writer.write_all(bytes).await
}

/// Read one bounded Noise ciphertext or handshake message.
pub async fn read_record<R: AsyncRead + Unpin>(reader: &mut R) -> io::Result<Vec<u8>> {
    let size = usize::from(reader.read_u16().await?);
    if size == 0 || size > MAX_RECORD {
        return Err(invalid("Noise record size is invalid"));
    }
    let mut bytes = vec![0; size];
    reader.read_exact(&mut bytes).await?;
    Ok(bytes)
}

/// Authenticate the app side before any proxied WebSocket byte is sent.
pub async fn app_handshake<S: AsyncRead + AsyncWrite + Unpin>(
    stream: &mut S,
    key: &[u8; 32],
    robot_id: &str,
) -> io::Result<Session> {
    let (client, first) =
        ClientStart::begin(key, robot_id).map_err(|_| invalid("Noise setup failed"))?;
    write_record(stream, &first).await?;
    let challenge = read_record(stream).await?;
    let (client, proof) = client
        .receive(&challenge)
        .map_err(|_| invalid("robot authentication failed"))?;
    write_record(stream, &proof).await?;
    let ready = read_record(stream).await?;
    client
        .receive(&ready)
        .map_err(|_| invalid("robot key confirmation failed"))
}

/// Authenticate the robot side before connecting to mediad.
pub async fn robot_handshake<S: AsyncRead + AsyncWrite + Unpin>(
    stream: &mut S,
    key: &[u8; 32],
    robot_id: &str,
) -> io::Result<Session> {
    let first = read_record(stream).await?;
    let (robot, challenge) = RobotConfirm::respond(key, robot_id, &first)
        .map_err(|_| invalid("app authentication failed"))?;
    write_record(stream, &challenge).await?;
    let proof = read_record(stream).await?;
    let (session, ready) = robot
        .receive(&proof)
        .map_err(|_| invalid("app key confirmation failed"))?;
    write_record(stream, &ready).await?;
    Ok(session)
}

/// Move opaque bytes between a local socket and the authenticated remote socket.
/// Both directions stop when either closes or an authenticated record fails.
pub async fn proxy<L, R>(local: L, remote: R, session: Session) -> io::Result<()>
where
    L: AsyncRead + AsyncWrite + Unpin,
    R: AsyncRead + AsyncWrite + Unpin,
{
    let (mut local_read, mut local_write) = tokio::io::split(local);
    let (mut remote_read, mut remote_write) = tokio::io::split(remote);
    let session = Arc::new(Mutex::new(session));
    let up_session = session.clone();
    let upload = async move {
        let mut buffer = [0; 4096];
        loop {
            let len = local_read.read(&mut buffer).await?;
            if len == 0 {
                return Ok(());
            }
            let ciphertext = up_session
                .lock()
                .unwrap()
                .seal(&buffer[..len])
                .map_err(|_| invalid("Noise encryption failed"))?;
            write_record(&mut remote_write, &ciphertext).await?;
        }
    };
    let download = async move {
        loop {
            let ciphertext = read_record(&mut remote_read).await?;
            let plaintext = session
                .lock()
                .unwrap()
                .open(&ciphertext)
                .map_err(|_| invalid("Noise decryption failed"))?;
            local_write.write_all(&plaintext).await?;
        }
    };
    tokio::select! {
        result = upload => result,
        result = download => result,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn authenticated_tunnel_moves_bytes_both_ways() {
        let key = [0x7b; 32];
        let (mut app, mut robot) = tokio::io::duplex(8192);
        let (client, server) = tokio::join!(
            app_handshake(&mut app, &key, "test-robot"),
            robot_handshake(&mut robot, &key, "test-robot")
        );
        let mut client = client.unwrap();
        let mut server = server.unwrap();
        write_record(&mut app, &client.seal(b"GET / HTTP/1.1\r\n").unwrap())
            .await
            .unwrap();
        let ciphertext = read_record(&mut robot).await.unwrap();
        assert_eq!(server.open(&ciphertext).unwrap(), b"GET / HTTP/1.1\r\n");
        write_record(&mut robot, &server.seal(b"HTTP/1.1 101\r\n").unwrap())
            .await
            .unwrap();
        let ciphertext = read_record(&mut app).await.unwrap();
        assert_eq!(client.open(&ciphertext).unwrap(), b"HTTP/1.1 101\r\n");
    }
}
