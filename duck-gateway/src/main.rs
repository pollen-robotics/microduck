//! Authenticated local media gateway.
//!
//! `mediad` stays on loopback. A remote phone must prove the per-robot secret through Noise
//! before this process opens a connection to mediad. No WebSocket or robot command is parsed
//! here; the tunnel only moves authenticated bytes.
//!
//! Enabled on every robot, and only serves one with a setup code: the key comes from the `ble-psk`
//! credential `robotctl system setup-code export` leaves before this starts. Without one there is
//! nothing to authenticate against, so it says so and exits cleanly rather than failing in a loop.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use clap::Parser;
use duck_auth::{ProvisionedKey, tcp};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::Semaphore;

const MAX_SESSIONS: usize = 8;

#[derive(Parser)]
struct Args {
    /// The setup-code key, 64 hex characters. Defaults to the imported `ble-psk` credential.
    #[arg(long)]
    key_file: Option<PathBuf>,
    #[arg(long, default_value = "0.0.0.0:9443")]
    listen: SocketAddr,
    #[arg(long, default_value = "127.0.0.1:8443")]
    mediad: SocketAddr,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    let args = Args::parse();
    let Some(key_file) = args.key_file.clone().or_else(imported_key) else {
        tracing::info!(
            "no setup code on this robot, so nothing to authenticate against; not serving"
        );
        return Ok(());
    };
    // The same id `btd` binds and the QR carries: the robot's first name, from the SoC serial.
    let robot_id = robotd_params::serial::this_first_name()
        .ok_or_else(|| std::io::Error::other("no SoC serial to derive the robot id from"))?;
    let credential = ProvisionedKey::load(&key_file, robot_id).map_err(std::io::Error::other)?;
    let listener = TcpListener::bind(args.listen).await?;
    let capacity = Arc::new(Semaphore::new(MAX_SESSIONS));
    tracing::info!(address = %listener.local_addr()?, "authenticated media gateway listening");
    loop {
        let (stream, peer) = listener.accept().await?;
        let Ok(permit) = capacity.clone().try_acquire_owned() else {
            tracing::warn!(%peer, sessions = MAX_SESSIONS, "gateway full; refusing a connection");
            continue;
        };
        let credential = credential.clone();
        let mediad = args.mediad;
        tokio::spawn(async move {
            let _permit = permit;
            // Each way a session ends gets its own level, because "the phone says No answer" has
            // three causes that look identical from the phone, and this journal is the only place
            // that can tell them apart. Before this, all three were `debug` and a failed drive
            // left no trace on the robot at all.
            match serve_one(stream, credential, mediad).await {
                Ok(()) => tracing::debug!(%peer, "gateway session ended"),
                Err(Ended::Authentication(error)) => tracing::warn!(
                    %peer, %error,
                    "a peer reached the gateway but did not prove the setup code; wrong or stale code?"
                ),
                Err(Ended::Mediad(error)) => tracing::warn!(
                    %peer, %error, %mediad,
                    "authenticated, but mediad's signalling is unreachable"
                ),
                Err(Ended::Session(error)) => {
                    tracing::debug!(%peer, %error, "gateway session closed")
                }
            }
        });
    }
}

/// The setup-code key systemd imported for this service (`ImportCredential=ble-psk`), if any.
fn imported_key() -> Option<PathBuf> {
    let path = PathBuf::from(std::env::var_os("CREDENTIALS_DIRECTORY")?).join("ble-psk");
    path.is_file().then_some(path)
}

/// Where a session stopped, so the journal can say which of the three it was.
#[derive(Debug)]
enum Ended {
    Authentication(std::io::Error),
    Mediad(std::io::Error),
    Session(std::io::Error),
}

async fn serve_one(
    mut remote: TcpStream,
    credential: ProvisionedKey,
    mediad: SocketAddr,
) -> Result<(), Ended> {
    let peer = remote.peer_addr().ok();
    let session = tokio::time::timeout(
        Duration::from_secs(10),
        tcp::robot_handshake(&mut remote, credential.key(), credential.robot_id()),
    )
    .await
    .map_err(|_| std::io::Error::new(std::io::ErrorKind::TimedOut, "authentication timed out"))
    .and_then(|handshake| handshake)
    .map_err(Ended::Authentication)?;
    let local = TcpStream::connect(mediad).await.map_err(Ended::Mediad)?;
    tracing::info!(?peer, "authenticated media session opened");
    tcp::proxy(local, remote, session)
        .await
        .map_err(Ended::Session)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    #[tokio::test]
    async fn mediad_is_opened_only_after_authentication() {
        let mediad = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let mediad_addr = mediad.local_addr().unwrap();
        let gateway = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let gateway_addr = gateway.local_addr().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("key");
        std::fs::write(&path, "71".repeat(32)).unwrap();
        let credential = ProvisionedKey::load(&path, "test-duck".into()).unwrap();

        let app = tokio::spawn(async move {
            let mut stream = TcpStream::connect(gateway_addr).await.unwrap();
            let session = tcp::app_handshake(&mut stream, &[0x71; 32], "test-duck")
                .await
                .unwrap();
            let (mut local_app, local_proxy) = tokio::io::duplex(4096);
            tokio::spawn(tcp::proxy(local_proxy, stream, session));
            local_app.write_all(b"GET / HTTP/1.1\r\n").await.unwrap();
            let mut reply = [0; 6];
            local_app.read_exact(&mut reply).await.unwrap();
            assert_eq!(&reply, b"101 OK");
        });
        let (remote, _) = gateway.accept().await.unwrap();
        let robot = tokio::spawn(serve_one(remote, credential, mediad_addr));
        let (mut media, _) = mediad.accept().await.unwrap();
        let mut request = [0; 16];
        media.read_exact(&mut request).await.unwrap();
        assert_eq!(&request, b"GET / HTTP/1.1\r\n");
        media.write_all(b"101 OK").await.unwrap();
        app.await.unwrap();
        robot.abort();
    }

    #[tokio::test]
    async fn wrong_secret_cannot_reach_mediad() {
        let mediad = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let gateway = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = gateway.local_addr().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("key");
        std::fs::write(&path, "71".repeat(32)).unwrap();
        let credential = ProvisionedKey::load(&path, "test-duck".into()).unwrap();
        let app = tokio::spawn(async move {
            let mut stream = TcpStream::connect(address).await.unwrap();
            assert!(
                tcp::app_handshake(&mut stream, &[0x72; 32], "test-duck")
                    .await
                    .is_err()
            );
        });
        let (remote, _) = gateway.accept().await.unwrap();
        assert!(
            serve_one(remote, credential, mediad.local_addr().unwrap())
                .await
                .is_err()
        );
        app.await.unwrap();
        assert!(
            tokio::time::timeout(Duration::from_millis(20), mediad.accept())
                .await
                .is_err()
        );
    }
}
