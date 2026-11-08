//! Authenticated local media gateway for development.
//!
//! `mediad` stays on loopback. A remote phone must prove the per-robot secret through Noise
//! before this process opens a connection to mediad. No WebSocket or robot command is parsed
//! here; the tunnel only moves authenticated bytes.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use duck_auth::{ProvisionedKey, tcp};
use tokio::net::{TcpListener, TcpStream};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    key_file: PathBuf,
    #[arg(long)]
    robot_id: String,
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
    let credential =
        ProvisionedKey::load(&args.key_file, args.robot_id).map_err(std::io::Error::other)?;
    let listener = TcpListener::bind(args.listen).await?;
    tracing::info!(address = %listener.local_addr()?, "authenticated media gateway listening");
    loop {
        let (stream, peer) = listener.accept().await?;
        let credential = credential.clone();
        let mediad = args.mediad;
        tokio::spawn(async move {
            if let Err(error) = serve_one(stream, credential, mediad).await {
                tracing::debug!(%peer, %error, "gateway session closed");
            }
        });
    }
}

async fn serve_one(
    mut remote: TcpStream,
    credential: ProvisionedKey,
    mediad: SocketAddr,
) -> std::io::Result<()> {
    let session = tokio::time::timeout(
        Duration::from_secs(10),
        tcp::robot_handshake(&mut remote, credential.key(), credential.robot_id()),
    )
    .await
    .map_err(|_| std::io::Error::new(std::io::ErrorKind::TimedOut, "authentication timed out"))??;
    let local = TcpStream::connect(mediad).await?;
    tcp::proxy(local, remote, session).await
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
}
