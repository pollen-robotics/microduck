//! `duckctl open`: the robot's console, reached through its authenticated media gateway.
//!
//! Since daemon 0.19 `mediad` serves the console and its signalling on the robot's loopback only,
//! and the one thing a LAN can reach is `duck-gateway` on port 9443, which admits a peer only after
//! a Noise handshake keyed by the robot's setup code. A browser cannot speak Noise, so this process
//! does, the way the phone app's `media_proxy.rs` does:
//!
//! - it serves the console page itself on `127.0.0.1`, with the signalling port pointed at
//! - a WebSocket listener, also on `127.0.0.1`, which tunnels each connection to the gateway.
//!
//! **The page is this duckctl's copy, not the robot's.** The gateway reaches `mediad`'s signalling
//! and nothing else, so the robot's own page is out of reach; serving it would mean teaching the
//! gateway a second destination. The page speaks the signalling protocol, which moves slowly, and
//! carries the API version it was built with, so a duckctl far from the robot's release says so in
//! the page's own version banner. `/frame` snapshots are unavailable for the same reason.
//!
//! # Why the listener checks a token and an Origin
//!
//! Loopback is not private to the person who ran this. Browsers apply no same-origin policy to
//! WebSockets, so any page open in any tab could connect to `ws://127.0.0.1:<port>` and drive the
//! robot through a tunnel this process had already authenticated; so could any local process or
//! user. A port is not a secret — it is scannable in milliseconds. So each run makes a random token
//! that only the page served here carries (another origin can send a request to the page, but the
//! browser will not let it read the answer), and a connection is tunnelled only if its path holds
//! that token **and** its `Origin` is this page's. Either check alone would do against a web page;
//! both together cost nothing, and the token is what stops a local process that forges headers.

use std::net::{IpAddr, SocketAddr};
use std::time::Duration;

use duck_auth::tcp;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};

use crate::setup_code::SetupCode;

/// `duck-gateway`'s port. One home on the robot side (`duck-gateway --listen`), one here.
const GATEWAY_PORT: u16 = 9443;

const PAGE: &str = include_str!("../../mediad/webclient/index.html");

/// A request head larger than this is not a browser asking for a page or an upgrade.
const MAX_HEAD: usize = 16 * 1024;

/// What the page needs to reach the tunnel: its port, and the path token that admits it.
struct Tunnel {
    port: u16,
    token: String,
}

/// The console page, with the placeholders `mediad`'s `web.rs` fills filled the same way, except
/// that the signalling port is the local tunnel and every WebSocket URL carries the token.
fn page(tunnel: &Tunnel) -> String {
    PAGE.replace("{{SIGNALLING_PORT}}", &tunnel.port.to_string())
        .replace("{{API_VERSION}}", &duck_ipc_proto::API_VERSION.to_string())
        // The page builds `ws://${location.hostname}:${SIGNALLING_PORT}`, with no path. The token
        // has to ride in the URL — a page cannot set headers on a WebSocket — so it is appended to
        // that one template, and `token_in_page` pins that the template is still there to append to.
        .replace(
            SIGNALLING_TEMPLATE,
            &format!("{SIGNALLING_TEMPLATE}/{}", tunnel.token),
        )
}

/// The line of the page this module rewrites. Kept as a constant so a test fails, rather than the
/// console silently connecting without its token, the day the page is reworded.
const SIGNALLING_TEMPLATE: &str = "ws://${location.hostname || \"localhost\"}:${SIGNALLING_PORT}";

/// Serve the console for the robot at `robot` until interrupted.
///
/// Returns the URL once both listeners are up, through `ready`, so the caller decides whether to
/// print it or open a browser; then serves until Ctrl-C.
pub async fn serve(
    robot: IpAddr,
    code: SetupCode,
    ready: impl FnOnce(&str),
) -> Result<(), Box<dyn std::error::Error>> {
    let gateway = SocketAddr::new(robot, GATEWAY_PORT);
    // Prove the code before telling anyone to open a page: a wrong code or a robot with no gateway
    // is a better error here than a page that loads and then shows a dead signalling socket.
    let probe = connect(gateway, &code).await?;
    drop(probe);

    let ws = TcpListener::bind(("127.0.0.1", 0)).await?;
    let http = TcpListener::bind(("127.0.0.1", 0)).await?;
    let tunnel = Tunnel {
        port: ws.local_addr()?.port(),
        token: token()?,
    };
    let origin = format!("http://127.0.0.1:{}", http.local_addr()?.port());
    let body = page(&tunnel);
    let url = format!("{origin}/");
    ready(&url);

    let pages = async {
        loop {
            let (stream, _) = http.accept().await?;
            let body = body.clone();
            tokio::spawn(async move {
                let _ = serve_page(stream, &body).await;
            });
        }
        #[allow(unreachable_code)]
        Ok::<(), std::io::Error>(())
    };
    let tunnels = async {
        loop {
            let (stream, peer) = ws.accept().await?;
            let code = code.clone();
            let token = tunnel.token.clone();
            let origin = origin.clone();
            tokio::spawn(async move {
                if let Err(error) = serve_tunnel(stream, gateway, &code, &token, &origin).await {
                    eprintln!("console connection from {peer} ended: {error}");
                }
            });
        }
        #[allow(unreachable_code)]
        Ok::<(), std::io::Error>(())
    };
    tokio::select! {
        result = pages => result?,
        result = tunnels => result?,
        _ = tokio::signal::ctrl_c() => eprintln!("console closed"),
    }
    Ok(())
}

/// 128 random bits, hex. The OS's generator, because this is an access credential.
fn token() -> Result<String, String> {
    let mut bytes = [0u8; 16];
    getrandom::fill(&mut bytes).map_err(|e| format!("no randomness for the console token: {e}"))?;
    Ok(bytes.iter().map(|b| format!("{b:02x}")).collect())
}

/// Open an authenticated session with the robot's gateway.
async fn connect(
    gateway: SocketAddr,
    code: &SetupCode,
) -> Result<(TcpStream, duck_auth::Session), String> {
    let mut stream = tokio::time::timeout(Duration::from_secs(5), TcpStream::connect(gateway))
        .await
        .map_err(|_| format!("{gateway} did not answer; is this machine on the robot's network?"))?
        .map_err(|e| match e.kind() {
            std::io::ErrorKind::ConnectionRefused => format!(
                "{gateway} refused: the robot's media gateway is not running. A robot with no \
                 setup code has none — `journalctl -u duck-gateway -b` on it says which."
            ),
            _ => format!("could not reach {gateway}: {e}"),
        })?;
    let session = tokio::time::timeout(
        Duration::from_secs(10),
        tcp::app_handshake(&mut stream, &code.key, &code.id),
    )
    .await
    .map_err(|_| format!("{gateway} did not finish authenticating"))?
    .map_err(|_| {
        format!(
            "{gateway} did not accept this machine's setup code for {}. A code from before the \
             robot's ID EEPROM was rewritten is the usual cause; `duckctl setup-code add` replaces it.",
            code.id
        )
    })?;
    Ok((stream, session))
}

/// Read one request head, up to the blank line.
async fn read_head(stream: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let mut head = Vec::with_capacity(1024);
    while !head.windows(4).any(|w| w == b"\r\n\r\n") {
        if head.len() > MAX_HEAD {
            return Err(std::io::Error::other("request head too large"));
        }
        let mut chunk = [0u8; 1024];
        let n = stream.read(&mut chunk).await?;
        if n == 0 {
            return Err(std::io::Error::other("connection closed mid-request"));
        }
        head.extend_from_slice(&chunk[..n]);
    }
    Ok(head)
}

async fn serve_page(mut stream: TcpStream, body: &str) -> std::io::Result<()> {
    let head = read_head(&mut stream).await?;
    let response = if head.starts_with(b"GET / ") {
        format!(
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\n\
             Cache-Control: no-store\r\nConnection: close\r\n\r\n{body}",
            body.len()
        )
    } else {
        // `/frame` among them: the gateway reaches signalling only. See the module comment.
        let text = "Not served by duckctl's console.\n";
        format!(
            "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: {}\r\n\
             Connection: close\r\n\r\n{text}",
            text.len()
        )
    };
    stream.write_all(response.as_bytes()).await?;
    stream.shutdown().await
}

/// Check one WebSocket upgrade, and if it is the page's, tunnel it to the robot.
///
/// The upgrade itself is forwarded to `mediad` with the token path stripped, so the signalling
/// server sees the `GET /` it always has.
async fn serve_tunnel(
    mut local: TcpStream,
    gateway: SocketAddr,
    code: &SetupCode,
    token: &str,
    origin: &str,
) -> Result<(), String> {
    let head = read_head(&mut local).await.map_err(|e| e.to_string())?;
    let Some(forward) = admit(&head, token, origin) else {
        let _ = local
            .write_all(b"HTTP/1.1 403 Forbidden\r\nConnection: close\r\nContent-Length: 0\r\n\r\n")
            .await;
        return Err("refused: not this console's page (wrong token or Origin)".into());
    };
    let (remote, mut session) = connect(gateway, code).await?;
    let mut remote = remote;
    let sealed = session
        .seal(&forward)
        .map_err(|_| "could not seal the upgrade")?;
    tcp::write_record(&mut remote, &sealed)
        .await
        .map_err(|e| e.to_string())?;
    tcp::proxy(local, remote, session)
        .await
        .map_err(|e| e.to_string())
}

/// The upgrade to forward, if `head` is this page's: path `/<token>` and `Origin` this page's.
fn admit(head: &[u8], token: &str, origin: &str) -> Option<Vec<u8>> {
    let expected = format!("GET /{token} HTTP/1.1\r\n");
    if !head.starts_with(expected.as_bytes()) {
        return None;
    }
    let text = std::str::from_utf8(head).ok()?;
    let origin_ok = text.lines().any(|line| {
        line.split_once(':').is_some_and(|(name, value)| {
            name.eq_ignore_ascii_case("origin") && value.trim() == origin
        })
    });
    if !origin_ok {
        return None;
    }
    let mut forward = b"GET / HTTP/1.1\r\n".to_vec();
    forward.extend_from_slice(&head[expected.len()..]);
    Some(forward)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORIGIN: &str = "http://127.0.0.1:4242";

    fn upgrade(path: &str, origin: Option<&str>) -> Vec<u8> {
        let mut head =
            format!("GET {path} HTTP/1.1\r\nHost: 127.0.0.1:5000\r\nUpgrade: websocket\r\n");
        if let Some(origin) = origin {
            head.push_str(&format!("Origin: {origin}\r\n"));
        }
        head.push_str("\r\n");
        head.into_bytes()
    }

    /// The whole point of the token: if the page were reworded so the replace found nothing, the
    /// console would load, connect without a token, be refused, and look like a dead robot.
    #[test]
    fn token_in_page() {
        assert!(
            PAGE.contains(SIGNALLING_TEMPLATE),
            "mediad's page no longer builds its URL this way"
        );
        let page = page(&Tunnel {
            port: 5000,
            token: "abc123".into(),
        });
        assert!(page.contains(&format!("{SIGNALLING_TEMPLATE}/abc123")));
        assert!(!page.contains("{{SIGNALLING_PORT}}"));
    }

    #[test]
    fn the_pages_own_upgrade_is_admitted_and_its_token_stripped() {
        let forward = admit(&upgrade("/abc123", Some(ORIGIN)), "abc123", ORIGIN).unwrap();
        assert!(forward.starts_with(b"GET / HTTP/1.1\r\n"));
        assert!(!String::from_utf8(forward).unwrap().contains("abc123"));
    }

    /// A web page in another tab: it can guess the port, not the token.
    #[test]
    fn no_token_is_refused() {
        assert!(admit(&upgrade("/", Some(ORIGIN)), "abc123", ORIGIN).is_none());
        assert!(admit(&upgrade("/abc12", Some(ORIGIN)), "abc123", ORIGIN).is_none());
    }

    /// A page from elsewhere that learned the token still carries its own Origin, which a browser
    /// will not let it forge.
    #[test]
    fn a_foreign_or_missing_origin_is_refused() {
        assert!(
            admit(
                &upgrade("/abc123", Some("https://evil.example")),
                "abc123",
                ORIGIN
            )
            .is_none()
        );
        assert!(admit(&upgrade("/abc123", None), "abc123", ORIGIN).is_none());
    }

    #[test]
    fn tokens_are_unique_and_long() {
        let (a, b) = (token().unwrap(), token().unwrap());
        assert_ne!(a, b);
        assert_eq!(a.len(), 32);
    }
}
