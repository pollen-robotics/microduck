//! The page, served by the daemon it drives.
//!
//! One route, one file, no build step. `http://<robot>:8080/` and there is nothing else to run —
//! which is the whole of it, and `webrtc-console.md` §1 is why it is worth a dependency and a
//! second port.
//!
//! **It deletes four problems rather than one.** No `python3 -m http.server`, so the instruction is
//! an address rather than two commands. No URL to type, because a page served by the robot knows
//! which robot it came from and derives its signalling target from `location.hostname`. Chrome's
//! Private Network Access check stops applying, because that check is on requests from a *public or
//! opaque* origin to a private address — and a page served from `192.168.x` has a private one, so
//! the failure that made `file://` unusable cannot happen. And page and binary ship together, so a
//! client from a checkout can no longer be pointed at a robot from a release.
//!
//! ## The port and the API version are filled in here, not carried by the page
//!
//! [`page`] substitutes both into the embedded page once, at startup. That is what makes `--port`
//! safe to change: nothing holds a second copy of it. The *host* is the browser's to fill —
//! `location.hostname`, which is the robot that served the page — so this is one number and not a
//! URL.
//!
//! The API version is the same idea applied to the skew banner. A page cannot compare versions
//! without knowing which one it speaks, and a literal in the page would be the second copy of
//! [`proto::API_VERSION`] — wrong on the day it is bumped, and wrong in the direction that reports
//! agreement. Substituted, a page served by a robot is a page that speaks that robot's release, and
//! the banner it shows is about a *checkout* page pointed at a robot, which is the case it exists
//! for.
//!
//! A page opened straight from the source tree keeps the token, reads it as `NaN`, and falls back to
//! `webrtcsink`'s own 8443. It also then knows it was *not* served by a robot, which is what lets it
//! tell "wrong host" apart from "the robot served me but its signalling port did not answer" — the
//! one failure two ports can produce, and §1.3's requirement that it reach a person as a diagnosis.
//!
//! ## Portable, unlike [`crate::pipeline`]
//!
//! Nothing here touches GStreamer, so it compiles and its tests run on a laptop. The substitution is
//! the part worth a test: a page served with the token still in it would connect to the wrong port
//! and say nothing about why.

use std::net::SocketAddr;

use anyhow::{Context, Result};
use axum::Router;
use axum::response::Html;
use axum::routing::get;
use duck_ipc_proto as proto;

/// The page as it sits in the source tree.
///
/// `include_str!` rather than a file read at request time, and that is a decision: installing it
/// under `current/webclient/` would cost an `--include` line in three places that already drift
/// (`_build-release.yml`, `dev.yml`, `scripts/dev-push.sh`), put a filesystem read behind a network
/// request in a unit running `ProtectSystem=strict`, and make "which page is this robot serving" a
/// question with two answers. The cost is a rebuild to change a stylesheet, which is the right trade
/// for a page that is part of the daemon's interface.
const PAGE: &str = include_str!("../webclient/index.html");

/// Where the page carries its signalling port until this module fills one in.
const PORT_TOKEN: &str = "{{SIGNALLING_PORT}}";

/// Where it carries the API version this release speaks.
const API_TOKEN: &str = "{{API_VERSION}}";

/// The page, with the signalling port and the API version filled in.
pub fn page(signalling_port: u32) -> String {
    PAGE.replace(PORT_TOKEN, &signalling_port.to_string())
        .replace(API_TOKEN, &proto::API_VERSION.to_string())
}

/// Serve `page` on `host:port` until the process ends.
///
/// Returns only on failure — a bind that was refused, or a listener that died. The caller decides
/// what that costs; in `mediad` it costs the page and not the video, because a robot that streams
/// and answers control calls with no console is a great deal better than one that does neither.
pub async fn serve(host: &str, port: u16, page: String, agent: crate::agent::State) -> Result<()> {
    let address: SocketAddr = format!("{host}:{port}")
        .parse()
        .with_context(|| format!("{host}:{port} is not an address to listen on"))?;
    let listener = tokio::net::TcpListener::bind(address)
        .await
        .with_context(|| format!("could not listen on {address}"))?;

    tracing::info!(%address, "serving the console and the agent socket");
    axum::serve(listener, router(page, agent))
        .await
        .context("the console's listener stopped")
}

/// The page, and the WebSocket a program drives the robot over.
///
/// One listener for both because they are one interface seen from two sides, and because a second
/// port is a second thing to configure, open and explain. `/` is for a person, `/agent` is for a
/// program, and `crate::agent` says why the second one exists.
fn router(page: String, agent: crate::agent::State) -> Router {
    Router::new()
        .route("/", get(move || std::future::ready(Html(page))))
        .route(
            "/agent",
            get(move |ws| crate::agent::upgrade(ws, agent.clone())),
        )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The whole of what this module does to the page.
    #[test]
    fn both_tokens_are_filled_in() {
        let served = page(8443);
        assert!(
            !served.contains(PORT_TOKEN),
            "the page went out with its port token still in it"
        );
        assert!(
            !served.contains(API_TOKEN),
            "the page went out with its API version token still in it"
        );
        assert!(served.contains("8443"));
        assert!(served.contains(&proto::API_VERSION.to_string()));
    }

    /// The banner exists to catch a page from a checkout against a robot from a release, so the
    /// version it compares against has to be this release's rather than a literal in the page.
    #[test]
    fn the_page_carries_the_api_token_this_module_replaces() {
        assert!(
            PAGE.contains(API_TOKEN),
            "the page has no {API_TOKEN} for the API version"
        );
    }

    /// **The page the robot serves must be in LAN mode, and the page in the repository must not
    /// be.**
    ///
    /// One file serves two hosts: substituting the port is what tells it a robot served it, and
    /// the copy pushed to the Space keeps the token so the page reaches for the rendezvous
    /// instead — a browser on an https page cannot open a `ws://` at all, so getting this
    /// backwards produces a console that connects to nothing and says nothing.
    /// `scripts/publish-console.sh` asserts the other half of it at deploy time.
    #[test]
    fn the_unsubstituted_page_is_the_remote_one() {
        assert!(
            PAGE.contains(PORT_TOKEN),
            "the page in the repository must carry the port token: it is what the Space copy \
             reads as `no robot served me`"
        );
        assert!(
            !page(8443).contains(PORT_TOKEN),
            "and the served page must not, or the robot would serve a page that ignores it"
        );
    }

    /// A non-default `--port` reaches the page, which is the reason the substitution exists at all:
    /// a page carrying a constant would still be dialling 8443.
    #[test]
    fn a_moved_port_reaches_the_page() {
        assert!(page(9000).contains("9000"));
    }

    /// The route, end to end, over a real socket: a `GET /` answers 200 with the page. Cheap, and
    /// it is the whole of what a browser does — the alternative is discovering on a board that the
    /// one route is mounted somewhere a browser does not ask.
    #[tokio::test]
    async fn the_route_answers_with_the_page() {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
            .await
            .expect("a loopback port");
        let address = listener.local_addr().expect("the port it took");
        tokio::spawn(async move {
            let (_video_tx, video) = tokio::sync::watch::channel(None);
            let agent = crate::agent::State {
                sockets: crate::upstream::Sockets::default(),
                video,
            };
            let _ = axum::serve(listener, router(page(8443), agent)).await;
        });

        let mut stream = tokio::net::TcpStream::connect(address)
            .await
            .expect("the server is listening");
        stream
            .write_all(b"GET / HTTP/1.1\r\nHost: robot\r\nConnection: close\r\n\r\n")
            .await
            .expect("wrote the request");
        let mut answer = String::new();
        stream
            .read_to_string(&mut answer)
            .await
            .expect("read the answer");

        assert!(
            answer.starts_with("HTTP/1.1 200"),
            "the console did not answer 200: {}",
            answer.lines().next().unwrap_or("nothing at all")
        );
        assert!(answer.contains("duck console"), "that was not the page");
        assert!(
            !answer.contains(PORT_TOKEN),
            "the page went out over the wire with its port token still in it"
        );
    }

    /// The token is in the page, spelled the way this module spells it. Without this, a rename on
    /// either side leaves a page that silently falls back to 8443 — which works on every robot
    /// until someone moves the port, and then fails with nothing to read.
    #[test]
    fn the_page_carries_the_token_this_module_replaces() {
        assert!(
            PAGE.contains(PORT_TOKEN),
            "the page has no {PORT_TOKEN} for the signalling port"
        );
    }
}
