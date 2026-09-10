//! Relay candidates, so a consumer that cannot reach this robot directly still can.
//!
//! `webrtcsink` gathers **host** candidates (this robot's own addresses) and **srflx** ones (what
//! a STUN server says its public address is). Between two peers on one network the host
//! candidates pair immediately. Between a robot behind a home router and a consumer behind
//! whatever a cloud provider gives a container, srflx-to-srflx needs both NATs to allow a hole to
//! be punched — often they do, and often enough they do not, and the failure looks like a session
//! that negotiates perfectly and carries nothing.
//!
//! A **TURN** server is a relay in the middle: a peer reserves an address on it and hands that out
//! as a `relay` candidate, and the other side simply sends there. It always works, at the cost of
//! somebody's bandwidth, which is why it is the last resort ICE tries rather than the first.
//!
//! # Only the robot offers one, and that is not a simplification
//!
//! A connection needs **one** relay candidate, not two: if this robot offers one, a consumer that
//! can reach the internet at all can use it. So the credentials live here and a consumer needs
//! none — which matters more than it sounds, because `aiortc`'s STUN client works where its TURN
//! client does not, so a Python consumer *cannot* be the side that relays.
//! `reachy_mini`'s #1182 established this and it is the same arrangement here.
//!
//! # The credentials are short-lived, and fetching them must never be in the way
//!
//! Hugging Face hosts a proxy that mints Cloudflare TURN credentials for an account, which is why
//! this needs the same token the relay signs in with and no new secret anywhere. They expire, so
//! a task refreshes them at half their lifetime.
//!
//! **[`Relays::uris`] never blocks and never fails**, and that is the whole design of this module.
//! Its only caller runs inside GStreamer's `consumer-added` signal, where the SDP offer for that
//! consumer is not generated until the handler returns — so an HTTP request there would delay
//! every connection, including the LAN ones that will never use a relay, by however long the
//! proxy takes to answer. It reads what the refresher last stored, or nothing.

use std::path::PathBuf;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use serde::Deserialize;

/// Hugging Face's TURN credentials proxy, as `reachy_mini` uses it.
pub const DEFAULT_TURN_ENDPOINT: &str = "https://turn.fastrtc.org/credentials";

/// How long to ask for the credentials to be valid.
const TTL: Duration = Duration::from_secs(600);

/// Refresh at half of [`TTL`], so a credential is replaced well before it expires.
const REFRESH_RATIO: f64 = 0.5;

/// How long to wait after a *transient* failure.
///
/// Only a transient one earns the short retry. A robot nobody has signed in has nothing to retry
/// for, and this task runs for the daemon's whole life — retrying that fast would put a line in
/// the journal every half minute forever.
const RETRY_AFTER_FAILURE: Duration = Duration::from_secs(30);

const HTTP_TIMEOUT: Duration = Duration::from_secs(10);

/// What the proxy answers with, of which two fields are read.
///
/// `iceServers` is camelCase on the wire — it is the browser's `RTCConfiguration` shape, which is
/// where these dicts are destined — while the keys *inside* a `meta` on the rendezvous are
/// snake_case. Two conventions in one system, and the cost of assuming either is a field that
/// silently deserialises to empty.
#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Credentials {
    #[serde(default)]
    ice_servers: Vec<IceServer>,
}

#[derive(Debug, Deserialize)]
struct IceServer {
    /// One URL or several; the proxy sends both shapes.
    urls: Urls,
    #[serde(default)]
    username: Option<String>,
    #[serde(default)]
    credential: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum Urls {
    One(String),
    Many(Vec<String>),
}

impl Urls {
    fn iter(&self) -> impl Iterator<Item = &String> {
        match self {
            Urls::One(url) => std::slice::from_ref(url).iter(),
            Urls::Many(urls) => urls.iter(),
        }
    }
}

/// The relay servers this robot currently holds, if any.
#[derive(Debug, Default)]
pub struct Relays {
    /// Replaced whole rather than mutated, so a reader sees the previous set or the new one and
    /// never half of either.
    uris: RwLock<Arc<[String]>>,
}

impl Relays {
    /// A holder with nothing in it, which is also what a robot has until the first refresh.
    pub fn empty() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// The `turn://` URIs to offer, or empty.
    ///
    /// **Never blocks and never panics.** `try_read` rather than `read`: the writer holds the lock
    /// for one assignment, so contention is close to impossible — and if it happens, offering
    /// host and srflx candidates for one consumer is a far better outcome than stalling a
    /// GStreamer signal handler, which stalls that consumer's offer.
    pub fn uris(&self) -> Arc<[String]> {
        match self.uris.try_read() {
            Ok(held) => Arc::clone(&held),
            Err(_) => Arc::from([] as [String; 0]),
        }
    }

    fn store(&self, uris: Vec<String>) {
        if let Ok(mut held) = self.uris.write() {
            *held = Arc::from(uris);
        }
    }
}

/// Keep [`Relays`] fresh for as long as this process runs.
///
/// Spawned once at startup, whatever the account state: a robot signed in later starts offering
/// relay candidates without a restart, which is the same property the relay itself has.
pub async fn maintain(relays: Arc<Relays>, token_path: PathBuf, endpoint: String) {
    let period = TTL.mul_f64(REFRESH_RATIO);
    let mut said_there_is_no_token = false;

    loop {
        let wait = match hf_robot_account::read_access_token(&token_path) {
            None => {
                // The steady state of a robot nobody has signed in. Said once, because it is not
                // news every five minutes for the life of the daemon.
                if !said_there_is_no_token {
                    said_there_is_no_token = true;
                    tracing::info!(
                        "no account token, so this robot offers no relay candidates; a login is \
                         what lets it be reached from a network that cannot punch a hole to it"
                    );
                }
                period
            }
            Some(token) => {
                said_there_is_no_token = false;
                match fetch(&endpoint, &token).await {
                    Ok(uris) if uris.is_empty() => {
                        // The proxy answered and offered no relay. Not worth hammering.
                        tracing::info!("the TURN proxy offered no relay servers");
                        period
                    }
                    Ok(uris) => {
                        tracing::info!(
                            servers = uris.len(),
                            hosts = ?uris.iter().map(|uri| hostname(uri)).collect::<Vec<_>>(),
                            "refreshed the relay credentials"
                        );
                        relays.store(uris);
                        period
                    }
                    // Best effort, always: a robot with no relay candidates is reachable from
                    // most places, and one that refused to stream because a proxy was down would
                    // be reachable from none.
                    Err(why) => {
                        tracing::warn!(%why, "could not fetch relay credentials");
                        RETRY_AFTER_FAILURE
                    }
                }
            }
        };
        tokio::time::sleep(wait).await;
    }
}

/// One request to the proxy, turned into the URIs `webrtcbin` takes.
async fn fetch(endpoint: &str, token: &str) -> Result<Vec<String>, String> {
    let client = reqwest::Client::builder()
        .timeout(HTTP_TIMEOUT)
        .build()
        .map_err(|e| format!("no HTTP client: {e}"))?;

    // The TTL in the URL rather than through `query`, which wants a `reqwest` feature this
    // workspace does not enable — and one integer needs no encoder.
    let separator = if endpoint.contains('?') { '&' } else { '?' };
    let url = format!("{endpoint}{separator}ttl={}", TTL.as_secs());
    let response = client
        .get(&url)
        .bearer_auth(token)
        .send()
        .await
        .map_err(|e| format!("GET {endpoint}: {}", because(&e)))?;
    if !response.status().is_success() {
        return Err(format!("GET {endpoint}: HTTP {}", response.status()));
    }
    let credentials: Credentials = response
        .json()
        .await
        .map_err(|e| format!("GET {endpoint}: {}", because(&e)))?;
    Ok(turn_uris(&credentials.ice_servers))
}

/// `{"urls", "username", "credential"}` to `turn://user:pass@host:port`.
///
/// `stun:` entries and entries with no credentials are skipped: `webrtcbin` takes a STUN server
/// through its own property, and `add-turn-server` rejects a URI with no userinfo.
fn turn_uris(servers: &[IceServer]) -> Vec<String> {
    let mut uris = Vec::new();
    for server in servers {
        let (Some(user), Some(secret)) = (&server.username, &server.credential) else {
            continue;
        };
        for url in server.urls.iter() {
            let (scheme, rest) = match url.split_once(':') {
                Some(parts) => parts,
                None => continue,
            };
            if !matches!(scheme.to_ascii_lowercase().as_str(), "turn" | "turns") || rest.is_empty()
            {
                continue;
            }
            // Percent-encoded, so a password containing `:`, `@` or `/` cannot corrupt the URI —
            // and these are generated secrets, so it will eventually contain one of them.
            uris.push(format!(
                "{scheme}://{}:{}@{rest}",
                encode(user),
                encode(secret)
            ));
        }
    }
    uris
}

/// The userinfo half of a URI, with everything that has meaning there escaped.
fn encode(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for byte in value.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => {
                out.push(byte as char);
            }
            other => out.push_str(&format!("%{other:02X}")),
        }
    }
    out
}

/// An error and everything under it, on one line.
///
/// **`reqwest`'s own message stops at "error sending request"**, and the half that matters is
/// underneath: a name that does not resolve, a refused connection and a certificate that does not
/// verify all print identically otherwise. This cost an afternoon of wondering whether a robot had
/// no network, when the answer was that the endpoint's whole domain had no DNS records —
/// unresolvable from the board and from three public resolvers alike.
fn because(error: &(dyn std::error::Error + 'static)) -> String {
    let mut out = error.to_string();
    let mut source = error.source();
    while let Some(cause) = source {
        // Repeated text is worse than none: `reqwest` wraps `hyper` wraps `io`, and each layer
        // often restates the one below it.
        let text = cause.to_string();
        if !out.contains(&text) {
            out.push_str(": ");
            out.push_str(&text);
        }
        source = cause.source();
    }
    out
}

/// A URI with its credentials removed, which is the only form that may be logged.
fn hostname(uri: &str) -> String {
    uri.rsplit_once('@')
        .map(|(_, host)| host.to_owned())
        .unwrap_or_else(|| uri.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn servers(json: &str) -> Vec<IceServer> {
        serde_json::from_str::<Credentials>(json)
            .unwrap()
            .ice_servers
    }

    /// What the proxy sends, in both shapes it sends it.
    #[test]
    fn credentials_become_uris_webrtcbin_accepts() {
        let uris = turn_uris(&servers(
            r#"{"iceServers":[
                {"urls":"stun:stun.cloudflare.com:3478"},
                {"urls":["turn:turn.cloudflare.com:3478?transport=udp",
                         "turns:turn.cloudflare.com:5349?transport=tcp"],
                 "username":"user-1","credential":"secret-1"}
            ]}"#,
        ));

        assert_eq!(
            uris,
            vec![
                "turn://user-1:secret-1@turn.cloudflare.com:3478?transport=udp",
                "turns://user-1:secret-1@turn.cloudflare.com:5349?transport=tcp",
            ],
            "one URI per URL, and the STUN entry is not one of them"
        );
    }

    /// A generated secret contains `:` and `/` sooner or later, and an unescaped one silently
    /// produces a URI naming the wrong host — with credentials in it, so it cannot be logged to
    /// find out.
    #[test]
    fn credentials_are_escaped_rather_than_interpolated() {
        let uris = turn_uris(&servers(
            r#"{"iceServers":[{"urls":"turn:relay:3478",
                "username":"a:b@c","credential":"p/q?r=s"}]}"#,
        ));
        assert_eq!(uris, vec!["turn://a%3Ab%40c:p%2Fq%3Fr%3Ds@relay:3478"]);
    }

    /// An entry with no credentials is not a relay this robot can offer.
    #[test]
    fn entries_without_credentials_are_skipped() {
        assert!(turn_uris(&servers(r#"{"iceServers":[{"urls":"turn:relay:3478"}]}"#)).is_empty());
        assert!(turn_uris(&servers(r#"{"iceServers":[]}"#)).is_empty());
    }

    /// An unreachable endpoint says *why* it was unreachable.
    #[tokio::test]
    async fn a_failure_names_its_cause_and_not_just_itself() {
        // A domain that cannot resolve, which is exactly what the real endpoint did.
        let error = fetch("https://turn.invalid./credentials", "hf_abc")
            .await
            .expect_err("`.invalid` does not resolve, by RFC 2606");
        assert!(
            error.to_lowercase().contains("dns") || error.to_lowercase().contains("resolve"),
            "the cause has to survive to the log line: {error}"
        );
    }

    /// **Credentials must never reach a log line.**
    #[test]
    fn only_the_host_is_loggable() {
        assert_eq!(
            hostname("turns://user-1:secret-1@turn.cloudflare.com:5349"),
            "turn.cloudflare.com:5349"
        );
        assert!(!hostname("turns://user-1:secret-1@relay:5349").contains("secret-1"));
        // A URI with no userinfo cannot leak one, and is logged whole.
        assert_eq!(hostname("turn:relay:3478"), "turn:relay:3478");
    }

    /// An empty holder answers, rather than making a caller handle "not yet".
    #[test]
    fn a_robot_that_has_fetched_nothing_offers_nothing() {
        let relays = Relays::empty();
        assert!(relays.uris().is_empty());

        relays.store(vec!["turn://u:p@relay:3478".to_owned()]);
        assert_eq!(relays.uris().len(), 1);

        // Replaced whole, so a reader never sees a mixture of two sets.
        relays.store(vec![
            "turn://u2:p2@relay:3478".to_owned(),
            "turns://u2:p2@relay:5349".to_owned(),
        ]);
        assert_eq!(relays.uris().len(), 2);
        assert!(relays.uris().iter().all(|uri| uri.contains("u2")));
    }

    /// The proxy refusing is not this robot's problem to solve, only to report.
    #[tokio::test]
    async fn a_proxy_that_refuses_leaves_the_robot_without_relays() {
        let app = axum::Router::new().route(
            "/credentials",
            axum::routing::get(|| async { axum::http::StatusCode::UNAUTHORIZED }),
        );
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let endpoint = format!("http://{}/credentials", listener.local_addr().unwrap());
        tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });

        let error = fetch(&endpoint, "hf_abc").await.expect_err("a 401");
        assert!(error.contains("401"), "{error}");
    }

    /// And the ordinary path, end to end against a proxy that answers what HF's answers.
    #[tokio::test]
    async fn a_refresh_stores_what_the_proxy_offers() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let asked = std::sync::Arc::new(AtomicUsize::new(0));
        let seen = std::sync::Arc::clone(&asked);
        let app = axum::Router::new().route(
            "/credentials",
            axum::routing::get(move |headers: axum::http::HeaderMap| {
                let seen = std::sync::Arc::clone(&seen);
                async move {
                    seen.fetch_add(1, Ordering::SeqCst);
                    assert_eq!(
                        headers.get("authorization").unwrap(),
                        "Bearer hf_abc",
                        "the robot's own token is what mints these"
                    );
                    axum::Json(serde_json::json!({
                        "iceServers": [{
                            "urls": ["turn:turn.cloudflare.com:3478?transport=udp"],
                            "username": "u", "credential": "p",
                        }],
                    }))
                }
            }),
        );
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let endpoint = format!("http://{}/credentials", listener.local_addr().unwrap());
        tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });

        let dir = tempfile::tempdir().unwrap();
        let token_path = dir.path().join("hf-token");
        std::fs::write(&token_path, r#"{"access_token":"hf_abc"}"#).unwrap();

        let relays = Relays::empty();
        let task = tokio::spawn(maintain(
            std::sync::Arc::clone(&relays),
            token_path,
            endpoint,
        ));
        for _ in 0..100 {
            if !relays.uris().is_empty() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        task.abort();

        assert_eq!(
            relays.uris().as_ref(),
            ["turn://u:p@turn.cloudflare.com:3478?transport=udp".to_owned()]
        );
        assert_eq!(asked.load(Ordering::SeqCst), 1, "once, not on a spin");
    }

    /// A robot with no account asks nobody for credentials.
    #[tokio::test]
    async fn no_token_means_no_request() {
        let dir = tempfile::tempdir().unwrap();
        let relays = Relays::empty();
        let task = tokio::spawn(maintain(
            std::sync::Arc::clone(&relays),
            dir.path().join("hf-token"),
            "http://127.0.0.1:1/credentials".to_owned(),
        ));
        tokio::time::sleep(Duration::from_millis(200)).await;
        task.abort();
        assert!(relays.uris().is_empty());
    }
}
