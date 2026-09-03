//! Which Hugging Face account this robot belongs to.
//!
//! A duck on a LAN needs no account: `mediad` serves the console, a browser on the same wifi
//! reaches it, and nothing here is involved. An account is what makes a robot reachable from
//! *outside* the LAN — the relay proves to a rendezvous service that this robot belongs to an
//! account, the service shows a client only its own robots, and the pair of those is the
//! authorisation a bridged session arrives with. `docs/design/remote-access-design.md` owns that
//! argument; this module owns the credential.
//!
//! # The flow is RFC 8628, because the robot has no browser
//!
//! `reachy_mini` started with authorization code + PKCE and HF's callback pointed at a URL on the
//! robot, which needs a redirect URI registered per hostname and a browser that can resolve the
//! robot. A duck asks Hugging Face for a code instead, says *"type `M8HJ-FMGN` at
//! hf.co/oauth/device"*, and polls. The phone that approves it need not be able to reach the
//! robot at all.
//!
//! Three consequences shape the code below:
//!
//! - **[`Account::login`] returns a code, not a token.** The waiting happens in a task this
//!   process owns, and a client comes back to [`Account::status`]. A login that reported success
//!   by holding the connection open would work from a laptop and fail from the device it is for:
//!   a phone that opens a browser backgrounds itself, and iOS tears the GATT link down.
//! - **The client displays the code, and the code has to be typed.** HF sends no
//!   `verification_uri_complete` and its device page prefills nothing, so no URL carries the code
//!   — which makes showing it the client's whole job. Whether a client also *opens* the page
//!   depends on what it is: `remote-access-design.md` §2.1 has three answers, for a robot, a
//!   terminal and a phone.
//! - **A token expires in 30 days and its refresh token rotates.** So [`maintain`] renews well
//!   before expiry, and the store is written atomically — see [`Store::save`] for the one window
//!   that cannot be closed.
//!
//! # The client id is Hugging Face's own, and that is a decision to revisit
//!
//! [`CLIENT_ID`] is the first-party device-code client `huggingface_hub` ships, so this needs no
//! OAuth app registered anywhere and works on a robot that has never met Pollen. What comes back
//! is a token with **every scope HF grants** — `write-repos`, `manage-repos`, `jobs`,
//! `read-billing` — because that client takes no `scope` parameter. A duck therefore holds a
//! credential that can push to its owner's repositories, which is worse than it needs to be for
//! something whose whole job is proving an identity to a rendezvous service.
//!
//! The alternative is a public device-code client registered to `pollen-robotics` with
//! `openid profile read-repos`, which is one constant here and one click by an org admin. Worth
//! doing before a duck ships; not worth blocking on now. `remote-access-design.md` §2.4.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;

use crate::Error;
use crate::proto;
use crate::source::http;

/// Hugging Face's first-party device-code OAuth client.
///
/// `huggingface_hub`'s `DEVICE_CODE_OAUTH_CLIENT_ID`, which is what `hf auth login` uses. Public
/// — no secret, and none can be sent: HF refuses the device grant for a *confidential* client
/// unless it is given the secret, which is why `reachy_mini`'s own OAuth app cannot be used here.
///
/// See the module header for what this costs in scopes and what replaces it.
pub const CLIENT_ID: &str = "26be6b09-91c5-47da-9861-d2d2bb7a7e36";

/// Hugging Face, overridable so tests can point the whole flow at a local server.
///
/// `HF_ENDPOINT` is the variable `huggingface_hub` itself reads, so a board pointed at a mirror
/// for one reason is pointed at it for all of them.
fn endpoint() -> String {
    std::env::var("HF_ENDPOINT")
        .ok()
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| "https://huggingface.co".to_string())
}

/// Where the credential lives.
///
/// **Not in `robotd.toml`.** Every mechanism that exists for that file is wrong for a secret:
/// `robotctl configure --list` prints what a robot changes, the config editor shows the whole
/// file, and "what was changed on this robot" is a report we generate. A bearer token would be in
/// all three.
pub const DEFAULT_PATH: &str = "/etc/robot/hf-token";

/// The group that may read the token file.
///
/// `mediad` runs as `User=mediad` with `SupplementaryGroups=robot`, and it is the process that
/// needs the token — it is the one holding the relay. `updaterd` runs as root and writes it. So
/// the file is `root:robot` and `0640`: readable by the daemons that belong to this robot, and by
/// nothing else with a login on the board.
const TOKEN_GROUP: &str = "robot";

/// How long an HTTP round trip to Hugging Face gets.
const HTTP_TIMEOUT: Duration = Duration::from_secs(20);

/// RFC 8628's fallback when the server sends no `interval`. HF sends none.
const DEFAULT_POLL_INTERVAL: u64 = 5;

/// RFC 8628 requires `expires_in`; defaulted defensively so a poll loop stays bounded.
const DEFAULT_EXPIRES_IN: u64 = 900;

/// How often [`maintain`] wakes to look at the token's remaining life.
const MAINTAIN_INTERVAL: Duration = Duration::from_secs(6 * 60 * 60);

/// Renew a token with less than this left.
///
/// HF issues 30 days, so this refreshes at three-quarters of the way through the token's life and
/// leaves a week of retries. A robot switched off for longer than the whole 30 days cannot be
/// saved by any margin — it comes back needing a login, which [`proto::Account::token_expires_in`]
/// going negative is how a client says so.
const REFRESH_WHEN_UNDER: Duration = Duration::from_secs(7 * 24 * 60 * 60);

// ── the stored credential ────────────────────────────────────────────────────

/// What is on disk.
///
/// The shape mirrors what the token endpoint returns, plus the two things it does not: an
/// absolute expiry (the response gives a duration, and a duration is meaningless after a reboot)
/// and the username, so [`Account::status`] answers without a network round trip.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Stored {
    pub access_token: String,
    /// Absent when HF issues none, which would make the token unrenewable rather than broken.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub refresh_token: Option<String>,
    /// Unix seconds. Absent means "HF did not say", which is treated as not expiring.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expires_at: Option<i64>,
    /// As `/oauth/userinfo` reported it at login.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub username: Option<String>,
}

impl Stored {
    /// Seconds until the access token expires; negative once it has. `i64::MAX` when unknown.
    fn expires_in(&self) -> i64 {
        match self.expires_at {
            None => i64::MAX,
            Some(at) => at.saturating_sub(now_secs()),
        }
    }
}

/// The token file, and the two operations anything has on it.
#[derive(Debug, Clone)]
pub struct Store {
    path: PathBuf,
}

impl Store {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// What is stored, or `None` when the robot belongs to nobody.
    ///
    /// A file that will not parse is `None` with a warning rather than an error: the recovery for
    /// a corrupt credential is signing in again, and a daemon that refuses to start — or a status
    /// call that fails — because of one would make that harder rather than safer.
    pub fn load(&self) -> Option<Stored> {
        let bytes = std::fs::read(&self.path).ok()?;
        match serde_json::from_slice::<Stored>(&bytes) {
            Ok(stored) => Some(stored),
            Err(e) => {
                tracing::warn!(
                    path = %self.path.display(),
                    error = %e,
                    "the stored account credential does not parse; treating this robot as signed out"
                );
                None
            }
        }
    }

    /// Replace the credential, atomically, readable only by root and the `robot` group.
    ///
    /// **The one window this cannot close.** HF rotates the refresh token on every refresh, so
    /// between "HF issued a new pair" and "the new pair is on disk" the old refresh token is
    /// already dead. A power cut in that window leaves a robot holding a credential HF will not
    /// renew, and no ordering here fixes it — the rotation happened on their side. It surfaces as
    /// a refresh failure in [`proto::AccountStatusResult::last_error`] and is fixed by signing in
    /// again, which is why [`maintain`] starts trying a week early rather than on the last day.
    pub fn save(&self, stored: &Stored) -> Result<(), Error> {
        let bytes = serde_json::to_vec_pretty(stored).map_err(|e| Error::Io {
            path: self.path.clone(),
            source: std::io::Error::other(e),
        })?;

        // Written 0600 first and relaxed to 0640 after the group is set, so the file is never
        // group-readable while it is owned by root's default group.
        write_private(&self.path, &bytes)?;
        if let Some(gid) = crate::unix::group_id(TOKEN_GROUP) {
            set_group(&self.path, gid)?;
            set_mode(&self.path, 0o640)?;
        } else {
            // No `robot` group means a developer's laptop or a half-provisioned board. Root can
            // still read it, so a login is not lost; `mediad` will not be able to, and saying so
            // once is better than a relay that cannot explain why it has no token.
            tracing::warn!(
                group = TOKEN_GROUP,
                path = %self.path.display(),
                "no such group, so the token stays root-only — mediad will not be able to read it"
            );
        }
        Ok(())
    }

    /// Forget the credential. Returns who it belonged to, if anyone.
    pub fn clear(&self) -> Result<Option<String>, Error> {
        let was = self.load().and_then(|s| s.username);
        match std::fs::remove_file(&self.path) {
            Ok(()) => Ok(was),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(Error::Io {
                path: self.path.clone(),
                source: e,
            }),
        }
    }
}

// ── the Hugging Face half ────────────────────────────────────────────────────

/// A started device authorization.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceCode {
    pub device_code: String,
    pub user_code: String,
    pub verification_uri: String,
    pub verification_uri_complete: String,
    pub expires_in: u64,
    pub interval: u64,
}

/// What `POST /oauth/device` answers, before normalisation.
#[derive(Debug, Deserialize)]
struct DeviceCodeResponse {
    device_code: String,
    user_code: String,
    verification_uri: String,
    verification_uri_complete: Option<String>,
    expires_in: Option<u64>,
    interval: Option<u64>,
}

/// What `POST /oauth/token` answers on success.
#[derive(Debug, Clone, Deserialize)]
pub struct TokenResponse {
    pub access_token: String,
    #[serde(default)]
    pub refresh_token: Option<String>,
    #[serde(default)]
    pub expires_in: Option<u64>,
}

/// What `POST /oauth/token` answers while nobody has approved yet.
#[derive(Debug, Deserialize)]
struct OAuthError {
    error: String,
    #[serde(default)]
    error_description: Option<String>,
}

/// One poll of the token endpoint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Poll {
    /// Nobody has approved it yet.
    Pending,
    /// The server wants a longer interval. RFC 8628 §3.5: add five seconds.
    SlowDown,
    /// Approved.
    Token(Box<TokenResponse>),
    /// The user said no, or the code ran out. Either way, start again.
    Refused(String),
    /// Anything inconclusive — a 5xx, a proxy's error page, a network blip. **Not** a failure:
    /// RFC 8628 §3.5 says keep polling until the code expires, and the deadline bounds the wait.
    Inconclusive(String),
}

impl PartialEq for TokenResponse {
    fn eq(&self, other: &Self) -> bool {
        self.access_token == other.access_token
            && self.refresh_token == other.refresh_token
            && self.expires_in == other.expires_in
    }
}
impl Eq for TokenResponse {}

/// Ask Hugging Face to start a device authorization.
pub async fn request_device_code(
    client: &reqwest::Client,
    base: &str,
) -> Result<DeviceCode, Error> {
    let url = format!("{base}/oauth/device");
    let response = client
        .post(&url)
        .timeout(HTTP_TIMEOUT)
        .form(&[("client_id", CLIENT_ID)])
        .send()
        .await
        .map_err(|e| Error::Network(format!("POST {url}: {e}")))?;

    let status = response.status();
    let body = response
        .text()
        .await
        .map_err(|e| Error::Network(format!("reading {url}: {e}")))?;
    if !status.is_success() {
        return Err(Error::Network(format!(
            "POST {url}: HTTP {status}: {}",
            body.chars().take(300).collect::<String>()
        )));
    }

    let raw: DeviceCodeResponse = serde_json::from_str(&body).map_err(|e| {
        Error::Network(format!(
            "POST {url}: could not parse the device code response: {e}"
        ))
    })?;

    // HF sends neither `interval` nor `verification_uri_complete`, so both are defaulted here —
    // the same normalisation `huggingface_hub` does, in the same place, so nothing downstream has
    // to know which fields a server bothered with.
    //
    // **The fallback is the plain URI, and the code still has to be typed.** An earlier version
    // of this appended `?user_code=`, on the strength of a claim in `reachy_mini`'s setup notes
    // that `huggingface_hub` synthesises that form. It does not — it falls back to
    // `verification_uri` unchanged — and Hugging Face's device page ignores the parameter: it
    // survives the login redirect and prefills nothing, which was confirmed in a browser. A URL
    // carrying a query the other end drops is worse than no query, because it reads like a
    // promise the page then breaks.
    //
    // The field stays because it is the protocol's, and a server that starts sending a real one
    // is then used without a change here.
    let verification_uri_complete = raw
        .verification_uri_complete
        .unwrap_or_else(|| raw.verification_uri.clone());
    Ok(DeviceCode {
        device_code: raw.device_code,
        user_code: raw.user_code,
        verification_uri: raw.verification_uri,
        verification_uri_complete,
        expires_in: raw.expires_in.unwrap_or(DEFAULT_EXPIRES_IN),
        interval: raw.interval.unwrap_or(DEFAULT_POLL_INTERVAL),
    })
}

/// Poll the token endpoint once.
pub async fn poll_token(client: &reqwest::Client, base: &str, device_code: &str) -> Poll {
    let url = format!("{base}/oauth/token");
    let response = client
        .post(&url)
        .timeout(HTTP_TIMEOUT)
        .form(&[
            ("grant_type", "urn:ietf:params:oauth:grant-type:device_code"),
            ("client_id", CLIENT_ID),
            ("device_code", device_code),
        ])
        .send()
        .await;

    let response = match response {
        Ok(response) => response,
        Err(e) => return Poll::Inconclusive(format!("POST {url}: {e}")),
    };
    // A 5xx is a Hugging Face problem, not an answer about this login.
    if response.status().is_server_error() {
        return Poll::Inconclusive(format!("POST {url}: HTTP {}", response.status()));
    }
    let body = match response.text().await {
        Ok(body) => body,
        Err(e) => return Poll::Inconclusive(format!("reading {url}: {e}")),
    };

    if let Ok(token) = serde_json::from_str::<TokenResponse>(&body) {
        return Poll::Token(Box::new(token));
    }
    match serde_json::from_str::<OAuthError>(&body) {
        Ok(err) => classify(&err),
        // JSON without an `error` member, or not JSON at all: a gateway's error page. Transient.
        Err(_) => Poll::Inconclusive(format!(
            "POST {url}: unexpected answer: {}",
            body.chars().take(200).collect::<String>()
        )),
    }
}

/// Which OAuth errors end a login and which are the login working normally.
fn classify(err: &OAuthError) -> Poll {
    let detail = err
        .error_description
        .clone()
        .unwrap_or_else(|| err.error.clone());
    match err.error.as_str() {
        "authorization_pending" => Poll::Pending,
        "slow_down" => Poll::SlowDown,
        "expired_token" => Poll::Refused(
            "the code expired before it was approved — start the login again".to_string(),
        ),
        "access_denied" => Poll::Refused("the login was refused on Hugging Face".to_string()),
        // An OAuth error we do not know is still an answer about this login rather than a blip:
        // `invalid_client` and `invalid_grant` will not fix themselves by being asked again.
        other => Poll::Refused(format!("Hugging Face said {other}: {detail}")),
    }
}

/// Exchange a refresh token for a new pair. HF rotates the refresh token, so the answer's is the
/// one to keep.
pub async fn refresh(
    client: &reqwest::Client,
    base: &str,
    refresh_token: &str,
) -> Result<TokenResponse, Error> {
    let url = format!("{base}/oauth/token");
    let response = client
        .post(&url)
        .timeout(HTTP_TIMEOUT)
        .form(&[
            ("grant_type", "refresh_token"),
            ("client_id", CLIENT_ID),
            ("refresh_token", refresh_token),
        ])
        .send()
        .await
        .map_err(|e| Error::Network(format!("POST {url}: {e}")))?;
    let body = response
        .text()
        .await
        .map_err(|e| Error::Network(format!("reading {url}: {e}")))?;
    serde_json::from_str::<TokenResponse>(&body).map_err(|_| {
        let detail = serde_json::from_str::<OAuthError>(&body)
            .map(|e| format!("{}: {}", e.error, e.error_description.unwrap_or_default()))
            .unwrap_or_else(|_| body.chars().take(200).collect());
        Error::Network(format!("could not refresh the account token: {detail}"))
    })
}

/// Who a token belongs to.
///
/// `/oauth/userinfo` rather than decoding the `id_token`: one round trip against an endpoint that
/// is part of the flow already, versus a JWT parser and a JWKS fetch for the same string.
pub async fn userinfo(
    client: &reqwest::Client,
    base: &str,
    access_token: &str,
) -> Result<String, Error> {
    #[derive(Deserialize)]
    struct UserInfo {
        /// The handle — `PierreRouanet`. `name` is the display name and can be anything.
        preferred_username: Option<String>,
        name: Option<String>,
    }

    let url = format!("{base}/oauth/userinfo");
    let response = client
        .get(&url)
        .timeout(HTTP_TIMEOUT)
        .bearer_auth(access_token)
        .send()
        .await
        .map_err(|e| Error::Network(format!("GET {url}: {e}")))?;
    let status = response.status();
    let body = response
        .text()
        .await
        .map_err(|e| Error::Network(format!("reading {url}: {e}")))?;
    if !status.is_success() {
        return Err(Error::Network(format!("GET {url}: HTTP {status}")));
    }
    let info: UserInfo = serde_json::from_str(&body)
        .map_err(|e| Error::Network(format!("GET {url}: could not parse the answer: {e}")))?;
    info.preferred_username
        .or(info.name)
        .ok_or_else(|| Error::Network(format!("GET {url}: no username in the answer")))
}

// ── what `updaterd` serves ───────────────────────────────────────────────────

/// A login in flight.
#[derive(Debug, Clone)]
struct Pending {
    login: proto::AccountLoginResult,
    /// When the code stops being good for anything, so `status` can count down.
    deadline: SystemTime,
}

/// The account, as the three `account.*` calls see it.
#[derive(Debug)]
pub struct Account {
    store: Store,
    /// Where Hugging Face is. Read once at construction rather than per call, so nothing below
    /// this struct reads the environment — which is what lets a test serve a fake one without
    /// mutating a variable the whole process shares.
    endpoint: String,
    /// `Mutex` rather than a channel because there is at most one login at a time and both
    /// `login` and `status` need to see it; the lock is held for a field read, and deliberately
    /// never across a network call — `status` is polled *while* a login is in flight, so
    /// anything that made it queue behind an HTTP round trip would make a wizard look stuck.
    pending: Mutex<Option<Pending>>,
    /// Which login is the current one.
    ///
    /// A flow lives in a spawned task that outlives the call that started it, and two things can
    /// happen to it while it waits: `--force` starts another, or `logout` says the robot belongs
    /// to nobody. In both cases the old task is still holding a device code Hugging Face will
    /// happily approve, and without this it would write that approval to the store — a robot
    /// signed back in a minute after being signed out, or signed in as the account somebody just
    /// replaced. Each flow carries the number it was started with and does nothing at all if it
    /// is no longer the current one.
    generation: std::sync::atomic::AtomicU64,
    /// Held for the whole of starting a login, which the one above cannot be.
    ///
    /// Two callers arriving together both read `pending` as empty, both ask Hugging Face for a
    /// code, and both spawn a poller: the store then holds whichever approval landed last, which
    /// is neither predictable nor explicable to whoever was reading the other code. The window is
    /// the round trip to `/oauth/device`, so the guard has to cover it — and it is its own lock
    /// rather than `pending` because `status` must not wait on that round trip.
    starting: Mutex<()>,
    last_error: Mutex<Option<String>>,
}

impl Account {
    /// Infallible, so a daemon whose TLS stack will not build still starts and still answers
    /// `account.status` — which is how anybody finds out. The HTTP client is built per operation
    /// instead: `status` and `logout` need none at all, and the one operation that polls in a
    /// loop wants a client that lives exactly as long as the loop.
    pub fn new(store: Store) -> Self {
        Self::with_endpoint(store, endpoint())
    }

    /// As [`Self::new`], against a named endpoint. For tests, and for a board pointed at a
    /// mirror by something other than `HF_ENDPOINT`.
    pub fn with_endpoint(store: Store, endpoint: String) -> Self {
        Self {
            store,
            endpoint,
            pending: Mutex::new(None),
            generation: std::sync::atomic::AtomicU64::new(0),
            starting: Mutex::new(()),
            last_error: Mutex::new(None),
        }
    }

    pub fn store(&self) -> &Store {
        &self.store
    }

    /// Start a device-code login, and answer with the code to show somebody.
    ///
    /// The polling runs in a task this returns without waiting for. `force` is what it takes to
    /// replace an account the robot already belongs to — see [`proto::AccountLoginParams::force`].
    pub async fn login(
        self: &std::sync::Arc<Self>,
        force: bool,
    ) -> Result<proto::AccountLoginResult, Error> {
        if let Some(stored) = self.store.load()
            && !force
        {
            let who = stored.username.unwrap_or_else(|| "another account".into());
            return Err(Error::AlreadySignedIn(who));
        }

        // One login at a time, and the gate has to hold across the round trip below rather than
        // only across the check — see [`Self::starting`] for what two of them leave behind.
        // `try_lock`, so the second caller is told so now instead of queueing behind somebody
        // else's twenty-second timeout and then starting a login nobody is waiting for.
        let _starting = self.starting.try_lock().map_err(|_| Error::LoginInFlight)?;
        // A code that is still good refuses a second login — **unless `force`**. Without that
        // exception the only ways out of a code nobody is going to approve are waiting five
        // minutes and `logout`, and `logout` throws away a working credential to clear a
        // *pending* one, which is a remedy worse than the problem. `force` already means "replace
        // what this robot belongs to"; replacing an attempt at it is the smaller version.
        if !force {
            let pending = self.pending.lock().await;
            if let Some(pending) = pending.as_ref()
                && pending.deadline > SystemTime::now()
            {
                return Err(Error::LoginInFlight);
            }
        }

        let client = http::client()?;
        let code = request_device_code(&client, &self.endpoint).await?;
        let login = proto::AccountLoginResult {
            user_code: code.user_code.clone(),
            verification_uri: code.verification_uri.clone(),
            verification_uri_complete: code.verification_uri_complete.clone(),
            expires_in: code.expires_in,
            interval: code.interval,
        };
        // Claimed before the old flow can be told it has been replaced, so there is no instant
        // in which two tasks both believe they are current.
        let generation = self
            .generation
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst)
            + 1;
        *self.pending.lock().await = Some(Pending {
            login: login.clone(),
            deadline: SystemTime::now() + Duration::from_secs(code.expires_in),
        });
        *self.last_error.lock().await = None;

        tracing::info!(
            user_code = %code.user_code,
            uri = %code.verification_uri,
            expires_in = code.expires_in,
            "account login started; waiting for approval"
        );

        let this = std::sync::Arc::clone(self);
        tokio::spawn(async move { this.wait_for_approval(client, code, generation).await });
        Ok(login)
    }

    /// Poll Hugging Face until the code is approved, refused, or out of time.
    async fn wait_for_approval(&self, client: reqwest::Client, code: DeviceCode, generation: u64) {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(code.expires_in);
        let mut interval = Duration::from_secs(code.interval.max(1));

        let outcome = loop {
            tokio::time::sleep(interval).await;
            if tokio::time::Instant::now() >= deadline {
                break Err("the code expired before it was approved".to_string());
            }
            match poll_token(&client, &self.endpoint, &code.device_code).await {
                Poll::Pending => continue,
                Poll::SlowDown => {
                    interval += Duration::from_secs(5);
                    continue;
                }
                // Logged rather than surfaced: a 502 from a proxy is not news about this login,
                // and RFC 8628 says to keep asking until the code expires.
                Poll::Inconclusive(why) => {
                    tracing::debug!(%why, "inconclusive poll; still waiting");
                    continue;
                }
                Poll::Refused(why) => break Err(why),
                Poll::Token(token) => break Ok(*token),
            }
        };

        // Superseded, and therefore silent: a `--force` login replaced this one, or `logout`
        // said the robot belongs to nobody. Either way an approval that arrives now is an answer
        // to a question that has been withdrawn, and writing it would sign the robot into the
        // account somebody just replaced or out of. Checked *before* the store is touched.
        if !self.is_current(generation) {
            tracing::info!(
                user_code = %code.user_code,
                "a login was superseded before it finished; dropping its result"
            );
            return;
        }

        let result = match outcome {
            Err(why) => Err(why),
            Ok(token) => self
                .persist(&client, token)
                .await
                .map_err(|e| e.to_string()),
        };

        // And again after it, because `persist` awaits: a `logout` landing during the write is
        // the same withdrawal, and the record it left behind has to go with it.
        if !self.is_current(generation) {
            let _ = self.store.clear();
            tracing::info!("a login completed after being superseded; its token was discarded");
            return;
        }

        *self.pending.lock().await = None;
        match result {
            Ok(username) => {
                let username = username.unwrap_or_else(|| "an account it cannot name yet".into());
                tracing::info!(%username, "this robot now belongs to a Hugging Face account");
                *self.last_error.lock().await = None;
            }
            Err(why) => {
                tracing::warn!(%why, "account login did not complete");
                *self.last_error.lock().await = Some(why);
            }
        }
    }

    /// Whether the flow started as `generation` is still the one this robot is waiting on.
    fn is_current(&self, generation: u64) -> bool {
        self.generation.load(std::sync::atomic::Ordering::SeqCst) == generation
    }

    /// Store a fresh token, and return the username it belongs to if the name could be had.
    ///
    /// The username is asked for *before* the write and stored with it, so `status` never needs
    /// the network — and a robot that is offline still knows who it belongs to.
    ///
    /// **A failure to read the name must not lose the token.** By this point somebody has
    /// approved a code on their phone; throwing the credential away because `/oauth/userinfo`
    /// answered a 502 would make them do the whole flow again for a field that is a label. So
    /// the name is optional: the record lands either way, `status` says `unknown` until it is
    /// known, and [`maintain`] fills it in within six hours — see [`Self::name_if_unknown`].
    async fn persist(
        &self,
        client: &reqwest::Client,
        token: TokenResponse,
    ) -> Result<Option<String>, Error> {
        let username = match userinfo(client, &self.endpoint, &token.access_token).await {
            Ok(username) => Some(username),
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "signed in, but could not read the account name; storing the token anyway"
                );
                None
            }
        };
        self.store.save(&Stored {
            access_token: token.access_token,
            refresh_token: token.refresh_token,
            expires_at: token.expires_in.map(|s| now_secs() + s as i64),
            username: username.clone(),
        })?;
        Ok(username)
    }

    /// Who this robot belongs to, and whether a login is in flight.
    pub async fn status(&self) -> proto::AccountStatusResult {
        let account = self.store.load().map(|stored| proto::Account {
            username: stored
                .username
                .clone()
                .unwrap_or_else(|| "unknown".to_string()),
            token_expires_in: stored.expires_in(),
            refreshable: stored.refresh_token.is_some(),
        });

        // The code's remaining life rather than its original one: a client polling this wants a
        // countdown, and `expires_in` is documented as what is *left* here.
        let login = self.pending.lock().await.as_ref().and_then(|pending| {
            let left = pending
                .deadline
                .duration_since(SystemTime::now())
                .ok()?
                .as_secs();
            Some(proto::AccountLoginResult {
                expires_in: left,
                ..pending.login.clone()
            })
        });

        proto::AccountStatusResult {
            account,
            login,
            last_error: self.last_error.lock().await.clone(),
        }
    }

    /// Forget the account. A login in flight is abandoned with it.
    ///
    /// **Forgets rather than revokes.** The file goes, so the robot stops being able to prove it
    /// belongs to anybody, which is what signing out is for. The token itself stays valid at
    /// Hugging Face until it expires — up to thirty days — for anything that read the file while
    /// it was there. `remote-access-design.md` §2.6 says why that is where the line is and what
    /// closing it would take.
    pub async fn logout(&self) -> Result<proto::AccountLogoutResult, Error> {
        // Before the file goes, so a flow that approves during this call sees itself superseded
        // rather than writing a token into a robot that has just been signed out.
        self.generation
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let was = self.store.clear()?;
        *self.pending.lock().await = None;
        *self.last_error.lock().await = None;
        if let Some(username) = &was {
            tracing::info!(%username, "this robot no longer belongs to a Hugging Face account");
        }
        Ok(proto::AccountLogoutResult { was })
    }

    /// Renew the token before it expires. One pass; [`maintain`] is the loop.
    ///
    /// Returns `true` when it wrote a new token. Does nothing when the robot is signed out, when
    /// there is no refresh token, or when there is plenty of time left.
    pub async fn refresh_if_due(&self) -> Result<bool, Error> {
        let Some(stored) = self.store.load() else {
            return Ok(false);
        };
        let Some(refresh_token) = stored.refresh_token.clone() else {
            return Ok(false);
        };
        if stored.expires_in() > REFRESH_WHEN_UNDER.as_secs() as i64 {
            return Ok(false);
        }

        let token = refresh(&http::client()?, &self.endpoint, &refresh_token).await?;
        // Keep the username rather than re-asking: a refresh cannot change who the token belongs
        // to, and this path runs unattended on a board whose network may be marginal.
        self.store.save(&Stored {
            access_token: token.access_token,
            refresh_token: token.refresh_token,
            expires_at: token.expires_in.map(|s| now_secs() + s as i64),
            username: stored.username,
        })?;
        Ok(true)
    }

    /// Ask who the token belongs to, when the stored record cannot say.
    ///
    /// Only ever the case after a login whose `/oauth/userinfo` call failed — [`Self::persist`]
    /// keeps the token in that case rather than losing it over a label. Left alone,
    /// `account.status` would answer `unknown` until somebody signed in again; this is what makes
    /// it answer properly on the next [`maintain`] pass instead. Returns `true` when it wrote a
    /// name.
    pub async fn name_if_unknown(&self) -> Result<bool, Error> {
        let Some(stored) = self.store.load() else {
            return Ok(false);
        };
        if stored.username.is_some() {
            return Ok(false);
        }
        let username = userinfo(&http::client()?, &self.endpoint, &stored.access_token).await?;
        self.store.save(&Stored {
            username: Some(username),
            ..stored
        })?;
        Ok(true)
    }
}

/// Keep the token fresh for as long as this process runs.
///
/// Spawned once at startup. It is a slow loop on purpose: the thing it guards against is a robot
/// that has been on for a month, and the cost of asking too often is a request to Hugging Face
/// that answers "not yet".
pub async fn maintain(account: std::sync::Arc<Account>) {
    loop {
        match account.refresh_if_due().await {
            Ok(true) => tracing::info!("renewed the account token"),
            Ok(false) => {}
            // Not fatal and not silent: a week of retries is left, and `account.status` carries
            // the reason for anyone asking why remote access stopped.
            Err(e) => {
                let why = e.to_string();
                tracing::warn!(%why, "could not renew the account token");
                *account.last_error.lock().await = Some(why);
            }
        }
        // A missing name is cosmetic, so its failure stays in the journal rather than going to
        // `last_error`: that field answers "why did remote access stop", and this did not stop
        // it. Nothing happens here at all on the overwhelmingly common path — the name is only
        // absent after a login that could not reach `/oauth/userinfo`.
        match account.name_if_unknown().await {
            Ok(true) => tracing::info!("filled in the account name a login could not read"),
            Ok(false) => {}
            Err(e) => tracing::debug!(error = %e, "still cannot read the account name"),
        }
        tokio::time::sleep(MAINTAIN_INTERVAL).await;
    }
}

// ── small things ─────────────────────────────────────────────────────────────

fn now_secs() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

/// Write via a temp file and rename, with the temp file created `0600` from the start.
///
/// `fsutil::write_atomic` is the same dance without the mode, and this cannot use it: a token
/// written `0644` and chmodded afterwards is world-readable for the moment in between, which is
/// exactly the kind of window that is invisible in testing and permanent in a log.
fn write_private(path: &Path, bytes: &[u8]) -> Result<(), Error> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;

    let tmp = path.with_extension("tmp");
    let io = |source: std::io::Error, path: &Path| Error::Io {
        path: path.to_path_buf(),
        source,
    };
    {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .mode(0o600)
            .open(&tmp)
            .map_err(|e| io(e, &tmp))?;
        file.write_all(bytes).map_err(|e| io(e, &tmp))?;
        file.sync_all().map_err(|e| io(e, &tmp))?;
    }
    std::fs::rename(&tmp, path).map_err(|e| io(e, path))?;
    crate::fsutil::fsync_parent(path)
}

fn set_mode(path: &Path, mode: u32) -> Result<(), Error> {
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).map_err(|e| Error::Io {
        path: path.to_path_buf(),
        source: e,
    })
}

fn set_group(path: &Path, gid: u32) -> Result<(), Error> {
    use std::os::unix::ffi::OsStrExt;
    let cpath = std::ffi::CString::new(path.as_os_str().as_bytes()).map_err(|_| Error::Io {
        path: path.to_path_buf(),
        source: std::io::Error::other("path contains a NUL"),
    })?;
    // SAFETY: both pointers are valid for the call; `-1` as the uid leaves the owner alone.
    let rc = unsafe { libc::chown(cpath.as_ptr(), u32::MAX, gid) };
    if rc != 0 {
        return Err(Error::Io {
            path: path.to_path_buf(),
            source: std::io::Error::last_os_error(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store_in(dir: &tempfile::TempDir) -> Store {
        Store::new(dir.path().join("hf-token"))
    }

    fn token(access: &str) -> Stored {
        Stored {
            access_token: access.to_string(),
            refresh_token: Some("refresh-1".into()),
            expires_at: Some(now_secs() + 30 * 24 * 60 * 60),
            username: Some("PierreRouanet".into()),
        }
    }

    /// The store is a file, and the file survives being replaced.
    #[test]
    fn a_credential_round_trips() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);

        assert!(store.load().is_none(), "nothing stored yet");
        store.save(&token("first")).unwrap();
        assert_eq!(store.load().unwrap().access_token, "first");

        store.save(&token("second")).unwrap();
        assert_eq!(store.load().unwrap().access_token, "second");
        assert_eq!(
            store.clear().unwrap(),
            Some("PierreRouanet".to_string()),
            "clear reports who it forgot, so a CLI can say so"
        );
        assert!(store.load().is_none());
        assert_eq!(
            store.clear().unwrap(),
            None,
            "clearing nothing is not an error — `logout` twice is not a failure"
        );
    }

    /// **A token must never be world-readable, at any point.**
    ///
    /// The mode is checked on the file that lands, and the temp file it was written through is
    /// checked by construction: `write_private` opens it `0600` rather than chmodding afterwards,
    /// which is the window this test cannot see and the reason the helper exists.
    #[test]
    fn the_token_file_is_not_world_readable() {
        use std::os::unix::fs::PermissionsExt;

        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        store.save(&token("secret")).unwrap();

        let mode = std::fs::metadata(store.path())
            .unwrap()
            .permissions()
            .mode()
            & 0o777;
        assert_eq!(
            mode & 0o007,
            0,
            "the token file is readable by other users: {mode:o}"
        );
        // 0640 on a board with a `robot` group, 0600 without one — a developer's laptop has no
        // such group, and the test has to pass in both places.
        assert!(mode == 0o600 || mode == 0o640, "unexpected mode {mode:o}");
        assert!(
            !dir.path().join("hf-token.tmp").exists(),
            "the temp file must not be left behind holding a copy of the token"
        );
    }

    /// A file somebody edited by hand reads as "signed out", not as a broken daemon.
    #[test]
    fn a_corrupt_credential_is_signed_out_rather_than_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        std::fs::write(store.path(), b"{ this is not json").unwrap();
        assert!(store.load().is_none());
    }

    /// Which OAuth errors end a login, and which are it working normally.
    ///
    /// The two in the middle are the ones worth pinning: treating `slow_down` as a failure would
    /// abandon a login that was about to succeed, and treating `expired_token` as transient would
    /// poll a dead code until the deadline.
    #[test]
    fn oauth_errors_are_classified() {
        let of = |error: &str| {
            classify(&OAuthError {
                error: error.to_string(),
                error_description: None,
            })
        };
        assert_eq!(of("authorization_pending"), Poll::Pending);
        assert_eq!(of("slow_down"), Poll::SlowDown);
        assert!(matches!(of("expired_token"), Poll::Refused(_)));
        assert!(matches!(of("access_denied"), Poll::Refused(_)));
        // An error we have never seen is still an answer about this login: `invalid_client` will
        // not fix itself by being asked again, and polling it for five minutes hides the cause.
        assert!(matches!(of("invalid_client"), Poll::Refused(_)));
    }

    /// Hugging Face sends neither `interval` nor `verification_uri_complete`. Both are
    /// synthesised here so nothing downstream has to know that.
    #[tokio::test]
    async fn a_device_code_response_is_normalised() {
        // Exactly what huggingface.co answered on 2026-09-02, field for field.
        let server = fake_hf(
            r#"{"device_code":"41ad39ae","user_code":"A6MY-0314",
                "verification_uri":"https://hf.co/oauth/device","expires_in":300}"#,
        )
        .await;

        let code = request_device_code(&http::client().unwrap(), &server.base)
            .await
            .unwrap();

        assert_eq!(code.user_code, "A6MY-0314");
        assert_eq!(code.expires_in, 300);
        assert_eq!(
            code.interval, DEFAULT_POLL_INTERVAL,
            "RFC 8628's fallback, because HF sends no interval"
        );
        assert_eq!(
            code.verification_uri_complete, "https://hf.co/oauth/device",
            "the plain URI when the server sends none — HF's device page ignores a `?user_code=` \
             query, so inventing one would promise a prefill that does not happen"
        );
    }

    /// A robot that already belongs to somebody refuses without `force`, and says who.
    ///
    /// The check is before any network call, which is what makes this testable offline — and is
    /// also the behaviour that matters: a login that reached Hugging Face first would have burned
    /// a device code to arrive at the same refusal.
    #[tokio::test]
    async fn signing_in_over_an_existing_account_is_refused_by_name() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        store.save(&token("already-here")).unwrap();

        let account = std::sync::Arc::new(Account::new(store));
        let error = account.login(false).await.expect_err("must refuse");
        assert!(
            matches!(&error, Error::AlreadySignedIn(who) if who == "PierreRouanet"),
            "{error:?}"
        );
        assert_eq!(
            error.code(),
            crate::proto::code::INVALID_PARAMS,
            "the fix is a parameter, and a client should be able to tell that from the code"
        );
        assert!(
            error.to_string().contains("--force"),
            "the message must name the way past it: {error}"
        );
    }

    /// `status` on a robot that belongs to nobody, which is every robot out of the box.
    #[tokio::test]
    async fn status_of_a_robot_that_belongs_to_nobody() {
        let dir = tempfile::tempdir().unwrap();
        let account = Account::new(store_in(&dir));
        let status = account.status().await;
        assert!(status.account.is_none());
        assert!(status.login.is_none());
        assert!(status.last_error.is_none());
    }

    /// `status` reports the account from disk, with no network call.
    #[tokio::test]
    async fn status_names_the_account_without_asking_anybody() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        store.save(&token("stored")).unwrap();

        let status = Account::new(store).status().await;
        let account = status.account.expect("signed in");
        assert_eq!(account.username, "PierreRouanet");
        assert!(account.refreshable, "a refresh token was stored");
        assert!(
            account.token_expires_in > 29 * 24 * 60 * 60,
            "about thirty days: {}",
            account.token_expires_in
        );
    }

    /// A token with a month left is not renewed, and a robot with no token is not renewed either.
    ///
    /// Both are the "do nothing" case and both would otherwise be an HTTP request on every tick
    /// of [`maintain`] — on a board whose network may not be there.
    #[tokio::test]
    async fn a_fresh_token_is_left_alone() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        let account = Account::new(store.clone());

        assert!(!account.refresh_if_due().await.unwrap(), "signed out");

        store.save(&token("fresh")).unwrap();
        assert!(!account.refresh_if_due().await.unwrap(), "plenty of time");

        // No refresh token: nothing to renew with, and it must not be an error — the robot is
        // signed in and working, it just cannot renew unattended.
        store
            .save(&Stored {
                refresh_token: None,
                expires_at: Some(now_secs() + 60),
                ..token("expiring")
            })
            .unwrap();
        assert!(!account.refresh_if_due().await.unwrap());
    }

    /// A one-route stand-in for huggingface.co.
    struct FakeHf {
        base: String,
        /// Every `grant_type` the token endpoint was asked for, in order. Empty for a fake that
        /// does not serve `/oauth/token`.
        grants: std::sync::Arc<std::sync::Mutex<Vec<String>>>,
        /// How many times the route under test was asked, for the tests where *once* is the
        /// property rather than the answer.
        hits: std::sync::Arc<std::sync::atomic::AtomicUsize>,
        _task: tokio::task::JoinHandle<()>,
    }

    impl FakeHf {
        fn grants(&self) -> Vec<String> {
            self.grants.lock().unwrap().clone()
        }

        fn hits(&self) -> usize {
            self.hits.load(std::sync::atomic::Ordering::SeqCst)
        }
    }

    #[derive(Default)]
    struct Records {
        grants: std::sync::Arc<std::sync::Mutex<Vec<String>>>,
        hits: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    }

    async fn serve(app: axum::Router, records: Records) -> FakeHf {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        let task = tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });
        FakeHf {
            base,
            grants: records.grants,
            hits: records.hits,
            _task: task,
        }
    }

    async fn fake_hf(device_response: &'static str) -> FakeHf {
        use axum::routing::post;

        let app = axum::Router::new().route(
            "/oauth/device",
            post(move || async move { ([("content-type", "application/json")], device_response) }),
        );
        serve(app, Records::default()).await
    }

    /// A stand-in for the token endpoint alone, which is all a refresh touches.
    ///
    /// It records the `grant_type` it was asked for, because a refresh sent as a device-code
    /// grant would be refused by Hugging Face and by nobody here.
    async fn fake_hf_token(answer: &'static str) -> FakeHf {
        use axum::routing::post;

        let records = Records::default();
        let seen = std::sync::Arc::clone(&records.grants);
        let app = axum::Router::new().route(
            "/oauth/token",
            post(
                move |axum::extract::Form(form): axum::extract::Form<
                    std::collections::HashMap<String, String>,
                >| {
                    let seen = std::sync::Arc::clone(&seen);
                    async move {
                        seen.lock()
                            .unwrap()
                            .push(form.get("grant_type").cloned().unwrap_or_default());
                        ([("content-type", "application/json")], answer)
                    }
                },
            ),
        );
        serve(app, records).await
    }

    /// A stand-in for `/oauth/userinfo` alone, answering with whatever status is asked for.
    async fn fake_hf_userinfo(answer: &'static str, status: u16) -> FakeHf {
        use axum::routing::get;

        let code = axum::http::StatusCode::from_u16(status).unwrap();
        let app = axum::Router::new().route(
            "/oauth/userinfo",
            get(move || async move { (code, [("content-type", "application/json")], answer) }),
        );
        serve(app, Records::default()).await
    }

    /// A device endpoint that takes its time, and counts how often it was asked.
    ///
    /// The delay is the point: the window two logins race for is exactly this round trip, so a
    /// fake that answered instantly would leave the test passing for the wrong reason.
    async fn fake_hf_slow_device() -> FakeHf {
        use axum::routing::post;

        let records = Records::default();
        let hits = std::sync::Arc::clone(&records.hits);
        let app = axum::Router::new().route(
            "/oauth/device",
            post(move || {
                let hits = std::sync::Arc::clone(&hits);
                async move {
                    hits.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    tokio::time::sleep(Duration::from_millis(200)).await;
                    (
                        [("content-type", "application/json")],
                        r#"{"device_code":"device-abc","user_code":"A6MY-0314",
                            "verification_uri":"https://hf.co/oauth/device",
                            "expires_in":60,"interval":1}"#,
                    )
                }
            }),
        );
        serve(app, records).await
    }

    /// Two logins arriving together: one starts, the other is told the robot is busy.
    ///
    /// The guard has to cover the round trip to `/oauth/device`, not just the check before it —
    /// two codes handed out means two pollers, and the store then holds whichever approval landed
    /// last while somebody stares at the other code wondering why it did nothing.
    #[tokio::test]
    async fn two_logins_at_once_do_not_both_reach_hugging_face() {
        let dir = tempfile::tempdir().unwrap();
        let hf = fake_hf_slow_device().await;
        let account = std::sync::Arc::new(Account::with_endpoint(store_in(&dir), hf.base.clone()));

        let (first, second) = tokio::join!(account.login(false), account.login(false));

        let outcomes = [first, second];
        assert_eq!(
            outcomes.iter().filter(|r| r.is_ok()).count(),
            1,
            "exactly one login starts: {outcomes:?}"
        );
        let refusal = outcomes
            .iter()
            .find_map(|r| r.as_ref().err())
            .expect("the other is refused");
        assert!(
            matches!(refusal, Error::LoginInFlight),
            "and refused as busy, which is a state that passes: {refusal:?}"
        );
        assert_eq!(refusal.code(), crate::proto::code::BUSY, "retryable");
        assert!(
            refusal.to_string().contains("account status"),
            "and it says where the code that already exists can be read: {refusal}"
        );
        assert_eq!(
            hf.hits(),
            1,
            "one device code asked for, so there is only one code to read"
        );
    }

    /// A whole fake Hugging Face: a device code, a token, and a name.
    ///
    /// `approve_after` is how many `authorization_pending` answers to give first; `0` approves on
    /// the first poll, which is what the tests about *abandoning* a login want — the approval has
    /// to land while the test is still watching.
    async fn fake_hf_full(approve_after: usize) -> FakeHf {
        use axum::routing::{get, post};

        let records = Records::default();
        let polls = std::sync::Arc::clone(&records.hits);
        let app = axum::Router::new()
            .route(
                "/oauth/device",
                post(|| async {
                    (
                        [("content-type", "application/json")],
                        r#"{"device_code":"device-abc","user_code":"A6MY-0314",
                            "verification_uri":"https://hf.co/oauth/device",
                            "expires_in":60,"interval":1}"#,
                    )
                }),
            )
            .route(
                "/oauth/token",
                post(move || {
                    let polls = std::sync::Arc::clone(&polls);
                    async move {
                        let n = polls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                        let body = if n < approve_after {
                            r#"{"error":"authorization_pending"}"#
                        } else {
                            r#"{"access_token":"approved","refresh_token":"refresh-1",
                                "expires_in":2591999}"#
                        };
                        ([("content-type", "application/json")], body)
                    }
                }),
            )
            .route(
                "/oauth/userinfo",
                get(|| async {
                    (
                        [("content-type", "application/json")],
                        r#"{"preferred_username":"PierreRouanet"}"#,
                    )
                }),
            );
        serve(app, records).await
    }

    /// `--force` abandons a code nobody is going to approve.
    ///
    /// Without this the only ways past a live code are waiting five minutes and `logout`, and
    /// `logout` destroys a working credential to clear a *pending* one. The refusal has to name a
    /// way through it, and this is that way.
    #[tokio::test]
    async fn force_replaces_a_login_that_is_still_waiting() {
        let dir = tempfile::tempdir().unwrap();
        let hf = fake_hf_slow_device().await;
        let account = std::sync::Arc::new(Account::with_endpoint(store_in(&dir), hf.base.clone()));

        let first = account.login(false).await.expect("the first login starts");
        assert_eq!(hf.hits(), 1);

        let refused = account
            .login(false)
            .await
            .expect_err("a live code refuses a second login");
        assert!(matches!(refused, Error::LoginInFlight), "{refused:?}");
        assert!(
            refused.to_string().contains("--force"),
            "and says how to get past itself: {refused}"
        );

        account.login(true).await.expect("`force` gets past it");
        assert_eq!(hf.hits(), 2, "a new code was asked for");
        let waiting = account
            .status()
            .await
            .login
            .expect("one login is in flight");
        assert_eq!(
            waiting.user_code, first.user_code,
            "this fake answers with one code, so what is pinned here is that `status` describes \
             the current login and not a stale one"
        );
    }

    /// An approval that lands after `logout` is dropped, not written.
    ///
    /// The flow lives in a task that outlives the call that started it, so signing out while a
    /// code is live leaves somebody able to approve it a minute later. Writing that would sign
    /// the robot back in on its own, which is the one thing `logout` has to be able to promise it
    /// will not do.
    #[tokio::test]
    async fn a_login_approved_after_logout_is_discarded() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        let hf = fake_hf_full(0).await;
        let account = std::sync::Arc::new(Account::with_endpoint(store.clone(), hf.base.clone()));

        account.login(false).await.expect("a login starts");
        account.logout().await.expect("and is signed out under it");

        // The fake approves on the first poll, one second in. Well past that, and nothing has
        // been written: the flow saw itself superseded before it touched the store.
        tokio::time::sleep(Duration::from_millis(2_500)).await;
        assert!(
            hf.hits() >= 1,
            "the abandoned flow did reach the token endpoint"
        );
        assert!(
            store.load().is_none(),
            "a robot signed out must stay signed out"
        );
        let status = account.status().await;
        assert!(status.account.is_none());
        assert!(status.login.is_none());
    }

    /// An approved token is not thrown away because `/oauth/userinfo` failed.
    ///
    /// By the time this runs somebody has typed a code into a phone. Losing the credential over
    /// the *label* would make them do all of it again, and on the board this is for — one whose
    /// network is the reason the call failed — quite possibly twice. So the name is optional, and
    /// the next `maintain` pass is what fills it in.
    #[tokio::test]
    async fn an_approved_token_survives_a_userinfo_failure() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        let approved = || TokenResponse {
            access_token: "approved".into(),
            refresh_token: Some("refresh-1".into()),
            expires_in: Some(2_591_999),
        };

        let broken = fake_hf_userinfo("<html>502 Bad Gateway</html>", 502).await;
        let account = Account::with_endpoint(store.clone(), broken.base.clone());
        let named = account
            .persist(&http::client().unwrap(), approved())
            .await
            .expect("a name that cannot be read is not a failed login");
        assert!(named.is_none());

        let stored = store.load().expect("the credential is on disk regardless");
        assert_eq!(stored.access_token, "approved");
        assert_eq!(stored.refresh_token.as_deref(), Some("refresh-1"));
        assert!(stored.username.is_none(), "the name is what is missing");
        assert_eq!(
            account.status().await.account.unwrap().username,
            "unknown",
            "a client is told the robot belongs to somebody it cannot name, not that it is \
             signed out"
        );

        // And the next pass names it, without touching the credential.
        let working = fake_hf_userinfo(r#"{"preferred_username":"PierreRouanet"}"#, 200).await;
        let account = Account::with_endpoint(store.clone(), working.base.clone());
        assert!(account.name_if_unknown().await.unwrap());
        let stored = store.load().unwrap();
        assert_eq!(stored.username.as_deref(), Some("PierreRouanet"));
        assert_eq!(
            stored.access_token, "approved",
            "the backfill writes the name and nothing else"
        );
        assert!(
            !account.name_if_unknown().await.unwrap(),
            "and asks nobody once it knows — this runs every six hours forever"
        );
    }

    /// A token near the end of its life is renewed, and **the rotated refresh token replaces the
    /// one that was spent**.
    ///
    /// The rotation is the part worth a test rather than a comment: Hugging Face spends the old
    /// refresh token on every refresh, so storing the one already on disk — the obvious mistake,
    /// since the rest of the record is carried over — leaves a robot that renews exactly once and
    /// then quietly stops being reachable. `remote-access-design.md` §2.7.
    #[tokio::test]
    async fn a_due_token_is_renewed_and_the_rotation_lands_on_disk() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        // Six days left: inside `REFRESH_WHEN_UNDER`, which is how a board with a marginal
        // network gets a week of retries instead of one last day.
        store
            .save(&Stored {
                expires_at: Some(now_secs() + 6 * 24 * 60 * 60),
                ..token("spent")
            })
            .unwrap();

        let hf = fake_hf_token(
            r#"{"access_token":"renewed","refresh_token":"refresh-2","expires_in":2591999}"#,
        )
        .await;
        let account = Account::with_endpoint(store.clone(), hf.base.clone());

        assert!(
            account.refresh_if_due().await.unwrap(),
            "a token with six days left is due"
        );

        let stored = store.load().unwrap();
        assert_eq!(stored.access_token, "renewed");
        assert_eq!(
            stored.refresh_token.as_deref(),
            Some("refresh-2"),
            "the answer's refresh token, not the one it was traded for"
        );
        assert_eq!(
            stored.username.as_deref(),
            Some("PierreRouanet"),
            "kept rather than re-asked: a refresh cannot change who a token belongs to, and this              path runs unattended"
        );
        assert!(
            stored.expires_in() > 29 * 24 * 60 * 60,
            "the new expiry is absolute and thirty days out: {}",
            stored.expires_in()
        );
        assert_eq!(
            hf.grants(),
            vec!["refresh_token".to_string()],
            "one refresh, sent as a refresh"
        );

        // And now it is not due again, which is what stops `maintain` renewing on every tick.
        assert!(!account.refresh_if_due().await.unwrap());
        assert_eq!(hf.grants().len(), 1, "no second round trip");
    }

    /// A refresh Hugging Face refuses leaves the credential alone and says so in `status`.
    ///
    /// This is the visible half of the window §2.7 says cannot be closed: once the old refresh
    /// token is spent, no ordering here can recover it, so what the code owes is (a) not making
    /// it worse by writing a half-record and (b) telling somebody. `maintain` is what turns the
    /// error into an answer a client can read, so it is driven here rather than trusted.
    #[tokio::test]
    async fn a_refused_refresh_is_left_alone_and_reported() {
        let dir = tempfile::tempdir().unwrap();
        let store = store_in(&dir);
        store
            .save(&Stored {
                expires_at: Some(now_secs() + 60 * 60),
                ..token("spent")
            })
            .unwrap();

        let hf = fake_hf_token(
            r#"{"error":"invalid_grant","error_description":"refresh token is expired"}"#,
        )
        .await;
        let account = std::sync::Arc::new(Account::with_endpoint(store.clone(), hf.base.clone()));

        let error = account
            .refresh_if_due()
            .await
            .expect_err("a refused refresh is an error");
        assert!(
            error.to_string().contains("invalid_grant"),
            "the reason has to survive to the surface: {error}"
        );
        assert_eq!(
            store.load().unwrap().access_token,
            "spent",
            "the stored credential is untouched — an hour of access left is worth more than a              record half-replaced by a failed refresh"
        );

        let task = tokio::spawn(maintain(std::sync::Arc::clone(&account)));
        let mut reported = None;
        for _ in 0..200 {
            if let Some(why) = account.status().await.last_error {
                reported = Some(why);
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        task.abort();
        let reported = reported.expect("`maintain` must put the failure where a client can see it");
        assert!(
            reported.contains("invalid_grant"),
            "`account.status` is where somebody asks why remote access stopped: {reported}"
        );
    }
}
