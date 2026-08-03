//! Shared HTTP plumbing for the network sources.
//!
//! TLS is rustls via `rustls-platform-verifier`, so certificate roots come from the
//! **OS trust store** rather than a bundled copy. That matters for a robot: roots
//! then follow Debian's security updates instead of needing a daemon release, and an
//! operator-installed CA works without a rebuild.
//!
//! Two properties everything here must have:
//!
//!  - **Bounded.** Every request has a connect timeout, and every download has a
//!    per-chunk stall timeout. A hung mirror must not hold an update open forever —
//!    but a slow one on a big artifact must not be killed by a total deadline either,
//!    which is why the timeout is per-chunk rather than overall.
//!  - **Resumable.** A dropped connection on a large artifact retries with a `Range`
//!    header instead of starting over. Robots are on domestic wifi.

use std::path::Path;
use std::time::Duration;

use futures_util::StreamExt;
use tokio::io::AsyncWriteExt;

use crate::Error;
use crate::source::ProgressSink;

/// Connect timeout. Generous enough for a slow uplink, short enough that an
/// unreachable host fails promptly.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);

/// Whole-request timeout for small JSON/metadata fetches.
const METADATA_TIMEOUT: Duration = Duration::from_secs(30);

/// Longest gap between two chunks of a download before we call it stalled.
///
/// Deliberately not a total deadline: a legitimately slow download of a large model
/// bundle must be allowed to take as long as it takes.
const CHUNK_STALL_TIMEOUT: Duration = Duration::from_secs(60);

/// Attempts per artifact, including the first. Retries resume via `Range`.
const DOWNLOAD_ATTEMPTS: usize = 4;

/// Whether a failure is worth retrying.
///
/// A wrong repo, tag or asset name (404) will be just as wrong in half a second, so
/// retrying only delays a clear error by the whole backoff budget. Transport errors
/// and server-side faults are the opposite: domestic wifi drops connections and
/// mirrors have bad minutes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Retry {
    Worth,
    Pointless,
}

fn classify(status: reqwest::StatusCode) -> Retry {
    match status.as_u16() {
        // Timeouts and rate limits are explicitly "come back later".
        408 | 429 => Retry::Worth,
        // Everything else in 4xx is a request we got wrong; repeating it won't help.
        400..=499 => Retry::Pointless,
        500..=599 => Retry::Worth,
        _ => Retry::Worth,
    }
}

/// Cap on a metadata response. A manifest is a few hundred bytes; anything near this
/// means we are talking to the wrong thing.
const MAX_METADATA_BYTES: u64 = 1024 * 1024;

/// Absolute ceiling on an artifact download, so a wrong URL cannot fill the eMMC
/// before the hash check gets a chance to reject it.
const MAX_ARTIFACT_BYTES: u64 = 4 * 1024 * 1024 * 1024;

pub fn user_agent() -> String {
    format!("updaterd/{}", env!("CARGO_PKG_VERSION"))
}

/// Build the shared client.
pub fn client() -> Result<reqwest::Client, Error> {
    reqwest::Client::builder()
        .user_agent(user_agent())
        .connect_timeout(CONNECT_TIMEOUT)
        // Redirects are followed, but not indefinitely: a redirect loop is a
        // misconfigured mirror, not something to chase.
        .redirect(reqwest::redirect::Policy::limited(5))
        .build()
        .map_err(|e| Error::Network(format!("could not build HTTP client: {e}")))
}

/// Fetch a small resource whole (manifests, signatures, API responses).
pub async fn get_bytes(
    client: &reqwest::Client,
    url: &str,
    accept: Option<&str>,
) -> Result<Vec<u8>, Error> {
    let mut request = client.get(url).timeout(METADATA_TIMEOUT);
    if let Some(accept) = accept {
        request = request.header(reqwest::header::ACCEPT, accept);
    }
    if let Some(token) = github_token() {
        // Only sent to github.com; see `github_token`.
        if url.contains("github.com") {
            request = request.bearer_auth(token);
        }
    }

    let response = request
        .send()
        .await
        .map_err(|e| Error::Network(format!("GET {url}: {e}")))?;

    let status = response.status();
    if !status.is_success() {
        return Err(Error::Network(describe_failure(url, status)));
    }

    if let Some(len) = response.content_length()
        && len > MAX_METADATA_BYTES
    {
        return Err(Error::Network(format!(
            "GET {url}: {len} bytes is implausibly large for metadata"
        )));
    }

    let bytes = response
        .bytes()
        .await
        .map_err(|e| Error::Network(format!("reading {url}: {e}")))?;

    if bytes.len() as u64 > MAX_METADATA_BYTES {
        return Err(Error::Network(format!("GET {url}: response too large")));
    }
    Ok(bytes.to_vec())
}

/// Stream a URL to `dest`, reporting progress and resuming across retries.
///
/// Returns the number of bytes written. Integrity is **not** checked here — the
/// caller verifies the hash and signature, which is the authoritative check.
/// Stream `url` to `dest`, resuming a partial file if one is there.
///
/// `accept` exists for GitHub's release-asset API, which serves the *metadata* for an asset
/// unless the request asks for `application/octet-stream`. Getting that wrong yields a JSON
/// blob written to disk with an artifact's name, which then fails its hash check — a
/// confusing way to discover a missing header.
pub async fn download_to(
    client: &reqwest::Client,
    url: &str,
    dest: &Path,
    accept: Option<&str>,
    progress: &ProgressSink,
) -> Result<u64, Error> {
    let mut last_error = None;

    for attempt in 1..=DOWNLOAD_ATTEMPTS {
        // Resume from whatever a previous attempt managed to write.
        let already = std::fs::metadata(dest).map(|m| m.len()).unwrap_or(0);
        if already > 0 {
            tracing::info!(url, already, attempt, "resuming download");
        }

        match attempt_download(client, url, dest, already, accept, progress).await {
            Ok(total) => return Ok(total),
            Err(e) => {
                let retry = e.retry;
                tracing::warn!(url, attempt, error = %e.error, ?retry, "download attempt failed");
                let error = e.error;

                if retry == Retry::Pointless {
                    // Nothing about waiting would change the answer, and a partial
                    // file from a rejected request is only misleading.
                    let _ = std::fs::remove_file(dest);
                    return Err(error);
                }

                last_error = Some(error);
                // Brief, increasing backoff: a mirror that just dropped us is often
                // fine a moment later.
                tokio::time::sleep(Duration::from_millis(500 * attempt as u64)).await;
            }
        }
    }

    Err(last_error
        .unwrap_or_else(|| Error::Network(format!("{url}: download failed with no error"))))
}

/// An error plus whether retrying it could help.
struct AttemptError {
    error: Error,
    retry: Retry,
}

impl AttemptError {
    /// Transport-level failures (reset, DNS, TLS, stall) are always worth a retry.
    fn transient(error: Error) -> Self {
        Self {
            error,
            retry: Retry::Worth,
        }
    }
}

async fn attempt_download(
    client: &reqwest::Client,
    url: &str,
    dest: &Path,
    resume_from: u64,
    accept: Option<&str>,
    progress: &ProgressSink,
) -> Result<u64, AttemptError> {
    let mut request = client.get(url);
    if resume_from > 0 {
        request = request.header(reqwest::header::RANGE, format!("bytes={resume_from}-"));
    }
    if let Some(accept) = accept {
        request = request.header(reqwest::header::ACCEPT, accept);
    }
    if let Some(token) = github_token()
        && url.contains("github.com")
    {
        request = request.bearer_auth(token);
    }

    let response = request
        .send()
        .await
        .map_err(|e| AttemptError::transient(Error::Network(format!("GET {url}: {e}"))))?;

    let status = response.status();
    if !status.is_success() {
        return Err(AttemptError {
            error: Error::Network(describe_failure(url, status)),
            retry: classify(status),
        });
    }

    // A server that ignores `Range` sends 200 with the whole body; starting over is
    // correct then, and appending would corrupt the file.
    let resuming = resume_from > 0 && status == reqwest::StatusCode::PARTIAL_CONTENT;
    let start = if resuming { resume_from } else { 0 };

    let remaining = response.content_length();
    let total = remaining.map(|r| start + r);
    if let Some(total) = total
        && total > MAX_ARTIFACT_BYTES
    {
        // A ceiling breach is a publishing mistake, not a bad minute.
        return Err(AttemptError {
            error: Error::Network(format!(
                "{url}: {total} bytes exceeds the {MAX_ARTIFACT_BYTES}-byte ceiling"
            )),
            retry: Retry::Pointless,
        });
    }

    let mut file = tokio::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(!resuming)
        .open(dest)
        .await
        .map_err(|e| {
            AttemptError::transient(Error::Io {
                path: dest.to_path_buf(),
                source: e,
            })
        })?;
    if resuming {
        file.seek(std::io::SeekFrom::Start(start))
            .await
            .map_err(|e| {
                AttemptError::transient(Error::Io {
                    path: dest.to_path_buf(),
                    source: e,
                })
            })?;
    }

    let mut written = start;
    let mut stream = response.bytes_stream();

    loop {
        // Per-chunk, so a stalled connection is caught without capping how long a
        // legitimately slow download may take.
        let next = tokio::time::timeout(CHUNK_STALL_TIMEOUT, stream.next()).await;
        let chunk = match next {
            Ok(Some(Ok(chunk))) => chunk,
            Ok(Some(Err(e))) => {
                return Err(AttemptError::transient(Error::Network(format!(
                    "reading {url}: {e}"
                ))));
            }
            Ok(None) => break,
            Err(_elapsed) => {
                return Err(AttemptError::transient(Error::Network(format!(
                    "{url}: no data for {}s; treating as stalled",
                    CHUNK_STALL_TIMEOUT.as_secs()
                ))));
            }
        };

        written += chunk.len() as u64;
        if written > MAX_ARTIFACT_BYTES {
            return Err(AttemptError {
                error: Error::Network(format!(
                    "{url}: exceeded the {MAX_ARTIFACT_BYTES}-byte ceiling mid-download"
                )),
                retry: Retry::Pointless,
            });
        }

        file.write_all(&chunk).await.map_err(|e| {
            AttemptError::transient(Error::Io {
                path: dest.to_path_buf(),
                source: e,
            })
        })?;

        // Send failures mean nobody is watching; never a reason to stop.
        let _ = progress.send((written, total));
    }

    file.flush().await.map_err(|e| {
        AttemptError::transient(Error::Io {
            path: dest.to_path_buf(),
            source: e,
        })
    })?;
    // The hash is read back from this file, so the data must actually be on disk.
    file.sync_data().await.map_err(|e| {
        AttemptError::transient(Error::Io {
            path: dest.to_path_buf(),
            source: e,
        })
    })?;

    if let Some(expected) = total
        && written != expected
    {
        // A truncated transfer: worth another go.
        return Err(AttemptError::transient(Error::Network(format!(
            "{url}: expected {expected} bytes, wrote {written}"
        ))));
    }

    Ok(written)
}

/// Optional token for private repos and to lift anonymous rate limits.
///
/// Read from the environment rather than config so it never lands in a file that
/// gets copied around. Only ever attached to github.com requests — sending it to a
/// redirect target would leak it.
fn github_token() -> Option<String> {
    std::env::var("GITHUB_TOKEN")
        .ok()
        .filter(|t| !t.trim().is_empty())
}

/// Turn a status code into something diagnosable from a support ticket.
fn describe_failure(url: &str, status: reqwest::StatusCode) -> String {
    let hint = match status.as_u16() {
        401 | 403 => " (private repo, or rate-limited — set GITHUB_TOKEN?)",
        // GitHub answers 404 — not 403 — for a private resource the caller cannot see, so
        // as not to disclose that it exists. So the one status that most often means "you
        // need a token" was the one suggesting a typo. That cost a real debugging round
        // trip on the first board: the repo name was right and the message said it was not.
        404 => " (wrong repo, tag, or asset name? or a private repo — set GITHUB_TOKEN?)",
        429 => " (rate-limited; retry later)",
        500..=599 => " (server-side; retry later)",
        _ => "",
    };
    format!("GET {url}: HTTP {status}{hint}")
}

use tokio::io::AsyncSeekExt;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn client_builds() {
        assert!(client().is_ok());
    }

    #[test]
    fn user_agent_identifies_us() {
        assert!(user_agent().starts_with("updaterd/"));
    }

    /// Failure messages must be actionable — "HTTP 404" alone sends someone hunting.
    ///
    /// **404 must mention the token.** GitHub answers 404, not 403, for a private resource
    /// the caller cannot see, so as not to disclose that it exists — which means the status
    /// most likely to mean "you need a token" was the one suggesting a typo. On the first
    /// real board that message sent someone checking a repository name that was correct.
    #[test]
    fn failures_carry_a_hint() {
        let msg = describe_failure("https://x/y", reqwest::StatusCode::NOT_FOUND);
        assert!(msg.contains("wrong repo"), "{msg}");
        assert!(
            msg.contains("GITHUB_TOKEN"),
            "404 must offer the token too: {msg}"
        );

        let msg = describe_failure("https://x/y", reqwest::StatusCode::FORBIDDEN);
        assert!(msg.contains("GITHUB_TOKEN"), "{msg}");
    }

    #[test]
    fn blank_token_is_treated_as_absent() {
        // Guards against `GITHUB_TOKEN=` in a unit file turning into `Bearer `.
        unsafe { std::env::set_var("GITHUB_TOKEN", "   ") };
        assert!(github_token().is_none());
        unsafe { std::env::remove_var("GITHUB_TOKEN") };
    }
}
