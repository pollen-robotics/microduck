//! GitHub Releases source — the daemon channel.
//!
//! "Latest" is resolved by listing releases and taking the highest **semver** among
//! tags matching `tag_prefix`, not by using `/releases/latest`: that endpoint is
//! repo-wide, so it breaks the moment a second channel shares the repo, and it
//! answers "most recently published" rather than "highest version" — which differ as
//! soon as you publish a patch to an older line.
//! See `docs/design/updater-design.md` §6.
//!
//! Nothing here is trusted. Tags, release names and asset URLs are all attacker- or
//! mistake-influenced; the only thing that makes an artifact acceptable is the
//! minisign signature the caller checks afterwards.

use std::path::Path;

use serde::Deserialize;

use crate::Error;
use crate::manifest::Manifest;
use crate::source::{FetchedArtifact, ProgressSink, SignedBytes, Source, http};

/// Releases fetched per page when scanning for the newest tag. One page is plenty
/// for any real channel; further pages are fetched only if a page comes back full.
const PER_PAGE: usize = 100;
const MAX_PAGES: usize = 5;

/// The signature that accompanies every signed file.
const SIG_SUFFIX: &str = ".minisig";

/// What the release-asset API needs to return bytes rather than JSON metadata.
const OCTET_STREAM: &str = "application/octet-stream";

pub struct GithubReleases {
    repo: String,
    tag_prefix: String,
    manifest_asset: String,
    ref_tag_prefix: String,
    staging_tag_prefix: String,
    client: reqwest::Client,
}

/// Only the fields we use. GitHub adds fields freely, so this is deliberately not
/// exhaustive.
#[derive(Debug, Deserialize)]
struct Release {
    tag_name: String,
    #[serde(default)]
    draft: bool,
    #[serde(default)]
    prerelease: bool,
    #[serde(default)]
    assets: Vec<Asset>,
}

#[derive(Debug, Deserialize)]
struct Asset {
    name: String,
    /// The API endpoint for this asset, `/repos/{owner}/{repo}/releases/assets/{id}`.
    ///
    /// Used in preference to `browser_download_url` because that one **404s on a private
    /// repository**, with or without a token — verified against this repo. The API endpoint
    /// serves the bytes with a token and `Accept: application/octet-stream`, and works for
    /// public repos too, so there is one path rather than two.
    url: String,
}

impl GithubReleases {
    pub fn new(
        repo: String,
        tag_prefix: String,
        manifest_asset: String,
        ref_tag_prefix: String,
        staging_tag_prefix: String,
    ) -> Self {
        Self {
            repo,
            tag_prefix,
            ref_tag_prefix,
            staging_tag_prefix,
            manifest_asset,
            // A failure here means a broken TLS setup, which is fatal for every
            // request anyway; fall back to a default client so construction stays
            // infallible and the error surfaces on first use.
            client: http::client().unwrap_or_default(),
        }
    }

    fn tag_for(&self, version: &semver::Version) -> String {
        format!("{}{}", self.tag_prefix, version)
    }

    /// The tag a named ref resolves to: `daemon-dev-` + the branch name.
    ///
    /// The ref is appended verbatim. Branch names are already valid git refs, so slashes in
    /// `feature/foo` need no handling — and rewriting them would resolve to a tag that does
    /// not exist, failing with "release not found" instead of anything informative.
    fn ref_tag_for(&self, git_ref: &str) -> String {
        format!("{}{}", self.ref_tag_prefix, git_ref)
    }

    /// The tag a release candidate lives under: `daemon-staging-v` + the version.
    fn staging_tag_for(&self, version: &semver::Version) -> String {
        format!("{}{}", self.staging_tag_prefix, version)
    }

    async fn release_for_tag(&self, tag: &str) -> Result<Release, Error> {
        let url = format!(
            "https://api.github.com/repos/{}/releases/tags/{tag}",
            self.repo
        );
        let bytes =
            http::get_bytes(&self.client, &url, Some("application/vnd.github+json")).await?;
        serde_json::from_slice(&bytes)
            .map_err(|e| Error::Network(format!("parsing release {tag}: {e}")))
    }

    /// Highest **stable** semver among matching tags.
    ///
    /// Drafts and prereleases are skipped: a draft isn't published, and a prerelease is
    /// by definition not what a client robot should install. Reach one with an explicit
    /// `--version` or `--ref`.
    ///
    /// Two independent reasons a build is skipped — GitHub's `prerelease` flag *and* a
    /// semver prerelease component — because dev builds (`0.2.0-dev.5.abc1234`) must
    /// never become `latest` for the fleet, and relying on someone remembering a
    /// checkbox is not a safeguard.
    async fn newest_version(&self) -> Result<semver::Version, Error> {
        self.newest_under(&self.tag_prefix, false).await
    }

    /// The newest release candidate, which is the same scan with the prerelease flag allowed.
    ///
    /// Only the *GitHub* flag is allowed, never a semver prerelease: candidates carry the plain
    /// version they will be promoted under (`0.3.0`), while dev builds carry `0.3.0-dev.17.abc`
    /// and live under a third prefix. So the two exclusions in [`Self::newest_under`] are not
    /// redundant here — dropping one still excludes branch builds, which is the point.
    async fn newest_staging_version(&self) -> Result<semver::Version, Error> {
        self.newest_under(&self.staging_tag_prefix, true).await
    }

    /// Highest version among tags carrying `prefix`.
    ///
    /// `allow_flagged_prerelease` is the whole difference between the stable channel and the
    /// staging one, and it is a parameter rather than a field so that the call site — one of
    /// exactly two — states which channel it means.
    async fn newest_under(
        &self,
        prefix: &str,
        allow_flagged_prerelease: bool,
    ) -> Result<semver::Version, Error> {
        let mut best: Option<semver::Version> = None;

        for page in 1..=MAX_PAGES {
            let url = format!(
                "https://api.github.com/repos/{}/releases?per_page={PER_PAGE}&page={page}",
                self.repo
            );
            let bytes =
                http::get_bytes(&self.client, &url, Some("application/vnd.github+json")).await?;
            let releases: Vec<Release> = serde_json::from_slice(&bytes)
                .map_err(|e| Error::Network(format!("parsing releases: {e}")))?;

            let count = releases.len();
            for release in releases {
                if release.draft || (release.prerelease && !allow_flagged_prerelease) {
                    continue;
                }
                if let Some(version) = version_under(prefix, &release.tag_name)
                    // A semver prerelease is a dev build, whatever the release was flagged as.
                    && version.pre.is_empty()
                    && best.as_ref().is_none_or(|b| version > *b)
                {
                    best = Some(version);
                }
            }

            // A short page is the last page.
            if count < PER_PAGE {
                break;
            }
        }

        best.ok_or_else(|| {
            Error::Network(format!(
                "no releases in {} with tag prefix {prefix:?}",
                self.repo
            ))
        })
    }

    /// The API download URL for a named asset. Pair it with [`OCTET_STREAM`].
    fn asset_url(release: &Release, name: &str) -> Result<String, Error> {
        release
            .assets
            .iter()
            .find(|a| a.name == name)
            .map(|a| a.url.clone())
            .ok_or_else(|| {
                let available: Vec<_> = release.assets.iter().map(|a| a.name.as_str()).collect();
                Error::Network(format!(
                    "release {} has no asset named {name:?} (has: {})",
                    release.tag_name,
                    available.join(", ")
                ))
            })
    }

    /// Split one of our own release-download URLs into `(tag, asset name)`.
    ///
    /// `None` for anything else, including another repository's release URL — which matters
    /// because the manifest this comes from is unverified at that point, so a URL naming a
    /// foreign repo must not become an authenticated API request against it.
    fn split_release_url(&self, url: &str) -> Option<(String, String)> {
        let prefix = format!("https://github.com/{}/releases/download/", self.repo);
        let (tag, name) = url.strip_prefix(&prefix)?.split_once('/')?;
        Some((tag.to_owned(), name.to_owned()))
    }

    /// Where to actually fetch a URL from a signed manifest, and with which `Accept`.
    ///
    /// A private repo's `releases/download/...` URL 404s even with a token, so one of ours is
    /// re-resolved through the release API. Anything else is fetched verbatim — a manifest
    /// pointing at a CDN keeps working, and the bytes are hash- and signature-checked either
    /// way.
    async fn resolve_download(&self, url: &str) -> Result<(String, Option<&'static str>), Error> {
        let Some((tag, name)) = self.split_release_url(url) else {
            return Ok((url.to_owned(), None));
        };

        let release = self.release_for_tag(&tag).await?;
        let api_url = Self::asset_url(&release, &name)?;
        tracing::debug!(%tag, %name, "resolved asset through the release API");
        Ok((api_url, Some(OCTET_STREAM)))
    }

    async fn signed_manifest(&self, tag: &str) -> Result<SignedBytes<Manifest>, Error> {
        let release = self.release_for_tag(tag).await?;

        let manifest_url = Self::asset_url(&release, &self.manifest_asset)?;
        let sig_name = format!("{}{SIG_SUFFIX}", self.manifest_asset);
        let sig_url = Self::asset_url(&release, &sig_name)?;

        let bytes = http::get_bytes(&self.client, &manifest_url, Some(OCTET_STREAM)).await?;
        let signature = http::get_bytes(&self.client, &sig_url, Some(OCTET_STREAM)).await?;

        // Parsed for convenience only; the *bytes* are what the caller verifies,
        // since the signature covers exactly what was received.
        let parsed: Manifest = serde_json::from_slice(&bytes)
            .map_err(|e| Error::Corrupt(format!("manifest at {manifest_url}: {e}")))?;

        Ok(SignedBytes {
            bytes,
            signature,
            parsed,
        })
    }
}

#[async_trait::async_trait]
impl Source for GithubReleases {
    async fn latest_manifest(&self) -> Result<SignedBytes<Manifest>, Error> {
        let version = self.newest_version().await?;
        let tag = self.tag_for(&version);
        tracing::debug!(repo = %self.repo, %tag, "resolved latest");
        self.signed_manifest(&tag).await
    }

    async fn manifest_for(
        &self,
        version: &semver::Version,
    ) -> Result<SignedBytes<Manifest>, Error> {
        self.signed_manifest(&self.tag_for(version)).await
    }

    async fn manifest_at_ref(&self, git_ref: &str) -> Result<SignedBytes<Manifest>, Error> {
        let tag = self.ref_tag_for(git_ref);
        tracing::debug!(repo = %self.repo, %tag, %git_ref, "resolving ref");
        self.signed_manifest(&tag).await
    }

    async fn staging_manifest(&self) -> Result<SignedBytes<Manifest>, Error> {
        let version = self.newest_staging_version().await?;
        let tag = self.staging_tag_for(&version);
        tracing::debug!(repo = %self.repo, %tag, "resolved newest candidate");
        self.signed_manifest(&tag).await
    }

    async fn staging_manifest_for(
        &self,
        version: &semver::Version,
    ) -> Result<SignedBytes<Manifest>, Error> {
        self.signed_manifest(&self.staging_tag_for(version)).await
    }

    async fn fetch_artifact(
        &self,
        manifest: &Manifest,
        dest_dir: &Path,
        progress: ProgressSink,
    ) -> Result<FetchedArtifact, Error> {
        tokio::fs::create_dir_all(dest_dir)
            .await
            .map_err(|e| Error::Io {
                path: dest_dir.to_path_buf(),
                source: e,
            })?;

        // The filename comes from a signed manifest, but the signature is only
        // checked *after* download — so treat it as untrusted here and refuse
        // anything that isn't a bare name.
        let artifact_name = safe_file_name(&manifest.url)?;
        let artifact = dest_dir.join(&artifact_name);
        let signature = dest_dir.join(format!("{artifact_name}{SIG_SUFFIX}"));

        let (artifact_url, accept) = self.resolve_download(&manifest.url).await?;
        let bytes =
            http::download_to(&self.client, &artifact_url, &artifact, accept, &progress).await?;

        let (sig_url, sig_accept) = self.resolve_download(&manifest.sig_url).await?;
        let sig_bytes = http::get_bytes(&self.client, &sig_url, sig_accept).await?;
        tokio::fs::write(&signature, &sig_bytes)
            .await
            .map_err(|e| Error::Io {
                path: signature.clone(),
                source: e,
            })?;

        Ok(FetchedArtifact {
            artifact,
            signature,
            bytes,
        })
    }
}

/// Parse a version out of a tag carrying `prefix`, or `None` if the tag is not one of ours.
///
/// Free-standing because two prefixes now feed it — stable and staging — and a method reading
/// `self.tag_prefix` invited exactly the bug where a staging scan silently measured stable tags.
fn version_under(prefix: &str, tag: &str) -> Option<semver::Version> {
    semver::Version::parse(tag.strip_prefix(prefix)?).ok()
}

/// Extract a filename from a URL, refusing anything that could escape `dest_dir`.
///
/// A manifest is signed, but this runs *before* verification, and a compromised
/// publisher is exactly the case where writing to an arbitrary path would matter.
pub(crate) fn safe_file_name(url: &str) -> Result<String, Error> {
    let tail = url.rsplit('/').next().unwrap_or_default();
    let tail = tail.split(['?', '#']).next().unwrap_or_default();

    let looks_safe = !tail.is_empty()
        && tail != "."
        && tail != ".."
        && !tail.contains('/')
        && !tail.contains('\\')
        && !tail.contains('\0');

    if looks_safe {
        Ok(tail.to_owned())
    } else {
        Err(Error::Verification(format!(
            "manifest url {url:?} does not end in a usable filename"
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source() -> GithubReleases {
        GithubReleases::new(
            "ORG/robot-daemon".into(),
            "daemon-v".into(),
            "manifest.json".into(),
            "daemon-dev-".into(),
            "daemon-staging-v".into(),
        )
    }

    #[test]
    fn tags_round_trip() {
        let s = source();
        let v = semver::Version::new(1, 4, 2);
        assert_eq!(s.tag_for(&v), "daemon-v1.4.2");
        assert_eq!(version_under(&s.tag_prefix, "daemon-v1.4.2"), Some(v));
    }

    #[test]
    fn staging_tags_round_trip() {
        let s = source();
        let v = semver::Version::new(0, 3, 0);
        assert_eq!(s.staging_tag_for(&v), "daemon-staging-v0.3.0");
        assert_eq!(
            version_under("daemon-staging-v", "daemon-staging-v0.3.0"),
            Some(v)
        );
    }

    /// The two channels must not read each other's tags.
    ///
    /// This is the failure that would matter and would not look like one: a staging scan that
    /// silently matched `daemon-v*` would report the newest *stable* release as the candidate,
    /// and `--staging` would install what a plain `apply` already installs while claiming to
    /// test something. It holds because `daemon-staging-v0.3.0` does not start with `daemon-v`
    /// — an accident of naming, so it is pinned here rather than left to be re-derived.
    #[test]
    fn the_two_channels_cannot_read_each_others_tags() {
        assert_eq!(version_under("daemon-v", "daemon-staging-v0.3.0"), None);
        assert_eq!(version_under("daemon-staging-v", "daemon-v0.3.0"), None);
        // And neither reads a dev build, which is what keeps `--staging` from resolving to a
        // branch someone pushed.
        assert_eq!(
            version_under("daemon-staging-v", "daemon-dev-my-branch"),
            None
        );
    }

    /// A ref becomes a dev tag, and the ref is appended verbatim.
    ///
    /// The slash case is the one that matters: `feature/foo` is a valid branch name, so
    /// anything that sanitised it would resolve to a tag nobody published and fail with
    /// "release not found" rather than anything that points at the cause.
    #[test]
    fn refs_become_dev_tags_verbatim() {
        let s = source();
        assert_eq!(s.ref_tag_for("my-branch"), "daemon-dev-my-branch");
        assert_eq!(s.ref_tag_for("feature/foo"), "daemon-dev-feature/foo");
    }

    /// **A dev tag must never be mistaken for a release.** `version_under` drives
    /// `newest_version`, which is what the fleet installs — so if a dev tag parsed as a
    /// version here, a branch build could become `latest` for every robot. That is the
    /// failure the two independent guards exist to prevent, and this is the first of them.
    #[test]
    fn a_dev_tag_is_not_a_release_version() {
        let s = source();
        assert_eq!(version_under(&s.tag_prefix, "daemon-dev-my-branch"), None);
        // Even when the dev tag ends in something version-shaped.
        assert_eq!(version_under(&s.tag_prefix, "daemon-dev-0.2.0"), None);
        // And the staging stream stays separate too.
        assert_eq!(version_under(&s.tag_prefix, "daemon-staging-v0.2.0"), None);
    }

    /// Another channel's tags in the same repo must be ignored, not misparsed.
    #[test]
    fn foreign_tags_are_ignored() {
        let s = source();
        assert_eq!(version_under(&s.tag_prefix, "model-v3.0.0"), None);
        assert_eq!(version_under(&s.tag_prefix, "v1.0.0"), None);
        assert_eq!(version_under(&s.tag_prefix, "daemon-vnot-a-version"), None);
    }

    #[test]
    fn asset_lookup_lists_what_was_available_on_failure() {
        let release = Release {
            tag_name: "daemon-v1.0.0".into(),
            draft: false,
            prerelease: false,
            assets: vec![Asset {
                name: "other.txt".into(),
                url: "https://api.github.com/repos/ORG/robot-daemon/releases/assets/1".into(),
            }],
        };
        let err = GithubReleases::asset_url(&release, "manifest.json").unwrap_err();
        // A support ticket needs to see what *was* there.
        assert!(err.to_string().contains("other.txt"), "{err}");
    }

    /// **A manifest must not redirect the asset lookup at another repository.**
    ///
    /// `resolve_download` runs before the manifest's signature is checked, so a URL naming a
    /// foreign repo must be left alone rather than turned into an API request carrying our
    /// token.
    #[test]
    fn only_our_own_release_urls_are_split() {
        let s = source();

        assert_eq!(
            s.split_release_url(
                "https://github.com/ORG/robot-daemon/releases/download/daemon-dev-my-branch/daemon-0.2.0-dev.1.abc1234.tar.zst"
            ),
            Some((
                "daemon-dev-my-branch".to_owned(),
                "daemon-0.2.0-dev.1.abc1234.tar.zst".to_owned()
            ))
        );

        for foreign in [
            "https://github.com/attacker/repo/releases/download/v1/x.tar.zst",
            "https://cdn.example.com/daemon-1.0.0.tar.zst",
            // Ours, but not a release-download URL.
            "https://github.com/ORG/robot-daemon/archive/refs/heads/main.tar.gz",
        ] {
            assert_eq!(s.split_release_url(foreign), None, "{foreign}");
        }
    }

    #[test]
    fn accepts_a_plain_filename() {
        assert_eq!(
            safe_file_name("https://example.com/a/b/daemon-1.0.0.tar.zst").unwrap(),
            "daemon-1.0.0.tar.zst"
        );
        // Query strings are common on signed CDN URLs.
        assert_eq!(
            safe_file_name("https://example.com/x.tar.zst?token=abc").unwrap(),
            "x.tar.zst"
        );
    }

    /// The download path must not be steerable by a manifest.
    #[test]
    fn refuses_names_that_could_escape() {
        for url in [
            "https://example.com/",
            "https://example.com/..",
            "https://example.com/.",
            "https://example.com/a/",
        ] {
            assert!(safe_file_name(url).is_err(), "should refuse {url}");
        }
    }

    /// A dev build must never be selected as `latest`, even if whoever published it
    /// forgot to tick "prerelease". Fleet-wide auto-updates read `latest`.
    #[test]
    fn dev_versions_are_recognised_as_prereleases() {
        let s = source();
        let dev = version_under(&s.tag_prefix, "daemon-v0.2.0-dev.5.abc1234").unwrap();
        assert!(
            !dev.pre.is_empty(),
            "dev builds must carry a semver prerelease"
        );

        let stable = version_under(&s.tag_prefix, "daemon-v0.2.0").unwrap();
        assert!(stable.pre.is_empty());
        // And a dev build sorts *below* the release it precedes, so it can never look
        // like an upgrade from it.
        assert!(dev < stable);
    }

    /// GitHub adds response fields regularly; deserialisation must not break.
    #[test]
    fn unknown_release_fields_are_tolerated() {
        let json = serde_json::json!({
            "tag_name": "daemon-v1.0.0",
            "some_new_field": 42,
            "assets": [{
                "name": "manifest.json",
                "url": "https://api.github.com/repos/ORG/robot-daemon/releases/assets/7",
                "browser_download_url": "https://example/m.json",
                "another_new_field": true
            }]
        });
        let release: Release = serde_json::from_value(json).unwrap();
        assert_eq!(release.tag_name, "daemon-v1.0.0");
        assert!(!release.draft, "missing `draft` should default to false");
        // `url` is required, not defaulted: it is how assets are fetched, and a release whose
        // assets lack it is not something to paper over with an empty string that would fail
        // later as a confusing HTTP error.
        assert!(release.assets[0].url.contains("/releases/assets/"));
    }
}
