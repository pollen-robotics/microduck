//! Where artifacts come from.
//!
//! One trait, three backends: GitHub Releases (daemon), HF Hub (models), and a
//! local directory. The local backend is not a toy — it is how the engine gets
//! tested against its real code path with no network
//! (`docs/design/updater-design.md` §16.1).

use std::path::{Path, PathBuf};

use crate::Error;
use crate::config::SourceConfig;
use crate::manifest::Manifest;

pub(crate) mod github;
mod hf_hub;
pub mod http;
mod local;

pub use github::GithubReleases;
pub use hf_hub::HfHub;
pub use local::LocalDir;

/// Reports download progress as `(bytes_so_far, total_if_known)`.
///
/// A plain channel sender rather than a closure: the download happens in a task,
/// and progress has to reach subscribers without borrowing the caller's stack.
pub type ProgressSink = tokio::sync::mpsc::UnboundedSender<(u64, Option<u64>)>;

#[async_trait::async_trait]
pub trait Source: Send + Sync {
    /// Fetch the manifest the source currently advertises as newest.
    ///
    /// Returns the raw bytes alongside the parsed manifest: the signature covers
    /// the bytes, so verification must happen against exactly what was received,
    /// not against a re-serialization.
    async fn latest_manifest(&self) -> Result<SignedBytes<Manifest>, Error>;

    /// Fetch the manifest for an exact version. Backs
    /// `robotctl update apply --version X`.
    async fn manifest_for(&self, version: &semver::Version)
    -> Result<SignedBytes<Manifest>, Error>;

    /// Fetch the manifest a named ref currently points at. Backs
    /// `robotctl update apply --ref my-branch`.
    ///
    /// Defaulted rather than required because "a ref" is not meaningful for every source: a
    /// local directory has no refs, and a source that cannot resolve one should say so
    /// rather than guess. Sources that *can* override it.
    async fn manifest_at_ref(&self, git_ref: &str) -> Result<SignedBytes<Manifest>, Error> {
        Err(Error::Incompatible(format!(
            "this source cannot resolve the ref {git_ref:?}; \
             only a github_releases source publishes per-branch builds"
        )))
    }

    /// Fetch the newest release candidate. Backs `robotctl update apply --staging`.
    ///
    /// Defaulted for the same reason as [`Source::manifest_at_ref`]: a channel split is a
    /// property of how a source publishes, and a local directory has no candidates to offer.
    async fn staging_manifest(&self) -> Result<SignedBytes<Manifest>, Error> {
        Err(Error::Incompatible(
            "this source has no staging channel; \
             only a github_releases source publishes release candidates"
                .to_owned(),
        ))
    }

    /// Fetch one named candidate. Backs `--staging --version X`.
    async fn staging_manifest_for(
        &self,
        version: &semver::Version,
    ) -> Result<SignedBytes<Manifest>, Error> {
        Err(Error::Incompatible(format!(
            "this source has no staging channel, so it cannot resolve the candidate {version}"
        )))
    }

    /// Download the artifact and its detached signature into `dest_dir`.
    ///
    /// Streams to disk — must not assume the artifact fits in memory — and is
    /// cancel-safe: dropping the future must leave only staging garbage, never a
    /// partially installed release.
    async fn fetch_artifact(
        &self,
        manifest: &Manifest,
        dest_dir: &Path,
        progress: ProgressSink,
    ) -> Result<FetchedArtifact, Error>;
}

/// Bytes as received, plus what they parsed into.
///
/// Keeping both is what lets signature verification run over the exact received
/// bytes.
#[derive(Debug, Clone)]
pub struct SignedBytes<T> {
    pub bytes: Vec<u8>,
    pub signature: Vec<u8>,
    pub parsed: T,
}

#[derive(Debug, Clone)]
pub struct FetchedArtifact {
    pub artifact: PathBuf,
    pub signature: PathBuf,
    pub bytes: u64,
}

/// Build a source from config.
pub fn from_config(config: &SourceConfig) -> Box<dyn Source> {
    match config {
        SourceConfig::GithubReleases {
            repo,
            tag_prefix,
            manifest_asset,
            ref_tag_prefix,
            staging_tag_prefix,
        } => Box::new(GithubReleases::new(
            repo.clone(),
            tag_prefix.clone(),
            manifest_asset.clone(),
            ref_tag_prefix.clone(),
            staging_tag_prefix.clone(),
        )),
        SourceConfig::HfHub {
            repo,
            revision,
            manifest_file,
        } => Box::new(HfHub::new(
            repo.clone(),
            revision.clone(),
            manifest_file.clone(),
        )),
        SourceConfig::LocalDir { path } => Box::new(LocalDir::new(path.clone())),
    }
}
