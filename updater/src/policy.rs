//! The official policy set: what is installed, what the Hub offers, and moving between them.
//!
//! **Why this is in `updaterd` at all.** It is not an update in the component sense — there is no
//! manifest, no signature, no health gate and no rollback, because a policy is not a binary
//! (`docs/design/policy-channel-design.md` §2). What it needs is a network stack and root, and
//! this is the daemon that has both: `robotd` has neither by design, and `robotctl` must not link
//! an HTTP client because it is the tool that has to work when everything else is broken.
//!
//! **Why it exists.** A board installs its set from a pin that ships inside the daemon release
//! (`scripts/seed-policies.sh`), which makes the pin a *floor* rather than a ceiling: without
//! something to move past it, a retrained gait would still need a daemon release to reach a
//! robot, which is the thing the whole channel was meant to stop. This is that something.
//!
//! The layout is the seeder's, and deliberately: `releases/<name>/` beside a `current` symlink,
//! swapped by rename. So a set installed here and a set installed by the seeder are the same
//! kind of thing, and each carries a `.source` record saying where it came from — which is how
//! [`check`] knows what repo to ask without anyone configuring it twice.

use std::path::{Path, PathBuf};

use crate::Error;
use crate::source::http;

/// Where the sets live. Matches `robotd_params::POLICY_DIR`'s parent and the seeder's default.
pub const POLICY_ROOT: &str = "/opt/robot/policies";

/// The provenance record the seeder writes beside a set.
const SOURCE_FILE: &str = ".source";

/// What a set says about itself, installed beside the policies it describes. `robotd` reads it
/// from `<current>/` to know which of them are skills.
const MANIFEST_FILE: &str = "manifest.json";

/// Sets installed from here are named for their revision, and the prefix marks them as *ours* —
/// the seeder uses the same one, and both refuse to disturb a `current` that has neither.
const SET_PREFIX: &str = "seed-";

/// What a policy set records about where it came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Source {
    pub repo: String,
    pub version: String,
}

impl Source {
    /// Parse the `key=value` file the seeder writes. Unknown keys are ignored, because a set
    /// installed by a newer tool must not be unreadable to an older daemon.
    pub fn parse(text: &str) -> Option<Self> {
        let mut repo = None;
        let mut version = None;
        for line in text.lines() {
            match line.split_once('=') {
                Some(("repo", v)) => repo = Some(v.trim().to_owned()),
                Some(("version", v)) => version = Some(v.trim().to_owned()),
                _ => {}
            }
        }
        Some(Self {
            repo: repo?,
            version: version?,
        })
    }

    fn render(&self) -> String {
        format!("repo={}\nversion={}\n", self.repo, self.version)
    }
}

/// The set `current` points at, and what it says about itself.
pub fn installed(root: &Path) -> Option<Source> {
    let text = std::fs::read_to_string(root.join("current").join(SOURCE_FILE)).ok()?;
    Source::parse(&text)
}

/// Every `.onnx` in the installed set — the fallback download list.
///
/// **Second choice.** [`set_files`] asks the revision what it contains, which is the only list
/// that can grow; this one can only ever re-fetch what the board already has, so a revision that
/// added a tenth policy would install nine and the tag would have been for nothing. It is here
/// for a revision tagged before the manifest existed, and goes when every tagged set carries one
/// — the same rule, and the same wording, as `seed-policies.sh`'s `FALLBACK_FILES`.
fn installed_files(root: &Path) -> Vec<String> {
    let Ok(entries) = std::fs::read_dir(root.join("current")) else {
        return Vec::new();
    };
    let mut names: Vec<String> = entries
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".onnx"))
        .collect();
    names.sort();
    names
}

/// A tag's version, as a list of numbers, or `None` when it is not a version at all.
///
/// `v3` is `[3]`, `v1.2.0` is `[1, 2, 0]`, `nightly` is nothing. Numbers rather than a string
/// compare, because `v10` has to sort above `v9` and lexically it does not.
fn version_of(tag: &str) -> Option<Vec<u64>> {
    let parts: Vec<u64> = tag
        .trim_start_matches(['v', 'V'])
        .split(['.', '-', '_'])
        .map(|part| part.parse().ok())
        .collect::<Option<Vec<u64>>>()?;
    (!parts.is_empty()).then_some(parts)
}

/// Tags in a Hub repo, newest first.
///
/// **The Hub returns refs in no useful order and carries no dates on them.** Our own set comes
/// back as `v3, v1, v2`, so neither the list nor its reverse says which is newest — and an
/// earlier version of this took the reverse and confidently reported v2 as newer than v3.
///
/// Ordering is therefore by version: the numbers in the tag, compared as numbers. A tag that is
/// not a version keeps its place at the end and never counts as newest, so
/// `robotctl policy update` cannot land on somebody's `experimental` because it sorted oddly —
/// naming it explicitly still works. That is a narrower claim than an ordering over arbitrary
/// names, which is the guess worth not making.
async fn tags(client: &reqwest::Client, repo: &str) -> Result<Vec<String>, Error> {
    let url = format!("https://huggingface.co/api/models/{repo}/refs");
    let bytes = http::get_bytes(client, &url, None).await?;
    let refs: serde_json::Value = serde_json::from_slice(&bytes)
        .map_err(|e| Error::Network(format!("parsing refs for {repo}: {e}")))?;
    let mut names: Vec<String> = refs
        .get("tags")
        .and_then(|t| t.as_array())
        .map(|tags| {
            tags.iter()
                .filter_map(|t| t.get("name")?.as_str().map(str::to_owned))
                .collect()
        })
        .unwrap_or_default();
    // Versions first and highest first; everything else after, in the order the Hub gave it.
    names.sort_by(|a, b| match (version_of(a), version_of(b)) {
        (Some(a), Some(b)) => b.cmp(&a),
        (Some(_), None) => std::cmp::Ordering::Less,
        (None, Some(_)) => std::cmp::Ordering::Greater,
        (None, None) => std::cmp::Ordering::Equal,
    });
    Ok(names)
}

/// What is installed against what the repo offers.
pub async fn check(root: &Path) -> crate::proto::PolicyCheckResult {
    // Nothing installed is not a network problem, and saying so through `unreachable` reported a
    // healthy Hub as unreachable on a board whose only fault was having no set yet. An empty
    // `repo` says it on its own, unambiguously — there is no other way to get one.
    let Some(source) = installed(root) else {
        return crate::proto::PolicyCheckResult::default();
    };

    let mut result = crate::proto::PolicyCheckResult {
        repo: Some(source.repo.clone()),
        installed: Some(source.version.clone()),
        ..Default::default()
    };

    let client = match http::client() {
        Ok(client) => client,
        Err(e) => {
            result.unreachable = Some(e.to_string());
            return result;
        }
    };
    match tags(&client, &source.repo).await {
        // An unreachable Hub is a fact about the network, not a failure of the question. The
        // robot is walking either way, and a caller shown "could not reach the Hub" beside what
        // is installed knows more than one shown an error.
        Err(e) => result.unreachable = Some(e.to_string()),
        Ok(versions) => {
            result.available = versions.iter().find(|t| version_of(t).is_some()).cloned();
            result.versions = versions;
        }
    }
    result
}

/// What a revision says it contains, and the manifest bytes to install beside it.
///
/// Empty when the revision has no `manifest.json`, or one nothing can be read out of — which is
/// a fact about an older tag rather than a failure, so the caller falls back rather than
/// refusing. The bytes are returned verbatim rather than re-serialised: `robotd` reads fields
/// this crate has no type for, and a round trip through the subset understood here would drop
/// them.
async fn set_files(
    client: &reqwest::Client,
    repo: &str,
    version: &str,
) -> (Vec<String>, Option<Vec<u8>>) {
    let url = format!("https://huggingface.co/{repo}/resolve/{version}/{MANIFEST_FILE}");
    let Ok(bytes) = http::get_bytes(client, &url, None).await else {
        return (Vec::new(), None);
    };
    let files = files_in_manifest(&bytes);
    match files.is_empty() {
        true => (files, None),
        false => (files, Some(bytes)),
    }
}

/// The `file` of every policy a manifest lists, and nothing that is not a plain file name.
///
/// The manifest is somebody else's document — the repo is configurable, and a community set is
/// anyone's — so a `file` naming `../../etc/anything` would otherwise choose the directory it
/// lands in. `seed-policies.sh` applies the same rule to the same field.
fn files_in_manifest(bytes: &[u8]) -> Vec<String> {
    let Ok(manifest) = serde_json::from_slice::<serde_json::Value>(bytes) else {
        return Vec::new();
    };
    manifest
        .get("policies")
        .and_then(|p| p.as_array())
        .map(|policies| {
            policies
                .iter()
                .filter_map(|p| p.get("file")?.as_str())
                .filter(|file| !file.contains(['/', '\\']) && !file.starts_with('.'))
                .map(str::to_owned)
                .collect()
        })
        .unwrap_or_default()
}

/// Download one revision of a repo into `root`, and point `current` at it.
///
/// Nothing partial goes live: the files land in a staging directory and the symlink moves only
/// once every one of them has arrived. Same rule the seeder follows, and for the same reason —
/// a half-written set is one a restarting `robotd` could read.
pub async fn install(
    root: &Path,
    version: Option<&str>,
) -> Result<(String, Option<String>), Error> {
    let source = installed(root).ok_or_else(|| {
        Error::Network("no policy set is installed, so there is no repo to install from".into())
    })?;
    let client = http::client()?;

    let version = match version {
        Some(version) => version.to_owned(),
        // The newest *version*, not merely the first tag: a repo whose tags are all names rather
        // than versions has no newest, and picking one would be a coin toss with a robot's gait
        // riding on it.
        None => tags(&client, &source.repo)
            .await?
            .into_iter()
            .find(|tag| version_of(tag).is_some())
            .ok_or_else(|| {
                Error::Network(format!(
                    "{} has no version tags — name a revision to install",
                    source.repo
                ))
            })?,
    };

    let previous = Some(source.version.clone()).filter(|v| *v != version);
    if previous.is_none() {
        return Ok((version, None));
    }

    let (files, manifest) = set_files(&client, &source.repo, &version).await;
    let files = match files.is_empty() {
        false => files,
        true => installed_files(root),
    };
    if files.is_empty() {
        return Err(Error::Network(format!(
            "{}@{version} lists no policies, and the installed set has no .onnx files to \
             replace — there is nothing to fetch",
            source.repo
        )));
    }

    let staging = root.join("releases").join(".staging-install");
    let _ = std::fs::remove_dir_all(&staging);
    std::fs::create_dir_all(&staging).map_err(|e| Error::Io {
        path: staging.clone(),
        source: e,
    })?;

    // `download_to`, not `get_bytes`. The latter is for manifests and API replies and enforces a
    // one-megabyte ceiling with the message "implausibly large for metadata" — which a policy is
    // not, and which today's are only just under. A retrain that produced a slightly larger
    // network would have failed to install with a sentence about metadata.
    let (progress, mut drain) = tokio::sync::mpsc::unbounded_channel();
    tokio::spawn(async move { while drain.recv().await.is_some() {} });
    for name in &files {
        let url = format!(
            "https://huggingface.co/{}/resolve/{version}/{name}",
            source.repo
        );
        http::download_to(&client, &url, &staging.join(name), None, &progress)
            .await
            .inspect_err(|_| {
                // Half a set is worse than none: leaving it would make the next attempt look
                // like a resume of something that was never coherent.
                let _ = std::fs::remove_dir_all(&staging);
            })?;
    }

    // **The manifest is installed with the set, not just read to build the list.** `robotd`
    // reads `<current>/manifest.json` to know which policies are skills and how each one is
    // tuned; a set installed without it falls back to the three names this build was compiled
    // with. So an update that fetched only the `.onnx` files would quietly undo every skill the
    // set declares — the exact coupling the manifest exists to remove, reintroduced by the
    // command whose whole job is moving between revisions.
    if let Some(manifest) = &manifest {
        std::fs::write(staging.join(MANIFEST_FILE), manifest).map_err(|e| Error::Io {
            path: staging.join(MANIFEST_FILE),
            source: e,
        })?;
    }

    let recorded = Source {
        repo: source.repo.clone(),
        version: version.clone(),
    };
    std::fs::write(staging.join(SOURCE_FILE), recorded.render()).map_err(|e| Error::Io {
        path: staging.join(SOURCE_FILE),
        source: e,
    })?;

    let name = format!("{SET_PREFIX}{version}");
    let dest = root.join("releases").join(&name);
    let _ = std::fs::remove_dir_all(&dest);
    std::fs::rename(&staging, &dest).map_err(|e| Error::Io {
        path: dest.clone(),
        source: e,
    })?;

    swap_current(root, &name)?;
    prune(root, &name, previous.as_deref());
    Ok((version, previous))
}

/// Drop older sets, keeping the live one and the one it replaced.
///
/// Every install is a new directory of about seven megabytes, in a place nothing else tidies, so
/// somebody moving back and forth between revisions would fill an eMMC. The previous one is kept
/// deliberately: **rollback does not run hooks** (`Engine::post_swap` is on the apply path only),
/// so reverting the daemon does not revert its policies, and pointing `current` back at the kept
/// set is the recovery when a policy is what went wrong.
///
/// Best effort. A set that cannot be removed is disk space, not a failed install, and undoing a
/// good install over it would be the wrong trade.
fn prune(root: &Path, keep: &str, previous: Option<&str>) {
    let previous = previous.map(|v| format!("{SET_PREFIX}{v}"));
    let Ok(entries) = std::fs::read_dir(root.join("releases")) else {
        return;
    };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        // `seed-` only. Anything else under here belongs to whatever installed it, which is the
        // rule the seeder opens with and this must not be the exception to.
        if !name.starts_with(SET_PREFIX) || name == keep || Some(&name) == previous.as_ref() {
            continue;
        }
        if let Err(e) = std::fs::remove_dir_all(entry.path()) {
            tracing::warn!(set = %name, error = %e, "could not remove an old policy set");
        }
    }
}

/// Point `current` at `releases/<name>`, atomically.
///
/// Relative to the link's own directory, like the seeder writes it and like the updater's own
/// `current` — an absolute target resolves against the wrong place the moment the root moves.
fn swap_current(root: &Path, name: &str) -> Result<(), Error> {
    let staged = root.join("current.new");
    let _ = std::fs::remove_file(&staged);
    let target: PathBuf = ["releases", name].iter().collect();
    std::os::unix::fs::symlink(&target, &staged).map_err(|e| Error::Io {
        path: staged.clone(),
        source: e,
    })?;
    std::fs::rename(&staged, root.join("current")).map_err(|e| Error::Io {
        path: root.join("current"),
        source: e,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_source_record_round_trips() {
        let source = Source {
            repo: "pollen-robotics/microduck-policies".into(),
            version: "v1".into(),
        };
        assert_eq!(Source::parse(&source.render()), Some(source));
    }

    /// The seeder writes a `fetched=` line this does not care about, and a later tool may write
    /// more. An older daemon must still be able to read where a set came from — the alternative
    /// is a robot that cannot answer "what am I running" because something added a field.
    #[test]
    fn unknown_keys_in_a_source_record_are_ignored() {
        let text = "repo=org/set\nversion=v3\nfetched=2026-08-31T00:00:00Z\nfuture=whatever\n";
        assert_eq!(
            Source::parse(text),
            Some(Source {
                repo: "org/set".into(),
                version: "v3".into()
            })
        );
    }

    /// A record missing either half is not a record. Guessing a repo would send `check` to ask
    /// the wrong one, and guessing a version would report an upgrade that is really a sidegrade.
    #[test]
    fn an_incomplete_source_record_is_no_record() {
        assert_eq!(Source::parse("repo=org/set\n"), None);
        assert_eq!(Source::parse("version=v1\n"), None);
        assert_eq!(Source::parse(""), None);
    }

    /// `current` is a relative symlink beside `releases/`, so a root that moves — a test
    /// directory, a board with a different layout — keeps working.
    #[test]
    fn current_is_swapped_to_a_relative_target() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        std::fs::create_dir_all(root.join("releases/seed-v2")).unwrap();
        std::fs::write(root.join("releases/seed-v2/alpha_walking.onnx"), "w").unwrap();

        swap_current(root, "seed-v2").unwrap();
        assert_eq!(
            std::fs::read_link(root.join("current")).unwrap(),
            Path::new("releases/seed-v2")
        );
        assert!(root.join("current/alpha_walking.onnx").exists());
    }

    /// Swapping over an existing link replaces it rather than landing inside what it points at,
    /// which is the mistake a plain `mv` onto a symlink-to-directory makes.
    #[test]
    fn swapping_replaces_an_existing_link() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        for name in ["seed-v1", "seed-v2"] {
            std::fs::create_dir_all(root.join("releases").join(name)).unwrap();
        }
        swap_current(root, "seed-v1").unwrap();
        swap_current(root, "seed-v2").unwrap();

        assert_eq!(
            std::fs::read_link(root.join("current")).unwrap(),
            Path::new("releases/seed-v2")
        );
        assert!(!root.join("releases/seed-v1/releases").exists());
    }

    /// **The download list comes from the revision being installed**, which is the only list
    /// that can contain a policy the board has never seen. Taking it from the installed set
    /// instead would make a tenth policy unreachable by `policy update` — installable only by a
    /// daemon release, which is what the whole channel exists to stop.
    #[test]
    fn the_download_list_is_what_the_revision_says_it_has() {
        let manifest = serde_json::json!({
            "schema_version": 1,
            "policies": [
                { "file": "alpha_walking.onnx", "kind": "perpetual" },
                { "file": "polite_bow.onnx", "kind": "episodic", "duration_s": 1.0 },
            ]
        });
        assert_eq!(
            files_in_manifest(&serde_json::to_vec(&manifest).unwrap()),
            vec![
                "alpha_walking.onnx".to_string(),
                "polite_bow.onnx".to_string()
            ]
        );
    }

    /// The manifest is somebody else's document and its `file` is pasted into a path. A name
    /// that climbs out of the staging directory is dropped rather than fetched.
    #[test]
    fn a_manifest_file_name_cannot_leave_the_staging_directory() {
        let manifest = serde_json::json!({
            "policies": [
                { "file": "../../etc/systemd/system/robotd.service" },
                { "file": "sub/dir/policy.onnx" },
                { "file": ".source" },
                { "file": "good.onnx" },
            ]
        });
        assert_eq!(
            files_in_manifest(&serde_json::to_vec(&manifest).unwrap()),
            vec!["good.onnx".to_string()]
        );
    }

    /// A revision tagged before the manifest existed says nothing, which is a fact about the tag
    /// rather than a failure — `install` falls back to what the board holds.
    #[test]
    fn a_revision_without_a_manifest_says_nothing() {
        assert!(files_in_manifest(b"not json at all").is_empty());
        assert!(files_in_manifest(b"{}").is_empty());
        assert!(files_in_manifest(br#"{"policies": []}"#).is_empty());
    }

    /// The fallback list is what is on the board, so a revision with no manifest still replaces
    /// a set file for file. The `.source` record is not one of them.
    #[test]
    fn the_fetch_list_is_what_the_installed_set_holds() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        std::fs::create_dir_all(root.join("releases/seed-v1")).unwrap();
        for name in ["alpha_walking.onnx", "roulade.onnx"] {
            std::fs::write(root.join("releases/seed-v1").join(name), "x").unwrap();
        }
        std::fs::write(
            root.join("releases/seed-v1/.source"),
            "repo=o/r\nversion=v1\n",
        )
        .unwrap();
        swap_current(root, "seed-v1").unwrap();

        assert_eq!(
            installed_files(root),
            vec!["alpha_walking.onnx".to_string(), "roulade.onnx".to_string()],
            "the .source record is not a policy"
        );
    }

    /// Installs accumulate a directory each, so moving between revisions a few times would fill
    /// an eMMC with sets nothing else tidies.
    #[test]
    fn old_sets_are_pruned_to_the_live_one_and_its_predecessor() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        for name in ["seed-v1", "seed-v2", "seed-v3", "from-a-tool"] {
            std::fs::create_dir_all(root.join("releases").join(name)).unwrap();
        }

        prune(root, "seed-v3", Some("v2"));

        let mut left: Vec<String> = std::fs::read_dir(root.join("releases"))
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        left.sort();
        assert_eq!(
            left,
            vec![
                "from-a-tool".to_string(),
                "seed-v2".to_string(),
                "seed-v3".to_string()
            ],
            "the live set, the one it replaced, and anything that is not ours"
        );
    }

    /// A first install has no predecessor, and must not read that as licence to keep nothing —
    /// nor to delete a set some other tool put there.
    #[test]
    fn pruning_without_a_predecessor_keeps_what_is_not_ours() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        for name in ["seed-v1", "from-a-tool"] {
            std::fs::create_dir_all(root.join("releases").join(name)).unwrap();
        }

        prune(root, "seed-v1", None);

        assert!(root.join("releases/seed-v1").exists());
        assert!(root.join("releases/from-a-tool").exists());
    }

    /// A revision directory, with an mtime old enough to order it.
    fn revision(dir: &Path, name: &str, age_s: u64) -> std::path::PathBuf {
        let path = dir.join(name);
        std::fs::create_dir_all(&path).expect("mkdir");
        std::fs::write(path.join("policy.onnx"), name).expect("policy");
        let when = std::time::SystemTime::now() - std::time::Duration::from_secs(age_s);
        std::fs::File::open(&path)
            .and_then(|f| f.set_times(std::fs::FileTimes::new().set_modified(when)))
            .expect("mtime");
        path
    }

    /// **The library was the one directory here that nothing tidied.** Every `policy load
    /// <org/repo>` left ~800 KB behind for good, and `policy.fetch` is served over both radio
    /// transports — so filling a robot's eMMC took no more than a loop.
    #[test]
    fn old_revisions_of_a_repo_are_pruned_to_the_new_one_and_its_predecessor() {
        let tmp = tempfile::tempdir().unwrap();
        let repo = tmp.path().join("someone/a-gait");
        for (name, age) in [("main", 0), ("v3", 10), ("v2", 20), ("v1", 30)] {
            revision(&repo, name, age);
        }

        prune_library(&repo, "main", Some(&[]));

        let mut left: Vec<String> = std::fs::read_dir(&repo)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        left.sort();
        assert_eq!(
            left,
            vec!["main".to_string(), "v3".to_string()],
            "the one just fetched and the one before it"
        );
    }

    /// **A revision the robot is running is kept however old it is.** Slots and skills both: a
    /// gait fetched last month and left in the walk slot is exactly what a prune must not take,
    /// and so is the bow somebody put on a button.
    #[test]
    fn a_revision_the_robot_is_using_is_never_pruned() {
        let tmp = tempfile::tempdir().unwrap();
        let repo = tmp.path().join("someone/a-gait");
        let old = revision(&repo, "v1", 300);
        for (name, age) in [("main", 0), ("v3", 10), ("v2", 20)] {
            revision(&repo, name, age);
        }

        let in_use = vec![old.join("policy.onnx").display().to_string()];
        prune_library(&repo, "main", Some(&in_use));

        assert!(old.exists(), "the gait the robot is running");
        assert!(repo.join("v3").exists(), "and the predecessor");
        assert!(!repo.join("v2").exists(), "but not the rest");
    }

    /// **A robot that did not answer prunes nothing.** Silence is not "using none of them" —
    /// it is a `robotd` that is down, which is the one state where deleting the policy it is
    /// about to come back up on would be unrecoverable.
    #[test]
    fn nothing_is_pruned_when_the_robot_did_not_say() {
        let tmp = tempfile::tempdir().unwrap();
        let repo = tmp.path().join("someone/a-gait");
        for (name, age) in [("main", 0), ("v3", 10), ("v2", 20), ("v1", 30)] {
            revision(&repo, name, age);
        }

        prune_library(&repo, "main", None);

        assert_eq!(std::fs::read_dir(&repo).unwrap().count(), 4);
    }

    fn here() -> Expectations {
        Expectations::here(Some(1))
    }

    fn manifest(json: serde_json::Value) -> PolicyManifest {
        serde_json::from_value(json).expect("a manifest")
    }

    /// **The real convention, taken from a policy actually published to the Hub.** These are the
    /// fields `RemiFabre/microduck-flamingo-cycle` carries, and this asserts we read the ones we
    /// act on rather than a shape we invented.
    #[test]
    fn a_real_community_manifest_parses_and_passes() {
        let m = manifest(serde_json::json!({
            "schema_version": 2, "model_api": 1, "name": "flamingo-cycle",
            "kind": "perpetual", "obs_len": 61, "action_len": 14, "action_scale": 1.0,
            "entry_pose": "standing", "duration_s": null,
            "description": "Stand on one foot, either side, on command.",
            "command": { "head": "unused (zeros)" },
            "robot": { "model": "microduck", "hw_rev": 1, "servos": "xl330" },
            "training": { "task_id": "Mjlab-FlamingoCycleHard-Flat-MicroDuck" }
        }));
        assert_eq!(m.name.as_deref(), Some("flamingo-cycle"));
        assert_eq!(m.kind.as_deref(), Some("perpetual"));
        assert_eq!(m.incompatibility(here()), None);
    }

    /// **The unified convention, schema 2.** The official set's fields and the community's are one
    /// vocabulary now (`docs/policy-manifest.md`); a single-policy repo may carry any of them, and
    /// this reads the ones a client acts on — the encoding above all, since it is what says whether
    /// the policy can be a one-shot.
    #[test]
    fn a_schema_2_manifest_carries_the_encoding_and_chain() {
        let m = manifest(serde_json::json!({
            "schema_version": 2, "model_api": 1, "name": "ground_pick", "kind": "episodic",
            "duration_s": 2.8, "chain": false, "mode": "walk",
            "command": { "encoding": "phase", "slots": "twist.vx,twist.vy",
                         "period_s": 4.0, "end_phase": 0.7 },
            "obs_len": 61, "action_len": 14, "robot": { "model": "microduck" }
        }));
        assert_eq!(m.schema_version, Some(2));
        let command = m.command.as_ref().unwrap();
        assert_eq!(command.encoding.as_deref(), Some("phase"));
        assert_eq!(command.period_s, Some(4.0));
        assert_eq!(command.end_phase, Some(0.7));
        assert_eq!(
            m.incompatibility(here()),
            None,
            "the encoding is not a shape problem"
        );

        let roll = manifest(serde_json::json!({
            "schema_version": 2, "kind": "episodic", "duration_s": 1.0, "chain": true
        }));
        assert!(roll.chain);
        assert!(
            !PolicyManifest::default().chain,
            "absent means a single run"
        );
    }

    /// The whole point of reading the manifest: a 51-D policy is refused before 800 KB is
    /// downloaded and before the robot is asked to run it. `robotd` would refuse it at load
    /// anyway — this is the same answer, arriving where somebody can act on it.
    #[test]
    fn a_policy_the_manifest_says_is_the_wrong_shape_is_refused() {
        let m = manifest(serde_json::json!({ "obs_len": 51, "action_len": 14 }));
        let why = m.incompatibility(here()).expect("refused");
        assert!(why.contains("51") && why.contains("61"), "{why}");
    }

    /// The model-API rule from `updater-design.md` §5.5, finally doing something: a policy needing
    /// a newer daemon is refused with the remedy in it, and an older one still loads.
    #[test]
    fn a_policy_needing_a_newer_daemon_says_so() {
        let newer = manifest(serde_json::json!({ "model_api": 2 }));
        let why = newer.incompatibility(here()).expect("refused");
        assert!(why.contains("update the daemon"), "{why}");

        let older = manifest(serde_json::json!({ "model_api": 1 }));
        assert_eq!(older.incompatibility(Expectations::here(Some(2))), None);
    }

    /// A policy published for a different robot is not this robot's to run.
    #[test]
    fn a_policy_for_another_robot_is_refused() {
        let m = manifest(serde_json::json!({ "robot": { "model": "reachy" } }));
        assert!(m.incompatibility(here()).unwrap().contains("reachy"));
    }

    /// **Absence is not evidence.** A repo with no manifest, or one that omits the fields we act
    /// on, must not be refused — most of the Hub follows no convention of ours, and the shape gate
    /// at load was always going to be the real check.
    #[test]
    fn a_manifest_that_claims_nothing_refuses_nothing() {
        assert_eq!(PolicyManifest::default().incompatibility(here()), None);
        let sparse = manifest(serde_json::json!({ "name": "something", "unknown_field": 3 }));
        assert_eq!(sparse.incompatibility(here()), None);
    }

    /// **The Hub returns tags in no useful order, and this is the order it actually returned.**
    ///
    /// `v3, v1, v2` for `pollen-robotics/microduck-policies`, with no dates on any of them. The
    /// rule used to be "the list, reversed", which made v2 the newest and had a board report
    /// "newest v2 — up to date" while listing v3 underneath it.
    #[test]
    fn the_newest_tag_is_the_highest_version_not_the_first_listed() {
        let mut tags = vec!["v3".to_string(), "v1".to_string(), "v2".to_string()];
        tags.sort_by(|a, b| match (version_of(a), version_of(b)) {
            (Some(a), Some(b)) => b.cmp(&a),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => std::cmp::Ordering::Equal,
        });
        assert_eq!(tags, ["v3", "v2", "v1"]);
    }

    /// Numbers compare as numbers, so a repo that gets past nine does not go backwards.
    #[test]
    fn ten_is_newer_than_nine() {
        assert!(version_of("v10") > version_of("v9"));
        assert!(version_of("v1.2.0") > version_of("v1.1.9"));
        assert_eq!(version_of("v3"), Some([3].to_vec()));
    }

    /// A tag that is not a version is never the newest, so `policy update` with no argument
    /// cannot land on somebody's scratch tag. Naming it explicitly still works.
    #[test]
    fn a_name_that_is_not_a_version_is_never_newest() {
        assert_eq!(version_of("experimental"), None);
        assert_eq!(version_of("v2-rc"), None, "not a plain version either");

        let mut tags = ["experimental", "v1", "v2"].map(str::to_owned);
        tags.sort_by(|a, b| match (version_of(a), version_of(b)) {
            (Some(a), Some(b)) => b.cmp(&a),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => std::cmp::Ordering::Equal,
        });
        assert_eq!(tags.first().unwrap(), "v2");
        assert_eq!(
            tags.iter().find(|t| version_of(t).is_some()).unwrap(),
            "v2",
            "and the selection skips it even if it sorted first"
        );
    }

    /// Origin is the org, and nothing else.
    #[test]
    fn origin_is_decided_by_the_org() {
        assert_eq!(
            origin_of_repo("pollen-robotics/microduck-policies"),
            "official"
        );
        assert_eq!(
            origin_of_repo("RemiFabre/microduck-flamingo-cycle"),
            "community"
        );
        // Not a prefix match: an org that merely starts the same way is somebody else.
        assert_eq!(origin_of_repo("pollen-robotics-fake/x"), "community");
        assert_eq!(origin_of_repo("nonsense"), "community");
    }

    /// One `.onnx` is the answer, and it is the answer for every microduck policy published so
    /// far — they all carry a single `policy.onnx` beside a README and a manifest.
    #[test]
    fn the_sole_policy_in_a_repo_is_the_one_to_take() {
        let files = vec![
            ".gitattributes".to_string(),
            "README.md".to_string(),
            "manifest.json".to_string(),
            "policy.onnx".to_string(),
        ];
        assert_eq!(sole_policy(&files, "org/x").unwrap(), "policy.onnx");
    }

    /// Several is a refusal naming them, not a guess. Picking wrong here means running the wrong
    /// network on a real robot, which is not a coin to toss.
    #[test]
    fn a_repo_with_several_policies_asks_which() {
        let files = vec!["walk.onnx".to_string(), "run.onnx".to_string()];
        let why = sole_policy(&files, "org/x").unwrap_err().to_string();
        assert!(
            why.contains("walk.onnx") && why.contains("run.onnx"),
            "{why}"
        );
        assert!(why.contains("<file>"), "and how to say which: {why}");

        let none: Vec<String> = vec!["README.md".to_string()];
        assert!(
            sole_policy(&none, "org/x")
                .unwrap_err()
                .to_string()
                .contains("no .onnx")
        );
    }

    /// A board with nothing installed has no repo to ask about, and says so rather than inventing
    /// one — the repo is a property of the set, not of this daemon.
    ///
    /// **And it is not a network failure.** Reporting it through `unreachable` made a board whose
    /// only fault was having no set yet print "the Hub could not be reached", which sent the
    /// reader after a problem that did not exist.
    #[tokio::test]
    async fn a_board_with_no_set_is_not_a_hub_that_cannot_be_reached() {
        let tmp = tempfile::tempdir().unwrap();
        let result = check(tmp.path()).await;
        assert!(result.repo.is_none());
        assert!(result.installed.is_none());
        assert!(
            result.unreachable.is_none(),
            "nothing installed is not a network problem: {:?}",
            result.unreachable
        );
    }

    /// A set whose provenance record is missing still counts as installed, and the version comes
    /// from the directory name — which is the shape of a board seeded before the record existed.
    /// The seeder back-fills it, but this must not report "nothing installed" in the meantime.
    #[test]
    fn a_set_without_a_record_is_still_a_set_on_disk() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        std::fs::create_dir_all(root.join("releases/seed-v1")).unwrap();
        std::fs::write(root.join("releases/seed-v1/alpha_walking.onnx"), "w").unwrap();
        swap_current(root, "seed-v1").unwrap();

        assert!(installed(root).is_none(), "no record, so no repo to name");
        assert_eq!(
            installed_files(root),
            vec!["alpha_walking.onnx".to_string()],
            "but the policies are plainly there"
        );
    }
}

// ── the community library ────────────────────────────────────────────────────
//
// One policy, from any Hub repo, into a slot. Separate from the official set above and
// deliberately so: a set is nine files that version together and fill every slot, and this is one
// file somebody wants to try in one of them.
//
// Nothing here is signed, per `docs/design/policy-channel-design.md` §2. A policy is not a
// binary: `robotd` holds the only write handle to the bus behind joint clamps, a fall reflex and
// an intent deadman, and refuses any graph that is not obs[1,61] -> actions[1,14] while the robot
// is standing still. That sandbox is the boundary, not a signature.

/// Where fetched policies live. Outside every release directory, per `updater-design.md` §5.7 —
/// a policy somebody chose must survive an update and a rollback.
pub const LIBRARY_ROOT: &str = "/var/lib/robot/policies";

/// The org whose policies are "official". One constant: a robot that can be *told* which org to
/// trust has a badge that means nothing.
pub const OFFICIAL_ORG: &str = "pollen-robotics";

/// `"official"` or `"community"`, from the repo that published it.
pub fn origin_of_repo(repo: &str) -> &'static str {
    match repo.split_once('/') {
        Some((org, _)) if org == OFFICIAL_ORG => "official",
        _ => "community",
    }
}

/// What a repo's `manifest.json` says about the policy in it.
///
/// **Untrusted.** It is a stranger's description of a stranger's file, and every field is taken as
/// a claim rather than a fact. It is worth reading anyway: a policy that *says* it is 51-D can be
/// refused before 800 KB is downloaded and before the robot is asked to run it, which is a much
/// better error than the same refusal arriving at load. A manifest that lies is caught there, by
/// the check that has always been the real one.
///
/// The shape is the convention the published microduck policies already use, and everything is
/// optional because a repo is under no obligation to carry any of it.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct PolicyManifest {
    /// Which revision of the convention wrote this. 1 is the official set's first shape, 2 the
    /// unified one `docs/policy-manifest.md` describes — a superset, so nothing is gated on it.
    pub schema_version: Option<u32>,
    pub name: Option<String>,
    pub description: Option<String>,
    /// `episodic`, `perpetual` or `scripted`: who supplies the ending.
    pub kind: Option<String>,
    /// Seconds the policy runs, for one that ends itself. The convention's field name.
    pub duration_s: Option<f64>,
    pub action_scale: Option<f64>,
    /// Seconds the policy needs to get back to `command.idle` from wherever it holds.
    ///
    /// Only a perpetual policy has one — an episodic policy is already back by the time its
    /// `duration_s` is up. It is what the daemon drives the idle command for before handing over
    /// to the gait, so that a robot holding a foot in the air is not simply let go of.
    pub unwind_s: Option<f64>,
    /// Whether holding the button chains another run.
    pub chain: bool,
    /// Seconds a scripted policy takes to settle after its flag flips.
    pub ramp_s: Option<f64>,
    /// `walk` or `roller`, for a policy that belongs to one drive mode.
    pub mode: Option<String>,
    pub command: Option<ManifestCommand>,
    pub obs_len: Option<usize>,
    pub action_len: Option<usize>,
    pub model_api: Option<u32>,
    pub robot: Option<ManifestRobot>,
}

/// The command block, as the published convention describes it.
///
/// `twist`, `head` and `body` are prose for a person — "flag: 0 = stand on two feet" — and are
/// not read. `idle` is the one machine-readable part and the one that matters: it is the command
/// that means "stop doing the thing", which is what a skill unwinds to.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct ManifestCommand {
    /// How the daemon is meant to drive the twist: absent or `"constant"` (a skill), `"phase"`
    /// (a ground pick), `"posture_flag"` (a sit↔stand). Read so a client can refuse to make a
    /// one-shot of something the daemon has to drive itself.
    pub encoding: Option<String>,
    pub idle: Option<[f64; 3]>,
    /// Phase encoding: seconds per cycle, and where in it the move hands back.
    pub period_s: Option<f64>,
    pub end_phase: Option<f64>,
}

#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct ManifestRobot {
    pub model: Option<String>,
}

/// What this robot expects of a policy.
///
/// A value rather than constants read in place, so the manifest check can be tested against
/// expectations that are not this board's. The defaults come from `duck_ipc_proto`, which is
/// where the shape contract is published precisely because it is a contract with whoever
/// publishes a policy — `duck_control` asserts at compile time that its own constants agree.
#[derive(Debug, Clone, Copy)]
pub struct Expectations {
    pub obs_len: usize,
    pub action_len: usize,
    pub model_api: u32,
    pub robot_model: &'static str,
}

impl Expectations {
    /// What this daemon believes, with the model API the running `robotd` reports.
    ///
    /// `None` — an unreachable robot — takes the contract's own version rather than refusing
    /// everything: fetching a policy onto a board whose control loop is down is a reasonable
    /// thing to be doing, and the load will check it properly when the loop comes back.
    pub fn here(model_api: Option<u32>) -> Self {
        Self {
            obs_len: crate::proto::POLICY_OBS_LEN,
            action_len: crate::proto::POLICY_ACTION_LEN,
            model_api: model_api.unwrap_or(1),
            robot_model: crate::proto::ROBOT_MODEL,
        }
    }
}

impl PolicyManifest {
    /// Refuse a policy the manifest itself says will not work here.
    ///
    /// Only refuses on a claim that is *present and wrong*. A manifest with no `obs_len` is not
    /// evidence of anything, and refusing on absence would reject every repo that does not follow
    /// a convention nobody has published.
    pub fn incompatibility(&self, expected: Expectations) -> Option<String> {
        if let Some(obs) = self.obs_len
            && obs != expected.obs_len
        {
            return Some(format!(
                "its manifest says observation width {obs}, and this robot builds {}",
                expected.obs_len
            ));
        }
        if let Some(actions) = self.action_len
            && actions != expected.action_len
        {
            return Some(format!(
                "its manifest says {actions} actions, and this robot has {}",
                expected.action_len
            ));
        }
        if let Some(api) = self.model_api
            && api > expected.model_api
        {
            return Some(format!(
                "it needs model API {api} and this daemon speaks {} — update the daemon first",
                expected.model_api
            ));
        }
        if let Some(model) = self.robot.as_ref().and_then(|r| r.model.as_deref())
            && !model.eq_ignore_ascii_case(expected.robot_model)
        {
            return Some(format!(
                "it is for a {model}, and this is a {}",
                expected.robot_model
            ));
        }
        None
    }
}

/// Everything in a repo revision, as the Hub lists it.
async fn tree(client: &reqwest::Client, repo: &str, revision: &str) -> Result<Vec<String>, Error> {
    let url = format!("https://huggingface.co/api/models/{repo}/tree/{revision}");
    let bytes = http::get_bytes(client, &url, None).await?;
    let listing: serde_json::Value = serde_json::from_slice(&bytes)
        .map_err(|e| Error::Network(format!("listing {repo}@{revision}: {e}")))?;
    Ok(listing
        .as_array()
        .map(|files| {
            files
                .iter()
                .filter_map(|f| f.get("path")?.as_str().map(str::to_owned))
                .collect()
        })
        .unwrap_or_default())
}

/// The commit a revision points at, so a moving branch can be noticed later.
async fn commit_of(client: &reqwest::Client, repo: &str, revision: &str) -> Option<String> {
    let url = format!("https://huggingface.co/api/models/{repo}/revision/{revision}");
    let bytes = http::get_bytes(client, &url, None).await.ok()?;
    let info: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    info.get("sha")?.as_str().map(str::to_owned)
}

/// Pick the policy file out of a repo listing.
///
/// Exactly one `.onnx` is the answer, and it is the answer for every microduck policy published
/// so far — they all carry a single `policy.onnx`. Several is a refusal naming them rather than a
/// guess: choosing wrong here means running the wrong network on a real robot.
fn sole_policy(files: &[String], repo: &str) -> Result<String, Error> {
    let mut candidates: Vec<&String> = files.iter().filter(|f| f.ends_with(".onnx")).collect();
    candidates.sort();
    match candidates.len() {
        1 => Ok(candidates[0].clone()),
        0 => Err(Error::Network(format!("{repo} has no .onnx in it"))),
        _ => Err(Error::Network(format!(
            "{repo} has {} policies — name one with `<repo>:<file>`: {}",
            candidates.len(),
            candidates
                .iter()
                .map(|c| c.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ))),
    }
}

/// Fetch one policy into the library and say what it turned out to be.
pub async fn fetch(
    library: &Path,
    repo: &str,
    revision: Option<&str>,
    file: Option<&str>,
    expected: Expectations,
    in_use: Option<&[String]>,
) -> Result<crate::proto::PolicyFetchResult, Error> {
    // A repo is `org/name`, and nothing else. Checked before it is pasted into a URL and before
    // any of it becomes a directory name.
    let Some((org, name)) = repo.split_once('/') else {
        return Err(Error::Network(format!("{repo} is not an org/name repo")));
    };
    for part in [org, name] {
        if part.is_empty() || part.contains(['.', '/', '\\']) {
            return Err(Error::Network(format!("{repo} is not an org/name repo")));
        }
    }
    let revision = revision.unwrap_or("main");
    if revision.contains(['/', '\\']) || revision.starts_with('.') {
        return Err(Error::Network(format!("{revision} is not a revision")));
    }

    let client = http::client()?;

    // The manifest first, so a policy that says it cannot work here costs one small request
    // rather than a download and a refusal from the control loop.
    let manifest_url = format!("https://huggingface.co/{repo}/resolve/{revision}/manifest.json");
    let manifest: PolicyManifest = match http::get_bytes(&client, &manifest_url, None).await {
        Ok(bytes) => serde_json::from_slice(&bytes).unwrap_or_default(),
        // No manifest is not a fault. Plenty of repos will not have one, and the shape gate at
        // load is the check that was always going to decide this.
        Err(_) => PolicyManifest::default(),
    };
    if let Some(why) = manifest.incompatibility(expected) {
        return Err(Error::Network(format!(
            "{repo} will not run on this robot: {why}"
        )));
    }

    let file = match file {
        Some(file) => {
            if file.contains('/') || file.starts_with('.') || !file.ends_with(".onnx") {
                return Err(Error::Network(format!("{file} is not a policy file name")));
            }
            file.to_owned()
        }
        // The listing's path, checked the same way a caller-supplied name is: a `.onnx` under a
        // subdirectory is a repo layout this cannot install, and saying so beats an ENOENT from
        // a staging path whose parent was never created.
        None => {
            let file = sole_policy(&tree(&client, repo, revision).await?, repo)?;
            if file.contains('/') {
                return Err(Error::Network(format!(
                    "{repo}'s only policy is {file}, which is not at the top of the repo — \
                     name it with `<repo>:<file>` if that is really the one"
                )));
            }
            file
        }
    };

    let dir = library.join(org).join(name).join(revision);
    std::fs::create_dir_all(&dir).map_err(|e| Error::Io {
        path: dir.clone(),
        source: e,
    })?;

    // Staged beside the destination, so a download interrupted halfway is never a file a slot
    // could be pointed at.
    let staged = dir.join(format!("{file}.part"));
    let (progress, mut drain) = tokio::sync::mpsc::unbounded_channel();
    tokio::spawn(async move { while drain.recv().await.is_some() {} });
    let url = format!("https://huggingface.co/{repo}/resolve/{revision}/{file}");
    http::download_to(&client, &url, &staged, None, &progress)
        .await
        .inspect_err(|_| {
            let _ = std::fs::remove_file(&staged);
        })?;
    let path = dir.join(&file);
    std::fs::rename(&staged, &path).map_err(|e| Error::Io {
        path: path.clone(),
        source: e,
    })?;

    prune_library(&library.join(org).join(name), revision, in_use);

    let commit = commit_of(&client, repo, revision).await;
    let record = format!(
        "repo={repo}\nversion={revision}\ncommit={}\nfile={file}\nfetched={}\n",
        commit.as_deref().unwrap_or("unknown"),
        now_utc(),
    );
    let _ = std::fs::write(dir.join(SOURCE_FILE), record);

    Ok(crate::proto::PolicyFetchResult {
        path: path.display().to_string(),
        repo: repo.to_owned(),
        revision: revision.to_owned(),
        commit,
        file,
        origin: origin_of_repo(repo).to_owned(),
        name: manifest.name,
        description: manifest.description,
        kind: manifest.kind,
        duration_s: manifest.duration_s,
        action_scale: manifest.action_scale,
        unwind_s: manifest.unwind_s,
        idle: manifest.command.as_ref().and_then(|c| c.idle),
        encoding: manifest.command.as_ref().and_then(|c| c.encoding.clone()),
        chain: manifest.chain,
        schema_version: manifest.schema_version,
    })
}

/// Seconds since the epoch, in the `@<secs>` form `date -d` and `systemd-analyze` both read.
///
/// Not RFC-3339, deliberately: rendering one means a date library or a hand-rolled civil-calendar
/// conversion, for a field nothing parses. The seeder writes a real timestamp because `date` is
/// already there; this is the same fact in the form that costs nothing.
fn now_utc() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("@{secs}")
}

/// How many revisions of one repo the library keeps.
///
/// **The library is the one directory here that nothing tidied.** Every `policy load <org/repo>`
/// leaves ~800 KB behind for good, and both the command and `policy.fetch` behind it are served
/// over BLE and WebRTC — where §4 of `remote-webrtc.md` says any LAN peer inherits them. A robot
/// whose eMMC fills up stops being a robot, and nothing about trying gaits should be able to do
/// that.
///
/// Two, matching the official sets: the one just fetched and the one before it, so going back to
/// the revision you just left is still a local operation.
const LIBRARY_REVISIONS_KEPT: usize = 2;

/// Drop older revisions of the repo just fetched, keeping what the robot is using.
///
/// **`in_use` is the whole safety of this**, and `None` means "could not ask" rather than
/// "nothing" — a silent `robotd` is exactly the robot whose gait must not be deleted while it is
/// down. The caller passes what `robot.policies` and `robot.skills` answered, so a policy filling
/// a slot or answering to a name is kept however old it is.
///
/// Per repo rather than across the library: a person trying two gaits alternately is not asking
/// to lose the third one they fetched last week from somewhere else.
///
/// Best effort, like [`prune`]. A revision that will not delete is disk space, not a failed
/// fetch, and undoing a good fetch over it would be the wrong trade.
fn prune_library(repo_dir: &Path, keep: &str, in_use: Option<&[String]>) {
    let Some(in_use) = in_use else {
        tracing::debug!(
            dir = %repo_dir.display(),
            "not pruning the policy library: the robot did not say what it is using"
        );
        return;
    };
    let Ok(entries) = std::fs::read_dir(repo_dir) else {
        return;
    };
    // Newest first, by the mtime the fetch left on the directory.
    let mut revisions: Vec<(std::time::SystemTime, std::path::PathBuf, String)> = entries
        .flatten()
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            let modified = e.metadata().ok()?.modified().ok()?;
            Some((modified, e.path(), name))
        })
        .collect();
    revisions.sort_by_key(|(modified, _, _)| std::cmp::Reverse(*modified));

    let mut kept = 0;
    for (_, path, name) in revisions {
        let used = in_use
            .iter()
            .any(|p| std::path::Path::new(p).starts_with(&path));
        if name == keep || used {
            continue;
        }
        kept += 1;
        if kept < LIBRARY_REVISIONS_KEPT {
            continue;
        }
        if let Err(e) = std::fs::remove_dir_all(&path) {
            tracing::warn!(revision = %name, error = %e, "could not remove an old policy");
        }
    }
}

/// Hub models matching a query.
///
/// No tag filter yet: `microduck` in the name is what the published policies have in common, and
/// a tag is something to add once there is something to tag. Every field is the publisher's.
pub async fn search(query: &str) -> Result<crate::proto::PolicySearchResult, Error> {
    let client = http::client()?;
    let encoded: String = query
        .chars()
        .filter(|c| c.is_alphanumeric() || matches!(c, '-' | '_' | ' '))
        .map(|c| if c == ' ' { '+' } else { c })
        .collect();
    let url = format!("https://huggingface.co/api/models?search={encoded}&limit=25");
    let bytes = http::get_bytes(&client, &url, None).await?;
    let hits: serde_json::Value = serde_json::from_slice(&bytes)
        .map_err(|e| Error::Network(format!("searching for {query}: {e}")))?;

    let models = hits
        .as_array()
        .map(|models| {
            models
                .iter()
                .filter_map(|m| {
                    let id = m.get("modelId")?.as_str()?.to_owned();
                    let origin = origin_of_repo(&id).to_owned();
                    Some(crate::proto::PolicySearchHit {
                        id,
                        origin,
                        likes: m.get("likes").and_then(|v| v.as_u64()),
                        downloads: m.get("downloads").and_then(|v| v.as_u64()),
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    Ok(crate::proto::PolicySearchResult { models })
}
