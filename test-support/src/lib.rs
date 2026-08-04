//! Publishing signed releases, for tests.
//!
//! Four test files each grew their own copy of this: generate a keypair, build a
//! `.tar.zst`, sign it, write a manifest, sign that. Changing the manifest format meant
//! editing all four, and they had already drifted on details like whether tar mtimes were
//! fixed.
//!
//! Uses the same `minisign`, `tar`, `zstd` and `sha2` crates the engine verifies with, so a
//! fixture cannot produce an artifact the real code rejects for a reason the fixture invented.

use std::path::{Path, PathBuf};

/// A directory of signed releases, plus the key that signed them.
///
/// Backs the `local_dir` source, which is what lets a test drive the real engine end to end
/// with no network.
pub struct Publisher {
    /// The trusted-keys directory the engine should be pointed at.
    pub keys_dir: PathBuf,
    /// Where published releases land — the `local_dir` source path.
    pub releases: PathBuf,
    keypair: minisign::KeyPair,
}

impl Publisher {
    /// Generate a keypair and write its public half into `keys_dir` as `prod.pub`.
    ///
    /// Unencrypted, because a test that had to type a passphrase would be a test nobody runs.
    pub fn new(keys_dir: PathBuf, releases: PathBuf) -> Self {
        std::fs::create_dir_all(&keys_dir).unwrap();
        std::fs::create_dir_all(&releases).unwrap();

        let keypair = minisign::KeyPair::generate_unencrypted_keypair().unwrap();
        let publisher = Self {
            keys_dir,
            releases,
            keypair,
        };
        std::fs::write(publisher.key_file(), publisher.public_key()).unwrap();
        publisher
    }

    /// The file the public key was written to — for a test that removes or replaces the
    /// trust anchor, which should not have to know the filename.
    pub fn key_file(&self) -> PathBuf {
        self.keys_dir.join("prod.pub")
    }

    /// The public key line, for tests that swap or compare the trust anchor.
    pub fn public_key(&self) -> String {
        self.keypair
            .pk
            .to_box()
            .unwrap()
            .to_string()
            .lines()
            .next_back()
            .unwrap()
            .to_owned()
    }

    /// A detached minisign signature over `data`.
    pub fn sign(&self, data: &[u8]) -> Vec<u8> {
        minisign::sign(None, &self.keypair.sk, data, None, None)
            .unwrap()
            .to_string()
            .into_bytes()
    }

    /// Publish a signed release on the `daemon` channel.
    pub fn publish(&self, version: &str) {
        self.release(version).write();
    }

    /// Start describing a release. Call [`Release::write`] to publish it.
    ///
    /// A builder because the call sites want different single deviations — a hook, a
    /// different channel, an extra manifest field — and a function taking all of them would
    /// be five arguments of which four are `None` at every call.
    pub fn release<'a>(&'a self, version: &str) -> Release<'a> {
        Release {
            publisher: self,
            version: version.to_owned(),
            channel: "daemon".to_owned(),
            files: Vec::new(),
            edit: None,
            dir: None,
        }
    }

    /// Corrupt a published artifact *after* signing: a tampered mirror or truncated transfer.
    pub fn tamper(&self, channel: &str, version: &str) {
        self.tamper_in(&self.releases, channel, version);
    }

    /// [`Publisher::tamper`], for a release published into its own directory with
    /// [`Release::dir`].
    ///
    /// Split out rather than duplicated at the call site: what "tampered" means has to be
    /// the same thing everywhere, or a caller can invent a corruption the engine happens
    /// not to catch and call it a passing test.
    pub fn tamper_in(&self, dir: &Path, channel: &str, version: &str) {
        let path = dir.join(format!("{channel}-{version}.tar.zst"));
        let mut bytes = std::fs::read(&path).unwrap();
        bytes.push(0xff);
        std::fs::write(&path, bytes).unwrap();
    }

    /// Remove a release, so `latest` resolves to an older one — a reverted mirror.
    pub fn unpublish(&self, channel: &str, version: &str) {
        for name in [
            format!("{version}.manifest.json"),
            format!("{version}.manifest.json.minisig"),
            format!("{channel}-{version}.tar.zst"),
            format!("{channel}-{version}.tar.zst.minisig"),
        ] {
            let _ = std::fs::remove_file(self.releases.join(name));
        }
    }

    /// Point a name at an already-published version, the way CI's moving
    /// `daemon-dev-<branch>` tag does.
    ///
    /// Copies the manifest and its signature, so the bytes and the signature still
    /// correspond: a ref is a pointer, never a re-signing.
    pub fn point_ref_at(&self, git_ref: &str, version: &str) {
        for (from, to) in [
            (
                format!("{version}.manifest.json"),
                format!("{git_ref}.manifest.json"),
            ),
            (
                format!("{version}.manifest.json.minisig"),
                format!("{git_ref}.manifest.json.minisig"),
            ),
        ] {
            std::fs::copy(self.releases.join(from), self.releases.join(to)).unwrap();
        }
    }
}

/// Edits a manifest in place before it is signed.
type ManifestEdit<'a> = Box<dyn FnOnce(&mut serde_json::Value) + 'a>;

/// A release being described. Nothing is written until [`Release::write`].
pub struct Release<'a> {
    publisher: &'a Publisher,
    version: String,
    channel: String,
    files: Vec<(String, Vec<u8>, u32)>,
    edit: Option<ManifestEdit<'a>>,
    dir: Option<PathBuf>,
}

impl<'a> Release<'a> {
    /// Publish into a different directory, signed with the same key.
    ///
    /// Model releases need their own remote: one shared directory would let the daemon's
    /// `latest` resolve to a model manifest, since `local_dir` picks the newest version it
    /// can see regardless of channel.
    pub fn dir(mut self, dir: PathBuf) -> Self {
        std::fs::create_dir_all(&dir).unwrap();
        self.dir = Some(dir);
        self
    }

    /// Publish on a channel other than `daemon` — model components, or a
    /// wrong-channel test.
    pub fn channel(mut self, channel: &str) -> Self {
        self.channel = channel.to_owned();
        self
    }

    /// Add a file to the artifact. Mode matters for `hooks/postinstall`, which must be
    /// executable or the engine's hook runner has nothing to run.
    pub fn file(mut self, path: &str, contents: &[u8], mode: u32) -> Self {
        self.files.push((path.to_owned(), contents.to_vec(), mode));
        self
    }

    /// Embed a post-install hook.
    pub fn hook(self, script: &str) -> Self {
        self.file("hooks/postinstall", script.as_bytes(), 0o755)
    }

    /// Mutate the manifest before it is signed — for compatibility floors and
    /// `min_supported`. Applied last, so it can override anything.
    pub fn manifest(mut self, edit: impl FnOnce(&mut serde_json::Value) + 'a) -> Self {
        self.edit = Some(Box::new(edit));
        self
    }

    /// Write the artifact, the manifest, and a detached signature for each.
    ///
    /// The manifest is named `<version>.manifest.json`, which is what the `local_dir` source
    /// resolves.
    pub fn write(self) {
        let out_dir = self
            .dir
            .clone()
            .unwrap_or_else(|| self.publisher.releases.clone());
        let artifact_name = format!("{}-{}.tar.zst", self.channel, self.version);
        let artifact = out_dir.join(&artifact_name);

        let out = std::fs::File::create(&artifact).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);

        // Always present: the engine reads it to report what is installed, and several tests
        // read it through the `current` symlink to prove the *content* swapped rather than
        // just the link.
        let marker = format!("version={}\n", self.version);
        append(&mut builder, "version.toml", marker.as_bytes(), 0o644);
        for (path, contents, mode) in &self.files {
            append(&mut builder, path, contents, *mode);
        }
        builder.finish().unwrap();
        drop(builder); // completes the zstd frame

        let bytes = std::fs::read(&artifact).unwrap();
        std::fs::write(
            out_dir.join(format!("{artifact_name}.minisig")),
            self.publisher.sign(&bytes),
        )
        .unwrap();

        let mut manifest = serde_json::json!({
            "channel": self.channel,
            "version": self.version,
            "url": artifact_name,
            "sha256": sha256_hex(&bytes),
            "sig_url": format!("{artifact_name}.minisig"),
            "size": bytes.len(),
            "schema_version": 1,
        });
        if let Some(edit) = self.edit {
            edit(&mut manifest);
        }

        let manifest_bytes = serde_json::to_vec(&manifest).unwrap();
        std::fs::write(
            out_dir.join(format!("{}.manifest.json", self.version)),
            &manifest_bytes,
        )
        .unwrap();
        std::fs::write(
            out_dir.join(format!("{}.manifest.json.minisig", self.version)),
            self.publisher.sign(&manifest_bytes),
        )
        .unwrap();
    }
}

/// Fixed mtime so the same inputs produce the same archive — the reproducibility property
/// `xtask` relies on, and one the copied fixtures disagreed about.
fn append<W: std::io::Write>(builder: &mut tar::Builder<W>, path: &str, data: &[u8], mode: u32) {
    let mut header = tar::Header::new_gnu();
    header.set_size(data.len() as u64);
    header.set_mode(mode);
    header.set_mtime(0);
    header.set_cksum();
    builder.append_data(&mut header, path, data).unwrap();
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher
        .finalize()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// The version `current` points at, or `None` if nothing is installed.
pub fn live_version(install_dir: &Path) -> Option<String> {
    let target = std::fs::read_link(install_dir.join("current")).ok()?;
    Some(target.file_name()?.to_str()?.to_owned())
}

/// `version.toml` read *through* the symlink — proves the content switched, not just the link.
pub fn live_marker(install_dir: &Path) -> Option<String> {
    std::fs::read_to_string(install_dir.join("current/version.toml")).ok()
}
