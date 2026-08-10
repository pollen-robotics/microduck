//! Signature and hash verification.
//!
//! The trust anchor is a *set* of minisign public keys on disk, not one baked-in
//! key, so a lost or compromised key is survivable
//! (`docs/design/updater-design.md` §5.4).
//!
//! Invariant: **no unsigned bytes are ever extracted to a live path or executed.**
//! Verification order is manifest signature → artifact hash → artifact
//! signature; extraction happens only after all three pass.
//!
//! We depend on `minisign-verify` (zero dependencies, **verify-only**) rather than
//! the full `minisign` crate: this process has no business being able to sign, so
//! it shouldn't link the code that can.

use std::fs::File;
use std::io::Read;
use std::path::Path;

use minisign_verify::{PublicKey, Signature};
use sha2::{Digest, Sha256};

use crate::Error;

/// Read buffer for streaming hash/verify. Kind to a small board while still
/// amortising syscalls.
const CHUNK: usize = 64 * 1024;

/// Bounds on what an archive may expand to.
///
/// Not a security boundary — the signature already established provenance — but a
/// guard against an accidentally enormous artifact filling the eMMC. Configurable
/// because a model bundle of several ONNX policies is legitimately much larger than
/// a daemon binary (`docs/design/updater-design.md` §5.5), and a too-low ceiling would
/// reject a genuine release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArchiveLimits {
    pub max_uncompressed_bytes: u64,
    pub max_entries: usize,
}

impl Default for ArchiveLimits {
    fn default() -> Self {
        Self {
            max_uncompressed_bytes: 2 * 1024 * 1024 * 1024,
            max_entries: 50_000,
        }
    }
}

/// Keys whose filename ends with this are usable only when `allow_dev_keys` is
/// set, so a production robot won't install a team member's local build.
const DEV_KEY_SUFFIX: &str = ".dev.pub";

pub struct TrustedKey {
    /// Filename, so the update log can record which key admitted an artifact.
    pub id: String,
    pub dev_only: bool,
    key: PublicKey,
}

/// Hand-written so the key material is never printed by accident — only its id.
impl std::fmt::Debug for TrustedKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TrustedKey")
            .field("id", &self.id)
            .field("dev_only", &self.dev_only)
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
pub struct KeyRing {
    keys: Vec<TrustedKey>,
    allow_dev_keys: bool,
}

impl KeyRing {
    /// Load every `*.pub` in `dir`.
    ///
    /// An empty keyring is an **error**, not an empty allow-list: silently
    /// trusting nothing looks identical to a misconfigured path, and guessing
    /// wrong here is catastrophic in either direction.
    pub fn load(dir: &Path, allow_dev_keys: bool) -> Result<Self, Error> {
        let entries = std::fs::read_dir(dir).map_err(|e| Error::Io {
            path: dir.to_path_buf(),
            source: e,
        })?;

        let mut keys = Vec::new();
        for entry in entries {
            let entry = entry.map_err(|e| Error::Io {
                path: dir.to_path_buf(),
                source: e,
            })?;
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) != Some("pub") {
                continue;
            }
            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default()
                .to_owned();

            let text = std::fs::read_to_string(&path).map_err(|e| Error::Io {
                path: path.clone(),
                source: e,
            })?;

            // `decode` handles minisign's two-line format (comment + key); fall
            // back to a bare base64 key for convenience.
            let key = PublicKey::decode(&text)
                .or_else(|_| PublicKey::from_base64(text.trim()))
                .map_err(|e| {
                    Error::Config(format!(
                        "{}: not a minisign public key: {e}",
                        path.display()
                    ))
                })?;

            keys.push(TrustedKey {
                dev_only: name.ends_with(DEV_KEY_SUFFIX),
                id: name,
                key,
            });
        }

        if keys.is_empty() {
            return Err(Error::Config(format!(
                "no trusted keys in {} — refusing to run with an empty trust anchor",
                dir.display()
            )));
        }

        Ok(Self {
            keys,
            allow_dev_keys,
        })
    }

    /// Construct directly. For tests, and for callers that already hold keys.
    pub fn from_keys(keys: Vec<TrustedKey>, allow_dev_keys: bool) -> Self {
        Self {
            keys,
            allow_dev_keys,
        }
    }

    /// Build a trusted key from a base64 minisign public key.
    pub fn key_from_base64(id: &str, b64: &str, dev_only: bool) -> Result<TrustedKey, Error> {
        let key = PublicKey::from_base64(b64)
            .map_err(|e| Error::Config(format!("{id}: invalid public key: {e}")))?;
        Ok(TrustedKey {
            id: id.to_owned(),
            dev_only,
            key,
        })
    }

    fn usable(&self) -> impl Iterator<Item = &TrustedKey> {
        self.keys
            .iter()
            .filter(move |k| self.allow_dev_keys || !k.dev_only)
    }

    fn parse_signature(sig: &[u8]) -> Result<Signature, Error> {
        let text = std::str::from_utf8(sig)
            .map_err(|_| Error::Verification("signature is not valid UTF-8".into()))?;
        Signature::decode(text)
            .map_err(|e| Error::Verification(format!("malformed signature: {e}")))
    }

    /// Verify a detached signature over in-memory bytes (manifests).
    ///
    /// Returns the key that matched, so the update log can record which key
    /// admitted the artifact — useful for spotting a robot still relying on a key
    /// we meant to retire.
    pub fn verify_bytes(&self, data: &[u8], signature: &[u8]) -> Result<&TrustedKey, Error> {
        let sig = Self::parse_signature(signature)?;
        let mut tried = 0;
        for key in self.usable() {
            tried += 1;
            if key.key.verify(data, &sig, false).is_ok() {
                return Ok(key);
            }
        }
        Err(Self::no_key_error(tried))
    }

    /// Verify a detached signature over a file, streaming it.
    ///
    /// Streaming rather than reading it whole: this runs on a board with limited
    /// RAM and artifacts are not small.
    pub fn verify_file(&self, path: &Path, signature: &[u8]) -> Result<&TrustedKey, Error> {
        let sig = Self::parse_signature(signature)?;

        let mut tried = 0;
        for key in self.usable() {
            tried += 1;
            let Ok(mut verifier) = key.key.verify_stream(&sig) else {
                // Wrong key id, or a legacy non-prehashed signature. Try the next.
                continue;
            };

            let mut file = File::open(path).map_err(|e| Error::Io {
                path: path.to_path_buf(),
                source: e,
            })?;
            let mut buf = vec![0u8; CHUNK];
            loop {
                let n = file.read(&mut buf).map_err(|e| Error::Io {
                    path: path.to_path_buf(),
                    source: e,
                })?;
                if n == 0 {
                    break;
                }
                verifier.update(&buf[..n]);
            }

            if verifier.finalize().is_ok() {
                return Ok(key);
            }
        }
        Err(Self::no_key_error(tried))
    }

    fn no_key_error(tried: usize) -> Error {
        Error::Verification(format!(
            "signature did not verify against any of {tried} usable trusted key(s)"
        ))
    }
}

/// Check a file's SHA-256 against the manifest's hex digest.
///
/// Integrity only — authenticity comes from the signature — so a plain comparison
/// is fine; case-insensitive because hex casing varies between tools.
pub fn verify_sha256(path: &Path, expected_hex: &str) -> Result<(), Error> {
    let actual = sha256_hex(path)?;
    if actual.eq_ignore_ascii_case(expected_hex.trim()) {
        Ok(())
    } else {
        Err(Error::Verification(format!(
            "sha256 mismatch: manifest says {expected_hex}, artifact is {actual}"
        )))
    }
}

pub fn sha256_hex(path: &Path) -> Result<String, Error> {
    let mut file = File::open(path).map_err(|e| Error::Io {
        path: path.to_path_buf(),
        source: e,
    })?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; CHUNK];
    loop {
        let n = file.read(&mut buf).map_err(|e| Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex(&hasher.finalize()))
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write;
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// Extract a `.tar.zst` artifact into `dest`.
///
/// **Only call this after signature and hash verification have both passed.**
///
/// Path-traversal safety comes from `tar`'s own `unpack_in`, which refuses
/// absolute paths and entries that would escape the destination, rather than from
/// a hand-rolled check. On top of that we cap total uncompressed size and entry
/// count, which the library does not do — a zip bomb would otherwise fill the
/// eMMC.
pub fn extract_artifact(archive: &Path, dest: &Path, limits: ArchiveLimits) -> Result<(), Error> {
    let file = File::open(archive).map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;
    let decoder = zstd::Decoder::new(file).map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;

    std::fs::create_dir_all(dest).map_err(|e| Error::Io {
        path: dest.to_path_buf(),
        source: e,
    })?;

    let mut tar = tar::Archive::new(decoder);
    tar.set_preserve_permissions(true); // hooks and binaries need the exec bit

    let entries = tar.entries().map_err(|e| Error::Io {
        path: archive.to_path_buf(),
        source: e,
    })?;

    let mut total: u64 = 0;
    let mut count = 0usize;

    for entry in entries {
        let mut entry = entry.map_err(|e| Error::Io {
            path: archive.to_path_buf(),
            source: e,
        })?;

        count += 1;
        if count > limits.max_entries {
            // Deliberately not a verification error: the signature checked out, so
            // this is an oversized artifact or a too-tight limit, not tampering.
            return Err(Error::ArchiveTooLarge(format!(
                "archive has more than {} entries",
                limits.max_entries
            )));
        }

        total = total.saturating_add(entry.size());
        if total > limits.max_uncompressed_bytes {
            return Err(Error::ArchiveTooLarge(format!(
                "archive expands beyond {} bytes (raise max_uncompressed_bytes if this \
                 release is legitimately this big)",
                limits.max_uncompressed_bytes
            )));
        }

        let name = entry
            .path()
            .map(|p| p.display().to_string())
            .unwrap_or_default();

        // `unpack_in` returns Ok(false) when the entry path is unsafe (absolute,
        // or escaping `dest`). Treat that as tampering, not something to skip:
        // we have already verified the signature, so a hostile entry means one of
        // our own keys signed it, which we must surface loudly.
        let unpacked = entry.unpack_in(dest).map_err(|e| Error::Io {
            path: dest.to_path_buf(),
            source: e,
        })?;
        if !unpacked {
            return Err(Error::Verification(format!(
                "archive entry {name:?} would escape the destination; refusing"
            )));
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Sign with the full `minisign` crate (dev-dependency only) so tests exercise
    /// real signatures rather than fixtures we might get subtly wrong.
    fn keypair() -> (minisign::KeyPair, String) {
        let kp = minisign::KeyPair::generate_unencrypted_keypair().unwrap();
        let boxed = kp.pk.to_box().unwrap().to_string();
        // Two-line format: comment, then the key. Take the key line.
        let key_line = boxed.lines().next_back().unwrap().to_owned();
        (kp, key_line)
    }

    fn sign(kp: &minisign::KeyPair, data: &[u8]) -> Vec<u8> {
        minisign::sign(None, &kp.sk, data, None, None)
            .unwrap()
            .to_string()
            .into_bytes()
    }

    fn ring(pk_b64: &str, dev_only: bool, allow_dev: bool) -> KeyRing {
        let key = KeyRing::key_from_base64("test", pk_b64, dev_only).unwrap();
        KeyRing::from_keys(vec![key], allow_dev)
    }

    #[test]
    fn valid_signature_verifies() {
        let (kp, pk) = keypair();
        let data = b"manifest bytes";
        let sig = sign(&kp, data);
        assert!(ring(&pk, false, false).verify_bytes(data, &sig).is_ok());
    }

    #[test]
    fn tampered_data_is_rejected() {
        let (kp, pk) = keypair();
        let sig = sign(&kp, b"original");
        let err = ring(&pk, false, false)
            .verify_bytes(b"tampered", &sig)
            .unwrap_err();
        assert!(matches!(err, Error::Verification(_)));
    }

    #[test]
    fn wrong_key_is_rejected() {
        let (kp, _) = keypair();
        let (_, other_pk) = keypair();
        let data = b"payload";
        let sig = sign(&kp, data);
        assert!(
            ring(&other_pk, false, false)
                .verify_bytes(data, &sig)
                .is_err()
        );
    }

    /// A dev-signed artifact must be refused on a production robot, and accepted
    /// only when dev keys are explicitly allowed.
    #[test]
    fn dev_key_is_gated() {
        let (kp, pk) = keypair();
        let data = b"local build";
        let sig = sign(&kp, data);

        assert!(
            ring(&pk, true, false).verify_bytes(data, &sig).is_err(),
            "dev key must not be usable in production"
        );
        assert!(
            ring(&pk, true, true).verify_bytes(data, &sig).is_ok(),
            "dev key must work when explicitly allowed"
        );
    }

    #[test]
    fn file_signature_verifies_and_detects_tampering() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("artifact.bin");
        std::fs::write(&path, b"a".repeat(200_000)).unwrap();

        let (kp, pk) = keypair();
        let sig = sign(&kp, &std::fs::read(&path).unwrap());

        assert!(ring(&pk, false, false).verify_file(&path, &sig).is_ok());

        std::fs::write(&path, b"b".repeat(200_000)).unwrap();
        assert!(ring(&pk, false, false).verify_file(&path, &sig).is_err());
    }

    #[test]
    fn empty_keyring_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let err = KeyRing::load(dir.path(), false).unwrap_err();
        assert!(matches!(err, Error::Config(_)));
    }

    #[test]
    fn keyring_loads_from_disk_and_flags_dev_keys() {
        let (_, pk) = keypair();
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("prod.pub"), &pk).unwrap();
        std::fs::write(dir.path().join("team.dev.pub"), &pk).unwrap();

        let ring = KeyRing::load(dir.path(), false).unwrap();
        assert_eq!(ring.keys.len(), 2);
        assert_eq!(ring.usable().count(), 1, "dev key must be excluded");

        let ring = KeyRing::load(dir.path(), true).unwrap();
        assert_eq!(ring.usable().count(), 2);
    }

    #[test]
    fn sha256_detects_change() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("f");
        std::fs::write(&path, b"hello").unwrap();
        // Known digest of "hello".
        let expected = "2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824";
        assert!(verify_sha256(&path, expected).is_ok());
        assert!(
            verify_sha256(&path, &expected.to_uppercase()).is_ok(),
            "hex casing must not matter"
        );

        std::fs::write(&path, b"hello!").unwrap();
        assert!(verify_sha256(&path, expected).is_err());
    }

    /// Build a `.tar.zst` with the given entries.
    ///
    /// The `zstd` encoder is `auto_finish`, so the frame is only completed when
    /// the builder (and thus the encoder) is dropped — hence the explicit `drop`
    /// before returning. Forgetting it yields a truncated archive.
    fn make_archive(dest: &Path, files: &[(&str, &[u8], u32)]) {
        let out = File::create(dest).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);
        for (name, body, mode) in files {
            let mut header = tar::Header::new_gnu();
            header.set_size(body.len() as u64);
            header.set_mode(*mode);
            header.set_cksum();
            builder.append_data(&mut header, name, *body).unwrap();
        }
        builder.finish().unwrap();
        drop(builder);
    }

    /// Build an archive containing a deliberately hostile entry path.
    ///
    /// `Builder::append_data` validates paths and refuses these, so the name is
    /// written straight into the header bytes to bypass it. That is the whole
    /// point: we need to prove *our extractor* rejects what a malicious producer
    /// could emit, and a well-behaved builder can't produce the input.
    fn make_archive_with_raw_name(dest: &Path, raw_name: &str, body: &[u8]) {
        let out = File::create(dest).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);

        let mut header = tar::Header::new_gnu();
        header.set_size(body.len() as u64);
        header.set_mode(0o644);
        header.set_entry_type(tar::EntryType::Regular);
        {
            let gnu = header.as_gnu_mut().unwrap();
            let bytes = raw_name.as_bytes();
            gnu.name[..bytes.len()].copy_from_slice(bytes);
        }
        header.set_cksum();

        builder.append(&header, body).unwrap();
        builder.finish().unwrap();
        drop(builder);
    }

    #[test]
    fn extracts_normal_archive() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("a.tar.zst");
        make_archive(
            &archive,
            &[
                ("bin/robotd", b"elf", 0o755),
                ("version.toml", b"v=1", 0o644),
            ],
        );

        let dest = dir.path().join("out");
        extract_artifact(&archive, &dest, ArchiveLimits::default()).unwrap();

        assert_eq!(std::fs::read(dest.join("bin/robotd")).unwrap(), b"elf");
        assert_eq!(std::fs::read(dest.join("version.toml")).unwrap(), b"v=1");
    }

    /// A traversal entry must be refused outright, not silently skipped.
    #[test]
    fn refuses_path_traversal() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("evil.tar.zst");
        make_archive_with_raw_name(&archive, "../escaped", b"pwned");

        let dest = dir.path().join("out");
        let err = extract_artifact(&archive, &dest, ArchiveLimits::default()).unwrap_err();
        assert!(matches!(err, Error::Verification(_)), "got {err:?}");
        assert!(
            !dir.path().join("escaped").exists(),
            "must not write outside dest"
        );
    }

    /// Absolute paths must not land at the filesystem root.
    #[test]
    fn refuses_absolute_path() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("abs.tar.zst");
        make_archive_with_raw_name(&archive, "/tmp/updater-should-not-exist", b"nope");

        let dest = dir.path().join("out");
        // Either refused, or stripped to a relative path inside dest — never
        // written to the absolute location.
        let _ = extract_artifact(&archive, &dest, ArchiveLimits::default());
        assert!(!Path::new("/tmp/updater-should-not-exist").exists());
    }

    #[test]
    fn refuses_too_many_entries() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("many.tar.zst");

        let out = File::create(&archive).unwrap();
        let enc = zstd::Encoder::new(out, 1).unwrap().auto_finish();
        let mut builder = tar::Builder::new(enc);
        for i in 0..32 {
            let mut header = tar::Header::new_gnu();
            header.set_size(1);
            header.set_mode(0o644);
            header.set_cksum();
            builder
                .append_data(&mut header, format!("f{i}"), &b"x"[..])
                .unwrap();
        }
        builder.finish().unwrap();
        drop(builder); // completes the zstd frame

        let dest = dir.path().join("out");
        // A tight limit for the test; the real ceiling is configurable.
        let limits = ArchiveLimits {
            max_entries: 16,
            ..ArchiveLimits::default()
        };
        let err = extract_artifact(&archive, &dest, limits).unwrap_err();
        assert!(matches!(err, Error::ArchiveTooLarge(_)), "got {err:?}");
    }

    /// The exec bit must survive extraction or post-install hooks won't run.
    #[test]
    fn preserves_executable_bit() {
        use std::os::unix::fs::PermissionsExt;

        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("x.tar.zst");
        make_archive(&archive, &[("hooks/postinstall", b"#!/bin/sh\n", 0o755)]);

        let dest = dir.path().join("out");
        extract_artifact(&archive, &dest, ArchiveLimits::default()).unwrap();

        let mode = std::fs::metadata(dest.join("hooks/postinstall"))
            .unwrap()
            .permissions()
            .mode();
        assert_ne!(mode & 0o111, 0, "exec bit lost, mode = {mode:o}");
    }
}
