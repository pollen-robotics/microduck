//! Build tooling: package, sign and promote robot releases.
//!
//! This is the **publisher** side of the update contract. It never ships to a robot —
//! notably it links the full `minisign` crate (which can sign), while `updaterd` links
//! only `minisign-verify` (which cannot).
//!
//! Written in Rust rather than as a shell script for one reason: it reuses the exact
//! same `minisign`, `tar`, `zstd` and `sha2` crates the updater's tests use. A shell
//! version would depend on separately-installed `minisign`/`tar`/`zstd` binaries whose
//! behaviour could drift from what the robot verifies with — which is the last place a
//! difference should be allowed to hide.
//!
//! ```text
//!   cargo xtask package --version 1.2.3 --channel daemon --bin-dir <dir> --out dist/
//!   cargo xtask sign    --dir dist/ --key secret.key
//!   cargo xtask promote --version 1.2.3 --staging-tag daemon-staging-v1.2.3 \
//!                       --stable-tag daemon-v1.2.3 \
//!                       --repo ORG/REPO --out dist/ --key secret.key
//! ```
//!
//! `promote` is what makes §16.3's `staging → stable` real: it emits a *stable*
//! manifest carrying the **same artifact bytes** already validated in staging —
//! same sha256 — rather than rebuilding. Promotion is therefore a re-signing, and
//! what ships is provably what was tested.
//!
//! The stable manifest points at the artifact on the *stable* release, which
//! `promote.yml` uploads alongside it. It used to point back at the staging release
//! instead, to avoid a second copy of the bytes. That made every stable release
//! depend on a tag named as if it were disposable — and it was duly disposed of:
//! deleting the `daemon-staging-v0.1.x` releases left three stable releases pointing
//! at nothing. The sha256 in the manifest is verified on the robot before install
//! (`updater::verify::verify_sha256`), so a copy that diverged could never install
//! silently, which is what the single-copy rule was protecting against.

use std::path::{Path, PathBuf};

use clap::{Parser, Subcommand};

/// Files inside the artifact that the robot expects.
const VERSION_FILE: &str = "version.toml";
const SIG_SUFFIX: &str = ".minisig";

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum KeyKind {
    /// Long-lived, encrypted at rest, trusted by every robot including customers'.
    Release,
    /// For signing branch builds. Unencrypted so CI needs no passphrase, and present
    /// only in the trusted set of *developer* boards.
    Dev,
}

#[derive(Parser)]
#[command(about = "Package, sign and promote robot releases", version)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Assemble a `.tar.zst` artifact and its unsigned manifest.
    Package {
        /// Release version. Must match the crate version — see `--allow-version-drift`.
        #[arg(long)]
        version: semver::Version,

        /// Channel name; must equal the component name in the robot's config.
        #[arg(long, default_value = "daemon")]
        channel: String,

        /// Directory holding the built binaries to ship.
        #[arg(long)]
        bin_dir: PathBuf,

        /// Where to write the artifact and manifest.
        #[arg(long, default_value = "dist")]
        out: PathBuf,

        /// Base URL the robot will download from. The manifest records
        /// `<base>/<artifact>`; for GitHub Releases this is the release's download URL.
        #[arg(long)]
        base_url: Option<String>,

        /// Git SHA, recorded for provenance (§16.4).
        #[arg(long)]
        revision: Option<String>,

        /// Minimum hardware revision this release supports.
        #[arg(long, default_value_t = 0)]
        min_hw_rev: u32,

        /// Force robots below this version to upgrade without waiting for a client
        /// (§8.1). Set this only when remediating a bad release.
        #[arg(long)]
        min_supported: Option<semver::Version>,

        /// Extra files to include, as `src=dest` (e.g. a post-install hook).
        #[arg(long = "include")]
        includes: Vec<String>,

        /// Skip the crate-version match. Only for testing the tool itself.
        #[arg(long)]
        allow_version_drift: bool,

        /// zstd compression level. The default is what a release should ship; lower it
        /// only when the artifact is thrown away, as CI's smoke test does — see the
        /// encoder below for why that one constant dominates the run.
        #[arg(long, default_value_t = 19)]
        zstd_level: i32,
    },

    /// Sign the artifact and manifest in `--dir` with a minisign secret key.
    Sign {
        #[arg(long, default_value = "dist")]
        dir: PathBuf,

        /// Secret key file. In CI, write the secret to a file first — passing a key on
        /// a command line would put it in the process list.
        #[arg(long)]
        key: PathBuf,

        /// Passphrase for an encrypted key. Prefer `MINISIGN_PASSWORD` in the
        /// environment; a passphrase in argv is visible to every process on the box.
        #[arg(long, env = "MINISIGN_PASSWORD", hide_env_values = true)]
        password: Option<String>,
    },

    /// Generate a signing keypair.
    ///
    /// Two kinds, because they have different threat models and different lifetimes —
    /// see the `keygen` function for why the release *spare* must be generated now.
    Keygen {
        /// `release` (encrypted, long-lived, trusted by every robot) or `dev`
        /// (unencrypted so CI can use it non-interactively, never on a customer robot).
        #[arg(long)]
        kind: KeyKind,

        /// Base name. Produces `<name>.pub` and `<name>.key`. A `dev` key is written as
        /// `<name>.dev.pub` so the updater's dev-key gating recognises it.
        #[arg(long)]
        name: String,

        /// Where to write them. Must be OUTSIDE the repository — see below.
        #[arg(long)]
        out: PathBuf,

        /// Passphrase for a release key. Prefer the environment over argv.
        #[arg(long, env = "MINISIGN_PASSWORD", hide_env_values = true)]
        password: Option<String>,
    },

    /// Check that a keypair is usable, and that the public half matches the secret.
    ///
    /// Worth doing *before* relying on a key. A key that turns out to be unusable — bad
    /// passphrase, mismatched pair, truncated file — is discovered either now, or at the
    /// moment you need to sign a fix for a fleet of robots.
    Keycheck {
        /// Secret key to test.
        #[arg(long)]
        key: PathBuf,

        /// Its public half. Defaults to the same path with `.key` → `.pub`.
        #[arg(long)]
        public: Option<PathBuf>,

        #[arg(long, env = "MINISIGN_PASSWORD", hide_env_values = true)]
        password: Option<String>,
    },

    /// Emit a *stable* manifest pointing at an already-published staging artifact.
    ///
    /// No rebuild: the artifact URL and sha256 are carried over unchanged, so what
    /// ships is byte-identical to what was validated.
    Promote {
        #[arg(long)]
        version: semver::Version,

        /// Tag of the staging release holding the validated artifact.
        #[arg(long)]
        staging_tag: String,

        /// Tag of the stable release being created. The manifest's `url` points here,
        /// so the release that `promote.yml` creates must carry the artifact itself.
        #[arg(long)]
        stable_tag: String,

        /// `ORG/REPO`, used to build the download URL.
        #[arg(long)]
        repo: String,

        /// The staging manifest to carry forward.
        #[arg(long)]
        staging_manifest: PathBuf,

        #[arg(long, default_value = "dist")]
        out: PathBuf,

        /// Set or clear the mandatory-update floor for the stable channel.
        #[arg(long)]
        min_supported: Option<semver::Version>,
    },
}

fn main() -> std::process::ExitCode {
    match run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    match Cli::parse().command {
        Command::Package {
            version,
            channel,
            bin_dir,
            out,
            base_url,
            revision,
            min_hw_rev,
            min_supported,
            includes,
            allow_version_drift,
            zstd_level,
        } => package(PackageArgs {
            version,
            channel,
            bin_dir,
            out,
            base_url,
            revision,
            min_hw_rev,
            min_supported,
            includes,
            allow_version_drift,
            zstd_level,
        }),
        Command::Keygen {
            kind,
            name,
            out,
            password,
        } => keygen(kind, &name, &out, password.as_deref()),
        Command::Keycheck {
            key,
            public,
            password,
        } => keycheck(&key, public.as_deref(), password.as_deref()),
        Command::Sign { dir, key, password } => sign_dir(&dir, &key, password.as_deref()),
        Command::Promote {
            version,
            staging_tag,
            stable_tag,
            repo,
            staging_manifest,
            out,
            min_supported,
        } => promote(
            &version,
            &staging_tag,
            &stable_tag,
            &repo,
            &staging_manifest,
            &out,
            min_supported.as_ref(),
        ),
    }
}

struct PackageArgs {
    version: semver::Version,
    channel: String,
    bin_dir: PathBuf,
    out: PathBuf,
    base_url: Option<String>,
    revision: Option<String>,
    min_hw_rev: u32,
    min_supported: Option<semver::Version>,
    includes: Vec<String>,
    allow_version_drift: bool,
    zstd_level: i32,
}

fn package(args: PackageArgs) -> Result<(), Box<dyn std::error::Error>> {
    // Catch the classic mistake: tagging a release without bumping Cargo.toml, so the
    // robot reports a version that doesn't match what it's running.
    if !args.allow_version_drift {
        let crate_version = workspace_version()?;
        // A dev build is the crate version plus a prerelease tag — `0.2.0-dev.17.abc1234`
        // against a crate at `0.2.0` — so its release triple must match while its prerelease
        // component is free. Accepted without `--allow-version-drift` because every branch
        // build would otherwise need the escape hatch, and a flag documented as "only for
        // testing the tool itself" would become part of the normal path, where it would stop
        // catching the mistake it exists for: tagging a release without bumping Cargo.toml.
        let same_release = (args.version.major, args.version.minor, args.version.patch)
            == (
                crate_version.major,
                crate_version.minor,
                crate_version.patch,
            );
        let is_prerelease_of_it = same_release && !args.version.pre.is_empty();

        if crate_version != args.version && !is_prerelease_of_it {
            return Err(format!(
                "--version {} does not match the workspace version {crate_version}.\n\
                 A prerelease of it ({crate_version}-dev.<run>.<sha>) is accepted.\n\
                 Bump Cargo.toml, or pass --allow-version-drift if this is deliberate.",
                args.version
            )
            .into());
        }
    }

    std::fs::create_dir_all(&args.out)?;
    let artifact_name = format!("{}-{}.tar.zst", args.channel, args.version);
    let artifact = args.out.join(&artifact_name);

    // ── build the artifact ──
    {
        let file = std::fs::File::create(&artifact)?;
        // Level 19 by default: publishing is a one-off, download bandwidth is not.
        //
        // But this is single-threaded, and the cost is set by what you feed it. A release
        // packs stripped aarch64 binaries in ~15s; CI's smoke test packs unstripped debug
        // ones and took ~400s at the same level — over half of that job, for an artifact
        // it deletes. Hence `--zstd-level`, so the throwaway case can pay level 1.
        let encoder = zstd::Encoder::new(file, args.zstd_level)?.auto_finish();
        let mut builder = tar::Builder::new(encoder);

        let mut shipped = Vec::new();
        for entry in std::fs::read_dir(&args.bin_dir)? {
            let path = entry?.path();
            if !path.is_file() {
                continue;
            }
            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .ok_or("binary has an unreadable name")?
                .to_owned();
            // Executable: the robot runs these straight out of the release directory.
            append_file(&mut builder, &path, &format!("bin/{name}"), 0o755)?;
            shipped.push(name);
        }
        if shipped.is_empty() {
            return Err(format!("no binaries found in {}", args.bin_dir.display()).into());
        }
        shipped.sort();

        for include in &args.includes {
            let (src, dest) = include
                .split_once('=')
                .ok_or_else(|| format!("--include expects src=dest, got {include:?}"))?;
            // Hooks and scripts must be executable; everything else needn't be. `scripts/` is
            // there for `robot-rescue`, which an operator may well run straight out of the
            // release directory on a board where nothing else works.
            let mode = if dest.starts_with("hooks/") || dest.starts_with("scripts/") {
                0o755
            } else {
                0o644
            };
            append_file(&mut builder, Path::new(src), dest, mode)?;
        }

        // The preinstall hook, always, generated from its template.
        //
        // Not an `--include` the release workflow has to remember: the board prerequisites it
        // asserts are a property of every release, and a check that ships only when someone
        // adds a flag is a check that will one day be missing from the release that needed it.
        const PREINSTALL_TEMPLATE: &str = "hooks/preinstall.in";
        if args
            .includes
            .iter()
            .any(|i| i.ends_with("=hooks/preinstall"))
        {
            return Err("hooks/preinstall is generated; remove the --include for it".into());
        }
        let template = std::fs::read_to_string(PREINSTALL_TEMPLATE)
            .map_err(|e| format!("reading {PREINSTALL_TEMPLATE}: {e}"))?;
        let hook = render_preinstall_hook(&template)?;
        append_bytes(&mut builder, "hooks/preinstall", hook.as_bytes(), 0o755)?;

        // Recorded inside the release so a robot can identify what it is running even
        // with no network and no manifest.
        let version_toml = format!(
            "version = \"{}\"\nchannel = \"{}\"\nrevision = \"{}\"\nbinaries = {:?}\n",
            args.version,
            args.channel,
            args.revision.as_deref().unwrap_or("unknown"),
            shipped
        );
        append_bytes(&mut builder, VERSION_FILE, version_toml.as_bytes(), 0o644)?;

        builder.finish()?;
        // Dropping the builder finishes the zstd frame; without this the archive is
        // truncated and only fails when someone tries to read it.
        drop(builder);
    }

    let bytes = std::fs::read(&artifact)?;
    let digest = sha256_hex(&bytes);

    // ── the manifest ──
    let url = match &args.base_url {
        Some(base) => format!("{}/{artifact_name}", base.trim_end_matches('/')),
        // Left bare so a later step can rewrite it; `LocalDir` also accepts a bare
        // filename.
        None => artifact_name.clone(),
    };

    let mut manifest = serde_json::json!({
        "channel": args.channel,
        "version": args.version,
        "url": url,
        "sha256": digest,
        "sig_url": format!("{url}{SIG_SUFFIX}"),
        "size": bytes.len(),
        "min_hw_rev": args.min_hw_rev,
        "schema_version": 1,
    });
    if let Some(revision) = &args.revision {
        manifest["source_revision"] = serde_json::json!(revision);
    }
    if let Some(floor) = &args.min_supported {
        manifest["min_supported"] = serde_json::json!(floor);
    }

    let manifest_path = args.out.join("manifest.json");
    std::fs::write(&manifest_path, serde_json::to_vec_pretty(&manifest)?)?;

    // A second manifest whose `url` is a bare filename, which is what `LocalDir`
    // expects. Emitted here so both variants are signed in the same pass:
    //
    //  - CI verifies the release through the robot's own code path without needing the
    //    signing key a second time (fewer places the key is handled is worth more than
    //    one fewer file);
    //  - a developer can drop artifact + this manifest into a directory and sideload it.
    let mut local = manifest.clone();
    local["url"] = serde_json::json!(artifact_name);
    local["sig_url"] = serde_json::json!(format!("{artifact_name}{SIG_SUFFIX}"));
    let local_path = args.out.join(format!("{}.manifest.json", args.version));
    std::fs::write(&local_path, serde_json::to_vec_pretty(&local)?)?;

    println!("packaged {} ({} bytes)", artifact.display(), bytes.len());
    println!("  sha256 {digest}");
    println!("  manifest {}", manifest_path.display());
    println!("  sideload manifest {}", local_path.display());
    println!(
        "\nnext: cargo xtask sign --dir {} --key <key>",
        args.out.display()
    );
    Ok(())
}

/// Generate a keypair and explain what to do with each half.
///
/// **Why the release *spare* must exist now.** A robot verifies against the *set* of
/// public keys baked into its image. If only one release key is baked in and it is
/// later lost or compromised, there is no way to introduce a replacement over the air —
/// the robot would have to be re-flashed by hand. Generating a second release key today
/// and shipping both public keys from the first image means rotation is later just "sign
/// with the other key". Cheap now, impossible to retrofit.
///
/// Refuses to write a secret key inside the repository. Committing a signing key is the
/// one mistake here that cannot be undone by deleting the file.
fn keygen(
    kind: KeyKind,
    name: &str,
    out: &Path,
    password: Option<&str>,
) -> Result<(), Box<dyn std::error::Error>> {
    let repo = std::env::current_dir()?;
    let target = out.canonicalize().unwrap_or_else(|_| {
        // Not created yet; resolve against cwd so the containment check still works.
        if out.is_absolute() {
            out.to_path_buf()
        } else {
            repo.join(out)
        }
    });
    if target.starts_with(&repo) {
        return Err(format!(
            "refusing to write keys inside the repository ({}).\n\
             A committed signing key cannot be un-leaked by deleting it later.\n\
             Pick a path outside the working tree, e.g. --out ~/robot-keys",
            repo.display()
        )
        .into());
    }

    std::fs::create_dir_all(&target)?;

    // The `.dev.` infix is load-bearing, not decoration: `verify::KeyRing` treats a key
    // whose filename ends in `.dev.pub` as usable only when `allow_dev_keys` is set.
    let (pub_name, key_name) = match kind {
        KeyKind::Release => (format!("{name}.pub"), format!("{name}.key")),
        KeyKind::Dev => (format!("{name}.dev.pub"), format!("{name}.dev.key")),
    };
    let pub_path = target.join(&pub_name);
    let key_path = target.join(&key_name);

    for path in [&pub_path, &key_path] {
        if path.exists() {
            return Err(format!(
                "{} already exists — refusing to overwrite a key",
                path.display()
            )
            .into());
        }
    }

    let comment = format!("robot {name} key");
    let keypair = match kind {
        KeyKind::Release => {
            let password = password.map(str::to_owned).ok_or(
                "a release key must be encrypted: set MINISIGN_PASSWORD or pass --password",
            )?;
            minisign::KeyPair::generate_encrypted_keypair(Some(password))?
        }
        // Unencrypted on purpose: CI signs non-interactively, and the secret store is
        // what protects it. An encrypted key plus its passphrase in the same secret
        // store buys little.
        KeyKind::Dev => minisign::KeyPair::generate_unencrypted_keypair()?,
    };

    std::fs::write(&pub_path, keypair.pk.to_box()?.to_string())?;
    write_private(&key_path, &keypair.sk.to_box(Some(&comment))?.to_string())?;

    println!("wrote {}", pub_path.display());
    println!("wrote {} (mode 0600)", key_path.display());
    println!();
    match kind {
        KeyKind::Release => {
            println!("This is a RELEASE key. It is the trust anchor for every robot.");
            println!();
            println!("  public  → into the trusted_keys_dir of every robot image, and");
            println!("            into the MINISIGN_PUBLIC_KEY CI secret");
            println!("  private → a password manager or offline store. Never in the repo,");
            println!("            never on a robot, never in a shared drive.");
            println!("            The CI secret MINISIGN_SECRET_KEY holds a copy for");
            println!("            publishing; treat that copy as the exposed one.");
            println!();
            println!("Generate a SECOND release key now and ship both public keys:");
            println!(
                "  cargo xtask keygen --kind release --name release-2 --out {}",
                out.display()
            );
            println!("Without a spare in the trusted set, a lost key means re-flashing by hand.");
        }
        KeyKind::Dev => {
            println!("This is a DEV key, for signing branch builds.");
            println!();
            println!("  public  → trusted_keys_dir of DEVELOPER boards only, alongside");
            println!("            allow_dev_keys = true in updater.toml");
            println!("  private → shared with the team (password manager / CI secret)");
            println!();
            println!("It must NOT reach a customer robot: a robot that trusts this key");
            println!("will install anything anyone on the team builds.");
        }
    }
    Ok(())
}

/// Write a secret key readable only by its owner.
///
/// Set before the bytes are written, not after: a key that is briefly world-readable on
/// a shared machine has already leaked.
fn write_private(path: &Path, contents: &str) -> Result<(), Box<dyn std::error::Error>> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;

    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)?;
    file.write_all(contents.as_bytes())?;
    file.sync_all()?;
    Ok(())
}

/// Prove a keypair can sign, and that the two halves belong together.
///
/// Does a real sign-and-verify round trip rather than inspecting the files: a key that
/// parses is not necessarily a key that works, and a `.pub` sitting next to a `.key` is
/// not necessarily *its* `.pub`.
fn keycheck(
    key_path: &Path,
    public_path: Option<&Path>,
    password: Option<&str>,
) -> Result<(), Box<dyn std::error::Error>> {
    let default_public = key_path.with_extension("pub");
    let public_path = public_path.unwrap_or(&default_public);

    let key_text = std::fs::read_to_string(key_path)
        .map_err(|e| format!("reading {}: {e}", key_path.display()))?;
    let boxed = minisign::SecretKeyBox::from_string(&key_text)?;

    // Try unencrypted first: that tells us which kind of key this is without needing to
    // be told, and gets it right rather than guessing from the filename.
    let (secret, encrypted) = match boxed.into_unencrypted_secret_key() {
        Ok(secret) => (secret, false),
        Err(_) => {
            let text = std::fs::read_to_string(key_path)?;
            let boxed = minisign::SecretKeyBox::from_string(&text)?;
            let password = password
                .map(str::to_owned)
                .ok_or("this key is encrypted; set MINISIGN_PASSWORD or pass --password")?;
            (boxed.into_secret_key(Some(password))?, true)
        }
    };

    let public_text = std::fs::read_to_string(public_path)
        .map_err(|e| format!("reading {}: {e}", public_path.display()))?;
    let public = minisign::PublicKeyBox::from_string(&public_text)?.into_public_key()?;

    // The actual test.
    let probe = b"xtask keycheck round trip";
    let signature = minisign::sign(None, &secret, &probe[..], None, None)?;
    minisign::verify(
        &public,
        &signature,
        std::io::Cursor::new(&probe[..]),
        true,
        false,
        false,
    )
    .map_err(|e| format!("the public key does not verify this secret key's signature: {e}"))?;

    println!("{}", key_path.display());
    println!(
        "  encrypted: {}",
        if encrypted { "yes" } else { "no — dev key" }
    );
    println!("  public:    {}", public_path.display());
    println!("  round trip: OK — this key can sign, and that .pub verifies it");

    if !encrypted {
        println!();
        println!("  note: an unencrypted key is correct for a DEV key (CI signs without a");
        println!("        passphrase) and wrong for a release key.");
    }
    Ok(())
}

/// Sign every artifact and manifest in `dir`.
///
/// Both are signed: the manifest so a robot can trust what it says, and the artifact so
/// the bytes can be verified independently of it.
fn sign_dir(
    dir: &Path,
    key_path: &Path,
    password: Option<&str>,
) -> Result<(), Box<dyn std::error::Error>> {
    let key_text = std::fs::read_to_string(key_path)
        .map_err(|e| format!("reading {}: {e}", key_path.display()))?;
    let boxed = minisign::SecretKeyBox::from_string(&key_text)?;

    // An unencrypted key is the CI case (the secret is already protected by the secret
    // store); an encrypted one needs the passphrase. Guessing wrong gives a confusing
    // error, so pick explicitly.
    let secret = match password {
        Some(password) => boxed.into_secret_key(Some(password.to_owned()))?,
        None => boxed.into_unencrypted_secret_key()?,
    };

    let mut signed = 0;
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or_default();
        if !path.is_file() || name.ends_with(SIG_SUFFIX) {
            continue;
        }

        let bytes = std::fs::read(&path)?;
        let signature = minisign::sign(None, &secret, bytes.as_slice(), None, None)?.to_string();
        let sig_path = PathBuf::from(format!("{}{SIG_SUFFIX}", path.display()));
        std::fs::write(&sig_path, signature)?;
        println!("signed {name}");
        signed += 1;
    }

    if signed == 0 {
        return Err(format!("nothing to sign in {}", dir.display()).into());
    }
    Ok(())
}

/// Carry a validated staging artifact into the stable channel.
///
/// The artifact is **not** rebuilt: the sha256 comes from the staging manifest, so the
/// stable channel serves the same bytes that passed staging. That is the whole point of
/// §16.3 — promotion is a decision, not a build.
fn promote(
    version: &semver::Version,
    staging_tag: &str,
    stable_tag: &str,
    repo: &str,
    staging_manifest: &Path,
    out: &Path,
    min_supported: Option<&semver::Version>,
) -> Result<(), Box<dyn std::error::Error>> {
    let staging: serde_json::Value = serde_json::from_slice(&std::fs::read(staging_manifest)?)?;

    let staged_version: semver::Version = serde_json::from_value(staging["version"].clone())?;
    if staged_version != *version {
        return Err(format!(
            "staging manifest is version {staged_version}, asked to promote {version}"
        )
        .into());
    }

    let artifact_name = staging["url"]
        .as_str()
        .and_then(|u| u.rsplit('/').next())
        .ok_or("staging manifest has no usable url")?;

    // Point at the artifact on the *stable* release — which makes that release
    // self-contained, and staging disposable once promotion succeeds. `promote.yml`
    // uploads these exact bytes under this tag; the two have to agree, and the test
    // `promote_yml_uploads_the_artifact_it_points_at` is what keeps them agreeing.
    let url = format!("https://github.com/{repo}/releases/download/{stable_tag}/{artifact_name}");

    let mut manifest = staging.clone();
    manifest["url"] = serde_json::json!(url);
    manifest["sig_url"] = serde_json::json!(format!("{url}{SIG_SUFFIX}"));
    match min_supported {
        Some(floor) => manifest["min_supported"] = serde_json::json!(floor),
        // Not inherited: a floor set to remediate a bad staging build should not
        // silently become a fleet-wide forced upgrade.
        None => {
            manifest.as_object_mut().map(|m| m.remove("min_supported"));
        }
    }

    std::fs::create_dir_all(out)?;
    let path = out.join("manifest.json");
    std::fs::write(&path, serde_json::to_vec_pretty(&manifest)?)?;

    println!("promoted {version} from {staging_tag}");
    println!("  artifact {url}");
    println!(
        "  sha256 {} (unchanged)",
        staging["sha256"].as_str().unwrap_or("?")
    );
    println!("  manifest {}", path.display());
    println!(
        "\nnext: cargo xtask sign --dir {} --key <key>",
        out.display()
    );
    Ok(())
}

// ── helpers ──────────────────────────────────────────────────────────────────

/// ONNX Runtime floor and target from `[workspace.metadata.onnxruntime]`.
///
/// One source of truth for a value that has to agree in three places — the preinstall hook,
/// `scripts/setup-board.sh`, and whatever `ort` requires. Two of those drifted apart once
/// already, and the board that resulted could install a release and then only load a policy
/// far enough to panic.
fn onnxruntime_versions() -> Result<(String, String), Box<dyn std::error::Error>> {
    let manifest: toml::Value = toml::from_str(&std::fs::read_to_string("Cargo.toml")?)?;
    let table = manifest
        .get("workspace")
        .and_then(|w| w.get("metadata"))
        .and_then(|m| m.get("onnxruntime"))
        .ok_or("Cargo.toml has no [workspace.metadata.onnxruntime]")?;
    let get = |key: &str| -> Result<String, Box<dyn std::error::Error>> {
        Ok(table
            .get(key)
            .and_then(|v| v.as_str())
            .ok_or_else(|| format!("[workspace.metadata.onnxruntime] has no {key}"))?
            .to_owned())
    };
    Ok((get("floor")?, get("target")?))
}

/// Fill in the preinstall hook from its template.
///
/// Generated rather than committed so the hook cannot disagree with the release it ships
/// inside: both come from the same `Cargo.toml` in the same build.
fn render_preinstall_hook(template: &str) -> Result<String, Box<dyn std::error::Error>> {
    let (floor, target) = onnxruntime_versions()?;
    let rendered = template
        .replace("@ONNX_FLOOR@", &floor)
        .replace("@ONNX_TARGET@", &target);
    if rendered.contains("@ONNX_") {
        return Err("preinstall template still has unsubstituted @ONNX_...@ placeholders".into());
    }
    Ok(rendered)
}

fn workspace_version() -> Result<semver::Version, Box<dyn std::error::Error>> {
    let manifest: toml::Value = toml::from_str(&std::fs::read_to_string("Cargo.toml")?)?;
    let raw = manifest
        .get("workspace")
        .and_then(|w| w.get("package"))
        .and_then(|p| p.get("version"))
        .and_then(|v| v.as_str())
        .ok_or("Cargo.toml has no [workspace.package] version")?;
    Ok(semver::Version::parse(raw)?)
}

fn append_file(
    builder: &mut tar::Builder<impl std::io::Write>,
    src: &Path,
    dest: &str,
    mode: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let bytes = std::fs::read(src).map_err(|e| format!("reading {}: {e}", src.display()))?;
    append_bytes(builder, dest, &bytes, mode)
}

fn append_bytes(
    builder: &mut tar::Builder<impl std::io::Write>,
    dest: &str,
    bytes: &[u8],
    mode: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut header = tar::Header::new_gnu();
    header.set_size(bytes.len() as u64);
    header.set_mode(mode);
    // Fixed mtime so the same inputs produce the same archive: a reproducible artifact
    // means a rebuild can be compared against what shipped.
    header.set_mtime(0);
    header.set_cksum();
    builder.append_data(&mut header, dest, bytes)?;
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    use std::fmt::Write;
    Sha256::digest(bytes)
        .iter()
        .fold(String::new(), |mut s, b| {
            let _ = write!(s, "{b:02x}");
            s
        })
}

#[cfg(test)]
mod tests {
    /// Every file that packages a release, which is where the `--include` list and the staged
    /// binaries live. Repository paths, because one of them is not a workflow.
    ///
    /// Named once, because the tests below all read the same files and the recipe has moved before:
    /// it used to sit in `release.yml`, and now lives in the reusable `_build-release.yml` that both
    /// the staging and stable paths call. A test that kept reading the old name would pass while
    /// guarding nothing, which is worse than failing.
    ///
    /// `scripts/dev-push.sh` is the third because it assembles the same artifact from its own copy
    /// of the same lists — a laptop build a board actually runs. `xtask/tests/artifact.rs` opens the
    /// tarball each of these produces; the tests below are the cheaper string form of the same
    /// question, and they have to look at the same set or the copy that drifts is whichever one they
    /// skip.
    const PACKAGING_SITES: [&str; 3] = [
        ".github/workflows/dev.yml",
        ".github/workflows/_build-release.yml",
        "scripts/dev-push.sh",
    ];

    /// Where promotion happens: the stable manifest, the artifact carried forward, the retire step.
    const PROMOTE_WORKFLOW: &str = "_promote-release.yml";

    /// Where a unit's `ExecStart` points when it runs a program out of the live release.
    ///
    /// Nearly all of them do, and for those the binary has to be staged and packaged or the unit
    /// fails with `203/EXEC`. The exception is the boot recovery net, which execs out of the base
    /// precisely so that a broken release cannot break it.
    const RELEASE_BIN_DIR: &str = "/opt/robot/daemon/current/bin/";
    /// Every unit `install.sh` installs must actually be in the artifact.
    ///
    /// The packaging workflows name each shipped file with an explicit `--include`, and
    /// `install.sh` reads them back out of the installed release — two lists with nothing tying
    /// them together. They drifted the first time it mattered: `configd.service` and
    /// `btd.service` were written, installed by `install.sh`, and not packaged, so a release
    /// carried both binaries and no way to run them. The failure is silent at build time and
    /// looks like a broken daemon on the board.
    #[test]
    fn every_unit_install_sh_expects_is_packaged() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");
        let install = std::fs::read_to_string(root.join("scripts/install.sh"))
            .expect("scripts/install.sh must exist");

        // Every `for unit in …` loop in the script, unioned. There is more than one — units are
        // installed, and also stopped for a forced re-install — and a unit named in any of them
        // is a unit the board is expected to have. Trailing `;` from `; do` is stripped, which
        // is how this test first failed to see `updaterd.service`.
        let mut units: Vec<String> = install
            .lines()
            .filter(|l| l.contains("for unit in"))
            .flat_map(|l| l.split_whitespace())
            .map(|w| w.trim_end_matches(';').to_owned())
            .filter(|w| w.ends_with(".service"))
            .collect();
        units.sort();
        units.dedup();
        assert!(units.len() >= 4, "expected several units, found {units:?}");

        for workflow in PACKAGING_SITES {
            let text = std::fs::read_to_string(root.join(workflow))
                .unwrap_or_else(|e| panic!("{workflow}: {e}"));
            for unit in &units {
                let expected = format!("=systemd/{unit}");
                assert!(
                    text.contains(&expected),
                    "{workflow} does not package {unit}, but install.sh installs it. \
                     Add:  --include \"<crate>/systemd/{unit}=systemd/{unit}\""
                );
            }
        }
    }

    /// Every script a hook runs out of the release must be packaged.
    ///
    /// The pre-install hook installs what the release needs and cannot have — ONNX Runtime, and the
    /// GStreamer stack — and for the second it runs `scripts/setup-gstreamer.sh` from the release
    /// rather than carrying a second copy of the package list, the pinned plugins version and the
    /// udev rule. A script that is referenced and not packaged makes that step a no-op that says so
    /// in a log nobody reads, on exactly the boards it exists for: the hook skips it and `mediad`
    /// then fails to start with a missing plugin.
    ///
    /// The same drift `every_unit_install_sh_expects_is_packaged` guards, one directory over.
    #[test]
    fn every_script_the_hooks_run_is_packaged() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        let mut scripts: Vec<String> = Vec::new();
        for hook in ["hooks/preinstall.in", "hooks/postinstall"] {
            let text =
                std::fs::read_to_string(root.join(hook)).unwrap_or_else(|e| panic!("{hook}: {e}"));
            // `script=scripts/<name>` — an assignment, which is how a hook names a path it runs,
            // rather than every mention of the word in a comment.
            for line in text.lines() {
                let Some((_, rest)) = line.split_once("=scripts/") else {
                    continue;
                };
                let name: String = rest
                    .chars()
                    .take_while(|c| c.is_alphanumeric() || "._-".contains(*c))
                    .collect();
                if !name.is_empty() {
                    scripts.push(format!("scripts/{name}"));
                }
            }
        }
        scripts.sort();
        scripts.dedup();
        assert!(
            !scripts.is_empty(),
            "no scripts/… path found in the hooks; this test is watching nothing"
        );

        for script in &scripts {
            assert!(
                root.join(script).exists(),
                "a hook runs {script}, which does not exist"
            );
            for workflow in PACKAGING_SITES {
                let text = std::fs::read_to_string(root.join(workflow))
                    .unwrap_or_else(|e| panic!("{workflow}: {e}"));
                let expected = format!("={script}");
                assert!(
                    text.contains(&expected),
                    "{workflow} does not package {script}, but a hook runs it. \
                     Add:  --include \"{script}={script}\""
                );
            }
        }
    }

    /// `install_*` steps a hook performs too, and where it does it.
    const ALSO_ON_UPDATE: [(&str, &str); 1] = [(
        "install_units",
        "hooks/postinstall installs, enables and starts every unit the release ships. Not the \
         journald drop-in and not the robotctl symlink, which the hook deliberately leaves alone \
         — board-test.sh pins that asymmetry, and it is the one thing here §9.1 does not cover",
    )];

    /// `install_*` steps only a fresh install performs, and why a board that only updates does
    /// not need them. Each of these is a decision belonging to the board rather than to a
    /// release, which is the only reason a release may leave it alone.
    const FIRST_INSTALL_ONLY: [(&str, &str); 3] = [
        (
            "install_config",
            "/etc/robot/*.toml belongs to the board: install.sh will not overwrite an existing \
             updater.toml, and an update must not either",
        ),
        (
            "install_dev_key",
            "a trust anchor is the operator's decision. A release that installed trusted keys \
             would be granting itself trust",
        ),
        (
            "install_token_dropin",
            "the fetch credential is supplied by whoever runs the install and is never in an \
             artifact; a customer robot never has one at all",
        ),
    ];

    /// Every step `install.sh` performs on a board is performed on an updated board too.
    ///
    /// This is the direction `docs/design/updater-design.md` §9.1 is about, and the one nothing
    /// watched. `every_script_the_hooks_run_is_packaged` above checks that a script a hook
    /// *already names* ships; it cannot notice a step no hook names at all. That is the mistake,
    /// four times: units left where systemd never looks, a GStreamer stack only provisioning
    /// installed, a `setup-npu.sh` packaged beside its model and never called, and a
    /// `/etc/profile.d` snippet that sat in `install.sh` alone while every board in the fleet
    /// updated past it. The fourth went unnoticed for a month because it is cosmetic — the
    /// robot works, the prompt is just wrong — which is the argument for a test rather than for
    /// a rule people are supposed to remember.
    ///
    /// Two halves, because the step can be missing in two shapes: a function `install.sh` runs
    /// and no hook does, and a shared `setup-*.sh` only `install.sh` calls.
    ///
    /// This is a forcing function, not a proof. An author can satisfy it by adding a name to
    /// `FIRST_INSTALL_ONLY` — but they have to write down why an already-provisioned board does
    /// not need the thing they just added, and every one of the four would have failed at that
    /// sentence.
    #[test]
    fn every_install_sh_step_reaches_an_updated_board() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");
        let install = std::fs::read_to_string(root.join("scripts/install.sh"))
            .expect("scripts/install.sh must exist");

        // ── half one: every `install_*` step is accounted for ──
        //
        // Call sites, not definitions: a function that is defined and never called does nothing
        // to a board. `    install_foo` on a line of its own is how this script calls one.
        let mut called: Vec<&str> = install
            .lines()
            .filter_map(|line| {
                let name = line.trim();
                let indented = line.starts_with(' ') || line.starts_with('\t');
                (indented
                    && name.starts_with("install_")
                    && !name.contains(|c: char| !(c.is_ascii_lowercase() || c == '_')))
                .then_some(name)
            })
            .collect();
        called.sort();
        called.dedup();
        assert!(
            !called.is_empty(),
            "no install_* call sites found in install.sh; this test is watching nothing"
        );

        let mut accounted: Vec<&str> = ALSO_ON_UPDATE
            .iter()
            .chain(FIRST_INSTALL_ONLY.iter())
            .map(|(name, _)| *name)
            .collect();
        accounted.sort();

        for name in &called {
            assert!(
                accounted.contains(name),
                "install.sh calls {name}, and nothing says whether an already-provisioned board \
                 ever gets it.\n\
                 Read docs/design/updater-design.md §9.1. Then either move the step into a \
                 scripts/setup-*.sh that hooks/postinstall runs too — which is what \
                 setup-login.sh is — or add {name} to FIRST_INSTALL_ONLY here with the reason a \
                 board that only updates does not need it."
            );
        }
        for name in &accounted {
            assert!(
                called.contains(name),
                "{name} is listed here but install.sh no longer calls it; drop the entry"
            );
        }

        // ── half two: a shared setup script both paths run ──
        //
        // `install.sh` runs one out of the installed release, as `current/scripts/setup-x.sh`.
        // Matching that exact form and not the bare name on purpose: the script also *names*
        // setup-board.sh in a message telling an operator to go run it, which is not the same
        // thing as running it.
        let mut shared: Vec<String> = Vec::new();
        for (_, rest) in install
            .split("current/scripts/setup-")
            .skip(1)
            .map(|r| ("", r))
        {
            let name: String = rest
                .chars()
                .take_while(|c| c.is_alphanumeric() || "._-".contains(*c))
                .collect();
            if name.ends_with(".sh") {
                shared.push(format!("scripts/setup-{name}"));
            }
        }
        shared.sort();
        shared.dedup();

        let hooks: String = ["hooks/preinstall.in", "hooks/postinstall"]
            .iter()
            .map(|h| std::fs::read_to_string(root.join(h)).unwrap_or_else(|e| panic!("{h}: {e}")))
            .collect();
        for script in &shared {
            assert!(
                hooks.contains(&format!("script={script}")),
                "install.sh runs {script} out of the release and no hook does. A board that only \
                 updates never gets it — see docs/design/updater-design.md §9.1. Add \
                 `script={script}` to hooks/postinstall."
            );
        }
    }

    /// The policy set, as `robotd-params` resolves it across both drive modes.
    ///
    /// The one list everything else must agree with: these are the files a slot can default to,
    /// so they are exactly the files that have to exist on a board.
    fn policies_robotd_expects() -> Vec<String> {
        let mut names = Vec::new();
        for mode in [robotd_params::Mode::Walk, robotd_params::Mode::Roller] {
            let params = robotd_params::PolicyParams {
                mode,
                ..Default::default()
            };
            let resolved = params.resolved();
            for slot in robotd_params::Slot::ALL {
                if let Some(path) = resolved.slot(slot) {
                    names.push(path.file_name().unwrap().to_string_lossy().into_owned());
                }
            }
        }
        names.sort();
        names.dedup();
        names
    }

    /// Run `scripts/seed-policies.sh` against a throwaway tree, with the Hub faked by a directory.
    ///
    /// `base_url` of `None` is an unreachable Hub, which is the case the fallback is for.
    /// Returns what `current` points at, and what the walking policy contains through it.
    fn seed(
        root: &std::path::Path,
        version: &str,
        base_url: Option<&std::path::Path>,
    ) -> (Option<String>, Option<String>) {
        let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");
        let url = match base_url {
            Some(dir) => format!("file://{}", dir.display()),
            None => "file:///nonexistent-hub".to_owned(),
        };
        let status = std::process::Command::new("sh")
            .arg(repo_root.join("scripts/seed-policies.sh"))
            .arg(root)
            .env("POLICY_VERSION", version)
            .env("POLICY_BASE_URL", url)
            .stderr(std::process::Stdio::null())
            .status()
            .expect("sh");
        assert!(status.success(), "the seeder must never fail an update");

        let link = std::fs::read_link(root.join("current"))
            .ok()
            .map(|p| p.display().to_string());
        let content = std::fs::read_to_string(root.join("current/alpha_walking.onnx")).ok();
        (link, content)
    }

    /// A directory standing in for the Hub repo at some revision.
    fn fake_hub(dir: &std::path::Path, marker: &str) {
        std::fs::create_dir_all(dir).expect("mkdir");
        for name in policies_robotd_expects() {
            std::fs::write(dir.join(&name), format!("{marker}-{name}")).expect("policy");
        }
    }

    /// **The pin is in two places and they must agree.** `seed-policies.sh` runs from inside a
    /// release and cannot read Cargo.toml, so it carries the repo and the version as literals —
    /// the same trap `setup-gstreamer.sh` and `setup-board.sh` already carry, where a drift is a
    /// board running weights nobody can name.
    #[test]
    fn seed_policies_pins_the_same_policy_set() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let manifest: toml::Value =
            toml::from_str(&std::fs::read_to_string(root.join("Cargo.toml")).unwrap()).unwrap();
        let meta = &manifest["workspace"]["metadata"]["policies"];
        let version = meta["version"].as_str().unwrap();
        let repo = meta["repo"].as_str().unwrap();

        let script = std::fs::read_to_string(root.join("scripts/seed-policies.sh")).unwrap();
        for expected in [
            format!("POLICY_VERSION=\"${{POLICY_VERSION:-{version}}}\""),
            format!("POLICY_REPO=\"${{POLICY_REPO:-{repo}}}\""),
        ] {
            assert!(
                script.contains(&expected),
                "seed-policies.sh must carry the line {expected:?}"
            );
        }
    }

    /// **The fallback list must still be what `robotd` can ask for.**
    ///
    /// The download list comes from the set's own `manifest.json` now, so a tenth policy is a tag
    /// rather than an edit here. What is left in the script is the fallback for a revision tagged
    /// before the manifest existed — and it is still a list that can go wrong in both directions:
    /// a name nothing asks for is dead weight on the eMMC, and one a slot defaults to that is
    /// missing is a slot that will not load, reported as degraded on every board.
    ///
    /// This goes with the fallback, once every tagged set carries a manifest.
    #[test]
    fn the_fallback_list_is_exactly_what_robotd_defaults_to() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let script = std::fs::read_to_string(root.join("scripts/seed-policies.sh")).unwrap();
        let line = script
            .lines()
            .find(|l| l.starts_with("FALLBACK_FILES="))
            .expect("seed-policies.sh must declare FALLBACK_FILES");
        let mut listed: Vec<String> = line
            .trim_start_matches("FALLBACK_FILES=")
            .trim_matches('"')
            .split_whitespace()
            .map(str::to_owned)
            .collect();
        listed.sort();

        assert_eq!(
            listed,
            policies_robotd_expects(),
            "the fallback list and robotd's own defaults have drifted"
        );
    }

    /// **The seeder's `sed` must match what the manifest actually says.**
    ///
    /// There is no JSON parser where that script runs — a release on a board with curl and a
    /// POSIX shell — so the file list is extracted with one pattern over a file whose shape is
    /// ours. That is fine exactly as long as the two agree, and silently downloads nothing the
    /// moment they do not: a fresh board would fall back to nine names and never see a tenth.
    #[test]
    fn the_seeders_pattern_reads_the_manifest_it_is_given() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let script = root.join("scripts/seed-policies.sh");

        // The shape `robotd_params::SetManifest` deserializes, written the way we would publish
        // it — including a policy whose name differs from its file, and one with nothing but a
        // file, so the pattern is not relying on neighbouring fields.
        let manifest = serde_json::json!({
            "schema_version": 1,
            "policies": [
                { "file": "alpha_walking.onnx", "kind": "perpetual" },
                { "file": "ball_kick_left.onnx", "name": "kick_left",
                  "kind": "episodic", "duration_s": 0.5 },
                // A command block: nested keys the pattern must neither match nor choke on.
                { "file": "alpha_ground_pick.onnx", "name": "ground_pick", "kind": "episodic",
                  "duration_s": 2.8, "command": { "encoding": "phase", "period_s": 4.0,
                  "end_phase": 0.7, "slots": "twist.vx,twist.vy" } },
                { "file": "roulade.onnx" }
            ]
        });
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("manifest.json");
        std::fs::write(&path, serde_json::to_string_pretty(&manifest).unwrap()).unwrap();

        // The same expression the script runs, so a change to one fails here rather than on a
        // board six weeks later.
        let text = std::fs::read_to_string(&script).unwrap();
        let sed = text
            .lines()
            .find(|l| l.contains("sed -n 's/.*\"file\""))
            .expect("the extraction line");
        let expression = sed
            .split_once('\'')
            .and_then(|(_, rest)| rest.rsplit_once('\''))
            .map(|(expr, _)| expr.to_owned())
            .expect("a quoted sed expression");

        let out = std::process::Command::new("sed")
            .arg("-n")
            .arg(&expression)
            .arg(&path)
            .output()
            .expect("sed");
        let files: Vec<String> = String::from_utf8_lossy(&out.stdout)
            .split_whitespace()
            .map(str::to_owned)
            .collect();

        assert_eq!(
            files,
            vec![
                "alpha_walking.onnx".to_string(),
                "ball_kick_left.onnx".to_string(),
                "alpha_ground_pick.onnx".to_string(),
                "roulade.onnx".to_string()
            ],
            "the pattern and the manifest have drifted"
        );
    }

    /// The ordinary path: nothing installed, the Hub reachable, the pinned set arrives whole.
    #[test]
    fn the_pinned_set_is_downloaded_from_the_hub() {
        let tmp = tempfile::tempdir().unwrap();
        let hub = tmp.path().join("hub");
        let root = tmp.path().join("policies");
        fake_hub(&hub, "hub");
        std::fs::create_dir_all(&root).unwrap();

        let (link, content) = seed(&root, "v1", Some(&hub));
        assert_eq!(link.as_deref(), Some("releases/seed-v1"));
        assert_eq!(content.as_deref(), Some("hub-alpha_walking.onnx"));

        let mut installed: Vec<String> = std::fs::read_dir(root.join("releases/seed-v1"))
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.ends_with(".onnx"))
            .collect();
        installed.sort();
        assert_eq!(installed, policies_robotd_expects(), "the whole set");
        assert!(
            root.join("releases/seed-v1/.source").exists(),
            "and a record of where it came from, which is what `policy check` reads"
        );
    }

    /// **A manifest entry that is not a file name is ignored, not fetched.** The repo this
    /// downloads from is an environment variable, and `${staging}/${name}` would otherwise let a
    /// `file` naming `../../etc/…` choose where a download lands. `files_in_manifest` in
    /// `updater::policy` applies the same rule to the same field, for the same reason.
    #[test]
    fn a_manifest_file_name_that_climbs_out_is_ignored() {
        let tmp = tempfile::tempdir().unwrap();
        let hub = tmp.path().join("hub");
        let root = tmp.path().join("policies");
        fake_hub(&hub, "hub");
        std::fs::create_dir_all(&root).unwrap();

        let escape = "../../escaped.onnx";
        std::fs::write(
            hub.join("manifest.json"),
            serde_json::to_string_pretty(&serde_json::json!({
                "schema_version": 1,
                "policies": [
                    { "file": "alpha_walking.onnx", "kind": "perpetual" },
                    { "file": escape },
                ]
            }))
            .unwrap(),
        )
        .unwrap();
        // Reachable if the seeder asks for it, so the test can tell "refused" from "404".
        std::fs::write(hub.join("escaped.onnx"), "escaped").unwrap();

        let (link, content) = seed(&root, "v1", Some(&hub));
        assert_eq!(link.as_deref(), Some("releases/seed-v1"));
        assert_eq!(content.as_deref(), Some("hub-alpha_walking.onnx"));
        assert!(
            !root.join("escaped.onnx").exists() && !tmp.path().join("escaped.onnx").exists(),
            "nothing was written outside the set"
        );
    }

    /// **The pinned set already installed means no network at all.** This runs inside every
    /// update, and re-downloading seven megabytes each time to arrive at the same bytes would be
    /// both slow and a way to spend the hook's timeout budget on nothing.
    #[test]
    fn an_already_pinned_set_is_not_downloaded_again() {
        let tmp = tempfile::tempdir().unwrap();
        let hub = tmp.path().join("hub");
        let root = tmp.path().join("policies");
        fake_hub(&hub, "hub");
        std::fs::create_dir_all(&root).unwrap();

        let first = seed(&root, "v1", Some(&hub));
        // No Hub at all the second time: reaching for one would fall back and change the answer.
        assert_eq!(seed(&root, "v1", None), first);
    }

    /// **A set installed before the provenance record existed must gain one.**
    ///
    /// From a board: the fast path — the pinned set is already installed, so no network — exits
    /// before anything is written, which is right for the policies and wrong for the record. A
    /// board seeded by the previous version of this script would take that branch forever and
    /// never gain one, and `robotctl policy check` reported a robot with a perfectly good set as
    /// having nothing installed.
    #[test]
    fn a_set_with_no_provenance_record_gains_one() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("policies");
        let seeded = root.join("releases/seed-v1");
        std::fs::create_dir_all(&seeded).unwrap();
        for name in policies_robotd_expects() {
            std::fs::write(seeded.join(&name), "x").unwrap();
        }
        std::os::unix::fs::symlink("releases/seed-v1", root.join("current")).unwrap();
        assert!(
            !root.join("current/.source").exists(),
            "the board's starting state"
        );

        // No Hub, deliberately: the point is that this happens on the branch that touches no
        // network at all, which is the branch such a board takes every time.
        seed(&root, "v1", None);

        let record = std::fs::read_to_string(root.join("current/.source")).unwrap();
        assert!(record.contains("version=v1"), "{record}");
        assert!(record.contains("repo=pollen-robotics/"), "{record}");
    }

    /// And it is written once. This runs on every update, and rewriting the file each time just
    /// to move a timestamp is churn on an eMMC for nothing.
    #[test]
    fn a_provenance_record_is_not_rewritten_on_every_update() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("policies");
        let hub = tmp.path().join("hub");
        fake_hub(&hub, "hub");
        std::fs::create_dir_all(&root).unwrap();

        seed(&root, "v1", Some(&hub));
        let first = std::fs::read_to_string(root.join("current/.source")).unwrap();
        seed(&root, "v1", Some(&hub));
        assert_eq!(
            std::fs::read_to_string(root.join("current/.source")).unwrap(),
            first
        );
    }

    /// **An installed set is never replaced, whatever the pin says.**
    ///
    /// The pin is a floor — what a board with nothing gets — and not a ceiling. This used to
    /// replace an older set on the reasoning that a daemon update was still how a retrained gait
    /// reached a board; `robotctl policy update` is now how, and the old rule became a trap. A
    /// board moved forward to v2 by hand had `current -> releases/seed-v2`, which matches the
    /// `seed-*` the seeder called its own, so the next unrelated daemon update would have put v1
    /// back — silently reverting somebody's gait as a side effect of a binary update.
    #[test]
    fn a_set_already_installed_is_left_alone_whatever_the_pin_says() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("policies");
        std::fs::create_dir_all(&root).unwrap();

        let v2 = tmp.path().join("hub-v2");
        fake_hub(&v2, "chosen");
        seed(&root, "v2", Some(&v2));

        // The release pins v1 and offers it. A board on v2 must stay on v2.
        let v1 = tmp.path().join("hub-v1");
        fake_hub(&v1, "pinned");
        let (link, content) = seed(&root, "v1", Some(&v1));

        assert_eq!(link.as_deref(), Some("releases/seed-v2"));
        assert_eq!(content.as_deref(), Some("chosen-alpha_walking.onnx"));
    }

    /// A board that cannot reach the Hub on a first install ends up with no policies, and that
    /// is the accepted shape rather than a bug: `robotd` holds its pose and reports *degraded*,
    /// the update gate passes, and the next update fetches. What must not happen is the seeder
    /// failing — a non-zero exit here fails the post-install hook, which rolls the update back
    /// over a network problem that has nothing to do with the release.
    #[test]
    fn an_unreachable_hub_installs_nothing_and_does_not_fail() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("policies");
        std::fs::create_dir_all(&root).unwrap();

        assert_eq!(seed(&root, "v1", None), (None, None));
    }

    /// **A set this script did not install is never touched.**
    ///
    /// The rule the whole handover rests on. Once anything else puts policies there — `robotctl
    /// policy`, or whatever tool publishes them — the release must stop replacing it, or the next
    /// unrelated daemon update silently reverts somebody's gait. No flag, no config.
    #[test]
    fn policies_this_script_did_not_install_are_left_alone() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("policies");
        let hub = tmp.path().join("hub");
        fake_hub(&hub, "hub");
        std::fs::create_dir_all(root.join("releases/from-a-tool")).unwrap();
        std::fs::write(
            root.join("releases/from-a-tool/alpha_walking.onnx"),
            "installed-by-something-else",
        )
        .unwrap();
        std::os::unix::fs::symlink("releases/from-a-tool", root.join("current")).unwrap();

        let (link, content) = seed(&root, "v1", Some(&hub));
        assert_eq!(link.as_deref(), Some("releases/from-a-tool"));
        assert_eq!(content.as_deref(), Some("installed-by-something-else"));
    }

    /// A partial download must never become the live set. `robotd` would refuse the truncated
    /// file at load and report degraded, which is the right end state — but the set it is
    /// refusing should not have replaced a working one to get there.
    #[test]
    fn a_partial_download_does_not_replace_a_working_set() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("policies");
        std::fs::create_dir_all(&root).unwrap();

        let good = tmp.path().join("hub-v1");
        fake_hub(&good, "one");
        seed(&root, "v1", Some(&good));

        // A revision that is missing one file, which is what a half-published set looks like.
        let broken = tmp.path().join("hub-v2");
        fake_hub(&broken, "two");
        std::fs::remove_file(broken.join("roulade.onnx")).unwrap();

        let (link, content) = seed(&root, "v2", Some(&broken));
        assert_ne!(
            link.as_deref(),
            Some("releases/seed-v2"),
            "a set missing a file must not be installed as the pinned one"
        );
        assert_eq!(
            content.as_deref(),
            Some("one-alpha_walking.onnx"),
            "and the board keeps something that works"
        );
    }

    /// The petting classifier still ships inside the release, and drifts the way the policies
    /// used to: three `--include` copies. It is a detector rather than a control policy, small,
    /// and nothing versions it independently — so it stays where the policies left. robotd's default `pet_model` path expects `models/pet_detect.onnx`
    /// inside the release, so a site that forgets it produces robots that silently cannot
    /// hear — the mic worker logs "unavailable" once and everything else looks fine.
    #[test]
    fn the_pet_model_is_packaged_at_every_site() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");
        assert!(
            root.join("pet-detect/models/pet_detect.onnx").exists(),
            "the vendored model is gone"
        );
        for site in PACKAGING_SITES {
            let text =
                std::fs::read_to_string(root.join(site)).unwrap_or_else(|e| panic!("{site}: {e}"));
            assert!(
                text.contains("=models/pet_detect.onnx"),
                "{site} does not package the petting classifier"
            );
        }
    }

    /// The stable manifest names an artifact URL under the stable tag — so the workflow
    /// that creates that release must actually upload the artifact to it.
    ///
    /// These two halves live in different languages and different files, and the failure
    /// mode when they disagree is invisible until a robot tries to update: the release
    /// looks complete, is correctly signed, and its `url` 404s. That is not hypothetical.
    /// It is exactly the state `daemon-v0.1.0`, `v0.1.1` and `v0.1.4` were left in when
    /// the manifest pointed at a staging release someone later deleted.
    #[test]
    fn promote_yml_uploads_the_artifact_it_points_at() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");
        let yml = std::fs::read_to_string(root.join(".github/workflows").join(PROMOTE_WORKFLOW))
            .unwrap_or_else(|e| panic!("{PROMOTE_WORKFLOW}: {e}"));

        assert!(
            yml.contains("--stable-tag"),
            "the promote workflow must pass --stable-tag, or the manifest url is built from the \
             wrong release"
        );
        assert!(
            yml.contains("\"artifact/$artifact_name\""),
            "the promote workflow must upload the artifact to the stable release — the manifest's \
             url points there"
        );
        assert!(
            yml.contains("\"artifact/$artifact_name.minisig\""),
            "the promote workflow must upload the artifact signature too — `sig_url` is derived \
             from `url` and points at the same release"
        );

        // Retiring staging is only safe because of the two uploads above. If someone
        // removes them, this assertion is the one that should look wrong.
        assert!(
            yml.contains("gh release delete \"$staging_tag\""),
            "the promote workflow should retire the staging release once stable is self-contained"
        );
    }

    /// A unit's `sysusers.d` file must be in the artifact too.
    ///
    /// The same drift as the unit test above, one level down, and it fails in a nastier way: a unit
    /// naming a `User=` that does not exist does not start, and the error reads as a broken daemon
    /// rather than as a missing account. `hooks/postinstall` installs every sysusers file the
    /// release ships, so being in the artifact is the whole requirement.
    ///
    /// Discovered from the repository rather than a list: `<crate>/systemd/sysusers.d/*.conf` is
    /// where they live, so adding a service user and forgetting to package it fails here.
    #[test]
    fn every_sysusers_file_in_the_repo_is_packaged() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        let mut found = 0;
        for crate_dir in std::fs::read_dir(root).expect("the workspace root must be readable") {
            let sysusers = crate_dir
                .expect("readable entry")
                .path()
                .join("systemd/sysusers.d");
            if !sysusers.is_dir() {
                continue;
            }
            for entry in std::fs::read_dir(&sysusers)
                .expect("readable sysusers.d")
                .flatten()
            {
                let name = entry.file_name().to_string_lossy().to_string();
                if !name.ends_with(".conf") {
                    continue;
                }
                found += 1;
                for workflow in PACKAGING_SITES {
                    let text = std::fs::read_to_string(root.join(workflow))
                        .unwrap_or_else(|e| panic!("{workflow}: {e}"));
                    let expected = format!("=systemd/sysusers.d/{name}");
                    assert!(
                        text.contains(&expected),
                        "{workflow} does not package {name}, so the account it creates will not \
                         exist and the unit naming it will not start"
                    );
                }
            }
        }
        assert!(found >= 2, "expected several sysusers files, found {found}");
    }

    /// A hook that exists in the repository must be in the artifact.
    ///
    /// `hooks/preinstall` is generated by `package` itself and so cannot be forgotten. Anything
    /// else under `hooks/` is an ordinary `--include`, which is precisely the kind of list that
    /// has now drifted twice — units, then the binaries they exec. `hooks/postinstall` installs
    /// the release's systemd units, so a release shipping without it silently returns to needing
    /// a manual step on every board.
    #[test]
    fn every_hook_in_the_repo_is_packaged() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        let hooks = std::fs::read_dir(root.join("hooks")).expect("hooks/ must exist");
        for entry in hooks.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            // `.in` files are templates; `package` renders and appends them itself.
            if name.ends_with(".in") {
                continue;
            }

            for workflow in PACKAGING_SITES {
                let text = std::fs::read_to_string(root.join(workflow))
                    .unwrap_or_else(|e| panic!("{workflow}: {e}"));
                let expected = format!("=hooks/{name}");
                assert!(
                    text.contains(&expected),
                    "{workflow} does not package hooks/{name}. Add:  \
                     --include \"hooks/{name}=hooks/{name}\""
                );
            }
        }
    }

    /// Every binary a packaged unit tries to exec must be staged into the artifact.
    ///
    /// The sibling of the test above, and the case it missed. The units were packaged and the
    /// binaries were not, so `btd.service` failed with `203/EXEC` — systemd could not execute
    /// `/opt/robot/daemon/current/bin/btd` because the release did not contain it. That reads on
    /// the board as a broken daemon rather than as an incomplete artifact, and it cost a second
    /// install cycle to find.
    ///
    /// Derived from the units rather than from a list kept by hand: each unit names its binary in
    /// `ExecStart`, so adding a service and forgetting to stage it fails here. A hand-kept list
    /// would have exactly the drift this exists to catch.
    #[test]
    fn every_binary_a_packaged_unit_execs_is_staged() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        for workflow in PACKAGING_SITES {
            let text = std::fs::read_to_string(root.join(workflow))
                .unwrap_or_else(|e| panic!("{workflow}: {e}"));

            // The units this workflow packages, as `<crate>/systemd/<unit>=systemd/<unit>`.
            for line in text.lines().filter(|l| l.contains("=systemd/")) {
                let Some(src) = line
                    .split('"')
                    .nth(1)
                    .and_then(|pair| pair.split('=').next())
                else {
                    continue;
                };
                if !src.ends_with(".service") {
                    continue;
                }

                let unit = std::fs::read_to_string(root.join(src)).unwrap_or_else(|e| {
                    panic!("{workflow} packages {src}, which does not exist: {e}")
                });

                // `ExecStart=/opt/robot/daemon/current/bin/<name> [args]`
                let Some(exec_path) = unit
                    .lines()
                    .find(|l| l.starts_with("ExecStart="))
                    .and_then(|l| l.split_whitespace().next())
                    .and_then(|l| l.strip_prefix("ExecStart="))
                else {
                    panic!("{src} has no ExecStart naming a binary");
                };

                // A unit that execs out of the *base* rather than the release, which the boot
                // recovery net does on purpose: it runs when the release cannot, so reading its
                // program through `current` would route the recovery through the thing being
                // recovered. Nothing to stage, and `xtask/tests/artifact.rs` checks that the
                // script it names is packaged and installed.
                if !exec_path.starts_with(RELEASE_BIN_DIR) {
                    continue;
                }

                let exec = exec_path
                    .rsplit('/')
                    .next()
                    .unwrap_or_else(|| panic!("{src}: ExecStart={exec_path} names nothing"));

                // The staged names, by basename of each `cp … staged/` line. Not
                // `contains("release/<exec> staged/")`: `dev-push.sh` builds in one of two
                // directories depending on the toolchain, so it names the source through a
                // variable, and a check keyed to a literal path would have quietly stopped
                // looking at the site that changes most often.
                let staged: Vec<&str> = text
                    .lines()
                    .map(str::trim)
                    .filter(|l| l.starts_with("cp ") && l.ends_with(" staged/"))
                    .filter_map(|l| l.trim_end_matches(" staged/").rsplit('/').next())
                    .collect();
                assert!(
                    staged.contains(&exec),
                    "{workflow} packages {src}, whose ExecStart is {exec:?}, but never stages \
                     that binary — it stages {staged:?}. Without it the unit fails on the board \
                     with 203/EXEC. Add:  cp <build dir>/{exec} staged/"
                );
            }
        }
    }

    /// `scripts/setup-board.sh` is fetched standalone with `curl`, so it cannot read
    /// Cargo.toml and has to carry a literal version. This is what stops that literal
    /// drifting from the value the preinstall hook is generated with — the exact failure that
    /// left 1.20.1 on a board against an `ort` that requires 1.23 and panics below it.
    #[test]
    fn setup_board_pins_the_same_onnx_target() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let manifest: toml::Value =
            toml::from_str(&std::fs::read_to_string(root.join("Cargo.toml")).unwrap()).unwrap();
        let target = manifest["workspace"]["metadata"]["onnxruntime"]["target"]
            .as_str()
            .unwrap();

        let script = std::fs::read_to_string(root.join("scripts/setup-board.sh")).unwrap();
        let expected = format!("ONNX_VERSION=\"${{ONNX_VERSION:-{target}}}\"");
        assert!(
            script.contains(&expected),
            "setup-board.sh must pin ONNX_VERSION to {target}; expected the line {expected:?}"
        );
    }

    /// `setup-rkaiq.sh` builds an LD_PRELOAD shim from a C file beside it, so the C file has to
    /// be packaged too.
    ///
    /// Not covered by `every_script_the_hooks_run_is_packaged`, which watches `script=scripts/…`
    /// assignments in the hooks: the shim is not a script anything runs, it is a source file the
    /// script compiles. Packaged without it, the engine is installed and then segfaults on this
    /// kernel — which looks like a broken camera rather than a missing file, and the script says
    /// so and stops rather than guessing.
    #[test]
    fn the_rkaiq_shim_travels_with_its_script() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        const SHIM: &str = "scripts/rkaiq-modinfo-shim.c";
        assert!(root.join(SHIM).exists(), "{SHIM} is missing");

        let script = std::fs::read_to_string(root.join("scripts/setup-rkaiq.sh")).unwrap();
        assert!(
            script.contains("rkaiq-modinfo-shim.c"),
            "setup-rkaiq.sh must name the shim source it builds"
        );

        for workflow in PACKAGING_SITES {
            let text = std::fs::read_to_string(root.join(workflow))
                .unwrap_or_else(|e| panic!("{workflow}: {e}"));
            assert!(
                text.contains(&format!("={SHIM}")),
                "{workflow} packages setup-rkaiq.sh but not {SHIM}, which it cannot run without"
            );
        }
    }

    /// `setup-npu.sh` compiles a device-tree overlay from a .dts beside it, so the .dts has to be
    /// packaged too.
    ///
    /// The same shape as `the_rkaiq_shim_travels_with_its_script`, and not covered by
    /// `every_script_the_hooks_run_is_packaged` for the same reason: the overlay is not a script
    /// anything runs, it is a source the script compiles. Packaged without it, the hook installs
    /// the runtime, cannot find the .dts, and leaves the NPU node disabled — so the detector runs
    /// on the CPU for ever and the log line saying why is one warning in an update that succeeded.
    #[test]
    fn the_npu_overlay_travels_with_its_script() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        const OVERLAY: &str = "deploy/overlays/rk3568-npu-enable.dts";
        assert!(root.join(OVERLAY).exists(), "{OVERLAY} is missing");

        let script = std::fs::read_to_string(root.join("scripts/setup-npu.sh")).unwrap();
        assert!(
            script.contains("rk3568-npu-enable.dts"),
            "setup-npu.sh must name the overlay source it compiles"
        );

        for workflow in PACKAGING_SITES {
            let text = std::fs::read_to_string(root.join(workflow))
                .unwrap_or_else(|e| panic!("{workflow}: {e}"));
            assert!(
                text.contains(&format!("={OVERLAY}")),
                "{workflow} packages setup-npu.sh but not {OVERLAY}, which it cannot run without"
            );
        }
    }

    /// `setup-npu.sh` pins the NPU runtime, and Cargo.toml pins it too.
    ///
    /// Third instance of the same trap — after ONNX Runtime and the GStreamer plugins — and the
    /// same fix: the script is fetched standalone with `curl` and cannot read Cargo.toml, so it
    /// carries a literal and this asserts the two agree. A runtime older than the model it is asked
    /// to load fails at `rknn_init` with a number, which is not a diagnosis.
    #[test]
    fn setup_npu_pins_the_same_runtime() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let manifest: toml::Value =
            toml::from_str(&std::fs::read_to_string(root.join("Cargo.toml")).unwrap()).unwrap();
        let pinned = manifest["workspace"]["metadata"]["rknpu"]["runtime"]
            .as_str()
            .unwrap();

        let script = std::fs::read_to_string(root.join("scripts/setup-npu.sh")).unwrap();
        let expected = format!("RUNTIME=\"{pinned}\"");
        assert!(
            script.contains(&expected),
            "setup-npu.sh must carry the line {expected:?}"
        );
    }

    /// Same trap, same shape: `scripts/setup-gstreamer.sh` is fetched standalone with `curl`, so
    /// it carries a literal plugin version and cannot read Cargo.toml. A drift here is a board
    /// running plugins nobody can name — which is exactly what building them ourselves, from
    /// pinned sources, was for.
    #[test]
    fn setup_gstreamer_pins_the_same_plugin_version() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let manifest: toml::Value =
            toml::from_str(&std::fs::read_to_string(root.join("Cargo.toml")).unwrap()).unwrap();
        let meta = &manifest["workspace"]["metadata"]["gst-plugins"];
        let version = meta["version"].as_str().unwrap();
        let repo = meta["repo"].as_str().unwrap();

        let script = std::fs::read_to_string(root.join("scripts/setup-gstreamer.sh")).unwrap();
        for expected in [
            format!("PLUGINS_VERSION=\"${{PLUGINS_VERSION:-{version}}}\""),
            format!("PLUGINS_REPO=\"${{PLUGINS_REPO:-{repo}}}\""),
        ] {
            assert!(
                script.contains(&expected),
                "setup-gstreamer.sh must carry the line {expected:?}"
            );
        }
    }

    /// The shipped hook must be fully substituted. A placeholder reaching a board would be
    /// compared against a version number and silently fail every board the same way.
    #[test]
    fn the_preinstall_template_renders_completely() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        let template = std::fs::read_to_string(root.join("hooks/preinstall.in")).unwrap();
        assert!(
            template.contains("@ONNX_FLOOR@") && template.contains("@ONNX_TARGET@"),
            "the template should carry both placeholders"
        );

        let rendered = template
            .replace("@ONNX_FLOOR@", "1.23")
            .replace("@ONNX_TARGET@", "1.28.0");
        assert!(
            !rendered.contains("@ONNX_"),
            "nothing may remain unsubstituted"
        );
        assert!(rendered.contains("ONNX_FLOOR=\"1.23\""));
        assert!(rendered.contains("ONNX_TARGET=\"1.28.0\""));
    }

    /// `board-test.sh` hands its whole container script to `sh -c` inside **one single-quoted
    /// string**, so a single quote anywhere in it ends that string early.
    ///
    /// Both ways this fails are quiet. An apostrophe in a comment — "the oneshot's job" — leaves the
    /// file syntactically broken, which at least fails loudly. Worse is a quoted argument:
    /// `grep -q '^\[Install\]'` arrives at the container as `grep -q ^\[Install\]`, and the shell
    /// there strips the backslashes, so grep is handed `^[Install]` — a bracket expression matching
    /// one character from `I n s t a l`. It runs, it exits 0 or 1 for the wrong reason, and the
    /// assertion built on it reports something that was never checked. That is how this test came to
    /// exist, and finding it took a CI round trip and a while.
    ///
    /// Comments are *not* exempt, unlike the check below: the shell does not know it is reading one.
    #[test]
    fn the_board_test_container_script_contains_no_single_quotes() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");
        let script = std::fs::read_to_string(root.join("scripts/board-test.sh"))
            .expect("scripts/board-test.sh must exist");

        // The container script is assigned as `CHECKS='` … `'` at the start of a line.
        let (_, rest) = script
            .split_once("\nCHECKS='")
            .expect("board-test.sh no longer assigns CHECKS with a single-quoted string");
        let (checks, _) = rest
            .split_once("\n'\n")
            .expect("the CHECKS string is no longer closed by a lone quote on its own line");

        let offenders: Vec<(usize, &str)> = checks
            .lines()
            .enumerate()
            .filter(|(_, line)| line.contains('\''))
            .map(|(i, line)| (i + 1, line.trim()))
            .collect();

        assert!(
            offenders.is_empty(),
            "single quotes inside the CHECKS string end it early. Use double quotes for grep \
             patterns (\"^\\\\[Install\\\\]\" survives; '^\\\\[Install\\\\]' does not) and reword \
             any apostrophe. Offending lines, numbered from the start of CHECKS: {offenders:#?}"
        );
    }

    /// Advice the provisioning scripts print must be runnable from where the operator is
    /// standing, which is their home directory and not wherever the file was downloaded to.
    ///
    /// `setup-board.sh` told people to run `sudo sh migrate-network.sh` — a bare relative name
    /// for a sibling script that a fresh board has not fetched at all. Both halves of that were
    /// wrong, and neither is the kind of thing anyone re-reads once their own board works.
    /// Comment lines are exempt: explaining the trap requires quoting it.
    ///
    /// It catches literals only. A `sh $VAR` holding a relative path passes, because the value
    /// is not knowable here — so this narrows the failure rather than closing it, and the
    /// paths those variables hold are declared at the top of each script for that reason.
    #[test]
    fn printed_commands_name_absolute_paths() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
        for name in [
            "setup-board.sh",
            "migrate-network.sh",
            "setup-gstreamer.sh",
            "setup-rkaiq.sh",
            "install.sh",
            "provision.sh",
            "provision-board.sh",
        ] {
            let script = std::fs::read_to_string(root.join("scripts").join(name)).unwrap();
            for (n, line) in script.lines().enumerate() {
                if line.trim_start().starts_with('#') {
                    continue;
                }
                for after in line.split("sh ").skip(1) {
                    let target = after.split_whitespace().next().unwrap_or_default();
                    // Only file-looking targets matter: `sudo sh` with nothing after it is the
                    // documented pipe form, and `$0`/`${VAR}` resolve at runtime.
                    if !target.ends_with(".sh") && !target.ends_with(".sh\"") {
                        continue;
                    }
                    assert!(
                        target.starts_with('/') || target.starts_with('$'),
                        "{name}:{} tells the operator to run {target:?}, which only works \
                         from the directory that happens to hold it",
                        n + 1
                    );
                }
            }
        }
    }
}
