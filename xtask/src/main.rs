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
            // Hooks must be executable; everything else needn't be.
            let mode = if dest.starts_with("hooks/") {
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

        for workflow in ["dev.yml", "release.yml"] {
            let text = std::fs::read_to_string(root.join(".github/workflows").join(workflow))
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
        let yml = std::fs::read_to_string(root.join(".github/workflows/promote.yml"))
            .expect(".github/workflows/promote.yml must exist");

        assert!(
            yml.contains("--stable-tag"),
            "promote.yml must pass --stable-tag, or the manifest url is built from the \
             wrong release"
        );
        assert!(
            yml.contains("\"artifact/$artifact_name\""),
            "promote.yml must upload the artifact to the stable release — the manifest's \
             url points there"
        );
        assert!(
            yml.contains("\"artifact/$artifact_name.minisig\""),
            "promote.yml must upload the artifact signature too — `sig_url` is derived \
             from `url` and points at the same release"
        );

        // Retiring staging is only safe because of the two uploads above. If someone
        // removes them, this assertion is the one that should look wrong.
        assert!(
            yml.contains("gh release delete \"$staging_tag\""),
            "promote.yml should retire the staging release once stable is self-contained"
        );
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

            for workflow in ["dev.yml", "release.yml"] {
                let text = std::fs::read_to_string(root.join(".github/workflows").join(workflow))
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

        for workflow in ["dev.yml", "release.yml"] {
            let text = std::fs::read_to_string(root.join(".github/workflows").join(workflow))
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
                let Some(exec) = unit
                    .lines()
                    .find(|l| l.starts_with("ExecStart="))
                    .and_then(|l| l.split_whitespace().next())
                    .and_then(|l| l.rsplit('/').next())
                else {
                    panic!("{src} has no ExecStart naming a binary");
                };

                let staged = format!("release/{exec} staged/");
                assert!(
                    text.contains(&staged),
                    "{workflow} packages {src}, whose ExecStart is {exec:?}, but never stages \
                     that binary. Without it the unit fails on the board with 203/EXEC. Add:  \
                     cp target/aarch64-unknown-linux-gnu/release/{exec} staged/"
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
