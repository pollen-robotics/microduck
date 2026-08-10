//! Package a release the way CI does, then open the tarball and check what is inside it.
//!
//! Every other packaging test in this repository asserts that two *source files* agree —
//! `.github/workflows/*.yml` against `scripts/install.sh`, or a unit's `ExecStart` against a `cp`
//! line. Those are worth having and they are the weaker form: they describe packaging without
//! observing it, so they pass whenever the description is self-consistent, including when the
//! description is wrong. `docs/install-path-gap.md` option A is this file.
//!
//! Two bugs reached a board through that gap on the same afternoon. A release shipped units its
//! artifact did not contain, and then — two commits after a test was added to stop the first one
//! recurring — binaries its units tried to exec. `btd.service` failed with `203/EXEC`, which reads
//! on a board as a broken daemon rather than as an incomplete release.
//!
//! What this adds over the string-matching tests, verified by breaking each one and watching the
//! old suite stay green:
//!
//! - **a binary no unit execs.** The existing check derives its list from `ExecStart` lines, so it
//!   structurally cannot see `robotctl` — which nothing execs and `install.sh` symlinks onto the
//!   operator's `PATH`. Dropping it from the workflow's staging passes all six existing tests and
//!   yields an artifact where `/usr/local/bin/robotctl` points at nothing;
//! - **hook mode.** A `hooks/postinstall` shipped without its executable bit fails inside the
//!   update gate, unattended, on every board. Nothing else in the tree looks at a mode;
//! - **the generated hook.** `hooks/preinstall` is rendered by `package` rather than passed as an
//!   `--include`, and `every_hook_in_the_repo_is_packaged` skips `.in` templates — so no test
//!   checked that the rendered hook ships;
//! - **`package` running at all.** The `src=dest` split, the mode assignment, the version-drift
//!   guard, the `binaries` list written into `version.toml`: all of it was reachable only by
//!   cutting a release.
//!
//! What it does *not* add: a typo in an `--include` dest is already caught, because
//! `every_unit_install_sh_expects_is_packaged` matches the exact expected `=systemd/<unit>` string.
//! The overlap is deliberate — these run in 0.4s together and the failure messages point at
//! different things.
//!
//! The inputs still come from the workflow YAML, because those files *are* the production
//! packaging recipe — reproducing the list here by hand would recreate exactly the drift these
//! tests exist to catch. What is no longer taken on trust is the result.
//!
//! **Both workflows, not just `release.yml`.** `dev.yml` maintains its own copy of the staging and
//! `--include` lists, and says in a comment why they must match:
//!
//! > Same contents as a real release: a dev build that omitted the units or robotd would fail its
//! > own restart step on the board and roll itself back, which is a confusing way to learn that the
//! > packaging differed.
//!
//! Nothing enforced that. Two hand-maintained parallel lists is the same drift class as a list that
//! disagrees with `install.sh`, and it matters more than it looks: every bug in
//! `install-path-gap.md` was found while installing a **dev** build onto a board, because that is
//! the artifact a branch actually ships. A test that covered only `release.yml` would have watched
//! all four of them go past.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Where in the release CI stages the binaries it is about to package.
const STAGED: &str = " staged/";

/// Every workflow that packages an artifact a board can install.
///
/// `dev.yml` first, deliberately: it is the one that runs on every push and the one whose output
/// reaches a board during development.
const PACKAGING_WORKFLOWS: [&str; 2] = ["dev.yml", "release.yml"];

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask/ has a parent")
        .to_path_buf()
}

/// The binaries a workflow copies into the staging directory, by name.
fn staged_binaries(workflow: &str) -> Vec<String> {
    workflow
        .lines()
        .map(str::trim)
        .filter(|l| l.starts_with("cp ") && l.ends_with(STAGED))
        .filter_map(|l| l.trim_end_matches(STAGED).rsplit('/').next())
        .map(str::to_owned)
        .collect()
}

/// The `--include src=dest` pairs a workflow passes, in order.
fn includes(workflow: &str) -> Vec<(String, String)> {
    workflow
        .lines()
        .filter(|l| l.contains("--include"))
        .filter_map(|l| l.split('"').nth(1))
        .filter_map(|pair| pair.split_once('='))
        .map(|(src, dest)| (src.to_owned(), dest.to_owned()))
        .collect()
}

/// Build the artifact a named workflow would build, and return its entries as
/// `path -> (mode, bytes)`.
///
/// Stub binaries rather than real ones: this is about what the packager places where, and
/// cross-compiling four aarch64 daemons to assert the presence of `bin/robotd` would make the test
/// too slow to keep. Their *names* are the load-bearing part, and those come from the workflow.
fn packaged_release(workflow_name: &str) -> BTreeMap<String, (u32, Vec<u8>)> {
    let root = root();
    let workflow = std::fs::read_to_string(root.join(".github/workflows").join(workflow_name))
        .unwrap_or_else(|e| panic!("{workflow_name}: {e}"));

    let binaries = staged_binaries(&workflow);
    assert!(
        binaries.len() >= 4,
        "{workflow_name} should stage several binaries, found {binaries:?} — the parsing above \
         has probably stopped matching the workflow's `cp` lines, which would make every \
         assertion in this file vacuous"
    );

    // Read the channel rather than rely on `package`'s default. A workflow that changed it would
    // otherwise leave this test packaging something production does not.
    let channel = workflow
        .lines()
        .map(str::trim)
        .find_map(|l| l.strip_prefix("--channel "))
        .map(|rest| rest.trim_end_matches(" \\").trim())
        .unwrap_or("daemon");

    let scratch = tempfile::tempdir().expect("tempdir");
    let bin_dir = scratch.path().join("staged");
    std::fs::create_dir_all(&bin_dir).expect("staging dir");
    for name in &binaries {
        std::fs::write(bin_dir.join(name), b"#!/bin/false\n").expect("stub binary");
    }

    // The real workspace version, so `package`'s version-drift guard is exercised rather than
    // waved past with `--allow-version-drift`.
    let manifest: toml::Value =
        toml::from_str(&std::fs::read_to_string(root.join("Cargo.toml")).expect("Cargo.toml"))
            .expect("parse Cargo.toml");
    let version = manifest["workspace"]["package"]["version"]
        .as_str()
        .expect("workspace version");

    let out = scratch.path().join("dist");
    let mut cmd = std::process::Command::new(env!("CARGO_BIN_EXE_xtask"));
    // `package` resolves `--include` sources and `hooks/preinstall.in` relative to the working
    // directory, exactly as the workflow does from the repository root.
    cmd.current_dir(&root)
        .arg("package")
        .arg("--version")
        .arg(version)
        .arg("--channel")
        .arg(channel)
        .arg("--bin-dir")
        .arg(&bin_dir)
        .arg("--out")
        .arg(&out)
        // Level 1: the artifact is read once and thrown away. At the shipping default of 19 this
        // test would dominate the suite, which is how a test stops being run.
        .arg("--zstd-level")
        .arg("1");
    for (src, dest) in includes(&workflow) {
        cmd.arg("--include").arg(format!("{src}={dest}"));
    }

    let output = cmd.output().expect("run xtask package");
    assert!(
        output.status.success(),
        "xtask package failed for {workflow_name}: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let artifact = out.join(format!("{channel}-{version}.tar.zst"));
    let file = std::fs::File::open(&artifact)
        .unwrap_or_else(|e| panic!("package reported success but wrote no {artifact:?}: {e}"));
    let decoder = zstd::Decoder::new(file).expect("zstd frame");

    let mut entries = BTreeMap::new();
    for entry in tar::Archive::new(decoder).entries().expect("tar entries") {
        let mut entry = entry.expect("tar entry");
        let path = entry.path().expect("entry path").display().to_string();
        let mode = entry.header().mode().expect("entry mode");
        let mut bytes = Vec::new();
        std::io::Read::read_to_end(&mut entry, &mut bytes).expect("entry body");
        entries.insert(path, (mode, bytes));
    }
    entries
}

/// The artifact must contain what `scripts/install.sh` reaches for on a bare board.
///
/// Each assertion names a line of that script rather than a general principle, because the
/// script is the consumer and its failure modes are what this is defending.
#[test]
fn the_artifact_carries_what_install_sh_reads() {
    for workflow in PACKAGING_WORKFLOWS {
        let entries = packaged_release(workflow);

        // install.sh dies outright without these two: "a robot without it has nothing to run".
        for unit in ["systemd/updaterd.service", "systemd/robotd.service"] {
            assert!(
                entries.contains_key(unit),
                "{workflow}'s artifact has no {unit}, which install.sh refuses to proceed \
                 without. Contents: {:?}",
                entries.keys().collect::<Vec<_>>()
            );
        }

        // `ln -sfn .../current/bin/robotctl /usr/local/bin/robotctl` — a dangling symlink
        // otherwise, and the operator's only tool.
        assert!(
            entries.contains_key("bin/robotctl"),
            "{workflow}'s artifact has no bin/robotctl, so install.sh would symlink \
             /usr/local/bin/robotctl at nothing"
        );

        // install.sh globs `current/systemd/*.service` rather than asserting a list, so a unit
        // landing anywhere else is silently not installed.
        let units: Vec<&String> = entries
            .keys()
            .filter(|p| p.starts_with("systemd/") && p.ends_with(".service"))
            .collect();
        assert!(
            units.len() >= 4,
            "{workflow} should ship several units directly under systemd/, found {units:?}"
        );
    }
}

/// Every unit in the artifact must be able to exec its binary out of the same artifact.
///
/// This is bug 3, asserted against the tarball instead of against the workflow that builds it.
/// The unit files were packaged and the binaries were not, so `btd.service` failed with
/// `203/EXEC` — systemd could not execute `current/bin/btd`, because the release did not contain
/// it. Derived from the units rather than from a list kept by hand: adding a service and
/// forgetting to stage its binary fails here.
#[test]
fn every_unit_in_the_artifact_can_exec_its_binary_from_it() {
    for workflow in PACKAGING_WORKFLOWS {
        let entries = packaged_release(workflow);

        for (path, (_, bytes)) in entries
            .iter()
            .filter(|(p, _)| p.starts_with("systemd/") && p.ends_with(".service"))
        {
            let unit = String::from_utf8_lossy(bytes);
            let exec = unit
                .lines()
                .find(|l| l.starts_with("ExecStart="))
                .and_then(|l| l.split_whitespace().next())
                .and_then(|l| l.rsplit('/').next())
                .unwrap_or_else(|| panic!("{path} has no ExecStart naming a binary"));

            assert!(
                entries.contains_key(&format!("bin/{exec}")),
                "{workflow} packages {path}, which execs {exec:?} — not in the artifact. On a \
                 board that is 203/EXEC, and it reads as a broken daemon rather than an \
                 incomplete release. Stage it in {workflow}:  \
                 cp target/aarch64-unknown-linux-gnu/release/{exec} staged/"
            );
        }
    }
}

/// Hooks must arrive executable, and `preinstall` must arrive at all.
///
/// The engine spawns these directly; a hook without its executable bit fails the update, inside
/// the gate, on every board, with nobody watching. `hooks/preinstall` is generated by `package`
/// from its template rather than passed as an `--include`, so its presence is a property of the
/// packager and is worth pinning where the packager actually runs.
#[test]
fn hooks_are_packaged_executable() {
    for workflow in PACKAGING_WORKFLOWS {
        let entries = packaged_release(workflow);

        let hooks: Vec<&String> = entries.keys().filter(|p| p.starts_with("hooks/")).collect();
        assert!(
            hooks.iter().any(|p| p.as_str() == "hooks/preinstall"),
            "{workflow}: package generates hooks/preinstall from hooks/preinstall.in; it is \
             missing. Found {hooks:?}"
        );
        assert!(
            hooks.iter().any(|p| p.as_str() == "hooks/postinstall"),
            "{workflow}: hooks/postinstall installs the release's units — without it a release \
             that adds a daemon silently returns to needing a manual step on every board. \
             Found {hooks:?}"
        );

        for hook in &hooks {
            let (mode, _) = &entries[hook.as_str()];
            assert_eq!(
                mode & 0o111,
                0o111,
                "{workflow} packages {hook} with mode {mode:o}; the engine execs it directly"
            );
        }
    }
}

/// Everything the workflow says it includes must be in the artifact at the path it names.
///
/// The blunt form of bug 2. Weaker than its siblings above and kept because it covers the files
/// they say nothing about — the policies, the journald drop-in, the shipped docs — where a silently
/// dropped `--include` has no unit and no `ExecStart` to give it away.
#[test]
fn every_include_lands_where_the_workflow_says() {
    for name in PACKAGING_WORKFLOWS {
        let workflow = std::fs::read_to_string(root().join(".github/workflows").join(name))
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let entries = packaged_release(name);

        for (src, dest) in includes(&workflow) {
            assert!(
                entries.contains_key(&dest),
                "{name} includes {src:?} as {dest:?}, which is not in the artifact"
            );
        }
    }
}
