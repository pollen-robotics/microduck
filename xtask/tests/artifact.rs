//! Package a release the way CI does, then open the tarball and check what is inside it.
//!
//! Every other packaging test in this repository asserts that two *source files* agree —
//! `.github/workflows/*.yml` against `scripts/install.sh`, or a unit's `ExecStart` against a `cp`
//! line. Those are worth having and they are the weaker form: they describe packaging without
//! observing it, so they pass whenever the description is self-consistent, including when the
//! description is wrong. `docs/project/install-path-gap.md` option A is this file.
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
//! **Both workflows.** `dev.yml` maintains its own copy of the staging and
//! `--include` lists, and says in a comment why they must match:
//!
//! > Same contents as a real release: a dev build that omitted the units or robotd would fail its
//! > own restart step on the board and roll itself back, which is a confusing way to learn that the
//! > packaging differed.
//!
//! Nothing enforced that. Two hand-maintained parallel lists is the same drift class as a list that
//! disagrees with `install.sh`, and it matters more than it looks: every bug in
//! `install-path-gap.md` was found while installing a **dev** build onto a board, because that is
//! the artifact a branch actually ships. A test that covered only the release path would have
//! watched all four of them go past.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Where in the release CI stages the binaries it is about to package.
const STAGED: &str = " staged/";

/// Every file that packages an artifact a board can install, by repository path.
///
/// `dev.yml` first, deliberately: it is the one that runs on every push and the one whose output
/// reaches a board during development.
/// `_build-release.yml` rather than `release.yml`: the recipe moved into the reusable workflow that
/// both the staging and the stable path call, and `release.yml` is now only the entry point that
/// decides between them. A constant that kept naming the old file would have left every assertion
/// here vacuous — which is why the parse below fails loudly when it matches nothing.
///
/// `scripts/dev-push.sh` is the third, and it is not a workflow — it is the laptop-to-board path,
/// which assembles the same artifact from the same lists so that what a developer runs on a board
/// is what a release would put there. Being a shell script changes nothing here: the parsers below
/// read `cp … staged/` and `--include "src=dest"` lines, and the script writes them in the same
/// literal form CI does precisely so this file covers it. Three hand-maintained copies is a worse
/// drift risk than the two this file was written for, and the copy that drifts unnoticed is
/// whichever one nobody is cutting a release from that week.
const PACKAGING_SITES: [&str; 3] = [
    ".github/workflows/dev.yml",
    ".github/workflows/_build-release.yml",
    "scripts/dev-push.sh",
];

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

/// The text of one packaging site.
fn recipe(site: &str) -> String {
    std::fs::read_to_string(root().join(site)).unwrap_or_else(|e| panic!("{site}: {e}"))
}

/// Build the artifact a named workflow would build, and return its entries as
/// `path -> (mode, bytes)`.
///
/// Stub binaries rather than real ones: this is about what the packager places where, and
/// cross-compiling four aarch64 daemons to assert the presence of `bin/robotd` would make the test
/// too slow to keep. Their *names* are the load-bearing part, and those come from the workflow.
fn packaged_release(site: &str) -> BTreeMap<String, (u32, Vec<u8>)> {
    let root = root();
    let workflow = recipe(site);

    let binaries = staged_binaries(&workflow);
    assert!(
        binaries.len() >= 4,
        "{site} should stage several binaries, found {binaries:?} — the parsing above \
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
        "xtask package failed for {site}: {}",
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
    for workflow in PACKAGING_SITES {
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

        // `install -m 755 .../current/scripts/robot-rescue /usr/local/sbin/robot-rescue`, which
        // `hooks/postinstall` repeats on every update. Executable in the artifact as well as at the
        // destination: a board where nothing else works is a board where someone runs it in place.
        let (mode, _) = entries.get("scripts/robot-rescue").unwrap_or_else(|| {
            panic!(
                "{workflow}'s artifact has no scripts/robot-rescue, so a board whose release \
                 cannot start has no recovery path that does not go through updaterd"
            )
        });
        assert_eq!(
            mode & 0o111,
            0o111,
            "{workflow} packages scripts/robot-rescue with mode {mode:o}; an operator runs it \
             out of the release directory"
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

/// Every unit in the artifact must be able to exec what it names, out of wherever it names it.
///
/// This is bug 3, asserted against the tarball instead of against the workflow that builds it.
/// The unit files were packaged and the binaries were not, so `btd.service` failed with
/// `203/EXEC` — systemd could not execute `current/bin/btd`, because the release did not contain
/// it. Derived from the units rather than from a list kept by hand: adding a service and
/// forgetting to stage its binary fails here.
///
/// Two destinations, because the boot recovery net execs out of `/usr/local/sbin` on purpose — it
/// runs when the release cannot, so reading its program through `current` would route the recovery
/// through the thing being recovered. That path is checked too, against the `scripts/` entry the
/// installers copy there, so an `ExecStart` pointing at neither still fails.
#[test]
fn every_unit_in_the_artifact_can_exec_what_it_names() {
    for workflow in PACKAGING_SITES {
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
                .and_then(|l| l.strip_prefix("ExecStart="))
                .unwrap_or_else(|| panic!("{path} has no ExecStart naming a program"));
            let name = exec.rsplit('/').next().expect("a path has a last segment");

            let expected = match exec.strip_prefix("/opt/robot/daemon/current/bin/") {
                Some(_) => format!("bin/{name}"),
                None => {
                    assert_eq!(
                        exec,
                        format!("/usr/local/sbin/{name}"),
                        "{workflow} packages {path}, whose ExecStart is {exec:?} — neither in the \
                         release ({}) nor in the base (/usr/local/sbin/). Nothing installs a \
                         program there, so the unit is 203/EXEC on the board",
                        "/opt/robot/daemon/current/bin/"
                    );
                    format!("scripts/{name}")
                }
            };

            assert!(
                entries.contains_key(&expected),
                "{workflow} packages {path}, which execs {exec:?}, but the artifact has no \
                 {expected}. On a board that is 203/EXEC, and it reads as a broken daemon rather \
                 than an incomplete release"
            );
        }
    }
}

/// The recovery net has to arrive whole, and the two halves land differently.
///
/// A timer with no oneshot never runs anything; a oneshot with no timer is never triggered; and
/// either script missing from the artifact leaves a unit pointing at `/usr/local/sbin` with nothing
/// installed there. Each of those is silent until a board needs the recovery it does not have.
#[test]
fn the_boot_recovery_net_is_packaged_whole() {
    for workflow in PACKAGING_SITES {
        let entries = packaged_release(workflow);

        for path in [
            "systemd/robot-boot-check.timer",
            "systemd/robot-boot-check.service",
            "scripts/robot-boot-check",
            "scripts/robot-rescue",
        ] {
            assert!(
                entries.contains_key(path),
                "{workflow}'s artifact has no {path}. Contents: {:?}",
                entries.keys().collect::<Vec<_>>()
            );
        }

        // The oneshot must carry no `[Install]`. Both installers `enable --now` every unit that has
        // one, and doing that here runs a rollback check in the middle of the update that installed
        // it — with daemons legitimately mid-restart, which is what it reads as a broken release.
        let (_, oneshot) = &entries["systemd/robot-boot-check.service"];
        let oneshot = String::from_utf8_lossy(oneshot);
        // A section header at the start of a line, which is what systemd parses and what
        // `hooks/postinstall` greps for. Matching the word anywhere would fire on the comment in
        // the unit that explains why the section is absent.
        assert!(
            !oneshot.lines().any(|l| l.starts_with("[Install]")),
            "robot-boot-check.service has an [Install] section, so the installers will enable and \
             start it during an update rather than leaving it to its timer"
        );

        let (mode, _) = &entries["scripts/robot-boot-check"];
        assert_eq!(mode & 0o111, 0o111, "the check ships without its exec bit");
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
    for workflow in PACKAGING_SITES {
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
    for name in PACKAGING_SITES {
        let workflow = recipe(name);
        let entries = packaged_release(name);

        for (src, dest) in includes(&workflow) {
            assert!(
                entries.contains_key(&dest),
                "{name} includes {src:?} as {dest:?}, which is not in the artifact"
            );
        }
    }
}

/// Package `binaries` (stubs, created in the order given) and return the artifact's bytes.
fn package_stubs(binaries: &[&str]) -> Vec<u8> {
    let root = root();
    let manifest: toml::Value =
        toml::from_str(&std::fs::read_to_string(root.join("Cargo.toml")).expect("Cargo.toml"))
            .expect("parse Cargo.toml");
    let version = manifest["workspace"]["package"]["version"]
        .as_str()
        .expect("workspace version");

    let scratch = tempfile::tempdir().expect("tempdir");
    let bin_dir = scratch.path().join("staged");
    std::fs::create_dir_all(&bin_dir).expect("staging dir");
    for name in binaries {
        std::fs::write(bin_dir.join(name), format!("#!/bin/false {name}\n")).expect("stub");
    }
    let out = scratch.path().join("dist");
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_xtask"))
        .current_dir(&root)
        .args([
            "package",
            "--version",
            version,
            "--channel",
            "daemon",
            "--zstd-level",
            "1",
        ])
        .arg("--bin-dir")
        .arg(&bin_dir)
        .arg("--out")
        .arg(&out)
        .output()
        .expect("run xtask package");
    assert!(
        output.status.success(),
        "xtask package failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    std::fs::read(out.join(format!("daemon-{version}.tar.zst"))).expect("artifact")
}

/// The binaries go into the archive in a fixed order, so the same inputs make the same archive.
///
/// `updater-design.md` §16 promises that — fixed mtimes, "so a rebuild can be compared against
/// what shipped" — but the binaries were appended in `read_dir` order, which is the filesystem's:
/// hash order on ext4, seeded per filesystem, so two CI runners packing identical binaries got two
/// different sha256s. Asserted on the entry order rather than by packing twice, because two
/// directories on one filesystem usually enumerate alike and that comparison would pass on the
/// bug it is meant to catch.
#[test]
fn binaries_are_packaged_in_a_fixed_order() {
    let names = [
        "zeta", "robotd", "alpha", "updaterd", "mid", "btd", "nfcd", "configd",
    ];
    let bytes = package_stubs(&names);

    let decoder = zstd::Decoder::new(bytes.as_slice()).expect("zstd frame");
    let order: Vec<String> = tar::Archive::new(decoder)
        .entries()
        .expect("tar entries")
        .map(|e| {
            e.expect("tar entry")
                .path()
                .expect("path")
                .display()
                .to_string()
        })
        .filter(|p| p.starts_with("bin/"))
        .collect();

    let mut sorted = order.clone();
    sorted.sort();
    assert_eq!(order.len(), names.len(), "{order:?}");
    assert_eq!(order, sorted, "binaries packed in directory order");

    // And the property itself: created in the opposite order, the same inputs, the same bytes.
    let mut reversed = names;
    reversed.reverse();
    assert_eq!(
        package_stubs(&reversed),
        bytes,
        "same inputs, different archive"
    );
}
