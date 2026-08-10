//! Per-robot configuration.
//!
//! The engine is generic; everything robot-specific lives here. Adapting to a
//! different robot should mean a new config file, new signing keys, and possibly
//! a new health probe — not engine changes. See `docs/design/updater-design.md` §10.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Duration;

use serde::{Deserialize, Serialize};

/// Root of `/etc/robot/updater.toml`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Directory of trusted minisign public keys. A signature is valid if it
    /// verifies against *any* key in here.
    ///
    /// A set rather than one key so a lost or compromised key is survivable —
    /// see `docs/design/updater-design.md` §5.4.
    pub trusted_keys_dir: PathBuf,

    /// Single forward-compatibility guard. An artifact declaring a higher
    /// `min_hw_rev` than this is refused.
    ///
    /// v1 targets one hardware configuration, so this is deliberately one
    /// integer and not a capability matrix (`docs/design/updater-design.md` §5.6).
    pub hw_rev: u32,

    /// Engine-owned state: lock file, update log, boot counter. Must NOT be
    /// inside any component's `install_dir` — it has to survive every swap and
    /// rollback (`docs/design/updater-design.md` §5.7).
    pub state_dir: PathBuf,

    /// Where `robotd` listens. Used by every `health = { probe = "socket" }` component
    /// and by the pre-restart `safeToRestart` query.
    ///
    /// Absent or silent is a normal state, not an error (`docs/design/architecture.md` §1.1):
    /// `robotd` may legitimately be stopped, crashed, or not yet installed.
    #[serde(default = "default_robot_socket")]
    pub robot_socket: PathBuf,

    /// Accept artifacts signed with a key marked dev-only. Off in production.
    /// See `docs/design/updater-design.md` §15.
    #[serde(default)]
    pub allow_dev_keys: bool,

    /// Permit `--inject-fault`. Off in production; a client robot must not be able
    /// to be told to fail on purpose. See [`crate::faults`].
    #[serde(default)]
    pub allow_fault_injection: bool,

    /// Largest uncompressed size an artifact may expand to, in bytes.
    ///
    /// Configurable because a model bundle of several ONNX policies is legitimately
    /// far larger than a daemon binary; a global ceiling would reject a real release.
    #[serde(default)]
    pub max_uncompressed_bytes: Option<u64>,

    /// Largest number of entries an artifact archive may contain.
    #[serde(default)]
    pub max_archive_entries: Option<usize>,

    /// How often to check each component's source for a new release.
    ///
    /// `None` disables periodic checking entirely, which also disables the only path
    /// that makes `min_supported` effective (§8.1) — a robot nobody taps update on
    /// never learns a floor exists.
    #[serde(default, with = "humantime_serde::option")]
    pub check_interval: Option<Duration>,

    /// Which updates the periodic check may apply with **no client attached**.
    ///
    /// Inert without [`Self::check_interval`]: the scheduler is the only thing that
    /// applies unattended, so a policy with no timer to run it does nothing.
    #[serde(default)]
    pub auto_apply: AutoApply,

    /// Extra uids permitted to perform **mutating** operations over the IPC socket.
    ///
    /// The uid `updaterd` itself runs as is always permitted — it could replace the
    /// daemon anyway, so denying it would be theatre. Everyone else needs listing
    /// here (or in [`Self::allow_gids`]) to apply, roll back, select or pin.
    ///
    /// Read-only requests (`status`, `log`, `listInstalled`, `check`, `subscribe`) are
    /// **not** gated by this: reaching the socket at all already requires its group
    /// (mode 0660), and support needs to be able to look at a robot without being
    /// authorised to change it.
    #[serde(default)]
    pub allow_uids: Vec<u32>,

    /// Groups permitted to perform mutating operations, by gid.
    #[serde(default)]
    pub allow_gids: Vec<u32>,

    /// Users permitted to perform mutating operations, **by name**.
    ///
    /// Preferred over [`Self::allow_uids`] for anything shipped, because
    /// `systemd-sysusers` allocates uids dynamically: a number that is correct on the
    /// board a config was written for is wrong on the next one. `btd` is listed here so
    /// the app can trigger an update, which is what M6 exists to deliver.
    ///
    /// A name that does not resolve is a warning, not a fatal error. A robot missing an
    /// optional service must still serve status and logs.
    #[serde(default)]
    pub allow_users: Vec<String>,

    /// Groups permitted to perform mutating operations, by name.
    ///
    /// Deliberately empty in the shipped config, and a test enforces that `robot` never
    /// appears here: membership of `robot` is what gets a process as far as *talking* to
    /// updaterd, and listing it would collapse that layer into the change-authority one.
    #[serde(default)]
    pub allow_groups: Vec<String>,

    /// Defaulted so a config with no components reaches [`Config::validate`] and
    /// gets a clear message, rather than a bare serde "missing field".
    #[serde(rename = "component", default)]
    pub components: BTreeMap<String, ComponentConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentConfig {
    pub source: SourceConfig,

    /// Root under which releases are staged and linked.
    pub install_dir: PathBuf,

    pub on_apply: ApplyAction,

    #[serde(default)]
    pub health: HealthCheck,

    /// Retained previous releases, for rollback. The golden release is kept
    /// independently of this count.
    #[serde(default = "default_keep_previous")]
    pub keep_previous: usize,

    /// Never-pruned known-good release (`docs/design/updater-design.md` §8.2).
    #[serde(default)]
    pub golden: Option<semver::Version>,

    /// Refuse anything but this version. Set by `robotctl pin`.
    #[serde(default)]
    pub pinned: Option<semver::Version>,
}

fn default_keep_previous() -> usize {
    1
}

/// Which updates the periodic check may apply with no client attached.
///
/// One ordered setting rather than a boolean per urgency. Two booleans would let a config
/// say "apply ordinary updates automatically but not mandatory ones" — auto-updating
/// everything *except* the releases published specifically to rescue a broken fleet. That
/// combination has no legitimate use and is not worth being able to write down.
///
/// Whatever the policy, an unattended apply is still an ordinary apply: it runs the same
/// preflight, so a robot that is walking or streaming refuses and retries at the next
/// interval rather than restarting under someone's hands (`preflight.rs`), and the same
/// health gate, so a release that does not come up is rolled back.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AutoApply {
    /// Never apply without a client. Availability is still logged, and a mandatory
    /// release is logged loudly — an ignored one is worth shouting about.
    Off,

    /// Only a release whose manifest declares the running version below `min_supported`.
    ///
    /// The default, and the remediation path for "we shipped a bad release": robots pull
    /// themselves forward instead of waiting for someone to open the app (§8.1). Ordinary
    /// releases still wait for a client, because when a robot restarts is the owner's
    /// decision.
    #[default]
    Mandatory,

    /// Any available release.
    ///
    /// For canary and bench robots — §16.2's Tier 2 wants lab robots that track
    /// `staging` and update on every candidate. On a client robot this takes the
    /// "when does my robot restart" decision away from its owner, which is the decision
    /// the whole app-driven update flow exists to give them.
    All,
}

impl AutoApply {
    /// May the scheduler apply a candidate of this urgency unattended?
    pub fn permits(self, mandatory: bool) -> bool {
        match self {
            AutoApply::Off => false,
            AutoApply::Mandatory => mandatory,
            AutoApply::All => true,
        }
    }
}

/// Where artifacts come from. Per-component because the daemon lives on GitHub
/// Releases and models on HF Hub (`docs/design/updater-design.md` §5.1).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SourceConfig {
    GithubReleases {
        /// `ORG/REPO`.
        repo: String,
        /// Tag prefix identifying the channel, e.g. `daemon-v`.
        tag_prefix: String,
        /// Release asset holding the signed manifest.
        #[serde(default = "default_manifest_asset")]
        manifest_asset: String,
        /// Tag prefix for per-branch dev builds, so `--ref my-branch` resolves to
        /// `daemon-dev-my-branch`.
        ///
        /// Separate from `tag_prefix` because the two streams must not be confusable: a
        /// dev tag *moves* (it points at whatever that branch built last) while a release
        /// tag is immutable, and `newest_version` must never consider a dev tag when
        /// resolving `latest` for the fleet.
        #[serde(default = "default_ref_tag_prefix")]
        ref_tag_prefix: String,
        /// Tag prefix for release candidates, so `--staging` resolves to
        /// `daemon-staging-v<version>`.
        ///
        /// A third prefix rather than a flag on `tag_prefix`, for the reason the second one
        /// exists: the streams must not be confusable. A candidate is flagged as a prerelease
        /// on GitHub precisely so `newest_version` cannot reach it, and giving that scan an
        /// "unless…" would put the fleet one config typo away from tracking candidates.
        #[serde(default = "default_staging_tag_prefix")]
        staging_tag_prefix: String,
    },
    HfHub {
        /// `ORG/MODEL`.
        repo: String,
        /// Branch, tag, or commit. A moving branch means "latest".
        revision: String,
        #[serde(default = "default_manifest_asset")]
        manifest_file: String,
    },
    /// A local directory. Not a production source — this is what makes the
    /// engine testable against the real code path with no network, and backs
    /// the dev sideload flow (`docs/design/updater-design.md` §16.1).
    LocalDir { path: PathBuf },
}

/// Default prefix for dev-build tags.
///
/// Names `daemon` because that is this robot's only source-backed component today; a robot
/// with a differently-named channel sets it explicitly, the same as `tag_prefix`.
fn default_ref_tag_prefix() -> String {
    "daemon-dev-".to_owned()
}

/// Default prefix for release-candidate tags, matching what `release.yml` pushes.
fn default_staging_tag_prefix() -> String {
    "daemon-staging-v".to_owned()
}

fn default_manifest_asset() -> String {
    "manifest.json".to_owned()
}

/// What to do once the new release is linked.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum ApplyAction {
    /// Nothing to do (the consumer picks it up on its own).
    None,
    /// Full restart. Drops motor control briefly — only for the daemon channel.
    Restart { units: Vec<String> },
    /// Signal in place, no restart. Used for models so a weights swap doesn't
    /// interrupt motor control (`docs/design/updater-design.md` §5.5).
    Reload { unit: String, signal: String },
}

/// How to decide whether the new release is good.
///
/// This gate is what makes auto-rollback meaningful, so a weak probe here
/// undermines the whole design — see `docs/design/updater-design.md` §8.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(tag = "probe", rename_all = "snake_case")]
pub enum HealthCheck {
    /// No gate: commit as soon as apply returns. Only acceptable for components
    /// that cannot break the robot.
    #[default]
    None,
    /// Ask `robotd` over its unix socket and wait for it to report healthy.
    ///
    /// No path here on purpose. The socket is one robot-wide fact, not a per-component
    /// one, so it lives in [`Config::robot_socket`].
    Socket {
        #[serde(with = "humantime_serde")]
        timeout: Duration,
    },
    /// Run a command; exit status 0 means healthy. Escape hatch for probes that
    /// don't fit the socket model.
    Command {
        program: PathBuf,
        #[serde(default)]
        args: Vec<String>,
        #[serde(with = "humantime_serde")]
        timeout: Duration,
    },
}

impl HealthCheck {
    pub fn timeout(&self) -> Option<Duration> {
        match self {
            HealthCheck::None => None,
            HealthCheck::Socket { timeout, .. } | HealthCheck::Command { timeout, .. } => {
                Some(*timeout)
            }
        }
    }
}

fn default_robot_socket() -> PathBuf {
    PathBuf::from("/run/robotd.sock")
}

impl Config {
    /// Parse from TOML text. Always validated — an invalid config must not be
    /// constructible.
    pub fn from_toml(text: &str) -> Result<Self, crate::Error> {
        let config: Self = toml::from_str(text).map_err(|e| crate::Error::Config(e.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    pub fn load(path: &std::path::Path) -> Result<Self, crate::Error> {
        let text = std::fs::read_to_string(path).map_err(|e| crate::Error::Io {
            path: path.to_path_buf(),
            source: e,
        })?;
        Self::from_toml(&text)
    }

    /// Bounds applied when extracting an artifact.
    pub fn archive_limits(&self) -> crate::verify::ArchiveLimits {
        let defaults = crate::verify::ArchiveLimits::default();
        crate::verify::ArchiveLimits {
            max_uncompressed_bytes: self
                .max_uncompressed_bytes
                .unwrap_or(defaults.max_uncompressed_bytes),
            max_entries: self.max_archive_entries.unwrap_or(defaults.max_entries),
        }
    }

    pub fn component(&self, id: &str) -> Result<&ComponentConfig, crate::Error> {
        self.components
            .get(id)
            .ok_or_else(|| crate::Error::UnknownComponent(id.to_owned()))
    }

    /// Reject self-inconsistent configs at load time rather than mid-update.
    ///
    /// Each of these would otherwise surface as data loss or a mysterious failure
    /// during an update, which is the worst time to discover them.
    pub fn validate(&self) -> Result<(), crate::Error> {
        let bad = |msg: String| Err(crate::Error::Config(msg));

        if self.components.is_empty() {
            return bad("no components configured; nothing could ever be updated".into());
        }

        for (name, component) in &self.components {
            // A relative install_dir would resolve against the daemon's cwd,
            // which systemd does not guarantee.
            if !component.install_dir.is_absolute() {
                return bad(format!(
                    "component {name}: install_dir must be absolute, got {}",
                    component.install_dir.display()
                ));
            }

            // The decisive one: engine state inside a release tree would be
            // destroyed by the very swap or rollback it exists to record.
            if self.state_dir.starts_with(&component.install_dir) {
                return bad(format!(
                    "state_dir {} is inside component {name}'s install_dir {} — a swap or \
                     rollback would destroy the update log and boot counter",
                    self.state_dir.display(),
                    component.install_dir.display()
                ));
            }

            // Two components sharing a tree would prune each other's releases.
            for (other_name, other) in &self.components {
                if other_name != name && component.install_dir == other.install_dir {
                    return bad(format!(
                        "components {name} and {other_name} share install_dir {} — they would \
                         prune each other's releases",
                        component.install_dir.display()
                    ));
                }
            }

            // Golden exists to be a rollback target that is never pruned; a
            // keep_previous of 0 leaves nothing else to fall back to.
            if component.keep_previous == 0 && component.golden.is_none() {
                return bad(format!(
                    "component {name}: keep_previous = 0 with no golden release leaves no \
                     rollback target"
                ));
            }
        }

        if !self.state_dir.is_absolute() {
            return bad(format!(
                "state_dir must be absolute, got {}",
                self.state_dir.display()
            ));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The example config shipped in the repo must actually parse. Catches drift
    /// between docs and code.
    #[test]
    fn example_config_parses() {
        let text = include_str!("../updater.example.toml");
        let config = Config::from_toml(text).expect("example config must be valid");

        let daemon = config.component("daemon").unwrap();

        // The bootstrap state is over: robotd exists, so the example config gates for
        // real. These assertions run in the other direction now — they catch a
        // *regression* to the inert values, which would silently disable auto-rollback
        // and look like nothing at all in a diff.
        let ApplyAction::Restart { units } = &daemon.on_apply else {
            panic!(
                "daemon on_apply must restart robotd, not {:?}",
                daemon.on_apply
            );
        };
        assert!(
            units.contains(&"robotd".to_string()),
            "must restart robotd: {units:?}"
        );
        // updaterd must never restart itself (it would die mid-swap) and must not
        // restart btd (it would drop the app's progress connection).
        assert!(
            !units.iter().any(|u| u == "updaterd" || u == "btd"),
            "updaterd must not restart itself or btd: {units:?}"
        );
        assert!(
            matches!(daemon.health, HealthCheck::Socket { .. }),
            "daemon must have a real health gate, got {:?}",
            daemon.health
        );

        // One component per model, each independently versioned (§5.5).
        assert!(config.component("model-walk").is_ok());
        assert!(config.component("model-jump").is_ok());
    }

    /// **Everything `on_apply` restarts must actually ship.**
    ///
    /// "What the daemon artifact contains" is stated in three places — this config, each
    /// unit's `ExecStart`, and the release workflows' copy lists — and nothing else compares
    /// them. An artifact missing a binary installs cleanly and then fails its own restart
    /// step, which rolls the release back on every robot, with the cause three files away
    /// from the symptom.
    ///
    /// The workflows are checked by string search rather than by parsing YAML: one assertion
    /// does not justify a YAML dependency, and the strings searched for are exactly the ones
    /// that must not go missing.
    #[test]
    fn every_unit_on_apply_restarts_is_actually_shipped() {
        let text = include_str!("../updater.example.toml");
        let config = Config::from_toml(text).unwrap();
        let daemon = config.component("daemon").unwrap();

        let ApplyAction::Restart { units } = &daemon.on_apply else {
            panic!("expected a restart action; the other assertions cover that");
        };

        // updater/ -> repo root
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("updater/ has a parent");
        // Both workflows that build an artifact. A dev build missing `robotd` fails on the
        // board in exactly the same way as a release missing it — `systemctl restart robotd`
        // with no such unit — and a teammate hitting that would have no reason to suspect the
        // packaging rather than their own branch.
        //
        // `_build-release.yml`, not `release.yml`: the packaging recipe lives in the reusable
        // workflow that both the staging and the stable path call, and `release.yml` is now only the
        // entry point choosing between them. The assertion below fails loudly on a file it cannot
        // parse, which is what caught this rename rather than silently passing.
        let workflows = [
            ".github/workflows/_build-release.yml",
            ".github/workflows/dev.yml",
        ]
        .map(|w| {
            (
                w,
                std::fs::read_to_string(repo.join(w)).unwrap_or_else(|_| panic!("{w} must exist")),
            )
        });
        let workspace = std::fs::read_to_string(repo.join("Cargo.toml")).unwrap();

        for unit in units {
            // A crate of that name must exist in the workspace, or there is no binary to
            // restart.
            assert!(
                workspace.contains(&format!("\"{unit}\"")),
                "on_apply restarts `{unit}` but no such workspace member exists"
            );

            // A unit file must exist in the repo...
            let unit_file = repo
                .join(unit)
                .join("systemd")
                .join(format!("{unit}.service"));
            assert!(
                unit_file.exists(),
                "on_apply restarts `{unit}` but {} does not exist",
                unit_file.display()
            );

            // ...and every workflow that builds an artifact must ship both the binary and
            // that unit file. Without these two lines the release installs successfully and
            // then fails its own restart step.
            for (name, workflow) in &workflows {
                assert!(
                    workflow.contains(&format!("release/{unit} staged/")),
                    "{name} does not copy the `{unit}` binary into the artifact"
                );
                assert!(
                    workflow.contains(&format!("{unit}/systemd/{unit}.service=")),
                    "{name} does not include `{unit}.service` in the artifact"
                );
            }
        }
    }

    /// `deploy/updater.toml` is what a client robot actually runs, so its safety-relevant
    /// values are asserted rather than reviewed.
    ///
    /// Every one of these is a single word or `true`/`false` away from being wrong in a way
    /// no diff makes obvious: a robot that trusts dev keys, one that can be told to fail on
    /// purpose, one that never polls and so can never be pulled off a withdrawn release, or
    /// one whose update gate does not gate. All four look fine and behave fine right up to
    /// the moment they matter.
    #[test]
    fn shipped_config_is_safe_for_a_client_robot() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("updater/ has a parent");
        let text = std::fs::read_to_string(repo.join("deploy/updater.toml"))
            .expect("deploy/updater.toml must exist — scripts/install.sh installs it");
        let config = Config::from_toml(&text).expect("the shipped config must be valid");

        assert!(
            !config.allow_dev_keys,
            "a client robot must not trust dev keys: it would install anything a teammate builds"
        );
        assert!(
            !config.allow_fault_injection,
            "a client robot must not accept --inject-fault"
        );
        assert!(
            config.check_interval.is_some(),
            "without a check interval `min_supported` is inert, so a withdrawn release \
             cannot be remediated on a robot nobody opens the app for"
        );
        // Numeric ids never belong in a shipped config: sysusers allocates dynamically, so a
        // number that is right on one board is wrong on the next. Names are resolved at startup.
        assert!(
            config.allow_uids.is_empty() && config.allow_gids.is_empty(),
            "the shipped config must grant by name, not by number: {:?} / {:?}",
            config.allow_uids,
            config.allow_gids
        );

        // The layering this whole design rests on. Naming a specific *service* is a narrow
        // claim — "btd may relay an update request from the app". Naming the `robot` group is
        // not: membership of it is what gets a process as far as *talking* to updaterd, so
        // listing it here would collapse the socket-access and change-authority layers into
        // one, and anything that could read status could replace the firmware.
        assert!(
            !config.allow_groups.iter().any(|g| g == "robot"),
            "the robot group must not have change authority: {:?}",
            config.allow_groups
        );

        // btd is expected: an update the owner starts from their phone is M6's headline, and
        // without this it returns PERMISSION_DENIED.
        assert!(
            config.allow_users.iter().any(|u| u == "btd"),
            "btd must be able to relay an update request from the app: {:?}",
            config.allow_users
        );
        assert_ne!(
            config.auto_apply,
            AutoApply::All,
            "a client robot must not install every release unattended: when it restarts is \
             its owner's decision, which is what the app-driven flow exists to give them. \
             `all` is the canary and bench setting."
        );

        let daemon = config
            .component("daemon")
            .expect("the daemon component must exist");
        let ApplyAction::Restart { units } = &daemon.on_apply else {
            panic!(
                "daemon on_apply must restart robotd, not {:?}",
                daemon.on_apply
            );
        };
        assert!(
            units.contains(&"robotd".to_string()),
            "must restart robotd: {units:?}"
        );
        assert!(
            !units.iter().any(|u| u == "updaterd" || u == "btd"),
            "updaterd must not restart itself or btd: {units:?}"
        );
        assert!(
            matches!(daemon.health, HealthCheck::Socket { .. }),
            "the shipped config must have a real health gate, got {:?}",
            daemon.health
        );

        // The stable channel, not staging. A robot on `daemon-staging-v` would install
        // every candidate build.
        let SourceConfig::GithubReleases {
            tag_prefix,
            staging_tag_prefix,
            ..
        } = &daemon.source
        else {
            panic!(
                "the shipped daemon source must be github_releases, got {:?}",
                daemon.source
            );
        };
        assert_eq!(
            tag_prefix, "daemon-v",
            "the shipped config must track the stable channel"
        );
        // The shipped config names no staging prefix, so `--staging` on a customer robot
        // depends on this default matching what `release.yml` actually pushes. A wrong
        // default would fail with "no releases with tag prefix", which reads as "there is no
        // candidate" rather than "this board is looking in the wrong place".
        assert_eq!(
            staging_tag_prefix, "daemon-staging-v",
            "the default candidate prefix must match the tag release.yml pushes"
        );

        // Only components that have somewhere real to fetch from. A component whose
        // source 404s makes the periodic check report a failure for something nobody has
        // shipped, which teaches whoever reads robot status to ignore failures.
        assert_eq!(
            config.components.keys().collect::<Vec<_>>(),
            vec!["daemon"],
            "the shipped config should carry only components whose source exists"
        );
    }

    /// The placeholder in `deploy/updater.toml` must be recognisable as one.
    ///
    /// `scripts/install.sh` refuses to install a config still containing `ORG/`, so that a
    /// robot cannot be provisioned pointing at a repository that does not exist — which
    /// would install fine and then never find another update. This test exists so that
    /// substituting the real repo does not silently break that guard by, say, leaving the
    /// literal in a comment where the script would still match it.
    #[test]
    fn installer_guard_and_shipped_config_agree_about_the_placeholder() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap();
        let config = std::fs::read_to_string(repo.join("deploy/updater.toml")).unwrap();
        let script = std::fs::read_to_string(repo.join("scripts/install.sh"))
            .expect("scripts/install.sh must exist");

        let placeholder_present = config.contains("ORG/");
        assert!(
            script.contains("ORG/"),
            "install.sh must keep checking for the placeholder while one can still appear"
        );
        if placeholder_present {
            assert!(
                config.contains("repo           = \"ORG/"),
                "the placeholder should be the `repo` value, not stray text the guard would \
                 also match"
            );
        }
    }

    /// The truth table, stated once. Every unattended-apply decision routes through this.
    #[test]
    fn auto_apply_permits_the_right_urgencies() {
        assert!(
            !AutoApply::Off.permits(true),
            "off must ignore even mandatory"
        );
        assert!(!AutoApply::Off.permits(false));

        assert!(AutoApply::Mandatory.permits(true));
        assert!(
            !AutoApply::Mandatory.permits(false),
            "the default must not install ordinary releases behind the owner's back"
        );

        assert!(AutoApply::All.permits(true), "all must include mandatory");
        assert!(AutoApply::All.permits(false));
    }

    /// The default has to be `mandatory`: a config that omits the field still needs to
    /// remediate a withdrawn release, and `off` would leave the fleet stuck on it.
    #[test]
    fn auto_apply_defaults_to_mandatory_and_parses_each_variant() {
        let base = r#"
            trusted_keys_dir = "/etc/robot/keys"
            hw_rev = 1
            state_dir = "/var/lib/robot/updater"
            [component.daemon]
            install_dir = "/opt/robot/daemon"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
        "#;
        assert_eq!(
            Config::from_toml(base).unwrap().auto_apply,
            AutoApply::Mandatory
        );

        for (text, expected) in [
            ("off", AutoApply::Off),
            ("mandatory", AutoApply::Mandatory),
            ("all", AutoApply::All),
        ] {
            let config = Config::from_toml(&format!("auto_apply = \"{text}\"\n{base}")).unwrap();
            assert_eq!(config.auto_apply, expected, "parsing {text:?}");
        }

        // A typo must be refused rather than silently defaulted. `auto_apply = "always"`
        // reading as `mandatory` would look like the setting took effect and quietly not.
        assert!(
            Config::from_toml(&format!("auto_apply = \"always\"\n{base}")).is_err(),
            "an unrecognised policy must be a config error"
        );

        // The old boolean must not linger anywhere: `deny_unknown_fields` means a config
        // still carrying it fails loudly instead of being read as "off".
        assert!(
            Config::from_toml(&format!("auto_apply_mandatory = true\n{base}")).is_err(),
            "the superseded field must be rejected, not ignored"
        );
    }

    #[test]
    fn health_timeout_accepts_humantime() {
        let config = Config::from_toml(
            r#"
            trusted_keys_dir = "/etc/robot/keys"
            hw_rev = 1
            state_dir = "/var/lib/robot/updater"
            [component.daemon]
            install_dir = "/opt/robot/daemon"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
            health = { probe = "socket", timeout = "45s" }
            "#,
        )
        .unwrap();

        assert_eq!(
            config.component("daemon").unwrap().health.timeout(),
            Some(Duration::from_secs(45))
        );
    }

    fn config_with(extra_component: &str) -> Result<Config, crate::Error> {
        Config::from_toml(&format!(
            r#"
            trusted_keys_dir = "/etc/robot/keys"
            hw_rev = 1
            state_dir = "/var/lib/robot/updater"
            {extra_component}
            "#
        ))
    }

    #[test]
    fn rejects_state_dir_inside_install_dir() {
        let err = Config::from_toml(
            r#"
            trusted_keys_dir = "/etc/robot/keys"
            hw_rev = 1
            state_dir = "/opt/robot/daemon/state"
            [component.daemon]
            install_dir = "/opt/robot/daemon"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
            "#,
        )
        .unwrap_err();
        assert!(
            err.to_string().contains("would destroy the update log"),
            "got: {err}"
        );
    }

    #[test]
    fn rejects_shared_install_dir() {
        let err = config_with(
            r#"
            [component.daemon]
            install_dir = "/opt/robot/shared"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
            [component.model]
            install_dir = "/opt/robot/shared"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
            "#,
        )
        .unwrap_err();
        assert!(err.to_string().contains("prune each other"), "got: {err}");
    }

    #[test]
    fn rejects_relative_install_dir() {
        let err = config_with(
            r#"
            [component.daemon]
            install_dir = "opt/robot/daemon"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
            "#,
        )
        .unwrap_err();
        assert!(err.to_string().contains("must be absolute"), "got: {err}");
    }

    #[test]
    fn rejects_no_rollback_target() {
        let err = config_with(
            r#"
            [component.daemon]
            install_dir = "/opt/robot/daemon"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            keep_previous = 0
            on_apply = { action = "none" }
            "#,
        )
        .unwrap_err();
        assert!(err.to_string().contains("no rollback target"), "got: {err}");
    }

    #[test]
    fn rejects_empty_components() {
        let err = Config::from_toml(
            r#"
            trusted_keys_dir = "/etc/robot/keys"
            hw_rev = 1
            state_dir = "/var/lib/robot/updater"
            "#,
        )
        .unwrap_err();
        assert!(err.to_string().contains("no components"), "got: {err}");
    }

    /// A typo in a key name must fail loudly rather than be silently ignored —
    /// a misspelled `keep_previous` would quietly prune the rollback target.
    #[test]
    fn rejects_unknown_fields() {
        let err = config_with(
            r#"
            [component.daemon]
            install_dir = "/opt/robot/daemon"
            source = { type = "local_dir", path = "/var/tmp/rel" }
            on_apply = { action = "none" }
            keep_previouss = 3
            "#,
        )
        .unwrap_err();
        assert!(matches!(err, crate::Error::Config(_)), "got: {err:?}");
    }
}
