//! `robotctl configure --allow-dev-daemon on|off` — make this robot a dev board, or stop it being one.
//!
//! A dev board installs what CI built for a branch (`robotctl update apply --ref <branch> daemon`).
//! Those builds are signed with the team dev key, which `updaterd` carries built in and uses only
//! when this robot allows it. Allowing it is one file: [`duck_ipc_proto::DEV_BUILDS_MARKER`] in
//! `updaterd`'s `state_dir`, which is under `/var/lib/robot` and so survives an A/B OS update.
//! `/etc/robot/updater.toml` is the image's on such a robot, and an edit there would not.
//!
//! `updaterd` reads the marker at startup, so a change here restarts it.
//! `docs/design/updater-design.md` §5.4 owns the mechanism.

use std::path::{Path, PathBuf};

/// Where a provisioned robot keeps `updaterd`'s config — read here for `state_dir` only.
pub const UPDATER_CONFIG: &str = "/etc/robot/updater.toml";

/// The marker path for the `updater.toml` at `config`.
pub fn marker_path(config: &Path) -> Result<PathBuf, String> {
    let text = std::fs::read_to_string(config)
        .map_err(|e| format!("cannot read {}: {e}", config.display()))?;
    let table: toml::Table =
        toml::from_str(&text).map_err(|e| format!("{}: {e}", config.display()))?;
    let state_dir = table
        .get("state_dir")
        .and_then(toml::Value::as_str)
        .ok_or_else(|| format!("{} sets no state_dir", config.display()))?;
    Ok(Path::new(state_dir).join(duck_ipc_proto::DEV_BUILDS_MARKER))
}

/// Write or remove the marker. Returns whether anything changed, so an `on` on a board that is
/// already a dev board does not restart `updaterd` for nothing.
pub fn set(config: &Path, on: bool) -> Result<bool, String> {
    let marker = marker_path(config)?;
    let present = marker.exists();
    if present == on {
        return Ok(false);
    }
    let result = if on {
        std::fs::write(
            &marker,
            "This robot is a dev board: updaterd accepts releases signed with the team dev key.\n\
             Written by `robotctl configure --allow-dev-daemon on`; `off` removes it.\n",
        )
    } else {
        std::fs::remove_file(&marker)
    };
    result.map_err(|e| {
        let hint = if e.kind() == std::io::ErrorKind::PermissionDenied {
            " — run it with sudo"
        } else {
            ""
        };
        format!("{}: {e}{hint}", marker.display())
    })?;
    Ok(true)
}

/// The command: set the marker, restart `updaterd` if it changed, say what to do next.
pub fn run(config: &Path, on: bool) -> Result<(), String> {
    let changed = set(config, on)?;
    let state = if on { "a dev board" } else { "not a dev board" };
    if !changed {
        println!("this robot is already {state} — nothing to do");
    } else {
        println!("this robot is now {state}; restarting updaterd…");
        crate::configure::restart_units(&["updaterd"])?;
        println!("updaterd restarted");
    }
    if on {
        println!(
            "\ninstall a branch build:  sudo robotctl update apply --ref <branch> daemon\n\
             without --ref or --version, `update apply` installs the latest stable release"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn board() -> (tempfile::TempDir, PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let state = dir.path().join("state");
        std::fs::create_dir(&state).unwrap();
        let config = dir.path().join("updater.toml");
        std::fs::write(
            &config,
            format!(
                "trusted_keys_dir = \"/etc/robot/trusted_keys\"\nstate_dir = \"{}\"\n\
                 allow_dev_keys = false\n",
                state.display()
            ),
        )
        .unwrap();
        (dir, config)
    }

    #[test]
    fn on_writes_the_marker_and_off_removes_it() {
        let (_dir, config) = board();
        let marker = marker_path(&config).unwrap();
        assert!(!marker.exists());

        assert!(set(&config, true).unwrap());
        assert!(marker.exists());
        assert!(
            !set(&config, true).unwrap(),
            "a second `on` changes nothing"
        );

        assert!(set(&config, false).unwrap());
        assert!(!marker.exists());
        assert!(
            !set(&config, false).unwrap(),
            "a second `off` changes nothing"
        );
    }

    #[test]
    fn a_config_without_state_dir_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let config = dir.path().join("updater.toml");
        std::fs::write(&config, "trusted_keys_dir = \"/x\"\n").unwrap();
        assert!(set(&config, true).unwrap_err().contains("state_dir"));
    }
}
