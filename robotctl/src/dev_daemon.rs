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

/// The switch as a row in `robotctl configure`'s editor: what the robot is now, and what the
/// operator has toggled it to and not yet saved.
///
/// Its own thing rather than a key in the editor's model, because that model is `robotd.toml`'s
/// and this is not in that file. It saves with the rest and adds `updaterd` to the restart offer.
#[derive(Debug)]
pub struct Switch {
    config: PathBuf,
    /// Whether this robot is a dev board right now.
    pub on: bool,
    /// What it has been toggled to, when that differs from [`Self::on`].
    pub pending: Option<bool>,
}

impl Switch {
    /// The name the editor shows.
    pub const NAME: &'static str = "allow_dev_daemon";

    /// The editor's one line of doc.
    pub const DOC: &'static str = "accept branch builds (update apply --ref <branch>) signed with \
         the team dev key · restarts updaterd · default off";

    /// The switch for the `updater.toml` at `config`, or `None` where there is no `updaterd` to
    /// configure — a laptop, a simulated duck — so the editor shows no row rather than an error.
    pub fn load(config: &Path) -> Option<Self> {
        let marker = marker_path(config).ok()?;
        Some(Self {
            config: config.to_path_buf(),
            on: marker.exists(),
            pending: None,
        })
    }

    /// The value the row shows: the toggled one, else the robot's.
    pub fn shown(&self) -> bool {
        self.pending.unwrap_or(self.on)
    }

    pub fn toggle(&mut self) {
        let next = !self.shown();
        self.pending = (next != self.on).then_some(next);
    }

    /// Back to the default, which is off.
    pub fn revert(&mut self) {
        self.pending = self.on.then_some(false);
    }

    /// Write the toggled value, if there is one. Returns whether anything was written.
    pub fn save(&mut self) -> Result<bool, String> {
        let Some(on) = self.pending else {
            return Ok(false);
        };
        let changed = set(&self.config, on)?;
        self.on = on;
        self.pending = None;
        Ok(changed)
    }
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
    fn the_editor_row_toggles_saves_and_reverts() {
        let (_dir, config) = board();
        let marker = marker_path(&config).unwrap();

        let mut switch = Switch::load(&config).unwrap();
        assert!(!switch.shown());
        switch.toggle();
        assert_eq!(switch.pending, Some(true));
        switch.toggle();
        assert_eq!(switch.pending, None, "toggled back is not a change");
        switch.toggle();
        assert!(switch.save().unwrap());
        assert!(marker.exists());
        assert!(switch.on && switch.pending.is_none());

        switch.revert();
        assert_eq!(switch.pending, Some(false));
        assert!(switch.save().unwrap());
        assert!(!marker.exists());
        switch.revert();
        assert_eq!(switch.pending, None, "already at the default");
        assert!(!switch.save().unwrap());
    }

    #[test]
    fn no_updater_config_means_no_row() {
        assert!(Switch::load(Path::new("/nonexistent/updater.toml")).is_none());
    }

    #[test]
    fn a_config_without_state_dir_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let config = dir.path().join("updater.toml");
        std::fs::write(&config, "trusted_keys_dir = \"/x\"\n").unwrap();
        assert!(set(&config, true).unwrap_err().contains("state_dir"));
    }
}
