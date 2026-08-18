//! `microduck_runtime`'s Bluetooth settings, as two files this can put on a board and take back off.
//!
//! ## What this is for, and why it is temporary
//!
//! Roughly half the boards built so far cannot keep an **Xbox Wireless Controller** bonded. The
//! pairing itself succeeds — `Pair()` returns, `Trusted` sticks, `padd` drives — and then every
//! reconnect dies:
//!
//! ```text
//! > HCI Event: Encryption Change — Status: PIN or Key Missing (0x06)
//! > HCI Event: Disconnect Complete — Reason: Authentication Failure (0x05)
//! ```
//!
//! flapping about 1.4 times a second. Boards built weeks apart, identical kernel, identical BlueZ,
//! identical `aic8800` firmware, identical HCI version — and one holds the pad while its twin does
//! not. It is also **not** a dead radio: a non-Xbox LE pad bonds on an affected board, stores keys,
//! and re-encrypts with `Encryption Change: Success`. So it is an interop problem with one make of
//! controller on some units, and it is under investigation.
//!
//! This module is what the affected boards run **in the meantime**. The end state is every board on
//! plain BLE with neither of these files set, so both halves are reversible and
//! [`crate::pad::Pads::status`] reports which boards still carry them.
//!
//! ## The two settings, and why one of them is probably inert
//!
//! Both come from `microduck_runtime`'s `install.sh`, which is the stack these robots ran before
//! this daemon existed and the one place an Xbox pad is known to have worked for a while.
//!
//!  - **`Privacy = device`** in `/etc/bluetooth/main.conf`. The runtime's notes credit it with
//!    fixing "an Xbox controller that pairs and then drops straight back out — an endless
//!    connect/disconnect loop", which is this symptom almost word for word.
//!  - **`options bluetooth disable_ertm=1`** in `/etc/modprobe.d/bluetooth.conf`. ERTM is an L2CAP
//!    **classic** feature, and every pad this robot has met is LE-only ([`crate::bluez`] has the
//!    bond dump), so on the hardware in the building this cannot be what is failing. It is applied
//!    anyway because it is free, because this repo sets it nowhere at all, and because "the
//!    runtime's settings" is a claim that should not quietly mean "one of them".
//!
//! ## The order is the whole trick
//!
//! `Privacy = device` **stops a pad bonding at all** on these boards. That is measured, not
//! inferred: LE Secure Connections pairing reaches the last step and the pad rejects it with `DHKey
//! check failed (0x0b)`, because the check is computed over both devices' addresses and privacy
//! pairs from a resolvable private one. `scripts/setup-board.sh` sets `Privacy = off` for exactly
//! that reason and says so at length.
//!
//! So the two facts are in genuine tension, and `setup-board.sh` already wrote down the way out:
//!
//! > *"If a pad ever does start dropping on connect, the two are in genuine tension and the answer
//! > is to pair with privacy off and then re-enable it — not to set `device` and lose the ability to
//! > pair at all."*
//!
//! That is what this implements. The bond is made under `Privacy = off`, where it can succeed; then
//! privacy goes on and the board reboots, and the pad reconnects to the bond it already has. Nothing
//! here may run before a pad is bonded, and `robotctl` is what enforces the sequence.
//!
//! ## Why a reboot rather than a restart
//!
//! bluetoothd reads `Privacy` at startup, so the setting needs it restarted — and restarting
//! `bluetooth.service` on this board leaves the kernel holding `hci0` while bluetoothd reports "No
//! default controller available", which only a reboot clears. `setup-board.sh` reached the same
//! conclusion for the same change. A reboot is therefore the honest instruction, not an extra step,
//! and it costs nothing here: the pad is bonded and trusted, so it comes back by itself.

use std::io::Write;
use std::path::{Path, PathBuf};

use duck_ipc_proto as proto;

/// BlueZ's own configuration. `Privacy` lives in its `[General]` section.
pub const MAIN_CONF: &str = "/etc/bluetooth/main.conf";

/// Where the runtime's installer puts the ERTM option, and therefore where this looks for it.
///
/// A board provisioned by that installer already has this exact path, so an affected board that was
/// once on the runtime reads as half-applied rather than as untouched — which is the truth.
pub const MODPROBE_CONF: &str = "/etc/modprobe.d/bluetooth.conf";

/// The live knob, for the running kernel. Writing it changes nothing already connected.
pub const ERTM_PARAM: &str = "/sys/module/bluetooth/parameters/disable_ertm";

/// The `modprobe.d` line, exactly as `microduck_runtime`'s installer writes it.
const ERTM_OPTION: &str = "options bluetooth disable_ertm=1";

/// The two files, addressable so the tests can point at a `tempdir` instead of at `/etc`.
pub struct PadFallback {
    main_conf: PathBuf,
    modprobe_conf: PathBuf,
    ertm_param: PathBuf,
}

impl Default for PadFallback {
    fn default() -> Self {
        Self::new()
    }
}

impl PadFallback {
    /// The real board.
    pub fn new() -> Self {
        Self::at(MAIN_CONF, MODPROBE_CONF, ERTM_PARAM)
    }

    pub fn at(
        main_conf: impl Into<PathBuf>,
        modprobe_conf: impl Into<PathBuf>,
        ertm_param: impl Into<PathBuf>,
    ) -> Self {
        Self {
            main_conf: main_conf.into(),
            modprobe_conf: modprobe_conf.into(),
            ertm_param: ertm_param.into(),
        }
    }

    /// What this board has right now.
    ///
    /// Never an error. An unreadable or absent file is reported as "that half is not applied", which
    /// is what it means for a board with no BlueZ config at all — a laptop running the test suite,
    /// most often — and asking about the settings must not fail on a machine that has none.
    pub fn state(&self) -> proto::PadFallback {
        proto::PadFallback {
            privacy_device: privacy_value(&read(&self.main_conf)).as_deref() == Some("device"),
            ertm_disabled: read(&self.modprobe_conf).lines().any(is_ertm_option),
        }
    }

    /// Apply both halves, or restore what `scripts/setup-board.sh` sets.
    ///
    /// Idempotent in both directions: `changed` is false when the board was already in the asked-for
    /// state, and a caller re-running this must not be told it did something.
    pub fn set(&self, enable: bool) -> Result<proto::PadFallbackResult, String> {
        let before = self.state();

        let privacy_changed = self.set_privacy(if enable { "device" } else { "off" })?;
        let ertm_changed = self.set_ertm(enable)?;

        // The live knob, best effort and last. It is not what `state` reads and it is not what
        // survives a reboot — `modprobe.d` is both — but setting it means a board that is somehow
        // never rebooted is at least not misreporting itself to `btmon`. A kernel that refuses the
        // write (no module, read-only param, not root) is not a failure of the change that matters.
        if let Err(e) = std::fs::write(&self.ertm_param, if enable { "Y\n" } else { "N\n" }) {
            tracing::debug!(error = %e, path = %self.ertm_param.display(), "could not set the live ERTM parameter");
        }

        let changed = privacy_changed || ertm_changed;
        let fallback = self.state();
        // Logged rather than returned: the caller asked for a state, not for a diff. But "this board
        // was already half-applied" is the case worth having in the journal, and it is the one an
        // affected board that once ran `microduck_runtime` will actually be in.
        tracing::info!(?before, ?fallback, enable, changed, "pad fallback set");
        Ok(proto::PadFallbackResult {
            fallback,
            changed,
            // Only when something moved. Telling someone to reboot a board that was already in the
            // state they asked for is how a reboot instruction stops being believed.
            reboot_required: changed,
        })
    }

    /// Write `Privacy = <value>` into `[General]`, wherever that section turns out to be.
    ///
    /// The same three cases `scripts/setup-board.sh` handles, and in the same order, because the
    /// boards this runs on were provisioned by that script and one of them wrote the line already:
    /// replace an existing `Privacy` line (commented or not), else insert under `[General]`, else
    /// append the section. Anything else in the file is left alone.
    fn set_privacy(&self, value: &str) -> Result<bool, String> {
        let existing = read(&self.main_conf);
        let current = privacy_value(&existing);
        if current.as_deref() == Some(value) {
            return Ok(false);
        }
        // An unset `Privacy` already *is* off — BlueZ defaults to it, which is why
        // `scripts/setup-board.sh` reports "not set (BlueZ defaults to off, which works)" rather
        // than correcting it. So taking the fallback off a board that never had the line writes
        // nothing, and does not claim a change that would send someone to reboot for it.
        if value == "off" && current.is_none() {
            return Ok(false);
        }

        let wanted = format!("Privacy = {value}");
        let mut lines: Vec<String> = existing.lines().map(str::to_owned).collect();

        if let Some(at) = lines.iter().position(|line| is_privacy_line(line)) {
            lines[at] = wanted;
        } else if let Some(at) = lines.iter().position(|line| line.trim() == "[General]") {
            lines.insert(at + 1, wanted);
        } else {
            // A blank line only when there is something to separate from. An empty file would
            // otherwise start with one, which is untidy in the one file someone reads while
            // debugging why a pad will not bond.
            if !lines.is_empty() {
                lines.push(String::new());
            }
            lines.push("[General]".to_owned());
            lines.push(wanted);
        }

        write_atomically(&self.main_conf, &(lines.join("\n") + "\n"))?;
        Ok(true)
    }

    /// Add or remove the ERTM option, leaving anything else in that file untouched.
    ///
    /// Removing takes the *line*, not the file: `/etc/modprobe.d/bluetooth.conf` is a shared name
    /// and something else may have an opinion in it. The file is deleted only once nothing but
    /// comments and blank lines is left, so a board this put the option on comes back clean.
    fn set_ertm(&self, enable: bool) -> Result<bool, String> {
        let existing = read(&self.modprobe_conf);
        let present = existing.lines().any(is_ertm_option);
        if present == enable {
            return Ok(false);
        }

        if enable {
            let mut body = existing;
            if !body.is_empty() && !body.ends_with('\n') {
                body.push('\n');
            }
            body.push_str(&format!(
                "# Temporary: some boards cannot keep an Xbox pad's LE bond. Set by\n\
                 # `robotctl pad fallback on`, removed by `robotctl pad fallback off`.\n\
                 {ERTM_OPTION}\n"
            ));
            write_atomically(&self.modprobe_conf, &body)?;
        } else {
            let kept: Vec<&str> = existing
                .lines()
                .filter(|line| !is_ertm_option(line))
                .collect();
            let anything_left = kept
                .iter()
                .any(|line| !line.trim().is_empty() && !line.trim_start().starts_with('#'));
            if anything_left {
                write_atomically(&self.modprobe_conf, &(kept.join("\n") + "\n"))?;
            } else if self.modprobe_conf.exists() {
                std::fs::remove_file(&self.modprobe_conf)
                    .map_err(|e| format!("cannot remove {}: {e}", self.modprobe_conf.display()))?;
            }
        }
        Ok(true)
    }
}

/// A file's contents, or empty for one that is not there or cannot be read.
fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_default()
}

/// Is this the `Privacy` setting, commented out or not?
fn is_privacy_line(line: &str) -> bool {
    let bare = line.trim_start().trim_start_matches('#').trim_start();
    bare.split('=')
        .next()
        .is_some_and(|key| key.trim().eq_ignore_ascii_case("Privacy"))
        && bare.contains('=')
}

/// The value of the first *active* `Privacy` line, lowercased. A commented-out one is not a setting.
fn privacy_value(conf: &str) -> Option<String> {
    conf.lines()
        .filter(|line| !line.trim_start().starts_with('#'))
        .filter(|line| is_privacy_line(line))
        .find_map(|line| line.split_once('='))
        .map(|(_, value)| value.trim().to_lowercase())
}

/// Is this an active line disabling ERTM? Whitespace varies; a commented one does not count.
fn is_ertm_option(line: &str) -> bool {
    let bare = line.trim();
    !bare.starts_with('#')
        && bare.split_whitespace().collect::<Vec<_>>()
            == ERTM_OPTION.split_whitespace().collect::<Vec<_>>()
}

/// Write, then rename over the target.
///
/// Not a nicety for `/etc/bluetooth/main.conf`: a truncated one is a board whose bluetoothd will not
/// start, on hardware nobody can attach a console to quickly, and the window for losing power on a
/// robot is not small. The temporary file is a sibling so the rename stays on one filesystem.
fn write_atomically(path: &Path, contents: &str) -> Result<(), String> {
    let parent = path
        .parent()
        .ok_or_else(|| format!("{} has no directory", path.display()))?;
    std::fs::create_dir_all(parent)
        .map_err(|e| format!("cannot create {}: {e}", parent.display()))?;

    let temporary = path.with_extension("configd-new");
    let mut file = std::fs::File::create(&temporary)
        .map_err(|e| format!("cannot write {}: {e}", temporary.display()))?;
    file.write_all(contents.as_bytes())
        .and_then(|()| file.sync_all())
        .map_err(|e| format!("cannot write {}: {e}", temporary.display()))?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        // Explicit, because a file created here would otherwise take the daemon's umask. Both of
        // these are read by processes that are not configd.
        let _ = std::fs::set_permissions(&temporary, std::fs::Permissions::from_mode(0o644));
    }

    std::fs::rename(&temporary, path).map_err(|e| format!("cannot replace {}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three files, under a directory that goes away with the test.
    fn board(dir: &Path) -> PadFallback {
        PadFallback::at(
            dir.join("bluetooth/main.conf"),
            dir.join("modprobe.d/bluetooth.conf"),
            dir.join("ertm"),
        )
    }

    fn main_conf(dir: &Path) -> String {
        read(&dir.join("bluetooth/main.conf"))
    }

    /// A board as `scripts/setup-board.sh` leaves it: privacy explicitly off, no modprobe file.
    fn provisioned(dir: &Path) {
        std::fs::create_dir_all(dir.join("bluetooth")).unwrap();
        std::fs::write(
            dir.join("bluetooth/main.conf"),
            "[General]\nPrivacy = off\nFastConnectable = true\n",
        )
        .unwrap();
    }

    /// On, then off, leaves the board exactly as provisioning left it — which is the whole point of
    /// this being reversible. The end state is every board on plain BLE.
    #[test]
    fn applying_and_removing_returns_the_board_to_privacy_off() {
        let dir = tempfile::tempdir().unwrap();
        provisioned(dir.path());
        let board = board(dir.path());
        assert!(board.state().is_off());

        let on = board.set(true).unwrap();
        assert!(on.fallback.is_on() && on.changed && on.reboot_required);
        assert!(main_conf(dir.path()).contains("Privacy = device"));
        assert!(dir.path().join("modprobe.d/bluetooth.conf").exists());

        let off = board.set(false).unwrap();
        assert!(off.fallback.is_off() && off.changed);
        assert!(main_conf(dir.path()).contains("Privacy = off"));
        // The file this created is gone, not merely emptied: a leftover with only comments in it
        // reads, to whoever compares two boards later, as a setting somebody meant.
        assert!(!dir.path().join("modprobe.d/bluetooth.conf").exists());
    }

    /// Re-running reports no change, so a script can assert a state without branching on it — the
    /// same idempotence rule the rest of the `pad.*` surface holds to.
    #[test]
    fn asking_for_the_state_the_board_is_already_in_changes_nothing() {
        let dir = tempfile::tempdir().unwrap();
        provisioned(dir.path());
        let board = board(dir.path());

        assert!(board.set(true).unwrap().changed);
        let again = board.set(true).unwrap();
        assert!(!again.changed, "{again:?}");
        // And no reboot, because nothing moved. A reboot instruction that fires every time is one
        // nobody reads.
        assert!(!again.reboot_required);
    }

    /// Everything else in `main.conf` survives. This file is BlueZ's whole configuration and this
    /// module has an opinion about exactly one line of it.
    #[test]
    fn the_rest_of_the_bluez_configuration_is_left_alone() {
        let dir = tempfile::tempdir().unwrap();
        provisioned(dir.path());
        board(dir.path()).set(true).unwrap();

        let conf = main_conf(dir.path());
        assert!(conf.contains("FastConnectable = true"), "{conf}");
        assert!(conf.contains("[General]"), "{conf}");
        assert_eq!(conf.matches("Privacy").count(), 1, "{conf}");
    }

    /// A commented-out `Privacy` is not a setting, and is the shape a stock `main.conf` ships in.
    /// Uncommenting it in place is what `setup-board.sh` does, so it is what this does.
    #[test]
    fn a_commented_out_privacy_line_is_replaced_rather_than_duplicated() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dir.path().join("bluetooth")).unwrap();
        std::fs::write(
            dir.path().join("bluetooth/main.conf"),
            "[General]\n#Privacy = off\n",
        )
        .unwrap();
        let board = board(dir.path());
        assert!(!board.state().privacy_device, "a comment is not a setting");

        board.set(true).unwrap();
        let conf = main_conf(dir.path());
        assert_eq!(conf.matches("Privacy").count(), 1, "{conf}");
        assert!(conf.contains("Privacy = device"), "{conf}");
    }

    /// A `main.conf` with no `[General]` at all still ends up with a valid one, rather than a key
    /// floating above every section it does not belong to.
    #[test]
    fn a_missing_general_section_is_created() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dir.path().join("bluetooth")).unwrap();
        std::fs::write(
            dir.path().join("bluetooth/main.conf"),
            "[Policy]\nAutoEnable=true\n",
        )
        .unwrap();

        board(dir.path()).set(true).unwrap();
        let conf = main_conf(dir.path());
        let general = conf.find("[General]").expect(&conf);
        let privacy = conf.find("Privacy = device").expect(&conf);
        assert!(general < privacy, "{conf}");
        assert!(conf.contains("AutoEnable=true"), "{conf}");
    }

    /// Someone else's `modprobe.d/bluetooth.conf` keeps its contents. The name is not ours.
    #[test]
    fn removing_the_ertm_option_keeps_other_options_in_that_file() {
        let dir = tempfile::tempdir().unwrap();
        provisioned(dir.path());
        std::fs::create_dir_all(dir.path().join("modprobe.d")).unwrap();
        std::fs::write(
            dir.path().join("modprobe.d/bluetooth.conf"),
            "options btusb enable_autosuspend=0\n",
        )
        .unwrap();
        let board = board(dir.path());

        board.set(true).unwrap();
        board.set(false).unwrap();

        let left = read(&dir.path().join("modprobe.d/bluetooth.conf"));
        assert!(left.contains("enable_autosuspend=0"), "{left}");
        assert!(!left.contains("disable_ertm"), "{left}");
    }

    /// A board provisioned by `microduck_runtime`'s installer already has both halves, and must read
    /// as on rather than as untouched — otherwise `pad status` invites someone to apply a crutch the
    /// board is already wearing.
    #[test]
    fn a_board_the_runtime_installer_touched_reads_as_already_on() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dir.path().join("bluetooth")).unwrap();
        std::fs::create_dir_all(dir.path().join("modprobe.d")).unwrap();
        std::fs::write(
            dir.path().join("bluetooth/main.conf"),
            "[General]\nPrivacy = device\n",
        )
        .unwrap();
        std::fs::write(
            dir.path().join("modprobe.d/bluetooth.conf"),
            "options bluetooth disable_ertm=1\n",
        )
        .unwrap();

        assert!(board(dir.path()).state().is_on());
    }

    /// Half-applied is reported as half-applied. One flag would have to round it to one of the two
    /// answers, and both roundings are wrong in a way that costs someone an afternoon.
    #[test]
    fn half_of_it_is_not_reported_as_all_or_nothing() {
        let dir = tempfile::tempdir().unwrap();
        provisioned(dir.path());
        std::fs::create_dir_all(dir.path().join("modprobe.d")).unwrap();
        std::fs::write(
            dir.path().join("modprobe.d/bluetooth.conf"),
            "options bluetooth disable_ertm=1\n",
        )
        .unwrap();

        let state = board(dir.path()).state();
        assert!(!state.is_on() && !state.is_off(), "{state:?}");
        assert!(state.ertm_disabled && !state.privacy_device, "{state:?}");
    }

    /// Taking the fallback off a board that never had a `Privacy` line writes nothing. BlueZ
    /// defaults to off, so the line would only restate the default — and reporting a change would
    /// send someone to reboot a board where nothing moved.
    #[test]
    fn removing_it_from_a_board_with_no_privacy_line_is_not_a_change() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(dir.path().join("bluetooth")).unwrap();
        std::fs::write(dir.path().join("bluetooth/main.conf"), "[General]\n").unwrap();

        let off = board(dir.path()).set(false).unwrap();
        assert!(!off.changed && !off.reboot_required, "{off:?}");
        assert!(!main_conf(dir.path()).contains("Privacy"), "{off:?}");
    }

    /// A machine with no BlueZ configuration — a laptop running this suite — answers "not applied"
    /// rather than failing. Asking what a board is set to must work on a board that is set to
    /// nothing.
    #[test]
    fn a_machine_with_no_bluetooth_configuration_reports_nothing_applied() {
        let dir = tempfile::tempdir().unwrap();
        assert!(board(dir.path()).state().is_off());
    }
}
