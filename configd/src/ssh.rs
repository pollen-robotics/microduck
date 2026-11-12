//! `system.openWifiSsh`: ssh on wifi until the next reboot.
//!
//! The shipping image answers ssh on the head's USB-C link only (microduck_yocto#110). Its
//! `microduck-ssh-access` package watches for [`SWITCH`] and, when it appears, installs a `/run`
//! drop-in that reopens port 22 on every interface. All this does is create that file: the image
//! owns the mechanism, and a reboot closes it again because `/run` is a tmpfs.
//!
//! The request reaches here only from an authenticated caller — over BLE that is the secure
//! session the setup code opens — so it is gated exactly as joining a network is.

use std::io;
use std::path::Path;

use duck_ipc_proto as proto;

/// The file the image's path unit watches.
pub const SWITCH: &str = "/run/microduck-ssh-wifi";

/// What the image installs alongside the path unit. Its absence means an image with no switch to
/// press: the dev image, which listens on wifi already, or a board with no USB link.
pub const INSTALLED: &str = "/usr/share/microduck-ssh-access/30-wifi-until-reboot.conf";

/// Press the switch, if this image has one.
///
/// Creating a file that is already there is success: the caller asked for ssh to be open, and it
/// is. Reporting `switched = false` there would read as "this image cannot", which is untrue.
pub fn open(switch: &Path, installed: &Path) -> io::Result<proto::OpenWifiSshResult> {
    if !installed.exists() {
        return Ok(proto::OpenWifiSshResult { switched: false });
    }
    std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(switch)?;
    Ok(proto::OpenWifiSshResult { switched: true })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An image without the package must not grow a stray file in `/run` that nothing watches, and
    /// must say it did nothing so a client does not promise the user a shell.
    #[test]
    fn an_image_without_the_switch_is_left_alone() {
        let dir = tempfile::tempdir().unwrap();
        let switch = dir.path().join("switch");
        let result = open(&switch, &dir.path().join("absent")).unwrap();
        assert!(!result.switched);
        assert!(!switch.exists());
    }

    /// Pressing it twice is the normal case — a phone that retried — and must not fail.
    #[test]
    fn pressing_twice_is_success() {
        let dir = tempfile::tempdir().unwrap();
        let installed = dir.path().join("installed");
        std::fs::write(&installed, "").unwrap();
        let switch = dir.path().join("switch");
        assert!(open(&switch, &installed).unwrap().switched);
        assert!(open(&switch, &installed).unwrap().switched);
        assert!(switch.exists());
    }
}
