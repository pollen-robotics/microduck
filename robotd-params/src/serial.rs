//! The SoC serial: the per-chip value a robot's identity hangs off.
//!
//! Three things derive from it, and all three need it to differ between two robots: the default
//! name (`configd::identity`), the voice and the chorale id (`sounds::hardware_seed`), and the
//! `hardware_id` remote access registers under (`mediad`, through `system.info`). One reader here so
//! that they cannot disagree about which value that is.
//!
//! **Where it is read depends on the board**, because the two places it can be read from are not
//! the same value on every board. Both are Rockchip's derivation — a CRC of sixteen bytes of the
//! SoC's OTP — computed twice: once by U-Boot, which hands it to the kernel as the device tree's
//! `serial-number`, and once by the kernel's `rockchip-cpuinfo`, which prints it as `/proc/cpuinfo`'s
//! `Serial`.
//!
//! - **zero3: the device tree.** That is what every Zero 3W's name and voice were derived from, and
//!   changing the source would rename them and change their voices.
//! - **beta: `/proc/cpuinfo`.** The U-Boot on the beta images reads the OTP three bytes early, at
//!   0x07 instead of 0x0a. Its sixteen bytes then end at 0x16, which is where the chip's model and
//!   lot end and the per-chip bytes begin, so every chip in a lot hashes to one value: two boards
//!   from Seeed's first batch both said `8b33c6e86a6a0c15` and both came up as `duck-d041`, with
//!   one voice, and one chorale id — which makes each duck drop the other's beacons as its own
//!   echo. The kernel reads from 0x0a, and those two boards said `62285dd716c88f0f` and
//!   `9ed5ef3c1c5ff3f1`. (Measured 2026-10-15 by hashing each board's OTP dump at both offsets.)
//!   A beta with no `Serial` line has no serial: falling back to the device tree would bring the
//!   collision back, so the callers' own fallbacks apply instead.
//!
//! The board is the declared one ([`Board::declared`]), as everywhere else that differs per board.

use std::path::Path;

use crate::board::Board;

/// Where U-Boot leaves its serial.
pub const DEVICE_TREE_PATH: &str = "/proc/device-tree/serial-number";

/// Where the kernel prints its own.
pub const CPUINFO_PATH: &str = "/proc/cpuinfo";

/// This robot's SoC serial, or `None` where there is not one to read — a laptop, a board whose
/// bootloader leaves the property out, a beta whose kernel prints no `Serial`.
pub fn soc_serial() -> Option<String> {
    soc_serial_at(
        Board::declared(Path::new(crate::DEFAULT_PATH)).unwrap_or_default(),
        Path::new(DEVICE_TREE_PATH),
        Path::new(CPUINFO_PATH),
    )
}

/// [`soc_serial`], for a given board and against arbitrary paths so it can be tested off one.
pub fn soc_serial_at(board: Board, device_tree: &Path, cpuinfo: &Path) -> Option<String> {
    match board {
        Board::Zero3 => device_tree_serial(device_tree),
        Board::Beta | Board::FaceBoardV1 => cpuinfo_serial(cpuinfo),
    }
}

fn device_tree_serial(path: &Path) -> Option<String> {
    let raw = std::fs::read(path).ok()?;
    let text = String::from_utf8_lossy(&raw);
    // Devicetree properties are NUL-terminated strings, and a property may hold several — so the
    // first NUL ends the value rather than being trailing junk to trim.
    valid(text.split('\0').next()?)
}

fn cpuinfo_serial(path: &Path) -> Option<String> {
    let text = std::fs::read_to_string(path).ok()?;
    let value = text.lines().find_map(|line| {
        let (key, value) = line.split_once(':')?;
        (key.trim() == "Serial").then_some(value)
    })?;
    // The kernel prints zeros when it found no OTP to read, and zeros are every such board.
    valid(value).filter(|serial| serial.chars().any(|c| c != '0'))
}

/// A value that is blank, or full of control characters, is no identity at all — and it reaches a
/// client verbatim through `system.info`.
fn valid(value: &str) -> Option<String> {
    let serial = value.trim();
    (!serial.is_empty() && serial.chars().all(|c| c.is_ascii_graphic())).then(|| serial.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two sources as the beta board at 192.168.10.133 reports them.
    fn beta_133(dir: &Path) -> (std::path::PathBuf, std::path::PathBuf) {
        let device_tree = dir.join("serial-number");
        std::fs::write(&device_tree, b"8b33c6e86a6a0c15\0").unwrap();
        let cpuinfo = dir.join("cpuinfo");
        std::fs::write(
            &cpuinfo,
            "processor\t: 3\nBogoMIPS\t: 48.00\nCPU part\t: 0xd05\n\nSerial\t\t: 62285dd716c88f0f\n",
        )
        .unwrap();
        (device_tree, cpuinfo)
    }

    /// The beta reads the kernel's value, not U-Boot's lot-wide one.
    #[test]
    fn a_beta_reads_the_kernels_serial() {
        let dir = tempfile::tempdir().unwrap();
        let (device_tree, cpuinfo) = beta_133(dir.path());
        assert_eq!(
            soc_serial_at(Board::Beta, &device_tree, &cpuinfo).as_deref(),
            Some("62285dd716c88f0f")
        );
        assert_eq!(
            soc_serial_at(Board::FaceBoardV1, &device_tree, &cpuinfo).as_deref(),
            Some("62285dd716c88f0f")
        );
    }

    /// A Zero 3W keeps the value its name and voice were made from.
    #[test]
    fn a_zero3_reads_the_device_tree() {
        let dir = tempfile::tempdir().unwrap();
        let (device_tree, cpuinfo) = beta_133(dir.path());
        assert_eq!(
            soc_serial_at(Board::Zero3, &device_tree, &cpuinfo).as_deref(),
            Some("8b33c6e86a6a0c15")
        );
    }

    /// A beta with no usable `Serial` has no serial — never the device tree's, which is the same on
    /// every board in a lot.
    #[test]
    fn a_beta_without_a_kernel_serial_has_none() {
        let dir = tempfile::tempdir().unwrap();
        let (device_tree, cpuinfo) = beta_133(dir.path());
        for text in [
            "processor\t: 0\n",
            "Serial\t\t: 0000000000000000\n",
            "Serial\t\t: \n",
        ] {
            std::fs::write(&cpuinfo, text).unwrap();
            assert_eq!(
                soc_serial_at(Board::Beta, &device_tree, &cpuinfo),
                None,
                "{text:?}"
            );
        }
        assert_eq!(
            soc_serial_at(Board::Beta, &device_tree, &dir.path().join("absent")),
            None
        );
    }

    /// The device tree's property is NUL-terminated; blank or unprintable is no serial.
    #[test]
    fn a_device_tree_serial_is_the_first_string_and_must_be_printable() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("serial-number");
        let cpuinfo = dir.path().join("absent");
        std::fs::write(&path, b"bb7b734a7717ac41\0").unwrap();
        assert_eq!(
            soc_serial_at(Board::Zero3, &path, &cpuinfo).as_deref(),
            Some("bb7b734a7717ac41")
        );
        for raw in [&b""[..], b"\0", b"   \0", b"\x01\x02\0"] {
            std::fs::write(&path, raw).unwrap();
            assert_eq!(
                soc_serial_at(Board::Zero3, &path, &cpuinfo),
                None,
                "{raw:?}"
            );
        }
        assert_eq!(
            soc_serial_at(Board::Zero3, &dir.path().join("absent"), &cpuinfo),
            None
        );
    }
}
