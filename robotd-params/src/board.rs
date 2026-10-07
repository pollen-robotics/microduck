//! `[board]` — which electronic board this robot is built on.
//!
//! **Declared, not detected.** Whatever sets a board up writes it — `provision-board.sh --board`,
//! or the image built for that board — and everything that differs between boards reads it from
//! here. Unset is `zero3`, which is every robot that existed before the key did.
//!
//! [`Board::detect`] reads the device tree, and only `robotctl health` uses it: to say when the
//! declaration and the hardware disagree. Nothing switches on what it finds, because a guess that
//! is right on the boards we have is not a fact about the next one.
//!
//! The board also decides which releases a robot can install, through the updater's `hw_rev`
//! (`docs/design/updater-design.md` §5.6).

use std::path::Path;

use serde::{Deserialize, Serialize};

/// Where the kernel exposes the device tree's root `compatible`: NUL-separated strings, most
/// specific first.
pub const COMPATIBLE_PATH: &str = "/proc/device-tree/compatible";

/// One electronic board.
///
/// **Ordered by [`Board::hw_rev`], oldest first**, and the order is what deprecation runs on:
/// a release refuses every board below its `min_hw_rev`, so boards can only be retired oldest
/// first. [`tests::revisions_ascend_and_retire_in_order`] holds both.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Board {
    /// The Radxa Zero 3W with the prototype's HAT: the AIC8800 radio, the TLV320AIC3104 codec,
    /// the IMX219 camera.
    #[default]
    Zero3,
    /// The custom board's first spin: Seeed's RK3566 main board.
    Beta,
}

/// Every board, in the order an editor cycles them — and the strings the file uses.
///
/// [`tests::every_board_label_round_trips`] pins it to the enum in both directions.
pub const BOARD_LABELS: &[&str] = &["zero3", "beta"];

impl Board {
    /// The boards, in [`BOARD_LABELS`] order.
    pub const ALL: [Board; 2] = [Board::Zero3, Board::Beta];

    /// The name this board has in the file.
    pub fn label(self) -> &'static str {
        match self {
            Board::Zero3 => "zero3",
            Board::Beta => "beta",
        }
    }

    /// The hardware revision the updater checks a release's `min_hw_rev` against.
    ///
    /// `1` is what every `updater.toml` said before the board was a setting, so a robot that
    /// still carries `hw_rev = 1` and one that derives it agree.
    pub fn hw_rev(self) -> u32 {
        match self {
            Board::Zero3 => 1,
            Board::Beta => 2,
        }
    }

    /// The head camera sensor this board is built with.
    ///
    /// A camera and its board go together: `mediad` refuses a media graph holding another sensor,
    /// unless `[media] sensor` forces one ([`crate::MediaSensor`]).
    pub fn camera_sensor(self) -> crate::CameraSensor {
        match self {
            Board::Zero3 => crate::CameraSensor::Imx219,
            Board::Beta => crate::CameraSensor::Gc2093,
        }
    }

    /// How far this board's head camera is mounted from upright, in degrees clockwise.
    ///
    /// The Zero 3W's IMX219 sits a quarter turn off; the beta's GC2093 sits the right way up.
    /// `mediad` tells every consumer this angle (`media.video`'s `rotate`) unless `--rotate`
    /// overrides it, so a consumer turns the picture by what this board needs and nothing else.
    pub fn camera_mount_degrees(self) -> u32 {
        match self {
            Board::Zero3 => 90,
            Board::Beta => 0,
        }
    }

    /// The last release this board gets, or `None` while it is supported.
    ///
    /// **Setting this is how a board is retired, and it is the only step.** From the release that
    /// sets it, `robotctl health` on that board warns that updates end at this version; every
    /// release after it is packaged with a `min_hw_rev` above this board's (`xtask package`), and
    /// the updater the board is already running refuses it with the reason it gives. Set it a few
    /// releases ahead, so the warning is on robots for a while before the updates stop.
    pub fn last_release(self) -> Option<&'static str> {
        match self {
            Board::Zero3 => None,
            Board::Beta => None,
        }
    }

    /// The board a device tree's root `compatible` describes, if it is one we know.
    ///
    /// The strings are the boards' own: radxa's tree for the Zero 3W (`radxa,zero3w-aic8800ds2`
    /// on the image, `radxa,zero3` on Armbian), our device tree for the beta
    /// (`pollen,microduck-beta`), and Seeed's factory image for the same board
    /// (`seeed,microduck`).
    pub fn detect(compatible: &[u8]) -> Option<Board> {
        compatible
            .split(|&b| b == 0)
            .filter_map(|s| std::str::from_utf8(s).ok())
            .find_map(|s| {
                if s.starts_with("pollen,microduck") || s.starts_with("seeed,microduck") {
                    Some(Board::Beta)
                } else if s.starts_with("radxa,zero3") || s.starts_with("radxa,zero-3") {
                    Some(Board::Zero3)
                } else {
                    None
                }
            })
    }

    /// What this machine's device tree says, or `None` off a board or on one we do not know.
    pub fn detected() -> Option<Board> {
        std::fs::read(COMPATIBLE_PATH)
            .ok()
            .and_then(|c| Self::detect(&c))
    }

    /// The board `robotd.toml` at `path` declares, read on its own — `None` when it does not
    /// say, which callers read as `zero3`.
    ///
    /// For a reader that wants this key and nothing else — the updater, before every check, and
    /// `robotctl health`. A file that is invalid somewhere else still says which board it is. One
    /// that is missing, does not say, or names a board this build does not know is `None`.
    pub fn declared(path: &Path) -> Option<Board> {
        #[derive(Deserialize)]
        struct OnlyBoard {
            board: BoardOnly,
        }
        #[derive(Deserialize)]
        struct BoardOnly {
            version: Board,
        }
        let text = std::fs::read_to_string(path).ok()?;
        let table = toml::from_str::<toml::Table>(&text).ok()?;
        toml::Value::Table(table)
            .try_into::<OnlyBoard>()
            .ok()
            .map(|only| only.board.version)
    }
}

impl std::fmt::Display for Board {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

/// The `min_hw_rev` a release is packaged with: the lowest revision among the boards it still
/// supports, so every board whose [`Board::last_release`] is behind it refuses it.
///
/// A prerelease sorts below its release, so a dev build of a board's last release is still that
/// board's. `None` when the release would support no board at all, which `xtask package` refuses
/// to build.
pub fn min_hw_rev(release: &semver::Version) -> Option<u32> {
    min_hw_rev_given(release, Board::last_release)
}

/// [`min_hw_rev`] over any retirement table, so the cut-off can be tested before a board is
/// actually retired.
fn min_hw_rev_given(
    release: &semver::Version,
    last_release: impl Fn(Board) -> Option<&'static str>,
) -> Option<u32> {
    Board::ALL
        .into_iter()
        .filter(|&board| {
            last_release(board).is_none_or(|last| {
                release <= &semver::Version::parse(last).expect("tested: a version")
            })
        })
        .map(Board::hw_rev)
        .min()
}

/// `[board]`.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct BoardParams {
    /// Which board this robot is built on. Unset is `zero3`.
    pub version: Board,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_board_label_round_trips() {
        assert_eq!(BOARD_LABELS.len(), Board::ALL.len());
        for (label, board) in BOARD_LABELS.iter().zip(Board::ALL) {
            assert_eq!(*label, board.label());
            let parsed: crate::Params =
                toml::from_str(&format!("[board]\nversion = \"{label}\"\n")).expect("parses");
            assert_eq!(parsed.board.version, board);
        }
    }

    /// The two properties `xtask package` relies on to turn [`Board::last_release`] into a
    /// `min_hw_rev`: revisions go up in [`Board::ALL`] order, and a board is never retired
    /// while an older one is still supported — `min_hw_rev` is one number, so it cannot leave
    /// a gap.
    #[test]
    fn revisions_ascend_and_retire_in_order() {
        for pair in Board::ALL.windows(2) {
            assert!(pair[0].hw_rev() < pair[1].hw_rev(), "{pair:?}");
            if let Some(newer) = pair[1].last_release() {
                let older = pair[0]
                    .last_release()
                    .unwrap_or_else(|| panic!("{} retired while {} is not", pair[1], pair[0]));
                let (older, newer) = (
                    semver::Version::parse(older).expect("a version"),
                    semver::Version::parse(newer).expect("a version"),
                );
                assert!(older <= newer, "{pair:?}");
            }
        }
        for board in Board::ALL {
            if let Some(last) = board.last_release() {
                semver::Version::parse(last)
                    .unwrap_or_else(|e| panic!("{board}: last_release {last}: {e}"));
            }
        }
    }

    /// Nothing is retired today, so every release supports every board — and `0` would have
    /// been the same answer for the robots in the field, whose `hw_rev` is `1`.
    #[test]
    fn a_release_supports_every_board_nothing_retired() {
        let release = semver::Version::new(0, 16, 0);
        assert_eq!(min_hw_rev(&release), Some(Board::Zero3.hw_rev()));
    }

    /// The zero3 retired at 0.21.0: up to and including that release, and a dev build of it,
    /// it is still supported; the first release after it is packaged for the beta only.
    #[test]
    fn the_release_after_a_board_is_last_leaves_it_behind() {
        let retired = |board| match board {
            Board::Zero3 => Some("0.21.0"),
            Board::Beta => None,
        };
        let at = |v: &str| min_hw_rev_given(&semver::Version::parse(v).unwrap(), retired);
        assert_eq!(at("0.20.3"), Some(1));
        assert_eq!(at("0.21.0"), Some(1));
        assert_eq!(at("0.21.0-dev.4.abc1234"), Some(1));
        assert_eq!(at("0.21.1"), Some(2));
        assert_eq!(at("0.22.0-dev.1.abc1234"), Some(2));

        let all_retired = |_| Some("0.21.0");
        let v = semver::Version::new(0, 22, 0);
        assert_eq!(min_hw_rev_given(&v, all_retired), None);
    }

    /// The Zero 3W's camera is a quarter turn off and the beta's is upright — a beta told 90°
    /// showed every consumer a sideways picture.
    #[test]
    fn each_board_mounts_its_camera_its_own_way() {
        assert_eq!(Board::Zero3.camera_mount_degrees(), 90);
        assert_eq!(Board::Beta.camera_mount_degrees(), 0);
        assert_eq!(Board::default().camera_mount_degrees(), 90);
    }

    #[test]
    fn the_device_tree_names_the_board() {
        let cases: &[(&[u8], Option<Board>)] = &[
            (
                b"radxa,zero3w-aic8800ds2\0rockchip,rk3566\0",
                Some(Board::Zero3),
            ),
            (b"radxa,zero3\0rockchip,rk3566\0", Some(Board::Zero3)),
            (
                b"pollen,microduck-beta\0rockchip,rk3566\0",
                Some(Board::Beta),
            ),
            (b"seeed,microduck\0rockchip,rk3566\0", Some(Board::Beta)),
            (b"rockchip,rk3566\0", None),
            (b"", None),
        ];
        for (compatible, board) in cases {
            assert_eq!(Board::detect(compatible), *board, "{compatible:?}");
        }
    }

    #[test]
    fn the_declaration_survives_a_file_robotd_would_reject() {
        let dir = tempfile::tempdir().expect("temp dir");
        let path = dir.path().join("robotd.toml");

        let read = |text: &str| {
            std::fs::write(&path, text).expect("write");
            Board::declared(&path)
        };
        assert_eq!(read("[board]\nversion = \"beta\"\n"), Some(Board::Beta));
        // A key from a newer release, and a section this build has never heard of: neither is
        // a reason to lose the board.
        assert_eq!(
            read("[board]\nversion = \"beta\"\nnext = 1\n[future]\nx = 2\n"),
            Some(Board::Beta)
        );
        assert_eq!(read("[board]\nversion = \"zero3\"\n"), Some(Board::Zero3));
        assert_eq!(read("[bus]\nport = \"/dev/ttyS2\"\n"), None);
        assert_eq!(read("[board]\n# version = \"zero3\"\n"), None);
        assert_eq!(read("[board]\nversion = \"gamma\"\n"), None);
        assert_eq!(read("not toml ["), None);
        std::fs::remove_file(&path).expect("remove");
        assert_eq!(Board::declared(&path), None);
    }
}
