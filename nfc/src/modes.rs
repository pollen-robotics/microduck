//! What a mode tag touched to the reader does: switch the robot to that mode.
//!
//! A mode tag holds one text, the name of a mode, the way a pairing tag holds
//! [`crate::ndef::PAIRING`]:
//!
//! | tag text              | what happens                                              |
//! |-----------------------|-----------------------------------------------------------|
//! | `walk_mode`           | laser tracking off, `robot.setMode` `walk`                |
//! | `roller_skating_mode` | laser tracking off, `robot.setMode` `roller`              |
//! | `laser_mode`          | `robot.setMode` `walk`, then `robot.laserTrack` on        |
//! | `soccer_mode`         | read and logged — nothing to run yet                      |
//! | `autonomous_mode`     | read and logged — nothing to run yet                      |
//!
//! Laser tracking is not one of `robotd`'s modes: it is a switch on top of whichever mode runs,
//! following the dot with the head and walking to it. So the laser tag puts the robot in `walk`
//! first, and the walk and roller tags switch the tracking off: a tag is the mode the robot is
//! then in, not something added to the last one. A `robotd` too old to know `robot.laserTrack`
//! refuses that call, which is logged and does not stop the mode switch.
//!
//! The last two are tags with no behaviour behind them yet. They are recognised here, so a tag
//! written today keeps working, and [`ModeTag::robot_mode`] is the line to change when one gets a
//! behaviour.
//!
//! The robot quacks when it reads any of them, so the person holding the tag knows it was read,
//! whether or not anything follows.

use duck_ipc_proto as proto;

use crate::ndef::Record;

/// "Tag read": the same quack a pairing tag gets.
pub const HEARD: proto::SoundTag = crate::pairing::HEARD;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeTag {
    Walk,
    RollerSkating,
    Soccer,
    Laser,
    Autonomous,
}

impl ModeTag {
    pub const ALL: [ModeTag; 5] = [
        ModeTag::Walk,
        ModeTag::RollerSkating,
        ModeTag::Soccer,
        ModeTag::Laser,
        ModeTag::Autonomous,
    ];

    /// What the tag is written with.
    pub fn text(self) -> &'static str {
        match self {
            ModeTag::Walk => "walk_mode",
            ModeTag::RollerSkating => "roller_skating_mode",
            ModeTag::Soccer => "soccer_mode",
            ModeTag::Laser => "laser_mode",
            ModeTag::Autonomous => "autonomous_mode",
        }
    }

    /// The `robot.setMode` this tag asks for, or `None` for a mode with no behaviour yet.
    pub fn robot_mode(self) -> Option<&'static str> {
        match self {
            ModeTag::Walk | ModeTag::Laser => Some("walk"),
            ModeTag::RollerSkating => Some("roller"),
            ModeTag::Soccer | ModeTag::Autonomous => None,
        }
    }

    /// Where this tag leaves laser tracking, `None` for a tag that does not touch it.
    pub fn laser(self) -> Option<bool> {
        match self {
            ModeTag::Laser => Some(true),
            ModeTag::Walk | ModeTag::RollerSkating => Some(false),
            ModeTag::Soccer | ModeTag::Autonomous => None,
        }
    }

    /// The mode one of these records names. The whole record, surrounding whitespace aside, the
    /// same rule as [`crate::ndef::asks_to_pair`].
    pub fn in_records(records: &[Record]) -> Option<ModeTag> {
        records.iter().find_map(|r| {
            let text = r.value().trim();
            Self::ALL.into_iter().find(|m| m.text() == text)
        })
    }
}

/// The rest of the robot, as far as a mode tag is concerned.
pub trait Driver {
    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String>;
    /// `robot.setMode`.
    fn set_mode(&mut self, mode: &str) -> Result<proto::IntentResult, String>;
    /// `robot.laserTrack`.
    fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String>;
}

/// How a touch ended, for the journal and the tests.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    /// `robotd` took the switch — or was already in that mode, which it also accepts. The mode,
    /// and whether laser tracking is now on.
    Switched(&'static str, bool),
    /// `robotd` said no — to the mode, or to `laser` for laser tracking — and why.
    Refused(&'static str, String),
    /// A mode with nothing behind it yet: read, quacked, nothing asked.
    NotYet,
    /// `robotd` could not be asked.
    Failed(String),
}

pub fn on_touch(robot: &mut dyn Driver, tag: ModeTag) -> Outcome {
    if let Err(why) = robot.sound(HEARD) {
        tracing::info!(%why, "the robot would not quack for the tag");
    }
    let Some(mode) = tag.robot_mode() else {
        return Outcome::NotYet;
    };
    let laser = tag.laser();
    // Off before the switch, so the new mode does not start by walking to an old dot. Not
    // stopping the switch when it fails: an older robotd has no laser to switch off.
    if laser == Some(false) {
        match robot.laser_track(false) {
            Ok(r) if r.accepted => {}
            Ok(r) => tracing::info!(reason = ?r.reason, "laser tracking not switched off"),
            Err(why) => tracing::info!(%why, "laser tracking not switched off"),
        }
    }
    match robot.set_mode(mode) {
        Err(e) => return Outcome::Failed(format!("robot.setMode: {e}")),
        Ok(r) if !r.accepted => {
            return Outcome::Refused(mode, r.reason.unwrap_or_else(|| "refused".into()));
        }
        Ok(_) => {}
    }
    if laser != Some(true) {
        return Outcome::Switched(mode, false);
    }
    match robot.laser_track(true) {
        Err(e) => Outcome::Failed(format!("robot.laserTrack: {e}")),
        Ok(r) if r.accepted => Outcome::Switched(mode, true),
        Ok(r) => Outcome::Refused("laser", r.reason.unwrap_or_else(|| "refused".into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Fake {
        refuse: Option<String>,
        /// A robotd from before laser tracking: it refuses the method.
        no_laser: bool,
        calls: Vec<String>,
    }

    impl Driver for Fake {
        fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String> {
            self.calls.push(tag.as_str().into());
            Ok(())
        }
        fn set_mode(&mut self, mode: &str) -> Result<proto::IntentResult, String> {
            self.calls.push(format!("setMode {mode}"));
            Ok(match &self.refuse {
                Some(why) => proto::IntentResult::refused(why.clone()),
                None => proto::IntentResult::accepted(),
            })
        }
        fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String> {
            self.calls
                .push(format!("laser {}", if active { "on" } else { "off" }));
            if self.no_laser {
                return Err("unknown method robot.laserTrack".into());
            }
            Ok(proto::LaserTrackResult::default())
        }
    }

    #[test]
    fn each_tag_text_names_its_mode_and_nothing_else_does() {
        for m in ModeTag::ALL {
            assert_eq!(
                ModeTag::in_records(&[Record::Text(m.text().into())]),
                Some(m)
            );
        }
        let none = |t: &str| ModeTag::in_records(&[Record::Text(t.into())]);
        assert_eq!(none(" walk_mode\n"), Some(ModeTag::Walk));
        assert_eq!(none("walk"), None);
        assert_eq!(none("WALK_MODE"), None);
        assert_eq!(none("roller_squating_mode"), None);
        assert_eq!(none(crate::ndef::PAIRING), None);
    }

    #[test]
    fn walk_and_roller_skating_switch_the_robot() {
        let mut robot = Fake::default();
        assert_eq!(
            on_touch(&mut robot, ModeTag::Walk),
            Outcome::Switched("walk", false)
        );
        assert_eq!(
            on_touch(&mut robot, ModeTag::RollerSkating),
            Outcome::Switched("roller", false)
        );
        assert_eq!(
            robot.calls,
            [
                "chirp",
                "laser off",
                "setMode walk",
                "chirp",
                "laser off",
                "setMode roller"
            ]
        );
    }

    #[test]
    fn the_laser_tag_walks_then_tracks() {
        let mut robot = Fake::default();
        assert_eq!(
            on_touch(&mut robot, ModeTag::Laser),
            Outcome::Switched("walk", true)
        );
        assert_eq!(robot.calls, ["chirp", "setMode walk", "laser on"]);
    }

    /// A robotd from before laser tracking still switches modes; only the laser tag says it could
    /// not.
    #[test]
    fn a_robotd_without_laser_still_switches_modes() {
        let mut robot = Fake {
            no_laser: true,
            ..Fake::default()
        };
        assert_eq!(
            on_touch(&mut robot, ModeTag::RollerSkating),
            Outcome::Switched("roller", false)
        );
        assert!(matches!(
            on_touch(&mut robot, ModeTag::Laser),
            Outcome::Failed(why) if why.contains("robot.laserTrack")
        ));
    }

    /// A refused walk is not followed by tracking a dot nobody can walk to.
    #[test]
    fn a_refused_walk_does_not_start_the_laser() {
        let mut robot = Fake {
            refuse: Some("no policy".into()),
            ..Fake::default()
        };
        assert_eq!(
            on_touch(&mut robot, ModeTag::Laser),
            Outcome::Refused("walk", "no policy".into())
        );
        assert_eq!(robot.calls, ["chirp", "setMode walk"]);
    }

    #[test]
    fn soccer_and_autonomous_quack_and_ask_nothing() {
        for m in [ModeTag::Soccer, ModeTag::Autonomous] {
            let mut robot = Fake::default();
            assert_eq!(on_touch(&mut robot, m), Outcome::NotYet);
            assert_eq!(robot.calls, ["chirp"]);
        }
    }

    #[test]
    fn a_refusal_says_why() {
        let mut robot = Fake {
            refuse: Some("no policy on this robot".into()),
            ..Fake::default()
        };
        assert_eq!(
            on_touch(&mut robot, ModeTag::RollerSkating),
            Outcome::Refused("roller", "no policy on this robot".into())
        );
    }
}
