//! What a mode tag touched to the reader does: switch the robot to that mode.
//!
//! A mode tag holds one text, the name of a mode, the way a pairing tag holds
//! [`crate::ndef::PAIRING`]:
//!
//! | tag text              | what happens                                              |
//! |-----------------------|-----------------------------------------------------------|
//! | `walk_mode`           | laser tracking and soccer off, `robot.setMode` `walk`     |
//! | `roller_skating_mode` | laser tracking and soccer off, `robot.setMode` `roller`   |
//! | `laser_mode`          | soccer off, `robot.setMode` `walk`, `robot.laserTrack` on |
//! | `soccer_mode`         | laser off, `robot.setMode` `walk`, `robot.soccer` on      |
//! | `autonomous_mode`     | read and logged — nothing to run yet                      |
//!
//! Laser tracking and soccer are not `robotd` modes: each is a switch on top of whichever mode
//! runs. So their tags put the robot in `walk` first, and every other tag switches them off: a
//! tag is the mode the robot is then in, not something added to the last one. A `robotd` too old
//! to know a switch refuses the call, which is logged and does not stop the mode switch.
//!
//! Soccer mode by tag is soccer *by hand*: the pad's bumpers kick, and D-pad down is where the
//! robot starts playing by itself (`pad_map`). The tag does not start autonomous play — a robot
//! that sets off after a ball the moment a card is touched to it is a surprise to whoever is
//! holding it.
//!
//! `autonomous_mode` has no behaviour behind it yet. It is recognised here, so a tag written
//! today keeps working, and [`ModeTag::robot_mode`] is the line to change when it gets one.
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
            ModeTag::Walk | ModeTag::Laser | ModeTag::Soccer => Some("walk"),
            ModeTag::RollerSkating => Some("roller"),
            ModeTag::Autonomous => None,
        }
    }

    /// Which switch this tag turns on top of its mode, if any.
    pub fn switch(self) -> Option<Switch> {
        match self {
            ModeTag::Laser => Some(Switch::Laser),
            ModeTag::Soccer => Some(Switch::Soccer),
            ModeTag::Walk | ModeTag::RollerSkating | ModeTag::Autonomous => None,
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

/// A switch on top of `robotd`'s mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Switch {
    /// `robot.laserTrack`.
    Laser,
    /// `robot.soccer`.
    Soccer,
}

impl Switch {
    pub const ALL: [Switch; 2] = [Switch::Laser, Switch::Soccer];

    pub fn name(self) -> &'static str {
        match self {
            Switch::Laser => "laser",
            Switch::Soccer => "soccer",
        }
    }

    /// Ask for it on or off. Laser tracking and soccer each switch the other off in `robotd`, so
    /// turning one on needs nothing more.
    fn set(self, robot: &mut dyn Driver, on: bool) -> Result<(bool, Option<String>), String> {
        match self {
            Switch::Laser => robot.laser_track(on).map(|r| (r.accepted, r.reason)),
            Switch::Soccer => robot.soccer(on).map(|r| (r.accepted, r.reason)),
        }
    }
}

/// The rest of the robot, as far as a mode tag is concerned.
pub trait Driver {
    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String>;
    /// `robot.setMode`.
    fn set_mode(&mut self, mode: &str) -> Result<proto::IntentResult, String>;
    /// `robot.laserTrack`.
    fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String>;
    /// `robot.soccer` — soccer mode by hand, or off.
    fn soccer(&mut self, active: bool) -> Result<proto::IntentResult, String>;
}

/// How a touch ended, for the journal and the tests.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    /// `robotd` took the switch — or was already in that mode, which it also accepts. The mode,
    /// and the switch now on top of it.
    Switched(&'static str, Option<Switch>),
    /// `robotd` said no — to the mode, or to `laser`/`soccer` for that switch — and why.
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
    let wanted = tag.switch();
    // The other switches off before the mode, so the new mode does not start by walking to an old
    // dot or ball. Not stopping the switch when it fails: an older robotd has no such switch.
    for switch in Switch::ALL.into_iter().filter(|s| Some(*s) != wanted) {
        match switch.set(robot, false) {
            Ok((true, _)) => {}
            Ok((false, reason)) => {
                tracing::info!(switch = switch.name(), ?reason, "not switched off")
            }
            Err(why) => tracing::info!(switch = switch.name(), %why, "not switched off"),
        }
    }
    match robot.set_mode(mode) {
        Err(e) => return Outcome::Failed(format!("robot.setMode: {e}")),
        Ok(r) if !r.accepted => {
            return Outcome::Refused(mode, r.reason.unwrap_or_else(|| "refused".into()));
        }
        Ok(_) => {}
    }
    let Some(switch) = wanted else {
        return Outcome::Switched(mode, None);
    };
    match switch.set(robot, true) {
        Err(e) => Outcome::Failed(format!("robot.{}: {e}", switch.name())),
        Ok((true, _)) => Outcome::Switched(mode, Some(switch)),
        Ok((false, reason)) => {
            Outcome::Refused(switch.name(), reason.unwrap_or_else(|| "refused".into()))
        }
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
        fn soccer(&mut self, active: bool) -> Result<proto::IntentResult, String> {
            self.calls
                .push(format!("soccer {}", if active { "on" } else { "off" }));
            if self.no_laser {
                return Err("unknown method robot.soccer".into());
            }
            Ok(proto::IntentResult::accepted())
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
            Outcome::Switched("walk", None)
        );
        assert_eq!(
            on_touch(&mut robot, ModeTag::RollerSkating),
            Outcome::Switched("roller", None)
        );
        assert_eq!(
            robot.calls,
            [
                "chirp",
                "laser off",
                "soccer off",
                "setMode walk",
                "chirp",
                "laser off",
                "soccer off",
                "setMode roller"
            ]
        );
    }

    #[test]
    fn the_laser_tag_walks_then_tracks() {
        let mut robot = Fake::default();
        assert_eq!(
            on_touch(&mut robot, ModeTag::Laser),
            Outcome::Switched("walk", Some(Switch::Laser))
        );
        assert_eq!(
            robot.calls,
            ["chirp", "soccer off", "setMode walk", "laser on"]
        );
    }

    /// The soccer tag: walking, laser off, and soccer mode by hand — the pad starts the robot
    /// playing by itself, not the tag.
    #[test]
    fn the_soccer_tag_walks_then_plays_by_hand() {
        let mut robot = Fake::default();
        assert_eq!(
            on_touch(&mut robot, ModeTag::Soccer),
            Outcome::Switched("walk", Some(Switch::Soccer))
        );
        assert_eq!(
            robot.calls,
            ["chirp", "laser off", "setMode walk", "soccer on"]
        );
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
            Outcome::Switched("roller", None)
        );
        assert!(matches!(
            on_touch(&mut robot, ModeTag::Laser),
            Outcome::Failed(why) if why.contains("robot.laser")
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
        assert_eq!(robot.calls, ["chirp", "soccer off", "setMode walk"]);
    }

    #[test]
    fn autonomous_quacks_and_asks_nothing() {
        let mut robot = Fake::default();
        assert_eq!(on_touch(&mut robot, ModeTag::Autonomous), Outcome::NotYet);
        assert_eq!(robot.calls, ["chirp"]);
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
