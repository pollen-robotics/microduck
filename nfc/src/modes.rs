//! What a mode tag touched to the reader does: switch the robot to that mode.
//!
//! A mode tag holds one text, the name of a mode, the way a pairing tag holds
//! [`crate::ndef::PAIRING`]:
//!
//! | tag text              | what happens                         |
//! |-----------------------|--------------------------------------|
//! | `walk_mode`           | `robot.setMode` `walk`               |
//! | `roller_skating_mode` | `robot.setMode` `roller`             |
//! | `soccer_mode`         | read and logged — nothing to run yet |
//! | `laser_mode`          | read and logged — nothing to run yet |
//! | `autonomous_mode`     | read and logged — nothing to run yet |
//!
//! The last three are tags with no behaviour behind them yet. They are recognised here, so a tag
//! written today keeps working, and [`ModeTag::robot_mode`] is the one line to change when one
//! gets a behaviour. `robotd` is not asked about them: its modes pick policy sets, and it has
//! none to load for these.
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
            ModeTag::Walk => Some("walk"),
            ModeTag::RollerSkating => Some("roller"),
            ModeTag::Soccer | ModeTag::Laser | ModeTag::Autonomous => None,
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
}

/// How a touch ended, for the journal and the tests.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    /// `robotd` took the switch — or was already in that mode, which it also accepts.
    Switched(&'static str),
    /// `robotd` said no, and why.
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
    match robot.set_mode(mode) {
        Err(e) => Outcome::Failed(format!("robot.setMode: {e}")),
        Ok(r) if r.accepted => Outcome::Switched(mode),
        Ok(r) => Outcome::Refused(mode, r.reason.unwrap_or_else(|| "refused".into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Fake {
        refuse: Option<String>,
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
            Outcome::Switched("walk")
        );
        assert_eq!(
            on_touch(&mut robot, ModeTag::RollerSkating),
            Outcome::Switched("roller")
        );
        assert_eq!(
            robot.calls,
            ["chirp", "setMode walk", "chirp", "setMode roller"]
        );
    }

    #[test]
    fn soccer_laser_and_autonomous_quack_and_ask_nothing() {
        for m in [ModeTag::Soccer, ModeTag::Laser, ModeTag::Autonomous] {
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
