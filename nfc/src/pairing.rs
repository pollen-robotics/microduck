//! What a pairing tag touched to the reader does: start the pairing, unless a pad is already
//! driving.
//!
//! Every tag says the same thing, [`crate::ndef::PAIRING`], and none names a pad. A touch is
//! `robotctl pad pair` with no address: the robot looks for a gamepad in pairing mode and takes the
//! one it finds — and refuses, saying so, when it finds two.
//!
//! The rule, in full:
//!  - a pad already **connected** means nothing happens. The robot has a driver, and a tag brushed
//!    against it must not hand the bond to whatever pad happens to be in pairing mode nearby;
//!  - otherwise the robot quacks to say it read the tag, pairs the pad it finds, and greets once it
//!    has — two different sounds, so someone holding the pad hears the difference between "got your
//!    tag, looking" and "done". A refusal gets no second sound.
//!
//! The pad has to be in pairing mode when the tag is touched — Sync on an Xbox pad, until the
//! light flashes fast. The tag starts the search; it cannot wake a pad up.

use duck_ipc_proto as proto;

/// "Tag read, pairing now": the mouth-trigger quack, the one `robotctl quack` plays.
pub const HEARD: proto::SoundTag = proto::SoundTag::Chirp;
/// "Paired": the wake-up quack, sometimes a double "wak-wak" — unmistakably not [`HEARD`].
pub const PAIRED: proto::SoundTag = proto::SoundTag::Greet;

/// Polls in a row without the tag before it counts as lifted.
///
/// One missed WUPA on a tag that never moved is ordinary at the edge of the field, and without
/// this a tag left on the reader would look like a fresh touch every time it happened.
pub const MISSES_TO_LIFT: u32 = 2;

/// Turns "which tag is on the reader now", five times a second, into touches.
///
/// **One attempt per touch.** A tag left on the reader is handled once, on arrival; lifting it and
/// touching again is how to retry. Otherwise a failed pairing would repeat every 200 ms for as long
/// as the tag sat there, each attempt holding discovery open for its whole window.
#[derive(Debug, Default)]
pub struct Touches {
    present: Option<Vec<u8>>,
    misses: u32,
}

impl Touches {
    /// This poll's UID, or `None` for no tag. True when it is a touch to act on.
    pub fn seen(&mut self, uid: Option<&[u8]>) -> bool {
        match uid {
            None => {
                self.misses += 1;
                if self.misses >= MISSES_TO_LIFT {
                    self.present = None;
                }
                false
            }
            Some(uid) => {
                self.misses = 0;
                if self.present.as_deref() == Some(uid) {
                    return false;
                }
                self.present = Some(uid.to_vec());
                true
            }
        }
    }

    /// The touch just reported could not be read. Not held against it: a tag read at the edge of
    /// the field gets another go on the next poll, rather than needing to be lifted.
    pub fn unread(&mut self) {
        self.present = None;
    }
}

/// The rest of the robot, as far as a tag is concerned.
pub trait Robot {
    fn pads(&mut self) -> Result<Vec<proto::Pad>, String>;
    /// `pad.pair` with no address: whichever pad is in pairing mode.
    fn pair(&mut self) -> Result<proto::PadPairResult, String>;
    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String>;
}

/// How a touch ended, for the journal and the tests.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    /// A pad is driving already; nothing was asked.
    AlreadyConnected(proto::Pad),
    Paired(proto::Pad),
    /// Paired, and the robot would not make its second sound — muted, or no voice bank. Still a
    /// pairing.
    PairedSilently(proto::Pad, String),
    NotPaired(proto::PadPairFailure, Option<String>),
    /// `configd` could not be asked, or its machinery failed.
    Failed(String),
}

pub fn on_touch(robot: &mut dyn Robot) -> Outcome {
    let pads = match robot.pads() {
        Ok(pads) => pads,
        Err(e) => return Outcome::Failed(format!("pad.status: {e}")),
    };
    if let Some(pad) = pads.into_iter().find(|p| p.connected) {
        return Outcome::AlreadyConnected(pad);
    }
    // Before the pairing, which blocks for up to fifteen seconds: this is the answer to "did it
    // see my tag". A robot that will not make a sound still pairs.
    if let Err(why) = robot.sound(HEARD) {
        tracing::info!(%why, "the robot would not quack for the tag");
    }
    match robot.pair() {
        Err(e) => Outcome::Failed(format!("pad.pair: {e}")),
        Ok(proto::PadPairResult::Failed { reason, detail, .. }) => {
            Outcome::NotPaired(reason, detail)
        }
        Ok(proto::PadPairResult::Paired { pad }) => match robot.sound(PAIRED) {
            Ok(()) => Outcome::Paired(pad),
            Err(e) => Outcome::PairedSilently(pad, e),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pad(mac: &str, connected: bool) -> proto::Pad {
        proto::Pad {
            mac: mac.into(),
            name: "Xbox Wireless Controller".into(),
            paired: true,
            trusted: true,
            connected,
        }
    }

    #[derive(Default)]
    struct Fake {
        pads: Vec<proto::Pad>,
        refuse: Option<proto::PadPairFailure>,
        calls: Vec<String>,
    }

    impl Robot for Fake {
        fn pads(&mut self) -> Result<Vec<proto::Pad>, String> {
            self.calls.push("status".into());
            Ok(self.pads.clone())
        }
        fn pair(&mut self) -> Result<proto::PadPairResult, String> {
            self.calls.push("pair".into());
            Ok(match self.refuse {
                Some(reason) => proto::PadPairResult::Failed {
                    reason,
                    detail: None,
                    candidates: Vec::new(),
                },
                None => proto::PadPairResult::Paired {
                    pad: pad(MAC, true),
                },
            })
        }
        fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String> {
            self.calls.push(tag.as_str().into());
            Ok(())
        }
    }

    /// The pad configd finds in pairing mode.
    const MAC: &str = "98:B6:E9:28:06:09";

    #[test]
    fn a_connected_pad_means_nothing_is_asked() {
        let mut robot = Fake {
            pads: vec![pad("AA:BB:CC:DD:EE:FF", true)],
            ..Fake::default()
        };
        let outcome = on_touch(&mut robot);
        assert!(matches!(outcome, Outcome::AlreadyConnected(p) if p.mac == "AA:BB:CC:DD:EE:FF"));
        assert_eq!(robot.calls, ["status"]);
    }

    #[test]
    fn with_no_pad_driving_it_quacks_pairs_the_one_found_then_greets() {
        // A pad bonded and switched off is not driving: it does not stop the pairing.
        let mut robot = Fake {
            pads: vec![pad("AA:BB:CC:DD:EE:FF", false)],
            ..Fake::default()
        };
        assert!(matches!(on_touch(&mut robot), Outcome::Paired(p) if p.mac == MAC));
        assert_eq!(robot.calls, ["status", "chirp", "pair", "greet"]);
    }

    #[test]
    fn a_refused_pairing_quacks_for_the_tag_and_does_not_greet() {
        let mut robot = Fake {
            refuse: Some(proto::PadPairFailure::NotFound),
            ..Fake::default()
        };
        assert_eq!(
            on_touch(&mut robot),
            Outcome::NotPaired(proto::PadPairFailure::NotFound, None)
        );
        assert_eq!(robot.calls, ["status", "chirp", "pair"]);
    }

    #[test]
    fn a_tag_left_on_the_reader_is_one_touch() {
        let mut touches = Touches::default();
        let tag: &[u8] = &[1, 2, 3, 4, 5, 6, 7];
        assert!(touches.seen(Some(tag)));
        assert!(!touches.seen(Some(tag)));
        // One missed poll is the edge of the field, not a lift.
        assert!(!touches.seen(None));
        assert!(!touches.seen(Some(tag)));
        // Two is a lift, and the next arrival is a new touch.
        touches.seen(None);
        touches.seen(None);
        assert!(touches.seen(Some(tag)));
    }

    #[test]
    fn a_different_tag_is_a_touch_at_once_and_an_unread_one_gets_another_go() {
        let mut touches = Touches::default();
        assert!(touches.seen(Some(&[1; 7])));
        assert!(touches.seen(Some(&[2; 7])));
        touches.unread();
        assert!(touches.seen(Some(&[2; 7])));
    }
}
