//! Gamepads, as a trait plus a fake.
//!
//! ## Why `configd` owns this and `padd` does not
//!
//! `padd` reads a pad and sends intents over `robotd`'s socket with **no privileged access at
//! all** — that is most of the reason it is a separate process. It exercises the same API the
//! phone app and the SDK will use, every day, so that API cannot quietly rot. Letting it configure
//! BlueZ would have undone exactly that property.
//!
//! So pairing lives here, alongside wifi, for the same reason wifi does: it is a *configuration*
//! question about the radio, it needs root, and it has to be answerable when the robot itself is
//! not working (`architecture.md` §3.1). It also comes out reachable from the phone — `btd`
//! forwards `pad.*` to this socket — which is where "pair a controller" belongs long term.
//!
//! ## What replaced what
//!
//! Before this, pairing a pad meant knowing its MAC address and running three `bluetoothctl`
//! commands in an order that is not the obvious one (`connect` before `pair`; leading with `pair`
//! returns `AuthenticationCanceled`). That order is now in one place — `crate::bluez` — with the
//! reason next to it, instead of in a provisioning script's comments and in whoever's shell
//! history.
//!
//! The trait exists for the same reason [`crate::net::Net`] does: the suite runs on a laptop with
//! no radio, and the logic worth testing is the dispatch, the authorisation and the "which of
//! these devices is a gamepad" decision — not BlueZ.

use std::time::Duration;

use async_trait::async_trait;
use duck_ipc_proto as proto;

/// What went wrong, in terms a caller can act on.
pub type PadResult<T> = Result<T, String>;

/// How long to look for a pad when the caller does not say.
///
/// Fifteen seconds because someone is standing there holding a sync button, and that is about how
/// long a person will wait before concluding it did not work. Long enough for BlueZ to report a
/// device that only advertises every few seconds; short enough that a phone gets an answer rather
/// than a spinner.
pub const DEFAULT_PAIR_TIMEOUT: Duration = Duration::from_secs(15);

/// Longest window a caller may ask for.
///
/// A cap rather than a courtesy: discovery is on for the whole window, and a client that asked for
/// an hour would leave the adapter scanning long after whoever typed it walked away.
pub const MAX_PAIR_TIMEOUT: Duration = Duration::from_secs(120);

#[async_trait]
pub trait Pads: Send + Sync {
    /// Every pad this robot is bonded to, connected first.
    async fn status(&self) -> PadResult<Vec<proto::Pad>>;

    /// Pair whatever pad is in pairing mode, or the one at `mac`.
    ///
    /// A *refusal* — nothing found, two candidates, BlueZ said no — is an `Ok` carrying
    /// [`proto::PadPairResult::Failed`]. `Err` is reserved for the machinery being broken, which is
    /// the same split [`crate::net::Net`] uses and the same one the dispatcher turns into either a
    /// result or an `INTERNAL_ERROR`.
    async fn pair(&self, mac: Option<&str>, timeout: Duration) -> PadResult<proto::PadPairResult>;

    /// Drop the bond, so this pad stops reconnecting.
    async fn forget(&self, mac: &str) -> PadResult<proto::PadForgetResult>;
}

/// Clamp a caller's timeout into something the adapter should be asked to do.
pub fn pair_timeout(requested: Option<u32>) -> Duration {
    match requested {
        // Zero is "look once", not "look forever": a scripted retry loop should be able to ask
        // whether a pad is there right now without holding discovery open.
        Some(seconds) => Duration::from_secs(u64::from(seconds)).min(MAX_PAIR_TIMEOUT),
        None => DEFAULT_PAIR_TIMEOUT,
    }
}

/// Is this Bluetooth device a gamepad?
///
/// Four signals, because no single one is present on every pad, and getting this wrong in either
/// direction is bad in a different way: too narrow and the pad in pairing mode is invisible, too
/// broad and `pad.pair` bonds the robot to a colleague's headphones.
///
///  - **`icon`** is BlueZ's own classification, derived from the class or the appearance. When it
///    says `input-gaming` the question is settled, and this is the signal an Xbox controller
///    actually presents on this board.
///  - **`class`** is the BR/EDR class-of-device: bits 8-12 are the major device class, and `0x05`
///    is Peripheral. Bits 6-7 of the minor field distinguish keyboard from pointing device from
///    gamepad — `0x01` in bits 2-5 with the keyboard/pointer bits clear is a joystick or gamepad.
///    Present for a classic pad, absent for a BLE-only one.
///  - **`appearance`** is the BLE equivalent: category 15 (`0x03C0..=0x03C4`) is HID, and `0x03C4`
///    is specifically Gamepad. Many pads never set it, which is why it cannot stand alone.
///  - **the name**, last and deliberately: it is the signal that works when the other three are
///    absent, which for a pad still in pairing mode is common, and it is the one that can be wrong.
///
/// A device that satisfies none of these is not offered as a pad. `mac` is how someone pairs
/// hardware this does not recognise, which is the escape hatch that keeps the heuristic from being
/// load-bearing.
pub fn looks_like_a_gamepad(
    name: &str,
    icon: Option<&str>,
    class: Option<u32>,
    appearance: Option<u16>,
) -> bool {
    if icon == Some("input-gaming") {
        return true;
    }

    if let Some(class) = class {
        let major = (class >> 8) & 0x1f;
        let minor = (class >> 2) & 0x3f;
        // Peripheral, with the joystick/gamepad/remote minor bits. 0x01 is joystick, 0x02 gamepad,
        // 0x03 remote control — the keyboard (0x10) and pointing-device (0x20) bits are the ones
        // this must not match, and they live above these values rather than overlapping them.
        if major == 0x05 && matches!(minor & 0x0f, 0x01 | 0x02) {
            return true;
        }
    }

    if let Some(appearance) = appearance {
        // 0x03C4 is Gamepad; the rest of category 15 is other HID. Only the gamepad value counts,
        // because a Bluetooth keyboard is category 15 too and must not be paired as a pad.
        if appearance == 0x03C4 {
            return true;
        }
    }

    // Last resort, and case-insensitive: BlueZ reports names as the device advertises them, and
    // pads are inconsistent about capitalisation.
    let lower = name.to_lowercase();
    [
        "controller",
        "gamepad",
        "joystick",
        "dualsense",
        "dualshock",
    ]
    .iter()
    .any(|needle| lower.contains(needle))
}

/// A set of pads that exists only in memory.
///
/// Used by the tests and by `--fake-pads`, which is what makes the whole `pad.*` surface — and the
/// failures that are awkward to arrange with real hardware, like two pads in pairing mode at once
/// — exercisable from a laptop with no radio.
pub struct FakePads {
    inner: tokio::sync::Mutex<FakeState>,
}

struct FakeState {
    /// Every pad the radio can see, bonded or not — which is the shape BlueZ reports and the reason
    /// this is one list rather than two.
    ///
    /// Modelling "in pairing mode" and "already bonded" separately looked tidier and hid the bug
    /// that mattered: a bonded pad shows up in *every* sweep, so the selection rule has to prefer an
    /// unbonded one or a robot can never be given a second pad.
    visible: Vec<proto::Pad>,
}

impl FakePads {
    /// One pad in pairing mode, nothing bonded — a fresh robot next to a controller with its sync
    /// light flashing.
    pub fn new() -> Self {
        Self::with(vec![unpaired(
            "78:86:2E:BB:13:28",
            "Xbox Wireless Controller",
        )])
    }

    pub fn with(visible: Vec<proto::Pad>) -> Self {
        Self {
            inner: tokio::sync::Mutex::new(FakeState { visible }),
        }
    }
}

/// A pad in pairing mode: seen, not bonded.
pub fn unpaired(mac: &str, name: &str) -> proto::Pad {
    proto::Pad {
        mac: mac.to_owned(),
        name: name.to_owned(),
        paired: false,
        trusted: false,
        connected: false,
    }
}

/// A pad this robot already has, in range and connected.
pub fn bonded(mac: &str, name: &str) -> proto::Pad {
    proto::Pad {
        mac: mac.to_owned(),
        name: name.to_owned(),
        paired: true,
        trusted: true,
        connected: true,
    }
}

impl Default for FakePads {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Pads for FakePads {
    async fn status(&self) -> PadResult<Vec<proto::Pad>> {
        let state = self.inner.lock().await;
        Ok(state
            .visible
            .iter()
            .filter(|pad| pad.paired)
            .cloned()
            .collect())
    }

    async fn pair(&self, mac: Option<&str>, _timeout: Duration) -> PadResult<proto::PadPairResult> {
        let mut state = self.inner.lock().await;

        let candidates: Vec<proto::Pad> = state
            .visible
            .iter()
            .filter(|pad| mac.is_none_or(|wanted| wanted.eq_ignore_ascii_case(&pad.mac)))
            .cloned()
            .collect();

        // The same rule as `crate::bluez`: an unbonded pad wins, because that is the one someone
        // just put into pairing mode. A pad the robot already has is not a competing answer, and
        // treating it as one is what made a second pad impossible to add.
        let fresh: Vec<&proto::Pad> = candidates.iter().filter(|pad| !pad.paired).collect();
        let pad = match (fresh.as_slice(), candidates.as_slice()) {
            ([only], _) => (*only).clone(),
            ([], []) => {
                return Ok(proto::PadPairResult::Failed {
                    reason: proto::PadPairFailure::NotFound,
                    detail: None,
                });
            }
            // Nothing new in pairing mode, so the answer is the pad already bonded: an idempotent
            // re-run, which is also how a lost `Trusted` is repaired.
            ([], already) => already
                .iter()
                .min_by(|a, b| a.mac.cmp(&b.mac))
                .expect("non-empty")
                .clone(),
            (several, _) => {
                return Ok(proto::PadPairResult::Failed {
                    reason: proto::PadPairFailure::Ambiguous,
                    detail: Some(format!("{} pads are in pairing mode", several.len())),
                });
            }
        };

        let paired = proto::Pad {
            paired: true,
            trusted: true,
            connected: true,
            ..pad
        };
        state.visible.retain(|p| p.mac != paired.mac);
        state.visible.push(paired.clone());
        Ok(proto::PadPairResult::Paired { pad: paired })
    }

    async fn forget(&self, mac: &str) -> PadResult<proto::PadForgetResult> {
        let mut state = self.inner.lock().await;
        let before = state.visible.len();
        // `RemoveDevice` drops the object, not just the keys, so the pad is no longer visible at all
        // until something rediscovers it.
        state.visible.retain(|p| !p.mac.eq_ignore_ascii_case(mac));
        Ok(proto::PadForgetResult {
            removed: state.visible.len() != before,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The signal that actually identifies an Xbox controller on this board. BlueZ sets `Icon`
    /// from the class or the appearance, so when it is there it is the answer.
    #[test]
    fn bluez_own_classification_is_enough() {
        assert!(looks_like_a_gamepad("", Some("input-gaming"), None, None));
    }

    /// A classic pad, identified by class-of-device with no icon and no name — which is what
    /// discovery reports before a device is queried.
    #[test]
    fn a_peripheral_joystick_class_is_a_gamepad() {
        // Major 0x05 (peripheral), minor 0x01 (joystick): 0x000504.
        assert!(looks_like_a_gamepad("", None, Some(0x000504), None));
        // Minor 0x02, gamepad: 0x000508.
        assert!(looks_like_a_gamepad("", None, Some(0x000508), None));
    }

    /// The direction that matters more. A keyboard and a mouse are peripherals too, and pairing
    /// one as a gamepad would bond the robot to whatever is on the desk next to it.
    #[test]
    fn keyboards_and_mice_are_not_gamepads() {
        // Major 0x05, minor 0x10 — keyboard.
        assert!(!looks_like_a_gamepad("", None, Some(0x000540), None));
        // Major 0x05, minor 0x20 — pointing device.
        assert!(!looks_like_a_gamepad("", None, Some(0x000580), None));
        // Audio/video major class: headphones, which are the other thing in the room in pairing
        // mode.
        assert!(!looks_like_a_gamepad(
            "WH-1000XM4",
            Some("audio-headset"),
            Some(0x240418),
            None
        ));
    }

    /// BLE appearance 0x03C4 is Gamepad. The rest of HID category 15 is not — a Bluetooth keyboard
    /// advertises 0x03C1.
    #[test]
    fn only_the_gamepad_appearance_counts() {
        assert!(looks_like_a_gamepad("", None, None, Some(0x03C4)));
        assert!(!looks_like_a_gamepad("", None, None, Some(0x03C1)));
    }

    /// The fallback, for a pad advertising nothing but a name.
    #[test]
    fn a_pad_is_recognised_by_name_when_nothing_else_is_set() {
        for name in [
            "Xbox Wireless Controller",
            "8BitDo Pro 2 gamepad",
            "DualSense Wireless Controller",
            "Logitech Joystick",
        ] {
            assert!(looks_like_a_gamepad(name, None, None, None), "{name}");
        }
        assert!(!looks_like_a_gamepad("Pierre's iPhone", None, None, None));
    }

    /// The whole arc, over the fake: pair the pad that is in pairing mode, see it bonded and
    /// trusted, forget it.
    #[tokio::test]
    async fn pairing_bonds_the_pad_and_forgetting_removes_it() {
        let pads = FakePads::new();
        assert!(pads.status().await.unwrap().is_empty());

        let result = pads.pair(None, DEFAULT_PAIR_TIMEOUT).await.unwrap();
        let proto::PadPairResult::Paired { pad } = result else {
            panic!("{result:?}");
        };
        // Trusted, not merely paired. A paired-but-untrusted pad looks right and does not
        // reconnect after a reboot, which is the failure this contract exists to prevent.
        assert!(pad.paired && pad.trusted, "{pad:?}");

        assert_eq!(pads.status().await.unwrap().len(), 1);
        assert!(pads.forget(&pad.mac).await.unwrap().removed);
        // Forgetting again is not an error, and a client must not present it as one.
        assert!(!pads.forget(&pad.mac).await.unwrap().removed);
        assert!(pads.status().await.unwrap().is_empty());
    }

    /// No pad in pairing mode is the most common failure by far, and it has to be distinguishable
    /// from "something broke" — the answer to it is "hold the sync button", not "file a bug".
    #[tokio::test]
    async fn an_absent_pad_is_not_found_rather_than_an_error() {
        let pads = FakePads::with(Vec::new());
        assert!(matches!(
            pads.pair(None, DEFAULT_PAIR_TIMEOUT).await.unwrap(),
            proto::PadPairResult::Failed {
                reason: proto::PadPairFailure::NotFound,
                ..
            }
        ));
    }

    /// **A pad already bonded must not block pairing a new one.**
    ///
    /// The bonded pad is in range and in every sweep, so a selection rule that took the first match
    /// would keep answering "you already have a pad" to someone holding a second one in pairing
    /// mode — and the only workaround would be to forget the working pad first, which is a bad
    /// trade for a robot two people want to drive.
    #[tokio::test]
    async fn a_bonded_pad_does_not_block_pairing_a_new_one() {
        let pads = FakePads::with(vec![
            bonded("78:86:2E:BB:13:28", "Xbox Wireless Controller"),
            unpaired("A4:AE:11:00:22:33", "DualSense Wireless Controller"),
        ]);

        let result = pads.pair(None, DEFAULT_PAIR_TIMEOUT).await.unwrap();
        let proto::PadPairResult::Paired { pad } = result else {
            panic!("{result:?}");
        };
        assert_eq!(pad.mac, "A4:AE:11:00:22:33", "the new pad must win");

        // And the robot now has both. `padd` drives whichever connects.
        let bonded_now = pads.status().await.unwrap();
        assert_eq!(bonded_now.len(), 2, "{bonded_now:?}");
    }

    /// Re-running with nothing new in pairing mode answers with the pad already bonded rather than
    /// failing. That is what repairs a lost `Trusted`, and what makes the command idempotent.
    #[tokio::test]
    async fn re_running_with_nothing_new_reports_the_pad_already_bonded() {
        let mut untrusted = bonded("78:86:2E:BB:13:28", "Xbox Wireless Controller");
        untrusted.trusted = false;
        let pads = FakePads::with(vec![untrusted]);

        let result = pads.pair(None, DEFAULT_PAIR_TIMEOUT).await.unwrap();
        let proto::PadPairResult::Paired { pad } = result else {
            panic!("{result:?}");
        };
        assert_eq!(pad.mac, "78:86:2E:BB:13:28");
        assert!(pad.trusted, "the re-run must restore trust: {pad:?}");
    }

    /// Two pads in pairing mode: refuse rather than guess, because guessing bonds the robot to
    /// whichever one BlueZ happened to report first.
    #[tokio::test]
    async fn two_pads_in_pairing_mode_are_refused_not_guessed() {
        let pads = FakePads::with(vec![
            unpaired("78:86:2E:BB:13:28", "Xbox Wireless Controller"),
            unpaired("A4:AE:11:00:22:33", "DualSense Wireless Controller"),
        ]);

        assert!(matches!(
            pads.pair(None, DEFAULT_PAIR_TIMEOUT).await.unwrap(),
            proto::PadPairResult::Failed {
                reason: proto::PadPairFailure::Ambiguous,
                ..
            }
        ));

        // And naming one resolves it, which is what the refusal tells the caller to do.
        let result = pads
            .pair(Some("a4:ae:11:00:22:33"), DEFAULT_PAIR_TIMEOUT)
            .await
            .unwrap();
        let proto::PadPairResult::Paired { pad } = result else {
            panic!("{result:?}");
        };
        assert_eq!(pad.mac, "A4:AE:11:00:22:33");
    }

    /// A caller cannot hold the adapter in discovery for as long as it likes, and zero means "look
    /// once" rather than "look forever" — a scripted retry needs to be able to ask.
    #[test]
    fn a_requested_timeout_is_clamped() {
        assert_eq!(pair_timeout(None), DEFAULT_PAIR_TIMEOUT);
        assert_eq!(pair_timeout(Some(30)), Duration::from_secs(30));
        assert_eq!(pair_timeout(Some(0)), Duration::ZERO);
        assert_eq!(pair_timeout(Some(9_999)), MAX_PAIR_TIMEOUT);
    }
}
