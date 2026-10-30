//! Per-device identity, and the default name derived from it.
//!
//! Three friends with three robots in one room must each reach *theirs*. Every board flashed from
//! one image used to advertise `radxa-zero3`, so a phone could not tell them apart — and picking
//! the wrong one means writing your wifi credentials into someone else's robot. The fix is that a
//! robot is distinguishable **before anyone renames anything**, which needs something per-board to
//! hang a name off.
//!
//! **The SoC serial, not the Bluetooth address.** The obvious candidate was the adapter address,
//! since a peer already sees it at the link layer, so a name derived from it leaks nothing new.
//! Measured on a board, it does not hold still: `btd`'s startup line recorded
//! `50:37:CD:16:2B:EC` and then `50:37:CD:16:1B:92` across sixteen boots, with no reflash in
//! between and nothing but BLE connections and gamepad pairing done to it. An identity that
//! changes for reasons nobody can name is not an identity. (That wandering is a bug in its own
//! right — an address change orphans every bond under `/var/lib/bluetooth/<address>/` and
//! invalidates the peripheral identifier a phone saved — but it is not this module's problem.)
//!
//! The serial comes out of the SoC's one-time-programmable fuses, so it survives a reflash,
//! survives swapping the radio module, and needs no provisioning step to exist — which is what makes
//! a hand-flashed board work. It is also readable *immediately*: a plain file, no root, no D-Bus, no
//! waiting the ~73 seconds it takes `hci0` to appear on this board. Which file depends on the board,
//! and [`robotd_params::serial`] says why.

use sha2::{Digest, Sha256};

/// The prefix every derived name carries.
///
/// Hardcoded rather than built from the hostname, which would give `radxa-zero3-7f3a` — longer
/// than what it replaces and no more meaningful. The robot is a duck.
const NAME_PREFIX: &str = "duck";

/// The board's serial number, or `None` where there is not one to read.
///
/// `None` on a macOS laptop, and on any board whose bootloader does not fill the property in. The
/// caller falls back rather than failing: a robot with no serial must still come up with a working
/// name.
pub fn serial() -> Option<String> {
    robotd_params::serial::soc_serial()
}

/// The name a robot answers to before anyone has renamed it: `duck-7f3a`.
///
/// **Hashed rather than sliced.** Taking the last four characters of the serial would assume that
/// is where two chips differ, and nothing guarantees it — a vendor is free to make any part of the
/// value sequential, or constant. A digest spreads whatever entropy the serial has across the
/// whole output.
///
/// **SHA-256 rather than [`std::hash::DefaultHasher`]**, whose output is explicitly not stable
/// across Rust releases. A `rustup update` would silently rename every robot in the field, and
/// nobody would ever connect the two.
///
/// Four hex characters, so 65 536 possibilities: three robots in a room share a name about once in
/// 22 000 times. This is a *default* meant to be distinguishable, not a unique key — `system.setName`
/// from a phone and `robotctl system set-name` from provisioning both override it, and renaming is
/// the escape hatch on the rare collision.
pub fn default_name(serial: &str) -> String {
    let digest = Sha256::digest(serial.as_bytes());
    format!("{NAME_PREFIX}-{:02x}{:02x}", digest[0], digest[1])
}

/// The beta board's ID EEPROM (FT24C32A on I2C1 at 0x50), as the at24 driver exposes it.
///
/// Root-only, which configd is. A Zero 3W has no such chip, so the file is absent there and the
/// robot keeps the factory PIN.
pub const LABEL_EEPROM_PATH: &str = "/sys/bus/i2c/devices/1-0050/eeprom";

/// How many decimal digits the factory writes: SKU (9) + year (2) + week (2) + unit (5), e.g.
/// `202004741` `26` `40` `00001`, which is also the serial printed on the label.
const LABEL_SERIAL_DIGITS: usize = 18;

/// The serial printed on the robot's label, as the factory wrote it into the ID EEPROM.
///
/// `None` where there is no EEPROM (a Zero 3W, a laptop) or nothing valid in it: a blank chip
/// reads all 0xFF, which is not a digit, and an all-zero one is not a serial.
pub fn label_serial() -> Option<String> {
    let raw = std::fs::read(LABEL_EEPROM_PATH).ok()?;
    decode_label_serial(&raw)
}

/// Decode the label serial from the start of the EEPROM.
///
/// **Assumed: 18 ASCII digits from address 0**, one byte per digit, exactly as on the label.
/// Seeed (2026-10-29) gave only "starting at address 0" and the field order, not the encoding;
/// this is pending their answer. If the first programmed board says otherwise, this function is
/// the only thing to change; read one with `xxd /sys/bus/i2c/devices/1-0050/eeprom | head -2`.
///
/// Anything but 18 digits is rejected rather than guessed at, so a binary encoding shows up as
/// "no label serial" and the factory PIN, never as a PIN that is not on the label.
pub fn decode_label_serial(raw: &[u8]) -> Option<String> {
    let text = raw.get(..LABEL_SERIAL_DIGITS)?;
    let serial = text
        .iter()
        .all(u8::is_ascii_digit)
        .then(|| String::from_utf8_lossy(text).into_owned())?;
    serial.bytes().any(|b| b != b'0').then_some(serial)
}

/// The pairing PIN a robot ships with: the last six digits of its label serial, as on Reachy
/// Mini, so the user reads it off the label.
///
/// Six because that is [`crate::store::PIN_DIGITS`]. Note what that covers: the last digit of the
/// week and the five-digit unit number. If the unit number restarts every week, robots made ten
/// weeks apart share a PIN. It still proves the user is holding *this* robot's label in a room of
/// robots, which is what a default PIN is for; `set-pin` remains the way to a secret one.
pub fn default_pin(label_serial: &str) -> Option<String> {
    let start = label_serial.len().checked_sub(crate::store::PIN_DIGITS)?;
    Some(label_serial[start..].to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The name has to be stable for the life of the board: it is what a phone remembers, and a
    /// robot that renames itself is one a kid has to find again. This pins the exact output for
    /// the serial the board actually has, so a change to the derivation fails here rather than in
    /// someone's Bluetooth list.
    #[test]
    fn the_name_derived_from_this_boards_serial_is_pinned() {
        assert_eq!(default_name("bb7b734a7717ac41"), "duck-c51b");
    }

    /// Two beta boards from one lot, which both came up as `duck-d041` while their name was made
    /// from U-Boot's serial (`8b33c6e86a6a0c15` on both). The kernel's serials tell them apart.
    #[test]
    fn two_beta_boards_from_one_lot_get_two_names() {
        assert_eq!(default_name("8b33c6e86a6a0c15"), "duck-d041");
        assert_eq!(default_name("62285dd716c88f0f"), "duck-2b6d");
        assert_ne!(
            default_name("62285dd716c88f0f"),
            default_name("9ed5ef3c1c5ff3f1")
        );
    }

    /// Two boards must not collide, which is the entire point.
    #[test]
    fn different_serials_give_different_names() {
        assert_ne!(
            default_name("bb7b734a7717ac41"),
            default_name("bb7b734a7717ac42")
        );
    }

    /// A one-character difference in the serial has to move the name, which slicing the last
    /// characters off a sequential serial would not guarantee.
    #[test]
    fn a_name_is_shaped_like_a_name() {
        let name = default_name("bb7b734a7717ac41");
        let suffix = name.strip_prefix("duck-").expect("the prefix");
        assert_eq!(suffix.len(), 4, "{name}");
        assert!(suffix.chars().all(|c| c.is_ascii_hexdigit()), "{name}");
        assert!(name.len() <= crate::store::MAX_NAME, "{name}");
    }

    /// Seeed's example, `202004741` `26` `40` `00001`.
    const SEEED_EXAMPLE: &[u8] = b"202004741264000001";

    #[test]
    fn the_label_serial_is_read_as_text() {
        // The rest of the 4 KiB chip follows, erased; only the first 18 bytes are the serial.
        let mut eeprom = SEEED_EXAMPLE.to_vec();
        eeprom.resize(4096, 0xff);
        assert_eq!(
            decode_label_serial(&eeprom).as_deref(),
            Some("202004741264000001")
        );
    }

    /// A blank chip, a zeroed one and a short read are all "no serial", never a PIN of `ffffff`.
    #[test]
    fn an_unprogrammed_eeprom_has_no_label_serial() {
        assert_eq!(decode_label_serial(&[0xff; 4096]), None);
        assert_eq!(decode_label_serial(&[0x00; 4096]), None);
        assert_eq!(decode_label_serial(&SEEED_EXAMPLE[..17]), None);
        assert_eq!(decode_label_serial(b"000000000000000000"), None);
    }

    /// The same serial in packed BCD is refused, not misread: if Seeed writes binary, the robot
    /// falls back to the factory PIN instead of inventing one that is not on the label.
    #[test]
    fn a_binary_label_serial_is_refused() {
        let mut eeprom = vec![0x20, 0x20, 0x04, 0x74, 0x12, 0x64, 0x00, 0x00, 0x01];
        eeprom.resize(4096, 0xff);
        assert_eq!(decode_label_serial(&eeprom), None);
    }

    /// Leading zeros are part of the PIN: `000001` is not `1`.
    #[test]
    fn the_default_pin_is_the_last_six_digits_of_the_label_serial() {
        assert_eq!(default_pin("202004741264000001").as_deref(), Some("000001"));
        assert_eq!(default_pin("12345"), None);
    }
}
