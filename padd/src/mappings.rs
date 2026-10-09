//! SDL mappings `gilrs` does not ship, for a pad it does know.
//!
//! ## Why a working pad can lose its right stick and both triggers
//!
//! `gilrs` decides which evdev axis is which stick from an SDL mapping, found by the pad's GUID. On
//! Linux that GUID is the bus, the vendor, the product **and the firmware version** — so the
//! bundled database lists the Xbox Wireless Controller over Bluetooth once per firmware it has
//! seen: 5.01, 5.05, 5.07, 5.09, 5.11, 5.13, 5.15, 5.17 and 5.22. A pad on a firmware not in that
//! list matches nothing, and `gilrs` quietly falls back to a default layout in which this pad's
//! right stick (`ABS_Z`, `ABS_RZ`) reads as a trigger and its triggers (`ABS_GAS`, `ABS_BRAKE`) as
//! nothing. The left stick and every button still work, which is what makes it look like a pad that
//! is simply not very responsive, and the only trace is one `No mapping found for UUID` line from
//! `gilrs`, with no hint of which pad or why that matters.
//!
//! Xbox firmware 5.23, current since early 2025, is exactly that case: `padd` drove it without the
//! turn stick or the mouth triggers.
//!
//! ## What this does about it
//!
//! Every Bluetooth Xbox Series pad in the bundled database has the **same** layout from 5.07 on,
//! with or without the Share button, so the missing versions can be given that layout. Rather than
//! chase each new version, this hands `gilrs` one entry for every 5.xx firmware. It matters that the
//! entries go in *before* the bundled database: `gilrs` inserts mappings into one table keyed by
//! GUID and the bundled ones are added when it is built, so wherever the database already knows a
//! version, its entry wins and this adds nothing.
//!
//! The Share button is `misc1:b15` in the layouts that have it; an entry that names a button the
//! pad does not report is harmless, which is why one layout serves them all.
//!
//! Only the Bluetooth bus (`0005`) is covered. The same pad on USB is a different driver with its
//! own numbering, and nobody has verified one of those here.

/// The pad's layout, copied from the bundled database's Bluetooth entries for the Xbox Series
/// controller (5.07 – 5.22), indices as `gilrs-core` enumerates this pad's evdev codes in order:
/// axes `a0`/`a1` the left stick, `a2`/`a3` the right (`ABS_Z`, `ABS_RZ`), `a4` the right trigger
/// (`ABS_GAS`), `a5` the left (`ABS_BRAKE`).
const XBOX_SERIES_BLE_LAYOUT: &str = "a:b0,b:b1,back:b10,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,\
dpup:h0.1,guide:b12,leftshoulder:b6,leftstick:b13,lefttrigger:a5,leftx:a0,lefty:a1,misc1:b15,\
rightshoulder:b7,rightstick:b14,righttrigger:a4,rightx:a2,righty:a3,start:b11,x:b3,y:b4,\
platform:Linux,";

/// Microsoft, and the Xbox Wireless Controller (Series X|S, model 1914) over Bluetooth LE.
const VENDOR: u16 = 0x045e;
const PRODUCT: u16 = 0x0b13;
/// The Linux input bus number for Bluetooth, which `gilrs` puts first in a GUID.
const BUS_BLUETOOTH: u16 = 0x0005;

/// The firmware `0x05xx` field the pad reports as its input version. Microsoft numbers it as the
/// release written out in hex digits — 5.23 is `0x0523`, 5.09 is `0x0509` — so the minor goes in
/// as two decimal digits read as BCD.
fn firmware_field(major: u16, minor: u16) -> u16 {
    let bcd = ((minor / 10) << 4) | (minor % 10);
    (major << 8) | bcd
}

/// A GUID in SDL's text form, which is how a mapping names its pad: bus, CRC (zero for Bluetooth
/// pads), vendor, product, version — each a little-endian `u16` followed by two zero bytes except
/// the bus and CRC pair.
fn sdl_guid(bus: u16, vendor: u16, product: u16, version: u16) -> String {
    let le = |value: u16| format!("{:02x}{:02x}", value & 0xff, value >> 8);
    format!(
        "{}0000{}0000{}0000{}0000",
        le(bus),
        le(vendor),
        le(product),
        le(version)
    )
}

/// A connected pad's GUID as `gilrs` reports it (`Gamepad::uuid`), in the form a mapping line
/// starts with. It is what to paste into an `SDL_GAMECONTROLLERCONFIG` entry for a pad this module
/// does not cover, and what the journal line for a pad without a mapping carries.
pub fn guid_text(uuid: [u8; 16]) -> String {
    uuid.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// One SDL mapping line per firmware release 5.00 – 5.99 of the Bluetooth Xbox Series pad, for
/// `GilrsBuilder::add_mappings`.
pub fn xbox_series_ble() -> String {
    (0..100)
        .map(|minor| {
            let guid = sdl_guid(BUS_BLUETOOTH, VENDOR, PRODUCT, firmware_field(5, minor));
            format!("{guid},Xbox Series Controller,{XBOX_SERIES_BLE_LAYOUT}\n")
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The version field is the release's own digits, so 5.23 — the pad this exists for — and the
    /// 5.09 the bundled database already knows are both what `/proc/bus/input/devices` prints.
    #[test]
    fn the_firmware_field_is_the_release_written_in_hex_digits() {
        assert_eq!(firmware_field(5, 23), 0x0523);
        assert_eq!(firmware_field(5, 9), 0x0509);
        assert_eq!(firmware_field(5, 0), 0x0500);
        assert_eq!(firmware_field(5, 99), 0x0599);
    }

    /// The GUID `gilrs` logged for the pad that sent us here — `05000000-5e04-0000-130b-000023050000`
    /// — and the one the bundled database has for 5.09, to the byte. A byte-order slip here would
    /// produce 100 entries that match no pad at all, with nothing to say so.
    #[test]
    fn a_guid_matches_the_form_gilrs_logs_and_the_bundled_database_uses() {
        assert_eq!(
            sdl_guid(BUS_BLUETOOTH, VENDOR, PRODUCT, 0x0523),
            "050000005e040000130b000023050000"
        );
        assert_eq!(
            sdl_guid(BUS_BLUETOOTH, VENDOR, PRODUCT, 0x0509),
            "050000005e040000130b000009050000"
        );
    }

    /// `Gamepad::uuid` is those same sixteen bytes, and what a person pastes into a mapping.
    #[test]
    fn a_connected_pads_guid_is_printed_the_way_a_mapping_names_it() {
        let uuid = [
            0x05, 0x00, 0x00, 0x00, 0x5e, 0x04, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x00, 0x23, 0x05,
            0x00, 0x00,
        ];
        assert_eq!(guid_text(uuid), "050000005e040000130b000023050000");
    }

    /// Every release from 5.00 to 5.99 gets one line, 5.23 among them, each a well-formed SDL
    /// mapping that `gilrs` will accept: a 32-digit GUID, a name, then `key:value` pairs ending in
    /// the platform. `gilrs` drops a line it cannot read without a word, so this is the only place
    /// a typo in the layout would show.
    #[test]
    fn every_release_gets_a_well_formed_line() {
        let text = xbox_series_ble();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 100);
        assert!(
            lines
                .iter()
                .any(|line| line.starts_with("050000005e040000130b000023050000,")),
            "5.23"
        );
        let mut seen = std::collections::HashSet::new();
        for line in &lines {
            let mut fields = line.split(',');
            let guid = fields.next().unwrap();
            assert_eq!(guid.len(), 32, "{line}");
            assert!(guid.chars().all(|c| c.is_ascii_hexdigit()), "{line}");
            assert!(seen.insert(guid), "a duplicate GUID: {guid}");
            assert_eq!(fields.next(), Some("Xbox Series Controller"));
            let rest: Vec<&str> = fields.collect();
            assert_eq!(rest.last(), Some(&""), "ends with a trailing comma: {line}");
            assert_eq!(rest[rest.len() - 2], "platform:Linux");
            for pair in &rest[..rest.len() - 2] {
                let (key, value) = pair.split_once(':').expect(pair);
                assert!(!key.is_empty() && !value.is_empty(), "{pair}");
            }
        }
    }

    /// The thing the right stick and the triggers depend on, spelled out: the right stick is the
    /// third and fourth axes `gilrs-core` lists for this pad (`ABS_Z`, `ABS_RZ`) and the triggers
    /// the fifth and sixth (`ABS_GAS` right, `ABS_BRAKE` left). Measured on a pad on 5.23 —
    /// moving the right stick moved `ABS_Z` and `ABS_RZ`, and the triggers moved `ABS_BRAKE` and
    /// `ABS_GAS` — and the same as the bundled entries for 5.07 – 5.22.
    #[test]
    fn the_layout_puts_the_right_stick_and_triggers_where_the_pad_reports_them() {
        for pair in [
            "leftx:a0",
            "lefty:a1",
            "rightx:a2",
            "righty:a3",
            "righttrigger:a4",
            "lefttrigger:a5",
        ] {
            assert!(
                XBOX_SERIES_BLE_LAYOUT.split(',').any(|p| p == pair),
                "{pair}"
            );
        }
    }
}
