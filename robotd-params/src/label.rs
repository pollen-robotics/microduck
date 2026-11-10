//! The robot's ID EEPROM: what the factory writes into it, and reading it back.
//!
//! The factory programs it with an external programmer before the robot leaves, as plain ASCII:
//!
//! ```text
//! offset 0    18 digits                  the label serial: SKU (9) + year (2) + week (2) + unit (5)
//! offset 18   md1:<first name>:<64 hex>  the setup code, ending at the first NUL or 0xFF
//! ```
//!
//! The setup code is the same string as the QR printed on the robot: `md1` (microduck setup code,
//! format 1), the robot's first name ([`crate::serial::first_name`]), and its 32-byte key. Holding
//! the label is what proves somebody has the robot in hand, so the key must be random — not derived
//! from the serial or anything else printed on it (`factory-provisioning.md`).
//!
//! Kept in the EEPROM rather than on the data partition so that a full reflash or a factory reset
//! keeps it, and the printed QR stays valid for the robot's life. The at24 driver exposes the chip
//! read-only, even to root, so software on the robot can read the key but not replace it.

/// The beta and face-board-v1 ID EEPROM (FT24C32A on I2C1 at 0x50), as the at24 driver exposes
/// it. Root-only. A Zero 3W has no such chip, so the file is absent there.
pub const EEPROM_PATH: &str = "/sys/bus/i2c/devices/1-0050/eeprom";

/// Where the setup code starts: right after the 18-digit label serial.
pub const SETUP_CODE_OFFSET: usize = 18;

/// A setup code: the id the secure handshake binds, and the key as 64 lowercase hex characters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetupCode {
    pub id: String,
    pub key: String,
}

impl SetupCode {
    /// Parse `md1:<id>:<64 hex>`, the form the app reads and the QR carries.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut fields = text.split(':');
        if fields.next() != Some("md1") {
            return Err("a setup code starts with `md1:`".to_owned());
        }
        let (Some(id), Some(key), None) = (fields.next(), fields.next(), fields.next()) else {
            return Err("a setup code is `md1:<robot id>:<key>`".to_owned());
        };
        let id_ok = !id.is_empty()
            && id.len() <= 128
            && id
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-'));
        if !id_ok {
            return Err(format!("`{id}` is not a valid robot id"));
        }
        if key.len() != 64 || !key.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err("the key must be 64 hex characters (32 bytes)".to_owned());
        }
        Ok(Self {
            id: id.to_owned(),
            key: key.to_ascii_lowercase(),
        })
    }

    /// Back to `md1:<id>:<key>`.
    pub fn text(&self) -> String {
        format!("md1:{}:{}", self.id, self.key)
    }
}

/// The setup code in an EEPROM image: `Ok(None)` when that area is blank (all 0xFF or NUL, as on
/// a robot the factory has not provisioned yet, or a dev board), `Err` when something is written
/// there that is not a valid code.
pub fn setup_code_in(raw: &[u8]) -> Result<Option<SetupCode>, String> {
    let area = raw.get(SETUP_CODE_OFFSET..).unwrap_or_default();
    let end = area
        .iter()
        .position(|&b| b == 0x00 || b == 0xff)
        .unwrap_or(area.len());
    if end == 0 {
        return Ok(None);
    }
    let text = std::str::from_utf8(&area[..end])
        .map_err(|_| "the setup code in the EEPROM is not text".to_owned())?;
    SetupCode::parse(text.trim()).map(Some)
}

/// This robot's setup code from its EEPROM. `Ok(None)` where there is no EEPROM or nothing in that
/// area; an unreadable chip that exists is an error, since a root-only file failing to read is the
/// caller not being root.
pub fn setup_code() -> Result<Option<SetupCode>, String> {
    match std::fs::read(EEPROM_PATH) {
        Ok(raw) => setup_code_in(&raw),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(format!("cannot read {EEPROM_PATH}: {e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: &str = "71717171717171717171717171717171717171717171717171717171717171aa";

    fn eeprom(after_serial: &[u8]) -> Vec<u8> {
        let mut raw = b"202004743264300098".to_vec();
        raw.extend_from_slice(after_serial);
        raw.resize(4096, 0xff);
        raw
    }

    #[test]
    fn a_programmed_eeprom_yields_its_code_after_the_label_serial() {
        let code = format!("md1:duck-ccf2:{}", KEY.to_uppercase());
        let found = setup_code_in(&eeprom(code.as_bytes())).unwrap().unwrap();
        assert_eq!(found.id, "duck-ccf2");
        assert_eq!(found.key, KEY);
        assert_eq!(found.text(), format!("md1:duck-ccf2:{KEY}"));
    }

    #[test]
    fn a_nul_ends_the_code_as_well_as_blank_bytes() {
        let mut bytes = format!("md1:duck-ccf2:{KEY}").into_bytes();
        bytes.extend_from_slice(b"\0garbage");
        assert!(setup_code_in(&eeprom(&bytes)).unwrap().is_some());
    }

    #[test]
    fn a_blank_area_or_short_chip_has_no_code() {
        assert_eq!(setup_code_in(&eeprom(b"")).unwrap(), None);
        assert_eq!(setup_code_in(&[0xff; 4096]).unwrap(), None);
        assert_eq!(setup_code_in(b"202004743264300098").unwrap(), None);
        assert_eq!(setup_code_in(&eeprom(b"\0")).unwrap(), None);
    }

    #[test]
    fn something_written_that_is_not_a_code_is_an_error_not_silence() {
        assert!(setup_code_in(&eeprom(b"hello")).is_err());
        assert!(
            setup_code_in(&eeprom(format!("md1:duck-ccf2:{}", &KEY[..62]).as_bytes())).is_err()
        );
        assert!(setup_code_in(&eeprom(format!("md2:duck-ccf2:{KEY}").as_bytes())).is_err());
        assert!(setup_code_in(&eeprom(format!("md1:duck ccf2:{KEY}").as_bytes())).is_err());
        assert!(setup_code_in(&eeprom(&[b'm', b'd', 0xc3, 0x28])).is_err());
    }
}
