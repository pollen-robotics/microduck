//! The robot's ID EEPROM: what the factory writes into it, and reading it back.
//!
//! The factory programs it with an external programmer before the robot leaves, as plain ASCII:
//!
//! ```text
//! offset 0    18 digits                  the label serial: SKU (9) + year (2) + week (2) + unit (5)
//! offset 18   :                          separator
//! offset 19   md1:<first name>:<64 hex>  the setup code, ending at the first NUL or 0xFF
//! ```
//!
//! The whole line, `<label serial>:md1:<first name>:<64 hex>`, is also the QR printed on the robot.
//! The setup code is `md1` (microduck setup code, format 1), the robot's first name
//! ([`crate::serial::first_name`]), and its 32-byte key. Holding the label is what proves somebody
//! has the robot in hand, so the key must be random — not derived from the serial or anything else
//! printed on it (`factory-provisioning.md`).
//!
//! Kept in the EEPROM rather than on the data partition so that a full reflash or a factory reset
//! keeps it, and the printed QR stays valid for the robot's life. The at24 driver exposes the chip
//! read-only, even to root, so software on the robot can read the key but not replace it.

/// The beta and face-board-v1 ID EEPROM (FT24C32A on I2C1 at 0x50), as the at24 driver exposes
/// it. Root-only. A Zero 3W has no such chip, so the file is absent there.
pub const EEPROM_PATH: &str = "/sys/bus/i2c/devices/1-0050/eeprom";

/// The label serial's length, from offset 0.
pub const LABEL_SERIAL_LEN: usize = 18;

/// What separates the label serial from the setup code, in the EEPROM and in the QR.
pub const SEPARATOR: u8 = b':';

/// Where the setup code starts: after the label serial and the separator.
pub const SETUP_CODE_OFFSET: usize = LABEL_SERIAL_LEN + 1;

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

/// The label serial in an EEPROM image: 18 ASCII digits from offset 0. `None` for a blank chip
/// (0xFF is not a digit) and for an all-zero one, which is not a serial.
pub fn label_serial_in(raw: &[u8]) -> Option<String> {
    let text = raw.get(..LABEL_SERIAL_LEN)?;
    (text.iter().all(u8::is_ascii_digit) && text.iter().any(|&b| b != b'0'))
        .then(|| String::from_utf8_lossy(text).into_owned())
}

/// The line printed as the QR: the label serial, the separator and the setup code — or the code
/// alone on a board with no label serial.
pub fn qr_line(label_serial: Option<&str>, code: &SetupCode) -> String {
    match label_serial {
        Some(serial) => format!("{serial}{}{}", SEPARATOR as char, code.text()),
        None => code.text(),
    }
}

/// The setup code in an EEPROM image: `Ok(None)` when it holds none (a blank byte after the label
/// serial, as on a robot the factory has not provisioned yet, or a dev board), `Err` when
/// something is written there that is not the separator and a valid code.
pub fn setup_code_in(raw: &[u8]) -> Result<Option<SetupCode>, String> {
    match raw.get(LABEL_SERIAL_LEN) {
        None | Some(0x00 | 0xff) => return Ok(None),
        Some(&b) if b == SEPARATOR => {}
        Some(_) => {
            return Err(format!(
                "byte {LABEL_SERIAL_LEN} of the EEPROM should be `{}` after the label serial",
                SEPARATOR as char
            ));
        }
    }
    let area = raw.get(SETUP_CODE_OFFSET..).unwrap_or_default();
    let end = area
        .iter()
        .position(|&b| b == 0x00 || b == 0xff)
        .unwrap_or(area.len());
    if end == 0 {
        return Err(
            "the EEPROM has the separator after the label serial but no setup code".to_owned(),
        );
    }
    let text = std::str::from_utf8(&area[..end])
        .map_err(|_| "the setup code in the EEPROM is not text".to_owned())?;
    SetupCode::parse(text.trim()).map(Some)
}

/// This robot's EEPROM image. `Ok(None)` where there is no EEPROM; an unreadable chip that exists
/// is an error, since a root-only file failing to read is the caller not being root.
pub fn read() -> Result<Option<Vec<u8>>, String> {
    match std::fs::read(EEPROM_PATH) {
        Ok(raw) => Ok(Some(raw)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(format!("cannot read {EEPROM_PATH}: {e}")),
    }
}

/// This robot's setup code from its EEPROM, `Ok(None)` where there is no EEPROM or no code in it.
pub fn setup_code() -> Result<Option<SetupCode>, String> {
    read()?.map_or(Ok(None), |raw| setup_code_in(&raw))
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: &str = "71717171717171717171717171717171717171717171717171717171717171aa";

    /// A 4 KiB image: the label serial, then `after_serial` from offset 18, then blank.
    fn image(after_serial: &[u8]) -> Vec<u8> {
        let mut raw = b"202004743264300098".to_vec();
        raw.extend_from_slice(after_serial);
        raw.resize(4096, 0xff);
        raw
    }

    /// As the factory writes it: the separator, then the code.
    fn eeprom(code: &[u8]) -> Vec<u8> {
        let mut after = vec![SEPARATOR];
        after.extend_from_slice(code);
        image(&after)
    }

    #[test]
    fn the_qr_line_is_the_eeprom_line() {
        let code = SetupCode::parse(&format!("md1:duck-ccf2:{KEY}")).unwrap();
        let raw = eeprom(code.text().as_bytes());
        let line = qr_line(label_serial_in(&raw).as_deref(), &code);
        assert_eq!(line, format!("202004743264300098:md1:duck-ccf2:{KEY}"));
        assert_eq!(line.as_bytes(), &raw[..line.len()]);
        assert_eq!(raw[line.len()], 0xff);
        assert_eq!(qr_line(None, &code), code.text());
    }

    #[test]
    fn a_blank_or_zero_label_serial_is_none() {
        assert_eq!(label_serial_in(&[0xff; 4096]), None);
        assert_eq!(label_serial_in(&[b'0'; 18]), None);
        assert_eq!(label_serial_in(b"20200474326430009"), None);
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
        assert_eq!(setup_code_in(&image(b"")).unwrap(), None);
        assert_eq!(setup_code_in(&[0xff; 4096]).unwrap(), None);
        assert_eq!(setup_code_in(b"202004743264300098").unwrap(), None);
        assert_eq!(setup_code_in(&image(b"\0")).unwrap(), None);
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
        // The code without its separator is the old layout, not a code one byte late.
        assert!(setup_code_in(&image(format!("md1:duck-ccf2:{KEY}").as_bytes())).is_err());
        // A separator with nothing after it is a half-written chip.
        assert!(setup_code_in(&eeprom(b"")).is_err());
    }
}
