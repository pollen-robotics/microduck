//! NDEF, read-only and limited to text and URI records — and whether they ask to pair a gamepad,
//! and which one.
//!
//! Pure: no hardware, entirely testable off a board.

const TLV_NULL: u8 = 0x00;
const TLV_NDEF: u8 = 0x03;
const TLV_TERMINATOR: u8 = 0xFE;

const FLAG_ME: u8 = 0x40;
const FLAG_SR: u8 = 0x10;
const FLAG_IL: u8 = 0x08;

/// The NFC Forum abbreviation table: a URI record's first byte indexes it.
const URI_PREFIXES: [&str; 36] = [
    "",
    "http://www.",
    "https://www.",
    "http://",
    "https://",
    "tel:",
    "mailto:",
    "ftp://anonymous:anonymous@",
    "ftp://ftp.",
    "ftps://",
    "sftp://",
    "smb://",
    "nfs://",
    "ftp://",
    "dav://",
    "news:",
    "telnet://",
    "imap:",
    "rtsp://",
    "urn:",
    "pop:",
    "sip:",
    "sips:",
    "tftp:",
    "btspp://",
    "btl2cap://",
    "btgoep://",
    "tcpobex://",
    "irdaobex://",
    "file://",
    "urn:epc:id:",
    "urn:epc:tag:",
    "urn:epc:pat:",
    "urn:epc:raw:",
    "urn:epc:",
    "urn:nfc:",
];

/// A text or URI record's content, as a person wrote it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Record {
    Text(String),
    Uri(String),
}

impl Record {
    pub fn value(&self) -> &str {
        match self {
            Self::Text(s) | Self::Uri(s) => s,
        }
    }
}

/// The NDEF TLV's total size, terminator included, from its first bytes — or `None` when they
/// are not an NDEF TLV, in which case where the message ends is unknown and everything is read.
pub fn tlv_size(head: &[u8]) -> Option<usize> {
    match *head {
        [TLV_NDEF, 0xFF, hi, lo, ..] => Some(4 + u16::from_be_bytes([hi, lo]) as usize + 1),
        [TLV_NDEF, 0xFF, ..] => None,
        [TLV_NDEF, len, ..] => Some(2 + len as usize + 1),
        _ => None,
    }
}

/// The records of the NDEF message in a tag's user memory. Empty for a blank tag.
pub fn parse(data: &[u8]) -> Result<Vec<Record>, String> {
    let message = message(data)?;
    let mut records = Vec::new();
    let mut i = 0;
    while i < message.len() {
        let header = message[i];
        let byte = |at: usize| {
            message
                .get(at)
                .copied()
                .ok_or_else(|| "truncated record".to_owned())
        };
        let type_len = byte(i + 1)? as usize;
        i += 2;
        let payload_len = if header & FLAG_SR != 0 {
            i += 1;
            byte(i - 1)? as usize
        } else {
            let bytes = message
                .get(i..i + 4)
                .ok_or_else(|| "truncated record length".to_owned())?;
            i += 4;
            u32::from_be_bytes(bytes.try_into().unwrap()) as usize
        };
        let id_len = if header & FLAG_IL != 0 {
            i += 1;
            byte(i - 1)? as usize
        } else {
            0
        };
        let kind = message
            .get(i..i + type_len)
            .ok_or_else(|| "truncated record type".to_owned())?;
        i += type_len + id_len;
        let payload = message
            .get(i..i + payload_len)
            .ok_or_else(|| "truncated record payload".to_owned())?;
        i += payload_len;

        match (kind, payload.split_first()) {
            (b"T", Some((&status, rest))) => {
                let lang = (status & 0x3F) as usize;
                let text = rest.get(lang..).unwrap_or_default();
                records.push(Record::Text(String::from_utf8_lossy(text).into_owned()));
            }
            (b"U", Some((&prefix, rest))) => {
                let prefix = URI_PREFIXES.get(prefix as usize).copied().unwrap_or("");
                records.push(Record::Uri(format!(
                    "{prefix}{}",
                    String::from_utf8_lossy(rest)
                )));
            }
            // Anything else is out of scope and skipped, not an error.
            _ => {}
        }
        if header & FLAG_ME != 0 {
            break;
        }
    }
    Ok(records)
}

/// The NDEF TLV's payload, skipping NULL TLVs and any other TLV before it.
fn message(data: &[u8]) -> Result<&[u8], String> {
    let mut i = 0;
    while i < data.len() {
        let tlv = data[i];
        if tlv == TLV_TERMINATOR {
            break;
        }
        if tlv == TLV_NULL {
            i += 1;
            continue;
        }
        let (len, start) = match data.get(i + 1..) {
            Some([0xFF, hi, lo, ..]) => (u16::from_be_bytes([*hi, *lo]) as usize, i + 4),
            Some([len, ..]) if *len != 0xFF => (*len as usize, i + 2),
            _ => return Err("truncated TLV".into()),
        };
        let value = data.get(start..start + len).ok_or_else(|| {
            format!(
                "TLV says {len} bytes, {} are there",
                data.len().saturating_sub(start)
            )
        })?;
        if tlv == TLV_NDEF {
            return Ok(value);
        }
        i = start + len;
    }
    Err("no NDEF message: a blank or unformatted tag".into())
}

/// What every pairing tag starts with. On its own it names no pad; followed by an address it names
/// one — see [`pairing`].
pub const PAIRING: &str = "microduck_gamepad_pairing";

/// A pairing tag, read: which pad it asks for, if it names one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pairing {
    /// The pad's Bluetooth address, upper case and colon-separated, as BlueZ writes it. `None`
    /// for a tag that names no pad: the robot takes the one in pairing mode.
    pub mac: Option<String>,
}

/// Does one of these records ask to pair a gamepad, and which one?
///
/// A record is a pairing record when it is exactly [`PAIRING`], or [`PAIRING`], a space and a
/// Bluetooth address — `microduck_gamepad_pairing 78:86:2E:BB:13:28`. Surrounding whitespace
/// aside, nothing else counts: a tag that merely mentions it in other text is not a pairing tag,
/// and neither is one whose address does not parse, because pairing whatever pad is in range is
/// not what a tag naming a particular one asked for. An address on its own, as the first tags
/// had, does not pair anything either.
pub fn pairing(records: &[Record]) -> Option<Pairing> {
    records.iter().find_map(|r| {
        let rest = r.value().trim().strip_prefix(PAIRING)?;
        if rest.is_empty() {
            return Some(Pairing { mac: None });
        }
        if !rest.starts_with(char::is_whitespace) {
            return None;
        }
        Some(Pairing {
            mac: Some(mac(rest.trim_start())?),
        })
    })
}

/// Does one of these records ask to pair a gamepad, named or not?
pub fn asks_to_pair(records: &[Record]) -> bool {
    pairing(records).is_some()
}

/// Six hex octets separated by `:`, in BlueZ's form: upper case.
fn mac(text: &str) -> Option<String> {
    let octets: Vec<&str> = text.split(':').collect();
    let valid = octets.len() == 6
        && octets
            .iter()
            .all(|o| o.len() == 2 && o.bytes().all(|b| b.is_ascii_hexdigit()));
    valid.then(|| text.to_ascii_uppercase())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tag's user memory holding one text record, the way `ntag write --text` writes it.
    fn text_tag(text: &str) -> Vec<u8> {
        let mut payload = vec![2, b'f', b'r'];
        payload.extend(text.as_bytes());
        let mut record = vec![0xD1, 1, payload.len() as u8, b'T'];
        record.extend(payload);
        let mut tag = vec![TLV_NDEF, record.len() as u8];
        tag.extend(record);
        tag.push(TLV_TERMINATOR);
        tag.resize(64, 0);
        tag
    }

    #[test]
    fn a_text_record_reads_back_without_its_language() {
        let records = parse(&text_tag(PAIRING)).unwrap();
        assert_eq!(records, vec![Record::Text(PAIRING.into())]);
    }

    #[test]
    fn a_uri_record_gets_its_prefix_back() {
        let data = [TLV_NDEF, 7, 0xD1, 1, 3, b'U', 4, b'a', b'b', TLV_TERMINATOR];
        assert_eq!(
            parse(&data).unwrap(),
            vec![Record::Uri("https://ab".into())]
        );
    }

    #[test]
    fn the_tlv_size_is_known_from_the_first_bytes() {
        let tag = text_tag(PAIRING);
        assert_eq!(tlv_size(&tag), Some(2 + tag[1] as usize + 1));
        assert_eq!(tlv_size(&[0x00, 0x00]), None);
    }

    #[test]
    fn a_blank_tag_is_an_error_not_an_empty_address() {
        assert!(parse(&[0u8; 16]).is_err());
        assert_eq!(parse(&[TLV_NDEF, 0, TLV_TERMINATOR]).unwrap(), vec![]);
    }

    /// Phone NFC apps write a Lock Control TLV ahead of the record; the walk has to skip
    /// unknown TLVs by their length, which is what keeps such a tag readable.
    #[test]
    fn a_lock_tlv_ahead_of_the_message_is_walked_past() {
        let mut tag = vec![0x01, 0x03, 0xA0, 0x0C, 0x34]; // a phone app's lock TLV
        tag.extend(text_tag(PAIRING));
        assert!(asks_to_pair(&parse(&tag).unwrap()));
    }

    #[test]
    fn only_the_pairing_text_itself_asks_to_pair() {
        let asks = |r: Record| asks_to_pair(&[r]);
        assert!(asks(Record::Text(PAIRING.into())));
        assert!(asks(Record::Text(format!(" {PAIRING}\n"))));
        assert!(asks(Record::Uri(PAIRING.into())));
        // The old tags, an address and nothing else, no longer pair anything.
        assert!(!asks(Record::Text("98:B6:E9:28:06:09".into())));
        assert!(!asks(Record::Text(format!("not {PAIRING}"))));
        assert!(!asks(Record::Text("MICRODUCK_GAMEPAD_PAIRING".into())));
        assert!(!asks_to_pair(&[]));
    }

    #[test]
    fn a_pairing_tag_may_name_its_pad() {
        let read = |text: &str| pairing(&[Record::Text(text.into())]);
        let named = |mac: &str| {
            Some(Pairing {
                mac: Some(mac.into()),
            })
        };
        assert_eq!(read(PAIRING), Some(Pairing { mac: None }));
        assert_eq!(
            read(&format!("{PAIRING} 78:86:2E:BB:13:28")),
            named("78:86:2E:BB:13:28")
        );
        // Typed on a phone: lower case, a trailing newline.
        assert_eq!(
            read(&format!("{PAIRING} 78:86:2e:bb:13:28\n")),
            named("78:86:2E:BB:13:28")
        );
        assert_eq!(
            pairing(&[Record::Uri(format!("{PAIRING} 98:B6:E9:28:06:09"))]),
            named("98:B6:E9:28:06:09")
        );
    }

    #[test]
    fn an_address_that_does_not_parse_pairs_nothing() {
        let read = |text: &str| pairing(&[Record::Text(text.into())]);
        assert_eq!(read(&format!("{PAIRING} 78:86:2E:BB:13")), None);
        assert_eq!(read(&format!("{PAIRING} 78:86:2E:BB:13:2G")), None);
        assert_eq!(read(&format!("{PAIRING} 78:86:2E:BB:13:28:00")), None);
        assert_eq!(read(&format!("{PAIRING} 788:6:2E:BB:13:28")), None);
        assert_eq!(read(&format!("{PAIRING} 78:86:2E:BB:13:28 extra")), None);
        assert_eq!(read(&format!("{PAIRING}78:86:2E:BB:13:28")), None);
    }
}
