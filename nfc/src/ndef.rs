//! NDEF, read-only and limited to text and URI records — and the Bluetooth address in them.
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
                let text = if status & 0x80 == 0 {
                    String::from_utf8_lossy(text).into_owned()
                } else {
                    let (little_endian, bytes) = match text {
                        [0xFF, 0xFE, rest @ ..] => (true, rest),
                        [0xFE, 0xFF, rest @ ..] => (false, rest),
                        _ => (false, text),
                    };
                    if bytes.len() % 2 != 0 {
                        return Err("truncated UTF-16 text".into());
                    }
                    let units: Vec<u16> = bytes
                        .chunks_exact(2)
                        .map(|pair| {
                            let pair = [pair[0], pair[1]];
                            if little_endian {
                                u16::from_le_bytes(pair)
                            } else {
                                u16::from_be_bytes(pair)
                            }
                        })
                        .collect();
                    String::from_utf16_lossy(&units)
                };
                records.push(Record::Text(text));
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

/// The first Bluetooth address in these records, as `AA:BB:CC:DD:EE:FF`.
///
/// Written by a person, so spelled however they like: `:` or `-` between the octets or nothing,
/// either case, inside other text. Twelve hex digits with a separator used consistently, and
/// not part of a longer run of hex — which is what keeps a serial number from reading as one.
pub fn mac_in(records: &[Record]) -> Option<String> {
    records.iter().find_map(|r| mac_in_text(r.value()))
}

fn mac_in_text(text: &str) -> Option<String> {
    let bytes = text.as_bytes();
    let hex = |i: usize| bytes.get(i).is_some_and(u8::is_ascii_hexdigit);
    for start in 0..bytes.len() {
        if start > 0 && hex(start - 1) {
            continue;
        }
        for sep in [Some(b':'), Some(b'-'), None] {
            let step = if sep.is_some() { 3 } else { 2 };
            let end = start + 5 * step + 2;
            let fits = (0..6).all(|k| hex(start + k * step) && hex(start + k * step + 1))
                && (1..6).all(|k| sep.is_none_or(|s| bytes.get(start + k * step - 1) == Some(&s)))
                && !hex(end);
            if fits {
                let octets: Vec<String> = (0..6)
                    .map(|k| text[start + k * step..start + k * step + 2].to_ascii_uppercase())
                    .collect();
                return Some(octets.join(":"));
            }
        }
    }
    None
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
        let records = parse(&text_tag("98:B6:E9:28:06:09")).unwrap();
        assert_eq!(records, vec![Record::Text("98:B6:E9:28:06:09".into())]);
    }

    #[test]
    fn utf16_text_records_preserve_the_pairing_address() {
        // Phone writers may choose UTF-16 instead of UTF-8 for the same tag text.
        let text = "🎮 98:B6:E9:28:06:09";
        for (little_endian, with_bom) in [(false, true), (true, true), (false, false)] {
            let mut payload = vec![0x82, b'e', b'n'];
            if with_bom {
                payload.extend(if little_endian {
                    [0xFF, 0xFE]
                } else {
                    [0xFE, 0xFF]
                });
            }
            for unit in text.encode_utf16() {
                payload.extend(if little_endian {
                    unit.to_le_bytes()
                } else {
                    unit.to_be_bytes()
                });
            }
            let mut record = vec![0xD1, 1, payload.len() as u8, b'T'];
            record.extend(payload);
            let mut tag = vec![TLV_NDEF, record.len() as u8];
            tag.extend(record);
            tag.push(TLV_TERMINATOR);
            let records = parse(&tag).unwrap();
            assert_eq!(records, vec![Record::Text(text.into())]);
            assert_eq!(mac_in(&records).as_deref(), Some("98:B6:E9:28:06:09"));
        }
    }

    #[test]
    fn an_incomplete_utf16_code_unit_is_rejected() {
        let tag = [
            TLV_NDEF,
            10,
            0xD1,
            1,
            6,
            b'T',
            0x82,
            b'e',
            b'n',
            0xFE,
            0xFF,
            0,
            TLV_TERMINATOR,
        ];
        assert!(parse(&tag).is_err());
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
        let tag = text_tag("98:B6:E9:28:06:09");
        assert_eq!(tlv_size(&tag), Some(2 + tag[1] as usize + 1));
        assert_eq!(tlv_size(&[0x00, 0x00]), None);
    }

    #[test]
    fn a_blank_tag_is_an_error_not_an_empty_address() {
        assert!(parse(&[0u8; 16]).is_err());
        assert_eq!(parse(&[TLV_NDEF, 0, TLV_TERMINATOR]).unwrap(), vec![]);
    }

    #[test]
    fn an_address_is_found_however_it_is_spelled() {
        let mac = |s: &str| mac_in(&[Record::Text(s.into())]);
        let want = Some("98:B6:E9:28:06:09".to_owned());
        assert_eq!(mac("98:B6:E9:28:06:09"), want);
        assert_eq!(mac("pad=98-b6-e9-28-06-09 !"), want);
        assert_eq!(mac("98b6e9280609"), want);
        // Mixed separators, or a longer run of hex, are not an address.
        assert_eq!(mac("98:B6-E9:28:06:09"), None);
        assert_eq!(mac("198:B6:E9:28:06:09"), None);
        assert_eq!(mac("98b6e92806091"), None);
        assert_eq!(mac("nothing here"), None);
    }
}
