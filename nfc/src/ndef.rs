//! NDEF, limited to text and URI records — and whether they ask to pair a gamepad. Read, and
//! written only as one text record ([`text_tlv`]), which is all `nfc.write` puts on a tag.
//!
//! Pure: no hardware, entirely testable off a board.

const TLV_NULL: u8 = 0x00;
const TLV_NDEF: u8 = 0x03;
const TLV_TERMINATOR: u8 = 0xFE;

const FLAG_MB: u8 = 0x80;
const FLAG_ME: u8 = 0x40;
const FLAG_SR: u8 = 0x10;
const FLAG_IL: u8 = 0x08;
const TNF_WELL_KNOWN: u8 = 0x01;

/// The language a written text record says it is in. Readers ignore it — [`parse`] does — and
/// `en` is what phone apps write by default.
const LANG: &[u8] = b"en";

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

/// A tag's user memory holding `text` as its one record: the NDEF TLV, then the terminator —
/// what a phone app writes for a text, and what [`parse`] reads back as [`Record::Text`].
///
/// Refused only for a text whose message no TLV length can say (64 KiB); whether it fits the tag
/// is the tag's question, asked by [`crate::tag::write_ndef_area`].
pub fn text_tlv(text: &str) -> Result<Vec<u8>, String> {
    let mut payload = Vec::with_capacity(1 + LANG.len() + text.len());
    payload.push(LANG.len() as u8); // UTF-8, and the language's length
    payload.extend(LANG);
    payload.extend(text.as_bytes());

    let mut record = Vec::with_capacity(payload.len() + 7);
    match u8::try_from(payload.len()) {
        Ok(len) => record.extend([FLAG_MB | FLAG_ME | FLAG_SR | TNF_WELL_KNOWN, 1, len]),
        Err(_) => {
            record.extend([FLAG_MB | FLAG_ME | TNF_WELL_KNOWN, 1]);
            record.extend((payload.len() as u32).to_be_bytes());
        }
    }
    record.push(b'T');
    record.extend(payload);

    let mut tlv = vec![TLV_NDEF];
    match u16::try_from(record.len()) {
        Ok(len) if len < 0xFF => tlv.push(len as u8),
        Ok(len) if len < 0xFFFF => {
            tlv.push(0xFF);
            tlv.extend(len.to_be_bytes());
        }
        _ => {
            return Err(format!(
                "a text of {} bytes is too long for any tag",
                text.len()
            ));
        }
    }
    tlv.extend(record);
    tlv.push(TLV_TERMINATOR);
    Ok(tlv)
}

/// What a pairing tag holds: every one is written with this text, and none names a pad.
pub const PAIRING: &str = "microduck_gamepad_pairing";

/// Does one of these records say "pair a gamepad"? The whole record, surrounding whitespace aside —
/// a tag that merely mentions it in other text is not a pairing tag.
pub fn asks_to_pair(records: &[Record]) -> bool {
    records.iter().any(|r| r.value().trim() == PAIRING)
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
    fn a_written_text_reads_back_as_itself() {
        for text in ["walk_mode", PAIRING, "é🦆", ""] {
            let tlv = text_tlv(text).unwrap();
            assert_eq!(tlv_size(&tlv), Some(tlv.len()));
            assert_eq!(parse(&tlv).unwrap(), [Record::Text(text.into())]);
        }
        // What `text_tag` above — a phone app's layout — holds, byte for byte but the language.
        let short = text_tlv("walk_mode").unwrap();
        assert_eq!(&short[..7], [TLV_NDEF, 16, 0xD1, 1, 12, b'T', 2]);
    }

    /// Past 255 bytes the record takes a four-byte length and the TLV a three-byte one; both
    /// still read back.
    #[test]
    fn a_long_text_takes_the_long_lengths() {
        let text = "a".repeat(300);
        let tlv = text_tlv(&text).unwrap();
        assert_eq!(&tlv[..2], [TLV_NDEF, 0xFF]);
        assert_eq!(tlv[4] & FLAG_SR, 0);
        assert_eq!(tlv_size(&tlv), Some(tlv.len()));
        assert_eq!(parse(&tlv).unwrap(), [Record::Text(text)]);
        assert!(text_tlv(&"a".repeat(70_000)).is_err());
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
}
