//! MIFARE Classic, NDEF-formatted: the three-pass authentication, encrypted block reads, and the
//! NFC Forum's mapping of an NDEF message onto the card's sectors (NXP AN1304/AN1305).
//!
//! A MIFARE Classic is selected like any ISO 14443A tag ([`crate::tag::select_tag`], SAK `08`
//! for a 1K), and there the resemblance stops: every sector opens only after an authentication
//! with that sector's key, and from then on every byte *and parity bit* on air is encrypted with
//! [`Crypto1`]. A reader with Crypto1 in silicon (the CLRC663) hides that; the ST25R100 has none,
//! so this module runs the cipher and hands the chip raw 9-bit groups through
//! [`Reader::transceive_bits`].
//!
//! An NDEF-formatted card uses keys everybody knows, which is what lets any phone read it:
//!
//! - sector 0 holds the MIFARE Application Directory (blocks 1 and 2), readable with key A
//!   [`KEY_MAD`]. It lists, sector by sector, the application each one belongs to; NDEF is
//!   `03E1`.
//! - each NDEF sector opens with key A [`KEY_NDEF`]. Its first three blocks are data (the fourth
//!   is the sector trailer, keys and access bits), and the NDEF sectors' data blocks, in order,
//!   are the same TLV area a Type 2 tag's user memory is — [`crate::ndef`] reads it unchanged.
//!
//! A card that does not open with those keys is not NDEF-formatted — a factory-fresh card keeps
//! `FFFFFFFFFFFF` everywhere and no NDEF — and is reported as [`Error::Unreadable`]: asking again
//! would get the same refusal. A phone app (NFC Tools, NXP TagWriter) formats one by writing a
//! record to it.
//!
//! Each sector is opened with a fresh selection rather than a nested authentication: one field
//! cycle more per sector, against a second, encrypted authentication path to get right. A
//! pairing tag spans one or two sectors.

use std::time::{SystemTime, UNIX_EPOCH};

use crate::crypto1::{Crypto1, odd_parity, successor};
use crate::reader::{Exchange, Reader};
use crate::{Error, Result, ndef, tag};

/// Key A of the MAD sector on an NDEF-formatted card (AN10787 §3.2.1).
pub const KEY_MAD: [u8; 6] = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5];
/// Key A of every NDEF sector (AN1305 §6.1): public, so any reader can read the message.
pub const KEY_NDEF: [u8; 6] = [0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7];

const CMD_AUTH_A: u8 = 0x60;
const CMD_READ: u8 = 0x30;
/// The application identifier the MAD gives an NDEF sector, `E1 03` on the card.
const NDEF_AID: u16 = 0x03E1;
/// Sectors the MAD of a 1K describes. A 4K's sectors beyond 15 are in a second MAD this does not
/// read: a pairing message fits in two sectors.
const MAD_SECTORS: u8 = 15;
/// The card answers an authentication step within a few hundred microseconds.
const AUTH_TIMEOUT_MS: u64 = 5;
const READ_TIMEOUT_MS: u64 = 10;

/// Whether a SAK is a MIFARE Classic's: bit 4 (`08`) — `08` for a 1K, `18` for a 4K, `09` for a
/// Mini, `88` for Infineon's 1K. An NTAG answers `00`.
pub fn is_classic(sak: u8) -> bool {
    sak & 0x08 != 0
}

/// CRC_A (ISO 14443-3 Annex B), low byte first as it is sent.
pub fn crc_a(data: &[u8]) -> [u8; 2] {
    let crc = data.iter().fold(0x6363u16, |crc, &b| {
        let mut ch = b ^ crc as u8;
        ch ^= ch << 4;
        let ch = u16::from(ch);
        (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4)
    });
    crc.to_le_bytes()
}

/// Bytes and their parity bits as the bit stream on air: each byte least significant bit first,
/// then its parity bit, running on across byte boundaries. Returns the stream and its length in
/// bits.
pub fn pack(bytes: &[u8], parity: &[u8]) -> (Vec<u8>, usize) {
    let bits = bytes.len() * 9;
    let mut stream = vec![0u8; bits.div_ceil(8)];
    let mut at = 0;
    let mut put = |bit: u8| {
        stream[at / 8] |= (bit & 1) << (at % 8);
        at += 1;
    };
    for (&byte, &p) in bytes.iter().zip(parity) {
        for i in 0..8 {
            put(byte >> i);
        }
        put(p);
    }
    (stream, bits)
}

/// The inverse of [`pack`]: every whole 9-bit group in the first `bits` bits, as (byte, parity).
pub fn unpack(stream: &[u8], bits: usize) -> Vec<(u8, u8)> {
    let bit = |at: usize| stream.get(at / 8).map_or(0, |b| b >> (at % 8) & 1);
    (0..bits.min(stream.len() * 8) / 9)
        .map(|n| {
            let byte = (0..8).fold(0, |v, i| v | bit(n * 9 + i) << i);
            (byte, bit(n * 9 + 8))
        })
        .collect()
}

/// Encrypt `plain` a byte at a time, each parity bit with the keystream bit after its byte.
/// With `feed`, the plaintext is shifted into the cipher as it goes — the reader's nonce is; every
/// later byte is not.
fn encrypt(cipher: &mut Crypto1, plain: &[u8], feed: bool) -> (Vec<u8>, Vec<u8>) {
    plain
        .iter()
        .map(|&p| {
            let byte = cipher.byte(if feed { p } else { 0 }, false) ^ p;
            (byte, cipher.peek() ^ odd_parity(p))
        })
        .unzip()
}

/// The 32 UID bits authentication mixes in: a 4-byte UID whole, the last four bytes of a longer
/// one (its last cascade level).
fn auth_uid(uid: &[u8]) -> Result<u32> {
    let Some(last) = uid.len().checked_sub(4).and_then(|from| uid.get(from..)) else {
        return Err(Error::Tag(format!("a UID of {} bytes", uid.len())));
    };
    Ok(u32::from_be_bytes(last.try_into().expect("four bytes")))
}

/// An authenticated sector: the cipher in step with the card's.
pub struct Session {
    cipher: Crypto1,
}

/// Authenticate with key A for the sector `block` is in, on a card just selected. `nr` is the
/// reader's nonce.
///
/// A card that does not share the key answers the reader's nonce with silence, and that is
/// [`Error::Unreadable`]; a card that answers, but not with the proof of the key, was garbled on
/// the way and is [`Error::Tag`], worth a retry.
pub fn authenticate<R: Reader>(
    chip: &mut R,
    uid: &[u8],
    block: u8,
    key: &[u8; 6],
    nr: u32,
) -> Result<Session> {
    let uid = auth_uid(uid)?;
    let auth = Exchange {
        rx_crc: Some(false),
        ..Exchange::framed(AUTH_TIMEOUT_MS)
    };
    let nt = match chip.transceive(&[CMD_AUTH_A, block], auth) {
        Err(Error::NoTag) => {
            return Err(Error::Tag(format!(
                "no nonce for block {block}: the card left, or is not a MIFARE Classic"
            )));
        }
        answer => answer?,
    };
    let Ok(nt) = <[u8; 4]>::try_from(nt.as_slice()) else {
        return Err(Error::Tag(format!(
            "a nonce of {} bytes, expected 4",
            nt.len()
        )));
    };
    let nt = u32::from_be_bytes(nt);

    let mut cipher = Crypto1::new(key);
    cipher.word(uid ^ nt, false);
    let (mut bytes, mut parity) = encrypt(&mut cipher, &nr.to_be_bytes(), true);
    let (ar, ar_parity) = encrypt(&mut cipher, &successor(nt, 64).to_be_bytes(), false);
    bytes.extend(ar);
    parity.extend(ar_parity);
    let (frame, bits) = pack(&bytes, &parity);

    let (answer, answer_bits) = match chip.transceive_bits(&frame, bits, AUTH_TIMEOUT_MS) {
        Err(Error::NoTag) => {
            return Err(Error::Unreadable(format!(
                "block {block} does not open with key {}",
                hex(key)
            )));
        }
        answer => answer?,
    };
    let at = unpack(&answer, answer_bits);
    let [a, b, c, d] = at[..] else {
        return Err(Error::Tag(format!(
            "the card's proof of the key is {answer_bits} bits, expected 36"
        )));
    };
    let at = u32::from_be_bytes([a.0, b.0, c.0, d.0]) ^ cipher.word(0, false);
    if at != successor(nt, 96) {
        return Err(Error::Tag(format!(
            "block {block}: the card's proof of the key does not check out"
        )));
    }
    Ok(Session { cipher })
}

impl Session {
    /// READ one 16-byte block of the authenticated sector.
    pub fn read_block<R: Reader>(&mut self, chip: &mut R, block: u8) -> Result<[u8; 16]> {
        let mut plain = vec![CMD_READ, block];
        plain.extend(crc_a(&plain));
        let (bytes, parity) = encrypt(&mut self.cipher, &plain, false);
        let (frame, bits) = pack(&bytes, &parity);
        let (answer, answer_bits) = chip.transceive_bits(&frame, bits, READ_TIMEOUT_MS)?;
        if answer_bits == 4 {
            let nak = (0..4).fold(0u8, |v, i| {
                v | (self.cipher.bit(0, false) ^ (answer[0] >> i & 1)) << i
            });
            return Err(Error::Tag(format!(
                "READ block {block}: refused ({nak:#x})"
            )));
        }
        let got = unpack(&answer, answer_bits);
        if got.len() != 18 {
            return Err(Error::Tag(format!(
                "READ block {block}: {answer_bits} bits, expected 162"
            )));
        }
        let data: Vec<u8> = got
            .iter()
            .map(|(b, _)| self.cipher.byte(0, false) ^ b)
            .collect();
        if crc_a(&data[..16]) != data[16..] {
            return Err(Error::Tag(format!("READ block {block}: CRC error")));
        }
        Ok(data[..16].try_into().expect("sixteen bytes"))
    }
}

/// The NDEF area of a MIFARE Classic just selected: the MAD, then the data blocks of the sectors
/// it gives to NDEF, reading only as far as the message needs. Empty when the MAD gives none.
pub fn read_ndef_area<R: Reader>(chip: &mut R, uid: &[u8]) -> Result<Vec<u8>> {
    read_ndef_area_with(chip, uid, &mut clock_nonce)
}

fn read_ndef_area_with<R: Reader>(
    chip: &mut R,
    uid: &[u8],
    nonce: &mut dyn FnMut() -> u32,
) -> Result<Vec<u8>> {
    let mut mad = authenticate(chip, uid, 0, &KEY_MAD, nonce()).map_err(|e| match e {
        Error::Unreadable(_) => Error::Unreadable(
            "a MIFARE Classic that is not NDEF-formatted: its directory does not open with the \
             MAD key. Write a record to it with a phone first"
                .into(),
        ),
        e => e,
    })?;
    let mut directory = mad.read_block(chip, 1)?.to_vec();
    directory.extend(mad.read_block(chip, 2)?);

    let mut data = Vec::new();
    for sector in ndef_sectors(&directory) {
        // The MAD session is still open on the card; a fresh selection ends it.
        reselect(chip, uid)?;
        let first = sector * 4;
        let mut session = authenticate(chip, uid, first, &KEY_NDEF, nonce())?;
        for block in first..first + 3 {
            data.extend(session.read_block(chip, block)?);
            if ndef::tlv_size(&data).is_some_and(|needed| data.len() >= needed) {
                return Ok(data);
            }
        }
    }
    Ok(data)
}

/// The sectors the MAD (blocks 1 and 2: a CRC, an info byte, then one little-endian AID per
/// sector 1 to 15) gives to NDEF, in order.
fn ndef_sectors(directory: &[u8]) -> Vec<u8> {
    (1..=MAD_SECTORS)
        .filter(|&s| {
            let at = 2 * usize::from(s);
            directory
                .get(at..at + 2)
                .is_some_and(|aid| u16::from_le_bytes([aid[0], aid[1]]) == NDEF_AID)
        })
        .collect()
}

fn reselect<R: Reader>(chip: &mut R, uid: &[u8]) -> Result<()> {
    let again = tag::select_tag(chip)?;
    if again.uid != uid {
        return Err(Error::Tag("another tag answered mid-read".into()));
    }
    Ok(())
}

/// The reader's nonce. It only has to differ from one authentication to the next; the keys are
/// public, so nothing rests on it being unpredictable.
fn clock_nonce() -> u32 {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    now.subsec_nanos() ^ (now.as_secs() as u32).rotate_left(16)
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_a_matches_the_standard_frames() {
        assert_eq!(crc_a(&[0x30, 0x00]), [0x02, 0xA8], "READ block 0");
        assert_eq!(crc_a(&[0x50, 0x00]), [0x57, 0xCD], "HLTA");
    }

    #[test]
    fn a_byte_goes_on_air_as_nine_bits() {
        assert_eq!(pack(&[0x30], &[1]), (vec![0x30, 0x01], 9));
        let (stream, bits) = pack(&[0xA5, 0x3C, 0xFF], &[0, 1, 1]);
        assert_eq!(bits, 27);
        assert_eq!(unpack(&stream, bits), vec![(0xA5, 0), (0x3C, 1), (0xFF, 1)]);
        assert_eq!(
            unpack(&stream, 26).len(),
            2,
            "a partial group is not a byte"
        );
    }

    #[test]
    fn the_uid_authentication_uses_is_the_last_four_bytes() {
        assert_eq!(auth_uid(&[1, 2, 3, 4]).unwrap(), 0x0102_0304);
        assert_eq!(auth_uid(&[9, 9, 9, 1, 2, 3, 4]).unwrap(), 0x0102_0304);
        assert!(auth_uid(&[1, 2]).is_err());
    }

    #[test]
    fn classic_saks() {
        for sak in [0x08, 0x18, 0x09, 0x88] {
            assert!(is_classic(sak), "{sak:#x}");
        }
        assert!(!is_classic(0x00), "an NTAG");
        assert!(!is_classic(0x20), "ISO 14443-4");
    }

    /// A MIFARE Classic 1K as the reader sees it: ISO 14443A selection, then Crypto1 from the
    /// card's side of the exchange.
    struct Card {
        uid: [u8; 4],
        blocks: [[u8; 16]; 64],
        keys: [[u8; 6]; 16],
        state: State,
        nonce: u32,
        authentications: usize,
    }

    enum State {
        Idle,
        Selected,
        Challenged {
            cipher: Crypto1,
            nt: u32,
            sector: u8,
        },
        Open {
            cipher: Crypto1,
            sector: u8,
        },
    }

    impl Card {
        fn new(keys: [[u8; 6]; 16]) -> Self {
            Self {
                uid: [0xDE, 0xAD, 0xBE, 0xEF],
                blocks: [[0; 16]; 64],
                keys,
                state: State::Idle,
                nonce: 0x0102_0304,
                authentications: 0,
            }
        }

        /// NDEF-formatted as a phone leaves it: the MAD giving `sectors` to NDEF, and `tlv`
        /// across their data blocks.
        fn formatted(sectors: &[u8], tlv: &[u8]) -> Self {
            let mut keys = [KEY_NDEF; 16];
            keys[0] = KEY_MAD;
            let mut card = Self::new(keys);
            let mut mad = vec![0u8, 0x01];
            for s in 1..=15 {
                let aid: u16 = if sectors.contains(&s) { NDEF_AID } else { 0 };
                mad.extend(aid.to_le_bytes());
            }
            card.blocks[1].copy_from_slice(&mad[..16]);
            card.blocks[2].copy_from_slice(&mad[16..]);
            let data_blocks = sectors.iter().flat_map(|s| s * 4..s * 4 + 3);
            for (block, chunk) in data_blocks.zip(tlv.chunks(16)) {
                card.blocks[usize::from(block)][..chunk.len()].copy_from_slice(chunk);
            }
            card
        }

        fn reply(cipher: &mut Crypto1, plain: &[u8]) -> (Vec<u8>, usize) {
            let (bytes, parity) = encrypt(cipher, plain, false);
            pack(&bytes, &parity)
        }
    }

    impl Reader for Card {
        fn reset_field(&mut self) -> Result<()> {
            self.state = State::Idle;
            Ok(())
        }

        fn transceive(&mut self, data: &[u8], _: Exchange) -> Result<Vec<u8>> {
            let [a, b, c, d] = self.uid;
            match (data, &self.state) {
                ([0x52], _) => Ok(vec![0x04, 0x00]),
                ([0x93, 0x20], _) => Ok(vec![a, b, c, d, a ^ b ^ c ^ d]),
                ([0x93, 0x70, ..], _) => {
                    self.state = State::Selected;
                    Ok(vec![0x08])
                }
                ([CMD_AUTH_A, block], State::Selected) => {
                    self.nonce = successor(self.nonce, 7);
                    let (nt, sector) = (self.nonce, block / 4);
                    let mut cipher = Crypto1::new(&self.keys[usize::from(sector)]);
                    cipher.word(u32::from_be_bytes(self.uid) ^ nt, false);
                    self.state = State::Challenged { cipher, nt, sector };
                    self.authentications += 1;
                    Ok(nt.to_be_bytes().to_vec())
                }
                _ => {
                    self.state = State::Idle;
                    Err(Error::NoTag)
                }
            }
        }

        fn transceive_bits(
            &mut self,
            data: &[u8],
            bits: usize,
            _: u64,
        ) -> Result<(Vec<u8>, usize)> {
            match std::mem::replace(&mut self.state, State::Idle) {
                State::Challenged {
                    mut cipher,
                    nt,
                    sector,
                } => {
                    let got = unpack(data, bits);
                    assert_eq!(got.len(), 8, "the reader's nonce and answer");
                    let mut parity_ok = true;
                    for &(enc, p) in &got[..4] {
                        let plain = cipher.byte(enc, true) ^ enc;
                        parity_ok &= p == cipher.peek() ^ odd_parity(plain);
                    }
                    let mut ar = [0u8; 4];
                    for (slot, &(enc, p)) in ar.iter_mut().zip(&got[4..]) {
                        *slot = cipher.byte(0, false) ^ enc;
                        parity_ok &= p == cipher.peek() ^ odd_parity(*slot);
                    }
                    // A reader without the key gets silence.
                    if !parity_ok || ar != successor(nt, 64).to_be_bytes() {
                        return Err(Error::NoTag);
                    }
                    let answer = Self::reply(&mut cipher, &successor(nt, 96).to_be_bytes());
                    self.state = State::Open { cipher, sector };
                    Ok(answer)
                }
                State::Open { mut cipher, sector } => {
                    let got = unpack(data, bits);
                    let plain: Vec<u8> = got
                        .iter()
                        .map(|&(enc, _)| cipher.byte(0, false) ^ enc)
                        .collect();
                    match plain[..] {
                        [CMD_READ, block, lo, hi]
                            if crc_a(&plain[..2]) == [lo, hi] && block / 4 == sector =>
                        {
                            let mut answer = self.blocks[usize::from(block)].to_vec();
                            answer.extend(crc_a(&answer));
                            let reply = Self::reply(&mut cipher, &answer);
                            self.state = State::Open { cipher, sector };
                            Ok(reply)
                        }
                        _ => {
                            // NAK 0x4, encrypted with four keystream bits.
                            let nak = (0..4).fold(0u8, |v, i| {
                                v | ((cipher.bit(0, false) ^ (0x4 >> i & 1)) << i)
                            });
                            Ok((vec![nak], 4))
                        }
                    }
                }
                _ => Err(Error::NoTag),
            }
        }
    }

    /// An NDEF text record holding `text`, in its TLV.
    fn text_tlv(text: &str) -> Vec<u8> {
        let payload_len = 3 + text.len();
        let mut record = vec![0xD1, 0x01, payload_len as u8, b'T', 0x02, b'e', b'n'];
        record.extend(text.as_bytes());
        let mut tlv = vec![0x03, record.len() as u8];
        tlv.extend(record);
        tlv.push(0xFE);
        tlv
    }

    fn counting() -> impl FnMut() -> u32 {
        let mut n = 0x1234_5678u32;
        move || {
            n = n.wrapping_add(0x9E37_79B9);
            n
        }
    }

    fn read(card: &mut Card) -> Result<Vec<u8>> {
        let uid = tag::select_tag(card)?.uid;
        read_ndef_area_with(card, &uid, &mut counting())
    }

    /// A 43-character text is 53 bytes of TLV: more than one sector's 48, so the read crosses
    /// into the second NDEF sector and stops as soon as the terminator is in.
    #[test]
    fn a_longer_message_is_read_across_two_sectors() {
        let text = "microduck_gamepad_pairing 78:86:2E:BB:13:28";
        let mut card = Card::formatted(&[1, 2, 3], &text_tlv(text));
        let data = read(&mut card).unwrap();
        assert_eq!(data.len(), 64, "three blocks of sector 1, one of sector 2");
        assert_eq!(
            card.authentications, 3,
            "the MAD, sector 1, sector 2 — not sector 3"
        );
        let records = ndef::parse(&data).unwrap();
        assert_eq!(records, vec![ndef::Record::Text(text.to_owned())]);
    }

    #[test]
    fn a_short_message_stays_in_one_sector() {
        let mut card = Card::formatted(&[1], &text_tlv("microduck_gamepad_pairing"));
        let data = read(&mut card).unwrap();
        assert_eq!(data.len(), 48);
        assert!(ndef::asks_to_pair(&ndef::parse(&data).unwrap()));
    }

    /// NDEF sectors need not start at 1: the MAD says where they are.
    #[test]
    fn the_mad_says_which_sectors() {
        let mut card = Card::formatted(&[5, 6], &text_tlv("microduck_gamepad_pairing"));
        let records = ndef::parse(&read(&mut card).unwrap()).unwrap();
        assert!(ndef::asks_to_pair(&records));
    }

    #[test]
    fn a_factory_card_is_unreadable_once_not_retried() {
        let mut card = Card::new([[0xFF; 6]; 16]);
        let e = read(&mut card).unwrap_err();
        assert!(
            matches!(&e, Error::Unreadable(why) if why.contains("not NDEF-formatted")),
            "{e}"
        );
    }

    #[test]
    fn a_sector_with_another_key_is_unreadable() {
        let mut card = Card::formatted(&[1], &text_tlv("microduck_gamepad_pairing"));
        card.keys[1] = [0xFF; 6];
        let e = read(&mut card).unwrap_err();
        assert!(
            matches!(&e, Error::Unreadable(why) if why.contains("block 4")),
            "{e}"
        );
    }

    #[test]
    fn a_mad_without_ndef_is_an_empty_area() {
        let mut card = Card::formatted(&[], &[]);
        let data = read(&mut card).unwrap();
        assert!(data.is_empty());
        assert!(ndef::parse(&data).is_err(), "said as a blank tag");
    }

    #[test]
    fn a_read_outside_the_open_sector_is_refused() {
        let mut card = Card::formatted(&[1], &text_tlv("x"));
        let uid = tag::select_tag(&mut card).unwrap().uid;
        let mut session = authenticate(&mut card, &uid, 4, &KEY_NDEF, 7).unwrap();
        assert!(session.read_block(&mut card, 4).is_ok());
        let e = session.read_block(&mut card, 8).unwrap_err();
        assert!(e.to_string().contains("refused (0x4)"), "{e}");
    }
}
