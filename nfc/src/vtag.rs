//! ISO 15693 (NFC Type V) inventory and the NFC Forum Type 5 reads, for an ST25TV or ICODE.
//!
//! One tag is assumed in the field, and addressed in every command by its UID — the single-slot
//! inventory answers whichever tag is there. [`crate::st25r100::St25r100::set_proto`] must have
//! framed the reader for [`crate::st25r100::Proto::V`] first.
//!
//! A Type 5 tag carries the same TLV structure a Type 2 one does, behind a capability container
//! in the first bytes (§7 of the NFC Forum Type 5 Tag specification): [`crate::ndef`] walks both
//! unchanged.

use crate::reader::{Exchange, Reader};
use crate::{Error, Result, ndef};

/// Flags of every request here: high data rate, one subcarrier, addressed by UID. The inventory
/// adds bit 5, which ST's reader library also sets — without it a tag does not answer this
/// reader (measured 2026-10-27); on the wire it reads as AFI-present, and no AFI byte follows.
const FLAG_HIGH_RATE: u8 = 0x02;
const FLAG_ONE_SUBCARRIER: u8 = 0x04;
const FLAG_INVENTORY_EXTRA: u8 = 0x20;
const FLAG_ADDRESS: u8 = 0x20;
const RES_ERROR: u8 = 0x01;

const CMD_INVENTORY: u8 = 0x01;
const CMD_READ_SINGLE_BLOCK: u8 = 0x20;

/// Single-slot inventory: flags, command, no mask bits.
const INVENTORY_REQ: [u8; 3] = [
    FLAG_HIGH_RATE | FLAG_ONE_SUBCARRIER | FLAG_INVENTORY_EXTRA,
    CMD_INVENTORY,
    0x00,
];
/// A tag answers within t1 (≈320 µs) plus its own thinking; 20 ms covers an EEPROM read too.
const TIMEOUT_MS: u64 = 20;
/// Reading stops here even without a terminator: a pairing tag's message is a tenth of this, and
/// what runs past a cap this size is not one.
const MAX_BLOCKS: u8 = 32;

/// Find the tag in the field: a single-slot inventory. Returns its UID, most significant byte
/// first — the order it is printed in, not the order it is sent in.
pub fn inventory<R: Reader>(chip: &mut R) -> Result<Vec<u8>> {
    let answer = chip.transceive(&INVENTORY_REQ, Exchange::framed(TIMEOUT_MS))?;
    let body = checked(answer, "inventory")?;
    // The body is a DSFID then the UID, least significant byte first.
    if body.len() != 9 {
        return Err(Error::Tag(format!(
            "inventory answer of {} bytes, expected 9",
            body.len()
        )));
    }
    let mut uid = body[1..].to_vec();
    uid.reverse();
    Ok(uid)
}

/// Read one block, addressed. Returns its data without the response flags byte.
pub fn read_block<R: Reader>(chip: &mut R, uid: &[u8], block: u8) -> Result<Vec<u8>> {
    let mut req = Vec::with_capacity(11);
    req.push(FLAG_HIGH_RATE | FLAG_ADDRESS);
    req.push(CMD_READ_SINGLE_BLOCK);
    req.extend(uid.iter().rev()); // least significant byte first on air
    req.push(block);
    let answer = chip.transceive(&req, Exchange::framed(TIMEOUT_MS))?;
    checked(answer, "block read")
}

/// The TLV area of the inventoried tag: everything after the capability container, reading only
/// as many blocks as the message needs — a pairing tag costs four or five exchanges, not the
/// whole memory.
///
/// The first block tells the block length; the container's magic tells whether four or eight
/// bytes of it precede the TLVs. Without a magic (a blank or foreign-format tag) the four a
/// written tag carries is assumed, which walks a bare TLV area just as well.
pub fn read_ndef_area<R: Reader>(chip: &mut R, uid: &[u8]) -> Result<Vec<u8>> {
    let mut data = read_block(chip, uid, 0)?;
    let blk_len = data.len();
    if blk_len == 0 {
        return Err(Error::Tag("empty block".into()));
    }
    let cc_len = match data[0] {
        0xE1 => 4,
        0xE2 => 8,
        _ => 4,
    };
    // The first two blocks carry the container and the TLV head, which is what sizes the message.
    while data.len() < cc_len + 8 {
        let next = (data.len() / blk_len) as u8;
        data.extend(read_block(chip, uid, next)?);
    }
    let needed = ndef::tlv_size(&data[cc_len..]).map(|tlv| cc_len + tlv);
    while needed.is_some_and(|n| data.len() < n) && (data.len() / blk_len) < MAX_BLOCKS as usize {
        let next = (data.len() / blk_len) as u8;
        data.extend(read_block(chip, uid, next)?);
    }
    Ok(data[cc_len..].to_vec())
}

/// The body of an answer without its flags byte — or the error the flags byte announces.
fn checked(answer: Vec<u8>, what: &str) -> Result<Vec<u8>> {
    match answer.split_first() {
        Some((&flags, body)) if flags & RES_ERROR == 0 => Ok(body.to_vec()),
        Some((&flags, body)) => Err(Error::Tag(format!(
            "{what} refused (flags {flags:#04x}: {body:02x?})"
        ))),
        None => Err(Error::Tag(format!("{what}: empty answer"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tag that answers from its blocks, recording every request.
    struct Fake {
        uid: Vec<u8>,
        blocks: Vec<[u8; 4]>,
        requests: Vec<Vec<u8>>,
        /// Answers served before the tag goes quiet, for the no-tag path.
        quiet_after: Option<usize>,
    }

    impl Fake {
        /// The tag written on the bring-up board: `microduck_gamepad_pairing` behind a CC, in
        /// 4-byte blocks, exactly as `nfc-test t5 ndef` wrote it.
        fn pairing() -> Self {
            Self::pairing_cc(4)
        }

        /// The same TLV area behind an 8-byte container, as a tag over 2 KB lays it out.
        fn pairing8() -> Self {
            Self::pairing_cc(8)
        }

        fn pairing_cc(cc_len: usize) -> Self {
            let mut area = match cc_len {
                8 => vec![0xE2, 0x40, 0x00, 0x01, 0x00, 0x00, 0x03, 0xFF],
                _ => vec![0xE1, 0x40, 0x00, 0x16],
            };
            // The TLV a pairing tag carries: one NDEF text record holding a pad's address.
            let text = b"98:B6:E9:3A:35:0D";
            let mut record = vec![0xD1, 0x01, (3 + text.len()) as u8, b'T', 0x02, b'e', b'n'];
            record.extend_from_slice(text);
            let mut tlv = vec![0x03, record.len() as u8];
            tlv.extend(record);
            tlv.push(0xFE);
            area.extend(tlv);
            area.resize(area.len() + (4 - area.len() % 4) % 4, 0);
            // Two spare blocks past the message, to see the reading stop before them.
            area.extend_from_slice(&[0; 8]);
            let blocks = area.chunks(4).map(|c| c.try_into().unwrap()).collect();
            Self {
                uid: vec![0xE0, 0x48, 0x04, 0x00, 0x0A, 0x3D, 0x2C, 0x9D],
                blocks,
                requests: Vec::new(),
                quiet_after: None,
            }
        }
    }

    impl Reader for Fake {
        fn reset_field(&mut self) -> Result<()> {
            Ok(())
        }

        fn transceive(&mut self, data: &[u8], _exchange: Exchange) -> Result<Vec<u8>> {
            self.requests.push(data.to_vec());
            if self.quiet_after.is_some_and(|n| self.requests.len() > n) {
                return Err(Error::NoTag);
            }
            match data[1] {
                CMD_INVENTORY => {
                    let mut answer = vec![0x00, 0x00]; // response flags, DSFID
                    answer.extend(self.uid.iter().rev());
                    Ok(answer)
                }
                CMD_READ_SINGLE_BLOCK => {
                    let block = data[10] as usize;
                    let mut answer = vec![0x00];
                    answer.extend_from_slice(&self.blocks[block]);
                    Ok(answer)
                }
                other => Err(Error::Tag(format!("unknown command {other:#04x}"))),
            }
        }
    }

    #[test]
    fn the_inventory_uid_comes_back_msb_first() {
        let mut tag = Fake::pairing();
        assert_eq!(
            inventory(&mut tag).unwrap(),
            vec![0xE0, 0x48, 0x04, 0x00, 0x0A, 0x3D, 0x2C, 0x9D]
        );
        assert_eq!(tag.requests, vec![vec![0x26, 0x01, 0x00]]);
    }

    #[test]
    fn an_error_answer_is_a_tag_error() {
        assert!(matches!(
            checked(vec![0x01, 0x02], "inventory"),
            Err(Error::Tag(_))
        ));
        assert_eq!(checked(vec![0x00, 0xAA], "block read").unwrap(), vec![0xAA]);
        assert!(matches!(checked(vec![], "inventory"), Err(Error::Tag(_))));
    }

    /// Reads stop once the TLV's bytes are covered — 31 bytes here, block 0's CC plus seven
    /// more; the two spare blocks past it are never asked for.
    #[test]
    fn a_pairing_tag_reads_back_its_address() {
        let mut tag = Fake::pairing();
        let uid = inventory(&mut tag).unwrap();
        let area = read_ndef_area(&mut tag, &uid).unwrap();
        assert_eq!(
            ndef::mac_in(&ndef::parse(&area).unwrap()),
            Some("98:B6:E9:3A:35:0D".into())
        );
        let reads = tag.requests.len() - 1;
        assert_eq!(reads, 8, "the CC block plus the seven the TLV spans");
        assert!(reads < tag.blocks.len());
    }

    /// A tag lifted mid-read is the ordinary not-read, not a daemon-ending one.
    #[test]
    fn a_tag_lifted_mid_read_is_no_tag() {
        let mut tag = Fake::pairing();
        tag.quiet_after = Some(3);
        let uid = inventory(&mut tag).unwrap();
        assert!(matches!(read_ndef_area(&mut tag, &uid), Err(Error::NoTag)));
    }

    /// An 8-byte container (a tag over 2 KB) moves the TLV area; the same walk finds the message.
    #[test]
    fn an_8_byte_cc_offsets_the_tlv_area() {
        let mut tag = Fake::pairing8();
        let uid = inventory(&mut tag).unwrap();
        let area = read_ndef_area(&mut tag, &uid).unwrap();
        assert_eq!(
            area[0], 0x03,
            "the area starts at the TLV, past the 8-byte CC"
        );
        assert_eq!(
            ndef::mac_in(&ndef::parse(&area).unwrap()),
            Some("98:B6:E9:3A:35:0D".into())
        );
    }
}
