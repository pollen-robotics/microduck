//! ISO 15693 (NFC Type V) inventory and the NFC Forum Type 5 reads and writes, for an ST25TV or
//! ICODE.
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

/// Request flags (ISO 15693-3 §7.3.1). Bits 5-8 mean different things with and without the
/// Inventory flag, which is why 0x20 has two names. Every request here is at the high data rate
/// with one subcarrier (bit 1 clear).
const FLAG_HIGH_RATE: u8 = 0x02;
/// Bit 3: this is an inventory, and bits 5-8 are read as the inventory's.
const FLAG_INVENTORY: u8 = 0x04;
/// With [`FLAG_INVENTORY`]: one slot. Without it a tag answers only in the one of 16 slots its UID
/// picks, which reads as silence to a reader asking once — the "0x06 is ignored" of the bring-up.
const FLAG_ONE_SLOT: u8 = 0x20;
/// Without [`FLAG_INVENTORY`]: the request carries a UID, and only that tag answers.
const FLAG_ADDRESS: u8 = 0x20;
const RES_ERROR: u8 = 0x01;

const CMD_INVENTORY: u8 = 0x01;
const CMD_READ_SINGLE_BLOCK: u8 = 0x20;
const CMD_WRITE_SINGLE_BLOCK: u8 = 0x21;
const CMD_GET_SYSTEM_INFO: u8 = 0x2B;
/// GET SYSTEM INFO's information flags (ISO 15693-3 §10.4.12): which optional fields follow the
/// UID, the memory size among them.
const INFO_DSFID: u8 = 0x01;
const INFO_AFI: u8 = 0x02;
const INFO_MEMORY: u8 = 0x04;

/// Single-slot inventory: flags, command, a mask length of 0.
const INVENTORY_REQ: [u8; 3] = [
    FLAG_HIGH_RATE | FLAG_INVENTORY | FLAG_ONE_SLOT,
    CMD_INVENTORY,
    0x00,
];
/// A tag answers within t1 (≈320 µs) plus its own thinking; 20 ms covers an EEPROM read too.
const TIMEOUT_MS: u64 = 20;
/// A write answers once the EEPROM has taken it: ISO 15693-3 allows the tag 20 ms for that, on
/// top of the ordinary answer time.
const WRITE_TIMEOUT_MS: u64 = 40;
/// Reading never goes past this many blocks, whatever the tag's memory: a pairing tag's message is
/// a few of them, and what runs past a cap this size is not one.
const MAX_BLOCKS: u8 = 32;

/// Capability container magic (NFC Forum Type 5 §4.3): `E1` for one-byte block addresses, `E2`
/// for two. Either says the tag is NDEF-formatted.
const CC_MAGIC_1: u8 = 0xE1;
const CC_MAGIC_2: u8 = 0xE2;
/// Mapping version 1.0 with read and write access granted (NFC Forum Type 5 §4.3.1): byte 1 of
/// the container this writes on a blank tag, as a phone app writes it.
const CC_VERSION_OPEN: u8 = 0x40;
/// Byte 1's low two bits: write access, `00` granted.
const CC_WRITE_ACCESS: u8 = 0x03;

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
/// as many blocks as the message needs — a pairing tag costs a handful of exchanges, not the
/// whole memory.
///
/// The container says how long it is and how much memory follows (NFC Forum Type 5 §4.3): an
/// `MLEN` in byte 2, in 8-byte units, or 0 there and the real one in bytes 6-7 of an 8-byte
/// container — told apart from a 4-byte one by its RFU bytes 4-5, which are zero, where a TLV
/// would already have started. That size bounds every read, so a TLV area that does not open with the NDEF TLV —
/// a phone app's Lock Control TLV first — is read whole without asking past the end of a small
/// tag, which the tag would refuse.
///
/// A tag with no container is not NDEF-formatted: nothing is read past it, and the empty area
/// comes back for [`ndef::parse`] to call blank.
pub fn read_ndef_area<R: Reader>(chip: &mut R, uid: &[u8]) -> Result<Vec<u8>> {
    let mut data = read_block(chip, uid, 0)?;
    let blk_len = data.len();
    if blk_len == 0 {
        return Err(Error::Tag("empty block".into()));
    }
    let read_to = |data: &mut Vec<u8>, chip: &mut R, len: usize| -> Result<()> {
        while data.len() < len {
            let next = (data.len() / blk_len) as u8;
            data.extend(read_block(chip, uid, next)?);
        }
        Ok(())
    };
    if !matches!(data[0], CC_MAGIC_1 | CC_MAGIC_2) {
        return Ok(Vec::new());
    }
    read_to(&mut data, chip, 8)?;
    let (cc_len, mlen) = match data[..8] {
        [_, _, mlen, ..] if mlen != 0 => (4, Some(mlen as usize * 8)),
        // MLEN 0 and the RFU bytes clear: the 8-byte container, its size in bytes 6-7.
        [_, _, 0, _, 0, 0, hi, lo] => (8, Some(u16::from_be_bytes([hi, lo]) as usize * 8)),
        // MLEN 0 with the TLVs already in bytes 4-5: a 4-byte container that does not say how big
        // the memory is. The bring-up's own writer left `E1 40 00 16`; the cap bounds those.
        _ => (4, None),
    };
    let cap = MAX_BLOCKS as usize * blk_len;
    let end = mlen.map_or(cap, |mlen| (cc_len + mlen).min(cap));
    // The TLV head is at most four bytes, and it is what sizes the message.
    read_to(&mut data, chip, (cc_len + 4).min(end))?;
    let needed = ndef::tlv_size(&data[cc_len..])
        .map(|tlv| cc_len + tlv)
        .unwrap_or(end);
    if needed > end {
        return Err(Error::Tag(format!(
            "the NDEF message needs {} bytes, past the {} read here",
            needed - cc_len,
            end - cc_len
        )));
    }
    read_to(&mut data, chip, needed)?;
    data.truncate(needed.max(cc_len));
    Ok(data.split_off(cc_len))
}

/// Write one block, addressed. `data` is exactly one block.
pub fn write_block<R: Reader>(chip: &mut R, uid: &[u8], block: u8, data: &[u8]) -> Result<()> {
    let mut req = Vec::with_capacity(11 + data.len());
    // Option flag clear: what NXP and ST tags take. A TI tag wants it set and answers an error.
    req.push(FLAG_HIGH_RATE | FLAG_ADDRESS);
    req.push(CMD_WRITE_SINGLE_BLOCK);
    req.extend(uid.iter().rev());
    req.push(block);
    req.extend(data);
    let answer = chip.transceive(&req, Exchange::framed(WRITE_TIMEOUT_MS))?;
    checked(answer, &format!("write of block {block}")).map(drop)
}

/// The tag's memory as GET SYSTEM INFO tells it — blocks, and bytes per block — or `None` for a
/// tag that does not answer it or leaves the size out. Optional in ISO 15693, so not having it is
/// no error; only the reader failing is.
pub fn memory_size<R: Reader>(chip: &mut R, uid: &[u8]) -> Result<Option<(usize, usize)>> {
    let mut req = Vec::with_capacity(10);
    req.push(FLAG_HIGH_RATE | FLAG_ADDRESS);
    req.push(CMD_GET_SYSTEM_INFO);
    req.extend(uid.iter().rev());
    let body = match chip.transceive(&req, Exchange::framed(TIMEOUT_MS)) {
        Ok(answer) => match checked(answer, "system info") {
            Ok(body) => body,
            Err(_) => return Ok(None),
        },
        Err(Error::NoTag | Error::Tag(_)) => return Ok(None),
        Err(e) => return Err(e),
    };
    // Information flags, the UID, then DSFID and AFI if flagged, then the memory size.
    let Some(&info) = body.first() else {
        return Ok(None);
    };
    if info & INFO_MEMORY == 0 {
        return Ok(None);
    }
    let at = 9 + usize::from(info & INFO_DSFID != 0) + usize::from(info & INFO_AFI != 0);
    Ok(match body.get(at..at + 2) {
        Some(&[blocks, size]) => Some((usize::from(blocks) + 1, usize::from(size & 0x1F) + 1)),
        _ => None,
    })
}

/// Write `tlv` — an NDEF TLV, [`ndef::text_tlv`] — as the tag's whole message, then read it back.
/// `Ok` means the tag holds it.
///
/// A tag already NDEF-formatted keeps its capability container and takes the message after it;
/// one locked read-only by that container is refused untouched. A blank tag — no container — is
/// formatted: a 4-byte container granting read and write, sized from GET SYSTEM INFO, or with an
/// `MLEN` of 0 when the tag does not say how big it is.
///
/// Only data blocks are written: no lock command, no AFI or DSFID, so nothing here can lock a
/// tag. Block 0, which holds the container, is written last, so a tag lifted half way through
/// is still blank, or still holds its container in front of half a message — either reads as no
/// NDEF, and writing again is the fix.
pub fn write_ndef_area<R: Reader>(chip: &mut R, uid: &[u8], tlv: &[u8]) -> Result<()> {
    let first = read_block(chip, uid, 0)?;
    let blk_len = first.len();
    if blk_len == 0 {
        return Err(Error::Tag("empty block".into()));
    }
    let (cc, capacity) = if matches!(first[0], CC_MAGIC_1 | CC_MAGIC_2) {
        let mut head = first.clone();
        while head.len() < 8 {
            head.extend(read_block(chip, uid, (head.len() / blk_len) as u8)?);
        }
        if head[1] & CC_WRITE_ACCESS != 0 {
            return Err(Error::Tag("the tag is locked read-only".into()));
        }
        let (cc_len, mlen) = match head[..8] {
            [_, _, mlen, ..] if mlen != 0 => (4, Some(usize::from(mlen) * 8)),
            [_, _, 0, _, 0, 0, hi, lo] => (8, Some(usize::from(u16::from_be_bytes([hi, lo])) * 8)),
            _ => (4, None),
        };
        (head[..cc_len].to_vec(), mlen)
    } else {
        let size = memory_size(chip, uid)?.map(|(blocks, size)| blocks * size);
        // Past 2 KB the container takes 8 bytes; a 4-byte one saying 255 still holds a pairing
        // message many times over, and is what every reader understands.
        let mlen = size.map_or(0, |size| (size.saturating_sub(4) / 8).min(0xFF) as u8);
        let capacity = size.map(|_| usize::from(mlen) * 8);
        (vec![CC_MAGIC_1, CC_VERSION_OPEN, mlen, 0x00], capacity)
    };
    // Without a declared size the read cap bounds it: what this tag would read back.
    let capacity = capacity.unwrap_or(MAX_BLOCKS as usize * blk_len - cc.len());
    if tlv.len() > capacity {
        return Err(Error::Tag(format!(
            "{} bytes to write, the tag holds {capacity}",
            tlv.len()
        )));
    }

    let mut image = cc;
    image.extend(tlv);
    image.resize(image.len().next_multiple_of(blk_len), 0);
    let blocks: Vec<&[u8]> = image.chunks(blk_len).collect();
    for (n, data) in blocks.iter().enumerate().skip(1) {
        write_block(chip, uid, n as u8, data)?;
    }
    if blocks[0] != first.as_slice() {
        write_block(chip, uid, 0, blocks[0])?;
    }

    let mut back = Vec::with_capacity(image.len());
    while back.len() < image.len() {
        back.extend(read_block(chip, uid, (back.len() / blk_len) as u8)?);
    }
    if back[..image.len()] != image[..] {
        return Err(Error::Tag(
            "read back differs from what was written — the tag may be locked".into(),
        ));
    }
    Ok(())
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
        /// Whether GET SYSTEM INFO is answered, with the memory size.
        system_info: bool,
        /// The blocks written, in order.
        writes: Vec<u8>,
    }

    /// The pairing text's TLV, as a phone writes it: one NDEF text record, then the terminator.
    fn pairing_tlv() -> Vec<u8> {
        let text = ndef::PAIRING.as_bytes();
        let mut record = vec![0xD1, 0x01, (3 + text.len()) as u8, b'T', 0x02, b'e', b'n'];
        record.extend_from_slice(text);
        let mut tlv = vec![0x03, record.len() as u8];
        tlv.extend(record);
        tlv.push(0xFE);
        tlv
    }

    impl Fake {
        /// A pairing tag behind a 4-byte container whose MLEN covers the memory after it: the
        /// message, then two spare blocks the reading must not ask for.
        fn pairing() -> Self {
            let mut area = pairing_tlv();
            area.extend([0; 8]);
            Self::sized(&[0xE1, 0x40], &area)
        }

        /// The same TLV area behind an 8-byte container, as a tag over 2 KB lays it out.
        fn pairing8() -> Self {
            let mut tag = vec![0xE2, 0x40, 0x00, 0x01, 0x00, 0x00, 0x03, 0xFF];
            tag.extend(pairing_tlv());
            tag.extend([0; 8]);
            Self::raw(tag)
        }

        /// `magic, version` then an MLEN that says exactly how much of `area` there is: the whole
        /// memory, so a block past it is one the tag refuses.
        fn sized(head: &[u8], area: &[u8]) -> Self {
            let mut area = area.to_vec();
            area.resize(area.len().div_ceil(8) * 8, 0);
            let mut tag = head.to_vec();
            tag.extend([(area.len() / 8) as u8, 0x00]);
            tag.extend(area);
            Self::raw(tag)
        }

        fn raw(mut memory: Vec<u8>) -> Self {
            memory.resize(memory.len().div_ceil(4) * 4, 0);
            Self {
                uid: vec![0xE0, 0x48, 0x04, 0x00, 0x0A, 0x3D, 0x2C, 0x9D],
                blocks: memory.chunks(4).map(|c| c.try_into().unwrap()).collect(),
                requests: Vec::new(),
                quiet_after: None,
                system_info: true,
                writes: Vec::new(),
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
                    // Past the end, a real tag answers with the error flag: 0x10, block not
                    // available.
                    Ok(match self.blocks.get(data[10] as usize) {
                        Some(block) => [&[0x00][..], block].concat(),
                        None => vec![RES_ERROR, 0x10],
                    })
                }
                CMD_WRITE_SINGLE_BLOCK => {
                    let n = data[10] as usize;
                    self.writes.push(data[10]);
                    match self.blocks.get_mut(n) {
                        Some(block) => {
                            block.copy_from_slice(&data[11..15]);
                            Ok(vec![0x00])
                        }
                        None => Ok(vec![RES_ERROR, 0x10]),
                    }
                }
                CMD_GET_SYSTEM_INFO if self.system_info => {
                    // Flags; DSFID, AFI and memory size present; the UID; DSFID, AFI; blocks - 1,
                    // block size - 1; no IC reference.
                    let mut answer = vec![0x00, INFO_DSFID | INFO_AFI | INFO_MEMORY];
                    answer.extend(self.uid.iter().rev());
                    answer.extend([0x00, 0x00, (self.blocks.len() - 1) as u8, 0x03]);
                    Ok(answer)
                }
                CMD_GET_SYSTEM_INFO => Ok(vec![RES_ERROR, 0x01]),
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

    /// Reads stop once the TLV's bytes are covered — 35 bytes here, block 0's CC plus nine
    /// more; the two spare blocks past it are never asked for.
    #[test]
    fn a_pairing_tag_reads_back_as_one() {
        let mut tag = Fake::pairing();
        let uid = inventory(&mut tag).unwrap();
        let area = read_ndef_area(&mut tag, &uid).unwrap();
        assert!(ndef::asks_to_pair(&ndef::parse(&area).unwrap()));
        let reads = tag.requests.len() - 1;
        assert_eq!(reads, 10, "the CC block plus the nine the TLV spans");
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
        assert!(ndef::asks_to_pair(&ndef::parse(&area).unwrap()));
    }

    /// A phone app's Lock Control TLV ahead of the NDEF one leaves the message unsized: the whole
    /// memory the container declares is read, and not a block past it — on a tag this small, a
    /// read past the end would be refused and the tag never paired.
    #[test]
    fn a_lock_tlv_first_reads_the_declared_memory_and_no_further() {
        let mut area = vec![0x01, 0x03, 0xA0, 0x0C, 0x34];
        area.extend(pairing_tlv());
        let mut tag = Fake::sized(&[0xE1, 0x40], &area);
        let uid = inventory(&mut tag).unwrap();
        let read = read_ndef_area(&mut tag, &uid).unwrap();
        assert!(ndef::asks_to_pair(&ndef::parse(&read).unwrap()));
        assert_eq!(
            tag.requests.len() - 1,
            tag.blocks.len(),
            "every block, once"
        );
    }

    /// What the bring-up's writer left: MLEN 0 in a 4-byte container. Still read, as before.
    #[test]
    fn a_container_without_its_size_is_still_read() {
        let mut tag = vec![0xE1, 0x40, 0x00, 0x16];
        tag.extend(pairing_tlv());
        tag.extend([0; 8]);
        let mut tag = Fake::raw(tag);
        let uid = inventory(&mut tag).unwrap();
        let area = read_ndef_area(&mut tag, &uid).unwrap();
        assert!(ndef::asks_to_pair(&ndef::parse(&area).unwrap()));
    }

    /// No container, no NDEF: one block read, and an empty area for the parser to call blank.
    #[test]
    fn a_tag_without_a_container_is_blank() {
        let mut tag = Fake::raw(vec![0x00; 32]);
        let uid = inventory(&mut tag).unwrap();
        let area = read_ndef_area(&mut tag, &uid).unwrap();
        assert!(area.is_empty());
        assert!(ndef::parse(&area).is_err());
        assert_eq!(tag.requests.len() - 1, 1);
    }

    /// A message longer than what is read says so, rather than coming back cut short.
    #[test]
    fn a_message_past_the_end_is_an_error_not_a_truncation() {
        // A TLV claiming 200 bytes, in a tag that declares 16.
        let mut tag = Fake::sized(&[0xE1, 0x40], &[0x03, 200, 0xD1, 0x01]);
        let uid = inventory(&mut tag).unwrap();
        assert!(matches!(read_ndef_area(&mut tag, &uid), Err(Error::Tag(_))));
    }

    fn written_reads_as(tag: &mut Fake, text: &str) {
        let uid = inventory(tag).unwrap();
        let area = read_ndef_area(tag, &uid).unwrap();
        assert_eq!(
            ndef::parse(&area).unwrap(),
            [ndef::Record::Text(text.into())]
        );
    }

    /// A blank tag — the one no phone app would format — gets a container sized from GET SYSTEM
    /// INFO, block 0 written last, and then reads as the text.
    #[test]
    fn a_blank_tag_is_formatted_and_written() {
        let mut tag = Fake::raw(vec![0x00; 112]); // 28 blocks of 4
        let uid = inventory(&mut tag).unwrap();
        let tlv = ndef::text_tlv(ndef::PAIRING).unwrap();
        write_ndef_area(&mut tag, &uid, &tlv).unwrap();
        assert_eq!(
            tag.blocks[0],
            [CC_MAGIC_1, CC_VERSION_OPEN, (112 - 4) / 8, 0x00]
        );
        assert_eq!(tag.writes.last(), Some(&0), "the container goes on last");
        assert_eq!(tag.writes[0], 1);
        written_reads_as(&mut tag, ndef::PAIRING);
    }

    /// A tag that keeps its size to itself still takes the message, behind an MLEN of 0 the reading
    /// already understands.
    #[test]
    fn a_blank_tag_without_system_info_is_written_too() {
        let mut tag = Fake::raw(vec![0x00; 112]);
        tag.system_info = false;
        let uid = inventory(&mut tag).unwrap();
        let tlv = ndef::text_tlv("walk_mode").unwrap();
        write_ndef_area(&mut tag, &uid, &tlv).unwrap();
        assert_eq!(tag.blocks[0], [CC_MAGIC_1, CC_VERSION_OPEN, 0x00, 0x00]);
        written_reads_as(&mut tag, "walk_mode");
    }

    /// A formatted tag keeps its container: block 0 is not written at all.
    #[test]
    fn a_formatted_tag_keeps_its_container() {
        let mut tag = Fake::pairing();
        let cc = tag.blocks[0];
        let uid = inventory(&mut tag).unwrap();
        let tlv = ndef::text_tlv("walk_mode").unwrap();
        write_ndef_area(&mut tag, &uid, &tlv).unwrap();
        assert_eq!(tag.blocks[0], cc);
        assert!(!tag.writes.contains(&0));
        written_reads_as(&mut tag, "walk_mode");
    }

    #[test]
    fn nothing_is_written_to_a_tag_that_cannot_take_it() {
        let tlv = ndef::text_tlv("walk_mode").unwrap();
        let refused = |mut tag: Fake, tlv: &[u8]| {
            let uid = inventory(&mut tag).unwrap();
            let e = write_ndef_area(&mut tag, &uid, tlv)
                .unwrap_err()
                .to_string();
            assert!(tag.writes.is_empty(), "{e}");
            e
        };
        let locked = Fake::sized(&[0xE1, 0x41], &[0x03, 0x00, 0xFE]);
        assert!(refused(locked, &tlv).contains("read-only"));
        let small = Fake::sized(&[0xE1, 0x40], &[0x03, 0x00, 0xFE]);
        let long = ndef::text_tlv(&"a".repeat(40)).unwrap();
        assert!(refused(small, &long).contains("the tag holds 8"));
    }
}
