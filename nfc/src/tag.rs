//! ISO-14443A selection and the NFC Forum Type 2 reads, for a NTAG215.
//!
//! One tag is assumed in the field. A collision is reported, not resolved.

use crate::reader::{Exchange, Reader};
use crate::{Error, Result};

const CMD_WUPA: u8 = 0x52;
const CASCADE_LEVELS: [u8; 3] = [0x93, 0x95, 0x97];
/// First byte of a level whose UID continues at the next one.
const CASCADE_TAG: u8 = 0x88;

const CMD_READ: u8 = 0x30;
const CMD_FAST_READ: u8 = 0x3A;
pub const PAGE_USER_START: u8 = 4;
pub const PAGE_USER_END: u8 = 129;

/// A selected tag: its UID, and the SAK its last cascade level answered — which says what it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Selected {
    pub uid: Vec<u8>,
    pub sak: u8,
}

/// Whether a SAK is a MIFARE Classic's: bit 4 (`08`) — `08` for a 1K, `18` for a 4K, `09` for a
/// Mini, `88` for Infineon's 1K. An NTAG answers `00`. A Classic selects like an NTAG and then
/// refuses its READ until a Crypto1 authentication, which nothing here does.
pub fn is_mifare_classic(sak: u8) -> bool {
    sak & 0x08 != 0
}

/// Wake whatever tag is in the field and select it, returning only its UID. See [`select_tag`].
pub fn select<R: Reader>(chip: &mut R) -> Result<Vec<u8>> {
    select_tag(chip).map(|tag| tag.uid)
}

/// Wake whatever tag is in the field — WUPA rather than REQA, since it also wakes one in HALT —
/// and select it.
///
/// Resets the field first: see [`crate::clrc663::Clrc663::reset_field`] for why that is what makes a second read
/// in a session work.
pub fn select_tag<R: Reader>(chip: &mut R) -> Result<Selected> {
    chip.reset_field()?;
    let atqa = chip.transceive(&[CMD_WUPA], Exchange::bare(5, 7))?;
    if atqa.len() != 2 {
        return Err(Error::Tag(format!(
            "ATQA of {} bytes, expected 2",
            atqa.len()
        )));
    }

    let mut uid = Vec::with_capacity(10);
    for (level, sel) in CASCADE_LEVELS.into_iter().enumerate() {
        let level = level + 1;
        // NVB 0x20: no UID bits known, so the tag answers its four for this level and the BCC.
        let answer = chip.transceive(&[sel, 0x20], Exchange::bare(5, 8))?;
        let [a, b, c, d, bcc] = answer[..] else {
            return Err(Error::Tag(format!(
                "level {level}: {} bytes, expected 5",
                answer.len()
            )));
        };
        if bcc != a ^ b ^ c ^ d {
            return Err(Error::Collision(format!("wrong BCC at level {level}")));
        }
        // An all-zero frame has a trivially valid BCC, and it is exactly what two tags that differ
        // from the first bit produce once every bit after the collision reads as zero. No real UID
        // is zero.
        if [a, b, c, d] == [0; 4] {
            return Err(Error::Collision(format!("zero UID at level {level}")));
        }

        let sak = chip.transceive(&[sel, 0x70, a, b, c, d, bcc], Exchange::framed(5))?;
        let [sak] = sak[..] else {
            return Err(Error::Tag(format!(
                "level {level}: SAK of {} bytes, expected 1",
                sak.len()
            )));
        };
        if a == CASCADE_TAG {
            uid.extend([b, c, d]);
        } else {
            uid.extend([a, b, c, d]);
        }
        if sak & 0x04 == 0 {
            return Ok(Selected { uid, sak });
        }
    }
    Err(Error::Tag(
        "UID still incomplete after three cascade levels".into(),
    ))
}

/// READ: 16 bytes, four pages from `page`.
pub fn read_pages<R: Reader>(chip: &mut R, page: u8) -> Result<Vec<u8>> {
    let answer = chip.transceive(&[CMD_READ, page], Exchange::framed(10))?;
    if answer.len() != 16 {
        return Err(Error::Tag(format!(
            "READ page {page}: {} bytes, expected 16",
            answer.len()
        )));
    }
    Ok(answer)
}

/// FAST_READ of pages `first..=last`, in one exchange.
pub fn fast_read<R: Reader>(chip: &mut R, first: u8, last: u8) -> Result<Vec<u8>> {
    let expected = (last - first + 1) as usize * 4;
    // 80 ms rather than 50: 504 bytes at 106 kbit/s is ~43 ms of radio alone, and 7 ms of margin
    // made for intermittent failures depending on how the tag sat on the antenna.
    let answer = chip.transceive(&[CMD_FAST_READ, first, last], Exchange::framed(80))?;
    if answer.len() != expected {
        return Err(Error::Tag(format!(
            "FAST_READ {first}-{last}: {} bytes, expected {expected}",
            answer.len()
        )));
    }
    Ok(answer)
}

/// The NDEF area of the selected tag, reading only as much of it as the message needs.
///
/// A full FAST_READ costs ~130 ms, most of a 200 ms poll. The first READ's 16 bytes carry the TLV
/// header and so the message's length, and an address fits in those 16 or the next — so a pad tag
/// is read in one or two short exchanges, and only a long message pays for the whole memory.
pub fn read_ndef_area<R: Reader>(chip: &mut R) -> Result<Vec<u8>> {
    let mut data = read_pages(chip, PAGE_USER_START)?;
    let Some(needed) = crate::ndef::tlv_size(&data) else {
        return fast_read(chip, PAGE_USER_START, PAGE_USER_END);
    };
    for _ in 0..3 {
        if data.len() >= needed {
            return Ok(data);
        }
        let page = PAGE_USER_START + (data.len() / 4) as u8;
        if page > PAGE_USER_END {
            break;
        }
        data.extend(read_pages(chip, page)?);
    }
    if data.len() >= needed {
        Ok(data)
    } else {
        fast_read(chip, PAGE_USER_START, PAGE_USER_END)
    }
}
