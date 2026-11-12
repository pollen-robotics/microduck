//! ISO-14443A selection and the NFC Forum Type 2 reads and writes, for a NTAG215.
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
const CMD_WRITE: u8 = 0xA2;
/// The capability container: NDEF magic, version, data area size in 8-byte units, access.
const PAGE_CC: u8 = 3;
const CC_MAGIC: u8 = 0xE1;
/// A WRITE's four-bit answer when the page took. Anything else is a NAK.
const ACK: u8 = 0x0A;
pub const PAGE_USER_START: u8 = 4;
pub const PAGE_USER_END: u8 = 129;

/// Wake whatever tag is in the field — WUPA rather than REQA, since it also wakes one in HALT —
/// and select it. Returns its UID.
///
/// Resets the field first: see [`crate::clrc663::Clrc663::reset_field`] for why that is what makes a second read
/// in a session work.
pub fn select<R: Reader>(chip: &mut R) -> Result<Vec<u8>> {
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
            return Ok(uid);
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

/// How many bytes of NDEF this tag holds, from its capability container — or why it cannot be
/// written: not a Type 2 tag at all (a MIFARE Classic, an e-ink display refuse the READ), not
/// formatted for NDEF, or locked.
pub fn ndef_capacity<R: Reader>(chip: &mut R) -> Result<usize> {
    let head = read_pages(chip, PAGE_CC)
        .map_err(|e| Error::Tag(format!("not an NTAG — its capability container: {e}")))?;
    let cc = &head[..4];
    if cc[0] != CC_MAGIC {
        return Err(Error::Tag(format!(
            "not formatted for NDEF: capability container {cc:02x?}"
        )));
    }
    // The low nibble of the access byte is write access: 0 granted, anything else not.
    if cc[3] & 0x0F != 0 {
        return Err(Error::Tag("the tag is locked read-only".into()));
    }
    Ok(usize::from(cc[2]) * 8)
}

/// Write `data` — an NDEF TLV, [`crate::ndef::text_tlv`] — at the start of the user memory, then
/// read it back. `Ok` means the tag holds it.
///
/// Page 4 onward only: the UID, the lock bytes and the capability container (pages 0 to 3) and the
/// configuration pages after the user memory are never written, so nothing here can lock a tag.
/// What the old content had past the new terminator stays, and is never read.
///
/// A tag lifted half way through holds half a message, which reads as no NDEF; writing it again
/// is the fix.
pub fn write_ndef_area<R: Reader>(chip: &mut R, data: &[u8]) -> Result<()> {
    let capacity = ndef_capacity(chip)?;
    if data.len() > capacity {
        return Err(Error::Tag(format!(
            "{} bytes to write, the tag holds {capacity}",
            data.len()
        )));
    }
    // Capacity is a multiple of 8, so the last page padded with zeroes still fits.
    let mut padded = data.to_vec();
    padded.resize(data.len().next_multiple_of(4), 0);
    for (page, bytes) in (PAGE_USER_START..).zip(padded.chunks(4)) {
        write_page(chip, page, bytes)?;
    }

    let mut back = Vec::with_capacity(padded.len() + 16);
    while back.len() < padded.len() {
        back.extend(read_pages(chip, PAGE_USER_START + (back.len() / 4) as u8)?);
    }
    if back[..padded.len()] != padded[..] {
        return Err(Error::Tag(
            "read back differs from what was written — the tag may be locked".into(),
        ));
    }
    Ok(())
}

/// WRITE: four bytes to one page.
///
/// The answer is four bits, an ACK or a NAK, with no CRC. A reader may still call that frame
/// short or mis-framed, so an answer it could not frame is taken as "the tag answered" and left to
/// the read-back to judge; only a clear NAK, or silence, fails here.
fn write_page<R: Reader>(chip: &mut R, page: u8, bytes: &[u8]) -> Result<()> {
    let mut frame = vec![CMD_WRITE, page];
    frame.extend(bytes);
    // The datasheet's write time is 4.1 ms; 10 is the margin the reads take too.
    let exchange = Exchange {
        rx_crc: Some(false),
        ..Exchange::framed(10)
    };
    match chip.transceive(&frame, exchange) {
        Ok(answer) => match answer.first() {
            Some(&nibble) if nibble & 0x0F != ACK => Err(Error::Tag(format!(
                "WRITE page {page} refused (NAK {:x})",
                nibble & 0x0F
            ))),
            _ => Ok(()),
        },
        Err(Error::Tag(_)) => Ok(()),
        Err(e) => Err(e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An NTAG215 as memory: READ, WRITE and nothing else, with pages 0-3 as a factory tag has
    /// them.
    struct Ntag {
        pages: Vec<[u8; 4]>,
        writes: Vec<u8>,
        /// Pages that NAK, as a locked page does.
        locked: Vec<u8>,
    }

    impl Ntag {
        fn new(cc: [u8; 4]) -> Self {
            let mut pages = vec![[0u8; 4]; 135];
            pages[0] = [0x04, 0xA1, 0xB2, 0x9F];
            pages[3] = cc;
            Self {
                pages,
                writes: Vec::new(),
                locked: Vec::new(),
            }
        }

        fn ntag215() -> Self {
            Self::new([CC_MAGIC, 0x10, 0x3F, 0x00])
        }
    }

    impl Reader for Ntag {
        fn reset_field(&mut self) -> Result<()> {
            Ok(())
        }

        fn transceive(&mut self, data: &[u8], _: Exchange) -> Result<Vec<u8>> {
            match *data {
                [CMD_READ, page] => Ok((0..4)
                    .flat_map(|i| self.pages[(page as usize + i) % self.pages.len()])
                    .collect()),
                [CMD_WRITE, page, a, b, c, d] => {
                    self.writes.push(page);
                    if self.locked.contains(&page) {
                        return Ok(vec![0x00]);
                    }
                    self.pages[page as usize] = [a, b, c, d];
                    // The ST25R100 may not frame the four-bit ACK at all.
                    Err(Error::Tag("parity error".into()))
                }
                _ => Err(Error::NoTag),
            }
        }
    }

    #[test]
    fn a_text_is_written_from_page_4_and_reads_back() {
        let mut tag = Ntag::ntag215();
        let data = crate::ndef::text_tlv("roller_skating_mode").unwrap();
        write_ndef_area(&mut tag, &data).unwrap();
        assert_eq!(
            tag.writes,
            (4..4 + data.len().div_ceil(4) as u8).collect::<Vec<_>>()
        );
        let area = read_ndef_area(&mut tag).unwrap();
        assert_eq!(
            crate::ndef::parse(&area).unwrap(),
            [crate::ndef::Record::Text("roller_skating_mode".into())]
        );
    }

    #[test]
    fn nothing_is_written_to_a_tag_that_cannot_take_it() {
        let data = crate::ndef::text_tlv("walk_mode").unwrap();
        let refused = |mut tag: Ntag| {
            let e = write_ndef_area(&mut tag, &data).unwrap_err().to_string();
            assert!(tag.writes.is_empty(), "{e}");
            e
        };
        assert!(refused(Ntag::new([0, 0, 0, 0])).contains("not formatted"));
        assert!(refused(Ntag::new([CC_MAGIC, 0x10, 0x3F, 0x0F])).contains("read-only"));
        // An NTAG213's 144 bytes do not hold 200.
        let mut small = Ntag::new([CC_MAGIC, 0x10, 0x12, 0x00]);
        let long = crate::ndef::text_tlv(&"a".repeat(200)).unwrap();
        let e = write_ndef_area(&mut small, &long).unwrap_err().to_string();
        assert!(e.contains("the tag holds 144"), "{e}");
        assert!(small.writes.is_empty());
    }

    #[test]
    fn a_nak_stops_the_write() {
        let mut tag = Ntag::ntag215();
        tag.locked.push(5);
        let data = crate::ndef::text_tlv("walk_mode").unwrap();
        let e = write_ndef_area(&mut tag, &data).unwrap_err().to_string();
        assert!(e.contains("page 5"), "{e}");
        assert_eq!(tag.writes, [4, 5]);
    }
}
