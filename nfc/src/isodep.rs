//! ISO 14443-4, the block protocol a card speaks once selected, which carries the APDUs an e-ink
//! display ([`crate::eink`]) is written with.
//!
//! A card whose SAK has bit 6 set (`20`) is asked for its ATS with a RATS, and from then on every
//! exchange is a *block*: a PCB byte, then what it carries.
//!
//! - **I-blocks** carry the APDU and its answer. Each has a block number, one bit, which the
//!   reader toggles every time an exchange completes; an APDU longer than the card's frame size
//!   (`FSC`, from the ATS) is sent as a chain of I-blocks, each acknowledged by the card with an
//!   R(ACK), and an answer longer than ours comes back the same way.
//! - **R-blocks** acknowledge a chained block (ACK) or ask for one again (NAK).
//! - **S-blocks** are the card asking for more time (WTX), which is answered with the same
//!   multiplier and waited out, and the reader's DESELECT.
//!
//! The card has `FWT` to answer each block, from the `FWI` in its ATS; [`IsoDep::set_fwi`] lets a
//! caller stretch it, which the displays need (their refresh answers late). A block lost on the
//! way — no answer, a CRC error — is recovered the way the standard says (ISO/IEC 14443-4 §7.5.4,
//! rules 4 and 6): an R(NAK) with the current block number, which the card answers by sending
//! its last block again if it had our block, or with an R(ACK) if it had not, and then ours is
//! sent again. Twice; after that the error is the caller's.
//!
//! No CID, no NAD: one card in the field, as everywhere in this crate.

use std::thread::sleep;
use std::time::Duration;

use crate::reader::{Exchange, Reader};
use crate::{Error, Result};

/// RATS, then its parameter: FSDI 8 (frames back of up to 256 bytes, which the reader's FIFO
/// holds) and CID 0.
const RATS: [u8; 2] = [0xe0, 0x80];
/// The card has `FWT` at `FWI` 4 to answer a RATS (ISO/IEC 14443-4 §5.1, 4.8 ms). With margin.
const ATS_WAIT_MS: u64 = 10;

const PCB_I: u8 = 0x02;
const PCB_CHAINING: u8 = 0x10;
const PCB_R_ACK: u8 = 0xa2;
const PCB_R_NAK: u8 = 0xb2;
const PCB_S_DESELECT: u8 = 0xc2;
const PCB_S_WTX: u8 = 0xf2;
const PCB_BLOCK_NUMBER: u8 = 0x01;
const PCB_NAK: u8 = 0x10;

/// How many times a lost block is asked for again before the exchange fails.
const RETRIES: u32 = 2;

/// `FSC` from `FSCI` (ISO/IEC 14443-4 Table 1). Values above 8 are reserved and read as 256.
const FSC: [usize; 9] = [16, 24, 32, 40, 48, 64, 96, 128, 256];

/// What a card says about itself in its ATS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ats {
    /// The longest frame it accepts, PCB and CRC included.
    pub fsc: usize,
    /// How long it may take to answer a block: [`fwt`] of this.
    pub fwi: u8,
    /// How long it needs after the ATS before the first block: [`sfgt`] of this.
    pub sfgi: u8,
    /// The historical bytes: the card's own, which the displays fill with their UID.
    pub historical: Vec<u8>,
}

/// Parse an ATS, CRC already removed: `TL T0 [TA] [TB] [TC] historical…`.
pub fn parse_ats(ats: &[u8]) -> Result<Ats> {
    let Some(&tl) = ats.first() else {
        return Err(Error::Tag("an empty ATS".into()));
    };
    let tl = usize::from(tl);
    if tl != ats.len() {
        return Err(Error::Tag(format!(
            "an ATS saying {tl} bytes in {} bytes",
            ats.len()
        )));
    }
    // Only TL: every default (ISO/IEC 14443-4 §5.2.3).
    let Some(&t0) = ats.get(1) else {
        return Ok(Ats {
            fsc: 32,
            fwi: 4,
            sfgi: 0,
            historical: Vec::new(),
        });
    };
    let fsc = FSC[usize::from(t0 & 0x0f).min(FSC.len() - 1)];
    let mut at = 2;
    let mut next = |present: bool| -> Result<Option<u8>> {
        if !present {
            return Ok(None);
        }
        let byte = *ats
            .get(at)
            .ok_or_else(|| Error::Tag("an ATS cut short of its interface bytes".into()))?;
        at += 1;
        Ok(Some(byte))
    };
    let _ta = next(t0 & 0x10 != 0)?;
    let tb = next(t0 & 0x20 != 0)?;
    let _tc = next(t0 & 0x40 != 0)?;
    let (fwi, sfgi) = tb.map_or((4, 0), |tb| (tb >> 4, tb & 0x0f));
    Ok(Ats {
        fsc,
        // 15 is reserved, and read as the default.
        fwi: if fwi == 15 { 4 } else { fwi },
        sfgi: if sfgi == 15 { 0 } else { sfgi },
        historical: ats[at..].to_vec(),
    })
}

/// The frame waiting time at `fwi`: (256 · 16 / fc) · 2^FWI, 302 µs · 2^FWI — 309 ms at 10, 4.9 s
/// at 14, the most the standard allows.
pub fn fwt(fwi: u8) -> Duration {
    Duration::from_nanos(((256.0 * 16.0 / 13.56e6) * 1e9) as u64 * (1u64 << fwi.min(14)))
}

/// The start-up frame guard time at `sfgi`: the same scale. Nothing at 0.
pub fn sfgt(sfgi: u8) -> Duration {
    if sfgi == 0 { Duration::ZERO } else { fwt(sfgi) }
}

/// An activated card: its frame size, how long it has to answer, and the block number.
#[derive(Debug, Clone)]
pub struct IsoDep {
    fsc: usize,
    fwt: Duration,
    /// The FWT the ATS asked for, which a DESELECT keeps: a card that stopped answering is not
    /// waited on for a stretched one.
    own_fwt: Duration,
    block: u8,
}

impl IsoDep {
    /// Send a RATS to the card just selected, and wait out its guard time. The card's ATS, and the
    /// session to exchange APDUs in.
    pub fn activate<R: Reader + ?Sized>(chip: &mut R) -> Result<(IsoDep, Ats)> {
        let answer = chip.transceive(&RATS, Exchange::framed(ATS_WAIT_MS))?;
        let ats = parse_ats(&answer)?;
        sleep(sfgt(ats.sfgi));
        let dep = IsoDep {
            fsc: ats.fsc,
            fwt: fwt(ats.fwi),
            own_fwt: fwt(ats.fwi),
            block: 0,
        };
        Ok((dep, ats))
    }

    /// Give the card [`fwt`] of `fwi` for every block from now on, whatever its ATS said.
    pub fn set_fwi(&mut self, fwi: u8) {
        self.fwt = fwt(fwi);
    }

    /// One APDU out, its answer back (status word included): chained either way as the frame
    /// sizes need, the card's requests for time granted, a lost block asked for again.
    pub fn exchange<R: Reader + ?Sized>(&mut self, chip: &mut R, apdu: &[u8]) -> Result<Vec<u8>> {
        // PCB before, CRC after.
        let room = self.fsc.saturating_sub(3).max(1);
        let mut pieces = apdu.chunks(room).peekable();
        if apdu.is_empty() {
            return Err(Error::Tag("an empty APDU".into()));
        }
        let mut reply = loop {
            let piece = pieces.next().expect("checked non-empty");
            let last = pieces.peek().is_none();
            let mut frame = Vec::with_capacity(1 + piece.len());
            frame.push(PCB_I | self.block | if last { 0 } else { PCB_CHAINING });
            frame.extend_from_slice(piece);
            let reply = self.send(chip, &frame)?;
            if last {
                break reply;
            }
            // A piece of a chain is acknowledged with our block number.
            if !is_r(reply[0])
                || reply[0] & PCB_NAK != 0
                || reply[0] & PCB_BLOCK_NUMBER != self.block
            {
                return Err(Error::Tag(format!(
                    "block {:02x} where an R(ACK) for a chained block was due",
                    reply[0]
                )));
            }
            self.block ^= 1;
        };
        let mut answer = Vec::new();
        loop {
            if !is_i(reply[0]) {
                return Err(Error::Tag(format!(
                    "block {:02x} where an I-block was due",
                    reply[0]
                )));
            }
            // The card toggles its number on every I-block it gets: one that answers with another
            // number than ours has lost count, and the count is taken from it.
            let theirs = reply[0] & PCB_BLOCK_NUMBER;
            if theirs != self.block {
                tracing::debug!(
                    ours = self.block,
                    theirs,
                    "block number taken from the card"
                );
            }
            self.block = theirs ^ 1;
            answer.extend_from_slice(&reply[1..]);
            if reply[0] & PCB_CHAINING == 0 {
                return Ok(answer);
            }
            reply = self.send(chip, &[PCB_R_ACK | self.block])?;
        }
    }

    /// DESELECT: the card goes to HALT. What a card that stopped answering gets before it is
    /// selected again; its answer does not matter.
    pub fn deselect<R: Reader + ?Sized>(&mut self, chip: &mut R) -> Result<()> {
        chip.transceive(&[PCB_S_DESELECT], self.exchange_for(self.own_fwt))
            .map(|_| ())
    }

    fn exchange_for(&self, wait: Duration) -> Exchange {
        // ΔFWT, the reader's margin on top: FWT / 256 (ISO/IEC 14443-4 §7.2), and a millisecond.
        Exchange::framed((wait + wait / 256).as_millis() as u64 + 1)
    }

    /// One block out and the block that answers it, with WTX granted and a lost block recovered.
    fn send<R: Reader + ?Sized>(&mut self, chip: &mut R, block: &[u8]) -> Result<Vec<u8>> {
        let mut out = block.to_vec();
        let mut wait = self.fwt;
        let mut retries = 0;
        loop {
            match chip.transceive(&out, self.exchange_for(wait)) {
                Ok(reply) if reply.is_empty() => {
                    return Err(Error::Tag("an empty block".into()));
                }
                Ok(reply) if reply[0] & 0xf7 == PCB_S_WTX => {
                    // Granted as asked: the multiplier back, and that many FWTs to answer in.
                    let wtxm = reply.get(1).map_or(1, |m| m & 0x3f).clamp(1, 59);
                    out = vec![PCB_S_WTX, wtxm];
                    wait = self.fwt * u32::from(wtxm);
                }
                // Our R(NAK) answered with an R(ACK) of the other number: the card never had our
                // block. Sent again.
                Ok(reply)
                    if out[0] == PCB_R_NAK | self.block
                        && is_r(reply[0])
                        && reply[0] & PCB_NAK == 0
                        && reply[0] & PCB_BLOCK_NUMBER != self.block =>
                {
                    out = block.to_vec();
                    wait = self.fwt;
                }
                Ok(reply) => return Ok(reply),
                Err(e) if e.is_tag() && retries < RETRIES => {
                    retries += 1;
                    tracing::debug!(error = %e, retries, "block lost; asking again");
                    // An R-block of ours is sent again as it is; anything else is asked for with
                    // an R(NAK).
                    out = if is_r(block[0]) {
                        block.to_vec()
                    } else {
                        vec![PCB_R_NAK | self.block]
                    };
                    wait = self.fwt;
                }
                Err(e) => return Err(e),
            }
        }
    }
}

fn is_i(pcb: u8) -> bool {
    pcb & 0xe2 == 0x02
}

fn is_r(pcb: u8) -> bool {
    pcb & 0xe6 == 0xa2
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A card as a script: what it answers to each frame, in order, and every frame it got.
    #[derive(Default)]
    struct Card {
        sent: Vec<Vec<u8>>,
        waits: Vec<u64>,
        answers: VecDeque<Result<Vec<u8>>>,
    }

    impl Card {
        fn answering(answers: Vec<Result<Vec<u8>>>) -> Self {
            Card {
                answers: answers.into(),
                ..Card::default()
            }
        }
    }

    impl Reader for Card {
        fn reset_field(&mut self) -> Result<()> {
            Ok(())
        }

        fn transceive(&mut self, data: &[u8], exchange: Exchange) -> Result<Vec<u8>> {
            self.sent.push(data.to_vec());
            self.waits.push(exchange.timeout_ms);
            self.answers.pop_front().unwrap_or(Err(Error::NoTag))
        }
    }

    fn session(fsc: usize) -> IsoDep {
        IsoDep {
            fsc,
            fwt: fwt(10),
            own_fwt: fwt(10),
            block: 0,
        }
    }

    /// The display's ATS, as `nfc-test` printed it: FSCI 8, FWI 10, its UID in the historical
    /// bytes.
    #[test]
    fn the_displays_ats() {
        let ats = [
            0x0b, 0x78, 0x80, 0xa0, 0x02, 0x90, 0xee, 0x5e, 0xc1, 0x1d, 0x00,
        ];
        let parsed = parse_ats(&ats).unwrap();
        assert_eq!(parsed.fsc, 256);
        assert_eq!(parsed.fwi, 10);
        assert_eq!(parsed.sfgi, 0);
        assert_eq!(parsed.historical, vec![0x90, 0xee, 0x5e, 0xc1, 0x1d, 0x00]);
    }

    #[test]
    fn an_ats_of_tl_alone_is_every_default_and_a_short_one_is_refused() {
        let ats = parse_ats(&[0x01]).unwrap();
        assert_eq!((ats.fsc, ats.fwi, ats.sfgi), (32, 4, 0));
        assert!(parse_ats(&[0x05, 0x70, 0x80]).is_err(), "TL says more");
        assert!(parse_ats(&[0x03, 0x70, 0x80]).is_err(), "TC missing");
        assert!(parse_ats(&[]).is_err());
    }

    #[test]
    fn frame_waiting_times() {
        assert_eq!(fwt(10).as_millis(), 309);
        assert_eq!(fwt(14).as_millis(), 4949);
        assert_eq!(sfgt(0), Duration::ZERO);
    }

    #[test]
    fn activation_sends_rats_for_256_byte_frames() {
        let mut card = Card::answering(vec![Ok(vec![0x05, 0x78, 0x80, 0xa0, 0x02])]);
        let (dep, ats) = IsoDep::activate(&mut card).unwrap();
        assert_eq!(card.sent, vec![vec![0xe0, 0x80]]);
        assert_eq!(ats.fwi, 10);
        assert_eq!(dep.fsc, 256);
    }

    #[test]
    fn a_short_apdu_is_one_i_block_and_the_number_toggles() {
        let mut card =
            Card::answering(vec![Ok(vec![0x02, 0x90, 0x00]), Ok(vec![0x03, 0x6a, 0x82])]);
        let mut dep = session(256);
        assert_eq!(
            dep.exchange(&mut card, &[0xf0, 0xde, 0, 0, 1]).unwrap(),
            vec![0x90, 0x00]
        );
        assert_eq!(
            dep.exchange(&mut card, &[0xf0, 0xde, 0, 0, 1]).unwrap(),
            vec![0x6a, 0x82]
        );
        assert_eq!(card.sent[0], vec![0x02, 0xf0, 0xde, 0, 0, 1]);
        assert_eq!(card.sent[1][0], 0x03);
        assert_eq!(card.waits[0], 311, "FWT 309 ms, ΔFWT and a millisecond");
    }

    /// The display's write packet: 255 bytes of APDU, past the 253 a 256-byte frame carries.
    #[test]
    fn a_long_apdu_is_chained_and_each_piece_acknowledged() {
        let apdu: Vec<u8> = (0..255).map(|i| i as u8).collect();
        let mut card = Card::answering(vec![
            Ok(vec![0xa2]),             // R(ACK) 0
            Ok(vec![0x03, 0x90, 0x00]), // the answer, block 1
        ]);
        let mut dep = session(256);
        assert_eq!(dep.exchange(&mut card, &apdu).unwrap(), vec![0x90, 0x00]);
        assert_eq!(card.sent[0][0], 0x12, "I-block 0, chaining");
        assert_eq!(card.sent[0].len(), 254, "the frame, CRC aside");
        assert_eq!(card.sent[1][0], 0x03, "I-block 1, last");
        assert_eq!(&card.sent[1][1..], &apdu[253..]);
        assert_eq!(dep.block, 0);
    }

    #[test]
    fn a_chained_answer_is_acknowledged_and_joined() {
        let mut card = Card::answering(vec![
            Ok(vec![0x12, 1, 2]),       // I-block 0, chaining
            Ok(vec![0x03, 3, 0x90, 0]), // I-block 1, last
        ]);
        let mut dep = session(256);
        assert_eq!(
            dep.exchange(&mut card, &[0x00, 0xb0]).unwrap(),
            vec![1, 2, 3, 0x90, 0]
        );
        assert_eq!(card.sent[1], vec![0xa3], "R(ACK) with the next number");
    }

    #[test]
    fn a_wait_request_is_granted_with_its_multiplier() {
        let mut card = Card::answering(vec![Ok(vec![0xf2, 0x05]), Ok(vec![0x02, 0x90, 0x00])]);
        let mut dep = session(256);
        assert_eq!(
            dep.exchange(&mut card, &[0xf0, 0xd4, 0x85, 0x80, 0x00])
                .unwrap(),
            vec![0x90, 0x00]
        );
        assert_eq!(card.sent[1], vec![0xf2, 0x05]);
        assert_eq!(card.waits[1], 1553, "five FWTs");
    }

    /// Rule 4: no answer, an R(NAK); the card had our block and sends its answer again.
    #[test]
    fn a_lost_answer_is_asked_for_with_a_nak() {
        let mut card = Card::answering(vec![Err(Error::NoTag), Ok(vec![0x02, 0x90, 0x00])]);
        let mut dep = session(256);
        assert_eq!(
            dep.exchange(&mut card, &[0xf0, 0xde, 0, 0, 1]).unwrap(),
            vec![0x90, 0x00]
        );
        assert_eq!(card.sent[1], vec![0xb2]);
    }

    /// Rule 6: the R(NAK) answered with an R(ACK) of the other number — the card never had our
    /// block, and it is sent again.
    #[test]
    fn a_lost_block_is_sent_again() {
        let mut card = Card::answering(vec![
            Err(Error::Tag("CRC error".into())),
            Ok(vec![0xa3]),
            Ok(vec![0x02, 0x90, 0x00]),
        ]);
        let mut dep = session(256);
        let apdu = [0xf0, 0xde, 0, 0, 1];
        assert_eq!(dep.exchange(&mut card, &apdu).unwrap(), vec![0x90, 0x00]);
        assert_eq!(card.sent[1], vec![0xb2]);
        assert_eq!(card.sent[2], card.sent[0]);
    }

    #[test]
    fn a_card_that_stays_silent_fails_after_two_naks() {
        let mut card = Card::default();
        let mut dep = session(256);
        assert!(matches!(
            dep.exchange(&mut card, &[0xf0, 0xde, 0, 0, 1]),
            Err(Error::NoTag)
        ));
        assert_eq!(card.sent.len(), 3);
        assert_eq!(card.sent[2], vec![0xb2]);
    }

    #[test]
    fn a_stretched_fwt_is_every_blocks_wait() {
        let mut card = Card::answering(vec![Ok(vec![0x02, 0x90, 0x00])]);
        let mut dep = session(256);
        dep.set_fwi(14);
        dep.exchange(&mut card, &[0xf0, 0xd4, 0xff, 0x80, 0x00])
            .unwrap();
        assert_eq!(card.waits[0], 4969);
    }
}
