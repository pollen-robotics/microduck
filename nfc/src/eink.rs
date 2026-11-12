//! An e-ink display touched to the reader: for now, what starts the photo sequence.
//!
//! The displays are cards that speak ISO 14443-4 (NFC Forum Type 4, "IsoDep" on a phone), and say
//! so in the SAK they select with: bit 6, `20`. The ones seen so far answer `28` — ISO 14443-4 and
//! a MIFARE Classic 1K emulation — with ATQA `0004`. Every display of one model has the same chip,
//! so the same SAK, which is what lets any of them start the sequence with nothing written on it.
//!
//! **Not the display's identity, only its kind.** Anything else that speaks ISO 14443-4 selects
//! the same way: a bank card, a transit card, a phone emulating a card. Telling a display apart
//! means asking it, with its maker's commands over ISO 14443-4 — which `nfcd` does not speak yet,
//! and which writing the photo to it needs anyway. Until then, this is the rule.
//!
//! It is checked before [`crate::tag::is_mifare_classic`], because a `28` also has the Classic
//! bit: taken for a Classic, a display would be turned away as unreadable.

/// Whether a SAK says ISO 14443-4, which is how a display is told from a tag.
pub fn is_eink(sak: u8) -> bool {
    sak & 0x20 != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_display_is_iso_14443_4_and_a_tag_is_not() {
        // 28: the display seen so far. 20: ISO 14443-4 alone.
        for sak in [0x28, 0x20] {
            assert!(is_eink(sak), "{sak:#x}");
        }
        // An NTAG, the Classics.
        for sak in [0x00, 0x08, 0x18, 0x09, 0x88] {
            assert!(!is_eink(sak), "{sak:#x}");
        }
    }
}
