//! Crypto1, the stream cipher MIFARE Classic encrypts with, and the tag's nonce generator.
//!
//! The ST25R100 has no Crypto1 of its own (a CLRC663 does), so [`crate::classic`] runs it here and
//! hands the reader frames whose bytes *and* parity bits are already encrypted. This is the cipher
//! as Garcia et al. described it ("Dismantling MIFARE Crypto-1", 2008) and as `crapto1` writes
//! it: a 48-bit LFSR kept as its odd and even bits, a two-layer filter function, and the 16-bit
//! LFSR the tag draws its nonces from.
//!
//! Nothing here is secret or a security boundary: the keys used are the NFC Forum's published
//! ones, which every phone also knows. The cipher is only the price of reading the card.

/// The feedback taps of the 48-bit LFSR, split into the odd and the even bits.
const LF_POLY_ODD: u32 = 0x29_CE5C;
const LF_POLY_EVEN: u32 = 0x87_0804;
const MASK_24: u32 = 0x00FF_FFFF;

/// The cipher's state. Created from a key, it is fed the tag's UID and nonce, then the reader's
/// nonce; after that every bit it outputs is keystream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Crypto1 {
    odd: u32,
    even: u32,
}

/// The filter function: 20 bits of the odd half in, one keystream bit out.
fn filter(x: u32) -> u8 {
    let mut f = (0xf22c0 >> (x & 0xf)) & 16;
    f |= (0x6c9c0 >> ((x >> 4) & 0xf)) & 8;
    f |= (0x3c8b0 >> ((x >> 8) & 0xf)) & 4;
    f |= (0x1e458 >> ((x >> 12) & 0xf)) & 2;
    f |= (0x0d938 >> ((x >> 16) & 0xf)) & 1;
    ((0xEC57_E80Au32 >> f) & 1) as u8
}

impl Crypto1 {
    /// The state a 48-bit key loads, the key's bytes most significant first as they are printed.
    pub fn new(key: &[u8; 6]) -> Self {
        let key = key.iter().fold(0u64, |k, b| k << 8 | u64::from(*b));
        let bit = |n: u32| ((key >> n) & 1) as u32;
        let (mut odd, mut even) = (0u32, 0u32);
        for i in (1..48).rev().step_by(2) {
            odd = odd << 1 | bit((i - 1) ^ 7);
            even = even << 1 | bit(i ^ 7);
        }
        Self { odd, even }
    }

    /// The keystream bit the next clock will output, without clocking — what the parity bit
    /// after a byte is encrypted with.
    pub fn peek(&self) -> u8 {
        filter(self.odd)
    }

    /// One clock: output a keystream bit and shift `input` in. With `encrypted`, `input` is a
    /// ciphertext bit and the plaintext under it is what is shifted in — how the tag takes the
    /// reader's nonce.
    pub fn bit(&mut self, input: u8, encrypted: bool) -> u8 {
        let out = filter(self.odd);
        let feed = u32::from((out & u8::from(encrypted)) ^ (input & 1))
            ^ (LF_POLY_ODD & self.odd)
            ^ (LF_POLY_EVEN & self.even);
        self.even = (self.even << 1 | (feed.count_ones() & 1)) & MASK_24;
        std::mem::swap(&mut self.odd, &mut self.even);
        out
    }

    /// Eight clocks, least significant bit first as the byte goes on air.
    pub fn byte(&mut self, input: u8, encrypted: bool) -> u8 {
        (0..8).fold(0, |ks, i| ks | self.bit(input >> i, encrypted) << i)
    }

    /// Thirty-two clocks over a word whose bytes go on air most significant first, each byte
    /// least significant bit first — how UIDs and nonces are written and sent.
    pub fn word(&mut self, input: u32, encrypted: bool) -> u32 {
        (0..32).fold(0, |ks, i| {
            let n = i ^ 24;
            ks | u32::from(self.bit((input >> n) as u8, encrypted)) << n
        })
    }
}

/// The tag's nonce after `n` more clocks of its 16-bit LFSR. The reader proves it shares the key
/// by answering `successor(nt, 64)`, the tag by answering `successor(nt, 96)`.
pub fn successor(nonce: u32, n: u32) -> u32 {
    let mut x = nonce.swap_bytes();
    for _ in 0..n {
        x = x >> 1 | ((x >> 16 ^ x >> 18 ^ x >> 19 ^ x >> 21) & 1) << 31;
    }
    x.swap_bytes()
}

/// The ISO 14443A parity bit for a byte: odd parity over the byte and the bit.
pub fn odd_parity(byte: u8) -> u8 {
    u8::from(byte.count_ones().is_multiple_of(2))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sniffed authentication, the one `mfkey64`'s help text recovers `FFFFFFFFFFFF` from:
    /// UID 9C599B32, tag nonce 82A4166C, then the encrypted reader nonce, reader answer and tag
    /// answer. Replaying it as the tag would — taking the encrypted reader nonce in — must leave
    /// exactly the keystream that turns both answers into the nonce's successors.
    #[test]
    fn a_sniffed_authentication_replays() {
        let (uid, nt) = (0x9C59_9B32u32, 0x82A4_166Cu32);
        let (nr_enc, ar_enc, at_enc) = (0xA1E4_58CEu32, 0x6EEA_41E0u32, 0x5CAD_F439u32);
        let mut cipher = Crypto1::new(&[0xFF; 6]);
        cipher.word(uid ^ nt, false);
        cipher.word(nr_enc, true);
        assert_eq!(cipher.word(0, false) ^ ar_enc, successor(nt, 64));
        assert_eq!(cipher.word(0, false) ^ at_enc, successor(nt, 96));
    }

    #[test]
    fn a_word_is_four_bytes_most_significant_first() {
        let mut by_word = Crypto1::new(&[0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7]);
        let mut by_byte = by_word.clone();
        let ks = by_word.word(0x1234_5678, false);
        let bytes: Vec<u8> = [0x12, 0x34, 0x56, 0x78]
            .iter()
            .map(|b| by_byte.byte(*b, false))
            .collect();
        assert_eq!(ks.to_be_bytes().to_vec(), bytes);
        assert_eq!(by_word, by_byte);
    }

    #[test]
    fn parity_makes_the_count_odd() {
        assert_eq!(odd_parity(0x00), 1);
        assert_eq!(odd_parity(0x01), 0);
        assert_eq!(odd_parity(0x30), 1);
    }
}
