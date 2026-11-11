//! What [`crate::tag`] needs from a reader chip, so the ISO-14443A code drives either one: the
//! CLRC663 on USB serial (the bench reader) or the ST25R100 on SPI (the beta board's).

use crate::Result;

/// A reader chip, as the tag layer sees it.
pub trait Reader {
    /// Field off long enough for every tag to forget its state, then on again with the guard
    /// time a tag needs before the first command. See [`crate::clrc663::Clrc663::reset_field`]
    /// for why every poll starts with this.
    fn reset_field(&mut self) -> Result<()>;

    /// Send `data` to the tag and return its answer, CRC and parity already checked and removed.
    fn transceive(&mut self, data: &[u8], exchange: Exchange) -> Result<Vec<u8>>;

    /// Send `bits` bits of `data` as they are — no parity, no CRC added — and return the bits
    /// that came back, unchecked, with how many there are. Each byte least significant bit first,
    /// the bit stream running on across byte boundaries.
    ///
    /// MIFARE Classic needs this: once authenticated, its parity bits are encrypted, so
    /// [`crate::classic`] builds every 9-bit group itself. A reader that cannot frame raw bits
    /// says so, once per touch — the CLRC663 could, through its own Crypto1, but nothing here
    /// drives that yet.
    fn transceive_bits(
        &mut self,
        data: &[u8],
        bits: usize,
        timeout_ms: u64,
    ) -> Result<(Vec<u8>, usize)> {
        let _ = (data, bits, timeout_ms);
        Err(crate::Error::Unreadable(
            "a MIFARE Classic card, and this reader does not read those".into(),
        ))
    }
}

/// How one exchange with a tag is framed.
#[derive(Debug, Clone, Copy)]
pub struct Exchange {
    pub timeout_ms: u64,
    pub tx_bits: u8,
    pub crc: bool,
    pub rx_crc: Option<bool>,
}

impl Exchange {
    /// A standard frame: 8-bit last byte, CRC both ways.
    pub const fn framed(timeout_ms: u64) -> Self {
        Self {
            timeout_ms,
            tx_bits: 8,
            crc: true,
            rx_crc: None,
        }
    }

    /// No CRC either way, as the anticollision and the short requests are.
    pub const fn bare(timeout_ms: u64, tx_bits: u8) -> Self {
        Self {
            timeout_ms,
            tx_bits,
            crc: false,
            rx_crc: None,
        }
    }

    /// Whether the answer carries a CRC: `crc` unless `rx_crc` splits the two directions.
    pub fn rx_crc(&self) -> bool {
        self.rx_crc.unwrap_or(self.crc)
    }
}
