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
