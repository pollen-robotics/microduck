//! The NFC reader: a CLRC663 on USB serial or an ST25R100 on SPI, reading Type 2 (NTAG) and
//! Type 5 (ISO 15693) tags, and on the ST25R100 NDEF-formatted MIFARE Classic cards too.
//!
//! A port of `ntag663` from the winnie repo, cut down to what `nfcd` does — find a tag, read its
//! NDEF — and without its writer, which stays a bench tool on a laptop. The measurements behind
//! the driver's choices (two soft resets, a field reset before every WUPA, no IRQ polling loop)
//! were made on that board and are kept next to the code they justify.
//!
//! Layered as the Python is: [`transport`] frames register accesses over the serial link,
//! [`clrc663`] and [`st25r100`] are the chips, [`tag`] is ISO-14443A selection and the Type 2
//! reads, [`vtag`] the ISO 15693 inventory and the Type 5 reads, [`classic`] the MIFARE Classic
//! authentication and reads over [`crypto1`], [`ndef`] is pure parsing.
//! [`pairing`], [`modes`] and [`photo`] are what `nfcd` does with a tag, and are the part worth
//! testing on a laptop.

pub mod classic;
pub mod clrc663;
pub mod crypto1;
pub mod modes;
pub mod ndef;
pub mod pairing;
pub mod photo;
pub mod reader;
pub mod registers;
pub mod serial;
pub mod spi;
pub mod st25r100;
pub mod tag;
pub mod transport;
pub mod vtag;

/// Everything that can go wrong between the serial port and a tag.
#[derive(Debug)]
pub enum Error {
    /// The port itself: gone, unplugged, not ours to open.
    Io(std::io::Error),
    /// The link answered, but not in step with what was sent. Recovered by a soft reset.
    Desync(String),
    /// Something answered that is not the chip this driver speaks. Distinct from no answer at all: the wiring is
    /// fine, the silicon is not what this driver speaks — or the port is some other device.
    WrongChip(u8),
    /// No tag in the field. The ordinary answer, five times a second.
    NoTag,
    /// Several tags in the field. Not resolved: one tag is assumed.
    Collision(String),
    /// A tag answered, and wrongly.
    Tag(String),
    /// A tag answered properly and will not give its data to this reader: a MIFARE Classic whose
    /// keys are not the NFC Forum's, or one touched to a reader without Crypto1 framing. Asking
    /// again gives the same answer, so it is said once per touch rather than retried.
    Unreadable(String),
}

impl Error {
    /// Is this about the tag — absent, doubled, badly placed — rather than the reader?
    pub fn is_tag(&self) -> bool {
        matches!(
            self,
            Self::NoTag | Self::Collision(_) | Self::Tag(_) | Self::Unreadable(_)
        )
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::Desync(why) => write!(f, "serial link out of step: {why}"),
            Self::WrongChip(id) => write!(
                f,
                "something answered with identity 0x{id:02X}: not a CLRC66303 (0x{:02X}) nor an ST25R100 (0xA8-0xAF)",
                registers::VERSION_CLRC663
            ),
            Self::NoTag => write!(f, "no tag"),
            Self::Collision(why) => write!(f, "several tags in the field: {why}"),
            Self::Tag(why) | Self::Unreadable(why) => write!(f, "{why}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
