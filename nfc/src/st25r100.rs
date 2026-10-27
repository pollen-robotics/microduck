//! The ST25R100 on the beta board: power-up, the RF field on either antenna, and transceive.
//!
//! Written from ST's datasheet (DS14139 Rev 5); section numbers below are that document's. The
//! board wires it to `/dev/spidev1.0` and has two single-ended antennas, one on each RFO/RFI pair
//! (schematic sheet 23), so an antenna is chosen in the General register rather than switched
//! outside the chip.
//!
//! Polled rather than interrupt-driven: the IRQ status registers say everything the IRQ line
//! would, `nfcd` polls five times a second, and polling them needs no GPIO access at all — only
//! the SPI device.

use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::reader::{Exchange, Reader};
use crate::spi::Bus;
use crate::{Error, Result};

/// Register addresses (§5.13, Table 5).
mod reg {
    pub const OPERATION: u8 = 0x00;
    pub const GENERAL: u8 = 0x01;
    pub const TX_MOD: u8 = 0x04;
    pub const RX_ANA2: u8 = 0x07;
    pub const RX_DIG: u8 = 0x08;
    pub const CORR1: u8 = 0x09;
    pub const CORR4: u8 = 0x0c;
    pub const CORR5: u8 = 0x0d;
    pub const DISPLAY1: u8 = 0x0f;
    pub const PROTOCOL: u8 = 0x12;
    pub const PROTOCOL_TX1: u8 = 0x13;
    pub const PROTOCOL_RX1: u8 = 0x16;
    pub const NRT_GPT_CONFIG: u8 = 0x1e;
    pub const TX_FRAME1: u8 = 0x34;
    pub const FIFO_STATUS1: u8 = 0x36;
    pub const IRQ_STATUS1: u8 = 0x3c;
    pub const IC_IDENTITY: u8 = 0x3f;
}

/// Direct commands (§5.14, Table 83).
mod cmd {
    pub const SET_DEFAULT: u8 = 0x60;
    pub const STOP_ALL: u8 = 0x62;
    pub const CLEAR_RX_GAIN: u8 = 0x66;
    pub const ADJUST_REGULATORS: u8 = 0x68;
    pub const TRANSMIT_DATA: u8 = 0x6a;
}

// SPI addressing (§5.8.2, Table 3): bit 7 is read, the FIFO is 0x5F.
const READ: u8 = 0x80;
const FIFO: u8 = 0x5f;

// Operation register (§5.13.1).
const OP_TX_EN: u8 = 1 << 5;
const OP_RX_EN: u8 = 1 << 4;
/// The AM regulator, which modulates through TX_MOD `reg_am`. Set in every operation state:
/// without it the ISO 15693 downlink does not reach the tag (measured 2026-10-27).
const OP_AM_EN: u8 = 1 << 3;
const OP_EN: u8 = 1 << 1;
// General register (§5.13.2): single-ended driving on the RFO/RFI pair `rfo2` names.
const GEN_SINGLE: u8 = 1 << 5;
const GEN_RFO2: u8 = 1 << 4;
// Display register 1 (§5.13.16).
const OSC_OK: u8 = 1 << 5;
// Protocol Tx 1 (§5.13.20): parity on, CRC as asked, OOK (tr_am = 0), default pulse width.
const TX_PARITY: u8 = 1 << 6;
const TX_CRC: u8 = 1 << 5;
// Protocol Rx 1 (§5.13.23): the B SOF/EOF bits keep their defaults; parity checked; CRC as asked.
const RX_B_SOF_EOF: u8 = 0b0011_0000;
const RX_PARITY: u8 = 1 << 3;
const RX_CRC: u8 = 1 << 2;
/// The anticollision bit, which the ISO 15693 receiver needs to see a subcarrier answer.
const RX_ANTCL: u8 = 1 << 0;
// IRQ status 1/2/3 (§5.13.61-63).
const I_COL: u8 = 1 << 6;
const I_RXE: u8 = 1 << 3;
const I_RXS: u8 = 1 << 2;
const I_NRE: u8 = 1 << 6;
const I_CRC: u8 = 1 << 3;
const I_PAR: u8 = 1 << 2;
const I_HFE: u8 = 1 << 1;
const I_DCT: u8 = 1 << 4;

/// Correlator 1 (§5.13.10): the IIR coefficients after down-conversion and after decimation.
/// 0xC3 decodes ISO 14443A at 106 kbit/s; the power-on 0x91 does not (see [`St25r100::begin`]).
/// The value is the one ST's own reader library uses for this mode.
const CORR1_NFCA_106: u8 = 0xc3;
/// Correlator 5 (§5.13.14): `dec_f<2:0>`, the IIR decimation factor. 2 for ISO 14443A at 106
/// kbit/s, against 3 at power-on.
const DEC_F_MASK: u8 = 0b0000_0111;
const DEC_F_NFCA_106: u8 = 0b010;
/// 4 for ISO 15693 at 26.48 kbit/s, and `en_subc_end` besides — both what ST's reader library
/// sets for that mode.
const DEC_F_NFCV_26: u8 = 0b100;
const EN_SUBC_END: u8 = 1 << 4;
/// The correlator-1 coefficients for ISO 15693 at 26.48 kbit/s.
const CORR1_NFCV_26: u8 = 0xc0;

/// Protocol register (§5.13.19): `om<3:0>` picks the framing. The rate fields stay 0 — 106 and
/// 26.48 kbit/s are the same code on either side.
const OM_MASK: u8 = 0x0f;
const OM_ISO14443A: u8 = 0x01;
const OM_ISO15693: u8 = 0x05;

/// Rx path digital register (§5.13.9), the filter coefficients: 0x4C for ISO 14443A at 106,
/// 0x08 for ISO 15693 at 26.48 (ST's reader library, both).
const RX_DIG_FILT_MASK: u8 = 0b0111_1100;
const RX_DIG_NFCA_106: u8 = 0x4c;
const RX_DIG_NFCV_26: u8 = 0x08;
/// Correlator 4 (§5.13.13): collision and data levels. Only written for ISO 15693, whose
/// subcarrier reception wants the lower levels; the power-on value serves ISO 14443A.
const CORR4_NFCV_26: u8 = 0xaa;

/// `ic_type` in the identity register's top five bits (§5.13.64).
const IC_TYPE_ST25R100: u8 = 0b10101;
/// The no-response timer's step at `nrt_step = 0`: 64 periods of 13.56 MHz.
const NRT_STEP_US: f64 = 64.0 / 13.56;
/// ISO 14443A: the field must be on this long before the first command (§5.10).
const GUARD: Duration = Duration::from_millis(5);
/// Field off this long puts every tag back in IDLE (the CLRC663 driver measured 10 ms).
const FIELD_RESET: Duration = Duration::from_millis(10);

/// Which of the board's two antennas the field is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Antenna {
    /// RFO1/RFI1.
    One,
    /// RFO2/RFI2.
    Two,
}

/// Which protocol the transceiver frames: the operation mode plus the receiver settings it
/// needs. The per-frame registers [`St25r100::transceive`] programs (parity, CRC) need no
/// change between them — the parity bit does nothing outside ISO 14443A.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Proto {
    /// ISO 14443A at 106 kbit/s: NFC Type 2 tags (NTAG).
    A,
    /// ISO 15693 at 26.48 kbit/s: NFC Type 5 tags (ST25TV, ICODE).
    V,
}

impl Antenna {
    fn general(self) -> u8 {
        GEN_SINGLE
            | match self {
                Antenna::One => 0,
                Antenna::Two => GEN_RFO2,
            }
    }
}

pub struct St25r100<B> {
    bus: B,
    antenna: Antenna,
    proto: Proto,
}

impl<B: Bus> St25r100<B> {
    pub fn new(bus: B, antenna: Antenna) -> Self {
        Self {
            bus,
            antenna,
            proto: Proto::A,
        }
    }

    pub fn antenna(&self) -> Antenna {
        self.antenna
    }

    fn read(&mut self, reg: u8) -> Result<u8> {
        let mut v = [0u8];
        self.bus.transfer(&[READ | reg], &mut v)?;
        Ok(v[0])
    }

    fn read_n(&mut self, reg: u8, out: &mut [u8]) -> Result<()> {
        Ok(self.bus.transfer(&[READ | reg], out)?)
    }

    fn write(&mut self, reg: u8, values: &[u8]) -> Result<()> {
        let mut tx = Vec::with_capacity(1 + values.len());
        tx.push(reg & 0x7f);
        tx.extend_from_slice(values);
        Ok(self.bus.transfer(&tx, &mut [])?)
    }

    fn command(&mut self, code: u8) -> Result<()> {
        Ok(self.bus.transfer(&[code], &mut [])?)
    }

    /// The three IRQ status registers in one read, which also clears them (§5.8.1).
    fn irqs(&mut self) -> Result<[u8; 3]> {
        let mut v = [0u8; 3];
        self.read_n(reg::IRQ_STATUS1, &mut v)?;
        Ok(v)
    }

    /// Power-up to ready, with the antenna chosen and the regulators adjusted (§5.2), field off.
    pub fn begin(&mut self) -> Result<()> {
        self.command(cmd::SET_DEFAULT)?;
        sleep(Duration::from_millis(2));
        let id = self.read(reg::IC_IDENTITY)?;
        if id >> 3 != IC_TYPE_ST25R100 {
            return Err(Error::WrongChip(id));
        }
        self.write(reg::GENERAL, &[self.antenna.general()])?;
        self.write(reg::OPERATION, &[OP_EN])?;
        self.wait(
            Duration::from_millis(20),
            "the oscillator to start",
            |chip| Ok(chip.read(reg::DISPLAY1)? & OSC_OK != 0),
        )?;
        // Adjusted under load (§5.14.5): transmitter and receiver on for the measurement.
        self.irqs()?;
        self.write(reg::OPERATION, &[OP_EN | OP_RX_EN | OP_TX_EN])?;
        self.command(cmd::ADJUST_REGULATORS)?;
        // Not fatal when slow: the power-on regulator setting works, and the adjustment only
        // improves the supply rejection. On the board it was done well inside 50 ms, once not.
        let adjusted = self.wait(
            Duration::from_millis(200),
            "the regulators to adjust",
            |chip| Ok(chip.irqs()?[2] & I_DCT != 0),
        );
        if let Err(e) = adjusted {
            tracing::warn!(error = %e, "carrying on with the power-on regulator setting");
        }
        // Modulation through the AM regulator at 12%, and the receiver gain ST's reader library
        // initialises to. Its measured-working ISO 15693 configuration, which the power-on
        // values are not: a tag answered nothing until these (2026-10-27).
        let tx_mod = self.read(reg::TX_MOD)?;
        self.write(reg::TX_MOD, &[(tx_mod & !0xf1) | 0x41])?;
        let rx_ana2 = self.read(reg::RX_ANA2)?;
        self.write(reg::RX_ANA2, &[(rx_ana2 & !0x0f) | 0x04])?;
        // The receiver's decoder, set for ISO 14443A at 106 kbit/s. The power-on correlator
        // settings do not decode it: on the board a tag's ATQA raised I_subc_start every time and
        // never I_rxs. With these it decoded every time (2026-10-06).
        self.set_proto(Proto::A)?;
        self.write(reg::OPERATION, &[OP_EN | OP_AM_EN])
    }

    /// Frame the transceiver for one protocol: the operation mode, the digital filters and the
    /// correlator. The register values are ST's own reader library's for each mode; the two
    /// were measured working on this board (2026-10-27, the beta's reader and an ST25TV).
    pub fn set_proto(&mut self, proto: Proto) -> Result<()> {
        self.proto = proto;
        let (om, rx_dig, corr1, dec_f) = match proto {
            Proto::A => (
                OM_ISO14443A,
                RX_DIG_NFCA_106,
                CORR1_NFCA_106,
                DEC_F_NFCA_106,
            ),
            Proto::V => (OM_ISO15693, RX_DIG_NFCV_26, CORR1_NFCV_26, DEC_F_NFCV_26),
        };
        let protocol = self.read(reg::PROTOCOL)?;
        self.write(reg::PROTOCOL, &[(protocol & !OM_MASK) | om])?;
        let dig = self.read(reg::RX_DIG)?;
        self.write(reg::RX_DIG, &[(dig & !RX_DIG_FILT_MASK) | rx_dig])?;
        self.write(reg::CORR1, &[corr1])?;
        let corr5 = self.read(reg::CORR5)?;
        let subc = if matches!(proto, Proto::V) {
            EN_SUBC_END
        } else {
            0
        };
        self.write(
            reg::CORR5,
            &[(corr5 & !(DEC_F_MASK | EN_SUBC_END)) | subc | dec_f],
        )?;
        if matches!(proto, Proto::V) {
            self.write(reg::CORR4, &[CORR4_NFCV_26])?;
        }
        Ok(())
    }

    /// Field off, chip still ready.
    pub fn field_off(&mut self) -> Result<()> {
        self.write(reg::OPERATION, &[OP_EN | OP_AM_EN])
    }

    /// Put the field on the other antenna. Takes effect at the next [`Reader::reset_field`].
    pub fn set_antenna(&mut self, antenna: Antenna) -> Result<()> {
        self.antenna = antenna;
        self.write(reg::OPERATION, &[OP_EN | OP_AM_EN])?;
        self.write(reg::GENERAL, &[antenna.general()])
    }

    fn wait(
        &mut self,
        limit: Duration,
        what: &str,
        mut done: impl FnMut(&mut Self) -> Result<bool>,
    ) -> Result<()> {
        let until = Instant::now() + limit;
        loop {
            if done(self)? {
                return Ok(());
            }
            if Instant::now() > until {
                return Err(Error::Desync(format!("timed out waiting for {what}")));
            }
            sleep(Duration::from_micros(200));
        }
    }
}

/// `ntx` (whole bytes) and `nbtx` (bits after them) for a frame of `len` bytes whose last byte
/// carries `tx_bits` bits (§5.13.53-54).
fn frame_length(len: usize, tx_bits: u8) -> (u16, u8) {
    if tx_bits >= 8 || len == 0 {
        (len as u16, 0)
    } else {
        ((len - 1) as u16, tx_bits)
    }
}

/// Tx frame registers 1 and 2: `ntx<12:5>`, then `ntx<4:0>` above `nbtx<2:0>`.
fn tx_frame(ntx: u16, nbtx: u8) -> [u8; 2] {
    [(ntx >> 5) as u8, ((ntx & 0x1f) as u8) << 3 | (nbtx & 0x07)]
}

/// The no-response timer's count for `timeout_ms`, in its 4.72 µs steps.
fn nrt_ticks(timeout_ms: u64) -> u16 {
    ((timeout_ms as f64 * 1000.0 / NRT_STEP_US).ceil() as u64).clamp(1, 0xffff) as u16
}

/// What a finished exchange's IRQ bits mean, if they mean it is over.
fn outcome(irq: [u8; 3]) -> Option<Result<()>> {
    let [i1, i2, _] = irq;
    if i1 & I_COL != 0 {
        return Some(Err(Error::Collision("bit collision".into())));
    }
    if i2 & I_HFE != 0 {
        return Some(Err(Error::Tag("hard framing error".into())));
    }
    if i2 & I_CRC != 0 {
        return Some(Err(Error::Tag("CRC error".into())));
    }
    if i2 & I_PAR != 0 {
        return Some(Err(Error::Tag("parity error".into())));
    }
    if i1 & I_RXE != 0 {
        return Some(Ok(()));
    }
    // The no-response timer only fires when no reply started (nrt_emd = 0, §5.9).
    if i2 & I_NRE != 0 && i1 & I_RXS == 0 {
        return Some(Err(Error::NoTag));
    }
    None
}

impl<B: Bus> Reader for St25r100<B> {
    fn reset_field(&mut self) -> Result<()> {
        self.write(reg::OPERATION, &[OP_EN | OP_AM_EN])?;
        sleep(FIELD_RESET);
        self.write(reg::OPERATION, &[OP_EN | OP_AM_EN | OP_RX_EN | OP_TX_EN])?;
        sleep(GUARD);
        Ok(())
    }

    /// The transceive sequence of §5.10: stop, clear the gain, frame it, load the FIFO,
    /// transmit, then read the IRQ status until the reception ends or the timer says nothing came.
    fn transceive(&mut self, data: &[u8], exchange: Exchange) -> Result<Vec<u8>> {
        self.command(cmd::STOP_ALL)?;
        self.command(cmd::CLEAR_RX_GAIN)?;
        let tx = TX_PARITY | if exchange.crc { TX_CRC } else { 0 };
        let mut rx = RX_B_SOF_EOF | RX_PARITY | if exchange.rx_crc() { RX_CRC } else { 0 };
        // The 15693 receiver wants the anticollision bit as well — ST's reader library sets it
        // for its inventory, without which a tag's answer does not register (measured
        // 2026-10-27).
        if matches!(self.proto, Proto::V) {
            rx |= RX_ANTCL;
        }
        self.write(reg::PROTOCOL_TX1, &[tx])?;
        self.write(reg::PROTOCOL_RX1, &[rx])?;
        let ticks = nrt_ticks(exchange.timeout_ms).to_be_bytes();
        // NRT GPT config, then NRT1 (MSB) and NRT2 (LSB), auto-incremented: nrt_step 0
        // (4.72 µs), normal mode.
        self.write(reg::NRT_GPT_CONFIG, &[0x00, ticks[0], ticks[1]])?;
        let (ntx, nbtx) = frame_length(data.len(), exchange.tx_bits);
        self.write(reg::TX_FRAME1, &tx_frame(ntx, nbtx))?;
        self.write(FIFO, data)?;
        self.command(cmd::TRANSMIT_DATA)?;

        let until = Instant::now() + Duration::from_millis(exchange.timeout_ms + 20);
        let mut seen = [0u8; 3];
        loop {
            let irq = self.irqs()?;
            for (s, i) in seen.iter_mut().zip(irq) {
                *s |= i;
            }
            if let Some(result) = outcome(seen) {
                result?;
                break;
            }
            if Instant::now() > until {
                return Err(Error::Desync(format!(
                    "no end of exchange (IRQ {:02x} {:02x} {:02x})",
                    seen[0], seen[1], seen[2]
                )));
            }
            sleep(Duration::from_micros(200));
        }

        let mut status = [0u8; 2];
        self.read_n(reg::FIFO_STATUS1, &mut status)?;
        let count = usize::from(status[0]) | (usize::from(status[1] >> 6 & 1) << 8);
        let mut answer = vec![0u8; count];
        if count > 0 {
            self.read_n(FIFO, &mut answer)?;
        }
        // The chip checks a received CRC but leaves it in the FIFO: on the board a SELECT's SAK
        // came back as three bytes. A CRC error has already been reported as I_crc above.
        if exchange.rx_crc() {
            answer.truncate(answer.len().saturating_sub(2));
        }
        Ok(answer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A bus that records every transaction and answers reads from a script.
    #[derive(Default)]
    struct Scripted {
        sent: Vec<Vec<u8>>,
        answers: VecDeque<Vec<u8>>,
    }

    impl Bus for Scripted {
        fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> std::io::Result<()> {
            self.sent.push(tx.to_vec());
            if !rx.is_empty() {
                let answer = self.answers.pop_front().unwrap_or_default();
                for (slot, byte) in rx
                    .iter_mut()
                    .zip(answer.iter().chain(std::iter::repeat(&0)))
                {
                    *slot = *byte;
                }
            }
            Ok(())
        }
    }

    #[test]
    fn register_access_is_bit7_read_and_commands_are_their_code() {
        let mut chip = St25r100::new(Scripted::default(), Antenna::One);
        chip.bus.answers.push_back(vec![0xa9]);
        assert_eq!(chip.read(reg::IC_IDENTITY).unwrap(), 0xa9);
        chip.write(reg::GENERAL, &[0x20]).unwrap();
        chip.command(cmd::STOP_ALL).unwrap();
        assert_eq!(
            chip.bus.sent,
            vec![vec![0xbf], vec![0x01, 0x20], vec![0x62]]
        );
    }

    /// The chip on the board answered 0xA9: ST25R100, revision 1.1.
    #[test]
    fn begin_refuses_another_chip() {
        let mut chip = St25r100::new(Scripted::default(), Antenna::One);
        chip.bus.answers.push_back(vec![0x2a]); // an ST25R3916's identity
        assert!(matches!(chip.begin(), Err(Error::WrongChip(0x2a))));
    }

    #[test]
    fn antennas_are_single_ended_on_either_pair() {
        assert_eq!(Antenna::One.general(), 0x20);
        assert_eq!(Antenna::Two.general(), 0x30);
    }

    /// REQA/WUPA: zero bytes and seven bits (§5.10.1).
    #[test]
    fn a_short_frame_is_no_bytes_and_seven_bits() {
        assert_eq!(frame_length(1, 7), (0, 7));
        assert_eq!(tx_frame(0, 7), [0x00, 0x07]);
        assert_eq!(frame_length(7, 8), (7, 0));
        assert_eq!(tx_frame(7, 0), [0x00, 0x38]);
        assert_eq!(tx_frame(0x123, 0), [0x09, 0x18]);
    }

    #[test]
    fn the_no_response_timer_counts_in_4_72_us() {
        assert_eq!(nrt_ticks(5), 1060);
        assert_eq!(nrt_ticks(80), 16_950);
        assert_eq!(nrt_ticks(1_000), 0xffff, "309 ms is the most it can count");
    }

    #[test]
    fn irq_outcomes() {
        assert!(outcome([0, 0, 0]).is_none(), "still going");
        assert!(outcome([I_RXS, 0, 0]).is_none(), "reply started, not ended");
        assert!(matches!(outcome([I_RXE | I_RXS, 0, 0]), Some(Ok(()))));
        assert!(matches!(outcome([0, I_NRE, 0]), Some(Err(Error::NoTag))));
        assert!(
            outcome([I_RXS, I_NRE, 0]).is_none(),
            "timer fired mid-reply"
        );
        assert!(matches!(
            outcome([I_COL | I_RXE, 0, 0]),
            Some(Err(Error::Collision(_)))
        ));
        assert!(matches!(
            outcome([I_RXE, I_CRC, 0]),
            Some(Err(Error::Tag(_)))
        ));
    }

    /// A whole exchange: the bytes on the bus for a WUPA, and the ATQA read back from the FIFO.
    #[test]
    fn a_wupa_exchange_on_the_bus() {
        let mut chip = St25r100::new(Scripted::default(), Antenna::One);
        chip.bus.answers.push_back(vec![I_RXS | I_RXE, 0, 0]); // IRQ status
        chip.bus.answers.push_back(vec![2, 0]); // FIFO status: two bytes
        chip.bus.answers.push_back(vec![0x44, 0x00]); // ATQA of an NTAG215
        let atqa = chip.transceive(&[0x52], Exchange::bare(5, 7)).unwrap();
        assert_eq!(atqa, vec![0x44, 0x00]);
        let sent = &chip.bus.sent;
        assert_eq!(sent[0], vec![cmd::STOP_ALL]);
        assert_eq!(sent[1], vec![cmd::CLEAR_RX_GAIN]);
        assert_eq!(sent[2], vec![reg::PROTOCOL_TX1, TX_PARITY], "no CRC out");
        assert_eq!(sent[3], vec![reg::PROTOCOL_RX1, 0x38], "no CRC back");
        assert_eq!(sent[5], vec![reg::TX_FRAME1, 0x00, 0x07]);
        assert_eq!(sent[6], vec![FIFO, 0x52]);
        assert_eq!(sent[7], vec![cmd::TRANSMIT_DATA]);
    }

    /// A SELECT's answer is the SAK and its CRC; only the SAK is the answer.
    #[test]
    fn a_received_crc_is_dropped() {
        let mut chip = St25r100::new(Scripted::default(), Antenna::One);
        chip.bus.answers.push_back(vec![I_RXS | I_RXE, 0, 0]);
        chip.bus.answers.push_back(vec![3, 0]);
        chip.bus.answers.push_back(vec![0x00, 0xfe, 0x51]);
        let sak = chip
            .transceive(&[0x93, 0x70, 1, 2, 3, 4, 4], Exchange::framed(5))
            .unwrap();
        assert_eq!(sak, vec![0x00]);
    }

    #[test]
    fn nothing_in_the_field_is_no_tag() {
        let mut chip = St25r100::new(Scripted::default(), Antenna::Two);
        chip.bus.answers.push_back(vec![0, I_NRE, 0]);
        assert!(matches!(
            chip.transceive(&[0x52], Exchange::bare(5, 7)),
            Err(Error::NoTag)
        ));
    }
}
