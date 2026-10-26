//! The beta board's NFC reader bus: `/dev/spidev1.0`, through the spidev ioctls.
//!
//! No SPI crate, for the reason `serial.rs` has none: the interface is a few `libc` calls, and
//! the crates that wrap it pull in more than they save. The ST25R100 wants SPI mode 1 (CPOL 0,
//! CPHA 1), MSB first, eight-bit words (datasheet DS14139 §5.8.2); the board's device tree caps
//! the clock at 5 MHz, which is what this asks for.

use std::io;

/// One full-duplex SPI transaction: chip select low, `tx` out, then as many bytes in as asked,
/// chip select high. What a register read, a FIFO read and a command all are on this chip.
pub trait Bus {
    /// Send `tx`, then clock `rx.len()` more bytes and keep what the chip sent during those.
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> io::Result<()>;
}

/// The SPI clock asked for, Hz. The board's device tree allows up to this.
pub const SPEED_HZ: u32 = 5_000_000;

#[cfg(target_os = "linux")]
pub struct Spidev {
    fd: std::os::fd::OwnedFd,
}

#[cfg(target_os = "linux")]
mod ioctl {
    // From linux/spi/spidev.h: `_IOW('k', nr, type)` = 0x40000000 | size << 16 | 'k' << 8 | nr.
    pub const SPI_IOC_WR_MODE: libc::c_ulong = 0x4001_6b01;
    pub const SPI_IOC_WR_BITS_PER_WORD: libc::c_ulong = 0x4001_6b03;
    pub const SPI_IOC_WR_MAX_SPEED_HZ: libc::c_ulong = 0x4004_6b04;
    /// `SPI_IOC_MESSAGE(1)`: one `spi_ioc_transfer`, 32 bytes.
    pub const SPI_IOC_MESSAGE_1: libc::c_ulong = 0x4020_6b00;
    pub const SPI_MODE_1: u8 = 0x01;

    /// `struct spi_ioc_transfer`.
    #[repr(C)]
    #[derive(Default)]
    pub struct Transfer {
        pub tx_buf: u64,
        pub rx_buf: u64,
        pub len: u32,
        pub speed_hz: u32,
        pub delay_usecs: u16,
        pub bits_per_word: u8,
        pub cs_change: u8,
        pub tx_nbits: u8,
        pub rx_nbits: u8,
        pub word_delay_usecs: u8,
        pub pad: u8,
    }
    const _: () = assert!(std::mem::size_of::<Transfer>() == 32);
}

#[cfg(target_os = "linux")]
impl Spidev {
    pub fn open(path: &std::path::Path) -> io::Result<Self> {
        use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
        use std::os::unix::ffi::OsStrExt;

        let cpath = std::ffi::CString::new(path.as_os_str().as_bytes())
            .map_err(|_| io::Error::from(io::ErrorKind::InvalidInput))?;
        // Safety: a NUL-terminated path; the descriptor is owned from here on.
        let raw = unsafe { libc::open(cpath.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
        if raw < 0 {
            return Err(io::Error::last_os_error());
        }
        // Safety: `raw` is a descriptor this call just opened and nothing else holds.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        let set = |request, value: *const libc::c_void| {
            // Safety: each request takes a pointer to a value of the size it encodes.
            if unsafe { libc::ioctl(fd.as_raw_fd(), request, value) } < 0 {
                Err(io::Error::last_os_error())
            } else {
                Ok(())
            }
        };
        let mode = ioctl::SPI_MODE_1;
        let bits = 8u8;
        let speed = SPEED_HZ;
        set(ioctl::SPI_IOC_WR_MODE, (&raw const mode).cast())?;
        set(ioctl::SPI_IOC_WR_BITS_PER_WORD, (&raw const bits).cast())?;
        set(ioctl::SPI_IOC_WR_MAX_SPEED_HZ, (&raw const speed).cast())?;
        Ok(Self { fd })
    }
}

#[cfg(target_os = "linux")]
impl Bus for Spidev {
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> io::Result<()> {
        use std::os::fd::AsRawFd;
        // One buffer each way covering the whole transaction: the bytes clocked in while `tx` goes
        // out are the chip's tristate, and only what follows is the answer.
        let len = tx.len() + rx.len();
        let mut out = vec![0u8; len];
        out[..tx.len()].copy_from_slice(tx);
        let mut back = vec![0u8; len];
        let transfer = ioctl::Transfer {
            tx_buf: out.as_ptr() as u64,
            rx_buf: back.as_mut_ptr() as u64,
            len: len as u32,
            speed_hz: SPEED_HZ,
            bits_per_word: 8,
            ..Default::default()
        };
        // Safety: both buffers outlive the call and are `len` bytes long.
        let done = unsafe {
            libc::ioctl(
                self.fd.as_raw_fd(),
                ioctl::SPI_IOC_MESSAGE_1,
                &raw const transfer,
            )
        };
        if done < 0 {
            return Err(io::Error::last_os_error());
        }
        rx.copy_from_slice(&back[tx.len()..]);
        Ok(())
    }
}

/// Off Linux there is no spidev to open, as there is no serial port: the type exists so `nfcd`
/// builds and the suite runs on a Mac, and the daemon says why it has nothing to watch.
#[cfg(not(target_os = "linux"))]
pub struct Spidev {
    never: std::convert::Infallible,
}

#[cfg(not(target_os = "linux"))]
impl Spidev {
    pub fn open(_path: &std::path::Path) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "the NFC reader is only driven on Linux",
        ))
    }
}

#[cfg(not(target_os = "linux"))]
impl Bus for Spidev {
    fn transfer(&mut self, _tx: &[u8], _rx: &mut [u8]) -> io::Result<()> {
        match self.never {}
    }
}

#[cfg(all(test, not(target_os = "linux")))]
mod tests {
    use super::*;

    /// `nfcd` names `Spidev` unconditionally, so without this stand-in the daemon — and with it
    /// `cargo test --workspace` — did not compile on a Mac. Opening one says why, the way
    /// `Serial::open` does, rather than pretending there is a bus.
    #[test]
    fn off_linux_a_spidev_cannot_be_opened_and_says_why() {
        let err = Spidev::open(std::path::Path::new("/dev/spidev1.0"))
            .err()
            .expect("no spidev off Linux");
        assert_eq!(err.kind(), io::ErrorKind::Unsupported);
    }
}
