//! Which bus the head ToF is on, and where on it.
//!
//! **The board decides the bus.** The Zero 3W robots carry the sensor on the HAT's I²C bus — a
//! VL53L5CX or a VL53L8CX, at one of two addresses. The beta board wires a VL53L8CX to SPI3, which
//! both its images expose as a spidev ([`BETA_SPI`]); the VL53L5CX has no SPI interface, so there
//! is nothing else that could be there. Trying both buses on every board would mean a beta probing
//! I²C addresses on the face board's bus for a sensor that is not on it, and a Zero 3W opening a
//! spidev it does not have — so the board, declared in `robotd.toml`, picks one, and the flags
//! below force either for bench work.

use std::fmt;
use std::path::{Path, PathBuf};

use robotd_params::board::Board;

/// I²C buses to try when none was named, in order.
///
/// `/dev/i2c-pihat` is the udev symlink `setup-board.sh` installs, which follows
/// the HAT bus; `/dev/i2c-3` is what the `i2c3-pihat` overlay creates and is the
/// answer on a board provisioned before that rule existed. Trying both means a
/// board that predates the rule still finds its sensor, and the log says which
/// path answered.
pub const BUS_CANDIDATES: [&str; 2] = ["/dev/i2c-pihat", "/dev/i2c-3"];

/// I²C addresses to try when none was named.
///
/// 0x29 is the factory default for both generations. 0x52 is where the prototype
/// moved a VL53L5CX when an I²C IMU wanted 0x29 — that IMU is gone, but a sensor
/// programmed then is still at 0x52, and the address survives power cycles.
pub const ADDRESS_CANDIDATES: [u8; 2] = [0x29, 0x52];

/// The beta board's ToF: SPI3, chip select 0. The same node on Seeed's image and on ours, both of
/// which bind it to spidev at 8 MHz.
pub const BETA_SPI: &str = "/dev/spidev3.0";

/// One place to look for the sensor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Link {
    /// A VL53L5CX or VL53L8CX at a 7-bit address on an i2c-dev bus.
    I2c { bus: PathBuf, address: u8 },
    /// A VL53L8CX on a spidev. SPI has no addresses: the chip select is the device.
    Spi { device: PathBuf },
}

impl Link {
    /// The device node, which has to exist before anything is worth trying on it.
    pub fn device(&self) -> &Path {
        match self {
            Self::I2c { bus, .. } => bus,
            Self::Spi { device } => device,
        }
    }
}

impl fmt::Display for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::I2c { bus, address } => write!(f, "{address:#04x} on {}", bus.display()),
            Self::Spi { device } => write!(f, "{} (SPI)", device.display()),
        }
    }
}

/// Where to look, in order.
///
/// `spi` names a spidev and wins over everything; `bus` or `address` force I²C, with whichever
/// half is unset taken from the candidates. Neither: the board's own bus.
pub fn candidates(
    board: Board,
    spi: Option<&Path>,
    bus: Option<&Path>,
    address: Option<u8>,
) -> Vec<Link> {
    if let Some(device) = spi {
        return vec![Link::Spi {
            device: device.to_path_buf(),
        }];
    }
    let forced_i2c = bus.is_some() || address.is_some();
    if board.is_beta_main_board() && !forced_i2c {
        return vec![Link::Spi {
            device: PathBuf::from(BETA_SPI),
        }];
    }
    let buses: Vec<PathBuf> = match bus {
        Some(bus) => vec![bus.to_path_buf()],
        None => BUS_CANDIDATES.iter().map(PathBuf::from).collect(),
    };
    let addresses: Vec<u8> = match address {
        Some(address) => vec![address],
        None => ADDRESS_CANDIDATES.to_vec(),
    };
    buses
        .iter()
        .flat_map(|bus| {
            addresses.iter().map(move |&address| Link::I2c {
                bus: bus.clone(),
                address,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_zero3_sweeps_its_i2c_buses_and_addresses() {
        let links = candidates(Board::Zero3, None, None, None);
        assert_eq!(links.len(), 4);
        assert_eq!(
            links[0],
            Link::I2c {
                bus: "/dev/i2c-pihat".into(),
                address: 0x29
            }
        );
        assert!(links.iter().all(|l| matches!(l, Link::I2c { .. })));
    }

    #[test]
    fn a_beta_looks_on_its_spi_bus_only() {
        for board in [Board::Beta, Board::FaceBoardV1] {
            assert_eq!(
                candidates(board, None, None, None),
                vec![Link::Spi {
                    device: BETA_SPI.into()
                }],
                "{board}"
            );
        }
    }

    /// The flags are for the bench, so they beat the board — either way round.
    #[test]
    fn a_flag_forces_the_bus_whatever_the_board() {
        let spi = candidates(Board::Zero3, Some(Path::new("/dev/spidev1.0")), None, None);
        assert_eq!(
            spi,
            vec![Link::Spi {
                device: "/dev/spidev1.0".into()
            }]
        );

        let i2c = candidates(Board::Beta, None, Some(Path::new("/dev/i2c-3")), Some(0x29));
        assert_eq!(
            i2c,
            vec![Link::I2c {
                bus: "/dev/i2c-3".into(),
                address: 0x29
            }]
        );
    }

    #[test]
    fn a_link_says_where_it_is() {
        assert_eq!(
            Link::I2c {
                bus: "/dev/i2c-3".into(),
                address: 0x29
            }
            .to_string(),
            "0x29 on /dev/i2c-3"
        );
        assert_eq!(
            Link::Spi {
                device: BETA_SPI.into()
            }
            .to_string(),
            "/dev/spidev3.0 (SPI)"
        );
    }
}
