//! Bench reader for the beta board, ISO 15693 side: bring the ST25R100 up, switch it to Type 5,
//! and print what an inventory and the NDEF read say on each antenna.
//! `st25r100_v [/dev/spidev1.0] [seconds]`
fn main() -> nfc::Result<()> {
    #[cfg(target_os = "linux")]
    {
        use nfc::reader::Reader;
        use nfc::st25r100::{Antenna, Proto, St25r100};
        use nfc::vtag;
        use std::time::{Duration, Instant};

        let mut args = std::env::args().skip(1);
        let path = args.next().unwrap_or_else(|| "/dev/spidev1.0".to_owned());
        let seconds: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(10);
        let bus = nfc::spi::Spidev::open(std::path::Path::new(&path))?;
        let mut chip = St25r100::new(bus, Antenna::One);
        chip.begin()?;
        chip.set_proto(Proto::V)?;
        println!("ST25R100 up on {path}, ISO 15693; touch a Type 5 tag for {seconds} s");
        let until = Instant::now() + Duration::from_secs(seconds);
        while Instant::now() < until {
            for antenna in [Antenna::One, Antenna::Two] {
                chip.set_antenna(antenna)?;
                chip.reset_field()?;
                match vtag::inventory(&mut chip) {
                    Ok(uid) => {
                        let area = vtag::read_ndef_area(&mut chip, &uid);
                        println!(
                            "{antenna:?}: tag {uid:02x?}  ndef {} bytes  {}",
                            area.as_ref().map(|d| d.len()).unwrap_or(0),
                            area.err().map(|e| e.to_string()).unwrap_or_default()
                        );
                    }
                    Err(nfc::Error::NoTag) => {}
                    Err(e) => println!("{antenna:?}: {e}"),
                }
            }
            std::thread::sleep(Duration::from_millis(150));
        }
    }
    Ok(())
}
