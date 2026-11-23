//! Bench reader for the beta board: bring the ST25R100 up, then poll for an NTAG on both
//! antennas in turn and print what each one sees. `st25r100_read [/dev/spidev1.0] [seconds]`
fn main() -> nfc::Result<()> {
    #[cfg(target_os = "linux")]
    {
        use nfc::st25r100::{Antenna, St25r100};
        use nfc::{ndef, tag};
        use std::time::{Duration, Instant};

        let mut args = std::env::args().skip(1);
        let path = args.next().unwrap_or_else(|| "/dev/spidev1.0".to_owned());
        let seconds: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);
        let bus = nfc::spi::Spidev::open(std::path::Path::new(&path))?;
        let mut chip = St25r100::new(bus, Antenna::One);
        chip.begin()?;
        println!("ST25R100 up on {path}; touch a tag to either antenna for {seconds} s");
        let until = Instant::now() + Duration::from_secs(seconds);
        let mut polls = [0u32; 2];
        while Instant::now() < until {
            for antenna in [Antenna::One, Antenna::Two] {
                chip.set_antenna(antenna)?;
                polls[antenna as usize] += 1;
                match tag::select(&mut chip) {
                    Ok(uid) => {
                        let data = tag::read_ndef_area(&mut chip);
                        let records = data
                            .as_ref()
                            .ok()
                            .and_then(|d| ndef::parse(d).ok())
                            .unwrap_or_default();
                        println!(
                            "{antenna:?}: tag {uid:02x?}  ndef {} bytes  {records:?}  pairing {}",
                            data.as_ref().map(|d| d.len()).unwrap_or(0),
                            ndef::asks_to_pair(&records)
                        );
                    }
                    Err(nfc::Error::NoTag) => {}
                    Err(e) => println!("{antenna:?}: {e}"),
                }
            }
            std::thread::sleep(Duration::from_millis(150));
        }
        println!("polls per antenna: {polls:?}");
    }
    Ok(())
}
