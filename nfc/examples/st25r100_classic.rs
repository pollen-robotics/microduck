//! Bench reader for the beta board, MIFARE Classic side: select whatever ISO 14443A tag is on
//! either antenna, print its UID and SAK, and for a MIFARE Classic go through the NDEF read one
//! step at a time — the MAD sector's authentication, its two blocks, then the whole NDEF area —
//! saying where it stops. `st25r100_classic [/dev/spidev1.0] [seconds]`
fn main() -> nfc::Result<()> {
    #[cfg(target_os = "linux")]
    {
        use nfc::classic::{self, KEY_MAD};
        use nfc::st25r100::{Antenna, St25r100};
        use nfc::{ndef, tag};
        use std::time::{Duration, Instant};

        let mut args = std::env::args().skip(1);
        let path = args.next().unwrap_or_else(|| "/dev/spidev1.0".to_owned());
        let seconds: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);
        let bus = nfc::spi::Spidev::open(std::path::Path::new(&path))?;
        let mut chip = St25r100::new(bus, Antenna::One);
        chip.begin()?;
        println!("ST25R100 up on {path}; touch a MIFARE Classic to either antenna for {seconds} s");
        let until = Instant::now() + Duration::from_secs(seconds);
        let mut last: Option<Vec<u8>> = None;
        while Instant::now() < until {
            let mut present = false;
            for antenna in [Antenna::One, Antenna::Two] {
                chip.set_antenna(antenna)?;
                let found = match tag::select_tag(&mut chip) {
                    Ok(found) => found,
                    Err(nfc::Error::NoTag) => continue,
                    Err(e) => {
                        println!("{antenna:?}: {e}");
                        continue;
                    }
                };
                present = true;
                if last.as_ref() == Some(&found.uid) {
                    continue;
                }
                last = Some(found.uid.clone());
                println!(
                    "{antenna:?}: tag {:02x?}  SAK {:#04x}",
                    found.uid, found.sak
                );
                if !classic::is_classic(found.sak) {
                    println!("  not a MIFARE Classic: st25r100_read reads it");
                    continue;
                }

                let step = Instant::now();
                match classic::authenticate(&mut chip, &found.uid, 0, &KEY_MAD, 0x0bad_cafe) {
                    Ok(mut mad) => {
                        println!(
                            "  MAD key opens sector 0 ({} ms)",
                            step.elapsed().as_millis()
                        );
                        for block in [1, 2] {
                            match mad.read_block(&mut chip, block) {
                                Ok(data) => println!("  block {block}: {data:02x?}"),
                                Err(e) => println!("  block {block}: {e}"),
                            }
                        }
                    }
                    Err(e) => println!("  sector 0 with the MAD key: {e}"),
                }

                let step = Instant::now();
                let selected = tag::select_tag(&mut chip).map(|t| t.uid);
                let area = selected.and_then(|uid| classic::read_ndef_area(&mut chip, &uid));
                let took = step.elapsed().as_millis();
                match area {
                    Ok(data) => println!(
                        "  NDEF area {} bytes in {took} ms: {:?}",
                        data.len(),
                        ndef::parse(&data)
                    ),
                    Err(e) => println!("  NDEF area after {took} ms: {e}"),
                }
            }
            // Lifted: the next touch is read again.
            if !present {
                last = None;
            }
            std::thread::sleep(Duration::from_millis(150));
        }
    }
    Ok(())
}
