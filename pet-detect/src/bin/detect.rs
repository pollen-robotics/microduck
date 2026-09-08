//! Live petting detector: 16-bit signed LE mono PCM at 16 kHz on stdin, one line per
//! inference on stdout: `<ts_ms>\t<p>\t<state>`.
//!
//! ```text
//! arecord -D plughw:aic3104,0 -f S16_LE -r 16000 -c 1 -t raw | pet-detect --model <onnx>
//! ```

use std::io::Read;
use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Result, anyhow};
use clap::Parser;
use pet_detect::{PettingDetector, PettingDetectorConfig, WINDOW_SAMPLES, i16_to_f32};

#[derive(Parser)]
#[command(
    name = "pet-detect",
    about = "Hear head scratches on the onboard mic",
    version
)]
struct Args {
    /// Path to the trained ONNX model.
    #[arg(
        long,
        default_value = "/opt/robot/daemon/current/models/pet_detect.onnx"
    )]
    model: PathBuf,
    /// Hop between inference windows in samples (default ≈ 250 ms).
    #[arg(long, default_value_t = WINDOW_SAMPLES / 4)]
    stride: usize,
    /// Probability above which petting is declared started.
    #[arg(long, default_value_t = 0.95)]
    threshold: f32,
    /// Probability below which petting is declared ended (hysteresis).
    #[arg(long, default_value_t = 0.85)]
    exit_threshold: f32,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut detector = PettingDetector::new(
        &args.model,
        PettingDetectorConfig {
            stride: args.stride,
            enter_threshold: args.threshold,
            exit_threshold: args.exit_threshold,
        },
    )?;

    let start_t = Instant::now();
    let stdin = std::io::stdin();
    let mut reader = stdin.lock();
    let mut buf = [0u8; 4096];
    let mut i16_buf: Vec<i16> = Vec::with_capacity(2048);

    loop {
        let n = reader.read(&mut buf)?;
        if n == 0 {
            break;
        }
        if !n.is_multiple_of(2) {
            return Err(anyhow!("odd byte count, expected an i16 stream"));
        }
        i16_buf.clear();
        for chunk in buf[..n].as_chunks::<2>().0 {
            i16_buf.push(i16::from_le_bytes(*chunk));
        }
        // Always audible: this reads a stream somebody is analysing, and a probability per
        // window — including the quiet ones — is the whole output. The live gate is
        // `pet_detect::worker`'s, off the ambient floor it is already measuring.
        let (events, last_p) = detector.push_samples(&i16_to_f32(&i16_buf), true)?;
        if let Some(p) = last_p {
            let state = if detector.is_petting() {
                "petting"
            } else {
                "normal"
            };
            println!("{}\t{:.3}\t{}", start_t.elapsed().as_millis(), p, state);
        }
        for ev in events {
            eprintln!("event: {ev:?}");
        }
    }
    Ok(())
}
