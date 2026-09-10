//! The robot's voice: a tiny seedable synth for pet vocalisations.
//!
//! A single integer seed deterministically derives a [`Personality`] — pitch register,
//! harmonic tilt, nasality, vibrato, quackiness, tempo. Two seeds sound like two different
//! creatures; the same seed always sounds the same. Each sound **tag** has several
//! **variants** (small re-rolls within the same voice) so the duck doesn't sound like a
//! stuck recording.
//!
//! Tags: `alarm`, `greet`, `inquire`, `peck`, `chirp`, `coo`, `wheee`. `wheee` is segmented
//! (start / loop / end) so `robotd` can stream it for as long as the ride lasts.
//!
//! ## Ported from Python, and what that changed
//!
//! This is `apirrone/microduck_sounds` (numpy) rewritten in Rust so a release carries its
//! own voice generator — no venv, no numpy, no ffmpeg on the board. The recipes are ported
//! faithfully, but the random streams are not numpy's, and rendering is 48 kHz native
//! instead of 22.05 kHz + resample — so **every robot's voice re-rolls once** when the bank
//! regenerates. That is a bank-version bump ([`BANK_VERSION`]), the same event as a synth
//! retune upstream, not data loss: the voice is derived from the SoC serial and stays
//! stable from here on, guarded by the pinned-RNG test in `rng.rs`.
//!
//! Not ported: the `parrot` module (mic → learned-phrase squawks) — an experiment nothing
//! in the runtime shipped; it can follow if it ever graduates.

pub mod chorale;
pub mod personality;
pub mod rng;
pub mod stream;
pub mod synth;
pub mod voices;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

pub use personality::Personality;
pub use stream::Stream;
pub use synth::SR;

/// Bump when the synth changes enough that existing banks should re-render on the next
/// install — the `.seed` marker includes it, so old banks stop matching and regenerate.
/// v4 was the last Python bank; v5 is the Rust port (new RNG, 48 kHz native).
pub const BANK_VERSION: u32 = 5;

/// A recipe: personality + variant in, f32 mono at [`SR`] out.
pub type Recipe = fn(&Personality, u32) -> Vec<f32>;

/// Every tag and its recipe. Order is the render order, nothing more.
pub const TAGS: [(&str, Recipe); 7] = [
    ("alarm", voices::alarm),
    ("greet", voices::greet),
    ("inquire", voices::inquire),
    ("peck", voices::peck),
    ("chirp", voices::chirp),
    ("coo", voices::coo),
    ("wheee", voices::wheee),
];

/// How many variants each tag gets. The robot picks a random variant at play time, so more
/// variants directly means a more organic-feeling duck. `chirp` (mouth trigger) and `greet`
/// (wake-up) are the most-heard tags, so they get the most.
pub fn variant_count(tag: &str) -> u32 {
    match tag {
        "greet" | "chirp" => 12,
        "wheee" => 6,
        _ => 10,
    }
}

/// Synthesize one sound. Returns f32 mono at [`SR`].
pub fn render(tag: &str, p: &Personality, variant: u32) -> Result<Vec<f32>> {
    let recipe = TAGS
        .iter()
        .find(|(name, _)| *name == tag)
        .map(|(_, f)| f)
        .with_context(|| format!("unknown tag {tag:?}"))?;
    Ok(recipe(p, variant))
}

/// Write a buffer as a 16-bit mono wav at [`SR`].
pub fn to_wav(buffer: &[f32], path: &Path) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SR,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer =
        hound::WavWriter::create(path, spec).with_context(|| path.display().to_string())?;
    for sample in synth::to_i16(buffer) {
        writer.write_sample(sample)?;
    }
    writer.finalize()?;
    Ok(())
}

/// Render every (tag, variant) pair into `<out_dir>/<tag>/<tag>_<letter>.wav` — the layout
/// `robotd` plays from. Segmented tags write `<tag>_start_<letter>.wav` / `_loop_` /
/// `_end_` triads instead.
pub fn render_all(p: &Personality, out_dir: &Path) -> Result<Vec<PathBuf>> {
    let mut paths = Vec::new();
    for (tag, _) in TAGS {
        for variant in 0..variant_count(tag) {
            let letter = char::from(b'a' + variant as u8);
            if tag == "wheee" {
                let (start, loop_seg, end) = voices::wheee_segments(p, variant);
                for (name, buf) in [("start", start), ("loop", loop_seg), ("end", end)] {
                    let path = out_dir.join(tag).join(format!("{tag}_{name}_{letter}.wav"));
                    to_wav(&buf, &path)?;
                    paths.push(path);
                }
            } else {
                let buf = render(tag, p, variant)?;
                let path = out_dir.join(tag).join(format!("{tag}_{letter}.wav"));
                to_wav(&buf, &path)?;
                paths.push(path);
            }
        }
    }
    Ok(paths)
}

/// The voice seed this hardware derives — the SoC's efuse serial hashed to a u32.
///
/// The serial is burned into the chip, survives reflashes, and the supported boards expose
/// it at the same path. `/etc/machine-id` (unique per OS install) is the fallback for a
/// board without one. sha256 decorrelates consecutive factory serials so two robots from
/// the same batch don't get neighbouring (and thus meaninglessly different) seeds.
pub fn hardware_seed() -> Result<u32> {
    // **One machine, several ducks** — the one situation where deriving identity from hardware is
    // wrong. The simulator runs a robot per container, or several on one laptop, and they would
    // otherwise share a serial and therefore a voice *and a chorale id*: that id is how a duck
    // recognises its own beacon reflected back, so identical ducks drop each other's beacons as
    // their own and can never hear anybody. Nothing on a robot sets this.
    if let Some(named) = std::env::var_os("DUCK_IDENTITY") {
        return Ok(seed_from_id(&named.to_string_lossy()));
    }
    let id = std::fs::read("/proc/device-tree/serial-number")
        .map(|raw| {
            String::from_utf8_lossy(&raw)
                .trim_matches(char::from(0))
                .trim()
                .to_owned()
        })
        .or_else(|_| std::fs::read_to_string("/etc/machine-id").map(|s| s.trim().to_owned()))
        .context("neither /proc/device-tree/serial-number nor /etc/machine-id is readable")?;
    Ok(seed_from_id(&id))
}

/// sha256(id), first 8 hex chars as u32 — the exact derivation the Python installer used,
/// so a robot keeps the seed (and personality traits) it already had.
pub fn seed_from_id(id: &str) -> u32 {
    use sha2::{Digest, Sha256};
    let digest = Sha256::digest(id.as_bytes());
    u32::from_be_bytes([digest[0], digest[1], digest[2], digest[3]])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The seed derivation is the robot's identity — pin it against the shell original
    /// (`sha256sum | cut -c1-8` read as hex).
    #[test]
    fn the_seed_derivation_matches_the_installer() {
        // printf 'test-serial' | sha256sum → c96f1146...
        assert_eq!(seed_from_id("test-serial"), 0xC96F_1146);
    }

    #[test]
    fn render_all_writes_the_layout_the_robot_plays_from() {
        let dir = std::env::temp_dir().join(format!("sounds-test-{}", std::process::id()));
        let p = Personality::from_seed(100);
        let paths = render_all(&p, &dir).unwrap();
        // 6 one-shot tags × their variants + wheee (6 × 3 segments).
        let expected = 10 + 12 + 10 + 10 + 12 + 10 + 6 * 3;
        assert_eq!(paths.len(), expected);
        assert!(dir.join("chirp/chirp_a.wav").exists());
        assert!(dir.join("chirp/chirp_l.wav").exists(), "12 chirp variants");
        assert!(dir.join("wheee/wheee_loop_f.wav").exists());
        std::fs::remove_dir_all(&dir).ok();
    }
}
