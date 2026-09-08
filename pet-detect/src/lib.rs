//! Petting detection: a tiny audio classifier that hears head scratches on the onboard mic.
//!
//! - **Features**: 40-band log-mel spectrogram over a 1 s window @ 16 kHz mono.
//! - **Model**: ~20 KB CNN (Conv → BN → ReLU → MaxPool → Conv → BN → ReLU → GAP → Linear),
//!   vendored at `models/pet_detect.onnx` and shipped in the release.
//! - **Output**: [`PettingEvent::Start`] / [`PettingEvent::End`] with hysteresis.
//!
//! Ported from `apirrone/microduck_pet_detect` unchanged in every number: the mel layout is
//! the training contract, and the `pet-features` binary exists precisely so training and
//! inference share this file. The arecord worker and the ambient sound sentry from the
//! prototype's `pet_worker.rs` live in [`worker`].

pub mod worker;

use std::f32::consts::PI;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use ort::session::{Session, builder::GraphOptimizationLevel};
use ort::value::Tensor;
use rustfft::{Fft, FftPlanner, num_complex::Complex32};

pub const SAMPLE_RATE: usize = 16_000;
pub const N_FFT: usize = 512;
/// 10 ms.
pub const HOP: usize = 160;
/// 25 ms.
pub const WIN: usize = 400;
pub const N_MELS: usize = 40;
/// 1.0 s of audio.
pub const WINDOW_FRAMES: usize = 100;
/// 16 240 samples.
pub const WINDOW_SAMPLES: usize = (WINDOW_FRAMES - 1) * HOP + WIN;
pub const FMIN: f32 = 0.0;
pub const FMAX: f32 = 8_000.0;
pub const LOG_EPS: f32 = 1e-6;

/// Log-mel extractor — the exact feature path the model was trained against.
pub struct MelExtractor {
    fft: Arc<dyn Fft<f32>>,
    hann: Vec<f32>,
    /// Sparse: for each mel band, (fft_bin, weight).
    mel_filters: Vec<Vec<(usize, f32)>>,
}

impl MelExtractor {
    pub fn new() -> Self {
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(N_FFT);
        let hann: Vec<f32> = (0..WIN)
            .map(|n| 0.5 - 0.5 * (2.0 * PI * n as f32 / (WIN as f32 - 1.0)).cos())
            .collect();
        let mel_filters = build_mel_filterbank(SAMPLE_RATE, N_FFT, N_MELS, FMIN, FMAX);
        Self {
            fft,
            hann,
            mel_filters,
        }
    }

    /// Process a 1-second window of mono f32 samples in [-1, 1]. Returns the log-mel matrix
    /// flattened row-major, `[N_MELS * WINDOW_FRAMES]`.
    pub fn log_mel(&self, samples: &[f32]) -> Vec<f32> {
        assert!(
            samples.len() >= WINDOW_SAMPLES,
            "need at least {WINDOW_SAMPLES} samples, got {}",
            samples.len()
        );
        let mut out = vec![0.0f32; N_MELS * WINDOW_FRAMES];
        let mut fft_buf = vec![Complex32::new(0.0, 0.0); N_FFT];

        for frame in 0..WINDOW_FRAMES {
            let start = frame * HOP;
            // Windowed frame, zero-padded to N_FFT.
            for (i, slot) in fft_buf.iter_mut().enumerate() {
                *slot = if i < WIN {
                    Complex32::new(samples[start + i] * self.hann[i], 0.0)
                } else {
                    Complex32::new(0.0, 0.0)
                };
            }
            self.fft.process(&mut fft_buf);

            // Power spectrum; only the first N_FFT/2 + 1 bins matter.
            let n_bins = N_FFT / 2 + 1;
            let power: Vec<f32> = fft_buf
                .iter()
                .take(n_bins)
                .map(|c| c.re * c.re + c.im * c.im)
                .collect();

            // Mel filterbank, then log.
            for (m, filt) in self.mel_filters.iter().enumerate() {
                let mut s = 0.0;
                for &(bin, w) in filt {
                    s += w * power[bin];
                }
                out[m * WINDOW_FRAMES + frame] = (s + LOG_EPS).ln();
            }
        }
        out
    }
}

impl Default for MelExtractor {
    fn default() -> Self {
        Self::new()
    }
}

fn hz_to_mel(f: f32) -> f32 {
    2595.0 * (1.0 + f / 700.0).log10()
}
fn mel_to_hz(m: f32) -> f32 {
    700.0 * (10f32.powf(m / 2595.0) - 1.0)
}

fn build_mel_filterbank(
    sr: usize,
    n_fft: usize,
    n_mels: usize,
    fmin: f32,
    fmax: f32,
) -> Vec<Vec<(usize, f32)>> {
    let n_bins = n_fft / 2 + 1;
    let mel_min = hz_to_mel(fmin);
    let mel_max = hz_to_mel(fmax);
    // n_mels + 2 evenly-spaced points on the mel scale.
    let hz_points: Vec<f32> = (0..n_mels + 2)
        .map(|i| mel_to_hz(mel_min + (mel_max - mel_min) * i as f32 / (n_mels + 1) as f32))
        .collect();
    let bin_hz: Vec<f32> = (0..n_bins)
        .map(|k| k as f32 * sr as f32 / n_fft as f32)
        .collect();

    let mut filters = Vec::with_capacity(n_mels);
    for m in 0..n_mels {
        let (lo, ctr, hi) = (hz_points[m], hz_points[m + 1], hz_points[m + 2]);
        let mut filt = Vec::new();
        for (k, &fk) in bin_hz.iter().enumerate() {
            let w = if fk <= lo || fk >= hi {
                0.0
            } else if fk <= ctr {
                (fk - lo) / (ctr - lo)
            } else {
                (hi - fk) / (hi - ctr)
            };
            if w > 0.0 {
                filt.push((k, w));
            }
        }
        filters.push(filt);
    }
    filters
}

/// Load a WAV (downmixing stereo by averaging) and resample linearly to 16 kHz.
/// Returns samples in [-1, 1].
pub fn load_wav_mono_16k(path: &str) -> Result<Vec<f32>> {
    let mut reader = hound::WavReader::open(path)?;
    let spec = reader.spec();
    let channels = spec.channels as usize;
    let raw: Vec<f32> = match spec.sample_format {
        hound::SampleFormat::Int => {
            let scale = (1i64 << (spec.bits_per_sample - 1)) as f32;
            reader
                .samples::<i32>()
                .map(|s| s.map(|v| v as f32 / scale))
                .collect::<Result<Vec<_>, _>>()?
        }
        hound::SampleFormat::Float => reader.samples::<f32>().collect::<Result<Vec<_>, _>>()?,
    };
    let mono: Vec<f32> = if channels == 1 {
        raw
    } else {
        raw.chunks(channels)
            .map(|c| c.iter().sum::<f32>() / channels as f32)
            .collect()
    };
    if spec.sample_rate as usize == SAMPLE_RATE {
        return Ok(mono);
    }
    Ok(resample_linear(
        &mono,
        spec.sample_rate as usize,
        SAMPLE_RATE,
    ))
}

fn resample_linear(input: &[f32], sr_in: usize, sr_out: usize) -> Vec<f32> {
    if input.is_empty() {
        return Vec::new();
    }
    let ratio = sr_in as f64 / sr_out as f64;
    let out_len = (input.len() as f64 / ratio).floor() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let src = i as f64 * ratio;
        let i0 = src.floor() as usize;
        let frac = (src - i0 as f64) as f32;
        let s0 = input[i0];
        let s1 = if i0 + 1 < input.len() {
            input[i0 + 1]
        } else {
            s0
        };
        out.push(s0 * (1.0 - frac) + s1 * frac);
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PettingEvent {
    Start,
    End,
}

/// Streaming petting detector: push 16 kHz mono f32 samples, receive Start/End events on
/// transitions.
///
/// Hysteresis: enters "petting" above `enter_threshold`, leaves below `exit_threshold` —
/// set the exit lower so the boundary doesn't flap.
pub struct PettingDetector {
    session: Session,
    extractor: MelExtractor,
    ring: Vec<f32>,
    samples_until_infer: usize,
    stride: usize,
    is_petting: bool,
    enter_threshold: f32,
    exit_threshold: f32,
}

pub struct PettingDetectorConfig {
    /// Samples between successive inference windows. Smaller = lower latency, more CPU.
    /// The default is ≈ 250 ms.
    pub stride: usize,
    /// Probability above which petting is declared started.
    pub enter_threshold: f32,
    /// Probability below which petting is declared ended.
    pub exit_threshold: f32,
}

impl Default for PettingDetectorConfig {
    fn default() -> Self {
        Self {
            stride: WINDOW_SAMPLES / 4,
            enter_threshold: 0.95,
            // Higher than you'd naively pick: even ambient mic noise hovers around p ≈ 0.7
            // with a small training set, so dropping below 0.85 cleanly means the petting
            // actually stopped.
            exit_threshold: 0.85,
        }
    }
}

impl PettingDetector {
    pub fn new(model_path: &Path, config: PettingDetectorConfig) -> Result<Self> {
        // Single-threaded on purpose: the default one-intra-op-worker-per-core spawns
        // threads that burn CPU on synchronisation overhead for a 20 KB model.
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(1)?
            .with_inter_threads(1)?
            .commit_from_file(model_path)
            .with_context(|| model_path.display().to_string())?;
        Ok(Self {
            session,
            extractor: MelExtractor::new(),
            ring: Vec::with_capacity(WINDOW_SAMPLES * 2),
            samples_until_infer: WINDOW_SAMPLES,
            stride: config.stride,
            is_petting: false,
            enter_threshold: config.enter_threshold,
            exit_threshold: config.exit_threshold,
        })
    }

    /// Push samples; returns the state-change events this batch triggered and the latest
    /// inference probability (when one ran), for logging.
    ///
    /// `audible` is the caller's answer to "has the room made a sound this could possibly
    /// find petting in". False skips the window's features and inference — the expensive
    /// part by far — while still advancing the ring and the schedule, so the audio a later
    /// window sees is continuous and not spliced across the gap.
    ///
    /// **Pass `true` from a caller with no measure of the room**, which is what this did
    /// before there was a gate: analysing a recording wants a probability for every window,
    /// including the quiet ones. `worker::SoundSentry::audible` is the live measure.
    pub fn push_samples(
        &mut self,
        samples: &[f32],
        audible: bool,
    ) -> Result<(Vec<PettingEvent>, Option<f32>)> {
        self.ring.extend_from_slice(samples);
        let mut events = Vec::new();
        let mut last_p = None;

        while self.ring.len() >= self.samples_until_infer {
            let start = self.samples_until_infer - WINDOW_SAMPLES;
            // Advanced here rather than at the end of the body, so that *every* path out of
            // this iteration has advanced it. The skip below is one such path, and a skip
            // that forgot to would not be a missed inference — it would be this loop
            // spinning on the same window until the thread is killed.
            self.samples_until_infer += self.stride;

            // Silence cannot be petting, and finding that out the honest way costs a
            // hundred FFTs and a forward pass.
            //
            // Only while *not* petting. The End edge is found by inference exactly as the
            // Start is, so a session whose room went quiet under a shut gate would never
            // end — the robot would sit there believing it was still being stroked.
            if !audible && !self.is_petting {
                continue;
            }
            let mel = self
                .extractor
                .log_mel(&self.ring[start..start + WINDOW_SAMPLES]);
            let input = Tensor::from_array(([1usize, 1, N_MELS, WINDOW_FRAMES], mel))?;
            let outputs = self.session.run(ort::inputs![input])?;
            let (_shape, probs) = outputs[0].try_extract_tensor::<f32>()?;
            let p = probs[1];
            last_p = Some(p);

            if !self.is_petting && p >= self.enter_threshold {
                self.is_petting = true;
                events.push(PettingEvent::Start);
            } else if self.is_petting && p < self.exit_threshold {
                self.is_petting = false;
                events.push(PettingEvent::End);
            }
        }

        // Cap the ring at 2× window so memory stays bounded.
        if self.ring.len() > WINDOW_SAMPLES * 4 {
            let drop = self.ring.len() - WINDOW_SAMPLES * 2;
            self.ring.drain(..drop);
            self.samples_until_infer -= drop;
        }
        Ok((events, last_p))
    }

    pub fn is_petting(&self) -> bool {
        self.is_petting
    }
}

/// i16 LE samples (what `arecord -f S16_LE` produces) to f32 in [-1, 1].
pub fn i16_to_f32(input: &[i16]) -> Vec<f32> {
    input.iter().map(|&s| f32::from(s) / 32768.0).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mel layout is the training contract; these numbers moving means retraining.
    #[test]
    fn the_feature_contract_is_pinned() {
        assert_eq!(WINDOW_SAMPLES, 16_240);
        assert_eq!(N_MELS * WINDOW_FRAMES, 4_000);
        let filters = build_mel_filterbank(SAMPLE_RATE, N_FFT, N_MELS, FMIN, FMAX);
        assert_eq!(filters.len(), N_MELS);
        assert!(filters.iter().all(|f| !f.is_empty()));
    }

    /// Silence must produce all-floor log-mels; a full-scale tone must not.
    #[test]
    fn log_mel_reacts_to_signal() {
        let extractor = MelExtractor::new();
        let silence = vec![0.0f32; WINDOW_SAMPLES];
        let quiet = extractor.log_mel(&silence);
        assert!(quiet.iter().all(|&v| (v - LOG_EPS.ln()).abs() < 1e-3));

        let tone: Vec<f32> = (0..WINDOW_SAMPLES)
            .map(|i| (2.0 * PI * 440.0 * i as f32 / SAMPLE_RATE as f32).sin())
            .collect();
        let loud = extractor.log_mel(&tone);
        assert!(loud.iter().cloned().fold(f32::MIN, f32::max) > 0.0);
    }
}
