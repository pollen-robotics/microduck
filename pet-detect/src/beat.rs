//! Hear a pulse on the same PCM the petting CNN uses, and say where in the beat we are.
//!
//! Classical DSP on purpose: spectral flux for onsets, autocorrelation for the period, then
//! a phase-locked loop that *holds* the period — the chorale lesson (`sounds::chorale::beat`)
//! is that re-fitting the tempo at the edge of the window is a worse clock than averaging
//! phase against a held period. A second CNN is only justified if this mic's motor hum
//! defeats flux; that has not been shown.
//!
//! [`BeatState`] is small enough to log. The mapper treats a missing or stale `t` as
//! unlocked. Capture EOF, a gap longer than one beat, or a quarantine (self-audio /
//! self-motion) drop the lock and reset the period — they do not leave `locked: true` on
//! a dead `arecord`.

use std::collections::VecDeque;
use std::f32::consts::PI;
use std::sync::Arc;

use rustfft::{Fft, FftPlanner, num_complex::Complex32};

use crate::{N_FFT, SAMPLE_RATE};

/// One analysis frame: 32 ms at 16 kHz, the same grain as the ambient sentry.
pub const FRAME: usize = 512;

/// Tempo band the stand-policy bob can still read after `cmd_alpha` smoothing.
pub const BPM_MIN: f32 = 60.0;
pub const BPM_MAX: f32 = 140.0;

/// Ignore PCM for this long after a quarantine lifts, then require a fresh lock.
pub const HANGOVER_S: f32 = 1.0;

const HANGOVER_FRAMES: u32 = (HANGOVER_S * SAMPLE_RATE as f32 / FRAME as f32) as u32;

/// Flux history long enough for several periods at 60 BPM (~4 s).
const FLUX_HISTORY: usize = 128;

/// Consecutive agreeing period estimates before `locked` flips true.
const LOCK_FRAMES: u32 = 12;

/// Last-value-wins snapshot the mapper reads. `t` is a frame counter, not wall time: it
/// advances on every analysed frame *and* on capture-loss, so a consumer can tell "this
/// is a new unlocked sample" from "the worker has gone quiet".
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeatState {
    pub t: u64,
    pub locked: bool,
    pub bpm: f32,
    pub phase: f32,
    pub onset_seq: u32,
    pub energy: f32,
}

impl Default for BeatState {
    fn default() -> Self {
        Self {
            t: 0,
            locked: false,
            bpm: 0.0,
            phase: 0.0,
            onset_seq: 0,
            energy: 0.0,
        }
    }
}

impl BeatState {
    /// Distance from phase 0, wrapping: 0 at the onset, 0.5 at the opposite point.
    pub fn phase_to_onset(self) -> f32 {
        wrap01(self.phase).min(1.0 - wrap01(self.phase))
    }
}

/// Streaming beat tracker. Push 16 kHz mono f32 in `[-1, 1]`.
pub struct BeatTracker {
    fft: Arc<dyn Fft<f32>>,
    hann: Vec<f32>,
    prev_mag: Vec<f32>,
    acc: Vec<f32>,
    flux_hist: VecDeque<f32>,
    last_flux: f32,
    rms_floor: f32,
    env: f32,
    frames: u64,
    bpm: f32,
    period_s: f32,
    phase: f32,
    onset_seq: u32,
    locked: bool,
    stable: u32,
    frames_since_onset: u32,
    hangover: u32,
    ignoring: bool,
    last_state: BeatState,
}

impl BeatTracker {
    pub fn new() -> Self {
        let mut planner = FftPlanner::<f32>::new();
        let fft = planner.plan_fft_forward(N_FFT);
        let hann: Vec<f32> = (0..FRAME)
            .map(|n| 0.5 - 0.5 * (2.0 * PI * n as f32 / (FRAME as f32 - 1.0)).cos())
            .collect();
        Self {
            fft,
            hann,
            prev_mag: vec![0.0; N_FFT / 2 + 1],
            acc: Vec::with_capacity(FRAME),
            flux_hist: VecDeque::with_capacity(FLUX_HISTORY),
            last_flux: 0.0,
            rms_floor: 0.003,
            env: 0.0,
            frames: 0,
            bpm: 0.0,
            period_s: 0.0,
            phase: 0.0,
            onset_seq: 0,
            locked: false,
            stable: 0,
            frames_since_onset: u32::MAX / 2,
            hangover: 0,
            ignoring: false,
            last_state: BeatState::default(),
        }
    }

    /// Drop the lock, forget the period, ignore PCM until [`Self::end_quarantine`] plus a
    /// hangover of quiet. Walking, a skill, limp-fall, or the duck's own speaker.
    pub fn quarantine(&mut self) {
        self.ignoring = true;
        self.reset_tempo();
        self.publish_unlocked();
    }

    pub fn end_quarantine(&mut self) {
        if self.ignoring {
            self.ignoring = false;
            self.hangover = HANGOVER_FRAMES;
            self.reset_tempo();
            self.publish_unlocked();
        }
    }

    pub fn quarantined(&self) -> bool {
        self.ignoring || self.hangover > 0
    }

    /// Capture died or a gap longer than one beat arrived. Bumps `t` so a mapper cannot
    /// keep posing on the last locked sample.
    pub fn capture_lost(&mut self) {
        self.acc.clear();
        self.reset_tempo();
        self.publish_unlocked();
    }

    /// Push samples; returns the latest state (last-value-wins).
    pub fn push(&mut self, samples: &[f32]) -> BeatState {
        if self.ignoring {
            // Still consume the clock so `t` moves, but do not let the duck's own noise
            // into the period estimate.
            self.skip_samples(samples.len());
            return self.last_state;
        }
        if self.hangover > 0 {
            self.skip_samples(samples.len());
            let frames = (samples.len() / FRAME).max(1) as u32;
            self.hangover = self.hangover.saturating_sub(frames);
            if self.hangover == 0 {
                self.reset_tempo();
            }
            return self.last_state;
        }
        for &s in samples {
            self.acc.push(s);
            if self.acc.len() >= FRAME {
                let frame: Vec<f32> = self.acc.drain(..FRAME).collect();
                self.analyse_frame(&frame);
            }
        }
        self.last_state
    }

    pub fn state(&self) -> BeatState {
        self.last_state
    }

    fn skip_samples(&mut self, n: usize) {
        let frames = n / FRAME;
        for _ in 0..frames.max(1) {
            self.frames = self.frames.saturating_add(1);
        }
        self.publish_unlocked();
    }

    fn reset_tempo(&mut self) {
        self.locked = false;
        self.stable = 0;
        self.bpm = 0.0;
        self.period_s = 0.0;
        self.phase = 0.0;
        self.flux_hist.clear();
        self.last_flux = 0.0;
        self.frames_since_onset = u32::MAX / 2;
        self.prev_mag.fill(0.0);
        self.acc.clear();
        self.env = 0.0;
    }

    fn analyse_frame(&mut self, frame: &[f32]) {
        self.frames = self.frames.saturating_add(1);
        self.frames_since_onset = self.frames_since_onset.saturating_add(1);

        let rms = (frame.iter().map(|s| s * s).sum::<f32>() / frame.len() as f32).sqrt();
        self.rms_floor = 0.995 * self.rms_floor + 0.005 * rms;
        if rms > self.env {
            self.env = rms;
        } else {
            self.env *= 0.97;
        }
        let energy = ((self.env - self.rms_floor) / (8.0 * self.rms_floor + 1e-4)).clamp(0.0, 1.0);

        let mag = self.magnitude(frame);
        let mut flux = 0.0;
        // High-frequency half of the spectrum: onsets live there; motor rumble does not.
        let lo = mag.len() / 8;
        for (bin, (&m, &p)) in mag.iter().zip(self.prev_mag.iter()).enumerate().skip(lo) {
            let d = m - p;
            if d > 0.0 {
                flux += d * (1.0 + bin as f32 / mag.len() as f32);
            }
        }
        self.prev_mag.copy_from_slice(&mag);

        if self.flux_hist.len() == FLUX_HISTORY {
            self.flux_hist.pop_front();
        }
        self.flux_hist.push_back(flux);

        // Median of recent flux, not a peak-tracking floor: a click train would otherwise
        // raise the bar until the clicks themselves no longer count as onsets.
        let onset_thresh = median_flux(&self.flux_hist).mul_add(4.0, 0.0).max(0.04);
        let refractory = (0.08 * SAMPLE_RATE as f32 / FRAME as f32) as u32;
        let onset = flux > onset_thresh
            && flux > self.last_flux
            && self.frames_since_onset >= refractory;
        self.last_flux = flux;
        if onset {
            self.frames_since_onset = 0;
        }

        if !self.locked {
            if let Some(period) = self.estimate_period() {
                let bpm = 60.0 / period;
                if (BPM_MIN..=BPM_MAX).contains(&bpm) {
                    if self.bpm > 1.0 && (bpm - self.bpm).abs() / self.bpm < 0.08 {
                        self.stable = self.stable.saturating_add(1);
                    } else {
                        self.stable = 1;
                    }
                    self.bpm = bpm;
                    self.period_s = period;
                    if self.stable >= LOCK_FRAMES {
                        self.locked = true;
                        if onset {
                            self.phase = 0.0;
                        }
                    }
                }
            }
        }

        if self.locked && self.period_s > 0.0 {
            let dt = FRAME as f32 / SAMPLE_RATE as f32;
            self.phase = wrap01(self.phase + dt / self.period_s);
            if onset {
                // Phase 0 *is* the onset. Snap, don't chase: a PLL that only nudges leaves
                // a visual lag bigger than the bob itself.
                self.phase = 0.0;
                self.onset_seq = self.onset_seq.wrapping_add(1);
            }
            // Drop the lock when the music (or the click train) actually stops — low
            // energy for more than a beat — not when a single onset is missed.
            let one_beat_frames = (self.period_s * SAMPLE_RATE as f32 / FRAME as f32).max(1.0);
            if energy < 0.08 && self.frames_since_onset as f32 > one_beat_frames {
                self.reset_tempo();
            }
        }

        self.last_state = BeatState {
            t: self.frames,
            locked: self.locked,
            bpm: if self.locked { self.bpm } else { 0.0 },
            phase: if self.locked { wrap01(self.phase) } else { 0.0 },
            onset_seq: self.onset_seq,
            energy,
        };
    }

    fn magnitude(&self, frame: &[f32]) -> Vec<f32> {
        let mut buf = vec![Complex32::new(0.0, 0.0); N_FFT];
        for (i, slot) in buf.iter_mut().enumerate().take(FRAME) {
            *slot = Complex32::new(frame[i] * self.hann[i], 0.0);
        }
        self.fft.process(&mut buf);
        buf.iter()
            .take(N_FFT / 2 + 1)
            .map(|c| (c.re * c.re + c.im * c.im).sqrt())
            .collect()
    }

    fn estimate_period(&self) -> Option<f32> {
        if self.flux_hist.len() < 32 {
            return None;
        }
        let x: Vec<f32> = self.flux_hist.iter().copied().collect();
        let mean = x.iter().sum::<f32>() / x.len() as f32;
        let y: Vec<f32> = x.iter().map(|v| v - mean).collect();
        let min_lag = ((60.0 / BPM_MAX) * SAMPLE_RATE as f32 / FRAME as f32).ceil() as usize;
        let max_lag = ((60.0 / BPM_MIN) * SAMPLE_RATE as f32 / FRAME as f32).floor() as usize;
        let max_lag = max_lag.min(y.len() / 2);
        if min_lag >= max_lag {
            return None;
        }
        let mut best_lag = min_lag;
        let mut best = f32::NEG_INFINITY;
        let mut second = f32::NEG_INFINITY;
        for lag in min_lag..=max_lag {
            let mut acc = 0.0;
            let n = y.len() - lag;
            for i in 0..n {
                acc += y[i] * y[i + lag];
            }
            acc /= n as f32;
            if acc > best {
                second = best;
                best = acc;
                best_lag = lag;
            } else if acc > second {
                second = acc;
            }
        }
        if best < 1e-8 || (second > 0.0 && best < 1.15 * second) {
            return None;
        }
        Some(best_lag as f32 * FRAME as f32 / SAMPLE_RATE as f32)
    }

    fn publish_unlocked(&mut self) {
        self.frames = self.frames.saturating_add(1);
        self.last_state = BeatState {
            t: self.frames,
            locked: false,
            bpm: 0.0,
            phase: 0.0,
            onset_seq: self.onset_seq,
            energy: 0.0,
        };
    }
}

impl Default for BeatTracker {
    fn default() -> Self {
        Self::new()
    }
}

fn wrap01(p: f32) -> f32 {
    let p = p % 1.0;
    if p < 0.0 { p + 1.0 } else { p }
}

fn median_flux(hist: &VecDeque<f32>) -> f32 {
    if hist.is_empty() {
        return 0.0;
    }
    let mut v: Vec<f32> = hist.iter().copied().collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v[v.len() / 2]
}

/// Synthetic click train: short decaying bursts at `bpm`. Test corpus — do not vendor songs.
pub fn click_train(bpm: f32, duration_s: f32) -> Vec<f32> {
    let period = 60.0 / bpm * SAMPLE_RATE as f32;
    let n = (duration_s * SAMPLE_RATE as f32) as usize;
    let mut x = vec![0.0f32; n];
    let click_len = SAMPLE_RATE / 250; // 4 ms
    let mut t = 0.0f32;
    while (t as usize) + 1 < n {
        let i = t as usize;
        for k in 0..click_len {
            if i + k < n {
                let env = (-(k as f32) / 6.0).exp();
                let tone = (2.0 * PI * 2000.0 * k as f32 / SAMPLE_RATE as f32).sin();
                x[i + k] = 0.95 * env * tone;
            }
        }
        t += period;
    }
    x
}

/// A one-shot "quack": decaying noise-plus-formant, not a pulse train. The tracker must
/// not lock onto the duck's own voice.
pub fn quack_fixture(duration_s: f32) -> Vec<f32> {
    let n = (duration_s * SAMPLE_RATE as f32) as usize;
    let mut x = vec![0.0f32; n];
    let voiced = (0.35 * SAMPLE_RATE as f32) as usize;
    let mut seed: u32 = 0xC0FFEE;
    for i in 0..voiced.min(n) {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let noise = (seed as f32 / u32::MAX as f32) * 2.0 - 1.0;
        let t = i as f32 / SAMPLE_RATE as f32;
        let formant = (2.0 * PI * (900.0 + 400.0 * t) * t).sin();
        let env = (-t / 0.12).exp();
        x[i] = 0.6 * env * (0.7 * formant + 0.3 * noise);
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(tracker: &mut BeatTracker, pcm: &[f32]) -> Vec<BeatState> {
        let mut out = Vec::new();
        for chunk in pcm.chunks(FRAME) {
            out.push(tracker.push(chunk));
        }
        out
    }

    fn locked_tail(states: &[BeatState], bpm: f32) -> Vec<&BeatState> {
        states
            .iter()
            .filter(|s| s.locked && (s.bpm - bpm).abs() < 8.0)
            .collect()
    }

    /// The tracker locks onto a synthetic click train in the readable 60–140 band and
    /// reports phase 0 on the beat onset within a generous visual bound.
    #[test]
    fn locks_onto_click_trains_and_phases_the_onset() {
        for bpm in [60.0, 90.0, 120.0, 140.0] {
            let mut tracker = BeatTracker::new();
            let pcm = click_train(bpm, 8.0);
            let states = run(&mut tracker, &pcm);
            let locked = locked_tail(&states, bpm);
            assert!(
                locked.len() > 10,
                "bpm={bpm}: never held a lock (last={:?})",
                states.last()
            );
            let last = *locked.last().unwrap();
            assert!(
                (last.bpm - bpm).abs() < 8.0,
                "bpm={bpm} got {}",
                last.bpm
            );

            // Sample phase at click times after lock, using the last state's period.
            let period = 60.0 / bpm * SAMPLE_RATE as f32;
            let lock_sample = states
                .iter()
                .position(|s| s.locked)
                .expect("locked")
                * FRAME;
            let mut near = 0;
            let mut total = 0;
            let mut t = 0.0f32;
            while (t as usize) < pcm.len() {
                let i = t as usize;
                if i >= lock_sample + SAMPLE_RATE {
                    // Push up to this click and read phase.
                    // The state at frame i/FRAME is already in `states`.
                    let idx = i / FRAME;
                    if idx < states.len() && states[idx].locked {
                        total += 1;
                        if states[idx].phase_to_onset() < 0.18 {
                            near += 1;
                        }
                    }
                }
                t += period;
            }
            assert!(
                total >= 3 && near * 2 >= total,
                "bpm={bpm}: phase 0 on {near}/{total} onsets (want most)"
            );
        }
    }

    /// Clicks stop → `locked` drops. A mapper must not keep posing on a held period.
    #[test]
    fn drops_lock_when_the_clicks_stop() {
        let mut tracker = BeatTracker::new();
        let mut pcm = click_train(100.0, 5.0);
        pcm.extend(std::iter::repeat_n(0.0, SAMPLE_RATE * 3));
        let states = run(&mut tracker, &pcm);
        assert!(states.iter().any(|s| s.locked), "should have locked first");
        let last = states.last().copied().unwrap();
        assert!(!last.locked, "silence must unlock, got {last:?}");
    }

    /// Capture EOF / a gap longer than one beat clears `locked` and bumps `t`.
    #[test]
    fn capture_loss_unlocks_and_bumps_t() {
        let mut tracker = BeatTracker::new();
        let _ = run(&mut tracker, &click_train(110.0, 5.0));
        assert!(tracker.state().locked, "precondition: locked");
        let t_before = tracker.state().t;
        tracker.capture_lost();
        let after = tracker.state();
        assert!(!after.locked);
        assert!(after.t > t_before, "t must move so a mapper cannot reuse the last lock");
    }

    /// A duck-voice-shaped burst is not a beat. One quack must not start a dance.
    #[test]
    fn does_not_lock_onto_a_quack() {
        let mut tracker = BeatTracker::new();
        let mut pcm = quack_fixture(0.5);
        pcm.extend(std::iter::repeat_n(0.0, SAMPLE_RATE * 3));
        pcm.extend(quack_fixture(0.5));
        pcm.extend(std::iter::repeat_n(0.0, SAMPLE_RATE * 3));
        let states = run(&mut tracker, &pcm);
        assert!(
            states.iter().all(|s| !s.locked),
            "quack locked: {:?}",
            states.iter().rev().find(|s| s.locked)
        );
    }

    /// Walking / self-audio quarantine drops the lock and does not relock until a hangover
    /// of non-quarantined PCM has passed.
    #[test]
    fn quarantine_drops_lock_and_needs_a_hangover() {
        let mut tracker = BeatTracker::new();
        let clicks = click_train(100.0, 5.0);
        let _ = run(&mut tracker, &clicks);
        assert!(tracker.state().locked, "precondition");

        tracker.quarantine();
        assert!(!tracker.state().locked);
        let _ = run(&mut tracker, &click_train(100.0, 2.0));
        assert!(
            !tracker.state().locked,
            "must ignore PCM while quarantined"
        );

        tracker.end_quarantine();
        // Hangover is ~1 s of ignoring even after the flag clears.
        let during = run(&mut tracker, &click_train(100.0, 0.6));
        assert!(
            during.iter().all(|s| !s.locked),
            "hangover must still suppress lock"
        );

        let after = run(&mut tracker, &click_train(100.0, 6.0));
        assert!(
            after.iter().any(|s| s.locked),
            "fresh clicks after hangover must be allowed to lock"
        );
    }
}
