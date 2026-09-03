//! The mic, as a background worker: `arecord` → petting classifier + ambient sound sentry.
//!
//! The audio source is an `arecord` subprocess rather than an in-process ALSA binding — the
//! same pattern the standalone `pet-detect` binary documents, and no new native dependency.
//! The capture device is single-client, so everything that analyses the mic shares this one
//! stream.
//!
//! Ported from the prototype's `pet_worker.rs`; `println!` diagnostics became `tracing`.

use std::io::Read;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Mutex;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::{PettingDetector, PettingDetectorConfig, PettingEvent, i16_to_f32};
use crate::beat::{BeatState, BeatTracker};

/// Ambient sound events, from the same stream the petting classifier consumes. Pure
/// RMS-envelope heuristics — no ML:
///   * `Noise`: a sharp transient (clap, bang, door) — ≤ ~0.38 s of loud.
///   * `Voice`: a sustained utterance (speech, a quack at the duck) — up to ~3 s of loud.
///     Longer runs are continuous noise (vacuum, music) and emit nothing; the adaptive
///     floor absorbs them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SoundEvent {
    Noise,
    Voice,
}

/// 32 ms at 16 kHz.
const SENTRY_FRAME: u32 = 512;

/// RMS envelope watcher over ~32 ms frames with a slowly-adapting ambient floor. Petting
/// sounds are loud on this mic (it's practically a contact mic for head scratches), so
/// events are suppressed while the classifier reports petting (+1 s hangover).
struct SoundSentry {
    floor: f32,
    frame_acc: f32,
    frame_n: u32,
    in_event: bool,
    event_frames: u32,
    quiet_frames: u32,
    cooldown_frames: u32,
    petting_hold_frames: u32,
    /// Loudest frame RMS of the current event, for the tuning log.
    event_peak: f32,
    /// Frames since the last periodic floor log (~every 60 s).
    floor_log_frames: u32,
}

impl SoundSentry {
    fn new() -> Self {
        Self {
            floor: 0.003,
            frame_acc: 0.0,
            frame_n: 0,
            in_event: false,
            event_frames: 0,
            quiet_frames: 0,
            cooldown_frames: 0,
            petting_hold_frames: 0,
            event_peak: 0.0,
            floor_log_frames: 0,
        }
    }

    fn push(&mut self, samples: &[f32], petting: bool, out: &mut Vec<SoundEvent>) {
        for &s in samples {
            self.frame_acc += s * s;
            self.frame_n += 1;
            if self.frame_n >= SENTRY_FRAME {
                let rms = (self.frame_acc / self.frame_n as f32).sqrt();
                self.frame_acc = 0.0;
                self.frame_n = 0;
                self.frame(rms, petting, out);
            }
        }
    }

    fn frame(&mut self, rms: f32, petting: bool, out: &mut Vec<SoundEvent>) {
        if petting {
            self.petting_hold_frames = 31; // ~1 s hangover after petting
        } else if self.petting_hold_frames > 0 {
            self.petting_hold_frames -= 1;
        }
        if self.cooldown_frames > 0 {
            self.cooldown_frames -= 1;
        }
        // Periodic ambient-floor log (~every 60 s at 31 frames/s) so threshold tuning on
        // the robot is data-driven.
        self.floor_log_frames += 1;
        if self.floor_log_frames >= 1875 {
            self.floor_log_frames = 0;
            tracing::debug!(floor = self.floor, "ambient floor");
        }
        // Absolute minimums exist only so 6× a near-zero floor doesn't trigger in silence —
        // the ratio thresholds do the real work. (Samples are ±1-normalised; a low capture
        // gain puts real claps well under an absolute guess.)
        let on_thresh = (self.floor * 6.0).max(0.002);
        let off_thresh = (self.floor * 3.0).max(0.0012);
        if !self.in_event {
            // The ambient floor adapts only from non-event frames (τ ≈ 6 s), so sustained
            // noise (gait servos, music) raises the bar instead of spamming events.
            self.floor = 0.995 * self.floor + 0.005 * rms;
            if rms > on_thresh && self.cooldown_frames == 0 && self.petting_hold_frames == 0 {
                self.in_event = true;
                self.event_frames = 1;
                self.quiet_frames = 0;
                self.event_peak = rms;
            }
        } else {
            self.event_frames += 1;
            self.event_peak = self.event_peak.max(rms);
            if rms < off_thresh {
                self.quiet_frames += 1;
            } else {
                self.quiet_frames = 0;
            }
            if self.event_frames > 94 {
                // > ~3 s: continuous noise — absorb into the floor, no event.
                self.in_event = false;
                self.floor = self.floor.max(rms * 0.5);
                self.cooldown_frames = 31;
            } else if self.quiet_frames >= 3 {
                self.in_event = false;
                self.cooldown_frames = 31; // ≥ 1 s between events
                if self.petting_hold_frames == 0 {
                    let dur = self.event_frames - self.quiet_frames;
                    // ≤ ~0.38 s = transient (clap + its reverb tail); longer = an utterance.
                    let ev = if dur <= 12 {
                        SoundEvent::Noise
                    } else {
                        SoundEvent::Voice
                    };
                    tracing::debug!(
                        event = ?ev,
                        dur_s = dur as f32 * SENTRY_FRAME as f32 / 16000.0,
                        peak = self.event_peak,
                        floor = self.floor,
                        "ambient sound"
                    );
                    out.push(ev);
                }
            }
        }
    }
}

pub struct PetConfig {
    /// ALSA capture device passed to arecord.
    pub alsa_device: String,
    /// Path to the trained ONNX model. `None` runs capture + sentry + beat with no CNN —
    /// dance must not inherit petting's "model on disk or no mic" coupling.
    pub model_path: Option<PathBuf>,
    /// Probability above which petting starts.
    pub enter_threshold: f32,
    /// Probability below which petting ends.
    pub exit_threshold: f32,
}

impl Default for PetConfig {
    fn default() -> Self {
        let lib = PettingDetectorConfig::default();
        Self {
            alsa_device: "plughw:aic3104,0".into(),
            model_path: Some(PathBuf::from("/opt/robot/daemon/current/models/pet_detect.onnx")),
            enter_threshold: lib.enter_threshold,
            exit_threshold: lib.exit_threshold,
        }
    }
}

/// The worker's handle: drain events per tick, shut down on drop of the process.
pub struct PetHandle {
    rx: Receiver<PettingEvent>,
    rx_sound: Receiver<SoundEvent>,
    beat: Arc<Mutex<BeatState>>,
    quarantine: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl PetHandle {
    pub fn spawn(config: PetConfig) -> Result<Self> {
        // Built here, not in the thread, so a missing model or runtime is an error the
        // caller sees instead of a worker that dies quietly on its first breath. Behind a
        // panic catch, because `ort` panics on failures it considers unrecoverable — a
        // missing libonnxruntime must read as "no mic worker", not a dead daemon.
        //
        // Dance capture has no model: skip the CNN entirely rather than inventing a
        // dummy path that then fails to load.
        let detector = match &config.model_path {
            Some(path) => Some(
                std::panic::catch_unwind(|| {
                    PettingDetector::new(
                        path,
                        PettingDetectorConfig {
                            stride: PettingDetectorConfig::default().stride,
                            enter_threshold: config.enter_threshold,
                            exit_threshold: config.exit_threshold,
                        },
                    )
                })
                .unwrap_or_else(|_| Err(anyhow::anyhow!("the ONNX runtime is not loadable")))?,
            ),
            None => None,
        };

        let shutdown = Arc::new(AtomicBool::new(false));
        let quarantine = Arc::new(AtomicBool::new(false));
        let beat = Arc::new(Mutex::new(BeatState::default()));
        let (tx, rx) = mpsc::channel();
        let (tx_sound, rx_sound) = mpsc::channel();
        let sd = shutdown.clone();
        let q = quarantine.clone();
        let beat_slot = beat.clone();
        let dev = config.alsa_device.clone();

        let join = thread::Builder::new()
            .name("pet-worker".into())
            .spawn(move || worker_loop(&dev, detector, &tx, &tx_sound, &beat_slot, &q, &sd))
            .expect("spawn pet worker");

        Ok(Self {
            rx,
            rx_sound,
            beat,
            quarantine,
            shutdown,
            join: Some(join),
        })
    }

    /// Non-blocking; the next petting event if one is queued. Once per control tick.
    pub fn try_recv_event(&self) -> Option<PettingEvent> {
        self.rx.try_recv().ok()
    }

    /// Non-blocking; the next ambient sound event if one is queued.
    pub fn try_recv_sound(&self) -> Option<SoundEvent> {
        self.rx_sound.try_recv().ok()
    }

    /// Last-value-wins beat snapshot. Cheap to copy; the mapper treats a frozen `t` as stale.
    pub fn beat(&self) -> BeatState {
        self.beat.lock().map(|g| *g).unwrap_or_default()
    }

    /// Self-audio / self-motion: the tracker must drop the lock, not just the mapper.
    pub fn set_quarantine(&self, on: bool) {
        self.quarantine.store(on, Ordering::Release);
    }

    pub fn shutdown(mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// Backoff between restarts of a capture that will not stay up: doubling from 250 ms to a
/// cap. A board where `arecord` exists but the codec does not — `configure_audio` fails soft
/// at every step, so a failed DKMS build leaves exactly that — makes `arecord` exit
/// immediately on every spawn. Without a backoff on *that* path (the original only slept
/// when the `arecord` binary itself was missing) the worker fork/execs as fast as the CPU
/// allows for the life of the daemon, with a `warn!` per iteration into the journal.
const RESTART_BACKOFF_MIN: Duration = Duration::from_millis(250);
const RESTART_BACKOFF_MAX: Duration = Duration::from_secs(30);

/// A capture that ran this long is not the failure this backoff is for; the delay resets.
const RESTART_HEALTHY: Duration = Duration::from_secs(5);

/// After this many consecutive immediate failures the per-restart line drops to `debug`.
/// The backoff is at its cap by then and the warning has been said — the worker keeps
/// trying (a codec that comes back should be heard) but stops narrating it.
const RESTART_QUIET_AFTER: u32 = 5;

fn worker_loop(
    alsa_device: &str,
    mut detector: Option<PettingDetector>,
    tx: &Sender<PettingEvent>,
    tx_sound: &Sender<SoundEvent>,
    beat_slot: &Mutex<BeatState>,
    quarantine: &Arc<AtomicBool>,
    shutdown: &Arc<AtomicBool>,
) {
    let mut sentry = SoundSentry::new();
    let mut beat = BeatTracker::new();
    let mut quarantined = false;
    let mut failures: u32 = 0;
    while !shutdown.load(Ordering::Acquire) {
        let now_q = quarantine.load(Ordering::Acquire);
        if now_q && !quarantined {
            beat.quarantine();
            if let Ok(mut slot) = beat_slot.lock() {
                *slot = beat.state();
            }
        } else if !now_q && quarantined {
            beat.end_quarantine();
            if let Ok(mut slot) = beat_slot.lock() {
                *slot = beat.state();
            }
        }
        quarantined = now_q;

        let started = Instant::now();
        match spawn_arecord(alsa_device) {
            Ok(mut c) => {
                if let Err(e) = pump(
                    &mut c,
                    detector.as_mut(),
                    &mut sentry,
                    &mut beat,
                    beat_slot,
                    quarantine,
                    tx,
                    tx_sound,
                    shutdown,
                ) {
                    log_restart(failures, &e.to_string());
                }
                let _ = c.kill();
                let _ = c.wait();
                // EOF, kill, or a gap: the last locked sample must not keep posing.
                beat.capture_lost();
                if let Ok(mut slot) = beat_slot.lock() {
                    *slot = beat.state();
                }
            }
            Err(e) => {
                log_restart(failures, &e.to_string());
                beat.capture_lost();
                if let Ok(mut slot) = beat_slot.lock() {
                    *slot = beat.state();
                }
            }
        }
        if started.elapsed() >= RESTART_HEALTHY {
            failures = 0;
            continue;
        }
        failures = failures.saturating_add(1);
        let backoff = RESTART_BACKOFF_MIN
            .saturating_mul(1u32 << (failures - 1).min(8))
            .min(RESTART_BACKOFF_MAX);
        if failures == RESTART_QUIET_AFTER {
            tracing::warn!(
                device = alsa_device,
                ?backoff,
                "pet worker: the mic will not stay up — retrying quietly from here"
            );
        }
        // Sliced, because `shutdown()` joins this thread: a 30 s sleep would be 30 s of
        // robotd not exiting.
        if !sleep_unless_shutdown(backoff, shutdown) {
            return;
        }
    }
}

fn log_restart(failures: u32, error: &str) {
    if failures < RESTART_QUIET_AFTER {
        tracing::warn!(error, "pet worker: restarting arecord");
    } else {
        tracing::debug!(error, "pet worker: restarting arecord");
    }
}

/// Sleep, waking early if shutdown is asked for. `false` means "stop".
fn sleep_unless_shutdown(total: Duration, shutdown: &Arc<AtomicBool>) -> bool {
    const SLICE: Duration = Duration::from_millis(100);
    let deadline = Instant::now() + total;
    while Instant::now() < deadline {
        if shutdown.load(Ordering::Acquire) {
            return false;
        }
        thread::sleep(SLICE.min(deadline.saturating_duration_since(Instant::now())));
    }
    !shutdown.load(Ordering::Acquire)
}

fn spawn_arecord(device: &str) -> Result<Child> {
    Ok(Command::new("arecord")
        .args([
            "-D", device, "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw",
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()?)
}

fn pump(
    child: &mut Child,
    mut detector: Option<&mut PettingDetector>,
    sentry: &mut SoundSentry,
    beat: &mut BeatTracker,
    beat_slot: &Mutex<BeatState>,
    quarantine: &AtomicBool,
    tx: &Sender<PettingEvent>,
    tx_sound: &Sender<SoundEvent>,
    shutdown: &Arc<AtomicBool>,
) -> Result<()> {
    let mut stdout = child
        .stdout
        .take()
        .ok_or_else(|| anyhow::anyhow!("arecord stdout missing"))?;
    let mut buf = [0u8; 4096];
    let mut i16_buf: Vec<i16> = Vec::with_capacity(2048);

    while !shutdown.load(Ordering::Acquire) {
        let n = stdout.read(&mut buf)?;
        if n == 0 {
            return Err(anyhow::anyhow!("arecord stdout closed (EOF)"));
        }
        if !n.is_multiple_of(2) {
            return Err(anyhow::anyhow!("odd byte count from arecord"));
        }
        i16_buf.clear();
        for chunk in buf[..n].as_chunks::<2>().0 {
            i16_buf.push(i16::from_le_bytes(*chunk));
        }
        let samples = i16_to_f32(&i16_buf);
        if quarantine.load(Ordering::Acquire) {
            beat.quarantine();
        } else {
            beat.end_quarantine();
        }
        if !push_pcm(
            &samples,
            detector.as_deref_mut(),
            sentry,
            beat,
            beat_slot,
            tx,
            tx_sound,
        ) {
            return Ok(());
        }
    }
    Ok(())
}

/// One PCM block into petting (optional), sentry, and beat. Shared so tests can drive the
/// same path without a second `arecord`.
fn push_pcm(
    samples: &[f32],
    detector: Option<&mut PettingDetector>,
    sentry: &mut SoundSentry,
    beat: &mut BeatTracker,
    beat_slot: &Mutex<BeatState>,
    tx: &Sender<PettingEvent>,
    tx_sound: &Sender<SoundEvent>,
) -> bool {
    let petting = if let Some(detector) = detector {
        match detector.push_samples(samples) {
            Ok((events, _)) => {
                for ev in events {
                    if tx.send(ev).is_err() {
                        return false;
                    }
                }
                detector.is_petting()
            }
            Err(_) => detector.is_petting(),
        }
    } else {
        false
    };

    let mut sounds = Vec::new();
    sentry.push(samples, petting, &mut sounds);
    for sev in sounds {
        let _ = tx_sound.send(sev);
    }

    let state = beat.push(samples);
    if let Ok(mut slot) = beat_slot.lock() {
        *slot = state;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::beat::click_train;

    /// Capture + beat must work with no pet ONNX. The CNN is opt-in; dance cannot inherit
    /// "model on disk or no mic".
    #[test]
    fn pumps_pcm_and_beat_with_no_pet_model() {
        let (tx, rx) = mpsc::channel();
        let (tx_sound, _rx_sound) = mpsc::channel();
        let beat_slot = Mutex::new(BeatState::default());
        let mut sentry = SoundSentry::new();
        let mut beat = BeatTracker::new();
        let pcm = click_train(100.0, 6.0);
        for chunk in pcm.chunks(crate::beat::FRAME) {
            assert!(push_pcm(
                chunk,
                None,
                &mut sentry,
                &mut beat,
                &beat_slot,
                &tx,
                &tx_sound,
            ));
        }
        assert!(
            rx.try_recv().is_err(),
            "no detector means no petting events"
        );
        let state = beat_slot.lock().map(|g| *g).unwrap_or_default();
        assert!(
            state.locked,
            "beat tracker must lock from the worker path with no CNN: {state:?}"
        );
        assert!(state.t > 0);
    }

    /// One PCM slice feeds beat (and would feed the CNN if loaded). Not two `arecord`s.
    #[test]
    fn one_pcm_slice_reaches_beat_and_the_petting_slot() {
        let (tx, rx) = mpsc::channel();
        let (tx_sound, rx_sound) = mpsc::channel();
        let beat_slot = Mutex::new(BeatState::default());
        let mut sentry = SoundSentry::new();
        let mut beat = BeatTracker::new();
        let pcm = click_train(110.0, 0.05);
        assert!(push_pcm(
            &pcm,
            None,
            &mut sentry,
            &mut beat,
            &beat_slot,
            &tx,
            &tx_sound,
        ));
        // Detector is the other consumer of this same slice. Absent, it emits nothing —
        // the slot is still the one `push_pcm` writes, not a second capture.
        assert!(rx.try_recv().is_err());
        let _ = rx_sound.try_recv();
        let state = beat_slot.lock().map(|g| *g).unwrap_or_default();
        assert!(state.t > 0, "beat saw the same slice, t={}", state.t);
    }
}

