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
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::{PettingDetector, PettingDetectorConfig, PettingEvent, i16_to_f32};

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

/// How long a sound keeps the classifier armed after it, in [`SENTRY_FRAME`]s.
///
/// **A whole window, derived rather than written down.** A sound heard now is still inside
/// the window the classifier looks at a window from now, so anything shorter would shut the
/// gate on audio still under examination — and a stroke is not continuous anyway, it is
/// scratches with gaps to sit through. Rounded up, because a hold that covers all but the
/// last 23 ms of a window covers the wrong thing.
///
/// Written as arithmetic on [`crate::WINDOW_SAMPLES`] because the obvious literal is wrong:
/// a window is 16 240 samples, which is 1.015 s and *not* the 31 frames that "about a
/// second" suggests.
const AUDIBLE_HOLD_FRAMES: u32 = (crate::WINDOW_SAMPLES as u32).div_ceil(SENTRY_FRAME);

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
    /// Frames left in which the room counts as having made a sound. See [`Self::audible`].
    audible_frames: u32,
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
            audible_frames: 0,
        }
    }

    /// Whether anything in the last second stood out from the room.
    ///
    /// **This is what decides whether the classifier runs at all.** The petting model is
    /// 40 log-mel bands over a second of audio — a hundred 512-point FFTs and a forward
    /// pass, four times a second, for ever. In a quiet room every one of those returns the
    /// same answer, and this is the cheap way to know it in advance: the sentry is already
    /// measuring the room frame by frame, so the gate reads its floor rather than keeping a
    /// second one that could drift from it.
    ///
    /// Armed by the *lower* of the sentry's two thresholds, the one that says an event has
    /// ended rather than the one that says a clap has started. This is not deciding whether
    /// a sound is worth reporting; it is deciding whether a pet is worth looking for, and
    /// the two want very different margins. A stroke gentle enough to stay under three
    /// times the ambient floor for a whole second would be missed — and would have scored
    /// far below the 0.95 enter threshold anyway, on a mic this model was trained through
    /// where a head scratch is loud enough to need muting.
    fn audible(&self) -> bool {
        self.audible_frames > 0
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
        // Arm the classifier, or let the arming run down. Above the event thresholds so it
        // is decided on every frame including the ones inside an event, where the floor is
        // frozen and a petting session is exactly what is going on.
        if rms > off_thresh {
            self.audible_frames = AUDIBLE_HOLD_FRAMES;
        } else {
            self.audible_frames = self.audible_frames.saturating_sub(1);
        }
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
    /// Path to the trained ONNX model.
    pub model_path: PathBuf,
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
            model_path: PathBuf::from("/opt/robot/daemon/current/models/pet_detect.onnx"),
            enter_threshold: lib.enter_threshold,
            exit_threshold: lib.exit_threshold,
        }
    }
}

/// The worker's handle: drain events per tick, shut down on drop of the process.
pub struct PetHandle {
    rx: Receiver<PettingEvent>,
    rx_sound: Receiver<SoundEvent>,
    shutdown: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl PetHandle {
    pub fn spawn(config: PetConfig) -> Result<Self> {
        // Built here, not in the thread, so a missing model or runtime is an error the
        // caller sees instead of a worker that dies quietly on its first breath. Behind a
        // panic catch, because `ort` panics on failures it considers unrecoverable — a
        // missing libonnxruntime must read as "no mic worker", not a dead daemon.
        let detector = std::panic::catch_unwind(|| {
            PettingDetector::new(
                &config.model_path,
                PettingDetectorConfig {
                    stride: PettingDetectorConfig::default().stride,
                    enter_threshold: config.enter_threshold,
                    exit_threshold: config.exit_threshold,
                },
            )
        })
        .unwrap_or_else(|_| Err(anyhow::anyhow!("the ONNX runtime is not loadable")))?;

        let shutdown = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::channel();
        let (tx_sound, rx_sound) = mpsc::channel();
        let sd = shutdown.clone();
        let dev = config.alsa_device.clone();

        let join = thread::Builder::new()
            .name("pet-worker".into())
            .spawn(move || worker_loop(&dev, detector, &tx, &tx_sound, &sd))
            .expect("spawn pet worker");

        Ok(Self {
            rx,
            rx_sound,
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
    mut detector: PettingDetector,
    tx: &Sender<PettingEvent>,
    tx_sound: &Sender<SoundEvent>,
    shutdown: &Arc<AtomicBool>,
) {
    let mut sentry = SoundSentry::new();
    let mut failures: u32 = 0;
    while !shutdown.load(Ordering::Acquire) {
        let started = Instant::now();
        match spawn_arecord(alsa_device) {
            Ok(mut c) => {
                if let Err(e) = pump(&mut c, &mut detector, &mut sentry, tx, tx_sound, shutdown) {
                    log_restart(failures, &e.to_string());
                }
                let _ = c.kill();
                let _ = c.wait();
            }
            Err(e) => log_restart(failures, &e.to_string()),
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
    detector: &mut PettingDetector,
    sentry: &mut SoundSentry,
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
        // Read before the sentry sees this batch, deliberately: the answer is a one-second
        // hangover rather than a verdict on these 128 ms, so it spans batches and reading it
        // one batch early costs nothing. Reading it after would mean pushing to the sentry
        // first, and the sentry's petting mute wants the state from *after* this batch's
        // inference — see below. Only one of the two can go first, and this is the one whose
        // ordering does not matter.
        let audible = sentry.audible();
        let (events, _p) = detector.push_samples(&samples, audible)?;
        for ev in events {
            if tx.send(ev).is_err() {
                // Receiver dropped — the daemon is shutting down.
                return Ok(());
            }
        }
        // Ambient sound events off the same stream, muted while petting (head scratches
        // are LOUD on this mic) + a short hangover. Read off the detector rather than
        // tracked here: a Start..End session spans many seconds, and `arecord` can flap
        // inside one. The detector survives a restart (it is owned by `worker_loop`), so a
        // local copy would come back `false` mid-session — and no second `Start` would ever
        // re-arm it, leaving the sentry emitting head-scratch noise as `Voice` events until
        // the probability finally fell below the exit threshold.
        let mut sounds = Vec::new();
        sentry.push(&samples, detector.is_petting(), &mut sounds);
        for sev in sounds {
            let _ = tx_sound.send(sev);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One sentry frame of a constant amplitude, pushed through as the mic would deliver it.
    fn frames(sentry: &mut SoundSentry, amplitude: f32, count: u32) {
        let block = vec![amplitude; SENTRY_FRAME as usize];
        let mut out = Vec::new();
        for _ in 0..count {
            sentry.push(&block, false, &mut out);
        }
    }

    /// **The point of the gate.** A room with nothing happening in it never arms the
    /// classifier, so the hundred FFTs and the forward pass never run.
    ///
    /// A regression is silent in exactly the way the camera one is: petting still works,
    /// and the only symptom is a robot warmer than it needs to be.
    #[test]
    fn a_quiet_room_never_arms_the_classifier() {
        let mut sentry = SoundSentry::new();
        assert!(!sentry.audible(), "nothing has happened yet");

        // Well under the 0.0012 absolute minimum the off threshold floors at.
        frames(&mut sentry, 0.0001, 200);
        assert!(!sentry.audible(), "silence must not arm it");
    }

    /// A sound arms it, and it stays armed across the gaps in a stroke.
    #[test]
    fn a_sound_arms_the_classifier_and_the_arming_outlives_it() {
        let mut sentry = SoundSentry::new();

        frames(&mut sentry, 0.05, 2);
        assert!(sentry.audible(), "a loud frame must arm it");

        // The gaps between scratches must not shut the gate mid-stroke.
        frames(&mut sentry, 0.0001, AUDIBLE_HOLD_FRAMES - 1);
        assert!(sentry.audible(), "the hangover must span a gap in a stroke");
    }

    /// And it lets go, or the gate would be a one-way switch and the saving would last
    /// until the first door slammed.
    #[test]
    fn the_arming_runs_out_after_the_hold() {
        let mut sentry = SoundSentry::new();

        frames(&mut sentry, 0.05, 1);
        assert!(sentry.audible());

        frames(&mut sentry, 0.0001, AUDIBLE_HOLD_FRAMES);
        assert!(!sentry.audible(), "the hold must expire");
    }

    /// The hold is at least a window wide, because a sound heard now is still inside the
    /// window the classifier looks at a second from now.
    #[test]
    fn the_hold_covers_a_whole_window() {
        let held = f64::from(AUDIBLE_HOLD_FRAMES) * f64::from(SENTRY_FRAME) / 16_000.0;
        let window = crate::WINDOW_SAMPLES as f64 / 16_000.0;
        assert!(
            held >= window,
            "{held:.2}s of hold for a {window:.2}s window"
        );
    }

    /// The gate opens on the sentry's *lower* threshold. Tying it to the upper one would
    /// tune "is a pet worth looking for" against "is a clap worth reporting", which are not
    /// the same question and do not want the same margin.
    #[test]
    fn the_gate_opens_below_the_event_threshold() {
        let mut sentry = SoundSentry::new();
        let floor = sentry.floor;
        // Between the off threshold (3x) and the on threshold (6x), and above both
        // absolute minimums at the starting floor.
        let between = floor * 4.5;
        assert!(between > (floor * 3.0).max(0.0012));
        assert!(between < (floor * 6.0).max(0.002));

        frames(&mut sentry, between, 1);
        assert!(
            sentry.audible(),
            "a sound too small to report is not too small to classify"
        );
    }
}
