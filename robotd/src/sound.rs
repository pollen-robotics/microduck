//! The robot's voice, at play time: pick a wav from the bank, feed it to `aplay`.
//!
//! Ported from the prototype's `play_voice` / `start_wheee`. The codec PCM is exclusive and
//! single-client, which two properties fall out of:
//!
//!  - **One playing child, and a new sound kills it.** That is what lets someone spam the
//!    chirp trigger cleanly — each press cuts the previous call off — and why everything
//!    that plays goes through this one struct, owned by the control loop.
//!  - **The wheee ride streams into a single `aplay`**: start → loop (repeating while held)
//!    → end, written by a paced thread so the pipe never queues more than ~250 ms — else
//!    the release would land that late. The ride has *two* exits, and they are not the
//!    same: a client that says "released" cuts it (kill the child, the writer exits on the
//!    broken pipe), while a hold that merely went stale lands it — the writer is let out of
//!    its loop and writes the end segment into a pipe that is still open. Only the second
//!    one ever plays `wheee_end_*`, which is why [`Ride`] is a state and not a bool.
//!
//!  - **The theremin is synthesized, not played.** Every other sound here is a wav the
//!    bank rendered in advance, which works because their shape is known in advance. A
//!    theremin's pitch is a hand's distance — 15 new values a second, none of them known
//!    until they arrive — so there is no file to pick. Its writer thread pulls blocks from
//!    a live [`sounds::Stream`] instead, reading the parameters the control loop last
//!    stored, and stays much closer behind playback than the ride does: a ride's 250 ms
//!    lead only delays its release, while the same lead on an instrument is the gap between
//!    moving your hand and hearing it.
//!
//! Playing is spawning: nothing here blocks the 50 Hz tick except the deliberately blocking
//! goodbye peck right before power-off, when there is no tick left to miss — and that one is
//! bounded, because a wedged PCM must not be able to hold up the power-off.

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use sounds::{Personality, Stream};

use crate::intents::WheeeHold;

/// How long the blocking goodbye peck may hold up the power-off. The longest bank sound is
/// well under a second; this is a ceiling on a wedged PCM, not a playback budget.
const BLOCKING_PLAY_MAX: Duration = Duration::from_millis(1500);

/// Audio block the theremin writer renders at a time: 10 ms.
///
/// Short, because it bounds how stale the parameters can be by the time they are heard, and
/// the stream is deliberately indifferent to block size (`sounds::stream`) so this is a
/// latency choice and nothing else.
const SYNTH_BLOCK: usize = sounds::SR as usize / 100;

/// How far ahead of playback the theremin writer stays. An instrument's whole quality is
/// this number — see the module docs for why it is not the ride's 250 ms.
const SYNTH_LEAD_S: f64 = 0.03;

/// How long the beak stays open around a one-shot. A chirp is ~200 ms; the servo is
/// slower, so a hold that ended with the wav would barely move. Long enough to read as a
/// quack from across the room, short enough not to leave the beak hanging.
const QUACK_MOUTH_HOLD: Duration = Duration::from_millis(450);

/// How open the beak goes for a one-shot. Fully open: `robotctl quack` is how you tell
/// ducks apart in a group, and a half-open beak is easy to miss.
const QUACK_MOUTH_OPEN: f64 = 1.0;

/// How loud one voice of an ensemble sings.
///
/// Under full scale, and not for headroom: **four ducks in a room sum acoustically.** The offline
/// preview divides its mix by the square root of the voice count for exactly this reason; on real
/// hardware nothing divides anything, so each duck has to arrive already knowing it is one of
/// several. A single duck singing alone is therefore a little quiet, which is the right way round —
/// the alternative was a quartet that saturated, which is what the first run on the robot sounded
/// like.
const CHORALE_LEVEL: f64 = 0.55;

/// Where the duck's own speaker gives up, hertz.
///
/// Measured by ear rather than from a datasheet: the chorale's 130 Hz bass line did not come
/// through the driver, and this is where it starts to. `sounds::Stream::set_speaker_rolloff` uses it
/// to carry a low note on harmonics the driver can make instead of a fundamental it cannot.
const SPEAKER_ROLLOFF_HZ: f64 = 300.0;

/// Keep `SIGPIPE` off a writer thread.
///
/// `robotd` restores its default disposition at startup — so that piping its stdout behaves — which
/// would make a write into a dead `aplay` kill the whole daemon. Blocked here, every such write is
/// merely a failed write, which every writer below already treats as "this voice is over".
fn block_sigpipe() {
    // SAFETY: masking one signal on the calling thread; nothing else observes the mask.
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGPIPE);
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
    }
}

/// What owns the single-client PCM. Not a bool and not two: every one of these is a state that a
/// one-shot, a trigger press and a power-off have to tell apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ride {
    /// No ride. The PCM is free for one-shots.
    Off,
    /// The writer thread is streaming start → loop into an open pipe.
    Riding,
    /// The writer is on the end segment, or `aplay` is draining it. Reaped when the child
    /// exits; a fresh press supersedes it.
    Landing,
    /// The theremin holds the PCM: a writer thread is synthesizing from live parameters,
    /// and will keep doing so until the loop takes the instrument away.
    Theremin,
    /// The chorale holds the PCM: a writer thread is rendering one part of a score at whatever
    /// position the loop last published.
    Singing,
}

/// The theremin's parameters, as the control loop hands them to the writer thread.
///
/// Atomics rather than a channel: the loop must never block on audio, the writer must never
/// block the loop, and a parameter is a *level* — a writer that missed one update wants the
/// newest value, not a queue of every value it slept through.
#[derive(Debug)]
struct Live {
    /// Carrier frequency, hertz, as `f64` bits.
    hz: AtomicU64,
    /// 0 silent, 1 full, as `f64` bits. The note's key.
    level: AtomicU64,
    /// Mouth opening, 0..1, as `f64` bits — the same value the servo is being sent, so
    /// what is heard and what is seen are one gesture.
    open: AtomicU64,
    /// Cleared to tell the writer to fade out and let go of the pipe.
    playing: AtomicBool,
}

impl Live {
    fn new() -> Self {
        Self {
            hz: AtomicU64::new(0.0f64.to_bits()),
            level: AtomicU64::new(0.0f64.to_bits()),
            open: AtomicU64::new(0.0f64.to_bits()),
            playing: AtomicBool::new(true),
        }
    }

    /// Where in the score to sing, in beats, and whether to sing at all.
    ///
    /// The position is published rather than the audio being driven: the control loop knows where
    /// the ensemble is (from the conductor or the phase lock) and the writer thread knows how to
    /// turn a score position into samples. A duck whose audio stalls therefore resumes *in the
    /// right place* instead of a bar behind, because position is a function of time and not a
    /// count of samples played.
    fn set_position(&self, beats: f64, singing: bool) {
        self.hz.store(beats.to_bits(), Ordering::Relaxed);
        self.level.store(
            if singing { 1.0f64 } else { 0.0 }.to_bits(),
            Ordering::Relaxed,
        );
    }

    fn position(&self) -> (f64, bool) {
        (
            f64::from_bits(self.hz.load(Ordering::Relaxed)),
            f64::from_bits(self.level.load(Ordering::Relaxed)) > 0.5,
        )
    }

    fn set(&self, hz: f64, level: f64, open: f64) {
        self.hz.store(hz.to_bits(), Ordering::Relaxed);
        self.level.store(level.to_bits(), Ordering::Relaxed);
        self.open.store(open.to_bits(), Ordering::Relaxed);
    }

    fn read(&self) -> (f64, f64, f64) {
        (
            f64::from_bits(self.hz.load(Ordering::Relaxed)),
            f64::from_bits(self.level.load(Ordering::Relaxed)),
            f64::from_bits(self.open.load(Ordering::Relaxed)),
        )
    }
}

/// The player. Constructed once by the control loop; `None`-like when the bank is missing —
/// every play degrades to a debug line, so a robot without a generated bank (or a codec)
/// walks fine and stays quiet.
pub struct Sound {
    bank: PathBuf,
    device: String,
    child: Option<Child>,
    /// The wheee rider loops while this is true. Shared with the writer thread; clearing
    /// it *without* killing the child is what makes the end segment play.
    wheee_held: Arc<AtomicBool>,
    /// What the ride is doing. Not a bool: a plain sound must not flip it, and "landing"
    /// (the end segment is being written) has to be told apart from both "riding" and
    /// "off", or the trigger restarts a ride that is still finishing.
    ride: Ride,
    /// Bank missing is logged once, not per press.
    warned_missing: bool,
    /// The live parameters of whatever the writer thread is rendering — a theremin's pitch, or a
    /// chorale's score position.
    live: Option<Arc<Live>>,
    /// The piece and part being sung, so a tick that asks for the same one does not restart the
    /// voice mid-phrase.
    singing: Option<(String, sounds::chorale::Part)>,
    /// This robot's voice, read once from the bank's own seed marker so the synthesized
    /// theremin and the rendered wavs are the same duck. `None` until first asked for;
    /// `Some(None)` once it has been asked for and there is no seed to be had.
    personality: Option<Option<Personality>>,
    /// Beak stays open until this instant while a one-shot plays. The wav is shorter
    /// than the servo's travel, so the hold is a duration, not `aplay`'s lifetime.
    oneshot_mouth_until: Option<Instant>,
}

impl Sound {
    pub fn new(bank: PathBuf, device: String) -> Self {
        Self {
            bank,
            device,
            child: None,
            wheee_held: Arc::new(AtomicBool::new(false)),
            ride: Ride::Off,
            warned_missing: false,
            live: None,
            singing: None,
            personality: None,
            oneshot_mouth_until: None,
        }
    }

    /// How open the beak should be for a one-shot that is in flight.
    ///
    /// The pad already opens the mouth via the analog triggers; `robotctl quack` has no
    /// trigger, so this is the visible half of that call. Zero unless a chirp/greet/peck
    /// (anything that went through [`Self::play`]) is still being shown, and never while a
    /// ride/theremin/chorale owns the PCM — those own the mouth themselves.
    pub fn quack_mouth(&mut self) -> f64 {
        if self.ride != Ride::Off {
            self.oneshot_mouth_until = None;
            return 0.0;
        }
        match self.oneshot_mouth_until {
            Some(until) if Instant::now() < until => QUACK_MOUTH_OPEN,
            _ => {
                self.oneshot_mouth_until = None;
                0.0
            }
        }
    }

    /// Random wav from the bank's `tag` directory — the prototype's nanos-based pick, which
    /// is exactly as random as a duck needs.
    fn pick(&mut self, tag: &str) -> Option<PathBuf> {
        let dir = self.bank.join(tag);
        let mut files: Vec<_> = std::fs::read_dir(&dir)
            .ok()?
            .filter_map(|e| Some(e.ok()?.path()))
            .filter(|p| p.extension().is_some_and(|x| x == "wav"))
            .collect();
        if files.is_empty() {
            return None;
        }
        files.sort();
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.subsec_nanos() as usize)
            .unwrap_or(0);
        Some(files.swap_remove(nanos % files.len()))
    }

    /// Stop whatever is on the PCM, now. The ride's abrupt exit, and what every one-shot
    /// does to its predecessor.
    fn stop_child(&mut self) {
        self.wheee_held.store(false, Ordering::Relaxed);
        // A theremin writer blocked in `write_all` on a pipe whose reader is about to die
        // needs telling as well: clearing this is what gets it out of its loop, and the
        // `EPIPE` its next write returns is what gets it out if it was already inside one.
        if let Some(live) = self.live.take() {
            live.playing.store(false, Ordering::Relaxed);
        }
        self.singing = None;
        self.ride = Ride::Off;
        self.oneshot_mouth_until = None;
        if let Some(mut child) = self.child.take()
            && let Ok(None) = child.try_wait()
        {
            let _ = child.kill();
            let _ = child.wait();
        }
    }

    /// The ride's *other* exit: let the writer out of its loop but leave the pipe open, so
    /// it writes the end segment and `aplay` drains it. The child stays parented here and
    /// is reaped by [`Self::reap_landing`] on a later tick.
    fn land_ride(&mut self) {
        self.wheee_held.store(false, Ordering::Relaxed);
        self.ride = Ride::Landing;
        if self.child.is_none() {
            // Nothing to land (the degraded path's one-shot already finished, or never
            // spawned): the PCM is free again straight away.
            self.ride = Ride::Off;
        }
    }

    /// Free the PCM once a landing ride has actually finished. Called per tick while
    /// landing, which is the only cadence available — nothing here waits.
    fn reap_landing(&mut self) {
        match self.child.as_mut().map(|c| c.try_wait()) {
            None | Some(Ok(Some(_))) | Some(Err(_)) => {
                self.child = None;
                self.ride = Ride::Off;
            }
            Some(Ok(None)) => {}
        }
    }

    /// Play a voice-bank sound, cutting off any still-playing one. `blocking` waits for
    /// playback — used right before poweroff so the goodbye peck is heard.
    pub fn play(&mut self, tag: &str, blocking: bool) {
        // A ride owns the single-client PCM for as long as it lasts. Cutting it for a
        // 200 ms chirp would restart the wheee from its start segment on every press of the
        // other trigger — and holding both triggers is the expected way to use them, since
        // either one opens the mouth. The one-shot is dropped, not queued: it is an event,
        // and an event that arrives during a ride is stale by the time the ride ends.
        //
        // The blocking goodbye peck is the exception, and the only one: it is the last
        // sound this process makes, so it takes the PCM off whatever holds it.
        if self.ride != Ride::Off && !blocking {
            tracing::debug!(tag, "sound skipped: the wheee ride has the PCM");
            return;
        }
        self.stop_child();
        let Some(wav) = self.pick(tag) else {
            if !self.warned_missing {
                self.warned_missing = true;
                tracing::warn!(
                    bank = %self.bank.display(),
                    "no voice bank — sounds are skipped (run `sounds ensure-bank`)"
                );
            }
            return;
        };
        // The visual half of the call, even if `aplay` then fails: a quiet speaker is
        // still a duck you can pick out by the beak. Not on the blocking goodbye peck —
        // torque is about to go and there is no tick left to close it.
        if !blocking {
            self.oneshot_mouth_until = Some(Instant::now() + QUACK_MOUTH_HOLD);
        }
        let child = Command::new("aplay")
            .args(["-q", "-D", &self.device])
            .arg(&wav)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        match child {
            Ok(mut c) if blocking => wait_bounded(&mut c, BLOCKING_PLAY_MAX),
            Ok(c) => self.child = Some(c),
            Err(e) => tracing::debug!(error = %e, tag, "aplay failed"),
        }
    }

    /// Take the PCM for a live theremin, if this robot has a voice to play it in.
    ///
    /// Idempotent: called on the rising edge, and safe to call again — an instrument already
    /// in hand is left playing rather than restarted, because restarting it would drop the
    /// note the player is in the middle of.
    pub fn theremin_start(&mut self) -> bool {
        if self.ride == Ride::Theremin && self.live.is_some() {
            return true;
        }
        let Some(personality) = self.voice() else {
            if !self.warned_missing {
                self.warned_missing = true;
                tracing::warn!(
                    bank = %self.bank.display(),
                    "no voice seed — the theremin has no voice to play in \
                     (run `sounds ensure-bank`)"
                );
            }
            return false;
        };
        self.stop_child();

        // A short ALSA buffer for the same reason as the short lead: this is an instrument,
        // and every millisecond queued here is a millisecond between the hand and the note.
        let child = Command::new("aplay")
            .args([
                "-q",
                "-D",
                &self.device,
                "-t",
                "raw",
                "-f",
                "S16_LE",
                "-c",
                "1",
            ])
            .args(["-r", &sounds::SR.to_string()])
            .args(["--buffer-time=40000", "--period-time=10000"])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        let Ok(mut child) = child else {
            tracing::debug!("aplay failed; no theremin");
            return false;
        };
        let Some(mut stdin) = child.stdin.take() else {
            return false;
        };
        self.child = Some(child);
        self.ride = Ride::Theremin;

        let live = Arc::new(Live::new());
        self.live = Some(live.clone());
        // The variant is fixed rather than picked per performance: an instrument that
        // re-rolled its own wobble rate every time it was picked up would be a different
        // instrument every time, which is the one kind of variety a duck does not want.
        let mut stream = Stream::wheee(&personality, 0);

        let spawned = std::thread::Builder::new()
            .name("theremin".into())
            .spawn(move || {
                use std::io::Write;
                block_sigpipe();
                let bytes_per_sec = f64::from(sounds::SR) * 2.0;
                let t0 = Instant::now();
                let mut sent = 0usize;
                let mut block = vec![0.0f32; SYNTH_BLOCK];
                let mut bytes = Vec::with_capacity(SYNTH_BLOCK * 2);

                loop {
                    let playing = live.playing.load(Ordering::Relaxed);
                    let (hz, level, open) = live.read();
                    // Letting go is a fade, not a cut: the level target goes to zero and the
                    // stream is rendered until it has actually reached silence, which is the
                    // difference between an instrument being put down and one being unplugged.
                    stream.set(hz, if playing { level } else { 0.0 }, open);
                    if !playing && stream.is_silent() {
                        return;
                    }

                    stream.block(&mut block);
                    bytes.clear();
                    for sample in &block {
                        let scaled = (sample * 32767.0).clamp(-32768.0, 32767.0) as i16;
                        bytes.extend_from_slice(&scaled.to_le_bytes());
                    }

                    // Stay just behind playback. Without this the pipe would take every
                    // block the moment it is rendered and the instrument would run at the
                    // speed of the CPU rather than of the sound card.
                    let ahead = sent as f64 / bytes_per_sec - t0.elapsed().as_secs_f64();
                    if ahead > SYNTH_LEAD_S {
                        std::thread::sleep(Duration::from_secs_f64(ahead - SYNTH_LEAD_S));
                    }
                    sent += bytes.len();
                    if stdin.write_all(&bytes).is_err() {
                        return; // `aplay` is gone; so is the instrument
                    }
                }
            });
        if spawned.is_err() {
            tracing::warn!("cannot spawn the theremin writer");
            self.stop_child();
            return false;
        }
        tracing::warn!(
            seed = personality.seed,
            "theremin: the PCM is an instrument"
        );
        true
    }

    /// Take the PCM to sing one part of a score, at whatever position the loop publishes.
    ///
    /// Idempotent for the same piece and part: called on every tick while a chorale runs, and a
    /// second call does not restart the voice. A *different* part does restart it, which is what
    /// happens when a duck is reseated between pieces.
    pub fn sing_start(
        &mut self,
        score: &sounds::chorale::Score,
        part: sounds::chorale::Part,
    ) -> bool {
        if self.ride == Ride::Singing
            && self.singing.as_ref() == Some(&(score.name.clone(), part))
            && self.live.is_some()
        {
            return true;
        }
        let Some(personality) = self.voice() else {
            if !self.warned_missing {
                self.warned_missing = true;
                tracing::warn!(
                    bank = %self.bank.display(),
                    "no voice seed — this robot cannot sing (run `sounds ensure-bank`)"
                );
            }
            return false;
        };
        self.stop_child();
        let Some(mut stdin) = self.open_synth_pcm() else {
            return false;
        };
        self.ride = Ride::Singing;
        self.singing = Some((score.name.clone(), part));

        let live = Arc::new(Live::new());
        self.live = Some(live.clone());
        // The part, flattened once: the writer must not walk a score on the audio thread.
        let line: Vec<sounds::chorale::Note> = score.line(part).copied().collect();
        let beat_s = score.beat_s();
        let seat = sounds::chorale::seat_for(&personality, part);
        let detune = 2.0f64.powf(seat.detune_cents / 1200.0);
        let mut voice = Stream::choral(&personality, part as u32);
        // The duck's own speaker, not a full-range one: a bass line's fundamental is not quiet
        // here, it is absent, so the note is carried by harmonics the driver can actually make.
        voice.set_speaker_rolloff(Some(SPEAKER_ROLLOFF_HZ));

        let spawned = std::thread::Builder::new()
            .name("chorale".into())
            .spawn(move || {
                use std::io::Write;
                block_sigpipe();
                let bytes_per_sec = f64::from(sounds::SR) * 2.0;
                let t0 = Instant::now();
                let mut sent = 0usize;
                let mut block = vec![0.0f32; SYNTH_BLOCK];
                let mut bytes = Vec::with_capacity(SYNTH_BLOCK * 2);

                loop {
                    let playing = live.playing.load(Ordering::Relaxed);
                    let (beats, singing) = live.position();
                    // Where the playhead is for *this* duck: the ensemble's position, offset by the
                    // few milliseconds of onset spread that make a choir a choir.
                    let at = beats * beat_s - seat.onset_offset_s;
                    let note = line.iter().find(|note| {
                        at >= note.start_beat * beat_s && at < note.end_beat() * beat_s
                    });
                    match note.filter(|_| playing && singing) {
                        Some(note) => {
                            // Release a little early so a change of chord re-articulates rather
                            // than sliding, exactly as the offline render does.
                            let sustain = note.beats * beat_s * 0.92;
                            let held = at - note.start_beat * beat_s < sustain;
                            voice.set_formant_shift(note.vowel.formant_shift());
                            voice.set(
                                sounds::chorale::midi_hz(f64::from(note.midi)) * detune,
                                if held {
                                    note.level * note.vowel.level_scale() * CHORALE_LEVEL
                                } else {
                                    0.0
                                },
                                note.vowel.open(),
                            );
                        }
                        // Between notes, or asked to stop: silence at the pitch it was on, which is
                        // a release rather than a slide out of the chord.
                        None => voice.set_level(0.0),
                    }
                    if !playing && voice.is_silent() {
                        return;
                    }

                    voice.block(&mut block);
                    bytes.clear();
                    for sample in &block {
                        let scaled = (sample * 32767.0).clamp(-32768.0, 32767.0) as i16;
                        bytes.extend_from_slice(&scaled.to_le_bytes());
                    }
                    let ahead = sent as f64 / bytes_per_sec - t0.elapsed().as_secs_f64();
                    if ahead > SYNTH_LEAD_S {
                        std::thread::sleep(Duration::from_secs_f64(ahead - SYNTH_LEAD_S));
                    }
                    sent += bytes.len();
                    if stdin.write_all(&bytes).is_err() {
                        return; // `aplay` is gone
                    }
                }
            });
        if spawned.is_err() {
            tracing::warn!("cannot spawn the chorale writer");
            self.stop_child();
            return false;
        }
        tracing::warn!(part = part.as_str(), piece = %score.name, "chorale: singing");
        true
    }

    /// Where in the score to sing. Cheap enough for every tick.
    pub fn sing_at(&self, beats: f64, singing: bool) {
        if let Some(live) = self.live.as_ref() {
            live.set_position(beats, singing);
        }
    }

    /// Stop singing: fade out and let the PCM go.
    pub fn sing_stop(&mut self) {
        if self.ride != Ride::Singing {
            return;
        }
        if let Some(live) = self.live.take() {
            live.playing.store(false, Ordering::Relaxed);
        }
        self.singing = None;
        self.ride = Ride::Landing;
    }

    /// The note a 0..1 closeness plays in this robot's voice, hertz.
    ///
    /// The mapping lives with the voice (`sounds::stream::hz_at`) rather than in the
    /// theremin, because it is a fact about *this duck's register*: a low duck plays low,
    /// and the playable span is the one its own joy-ride recipe uses.
    pub fn theremin_hz_at(&mut self, closeness: f64) -> Option<f64> {
        let personality = self.voice()?;
        Some(sounds::stream::hz_at(&personality, closeness))
    }

    /// Open the PCM for a synthesized voice, and keep the child.
    ///
    /// A short ALSA buffer, for the same reason as the short lead: everything queued here is time
    /// between a gesture and its sound, whether the gesture is a hand in front of the beak or a
    /// beat heard from another duck.
    fn open_synth_pcm(&mut self) -> Option<std::process::ChildStdin> {
        let child = Command::new("aplay")
            .args([
                "-q",
                "-D",
                &self.device,
                "-t",
                "raw",
                "-f",
                "S16_LE",
                "-c",
                "1",
            ])
            .args(["-r", &sounds::SR.to_string()])
            .args(["--buffer-time=40000", "--period-time=10000"])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        let Ok(mut child) = child else {
            tracing::debug!("aplay failed; no synthesized voice");
            return None;
        };
        let stdin = child.stdin.take()?;
        self.child = Some(child);
        Some(stdin)
    }

    /// Hand the writer the parameters it should be rendering. Cheap enough for every tick.
    ///
    /// Silently ignored when no instrument is in hand, so the loop can call it
    /// unconditionally rather than mirroring this module's state.
    pub fn theremin_set(&self, hz: f64, level: f64, open: f64) {
        if let Some(live) = self.live.as_ref() {
            live.set(hz, level, open);
        }
    }

    /// Put the instrument down: fade the note out, then let the PCM go.
    ///
    /// The writer owns the fade, so this returns at once and the child is reaped on a later
    /// tick by [`Self::theremin_settle`] — the loop has no time to wait for audio.
    ///
    pub fn theremin_stop(&mut self) {
        if self.ride != Ride::Theremin {
            return;
        }
        if let Some(live) = self.live.take() {
            live.playing.store(false, Ordering::Relaxed);
        }
        self.ride = Ride::Landing;
    }

    /// Reap a faded-out instrument, freeing the PCM for one-shots again.
    ///
    /// Called once per tick, which is the only cadence available: the fade happens in the
    /// writer thread and the loop cannot wait for it. Until this reaps, the PCM is still
    /// held — which is correct, and is why a quack right after letting go is dropped rather
    /// than talking over the tail of the note.
    pub fn theremin_settle(&mut self) {
        if self.ride == Ride::Landing && self.live.is_none() {
            self.reap_landing();
        }
    }

    /// This robot's voice, for anyone that needs its register or its seed — the chorale casts from
    /// the first and identifies itself with a byte of the second.
    pub fn personality(&mut self) -> Option<Personality> {
        self.voice()
    }

    /// This robot's voice, from the seed its own bank was rendered with.
    ///
    /// The bank's marker rather than the hardware id, and deliberately: the marker is the
    /// seed the wavs on this disk were made from, so a robot whose bank predates a re-seed
    /// still has a theremin that matches the quacks it is sitting next to. Falling back to
    /// the hardware id covers a robot that has a codec but no bank yet.
    fn voice(&mut self) -> Option<Personality> {
        if let Some(cached) = self.personality {
            return cached;
        }
        let seed = std::fs::read_to_string(self.bank.join(".seed"))
            .ok()
            .and_then(|marker| marker.trim().split(':').next()?.parse::<u32>().ok())
            .or_else(|| sounds::hardware_seed().ok());
        let personality = seed.map(Personality::from_seed);
        self.personality = Some(personality);
        personality
    }

    /// The ride, level-driven: the loop calls this every tick with what the wheee hold
    /// currently says. The rising edge starts it; the two ways of stopping are different
    /// sounds, which is the whole point of [`WheeeHold`] carrying three states.
    pub fn wheee(&mut self, hold: WheeeHold) {
        // The theremin is played *with* the trigger held in practice (the same hand is on
        // the pad), and a ride restarting under a theremin would be an instrument that
        // stops whenever the player touches anything. The instrument keeps the PCM until
        // the loop takes it away; the trigger is simply inaudible while it does.
        if matches!(self.ride, Ride::Theremin | Ride::Singing) {
            return;
        }
        match hold {
            // A press during a landing ride supersedes it — `start_wheee` takes the PCM.
            WheeeHold::Held if self.ride != Ride::Riding => self.start_wheee(),
            WheeeHold::Held => {}
            // The client said "released": cut it, as the prototype does.
            WheeeHold::Released if self.ride != Ride::Off => self.stop_child(),
            // The hold went stale — the client stopped re-notifying, or stopped existing.
            // Land the ride rather than chopping it: nobody asked for a cut.
            WheeeHold::Decayed if self.ride == Ride::Riding => self.land_ride(),
            WheeeHold::Decayed if self.ride == Ride::Landing => self.reap_landing(),
            WheeeHold::Released | WheeeHold::Decayed => {}
        }
    }

    /// A bank with no wheee triads — a half-rendered `ensure-bank`, or a bank from before
    /// the wheee was segmented. Fall back to the plain one-shot, and *latch the ride
    /// anyway*: the trigger asks every 20 ms, and without a latch this path would fork,
    /// exec and kill `aplay` fifty times a second (plus a `read_dir` and three ~110 KB
    /// reads per tick) for as long as the trigger is down, with nothing audible to show
    /// for it. Landing it is a no-op — there is no end segment to write.
    fn degraded_wheee(&mut self) {
        self.play("wheee", false);
        self.ride = Ride::Riding;
    }

    /// Stream start → loop (while held) → end into one `aplay`, so the loop wraps gap-free.
    fn start_wheee(&mut self) {
        self.stop_child();
        let dir = self.bank.join("wheee");
        let mut letters: Vec<String> = std::fs::read_dir(&dir)
            .map(|rd| {
                rd.filter_map(|e| {
                    let name = e.ok()?.file_name().into_string().ok()?;
                    Some(
                        name.strip_prefix("wheee_start_")?
                            .strip_suffix(".wav")?
                            .to_owned(),
                    )
                })
                .collect()
            })
            .unwrap_or_default();
        if letters.is_empty() {
            // A bank without triads (or no bank): the one-shot path says what's wrong.
            self.degraded_wheee();
            return;
        }
        letters.sort();
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.subsec_nanos() as usize)
            .unwrap_or(0);
        let letter = letters.swap_remove(nanos % letters.len());
        let seg = |name: &str| read_wav_pcm(&dir.join(format!("wheee_{name}_{letter}.wav")));
        let (Some((rate, start_pcm)), Some((_, loop_pcm)), Some((_, end_pcm))) =
            (seg("start"), seg("loop"), seg("end"))
        else {
            self.degraded_wheee();
            return;
        };
        let child = Command::new("aplay")
            .args([
                "-q",
                "-D",
                &self.device,
                "-t",
                "raw",
                "-f",
                "S16_LE",
                "-c",
                "1",
            ])
            .args(["-r", &rate.to_string(), "--buffer-time=120000"])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();
        let Ok(mut child) = child else {
            tracing::debug!("aplay failed; no wheee");
            return;
        };
        let Some(mut stdin) = child.stdin.take() else {
            return;
        };
        self.child = Some(child);
        self.ride = Ride::Riding;
        self.wheee_held.store(true, Ordering::Relaxed);
        let held = self.wheee_held.clone();

        std::thread::Builder::new()
            .name("wheee".into())
            .spawn(move || {
                use std::io::Write;
                // robotd restores SIGPIPE's default disposition at startup (so piping its
                // stdout behaves), which would make a write into a dead aplay kill the
                // whole daemon. Block it on this thread — the writes then fail with EPIPE,
                // which every `send` below already treats as "the ride is over".
                unsafe {
                    let mut set: libc::sigset_t = std::mem::zeroed();
                    libc::sigemptyset(&mut set);
                    libc::sigaddset(&mut set, libc::SIGPIPE);
                    libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
                }
                // Pace writes to stay only ~250 ms ahead of playback — otherwise the pipe
                // queues ~0.7 s of loop audio and the end starts that late after release.
                const CHUNK: usize = 8192;
                let bytes_per_sec = f64::from(rate) * 2.0;
                let t0 = Instant::now();
                let mut sent = 0usize;
                let mut send = |stdin: &mut std::process::ChildStdin, chunk: &[u8]| -> bool {
                    let ahead = sent as f64 / bytes_per_sec - t0.elapsed().as_secs_f64();
                    if ahead > 0.25 {
                        std::thread::sleep(Duration::from_secs_f64(ahead - 0.25));
                    }
                    sent += chunk.len();
                    stdin.write_all(chunk).is_ok()
                };
                for chunk in start_pcm.chunks(CHUNK) {
                    if !send(&mut stdin, chunk) {
                        return;
                    }
                }
                'ride: while held.load(Ordering::Relaxed) {
                    for chunk in loop_pcm.chunks(CHUNK) {
                        if !send(&mut stdin, chunk) {
                            break 'ride;
                        }
                    }
                }
                // Out of the loop with the pipe still open: the hold decayed and
                // `land_ride` cleared the flag without killing us. Play the ride out.
                for chunk in end_pcm.chunks(CHUNK) {
                    if !send(&mut stdin, chunk) {
                        return;
                    }
                }
            })
            .ok();
    }
}

/// Wait for a child, but not forever. The goodbye peck runs with `poweroff()` on the next
/// line: if the PCM wedges — it is single-client, and the ride this just killed does not
/// release the device synchronously — an unbounded `wait()` leaves the robot powered on with
/// its intents already disabled, which is worse than a clipped goodbye.
fn wait_bounded(child: &mut Child, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return,
            Err(_) => return,
            Ok(None) if Instant::now() >= deadline => break,
            Ok(None) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    tracing::warn!(
        ?timeout,
        "aplay did not finish in time; going on without it"
    );
    let _ = child.kill();
    let _ = child.wait();
}

/// Read a PCM wav: (sample_rate, raw S16LE payload). A minimal RIFF chunk walk — enough for
/// the mono 16-bit files the voice bank contains.
fn read_wav_pcm(path: &Path) -> Option<(u32, Vec<u8>)> {
    let b = std::fs::read(path).ok()?;
    if b.len() < 12 || &b[..4] != b"RIFF" || &b[8..12] != b"WAVE" {
        return None;
    }
    let mut rate = 0u32;
    let mut pos = 12usize;
    while pos + 8 <= b.len() {
        let sz = u32::from_le_bytes(b[pos + 4..pos + 8].try_into().ok()?) as usize;
        let body = pos + 8;
        match &b[pos..pos + 4] {
            b"fmt " if body + 8 <= b.len() => {
                rate = u32::from_le_bytes(b[body + 4..body + 8].try_into().ok()?);
            }
            b"data" if rate > 0 => {
                return Some((rate, b[body..(body + sz).min(b.len())].to_vec()));
            }
            _ => {}
        }
        pos = body + sz + (sz & 1);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The RIFF walk must read back what the `sounds` crate writes — the two halves of the
    /// wheee pipeline meet at this file format.
    #[test]
    fn read_wav_pcm_reads_what_sounds_writes() {
        let dir = std::env::temp_dir().join(format!("sound-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.wav");
        let buf: Vec<f32> = (0..480).map(|i| (i as f32 / 480.0).sin()).collect();
        sounds::to_wav(&buf, &path).unwrap();

        let (rate, pcm) = read_wav_pcm(&path).expect("wav must parse");
        assert_eq!(rate, sounds::SR);
        assert_eq!(pcm.len(), 480 * 2, "S16LE mono payload");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A missing bank must degrade to silence, not errors — robots without a codec walk on.
    #[test]
    fn a_missing_bank_is_silent_not_fatal() {
        let mut sound = Sound::new(PathBuf::from("/nonexistent/bank"), "default".into());
        sound.play("chirp", false);
        sound.wheee(WheeeHold::Held);
        sound.wheee(WheeeHold::Released);
        assert!(sound.child.is_none());
        assert_eq!(sound.ride, Ride::Off);
        assert_eq!(
            sound.quack_mouth(),
            0.0,
            "a missing bank must not flap the beak"
        );
    }

    /// A one-shot opens the beak even if `aplay` cannot run: `robotctl quack` is how you
    /// tell ducks apart, and a quiet speaker is still a duck you can pick out visually.
    #[test]
    fn a_one_shot_opens_the_beak() {
        let dir = std::env::temp_dir().join(format!("sound-quack-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("chirp")).unwrap();
        std::fs::write(dir.join("chirp/a.wav"), b"RIFF").unwrap();
        let mut sound = Sound::new(dir.clone(), "null".into());
        sound.play("chirp", false);
        assert_eq!(sound.quack_mouth(), 1.0);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A wheee ride owns the mouth via the analog trigger; a one-shot skipped because the
    /// ride has the PCM must not steal it.
    #[test]
    fn a_ride_does_not_open_the_beak_for_a_skipped_one_shot() {
        let dir = std::env::temp_dir().join(format!("sound-ride-mouth-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("wheee")).unwrap();
        std::fs::create_dir_all(dir.join("chirp")).unwrap();
        std::fs::write(dir.join("chirp/a.wav"), b"RIFF").unwrap();
        let mut sound = Sound::new(dir.clone(), "null".into());
        sound.wheee(WheeeHold::Held);
        assert_eq!(sound.ride, Ride::Riding);
        sound.play("chirp", false);
        assert_eq!(sound.quack_mouth(), 0.0);
        sound.wheee(WheeeHold::Released);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A bank whose `wheee/` holds no triads (a half-rendered `ensure-bank`, or a bank from
    /// before the wheee was segmented) must latch the ride on the fallback. Without the
    /// latch the trigger re-enters `start_wheee` every 20 ms for as long as it is held —
    /// a `read_dir` and an `aplay` spawn/kill pair, fifty times a second, silently.
    #[test]
    fn a_bank_without_wheee_triads_latches_instead_of_respawning() {
        let dir = std::env::temp_dir().join(format!("sound-triads-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("wheee")).unwrap();
        let mut sound = Sound::new(dir.clone(), "null".into());

        sound.wheee(WheeeHold::Held);
        assert_eq!(sound.ride, Ride::Riding, "the fallback must latch");
        sound.wheee(WheeeHold::Held);
        assert_eq!(sound.ride, Ride::Riding, "a held trigger must not re-enter");
        sound.wheee(WheeeHold::Released);
        assert_eq!(sound.ride, Ride::Off);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The two exits are different sounds, and only one of them plays `wheee_end_*`: a
    /// client that says "released" gets the cut, a hold that goes stale gets the landing.
    /// Both must end at `Off` — a ride that cannot leave `Landing` blocks every one-shot.
    #[test]
    fn a_decayed_hold_lands_and_a_release_cuts() {
        let dir = std::env::temp_dir().join(format!("sound-exits-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("wheee")).unwrap();
        let mut sound = Sound::new(dir.clone(), "null".into());

        // Decay: land, then reap. No child ever spawned here, so the reap is immediate —
        // on a real ride it takes as many ticks as the end segment lasts.
        sound.wheee(WheeeHold::Held);
        sound.wheee(WheeeHold::Decayed);
        assert_eq!(
            sound.ride,
            Ride::Off,
            "a landing ride with no child frees the PCM"
        );

        // Release: cut, straight to Off, and the writer's loop flag is down either way.
        sound.wheee(WheeeHold::Held);
        sound.wheee(WheeeHold::Released);
        assert_eq!(sound.ride, Ride::Off);
        assert!(!sound.wheee_held.load(Ordering::Relaxed));
        std::fs::remove_dir_all(&dir).ok();
    }
}
