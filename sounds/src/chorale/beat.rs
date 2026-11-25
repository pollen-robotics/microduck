//! Keeping several ducks together without a shared clock.
//!
//! Four robots singing a chord have to agree on when the beat is, to about **±20 ms** — the
//! figure comes from [`super`]: an ensemble that agreed to the millisecond would sound like one
//! organ, so a few tens of milliseconds of spread is deliberately *added*, and the sync only has
//! to be tight enough not to swamp it.
//!
//! The obvious way is to sync the clocks and agree a start time. This does not do that, because
//! there is no clock to sync: the boards have no RTC agreement and no NTP, so establishing an
//! offset would mean a connection, a bond, and a central-role Bluetooth client that `btd` does
//! not have.
//!
//! ## The conductor is a beacon
//!
//! Instead, nobody shares a clock — they share a **beat**. The duck conducting puts a beat
//! counter in its BLE advertisement and bumps it once per musical beat. Everyone else passively
//! scans, and *the arrival of a new counter value is the downbeat*. No timestamps to compare, no
//! offset to estimate, no connection, no pairing: passive scanning is the whole radio
//! requirement, and it is the cheapest thing a Bluetooth controller can do.
//!
//! ## Why the radio's jitter does not sink it
//!
//! Air time is microseconds; the error is the **advertising slot**. The conductor hands new
//! payload to its controller and it goes out at the next slot, which `btd` spaces 100–150 ms
//! apart on purpose (one antenna carries BLE, the gamepad and wifi). So a follower hears each
//! beat somewhere in a 50 ms window, and — crucially — *independently per beat*.
//!
//! Two things turn that into ±6 ms:
//!
//!  - The conductor delays its **own** playback by [`SLOT_MEAN_S`], the middle of that window,
//!    so it is wrong in the same direction as everyone else rather than early by default.
//!  - A follower does not chase individual beats. It averages the *phase* over a sliding window
//!    ([`Follower::WINDOW`]), so independent jitter falls as the square root of the window.
//!
//! **The tempo is not estimated, and that was the fix rather than the shortcut.** The first
//! version fitted a straight line through (beat, arrival) and read the tempo off its slope, which
//! is the textbook thing to do and was measurably worse: a line fit evaluated at the *newest*
//! point — the edge of the window, which is exactly where "where are we now" is asked — carries
//! about four times the error it carries at the window's centre, because slope uncertainty
//! compounds with distance from the centroid. Two simulated followers came out 36 ms apart.
//! Holding the period at the score's tempo and averaging only the phase removes the slope term
//! entirely and lands at a third of that. It is also the honest model: both ducks read the tempo
//! off the same score, and their crystals differ by parts per million — 3 ms over a minute-long
//! piece, against the tens of milliseconds the estimator itself was contributing.
//!
//! Everything in this module is a pure function of observed arrival times, so the claim above is
//! a test rather than an argument: [`tests::two_followers_agree_within_the_ensemble_budget`]
//! drives simulated beacons through a simulated 50 ms slot window and checks what comes out.
//!
//! ## What it does when the radio stops
//!
//! Keeps singing. A follower that hears nothing holds its last phase and carries on at the
//! score's tempo — good to a crystal's accuracy, milliseconds per minute — and picks the beat back
//! up when the conductor reappears. That is also exactly what a singer does when they cannot hear
//! the conductor, and it is why the correction is a slow drag on the tempo rather than a jump.

/// Middle of the advertising-slot window `btd` uses (100–150 ms).
///
/// The conductor waits this long after handing a beat to the radio before playing it, so that its
/// own beat and the one everybody hears are the same beat. Getting this wrong shifts the
/// conductor against the whole ensemble, which is the one error that is *not* common-mode.
pub const SLOT_MEAN_S: f64 = 0.125;

/// The duck holding the beat.
///
/// Deliberately dumb: it counts beats and says when to put each one on the air. It does not know
/// or care whether anyone is listening, which is what makes a chorale that loses a follower carry
/// on rather than stall.
#[derive(Debug, Clone)]
pub struct Conductor {
    beat_s: f64,
    /// The last beat actually handed to the radio, or `None` before the first.
    ///
    /// An `Option`, not a counter starting at zero: "beat 0 has gone out" and "nothing has gone
    /// out yet" are different states, and collapsing them made [`Conductor::due`] re-announce
    /// beat 0 on every poll — which, since the caller polls at the advertising rate, is eight
    /// beat-zeroes a second and a follower's fit built on nonsense.
    emitted: Option<u64>,
    /// Local time at which beat 0 was handed to the radio.
    started_at: f64,
}

impl Conductor {
    pub fn new(bpm: f64, now: f64) -> Self {
        Self {
            beat_s: 60.0 / bpm.max(1.0),
            emitted: None,
            started_at: now,
        }
    }

    /// The next beat that should go on the air, if it is due.
    ///
    /// Called as often as the caller likes; it returns `Some` only when the beat has actually
    /// turned over, which is the moment the advertisement payload should change.
    pub fn due(&mut self, now: f64) -> Option<u64> {
        let elapsed = (now - self.started_at).max(0.0);
        let should_be = (elapsed / self.beat_s).floor() as u64;
        match self.emitted {
            Some(last) if should_be <= last => None,
            _ => {
                self.emitted = Some(should_be);
                Some(should_be)
            }
        }
    }

    /// Where in the score the conductor should be singing, in beats.
    ///
    /// Behind its own beat counter by [`SLOT_MEAN_S`] — the point of the whole constant. The
    /// conductor is singing what the room is *hearing*, not what it has most recently told the
    /// radio.
    pub fn position_beats(&self, now: f64) -> f64 {
        (now - self.started_at - SLOT_MEAN_S) / self.beat_s
    }

    /// The beat currently on the air, as the wire carries it — a byte, wrapping.
    ///
    /// The caller assembles the advertisement; this module knows nothing about radios. A byte is
    /// ~4 minutes at a chorale tempo, and [`Follower`] unwraps it.
    pub fn wire_beat(&self) -> u8 {
        (self.emitted.unwrap_or(0) % 256) as u8
    }

    /// Beats handed to the radio so far, unwrapped. `None` before the first.
    pub fn beat(&self) -> Option<u64> {
        self.emitted
    }

    /// The local time beat `beat` goes on the air — exact, not as polled — so that the next piece
    /// can start on this one's grid, where every follower can work out the downbeat in advance.
    pub fn time_of_beat(&self, beat: f64) -> f64 {
        self.started_at + beat * self.beat_s
    }
}

/// One heard beat: which beat it was, and when it arrived here.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Heard {
    beat: f64,
    at: f64,
}

/// A duck locking onto somebody else's beat.
///
/// Holds one number: where the conductor's beat 0 landed, averaged over the last
/// [`Follower::WINDOW`] beats. The tempo comes from the score, so this is the only thing that has
/// to be measured — and measuring one thing rather than two is what keeps it inside the budget.
#[derive(Debug, Clone)]
pub struct Follower {
    /// Seconds per beat, from the score. Held, not estimated — see the module docs.
    beat_s: f64,
    heard: Vec<Heard>,
    /// Unwrapped beat number of the last observation, so a counter rolling over 255 does not read
    /// as jumping backwards by 255 beats.
    last_beat: Option<f64>,
    /// Local time the conductor's beat 0 landed here, averaged over the window.
    phase: Option<f64>,
    /// Outliers in a row. See [`Follower::REACQUIRE`].
    rejected: Vec<Heard>,
}

impl Follower {
    /// How many beats the phase is averaged over.
    ///
    /// The jitter is independent per beat, so the error falls as `1/sqrt(WINDOW)`. A 50 ms slot
    /// window has a standard deviation of ~14 ms on one beat, so 24 beats brings it to ~3 ms —
    /// and two independent followers to ~4 ms, comfortably inside the ±20 ms an ensemble needs.
    /// Longer converges slower for no audible gain; 24 beats is ~25 s at a chorale tempo, and a
    /// follower is already singing after four.
    pub const WINDOW: usize = 24;

    /// A residual this far from the fit is not a beat — a scan that sat in a queue, or a stale
    /// advertisement re-read. Dropped rather than averaged in, because one outlier of half a
    /// second would drag the fit further than sixteen good beats pull it back.
    pub const OUTLIER_S: f64 = 0.20;

    /// This many outliers in a row, **agreeing with each other** to within
    /// [`Follower::REACQUIRE_AGREE_S`], and the lock is what is wrong, not the beats: the window is
    /// replaced by them, and the duck stays locked — on the new phase — throughout.
    ///
    /// Agreeing, because a lossy radio produces runs of *late* sightings too, and those scatter;
    /// dropping a good lock on them would leave a duck unlocked, and silent, for a bar.
    ///
    /// Without it, outlier rejection is a trap. A lock that settled a few hundred milliseconds off —
    /// four bad first observations are enough — rejects every correct beat that would pull it back,
    /// and the duck sings audibly behind the others for the rest of the piece. Seen in the room
    /// simulation as one duck a quarter-beat late, forever, with a perfectly healthy spread.
    pub const REACQUIRE: usize = 4;

    /// How closely the rejected beats have to agree to be believed over the lock, seconds.
    pub const REACQUIRE_AGREE_S: f64 = 0.06;

    pub fn new(bpm: f64) -> Self {
        Self {
            beat_s: 60.0 / bpm.max(1.0),
            heard: Vec::new(),
            last_beat: None,
            phase: None,
            rejected: Vec::new(),
        }
    }

    /// A follower that already knows where beat 0 lands — locked from the start.
    ///
    /// For the next piece of a performance this duck was already locked to: the conductor starts
    /// it on a beat of the old grid that both sides can compute, so the phase is known before the
    /// first beat of the new piece is heard. Seeded as [`Follower::locked`]'s worth of exact
    /// observations, which real ones then outvote — and if the seed was wrong,
    /// [`Follower::REACQUIRE`] throws it away.
    pub fn seeded(bpm: f64, phase: f64) -> Self {
        let mut follower = Self::new(bpm);
        let beat_s = follower.beat_s;
        follower.heard = (0..4)
            .map(|beat| Heard {
                beat: f64::from(beat) - 4.0,
                at: phase + beat_s * (f64::from(beat) - 4.0),
            })
            .collect();
        follower.refit();
        follower.last_beat = None;
        follower
    }

    /// Local time the conductor's beat 0 landed here, once there is an estimate.
    pub fn phase(&self) -> Option<f64> {
        self.phase
    }

    /// Take one heard beat: `beat` is the wrapped byte off the wire, `at` the local time the new
    /// value arrived.
    ///
    /// Call this **only when the counter changes**. A conductor's advertisement repeats seven or
    /// eight times per beat, and re-reading the same value is not another beat — it is the same
    /// beat, later, and feeding those in would drag the phase late by half an advertising
    /// interval.
    pub fn observe(&mut self, beat: u8, at: f64) {
        let unwrapped = self.unwrap(beat);
        // Reject what cannot be a beat before it can pollute the average.
        if let Some(phase) = self.phase {
            let predicted = phase + self.beat_s * unwrapped;
            if (at - predicted).abs() > Self::OUTLIER_S && self.heard.len() >= 4 {
                self.rejected.push(Heard {
                    beat: unwrapped,
                    at,
                });
                if self.rejected.len() > Self::REACQUIRE {
                    self.rejected.remove(0);
                }
                let implied: Vec<f64> = self
                    .rejected
                    .iter()
                    .map(|heard| heard.at - self.beat_s * heard.beat)
                    .collect();
                let low = implied.iter().copied().fold(f64::INFINITY, f64::min);
                let high = implied.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                if self.rejected.len() < Self::REACQUIRE || high - low > Self::REACQUIRE_AGREE_S {
                    return;
                }
                self.heard = std::mem::take(&mut self.rejected);
                self.refit();
                return;
            }
        }
        self.rejected.clear();
        self.heard.push(Heard {
            beat: unwrapped,
            at,
        });
        if self.heard.len() > Self::WINDOW {
            self.heard.remove(0);
        }
        self.refit();
    }

    /// A wrapped byte back into a monotonically rising beat number.
    fn unwrap(&self, beat: u8) -> f64 {
        let Some(last) = self.last_beat else {
            return f64::from(beat);
        };
        let base = (last / 256.0).floor() * 256.0;
        let mut candidate = base + f64::from(beat);
        // The counter only ever goes forward, so a value that reads as behind is a wrap.
        while candidate < last - 128.0 {
            candidate += 256.0;
        }
        while candidate > last + 128.0 {
            candidate -= 256.0;
        }
        candidate
    }

    fn refit(&mut self) {
        if let Some(latest) = self.heard.last() {
            self.last_beat = Some(latest.beat);
        }
        if self.heard.is_empty() {
            return;
        }
        // The phase each observation implies, averaged. Every term is an independent estimate of
        // the same quantity, which is what makes the average the whole trick.
        let n = self.heard.len() as f64;
        self.phase = Some(
            self.heard
                .iter()
                .map(|heard| heard.at - self.beat_s * heard.beat)
                .sum::<f64>()
                / n,
        );
    }

    /// Whether enough beats have been heard to sing to.
    ///
    /// Four, not one: a single beat gives a phase with the whole slot window as its error bar, and
    /// coming in on that would be audibly early or late. Four is about a bar, which is also how
    /// long a singer takes to join something already in progress.
    pub fn locked(&self) -> bool {
        self.heard.len() >= 4
    }

    /// Where in the score to be singing now, in beats. `None` until [`Follower::locked`].
    pub fn position_beats(&self, now: f64) -> Option<f64> {
        if !self.locked() {
            return None;
        }
        Some((now - self.phase?) / self.beat_s)
    }

    /// How far this duck's idea of the beat has moved over the window, in seconds — the spread of
    /// the individual phase estimates.
    ///
    /// Diagnostic, and the one worth having: a healthy lock sits at a few milliseconds, and a
    /// number in the hundreds means this duck is hearing something that is not the conductor's
    /// beat — a second conductor, or a stale advertisement being re-read as new.
    pub fn spread_s(&self) -> Option<f64> {
        let phase = self.phase?;
        if self.heard.len() < 2 {
            return None;
        }
        let n = self.heard.len() as f64;
        let variance = self
            .heard
            .iter()
            .map(|heard| {
                let implied = heard.at - self.beat_s * heard.beat;
                (implied - phase) * (implied - phase)
            })
            .sum::<f64>()
            / n;
        Some(variance.sqrt())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A deterministic stand-in for the advertising slot: a beat handed to the radio goes out
    /// somewhere in `btd`'s 100–150 ms window, independently each time.
    ///
    /// Deterministic because a flaky timing test is worse than none — the sequence is fixed, and
    /// the claim under test is about the *distribution*, which a fixed sequence samples fine.
    struct Slot {
        state: u64,
    }

    impl Slot {
        fn new(seed: u64) -> Self {
            Self { state: seed | 1 }
        }

        /// Uniform in 100–150 ms.
        fn delay(&mut self) -> f64 {
            // xorshift; any decent bit-mixer would do.
            self.state ^= self.state << 13;
            self.state ^= self.state >> 7;
            self.state ^= self.state << 17;
            0.100 + 0.050 * ((self.state >> 11) as f64 / (1u64 << 53) as f64)
        }
    }

    /// The claim the whole design rests on: every duck hearing the same beacon through its own
    /// independent 50 ms slot jitter ends up agreeing where the beat is, to well inside the ±20 ms
    /// an ensemble needs.
    ///
    /// Run for a full quartet — a conductor and three followers — because that is what the piece is
    /// written for, and because the bar gets *harder* with more of them: the thing that has to hold
    /// is the worst disagreement between any two, and three followers have three pairs to go wrong
    /// rather than one. This is the test that says the conductor-beacon idea works, and it runs on a
    /// laptop with no radio in it.
    #[test]
    fn a_whole_quartet_agrees_within_the_ensemble_budget() {
        const BPM: f64 = 58.0;
        let beat_s = 60.0 / BPM;
        let mut conductor = Conductor::new(BPM, 0.0);
        // Three followers, each with its own jitter sequence: they are not hearing the same delays.
        let mut followers: Vec<(Follower, Slot)> =
            [0x2545F4914F6CDD1D, 0x9E3779B97F4A7C15, 0x8EBC6AF09C88C6E3]
                .into_iter()
                .map(|seed| (Follower::new(BPM), Slot::new(seed)))
                .collect();

        let mut worst_pair = 0.0f64;
        let mut worst_against_conductor = 0.0f64;
        // Two minutes of beats — twice the length of the shipped piece.
        let mut now = 0.0;
        while now < 120.0 {
            if let Some(beat) = conductor.due(now) {
                let wire = (beat % 256) as u8;
                for (follower, slot) in followers.iter_mut() {
                    let delay = slot.delay();
                    follower.observe(wire, now + delay);
                }
            }
            // Relative agreement is what an ensemble is; the absolute value is a common-mode offset
            // nobody can hear.
            let positions: Vec<f64> = followers
                .iter()
                .filter_map(|(follower, _)| follower.position_beats(now))
                .collect();
            if positions.len() == followers.len() {
                let low = positions.iter().copied().fold(f64::MAX, f64::min);
                let high = positions.iter().copied().fold(f64::MIN, f64::max);
                worst_pair = worst_pair.max((high - low) * beat_s);
                let at = conductor.position_beats(now);
                for position in &positions {
                    worst_against_conductor =
                        worst_against_conductor.max(((position - at) * beat_s).abs());
                }
            }
            now += 0.010;
        }

        assert!(
            worst_pair < 0.020,
            "the widest pair of followers drifted {:.1} ms apart, budget is 20 ms",
            worst_pair * 1000.0
        );
        // The conductor sings what the room hears, so it has to be inside the budget too — this is
        // what `SLOT_MEAN_S` is for, and it fails if that constant is wrong.
        assert!(
            worst_against_conductor < 0.020,
            "the conductor is {:.1} ms out of its own ensemble",
            worst_against_conductor * 1000.0
        );
        // And every lock is *tight*, not merely agreeing: a wide spread that happens to average to
        // the same place is a duck about to wander off.
        for (index, (follower, _)) in followers.iter().enumerate() {
            let spread = follower.spread_s().expect("locked");
            assert!(
                spread < 0.020,
                "follower {index}'s phase estimates are {:.1} ms apart — that is not a lock",
                spread * 1000.0
            );
        }
    }

    /// Averaging is the mechanism, so it has to be visible: one beat is worth ±25 ms and a full
    /// window is worth single digits.
    #[test]
    fn the_fit_averages_the_jitter_down() {
        const BPM: f64 = 60.0;
        let error_after = |beats: usize| -> f64 {
            let mut conductor = Conductor::new(BPM, 0.0);
            let mut follower = Follower::new(BPM);
            let mut slot = Slot::new(0x1234_5678_9ABC_DEF1);
            let mut now = 0.0;
            let mut seen = 0;
            while seen < beats {
                if let Some(beat) = conductor.due(now) {
                    follower.observe((beat % 256) as u8, now + slot.delay());
                    seen += 1;
                }
                now += 0.010;
            }
            let follower_at = follower.position_beats(now).expect("locked");
            ((follower_at - conductor.position_beats(now)) * 60.0 / BPM).abs()
        };
        let few = error_after(4);
        let many = error_after(Follower::WINDOW * 3);
        assert!(many < 0.012, "a full window should be tight, got {many}");
        assert!(
            many <= few,
            "more beats must not be worse: {few} then {many}"
        );
    }

    /// The counter is a byte, and a piece longer than four minutes rolls it over. A follower that
    /// read that as jumping back 255 beats would stop dead in the middle of the piece.
    #[test]
    fn the_beat_counter_wraps_without_the_music_jumping() {
        const BPM: f64 = 60.0;
        let mut follower = Follower::new(BPM);
        // Start near the wrap and walk through it.
        for beat in 250u32..=260 {
            follower.observe((beat % 256) as u8, f64::from(beat));
        }
        let position = follower.position_beats(260.0).expect("locked");
        assert!(
            (position - 260.0).abs() < 0.1,
            "after the wrap the follower thinks it is at beat {position}, not 260"
        );
        // And the lock stayed tight through it — a mis-unwrap shows up as an enormous spread.
        assert!(follower.spread_s().expect("locked") < 0.01);
    }

    /// A conductor that goes off the air must not stop the music: the fit holds, the tempo is
    /// crystal-accurate, and the beat is picked back up on return.
    #[test]
    fn silence_from_the_conductor_is_not_silence_from_the_duck() {
        const BPM: f64 = 60.0;
        let mut follower = Follower::new(BPM);
        for beat in 0u32..8 {
            follower.observe(beat as u8, f64::from(beat) + 0.125);
        }
        let before = follower.position_beats(8.0).expect("locked");

        // Ten seconds of nothing. Still singing, and still in the right place.
        let during = follower.position_beats(18.0).expect("still locked");
        assert!(
            (during - before - 10.0).abs() < 0.05,
            "{before} then {during}"
        );

        // The conductor comes back, mid-piece, and is picked up without a jump.
        follower.observe(18u8, 18.125);
        let after = follower.position_beats(18.0).expect("locked");
        assert!(
            (after - during).abs() < 0.1,
            "rejoining moved the position by {} beats",
            (after - during).abs()
        );
    }

    /// A stale or queued read is not a beat. One half-second outlier must not drag the fit
    /// further than a window of good beats can pull it back.
    #[test]
    fn an_outlier_is_dropped_rather_than_averaged_in() {
        const BPM: f64 = 60.0;
        let mut follower = Follower::new(BPM);
        for beat in 0u32..8 {
            follower.observe(beat as u8, f64::from(beat) + 0.125);
        }
        let clean = follower.position_beats(8.0).expect("locked");
        // A beat that arrives half a second late — a scan that sat in a queue.
        follower.observe(8, 8.125 + 0.5);
        let after = follower.position_beats(8.0).expect("locked");
        assert!(
            (after - clean).abs() < 0.02,
            "one outlier moved the fit by {} beats",
            (after - clean).abs()
        );
    }

    /// Nobody sings on the first beat they hear. One observation carries the whole slot window as
    /// its error bar, and entering on it would be audibly early or late.
    /// A lock that settled in the wrong place lets go of it, instead of rejecting every correct
    /// beat as an outlier for the rest of the piece.
    #[test]
    fn a_lock_in_the_wrong_place_is_reacquired() {
        const BPM: f64 = 60.0;
        let mut follower = Follower::new(BPM);
        // Locked a third of a beat late.
        for beat in 0..8u8 {
            follower.observe(beat, f64::from(beat) + 0.33);
        }
        assert!((follower.position_beats(10.0).expect("locked") - 9.67).abs() < 1e-6);
        // The true beat, from now on.
        for beat in 8..16u8 {
            follower.observe(beat, f64::from(beat));
        }
        let at = follower.position_beats(20.0).expect("locked again");
        assert!((at - 20.0).abs() < 0.01, "back on the beat: {at}");
    }

    /// A seed that was wrong — the conductor started somewhere else — is thrown away as soon as
    /// the real beats agree with each other.
    #[test]
    fn a_wrong_seed_is_reacquired() {
        const BPM: f64 = 60.0;
        let mut follower = Follower::seeded(BPM, 100.0);
        for beat in 0..8u8 {
            follower.observe(beat, 40.0 + f64::from(beat));
        }
        let at = follower.position_beats(50.0).expect("locked");
        assert!((at - 10.0).abs() < 0.01, "on the real beat: {at}");
    }

    #[test]
    fn a_follower_waits_before_it_sings() {
        let mut follower = Follower::new(60.0);
        assert!(!follower.locked());
        assert_eq!(follower.position_beats(1.0), None);
        for beat in 0u32..3 {
            follower.observe(beat as u8, f64::from(beat));
            assert!(!follower.locked(), "{beat} beats is not a lock");
        }
        follower.observe(3, 3.0);
        assert!(follower.locked());
        assert!(follower.position_beats(3.0).is_some());
    }

    /// The conductor bumps its counter once per beat, and no more — the advertisement repeats
    /// itself seven or eight times per beat and the counter must not move with it.
    #[test]
    fn the_conductor_counts_beats_and_not_polls() {
        let mut conductor = Conductor::new(60.0, 0.0);
        let mut bumps = Vec::new();
        let mut now = 0.0;
        while now < 5.0 {
            if let Some(beat) = conductor.due(now) {
                bumps.push((beat, now));
            }
            // Polled at the advertising rate, far faster than the beat.
            now += 0.010;
        }
        // Beats 0 through 4 — the downbeat counts, and is announced once like every other.
        assert_eq!(bumps.len(), 5, "five beats in five seconds: {bumps:?}");
        for (index, (beat, at)) in bumps.iter().enumerate() {
            assert_eq!(*beat, index as u64);
            assert!((at - index as f64).abs() < 0.02, "{bumps:?}");
        }
    }
}
