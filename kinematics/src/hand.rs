//! The nearest thing in front of the beak, as an instrument wants it.
//!
//! This started out clever. It carried a background captured when the theremin armed, so
//! that a duck facing a wall could still tell a hand from the wall; a plane fit to exempt
//! walls from being mistaken for one enormous hand; a slow drift so the room could be
//! rearranged. All of it worked in tests and none of it survived a duck, for one reason
//! worth writing down: **the sensor's input is not stable enough to reason that hard
//! about.** Which zones carry a usable status varies frame to frame, so a background
//! captured over half a second was a different background every time, and the *same*
//! gesture would arm one moment and be refused the next. Cleverness on top of a noisy input
//! multiplies the noise; it does not filter it.
//!
//! So this is the simple thing, and the theremin is an explicit mode instead. There is no
//! background and no arming: while the instrument is up, the nearest return inside the
//! playable band is the hand. Point the duck at open space and it is silent; point it at a
//! wall 40 cm away and it plays a steady note, which is correct — you turned the mode on,
//! and a mode turned on deliberately is allowed to do the obvious thing.
//!
//! What is left is the two pieces of robustness that are about the *sensor* rather than
//! about the room:
//!
//!   - **Which status bytes count.** ST calls 5 and 9 "valid", and on this sensor at 15 Hz a
//!     hand past ~30 cm routinely comes back as 4 or 13 — *consistency failed*, sigma too
//!     high — carrying a distance perfectly good enough for a pitch. Accepting only 5 and 9
//!     is why the first version died at 30 cm. The set is [`Config::statuses`], and it is
//!     config rather than a constant because it is the one number a bench session needs to
//!     move.
//!   - **A hold.** A zone that flickers between usable and not, at 15 Hz, chops a note into
//!     gravel. [`Config::hold`] keeps the last hand for a few frames, so a dropout is
//!     inaudible while a hand actually leaving still stops the note promptly.
//!
//! There is no floor filter and no reprojection here, on purpose. Both need the head's
//! forward kinematics and the IMU, and both were another way for a note to disappear for a
//! reason the player cannot see. An instrument that plays the raw beam is one you can
//! predict.

use std::time::{Duration, Instant};

use crate::tof::{COLS, ROWS};

const N_ZONES: usize = ROWS * COLS;

/// What counts as a hand.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// Nearest playable range, metres. Below ~10 cm the sensor's own cover-glass crosstalk
    /// invents short returns, so this is a floor on believability rather than on taste.
    pub near_m: f64,
    /// Farthest playable range, metres.
    pub far_m: f64,
    /// How many zones in the band make a hand. Low: a hand at the far end of the band is a
    /// handful of zones, and this sensor will only be giving usable status for some of them.
    pub min_zones: usize,
    /// ST status bytes whose distance is believed. See the module docs — accepting only the
    /// two ST calls "valid" is what made the first version stop working at 30 cm.
    pub statuses: Vec<u8>,
    /// How long the last hand is held through a dropout. Long enough to bridge the sensor's
    /// flicker, short enough that a hand actually withdrawn stops the note.
    pub hold: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            near_m: 0.10,
            far_m: 0.70,
            min_zones: 2,
            // 5 and 9 are ST's "valid" and "valid, large pulse". 6 is a first range whose
            // wrap-around check was not performed, 10 a valid range where the previous one
            // saw nothing, 12 a target blurred by a sharp edge — which is what the edge of a
            // hand *is*. 4 and 13 are the consistency failures a moving hand produces past
            // 30 cm, and they are in because a pitch does not need a millimetre.
            statuses: vec![4, 5, 6, 9, 10, 12, 13],
            hold: Duration::from_millis(250),
        }
    }
}

impl Config {
    /// Whether this status byte's distance is worth believing.
    pub fn believes(&self, status: u8) -> bool {
        self.statuses.contains(&status)
    }
}

/// A hand, as the theremin needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hand {
    /// Robust distance to it, metres — a low percentile of what is in the band rather than
    /// the single nearest zone, which on this sensor is regularly a flier.
    pub range_m: f64,
    /// Where that sits in the playable band: 0 at [`Config::far_m`], 1 at
    /// [`Config::near_m`]. Closer is *higher*, which is the direction the pitch and the
    /// mouth both move.
    pub closeness: f64,
    /// How many zones it covers. Reported because it is the number that says whether a
    /// dropout was the hand leaving or the sensor blinking.
    pub zones: usize,
    /// True when this hand is the held memory of an earlier frame rather than one measured
    /// now — so a readout can show a bridged dropout instead of pretending it saw something.
    pub held: bool,
}

/// Finds the hand, and remembers it briefly.
///
/// Stateful only for the hold: everything else about a frame's verdict depends on that frame
/// alone, which is what makes the instrument predictable.
#[derive(Debug)]
pub struct Tracker {
    config: Config,
    last: Option<(Hand, Instant)>,
}

impl Tracker {
    pub fn new(config: Config) -> Self {
        Self { config, last: None }
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    /// The hand in this frame, or the one from a frame just gone.
    ///
    /// `distance_mm` and `status` are the wire's own arrays — raw, parallel, row-major, and
    /// possibly shorter than the grid if the peer is from another release. Taken raw rather
    /// than pre-interpreted on purpose: which statuses count is this module's decision, and
    /// it is the decision that matters most.
    pub fn track(&mut self, distance_mm: &[i16], status: &[u8], now: Instant) -> Option<Hand> {
        let mut in_band: Vec<f64> = Vec::new();
        for zone in 0..N_ZONES {
            let (Some(&mm), Some(&code)) = (distance_mm.get(zone), status.get(zone)) else {
                continue;
            };
            // A negative distance comes back on a failed convergence whatever the status
            // says, and is not a measurement.
            if mm <= 0 || !self.config.believes(code) {
                continue;
            }
            let range = f64::from(mm) / 1000.0;
            if (self.config.near_m..=self.config.far_m).contains(&range) {
                in_band.push(range);
            }
        }

        // The band must hold at least one zone whatever `min_zones` says: at 0 the length
        // check passes on an empty band, and the percentile index below would panic on it.
        if in_band.len() < self.config.min_zones.max(1) {
            // Hold the last hand briefly. This is the whole anti-chop mechanism: the note
            // rides over a dropout, and stops when one lasts.
            return match self.last {
                Some((hand, at)) if now.duration_since(at) < self.config.hold => {
                    Some(Hand { held: true, ..hand })
                }
                _ => {
                    self.last = None;
                    None
                }
            };
        }

        in_band.sort_by(|a, b| a.partial_cmp(b).expect("ranges are finite"));
        // A low percentile, not the minimum: single-zone fliers a few centimetres short of
        // the truth are routine here, and as the pitch input one would be a chirp.
        let range_m = in_band[in_band.len() / 5];
        let span = (self.config.far_m - self.config.near_m).max(1e-6);
        let hand = Hand {
            range_m,
            closeness: ((self.config.far_m - range_m) / span).clamp(0.0, 1.0),
            zones: in_band.len(),
            held: false,
        };
        self.last = Some((hand, now));
        Some(hand)
    }

    /// Forget the held hand — for putting the instrument down, so picking it back up does
    /// not open with a note from before.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// How many zones came back with each status byte, most common first.
///
/// Purely diagnostic, and the diagnostic that matters: "it stops working past 30 cm" and
/// "status 4 covers 31 zones of the frame" are the same sentence, but only the second tells
/// you what to change. Rendered into the live readout so a bench session never has to guess
/// what the sensor is actually saying.
pub fn status_histogram(status: &[u8]) -> Vec<(u8, usize)> {
    let mut counts: Vec<(u8, usize)> = Vec::new();
    for &code in status.iter().take(N_ZONES) {
        match counts.iter_mut().find(|(c, _)| *c == code) {
            Some((_, n)) => *n += 1,
            None => counts.push((code, 1)),
        }
    }
    counts.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    counts
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame with `zones` zones at `distance_m`, all carrying `status_code`.
    fn frame(distance_m: f64, status_code: u8, zones: usize) -> (Vec<i16>, Vec<u8>) {
        let mm = (distance_m * 1000.0) as i16;
        let mut distance = vec![0i16; N_ZONES];
        let mut status = vec![255u8; N_ZONES];
        for zone in 0..zones.min(N_ZONES) {
            distance[zone] = mm;
            status[zone] = status_code;
        }
        (distance, status)
    }

    /// The bug that killed the first version, pinned: a hand past 30 cm arrives with a
    /// consistency-failure status, and it must still play. If someone narrows
    /// `Config::statuses` back to ST's two "valid" codes, this is the test that says why not.
    #[test]
    fn the_statuses_a_real_hand_arrives_with_are_believed() {
        let mut tracker = Tracker::new(Config::default());
        for code in Config::default().statuses {
            let (distance, status) = frame(0.40, code, 8);
            let hand = tracker
                .track(&distance, &status, Instant::now())
                .unwrap_or_else(|| panic!("status {code} must play"));
            assert!(
                (hand.range_m - 0.40).abs() < 0.01,
                "status {code}: {hand:?}"
            );
            tracker.reset();
        }
        // 255 is the sensor saying it looked and found nothing. That is silence, not a note.
        let (distance, status) = frame(0.40, 255, 8);
        assert_eq!(tracker.track(&distance, &status, Instant::now()), None);
        // And a status nobody believes stays unbelieved.
        let (distance, status) = frame(0.40, 1, 8);
        assert_eq!(tracker.track(&distance, &status, Instant::now()), None);
    }

    /// Closer is higher — the direction the pitch and the mouth both move — and the ends of
    /// the band are 0 and 1 exactly.
    #[test]
    fn closeness_rises_as_the_hand_approaches() {
        let config = Config::default();
        let mut tracker = Tracker::new(config.clone());
        let at = |tracker: &mut Tracker, distance: f64| {
            let (d, s) = frame(distance, 5, 8);
            tracker.track(&d, &s, Instant::now()).map(|h| h.closeness)
        };
        assert_eq!(at(&mut tracker, config.near_m), Some(1.0));
        assert_eq!(at(&mut tracker, config.far_m), Some(0.0));
        let middle = at(&mut tracker, 0.5 * (config.near_m + config.far_m)).expect("plays");
        assert!((0.0..1.0).contains(&middle));

        // Outside the band there is no note, rather than a clamped one.
        tracker.reset();
        let (d, s) = frame(config.far_m + 0.2, 5, 8);
        assert_eq!(tracker.track(&d, &s, Instant::now()), None);
    }

    /// The anti-chop mechanism, which is the whole reason this is stateful: a frame the
    /// sensor fumbles must not cut the note, and a hand actually withdrawn must stop it.
    #[test]
    fn a_dropped_frame_holds_the_note_and_a_withdrawn_hand_ends_it() {
        let config = Config::default();
        let mut tracker = Tracker::new(config.clone());
        let start = Instant::now();
        let (good, good_status) = frame(0.30, 5, 8);
        let (empty, empty_status) = frame(0.30, 255, 8);

        let hand = tracker.track(&good, &good_status, start).expect("plays");
        assert!(!hand.held);

        // Dropouts inside the hold keep the note, and say they are held.
        for frames in 1..=3 {
            let at = start + Duration::from_millis(66 * frames);
            let held = tracker
                .track(&empty, &empty_status, at)
                .unwrap_or_else(|| panic!("frame {frames} must ride the dropout"));
            assert!(held.held);
            assert_eq!(held.range_m, hand.range_m);
        }
        // Past the hold, silence.
        let after = start + config.hold + Duration::from_millis(1);
        assert_eq!(tracker.track(&empty, &empty_status, after), None);
        // And the hold does not resurrect: still silent a frame later.
        assert_eq!(
            tracker.track(&empty, &empty_status, after + Duration::from_millis(66)),
            None
        );
    }

    /// The hold is measured from the last *seen* hand, not from the first dropout — a
    /// gesture held for a minute must not expire mid-note.
    #[test]
    fn a_hand_that_keeps_being_seen_never_expires() {
        let mut tracker = Tracker::new(Config::default());
        let start = Instant::now();
        let (good, status) = frame(0.30, 5, 8);
        for frame_index in 0..(15 * 60) {
            let at = start + Duration::from_millis(66 * frame_index);
            let hand = tracker
                .track(&good, &status, at)
                .expect("a held hand plays");
            assert!(!hand.held, "a seen hand is not a held one");
        }
    }

    /// Too few zones is not a hand: one stray zone at 20 cm is the sensor, not a gesture.
    #[test]
    fn one_stray_zone_is_not_a_hand() {
        let config = Config::default();
        let mut tracker = Tracker::new(config.clone());
        let (distance, status) = frame(0.25, 5, 1);
        assert_eq!(tracker.track(&distance, &status, Instant::now()), None);
        let (distance, status) = frame(0.25, 5, config.min_zones);
        assert!(tracker.track(&distance, &status, Instant::now()).is_some());
    }

    /// `min_zones = 0` is a legal config, and an empty band then passes the length check —
    /// straight into a percentile index that has nothing to index. No zones is no hand,
    /// whatever the floor is.
    #[test]
    fn an_empty_band_is_not_a_hand_even_with_min_zones_zero() {
        let mut tracker = Tracker::new(Config {
            min_zones: 0,
            ..Config::default()
        });
        assert_eq!(tracker.track(&[], &[], Instant::now()), None);
        // But a single usable zone still is one: the floor is on emptiness, not on count.
        let (distance, status) = frame(0.25, 5, 1);
        assert!(tracker.track(&distance, &status, Instant::now()).is_some());
    }

    /// A negative distance under a believed status is a failed convergence, and a frame
    /// shorter than the grid must not panic or read past its end — the wire carries vectors,
    /// and a peer from another release can send fewer.
    #[test]
    fn a_bad_frame_is_not_a_note() {
        let mut tracker = Tracker::new(Config::default());
        let now = Instant::now();
        assert_eq!(tracker.track(&[-3; N_ZONES], &[5; N_ZONES], now), None);
        assert_eq!(tracker.track(&[], &[], now), None);
        // Three distances, two statuses: only zones with both count, so this is one usable
        // zone and not a hand.
        assert_eq!(tracker.track(&[300, 300, 300], &[5, 255], now), None);
        assert!(
            tracker.track(&[300, 300, 300], &[5, 5], now).is_some(),
            "two usable zones is the minimum, and it is met"
        );
    }

    /// The histogram is the diagnostic that ends an argument about what the sensor is
    /// saying, so it has to be ordered by how much of the frame each status covers.
    #[test]
    fn the_histogram_leads_with_what_dominates_the_frame() {
        let mut status = vec![4u8; 40];
        status.extend(vec![255u8; 20]);
        status.extend(vec![5u8; 4]);
        let histogram = status_histogram(&status);
        assert_eq!(histogram[0], (4, 40));
        assert_eq!(histogram[1], (255, 20));
        assert_eq!(histogram[2], (5, 4));

        // Never reads past the grid, however long the wire's array is.
        let long = vec![7u8; N_ZONES * 3];
        assert_eq!(status_histogram(&long), vec![(7, N_ZONES)]);
        assert!(status_histogram(&[]).is_empty());
    }
}
