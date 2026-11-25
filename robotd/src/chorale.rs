//! Several ducks singing one piece: who conducts, who sings what, and where in the score we are.
//!
//! The behaviour half of the duck chorale. `btd` owns the radio and does no thinking; this owns the
//! thinking and touches no radio. What passes between them is `chorale.*`: a beacon to advertise
//! going down, and beacons heard coming up.
//!
//! ## The radio this has to work on
//!
//! Every failure seen on the floor — two songs at once, a duck sitting out while the others sing,
//! nothing happening at all — came from one property of BLE that the first design assumed away:
//! **discovery is slow, lossy and one-sided.** Duck A can have heard B for ten seconds while B has
//! not heard A at all, and C may hear A's followers long before it hears A. A protocol in which only
//! the conductor speaks, and in which each duck acts on the subset it has heard, splits on that
//! radio however its election is tuned. So the design below is judged against a simulated room with
//! a deliberately bad radio — per-pair discovery delays, loss, dead links, restarts — in
//! [`sim`], over hundreds of seeded rooms, and not against two ducks on a desk.
//!
//! ## A performance is something every singer advertises
//!
//! A performance is named by its conductor's id and a round the conductor bumps every piece. **Every
//! duck in a performance puts that name on the air**, with the piece, the roster and — once its
//! lock has settled — the beat, relayed. That one change is what the rest stands on:
//!
//!  - A duck arriving joins the moment it hears *anybody* singing, not only once it has found the
//!    conductor; and it can take the beat from a follower until it hears the conductor itself.
//!  - Two performances in one room meet as soon as any duck of one hears any duck of the other, and
//!    **the lower conductor id is the one that stays** — every duck that hears a lower performance
//!    moves to it, conductor included. Strictly lower, so nothing can move back, and every move is
//!    towards the room's minimum: from any arrangement the room converges.
//!  - A conductor that stops leaves a performance its followers can still finish together, because
//!    they are still telling each other where the beat is.
//!
//! ## Nobody is in charge until somebody is
//!
//! A duck asked for a chorale starts *listening*: it advertises an idle beacon and watches. If it
//! hears a performance it joins it. If it hears only idle ducks, the lowest id it can see conducts —
//! but deference is a timer, one extra [`SETTLE`] per lower id in view, because the lowest id in
//! one duck's view may be a duck that has heard nobody and so will never start. If two ducks do
//! start at once, the rule above merges them, and the count-in below makes that inaudible.
//!
//! ## Every piece starts with a count-in
//!
//! The conductor's beat counter runs [`count_in`] beats ahead of the score. Those beats are silent
//! and they are what the followers lock on — so everyone comes in on the first note together,
//! where before the conductor sang the first bar alone while the others' phase locks filled. It also
//! means a merge of two performances that started at the same moment happens before anyone sings.
//!
//! At the end of a piece the conductor does not go back to an election: after a breath it starts the
//! next piece, a new round, with the same ensemble. A piece boundary used to be a fresh chance for
//! the room to split.
//!
//! ## The conductor owns the seating
//!
//! Parts depend on who is singing ([`sounds::chorale::seat_all`]), so a duck seating *itself* from
//! whatever it happened to hear disagrees with a duck that heard a different subset. So the
//! conductor keeps the roster, broadcasts it, and everyone replays the seating over it; followers
//! relay it. A duck not in the roster yet is seated when the conductor hears it — and if the
//! conductor cannot hear it (a one-way link), it takes the seat it *would* be given after a few
//! seconds rather than sitting out the piece: singing a part twice is better than not singing.
//!
//! ## Where the score position comes from
//!
//! Not a start time. There is no clock to agree on, so the conductor's beat counter *is* the
//! timebase: [`sounds::chorale::beat::Conductor`] for the duck holding it and `Follower` for
//! everyone else. A follower relays the beat **early by the radio's own latency**
//! ([`SLOT_MEAN_S`]), so that a duck locked on a relay lands where a duck locked on the conductor
//! does.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use sounds::chorale::beat::{Conductor, Follower, SLOT_MEAN_S};
use sounds::chorale::{Part, Score, seat_all};

#[cfg(test)]
mod sim;

/// How long a heard beacon counts for.
///
/// A duck that has walked out of range stops being in the piece — otherwise a chorale would hold a
/// seat for a duck that left. Generous against a missed advertisement: at the beacon's interval
/// this is dozens of chances.
const PEER_STALE: Duration = Duration::from_secs(3);

/// How long to listen before starting a piece, and the unit of deference to a lower id.
///
/// A duck that has heard nobody does not sing: a solo chorale is a duck quacking to itself.
const SETTLE: Duration = Duration::from_millis(1500);

/// How long the count-in lasts, at least. Rounded up to whole beats and never fewer than
/// [`COUNT_IN_MIN_BEATS`].
const COUNT_IN_S: f64 = 3.0;

/// A follower needs four beats to lock (`Follower::locked`), so a count-in shorter than that would
/// have the followers come in late on every piece — which is the bug the count-in is for.
const COUNT_IN_MIN_BEATS: f64 = 5.0;

/// The silence between one piece and the next, in beats, after the last note.
const BREATH_BEATS: f64 = 2.0;

/// How long after the conductor's end of piece a follower gives up on it, in beats. Later than the
/// conductor's breath, so it is the conductor's next round that leads when both are healthy — this
/// is for a follower whose conductor vanished mid-final-chord.
const LET_GO_BEATS: f64 = 6.0;

/// While the conductor itself has been heard this recently, relayed beats are not used: they are a
/// second-hand estimate of what is available first-hand.
const DIRECT_FRESH_S: f64 = 2.0;

/// How long a locked follower waits for the conductor to seat it before seating itself.
const PROVISIONAL: Duration = Duration::from_secs(3);

/// How often an *idle* beacon changes, so that it is noticed at all.
///
/// A payload that never changes is an advertisement that is never re-registered, and on this
/// stack a duck is reported to a scanner mainly when it turns up at a new address — which,
/// with BLE privacy on, happens on the radio's schedule and not on ours. Two willing ducks in
/// a room took tens of seconds to find each other. So the idle beacon carries a slow counter
/// purely to make itself change; slow, because each change costs an advertisement
/// re-registration and a fresh random address.
const IDLE_HEARTBEAT: Duration = Duration::from_millis(1500);

/// The pieces a duck can sing, by the id the beacon carries.
///
/// The registry is what makes the beacon's `piece` byte mean something: the conductor picks an
/// id, followers load the same score by it, and a duck that does not know an id **does not join
/// it** — it cannot sing a piece it does not have.
const PIECE_WISTFUL: u8 = 1;
const PIECE_DUCK_STRUT: u8 = 2;
/// TEST ONLY — remove before release, with `Score::outer_wilds` and its asset.
const PIECE_OUTER_WILDS: u8 = 3;

/// Every piece a conductor may pick from.
const PIECES: [u8; 3] = [PIECE_WISTFUL, PIECE_DUCK_STRUT, PIECE_OUTER_WILDS];

/// Whether this build can sing a piece — what `robot.chorale` validates a `--piece` pin
/// against at the door.
///
/// From the id table, not by building the score: this is asked of every beacon heard, every tick.
pub fn known_piece(id: u8) -> bool {
    PIECES.contains(&id)
}

/// The catalogue, for the refusal that names what *is* on this robot.
pub fn piece_catalogue() -> String {
    PIECES
        .iter()
        .filter_map(|id| piece(*id).map(|score| format!("{id} ({})", score.name)))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The score for a piece id, or `None` for one this build does not know.
fn piece(id: u8) -> Option<Score> {
    match id {
        PIECE_WISTFUL => Some(Score::wistful()),
        PIECE_DUCK_STRUT => Some(Score::duck_strut()),
        PIECE_OUTER_WILDS => Some(Score::outer_wilds()),
        _ => None,
    }
}

/// How many silent beats precede the first note of a piece.
fn count_in(score: &Score) -> f64 {
    (COUNT_IN_S / score.beat_s()).ceil().max(COUNT_IN_MIN_BEATS)
}

/// The length of a piece, in beats.
fn length_beats(score: &Score) -> f64 {
    score.duration_s() / score.beat_s()
}

/// The beat of a performance's counter on which the conductor starts its next piece: the first
/// whole beat after the last note and a breath.
///
/// A function of the score alone, so a follower computes it too — which is what lets it know where
/// the next piece's beat 0 falls before it has heard a single beat of it, and come in on the first
/// note however it hears the conductor.
fn next_downbeat(score: &Score) -> f64 {
    (count_in(score) + length_beats(score) + BREATH_BEATS).ceil()
}

/// Whether round `a` is after round `b`, on a counter that wraps.
fn newer(a: u8, b: u8) -> bool {
    (1..128).contains(&a.wrapping_sub(b))
}

/// Another duck, as last heard.
#[derive(Debug, Clone)]
struct Peer {
    beacon: proto::ChoraleBeacon,
    /// Local time the beacon was heard — `btd` reports an age, and this is that age subtracted
    /// from the clock on arrival. The two daemons share a machine but not an epoch.
    at: Instant,
}

/// Which performance: whose, which of theirs, and what is being sung.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Performance {
    conductor: u16,
    round: u8,
    piece: u8,
}

impl Performance {
    /// The performance a singing beacon belongs to.
    fn of(beacon: &proto::ChoraleBeacon) -> Option<Self> {
        beacon.singing().then_some(Self {
            conductor: beacon.conductor,
            round: beacon.round,
            piece: beacon.piece,
        })
    }
}

/// A performance heard on the air.
#[derive(Debug, Clone)]
struct Heard {
    performance: Performance,
    /// The longest roster any of its singers is carrying — the conductor's only grows during a
    /// round, so the longest is the newest.
    roster: Vec<(u8, u16)>,
    /// Whether the conductor itself was heard saying this, rather than only its followers.
    first_hand: bool,
}

/// What this duck is doing.
enum State {
    /// Not asked for, or not allowed. Nothing on the air.
    Off,
    /// Willing: an idle beacon out, and listening.
    Listening { since: Instant },
    /// Holding the beat for everyone.
    Conducting {
        conductor: Conductor,
        round: u8,
        /// The roster, in seating order, as `(register, id)`. The conductor's own copy is the
        /// authority — everyone else reads it off the air.
        roster: Vec<(u8, u16)>,
    },
    /// Singing to somebody else's beat.
    Following {
        /// By the conductor's **beacon id** — never by its radio address, which rotates under BLE
        /// privacy every few seconds and once made a follower reject every beat after the first.
        performance: Performance,
        follower: Follower,
        roster: Vec<(u8, u16)>,
        /// The last counter taken from each duck, so a repeated advertisement is not another beat.
        /// A beacon repeats several times per beat, and feeding the repeats in would drag the
        /// phase late.
        last_beat: Vec<(u16, u8)>,
        /// When the conductor's own beat last arrived, in [`Chorale::seconds`].
        first_hand_at: Option<f64>,
        /// When the phase lock first held.
        locked_at: Option<Instant>,
    },
}

/// What the loop should do about the chorale this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// What to ask `btd` to advertise, when it has changed. `None` means "no change" — the beacon
    /// only needs resending when it changes, not fifty times a second.
    pub advertise: Option<proto::ChoraleAdvertise>,
    /// Which part this duck is singing, and how far into the score, in beats. `None` when it is not
    /// singing — listening, alone, or off. Negative during the count-in: the voice is up and
    /// silent, so the first note is not late by however long the audio takes to open.
    pub singing: Option<(Part, f64)>,
    /// In a performance but not yet singing — the phase lock is still filling, or the roster has
    /// not seated this duck yet. The state a readout must not call "listening".
    pub joining: bool,
    /// How many ducks are actually singing, this one included.
    ///
    /// Not the roster's length: a duck that walks out of range keeps its *seat* — pruning it would
    /// shift everyone onto a different part — so its line simply goes unsung, exactly as it would
    /// in a choir somebody left.
    pub voices: usize,
}

pub struct Chorale {
    /// This duck's own beacon identity: its register, quantised, and a sixteen-bit tie-break.
    register: u8,
    id: u16,
    /// The epoch every time in this module is measured from.
    started: Instant,
    /// The piece currently loaded — what [`Chorale::score`] serves the audio side — and its id.
    score: Score,
    piece_id: u8,
    state: State,
    peers: Vec<Peer>,
    /// A piece the operator has pinned — `robotctl chorale --piece`, or the
    /// `DUCK_CHORALE_PIECE` environment as the standing fallback. Only consulted when *this*
    /// duck conducts — a follower sings what the performance is singing.
    forced_piece: Option<u8>,
    /// The environment's standing pin, kept so a plain activation after a `--piece` one falls
    /// back to it rather than to the coin.
    env_piece: Option<u8>,
    /// The round this duck's next performance will carry. Kept across activations, so that
    /// `robotctl chorale` restarted on a conductor is a *new* performance to its old followers
    /// and not a confusing continuation of the one they are still finishing.
    next_round: u8,
    /// The beacon last handed to `btd`, so it is only resent when it changes.
    advertised: Option<proto::ChoraleBeacon>,
    listening: bool,
    /// Another duck is using this duck's id. See [`Chorale::listen`].
    collided: bool,
    /// What the last tick said, for the room simulation to look at.
    #[cfg(test)]
    last_singing: Option<(Part, f64)>,
}

impl Chorale {
    /// `seed` is the robot's voice seed — the identity everything else is derived from, and which
    /// deliberately does not go on the air.
    pub fn new(pitch_center_hz: f64, seed: u32, env_piece: Option<u8>) -> Self {
        // A round that a restarted daemon is unlikely to repeat, so its first performance is not
        // mistaken for the one its old followers may still be relaying.
        let round = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|t| t.subsec_nanos() as u8 ^ (t.as_secs() as u8))
            .unwrap_or(0);
        Self::with_round(pitch_center_hz, seed, env_piece, round)
    }

    fn with_round(pitch_center_hz: f64, seed: u32, env_piece: Option<u8>, round: u8) -> Self {
        let env_piece = env_piece.filter(|id| {
            let known = piece(*id).is_some();
            if !known {
                tracing::warn!(
                    piece = id,
                    "DUCK_CHORALE_PIECE names a piece this build lacks; ignoring"
                );
            }
            known
        });
        Self {
            register: proto::ChoraleBeacon::quantise_register(pitch_center_hz),
            // Sixteen bits of the seed, mixed — not the eight it used to be: this id is also
            // the own-reflection filter and the peer-merge key, and a real four-duck room
            // collided a byte-sized pair on day one.
            id: (seed.wrapping_mul(2_654_435_761) >> 16) as u16,
            started: Instant::now(),
            score: Score::wistful(),
            piece_id: PIECE_WISTFUL,
            forced_piece: env_piece,
            env_piece,
            next_round: round,
            state: State::Off,
            peers: Vec::new(),
            advertised: None,
            listening: false,
            collided: false,
            #[cfg(test)]
            last_singing: None,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.state, State::Off)
    }

    /// Start listening for other ducks, or stop and fall silent. `piece_pin` pins what this
    /// duck picks if it ends up conducting; `None` falls back to the environment's pin, then
    /// the coin. Validated at the door by `robot.chorale`, so an unknown id never gets here.
    pub fn set_active(&mut self, active: bool, now: Instant, piece_pin: Option<u8>) {
        if active {
            self.forced_piece = piece_pin
                .filter(|id| piece(*id).is_some())
                .or(self.env_piece);
        }
        match (active, &self.state) {
            (true, State::Off) => {
                tracing::warn!(register = self.register, id = self.id, "chorale: listening");
                self.state = State::Listening { since: now };
            }
            (true, _) => {}
            (false, _) => {
                if self.active() {
                    tracing::warn!("chorale: stopping");
                }
                self.state = State::Off;
                self.peers.clear();
            }
        }
    }

    /// A beacon `btd` heard.
    pub fn heard(&mut self, heard: &proto::ChoraleHeard, now: Instant) {
        if !self.active() {
            return;
        }
        // Our own beacon, reflected by something. Not a peer.
        if heard.beacon.register == self.register && heard.beacon.id == self.id {
            return;
        }
        // Somebody else with our id. Sixteen bits make that rare, not impossible, and two ducks
        // sharing one would each think the other's performance is its own ghost.
        if heard.beacon.id == self.id {
            if !self.collided {
                tracing::warn!(id = self.id, "chorale: another duck has this id");
            }
            self.collided = true;
            return;
        }
        let at = now
            .checked_sub(Duration::from_micros(heard.age_us))
            .unwrap_or(now);
        let beacon = &heard.beacon;
        match self.peers.iter_mut().find(|p| p.beacon.id == beacon.id) {
            Some(existing) => {
                // Arrivals can be reported out of order; the newest one is the duck's state.
                if at >= existing.at {
                    existing.beacon = beacon.clone();
                    existing.at = at;
                }
            }
            None => {
                tracing::warn!(
                    register = beacon.register,
                    id = beacon.id,
                    from = %heard.from,
                    "chorale: another duck"
                );
                self.peers.push(Peer {
                    beacon: beacon.clone(),
                    at,
                });
            }
        }

        // A beat of the performance this duck is following: hand it to the phase lock, stamped when
        // it was heard rather than when it was processed. Membership is decided in `tick`; here
        // only the timing is taken, because it is the one thing that cannot wait for a tick.
        let arrival = self.seconds(at);
        let State::Following {
            performance,
            follower,
            last_beat,
            first_hand_at,
            ..
        } = &mut self.state
        else {
            return;
        };
        if Performance::of(beacon) != Some(*performance) || !beacon.timed {
            return;
        }
        let first_hand = beacon.id == performance.conductor;
        let previous = match last_beat.iter_mut().find(|(id, _)| *id == beacon.id) {
            Some((_, last)) => Some(std::mem::replace(last, beacon.beat)),
            None => {
                last_beat.push((beacon.id, beacon.beat));
                None
            }
        };
        // **Only a change of counter is a beat.** The first sighting of a duck says which beat it
        // is on, not when that beat began — it arrived somewhere in the middle of it, up to a whole
        // beat late. Taking it as a downbeat is how four early observations poisoned a lock.
        let Some(previous) = previous else {
            return;
        };
        if previous == beacon.beat {
            return;
        }
        if first_hand {
            // The conductor's counter jumping backwards inside one round is a daemon restart that
            // happened to land on the same round. A new phase lock, or the old fit is poisoned.
            if (4..128).contains(&previous.wrapping_sub(beacon.beat)) {
                *follower = Follower::new(self.score.bpm);
                last_beat.retain(|(id, _)| *id == beacon.id);
            }
            *first_hand_at = Some(arrival);
        } else if first_hand_at.is_some_and(|t| arrival - t < DIRECT_FRESH_S) {
            return;
        }
        follower.observe(beacon.beat, arrival);
    }

    /// A slowly-turning byte, so an idle beacon changes and is therefore noticed. See
    /// [`IDLE_HEARTBEAT`].
    fn heartbeat(&self, now: Instant) -> u8 {
        (self.seconds(now) / IDLE_HEARTBEAT.as_secs_f64()) as u64 as u8
    }

    /// Seconds since this module's epoch — the clock `sounds::chorale::beat` speaks in.
    fn seconds(&self, at: Instant) -> f64 {
        at.saturating_duration_since(self.started).as_secs_f64()
    }

    /// The performance in the room this duck should be part of, if any: the lowest conductor id
    /// heard singing anything this build knows, from anyone.
    ///
    /// A performance led by this duck's own id is not a candidate — it is this duck's, or the ghost
    /// of one it stopped, which its old followers are finishing without it.
    fn loudest(&self) -> Option<Heard> {
        let conductor = self
            .peers
            .iter()
            .filter_map(|p| Performance::of(&p.beacon))
            .filter(|perf| perf.conductor != self.id && known_piece(perf.piece))
            .map(|perf| perf.conductor)
            .min()?;
        let singers: Vec<&proto::ChoraleBeacon> = self
            .peers
            .iter()
            .map(|p| &p.beacon)
            .filter(|b| b.singing() && b.conductor == conductor && known_piece(b.piece))
            .collect();
        // Which of that conductor's rounds: its own word if it can be heard, otherwise the newest
        // any of its followers is relaying.
        let own = singers.iter().find(|b| b.id == conductor).copied();
        let chosen = own.or_else(|| {
            singers
                .iter()
                .copied()
                .find(|b| {
                    singers
                        .iter()
                        .all(|o| o.round == b.round || newer(b.round, o.round))
                })
                .or(singers.first().copied())
        })?;
        let performance = Performance::of(chosen)?;
        let roster = match own {
            Some(own) => own.roster.clone(),
            None => singers
                .iter()
                .filter(|b| Performance::of(b) == Some(performance))
                .map(|b| &b.roster)
                .max_by_key(|r| r.len())
                .cloned()
                .unwrap_or_default(),
        };
        Some(Heard {
            performance,
            roster,
            first_hand: own.is_some(),
        })
    }

    /// One tick. Cheap enough for every one; only the beacon is rate-limited, by changing rarely.
    pub fn tick(&mut self, now: Instant) -> Tick {
        self.peers
            .retain(|peer| now.saturating_duration_since(peer.at) < PEER_STALE);
        let tick = match &self.state {
            State::Off => Tick {
                advertise: self.publish(None, false),
                singing: None,
                joining: false,
                voices: 0,
            },
            State::Listening { since } => {
                let since = *since;
                self.listen(since, now)
            }
            State::Conducting { .. } => self.conduct(now),
            State::Following { .. } => self.follow(now),
        };
        #[cfg(test)]
        {
            self.last_singing = tick.singing;
        }
        tick
    }

    fn listen(&mut self, since: Instant, now: Instant) -> Tick {
        // An id collision is settled here, the one state where changing identity breaks nothing:
        // both ducks mix their own register in, and their registers differ — they told each other
        // apart by exactly that.
        if self.collided {
            self.collided = false;
            self.id = (u32::from(self.id) ^ (u32::from(self.register) << 8 | 0x5A))
                .wrapping_mul(2_654_435_761) as u16
                | 1;
            tracing::warn!(id = self.id, "chorale: took a new id");
        }
        // Somebody is singing: join it rather than starting a second piece.
        if let Some(heard) = self.loudest() {
            self.join(heard, None);
            return self.follow(now);
        }
        // Nobody singing. The lowest id conducts — but only for as long as it takes, because the
        // lowest id in *this* duck's view may be a duck that has heard nobody and will never start.
        // One extra settle per lower id in view, so a takeover goes in id order rather than three
        // ducks starting three pieces at once.
        let lower = self
            .peers
            .iter()
            .filter(|peer| peer.beacon.id < self.id)
            .count();
        let patience = SETTLE + SETTLE.mul_f64(lower as f64);
        // Somebody singing a piece this build does not know is still somebody singing: no rival
        // performance over the top of it.
        // (Its own old performance, which a restarted conductor's followers may still be
        // finishing, is not that: that duck starts the next round, and they move to it.)
        let unknown_singing = self
            .peers
            .iter()
            .any(|p| p.beacon.singing() && !known_piece(p.beacon.piece));
        if !self.peers.is_empty()
            && !unknown_singing
            && now.saturating_duration_since(since) >= patience
        {
            let mut roster: Vec<(u8, u16)> = vec![(self.register, self.id)];
            roster.extend(self.peers.iter().map(|p| (p.beacon.register, p.beacon.id)));
            roster.truncate(proto::ChoraleBeacon::MAX_ROSTER);
            tracing::warn!(
                voices = roster.len(),
                deferred_to = lower,
                "chorale: conducting"
            );
            self.start_piece(roster, self.seconds(now), now);
            return self.conduct(now);
        }
        let idle = self.idle_beacon(now);
        Tick {
            advertise: self.publish(Some(idle), true),
            singing: None,
            joining: false,
            voices: self.peers.len(),
        }
    }

    /// Start a performance of a freshly picked piece, with this duck conducting, its beat 0 at
    /// `at` in [`Chorale::seconds`].
    fn start_piece(&mut self, roster: Vec<(u8, u16)>, at: f64, now: Instant) {
        // The conductor picks the piece — forced by the operator if they said so, otherwise from
        // the clock's low bits: as good as a coin, and deterministic under a test.
        let pick = self
            .forced_piece
            .unwrap_or_else(|| PIECES[(self.seconds(now) * 997.0) as u64 as usize % PIECES.len()]);
        self.score = piece(pick).expect("every listed piece exists");
        self.piece_id = pick;
        let round = self.next_round;
        self.next_round = round.wrapping_add(1);
        tracing::warn!(piece = pick, round, "chorale: a new piece");
        self.state = State::Conducting {
            conductor: Conductor::new(self.score.bpm, at),
            round,
            roster,
        };
    }

    /// Become a singer in somebody else's performance — with the phase already known, when it is
    /// the next piece of the one this duck was locked to.
    fn join(&mut self, heard: Heard, phase: Option<f64>) {
        let score = piece(heard.performance.piece).expect("only known pieces are heard");
        tracing::warn!(
            conductor = heard.performance.conductor,
            round = heard.performance.round,
            piece = heard.performance.piece,
            first_hand = heard.first_hand,
            "chorale: joining"
        );
        self.state = State::Following {
            performance: heard.performance,
            follower: match phase {
                Some(phase) => Follower::seeded(score.bpm, phase),
                None => Follower::new(score.bpm),
            },
            roster: heard.roster,
            last_beat: Vec::new(),
            first_hand_at: None,
            locked_at: None,
        };
        self.score = score;
        self.piece_id = heard.performance.piece;
    }

    fn conduct(&mut self, now: Instant) -> Tick {
        // **A lower conductor is singing, and this one gives way** — to whichever duck it heard it
        // from, conductor or follower. Strictly lower, so the room only ever moves towards its
        // minimum and nothing oscillates.
        if let Some(heard) = self.loudest()
            && heard.performance.conductor < self.id
        {
            tracing::warn!(
                to = heard.performance.conductor,
                "chorale: yielding to a lower conductor"
            );
            self.join(heard, None);
            return self.follow(now);
        }

        // Seat everyone willing who is not committed to a performance that will win over this one:
        // idle ducks, this performance's own joiners, and the singers of a higher one, who are
        // about to give way to it.
        //
        // And everyone this performance's followers say is singing in it: a follower relays the
        // roster with any unseated singer it can hear appended, which is how a duck the conductor
        // cannot hear — a one-way link — still gets a seat everyone agrees on.
        let mut seatable: Vec<(u8, u16)> = self
            .peers
            .iter()
            .filter(|p| !p.beacon.singing() || p.beacon.conductor >= self.id)
            .map(|p| (p.beacon.register, p.beacon.id))
            .collect();
        seatable.extend(
            self.peers
                .iter()
                .filter(|p| p.beacon.singing() && p.beacon.conductor == self.id)
                .flat_map(|p| p.beacon.roster.iter().copied()),
        );
        let seconds = self.seconds(now);
        let State::Conducting {
            conductor,
            round,
            roster,
        } = &mut self.state
        else {
            unreachable!("checked by the caller");
        };
        for entry in seatable {
            if roster.len() < proto::ChoraleBeacon::MAX_ROSTER
                && !roster.iter().any(|(_, id)| *id == entry.1)
            {
                tracing::warn!(id = entry.1, "chorale: seating a new voice");
                roster.push(entry);
            }
        }
        conductor.due(seconds);
        let position = conductor.position_beats(seconds) - count_in(&self.score);
        let beat = conductor.wire_beat();
        let next = conductor.time_of_beat(next_downbeat(&self.score));
        let round = *round;
        let roster = roster.clone();

        // The piece ends, and the next one starts after a breath — with the same ensemble, without
        // going back to an election, and **on this piece's grid**, at the beat [`next_downbeat`]
        // names, so that every follower already knows where it falls. Unless nobody is left.
        if seconds >= next {
            let here: Vec<(u8, u16)> = roster
                .iter()
                .copied()
                .filter(|entry| self.audible(entry))
                .collect();
            if here.len() > 1 {
                self.start_piece(here, next, now);
                return self.conduct(now);
            }
            tracing::warn!("chorale: the performance ended, and nobody is left");
            self.state = State::Listening { since: now };
            return self.listen(now, now);
        }

        let beacon = proto::ChoraleBeacon {
            piece: self.piece_id,
            beat,
            register: self.register,
            id: self.id,
            conductor: self.id,
            round,
            timed: true,
            roster: roster.clone(),
        };
        Tick {
            advertise: self.publish(Some(beacon), true),
            singing: self.part(&roster, false).map(|part| (part, position)),
            joining: false,
            voices: self.voices(&roster),
        }
    }

    fn follow(&mut self, now: Instant) -> Tick {
        let State::Following { performance, .. } = &self.state else {
            unreachable!("checked by the caller");
        };
        let mine = *performance;
        // Somewhere better to be: a lower conductor, or this conductor's next piece.
        if let Some(heard) = self.loudest() {
            let other = heard.performance;
            let move_on = other.conductor < mine.conductor
                || (other.conductor == mine.conductor
                    && other != mine
                    && (heard.first_hand || newer(other.round, mine.round)));
            if move_on {
                // This conductor's very next piece starts on the old grid: carry the lock over.
                // Only when this duck's own count agrees the old piece has reached its end —
                // a conductor restarted mid-piece also starts the next round, from nowhere near
                // that beat, and seeding from the grid there put its followers 64 beats out.
                let downbeat = next_downbeat(&self.score);
                let seconds = self.seconds(now);
                let phase = match &self.state {
                    State::Following { follower, .. }
                        if other.conductor == mine.conductor
                            && other.round == mine.round.wrapping_add(1) =>
                    {
                        follower
                            .position_beats(seconds)
                            .filter(|counter| (downbeat - 1.0..downbeat + 4.0).contains(counter))
                            .and(follower.phase())
                            .map(|phase| phase + downbeat * self.score.beat_s())
                    }
                    _ => None,
                };
                self.join(heard, phase);
                return self.follow(now);
            }
        }

        // The roster, from whoever in this performance has the freshest: the conductor if it can be
        // heard, otherwise the longest any follower relays.
        let roster_heard = self
            .peers
            .iter()
            .find(|p| p.beacon.id == mine.conductor && Performance::of(&p.beacon) == Some(mine))
            .or_else(|| {
                self.peers
                    .iter()
                    .filter(|p| Performance::of(&p.beacon) == Some(mine))
                    .max_by_key(|p| p.beacon.roster.len())
            })
            .map(|p| p.beacon.roster.clone());

        let seconds = self.seconds(now);
        let count_in = count_in(&self.score);
        let length = length_beats(&self.score);
        let beat_s = self.score.beat_s();
        let State::Following {
            follower,
            roster,
            locked_at,
            ..
        } = &mut self.state
        else {
            unreachable!("checked above");
        };
        if let Some(heard) = roster_heard
            && !heard.is_empty()
        {
            *roster = heard;
        }
        let counter = follower.position_beats(seconds);
        if counter.is_some() && locked_at.is_none() {
            *locked_at = Some(now);
        }
        let provisional =
            locked_at.is_some_and(|t| now.saturating_duration_since(t) >= PROVISIONAL);
        let roster = roster.clone();
        let position = counter.map(|c| c - count_in);
        // What goes on the air is the roster plus a proposal: every singer of this performance this
        // duck can hear — itself included — that the roster has not seated yet. The conductor seats
        // what it hears proposed. Only a duck that cannot hear the conductor at all sings from a
        // relayed roster, proposals and all, and that is the best guess there is.
        let mut relayed = roster.clone();
        let me = (self.register, self.id);
        let proposals = std::iter::once(me).chain(
            self.peers
                .iter()
                .filter(|p| Performance::of(&p.beacon) == Some(mine))
                .map(|p| (p.beacon.register, p.beacon.id)),
        );
        for entry in proposals {
            if relayed.len() < proto::ChoraleBeacon::MAX_ROSTER
                && !relayed.iter().any(|(_, id)| *id == entry.1)
            {
                relayed.push(entry);
            }
        }

        if position.is_some_and(|beats| beats > length + LET_GO_BEATS) {
            tracing::warn!(conductor = mine.conductor, "chorale: the performance ended");
            self.state = State::Listening { since: now };
            return self.listen(now, now);
        }

        // Relay the performance — and the beat, once locked, put on the air early by the radio's own
        // latency so that a duck locked on this one lands where a duck locked on the conductor does.
        let beacon = proto::ChoraleBeacon {
            piece: mine.piece,
            beat: counter.map_or(0, |c| {
                (c + SLOT_MEAN_S / beat_s).floor().rem_euclid(256.0) as u8
            }),
            register: self.register,
            id: self.id,
            conductor: mine.conductor,
            round: mine.round,
            timed: counter.is_some(),
            roster: relayed,
        };
        let singing =
            position.and_then(|at| self.part(&roster, provisional).map(|part| (part, at)));
        Tick {
            advertise: self.publish(Some(beacon), true),
            joining: singing.is_none(),
            singing,
            voices: self.voices(&roster),
        }
    }

    /// Whether a roster entry is singing within earshot: this duck, or a peer heard recently.
    fn audible(&self, (register, id): &(u8, u16)) -> bool {
        (*register == self.register && *id == self.id)
            || self.peers.iter().any(|peer| peer.beacon.id == *id)
    }

    /// Roster entries that are still audible. The roster keeps a departed duck's seat so nobody
    /// else is reseated; this is the count of who is actually singing.
    fn voices(&self, roster: &[(u8, u16)]) -> usize {
        roster.iter().filter(|entry| self.audible(entry)).count()
    }

    /// This duck's part, from the roster the conductor published.
    ///
    /// `None` when it is not in the roster yet — the ordinary state of a duck that has just
    /// arrived; the conductor seats it on the next beat. With `provisional`, a duck the conductor
    /// has not seated takes the part it would be given if it were seated next — what a duck the
    /// conductor cannot hear does rather than sitting out the piece. Not past a full roster: a fifth
    /// duck listening is the design.
    fn part(&self, roster: &[(u8, u16)], provisional: bool) -> Option<Part> {
        let mut registers: Vec<f64> = roster
            .iter()
            .map(|(register, _)| register_hz(*register))
            .collect();
        let seat = match roster
            .iter()
            .position(|(register, id)| *register == self.register && *id == self.id)
        {
            Some(seat) => seat,
            None if provisional && roster.len() < proto::ChoraleBeacon::MAX_ROSTER => {
                registers.push(register_hz(self.register));
                registers.len() - 1
            }
            None => return None,
        };
        seat_all(&registers).get(seat).copied()
    }

    fn idle_beacon(&self, now: Instant) -> proto::ChoraleBeacon {
        proto::ChoraleBeacon {
            piece: proto::ChoraleBeacon::IDLE,
            beat: self.heartbeat(now),
            register: self.register,
            id: self.id,
            conductor: 0,
            round: 0,
            timed: false,
            roster: Vec::new(),
        }
    }

    /// Hand `btd` a beacon, but only when it has actually changed.
    ///
    /// The loop runs at 50 Hz and the beat turns over about once a second, so this is the
    /// difference between two D-Bus round trips a second and a hundred.
    fn publish(
        &mut self,
        beacon: Option<proto::ChoraleBeacon>,
        listening: bool,
    ) -> Option<proto::ChoraleAdvertise> {
        if beacon == self.advertised && listening == self.listening {
            return None;
        }
        self.advertised = beacon.clone();
        self.listening = listening;
        Some(proto::ChoraleAdvertise { beacon, listening })
    }

    /// The score being sung, for the audio side to render.
    pub fn score(&self) -> &Score {
        &self.score
    }

    #[cfg(test)]
    fn sim_debug(&self) -> String {
        match &self.state {
            State::Following {
                performance,
                follower,
                last_beat,
                first_hand_at,
                roster,
                ..
            } => format!(
                "id {} following {:?} sources {:?} first_hand_at {:?} spread {:?} roster {:?}",
                self.id,
                performance,
                last_beat,
                first_hand_at,
                follower.spread_s(),
                roster
            ),
            State::Conducting { round, roster, .. } => {
                format!("id {} conducting round {round} roster {roster:?}", self.id)
            }
            State::Listening { .. } => format!("id {} listening", self.id),
            State::Off => format!("id {} off", self.id),
        }
    }

    /// What the room simulation looks at: the performance, the part and position as of the last
    /// tick, and the piece's beat and length.
    #[cfg(test)]
    fn sim_view(&self) -> sim::View {
        let performance = match &self.state {
            State::Conducting { round, .. } => Some((self.id, *round, self.piece_id)),
            State::Following { performance, .. } => {
                Some((performance.conductor, performance.round, performance.piece))
            }
            _ => None,
        };
        (
            performance,
            self.last_singing,
            self.score.beat_s(),
            length_beats(&self.score),
        )
    }
}

/// A register byte as the pitch centre it stands for.
fn register_hz(register: u8) -> f64 {
    proto::ChoraleBeacon::REGISTER_LOW_HZ
        + f64::from(register) * proto::ChoraleBeacon::REGISTER_STEP_HZ
}

/// Expressive head offsets while singing: `[neck_pitch, head_pitch, head_yaw, head_roll]`.
///
/// Driven by the **score position**, not by local time — and that is the trick: every duck in
/// the ensemble computes this from the same shared beat, so the whole group sways in phase
/// with nobody coordinating anything. Choreography falls out of the sync work for free.
///
/// `reach` is where the current note sits in this duck's own line (0 low, 1 high): the head
/// lifts on the high notes, which is what a singer actually does.
///
/// Amplitudes are deliberately small — the head carries the ToF and the policy's balance has
/// opinions about mass this high up. The pitch sign assumes negative is up, as `robot.look`'s
/// examples suggest; if hardware says otherwise, flip `REACH_LIFT`.
pub fn head_expression(beats: f64, reach: f64) -> [f64; 4] {
    const SWAY_ROLL: f64 = 0.08;
    const DRIFT_YAW: f64 = 0.05;
    const REACH_LIFT: f64 = -0.10;
    const BOB_PITCH: f64 = 0.025;
    let bar = std::f64::consts::TAU * beats / 4.0;
    let phrase = std::f64::consts::TAU * beats / 8.0;
    let beat = std::f64::consts::TAU * beats;
    [
        0.0,
        REACH_LIFT * reach + BOB_PITCH * beat.sin(),
        DRIFT_YAW * phrase.sin(),
        SWAY_ROLL * bar.sin(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chorale() -> Chorale {
        Chorale::with_round(214.4, 7, None, 0)
    }

    fn idle(id: u16, register: u8, heartbeat: u8) -> proto::ChoraleHeard {
        heard(proto::ChoraleBeacon {
            piece: proto::ChoraleBeacon::IDLE,
            beat: heartbeat,
            register,
            id,
            conductor: 0,
            round: 0,
            timed: false,
            roster: vec![],
        })
    }

    /// A conductor's own beacon.
    fn conducting(
        id: u16,
        register: u8,
        piece: u8,
        round: u8,
        beat: u8,
        roster: Vec<(u8, u16)>,
    ) -> proto::ChoraleHeard {
        heard(proto::ChoraleBeacon {
            piece,
            beat,
            register,
            id,
            conductor: id,
            round,
            timed: true,
            roster,
        })
    }

    fn heard(beacon: proto::ChoraleBeacon) -> proto::ChoraleHeard {
        proto::ChoraleHeard {
            from: format!("AA:BB:CC:DD:{:04X}", beacon.id),
            beacon,
            age_us: 2_000,
        }
    }

    fn beacon_of(tick: &Tick) -> Option<&proto::ChoraleBeacon> {
        tick.advertise.as_ref().and_then(|a| a.beacon.as_ref())
    }

    /// Feed `beats` beats of a conductor, one per `beat_s`, ticking in between as the loop does.
    fn follow_beats(
        c: &mut Chorale,
        start: Instant,
        beat_s: f64,
        beats: std::ops::Range<u8>,
        beacon: impl Fn(u8) -> proto::ChoraleHeard,
    ) -> Instant {
        let mut at = start;
        for (i, beat) in beats.enumerate() {
            at = start + Duration::from_secs_f64(beat_s * i as f64);
            c.heard(&beacon(beat), at);
            c.tick(at);
        }
        at
    }

    /// A duck that hears nobody must not sing. A solo chorale is a duck quacking to itself.
    #[test]
    fn one_duck_alone_does_not_sing() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let first = c.tick(now);
        let advertise = first.advertise.clone().expect("something to advertise");
        assert!(advertise.listening);
        let beacon = advertise.beacon.expect("an idle beacon");
        assert!(!beacon.singing());
        assert!(beacon.roster.is_empty());
        for step in 0..200 {
            let tick = c.tick(now + Duration::from_millis(50 * step));
            assert_eq!(tick.singing, None, "step {step}");
        }
    }

    /// Off means off: nothing on the air, nothing heard, nothing sung.
    #[test]
    fn a_duck_that_was_not_asked_is_silent_and_invisible() {
        let mut c = chorale();
        let now = Instant::now();
        let tick = c.tick(now);
        assert_eq!(tick.singing, None);
        assert_eq!(tick.advertise, None);
        c.heard(
            &conducting(2, 120, PIECE_WISTFUL, 0, 5, vec![(120, 2)]),
            now,
        );
        assert_eq!(c.tick(now).singing, None);
        assert!(!c.active());
    }

    /// Two willing ducks agree who conducts with no election: the lower id does.
    #[test]
    fn the_lower_id_conducts_and_the_other_waits_its_turn() {
        let now = Instant::now();
        let mine = chorale().id;

        let mut leads = chorale();
        leads.set_active(true, now, None);
        leads.heard(&idle(mine.wrapping_add(1), 200, 0), now);
        let tick = leads.tick(now + SETTLE);
        let beacon = beacon_of(&tick).expect("a beacon");
        assert!(beacon.singing(), "the lower id conducts: {tick:?}");
        assert_eq!(beacon.conductor, mine, "and names itself as the conductor");
        assert!(beacon.timed);
        assert_eq!(tick.voices, 2);

        let mut defers = chorale();
        defers.set_active(true, now, None);
        defers.heard(&idle(mine.wrapping_sub(1), 200, 0), now);
        let tick = defers.tick(now + SETTLE);
        assert!(
            beacon_of(&tick).is_none_or(|b| !b.singing()),
            "the higher id must not also conduct: {tick:?}"
        );
    }

    /// Deference expires, or a duck that thinks it is alone holds the whole room silent: the
    /// lowest id in view may be a duck whose scan has found nobody, and which will never start.
    #[test]
    fn a_lower_duck_that_never_starts_does_not_hold_the_room() {
        let now = Instant::now();
        let lower = chorale().id.wrapping_sub(1);
        let mut c = chorale();
        c.set_active(true, now, None);
        let mut singing = false;
        for step in 0..40u64 {
            let at = now + Duration::from_millis(100 * step);
            // Idle and on the air the whole time, its heartbeat turning over.
            c.heard(&idle(lower, 200, (step / 15) as u8), at);
            let tick = c.tick(at);
            let conducting = beacon_of(&tick).is_some_and(|b| b.singing());
            if at < now + SETTLE * 2 {
                assert!(!conducting, "it is the lower duck's turn first ({step})");
            }
            singing |= conducting;
        }
        assert!(
            singing,
            "the lower duck heard and silent, this one must start"
        );
    }

    /// A conductor that hears a lower performance gives way — and it does not have to hear that
    /// performance's conductor to do so: any of its singers will do. Strictly lower, so a higher
    /// one never unseats it.
    #[test]
    fn a_conductor_yields_to_a_lower_performance_heard_from_a_follower() {
        let now = Instant::now();
        let mine = chorale().id;
        let lower = mine.wrapping_sub(10);
        let higher = mine.wrapping_add(1);

        let mut c = chorale();
        c.set_active(true, now, None);
        c.heard(&idle(higher, 200, 0), now);
        let tick = c.tick(now + SETTLE);
        let my_piece = beacon_of(&tick).expect("conducting").piece;
        let other = if my_piece == PIECE_WISTFUL {
            PIECE_DUCK_STRUT
        } else {
            PIECE_WISTFUL
        };

        // A higher-id rival performance does not move it.
        let at = now + SETTLE + Duration::from_millis(100);
        c.heard(
            &conducting(higher, 200, other, 9, 4, vec![(200, higher)]),
            at,
        );
        let tick = c.tick(at);
        assert!(beacon_of(&tick).is_none_or(|b| b.conductor == mine));
        assert_eq!(c.piece_id, my_piece, "a higher rival must not unseat it");

        // A follower of a lower conductor does: relayed, untimed even.
        let at = at + Duration::from_millis(100);
        c.heard(
            &heard(proto::ChoraleBeacon {
                piece: other,
                beat: 0,
                register: 90,
                id: higher.wrapping_add(7),
                conductor: lower,
                round: 3,
                timed: false,
                roster: vec![(150, lower)],
            }),
            at,
        );
        let tick = c.tick(at);
        let beacon = beacon_of(&tick).expect("a beacon");
        assert_eq!(
            beacon.conductor, lower,
            "now in the lower performance: {beacon:?}"
        );
        assert_ne!(
            beacon.id, beacon.conductor,
            "as a singer, not its conductor"
        );
        assert_eq!(c.piece_id, other, "singing what it is singing");
    }

    /// A follower relays its performance — conductor, round, piece, roster, and the beat once
    /// locked — which is what lets a duck that has not found the conductor yet join it anyway.
    #[test]
    fn a_follower_relays_the_performance_it_is_in() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let roster = vec![(120u8, 9u16), mine];
        let tick = c.tick(now);
        assert!(beacon_of(&tick).is_some_and(|b| !b.singing()));

        c.heard(
            &conducting(9, 120, PIECE_WISTFUL, 4, 0, roster.clone()),
            now,
        );
        let tick = c.tick(now);
        let beacon = beacon_of(&tick).expect("relaying");
        assert_eq!(
            (beacon.conductor, beacon.round, beacon.piece),
            (9, 4, PIECE_WISTFUL)
        );
        assert!(!beacon.timed, "no beat to relay before the lock");

        let beat_s = c.score().beat_s();
        let at = follow_beats(&mut c, now, beat_s, 0..8, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 4, beat, roster.clone())
        });
        let beacon = c.advertised.clone().expect("relaying");
        assert!(beacon.timed, "locked, so the beat is relayed: {beacon:?}");
        assert_eq!(beacon.roster, roster);
        let _ = at;
    }

    /// A duck that can only hear a follower — the conductor out of its earshot — still joins and
    /// sings, on the follower's relayed beat.
    #[test]
    fn a_duck_that_cannot_hear_the_conductor_sings_on_a_relayed_beat() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let roster = vec![(120u8, 9u16), (60, 30), mine];
        let beat_s = Score::wistful().beat_s();
        let at = follow_beats(&mut c, now, beat_s, 0..10, |beat| {
            heard(proto::ChoraleBeacon {
                piece: PIECE_WISTFUL,
                beat,
                register: 60,
                id: 30,
                conductor: 9,
                round: 1,
                timed: true,
                roster: roster.clone(),
            })
        });
        let tick = c.tick(at);
        assert!(tick.singing.is_some(), "singing from a relay: {tick:?}");
    }

    /// The first sighting of a duck is not a beat: it says which beat, not when it began. Feeding
    /// it to the phase lock — which the first design did — puts up to a whole beat of error into
    /// a four-beat average, and the outlier filter then defends the wrong answer.
    #[test]
    fn the_first_sighting_is_not_a_downbeat() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let roster = vec![(120u8, 9u16), mine];
        let beat_s = Score::wistful().beat_s();
        // First heard most of the way through beat 10; then on the beat, from 11.
        c.heard(
            &conducting(9, 120, PIECE_WISTFUL, 1, 10, roster.clone()),
            now,
        );
        c.tick(now);
        let start = now + Duration::from_secs_f64(0.2 * beat_s);
        let at = follow_beats(&mut c, start, beat_s, 11..16, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 1, beat, roster.clone())
        });
        let State::Following { follower, .. } = &c.state else {
            panic!("following");
        };
        let spread = follower.spread_s().expect("locked");
        assert!(spread < 1e-6, "every observation on the beat: {spread}");
        let _ = at;
    }

    /// The conductor's next piece is a new round, and a follower moves to it with its lock carried
    /// over — singing during the count-in, not a bar late.
    #[test]
    fn the_next_piece_is_followed_without_a_gap() {
        let now = Instant::now();
        let mut conductor = Chorale::with_round(300.0, 1, Some(PIECE_DUCK_STRUT), 40);
        let mut follower = Chorale::with_round(200.0, 2, None, 0);
        // Make the conductor the lower id, whatever the seeds give.
        if conductor.id > follower.id {
            std::mem::swap(&mut conductor.id, &mut follower.id);
        }
        conductor.set_active(true, now, Some(PIECE_DUCK_STRUT));
        follower.set_active(true, now, None);
        let mut round_changed_at = None;
        let mut gap = 0;
        // Two pieces of duck strut, the radio a perfect one 120 ms long.
        let mut on_air: Vec<(Instant, proto::ChoraleBeacon)> = Vec::new();
        let mut follower_air: Option<proto::ChoraleBeacon> = None;
        for step in 0..12_000u64 {
            let at = now + Duration::from_millis(20 * step);
            if let Some(b) = on_air.iter().rev().find(|(t, _)| *t <= at) {
                follower.heard(&heard(b.1.clone()), at);
            }
            if let Some(b) = &follower_air {
                conductor.heard(&heard(b.clone()), at);
            }
            let tick = conductor.tick(at);
            if let Some(b) = tick.advertise.and_then(|a| a.beacon) {
                if on_air
                    .last()
                    .is_some_and(|(_, last)| last.singing() && last.round != b.round)
                {
                    round_changed_at = Some(at);
                }
                on_air.push((at + Duration::from_millis(120), b));
            }
            let tick = follower.tick(at);
            if let Some(b) = tick.advertise.and_then(|a| a.beacon) {
                follower_air = Some(b);
            }
            if round_changed_at.is_some_and(|t| at > t + Duration::from_millis(300)) {
                match tick.singing {
                    Some(_) => break,
                    None => gap += 1,
                }
            }
        }
        assert!(
            round_changed_at.is_some(),
            "the conductor went on to a second piece"
        );
        assert_eq!(
            gap, 0,
            "the follower was singing (the count-in) from the new round's start"
        );
        let (_, beats) = follower.last_singing.expect("singing");
        assert!(
            beats < 0.0,
            "and the new piece is still in its count-in: {beats}"
        );
    }

    /// A restarted conductor — `robotctl chorale` killed and run again — starts a new round, and its
    /// old followers, still finishing the old one together, move to it.
    #[test]
    fn a_conductor_restarting_takes_its_followers_to_a_new_round() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let beat_s = Score::wistful().beat_s();
        follow_beats(&mut c, now, beat_s, 10..16, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 7, beat, vec![(120, 9), mine])
        });
        assert_eq!(c.score().name, "wistful");

        let later = now + Duration::from_secs(20);
        let strut = Score::duck_strut().beat_s();
        let at = follow_beats(&mut c, later, strut, 0..8, |beat| {
            conducting(9, 120, PIECE_DUCK_STRUT, 8, beat, vec![(120, 9), mine])
        });
        assert_eq!(c.score().name, "duck-strut", "moved to the new performance");
        assert!(c.tick(at).singing.is_some(), "and locked onto its beat");
    }

    /// The same round with the conductor's counter going backwards is a daemon restart that landed
    /// on the same round: the phase lock starts again rather than free-running on a stale fit.
    #[test]
    fn a_beat_counter_going_backwards_is_a_new_lock() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let beat_s = Score::wistful().beat_s();
        let at = follow_beats(&mut c, now, beat_s, 40..46, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 2, beat, vec![(120, 9), mine])
        });
        let before = c.tick(at).singing.expect("singing").1;
        assert!(before > 25.0, "well into the piece: {before}");

        let later = now + Duration::from_secs(30);
        let at = follow_beats(&mut c, later, beat_s, 0..8, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 2, beat, vec![(120, 9), mine])
        });
        let after = c.tick(at).singing.expect("singing").1;
        assert!(after < 5.0, "the position followed the restart: {after}");
    }

    /// `DUCK_CHORALE_PIECE` pins the conductor's pick; an id this build lacks is ignored rather
    /// than obeyed into silence.
    #[test]
    fn a_forced_piece_wins_the_coin_toss() {
        let now = Instant::now();
        let mut c = Chorale::with_round(214.4, 7, Some(PIECE_OUTER_WILDS), 0);
        c.set_active(true, now, None);
        c.heard(&idle(c.id.wrapping_add(1), 250, 0), now);
        let tick = c.tick(now + SETTLE);
        assert_eq!(
            beacon_of(&tick).expect("conducting").piece,
            PIECE_OUTER_WILDS
        );
        assert_eq!(c.score().name, "outer-wilds");

        let mut c = Chorale::with_round(214.4, 7, Some(200), 0);
        c.set_active(true, now, None);
        c.heard(&idle(c.id.wrapping_add(1), 250, 0), now);
        let tick = c.tick(now + SETTLE);
        assert!(known_piece(beacon_of(&tick).expect("conducting").piece));
    }

    /// The `--piece` pin arrives with the activation and wins the coin; a plain activation
    /// afterwards goes back to the coin.
    #[test]
    fn a_piece_pinned_at_activation_wins_and_expires() {
        let now = Instant::now();
        let mut c = chorale();
        c.set_active(true, now, Some(PIECE_DUCK_STRUT));
        c.heard(&idle(c.id.wrapping_add(1), 250, 0), now);
        let tick = c.tick(now + SETTLE);
        assert_eq!(
            beacon_of(&tick).expect("conducting").piece,
            PIECE_DUCK_STRUT
        );

        c.set_active(false, now, None);
        c.tick(now + Duration::from_secs(8));
        assert_eq!(c.forced_piece, Some(PIECE_DUCK_STRUT), "until reactivated");
        c.set_active(true, now + Duration::from_secs(9), None);
        assert_eq!(
            c.forced_piece, None,
            "a plain activation goes back to the coin"
        );
    }

    /// Every piece opens with a silent count-in long enough for a follower to lock, so nobody
    /// sings the first bar alone — the conductor used to, while its followers' locks filled.
    #[test]
    fn a_piece_starts_with_a_count_in() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        c.heard(&idle(c.id.wrapping_add(1), 250, 0), now);
        let tick = c.tick(now + SETTLE);
        let (_, beats) = tick.singing.expect("the conductor's voice is up");
        assert!(
            beats < -4.0,
            "in the count-in, before the first note: {beats}"
        );
        for score in [Score::wistful(), Score::duck_strut(), Score::outer_wilds()] {
            assert!(
                count_in(&score) >= 5.0,
                "{}: a follower needs a beat to see a change and four to lock",
                score.name
            );
        }
    }

    /// At the end of a piece the conductor goes straight on to the next — a new round, the same
    /// ensemble — rather than back to an election. Alone, it stops.
    #[test]
    fn the_conductor_goes_on_to_the_next_piece_and_stops_when_alone() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let other = c.id.wrapping_add(1);
        c.heard(&idle(other, 250, 0), now);
        let first = beacon_of(&c.tick(now + SETTLE))
            .expect("conducting")
            .clone();
        let end = next_downbeat(c.score()) * c.score().beat_s();

        // Its follower relaying, all the way through.
        let at = now + SETTLE + Duration::from_secs_f64(end + 0.1);
        c.heard(
            &heard(proto::ChoraleBeacon {
                piece: first.piece,
                beat: 0,
                register: 250,
                id: other,
                conductor: c.id,
                round: first.round,
                timed: true,
                roster: first.roster.clone(),
            }),
            at,
        );
        let tick = c.tick(at);
        let next = beacon_of(&tick).expect("a new beacon");
        assert!(next.singing(), "still conducting: {next:?}");
        assert_eq!(next.round, first.round.wrapping_add(1), "a new round");
        assert!(tick.singing.expect("singing").1 < 0.0, "in its count-in");

        // Nobody left: when that piece ends, back to listening.
        let end = next_downbeat(c.score()) * c.score().beat_s();
        let tick = c.tick(at + Duration::from_secs_f64(end + 1.0));
        assert_eq!(tick.singing, None);
        assert!(
            beacon_of(&tick).is_some_and(|b| !b.singing()),
            "an idle beacon"
        );
    }

    /// A follower whose conductor vanished mid-piece finishes it — singing on its lock, as a
    /// singer does when they cannot hear the conductor — and then lets go.
    #[test]
    fn a_follower_finishes_the_piece_and_lets_go() {
        let mut f = chorale();
        let now = Instant::now();
        f.set_active(true, now, None);
        let mine = (f.register, f.id);
        let beat_s = Score::wistful().beat_s();
        let at = follow_beats(&mut f, now, beat_s, 0..8, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 0, beat, vec![(120, 9), mine])
        });
        let silent = at + PEER_STALE + Duration::from_secs(1);
        assert!(
            f.tick(silent).singing.is_some(),
            "still singing with the conductor gone"
        );
        let total = count_in(f.score()) + length_beats(f.score()) + LET_GO_BEATS + 1.0;
        let after = now + Duration::from_secs_f64(total * beat_s);
        assert_eq!(f.tick(after).singing, None, "and lets go at the end");
    }

    /// The collision that once hid a real duck: a same-register, different-id peer is a peer.
    #[test]
    fn a_duck_sharing_a_register_is_still_a_different_duck() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let near = c.id ^ 0x0100;
        let beat_s = Score::wistful().beat_s();
        let at = follow_beats(&mut c, now, beat_s, 0..8, |beat| {
            conducting(
                near,
                c_register(),
                PIECE_WISTFUL,
                0,
                beat,
                vec![(c_register(), near), mine],
            )
        });
        assert!(c.tick(at).singing.is_some());
        let before = c.tick(at).voices;
        c.heard(&idle(c.id, c.register, 0), at);
        assert_eq!(
            c.tick(at).voices,
            before,
            "its own reflection is still dropped"
        );
    }

    fn c_register() -> u8 {
        chorale().register
    }

    /// Two ducks on one id each re-derive theirs — from their registers, which differ — the next
    /// time they are listening. Rare at sixteen bits, and fatal when it happens: each takes the
    /// other's performance for its own ghost.
    #[test]
    fn an_id_collision_is_resolved() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let before = c.id;
        c.heard(&idle(c.id, c.register.wrapping_add(40), 0), now);
        c.tick(now);
        assert_ne!(c.id, before, "a new id");
    }

    #[test]
    fn the_head_sways_in_phase_and_stays_small() {
        for step in 0..200 {
            let beats = f64::from(step) * 0.13;
            let a = head_expression(beats, 0.3);
            assert_eq!(a, head_expression(beats, 0.3), "same beat, same sway");
            for (joint, offset) in a.iter().enumerate() {
                assert!(offset.abs() <= 0.15, "joint {joint} at {offset} rad");
            }
        }
        assert_ne!(head_expression(0.0, 0.0), head_expression(1.0, 0.0));
        let low = head_expression(2.0, 0.0)[1];
        let high = head_expression(2.0, 1.0)[1];
        assert!((high - low).abs() > 0.05, "reach must be visible");
    }

    /// The beacon's piece byte decides the song, bpm and all.
    #[test]
    fn a_joiner_sings_the_piece_the_beacon_names() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        c.heard(
            &conducting(9, 120, PIECE_DUCK_STRUT, 0, 0, vec![(120, 9), mine]),
            now,
        );
        let _ = c.tick(now);
        assert_eq!(c.score().name, "duck-strut");
        assert!((c.score().bpm - 126.0).abs() < 0.5);
    }

    /// A piece this build does not know is not joined, and not sung over either.
    #[test]
    fn an_unknown_piece_is_declined_not_guessed() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        for step in 0..50 {
            let at = now + Duration::from_millis(100 * step);
            c.heard(&conducting(9, 120, 200, 0, 4, vec![(120, 9)]), at);
            let tick = c.tick(at);
            assert_eq!(tick.singing, None, "step {step}");
            if let Some(beacon) = beacon_of(&tick) {
                assert!(!beacon.singing(), "conducting over an ongoing piece");
            }
        }
    }

    /// THE regression test for the bug present twice: the conductor's radio address rotates every
    /// few seconds, so identity must come from the beacon.
    #[test]
    fn the_conductor_is_followed_across_its_rotating_addresses() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let beat_s = Score::wistful().beat_s();
        let mut at = now;
        for beat in 0..8u8 {
            at = now + Duration::from_secs_f64(beat_s * f64::from(beat));
            let mut h = conducting(9, 120, PIECE_WISTFUL, 0, beat, vec![(120, 9), mine]);
            h.from = format!("{beat:02X}:AA:BB:CC:DD:EE");
            c.heard(&h, at);
            c.tick(at);
        }
        let tick = c.tick(at);
        let (part, _) = tick.singing.expect("locked and seated, so singing");
        // Register bytes decode near 234 Hz (this duck) and 347 Hz (the conductor): the bass.
        assert_eq!(part, Part::Bass);
        assert_eq!(tick.voices, 2, "{tick:?}");
    }

    /// An idle beacon must change on its own, or a waiting duck is only noticed when the radio
    /// happens to rotate its address.
    #[test]
    fn an_idle_beacon_has_a_heartbeat() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let first = beacon_of(&c.tick(now)).expect("idle").clone();
        assert_eq!(c.tick(now + Duration::from_millis(300)).advertise, None);
        let later = c.tick(now + IDLE_HEARTBEAT + Duration::from_millis(100));
        let later = beacon_of(&later).expect("the heartbeat re-advertises");
        assert_ne!(&first, later);
        assert!(!later.singing());
    }

    /// The conductor seats a newcomer without moving anyone: the roster grows and never reorders.
    #[test]
    fn the_roster_grows_and_never_reorders() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = c.id;
        c.heard(&idle(mine.wrapping_add(1), 250, 0), now);
        let first = beacon_of(&c.tick(now + SETTLE))
            .expect("conducting")
            .clone();
        assert_eq!(first.roster.len(), 2);
        c.heard(&idle(mine.wrapping_add(2), 60, 0), now + SETTLE);
        let tick = c.tick(now + SETTLE + Duration::from_millis(20));
        let second = beacon_of(&tick).expect("still conducting");
        assert_eq!(second.roster.len(), 3);
        assert_eq!(
            &second.roster[..2],
            &first.roster[..],
            "the newcomer went on the end"
        );
    }

    /// A duck the conductor cannot hear is proposed for a seat by a follower that can, and the
    /// conductor seats it from the proposal — everyone then agrees on the parts.
    #[test]
    fn a_conductor_seats_a_duck_it_only_hears_about() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let follower = c.id.wrapping_add(1);
        c.heard(&idle(follower, 250, 0), now);
        let first = beacon_of(&c.tick(now + SETTLE))
            .expect("conducting")
            .clone();
        let unheard = (60u8, c.id.wrapping_add(2));
        let mut proposal = first.roster.clone();
        proposal.push(unheard);
        let at = now + SETTLE + Duration::from_millis(100);
        c.heard(
            &heard(proto::ChoraleBeacon {
                piece: first.piece,
                beat: 0,
                register: 250,
                id: follower,
                conductor: c.id,
                round: first.round,
                timed: true,
                roster: proposal,
            }),
            at,
        );
        let tick = c.tick(at);
        let roster = &beacon_of(&tick).expect("still conducting").roster;
        assert!(
            roster.contains(&unheard),
            "seated from the proposal: {roster:?}"
        );
    }

    /// A follower the conductor never seats — a one-way link nobody relays across — sings the part
    /// it would be given rather than sitting the piece out, after a grace.
    #[test]
    fn an_unseated_follower_seats_itself_after_a_grace() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let beat_s = Score::wistful().beat_s();
        let roster = vec![(120u8, 9u16), (250, 11)];
        let at = follow_beats(&mut c, now, beat_s, 0..8, |beat| {
            conducting(9, 120, PIECE_WISTFUL, 0, beat, roster.clone())
        });
        // Locked, unseated: waiting.
        let tick = c.tick(at);
        assert_eq!(tick.singing, None, "{tick:?}");
        assert!(tick.joining);
        // And it asks: its relayed roster proposes itself.
        let relayed = c.advertised.clone().expect("relaying").roster;
        assert!(relayed.contains(&(c.register, c.id)), "{relayed:?}");
        let later = at + PROVISIONAL + Duration::from_millis(100);
        c.heard(
            &conducting(9, 120, PIECE_WISTFUL, 0, 9, roster.clone()),
            later,
        );
        assert!(c.tick(later).singing.is_some(), "seated itself");
    }

    /// A duck that leaves stops being counted, and keeps its seat.
    #[test]
    fn a_duck_that_leaves_stops_being_counted() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        c.heard(&idle(c.id.wrapping_add(1), 250, 0), now);
        assert_eq!(c.tick(now + SETTLE).voices, 2);
        let later = now + SETTLE + PEER_STALE + Duration::from_millis(100);
        let tick = c.tick(later);
        assert_eq!(tick.voices, 1, "{tick:?}");
        assert_eq!(
            tick.singing.expect("still singing").0,
            Part::Bass,
            "its part kept"
        );
        assert_eq!(c.advertised.as_ref().expect("conducting").roster.len(), 2);
    }

    /// The beacon is resent only when it changes.
    #[test]
    fn the_beacon_is_only_published_when_it_changes() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        assert!(c.tick(now).advertise.is_some());
        for step in 1..20 {
            assert_eq!(
                c.tick(now + Duration::from_millis(10 * step)).advertise,
                None
            );
        }
        c.heard(&idle(c.id.wrapping_add(1), 250, 0), now);
        assert!(c.tick(now + SETTLE).advertise.is_some());
    }
}

/// Whole rooms on a bad radio — see [`sim`]. These are the tests the field reports were about.
#[cfg(test)]
mod room {
    use super::sim::*;

    fn quartet() -> Vec<(f64, u32)> {
        vec![(180.0, 11), (214.4, 7), (240.0, 23), (300.0, 31)]
    }

    fn assert_clean(name: &str, scenario: &Scenario, seed: u64) -> Verdict {
        let verdict = run(scenario, seed);
        assert!(verdict.samples > 0);
        assert_eq!(
            verdict.bad(),
            0,
            "{name}: {verdict:?}\n{}",
            verdict
                .log
                .iter()
                .take(40)
                .cloned()
                .collect::<Vec<_>>()
                .join("\n")
        );
        verdict
    }

    /// Four ducks on a perfect radio come in together on the first note, and stay together
    /// through the change of piece.
    #[test]
    fn a_quartet_on_a_good_radio() {
        let scenario = Scenario {
            ducks: quartet(),
            events: (0..4).map(|i| (i as u64 * 300, Event::On(i))).collect(),
            weather: Weather::perfect(),
            duration_ms: 200_000,
            judge_from_ms: 8_000,
        };
        let verdict = assert_clean("perfect", &scenario, 1);
        assert!(verdict.worst_spread_s < 0.03, "{}", verdict.worst_spread_s);
    }

    /// The reported room (PR #151): `robotctl chorale` typed by hand seconds apart, and two halves
    /// of the room electing their own conductors before they can hear each other. They must end
    /// on one performance — and since the count-in hides the merge, without a duck singing the
    /// wrong song for long.
    #[test]
    fn two_islands_that_meet_end_on_one_performance() {
        let scenario = Scenario {
            ducks: quartet(),
            events: vec![
                (0, Event::On(0)),
                (400, Event::On(1)),
                (1_000, Event::On(2)),
                (1_600, Event::On(3)),
            ],
            weather: Weather::perfect().islands(&[0, 1], &[2, 3], 20_000),
            duration_ms: 160_000,
            judge_from_ms: 25_000,
        };
        assert_clean("islands", &scenario, 2);
    }

    /// A duck that cannot hear the conductor at all — but can hear the others — sings, in time,
    /// with a seat everyone agrees on.
    #[test]
    fn a_duck_deaf_to_the_conductor_still_sings() {
        for deaf_to in 0..4 {
            let mut weather = Weather {
                loss: 0.2,
                hit: 0.5,
                ..Weather::perfect()
            };
            // Whoever conducts, duck 2 cannot hear duck `deaf_to`.
            if deaf_to != 2 {
                weather.deaf.push((deaf_to, 2, u64::MAX));
            }
            let scenario = Scenario {
                ducks: quartet(),
                events: (0..4).map(|i| (i as u64 * 700, Event::On(i))).collect(),
                weather,
                duration_ms: 160_000,
                judge_from_ms: 20_000,
            };
            assert_clean(&format!("deaf to {deaf_to}"), &scenario, 3 + deaf_to as u64);
        }
    }

    /// Restarts in the middle of a piece — every duck in turn, the conductor included.
    #[test]
    fn restarts_mid_piece() {
        for who in 0..4 {
            let mut events: Vec<(u64, Event)> =
                (0..4).map(|i| (i as u64 * 200, Event::On(i))).collect();
            events.push((30_000, Event::Off(who)));
            events.push((33_000, Event::On(who)));
            let scenario = Scenario {
                ducks: quartet(),
                events,
                weather: Weather {
                    discovery_ms: 3_000,
                    loss: 0.3,
                    hit: 0.5,
                    ..Weather::perfect()
                },
                duration_ms: 180_000,
                judge_from_ms: 55_000,
            };
            assert_clean(&format!("restart {who}"), &scenario, 10 + who as u64);
        }
    }

    /// The sweep: random rooms — two to four ducks, together or apart, latecomers, restarts,
    /// dead links, and a radio anywhere from perfect to awful. Each seed is a room that
    /// `ROOM_SEED=<n> cargo test -p robotd chorale::room::replay -- --nocapture` replays with its log.
    ///
    /// No room may ever sing two songs at once, leave a duck sitting out, or put two ducks on one
    /// line. Timing is held to the ensemble's standard — off the beat for under 2% of a run, a
    /// moment of jitter — except on the very worst radios, a sighting every 70 ms or rarer, where a
    /// follower's lag behind the beat is mostly the radio's latency differing from
    /// `beat::SLOT_MEAN_S`, and a duck behind a dead link inherits it once more from the relay. That is a constant to measure on hardware, not a protocol fault, and the room still
    /// sings one song.
    #[test]
    fn random_rooms() {
        let rooms: u64 = std::env::var("ROOMS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(if cfg!(debug_assertions) { 60 } else { 400 });
        let mut timing = 0;
        for seed in 0..rooms {
            let scenario = random_scenario(seed);
            let verdict = run(&scenario, seed);
            let w = &scenario.weather;
            assert_eq!(
                verdict.split + verdict.sitting_out + verdict.doubled,
                0,
                "seed {seed}: {w:?} {verdict:?}"
            );
            let catch = w.hit * (1.0 - w.loss);
            if verdict.discord > 0 {
                assert!(
                    catch <= 0.2 + 1e-9 || verdict.discord * 50 < verdict.samples,
                    "seed {seed}: off the beat on a usable radio: {w:?} {verdict:?}"
                );
                timing += 1;
            }
        }
        println!("{rooms} rooms: one song every time; {timing} drifted on the worst radios");
    }

    /// Replay one seed of [`random_rooms`] with its log.
    #[test]
    fn replay() {
        let Some(seed) = std::env::var("ROOM_SEED").ok().and_then(|v| v.parse().ok()) else {
            return;
        };
        let scenario = random_scenario(seed);
        println!("{scenario:#?}");
        let verdict = run(&scenario, seed);
        for line in &verdict.log {
            println!("{line}");
        }
        println!("{verdict:?}");
    }
}
