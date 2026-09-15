//! Several ducks singing one piece: who conducts, who sings what, and where in the score we are.
//!
//! The behaviour half of the duck chorale. `btd` owns the radio and does no thinking; this owns the
//! thinking and touches no radio. What passes between them is `chorale.*`: a beacon to advertise
//! going down, and beacons heard coming up.
//!
//! ## Nobody is in charge until somebody is
//!
//! A duck asked for a chorale starts *listening*: it advertises an idle beacon saying it is
//! willing, and watches for others. When two willing ducks see each other the lower id conducts —
//! deterministic, so there is no election to go wrong and no message to lose. A duck that hears a
//! beacon already carrying a piece does not argue about it; it joins.
//!
//! ## The conductor owns the seating, and that is not a convenience
//!
//! Seating depends on join order ([`sounds::chorale::seat`]), so a duck seating *itself* from
//! whatever it happened to hear will disagree with a duck that heard a different subset — and both
//! will sing alto. So the conductor keeps the roster, broadcasts it, and everyone replays
//! [`sounds::chorale::seat_all`] over it. One source of truth, which is what a conductor is.
//!
//! A duck that cannot find itself in the roster is not in the piece yet: it keeps listening, and
//! the conductor adds it on the next beat. That is why joining is free rather than negotiated.
//!
//! ## Where the score position comes from
//!
//! Not a start time. There is no clock to agree on, so the conductor's beat counter *is* the
//! timebase: [`sounds::chorale::beat::Conductor`] for the duck holding it and `Follower` for
//! everyone else, and both answer the same question — how many beats into the piece are we, right
//! now. The audio side reads that and renders the score at it, so a duck whose audio stalls
//! resumes in the right place rather than a bar behind.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use sounds::chorale::beat::{Conductor, Follower};
use sounds::chorale::{Part, Score, seat_all};

/// How long a heard beacon counts for.
///
/// A duck that has walked out of range stops being in the piece — otherwise a chorale would hold a
/// seat for a duck that left, and the roster it broadcasts would name someone who is not singing.
/// Generous against a missed advertisement: at the beacon's interval this is dozens of chances.
const PEER_STALE: Duration = Duration::from_secs(3);

/// How long to listen before starting a piece alone.
///
/// A duck that has heard nobody does not sing: a solo chorale is a duck quacking to itself. This is
/// only the settling time before *two* ducks that can both see each other agree who conducts —
/// long enough that they have both certainly heard the other, so they cannot both decide they are
/// alone.
const SETTLE: Duration = Duration::from_millis(1500);

/// The pieces a duck can sing, by the id the beacon carries.
///
/// The registry is what makes the beacon's `piece` byte mean something: the conductor picks an
/// id, followers load the same score by it, and a duck that does not know an id **keeps
/// listening rather than joining** — it cannot sing a piece it does not have, and guessing one
/// is how two ducks end up performing different songs at each other. That is also the right
/// degradation for a mixed-version flock: an old duck near new ones stays politely quiet.
const PIECE_WISTFUL: u8 = 1;
const PIECE_DUCK_STRUT: u8 = 2;
/// TEST ONLY — remove before release, with `Score::outer_wilds` and its asset.
const PIECE_OUTER_WILDS: u8 = 3;

/// Every piece a conductor may pick from.
const PIECES: [u8; 3] = [PIECE_WISTFUL, PIECE_DUCK_STRUT, PIECE_OUTER_WILDS];

/// Whether this build can sing a piece — what `robot.chorale` validates a `--piece` pin
/// against at the door.
pub fn known_piece(id: u8) -> bool {
    piece(id).is_some()
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

/// How often an *idle* beacon changes, so that it is noticed at all.
///
/// A payload that never changes is an advertisement that is never re-registered, and on this
/// stack a duck is reported to a scanner mainly when it turns up at a new address — which,
/// with BLE privacy on, happens on the radio's schedule and not on ours. Two willing ducks in
/// a room took tens of seconds to find each other. So the idle beacon carries a slow counter
/// purely to make itself change; slow, because each change costs an advertisement
/// re-registration and a fresh random address.
const IDLE_HEARTBEAT: Duration = Duration::from_millis(1500);

/// Another duck, as last heard.
#[derive(Debug, Clone)]
struct Peer {
    beacon: proto::ChoraleBeacon,
    /// Local time the beacon was heard — `btd` reports an age, and this is that age subtracted
    /// from the clock on arrival. The two daemons share a machine but not an epoch.
    at: Instant,
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
        /// The roster, in seating order, as `(register, id)`. The conductor's own copy is the
        /// authority — everyone else reads it off the beacon.
        roster: Vec<(u8, u16)>,
    },
    /// Singing to somebody else's beat.
    Following {
        follower: Follower,
        /// Which duck is conducting, by its **beacon id** — never by its radio address.
        ///
        /// This was the address, and it is the bug that made a chorale never synchronise —
        /// twice, because the first fix was committed with a message describing it and a patch
        /// that had silently not applied. These robots advertise with BLE privacy on, so the
        /// address is a resolvable random one that rotates every few seconds: a follower that
        /// adopted the conductor at one address rejected every beat after the rotation, the
        /// phase lock starved on its single observation, and the duck never sang. The beacon
        /// carries `(register, id)` precisely so identity never comes from the radio layer.
        ///
        /// `None` until the first singing beacon names one.
        conductor: Option<u16>,
        roster: Vec<(u8, u16)>,
        /// The counter last taken, so a repeated beacon is not counted as another beat.
        last_beat: Option<u8>,
    },
}

/// What the loop should do about the chorale this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// What to ask `btd` to advertise, when it has changed. `None` means "no change" — the beacon
    /// only needs resending when the beat turns over, not fifty times a second.
    pub advertise: Option<proto::ChoraleAdvertise>,
    /// Which part this duck is singing, and how far into the score, in beats. `None` when it is not
    /// singing — listening, alone, or off.
    pub singing: Option<(Part, f64)>,
    /// Following a conductor but not yet singing — the phase lock is still filling, or the
    /// roster has not seated this duck yet. The state a readout must not call "listening".
    pub joining: bool,
    /// How many ducks are actually singing, this one included.
    ///
    /// Not the roster's length: a duck that walks out of range keeps its *seat* — pruning it would
    /// shift everyone below it onto a different part, which is the one thing the roster exists to
    /// prevent — so its line simply goes unsung, exactly as it would in a choir somebody left.
    pub voices: usize,
}

pub struct Chorale {
    /// This duck's own beacon identity: its register, quantised, and a sixteen-bit tie-break.
    register: u8,
    id: u16,
    /// The epoch every time in this module is measured from.
    ///
    /// `sounds::chorale::beat` works in plain seconds and `robotd` works in `Instant`s, so one of
    /// them has to convert. Doing it here keeps the beat maths free of anything platform-shaped,
    /// which is what lets it be tested against simulated jitter on a laptop.
    started: Instant,
    /// The piece currently loaded — what [`Chorale::score`] serves the audio side, and what
    /// [`Self::piece_id`] names on the air. Swapped when a performance starts or is joined.
    score: Score,
    piece_id: u8,
    state: State,
    peers: Vec<Peer>,
    /// A piece the operator has pinned — `robotctl chorale --piece`, or the
    /// `DUCK_CHORALE_PIECE` environment as the standing fallback. Only consulted when *this*
    /// duck conducts — a follower sings what the beacon names, because an ensemble where
    /// everyone insists on their own song is not one.
    forced_piece: Option<u8>,
    /// The environment's standing pin, kept so a plain activation after a `--piece` one falls
    /// back to it rather than to the coin.
    env_piece: Option<u8>,
    /// The beacon last handed to `btd`, so it is only resent when it changes.
    advertised: Option<proto::ChoraleBeacon>,
    listening: bool,
}

impl Chorale {
    /// `seed` is the robot's voice seed — the identity everything else is derived from, and which
    /// deliberately does not go on the air.
    pub fn new(pitch_center_hz: f64, seed: u32, env_piece: Option<u8>) -> Self {
        Self {
            register: proto::ChoraleBeacon::quantise_register(pitch_center_hz),
            // Sixteen bits of the seed, mixed — not the eight it used to be: this id is also
            // the own-reflection filter and the peer-merge key, and a real four-duck room
            // collided a byte-sized pair on day one (the fourth duck rolled the conductor's
            // byte, was merged into it by everyone, and could never join). Still not enough
            // to identify a robot, which is the point of it not being the seed.
            id: (seed.wrapping_mul(2_654_435_761) >> 16) as u16,
            started: Instant::now(),
            score: Score::wistful(),
            piece_id: PIECE_WISTFUL,
            // Filtered once, here — the raw value must not survive anywhere, or an unknown id
            // would resurface through the fallback and reach the pick as a panic.
            forced_piece: env_piece.filter(|id| piece(*id).is_some()),
            env_piece: env_piece.filter(|id| {
                let known = piece(*id).is_some();
                if !known {
                    tracing::warn!(
                        piece = id,
                        "DUCK_CHORALE_PIECE names a piece this build lacks; ignoring"
                    );
                }
                known
            }),
            state: State::Off,
            peers: Vec::new(),
            advertised: None,
            listening: false,
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
        self.set_active_inner(active, now);
    }

    fn set_active_inner(&mut self, active: bool, now: Instant) {
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
        // `btd` reports an age rather than a timestamp — the two daemons share a machine but not an
        // epoch — so the arrival time is that age subtracted from the clock here.
        let at = now
            .checked_sub(Duration::from_micros(heard.age_us))
            .unwrap_or(now);
        let peer = Peer {
            beacon: heard.beacon.clone(),
            at,
        };
        match self
            .peers
            .iter_mut()
            .find(|existing| existing.beacon.id == peer.beacon.id)
        {
            Some(existing) => *existing = peer,
            None => {
                tracing::warn!(
                    register = heard.beacon.register,
                    id = heard.beacon.id,
                    from = %heard.from,
                    "chorale: another duck"
                );
                self.peers.push(peer);
            }
        }

        // A beat from the duck we are following: hand it to the phase lock, stamped when it was
        // heard rather than when it was processed.
        let arrival = at.saturating_duration_since(self.started).as_secs_f64();
        if let State::Following {
            follower,
            conductor,
            roster,
            last_beat,
        } = &mut self.state
        {
            // The first singing beacon names the conductor; after that another duck's beacon
            // cannot pull this one off its beat — which matters when two pieces briefly
            // overlap in one room. By beacon id, never by `heard.from`: the radio address
            // rotates under BLE privacy, and keying on it is how a follower ends up rejecting
            // every beat its conductor sends after the first few seconds.
            if conductor.is_none() && heard.beacon.singing() {
                *conductor = Some(heard.beacon.id);
                tracing::warn!(conductor = heard.beacon.id, "chorale: following");
            }
            if *conductor != Some(heard.beacon.id) || !heard.beacon.singing() {
                return;
            }
            // The same conductor starting a *new performance* — a different piece, or the
            // beat counter jumping backwards (its `robotctl chorale` was restarted, which on
            // a bench happens constantly). Without this, a follower kept singing the old song
            // against the conductor's new one: it never re-read the piece byte, and the beat
            // reset poisoned its phase lock into free-running on a stale fit. A backwards
            // step of a few beats is a restart; a small forwards gap is just beats missed.
            let restarted = heard.beacon.piece != self.piece_id
                || last_beat.is_some_and(|last| {
                    let back = last.wrapping_sub(heard.beacon.beat);
                    (4..128).contains(&back)
                });
            if restarted {
                let Some(score) = piece(heard.beacon.piece) else {
                    // It now sings something this build does not know. Back to listening —
                    // quiet is better than wrong.
                    tracing::warn!(
                        piece = heard.beacon.piece,
                        "chorale: unknown piece; leaving"
                    );
                    self.state = State::Listening { since: at };
                    return;
                };
                tracing::warn!(
                    piece = heard.beacon.piece,
                    "chorale: the conductor started a new performance"
                );
                self.piece_id = heard.beacon.piece;
                self.state = State::Following {
                    follower: Follower::new(score.bpm),
                    conductor: Some(heard.beacon.id),
                    roster: heard.beacon.roster.clone(),
                    last_beat: Some(heard.beacon.beat),
                };
                self.score = score;
                if let State::Following { follower, .. } = &mut self.state {
                    follower.observe(heard.beacon.beat, arrival);
                }
                return;
            }
            roster.clone_from(&heard.beacon.roster);
            // **Only on a change of counter.** A beacon repeats several times per beat, and
            // re-reading the same value is not another beat — it is the same beat, later, and
            // feeding those in would drag the phase late by half an advertising interval.
            if *last_beat != Some(heard.beacon.beat) {
                *last_beat = Some(heard.beacon.beat);
                follower.observe(heard.beacon.beat, arrival);
            }
        }
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

    /// One tick. Cheap enough for every one; only the beacon is rate-limited, by changing rarely.
    pub fn tick(&mut self, now: Instant) -> Tick {
        self.peers
            .retain(|peer| now.saturating_duration_since(peer.at) < PEER_STALE);

        match &mut self.state {
            State::Off => {
                let advertise = self.publish(None, false);
                Tick {
                    advertise,
                    singing: None,
                    joining: false,
                    voices: 0,
                }
            }
            State::Listening { since } => {
                let since = *since;
                // Somebody is already singing: join it rather than starting a second piece —
                // but only a piece this build knows. An unknown id means a newer flock; the
                // right move is to keep listening, not to guess at a song.
                if let Some((peer, score)) = self
                    .peers
                    .iter()
                    .filter(|peer| peer.beacon.singing())
                    .find_map(|peer| piece(peer.beacon.piece).map(|score| (peer.clone(), score)))
                {
                    tracing::warn!(
                        piece = peer.beacon.piece,
                        conductor = peer.beacon.id,
                        "chorale: joining"
                    );
                    self.piece_id = peer.beacon.piece;
                    self.state = State::Following {
                        follower: Follower::new(score.bpm),
                        conductor: None,
                        roster: peer.beacon.roster.clone(),
                        last_beat: None,
                    };
                    self.score = score;
                    // The address is not in the beacon, so the first `heard` from this conductor
                    // adopts it — until then this duck listens without a lock, which is what
                    // `Follower` does anyway for its first few beats.
                    return self.tick(now);
                }
                // Nobody singing. If anyone is here and this duck has the lowest id, conduct.
                let alone = self.peers.is_empty();
                let lowest = self.peers.iter().all(|peer| self.id < peer.beacon.id);
                if !alone && lowest && now.saturating_duration_since(since) >= SETTLE {
                    let mut roster: Vec<(u8, u16)> = vec![(self.register, self.id)];
                    roster.extend(self.peers.iter().map(|p| (p.beacon.register, p.beacon.id)));
                    roster.truncate(proto::ChoraleBeacon::MAX_ROSTER);
                    // The conductor picks the piece — forced by the operator if they said so,
                    // otherwise from the clock's low bits at the moment the performance starts:
                    // as good as a coin for something that happens seconds after humans put
                    // ducks near each other, and deterministic under a test that controls the
                    // clock.
                    let pick = self.forced_piece.unwrap_or_else(|| {
                        PIECES[(self.seconds(now) * 997.0) as u64 as usize % PIECES.len()]
                    });
                    self.piece_id = pick;
                    self.score = piece(pick).expect("both built-in pieces exist");
                    tracing::warn!(voices = roster.len(), piece = pick, "chorale: conducting");
                    let roster = roster;
                    self.state = State::Conducting {
                        conductor: Conductor::new(self.score.bpm, self.seconds(now)),
                        roster,
                    };
                    return self.tick(now);
                }
                let idle = self.beacon(proto::ChoraleBeacon::IDLE, self.heartbeat(now), Vec::new());
                let advertise = self.publish(Some(idle), true);
                Tick {
                    advertise,
                    singing: None,
                    joining: false,
                    voices: self.peers.len(),
                }
            }
            State::Conducting { .. } => self.conduct(now),
            State::Following { .. } => self.follow(now),
        }
    }

    fn conduct(&mut self, now: Instant) -> Tick {
        // Everyone in range who is willing, in join order: the roster grows as ducks arrive and
        // never reorders, which is what keeps anyone already singing on their own part.
        let heard: Vec<(u8, u16)> = self
            .peers
            .iter()
            .map(|p| (p.beacon.register, p.beacon.id))
            .collect();
        let State::Conducting { conductor, roster } = &mut self.state else {
            unreachable!("checked by the caller");
        };
        for entry in heard {
            if roster.len() < proto::ChoraleBeacon::MAX_ROSTER && !roster.contains(&entry) {
                tracing::warn!(id = entry.1, "chorale: seating a new voice");
                roster.push(entry);
            }
        }
        let seconds = now.saturating_duration_since(self.started).as_secs_f64();
        conductor.due(seconds);
        let position = conductor.position_beats(seconds);
        let beat = conductor.wire_beat();
        let roster = roster.clone();

        // The piece ends. It used to not — position counted on past the last note forever,
        // ducks reporting "bar 48" of a piece eleven bars long, singing nothing, holding the
        // advertising instance. A finished performance goes back to listening (with the settle
        // timer, so it is a breath, not a gap), and the next one picks a piece afresh.
        let total = self.score.duration_s() / self.score.beat_s();
        if position > total + 2.0 {
            tracing::warn!(piece = self.piece_id, "chorale: the performance ended");
            self.state = State::Listening { since: now };
            return self.tick(now);
        }

        let beacon = self.beacon(self.piece_id, beat, roster.clone());
        let advertise = self.publish(Some(beacon), true);
        Tick {
            advertise,
            singing: self.my_part(&roster).map(|part| (part, position)),
            joining: false,
            voices: self.voices(&roster),
        }
    }

    /// Roster entries that are still audible: this duck, plus peers heard recently enough.
    ///
    /// The roster keeps a departed duck's seat so nobody else is reseated; this is the count that
    /// says how many are actually singing.
    fn voices(&self, roster: &[(u8, u16)]) -> usize {
        roster
            .iter()
            .filter(|(register, id)| {
                (*register == self.register && *id == self.id)
                    || self.peers.iter().any(|peer| peer.beacon.id == *id)
            })
            .count()
    }

    fn follow(&mut self, now: Instant) -> Tick {
        let State::Following {
            follower, roster, ..
        } = &mut self.state
        else {
            unreachable!("checked by the caller");
        };
        let roster = roster.clone();
        let seconds = now.saturating_duration_since(self.started).as_secs_f64();
        let position = follower.position_beats(seconds);
        // The follower's own end-of-piece, a few beats later than the conductor's so it is the
        // conductor's restart that leads when both are healthy — this path is for a follower
        // whose conductor vanished mid-final-chord.
        let total = self.score.duration_s() / self.score.beat_s();
        if position.is_some_and(|beats| beats > total + 6.0) {
            tracing::warn!(piece = self.piece_id, "chorale: the performance ended");
            self.state = State::Listening { since: now };
            return self.tick(now);
        }
        // A follower advertises an idle beacon: it is willing and findable, but it is not the one
        // holding the beat, and two beacons carrying a piece would be two conductors.
        let idle = self.beacon(proto::ChoraleBeacon::IDLE, self.heartbeat(now), Vec::new());
        let advertise = self.publish(Some(idle), true);
        let singing = self.my_part(&roster).zip(position);
        Tick {
            advertise,
            joining: singing.is_none(),
            singing,
            voices: self.voices(&roster),
        }
    }

    /// This duck's part, from the roster the conductor published.
    ///
    /// `None` when it is not in the roster yet — which is not an error but the ordinary state of a
    /// duck that has just arrived. It keeps listening, and the conductor seats it on the next beat.
    fn my_part(&self, roster: &[(u8, u16)]) -> Option<Part> {
        let seat = roster
            .iter()
            .position(|(register, id)| *register == self.register && *id == self.id)?;
        let registers: Vec<f64> = roster
            .iter()
            .map(|(register, _)| {
                proto::ChoraleBeacon::REGISTER_LOW_HZ
                    + f64::from(*register) * proto::ChoraleBeacon::REGISTER_STEP_HZ
            })
            .collect();
        seat_all(&registers).get(seat).copied()
    }

    fn beacon(&self, piece: u8, beat: u8, roster: Vec<(u8, u16)>) -> proto::ChoraleBeacon {
        proto::ChoraleBeacon {
            piece,
            beat,
            register: self.register,
            id: self.id,
            roster,
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
        Chorale::new(214.4, 7, None)
    }

    /// A duck that hears nobody must not sing. A solo chorale is a duck quacking to itself.
    #[test]
    fn one_duck_alone_does_not_sing() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        for step in 0..200 {
            let tick = c.tick(now + Duration::from_millis(50 * step));
            assert_eq!(tick.singing, None, "step {step}");
        }
        // But it is on the air, saying it is willing, and listening for company.
        let advertise = {
            let mut c = chorale();
            c.set_active(true, now, None);
            c.tick(now).advertise.expect("something to advertise")
        };
        assert!(advertise.listening);
        let beacon = advertise.beacon.expect("an idle beacon");
        assert!(!beacon.singing());
        assert!(beacon.roster.is_empty());
    }

    /// Off means off: nothing on the air, nothing heard, nothing sung. This is what
    /// `[chorale] accept = false` buys — invisible rather than visibly declining.
    #[test]
    fn a_duck_that_was_not_asked_is_silent_and_invisible() {
        let mut c = chorale();
        let now = Instant::now();
        let tick = c.tick(now);
        assert_eq!(tick.singing, None);
        // Nothing to *say*, because nothing has to change: `btd` starts out advertising nothing and
        // scanning for nothing, which is already what an unasked duck wants.
        assert_eq!(tick.advertise, None);

        // A beacon arriving is ignored rather than answered.
        c.heard(&heard_from(2, 120, 1, 5, vec![(120, 2)]), now);
        assert_eq!(c.tick(now).singing, None);
        assert!(!c.active());
    }

    fn heard_from(
        id: u16,
        register: u8,
        piece: u8,
        beat: u8,
        roster: Vec<(u8, u16)>,
    ) -> proto::ChoraleHeard {
        proto::ChoraleHeard {
            beacon: proto::ChoraleBeacon {
                piece,
                beat,
                register,
                id,
                roster,
            },
            from: format!("AA:BB:CC:DD:{id:04X}"),
            age_us: 2_000,
        }
    }

    /// Two willing ducks agree who conducts with no election and no message: the lower id does. Both
    /// ducks reach the same answer from the same beacons, which is why there is nothing to lose.
    #[test]
    fn the_lower_id_conducts_and_the_other_follows() {
        let now = Instant::now();
        // This duck's id, from seed 7.
        let mine = chorale().id;
        let higher = mine.wrapping_add(1);
        let lower = mine.wrapping_sub(1);

        let mut leads = chorale();
        leads.set_active(true, now, None);
        leads.heard(
            &heard_from(higher, 200, proto::ChoraleBeacon::IDLE, 0, vec![]),
            now,
        );
        let tick = leads.tick(now + SETTLE);
        assert!(
            tick.advertise
                .as_ref()
                .and_then(|a| a.beacon.as_ref())
                .is_some_and(|b| b.singing()),
            "the lower id should be conducting: {tick:?}"
        );
        assert_eq!(tick.voices, 2);

        let mut defers = chorale();
        defers.set_active(true, now, None);
        defers.heard(
            &heard_from(lower, 200, proto::ChoraleBeacon::IDLE, 0, vec![]),
            now,
        );
        let tick = defers.tick(now + SETTLE);
        assert!(
            tick.advertise
                .as_ref()
                .and_then(|a| a.beacon.as_ref())
                .is_none_or(|b| !b.singing()),
            "the higher id must not also conduct: {tick:?}"
        );
    }

    /// The bug seen on four real ducks: the conductor's `robotctl chorale` is restarted (which
    /// on a bench happens constantly), it re-picks a *different* piece — and its follower,
    /// never re-reading the piece byte, kept singing the old song against the new one. A
    /// restart must be treated as a new performance: new score, new phase lock, new roster.
    #[test]
    fn a_conductor_restarting_with_a_new_piece_takes_its_followers_with_it() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        // A performance of piece 1, followed properly: beats 10..15 from rotating addresses.
        for beat in 10..15u8 {
            c.heard(
                &heard_from(9, 120, PIECE_WISTFUL, beat, vec![(120, 9), mine]),
                now + Duration::from_secs(u64::from(beat - 10)),
            );
            c.tick(now + Duration::from_secs(u64::from(beat - 10)));
        }
        assert_eq!(c.score().name, "wistful");
        assert!(c.tick(now + Duration::from_secs(5)).singing.is_some());

        // The conductor restarts: same id, piece 2, beat counter back at zero.
        let later = now + Duration::from_secs(20);
        for beat in 0..6u8 {
            let at = later + Duration::from_millis(u64::from(beat) * 476); // duck strut's beat
            c.heard(&heard_from(9, 120, 2, beat, vec![(120, 9), mine]), at);
            c.tick(at);
        }
        assert_eq!(
            c.score().name,
            "duck-strut",
            "the follower moved to the new performance"
        );
        let tick = c.tick(later + Duration::from_secs(3));
        assert!(
            tick.singing.is_some(),
            "and locked onto its new beat: {tick:?}"
        );
    }

    /// The same restart with the *same* piece still resets the phase lock: the beat counter
    /// went backwards, and feeding that into the old fit poisons it into free-running.
    #[test]
    fn a_beat_counter_going_backwards_is_a_new_performance() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        for beat in 40..45u8 {
            let at = now + Duration::from_secs(u64::from(beat - 40));
            c.heard(
                &heard_from(9, 120, PIECE_WISTFUL, beat, vec![(120, 9), mine]),
                at,
            );
            c.tick(at);
        }
        let before = c
            .tick(now + Duration::from_secs(5))
            .singing
            .expect("singing")
            .1;
        assert!(before > 30.0, "well into the piece: {before}");

        // Restart at beat zero. The position must come back to the start, not free-run on.
        let later = now + Duration::from_secs(30);
        for beat in 0..6u8 {
            let at = later + Duration::from_secs(u64::from(beat));
            c.heard(
                &heard_from(9, 120, PIECE_WISTFUL, beat, vec![(120, 9), mine]),
                at,
            );
            c.tick(at);
        }
        let after = c
            .tick(later + Duration::from_secs(6))
            .singing
            .expect("singing")
            .1;
        assert!(after < 15.0, "the position followed the restart: {after}");
    }

    /// `DUCK_CHORALE_PIECE` pins the conductor's pick; an id this build lacks is ignored with
    /// a warning rather than obeyed into silence.
    #[test]
    fn a_forced_piece_wins_the_coin_toss() {
        let now = Instant::now();
        let mut c = Chorale::new(214.4, 7, Some(3));
        c.set_active(true, now, None);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let beacon = c
            .tick(now + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert_eq!(beacon.piece, 3);
        assert_eq!(c.score().name, "outer-wilds");

        // An unknown forced id falls back to the coin rather than wedging the chorale.
        let mut c = Chorale::new(214.4, 7, Some(200));
        c.set_active(true, now, None);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let beacon = c
            .tick(now + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert!(piece(beacon.piece).is_some(), "picked {}", beacon.piece);
    }

    /// A performance ends. It used to not: position counted past the last note forever, ducks
    /// reporting bar 48 of an eleven-bar piece, singing nothing, holding the advertising
    /// instance. The conductor goes back to listening after the piece plus a breath — and
    /// because listening re-settles and re-picks, the ducks then sing something else.
    #[test]
    fn a_performance_ends_and_the_ducks_go_back_to_listening() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let tick = c.tick(now + SETTLE);
        assert!(
            tick.advertise
                .and_then(|a| a.beacon)
                .is_some_and(|b| b.singing()),
            "conducting"
        );
        let total_beats = c.score().duration_s() / c.score().beat_s();
        let done = now + SETTLE + Duration::from_secs_f64((total_beats + 5.0) * c.score().beat_s());
        let tick = c.tick(done);
        assert_eq!(tick.singing, None, "the piece is over: {tick:?}");
        let beacon = tick.advertise.and_then(|a| a.beacon);
        assert!(
            beacon.is_none_or(|b| !b.singing()),
            "back to an idle beacon after the last note"
        );

        // A follower whose conductor vanished mid-final-chord ends its own performance too,
        // a few beats later, instead of free-running forever.
        let mut f = chorale();
        f.set_active(true, now, None);
        let mine = (f.register, f.id);
        for beat in 0..6u8 {
            let at = now + Duration::from_secs(u64::from(beat));
            f.heard(
                &heard_from(9, 120, PIECE_WISTFUL, beat, vec![(120, 9), mine]),
                at,
            );
            f.tick(at);
        }
        assert!(f.tick(now + Duration::from_secs(6)).singing.is_some());
        let total = f.score().duration_s() / f.score().beat_s();
        let after = now + Duration::from_secs_f64((total + 10.0) * f.score().beat_s());
        assert_eq!(f.tick(after).singing, None, "the follower let go too");
    }

    /// The collision that hid a real duck: with byte-sized ids, the fourth duck rolled the
    /// conductor's byte, dropped every conductor beacon as its own reflection, and could never
    /// join. Ids are sixteen bits now, and a same-register different-id peer is a peer.
    #[test]
    fn a_duck_sharing_a_register_is_still_a_different_duck() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        // A conductor with this duck's register and an id differing only above the old byte.
        let near_id = c.id ^ 0x0100;
        assert_ne!(near_id, c.id);
        for beat in 0..6u8 {
            let at = now + Duration::from_secs(u64::from(beat));
            c.heard(
                &heard_from(
                    near_id,
                    c.register,
                    PIECE_WISTFUL,
                    beat,
                    vec![(c.register, near_id), mine],
                ),
                at,
            );
            c.tick(at);
        }
        assert!(
            c.tick(now + Duration::from_secs(6)).singing.is_some(),
            "an id one bit apart in the high byte is somebody else"
        );
        // Its own true reflection is still dropped.
        let before = c.tick(now + Duration::from_secs(6)).voices;
        c.heard(
            &heard_from(c.id, c.register, proto::ChoraleBeacon::IDLE, 0, vec![]),
            now + Duration::from_secs(6),
        );
        assert_eq!(c.tick(now + Duration::from_secs(6)).voices, before);
    }

    /// The `--piece` pin arrives with the activation and wins the coin; a plain activation
    /// afterwards goes back to the coin rather than staying pinned forever.
    #[test]
    fn a_piece_pinned_at_activation_wins_and_expires() {
        let now = Instant::now();
        let mut c = Chorale::new(214.4, 7, None);
        c.set_active(true, now, Some(PIECE_DUCK_STRUT));
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let beacon = c
            .tick(now + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert_eq!(beacon.piece, PIECE_DUCK_STRUT);

        // Deactivate, reactivate plain: the pin is gone and the coin is back (whatever it
        // lands on, it is a real piece). The tick between flushes the beacon-down to btd, so
        // the next performance's beacon registers as a change.
        c.set_active(false, now, None);
        c.tick(now + Duration::from_secs(8));
        c.set_active(true, now + Duration::from_secs(9), None);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now + Duration::from_secs(9),
        );
        let beacon = c
            .tick(now + Duration::from_secs(9) + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert!(piece(beacon.piece).is_some());
    }

    /// The head expression is a function of the shared beat alone, so every duck computes the
    /// same sway — group choreography with no coordination. And it stays small: the head
    /// carries a sensor, and the policy balances the mass up there.
    #[test]
    fn the_head_sways_in_phase_and_stays_small() {
        for step in 0..200 {
            let beats = f64::from(step) * 0.13;
            let a = head_expression(beats, 0.3);
            let b = head_expression(beats, 0.3);
            assert_eq!(a, b, "same beat, same sway, on every duck");
            for (joint, offset) in a.iter().enumerate() {
                assert!(offset.abs() <= 0.15, "joint {joint} at {offset} rad");
            }
        }
        // The sway actually moves, and the high note actually lifts.
        assert_ne!(head_expression(0.0, 0.0), head_expression(1.0, 0.0));
        let low = head_expression(2.0, 0.0)[1];
        let high = head_expression(2.0, 1.0)[1];
        assert!(
            (high - low).abs() > 0.05,
            "reach must be visible: {low} vs {high}"
        );
    }

    /// The beacon's piece byte decides the song: a duck joining a duck-strut performance loads
    /// duck strut, not whatever it had loaded before — bpm and all, or the phase lock would be
    /// counting the wrong beat length.
    #[test]
    fn a_joiner_sings_the_piece_the_beacon_names() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        c.heard(&heard_from(9, 120, 2, 0, vec![(120, 9), mine]), now);
        let _ = c.tick(now);
        assert_eq!(c.score().name, "duck-strut", "loaded from the beacon's id");
        assert!((c.score().bpm - 126.0).abs() < 0.5, "and its tempo with it");
    }

    /// A piece this build does not know is not joined and not guessed at: the duck keeps
    /// listening, which is the right shape for a mixed-version flock — an old duck near newer
    /// ones stays politely quiet instead of performing a different song at them.
    #[test]
    fn an_unknown_piece_is_declined_not_guessed() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        c.heard(&heard_from(9, 120, 200, 4, vec![(120, 9)]), now);
        for step in 0..50 {
            let tick = c.tick(now + Duration::from_millis(100 * step));
            assert_eq!(tick.singing, None, "step {step}");
            // And it does not start a rival performance in the same room either: someone is
            // singing, even if we cannot join them.
            if let Some(beacon) = tick.advertise.and_then(|a| a.beacon) {
                assert!(!beacon.singing(), "conducting over an ongoing piece");
            }
        }
    }

    /// The conductor names its pick on the air, and the pick is one of the pieces that exist.
    #[test]
    fn the_conductor_picks_a_real_piece_and_broadcasts_it() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let beacon = c
            .tick(now + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert!(piece(beacon.piece).is_some(), "picked {}", beacon.piece);
        assert!(!c.score().name.is_empty());
        assert!(
            (c.score().bpm - piece(beacon.piece).expect("exists").bpm).abs() < 1e-9,
            "the loaded score is the broadcast one"
        );
    }

    /// THE regression test for this feature's worst bug, present twice: the conductor's radio
    /// address rotates every few seconds (BLE privacy, and re-registering an advertisement
    /// rotates it too), so a follower keyed on the address adopted the conductor once and then
    /// rejected every beat it ever sent again — one observation, no lock, no singing, while the
    /// conductor happily counted two voices. Identity must come from the beacon.
    #[test]
    fn the_conductor_is_followed_across_its_rotating_addresses() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = (c.register, c.id);
        let roster = vec![(120u8, 9u16), mine];

        // Six beats, each from a brand-new address, one per second — exactly what the radio
        // does — with the loop ticking in between, as it does on the robot. The beat counter
        // is what says the beacons are the same conductor; the addresses say otherwise.
        let mut tick = c.tick(now);
        for beat in 0..6u8 {
            let at = now + Duration::from_secs(u64::from(beat));
            let heard = proto::ChoraleHeard {
                beacon: proto::ChoraleBeacon {
                    piece: PIECE_WISTFUL,
                    beat,
                    register: 120,
                    id: 9,
                    roster: roster.clone(),
                },
                from: format!("{beat:02X}:AA:BB:CC:DD:EE"),
                age_us: 2_000,
            };
            c.heard(&heard, at);
            tick = c.tick(at);
        }
        let _ = tick;
        let tick = c.tick(now + Duration::from_secs(6));
        let (part, position) = tick.singing.expect("locked and seated, so singing");
        // Register bytes decode near 234 Hz (this duck, 65) and 347 Hz (the conductor, 120):
        // this duck is the low voice, so it takes the bass under the conductor's soprano.
        assert_eq!(part, Part::Bass);
        assert!(position > 0.0, "{position}");
        assert_eq!(tick.voices, 2, "{tick:?}");
    }

    /// An idle beacon must change on its own, or nothing re-registers the advertisement and a
    /// waiting duck is only noticed when the radio happens to rotate its address — tens of
    /// seconds, measured. The heartbeat is that change.
    #[test]
    fn an_idle_beacon_has_a_heartbeat() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let first = c
            .tick(now)
            .advertise
            .and_then(|a| a.beacon)
            .expect("an idle beacon goes out");
        // Within one heartbeat: no change, nothing re-sent.
        assert_eq!(c.tick(now + Duration::from_millis(300)).advertise, None);
        // Past it: the beacon differs, so btd re-registers and the duck is re-noticed.
        let later = c
            .tick(now + IDLE_HEARTBEAT + Duration::from_millis(100))
            .advertise
            .and_then(|a| a.beacon)
            .expect("the heartbeat re-advertises");
        assert_ne!(first, later);
        assert!(!later.singing(), "still idle, only different");
    }

    /// A duck hearing a piece already under way joins it rather than starting a second one — which
    /// is the whole of "and other ducks can then join".
    #[test]
    fn a_duck_arriving_late_joins_what_it_finds() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        // A conductor already singing, with this duck already seated in its roster.
        let mine = (c.register, c.id);
        let roster = vec![(120u8, 9u16), mine];
        c.heard(&heard_from(9, 120, PIECE_WISTFUL, 4, roster.clone()), now);
        let tick = c.tick(now);
        assert_eq!(tick.voices, 2, "{tick:?}");
        // It does not conduct — there is already a conductor — and its own beacon stays idle.
        let beacon = tick
            .advertise
            .as_ref()
            .and_then(|a| a.beacon.as_ref())
            .expect("still findable");
        assert!(!beacon.singing(), "two conductors: {beacon:?}");
    }

    /// The conductor seats a newcomer without moving anyone: the roster grows and never reorders.
    #[test]
    fn the_roster_grows_and_never_reorders() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        let mine = c.id;
        // Two ducks with higher ids, so this one conducts.
        c.heard(
            &heard_from(
                mine.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        let first = c
            .tick(now + SETTLE)
            .advertise
            .and_then(|a| a.beacon)
            .expect("conducting");
        assert_eq!(first.roster.len(), 2);

        c.heard(
            &heard_from(
                mine.wrapping_add(2),
                60,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now + SETTLE,
        );
        let second = c
            .tick(now + SETTLE + Duration::from_millis(20))
            .advertise
            .and_then(|a| a.beacon)
            .expect("still conducting");
        assert_eq!(second.roster.len(), 3);
        assert_eq!(
            &second.roster[..2],
            &first.roster[..],
            "the newcomer went on the end"
        );
    }

    /// A duck that walks out of range stops being in the piece, or the roster names someone who is
    /// not singing and a part goes missing.
    #[test]
    fn a_duck_that_leaves_stops_being_counted() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        assert_eq!(c.tick(now + SETTLE).voices, 2);
        // Long enough that its last beacon is stale.
        let later = now + SETTLE + PEER_STALE + Duration::from_millis(100);
        let tick = c.tick(later);
        assert_eq!(tick.voices, 1, "{tick:?}");
        // But its seat survives, so this duck is still singing the part it started on rather than
        // being reseated mid-piece.
        let beacon = tick
            .advertise
            .and_then(|a| a.beacon)
            .expect("still conducting");
        assert_eq!(beacon.roster.len(), 2, "the seat is kept: {beacon:?}");
        // This duck's register is the lower of the two, so it has the bass — and keeps it after the
        // other one leaves rather than being reseated onto the line nobody is singing.
        assert_eq!(tick.singing.expect("still singing").0, Part::Bass);
    }

    /// The beacon is resent only when it changes. The loop runs at 50 Hz and the beat turns over
    /// about once a second; the difference is two D-Bus round trips a second against a hundred.
    #[test]
    fn the_beacon_is_only_published_when_it_changes() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        assert!(
            c.tick(now).advertise.is_some(),
            "the first one has to go out"
        );
        for step in 1..20 {
            assert_eq!(
                c.tick(now + Duration::from_millis(10 * step)).advertise,
                None,
                "nothing changed at step {step}"
            );
        }
        // And a peer arriving *does* change it, eventually — the roster is in the beacon.
        c.heard(
            &heard_from(
                c.id.wrapping_add(1),
                250,
                proto::ChoraleBeacon::IDLE,
                0,
                vec![],
            ),
            now,
        );
        assert!(c.tick(now + SETTLE).advertise.is_some());
    }

    /// A duck not yet in the conductor's roster does not sing — it has no part, and guessing one is
    /// how two ducks end up on the same line.
    #[test]
    fn a_duck_not_yet_seated_waits_rather_than_guessing() {
        let mut c = chorale();
        let now = Instant::now();
        c.set_active(true, now, None);
        // A piece under way whose roster does not mention this duck.
        c.heard(
            &heard_from(9, 120, PIECE_WISTFUL, 4, vec![(120, 9), (250, 11)]),
            now,
        );
        let tick = c.tick(now);
        assert_eq!(tick.singing, None, "no seat, no part: {tick:?}");
    }
}
