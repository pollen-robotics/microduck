//! The vet book: the robot's lifetime record, kept by the loop that sees everything it counts.
//!
//! What it holds, and why the wire answer and the file are one struct, is
//! [`duck_ipc_proto::vetbook`]. This is how it is kept.
//!
//! **The loop never waits on the book.** Each tick adds to a [`Tracker`] the loop owns outright —
//! a few dozen additions, no lock, no allocation. Once a second the loop hands what it has
//! counted to the shared [`Book`] with a `try_lock`; if a reader or the saver holds it, the
//! counts stay in the tracker and go with the next handoff. Nothing is lost and nothing blocks.
//!
//! **Saved once a minute and on a clean stop**, atomically: temp file, `fsync`, rename, `fsync`
//! the directory — the update log's discipline (`architecture.md` §8.2). A duck switched off at
//! the battery loses at most the last minute, and never the book. Once a minute is ~1 500 small
//! writes a day, which an eMMC does not notice.
//!
//! Lives at `/var/lib/robot/vetbook.json`, outside every release directory, so an update and a
//! rollback both keep it; a reflash starts a fresh one. `--fake` and `--sim` keep the book in
//! memory unless `--vetbook` names a file, so a laptop running the twin writes nothing to
//! `/var/lib`.

use std::path::{Path, PathBuf};
use std::sync::Mutex;

use duck_control::model::{JOINT_IDS, NUM_JOINTS};
use duck_ipc_proto::vetbook::{
    HOT_C, Identity, Life, Vetbook, VetbookTallyParams, Wear, key, merge_maps,
};

/// Where a robot keeps its book.
pub const DEFAULT_PATH: &str = "/var/lib/robot/vetbook.json";

/// How often the loop reads the servos' latched alerts — see [`Tracker::alerts`].
pub const ALERT_INTERVAL: std::time::Duration = std::time::Duration::from_secs(10);

/// How often the book is written to disk.
pub const SAVE_INTERVAL: std::time::Duration = std::time::Duration::from_secs(60);

/// A first reading this far above the last saved one means the pack was charged in between. The
/// smoothed voltage of a resting pack recovers by a few tenths after a walk; a charge moves it by
/// the better part of a volt.
const CHARGE_JUMP_V: f64 = 0.4;

/// Below these, a commanded twist is standing still: metres per second across the ground, and
/// radians per second about the vertical. The applied command is smoothed, so a released stick
/// decays towards zero rather than reaching it — a threshold is what ends the walk.
const WALK_SPEED: f64 = 0.02;
const WALK_TURN: f64 = 0.1;

/// Unix seconds now, or zero on a clock before 1970 — which a board with no RTC can have.
pub fn now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// The book, shared between the loop, the IPC side and the saver.
pub struct Book {
    path: Option<PathBuf>,
    inner: Mutex<Vetbook>,
}

impl Book {
    /// Open the book at `path`, or start one, and write down that `robotd` started — and, when it
    /// is so, that the board booted and that the release changed.
    ///
    /// A file that is there and will not parse is **set aside, never overwritten**: it is moved to
    /// `vetbook.json.unreadable-<time>` and a fresh book starts, so a bug in a future reader costs
    /// a gap in the record rather than the record.
    pub fn open(path: Option<PathBuf>, version: &str) -> Self {
        let book = Self::load(path);
        book.started(version);
        book.save();
        book
    }

    /// The book at `path` as it is, or a new one — without counting a start. For `robotd init`,
    /// which is not the daemon starting but may adopt a servo.
    pub fn load(path: Option<PathBuf>) -> Self {
        let at = now();
        let loaded = path.as_deref().and_then(|p| load(p, at));
        let mut book = loaded.unwrap_or_else(|| Vetbook::open(at));
        book.repair();
        Self {
            path,
            inner: Mutex::new(book),
        }
    }

    fn started(&self, version: &str) {
        let at = now();
        let mut book = self.lock();
        book.life.starts += 1;
        let boot_id = std::fs::read_to_string("/proc/sys/kernel/random/boot_id")
            .map(|s| s.trim().to_owned())
            .unwrap_or_default();
        if boot_id.is_empty() || boot_id != book.last_boot_id {
            book.life.boots += 1;
            book.last_boot_id = boot_id;
        }
        if !book.last_version.is_empty() && book.last_version != version {
            book.life.updates += 1;
            let text = format!("updated from {} to {version}", book.last_version);
            book.event(at, "update", text);
        }
        book.last_version = version.to_owned();
    }

    /// A copy of the book as it stands.
    pub fn snapshot(&self) -> Vetbook {
        self.lock().clone()
    }

    /// `robot.vetbookTally`: another daemon's counts and records, merged in. Not saved at once:
    /// they arrive every minute or so, and the next save is never more than one away.
    pub fn tally(&self, p: &VetbookTallyParams) {
        let mut book = self.lock();
        let life = &mut book.life;
        merge_maps(&mut life.tallies, &mut life.highs, &mut life.lows, p);
    }

    /// Add `n` to one tally, from the IPC side — a gait loaded, say.
    pub fn bump(&self, key: &str, n: f64) {
        let mut p = VetbookTallyParams::default();
        p.tallies.insert(key.to_owned(), n);
        self.tally(&p);
    }

    /// `robot.vetbookNote`. Saved at once: whoever wrote it may well switch the robot off next.
    pub fn note(&self, text: &str) -> Result<(), String> {
        let text = text.trim();
        if text.is_empty() {
            return Err("an empty note".into());
        }
        self.lock().event(now(), "note", text);
        self.save();
        Ok(())
    }

    /// A slot or skill started or stopped running a community policy — what
    /// `community_policy_events` in `main.rs` says. Left for the next save, like a tally: nobody
    /// switches a robot off because a gait changed.
    pub fn policy(&self, text: &str) {
        self.lock().event(now(), "policy", text);
    }

    /// `robotd` just adopted a factory-fresh servo into joint `id`: the old one is retired with
    /// what it did, and the new one starts from zero. Saved at once, for the same reason as a note.
    ///
    /// This is the only way a swap gets into the book. A servo that went in already configured
    /// with the joint's ID is invisible to the bus — there is no serial number to notice — so it
    /// carries on the old one's record.
    pub fn adopted(&self, id: u8) {
        if let Err(e) = self.lock().swap(id, now()) {
            tracing::warn!(error = %e, "vet book: cannot record the swap");
        }
        self.save();
    }

    /// What each servo says it is, read once a start.
    pub fn identify(&self, ids: &[duck_control::io::ServoIdentity; NUM_JOINTS]) {
        let mut book = self.lock();
        for (joint, ident) in ids.iter().enumerate() {
            if let Some(m) = book.motors.iter_mut().find(|m| m.id == JOINT_IDS[joint]) {
                m.identity = Some(Identity {
                    model: ident.model,
                    firmware: ident.firmware,
                });
            }
        }
    }

    /// Write the book out, if it has a file. Logged rather than returned: the caller is a timer
    /// or a shutdown, and neither has anything better to do with the error.
    pub fn save(&self) {
        let Some(path) = self.path.as_deref() else {
            return;
        };
        let json = match serde_json::to_vec_pretty(&*self.lock()) {
            Ok(json) => json,
            Err(e) => return tracing::warn!(error = %e, "vet book: cannot serialise"),
        };
        if let Err(e) = write_atomically(path, &json) {
            tracing::warn!(path = %path.display(), error = %e, "vet book: cannot save");
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Vetbook> {
        // A panic while holding the book leaves counts, not invariants: keep going with them.
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn try_lock(&self) -> Option<std::sync::MutexGuard<'_, Vetbook>> {
        match self.inner.try_lock() {
            Ok(guard) => Some(guard),
            Err(std::sync::TryLockError::Poisoned(e)) => Some(e.into_inner()),
            Err(std::sync::TryLockError::WouldBlock) => None,
        }
    }
}

fn load(path: &Path, at: u64) -> Option<Vetbook> {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            tracing::warn!(path = %path.display(), "vet book: none yet, opening a new one");
            return None;
        }
        Err(e) => {
            tracing::warn!(path = %path.display(), error = %e, "vet book: cannot read; starting a new one");
            return None;
        }
    };
    match serde_json::from_slice(&bytes) {
        Ok(book) => Some(book),
        Err(e) => {
            let aside = path.with_extension(format!("json.unreadable-{at}"));
            let moved = std::fs::rename(path, &aside);
            tracing::error!(
                path = %path.display(),
                error = %e,
                set_aside = %aside.display(),
                moved = moved.is_ok(),
                "vet book: unreadable; set aside and starting a new one"
            );
            None
        }
    }
}

fn write_atomically(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let dir = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir)?;
    let tmp = path.with_extension("json.tmp");
    {
        let mut file = std::fs::File::create(&tmp)?;
        file.write_all(bytes)?;
        file.sync_all()?;
    }
    std::fs::rename(&tmp, path)?;
    std::fs::File::open(dir)?.sync_all()
}

/// What the loop knows about one tick.
pub struct Facts<'a> {
    pub dt: f64,
    /// Torque on: brought up and not since let go.
    pub torqued: bool,
    /// Which net drove, as `robot.state` labels it.
    pub label: &'a str,
    /// The twist the gait was handed this tick, after smoothing and safety: `[vx, vy, wz]`.
    pub twist: [f64; 3],
    pub in_hands: bool,
    pub fallen: bool,
    pub limp_falling: bool,
    pub theremin: bool,
    pub singing: bool,
    /// Odometry: position, and yaw.
    pub odom: ([f64; 3], f64),
    /// This tick's sample, when the read did not drop.
    pub fresh: Option<&'a duck_control::io::Sensors>,
    pub overran: bool,
    /// The flashlight lit.
    pub flashlight: bool,
    /// How far the beak is open, 0 to 1, whoever is opening it.
    pub mouth: f64,
    /// Sounds the voice has played since this process started, when there is a voice.
    pub sounds_played: Option<u64>,
}

/// The robot is upside down past this tilt, degrees from upright.
const UPSIDE_DOWN_DEG: f64 = 120.0;

/// A tilt counts as survived once this many ticks pass after it with no fall — two windows of it,
/// so a tilt at the very end of one window still has a whole window to be disowned by a fall.
const TILT_WINDOW_TICKS: u32 = 100;

/// Personal records and counters gathered between handoffs, typed here and named on the way out
/// ([`key`]) so the tick touches no map and no string.
#[derive(Default)]
struct Records {
    upside_down_s: f64,
    flashlight_s: f64,
    flashlight_on: f64,
    sounds: f64,
    beak_flaps: f64,
    throttled_s: f64,
    spin_dps: f64,
    top_speed_mps: f64,
    walk_streak_m: f64,
    tilt_survived_deg: f64,
    peak_current_a: f64,
    joint_speed_dps: f64,
}

impl Records {
    fn take(&mut self) -> VetbookTallyParams {
        let r = std::mem::take(self);
        let mut p = VetbookTallyParams::default();
        for (k, v) in [
            (key::UPSIDE_DOWN_S, r.upside_down_s),
            (key::FLASHLIGHT_S, r.flashlight_s),
            (key::FLASHLIGHT_ON, r.flashlight_on),
            (key::SOUNDS, r.sounds),
            (key::BEAK_FLAPS, r.beak_flaps),
            (key::THROTTLED_S, r.throttled_s),
        ] {
            if v > 0.0 {
                p.tallies.insert(k.to_owned(), v);
            }
        }
        for (k, v) in [
            (key::SPIN_DPS, r.spin_dps),
            (key::TOP_SPEED_MPS, r.top_speed_mps),
            (key::WALK_STREAK_M, r.walk_streak_m),
            (key::TILT_SURVIVED_DEG, r.tilt_survived_deg),
            (key::PEAK_CURRENT_A, r.peak_current_a),
            (key::JOINT_SPEED_DPS, r.joint_speed_dps),
        ] {
            if v > 0.0 {
                p.highs.insert(k.to_owned(), v);
            }
        }
        p
    }
}

/// Labels that are not one-shot skills. Anything else a tick reports is a skill's own name.
fn is_skill(label: &str) -> bool {
    !matches!(
        label,
        "walk" | "stand" | "ground_pick" | "sit" | "rise" | "held" | "homing" | "rest" | "skill"
    ) && !label.starts_with("limp")
        && !label.contains(':')
}

/// The loop's own tally since the last handoff, and what it needs to see edges.
#[derive(Default)]
pub struct Tracker {
    life: Life,
    wear: [Wear; NUM_JOINTS],
    label: String,
    torqued: bool,
    fallen: bool,
    in_hands: bool,
    limp_falling: bool,
    singing: bool,
    positions: Option<[f64; NUM_JOINTS]>,
    odom: Option<([f64; 3], f64)>,
    /// Whether this process has had its first battery reading yet — the one a charge is judged on.
    volts_seen: bool,
    /// A charge spotted on the first reading, to be dated in the book at the next handoff.
    charged: Option<(f64, f64)>,
    /// Each servo's alert bits at the last read, so a trip is counted as it comes up.
    alerts: [u8; NUM_JOINTS],
    rec: Records,
    flashlight: bool,
    beak_open: bool,
    sounds_seen: Option<u64>,
    /// Metres walked since the last fall, and in the current second.
    streak_m: f64,
    speed_window_m: f64,
    /// The steepest tilt in the current window, and in the one before it, waiting to be disowned
    /// by a fall before they count — see [`TILT_WINDOW_TICKS`].
    tilt_now: f64,
    tilt_before: f64,
    tilt_ticks: u32,
}

impl Tracker {
    pub fn tick(&mut self, f: &Facts) {
        let life = &mut self.life;
        let dt = f.dt;
        life.powered_s += dt;
        if f.torqued {
            life.torqued_s += dt;
            for wear in &mut self.wear {
                wear.torqued_s += dt;
            }
        }
        // **Walking is a velocity, not a network.** Velstand is one net that walks and stands,
        // and labels every tick `walk`; a robot with a separate stand net labels the still ones
        // `stand`. Either way, what separates the two is whether it was asked to move.
        let gait = matches!(f.label, "walk" | "stand");
        let walking =
            gait && (f.twist[0].hypot(f.twist[1]) > WALK_SPEED || f.twist[2].abs() > WALK_TURN);
        match f.label {
            _ if walking => life.walking_s += dt,
            _ if gait => life.standing_s += dt,
            "sit" | "rise" => life.sitting_s += dt,
            _ => {}
        }
        if f.in_hands {
            life.in_hands_s += dt;
        }
        if f.fallen {
            life.fallen_s += dt;
        }
        if f.theremin {
            life.theremin_s += dt;
        }
        if f.singing {
            life.singing_s += dt;
        }
        if f.flashlight {
            self.rec.flashlight_s += dt;
            if !self.flashlight {
                self.rec.flashlight_on += 1.0;
            }
        }
        self.flashlight = f.flashlight;
        // A flap is the beak opening past halfway; it has to close below a third to flap again.
        if !self.beak_open && f.mouth > 0.5 {
            self.beak_open = true;
            self.rec.beak_flaps += 1.0;
        } else if self.beak_open && f.mouth < 0.3 {
            self.beak_open = false;
        }
        if let Some(n) = f.sounds_played {
            if let Some(was) = self.sounds_seen {
                self.rec.sounds += n.saturating_sub(was) as f64;
            }
            self.sounds_seen = Some(n);
        }
        if f.overran {
            life.missed_ticks += 1;
        }

        // Edges.
        if f.label != self.label {
            match f.label {
                "ground_pick" => life.ground_picks += 1,
                "sit" => life.sit_downs += 1,
                "rise" => life.stand_ups += 1,
                skill if is_skill(skill) => *life.skills.entry(skill.to_owned()).or_default() += 1,
                _ => {}
            }
            self.label.clear();
            self.label.push_str(f.label);
        }
        if f.torqued && !self.torqued {
            life.wake_ups += 1;
        }
        self.torqued = f.torqued;
        if f.fallen && !self.fallen {
            life.falls += 1;
            // A fall ends the walk streak, and disowns every tilt still waiting to count.
            self.streak_m = 0.0;
            self.tilt_now = 0.0;
            self.tilt_before = 0.0;
            self.tilt_ticks = 0;
        }
        if f.in_hands && !self.in_hands {
            life.pickups += 1;
        }
        if f.limp_falling && !self.limp_falling {
            life.soft_landings += 1;
        }
        if f.singing && !self.singing {
            life.songs += 1;
        }
        self.fallen = f.fallen;
        self.in_hands = f.in_hands;
        self.limp_falling = f.limp_falling;
        self.singing = f.singing;

        // Ground covered: only while walking, so odometry's drift on a standing robot and a robot
        // carried across the room are not distance walked.
        let (pos, yaw) = f.odom;
        if let Some((was, was_yaw)) = self.odom
            && walking
        {
            let step = (pos[0] - was[0]).hypot(pos[1] - was[1]);
            life.distance_m += step;
            self.streak_m += step;
            self.speed_window_m += step;
            self.rec.walk_streak_m = self.rec.walk_streak_m.max(self.streak_m);
            let turn = (yaw - was_yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            life.turned_rad += turn.abs();
        }
        self.odom = Some(f.odom);

        match f.fresh {
            None => life.bus_drops += 1,
            Some(sensors) => {
                let rec = &mut self.rec;
                let g = sensors.imu.gravity;
                let tilt = (-g[2]).clamp(-1.0, 1.0).acos().to_degrees();
                if tilt > UPSIDE_DOWN_DEG {
                    rec.upside_down_s += dt;
                }
                let spin = sensors.imu.gyro.iter().map(|w| w * w).sum::<f64>().sqrt();
                rec.spin_dps = rec.spin_dps.max(spin.to_degrees());
                for joint in 0..NUM_JOINTS {
                    rec.peak_current_a = rec
                        .peak_current_a
                        .max(sensors.currents_ma[joint].abs() / 1000.0);
                    rec.joint_speed_dps = rec
                        .joint_speed_dps
                        .max(sensors.velocities[joint].abs().to_degrees());
                }
                // Only a tilt the gait was holding the robot through, on its own feet.
                if f.torqued && !f.in_hands && !f.fallen && matches!(f.label, "walk" | "stand") {
                    self.tilt_now = self.tilt_now.max(tilt);
                }
                self.tilt_ticks += 1;
                if self.tilt_ticks >= TILT_WINDOW_TICKS {
                    rec.tilt_survived_deg = rec.tilt_survived_deg.max(self.tilt_before);
                    self.tilt_before = self.tilt_now;
                    self.tilt_now = 0.0;
                    self.tilt_ticks = 0;
                }
                if let Some(was) = self.positions {
                    for (joint, wear) in self.wear.iter_mut().enumerate() {
                        wear.travel_rad += (sensors.positions[joint] - was[joint]).abs();
                    }
                }
                for (joint, wear) in self.wear.iter_mut().enumerate() {
                    wear.charge_as += sensors.currents_ma[joint].abs() / 1000.0 * dt;
                }
                self.positions = Some(sensors.positions);
            }
        }
    }

    /// The once-a-second sample: per-servo temperature, the pack, the board.
    pub fn slow(
        &mut self,
        temps_c: Option<&[f64; NUM_JOINTS]>,
        volts: f64,
        cpu_c: f64,
        throttled: bool,
        dt: f64,
    ) {
        if throttled {
            self.rec.throttled_s += dt;
        }
        if dt > 0.0 {
            self.rec.top_speed_mps = self.rec.top_speed_mps.max(self.speed_window_m / dt);
        }
        self.speed_window_m = 0.0;
        let life = &mut self.life;
        if let Some(temps) = temps_c {
            for (wear, &t) in self.wear.iter_mut().zip(temps) {
                wear.hottest_c = wear.hottest_c.max(t);
                if t >= HOT_C {
                    wear.hot_s += dt;
                }
                life.hottest_motor_c = life.hottest_motor_c.max(t);
            }
        }
        if cpu_c > 0.0 {
            life.hottest_cpu_c = life.hottest_cpu_c.max(cpu_c);
        }
        if volts > 0.0 {
            life.lowest_volts = if life.lowest_volts > 0.0 {
                life.lowest_volts.min(volts)
            } else {
                volts
            };
            life.highest_volts = life.highest_volts.max(volts);
        }
    }

    /// The servos' latched alert bits — see [`duck_control::io::RobotIo::hardware_errors`].
    pub fn alerts(&mut self, now: &[u8; NUM_JOINTS]) {
        for (joint, wear) in self.wear.iter_mut().enumerate() {
            wear.trips.count(self.alerts[joint], now[joint]);
        }
        self.alerts = *now;
    }

    /// The first battery reading of this process, against the last one the book saved.
    fn judge_charge(&mut self, last_volts: f64, volts: f64) {
        if self.volts_seen || volts <= 0.0 {
            return;
        }
        self.volts_seen = true;
        if last_volts > 0.0 && volts > last_volts + CHARGE_JUMP_V {
            self.life.charges += 1;
            self.charged = Some((last_volts, volts));
        }
    }

    pub fn pet(&mut self) {
        self.life.pets += 1;
    }

    pub fn motor_reboot(&mut self) {
        self.life.motor_reboots += 1;
    }

    pub fn power_off(&mut self) {
        self.life.power_offs += 1;
    }

    /// Give the book what has been counted since the last handoff. With `wait` false — every
    /// handoff but the last — a book somebody else is holding is left for next time, and the
    /// counts stay here.
    pub fn handoff(&mut self, book: &Book, volts: f64, wait: bool) {
        let guard = if wait {
            Some(book.lock())
        } else {
            book.try_lock()
        };
        let Some(mut b) = guard else {
            return;
        };
        self.judge_charge(b.last_volts, volts);
        if let Some((was, now_v)) = self.charged.take() {
            b.event(
                now(),
                "charge",
                format!("pack charged: {was:.2} V → {now_v:.2} V"),
            );
            b.life.tallies.insert(key::SINCE_CHARGE_S.to_owned(), 0.0);
        }
        let powered = self.life.powered_s;
        merge_life(&mut b.life, &std::mem::take(&mut self.life));
        let records = self.rec.take();
        let life = &mut b.life;
        merge_maps(&mut life.tallies, &mut life.highs, &mut life.lows, &records);
        // The longest stretch on one pack, as it is being run.
        let since = life
            .tallies
            .entry(key::SINCE_CHARGE_S.to_owned())
            .or_default();
        *since += powered;
        let since = *since;
        let best = life
            .highs
            .entry(key::ON_ONE_CHARGE_S.to_owned())
            .or_default();
        *best = best.max(since);
        for (joint, wear) in self.wear.iter_mut().enumerate() {
            let wear = std::mem::take(wear);
            if let Some(m) = b.motors.iter_mut().find(|m| m.id == JOINT_IDS[joint]) {
                m.wear.add(&wear);
            }
        }
        if volts > 0.0 {
            b.last_volts = volts;
        }
    }
}

fn merge_life(into: &mut Life, d: &Life) {
    into.powered_s += d.powered_s;
    into.torqued_s += d.torqued_s;
    into.walking_s += d.walking_s;
    into.standing_s += d.standing_s;
    into.sitting_s += d.sitting_s;
    into.in_hands_s += d.in_hands_s;
    into.fallen_s += d.fallen_s;
    into.theremin_s += d.theremin_s;
    into.singing_s += d.singing_s;
    into.distance_m += d.distance_m;
    into.turned_rad += d.turned_rad;
    into.boots += d.boots;
    into.starts += d.starts;
    into.updates += d.updates;
    into.falls += d.falls;
    into.soft_landings += d.soft_landings;
    into.pickups += d.pickups;
    into.ground_picks += d.ground_picks;
    into.sit_downs += d.sit_downs;
    into.stand_ups += d.stand_ups;
    into.wake_ups += d.wake_ups;
    into.pets += d.pets;
    into.songs += d.songs;
    into.motor_reboots += d.motor_reboots;
    into.power_offs += d.power_offs;
    into.charges += d.charges;
    for (skill, n) in &d.skills {
        *into.skills.entry(skill.clone()).or_default() += n;
    }
    into.bus_drops += d.bus_drops;
    into.missed_ticks += d.missed_ticks;
    into.hottest_motor_c = into.hottest_motor_c.max(d.hottest_motor_c);
    into.hottest_cpu_c = into.hottest_cpu_c.max(d.hottest_cpu_c);
    if d.lowest_volts > 0.0 {
        into.lowest_volts = if into.lowest_volts > 0.0 {
            into.lowest_volts.min(d.lowest_volts)
        } else {
            d.lowest_volts
        };
    }
    into.highest_volts = into.highest_volts.max(d.highest_volts);
    merge_maps(
        &mut into.tallies,
        &mut into.highs,
        &mut into.lows,
        &VetbookTallyParams {
            tallies: d.tallies.clone(),
            highs: d.highs.clone(),
            lows: d.lows.clone(),
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_control::io::Sensors;

    fn facts<'a>(label: &'a str, fresh: Option<&'a Sensors>) -> Facts<'a> {
        Facts {
            dt: 0.02,
            torqued: true,
            label,
            twist: [0.2, 0.0, 0.0],
            in_hands: false,
            fallen: false,
            limp_falling: false,
            theremin: false,
            singing: false,
            odom: ([0.0; 3], 0.0),
            fresh,
            overran: false,
            flashlight: false,
            mouth: 0.0,
            sounds_played: None,
        }
    }

    fn records(t: &mut Tracker) -> VetbookTallyParams {
        t.rec.take()
    }

    fn tilted(deg: f64) -> Sensors {
        let mut s = Sensors::default();
        let r = deg.to_radians();
        s.imu.gravity = [r.sin(), 0.0, -r.cos()];
        s
    }

    #[test]
    fn a_fall_ends_the_walk_streak_but_not_the_record() {
        let mut t = Tracker::default();
        let mut f = facts("walk", None);
        t.tick(&f);
        f.odom = ([3.0, 0.0, 0.0], 0.0);
        t.tick(&f);
        f.fallen = true;
        t.tick(&f);
        f.fallen = false;
        f.odom = ([4.0, 0.0, 0.0], 0.0);
        t.tick(&f);
        let r = records(&mut t);
        assert_eq!(r.highs[key::WALK_STREAK_M], 3.0);
        assert_eq!(t.streak_m, 1.0);
    }

    #[test]
    fn a_tilt_counts_only_when_no_fall_follows() {
        // Leans to 40°, stays up for two windows: survived.
        let mut t = Tracker::default();
        let lean = tilted(40.0);
        let upright = tilted(0.0);
        t.tick(&facts("stand", Some(&lean)));
        for _ in 0..(2 * TILT_WINDOW_TICKS) {
            t.tick(&facts("stand", Some(&upright)));
        }
        assert!((records(&mut t).highs[key::TILT_SURVIVED_DEG] - 40.0).abs() < 1e-6);

        // Leans to 70° and falls within the window: not survived.
        let mut t = Tracker::default();
        let lean = tilted(70.0);
        t.tick(&facts("stand", Some(&lean)));
        let mut f = facts("stand", Some(&upright));
        f.fallen = true;
        t.tick(&f);
        for _ in 0..(3 * TILT_WINDOW_TICKS) {
            t.tick(&facts("stand", Some(&upright)));
        }
        assert!(!records(&mut t).highs.contains_key(key::TILT_SURVIVED_DEG));
    }

    #[test]
    fn upside_down_spin_and_servo_records() {
        let mut t = Tracker::default();
        let mut s = tilted(170.0);
        s.imu.gyro = [0.0, 0.0, std::f64::consts::PI];
        s.currents_ma[4] = -1500.0;
        s.velocities[2] = -std::f64::consts::FRAC_PI_2;
        t.tick(&facts("held", Some(&s)));
        let r = records(&mut t);
        assert!((r.tallies[key::UPSIDE_DOWN_S] - 0.02).abs() < 1e-9);
        assert!((r.highs[key::SPIN_DPS] - 180.0).abs() < 1e-6);
        assert!((r.highs[key::PEAK_CURRENT_A] - 1.5).abs() < 1e-9);
        assert!((r.highs[key::JOINT_SPEED_DPS] - 90.0).abs() < 1e-6);
    }

    #[test]
    fn flaps_lights_and_sounds_count_their_edges() {
        let mut t = Tracker::default();
        let mut f = facts("stand", None);
        // Beak: open, wobble above a third, close, open again: two flaps.
        for mouth in [0.6, 0.4, 0.7, 0.1, 0.9] {
            f.mouth = mouth;
            t.tick(&f);
        }
        // Flashlight: on for three ticks, one switch-on.
        f.flashlight = true;
        for _ in 0..3 {
            t.tick(&f);
        }
        // Sounds: the voice's own count, as a difference from the first one seen.
        for n in [5, 5, 7, 9] {
            f.sounds_played = Some(n);
            t.tick(&f);
        }
        let r = records(&mut t);
        assert_eq!(r.tallies[key::BEAK_FLAPS], 2.0);
        assert_eq!(r.tallies[key::FLASHLIGHT_ON], 1.0);
        assert!((r.tallies[key::FLASHLIGHT_S] - 0.14).abs() < 1e-9);
        assert_eq!(r.tallies[key::SOUNDS], 4.0);
    }

    #[test]
    fn the_longest_stretch_on_one_charge_is_kept_across_charges() {
        let book = Book::open(None, "0.17.2");
        let mut t = Tracker::default();
        t.life.powered_s = 100.0;
        t.handoff(&book, 7.0, true);
        t.life.powered_s = 50.0;
        t.handoff(&book, 7.0, true);
        // A new process sees the pack charged: the stretch starts again; the record stays.
        let mut t = Tracker::default();
        t.life.powered_s = 20.0;
        t.handoff(&book, 8.2, true);
        let b = book.snapshot();
        assert_eq!(b.life.highs[key::ON_ONE_CHARGE_S], 150.0);
        assert_eq!(b.life.tallies[key::SINCE_CHARGE_S], 20.0);
    }

    #[test]
    fn another_daemons_tally_lands_in_the_book() {
        let book = Book::open(None, "0.17.2");
        let mut p = VetbookTallyParams::default();
        p.tallies.insert(key::PHOTOS.into(), 3.0);
        p.lows.insert(key::CLOSEST_MM.into(), 42.0);
        book.tally(&p);
        book.tally(&p);
        book.bump(key::GAIT_CHANGES, 1.0);
        let b = book.snapshot();
        assert_eq!(b.life.tallies[key::PHOTOS], 6.0);
        assert_eq!(b.life.lows[key::CLOSEST_MM], 42.0);
        assert_eq!(b.life.tallies[key::GAIT_CHANGES], 1.0);
    }

    fn scratch() -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "vetbook-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        dir.join("vetbook.json")
    }

    #[test]
    fn a_skill_is_counted_once_per_run_not_once_per_tick() {
        let mut t = Tracker::default();
        let sensors = Sensors::default();
        for label in [
            "stand",
            "kick_left",
            "kick_left",
            "kick_left:unwind",
            "stand",
            "kick_left",
        ] {
            t.tick(&facts(label, Some(&sensors)));
        }
        assert_eq!(t.life.skills.get("kick_left"), Some(&2));
        assert!(!t.life.skills.contains_key("kick_left:unwind"));
        assert!(!t.life.skills.contains_key("stand"));
    }

    #[test]
    fn velstand_standing_still_is_standing() {
        let mut t = Tracker::default();
        // Velstand labels every tick `walk`; a zero twist is a robot standing.
        let mut f = facts("walk", None);
        f.twist = [0.0, 0.0, 0.0];
        t.tick(&f);
        // A released stick still decaying through the smoothing: standing.
        f.twist = [0.01, 0.0, 0.05];
        t.tick(&f);
        // Turning on the spot is walking.
        f.twist = [0.0, 0.0, 0.5];
        t.tick(&f);
        assert!((t.life.standing_s - 0.04).abs() < 1e-9);
        assert!((t.life.walking_s - 0.02).abs() < 1e-9);
    }

    #[test]
    fn only_walking_covers_ground() {
        let mut t = Tracker::default();
        let mut f = facts("walk", None);
        t.tick(&f);
        f.odom = ([0.3, 0.4, 0.0], 0.5);
        t.tick(&f);
        // Odometry moving a metre while asked to stand still — drift, or a shove: not walked.
        f.twist = [0.0; 3];
        f.odom = ([1.3, 0.4, 0.0], 0.5);
        t.tick(&f);
        assert!((t.life.distance_m - 0.5).abs() < 1e-9);
        assert!((t.life.turned_rad - 0.5).abs() < 1e-9);
        assert_eq!(t.life.bus_drops, 3);
    }

    #[test]
    fn edges_count_falls_and_pickups_once() {
        let mut t = Tracker::default();
        let mut f = facts("held", None);
        for fallen in [false, true, true, false, true] {
            f.fallen = fallen;
            f.in_hands = fallen;
            t.tick(&f);
        }
        assert_eq!(t.life.falls, 2);
        assert_eq!(t.life.pickups, 2);
    }

    #[test]
    fn a_joint_wears_by_travel_and_by_current() {
        let mut t = Tracker::default();
        let mut s = Sensors::default();
        t.tick(&facts("walk", Some(&s)));
        s.positions[2] = 0.25;
        s.currents_ma[2] = 500.0;
        t.tick(&facts("walk", Some(&s)));
        s.positions[2] = 0.0;
        t.tick(&facts("walk", Some(&s)));
        assert!((t.wear[2].travel_rad - 0.5).abs() < 1e-9);
        assert!((t.wear[2].charge_as - 0.02).abs() < 1e-9);
        assert_eq!(t.wear[0].travel_rad, 0.0);
    }

    #[test]
    fn the_book_survives_a_restart_and_counts_it() {
        let path = scratch();
        let book = Book::open(Some(path.clone()), "0.17.2");
        let mut t = Tracker::default();
        t.tick(&facts("walk", None));
        t.alerts(&[0b10_0000; NUM_JOINTS]);
        t.handoff(&book, 7.9, true);
        book.adopted(JOINT_IDS[0]);
        book.note("  reglued the beak  ").unwrap();
        book.save();
        drop(book);

        let book = Book::open(Some(path.clone()), "0.18.0");
        let b = book.snapshot();
        assert_eq!(b.life.starts, 2);
        assert_eq!(b.life.updates, 1);
        assert!((b.life.walking_s - 0.02).abs() < 1e-9);
        assert_eq!(b.motors[0].retired.len(), 1);
        assert_eq!(b.motors[0].retired[0].wear.trips.overload, 1);
        assert_eq!(b.motors[0].wear.trips.overload, 0);
        assert_eq!(b.motors[1].wear.trips.overload, 1);
        assert!(
            b.events
                .iter()
                .any(|e| e.kind == "note" && e.text == "reglued the beak")
        );
        assert!(
            b.events
                .iter()
                .any(|e| e.kind == "update" && e.text.contains("0.18.0"))
        );
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    #[test]
    fn an_unreadable_book_is_set_aside_not_overwritten() {
        let path = scratch();
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, b"{ not json").unwrap();
        let book = Book::open(Some(path.clone()), "0.17.2");
        assert_eq!(book.snapshot().life.starts, 1);
        let aside = std::fs::read_dir(path.parent().unwrap())
            .unwrap()
            .filter_map(Result::ok)
            .any(|e| e.file_name().to_string_lossy().contains("unreadable"));
        assert!(aside);
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    #[test]
    fn a_charge_is_guessed_from_the_first_reading_only() {
        let book = Book::open(None, "0.17.2");
        book.lock().last_volts = 6.9;
        let mut t = Tracker::default();
        t.handoff(&book, 8.1, true);
        t.handoff(&book, 6.8, true);
        t.handoff(&book, 8.1, true);
        let b = book.snapshot();
        assert_eq!(b.life.charges, 1);
        assert!(b.events.iter().any(|e| e.kind == "charge"));
    }

    #[test]
    fn an_empty_note_is_refused() {
        let book = Book::open(None, "0.17.2");
        assert!(book.note("   ").is_err());
    }
}
