//! Exclusive raw controller. Only tick() can touch Safety/RobotIo; IPC only enqueues.
use arc_swap::ArcSwap;
use duck_control::{
    io::RobotIo,
    model::{JOINT_IDS, JOINT_NAMES, NUM_JOINTS},
    safety::Safety,
};
use duck_ipc_proto::diagnostics as d;
use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
use tokio::sync::{mpsc, oneshot};
const LEASE: Duration = Duration::from_millis(300);
const CACHE: usize = 256;
pub struct Pending {
    pub connection: u64,
    pub call: d::Call,
    pub reply: oneshot::Sender<d::Reply>,
}
pub struct Shared {
    pub blocked: AtomicBool,
    pub owner: AtomicU64,
    last_owner: AtomicU64,
    pub cancel: AtomicBool,
    pub relax: AtomicBool,
    pub status: ArcSwap<d::Status>,
    pub tx: mpsc::Sender<Pending>,
    priority: Mutex<Option<Pending>>,
    rx: Mutex<Option<mpsc::Receiver<Pending>>>,
    next_connection: AtomicU64,
    refresh_at_ns: AtomicU64,
}
impl Shared {
    pub fn new() -> Self {
        let (tx, rx) = mpsc::channel(16);
        Self {
            blocked: AtomicBool::new(false),
            owner: AtomicU64::new(0),
            last_owner: AtomicU64::new(0),
            cancel: AtomicBool::new(false),
            relax: AtomicBool::new(false),
            status: ArcSwap::from_pointee(d::Status::default()),
            tx,
            priority: Mutex::new(None),
            rx: Mutex::new(Some(rx)),
            next_connection: AtomicU64::new(1),
            refresh_at_ns: AtomicU64::new(0),
        }
    }
    pub fn connection(self: &Arc<Self>) -> Connection {
        Connection {
            shared: self.clone(),
            id: self.next_connection.fetch_add(1, Ordering::Relaxed),
        }
    }
    pub fn enqueue(&self, p: Pending) -> Result<(), &'static str> {
        if matches!(p.call, d::Call::Refresh(_))
            && !self.admit_refresh(duck_ipc_proto::clock::monotonic_ns())
        {
            return Err("rate_limited");
        }
        let owner = self.owner.load(Ordering::Acquire);
        let terminal_release = matches!(p.call, d::Call::Release(_))
            && owner == 0
            && self.last_owner.load(Ordering::Acquire) == p.connection;
        if matches!(p.call, d::Call::Stop(_) | d::Call::Release(_))
            && owner != p.connection
            && !terminal_release
        {
            return Err("wrong_session");
        }
        if matches!(
            p.call,
            d::Call::Stop(_) | d::Call::Release(_) | d::Call::Cancel(_)
        ) {
            let mut slot = self.priority.lock().map_err(|_| "busy")?;
            if slot.is_some() {
                return Err("stop_pending");
            }
            *slot = Some(p);
            Ok(())
        } else {
            self.tx.try_send(p).map_err(|_| "busy")
        }
    }
    fn admit_refresh(&self, now: u64) -> bool {
        let previous = self.refresh_at_ns.load(Ordering::Acquire);
        now.saturating_sub(previous) >= 1_000_000_000
            && self
                .refresh_at_ns
                .compare_exchange(previous, now, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
    }
    pub fn receiver(&self) -> mpsc::Receiver<Pending> {
        self.rx.lock().unwrap().take().expect("one control owner")
    }
}
pub struct Connection {
    shared: Arc<Shared>,
    pub id: u64,
}
impl Drop for Connection {
    fn drop(&mut self) {
        if self.shared.owner.load(Ordering::Acquire) == self.id {
            self.shared.cancel.store(true, Ordering::Release)
        }
    }
}
struct Session {
    connection: u64,
    id: u64,
    lease: Instant,
    heartbeat: u64,
    high: u64,
    cache: VecDeque<(u64, d::Call, d::Reply)>,
    acquire_request: d::Acquire,
    acquire_reply: Option<d::Reply>,
}
pub struct Controller {
    rx: mpsc::Receiver<Pending>,
    session: Option<Session>,
    acquiring: Option<(Pending, Instant)>,
    motors: Vec<d::Motor>,
    scan: usize,
    controlled: [bool; NUM_JOINTS],
    inhibit: [bool; NUM_JOINTS],
    deadlines: [Option<Instant>; NUM_JOINTS],
    generation: u64,
    sequence: u64,
    tick_sequence: u64,
    last: Option<d::Reply>,
    sent_targets: std::collections::BTreeMap<String, f64>,
    last_command: Option<d::Command>,
    terminal: Option<(u64, d::Call, d::Reply)>,
    exit_owed: bool,
    shared: Arc<Shared>,
    last_refresh: Instant,
    refresh_remaining: usize,
}
impl Controller {
    pub fn new(shared: Arc<Shared>) -> Self {
        Self {
            rx: shared.receiver(),
            shared,
            session: None,
            acquiring: None,
            motors: JOINT_IDS
                .iter()
                .map(|id| duck_control::diagnostics::empty_motor(*id).unwrap())
                .collect(),
            scan: 0,
            controlled: [false; NUM_JOINTS],
            inhibit: [false; NUM_JOINTS],
            deadlines: [None; NUM_JOINTS],
            generation: 0,
            sequence: 0,
            tick_sequence: 0,
            last: None,
            sent_targets: Default::default(),
            last_command: None,
            terminal: None,
            exit_owed: false,
            last_refresh: Instant::now() - Duration::from_secs(2),
            refresh_remaining: NUM_JOINTS,
        }
    }
    pub fn blocked(&self) -> bool {
        self.exit_owed
            || self.session.is_some()
            || self.acquiring.is_some()
            || self.inhibit.iter().any(|v| *v)
    }
    fn read<T: RobotIo>(&mut self, s: &mut Safety<T>, i: usize) {
        match s.diagnostic_read(JOINT_IDS[i]) {
            Ok(mut m) => {
                m.observed_at_ns = duck_ipc_proto::clock::monotonic_ns();
                self.motors[i] = m
            }
            Err(e) => {
                let mut m = duck_control::diagnostics::empty_motor(JOINT_IDS[i]).unwrap();
                m.before_startup_fix = self.motors[i].before_startup_fix;
                m.error = Some(e.to_string());
                m.observed_at_ns = duck_ipc_proto::clock::monotonic_ns();
                self.motors[i] = m;
            }
        }
    }
    fn off<T: RobotIo>(&mut self, s: &mut Safety<T>, indices: &[usize]) -> Vec<d::Outcome> {
        indices
            .iter()
            .map(|&i| {
                self.deadlines[i] = None;
                let write = s.diagnostic_torque(JOINT_IDS[i], false);
                self.read(s, i);
                let confirmed = self.motors[i].torque == Some(false);
                self.inhibit[i] = !confirmed;
                if confirmed {
                    self.controlled[i] = false;
                    self.deadlines[i] = None;
                    self.sent_targets.remove(JOINT_NAMES[i]);
                }
                d::Outcome {
                    joint: JOINT_NAMES[i].into(),
                    status: if confirmed {
                        "readback_confirmed"
                    } else {
                        "stop_unconfirmed"
                    }
                    .into(),
                    error: write
                        .err()
                        .map(|e| e.to_string())
                        .or_else(|| self.motors[i].error.clone()),
                }
            })
            .collect()
    }
    fn stop<T: RobotIo>(&mut self, s: &mut Safety<T>, close: bool) -> Vec<d::Outcome> {
        if close && (self.session.is_some() || self.inhibit.iter().any(|v| *v)) {
            self.exit_owed = true;
        }
        let ids: Vec<_> = (0..NUM_JOINTS)
            .filter(|&i| self.controlled[i] || self.inhibit[i])
            .collect();
        let out = self.off(s, &ids);
        if close {
            if let Some((p, _)) = self.acquiring.take() {
                let _ = p.reply.send(d::Reply::refused("cancelled"));
            }
            if let Some(session) = self.session.as_ref() {
                self.shared
                    .last_owner
                    .store(session.connection, Ordering::Release);
            }
            self.session = None;
            self.shared.owner.store(0, Ordering::Release);
        }
        out
    }
    fn discard_pending(&mut self) {
        while let Ok(p) = self.rx.try_recv() {
            let _ = p.reply.send(d::Reply::refused("cancelled"));
        }
    }
    fn publish(&self, now: Instant) {
        let name = |bits: &[bool; NUM_JOINTS]| {
            (0..NUM_JOINTS)
                .filter(|&i| bits[i])
                .map(|i| JOINT_NAMES[i].into())
                .collect()
        };
        let st = d::Status {
            exit_pending: self.exit_owed,
            mode: self.blocked().then(|| "maintenance_raw".into()),
            session_id: self.session.as_ref().map(|s| s.id),
            lease_remaining_ms: self.session.as_ref().map_or(0, |s| {
                s.lease.saturating_duration_since(now).as_millis() as u64
            }),
            controlled: name(&self.controlled),
            stop_unconfirmed: name(&self.inhibit),
            motors: self.motors.clone(),
            last_reply: self.last.clone(),
            pickup_enforced: self.blocked().then_some(false),
            snapshot_at_ns: duck_ipc_proto::clock::monotonic_ns(),
            sent_targets_rad: self.sent_targets.clone(),
            last_command: self.last_command.clone(),
            command_deadlines_ms: (0..NUM_JOINTS)
                .filter_map(|i| {
                    self.deadlines[i].map(|d| {
                        (
                            JOINT_NAMES[i].into(),
                            d.saturating_duration_since(now).as_millis() as u64,
                        )
                    })
                })
                .collect(),
        };
        self.shared.status.store(Arc::new(st));
        self.shared.blocked.store(self.blocked(), Ordering::Release);
    }
    /// Process one mailbox operation and one background read, never IPC or disk work.
    pub fn tick<T: RobotIo>(&mut self, s: &mut Safety<T>, now: Instant, normal_idle: bool) -> bool {
        self.tick_sequence += 1;

        if self.shared.relax.swap(false, Ordering::AcqRel) {
            self.controlled.fill(true);
            let outcomes = self.stop(s, true);
            self.last = Some(self.reply(outcomes));
            self.discard_pending();
        }
        if self.shared.cancel.swap(false, Ordering::AcqRel)
            || self.session.as_ref().is_some_and(|v| now >= v.lease)
        {
            let out = self.stop(s, true);
            self.discard_pending();
            self.last = Some(d::Reply {
                accepted: !self.inhibit.iter().any(|v| *v),
                reason: Some("session_terminated".into()),
                session_id: None,
                sequence: self.sequence,
                committed_at_ns: duck_ipc_proto::clock::monotonic_ns(),
                commit_tick: self.tick_sequence,
                lease_ms: 300,
                snapshot: None,
                outcomes: out,
            });
        }
        let priority = self
            .shared
            .priority
            .try_lock()
            .ok()
            .and_then(|mut s| s.take());
        if let Some(p) = priority {
            let may_stop = matches!(p.call, d::Call::Cancel(_))
                || self.session.as_ref().is_some_and(|x| {
                    x.connection == p.connection
                        && matches!(&p.call,d::Call::Stop(v)|d::Call::Release(v) if v.session_id==x.id)
                });
            let r = self.execute(s, now, p.connection, &p.call);
            if may_stop {
                self.discard_pending();
            }
            self.last = Some(r.clone());
            let _ = p.reply.send(r);
        }
        let expired: Vec<_> = (0..NUM_JOINTS)
            .filter(|&i| self.deadlines[i].is_some_and(|d| now >= d))
            .collect();
        if !expired.is_empty() {
            let out = self.off(s, &expired);
            self.last = Some(d::Reply {
                accepted: !self.inhibit.iter().any(|v| *v),
                reason: Some("command_expired".into()),
                session_id: self.session.as_ref().map(|v| v.id),
                sequence: self.sequence,
                committed_at_ns: duck_ipc_proto::clock::monotonic_ns(),
                commit_tick: self.tick_sequence,
                lease_ms: 300,
                snapshot: None,
                outcomes: out,
            });
        }
        // Cache reads are spaced over ticks. Acquisition waits for a complete post-reservation sweep.
        if self.blocked() || (self.refresh_remaining > 0 && normal_idle) {
            self.read(s, self.scan);
            self.scan = (self.scan + 1) % NUM_JOINTS;
            self.refresh_remaining = self.refresh_remaining.saturating_sub(1);
        }
        if let Some((p, since)) = self.acquiring.take() {
            let epoch = duck_ipc_proto::clock::monotonic_ns()
                .saturating_sub(now.duration_since(since).as_nanos() as u64);
            if p.reply.is_closed() || since.elapsed() > Duration::from_secs(2) {
                let _ = p.reply.send(d::Reply::refused("acquire_expired"));
            } else if !normal_idle {
                let _ = p.reply.send(d::Reply::refused("controller_active"));
            } else if self.motors.iter().all(|m| m.observed_at_ns >= epoch) {
                let reply = if self
                    .motors
                    .iter()
                    .any(|m| m.error.is_some() || m.torque.is_none())
                {
                    let mut r = d::Reply::refused("torque_unknown");
                    r.outcomes = self
                        .motors
                        .iter()
                        .filter(|m| m.torque.is_none())
                        .map(|m| d::Outcome {
                            joint: m.joint.clone(),
                            status: "unknown".into(),
                            error: m.error.clone(),
                        })
                        .collect();
                    r
                } else if self.motors.iter().any(|m| m.torque == Some(true)) {
                    let mut r = d::Reply::refused("torque_on");
                    r.outcomes = self
                        .motors
                        .iter()
                        .filter(|m| m.torque == Some(true))
                        .map(|m| d::Outcome {
                            joint: m.joint.clone(),
                            status: "torque_on".into(),
                            error: None,
                        })
                        .collect();
                    r
                } else if self.motors.iter().any(|m| !goal_writes_preserve_torque(m)) {
                    let mut r = d::Reply::refused("unsupported_drive_mode");
                    r.outcomes = self
                        .motors
                        .iter()
                        .filter(|m| !goal_writes_preserve_torque(m))
                        .map(|m| d::Outcome {
                            joint: m.joint.clone(),
                            status: "unsupported_drive_mode".into(),
                            error: Some(
                                "drive mode is unknown or enables torque on goal update".into(),
                            ),
                        })
                        .collect();
                    r
                } else {
                    self.sent_targets.clear();
                    self.last_command = None;
                    self.generation += 1;
                    let id = self.generation;
                    let req = match &p.call {
                        d::Call::Acquire(v) => v.request_id,
                        _ => 0,
                    };
                    self.session = Some(Session {
                        connection: p.connection,
                        id,
                        lease: now + LEASE,
                        heartbeat: 0,
                        high: req,
                        cache: VecDeque::new(),
                        acquire_reply: None,
                        acquire_request: match &p.call {
                            d::Call::Acquire(v) => v.clone(),
                            _ => unreachable!(),
                        },
                    });
                    self.shared.owner.store(p.connection, Ordering::Release);
                    d::Reply {
                        accepted: true,
                        reason: None,
                        session_id: Some(id),
                        sequence: self.sequence,
                        committed_at_ns: duck_ipc_proto::clock::monotonic_ns(),
                        commit_tick: self.tick_sequence,
                        lease_ms: 300,
                        snapshot: None,
                        outcomes: vec![],
                    }
                };
                let mut reply = reply;
                if reply.accepted {
                    reply.snapshot = Some(self.motors.clone());
                    if let Some(session) = self.session.as_mut() {
                        session.acquire_reply = Some(reply.clone());
                    }
                }
                self.last = Some(reply.clone());
                let _ = p.reply.send(reply);
            } else {
                self.acquiring = Some((p, since));
            }
        }
        if self.acquiring.is_none()
            && let Ok(p) = self.rx.try_recv()
            && !p.reply.is_closed()
        {
            if let d::Call::Acquire(v) = &p.call {
                if self
                    .session
                    .as_ref()
                    .is_some_and(|x| x.connection == p.connection)
                {
                    let session = self.session.as_ref().unwrap();
                    let r = if *v == session.acquire_request {
                        session
                            .acquire_reply
                            .clone()
                            .unwrap_or_else(|| d::Reply::refused("missing_acquire_outcome"))
                    } else {
                        d::Reply::refused("request_conflict")
                    };
                    let _ = p.reply.send(r);
                } else if self.blocked() {
                    let _ = p.reply.send(d::Reply::refused("busy"));
                } else if !normal_idle {
                    let _ = p.reply.send(d::Reply::refused("controller_active"));
                } else if v.mode != "maintenance_raw" || !v.acknowledge_raw {
                    let _ = p.reply.send(d::Reply::refused("acknowledge_raw_required"));
                } else {
                    self.acquiring = Some((p, now));
                }
            } else {
                let reply = self.execute(s, now, p.connection, &p.call);
                self.last = Some(reply.clone());
                let _ = p.reply.send(reply);
            }
        }
        self.publish(now);
        self.exit_owed
            && self.session.is_none()
            && self.acquiring.is_none()
            && !self.inhibit.iter().any(|v| *v)
    }
    fn execute<T: RobotIo>(
        &mut self,
        s: &mut Safety<T>,
        now: Instant,
        connection: u64,
        call: &d::Call,
    ) -> d::Reply {
        if let Some((owner, old, r)) = &self.terminal
            && *owner == connection
            && old == call
        {
            return r.clone();
        }
        let rid = match call {
            d::Call::Torque(v) => Some((v.session_id, v.request_id)),
            d::Call::Gains(v) => Some((v.session_id, v.request_id)),
            d::Call::Command(v) => Some((v.session_id, v.request_id)),
            d::Call::Stop(v) | d::Call::Release(v) => Some((v.session_id, v.request_id)),
            _ => None,
        };
        let new = rid.filter(|(sid, id)| {
            self.session
                .as_ref()
                .is_some_and(|x| x.connection == connection && x.id == *sid && *id > x.high)
        });
        let reply = self.execute_inner(s, now, connection, call);
        if new.is_some() && matches!(call, d::Call::Release(_)) && self.session.is_none() {
            self.terminal = Some((connection, call.clone(), reply.clone()));
        }
        if let (Some((sid, id)), Some(session)) = (new, self.session.as_mut())
            && session.id == sid
        {
            session.high = id;
            session.cache.push_back((id, call.clone(), reply.clone()));
            if session.cache.len() > CACHE {
                session.cache.pop_front();
            }
        }
        reply
    }
    fn execute_inner<T: RobotIo>(
        &mut self,
        s: &mut Safety<T>,
        now: Instant,
        connection: u64,
        call: &d::Call,
    ) -> d::Reply {
        if matches!(call, d::Call::Refresh(_)) {
            if now.duration_since(self.last_refresh) < Duration::from_secs(1) {
                return d::Reply::refused("rate_limited");
            }
            self.last_refresh = now;
            self.refresh_remaining = NUM_JOINTS;
            return d::Reply {
                accepted: true,
                reason: Some("scheduled".into()),
                session_id: self.session.as_ref().map(|x| x.id),
                sequence: self.sequence,
                committed_at_ns: duck_ipc_proto::clock::monotonic_ns(),
                commit_tick: self.tick_sequence,
                lease_ms: 300,
                snapshot: None,
                outcomes: vec![],
            };
        }
        if matches!(call, d::Call::Cancel(_)) {
            let out = self.stop(s, true);
            return self.reply(out);
        }
        let Some(session) = self.session.as_ref() else {
            return d::Reply::refused("no_session");
        };
        let (sid, rid) = match call {
            d::Call::Torque(v) => (v.session_id, Some(v.request_id)),
            d::Call::Gains(v) => (v.session_id, Some(v.request_id)),
            d::Call::Command(v) => (v.session_id, Some(v.request_id)),
            d::Call::Stop(v) | d::Call::Release(v) => (v.session_id, Some(v.request_id)),
            d::Call::Keepalive(v) => (v.session_id, None),
            _ => return d::Reply::refused("invalid_operation"),
        };
        if session.connection != connection || session.id != sid {
            return d::Reply::refused("wrong_session");
        }
        if let Some(rid) = rid {
            if let Some((_, old, r)) = session.cache.iter().find(|x| x.0 == rid) {
                return if old == call {
                    r.clone()
                } else {
                    d::Reply::refused("request_conflict")
                };
            }
            if rid <= session.high {
                return d::Reply::refused("request_expired");
            }
        }
        if let d::Call::Keepalive(v) = call {
            let session = self.session.as_mut().unwrap();
            if v.sequence <= session.heartbeat {
                return d::Reply::refused("stale_sequence");
            }
            session.heartbeat = v.sequence;
            session.lease = now + LEASE;
            return self.reply(vec![]);
        }
        // Validate the entire selection before doing any writes.
        let selection: Vec<&str> = match call {
            d::Call::Torque(v) => v.joints.iter().map(String::as_str).collect(),
            d::Call::Gains(v) => v.gains.keys().map(String::as_str).collect(),
            d::Call::Command(v) => v.positions_rad.keys().map(String::as_str).collect(),
            _ => vec![],
        };
        let mut ids = vec![];
        for name in selection {
            let Some(i) = JOINT_NAMES.iter().position(|n| *n == name) else {
                return d::Reply::refused("unknown_joint");
            };
            if ids.contains(&i) {
                return d::Reply::refused("duplicate_joint");
            }
            ids.push(i);
        }
        if matches!(
            call,
            d::Call::Torque(_) | d::Call::Gains(_) | d::Call::Command(_)
        ) && ids.is_empty()
        {
            return d::Reply::refused("empty_selection");
        }
        if let d::Call::Command(v) = call {
            if !(1..=60000).contains(&v.valid_for_ms)
                || v.positions_rad
                    .values()
                    .any(|q| !duck_control::diagnostics::position_valid(*q))
            {
                return d::Reply::refused("invalid_value");
            }
            for &i in &ids {
                if !self.controlled[i] || self.motors[i].torque != Some(true) {
                    return d::Reply::refused("torque_off");
                }
                if self.motors[i].operating_mode != Some(3) {
                    return d::Reply::refused("wrong_mode");
                }
                let q = v.positions_rad[JOINT_NAMES[i]];
                if self.motors[i].min_position_rad.is_none_or(|x| q < x)
                    || self.motors[i].max_position_rad.is_none_or(|x| q > x)
                {
                    return d::Reply::refused("native_position_limit");
                }
            }
        }
        if let d::Call::Gains(v) = call
            && v.gains
                .values()
                .any(|p| !duck_control::diagnostics::pid_valid(*p))
        {
            return d::Reply::refused("invalid_value");
        }
        if self.inhibit.iter().any(|v| *v)
            && !matches!(
                call,
                d::Call::Stop(_)
                    | d::Call::Release(_)
                    | d::Call::Torque(d::Torque { on: false, .. })
            )
        {
            return d::Reply::refused("stop_unconfirmed");
        }
        self.sequence += 1;
        let out = match call {
            d::Call::Stop(_) => self.stop(s, false),
            d::Call::Release(_) => self.stop(s, true),
            d::Call::Torque(v) if !v.on => self.off(s, &ids),
            d::Call::Torque(_) => {
                let mut out = vec![];
                for &i in &ids {
                    self.read(s, i);
                    let m = &self.motors[i];
                    let result = if m.torque == Some(true) && self.controlled[i] {
                        Ok(())
                    } else if !goal_writes_preserve_torque(m) {
                        Err("unsupported_drive_mode".into())
                    } else if m.operating_mode != Some(3) || m.torque != Some(false) {
                        Err("wrong_mode_or_unknown_torque".into())
                    } else {
                        let q = m.position_counts.map(|p| {
                            2.0 * std::f64::consts::PI * p as f64 / 4096.0 - std::f64::consts::PI
                        });
                        match q {
                            Some(q) if duck_control::diagnostics::position_valid(q) => {
                                let seeded = s
                                    .diagnostic_goals(&[(JOINT_IDS[i], q)])
                                    .map_err(|e| e.to_string());
                                self.read(s, i);
                                if seeded.is_ok()
                                    && self.motors[i].goal_rad.is_some_and(|g| {
                                        (g - q).abs() <= 2.0 * std::f64::consts::PI / 4096.0
                                    })
                                {
                                    self.controlled[i] = true;
                                    let r = s
                                        .diagnostic_torque(JOINT_IDS[i], true)
                                        .map_err(|e| e.to_string());
                                    self.read(s, i);
                                    if r.is_err() || self.motors[i].torque != Some(true) {
                                        self.off(s, &[i]);
                                        Err("enable_unconfirmed".into())
                                    } else {
                                        Ok(())
                                    }
                                } else {
                                    Err("goal_seed_failed".into())
                                }
                            }
                            _ => Err("invalid_position".into()),
                        }
                    };
                    out.push(d::Outcome {
                        joint: JOINT_NAMES[i].into(),
                        status: if result.is_ok() {
                            "readback_confirmed"
                        } else {
                            "failed"
                        }
                        .into(),
                        error: result.err(),
                    });
                }
                out
            }
            d::Call::Gains(v) => ids
                .iter()
                .map(|&i| {
                    let pid = v.gains[JOINT_NAMES[i]];
                    let r = s.diagnostic_pid(JOINT_IDS[i], pid);
                    self.read(s, i);
                    let ok = r.is_ok() && self.motors[i].pid == Some(pid);
                    d::Outcome {
                        joint: JOINT_NAMES[i].into(),
                        status: if ok { "readback_confirmed" } else { "failed" }.into(),
                        error: r
                            .err()
                            .map(|e| e.to_string())
                            .or_else(|| (!ok).then(|| "readback_mismatch".into())),
                    }
                })
                .collect(),
            d::Call::Command(v) => {
                let goals: Vec<_> = ids
                    .iter()
                    .map(|&i| (JOINT_IDS[i], v.positions_rad[JOINT_NAMES[i]]))
                    .collect();
                self.last_command = Some(v.clone());
                let r = s.diagnostic_goals(&goals);
                if r.is_ok() {
                    self.sent_targets.extend(v.positions_rad.clone());
                }
                let error = r.err().map(|e| e.to_string());
                for &i in &ids {
                    self.deadlines[i] = Some(now + Duration::from_millis(v.valid_for_ms as u64));
                }
                ids.iter()
                    .map(|&i| d::Outcome {
                        joint: JOINT_NAMES[i].into(),
                        status: if error.is_none() {
                            "sent"
                        } else {
                            "delivery_unknown"
                        }
                        .into(),
                        error: error.clone(),
                    })
                    .collect()
            }
            _ => vec![],
        };
        self.reply(out)
    }
    /// The integration acknowledges only after fresh normal goals/PID are restored off-torque.
    pub fn finish_exit(&mut self) {
        if self.session.is_none() && self.acquiring.is_none() && !self.inhibit.iter().any(|v| *v) {
            self.exit_owed = false;
            self.publish(Instant::now());
        }
    }
    pub fn shutdown<T: RobotIo>(&mut self, s: &mut Safety<T>) {
        if self.blocked() {
            self.stop(s, true);
            self.publish(Instant::now());
        }
    }
    fn reply(&self, outcomes: Vec<d::Outcome>) -> d::Reply {
        let accepted = outcomes
            .iter()
            .all(|o| matches!(o.status.as_str(), "sent" | "readback_confirmed"));
        d::Reply {
            accepted,
            reason: (!accepted).then(|| "partial_failure".into()),
            session_id: self.session.as_ref().map(|v| v.id),
            sequence: self.sequence,
            committed_at_ns: duck_ipc_proto::clock::monotonic_ns(),
            commit_tick: self.tick_sequence,
            lease_ms: 300,
            snapshot: None,
            outcomes,
        }
    }
}

// XL330 Drive Mode bit 3 energizes an off motor when a goal is written. Both goal seeding
// and the normal-goal exit restore rely on goal writes preserving the confirmed torque state.
fn goal_writes_preserve_torque(motor: &d::Motor) -> bool {
    motor.drive_mode.is_some_and(|mode| mode & 0x08 == 0)
}

/// Relay UIDs never acquire maintenance authority, even if in an operator group.
pub fn authorized(uid: Option<u32>, owner: u32, allowed: &[u32], denied: &[u32]) -> bool {
    uid.is_some_and(|uid| {
        !denied.contains(&uid) && (uid == owner || uid == 0 || allowed.contains(&uid))
    })
}

/// Resolve account names once at startup, before accepting untrusted connections.
pub fn user_id(name: &str) -> Option<u32> {
    let name = std::ffi::CString::new(name).ok()?;
    // getpwnam's static storage is used only during single-threaded startup initialization.
    let p = unsafe { libc::getpwnam(name.as_ptr()) };
    if p.is_null() {
        tracing::warn!(user=%name.to_string_lossy(),"account missing; no maintenance permission granted");
        None
    } else {
        Some(unsafe { (*p).pw_uid })
    }
}
pub fn blocks(call: &duck_ipc_proto::Call) -> bool {
    let m = call.method();
    m.starts_with("robot.")
        && !matches!(
            m,
            "robot.health"
                | "robot.model"
                | "robot.modelApi"
                | "robot.mode"
                | "robot.subscribe"
                | "robot.safeToRestart"
                | "robot.vetbook"
                | "robot.policies"
                | "robot.relax"
                | "robot.remoteSessionActive"
        )
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_control::{FakeIo, safety::SafetyConfig};
    use std::collections::BTreeMap;
    fn setup() -> (Controller, Safety<FakeIo>) {
        let shared = Arc::new(Shared::new());
        (
            Controller::new(shared),
            Safety::new(FakeIo::new(), SafetyConfig::default()),
        )
    }
    fn acquired(c: &mut Controller, now: Instant) {
        c.finish_exit();
        c.shared.owner.store(1, Ordering::Release);
        c.session = Some(Session {
            connection: 1,
            id: 1,
            lease: now + Duration::from_secs(10),
            heartbeat: 0,
            high: 1,
            cache: VecDeque::new(),
            acquire_reply: None,
            acquire_request: d::Acquire {
                request_id: 1,
                mode: "maintenance_raw".into(),
                acknowledge_raw: true,
            },
        });
    }
    fn req(c: &mut Controller, s: &mut Safety<FakeIo>, call: d::Call) -> d::Reply {
        c.execute(s, Instant::now(), 1, &call)
    }
    #[test]
    fn all_fifteen_selected_operations_and_idempotency() {
        for i in 0..NUM_JOINTS {
            let (mut c, mut s) = setup();
            acquired(&mut c, Instant::now());
            let name = JOINT_NAMES[i].to_string();
            let on = d::Call::Torque(d::Torque {
                session_id: 1,
                request_id: 2,
                joints: vec![name.clone()],
                on: true,
            });
            assert!(req(&mut c, &mut s, on.clone()).accepted);
            assert!(req(&mut c, &mut s, on).accepted);
            for (j, id) in JOINT_IDS.iter().enumerate() {
                assert_eq!(s.diagnostic_read(*id).unwrap().torque, Some(i == j));
            }
            let pid = d::Pid { p: 123, i: 3, d: 4 };
            assert!(
                req(
                    &mut c,
                    &mut s,
                    d::Call::Gains(d::Gains {
                        session_id: 1,
                        request_id: 3,
                        gains: BTreeMap::from([(name.clone(), pid)])
                    })
                )
                .accepted
            );
            assert_eq!(s.diagnostic_read(JOINT_IDS[i]).unwrap().pid, Some(pid));
            let cmd = d::Call::Command(d::Command {
                session_id: 1,
                request_id: 4,
                positions_rad: BTreeMap::from([(name.clone(), 0.1)]),
                valid_for_ms: 500,
            });
            let r = req(&mut c, &mut s, cmd.clone());
            assert!(r.accepted);
            assert_eq!(r.outcomes[0].status, "sent");
            assert_eq!(req(&mut c, &mut s, cmd), r);
            assert!(
                req(
                    &mut c,
                    &mut s,
                    d::Call::Release(d::Session {
                        session_id: 1,
                        request_id: 5
                    })
                )
                .accepted
            );
            assert_eq!(s.diagnostic_read(JOINT_IDS[i]).unwrap().torque, Some(false));
            assert!(c.exit_owed);
            c.finish_exit();
            assert!(!c.blocked());
        }
    }
    #[test]
    fn expiry_does_not_cut_unrelated_motor_and_superseded_deadline() {
        let (mut c, mut s) = setup();
        let now = Instant::now();
        acquired(&mut c, now);
        for (i, id) in [(3, 2), (9, 3)] {
            assert!(
                req(
                    &mut c,
                    &mut s,
                    d::Call::Torque(d::Torque {
                        session_id: 1,
                        request_id: id,
                        joints: vec![JOINT_NAMES[i].into()],
                        on: true
                    })
                )
                .accepted
            );
        }
        c.deadlines[9] = Some(now - Duration::from_millis(1));
        c.tick(&mut s, now, true);
        assert_eq!(s.diagnostic_read(JOINT_IDS[9]).unwrap().torque, Some(false));
        assert_eq!(s.diagnostic_read(JOINT_IDS[3]).unwrap().torque, Some(true));
        assert!(
            req(
                &mut c,
                &mut s,
                d::Call::Command(d::Command {
                    session_id: 1,
                    request_id: 4,
                    positions_rad: BTreeMap::from([(JOINT_NAMES[3].into(), 0.1)]),
                    valid_for_ms: 60000
                })
            )
            .accepted
        );
        assert!(
            req(
                &mut c,
                &mut s,
                d::Call::Command(d::Command {
                    session_id: 1,
                    request_id: 5,
                    positions_rad: BTreeMap::from([(JOINT_NAMES[3].into(), 0.2)]),
                    valid_for_ms: 60000
                })
            )
            .accepted
        );
        c.tick(&mut s, now + Duration::from_secs(1), true);
        assert_eq!(s.diagnostic_read(JOINT_IDS[3]).unwrap().torque, Some(true));
    }
    #[test]
    fn lease_and_disconnect_close_without_resuming() {
        let (mut c, mut s) = setup();
        let now = Instant::now();
        acquired(&mut c, now);
        c.session.as_mut().unwrap().lease = now;
        s.diagnostic_torque(JOINT_IDS[0], true).unwrap();
        c.controlled[0] = true;
        c.tick(&mut s, now, true);
        assert!(c.session.is_none());
        assert_eq!(s.diagnostic_read(JOINT_IDS[0]).unwrap().torque, Some(false));
        acquired(&mut c, now);
        c.shared.owner.store(1, Ordering::Release);
        let guard = Connection {
            shared: c.shared.clone(),
            id: 1,
        };
        drop(guard);
        c.tick(&mut s, now, true);
        assert!(c.session.is_none());
    }
    #[test]
    fn unknown_off_stays_inhibited_and_administrative_cancel_recovers() {
        let shared = Arc::new(Shared::new());
        let mut c = Controller::new(shared);
        let mut io = FakeIo::new();
        io.diagnostic_fail_id = Some(JOINT_IDS[2]);
        let mut s = Safety::new(io, SafetyConfig::default());
        acquired(&mut c, Instant::now());
        c.controlled[2] = true;
        let r = req(
            &mut c,
            &mut s,
            d::Call::Release(d::Session {
                session_id: 1,
                request_id: 2,
            }),
        );
        assert!(!r.accepted);
        assert!(c.session.is_none());
        assert!(c.inhibit[2]);
        assert!(c.blocked());
        let mut working = Safety::new(FakeIo::new(), SafetyConfig::default());
        assert!(
            req(
                &mut c,
                &mut working,
                d::Call::Cancel(d::Cancel { request_id: 1 })
            )
            .accepted
        );
        assert!(c.exit_owed);
        c.finish_exit();
        assert!(!c.blocked());
    }
    #[test]
    fn invalid_selection_pid_and_goal_do_not_write() {
        let (mut c, mut s) = setup();
        acquired(&mut c, Instant::now());
        assert!(
            !req(
                &mut c,
                &mut s,
                d::Call::Torque(d::Torque {
                    session_id: 1,
                    request_id: 2,
                    joints: vec!["mouth".into(), "mouth".into()],
                    on: true
                })
            )
            .accepted
        );
        assert_eq!(s.diagnostic_read(34).unwrap().torque, Some(false));
        assert!(
            !req(
                &mut c,
                &mut s,
                d::Call::Gains(d::Gains {
                    session_id: 1,
                    request_id: 3,
                    gains: BTreeMap::from([(
                        "mouth".into(),
                        d::Pid {
                            p: 65535,
                            i: 0,
                            d: 0
                        }
                    )])
                })
            )
            .accepted
        );
        assert!(
            !req(
                &mut c,
                &mut s,
                d::Call::Command(d::Command {
                    session_id: 1,
                    request_id: 4,
                    positions_rad: BTreeMap::from([("mouth".into(), f64::NAN)]),
                    valid_for_ms: 1
                })
            )
            .accepted
        );
    }
    #[test]
    fn partial_pid_failure_does_not_cut_other_torque() {
        let (mut c, _) = setup();
        let mut io = FakeIo::new();
        io.diagnostic_fail_id = Some(34);
        let mut s = Safety::new(io, SafetyConfig::default());
        acquired(&mut c, Instant::now());
        let pid = d::Pid { p: 123, i: 1, d: 2 };
        let r = req(
            &mut c,
            &mut s,
            d::Call::Gains(d::Gains {
                session_id: 1,
                request_id: 2,
                gains: BTreeMap::from([("mouth".into(), pid), ("head_roll".into(), pid)]),
            }),
        );
        assert!(!r.accepted);
        assert_eq!(s.diagnostic_read(33).unwrap().pid, Some(pid));
        assert!(c.session.is_some());
    }
    #[test]
    fn replay_cache_never_reexecutes_evicted_requests() {
        let (mut c, mut s) = setup();
        acquired(&mut c, Instant::now());
        for n in 2..270 {
            assert!(
                req(
                    &mut c,
                    &mut s,
                    d::Call::Stop(d::Session {
                        session_id: 1,
                        request_id: n
                    })
                )
                .accepted
            );
        }
        assert_eq!(
            req(
                &mut c,
                &mut s,
                d::Call::Stop(d::Session {
                    session_id: 1,
                    request_id: 2
                })
            )
            .reason
            .as_deref(),
            Some("request_expired")
        );
        assert_eq!(c.session.as_ref().unwrap().cache.len(), CACHE);
        assert_eq!(
            req(
                &mut c,
                &mut s,
                d::Call::Keepalive(d::Keepalive {
                    session_id: 1,
                    sequence: 0
                })
            )
            .reason
            .as_deref(),
            Some("stale_sequence")
        );
    }
    #[test]
    fn relay_and_unknown_peers_are_denied() {
        assert!(!authorized(None, 0, &[1000], &[20]));
        assert!(!authorized(Some(20), 0, &[20], &[20]));
        assert!(authorized(Some(1000), 0, &[1000], &[20]));
    }
    #[test]
    fn admission_waits_for_fresh_torque_off_scan() {
        let (mut c, mut s) = setup();
        let (tx, mut rx) = oneshot::channel();
        c.shared
            .tx
            .try_send(Pending {
                connection: 1,
                call: d::Call::Acquire(d::Acquire {
                    request_id: 1,
                    mode: "maintenance_raw".into(),
                    acknowledge_raw: true,
                }),
                reply: tx,
            })
            .unwrap();
        // Use actual monotonic clock rather than advancing Instant while cached read timestamps stay real.
        for _ in 0..18 {
            c.tick(&mut s, Instant::now(), true);
        }
        assert!(rx.try_recv().unwrap().accepted);
        assert!(c.blocked());
    }
    #[test]
    fn unsafe_or_unknown_drive_modes_refuse_admission_without_writes() {
        // An all-off snapshot alone is insufficient: goal seeding/exit restore can energize
        // native Torque On by Goal Update motors. Every selection, including mouth, is checked.
        for (i, joint) in JOINT_NAMES.iter().enumerate() {
            for mode in [None, Some(0x08)] {
                let shared = Arc::new(Shared::new());
                let mut c = Controller::new(shared);
                let mut io = FakeIo::new();
                io.diagnostic_drive_modes[i] = mode;
                let writes = io.diagnostic_write_attempts.clone();
                let mut s = Safety::new(io, SafetyConfig::default());
                let (tx, mut rx) = oneshot::channel();
                c.shared
                    .tx
                    .try_send(Pending {
                        connection: 1,
                        call: d::Call::Acquire(d::Acquire {
                            request_id: 1,
                            mode: "maintenance_raw".into(),
                            acknowledge_raw: true,
                        }),
                        reply: tx,
                    })
                    .unwrap();
                for _ in 0..18 {
                    c.tick(&mut s, Instant::now(), true);
                }
                let reply = rx.try_recv().unwrap();
                assert!(!reply.accepted);
                assert_eq!(reply.reason.as_deref(), Some("unsupported_drive_mode"));
                assert_eq!(reply.outcomes[0].joint, *joint);
                assert!(c.session.is_none());
                assert_eq!(c.shared.owner.load(Ordering::Acquire), 0);
                assert_eq!(writes.load(Ordering::Relaxed), 0);
            }
        }
    }
    #[test]
    fn fresh_unsafe_drive_mode_refuses_goal_seeding_without_writes() {
        // A fresh selected read must still guard the seed even if configuration changed after
        // admission. This models that state without introducing any EEPROM-writing API.
        for mode in [None, Some(0x08)] {
            let shared = Arc::new(Shared::new());
            let mut c = Controller::new(shared);
            let mut io = FakeIo::new();
            io.diagnostic_drive_modes[9] = mode;
            let writes = io.diagnostic_write_attempts.clone();
            let mut s = Safety::new(io, SafetyConfig::default());
            acquired(&mut c, Instant::now());
            let reply = req(
                &mut c,
                &mut s,
                d::Call::Torque(d::Torque {
                    session_id: 1,
                    request_id: 2,
                    joints: vec!["mouth".into()],
                    on: true,
                }),
            );
            assert!(!reply.accepted);
            assert_eq!(
                reply.outcomes[0].error.as_deref(),
                Some("unsupported_drive_mode")
            );
            assert_eq!(writes.load(Ordering::Relaxed), 0);
            assert!(!c.controlled[9]);
        }
    }
    #[test]
    fn a_torqued_or_busy_robot_is_not_adopted() {
        let (mut c, mut s) = setup();
        s.diagnostic_torque(34, true).unwrap();
        let (tx, mut rx) = oneshot::channel();
        c.shared
            .tx
            .try_send(Pending {
                connection: 1,
                call: d::Call::Acquire(d::Acquire {
                    request_id: 1,
                    mode: "maintenance_raw".into(),
                    acknowledge_raw: true,
                }),
                reply: tx,
            })
            .unwrap();
        for _ in 0..18 {
            c.tick(&mut s, Instant::now(), true);
        }
        assert_eq!(rx.try_recv().unwrap().reason.as_deref(), Some("torque_on"));
        assert_eq!(s.diagnostic_read(34).unwrap().torque, Some(true));
    }
    #[test]
    fn priority_stop_discards_old_work_even_when_mailbox_is_full() {
        let (mut c, mut s) = setup();
        let now = Instant::now();
        acquired(&mut c, now);
        s.diagnostic_torque(34, true).unwrap();
        c.controlled[9] = true;
        let mut receivers = vec![];
        for n in 2..18 {
            let (tx, rx) = oneshot::channel();
            receivers.push(rx);
            c.shared
                .tx
                .try_send(Pending {
                    connection: 1,
                    call: d::Call::Command(d::Command {
                        session_id: 1,
                        request_id: n,
                        positions_rad: BTreeMap::from([("mouth".into(), 0.5)]),
                        valid_for_ms: 1000,
                    }),
                    reply: tx,
                })
                .unwrap();
        }
        let (tx, mut rx) = oneshot::channel();
        c.shared
            .enqueue(Pending {
                connection: 1,
                call: d::Call::Stop(d::Session {
                    session_id: 1,
                    request_id: 18,
                }),
                reply: tx,
            })
            .unwrap();
        c.tick(&mut s, now, true);
        assert!(rx.try_recv().unwrap().accepted);
        assert_eq!(s.diagnostic_read(34).unwrap().torque, Some(false));
        assert_eq!(s.diagnostic_read(34).unwrap().goal_rad, Some(0.0));
        for mut r in receivers {
            assert_eq!(r.try_recv().unwrap().reason.as_deref(), Some("cancelled"));
        }
    }
    #[test]
    fn rejected_new_request_id_cannot_be_repurposed_later() {
        let (mut c, mut s) = setup();
        acquired(&mut c, Instant::now());
        let call = d::Call::Command(d::Command {
            session_id: 1,
            request_id: 2,
            positions_rad: BTreeMap::from([("mouth".into(), 0.2)]),
            valid_for_ms: 1000,
        });
        let r = req(&mut c, &mut s, call.clone());
        assert!(!r.accepted);
        s.diagnostic_torque(34, true).unwrap();
        c.controlled[9] = true;
        c.read(&mut s, 9);
        assert_eq!(req(&mut c, &mut s, call), r);
        let changed = d::Call::Torque(d::Torque {
            session_id: 1,
            request_id: 2,
            joints: vec!["mouth".into()],
            on: false,
        });
        assert_eq!(
            req(&mut c, &mut s, changed).reason.as_deref(),
            Some("request_conflict")
        );
    }
    #[test]
    fn repeated_release_does_not_terminate_a_new_session() {
        let (mut c, mut s) = setup();
        acquired(&mut c, Instant::now());
        let release = d::Call::Release(d::Session {
            session_id: 1,
            request_id: 2,
        });
        let r = req(&mut c, &mut s, release.clone());
        assert!(r.accepted);
        assert_eq!(req(&mut c, &mut s, release.clone()), r);
        acquired(&mut c, Instant::now());
        c.session.as_mut().unwrap().id = 2;
        assert_eq!(req(&mut c, &mut s, release), r);
        assert_eq!(c.session.as_ref().unwrap().id, 2);
    }
    #[test]
    fn failed_readback_cannot_confirm_pid_or_goal_and_expiry_does_not_hammer_bus() {
        let (mut c, _) = setup();
        let mut io = FakeIo::new();
        io.diagnostic_fail_after_reads = Some(1);
        let mut safety = Safety::new(io, SafetyConfig::default());
        acquired(&mut c, Instant::now());
        c.read(&mut safety, 9);
        assert!(c.motors[9].pid.is_some());
        c.read(&mut safety, 9);
        assert!(c.motors[9].pid.is_none());
        assert!(c.motors[9].goal_rad.is_none());
        assert!(c.motors[9].position_counts.is_none());
        c.controlled[9] = true;
        c.deadlines[9] = Some(Instant::now());
        c.tick(&mut safety, Instant::now(), true);
        assert!(c.inhibit[9]);
        assert!(c.deadlines[9].is_none());
    }
    #[test]
    fn unconfirmed_enable_is_cut_and_latched() {
        let (mut c, _) = setup();
        let mut io = FakeIo::new();
        io.diagnostic_fail_after_reads = Some(2);
        let mut s = Safety::new(io, SafetyConfig::default());
        acquired(&mut c, Instant::now());
        let r = req(
            &mut c,
            &mut s,
            d::Call::Torque(d::Torque {
                session_id: 1,
                request_id: 2,
                joints: vec!["mouth".into()],
                on: true,
            }),
        );
        assert!(!r.accepted);
        assert!(c.inhibit[9]);
        assert!(c.controlled[9]);
    }
    #[test]
    fn refresh_admission_is_rate_limited_before_enqueueing_and_normal_motion_is_not_scanned() {
        let (mut c, _) = setup();
        assert!(c.shared.admit_refresh(2_000_000_000));
        assert!(!c.shared.admit_refresh(2_000_000_001));
        assert!(c.shared.admit_refresh(3_000_000_000));
        let mut io = FakeIo::new();
        io.diagnostic_fail_id = Some(34);
        let mut s = Safety::new(io, SafetyConfig::default());
        for _ in 0..20 {
            c.tick(&mut s, Instant::now(), false);
        }
        assert!(c.motors.iter().all(|m| m.observed_at_ns == 0));
    }
    #[test]
    fn exit_is_owed_until_integration_acknowledges_even_if_an_acquire_races() {
        let (mut c, mut s) = setup();
        acquired(&mut c, Instant::now());
        assert!(
            req(
                &mut c,
                &mut s,
                d::Call::Release(d::Session {
                    session_id: 1,
                    request_id: 2
                })
            )
            .accepted
        );
        assert!(c.blocked());
        assert!(c.exit_owed);
        let (tx, mut rx) = oneshot::channel();
        c.shared
            .tx
            .try_send(Pending {
                connection: 2,
                call: d::Call::Acquire(d::Acquire {
                    request_id: 1,
                    mode: "maintenance_raw".into(),
                    acknowledge_raw: true,
                }),
                reply: tx,
            })
            .unwrap();
        assert!(c.tick(&mut s, Instant::now(), true));
        assert_eq!(rx.try_recv().unwrap().reason.as_deref(), Some("busy"));
        assert!(c.tick(&mut s, Instant::now(), true));
        assert!(c.shared.blocked.load(Ordering::Acquire));
        c.finish_exit();
        assert!(!c.blocked());
        assert!(!c.shared.blocked.load(Ordering::Acquire));
    }
    #[test]
    fn non_owner_cannot_replace_a_pending_release() {
        let (mut c, _) = setup();
        acquired(&mut c, Instant::now());
        let (tx, _rx) = oneshot::channel();
        c.shared
            .enqueue(Pending {
                connection: 1,
                call: d::Call::Release(d::Session {
                    session_id: 1,
                    request_id: 2,
                }),
                reply: tx,
            })
            .unwrap();
        let (tx, _rx) = oneshot::channel();
        assert_eq!(
            c.shared.enqueue(Pending {
                connection: 2,
                call: d::Call::Stop(d::Session {
                    session_id: 1,
                    request_id: 2
                }),
                reply: tx
            }),
            Err("wrong_session")
        );
        assert!(matches!(
            c.shared.priority.lock().unwrap().as_ref().unwrap().call,
            d::Call::Release(_)
        ));
    }
}
