//! The raw input tap: a second reader of the same pad, on a socket of its own.
//!
//! ## Why `padd` grew a socket to serve one read-only stream
//!
//! One question about a gamepad cannot be answered from anywhere else in this system. `padd` polls
//! the last known stick value and sends it at a steady 50 Hz, so a radio that has stopped
//! delivering reports still produces perfectly fresh intents: `robotd` sees a live driver, the
//! deadman never fires, and the robot keeps walking on a command nobody is still giving. Every
//! surface downstream — `robot.state`, the monitor's `requested` column, the journal — shows a
//! healthy robot, because from their side it is one.
//!
//! The evidence lives one layer below `padd`: the event stream itself, where a report that never
//! arrived leaves a hole in the cadence. So this hands that stream out unaltered rather than
//! summarising it, and lets whoever is investigating do the arithmetic.
//!
//! ## Why it reads the device a second time instead of forwarding what gilrs gives it
//!
//! `Gilrs::next_event` is not the raw stream and cannot be made into one. It applies three filters
//! by default — `axis_dpad_to_button`, `Jitter`, `deadzone` — which rewrite values, drop small
//! movements, and swallow whole events; `gilrs-core` turns `SYN_DROPPED` into an internal resync
//! flag that never reaches a consumer; and neither `SYN_REPORT` nor `MSC_SCAN` survives the trip.
//! Every one of those is a thing someone chasing an unreliable link needs to see.
//!
//! Opening the node twice costs nothing and takes nothing away: an evdev reader gets its own queue,
//! so this cannot starve gilrs of an event, and `scripts/pad-link-test.sh` has been reading the same
//! node alongside `padd` since before this existed. The node comes from gilrs itself
//! ([`gilrs::LinuxGamepadExt::devpath`]), which is the only way to be sure this is watching the pad
//! that is actually driving — one Xbox controller registers several input devices, and the first
//! one in `/proc/bus/input/devices` is a media-key keyboard that never sends anything.
//!
//! ## What it costs while nobody is watching
//!
//! Nothing. The device is not opened until a subscriber connects, and it is closed again after the
//! first report following the last one leaving — the same bargain `robotd` strikes by only
//! assembling a `robot.state` frame when someone is subscribed. A pad at rest is silent, so a
//! parked reader is not a wakeup either.
//!
//! ## The pad's IMU, when it has one
//!
//! Some pads carry an inertial unit. The "Pro Controller" Switch clones do, and the kernel's
//! `hid-nintendo` exposes it as a **second** evdev node under the same HID device, flagged
//! `INPUT_PROP_ACCELEROMETER` and named "... IMU". gilrs never opens it — an accelerometer's `ABS_X`
//! is not a stick — so `padd` drives from the other node and this tap reads the IMU beside it, on a
//! reader of its own with the same lifecycle: open while somebody subscribes, closed when nobody
//! does. It is found by walking sysfs from the node gilrs named to its HID parent and back down to a
//! sibling with the accelerometer property; a pad without one has no such sibling and nothing here
//! changes for it.
//!
//! Unlike a stick it is never silent: the clone streams several hundred samples a second whether
//! or not anyone touches it. That is why a sample is six integers rather than an event list, why it
//! has its own drop counter, and why the IMU is not opened at all unless someone is watching —
//! or unless `padd` itself is, which is the second reader below.
//!
//! ## The IMU as a control, not only a stream
//!
//! With `[imu_head] enabled`, `padd` poses the robot's head from the pad's tilt. The attitude that
//! needs comes from the same node the tap streams, so rather than a second reader on the same
//! device this reader runs a `pad_imu::Imu` filter over every batch it reads and the main loop asks
//! for the attitude ([`Tap::attitude`]). [`Tap::imu_control`] is the main loop saying "keep the
//! IMU open for me": the node then stays open with or without a subscriber, and closes again when
//! the feature goes off. One reader, one filter, and `robotctl monitor` draws the same attitude
//! `padd` steers by.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

use duck_ipc_proto as proto;
use evdev::raw_stream::RawDevice;
use evdev::{
    AbsoluteAxisCode, AttributeSet, EventType, KeyCode, MiscCode, RelativeAxisCode,
    SynchronizationCode,
};
use gilrs::LinuxGamepadExt;

/// Reports a subscriber may fall behind by before frames start being dropped for it.
///
/// Two seconds of a busy pad. Generous, because the cost of a queue is memory and the cost of a
/// drop is a hole in the very measurement this exists to make — but bounded, because the alternative
/// is a slow client turning into unbounded memory on a robot. A drop is counted and reported rather
/// than hidden ([`proto::PadFrame::socket_dropped`]).
const QUEUE: usize = 256;

/// The group that may read the tap, matching `robotd`'s own socket.
///
/// Deliberately the same one: whoever may watch `robot.state` may watch the pad driving it, and
/// nobody else gains anything by this existing.
const GROUP: &str = "robot";

/// Socket mode. Same reasoning as every other socket here — the group decides who may ask.
const SOCKET_MODE: u32 = 0o660;

/// How long to wait before opening the node again after a stream ended.
///
/// It bounds a spin. Two of the ways a stream ends leave the state that started it unchanged — the
/// node cannot be opened at all (no `input` group, so every attempt fails identically), and a device
/// that is already gone when it is opened — and without this the reader would reopen, fail, and
/// reopen again as fast as the kernel could refuse it. On a robot being driven, by the tool brought
/// in to find out why driving is unreliable, which is the same trap `pad-link-test.sh` documents
/// hitting in its watch loop.
///
/// A quarter of a second: four attempts a second is nothing, and nobody notices it when a pad that
/// really has come back takes that long to be picked up.
const REOPEN_AFTER: std::time::Duration = std::time::Duration::from_millis(250);

/// The tap, as the main loop holds it.
pub struct Tap {
    shared: Arc<Shared>,
}

/// Shared between the accept loop, each subscriber's writer, and the reader.
struct Shared {
    state: Mutex<State>,
    /// Signalled whenever the reader might have work: a pad appeared, or someone subscribed.
    wake: Condvar,
    /// The IMU's attitude, kept by the IMU reader for the main loop. `None` while no IMU is
    /// open. Its own lock, not `state`'s: the main loop reads it fifty times a second and must not
    /// queue behind a subscriber's send.
    attitude: Mutex<Option<pad_imu::Imu>>,
}

struct State {
    /// The node to read, as gilrs named it. `None` when no pad is connected.
    wanted: Option<PathBuf>,
    /// The pad's IMU node, when the pad has one — see [`imu_sibling`]. Resolved once per pad node
    /// rather than per tick, because it is a walk through sysfs.
    wanted_imu: Option<PathBuf>,
    /// Does `padd` itself want the IMU read, for head control? A second reason to hold the node
    /// open, beside a subscriber.
    control: bool,
    subscribers: Vec<Subscriber>,
    /// The `Attached` report for the device currently open.
    ///
    /// Kept so a subscriber arriving mid-stream is told what it is looking at immediately, rather
    /// than having to wait for the pad to be unplugged and put back to find out.
    attached: Option<Arc<proto::PadReport>>,
    /// The `ImuAttached` report for the IMU currently open, for the same reason.
    imu_attached: Option<Arc<proto::PadReport>>,
}

struct Subscriber {
    reports: SyncSender<Arc<proto::PadReport>>,
    /// Frames dropped because this subscriber's queue was full. Its writer stamps them into the
    /// next frame it does send, so a slow client cannot read as a stalled radio.
    dropped: Arc<AtomicU64>,
    /// IMU samples dropped for the same reason, counted apart: a lost sample says nothing about
    /// the stick reports, and stamping it onto a `PadFrame` would say it did.
    dropped_imu: Arc<AtomicU64>,
}

impl Tap {
    /// Bind the socket and start serving.
    ///
    /// Fails only where the socket cannot be created. The caller is expected to carry on driving
    /// without a tap in that case: `padd`'s job is to make the robot drivable, and a debug facility
    /// that refuses to be optional would be a way for this file to stop a robot.
    pub fn serve(socket: &Path) -> std::io::Result<Self> {
        // The directory is `RuntimeDirectory=padd`, which systemd has already made on a board. Tried
        // anyway for `padd` run by hand, and its failure left to `bind` to report against the full
        // path — which is the message that actually helps.
        if let Some(parent) = socket.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        // systemd removes the runtime directory when the unit stops, so a stale socket only happens
        // to a `padd` killed outside its unit. Removing it is still right: the alternative is a
        // daemon that refuses to start because of a file whose owner is gone.
        let _ = std::fs::remove_file(socket);

        let listener = UnixListener::bind(socket)?;
        std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))?;
        if let Err(e) = give_to_group(socket, GROUP) {
            // Not fatal, and said out loud with what it means: the socket still exists, and only
            // `padd` itself and root can read it. On a board this is a broken install; on a laptop
            // it is a machine with no `robot` group, which is ordinary.
            tracing::warn!(
                error = %e, group = GROUP, socket = %socket.display(),
                "the tap's socket stays private to padd — nothing else can read the pad stream"
            );
        }

        let shared = Arc::new(Shared {
            state: Mutex::new(State {
                wanted: None,
                wanted_imu: None,
                control: false,
                subscribers: Vec::new(),
                attached: None,
                imu_attached: None,
            }),
            wake: Condvar::new(),
            attitude: Mutex::new(None),
        });

        let accepting = Arc::clone(&shared);
        thread::Builder::new()
            .name("pad-tap-accept".to_owned())
            .spawn(move || accept(listener, &accepting))?;

        let reading = Arc::clone(&shared);
        thread::Builder::new()
            .name("pad-tap-read".to_owned())
            .spawn(move || read(&reading))?;

        let reading_imu = Arc::clone(&shared);
        thread::Builder::new()
            .name("pad-tap-imu".to_owned())
            .spawn(move || read_imu(&reading_imu))?;

        Ok(Self { shared })
    }

    /// Read this pad from now on.
    ///
    /// Safe to call every tick: it compares before it disturbs anything, because the alternative is
    /// tearing the reader down and rebuilding it 50 times a second.
    ///
    /// It takes the gamepad rather than a path so the node can only come from gilrs — the one place
    /// that knows which of a pad's several input devices is the one being driven from.
    pub fn watch(&self, pad: &gilrs::Gamepad<'_>) {
        let node = pad.devpath().to_path_buf();
        // The sysfs walk for the IMU happens only when the pad's node changes, which is the
        // comparison `want` makes anyway — so it is made here first, before the walk.
        if self.shared.lock().wanted.as_deref() == Some(node.as_path()) {
            return;
        }
        let imu = imu_sibling(Path::new("/sys"), Path::new("/dev"), &node);
        if let Some(imu) = imu.as_deref() {
            tracing::info!(node = %node.display(), imu = %imu.display(), "the pad has an IMU");
        }
        self.want(Some(node), imu);
    }

    /// No pad. The readers close their devices and say so to whoever is watching.
    pub fn idle(&self) {
        self.want(None, None);
    }

    /// Keep the pad's IMU open for `padd`'s own use, or stop. Safe to call every tick.
    pub fn imu_control(&self, on: bool) {
        let mut state = self.shared.lock();
        if state.control == on {
            return;
        }
        state.control = on;
        drop(state);
        self.shared.wake.notify_all();
    }

    /// Does the connected pad have an IMU at all? True as soon as the node is known, before it is
    /// open — the question "can Y mean IMU head control on this pad" needs answering on the press.
    pub fn has_imu(&self) -> bool {
        self.shared.lock().wanted_imu.is_some()
    }

    /// The pad's attitude, body → world, `w` first. `None` while no IMU is open or before its
    /// first believable sample.
    pub fn attitude(&self) -> Option<[f32; 4]> {
        self.shared
            .attitude
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
            .and_then(pad_imu::Imu::quaternion)
    }

    fn want(&self, node: Option<PathBuf>, imu: Option<PathBuf>) {
        let mut state = self.shared.lock();
        if state.wanted == node && state.wanted_imu == imu {
            return;
        }
        state.wanted = node;
        state.wanted_imu = imu;
        drop(state);
        self.shared.wake.notify_all();
    }
}

/// The IMU node that belongs to the same pad as `node`, if the pad has one.
///
/// A pad's several input devices share one HID parent in sysfs, and only the inertial one carries
/// `INPUT_PROP_ACCELEROMETER` (bit 6 of `properties`). So: from `/sys/class/input/eventN/device`
/// up one to the HID device, then over every other `eventM` whose HID device is the same one and
/// whose properties say accelerometer. The first such node is it — a pad has one IMU.
///
/// `sysfs` and `dev` are parameters so a test can lay the tree out in a temporary directory;
/// production passes `/sys` and `/dev`.
fn imu_sibling(sysfs: &Path, dev: &Path, node: &Path) -> Option<PathBuf> {
    let event = node.file_name()?.to_owned();
    let class = sysfs.join("class/input");
    let hid_parent = |event: &std::ffi::OsStr| {
        std::fs::canonicalize(class.join(event).join("device/device")).ok()
    };
    let parent = hid_parent(&event)?;

    let mut candidates: Vec<PathBuf> = std::fs::read_dir(&class)
        .ok()?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with("event"))
        })
        .collect();
    // Sorted so "the first" is the same one on every call rather than directory order.
    candidates.sort();

    for candidate in candidates {
        let name = candidate.file_name()?.to_owned();
        if name == event {
            continue;
        }
        if hid_parent(&name).as_deref() != Some(parent.as_path()) {
            continue;
        }
        let properties = std::fs::read_to_string(candidate.join("device/properties")).ok()?;
        if has_accelerometer_property(&properties) {
            return Some(dev.join("input").join(name));
        }
    }
    None
}

/// Does a sysfs `properties` bitmask carry `INPUT_PROP_ACCELEROMETER`?
///
/// The file is hex words, most significant first, space separated — `40` for a device with only
/// that bit, `0` for a pad. Bit 6, from `linux/input.h`.
fn has_accelerometer_property(properties: &str) -> bool {
    // The property bits fit one word on every kernel, so the last word is the one that matters
    // and any earlier ones are zero padding.
    let Some(low) = properties.split_whitespace().last() else {
        return false;
    };
    u64::from_str_radix(low, 16).is_ok_and(|bits| bits & (1 << 6) != 0)
}

impl Shared {
    /// The state, whether or not a thread panicked while holding it.
    ///
    /// A poisoned lock must not take the tap down with it: the data behind it is a subscriber list
    /// and a path, neither of which a panic can leave half-written into an invalid shape.
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Wait until there is both a pad to read and somebody to read it for.
    fn wait_for_work(&self) -> PathBuf {
        let mut state = self.lock();
        loop {
            if let Some(node) = state.wanted.clone()
                && !state.subscribers.is_empty()
            {
                return node;
            }
            state = self
                .wake
                .wait(state)
                .unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// Should the reader let go of the node it has open?
    fn done_with(&self, node: &Path) -> Option<&'static str> {
        let state = self.lock();
        if state.subscribers.is_empty() {
            // Said as a reason rather than dropped silently: a client that reconnects wants to know
            // why the previous stream ended, and "nobody was watching" is not a fault.
            return Some("nobody is watching any more");
        }
        if state.wanted.as_deref() != Some(node) {
            return Some("the pad changed");
        }
        None
    }

    /// The IMU reader's [`Self::wait_for_work`]: a pad with an IMU, and somebody to read it for —
    /// a subscriber, or `padd` itself.
    fn wait_for_imu_work(&self) -> PathBuf {
        let mut state = self.lock();
        loop {
            if let Some(node) = state.wanted_imu.clone()
                && (!state.subscribers.is_empty() || state.control)
            {
                return node;
            }
            state = self
                .wake
                .wait(state)
                .unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// The IMU reader's [`Self::done_with`].
    fn done_with_imu(&self, node: &Path) -> Option<&'static str> {
        let state = self.lock();
        if state.subscribers.is_empty() && !state.control {
            return Some("nobody is watching any more");
        }
        if state.wanted_imu.as_deref() != Some(node) {
            return Some("the pad changed");
        }
        None
    }

    fn attach_imu(&self, device: proto::PadImuDevice) {
        *self.attitude_slot() = Some(pad_imu::Imu::new(device.clone()));
        let report = Arc::new(proto::PadReport::ImuAttached {
            device: Box::new(device),
        });
        let mut state = self.lock();
        state.imu_attached = Some(Arc::clone(&report));
        state.send(&report);
    }

    fn detach_imu(&self, why: String) {
        *self.attitude_slot() = None;
        let mut state = self.lock();
        state.imu_attached = None;
        state.send(&Arc::new(proto::PadReport::ImuDetached { why }));
    }

    /// One batch: into the filter first, then out to whoever is watching.
    fn samples(&self, samples: Vec<proto::PadImuSample>) {
        let batch = proto::PadImuBatch {
            samples,
            socket_dropped: 0,
        };
        if let Some(imu) = self.attitude_slot().as_mut() {
            imu.absorb(&batch);
        }
        self.lock().send(&Arc::new(proto::PadReport::Imu(batch)));
    }

    fn attitude_slot(&self) -> MutexGuard<'_, Option<pad_imu::Imu>> {
        self.attitude.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn attach(&self, device: proto::PadInputDevice) {
        let report = Arc::new(proto::PadReport::Attached {
            device: Box::new(device),
        });
        let mut state = self.lock();
        state.attached = Some(Arc::clone(&report));
        state.send(&report);
    }

    fn detach(&self, why: String) {
        let mut state = self.lock();
        state.attached = None;
        state.send(&Arc::new(proto::PadReport::Detached { why }));
    }

    fn frame(&self, frame: proto::PadFrame) {
        self.frame_report(&Arc::new(proto::PadReport::Frame(frame)));
    }

    /// Hand out one already-built report. Split from [`Self::frame`] so a test can send the same
    /// report many times without rebuilding it.
    fn frame_report(&self, report: &Arc<proto::PadReport>) {
        self.lock().send(report);
    }

    /// Add a subscriber, seeded with the device — and the IMU, if any — it is about to see
    /// reports from.
    ///
    /// Hands back the counters of what gets dropped for it, which only its own writer may clear.
    fn subscribe(&self) -> (Receiver<Arc<proto::PadReport>>, Dropped) {
        let (reports, rx) = sync_channel(QUEUE);
        let mut state = self.lock();
        if let Some(attached) = state.attached.clone() {
            // Cannot fail: the queue is empty and this is the first thing in it.
            let _ = reports.try_send(attached);
        }
        if let Some(attached) = state.imu_attached.clone() {
            let _ = reports.try_send(attached);
        }
        let dropped = Dropped {
            frames: Arc::new(AtomicU64::new(0)),
            samples: Arc::new(AtomicU64::new(0)),
        };
        state.subscribers.push(Subscriber {
            reports,
            dropped: Arc::clone(&dropped.frames),
            dropped_imu: Arc::clone(&dropped.samples),
        });
        drop(state);
        // A subscriber is the reader's other precondition, so it may have work now.
        self.wake.notify_all();
        (rx, dropped)
    }
}

/// One subscriber's drop counters, as its writer holds them.
struct Dropped {
    frames: Arc<AtomicU64>,
    samples: Arc<AtomicU64>,
}

impl State {
    /// Hand one report to every subscriber, dropping it for any that has fallen behind.
    ///
    /// **Never blocks.** The reader thread is the only thing that can measure the pad's cadence, and
    /// a subscriber that stalled it would corrupt the measurement it asked for — every frame after
    /// the stall would carry a gap the radio had nothing to do with.
    fn send(&mut self, report: &Arc<proto::PadReport>) {
        let is_sample = matches!(**report, proto::PadReport::Imu(_));
        self.subscribers.retain(|subscriber| {
            match subscriber.reports.try_send(Arc::clone(report)) {
                Ok(()) => true,
                Err(TrySendError::Full(_)) => {
                    let counter = if is_sample {
                        &subscriber.dropped_imu
                    } else {
                        &subscriber.dropped
                    };
                    counter.fetch_add(1, Ordering::Relaxed);
                    true
                }
                // The writer is gone: the client disconnected.
                Err(TrySendError::Disconnected(_)) => false,
            }
        });
    }
}

/// Accept subscribers forever. One thread each: a subscriber does nothing but be written to, and
/// the write must not be able to block the reader.
fn accept(listener: UnixListener, shared: &Arc<Shared>) {
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                let shared = Arc::clone(shared);
                if let Err(e) = thread::Builder::new()
                    .name("pad-tap-client".to_owned())
                    .spawn(move || subscriber(stream, &shared))
                {
                    tracing::warn!(error = %e, "cannot serve a pad tap subscriber");
                }
            }
            Err(e) => tracing::warn!(error = %e, "pad tap accept failed"),
        }
    }
}

/// One subscriber, from its request to its socket closing.
fn subscriber(stream: UnixStream, shared: &Arc<Shared>) {
    let mut writer = match stream.try_clone() {
        Ok(writer) => writer,
        Err(e) => {
            tracing::warn!(error = %e, "cannot split a pad tap connection");
            return;
        }
    };
    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    if reader.read_line(&mut line).is_err() || line.trim().is_empty() {
        return;
    }

    let request = match serde_json::from_str::<proto::Request>(&line) {
        Ok(request) => request,
        Err(e) => {
            let error = proto::Error::new(proto::code::PARSE_ERROR, e.to_string());
            let _ = write_line(&mut writer, &proto::Response::err(None, error));
            return;
        }
    };
    let id = request.id.clone();
    match request.as_call() {
        Ok(proto::Call::PadInput) => {}
        Ok(other) => {
            // Served by this socket and nothing else. A client that meant to reach `robotd` or
            // `configd` gets told which door it is knocking on rather than a silent hang.
            let error = proto::Error::new(
                proto::code::METHOD_NOT_FOUND,
                format!(
                    "padd's socket serves {} only, not {}",
                    proto::method::PAD_INPUT,
                    other.method()
                ),
            );
            let _ = write_line(&mut writer, &proto::Response::err(id, error));
            return;
        }
        Err(e) => {
            let _ = write_line(&mut writer, &proto::Response::err(id, e));
            return;
        }
    }

    // Answered before subscribing, so the reply cannot arrive after a notification it precedes.
    if let Some(id) = id {
        let accepted = proto::PadInputResult {
            accepted: true,
            reason: None,
        };
        if write_line(&mut writer, &proto::Response::ok(Some(id), &accepted)).is_err() {
            return;
        }
    }

    let (reports, dropped) = shared.subscribe();
    tracing::info!("pad tap subscriber attached");

    for report in reports {
        // Whatever this subscriber missed goes on the next frame it does get, where it belongs:
        // beside the gap it explains. Left on the counter until then, so it cannot be lost to an
        // `Attached` or a `Detached` that carries nowhere to put it.
        let stamped = match &*report {
            proto::PadReport::Frame(frame) => {
                let missed = dropped.frames.swap(0, Ordering::Relaxed);
                (missed > 0).then(|| {
                    proto::PadReport::Frame(proto::PadFrame {
                        socket_dropped: missed,
                        ..frame.clone()
                    })
                })
            }
            proto::PadReport::Imu(batch) => {
                let missed = dropped.samples.swap(0, Ordering::Relaxed);
                (missed > 0).then(|| {
                    proto::PadReport::Imu(proto::PadImuBatch {
                        socket_dropped: missed,
                        ..batch.clone()
                    })
                })
            }
            _ => None,
        };
        let note = proto::Request::notify_pad_report(stamped.as_ref().unwrap_or(&report));
        if write_line(&mut writer, &note).is_err() {
            break;
        }
    }
    tracing::info!("pad tap subscriber gone");
}

fn write_line(writer: &mut impl Write, message: &impl serde::Serialize) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)?;
    line.push(b'\n');
    writer.write_all(&line)?;
    writer.flush()
}

/// Open the wanted node, stream it, and go back to waiting when it ends. Forever.
fn read(shared: &Arc<Shared>) {
    loop {
        let node = shared.wait_for_work();
        let why = stream(shared, &node);
        tracing::info!(node = %node.display(), why, "pad tap stopped reading");
        shared.detach(why);
        thread::sleep(REOPEN_AFTER);
    }
}

/// Read one device until it ends, describing how it ended.
fn stream(shared: &Arc<Shared>, node: &Path) -> String {
    let mut device = match RawDevice::open(node) {
        Ok(device) => device,
        // The everyday cause is permissions: reading an input device needs the `input` group, which
        // `padd`'s unit grants and a hand-run `padd` usually has not been given.
        Err(e) => return format!("cannot open {}: {e}", node.display()),
    };
    shared.attach(describe(&device, node));
    tracing::info!(node = %node.display(), "pad tap reading");

    let mut seq = 0u64;
    let mut previous_us: Option<u64> = None;
    let mut events: Vec<proto::PadEvent> = Vec::new();
    // The kernel's contract after `SYN_DROPPED`: everything up to the next `SYN_REPORT` is a
    // half-report and must be thrown away, because the events that completed it are already gone.
    let mut resyncing = false;
    let mut after_drop = false;

    loop {
        let batch = match device.fetch_events() {
            Ok(batch) => batch,
            // ENODEV, every time, and it is the measurement rather than an error: this is what a
            // pad switching off or leaving range looks like from here.
            Err(e) => return format!("{} ended: {e}", node.display()),
        };

        for event in batch {
            let kind = event.event_type().0;
            let code = event.code();
            let syn = (EventType(kind) == EventType::SYNCHRONIZATION)
                .then_some(SynchronizationCode(code));

            if syn == Some(SynchronizationCode::SYN_DROPPED) {
                // Not the radio. This reader fell behind and the kernel emptied its queue, which
                // makes the gap around it unmeasurable — so the next complete report says so.
                events.clear();
                resyncing = true;
                after_drop = true;
                continue;
            }

            events.push(proto::PadEvent {
                kind,
                code,
                value: event.value(),
                name: event_name(kind, code),
            });

            if syn != Some(SynchronizationCode::SYN_REPORT) {
                continue;
            }
            // A report ends at its `SYN_REPORT`, which is kept: this is a copy of the stream, and
            // an event removed for being bookkeeping is an event someone has to take on trust.
            let report = std::mem::take(&mut events);
            if resyncing {
                resyncing = false;
                continue;
            }

            let at_us = micros(event.timestamp());
            seq += 1;
            shared.frame(proto::PadFrame {
                seq,
                at_us,
                since_us: previous_us.map(|previous| at_us as i64 - previous as i64),
                events: report,
                after_drop,
                socket_dropped: 0,
            });
            previous_us = Some(at_us);
            after_drop = false;
        }

        // Checked between reads rather than per event: the state cannot change usefully inside one
        // batch, and this is a lock per report at most.
        if let Some(why) = shared.done_with(node) {
            return why.to_owned();
        }
    }
}

/// Open the wanted IMU node, stream it, and go back to waiting when it ends. Forever.
///
/// [`read`], for the second device. Its own thread because `fetch_events` blocks, and one reader
/// cannot block on two nodes.
fn read_imu(shared: &Arc<Shared>) {
    loop {
        let node = shared.wait_for_imu_work();
        let why = stream_imu(shared, &node);
        tracing::info!(node = %node.display(), why, "pad tap stopped reading the IMU");
        shared.detach_imu(why);
        thread::sleep(REOPEN_AFTER);
    }
}

/// Read one IMU node until it ends, describing how it ended.
///
/// A sample is the six `ABS_*` values as they stand at each `SYN_REPORT`. Axes a report did not
/// mention keep their last value, which is the kernel's own contract for absolute axes — a driver
/// that leaves an unchanged axis out of a report has not zeroed it. The samples one read produced
/// go out together, as one report: that is what keeps this affordable at six hundred a second.
fn stream_imu(shared: &Arc<Shared>, node: &Path) -> String {
    let mut device = match RawDevice::open(node) {
        Ok(device) => device,
        Err(e) => return format!("cannot open {}: {e}", node.display()),
    };
    let (described, mut current) = describe_imu(&device, node);
    shared.attach_imu(described);
    tracing::info!(node = %node.display(), "pad tap reading the IMU");

    let mut seq = 0u64;
    let mut resyncing = false;
    let mut samples: Vec<proto::PadImuSample> = Vec::new();
    loop {
        let batch = match device.fetch_events() {
            Ok(batch) => batch,
            Err(e) => return format!("{} ended: {e}", node.display()),
        };
        for event in batch {
            let kind = EventType(event.event_type().0);
            let code = event.code();
            if kind == EventType::SYNCHRONIZATION {
                match SynchronizationCode(code) {
                    SynchronizationCode::SYN_DROPPED => resyncing = true,
                    SynchronizationCode::SYN_REPORT => {
                        if resyncing {
                            // The half-sample after a drop is discarded, as for the pad; the
                            // values themselves are still the newest the kernel has, so the next
                            // complete report is right.
                            resyncing = false;
                            continue;
                        }
                        seq += 1;
                        samples.push(proto::PadImuSample {
                            seq,
                            at_us: micros(event.timestamp()),
                            accel: [current[0], current[1], current[2]],
                            gyro: [current[3], current[4], current[5]],
                        });
                    }
                    _ => {}
                }
                continue;
            }
            if kind == EventType::ABSOLUTE
                && let Some(slot) = imu_slot(code)
            {
                current[slot] = event.value();
            }
        }
        if !samples.is_empty() {
            shared.samples(std::mem::take(&mut samples));
        }
        if let Some(why) = shared.done_with_imu(node) {
            return why.to_owned();
        }
    }
}

/// Which of the six sample slots an `ABS_*` code fills: X, Y, Z, RX, RY, RZ.
fn imu_slot(code: u16) -> Option<usize> {
    (code <= AbsoluteAxisCode::ABS_RZ.0).then_some(usize::from(code))
}

/// The IMU as the kernel describes it, and where its axes stand right now.
fn describe_imu(device: &RawDevice, node: &Path) -> (proto::PadImuDevice, [i32; 6]) {
    let mut current = [0i32; 6];
    let mut resolution = [0i32; 6];
    let mut max = [0i32; 6];
    if let Ok(axes) = device.get_absinfo() {
        for (code, info) in axes {
            if let Some(slot) = imu_slot(code.0) {
                current[slot] = info.value();
                resolution[slot] = info.resolution();
                max[slot] = info.maximum();
            }
        }
    }
    let described = proto::PadImuDevice {
        name: device.name().unwrap_or("unnamed IMU").to_owned(),
        node: node.display().to_string(),
        accel_per_g: resolution[0],
        gyro_per_dps: resolution[3],
        accel_max: max[0],
        gyro_max: max[3],
    };
    (described, current)
}

/// Everything about the device that a number in a frame has to be read against.
fn describe(device: &RawDevice, node: &Path) -> proto::PadInputDevice {
    let id = device.input_id();
    let axes = device
        .get_absinfo()
        .map(|axes| {
            axes.map(|(code, info)| proto::PadAxis {
                code: code.0,
                name: event_name(EventType::ABSOLUTE.0, code.0),
                min: info.minimum(),
                max: info.maximum(),
                flat: info.flat(),
                fuzz: info.fuzz(),
                value: info.value(),
            })
            .collect()
        })
        .unwrap_or_default();
    // Which buttons are already held. Asked of the kernel rather than assumed clear, so a
    // subscriber that attaches while a trigger is down is not told the pad is at rest.
    let held = device
        .get_key_state()
        .unwrap_or_else(|_| AttributeSet::new());
    let buttons = device
        .supported_keys()
        .map(|keys| {
            keys.iter()
                .map(|code| proto::PadKey {
                    code: code.0,
                    name: event_name(EventType::KEY.0, code.0),
                    pressed: held.contains(code),
                })
                .collect()
        })
        .unwrap_or_default();

    proto::PadInputDevice {
        name: device.name().unwrap_or("unnamed input device").to_owned(),
        node: node.display().to_string(),
        unique: device.unique_name().map(str::to_owned),
        bus: id.bus_type().0,
        vendor: id.vendor(),
        product: id.product(),
        axes,
        buttons,
    }
}

/// The name `linux/input-event-codes.h` gives one type/code pair — `ABS_X`, `BTN_SOUTH`,
/// `SYN_REPORT`.
///
/// evdev's `Debug` prints the constant, and `unknown key: 42` for a code it has no constant for.
/// That prose must not reach the wire: a pad with an axis nobody has mapped is exactly the pad
/// someone is reading this stream to understand, and `3:42` they can look up beats a sentence they
/// cannot grep for. Anything with a space in it is the unknown form.
fn event_name(kind: u16, code: u16) -> String {
    let named = match EventType(kind) {
        EventType::SYNCHRONIZATION => format!("{:?}", SynchronizationCode(code)),
        EventType::KEY => format!("{:?}", KeyCode(code)),
        EventType::ABSOLUTE => format!("{:?}", AbsoluteAxisCode(code)),
        EventType::RELATIVE => format!("{:?}", RelativeAxisCode(code)),
        EventType::MISC => format!("{:?}", MiscCode(code)),
        _ => String::new(),
    };
    if named.is_empty() || named.contains(' ') {
        format!("{kind}:{code}")
    } else {
        named
    }
}

/// A kernel timestamp as microseconds since the epoch.
///
/// A time before the epoch reads as zero, which cannot happen on a running kernel and is not worth
/// a second failure path in a debug stream: the frame's `since_us` would carry the absurdity into
/// view anyway.
fn micros(at: SystemTime) -> u64 {
    at.duration_since(UNIX_EPOCH)
        .map(|since| since.as_micros() as u64)
        .unwrap_or(0)
}

/// Hand the socket to a group, leaving its owner alone.
///
/// `robotd` gets the same effect from `Group=robot` in its unit, because a socket inherits its
/// creator's primary group. `padd` cannot copy that: its primary group is its own, and it reaches
/// the robot group as a supplementary one — which is enough for this, since POSIX lets the owner of
/// a file give it to any group the owner belongs to.
fn give_to_group(socket: &Path, group: &str) -> std::io::Result<()> {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;

    let name = CString::new(group).map_err(std::io::Error::other)?;
    // SAFETY: `getgrnam` reads the group database and returns a pointer into storage it owns. The
    // name is a valid C string for the length of the call, and the result is only read here — no
    // other thread of `padd` calls into the group database.
    let entry = unsafe { libc::getgrnam(name.as_ptr()) };
    if entry.is_null() {
        return Err(std::io::Error::other(format!(
            "no {group} group on this system"
        )));
    }
    // SAFETY: checked non-null immediately above, and `struct group` is fully initialised by
    // `getgrnam` when it returns a pointer at all.
    let gid = unsafe { (*entry).gr_gid };

    let path = CString::new(socket.as_os_str().as_bytes()).map_err(std::io::Error::other)?;
    // SAFETY: a valid C string path; `-1` for the owner is the documented "leave it alone".
    if unsafe { libc::chown(path.as_ptr(), u32::MAX, gid) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn a_device() -> proto::PadInputDevice {
        proto::PadInputDevice {
            name: "Xbox Wireless Controller".to_owned(),
            node: "/dev/input/event5".to_owned(),
            unique: Some("78:86:2e:bb:13:28".to_owned()),
            bus: proto::PadInputDevice::BUS_BLUETOOTH,
            vendor: 0x045e,
            product: 0x0b13,
            axes: vec![],
            buttons: vec![],
        }
    }

    fn a_frame(seq: u64) -> Arc<proto::PadReport> {
        Arc::new(proto::PadReport::Frame(proto::PadFrame {
            seq,
            at_us: 1_000_000 + seq * 8_000,
            since_us: Some(8_000),
            events: vec![],
            after_drop: false,
            socket_dropped: 0,
        }))
    }

    fn a_shared() -> Arc<Shared> {
        Arc::new(Shared {
            state: Mutex::new(State {
                wanted: None,
                wanted_imu: None,
                control: false,
                subscribers: Vec::new(),
                attached: None,
                imu_attached: None,
            }),
            wake: Condvar::new(),
            attitude: Mutex::new(None),
        })
    }

    /// A subscriber that connects while a pad is already being read is told what it is watching.
    ///
    /// Without this it would see values with no ranges to read them against and no device name,
    /// until the pad was unplugged and put back — which is the one thing nobody debugging a link
    /// wants to be told to do.
    #[test]
    fn a_subscriber_arriving_mid_stream_is_told_the_device() {
        let shared = a_shared();
        shared.attach(a_device());

        let (reports, _dropped) = shared.subscribe();
        let first = reports.try_recv().expect("seeded with the device");
        assert!(matches!(&*first, proto::PadReport::Attached { .. }));
    }

    /// A subscriber that falls behind loses reports and is *told how many*, on the next frame it
    /// does get. The reader is never blocked: a client that could stall it would corrupt the very
    /// cadence it asked for, since every gap after the stall would be its own fault.
    #[test]
    fn a_slow_subscriber_is_dropped_from_rather_than_blocking_the_reader() {
        let shared = a_shared();
        let (reports, dropped) = shared.subscribe();

        for seq in 0..(QUEUE as u64 + 10) {
            shared.frame_report(&a_frame(seq));
        }

        assert_eq!(
            dropped.frames.load(Ordering::Relaxed),
            10,
            "the overflow is counted"
        );
        assert_eq!(
            dropped.samples.load(Ordering::Relaxed),
            0,
            "and not against the IMU, which sent nothing"
        );
        assert_eq!(
            shared.lock().subscribers.len(),
            1,
            "and the subscriber is kept — it is behind, not gone"
        );
        drop(reports);
    }

    /// A subscriber whose socket closed is forgotten, so a monitor opened and shut fifty times does
    /// not leave fifty senders behind for the reader to serialise into.
    #[test]
    fn a_departed_subscriber_is_forgotten() {
        let shared = a_shared();
        let (reports, _dropped) = shared.subscribe();
        assert_eq!(shared.lock().subscribers.len(), 1);

        drop(reports);
        shared.frame_report(&a_frame(1));
        assert!(shared.lock().subscribers.is_empty());
    }

    /// The reader opens nothing until there is both a pad and somebody watching, and lets go as soon
    /// as either goes away. On a robot nobody is usually watching, and a per-report wakeup for an
    /// audience of none is the cost `robotd` refuses to pay for `robot.state` either.
    #[test]
    fn the_device_is_only_held_while_a_pad_and_a_watcher_both_exist() {
        let shared = a_shared();
        let node = Path::new("/dev/input/event5");

        let (reports, _dropped) = shared.subscribe();
        assert_eq!(
            shared.done_with(node),
            Some("the pad changed"),
            "nothing is wanted yet, so this node is not it"
        );

        shared.lock().wanted = Some(node.to_path_buf());
        assert_eq!(shared.done_with(node), None, "a pad and a watcher");

        shared.lock().wanted = Some(PathBuf::from("/dev/input/event7"));
        assert_eq!(
            shared.done_with(node),
            Some("the pad changed"),
            "a pad that came back as a different node"
        );

        shared.lock().wanted = Some(node.to_path_buf());
        drop(reports);
        shared.frame_report(&a_frame(1)); // the send that notices the departure
        assert_eq!(shared.done_with(node), Some("nobody is watching any more"));
    }

    /// Names come from the kernel's own tables, and a code with no name there becomes numbers rather
    /// than evdev's `unknown key: 42` prose — which nobody can grep for and which would land in the
    /// middle of a JSON line.
    #[test]
    fn every_code_gets_a_name_or_a_number() {
        assert_eq!(event_name(EventType::ABSOLUTE.0, 0), "ABS_X");
        assert_eq!(event_name(EventType::KEY.0, 0x130), "BTN_SOUTH");
        assert_eq!(event_name(EventType::SYNCHRONIZATION.0, 0), "SYN_REPORT");
        assert_eq!(event_name(EventType::SYNCHRONIZATION.0, 3), "SYN_DROPPED");
        assert_eq!(event_name(EventType::MISC.0, 4), "MSC_SCAN");

        // A type this build has no table for, and a code inside one it has.
        assert_eq!(event_name(0x15, 1), "21:1");
        let unnamed = event_name(EventType::ABSOLUTE.0, 0x3e);
        assert!(!unnamed.contains(' '), "no prose on the wire: {unnamed}");
    }

    fn a_sample(seq: u64) -> Arc<proto::PadReport> {
        Arc::new(proto::PadReport::Imu(proto::PadImuBatch {
            samples: vec![proto::PadImuSample {
                seq,
                at_us: 1_000_000 + seq * 1_667,
                accel: [-391, -35, 4270],
                gyro: [25_000, -9_000, 171_000],
            }],
            socket_dropped: 0,
        }))
    }

    /// IMU samples that overflow a slow subscriber are counted against the IMU, not the pad. The
    /// clone sends six hundred a second and the sticks a handful; a shared counter would report a
    /// pad link losing reports it never lost.
    #[test]
    fn dropped_samples_are_not_dropped_frames() {
        let shared = a_shared();
        let (reports, dropped) = shared.subscribe();

        for seq in 0..(QUEUE as u64 + 25) {
            shared.frame_report(&a_sample(seq));
        }

        assert_eq!(dropped.samples.load(Ordering::Relaxed), 25);
        assert_eq!(dropped.frames.load(Ordering::Relaxed), 0);
        drop(reports);
    }

    /// `padd` wanting the attitude is reason enough to hold the IMU open, with nobody subscribed —
    /// and the attitude it reads is the filter's, fed from the same batches the subscribers get.
    #[test]
    fn head_control_holds_the_imu_open_and_reads_the_filter() {
        let shared = a_shared();
        let node = Path::new("/dev/input/event5");
        shared.lock().wanted_imu = Some(node.to_path_buf());
        assert_eq!(
            shared.done_with_imu(node),
            Some("nobody is watching any more"),
            "no subscriber, no control: let go"
        );

        shared.lock().control = true;
        assert_eq!(shared.done_with_imu(node), None, "control alone holds it");

        shared.attach_imu(proto::PadImuDevice {
            name: "Nintendo Switch Pro Controller IMU".to_owned(),
            node: node.display().to_string(),
            accel_per_g: 4096,
            gyro_per_dps: 14247,
            accel_max: 32767,
            gyro_max: 32_767_000,
        });
        let read = |shared: &Shared| {
            shared
                .attitude_slot()
                .as_ref()
                .and_then(pad_imu::Imu::quaternion)
        };
        assert!(read(&shared).is_none(), "no sample yet, no attitude");
        // Flat on the table, one sample: seeded level.
        shared.samples(vec![proto::PadImuSample {
            seq: 1,
            at_us: 1_000_000,
            accel: [0, 0, 4096],
            gyro: [0, 0, 0],
        }]);
        let q = read(&shared).expect("an attitude after a believable sample");
        let euler = pad_imu::euler_deg(q);
        assert!(euler.iter().all(|a| a.abs() < 0.5), "{euler:?}");

        shared.detach_imu("the pad changed".to_owned());
        assert!(read(&shared).is_none(), "gone with the node");
    }

    /// A subscriber arriving while an IMU is open is told about it, as it is told about the pad.
    #[test]
    fn a_subscriber_arriving_mid_stream_is_told_the_imu_too() {
        let shared = a_shared();
        shared.attach(a_device());
        shared.attach_imu(proto::PadImuDevice {
            name: "Nintendo Switch Pro Controller IMU".to_owned(),
            node: "/dev/input/event5".to_owned(),
            accel_per_g: 4096,
            gyro_per_dps: 14247,
            accel_max: 32767,
            gyro_max: 32_767_000,
        });

        let (reports, _dropped) = shared.subscribe();
        assert!(matches!(
            &*reports.try_recv().unwrap(),
            proto::PadReport::Attached { .. }
        ));
        assert!(matches!(
            &*reports.try_recv().unwrap(),
            proto::PadReport::ImuAttached { .. }
        ));
    }

    /// The IMU is a sibling node under the pad's HID device with the accelerometer property, and
    /// nothing else qualifies: not the pad itself, not a node of another device, not a sibling
    /// without the property. Laid out as sysfs lays it out — `eventN/device` is the input device,
    /// its `device` the HID one — so the walk here is the walk on a board.
    #[test]
    fn the_imu_is_the_accelerometer_sibling_under_the_same_hid_device() {
        let root = tempfile::tempdir().unwrap();
        let sysfs = root.path().join("sys");
        let dev = root.path().join("dev");
        let hid_pad = sysfs.join("devices/hci0/0005:057E:2009.0001");
        let hid_other = sysfs.join("devices/hci0/0005:045E:0B13.0002");
        let class = sysfs.join("class/input");
        std::fs::create_dir_all(&class).unwrap();

        // (event, hid device, properties)
        let nodes = [
            ("event3", &hid_other, "0"),
            ("event4", &hid_pad, "0"),
            ("event5", &hid_pad, "40"),
            ("event6", &hid_other, "40"),
        ];
        for (index, (event, hid, props)) in nodes.iter().enumerate() {
            // As on a board: `<hid>/input/inputN/eventN` is the node's directory, its `device`
            // is the input device `inputN`, and *that* device's `device` is the HID one.
            let input = hid.join(format!("input/input{index}"));
            let node = input.join(event);
            std::fs::create_dir_all(&node).unwrap();
            std::fs::write(input.join("properties"), format!("{props}\n")).unwrap();
            std::os::unix::fs::symlink(&node, class.join(event)).unwrap();
            std::os::unix::fs::symlink(&input, node.join("device")).unwrap();
            std::os::unix::fs::symlink(hid, input.join("device")).unwrap();
        }

        assert_eq!(
            imu_sibling(&sysfs, &dev, Path::new("/dev/input/event4")),
            Some(dev.join("input/event5")),
            "the pad's own accelerometer sibling"
        );
        assert_eq!(
            imu_sibling(&sysfs, &dev, Path::new("/dev/input/event3")),
            Some(dev.join("input/event6")),
            "another pad, its own sibling — never the first pad's"
        );
        assert_eq!(
            imu_sibling(&sysfs, &dev, Path::new("/dev/input/event9")),
            None,
            "a node sysfs has never heard of"
        );
        assert!(has_accelerometer_property("40"));
        assert!(has_accelerometer_property("0 40"));
        assert!(!has_accelerometer_property("0"));
        assert!(!has_accelerometer_property(""));
    }

    /// The tap's wire types are the protocol's, so a frame it builds is a frame `robotctl` parses.
    /// Cheap to assert here, and the alternative is finding out on a board.
    #[test]
    fn a_frame_survives_the_wire() {
        let report = a_frame(7);
        let line = serde_json::to_string(&proto::Request::notify_pad_report(&report)).unwrap();
        let back: proto::Request = serde_json::from_str(&line).unwrap();
        assert_eq!(back.as_pad_report().as_ref(), Some(&*report));
    }
}
