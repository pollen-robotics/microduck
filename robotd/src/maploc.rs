//! robotd's mapping host: the `maploc` pipeline on its own worker thread.
//!
//! Off unless `[maploc] enabled = true` in robotd.toml. The design promise
//! is that the control loop pays a `try_send` per tick and nothing more:
//!
//!   - odometry, posture, head joints and the moving flag arrive from the
//!     loop as one small struct per tick over a bounded channel that drops
//!     when full — mapping lag can never become loop backpressure;
//!   - depth frames arrive on the same channel from a tokio task that
//!     subscribes to `tofd`'s socket like any other client, reconnecting
//!     with backoff — tofd down means mapping idles, not robotd caring;
//!   - the worker thread runs niced (+10): the scheduler gives the control
//!     loop the core whenever they compete.
//!
//! Frames are reprojected through the head FK with the IMU-levelled floor
//! filter and handed to [`maploc::mapper::Mapper`], which owns every
//! mapping decision — stillness, window vetting, the tracking watchdog,
//! kidnap relocalization. This file only moves bytes: channel in, log
//! lines and map frames out, the session to disk, and (when
//! `record_dir` is set) a ground-truth `.mdlg` recording of everything the
//! mapper consumed, replayable through the offline bench byte-for-byte.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use duck_ipc_proto as proto;
use kinematics::tof::{Posture, Reprojector};
use maploc::mapper::{Mapper, MapperConfig, MapperSample, Note};
use maploc::pipeline::{Slam, SlamConfig};
use maploc::record::SessionRecorder;
use maploc::session::SessionState;
use maploc::submap::Scan;

use crate::params::{MaplocMode, MaplocParams};

/// The loop-side channel depth. Sized for ~2 s of ticks plus frames: if the
/// worker stalls longer than that (a relocalize search, say), dropping
/// samples is the correct behaviour — odometry deltas re-fold on the next
/// accepted sample; a dropped depth frame is one of fifteen a second.
const EVENT_BUFFER: usize = 128;

/// ST's status codes for a usable range — the same wire contract
/// `robotctl`'s monitor applies, restated here because robotd deliberately
/// never links the `tof` driver crate.
const TOF_STATUS_VALID: [u8; 2] = [5, 9];

/// Autosave cadence. Sessions are small (a few hundred KB) and the write is
/// atomic, but flash on the board is not free — once a minute is plenty for
/// a map that took minutes to walk.
const AUTOSAVE_EVERY: Duration = Duration::from_secs(60);

/// Map publish cadence when someone is subscribed.
const PUBLISH_EVERY: Duration = Duration::from_secs(1);

/// One tick's worth of the robot's own state, as the control loop sees it.
#[derive(Debug, Clone, Copy)]
pub struct OdomSample {
    /// Contact odometry x, y, yaw.
    pub odom: (f32, f32, f32),
    /// Projected gravity in the trunk frame.
    pub gravity: [f64; 3],
    /// Odometry's trunk height above the floor, metres.
    pub trunk_z: f64,
    /// `[neck_pitch, head_pitch, head_yaw, head_roll]`, measured.
    pub head: [f64; 4],
    /// The loop's own "the robot is doing something" verdict.
    pub moving: bool,
    /// Seated. The mapper never maps from sitting height, and the
    /// ground-truth protocol uses the sit as its kidnap marker.
    pub sitting: bool,
    /// Fallen over (the safety layer's verdict).
    pub fallen: bool,
}

enum Event {
    Odom(OdomSample),
    Frame(Box<proto::TofFrame>),
    /// Reset everything: map, graph, tracked pose, suspicion — and delete
    /// the saved session. `robotctl robot map-wipe`.
    Wipe,
    /// Save and stop; the ack says the session is on disk. Sent by robotd's
    /// shutdown path — the channel never closes on its own, because the
    /// tofd feed thread and the IPC side hold Host clones for the life of
    /// the process, so "every sender dropped" is a signal that never fires.
    Shutdown(mpsc::SyncSender<()>),
}

/// Handle the rest of robotd holds. Dropping every clone (robotd shutting
/// down) closes the channel; the worker saves the session and exits.
#[derive(Clone)]
pub struct Host {
    tx: mpsc::SyncSender<Event>,
    searching: Arc<AtomicBool>,
}

impl Host {
    /// Feed one control-loop tick. Never blocks: a full channel drops the
    /// sample, and the next one carries the newer truth anyway.
    pub fn observe(&self, sample: OdomSample) {
        let _ = self.tx.try_send(Event::Odom(sample));
    }

    /// Feed one depth frame (called from the tofd subscription task).
    pub fn frame(&self, frame: proto::TofFrame) {
        let _ = self.tx.try_send(Event::Frame(Box::new(frame)));
    }

    /// Ask the worker to reset the mapping session. Returns false when the
    /// channel is jammed (the caller reports the refusal; retrying is the
    /// operator's one keystroke).
    pub fn wipe(&self) -> bool {
        self.tx.try_send(Event::Wipe).is_ok()
    }

    /// Save the session and stop the worker, waiting briefly for the disk
    /// write. Called from robotd's shutdown path: without it the final
    /// save documented below never runs, and a `systemctl restart` costs
    /// up to a full autosave interval of mapping.
    pub fn shutdown(&self) {
        let (ack_tx, ack_rx) = mpsc::sync_channel(1);
        if self.tx.send(Event::Shutdown(ack_tx)).is_ok() {
            let _ = ack_rx.recv_timeout(Duration::from_secs(3));
        }
    }

    /// Is the mapper's pose suspect or lost right now? The control loop
    /// polls this to sweep the head while standing: a single 45° wedge
    /// aliases onto any wall at the same range, and the accumulator merges
    /// a pan into one wide composite — the difference between the bench's
    /// 0-for-13 relocalizes on wedges and 6-for-10 on sweeps.
    pub fn searching(&self) -> bool {
        self.searching.load(Ordering::Relaxed)
    }
}

/// Start the worker and its tofd feed. `map_tx` is where rendered maps go;
/// the connection handler hands subscribers its receivers.
pub fn spawn(
    params: MaplocParams,
    map_tx: tokio::sync::broadcast::Sender<proto::MapFrame>,
) -> Host {
    let (tx, rx) = mpsc::sync_channel(EVENT_BUFFER);
    let searching = Arc::new(AtomicBool::new(false));
    let searching_worker = searching.clone();
    std::thread::Builder::new()
        .name("maploc".into())
        .spawn(move || {
            // The control loop must win every contest for a core. PRIO_PROCESS
            // with tid 0 renices only this thread on Linux.
            unsafe {
                libc::setpriority(libc::PRIO_PROCESS, 0, 10);
            }
            worker(params, rx, map_tx, &searching_worker);
        })
        .expect("spawning the maploc thread cannot fail");

    // The tofd subscription gets its own thread and its own tiny runtime,
    // WITH an IO driver. It must not ride the control loop's runtime: that
    // one is built time-only, and a socket task spawned there dies on its
    // first connect — silently, inside the JoinHandle nobody reads. That is
    // not hypothetical; it is how field test three produced `frames=0`.
    let feed = Host {
        tx: tx.clone(),
        searching: searching.clone(),
    };
    std::thread::Builder::new()
        .name("maploc-tof".into())
        .spawn(move || {
            unsafe {
                libc::setpriority(libc::PRIO_PROCESS, 0, 10);
            }
            match tokio::runtime::Builder::new_current_thread()
                .enable_io()
                .enable_time()
                .build()
            {
                Ok(rt) => rt.block_on(feed_tof(feed)),
                Err(e) => tracing::error!(error = %e, "maploc: no runtime for the tofd feed"),
            }
        })
        .expect("spawning the maploc feed thread cannot fail");

    Host { tx, searching }
}

fn worker(
    params: MaplocParams,
    rx: mpsc::Receiver<Event>,
    map_tx: tokio::sync::broadcast::Sender<proto::MapFrame>,
    searching: &AtomicBool,
) {
    let slam = if params.wipe_on_boot {
        tracing::info!("maploc: starting fresh (wipe_on_boot)");
        Slam::new(SlamConfig::default())
    } else {
        match SessionState::load(&params.map_path) {
            Ok(Some(session)) => {
                tracing::info!(path = %params.map_path.display(), "maploc: resumed saved session");
                Slam::from_session(SlamConfig::default(), session)
            }
            Ok(None) => Slam::new(SlamConfig::default()),
            Err(e) => {
                tracing::warn!(error = %e, "maploc: saved session unreadable; starting fresh");
                Slam::new(SlamConfig::default())
            }
        }
    };
    let mut mapper = Mapper::new(
        MapperConfig {
            continuous: params.mode == MaplocMode::Continuous,
            ..MapperConfig::default()
        },
        slam,
    );

    if !mapper.tracking() {
        tracing::info!("maploc: resumed map with a suspect pose — confirming before anything inks");
    }

    let mut recorder = params.record_dir.as_ref().and_then(|dir| {
        if let Err(e) = std::fs::create_dir_all(dir) {
            tracing::warn!(error = %e, dir = %dir.display(), "maploc: cannot create record_dir");
            return None;
        }
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let path = dir.join(format!("{stamp}.mdlg"));
        match SessionRecorder::create(&path) {
            Ok(rec) => {
                tracing::info!(path = %path.display(), "maploc: recording session");
                Some(rec)
            }
            Err(e) => {
                tracing::warn!(error = %e, "maploc: cannot open recording; not recording");
                None
            }
        }
    });

    let reprojector = Reprojector::alpha();
    let started = Instant::now();
    let mut latest: Option<OdomSample> = None;
    let mut notes: Vec<Note> = Vec::new();
    let mut last_publish = Instant::now();
    let mut last_save = Instant::now();
    let mut seq = 0u64;
    let mut unsaved = false;
    let mut rendered: Option<RenderedGrid> = None;
    let mut render_stale = true;
    // Field diagnostics: one line every 5 s says what mapping is actually
    // doing, because "the map shows nothing" has half a dozen distinct
    // causes and a journal that names the live one beats guessing.
    let mut last_status = Instant::now();
    let (mut n_odom, mut n_frames, mut n_frames_kept) = (0u64, 0u64, 0u64);

    searching.store(!mapper.tracking(), Ordering::Relaxed);

    // Blocking recv; a closed channel is the shutdown signal.
    while let Ok(event) = rx.recv() {
        match event {
            Event::Odom(sample) => {
                if let Some(rec) = recorder.as_mut()
                    && rec
                        .odom(
                            sample.odom,
                            sample.gravity.map(|g| g as f32),
                            sample.trunk_z as f32,
                            sample.head.map(|h| h as f32),
                            sample.moving,
                            sample.sitting,
                            sample.fallen,
                        )
                        .is_err()
                {
                    tracing::warn!("maploc: recording write failed; recording stopped");
                    recorder = None;
                }
                mapper.observe(
                    started.elapsed().as_secs_f32(),
                    MapperSample {
                        odom: sample.odom,
                        moving: sample.moving,
                        sitting: sample.sitting,
                        fallen: sample.fallen,
                    },
                    &mut notes,
                );
                latest = Some(sample);
                n_odom += 1;
            }
            Event::Shutdown(ack) => {
                if unsaved {
                    match mapper.slam().save(&params.map_path) {
                        Ok(()) => {
                            tracing::info!(path = %params.map_path.display(), "maploc: session saved");
                        }
                        Err(e) => tracing::warn!(error = %e, "maploc: final save failed"),
                    }
                }
                if let Some(rec) = recorder.as_mut() {
                    let _ = rec.flush();
                }
                let _ = ack.send(());
                return;
            }
            Event::Wipe => {
                mapper = Mapper::new(
                    MapperConfig {
                        continuous: params.mode == MaplocMode::Continuous,
                        ..MapperConfig::default()
                    },
                    Slam::new(SlamConfig::default()),
                );
                if let Err(e) = std::fs::remove_file(&params.map_path)
                    && e.kind() != std::io::ErrorKind::NotFound
                {
                    tracing::warn!(error = %e, "maploc: wipe could not delete the session file");
                }
                unsaved = false;
                rendered = None;
                render_stale = true;
                tracing::info!("maploc: session wiped by request");
            }
            Event::Frame(frame) => {
                n_frames += 1;
                if let Some(rec) = recorder.as_mut()
                    && rec
                        .tof(
                            frame.at_us as f64 / 1e6,
                            frame.rows,
                            frame.cols,
                            &frame.distance_mm,
                            &frame.status,
                        )
                        .is_err()
                {
                    tracing::warn!("maploc: recording write failed; recording stopped");
                    recorder = None;
                }
                let Some(sample) = latest else { continue };
                let Some(ranges) = decode_ranges(&frame) else {
                    continue;
                };
                let posture = Posture {
                    gravity: sample.gravity,
                    trunk_height_m: (sample.trunk_z > 0.02).then_some(sample.trunk_z),
                };
                let flat = reprojector.flatten(&ranges, sample.head, &posture);
                if flat.angles_body.is_empty() {
                    continue;
                }
                let scan = Scan::from_polar(&flat.angles_body, &flat.ranges, flat.sensor_xy, 1e-3);
                if mapper.frame(started.elapsed().as_secs_f32(), scan) {
                    n_frames_kept += 1;
                }
            }
        }

        for note in notes.drain(..) {
            match note {
                Note::WindowIntegrated {
                    beams,
                    windows,
                    mean_residual_m,
                    n_observed,
                    ..
                } => {
                    tracing::info!(
                        beams,
                        windows,
                        agree = format!("{mean_residual_m:.3}/{n_observed}"),
                        "maploc: still window integrated"
                    );
                }
                Note::WindowDiscarded { beams } => {
                    tracing::debug!(beams, "maploc: window too thin to ink; discarded");
                }
                Note::WindowQuarantined {
                    mean_residual_m,
                    n_observed,
                } => {
                    tracing::info!(
                        residual = format!("{mean_residual_m:.3}"),
                        n_observed,
                        "maploc: window contradicts the map; quarantined"
                    );
                }
                Note::SuspectAfterSit => {
                    tracing::info!("maploc: robot sat — pose suspect until a window confirms it");
                }
                Note::SuspectAfterFall => {
                    tracing::info!("maploc: robot fell — pose suspect until a window confirms it");
                }
                Note::ResumedUnverified { pose } => {
                    tracing::warn!(
                        x = format!("{:.2}", pose.0),
                        y = format!("{:.2}", pose.1),
                        "maploc: nothing could judge the pose; resumed unverified"
                    );
                }
                Note::RelocalizeCandidate {
                    pose,
                    mean_residual_m,
                } => {
                    tracing::info!(
                        x = format!("{:.2}", pose.0),
                        y = format!("{:.2}", pose.1),
                        yaw = format!("{:.2}", pose.2),
                        residual = format!("{mean_residual_m:.3}"),
                        "maploc: relocalize candidate; awaiting confirmation"
                    );
                }
                Note::LostTracking {
                    mean_residual_m,
                    n_observed,
                } => {
                    tracing::warn!(
                        residual = format!("{mean_residual_m:.3}"),
                        n_observed,
                        "maploc: scans contradict the map here — tracking lost, searching"
                    );
                }
                Note::Relocalized {
                    pose,
                    mean_residual_m,
                } => {
                    tracing::info!(
                        x = format!("{:.2}", pose.0),
                        y = format!("{:.2}", pose.1),
                        yaw = format!("{:.2}", pose.2),
                        residual = format!("{mean_residual_m:.3}"),
                        "maploc: relocalized"
                    );
                }
                Note::RelocalizeRejected {
                    best_pose,
                    mean_residual_m,
                } => {
                    tracing::debug!(
                        x = format!("{:.2}", best_pose.0),
                        y = format!("{:.2}", best_pose.1),
                        residual = format!("{mean_residual_m:.3}"),
                        "maploc: relocalize attempt rejected"
                    );
                }
                Note::LoopClosed {
                    n_loops,
                    dx,
                    dy,
                    dyaw,
                } => {
                    tracing::info!(
                        loops = n_loops,
                        dx = format!("{dx:.3}"),
                        dy = format!("{dy:.3}"),
                        dyaw = format!("{dyaw:.3}"),
                        "maploc: loop closed; tracked pose corrected"
                    );
                }
            }
        }
        if mapper.slam_mut().take_dirty() {
            unsaved = true;
            render_stale = true;
        }
        searching.store(!mapper.tracking(), Ordering::Relaxed);

        if last_status.elapsed() >= Duration::from_secs(5) {
            last_status = Instant::now();
            tracing::info!(
                odom = n_odom,
                frames = n_frames,
                kept = n_frames_kept,
                windows = mapper.windows(),
                still = mapper.still(),
                tracking = mapper.tracking(),
                moving = latest.as_ref().is_some_and(|s| s.moving),
                sitting = latest.as_ref().is_some_and(|s| s.sitting),
                fallen = latest.as_ref().is_some_and(|s| s.fallen),
                window_frames = mapper.window_frames(),
                submaps = mapper.slam().n_submaps(),
                "maploc: status"
            );
            if let Some(rec) = recorder.as_mut()
                && rec.flush().is_err()
            {
                tracing::warn!("maploc: recording flush failed; recording stopped");
                recorder = None;
            }
        }

        if map_tx.receiver_count() > 0 && last_publish.elapsed() >= PUBLISH_EVERY {
            last_publish = Instant::now();
            // The grid re-renders only when the map changed: compositing
            // every submap once a second for a robot that is just walking
            // (pose changes, ink does not) grows with the map for nothing.
            // The pose, tracking and posture fields ride every frame.
            if render_stale || rendered.is_none() {
                rendered = render_grid(&mapper);
                render_stale = false;
            }
            if let Some(grid) = &rendered {
                seq += 1;
                let seated = latest.as_ref().is_some_and(|s| s.sitting || s.fallen);
                let _ = map_tx.send(frame_from(&mapper, grid, seq, seated));
            }
        }

        if unsaved && last_save.elapsed() >= AUTOSAVE_EVERY {
            last_save = Instant::now();
            match mapper.slam().save(&params.map_path) {
                Ok(()) => unsaved = false,
                Err(e) => tracing::warn!(error = %e, "maploc: autosave failed"),
            }
        }
    }

    // Every sender dropped — robotd is tearing down without having sent
    // Shutdown (a panic path). Save anyway; the session is the product.
    if unsaved {
        if let Err(e) = mapper.slam().save(&params.map_path) {
            tracing::warn!(error = %e, "maploc: final save failed");
        } else {
            tracing::info!(path = %params.map_path.display(), "maploc: session saved");
        }
    }
    if let Some(rec) = recorder.as_mut() {
        let _ = rec.flush();
    }
}

/// The wire frame's 64 zones as metres, `None` where the sensor said the
/// measurement is not to be trusted.
fn decode_ranges(
    frame: &proto::TofFrame,
) -> Option<[Option<f64>; kinematics::tof::ROWS * kinematics::tof::COLS]> {
    const N: usize = kinematics::tof::ROWS * kinematics::tof::COLS;
    if frame.distance_mm.len() != N || frame.status.len() != N {
        return None;
    }
    let mut out = [None; N];
    for (slot, (&mm, &status)) in out
        .iter_mut()
        .zip(frame.distance_mm.iter().zip(frame.status.iter()))
    {
        if TOF_STATUS_VALID.contains(&status) && mm > 0 {
            *slot = Some(f64::from(mm) / 1000.0);
        }
    }
    Some(out)
}

/// A rendered composite, already trinarized and base64'd — everything in a
/// [`proto::MapFrame`] that only changes when the map's ink does.
struct RenderedGrid {
    x_min: f32,
    y_min: f32,
    cell_m: f32,
    rows: u32,
    cols: u32,
    cells: String,
}

fn render_grid(mapper: &Mapper) -> Option<RenderedGrid> {
    let grid = mapper.slam().render()?;
    let mut cells = Vec::with_capacity(grid.width() * grid.height());
    for i in 0..grid.height() {
        for j in 0..grid.width() {
            let lo = grid.log_at(i, j);
            cells.push(if lo > 150 {
                2u8
            } else if lo < -50 {
                1
            } else {
                0
            });
        }
    }
    Some(RenderedGrid {
        x_min: grid.cfg().x_range.0,
        y_min: grid.cfg().y_range.0,
        cell_m: grid.cell(),
        rows: grid.height() as u32,
        cols: grid.width() as u32,
        cells: proto::b64::encode(&cells),
    })
}

/// The wire frame: the cached grid plus everything that moves every second.
fn frame_from(mapper: &Mapper, grid: &RenderedGrid, seq: u64, seated: bool) -> proto::MapFrame {
    let (x, y, yaw) = mapper.slam().tracked();
    proto::MapFrame {
        seq,
        x: f64::from(x),
        y: f64::from(y),
        yaw: f64::from(yaw),
        tracking: mapper.tracking() && mapper.slam().n_submaps() > 0,
        x_min: grid.x_min,
        y_min: grid.y_min,
        cell_m: grid.cell_m,
        rows: grid.rows,
        cols: grid.cols,
        cells: grid.cells.clone(),
        n_submaps: mapper.slam().n_submaps() as u32,
        n_loops: mapper.slam().n_loops() as u32,
        windows: mapper.windows(),
        still: mapper.still(),
        seated,
    }
}

/// Subscribe to `tofd`'s depth stream and pump frames into the host.
/// Reconnects with backoff forever: tofd restarting (or absent on a board
/// with no sensor) idles mapping, nothing more.
async fn feed_tof(host: Host) {
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    let mut backoff = Duration::from_millis(500);
    loop {
        match tokio::net::UnixStream::connect(proto::socket::TOF).await {
            Ok(stream) => {
                tracing::info!("maploc: connected to tofd's depth stream");
                backoff = Duration::from_millis(500);
                let (read, mut write) = stream.into_split();
                let hello = serde_json::to_string(&proto::Request::call(
                    proto::Id::Number(1),
                    &proto::Call::Hello(proto::HelloParams {
                        api_version: proto::API_VERSION,
                    }),
                ))
                .expect("hello serializes");
                let subscribe = serde_json::to_string(&proto::Request::call(
                    proto::Id::Number(2),
                    &proto::Call::TofStream,
                ))
                .expect("subscribe serializes");
                if write
                    .write_all(format!("{hello}\n{subscribe}\n").as_bytes())
                    .await
                    .is_err()
                {
                    tokio::time::sleep(backoff).await;
                    continue;
                }
                let mut lines = BufReader::new(read).lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    if let Ok(request) = serde_json::from_str::<proto::Request>(&line)
                        && let Some(frame) = request.as_tof_frame()
                    {
                        host.frame(frame);
                    }
                }
                tracing::debug!("maploc: tofd stream ended; reconnecting");
            }
            Err(e) => {
                // Common on a no-sensor board; say it once per backoff step
                // at debug, because a silent failure here already cost a
                // field-test afternoon.
                tracing::debug!(error = %e, "maploc: cannot reach tofd");
            }
        }
        tokio::time::sleep(backoff).await;
        backoff = (backoff * 2).min(Duration::from_secs(10));
    }
}
