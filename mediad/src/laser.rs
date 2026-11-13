//! Finding a laser dot in the frames already on the tee, without touching the picture.
//!
//! The prototype runtime did this with a 2 ms manual shutter, which makes a dot the only bright
//! thing in a black frame — and makes the video useless. Here the stream keeps its own
//! auto-exposure and this module only *reads* frames the tee hands out on request, so watching the
//! camera and following a laser are not alternatives. The price is that a dot at normal exposure
//! is a saturated white core with a red rim, sitting in a lit room, so it is found as "bright, red
//! and not blue, and small" rather than "bright". Those four are `[laser]`'s thresholds, and every
//! answer carries the colour of the best candidate whether it passed or not, because that number is
//! the whole of tuning them.
//!
//! **Features, not frames** (`architecture.md` §2.4): what leaves this module is a *direction* — a
//! ray in the upright camera's cv2 frame, with the lens's intrinsics and the board's mount (a
//! quarter turn on the Zero 3W, none on the beta boards) already undone. `robotd` turns that into a place on the floor with the head FK and odometry it
//! already has, and needs to know nothing about pixels.
//!
//! **Cost.** A look is one frame copied out of the tee (the rendezvous in
//! [`crate::pipeline::Frames`] — read in place, never cloned again) and one scan of every other row
//! and column of it: about 230 k samples at 720p, about 5 ms on a face-board-v1. Paced at
//! `search_hz` until something is seen and `track_hz` while it is — so a tracker left on all day
//! costs a few looks a second.
//!
//! While tracking, a look scans only a window around the last dot ([`WINDOW_RADII`] times
//! `radius_px` each way, about 4% of a 720p frame at the defaults), and the whole frame when the
//! dot is not found near where it was ([`FOLLOW_RADII`]), and once every [`FULL_SCAN_EVERY`]. What
//! that changes: a higher-scoring dot outside the window, or a red thing within `FOLLOW_RADII` of a
//! dot that jumped away, holds until the next whole-frame scan, a second at most, where the old scan
//! settled it on the next look; and `peak` outside the window is as of that scan. [`Scope`]
//! decides.

use duck_ipc_proto::{LaserDetectParams, LaserDot};
use uyvy::Turn;

/// Every other row and every other pixel. A dot at any distance worth walking to covers more than
/// a 2×2 block, and halving both axes is a quarter of the work.
const STEP: usize = 2;

/// How long after the last sighting the detector stays at its tracking rate. Long enough to ride
/// over a dot that blinks out for a frame or two, short enough that a laser put away stops costing.
pub const TRACK_HOLD: std::time::Duration = std::time::Duration::from_secs(1);

/// How far around the last dot a tracking look scans, in `radius_px` each way: 96 px at the
/// default 24, a 193 px square counting both edges.
pub const WINDOW_RADII: usize = 4;

/// How far from the last dot the window's answer may be and still count as that dot, in
/// `radius_px`: 48 px at the default 24. Further, the laser may have jumped away and left a red
/// thing behind in the window, so the same look scans the whole frame and takes its answer, as
/// every look did before the window. A dot that moves this far between two looks costs a whole
/// scan, nothing more.
pub const FOLLOW_RADII: usize = 2;

/// How often a look scans the whole frame while tracking, window or not (the first look after it
/// has passed): how long a higher-scoring dot elsewhere can wait to take over, and how old `peak`
/// outside the window can be. A second, as long as `search_hz` takes to notice a dot by default.
pub const FULL_SCAN_EVERY: std::time::Duration = std::time::Duration::from_secs(1);

/// What turns a pixel of the frame the camera delivered into a direction.
#[derive(Debug, Clone, PartialEq)]
pub struct Geometry {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// OpenCV's `k1 k2 p1 p2 k3`, or empty.
    pub distortion: Vec<f64>,
    /// How far the picture has to turn to be upright — the same turn the duck detector and the
    /// JPEG streamer use. `None` when the pipeline already flipped it.
    pub turn: Turn,
}

impl Geometry {
    pub fn from_intrinsics(intrinsics: &crate::camera::Intrinsics, turn: Turn) -> Self {
        Self {
            fx: intrinsics.fx,
            fy: intrinsics.fy,
            cx: intrinsics.cx,
            cy: intrinsics.cy,
            distortion: intrinsics.distortion.clone(),
            turn,
        }
    }

    /// The ray `[x, y]` (of `[x, y, 1]`) for a pixel of the delivered frame, in the upright
    /// camera's cv2 axes: +x right, +y down.
    pub fn ray(&self, u: f64, v: f64) -> [f64; 2] {
        let (x, y) = self.undistort((u - self.cx) / self.fx, (v - self.cy) / self.fy);
        // The turn that makes the picture upright, applied to a direction. `uyvy::Turn` maps an
        // upright pixel back to the source one; this is the forward map — a quarter turn clockwise
        // sends source (x, y) to upright (-y, x) about the centre.
        match self.turn {
            Turn::None => [x, y],
            Turn::Right => [-y, x],
            Turn::Half => [-x, -y],
            Turn::Left => [y, -x],
        }
    }

    /// Where a pixel of the delivered frame lands in the upright picture, for a readout.
    pub fn upright_px(&self, u: f64, v: f64, width: usize, height: usize) -> [f64; 2] {
        let (w, h) = (width as f64 - 1.0, height as f64 - 1.0);
        match self.turn {
            Turn::None => [u, v],
            Turn::Right => [h - v, u],
            Turn::Half => [w - u, h - v],
            Turn::Left => [v, w - u],
        }
    }

    /// Normalised distorted coordinates back to undistorted ones, by fixed-point iteration —
    /// what OpenCV's `undistortPoints` does. A handful of rounds is plenty at this lens's
    /// distortion, and the answer feeds a tracker, not a map.
    fn undistort(&self, xd: f64, yd: f64) -> (f64, f64) {
        let d = &self.distortion;
        if d.len() < 4 {
            return (xd, yd);
        }
        let (k1, k2, p1, p2) = (d[0], d[1], d[2], d[3]);
        let k3 = d.get(4).copied().unwrap_or(0.0);
        let (mut x, mut y) = (xd, yd);
        for _ in 0..8 {
            let r2 = x * x + y * y;
            let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
            let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            x = (xd - dx) / radial;
            y = (yd - dy) / radial;
        }
        (x, y)
    }
}

/// One scan's answer, in the delivered frame's pixels.
#[derive(Debug, Clone, PartialEq)]
pub struct Found {
    /// The dot's centre, when one passed.
    pub at: Option<[f64; 2]>,
    /// The brightest hit, passed or not — where a rejected candidate was.
    pub candidate: Option<[f64; 2]>,
    /// Sampled pixels in the dot (or in the rejected candidate).
    pub pixels: u32,
    /// The reddest bright sample in the frame, `[Y, U, V]`, passed or not. [`find_near`]'s is its
    /// window's; a tracking look's is the window's and, outside it, the last whole-frame scan's
    /// ([`Scope`]).
    pub peak: Option<[u8; 3]>,
    pub rejected: Option<&'static str>,
}

/// One sample of a `UYVY` frame: the brighter of the macropixel's two lumas, and its shared chroma.
#[inline]
fn sample(row: &[u8], macro_px: usize) -> (u8, u8, u8) {
    let i = macro_px * 4;
    // U Y0 V Y1
    (row[i + 1].max(row[i + 3]), row[i], row[i + 2])
}

/// How red-and-bright a sample is, for picking the candidate: luma, plus red chroma counted
/// double, minus blue. Only ever compared, never thresholded.
#[inline]
fn score(y: u8, u: u8, v: u8) -> i32 {
    i32::from(y) + 2 * (i32::from(v) - 128) - (i32::from(u) - 128)
}

#[inline]
fn hit(y: u8, u: u8, v: u8, p: &LaserDetectParams) -> bool {
    y >= p.y_min && v >= p.v_min && u <= p.u_max
}

/// Find the dot in one `UYVY` frame, `width × height`, tightly packed.
///
/// Two passes. The first finds the highest-scoring sample that passes the thresholds, and the
/// reddest bright one for the report. The second counts and averages the passing samples within
/// `radius_px` of that best one — so a stray hit across the room neither moves the centre nor
/// inflates the count, which a frame-wide centroid would let it do.
pub fn find(data: &[u8], width: usize, height: usize, p: &LaserDetectParams) -> Found {
    whole(data, width, height, p).0
}

/// [`find`], and where its peak is.
fn whole(data: &[u8], width: usize, height: usize, p: &LaserDetectParams) -> (Found, Option<Peak>) {
    if !fits(data, width, height) {
        return (NONE, None);
    }
    let (peak, best) = first_pass(data, width, [0, last_column(width)], [0, height - 1], p);
    let found = second_pass(data, width, height, p, peak.map(|(_, yuv, _)| yuv), best);
    (found, peak)
}

/// [`find`], scanning only the window around `near` (a dot's centre from an earlier look, in the
/// same frame's pixels). `None` unless a dot passed in it: a miss or a rejected candidate there is
/// for the whole-frame scan to report.
pub fn find_near(
    data: &[u8],
    width: usize,
    height: usize,
    p: &LaserDetectParams,
    near: [f64; 2],
) -> Option<Found> {
    windowed(data, width, height, p, near).map(|(found, _)| found)
}

/// [`find_near`], and the window it scanned: pixel columns and rows, both inclusive.
fn windowed(
    data: &[u8],
    width: usize,
    height: usize,
    p: &LaserDetectParams,
    near: [f64; 2],
) -> Option<(Found, [[usize; 2]; 2])> {
    if !fits(data, width, height) {
        return None;
    }
    let r = (p.radius_px as usize * WINDOW_RADII).max(STEP);
    // `as` saturates: a negative or NaN centre is 0, and the `min` keeps a stale one in the frame.
    let cx = (near[0] as usize).min(width - 1);
    let cy = (near[1] as usize).min(height - 1);
    let xs = [cx.saturating_sub(r), (cx + r).min(last_column(width))];
    let ys = [cy.saturating_sub(r), (cy + r).min(height - 1)];
    let (peak, best) = first_pass(data, width, xs, ys, p);
    let found = second_pass(data, width, height, p, peak.map(|(_, yuv, _)| yuv), best);
    found.at.is_some().then_some((found, [xs, ys]))
}

const NONE: Found = Found {
    at: None,
    candidate: None,
    pixels: 0,
    peak: None,
    rejected: None,
};

fn fits(data: &[u8], width: usize, height: usize) -> bool {
    width >= 2 && height > 0 && data.len() >= width * 2 * height
}

/// The last pixel column a scan samples: the second pixel of the last whole macropixel.
fn last_column(width: usize) -> usize {
    width / 2 * 2 - 1
}

/// The rows (`STEP` apart, from 0, so every scan samples the same grid) and macropixels between
/// two pixel columns and two rows, both inclusive.
fn grid(
    xs: [usize; 2],
    ys: [usize; 2],
) -> (impl Iterator<Item = usize>, std::ops::RangeInclusive<usize>) {
    let first_row = ys[0].div_ceil(STEP) * STEP;
    ((first_row..=ys[1]).step_by(STEP), xs[0] / 2..=xs[1] / 2)
}

/// A sample that passed the thresholds: its score, and where it is in pixels.
type Hit = (i32, usize, usize);

/// The reddest bright sample: its score, its `[Y, U, V]`, and where it is in pixels.
type Peak = (i32, [u8; 3], [usize; 2]);

/// The reddest bright sample and the highest-scoring hit within `xs × ys`.
fn first_pass(
    data: &[u8],
    width: usize,
    xs: [usize; 2],
    ys: [usize; 2],
    p: &LaserDetectParams,
) -> (Option<Peak>, Option<Hit>) {
    let stride = width * 2;
    let mut peak: Option<Peak> = None;
    let mut best: Option<Hit> = None;
    let (rows, macros) = grid(xs, ys);
    for py in rows {
        let row = &data[py * stride..(py + 1) * stride];
        // One sample per macropixel is every other pixel, which is the column step.
        for m in macros.clone() {
            let (y, u, v) = sample(row, m);
            let s = score(y, u, v);
            if peak.is_none_or(|(ps, _, _)| s > ps) {
                peak = Some((s, [y, u, v], [m * 2, py]));
            }
            if hit(y, u, v, p) && best.is_none_or(|(bs, _, _)| s > bs) {
                best = Some((s, m * 2, py));
            }
        }
    }
    (peak, best)
}

/// Count and average the hits within `radius_px` of the best one, and judge their number.
fn second_pass(
    data: &[u8],
    width: usize,
    height: usize,
    p: &LaserDetectParams,
    peak: Option<[u8; 3]>,
    best: Option<Hit>,
) -> Found {
    let stride = width * 2;
    let Some((_, bx, by)) = best else {
        return Found { peak, ..NONE };
    };

    let r = p.radius_px as usize;
    let (x0, x1) = (bx.saturating_sub(r), (bx + r).min(last_column(width)));
    let (y0, y1) = (by.saturating_sub(r), (by + r).min(height - 1));
    let (mut n, mut sx, mut sy) = (0u32, 0u64, 0u64);
    // Rows on the same grid as the first pass, so the count means the same thing in both.
    let first_row = y0.div_ceil(STEP) * STEP;
    for py in (first_row..=y1).step_by(STEP) {
        let row = &data[py * stride..(py + 1) * stride];
        for m in (x0 / 2)..=(x1 / 2) {
            let (y, u, v) = sample(row, m);
            if hit(y, u, v, p) {
                n += 1;
                // The macropixel's centre: between its two pixels.
                sx += (m * 2) as u64 + 1;
                sy += py as u64;
            }
        }
    }
    let rejected = if n < p.min_pixels {
        Some("too small")
    } else if n > p.max_pixels {
        Some("too big")
    } else {
        None
    };
    // Two horizontal samples per macropixel were folded into one, so the centre is at the pair's
    // middle — the `+ 1` above — and the vertical at the sampled row.
    let at = rejected
        .is_none()
        .then(|| [sx as f64 / f64::from(n) - 0.5, sy as f64 / f64::from(n)]);
    Found {
        at,
        candidate: Some([bx as f64 + 0.5, by as f64]),
        pixels: n,
        peak,
        rejected,
    }
}

/// Where each look scans, from what the looks before it found: around the last dot while there is
/// one and it is still there, the whole frame otherwise and once every [`FULL_SCAN_EVERY`].
#[derive(Debug, Default)]
pub struct Scope {
    /// The last look's dot, if it had one.
    last: Option<[f64; 2]>,
    /// When a look last scanned the whole frame.
    full: Option<std::time::Instant>,
    /// That scan's peak, which stands for the part of the frame a window look does not read, so
    /// the readout does not swap to the window's every look and back every second.
    full_peak: Option<Peak>,
}

impl Scope {
    /// One look at a frame, taken at `now`.
    pub fn look(
        &mut self,
        data: &[u8],
        width: usize,
        height: usize,
        p: &LaserDetectParams,
        now: std::time::Instant,
    ) -> Found {
        let full_due = self
            .full
            .is_none_or(|at| now.saturating_duration_since(at) >= FULL_SCAN_EVERY);
        let follow = (p.radius_px as usize * FOLLOW_RADII) as f64;
        let near = self.last.filter(|_| !full_due).and_then(|[lx, ly]| {
            windowed(data, width, height, p, [lx, ly])
                .filter(|(f, _)| f.at.is_some_and(|[x, y]| (x - lx).hypot(y - ly) <= follow))
        });
        let found = match near {
            Some((mut found, [xs, ys])) => {
                // Inside the window this look is the fresh one; outside it, the last whole scan.
                let outside = self.full_peak.filter(|(_, _, [x, y])| {
                    !(xs[0]..=xs[1]).contains(x) || !(ys[0]..=ys[1]).contains(y)
                });
                found.peak = redder(found.peak, outside.map(|(_, yuv, _)| yuv));
                found
            }
            None => {
                let (found, peak) = whole(data, width, height, p);
                self.full = Some(now);
                self.full_peak = peak;
                found
            }
        };
        self.last = found.at;
        found
    }
}

/// The higher-scoring of two peaks.
fn redder(a: Option<[u8; 3]>, b: Option<[u8; 3]>) -> Option<[u8; 3]> {
    let s = |[y, u, v]: [u8; 3]| score(y, u, v);
    match (a, b) {
        (Some(a), Some(b)) => Some(if s(b) > s(a) { b } else { a }),
        _ => a.or(b),
    }
}

/// [`find`]'s answer, as the wire carries it.
pub fn dot(
    found: &Found,
    geometry: &Geometry,
    width: usize,
    height: usize,
    hz: f64,
    took_ms: f64,
) -> LaserDot {
    LaserDot {
        seen: found.at.is_some(),
        ray: found.at.map(|[u, v]| geometry.ray(u, v)),
        px: found
            .at
            .map(|[u, v]| geometry.upright_px(u, v, width, height)),
        frame: Some({
            let (w, h) = geometry.turn.upright(width, height);
            [w as u32, h as u32]
        }),
        candidate_px: found
            .rejected
            .and(found.candidate)
            .map(|[u, v]| geometry.upright_px(u, v, width, height)),
        pixels: found.pixels,
        peak_yuv: found.peak,
        rejected: found.rejected.map(str::to_owned),
        hz,
        took_ms,
    }
}

/// Where the detectors' answers are shared, beyond the subscriber that asked for them.
///
/// The geometry `main` fills in once the intrinsics are known, and a broadcast of every look —
/// the laser's and the ball's — so a console watching the video can draw what the detector is
/// seeing. A look is only ever taken for a subscriber — `robotd` while tracking is on — so with
/// tracking off these carry nothing, and a console with nobody to listen costs the detector a
/// failed `send`.
pub struct Hub {
    pub geometry: std::sync::OnceLock<Geometry>,
    pub dots: tokio::sync::broadcast::Sender<std::sync::Arc<LaserDot>>,
    pub balls: tokio::sync::broadcast::Sender<std::sync::Arc<duck_ipc_proto::BallSeen>>,
}

impl Default for Hub {
    fn default() -> Self {
        Self {
            geometry: std::sync::OnceLock::new(),
            // A handful: a console that falls behind wants the newest dot, not a queue of old ones.
            dots: tokio::sync::broadcast::channel(4).0,
            balls: tokio::sync::broadcast::channel(4).0,
        }
    }
}

/// One look as the console reads it, a JSON-RPC notification with no id — the same shape
/// `media.detections` already has on that channel.
pub fn notification(dot: &LaserDot) -> String {
    serde_json::json!({
        "jsonrpc": "2.0",
        "method": duck_ipc_proto::method::MEDIA_LASER_DOT,
        "params": dot,
    })
    .to_string()
}

/// The local subscription: answer, then one [`duck_ipc_proto::method::MEDIA_LASER_DOT`] per look
/// until the subscriber hangs up. [`serve_looks`] is how.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub async fn serve<R>(
    reader: R,
    write: tokio::net::unix::OwnedWriteHalf,
    frames: crate::pipeline::Frames,
    geometry: Geometry,
    params: LaserDetectParams,
    watchers: tokio::sync::broadcast::Sender<std::sync::Arc<LaserDot>>,
) -> anyhow::Result<()>
where
    R: tokio::io::AsyncRead + Unpin,
{
    tracing::info!(?params, "laser: looking");
    // Where the last dot was and when the whole frame was last scanned: the window's memory, one
    // per subscription, so a new one starts with a whole frame scan.
    let mut scope = Scope::default();
    let looks = Looks {
        name: "laser",
        search_hz: params.search_hz,
        track_hz: params.track_hz,
        seen: |d: &LaserDot| d.seen,
        notification,
    };
    serve_looks(
        reader,
        write,
        frames,
        looks,
        watchers,
        move |data, w, h, hz| {
            let started = std::time::Instant::now();
            let found = scope.look(data, w, h, &params, started);
            let took_ms = started.elapsed().as_secs_f64() * 1e3;
            dot(&found, &geometry, w, h, hz, took_ms)
        },
    )
    .await
}

/// What [`serve_looks`] needs to know about one detector beyond the scan itself.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub(crate) struct Looks<T> {
    /// For the thread's name and the journal.
    pub name: &'static str,
    pub search_hz: f64,
    pub track_hz: f64,
    /// Whether a look found its thing — which is what moves the rate to `track_hz`.
    pub seen: fn(&T) -> bool,
    /// The look as a line for the subscriber.
    pub notification: fn(&T) -> String,
}

/// A detector's subscription: one scan per look, paced, until the subscriber hangs up.
///
/// The looking runs on a thread of its own — waiting for a capture blocks, and the scan is
/// milliseconds of arithmetic that has no business on the runtime that serves WebRTC signalling —
/// and stops on the first answer it cannot hand over, which is what a closed connection becomes.
/// It looks at `search_hz` until something is seen and at `track_hz` for [`TRACK_HOLD`] after.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub(crate) async fn serve_looks<R, T, F>(
    mut reader: R,
    mut write: tokio::net::unix::OwnedWriteHalf,
    frames: crate::pipeline::Frames,
    looks: Looks<T>,
    watchers: tokio::sync::broadcast::Sender<std::sync::Arc<T>>,
    mut scan: F,
) -> anyhow::Result<()>
where
    R: tokio::io::AsyncRead + Unpin,
    T: Clone + Send + Sync + 'static,
    F: FnMut(&[u8], usize, usize, f64) -> T + Send + 'static,
{
    use std::time::{Duration, Instant};
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    let Looks {
        name,
        search_hz,
        track_hz,
        seen,
        notification,
    } = looks;
    let (tx, mut rx) = tokio::sync::mpsc::channel::<T>(4);
    std::thread::Builder::new()
        .name(name.into())
        .spawn(move || {
            let mut last_seen: Option<Instant> = None;
            let mut next = Instant::now();
            loop {
                let tracking = last_seen.is_some_and(|at| at.elapsed() < TRACK_HOLD);
                let hz = if tracking { track_hz } else { search_hz };
                let now = Instant::now();
                if next > now {
                    std::thread::sleep(next - now);
                }
                // From the deadline, so a slow look eats its own slot — but never queued up
                // behind a stall: a camera that went quiet for a second does not owe ten looks.
                next = (next + Duration::from_secs_f64(1.0 / hz.max(0.1))).max(Instant::now());

                let looked = frames.inspect_next(|frame| {
                    if frame.format != crate::pipeline::CAPTURE_FORMAT {
                        return None;
                    }
                    let (w, h) = (frame.width as usize, frame.height as usize);
                    Some(scan(&frame.data, w, h, hz))
                });
                let Some(Some(look)) = looked else {
                    // No frame in the capture timeout, or a format this does not know: say
                    // nothing, so the subscriber's own staleness check is what lets go.
                    if tx.is_closed() {
                        return;
                    }
                    continue;
                };
                if seen(&look) {
                    last_seen = Some(Instant::now());
                }
                // Nobody watching is the ordinary case, and not an error.
                let _ = watchers.send(std::sync::Arc::new(look.clone()));
                if tx.blocking_send(look).is_err() {
                    tracing::info!(detector = name, "subscriber gone, stopping");
                    return;
                }
            }
        })?;

    let mut scratch = [0u8; 256];
    loop {
        tokio::select! {
            look = rx.recv() => {
                let Some(look) = look else { return Ok(()) };
                let mut line = notification(&look);
                line.push('\n');
                write.write_all(line.as_bytes()).await?;
            }
            // Anything the subscriber sends is ignored; end of stream is it hanging up.
            read = reader.read(&mut scratch) => {
                if read? == 0 {
                    return Ok(());
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grey room, `w × h`, as UYVY.
    fn room(w: usize, h: usize, luma: u8) -> Vec<u8> {
        let mut data = vec![0u8; w * h * 2];
        for px in data.as_chunks_mut::<4>().0 {
            *px = [128, luma, 128, luma];
        }
        data
    }

    /// Paint a square of one colour, `size` pixels across, centred on (cx, cy).
    fn paint(data: &mut [u8], w: usize, cx: usize, cy: usize, size: usize, yuv: [u8; 3]) {
        let [y, u, v] = yuv;
        for py in cy - size / 2..cy + size / 2 {
            for m in (cx - size / 2) / 2..(cx + size / 2) / 2 {
                let i = py * w * 2 + m * 4;
                data[i..i + 4].copy_from_slice(&[u, y, v, y]);
            }
        }
    }

    const RED_RIM: [u8; 3] = [200, 110, 200];

    /// The ordinary case: a small, bright, red patch is found where it is.
    #[test]
    fn a_red_dot_is_found_at_its_centre() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 60, 8, RED_RIM);
        let found = find(&frame, w, h, &LaserDetectParams::default());
        let [x, y] = found.at.expect("the dot");
        assert!((x - 200.0).abs() < 2.0 && (y - 60.0).abs() < 2.0, "{x},{y}");
        assert_eq!(found.peak, Some(RED_RIM));

        // And as the console gets it: in the upright picture, with that picture's size, so the
        // overlay needs to know nothing about the mount.
        let dot = dot(&found, &geometry(Turn::Right), w, h, 15.0, 1.0);
        assert_eq!(dot.frame, Some([240, 320]));
        let [ux, uy] = dot.px.expect("upright");
        assert!(
            (ux - (239.0 - 60.0)).abs() < 2.0 && (uy - 200.0).abs() < 2.0,
            "{ux},{uy}"
        );
        assert_eq!(
            dot.candidate_px, None,
            "a dot that passed is not a near miss"
        );
    }

    /// **The tuning readout.** A lit room with nothing red in it finds no dot — but still says
    /// what the best candidate looked like, because that is what an operator reads to set the
    /// thresholds.
    #[test]
    fn nothing_found_still_reports_the_best_candidate() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 100, 100, 8, [250, 128, 128]); // a white lamp
        let found = find(&frame, w, h, &LaserDetectParams::default());
        assert_eq!(found.at, None);
        assert_eq!(found.peak, Some([250, 128, 128]));
    }

    /// A big red lit thing is not a laser: a frame-wide "brightest red" would chase a lamp.
    #[test]
    fn a_red_area_is_rejected_as_too_big() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 160, 120, 60, RED_RIM);
        let params = LaserDetectParams {
            max_pixels: 100,
            ..Default::default()
        };
        let found = find(&frame, w, h, &params);
        assert_eq!(found.at, None);
        assert_eq!(found.rejected, Some("too big"));
        // The overlay draws the near miss where it was.
        let dot = dot(&found, &geometry(Turn::None), w, h, 3.0, 1.0);
        let [cx, cy] = dot.candidate_px.expect("a near miss");
        assert!(
            (cx - 160.0).abs() < 31.0 && (cy - 120.0).abs() < 31.0,
            "{cx},{cy}"
        );
    }

    /// A stray hit across the frame must not drag the centre: only the best hit's neighbourhood
    /// counts.
    #[test]
    fn a_stray_hit_elsewhere_does_not_move_the_dot() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 60, 60, 8, [255, 100, 230]);
        paint(&mut frame, w, 260, 200, 4, [160, 130, 170]);
        let [x, y] = find(&frame, w, h, &LaserDetectParams::default())
            .at
            .expect("the dot");
        assert!((x - 60.0).abs() < 2.0 && (y - 60.0).abs() < 2.0, "{x},{y}");
    }

    const BRIGHTER: [u8; 3] = [255, 100, 230];

    fn near(found: &Found, x: f64, y: f64) -> bool {
        found
            .at
            .is_some_and(|[fx, fy]| (fx - x).abs() < 2.0 && (fy - y).abs() < 2.0)
    }

    /// Around a dot it already knows, a tracking look gives the answer a whole-frame scan gives,
    /// up to the frame's edge.
    #[test]
    fn the_window_finds_what_the_whole_frame_scan_finds() {
        let (w, h) = (1280, 720);
        for (x, y) in [(900, 500), (8, 8), (1272, 712)] {
            let mut frame = room(w, h, 90);
            paint(&mut frame, w, x, y, 8, RED_RIM);
            let whole = find(&frame, w, h, &LaserDetectParams::default());
            assert!(near(&whole, x as f64, y as f64), "{whole:?}");
            let window = [x as f64 + 30.0, y as f64 - 30.0];
            assert_eq!(
                find_near(&frame, w, h, &LaserDetectParams::default(), window),
                Some(whole.clone())
            );
        }
        // A centre off the frame, or not a number, is its nearest corner.
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 8, 8, 8, RED_RIM);
        let p = LaserDetectParams::default();
        let whole = find(&frame, w, h, &p);
        assert_eq!(find_near(&frame, w, h, &p, [f64::NAN, -5.0]), Some(whole));
    }

    /// A frame an odd number of pixels wide has a half macropixel at its right edge, which neither
    /// scan reads past.
    #[test]
    fn an_odd_width_is_scanned_up_to_its_last_whole_macropixel() {
        let (w, h) = (321, 240);
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 310, 120, 8, RED_RIM);
        let p = LaserDetectParams::default();
        let whole = find(&frame, w, h, &p);
        assert!(near(&whole, 310.0, 120.0), "{whole:?}");
        assert_eq!(find_near(&frame, w, h, &p, [310.0, 120.0]), Some(whole));
    }

    /// A candidate the window rejects is not its answer: what the whole frame says about it is.
    #[test]
    fn a_rejected_candidate_in_the_window_is_left_to_the_whole_frame() {
        let (w, h) = (1280, 720);
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 60, RED_RIM);
        let p = LaserDetectParams {
            max_pixels: 100,
            ..Default::default()
        };
        assert_eq!(find(&frame, w, h, &p).rejected, Some("too big"));
        assert_eq!(find_near(&frame, w, h, &p, [200.0, 200.0]), None);
    }

    /// **The saving.** The window is all a tracking look reads: a brighter dot outside it is
    /// not seen, and a dot that left it is not found there but left to the whole-frame scan.
    #[test]
    fn the_window_does_not_look_past_its_edge() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, RED_RIM);
        paint(&mut frame, w, 1000, 600, 8, BRIGHTER);
        assert!(near(&find(&frame, w, h, &p), 1000.0, 600.0));
        let tracked = find_near(&frame, w, h, &p, [200.0, 200.0]).expect("the tracked dot");
        assert!(near(&tracked, 200.0, 200.0), "{tracked:?}");
        assert_eq!(tracked.peak, Some(RED_RIM), "the window's peak");

        let edge = (WINDOW_RADII * p.radius_px as usize) as f64;
        assert_eq!(
            find_near(&frame, w, h, &p, [200.0 + edge + 20.0, 200.0]),
            None
        );
    }

    /// Tracking reads the window, and the whole frame once a second: so a brighter dot elsewhere
    /// takes over within that second, as it would have on the next search look.
    #[test]
    fn a_brighter_dot_elsewhere_takes_over_within_a_second() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, RED_RIM);
        assert!(near(&scope.look(&frame, w, h, &p, t0), 200.0, 200.0));

        paint(&mut frame, w, 1000, 600, 8, BRIGHTER);
        let ms = std::time::Duration::from_millis;
        for t in [ms(66), ms(500), ms(933)] {
            let found = scope.look(&frame, w, h, &p, t0 + t);
            assert!(near(&found, 200.0, 200.0), "at {t:?}: {found:?}");
        }
        let found = scope.look(&frame, w, h, &p, t0 + FULL_SCAN_EVERY);
        assert!(near(&found, 1000.0, 600.0), "{found:?}");
    }

    /// A dot that jumped out of the window between two looks is found by the same look.
    #[test]
    fn a_dot_that_jumps_is_found_in_the_same_look() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, RED_RIM);
        scope.look(&frame, w, h, &p, t0);

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 1000, 600, 8, RED_RIM);
        let found = scope.look(&frame, w, h, &p, t0 + std::time::Duration::from_millis(66));
        assert!(near(&found, 1000.0, 600.0), "{found:?}");
    }

    /// The laser jumps away and leaves a red thing inside the window, further than
    /// [`FOLLOW_RADII`] from where the dot was: too far to be the dot having moved, so the whole
    /// frame decides, in the same look. (One closer than that is indistinguishable from the dot
    /// moving, and holds until the next whole-frame scan.)
    #[test]
    fn a_red_thing_left_in_the_window_away_from_the_dot_does_not_take_over() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, BRIGHTER);
        paint(&mut frame, w, 260, 200, 8, RED_RIM);
        assert!(near(&scope.look(&frame, w, h, &p, t0), 200.0, 200.0));

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 260, 200, 8, RED_RIM);
        paint(&mut frame, w, 1000, 600, 8, BRIGHTER);
        let found = scope.look(&frame, w, h, &p, t0 + std::time::Duration::from_millis(66));
        assert!(near(&found, 1000.0, 600.0), "{found:?}");
    }

    /// `peak` is the frame's between whole-frame scans too: a redder light away from the dot that
    /// fails the thresholds stays in the readout, instead of showing once a second.
    #[test]
    fn the_readout_keeps_the_whole_frame_peak_between_scans() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();
        let pink = [255, 150, 255]; // redder than the dot, too blue to pass `u_max`

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, RED_RIM);
        paint(&mut frame, w, 1000, 600, 8, pink);
        let ms = std::time::Duration::from_millis;
        for t in [ms(0), ms(66), ms(500), ms(1000), ms(1066)] {
            let found = scope.look(&frame, w, h, &p, t0 + t);
            assert!(near(&found, 200.0, 200.0), "at {t:?}: {found:?}");
            assert_eq!(found.peak, Some(pink), "at {t:?}");
        }
    }

    /// **The saving, in the ordinary case.** A dot that moves a little between looks is followed in
    /// the window, with no whole-frame scan until the second is up.
    #[test]
    fn a_dot_moving_inside_the_window_costs_no_whole_frame_scan() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();
        let ms = std::time::Duration::from_millis;

        for (i, (x, y)) in [(200, 200), (230, 210), (260, 220), (290, 230)]
            .into_iter()
            .enumerate()
        {
            let mut frame = room(w, h, 90);
            paint(&mut frame, w, x, y, 8, RED_RIM);
            let found = scope.look(&frame, w, h, &p, t0 + ms(66 * i as u64));
            assert!(near(&found, x as f64, y as f64), "look {i}: {found:?}");
            assert_eq!(scope.full, Some(t0), "look {i} scanned the whole frame");
        }
    }

    /// Inside the window the readout is this look's: the dot's colour from the last whole scan does
    /// not outlive the dot, whatever it changed to.
    #[test]
    fn the_readout_inside_the_window_is_the_current_look() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, BRIGHTER);
        assert_eq!(scope.look(&frame, w, h, &p, t0).peak, Some(BRIGHTER));

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 210, 200, 8, RED_RIM);
        let found = scope.look(&frame, w, h, &p, t0 + std::time::Duration::from_millis(66));
        assert!(near(&found, 210.0, 200.0), "{found:?}");
        assert_eq!(found.peak, Some(RED_RIM));
    }

    /// **The tuning readout keeps working.** A tracking look that loses the dot reports what a
    /// whole-frame scan would: the peak anywhere in the picture, not just in the window.
    #[test]
    fn a_miss_while_tracking_reports_the_whole_frame() {
        let (w, h) = (1280, 720);
        let p = LaserDetectParams::default();
        let t0 = std::time::Instant::now();
        let mut scope = Scope::default();

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 200, 200, 8, RED_RIM);
        scope.look(&frame, w, h, &p, t0);

        let mut frame = room(w, h, 90);
        paint(&mut frame, w, 1000, 600, 8, [250, 128, 128]); // a white lamp, far off
        let found = scope.look(&frame, w, h, &p, t0 + std::time::Duration::from_millis(66));
        assert_eq!(found.at, None);
        assert_eq!(found.peak, Some([250, 128, 128]));
    }

    fn geometry(turn: Turn) -> Geometry {
        Geometry {
            fx: 100.0,
            fy: 100.0,
            cx: 160.0,
            cy: 120.0,
            distortion: Vec::new(),
            turn,
        }
    }

    /// **The orientation, which is what the old runtime got wrong per variant.** On a camera
    /// mounted a quarter turn clockwise, a dot to the *right* of the delivered frame's centre is
    /// *below* centre once upright — so the ray must point down (+y), and its pixel must agree
    /// with where `uyvy::Turn` puts that same pixel.
    #[test]
    fn the_ray_is_in_the_upright_camera_frame() {
        let g = geometry(Turn::Right);
        let ray = g.ray(260.0, 120.0); // 100 px right of centre in the raw frame
        assert!(
            (ray[0]).abs() < 1e-9 && (ray[1] - 1.0).abs() < 1e-9,
            "{ray:?}"
        );
        // A dot above centre in the raw frame is to the right once upright.
        let ray = g.ray(160.0, 20.0);
        assert!(
            (ray[0] - 1.0).abs() < 1e-9 && ray[1].abs() < 1e-9,
            "{ray:?}"
        );

        // The pixel readout and the ray agree on which way is which.
        let [ux, uy] = g.upright_px(260.0, 120.0, 320, 240);
        let (uw, uh) = Turn::Right.upright(320, 240);
        assert!(uy > uh as f64 / 2.0 + 50.0, "below centre: {uy}");
        assert!((ux - (uw as f64 - 1.0) / 2.0).abs() < 1.0, "centred: {ux}");

        // Unturned is the identity.
        assert_eq!(geometry(Turn::None).ray(260.0, 120.0), [1.0, 0.0]);
    }

    /// Undistortion inverts the model it undoes.
    #[test]
    fn undistortion_inverts_the_distortion_model() {
        let mut g = geometry(Turn::None);
        g.distortion = vec![-0.3, 0.1, 0.001, -0.002, 0.0];
        let (x, y) = (0.4f64, -0.25f64);
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (-0.3 + r2 * 0.1);
        let xd = x * radial + 2.0 * 0.001 * x * y + -0.002 * (r2 + 2.0 * x * x);
        let yd = y * radial + 0.001 * (r2 + 2.0 * y * y) + 2.0 * -0.002 * x * y;
        let [ux, uy] = g.ray(xd * 100.0 + 160.0, yd * 100.0 + 120.0);
        assert!((ux - x).abs() < 1e-4 && (uy - y).abs() < 1e-4, "{ux},{uy}");
    }
}
