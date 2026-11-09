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
//! and column of it: about 230 k samples at 720p, a couple of milliseconds on the board. Paced at
//! `search_hz` until something is seen and `track_hz` while it is — so a tracker left on all day
//! costs a few looks a second.

use duck_ipc_proto::{LaserDetectParams, LaserDot};
use uyvy::Turn;

/// Every other row and every other pixel. A dot at any distance worth walking to covers more than
/// a 2×2 block, and halving both axes is a quarter of the work.
const STEP: usize = 2;

/// How long after the last sighting the detector stays at its tracking rate. Long enough to ride
/// over a dot that blinks out for a frame or two, short enough that a laser put away stops costing.
pub const TRACK_HOLD: std::time::Duration = std::time::Duration::from_secs(1);

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
    /// The reddest bright sample in the frame, `[Y, U, V]`, passed or not.
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
    let stride = width * 2;
    let none = Found {
        at: None,
        candidate: None,
        pixels: 0,
        peak: None,
        rejected: None,
    };
    if width < 2 || height == 0 || data.len() < stride * height {
        return none;
    }
    let macros = width / 2;

    let mut peak: Option<(i32, [u8; 3])> = None;
    let mut best: Option<(i32, usize, usize)> = None;
    for py in (0..height).step_by(STEP) {
        let row = &data[py * stride..(py + 1) * stride];
        // One sample per macropixel is every other pixel, which is the column step.
        for m in 0..macros {
            let (y, u, v) = sample(row, m);
            let s = score(y, u, v);
            if peak.is_none_or(|(ps, _)| s > ps) {
                peak = Some((s, [y, u, v]));
            }
            if hit(y, u, v, p) && best.is_none_or(|(bs, _, _)| s > bs) {
                best = Some((s, m * 2, py));
            }
        }
    }
    let peak = peak.map(|(_, yuv)| yuv);
    let Some((_, bx, by)) = best else {
        return Found { peak, ..none };
    };

    let r = p.radius_px as usize;
    let (x0, x1) = (bx.saturating_sub(r), (bx + r).min(width - 1));
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

/// Where the laser detector's answers are shared, beyond the subscriber that asked for them.
///
/// The geometry `main` fills in once the intrinsics are known, and a broadcast of every look, so a
/// console watching the video can draw what the detector is seeing. A look is only ever taken for a
/// subscriber — `robotd` while tracking is on — so with tracking off this carries nothing, and a
/// console with nobody to listen costs the detector a failed `send`.
pub struct Hub {
    pub geometry: std::sync::OnceLock<Geometry>,
    pub dots: tokio::sync::broadcast::Sender<std::sync::Arc<LaserDot>>,
}

impl Default for Hub {
    fn default() -> Self {
        Self {
            geometry: std::sync::OnceLock::new(),
            // A handful: a console that falls behind wants the newest dot, not a queue of old ones.
            dots: tokio::sync::broadcast::channel(4).0,
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
/// until the subscriber hangs up.
///
/// The looking runs on a thread of its own — waiting for a capture blocks, and the scan is
/// milliseconds of arithmetic that has no business on the runtime that serves WebRTC signalling —
/// and stops on the first answer it cannot hand over, which is what a closed connection becomes.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub async fn serve<R>(
    mut reader: R,
    mut write: tokio::net::unix::OwnedWriteHalf,
    frames: crate::pipeline::Frames,
    geometry: Geometry,
    params: LaserDetectParams,
    watchers: tokio::sync::broadcast::Sender<std::sync::Arc<LaserDot>>,
) -> anyhow::Result<()>
where
    R: tokio::io::AsyncRead + Unpin,
{
    use std::time::{Duration, Instant};
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    let (tx, mut rx) = tokio::sync::mpsc::channel::<LaserDot>(4);
    tracing::info!(?params, "laser: looking");
    std::thread::Builder::new()
        .name("laser".into())
        .spawn(move || {
            let mut last_seen: Option<Instant> = None;
            let mut next = Instant::now();
            loop {
                let tracking = last_seen.is_some_and(|at| at.elapsed() < TRACK_HOLD);
                let hz = if tracking {
                    params.track_hz
                } else {
                    params.search_hz
                };
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
                    let started = Instant::now();
                    let (w, h) = (frame.width as usize, frame.height as usize);
                    let found = find(&frame.data, w, h, &params);
                    let took_ms = started.elapsed().as_secs_f64() * 1e3;
                    Some(dot(&found, &geometry, w, h, hz, took_ms))
                });
                let Some(Some(dot)) = looked else {
                    // No frame in the capture timeout, or a format this does not know: say
                    // nothing, so the subscriber's own staleness check is what lets go.
                    if tx.is_closed() {
                        return;
                    }
                    continue;
                };
                if dot.seen {
                    last_seen = Some(Instant::now());
                }
                // Nobody watching is the ordinary case, and not an error.
                let _ = watchers.send(std::sync::Arc::new(dot.clone()));
                if tx.blocking_send(dot).is_err() {
                    tracing::info!("laser: subscriber gone, stopping");
                    return;
                }
            }
        })?;

    let mut scratch = [0u8; 256];
    loop {
        tokio::select! {
            dot = rx.recv() => {
                let Some(dot) = dot else { return Ok(()) };
                let mut line = notification(&dot);
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
