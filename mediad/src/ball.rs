//! Finding the purple ball in the frames already on the tee — the laser detector's sibling, for a
//! 70 mm ball rather than a dot.
//!
//! Everything about *how* it looks is [`crate::laser`]'s: frames read in place from the tee at the
//! stream's own exposure, a direction leaving rather than a pixel (the same [`Geometry`]), and
//! [`crate::laser::serve_looks`] pacing it at `search_hz` until something is seen. What differs is
//! what a ball is.
//!
//! **Colour.** Purple is the one hue with *both* chromas above grey: blue has V under 128, red,
//! wood and skin have U under it. So a ball pixel is one whose U and V each reach a minimum, and
//! is bright enough for its chroma to be colour rather than sensor noise. Every answer carries the
//! most purple sample in the frame whether it passed or not — the number tuning reads.
//!
//! **Shape.** A purple sock is not a ball. The frame is cut into 4×4-pixel cells, one sample each
//! (about 58 k at 720p, the laser's cost), the passing cells are grouped into blobs by
//! 4-connectivity, and a blob is a ball when it is big enough, fills enough of its bounding box (a
//! disc fills π/4), and is about as tall as it is wide. The largest blob that passes wins. A blob
//! against the frame's edge is a ball partly out of the picture — which is where the ball *is*
//! for the last steps of an approach — so it skips the aspect test, and its centre and radius are
//! taken from the sides that are not cut.

use duck_ipc_proto::{BallDetectParams, BallSeen};

use crate::laser::Geometry;

/// One sample per cell this many pixels on a side. A ball worth walking to is several cells
/// across: 70 mm at 2 m is about 20 pixels on this lens.
pub const CELL: usize = 4;

/// A blob of passing cells, in cell coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Blob {
    cells: u32,
    min: [usize; 2],
    max: [usize; 2],
}

/// One scan's answer, in the delivered frame's pixels.
#[derive(Debug, Clone, PartialEq)]
pub struct Found {
    /// The ball's centre and radius, when one passed.
    pub at: Option<([f64; 2], f64)>,
    /// The biggest blob's centre, when it was rejected.
    pub candidate: Option<[f64; 2]>,
    /// Cells in the ball, or in the rejected blob.
    pub cells: u32,
    /// The most purple bright sample, `[Y, U, V]`, passed or not.
    pub peak: Option<[u8; 3]>,
    pub rejected: Option<&'static str>,
}

/// The sample for cell `(cx, cy)`: the macropixel at the cell's top-left, `(Y, U, V)` with the
/// brighter of its two lumas.
#[inline]
fn sample(data: &[u8], stride: usize, cx: usize, cy: usize) -> (u8, u8, u8) {
    let i = cy * CELL * stride + cx * CELL * 2;
    // U Y0 V Y1
    (data[i + 1].max(data[i + 3]), data[i], data[i + 2])
}

#[inline]
fn hit(y: u8, u: u8, v: u8, p: &BallDetectParams) -> bool {
    y >= p.y_min && u >= p.u_min && v >= p.v_min
}

/// How purple a sample is, for the peak: the lesser of its two chromas' lift above grey.
#[inline]
fn purple(u: u8, v: u8) -> i32 {
    (i32::from(u) - 128).min(i32::from(v) - 128)
}

/// Find the ball in one `UYVY` frame, `width × height`, tightly packed.
pub fn find(data: &[u8], width: usize, height: usize, p: &BallDetectParams) -> Found {
    let none = Found {
        at: None,
        candidate: None,
        cells: 0,
        peak: None,
        rejected: None,
    };
    let stride = width * 2;
    if width < CELL || height < CELL || data.len() < stride * height {
        return none;
    }
    let (gw, gh) = (width / CELL, height / CELL);

    let mut mask = vec![false; gw * gh];
    let mut peak: Option<(i32, [u8; 3])> = None;
    for cy in 0..gh {
        for cx in 0..gw {
            let (y, u, v) = sample(data, stride, cx, cy);
            if y >= p.y_min {
                let s = purple(u, v);
                if peak.is_none_or(|(ps, _)| s > ps) {
                    peak = Some((s, [y, u, v]));
                }
            }
            mask[cy * gw + cx] = hit(y, u, v, p);
        }
    }
    let peak = peak.map(|(_, yuv)| yuv);

    // Blobs by flood fill, clearing the mask as it goes so each cell is visited once.
    let mut blobs = Vec::new();
    let mut stack = Vec::new();
    for start in 0..mask.len() {
        if !mask[start] {
            continue;
        }
        mask[start] = false;
        stack.push(start);
        let mut blob = Blob {
            cells: 0,
            min: [usize::MAX; 2],
            max: [0; 2],
        };
        while let Some(i) = stack.pop() {
            let (x, y) = (i % gw, i / gw);
            blob.cells += 1;
            blob.min = [blob.min[0].min(x), blob.min[1].min(y)];
            blob.max = [blob.max[0].max(x), blob.max[1].max(y)];
            let mut push = |j: usize| {
                if mask[j] {
                    mask[j] = false;
                    stack.push(j);
                }
            };
            if x > 0 {
                push(i - 1);
            }
            if x + 1 < gw {
                push(i + 1);
            }
            if y > 0 {
                push(i - gw);
            }
            if y + 1 < gh {
                push(i + gw);
            }
        }
        blobs.push(blob);
    }
    // Biggest first: the first that passes is the ball, and the first at all is what a rejection
    // reports.
    blobs.sort_by_key(|b| std::cmp::Reverse(b.cells));
    let Some(biggest) = blobs.first().copied() else {
        return Found { peak, ..none };
    };
    for blob in blobs.iter().take_while(|b| b.cells >= p.min_cells) {
        if shape(blob, gw, gh, p).is_none() {
            let (centre, radius) = circle(blob, gw, gh);
            return Found {
                at: Some((centre, radius)),
                candidate: None,
                cells: blob.cells,
                peak,
                rejected: None,
            };
        }
    }
    let rejected = if biggest.cells < p.min_cells {
        "too small"
    } else {
        shape(&biggest, gw, gh, p).unwrap_or("not round")
    };
    Found {
        at: None,
        candidate: Some(circle(&biggest, gw, gh).0),
        cells: biggest.cells,
        peak,
        rejected: Some(rejected),
    }
}

/// Why a blob is not a ball, by shape — `None` when it is one.
fn shape(b: &Blob, gw: usize, gh: usize, p: &BallDetectParams) -> Option<&'static str> {
    let (w, h) = (
        (b.max[0] - b.min[0] + 1) as f64,
        (b.max[1] - b.min[1] + 1) as f64,
    );
    if f64::from(b.cells) / (w * h) < p.min_fill {
        return Some("not filled");
    }
    if !touches_edge(b, gw, gh) && w.max(h) / w.min(h) > p.max_aspect {
        return Some("not round");
    }
    None
}

fn touches_edge(b: &Blob, gw: usize, gh: usize) -> bool {
    b.min[0] == 0 || b.min[1] == 0 || b.max[0] + 1 == gw || b.max[1] + 1 == gh
}

/// A blob's circle — centre and radius in the delivered frame's pixels.
///
/// The radius is half the longer side, since a ball cut by the frame's edge loses its extent
/// across the cut and keeps it along it. The centre is the box's middle on an axis the edge does
/// not cut, and one radius in from the side that is whole on an axis it does.
fn circle(b: &Blob, gw: usize, gh: usize) -> ([f64; 2], f64) {
    let side = |axis: usize| (b.max[axis] - b.min[axis] + 1) as f64;
    let r = side(0).max(side(1)) / 2.0;
    let centre = |axis: usize, cells: usize| {
        let (lo, hi) = (b.min[axis] as f64, b.max[axis] as f64 + 1.0);
        let (cut_lo, cut_hi) = (b.min[axis] == 0, b.max[axis] + 1 == cells);
        match (cut_lo, cut_hi) {
            (true, false) => hi - r,
            (false, true) => lo + r,
            _ => (lo + hi) / 2.0,
        }
    };
    let c = CELL as f64;
    ([centre(0, gw) * c, centre(1, gh) * c], r * c)
}

/// [`find`]'s answer, as the wire carries it.
pub fn seen(
    found: &Found,
    geometry: &Geometry,
    width: usize,
    height: usize,
    hz: f64,
    took_ms: f64,
) -> BallSeen {
    let upright = |[u, v]: [f64; 2]| geometry.upright_px(u, v, width, height);
    // The apparent diameter as the distance between the rays through the ball's two sides, so the
    // mount and the lens are undone the same way the centre's are.
    let width_of = |[u, v]: [f64; 2], r: f64| {
        let (a, b) = (geometry.ray(u - r, v), geometry.ray(u + r, v));
        (a[0] - b[0]).hypot(a[1] - b[1])
    };
    BallSeen {
        seen: found.at.is_some(),
        ray: found.at.map(|([u, v], _)| geometry.ray(u, v)),
        width: found.at.map(|(c, r)| width_of(c, r)),
        px: found.at.map(|(c, _)| upright(c)),
        radius_px: found.at.map(|(_, r)| r),
        frame: Some({
            let (w, h) = geometry.turn.upright(width, height);
            [w as u32, h as u32]
        }),
        candidate_px: found.candidate.map(upright),
        cells: found.cells,
        peak_yuv: found.peak,
        rejected: found.rejected.map(str::to_owned),
        hz,
        took_ms,
    }
}

/// One look as the console reads it, the laser's [`crate::laser::notification`] for a ball.
pub fn notification(seen: &BallSeen) -> String {
    serde_json::json!({
        "jsonrpc": "2.0",
        "method": duck_ipc_proto::method::MEDIA_BALL_SEEN,
        "params": seen,
    })
    .to_string()
}

/// The local subscription: answer, then one [`duck_ipc_proto::method::MEDIA_BALL_SEEN`] per look
/// until the subscriber hangs up.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub async fn serve<R>(
    reader: R,
    write: tokio::net::unix::OwnedWriteHalf,
    frames: crate::pipeline::Frames,
    geometry: Geometry,
    params: BallDetectParams,
    watchers: tokio::sync::broadcast::Sender<std::sync::Arc<BallSeen>>,
) -> anyhow::Result<()>
where
    R: tokio::io::AsyncRead + Unpin,
{
    tracing::info!(?params, "ball: looking");
    let looks = crate::laser::Looks {
        name: "ball",
        search_hz: params.search_hz,
        track_hz: params.track_hz,
        seen: |s: &BallSeen| s.seen,
        notification,
    };
    crate::laser::serve_looks(
        reader,
        write,
        frames,
        looks,
        watchers,
        move |data, w, h, hz| {
            let started = std::time::Instant::now();
            let found = find(data, w, h, &params);
            let took_ms = started.elapsed().as_secs_f64() * 1e3;
            seen(&found, &geometry, w, h, hz, took_ms)
        },
    )
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use uyvy::Turn;

    /// A grey room, `w × h`, as UYVY.
    fn room(w: usize, h: usize, luma: u8) -> Vec<u8> {
        let mut data = vec![0u8; w * h * 2];
        for px in data.as_chunks_mut::<4>().0 {
            *px = [128, luma, 128, luma];
        }
        data
    }

    /// Paint a disc of one colour, radius `r` pixels, centred on (cx, cy) — clipped to the frame.
    fn disc(data: &mut [u8], w: usize, h: usize, cx: f64, cy: f64, r: f64, yuv: [u8; 3]) {
        let [y, u, v] = yuv;
        for py in 0..h {
            for m in 0..w / 2 {
                let px = (m * 2) as f64 + 0.5;
                if (px - cx).hypot(py as f64 - cy) <= r {
                    let i = py * w * 2 + m * 4;
                    data[i..i + 4].copy_from_slice(&[u, y, v, y]);
                }
            }
        }
    }

    /// Paint a rectangle.
    fn rect(data: &mut [u8], w: usize, x: [usize; 2], y: [usize; 2], yuv: [u8; 3]) {
        let [ly, u, v] = yuv;
        for py in y[0]..y[1] {
            for m in x[0] / 2..x[1] / 2 {
                let i = py * w * 2 + m * 4;
                data[i..i + 4].copy_from_slice(&[u, ly, v, ly]);
            }
        }
    }

    /// A purple ball in room light: RGB about (110, 60, 160).
    const PURPLE: [u8; 3] = [86, 169, 145];

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

    /// The ordinary case: a purple disc is found at its centre, at about its size.
    #[test]
    fn a_purple_disc_is_found_where_it_is() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 120);
        disc(&mut frame, w, h, 200.0, 80.0, 20.0, PURPLE);
        let found = find(&frame, w, h, &BallDetectParams::default());
        let ([x, y], r) = found.at.expect("the ball");
        assert!((x - 200.0).abs() < 4.0 && (y - 80.0).abs() < 4.0, "{x},{y}");
        assert!((r - 20.0).abs() < 4.0, "radius {r}");
        assert_eq!(found.peak, Some(PURPLE));

        // On the wire: a width about the diameter over fx, since the lens here is ideal.
        let seen = seen(&found, &geometry(Turn::None), w, h, 5.0, 1.0);
        let width = seen.width.unwrap();
        assert!((width - 0.4).abs() < 0.1, "{width}");
        assert_eq!(seen.ray, Some([(x - 160.0) / 100.0, (y - 120.0) / 100.0]));
    }

    /// Blue, red and skin are not purple: each has one chroma below grey.
    #[test]
    fn neighbouring_hues_are_not_the_ball() {
        let (w, h) = (320, 240);
        for (name, yuv) in [
            ("blue", [70, 180, 110]),
            ("red", [80, 100, 200]),
            ("skin", [180, 105, 150]),
            ("grey", [120, 128, 128]),
        ] {
            let mut frame = room(w, h, 120);
            disc(&mut frame, w, h, 160.0, 120.0, 25.0, yuv);
            let found = find(&frame, w, h, &BallDetectParams::default());
            assert_eq!(found.at, None, "{name}");
        }
    }

    /// A purple stripe is not a ball, and says why; a ball elsewhere in the same frame still is.
    #[test]
    fn a_long_purple_thing_is_not_round() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 120);
        rect(&mut frame, w, [20, 300], [20, 36], PURPLE);
        let found = find(&frame, w, h, &BallDetectParams::default());
        assert_eq!(found.at, None);
        assert_eq!(found.rejected, Some("not round"));
        assert!(found.candidate.is_some());

        disc(&mut frame, w, h, 160.0, 160.0, 14.0, PURPLE);
        let ([x, y], _) = find(&frame, w, h, &BallDetectParams::default())
            .at
            .expect("the ball, smaller than the stripe");
        assert!(
            (x - 160.0).abs() < 4.0 && (y - 160.0).abs() < 4.0,
            "{x},{y}"
        );
    }

    /// A speck is too small.
    #[test]
    fn a_speck_is_too_small() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 120);
        disc(&mut frame, w, h, 100.0, 100.0, 3.0, PURPLE);
        let found = find(&frame, w, h, &BallDetectParams::default());
        assert_eq!(found.at, None);
        assert_eq!(found.rejected, Some("too small"));
    }

    /// **Under the beak.** The last steps of an approach have the ball cut off by the bottom of
    /// the picture. It is still the ball, and its centre is still a radius up from the bottom of
    /// the part that shows — below the box's middle, where a centroid would put it.
    #[test]
    fn a_ball_cut_by_the_frame_edge_keeps_its_centre() {
        let (w, h) = (320, 240);
        let mut frame = room(w, h, 120);
        disc(&mut frame, w, h, 160.0, 235.0, 30.0, PURPLE);
        let ([x, y], r) = find(&frame, w, h, &BallDetectParams::default())
            .at
            .expect("the ball, cut");
        assert!((r - 30.0).abs() < 5.0, "radius {r}");
        assert!((x - 160.0).abs() < 4.0, "{x}");
        assert!(
            (y - 235.0).abs() < 6.0,
            "centre {y}, a radius up from the cut"
        );
    }
}
