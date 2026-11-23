//! Finding other Microducks in this Microduck's camera, on the NPU.
//!
//! The model is trained in [`duck_detector`](https://github.com/pollen-robotics/duck_detector) and
//! arrives here as an INT8 `.rknn`: one class, 320×320 in, 2100 candidate boxes out. This crate is
//! the three things between a camera frame and a bounding box — the letterbox, the runtime, and the
//! decode — plus `duck-bench`, which measures them on a real board.
//!
//! The sampler that turns a `UYVY` frame the right way up and shrinks it is not here: it is in
//! [`uyvy`], because `mediad` and `robotctl monitor` need the same arithmetic and must not link a
//! model runtime to get it.
//!
//! **Everything here has to agree with how the model was trained**, and nothing enforces that
//! across the two repositories except this comment and the numbers below:
//!
//! * frames are letterboxed, not stretched, into a square, padded with 114 grey;
//! * RGB, not BGR;
//! * the picture is the one `mediad` has already turned the right way up (`--rotate`, default 90°),
//!   because that is what the dataset was captured through.
//!
//! Get one of those wrong and the detector does not fail — it just gets quietly worse, which is the
//! failure mode this crate is most exposed to.

pub mod onnx;
pub mod rknn;

// Its own signatures speak in these: `letterbox_rgb` returns a [`Letterbox`] and `decode` takes
// one back. The samplers themselves are not re-exported — a consumer that wants pixels wants
// [`uyvy`], not a second name for it here.
pub use uyvy::{Letterbox, PAD};

/// What the head emits per candidate: cx, cy, w, h, score.
const STRIDE: usize = 5;

/// One detection, in the coordinates of the frame that went in — not of the letterbox.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    pub score: f32,
    /// Pixels, `x0 y0 x1 y1`, in the original frame.
    pub box_: [f32; 4],
}

impl Detection {
    pub fn width(&self) -> f32 {
        self.box_[2] - self.box_[0]
    }

    pub fn height(&self) -> f32 {
        self.box_[3] - self.box_[1]
    }

    /// Where the duck is, across the frame: -1 hard left, 0 straight ahead, 1 hard right.
    ///
    /// The one number a behaviour actually wants — "turn towards it" needs a bearing, not a box.
    pub fn bearing(&self, frame_width: f32) -> f32 {
        let centre = (self.box_[0] + self.box_[2]) / 2.0;
        (centre / frame_width) * 2.0 - 1.0
    }
}

/// Scale to fit and pad to square, writing NHWC RGB bytes into `out`.
///
/// Nearest-neighbour on purpose: this runs per frame beside a 50 Hz control loop, the input is a
/// blurred 720×1280 photograph of a room, and a bilinear resize costs three times as much to move a
/// box by a pixel. If a measurement ever says the accuracy is worth it, the RGA can do it for free.
pub fn letterbox_rgb(
    frame: &[u8],
    width: usize,
    height: usize,
    size: usize,
    out: &mut Vec<u8>,
) -> Letterbox {
    let scale = (size as f32 / width as f32).min(size as f32 / height as f32);
    let fitted_w = ((width as f32 * scale).round() as usize).max(1).min(size);
    let fitted_h = ((height as f32 * scale).round() as usize).max(1).min(size);
    let pad_x = (size - fitted_w) / 2;
    let pad_y = (size - fitted_h) / 2;

    out.clear();
    out.resize(size * size * 3, PAD);
    for y in 0..fitted_h {
        // Nearest source row/column, computed from the fitted size so rounding cannot walk off the
        // end of the source on the last row.
        let source_y = (y * height) / fitted_h.max(1);
        for x in 0..fitted_w {
            let source_x = (x * width) / fitted_w.max(1);
            let source = (source_y * width + source_x) * 3;
            let target = ((y + pad_y) * size + (x + pad_x)) * 3;
            out[target..target + 3].copy_from_slice(&frame[source..source + 3]);
        }
    }

    Letterbox {
        scale,
        pad_x: pad_x as f32,
        pad_y: pad_y as f32,
    }
}

/// Candidates over `threshold`, suppressed, and mapped back to the original frame.
///
/// **The head does not suppress anything.** 2100 candidates means one duck comes back as twenty
/// overlapping boxes, and every consumer of this crate would otherwise have to know that. The
/// threshold is a property of *this* model, quantised: an INT8 output tensor carries its own scale,
/// so a value that means 0.9 on the float model is not 0.9 here. Tune it against the board.
pub fn decode(raw: &[f32], letterbox: Letterbox, threshold: f32, iou_limit: f32) -> Vec<Detection> {
    let candidates = raw.len() / STRIDE;
    let mut found: Vec<Detection> = Vec::new();

    // The tensor is `[1, 5, N]`: all the cx values, then all the cy values, and so on — not five
    // numbers per box. Reading it as interleaved gives boxes that are almost plausible, which is
    // the worst kind of wrong.
    for index in 0..candidates {
        let score = raw[4 * candidates + index];
        if score < threshold {
            continue;
        }
        let cx = raw[index];
        let cy = raw[candidates + index];
        let w = raw[2 * candidates + index];
        let h = raw[3 * candidates + index];
        // Out of the letterbox, back into the frame the camera took.
        let unpad = |value: f32, pad: f32| (value - pad) / letterbox.scale;
        found.push(Detection {
            score,
            box_: [
                unpad(cx - w / 2.0, letterbox.pad_x),
                unpad(cy - h / 2.0, letterbox.pad_y),
                unpad(cx + w / 2.0, letterbox.pad_x),
                unpad(cy + h / 2.0, letterbox.pad_y),
            ],
        });
    }

    found.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut kept: Vec<Detection> = Vec::new();
    for detection in found {
        if kept
            .iter()
            .all(|other| iou(&detection.box_, &other.box_) < iou_limit)
        {
            kept.push(detection);
        }
    }
    kept
}

fn iou(a: &[f32; 4], b: &[f32; 4]) -> f32 {
    let x0 = a[0].max(b[0]);
    let y0 = a[1].max(b[1]);
    let x1 = a[2].min(b[2]);
    let y1 = a[3].min(b[3]);
    let overlap = (x1 - x0).max(0.0) * (y1 - y0).max(0.0);
    let area = |r: &[f32; 4]| (r[2] - r[0]).max(0.0) * (r[3] - r[1]).max(0.0);
    let union = area(a) + area(b) - overlap;
    if union <= 0.0 { 0.0 } else { overlap / union }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tall frame fits inside the square with grey left and right, and nothing stretches.
    ///
    /// The Zero 3W's frame is portrait once its quarter turn is undone — 720×1280 — and getting the
    /// padding wrong moves every box by a constant nobody notices until the robot reaches for a
    /// duck that is not there.
    #[test]
    fn a_portrait_frame_is_padded_left_and_right() {
        // 4×8 red, into a 8×8 square: scale 1.0, 2 columns of padding each side.
        let frame = vec![255u8; 4 * 8 * 3];
        let mut out = Vec::new();
        let fit = letterbox_rgb(&frame, 4, 8, 8, &mut out);
        assert_eq!(out.len(), 8 * 8 * 3);
        assert_eq!((fit.scale, fit.pad_x, fit.pad_y), (1.0, 2.0, 0.0));
        // Left column is padding, middle is the frame.
        assert_eq!(&out[0..3], &[PAD, PAD, PAD]);
        assert_eq!(&out[2 * 3..2 * 3 + 3], &[255, 255, 255]);
        // And the last column is padding again.
        assert_eq!(&out[7 * 3..7 * 3 + 3], &[PAD, PAD, PAD]);
    }

    /// A wide frame fits with grey above and below — the beta boards' case: their camera is
    /// upright, so the frame stays 1280×720 landscape.
    #[test]
    fn a_landscape_frame_is_padded_above_and_below() {
        // 8×4 white, into an 8×8 square: scale 1.0, 2 rows of padding top and bottom.
        let frame = vec![255u8; 8 * 4 * 3];
        let mut out = Vec::new();
        let fit = letterbox_rgb(&frame, 8, 4, 8, &mut out);
        assert_eq!(out.len(), 8 * 8 * 3);
        assert_eq!((fit.scale, fit.pad_x, fit.pad_y), (1.0, 0.0, 2.0));
        // The first two rows are padding, the middle four the frame, the last two padding again.
        let row = |r: usize| &out[r * 8 * 3..r * 8 * 3 + 3];
        assert_eq!(row(0), &[PAD, PAD, PAD]);
        assert_eq!(row(1), &[PAD, PAD, PAD]);
        assert_eq!(row(2), &[255, 255, 255]);
        assert_eq!(row(5), &[255, 255, 255]);
        assert_eq!(row(6), &[PAD, PAD, PAD]);
        assert_eq!(row(7), &[PAD, PAD, PAD]);
    }

    /// The layout of the head, which is the thing most likely to be read wrong.
    ///
    /// `[1, 5, N]` is *planar*: every cx, then every cy. Read as five-per-box it produces boxes
    /// that are almost plausible — the worst kind of wrong, because it looks like a bad model
    /// rather than a bad reader.
    #[test]
    fn the_head_is_planar_not_interleaved() {
        // Two candidates. Planar: cx = [10, 100], cy = [20, 200], w = [4, 40], h = [6, 60],
        // score = [0.9, 0.1].
        let raw = vec![10.0, 100.0, 20.0, 200.0, 4.0, 40.0, 6.0, 60.0, 0.9, 0.1];
        let fit = Letterbox {
            scale: 1.0,
            pad_x: 0.0,
            pad_y: 0.0,
        };
        let found = decode(&raw, fit, 0.5, 0.5);
        assert_eq!(found.len(), 1, "only the first candidate is over 0.5");
        let detection = found[0];
        assert_eq!(detection.score, 0.9);
        assert_eq!(detection.box_, [8.0, 17.0, 12.0, 23.0]);
    }

    /// One duck, twenty boxes, one survivor — and the padding undone on the way out.
    #[test]
    fn overlapping_candidates_collapse_and_map_back_to_the_frame() {
        // Five near-identical candidates around (160, 160) in a letterbox that scaled by 0.25 and
        // padded 40px on the left — i.e. a 1280-wide frame squeezed into 320.
        let count = 5;
        let mut raw = vec![0.0f32; count * STRIDE];
        for index in 0..count {
            raw[index] = 160.0 + index as f32; // cx
            raw[count + index] = 160.0; // cy
            raw[2 * count + index] = 40.0; // w
            raw[3 * count + index] = 80.0; // h
            raw[4 * count + index] = 0.9 - index as f32 * 0.01; // score
        }
        let fit = Letterbox {
            scale: 0.25,
            pad_x: 40.0,
            pad_y: 0.0,
        };
        let found = decode(&raw, fit, 0.5, 0.5);
        assert_eq!(found.len(), 1, "the cluster is one duck");
        let detection = found[0];
        // (160 - 40 - 20) / 0.25 = 400, and the width is 40 / 0.25 = 160.
        assert_eq!(detection.box_[0], 400.0);
        assert_eq!(detection.width(), 160.0);
        assert_eq!(detection.height(), 320.0);
        // A duck at x 400..560 of a 1280-wide frame is left of centre.
        assert!(detection.bearing(1280.0) < 0.0);
    }

    /// Two ducks far apart are two ducks, however much the head repeats itself about each.
    #[test]
    fn two_ducks_stay_two() {
        let count = 4;
        let mut raw = vec![0.0f32; count * STRIDE];
        let centres = [50.0, 52.0, 250.0, 248.0];
        for (index, centre) in centres.iter().enumerate() {
            raw[index] = *centre;
            raw[count + index] = 160.0;
            raw[2 * count + index] = 30.0;
            raw[3 * count + index] = 60.0;
            raw[4 * count + index] = 0.8;
        }
        let fit = Letterbox {
            scale: 1.0,
            pad_x: 0.0,
            pad_y: 0.0,
        };
        assert_eq!(decode(&raw, fit, 0.5, 0.5).len(), 2);
    }
}
