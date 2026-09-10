//! Finding other Microducks in this Microduck's camera, on the NPU.
//!
//! The model is trained in [`duck_detector`](https://github.com/pollen-robotics/duck_detector) and
//! arrives here as an INT8 `.rknn`: one class, 320×320 in, 2100 candidate boxes out. This crate is
//! the three things between a camera frame and a bounding box — the letterbox, the runtime, and the
//! decode — plus `duck-bench`, which measures them on a real board.
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

/// What the head emits per candidate: cx, cy, w, h, score.
const STRIDE: usize = 5;

/// The grey ultralytics pads a letterbox with, and therefore what calibration and training saw.
pub const PAD: u8 = 114;

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

/// How a frame was fitted into the model's square, so detections can be mapped back out of it.
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    pub scale: f32,
    pub pad_x: f32,
    pub pad_y: f32,
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

/// How the camera is mounted, and therefore how far the sampler has to turn the picture.
///
/// **Turned here rather than in the pipeline, and that is a performance decision, not a taste.**
/// A `videoflip` before the tee cost 145% of a core on the robot: `mpph264enc` hands UYVY→NV12 to
/// the SoC's 2D engine for free, and the flip's buffers are ones the RGA refuses
/// (`RGA_BLIT fail: Bad address`), so MPP fell back to converting every frame in software — 97 °C,
/// the CPU throttled to 408 MHz, and 8 fps out of a 30 fps camera. This sampler is already
/// resampling to 320×320, so doing the turn in the same pass costs nothing at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Turn {
    #[default]
    None,
    /// A quarter turn clockwise: what this robot's camera mount needs.
    Right,
    Half,
    Left,
}

impl Turn {
    /// From degrees clockwise, which is how the flag is written.
    pub fn from_degrees(degrees: u32) -> Option<Self> {
        match degrees % 360 {
            0 => Some(Self::None),
            90 => Some(Self::Right),
            180 => Some(Self::Half),
            270 => Some(Self::Left),
            _ => None,
        }
    }

    /// The frame's size once turned — a quarter turn swaps the axes.
    pub fn upright(self, width: usize, height: usize) -> (usize, usize) {
        match self {
            Self::Right | Self::Left => (height, width),
            Self::None | Self::Half => (width, height),
        }
    }

    /// Where a pixel of the *upright* picture is in the frame the camera took.
    ///
    /// The inverse mapping, because the sampler walks the output and pulls from the input. Written
    /// as one function so the four cases are in one place rather than spread through a loop.
    fn source(self, ux: usize, uy: usize, width: usize, height: usize) -> (usize, usize) {
        match self {
            Self::None => (ux, uy),
            // A quarter turn clockwise sends source (x, y) to upright (h-1-y, x), so the inverse
            // takes upright (ux, uy) from source (uy, h-1-ux).
            Self::Right => (uy, height.saturating_sub(1).saturating_sub(ux)),
            Self::Half => (
                width.saturating_sub(1).saturating_sub(ux),
                height.saturating_sub(1).saturating_sub(uy),
            ),
            Self::Left => (width.saturating_sub(1).saturating_sub(uy), ux),
        }
    }
}

/// `UYVY` straight into a letterboxed RGB square — one pass, and only the pixels that survive.
///
/// **This replaced converting the frame and then shrinking it, which cost 345 ms of the 407 ms a
/// look took on the robot.** The tee carries 720×1280 4:2:2 because that is what the encoder wants;
/// the model wants 320×320 RGB. Converting all 921 600 pixels to throw 89% of them away is nine
/// times the arithmetic for the same answer, so this samples the source at the target grid instead —
/// 102 400 pixels, integer maths, no intermediate buffer.
///
/// Nearest-neighbour, and chroma from the pair without interpolation: the input is a blurred
/// photograph of a room being downscaled by four, and nothing in a bounding box survives at that
/// precision.
pub fn letterbox_from_uyvy(
    uyvy: &[u8],
    width: usize,
    height: usize,
    size: usize,
    turn: Turn,
    out: &mut Vec<u8>,
) -> Letterbox {
    // Everything below is in *upright* coordinates — the picture the right way up, which is what
    // the model was trained on and what a detection has to be reported in. The turn is undone only
    // at the moment a source pixel is fetched.
    let (upright_w, upright_h) = turn.upright(width, height);
    let scale = (size as f32 / upright_w as f32).min(size as f32 / upright_h as f32);
    let fitted_w = ((upright_w as f32 * scale).round() as usize)
        .max(1)
        .min(size);
    let fitted_h = ((upright_h as f32 * scale).round() as usize)
        .max(1)
        .min(size);
    let pad_x = (size - fitted_w) / 2;
    let pad_y = (size - fitted_h) / 2;
    let stride = width * 2;

    out.clear();
    out.resize(size * size * 3, PAD);
    for y in 0..fitted_h {
        let uy = (y * upright_h) / fitted_h;
        for x in 0..fitted_w {
            let ux = (x * upright_w) / fitted_w;
            let (source_x, source_y) = turn.source(ux, uy, width, height);
            let row = source_y * stride;
            if row + stride > uyvy.len() {
                // A frame that arrives mid-teardown is short. What is missing stays padding rather
                // than taking the daemon down over a picture.
                continue;
            }
            let pair = row + (source_x / 2) * 4;
            // U Y0 V Y1: the luma is the odd byte of the half this pixel falls in.
            let luma = uyvy[pair + 1 + 2 * (source_x & 1)] as i32 - 16;
            let u = uyvy[pair] as i32 - 128;
            let v = uyvy[pair + 2] as i32 - 128;

            // BT.601 limited range in fixed point — the ISP's convention, and the one every JPEG
            // the dataset was labelled from went through. Integer because this is the inner loop.
            let r = (298 * luma + 409 * v + 128) >> 8;
            let g = (298 * luma - 100 * u - 208 * v + 128) >> 8;
            let b = (298 * luma + 516 * u + 128) >> 8;

            let target = ((y + pad_y) * size + (x + pad_x)) * 3;
            out[target] = r.clamp(0, 255) as u8;
            out[target + 1] = g.clamp(0, 255) as u8;
            out[target + 2] = b.clamp(0, 255) as u8;
        }
    }

    Letterbox {
        scale,
        pad_x: pad_x as f32,
        pad_y: pad_y as f32,
    }
}

/// The frame as an upright RGB picture, scaled to fit a box, with no padding.
///
/// [`letterbox_from_uyvy`] above is for the model: a square, padded, at whatever size the network
/// wants. This is for a *person or a program looking at the picture* — a JPEG on its way to a
/// Space that runs a model of its own — so it keeps the aspect ratio and pads nothing.
///
/// **The turn is applied here rather than reported.** `mediad`'s pipeline deliberately does not
/// rotate: a `videoflip` cost the encoder its zero-copy path and the board 22 fps, so a WebRTC
/// consumer is told the mount angle and turns the picture itself (`media.video`). That reasoning
/// does not carry over to this path, because the conversion is a per-pixel loop either way and the
/// turn is a change of which source pixel is fetched — free, inside a loop that is already
/// running. And what receives these frames is a model, which wants them the way up it was trained
/// on rather than a rotation flag to honour.
///
/// Returns the size written, which is not `(long, short)` in any predictable order: a quarter turn
/// swaps the axes, so the caller is told rather than left to work it out.
pub fn rgb_from_uyvy(
    uyvy: &[u8],
    width: usize,
    height: usize,
    longest: usize,
    turn: Turn,
    out: &mut Vec<u8>,
) -> (usize, usize) {
    let (upright_w, upright_h) = turn.upright(width, height);
    // Downscale only. Asking for a box bigger than the sensor would interpolate detail that was
    // never captured and cost the bandwidth of pretending.
    let scale = (longest as f32 / upright_w.max(upright_h) as f32).min(1.0);
    let out_w = ((upright_w as f32 * scale).round() as usize).max(1);
    let out_h = ((upright_h as f32 * scale).round() as usize).max(1);
    let stride = width * 2;

    out.clear();
    out.resize(out_w * out_h * 3, 0);
    for y in 0..out_h {
        let uy = (y * upright_h) / out_h;
        for x in 0..out_w {
            let ux = (x * upright_w) / out_w;
            let (source_x, source_y) = turn.source(ux, uy, width, height);
            let row = source_y * stride;
            if row + stride > uyvy.len() {
                // A frame that arrives mid-teardown is short. What is missing stays black rather
                // than taking the daemon down over a picture — the same call `letterbox_from_uyvy`
                // makes.
                continue;
            }
            let pair = row + (source_x / 2) * 4;
            // U Y0 V Y1: the luma is the odd byte of the half this pixel falls in.
            let luma = uyvy[pair + 1 + 2 * (source_x & 1)] as i32 - 16;
            let u = uyvy[pair] as i32 - 128;
            let v = uyvy[pair + 2] as i32 - 128;

            // BT.601 limited range in fixed point, as above: the ISP's convention.
            let r = (298 * luma + 409 * v + 128) >> 8;
            let g = (298 * luma - 100 * u - 208 * v + 128) >> 8;
            let b = (298 * luma + 516 * u + 128) >> 8;

            let target = (y * out_w + x) * 3;
            out[target] = r.clamp(0, 255) as u8;
            out[target + 1] = g.clamp(0, 255) as u8;
            out[target + 2] = b.clamp(0, 255) as u8;
        }
    }
    (out_w, out_h)
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
    /// A quarter turn swaps the axes and moves a known corner, which is the whole of what a
    /// rotation can get wrong: an upside-down picture and an unrotated one have the same shape.
    #[test]
    fn the_rgb_scaler_turns_the_picture_and_keeps_its_shape() {
        // 8x4 UYVY, all grey except the top-left pixel, which is bright.
        let (width, height) = (8usize, 4usize);
        let mut uyvy = [128u8, 16, 128, 16].repeat(width * height / 2);
        uyvy[1] = 235; // Y0 of the first pair: the top-left pixel, white.

        let mut out = Vec::new();
        let (w, h) = rgb_from_uyvy(&uyvy, width, height, 8, Turn::None, &mut out);
        assert_eq!((w, h), (8, 4), "unturned, the shape is the frame's");
        assert!(out[0] > 200, "and the bright pixel is top-left: {}", out[0]);

        let (w, h) = rgb_from_uyvy(&uyvy, width, height, 8, Turn::Right, &mut out);
        assert_eq!((w, h), (4, 8), "a quarter turn swaps the axes");
        // Turned clockwise, the frame's top-left corner is the picture's top-right — row zero,
        // last column.
        let top_right = (w - 1) * 3;
        assert!(
            out[top_right] > 200,
            "the bright pixel moved to the top-right: {:?}",
            &out[top_right..top_right + 3]
        );
        assert!(out[0] < 200, "and is no longer top-left: {}", out[0]);
    }

    /// Never upscales: a box larger than the sensor would interpolate detail nobody captured.
    #[test]
    fn the_rgb_scaler_only_ever_shrinks() {
        let uyvy = [128u8, 16, 128, 16].repeat(8 * 4 / 2);
        let mut out = Vec::new();
        assert_eq!(rgb_from_uyvy(&uyvy, 8, 4, 64, Turn::None, &mut out), (8, 4));
        assert_eq!(rgb_from_uyvy(&uyvy, 8, 4, 4, Turn::None, &mut out), (4, 2));
    }

    use super::*;

    /// A tall frame fits inside the square with grey above and below, and nothing stretches.
    ///
    /// The camera is portrait after the daemon's quarter turn — 720×1280 — so this is the only case
    /// that actually happens, and getting the padding wrong moves every box by a constant nobody
    /// notices until the robot reaches for a duck that is not there.
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

    /// The one-pass conversion agrees with the two-step one it replaced, geometry included.
    #[test]
    fn uyvy_letterboxes_in_one_pass() {
        // A 4×8 frame of mid grey, into an 8×8 square: same geometry as the RGB case above.
        let uyvy: Vec<u8> = std::iter::repeat_n([128u8, 126, 128, 126], 4 * 8 / 2)
            .flatten()
            .collect();
        let mut out = Vec::new();
        let fit = letterbox_from_uyvy(&uyvy, 4, 8, 8, Turn::None, &mut out);
        assert_eq!((fit.scale, fit.pad_x, fit.pad_y), (1.0, 2.0, 0.0));
        assert_eq!(out.len(), 8 * 8 * 3);
        // Padding at the edges, picture in the middle, and grey that stayed grey.
        assert_eq!(&out[0..3], &[PAD, PAD, PAD]);
        let middle = &out[2 * 3..2 * 3 + 3];
        assert!(
            middle.iter().all(|v| (125..=133).contains(v)),
            "grey stayed grey: {middle:?}"
        );

        // And the channels are not swapped: V high is red.
        let red: Vec<u8> = std::iter::repeat_n([64u8, 126, 200, 126], 2)
            .flatten()
            .collect();
        letterbox_from_uyvy(&red, 4, 1, 4, Turn::None, &mut out);
        // Row 1 (the padded square is 4 wide), first column: the picture's own first pixel.
        let first = 4 * 3;
        let pixel = &out[first..first + 3];
        assert!(pixel[0] > pixel[2], "V high is red, not blue: {pixel:?}");
    }

    /// A short frame leaves the rest as padding rather than panicking.
    #[test]
    fn a_short_uyvy_frame_does_not_panic() {
        let uyvy = vec![128u8; 2 * 2 * 2];
        let mut out = Vec::new();
        letterbox_from_uyvy(&uyvy, 2, 8, 8, Turn::None, &mut out);
        assert_eq!(out.len(), 8 * 8 * 3);
    }

    /// A quarter turn swaps the axes and lands the corners where a rotation should.
    ///
    /// This is the arithmetic that replaced a `videoflip` costing 145% of a core, so it had better
    /// be right: a mirrored or transposed picture would still detect *something*, on a model
    /// trained on neither.
    #[test]
    fn a_quarter_turn_happens_while_sampling() {
        assert_eq!(Turn::from_degrees(90), Some(Turn::Right));
        assert_eq!(Turn::from_degrees(270), Some(Turn::Left));
        assert_eq!(Turn::from_degrees(45), None);
        // 1280x720 landscape becomes 720x1280 upright.
        assert_eq!(Turn::Right.upright(1280, 720), (720, 1280));
        assert_eq!(Turn::Half.upright(1280, 720), (1280, 720));

        // Clockwise: the source's top-left corner ends up at the upright picture's top-right.
        // Checked through the inverse, which is what the sampler uses: the upright top-right pixel
        // is fetched from source (0, 0).
        let (w, h) = (4usize, 2usize);
        let (upright_w, _upright_h) = Turn::Right.upright(w, h);
        assert_eq!(Turn::Right.source(upright_w - 1, 0, w, h), (0, 0));
        // And the upright top-left comes from the source's bottom-left.
        assert_eq!(Turn::Right.source(0, 0, w, h), (0, h - 1));
        // Anticlockwise is the other way about.
        assert_eq!(Turn::Left.source(0, 0, w, h), (w - 1, 0));
    }

    /// The turn is visible in the pixels, not just in the arithmetic.
    #[test]
    fn a_turned_frame_puts_the_bright_row_on_the_right_side() {
        // A 4x2 UYVY frame: top row black, bottom row white. Turned clockwise, the bottom row
        // becomes the *left* column — so the left of the output must be bright.
        let dark = [128u8, 16, 128, 16];
        let bright = [128u8, 235, 128, 235];
        let mut uyvy = Vec::new();
        uyvy.extend(dark.iter().chain(dark.iter())); // row 0, 4 pixels
        uyvy.extend(bright.iter().chain(bright.iter())); // row 1
        let mut out = Vec::new();
        // Into a 4x4 square: upright is 2 wide, 4 tall, so it fits exactly with side padding.
        let fit = letterbox_from_uyvy(&uyvy, 4, 2, 4, Turn::Right, &mut out);
        assert_eq!(
            (fit.pad_x, fit.pad_y),
            (1.0, 0.0),
            "2x4 upright inside a 4x4 square"
        );
        let pixel = |x: usize, y: usize| out[(y * 4 + x) * 3];
        assert!(
            pixel(1, 0) > 200,
            "the source's bottom row is now the left column"
        );
        assert!(pixel(2, 0) < 60, "and its top row is the right column");
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
