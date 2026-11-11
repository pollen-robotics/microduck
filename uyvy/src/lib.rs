//! What a camera frame is, before anything decides what it means.
//!
//! The head camera hands out `UYVY` at 1280×720, mounted however the board mounts it — a quarter
//! turn off on the Zero 3W, upright on the beta boards (`robotd_params::board`'s
//! `camera_mount_degrees` owns that) — so every consumer of a frame does the same two things
//! first: turn it the right way up, whatever that takes, and sample it
//! down to the size it actually wants. Three of them do — `mediad` encoding a JPEG or a PNG,
//! `duck-detect` feeding a model, and `robotctl monitor` drawing one in a terminal — and this is
//! the one copy of that arithmetic, so a picture cannot come out upright in one and sideways in
//! the next.
//!
//! **The turn happens while sampling, not before it.** Both samplers walk their *output* and pull
//! the source pixel each one needs, so the rotation is a change of index inside a loop that was
//! already running rather than a pass of its own. [`Turn`] says why that matters: the pipeline
//! tried rotating in GStreamer once and it cost the board 22 fps.

/// The grey ultralytics pads a letterbox with, and therefore what calibration and training saw.
pub const PAD: u8 = 114;

/// How a frame was fitted into the model's square, so detections can be mapped back out of it.
#[derive(Debug, Clone, Copy)]
pub struct Letterbox {
    pub scale: f32,
    pub pad_x: f32,
    pub pad_y: f32,
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
    /// A quarter turn clockwise: what the Zero 3W's camera mount needs.
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
