//! A picture as the display takes it: 240 × 240 pixels in its six inks, two to a byte.
//!
//! Every step is the one `nfc-test` takes (microduck-test, `nfc/nfc-test.c` at `f0c591f`), to the
//! byte — the tests hold hashes of what that program makes of the same pixels:
//!
//! 1. The image fitted to the panel ([`fit`]): the centre square, scaled to 240 × 240. A picture
//!    of that size already is left as it is, which is all `nfc-test` accepts.
//! 2. An optional box blur ([`blur`]), which `nfc-test` offers for camera pictures, whose pixel
//!    grid otherwise dithers as content.
//! 3. Each pixel to the nearest of the six inks, its error spread to the neighbours not yet done
//!    ([`Dither::Atkinson`], the vendor's iOS app's choice: 1/8 to six of them, so a quarter of
//!    it is dropped, which keeps flat areas clean).
//! 4. The panel's order ([`pack`]): rows bottom to top, columns left to right, two pixels a byte,
//!    high nibble first, each pixel its ink's code.
//!
//! The 28 800 bytes go to the display as four images of 7 200 ([`PLANE`]), which is
//! [`super::Writer`]'s.

use image::RgbImage;
use image::imageops::{self, FilterType};

/// The panel's side, in pixels.
pub const SIDE: u32 = 240;
/// A packed picture: two pixels a byte.
pub const BYTES: usize = (SIDE * SIDE / 2) as usize;
/// One of the four images the picture is sent as.
pub const PLANE: usize = BYTES / 4;

/// The six inks: what each looks like, and its code on the wire — `nfc-test`'s table, from the
/// display's own TLV `E0`. Code 4 is not one of them.
pub const INKS: [([u8; 3], u8); 6] = [
    ([0, 0, 0], 0x00),
    ([255, 255, 255], 0x01),
    ([255, 255, 0], 0x02),
    ([255, 0, 0], 0x03),
    ([0, 0, 255], 0x05),
    ([0, 255, 0], 0x06),
];

/// How the colours between the inks are drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Dither {
    /// Error diffusion to six neighbours, 1/8 each.
    #[default]
    Atkinson,
    /// Each pixel its nearest ink, and nothing more: flat areas, for drawings.
    None,
}

/// The centre square of `image`, scaled to the panel. Left alone when it already is the panel's
/// size.
pub fn fit(image: &RgbImage) -> RgbImage {
    let (w, h) = image.dimensions();
    if (w, h) == (SIDE, SIDE) {
        return image.clone();
    }
    let side = w.min(h);
    let square = imageops::crop_imm(image, (w - side) / 2, (h - side) / 2, side, side).to_image();
    imageops::resize(&square, SIDE, SIDE, FilterType::Triangle)
}

/// A separable box blur of `radius`, edges repeated: `nfc-test`'s `NFC_BLUR`. Nothing at 0.
pub fn blur(image: &mut RgbImage, radius: u32) {
    if radius == 0 {
        return;
    }
    let (w, h) = image.dimensions();
    let r = radius as i64;
    let span = 2 * radius + 1;
    let along = |from: &RgbImage, horizontal: bool| {
        RgbImage::from_fn(w, h, |x, y| {
            let mut sum = [0u32; 3];
            for k in -r..=r {
                let (nx, ny) = if horizontal {
                    ((x as i64 + k).clamp(0, w as i64 - 1) as u32, y)
                } else {
                    (x, (y as i64 + k).clamp(0, h as i64 - 1) as u32)
                };
                for (s, v) in sum.iter_mut().zip(from.get_pixel(nx, ny).0) {
                    *s += u32::from(v);
                }
            }
            image::Rgb(sum.map(|s| (s / span) as u8))
        })
    };
    let across = along(image, true);
    *image = along(&across, false);
}

/// The index in [`INKS`] of the ink nearest `rgb`, by squared distance; the first on a tie.
fn nearest(rgb: [u8; 3]) -> usize {
    let distance = |ink: &[u8; 3]| -> i32 {
        rgb.iter()
            .zip(ink)
            .map(|(&a, &b)| (i32::from(a) - i32::from(b)).pow(2))
            .sum()
    };
    let mut best = 0;
    for (i, (ink, _)) in INKS.iter().enumerate().skip(1) {
        if distance(ink) < distance(&INKS[best].0) {
            best = i;
        }
    }
    best
}

/// Every pixel of a [`SIDE`]-square picture as an index in [`INKS`], row by row from the top.
pub fn quantize(image: &RgbImage, dither: Dither) -> Vec<u8> {
    assert_eq!(image.dimensions(), (SIDE, SIDE), "fit() it first");
    let side = SIDE as usize;
    let mut rgb = image.as_raw().clone();
    let mut inks = vec![0u8; side * side];
    // Right, two right; below left, below, below right; two below.
    const SPREAD: [(isize, usize); 6] = [(1, 0), (2, 0), (-1, 1), (0, 1), (1, 1), (0, 2)];
    for y in 0..side {
        for x in 0..side {
            let at = (y * side + x) * 3;
            let ink = nearest([rgb[at], rgb[at + 1], rgb[at + 2]]);
            inks[y * side + x] = ink as u8;
            if dither == Dither::None {
                continue;
            }
            for c in 0..3 {
                let error = i32::from(rgb[at + c]) - i32::from(INKS[ink].0[c]);
                // Rounded half away from zero, then truncated toward it, as the C does.
                let share = (error + if error >= 0 { 4 } else { -4 }) / 8;
                for (dx, dy) in SPREAD {
                    let nx = x as isize + dx;
                    let ny = y + dy;
                    if nx < 0 || nx as usize >= side || ny >= side {
                        continue;
                    }
                    let n = (ny * side + nx as usize) * 3 + c;
                    rgb[n] = (i32::from(rgb[n]) + share).clamp(0, 255) as u8;
                }
            }
        }
    }
    inks
}

/// The picture as the display stores it: rows bottom to top, two pixels a byte, high nibble
/// first, each its ink's code.
pub fn pack(inks: &[u8]) -> Vec<u8> {
    let side = SIDE as usize;
    assert_eq!(inks.len(), side * side);
    let code = |i: u8| INKS[usize::from(i)].1 & 0x0f;
    (0..side)
        .rev()
        .flat_map(|y| inks[y * side..(y + 1) * side].chunks(2))
        .map(|pair| code(pair[0]) << 4 | code(pair[1]))
        .collect()
}

/// `image` as the display takes it: fitted, blurred by `blur`, drawn in its inks, packed.
pub fn render(image: &RgbImage, dither: Dither, blur_radius: u32) -> Vec<u8> {
    let mut fitted = fit(image);
    blur(&mut fitted, blur_radius);
    pack(&quantize(&fitted, dither))
}

/// The photo the sequence took, a JPEG, as the display takes it in the beak: dithered, and turned
/// half a turn, since the beak holds the display upside down to how [`render`] draws it (seen on
/// the robot, 2026-11-17).
pub fn from_photo(jpeg: &[u8]) -> Result<Vec<u8>, String> {
    let image = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg)
        .map_err(|e| format!("the photo does not decode: {e}"))?;
    let turned = imageops::rotate180(&image.to_rgb8());
    Ok(render(&turned, Dither::Atkinson, 0))
}

/// `nfc-test`'s picture when it is given none: the six inks as vertical bars in a 4-pixel black
/// frame, which shows every colour and every edge of the panel.
pub fn test_pattern() -> RgbImage {
    let bar = SIDE / 6;
    RgbImage::from_fn(SIDE, SIDE, |x, y| {
        let frame = x < 4 || y < 4 || x >= SIDE - 4 || y >= SIDE - 4;
        let ink = if frame { 0 } else { (x / bar) as usize % 6 };
        image::Rgb(INKS[ink].0)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// FNV-1a, 64 bits: the hash the reference harness prints.
    fn fnv(bytes: &[u8]) -> u64 {
        bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
            (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3)
        })
    }

    /// A picture with every kind of colour in it, which the reference harness draws the same way:
    /// red across, green down, blue in diagonal bands.
    fn sweep() -> RgbImage {
        RgbImage::from_fn(SIDE, SIDE, |x, y| {
            image::Rgb([
                (x * 255 / 239) as u8,
                (y * 255 / 239) as u8,
                (((x + y) * 7) & 255) as u8,
            ])
        })
    }

    // What `nfc-test.c`'s own functions (palette, nearest_color, box_blur, quantize_image,
    // build_4bpp, gen_test_pattern — copied unchanged into a harness, compiled on a laptop) make
    // of the same pixels.
    #[test]
    fn atkinson_is_byte_for_byte_nfc_tests() {
        assert_eq!(
            fnv(&render(&sweep(), Dither::Atkinson, 0)),
            0x1a64_4c2f_7f62_af81
        );
    }

    #[test]
    fn nearest_ink_alone_is_byte_for_byte_nfc_tests() {
        assert_eq!(
            fnv(&render(&sweep(), Dither::None, 0)),
            0x1f21_4262_7fd3_4172
        );
    }

    #[test]
    fn the_blur_is_byte_for_byte_nfc_tests() {
        assert_eq!(
            fnv(&render(&sweep(), Dither::Atkinson, 2)),
            0xeb5c_7947_c7e6_41a4
        );
    }

    #[test]
    fn the_test_pattern_is_byte_for_byte_nfc_tests() {
        assert_eq!(
            fnv(&render(&test_pattern(), Dither::Atkinson, 0)),
            0x5e71_2632_6b23_20c5
        );
    }

    /// The photo is turned for the beak: its top-left corner ends bottom right.
    #[test]
    fn the_photo_is_turned_half_a_turn() {
        let image = RgbImage::from_fn(SIDE, SIDE, |x, y| {
            image::Rgb(if x < 8 && y < 8 {
                [255, 0, 0]
            } else {
                [255, 255, 255]
            })
        });
        let mut jpeg = Vec::new();
        image::codecs::jpeg::JpegEncoder::new_with_quality(&mut jpeg, 100)
            .encode_image(&image)
            .unwrap();
        let turned = from_photo(&jpeg).unwrap();
        let straight = render(&image, Dither::Atkinson, 0);
        // Packed bottom row first, left to right: byte 119 ends the bottom row.
        assert_eq!(turned[119] & 0x0f, 0x03, "red, bottom right");
        assert_eq!(straight[119] & 0x0f, 0x01, "white there unturned");
        assert_eq!(turned[BYTES - 120] >> 4, 0x01, "white, top left");
    }

    #[test]
    fn rows_go_bottom_to_top_two_pixels_a_byte() {
        // White everywhere but a red top-left pixel and a blue bottom-left one.
        let mut image = RgbImage::from_pixel(SIDE, SIDE, image::Rgb([255, 255, 255]));
        image.put_pixel(0, 0, image::Rgb([255, 0, 0]));
        image.put_pixel(0, SIDE - 1, image::Rgb([0, 0, 255]));
        let packed = render(&image, Dither::None, 0);
        assert_eq!(packed.len(), BYTES);
        assert_eq!(packed[0], 0x51, "the bottom row first: blue, then white");
        assert_eq!(
            packed[BYTES - 120],
            0x31,
            "the top row last: red, then white"
        );
        assert_eq!(packed[1], 0x11);
    }

    #[test]
    fn a_camera_frame_is_cropped_to_its_centre_and_scaled() {
        // 1280 × 720 with the left and right 280 columns black: the centre square is all white.
        let image = RgbImage::from_fn(1280, 720, |x, _| {
            let edge = !(280..1000).contains(&x);
            image::Rgb(if edge { [0, 0, 0] } else { [255, 255, 255] })
        });
        let fitted = fit(&image);
        assert_eq!(fitted.dimensions(), (SIDE, SIDE));
        assert!(fitted.pixels().all(|p| p.0 == [255, 255, 255]));
    }
}
