//! `duckctl setup-code add`: the robot's setup-code QR, read with this computer's camera.
//!
//! The label's QR carries `<label serial>:md1:<robot id>:<64 hex>` — the line the phone app scans
//! and `SetupCode::parse` already accepts pasted. Typing it is what this replaces.
//!
//! All in the terminal: `nokhwa` holds the camera, every frame is drawn as a greyscale preview in
//! half blocks so there is something to aim with, and `rqrr` looks for a QR in the same frame. The
//! first QR that parses as a setup code ends the scan; any other QR says so and keeps the camera
//! open, since a label is the only thing anyone is pointing at here.
//!
//! The preview is mirrored, like every video call, because that is how a person expects a camera
//! facing them to move. Decoding is not: it reads the frame as captured, and tries it mirrored
//! only when nothing decoded, for the cameras that flip the picture themselves.

use std::time::Duration;

use nokhwa::Camera;
use nokhwa::pixel_format::LumaFormat;
use nokhwa::utils::{ApiBackend, CameraInfo, RequestedFormat, RequestedFormatType};
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize as _};
use ratatui::text::Line;
use ratatui::widgets::{Paragraph, Widget};

use crate::setup_code::SetupCode;

/// What one frame held.
#[derive(Debug, PartialEq, Eq)]
pub enum Seen {
    /// No QR that decoded.
    Nothing,
    /// A QR, but not a setup code.
    NotACode,
    /// A QR carrying `md1:`, that `SetupCode::parse` refuses, and why — never the key itself.
    /// Worth its own message: a code one character short is a misprinted label, not the wrong QR.
    BadCode(String),
    Code(SetupCode),
}

/// Every QR in a greyscale frame, `width` × `height`, one byte a pixel, row by row.
pub fn look(width: usize, height: usize, luma: &[u8]) -> Seen {
    let upright = decode(width, height, |x, y| luma[y * width + x]);
    let texts = if upright.is_empty() {
        decode(width, height, |x, y| luma[y * width + (width - 1 - x)])
    } else {
        upright
    };
    if texts.is_empty() {
        return Seen::Nothing;
    }
    if let Some(code) = texts.iter().find_map(|text| SetupCode::parse(text).ok()) {
        return Seen::Code(code);
    }
    match texts.iter().find(|text| text.contains("md1:")) {
        Some(text) => Seen::BadCode(SetupCode::parse(text).unwrap_err()),
        None => Seen::NotACode,
    }
}

fn decode(width: usize, height: usize, pixel: impl FnMut(usize, usize) -> u8) -> Vec<String> {
    let mut image = rqrr::PreparedImage::prepare_from_greyscale(width, height, pixel);
    image
        .detect_grids()
        .into_iter()
        .filter_map(|grid| grid.decode().ok().map(|(_, text)| text))
        .collect()
}

/// Open the camera, preview it in the terminal, and return the first setup code it reads.
///
/// Blocking: the camera hands frames out synchronously. Esc, `q` or Ctrl-C cancels; `c` moves to
/// the next camera when there is more than one (a laptop with an external webcam, or a phone
/// offered as one).
pub fn scan() -> Result<SetupCode, String> {
    use std::io::IsTerminal as _;
    if !std::io::stdout().is_terminal() {
        return Err("the scan draws the camera in the terminal, and this output is not one".into());
    }
    allow_camera()?;
    let cameras = nokhwa::query(ApiBackend::Auto).map_err(|e| format!("no camera list: {e}"))?;
    if cameras.is_empty() {
        return Err("no camera found on this computer — paste the code instead".into());
    }

    let mut which = 0;
    let mut camera = open(&cameras[which])?;
    let mut screen = Screen(ratatui::init());
    let mut seen = Seen::Nothing;
    loop {
        while event::poll(Duration::ZERO).map_err(|e| e.to_string())? {
            let Event::Key(key) = event::read().map_err(|e| e.to_string())? else {
                continue;
            };
            if key.kind != KeyEventKind::Press {
                continue;
            }
            match key.code {
                KeyCode::Esc | KeyCode::Char('q') => {
                    return Err("scan cancelled".into());
                }
                KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    return Err("scan cancelled".into());
                }
                KeyCode::Char('c') if cameras.len() > 1 => {
                    drop(camera);
                    which = (which + 1) % cameras.len();
                    camera = open(&cameras[which])?;
                }
                _ => {}
            }
        }

        let frame = camera
            .frame()
            .and_then(|frame| frame.decode_image::<LumaFormat>())
            .map_err(|e| format!("the camera stopped giving frames: {e}"))?;
        let (width, height) = (frame.width() as usize, frame.height() as usize);
        let luma = frame.into_raw();
        match look(width, height, &luma) {
            Seen::Code(code) => return Ok(code),
            // A frame without a QR does not clear "that is not a setup code" straight away: the
            // label wobbles in and out of focus, and the message would flicker.
            Seen::Nothing => {}
            other => seen = other,
        }

        let name = cameras[which].human_name();
        screen
            .0
            .draw(|f| {
                let [preview, status] =
                    Layout::vertical([Constraint::Min(4), Constraint::Length(3)]).areas(f.area());
                f.render_widget(
                    Preview {
                        width,
                        height,
                        luma: &luma,
                    },
                    preview,
                );
                let hint = match &seen {
                    Seen::NotACode => {
                        "That QR is not a duck setup code — show the one on the duck's label."
                            .yellow()
                    }
                    Seen::BadCode(why) => {
                        format!("That QR is a setup code that does not read: {why}.").red()
                    }
                    _ => "Hold the QR on your duck's label up to the camera.".bold(),
                };
                let mut keys = "Esc to cancel".to_owned();
                if cameras.len() > 1 {
                    keys.push_str(&format!(
                        " · c for the next camera ({name}, {}/{})",
                        which + 1,
                        cameras.len()
                    ));
                } else {
                    keys.push_str(&format!(" · {name}"));
                }
                f.render_widget(
                    Paragraph::new(vec![Line::default(), Line::from(hint), keys.dim().into()])
                        .centered(),
                    status,
                );
            })
            .map_err(|e| e.to_string())?;
    }
}

/// The terminal, given back the way it was found however `scan` returns.
struct Screen(ratatui::DefaultTerminal);

impl Drop for Screen {
    fn drop(&mut self) {
        ratatui::restore();
    }
}

fn open(info: &CameraInfo) -> Result<Camera, String> {
    let format = RequestedFormat::new::<LumaFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
    let mut camera = Camera::new(info.index().clone(), format)
        .map_err(|e| format!("could not open {}: {e}", info.human_name()))?;
    camera
        .open_stream()
        .map_err(|e| format!("could not start {}: {e}", info.human_name()))?;
    Ok(camera)
}

/// macOS asks the person once, for the terminal app this runs in, and remembers the answer.
#[cfg(target_os = "macos")]
fn allow_camera() -> Result<(), String> {
    let (tx, rx) = std::sync::mpsc::channel();
    nokhwa::nokhwa_initialize(move |granted| {
        let _ = tx.send(granted);
    });
    // Long enough for a person to read the prompt and answer it.
    match rx.recv_timeout(Duration::from_secs(120)) {
        Ok(true) => Ok(()),
        _ => Err(
            "camera access is off for this terminal: allow it in System Settings › Privacy \
                  & Security › Camera, then run this again — or paste the code instead"
                .into(),
        ),
    }
}

#[cfg(not(target_os = "macos"))]
fn allow_camera() -> Result<(), String> {
    Ok(())
}

/// A greyscale frame, mirrored and fitted to the area, two pixels to a cell: `▀` with the upper
/// pixel as its foreground and the lower as its background, so pixels come out about square.
struct Preview<'a> {
    width: usize,
    height: usize,
    luma: &'a [u8],
}

impl Widget for Preview<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let (cols, rows) = (area.width as usize, area.height as usize * 2);
        if self.width == 0 || self.height == 0 || cols == 0 || rows == 0 {
            return;
        }
        // Fit the frame: the smaller scale of the two, so all of it shows.
        let scale = (cols as f32 / self.width as f32).min(rows as f32 / self.height as f32);
        let (w, h) = (
            ((self.width as f32 * scale) as usize).max(1),
            ((self.height as f32 * scale) as usize).max(1),
        );
        let (left, top) = ((cols - w) / 2, (rows - h) / 2 / 2);
        let pixel = |x: usize, y: usize| {
            let (sx, sy) = (
                x * self.width / w,
                (y * self.height / h).min(self.height - 1),
            );
            self.luma[sy * self.width + (self.width - 1 - sx)]
        };
        for cy in 0..h.div_ceil(2) {
            for x in 0..w {
                let upper = pixel(x, cy * 2);
                let lower = if cy * 2 + 1 < h {
                    pixel(x, cy * 2 + 1)
                } else {
                    0
                };
                let at = (area.x + (left + x) as u16, area.y + (top + cy) as u16);
                if let Some(cell) = buf.cell_mut(at) {
                    cell.set_symbol("▀")
                        .set_style(Style::new().fg(grey(upper)).bg(grey(lower)));
                }
            }
        }
    }
}

/// The 256-colour greyscale ramp rather than 24-bit colour, which macOS's Terminal lacks.
fn grey(value: u8) -> Color {
    match value {
        0..=3 => Color::Indexed(16),
        252..=255 => Color::Indexed(231),
        // 232..=255 runs from 8 to 238 in steps of 10.
        v => Color::Indexed(232 + ((v.saturating_sub(3)) as u16 * 24 / 249).min(23) as u8),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The line a robot's label QR carries: the label serial, then the code.
    const LABEL: &str = "123456789012345678:md1:microduck-7f3a:\
                         00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";

    /// `text` as a QR, `scale` pixels a module, on a white margin — a label held up to a camera,
    /// minus the camera.
    fn render(text: &str, scale: usize) -> (usize, Vec<u8>) {
        let qr = qrcode::QrCode::new(text.as_bytes()).unwrap();
        let modules = qr.width();
        let colors = qr.to_colors();
        let side = (modules + 8) * scale;
        let mut luma = vec![255u8; side * side];
        for y in 0..side {
            for x in 0..side {
                let (mx, my) = ((x / scale).wrapping_sub(4), (y / scale).wrapping_sub(4));
                if mx < modules && my < modules && colors[my * modules + mx] == qrcode::Color::Dark
                {
                    luma[y * side + x] = 0;
                }
            }
        }
        (side, luma)
    }

    #[test]
    fn the_label_qr_reads_as_its_setup_code() {
        let (side, luma) = render(LABEL, 4);
        assert_eq!(
            look(side, side, &luma),
            Seen::Code(SetupCode::parse(LABEL).unwrap())
        );
    }

    #[test]
    fn a_mirrored_camera_still_reads_it() {
        let (side, luma) = render(LABEL, 4);
        let mirrored: Vec<u8> = luma
            .chunks(side)
            .flat_map(|row| row.iter().rev().copied())
            .collect();
        assert!(matches!(look(side, side, &mirrored), Seen::Code(_)));
    }

    #[test]
    fn another_qr_is_not_a_code() {
        let (side, luma) = render("https://example.com/receipt", 4);
        assert_eq!(look(side, side, &luma), Seen::NotACode);
    }

    /// One hex character short, as a misprinted label would be: said so, rather than "not a code".
    #[test]
    fn a_malformed_code_says_why() {
        let (side, luma) = render(&LABEL[..LABEL.len() - 1], 4);
        let Seen::BadCode(why) = look(side, side, &luma) else {
            panic!("a short key read as something else");
        };
        assert!(why.contains("63"), "{why}");
    }

    #[test]
    fn a_blank_frame_is_nothing() {
        assert_eq!(look(64, 48, &[200u8; 64 * 48]), Seen::Nothing);
    }

    #[test]
    fn the_preview_fills_without_running_off_the_area() {
        let luma = vec![128u8; 640 * 480];
        let area = Rect::new(0, 0, 80, 24);
        let mut buf = Buffer::empty(area);
        Preview {
            width: 640,
            height: 480,
            luma: &luma,
        }
        .render(area, &mut buf);
        assert!(buf.content().iter().any(|cell| cell.symbol() == "▀"));
    }

    #[test]
    fn grey_stays_on_the_ramp() {
        for v in 0..=255u8 {
            assert!(matches!(grey(v), Color::Indexed(16 | 231 | 232..=255)));
        }
    }
}
