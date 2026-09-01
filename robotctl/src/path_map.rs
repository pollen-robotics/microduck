//! A top-down map of where odometry says the robot has been.
//!
//! Lives under the 3D robot view: the view answers "what is the robot doing",
//! this answers "where has it gone". The panel's size never changes — instead
//! the *world* scales, zooming out as the track grows so the whole path is
//! always in frame. Drawn in braille (2×4 dots per cell), which is the finest
//! resolution a terminal offers for a line that curves.
//!
//! Conventions: world +x (the robot's heading at boot) points up the screen,
//! world +y (its left) points screen-left — the map is what you would see
//! standing over the robot's starting pose. The origin is marked `+`, the
//! robot `●` with a short ray for its heading.

use duck_ipc_proto as proto;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::{Color, Style};

/// Metres the robot must move before another point is recorded. Doubles on
/// every decimation, so the track's memory stays bounded while its shape —
/// which a longer walk draws at a smaller scale anyway — survives.
const FIRST_STEP: f64 = 0.02;

/// Recorded points cap. At [`FIRST_STEP`] this is ~40 m of walking before the
/// first decimation; each decimation doubles the reach.
const CAPACITY: usize = 2048;

/// The world is never drawn narrower than this many metres across, so IMU
/// jitter on a standing robot stays a dot instead of zooming into a scribble.
const MIN_SPAN_M: f64 = 1.0;

pub struct PathMap {
    /// The track, oldest first, spaced at least `min_step` apart.
    points: Vec<(f64, f64)>,
    min_step: f64,
    /// Where the robot is right now: x, y, yaw. Tracked separately from the
    /// points so the marker moves every frame even between recorded steps.
    here: Option<(f64, f64, f64)>,
}

impl PathMap {
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            min_step: FIRST_STEP,
            here: None,
        }
    }

    /// Feed one odometry estimate, as `robot.state` reports it.
    pub fn observe(&mut self, x: f64, y: f64, yaw: f64) {
        self.here = Some((x, y, yaw));
        let far_enough = match self.points.last() {
            None => true,
            Some((px, py)) => (x - px).hypot(y - py) >= self.min_step,
        };
        if !far_enough {
            return;
        }
        self.points.push((x, y));

        if self.points.len() >= CAPACITY {
            // Keep every other point but always the first (the origin end) and
            // the last (the fresh end) — the shape thins, the ends hold.
            let last = self.points.len() - 1;
            self.points = std::mem::take(&mut self.points)
                .into_iter()
                .enumerate()
                .filter(|(i, _)| *i == 0 || *i == last || i.is_multiple_of(2))
                .map(|(_, p)| p)
                .collect();
            self.min_step *= 2.0;
        }
    }

    /// The recorded track, world (x, y), oldest first — for overlaying the
    /// path on views that own their own projection (the live map).
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// How wide the tracked world currently is, metres — the number the panel
    /// caption shows so the zoom level is legible.
    pub fn extent_m(&self) -> f64 {
        let (min, max) = self.bounds();
        (max.0 - min.0).max(max.1 - min.1).max(MIN_SPAN_M)
    }

    /// The world box the map must show: every recorded point, the live
    /// position, and always the origin — you can always see where you started.
    fn bounds(&self) -> ((f64, f64), (f64, f64)) {
        let mut min = (0.0f64, 0.0f64);
        let mut max = (0.0f64, 0.0f64);
        let here = self.here.map(|(x, y, _)| (x, y));
        for &(x, y) in self.points.iter().chain(here.iter()) {
            min = (min.0.min(x), min.1.min(y));
            max = (max.0.max(x), max.1.max(y));
        }
        (min, max)
    }

    /// Paint the map into `area` (the inside of the caller's block).
    pub fn draw(&self, area: Rect, buf: &mut Buffer) {
        let (w, h) = (area.width as usize * 2, area.height as usize * 4);
        if w < 8 || h < 8 {
            return;
        }

        // One scale for both axes — a map that stretches distances differently
        // per direction is not a map — chosen so the whole box fits, then
        // centred. The 6% margin keeps the track off the border.
        let (min, max) = self.bounds();
        let centre = ((min.0 + max.0) / 2.0, (min.1 + max.1) / 2.0);
        let span = (
            (max.0 - min.0).max(MIN_SPAN_M) * 1.06,
            (max.1 - min.1).max(MIN_SPAN_M) * 1.06,
        );
        // +x up the screen, +y screen-left: dots are (column, row) from top-left.
        let scale = ((h - 1) as f64 / span.0).min((w - 1) as f64 / span.1);
        let dot = |x: f64, y: f64| -> (isize, isize) {
            (
                (w as f64 / 2.0 - (y - centre.1) * scale).round() as isize,
                (h as f64 / 2.0 - (x - centre.0) * scale).round() as isize,
            )
        };

        let mut grid = Grid::new(area.width as usize, area.height as usize);
        for pair in self.points.windows(2) {
            grid.line(
                dot(pair[0].0, pair[0].1),
                dot(pair[1].0, pair[1].1),
                Some(Color::Red),
            );
        }

        // Origin first, robot second: on the tick the robot is still at the
        // start, the robot is the thing to see.
        grid.mark(dot(0.0, 0.0), '+', Color::DarkGray);

        if let Some((x, y, yaw)) = self.here {
            if let Some(&(px, py)) = self.points.last() {
                grid.line(dot(px, py), dot(x, y), Some(Color::Red));
            }
            // The heading ray: from the robot, one panel-relative step forward,
            // so it reads at every zoom level rather than shrinking with the map.
            let reach = 5.0 / scale;
            let tip = (x + reach * yaw.cos(), y + reach * yaw.sin());
            grid.line(dot(x, y), dot(tip.0, tip.1), Some(Color::Yellow));
            grid.mark(dot(x, y), '●', Color::Yellow);
        }

        grid.paint(area, buf);
    }
}

/// Confirmed-floor fill. An indexed colour (256-colour dark grey) rather
/// than RGB, so it renders everywhere the monitor does.
const FLOOR: Color = Color::Indexed(236);

/// Draw a rendered occupancy map ([`proto::MapFrame`]) into `area`: the
/// confirmed floor as a solid dark fill, walls as bright braille covering
/// their whole footprint, the robot's map-frame pose as the same yellow
/// marker the odometry path uses. The room must read as a *shape* — the
/// first cut drew one dot per wall cell over a sparse floor stipple, and a
/// real lap rendered as scattered stars. The panel never grows; the world
/// scales to fit — the map's whole point is seeing all of it.
pub fn draw_map(frame: &proto::MapFrame, path: &PathMap, area: Rect, buf: &mut Buffer) {
    let (w, h) = (area.width as usize * 2, area.height as usize * 4);
    if w < 8 || h < 8 {
        return;
    }
    let Some(cells) = proto::b64::decode(&frame.cells) else {
        return;
    };
    let (rows, cols) = (frame.rows as usize, frame.cols as usize);
    if cells.len() != rows * cols || rows == 0 || cols == 0 {
        return;
    }

    let cell = frame.cell_m;
    let span_x = cols as f32 * cell; // world x = columns
    let span_y = rows as f32 * cell;
    // +x up the screen, +y screen-left, like the odometry path map.
    let scale = ((h - 1) as f32 / span_x).min((w - 1) as f32 / span_y);
    let centre = (frame.x_min + span_x / 2.0, frame.y_min + span_y / 2.0);
    let dot = |x: f32, y: f32| -> (isize, isize) {
        (
            (w as f32 / 2.0 - (y - centre.1) * scale).round() as isize,
            (h as f32 / 2.0 - (x - centre.0) * scale).round() as isize,
        )
    };

    let mut grid = Grid::new(area.width as usize, area.height as usize);
    // Each map cell covers a little rectangle of braille dots (or, zoomed
    // out, a single dot) — filling the footprint is what joins a wall's
    // cells into a line and a floor's into a surface.
    let half = cell / 2.0;
    for i in 0..rows {
        for j in 0..cols {
            let v = cells[i * cols + j];
            if v == 0 {
                continue;
            }
            let x = frame.x_min + (j as f32 + 0.5) * cell;
            let y = frame.y_min + (i as f32 + 0.5) * cell;
            let a = dot(x - half, y - half);
            let b = dot(x + half, y + half);
            if v == 2 {
                grid.fill(a, b, Some(Color::Gray));
            } else {
                grid.shade(a, b);
            }
        }
    }

    // The walked path, red, on top of the rooms it was walked through. The
    // path lives in the odometry frame, the map in the map frame — identical
    // until loop closures diverge them, and worth seeing together even then.
    let pts = path.points();
    for pair in pts.windows(2) {
        grid.line(
            dot(pair[0].0 as f32, pair[0].1 as f32),
            dot(pair[1].0 as f32, pair[1].1 as f32),
            Some(Color::Red),
        );
    }

    grid.mark(dot(0.0, 0.0), '+', Color::DarkGray);
    let (rx, ry, ryaw) = (frame.x as f32, frame.y as f32, frame.yaw as f32);
    let reach = 5.0 / scale;
    grid.line(
        dot(rx, ry),
        dot(rx + reach * ryaw.cos(), ry + reach * ryaw.sin()),
        Some(Color::Yellow),
    );
    // Searching robots draw differently: a pose not to be trusted must not
    // look like one that is.
    if frame.tracking {
        grid.mark(dot(rx, ry), '●', Color::Yellow);
    } else {
        grid.mark(dot(rx, ry), '?', Color::Magenta);
    }
    grid.paint(area, buf);
}

/// A braille dot grid with per-cell colour, an optional background shade
/// per dot (the map's floor), and a few character overrides.
struct Grid {
    w: usize,
    h: usize,
    /// Braille dot mask per cell (`U+2800 + mask` is the glyph).
    dots: Vec<u8>,
    color: Vec<Option<Color>>,
    /// Shaded ("floor") dots, one bit-mask per cell like `dots`. A cell
    /// mostly covered by floor paints its background.
    shade: Vec<u8>,
    /// A character that replaces the braille in its cell — markers beat track.
    over: Vec<Option<(char, Color)>>,
}

impl Grid {
    fn new(w: usize, h: usize) -> Self {
        Self {
            w,
            h,
            dots: vec![0; w * h],
            color: vec![None; w * h],
            shade: vec![0; w * h],
            over: vec![None; w * h],
        }
    }

    /// Set every dot in the rectangle spanned by two (possibly unordered,
    /// possibly off-screen) corners.
    fn fill(&mut self, a: (isize, isize), b: (isize, isize), color: Option<Color>) {
        let (x0, x1) = (
            a.0.min(b.0).max(0),
            a.0.max(b.0).min(self.w as isize * 2 - 1),
        );
        let (y0, y1) = (
            a.1.min(b.1).max(0),
            a.1.max(b.1).min(self.h as isize * 4 - 1),
        );
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.set((x, y), color);
            }
        }
    }

    /// Mark every dot in the rectangle as floor.
    fn shade(&mut self, a: (isize, isize), b: (isize, isize)) {
        let (x0, x1) = (
            a.0.min(b.0).max(0),
            a.0.max(b.0).min(self.w as isize * 2 - 1),
        );
        let (y0, y1) = (
            a.1.min(b.1).max(0),
            a.1.max(b.1).min(self.h as isize * 4 - 1),
        );
        const BITS: [[u8; 2]; 4] = [[0x01, 0x08], [0x02, 0x10], [0x04, 0x20], [0x40, 0x80]];
        for y in y0..=y1 {
            for x in x0..=x1 {
                let cell = (y as usize / 4) * self.w + x as usize / 2;
                self.shade[cell] |= BITS[y as usize % 4][x as usize % 2];
            }
        }
    }

    fn set(&mut self, (x, y): (isize, isize), color: Option<Color>) {
        if x < 0 || y < 0 || x as usize >= self.w * 2 || y as usize >= self.h * 4 {
            return;
        }
        let (x, y) = (x as usize, y as usize);
        let cell = (y / 4) * self.w + x / 2;
        // The braille bit layout: dots 1-2-3-7 down the left column, 4-5-6-8
        // down the right.
        const BITS: [[u8; 2]; 4] = [[0x01, 0x08], [0x02, 0x10], [0x04, 0x20], [0x40, 0x80]];
        self.dots[cell] |= BITS[y % 4][x % 2];
        if color.is_some() {
            self.color[cell] = color;
        }
    }

    /// Bresenham over the dot grid. Endpoints may be far off-screen — `set`
    /// clips — but the step count is bounded by the panel, not the world.
    fn line(&mut self, a: (isize, isize), b: (isize, isize), color: Option<Color>) {
        let (dx, dy) = ((b.0 - a.0).abs(), -(b.1 - a.1).abs());
        let steps = dx.max(-dy).max(1);
        if steps > (self.w * 2 + self.h * 4) as isize * 4 {
            return; // both ends far outside; nothing visible to draw
        }
        let (sx, sy) = (
            if a.0 < b.0 { 1 } else { -1 },
            if a.1 < b.1 { 1 } else { -1 },
        );
        let (mut x, mut y, mut err) = (a.0, a.1, dx + dy);
        loop {
            self.set((x, y), color);
            if (x, y) == b {
                return;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    fn mark(&mut self, (x, y): (isize, isize), glyph: char, color: Color) {
        if x < 0 || y < 0 || x as usize >= self.w * 2 || y as usize >= self.h * 4 {
            return;
        }
        let cell = (y as usize / 4) * self.w + x as usize / 2;
        self.over[cell] = Some((glyph, color));
    }

    fn paint(&self, area: Rect, buf: &mut Buffer) {
        for row in 0..self.h {
            for col in 0..self.w {
                let cell = row * self.w + col;
                let pos = (area.x + col as u16, area.y + row as u16);
                // Floor: paint the background when at least half the
                // cell's dots are confirmed floor — a solid surface reads
                // as a room where a stipple read as noise.
                if self.shade[cell].count_ones() >= 4 {
                    buf[pos].set_bg(FLOOR);
                }
                if let Some((glyph, color)) = self.over[cell] {
                    buf[pos].set_char(glyph).set_style(Style::new().fg(color));
                } else if self.dots[cell] != 0 {
                    let glyph = char::from_u32(0x2800 + u32::from(self.dots[cell]))
                        .expect("braille block is dense");
                    let style = match self.color[cell] {
                        Some(color) => Style::new().fg(color),
                        None => Style::new().dim(),
                    };
                    buf[pos].set_char(glyph).set_style(style);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn painted(map: &PathMap, w: u16, h: u16) -> Vec<String> {
        let area = Rect::new(0, 0, w, h);
        let mut buf = Buffer::empty(area);
        map.draw(area, &mut buf);
        (0..h)
            .map(|y| (0..w).map(|x| buf[(x, y)].symbol().to_owned()).collect())
            .collect()
    }

    /// A wall in the frame must ink the panel; the robot must be its yellow
    /// marker; a searching robot must not look locked.
    #[test]
    fn a_map_frame_draws_walls_and_the_robot() {
        let mut cells = vec![1u8; 8 * 8]; // all free
        for j in 0..8 {
            cells[7 * 8 + j] = 2; // a wall along the top row (max y? row 7)
        }
        let frame = proto::MapFrame {
            seq: 1,
            x: 0.1,
            y: 0.1,
            yaw: 0.0,
            tracking: true,
            x_min: 0.0,
            y_min: 0.0,
            cell_m: 0.05,
            rows: 8,
            cols: 8,
            cells: proto::b64::encode(&cells),
            n_submaps: 1,
            n_loops: 0,
            windows: 1,
            still: false,
            seated: false,
        };
        let area = Rect::new(0, 0, 30, 10);
        let mut buf = Buffer::empty(area);
        draw_map(&frame, &PathMap::new(), area, &mut buf);
        let rendered: String = (0..10)
            .flat_map(|y| (0..30).map(move |x| (x, y)))
            .map(|(x, y)| buf[(x, y)].symbol().to_owned())
            .collect();
        assert!(rendered.contains('●'), "robot marker missing: {rendered:?}");
        assert!(
            rendered
                .chars()
                .any(|c| ('\u{2800}'..='\u{28FF}').contains(&c)),
            "no braille ink at all"
        );
        let floor_cells = (0..10)
            .flat_map(|y| (0..30).map(move |x| (x, y)))
            .filter(|&(x, y)| buf[(x, y)].bg == FLOOR)
            .count();
        assert!(
            floor_cells > 10,
            "the confirmed floor must paint a solid background, got {floor_cells} cells"
        );

        let searching = proto::MapFrame {
            tracking: false,
            ..frame
        };
        let mut buf = Buffer::empty(area);
        draw_map(&searching, &PathMap::new(), area, &mut buf);
        let rendered: String = (0..10)
            .flat_map(|y| (0..30).map(move |x| (x, y)))
            .map(|(x, y)| buf[(x, y)].symbol().to_owned())
            .collect();
        assert!(rendered.contains('?'), "a searching pose must say so");
        assert!(!rendered.contains('●'));
    }

    /// Not a test — a stopwatch for a full-capacity track. Run with:
    ///
    ///     cargo test -p robotctl --release time_the_map -- --ignored --nocapture
    #[test]
    #[ignore = "perf probe, run manually with --release --nocapture"]
    fn time_the_map() {
        let mut map = PathMap::new();
        for i in 0..(CAPACITY as i32 * 2) {
            let t = f64::from(i) * 0.05;
            map.observe(t.sin() * t * 0.01, t.cos() * t * 0.01, t);
        }
        let area = Rect::new(0, 0, 40, 10);
        let mut buf = Buffer::empty(area);
        let start = std::time::Instant::now();
        const ITERS: u32 = 10_000;
        for _ in 0..ITERS {
            map.draw(area, &mut buf);
        }
        let per = start.elapsed().as_nanos() as f64 / f64::from(ITERS);
        println!("path map: {per:.0} ns/draw at {} points", map.points.len());
    }

    /// Not a test — eyes on the actual pixels. Run with:
    ///
    ///     cargo test -p robotctl path_map -- --ignored --nocapture
    #[test]
    #[ignore = "visual probe, run manually with --nocapture"]
    fn show_me_a_loop_walk() {
        let mut map = PathMap::new();
        for i in 0..400 {
            // A lap: out, a wide arc, and back past the origin.
            let t = f64::from(i) * 0.02;
            map.observe(
                2.0 * t.sin(),
                1.2 * (1.0 - t.cos()),
                t + std::f64::consts::FRAC_PI_2,
            );
        }
        for row in painted(&map, 40, 10) {
            println!("|{row}|");
        }
        println!("extent: {:.1} m", map.extent_m());
    }

    /// A robot that has not moved is a marker at the middle of the panel, not
    /// a blank and not a zoomed-in mess of jitter.
    #[test]
    fn a_standing_robot_is_a_dot_in_the_middle() {
        let mut map = PathMap::new();
        map.observe(0.0, 0.0, 0.0);
        let rows = painted(&map, 30, 10);
        let mid = &rows[5];
        assert!(mid.contains('●'), "no robot marker: {rows:?}");
        assert!(
            (map.extent_m() - 1.0).abs() < 1e-9,
            "standing still must not zoom in"
        );
    }

    /// Walking forward (world +x) must draw *up* the panel: the track's ink
    /// ends higher than it starts, and the extent caption grows with the walk.
    #[test]
    fn forward_is_up_and_the_zoom_follows_the_walk() {
        let mut map = PathMap::new();
        for i in 0..=100 {
            map.observe(f64::from(i) * 0.05, 0.0, 0.0);
        }
        assert!((map.extent_m() - 5.0).abs() < 0.01, "{}", map.extent_m());

        let rows = painted(&map, 30, 12);
        let ink = |row: &String| row.chars().filter(|c| *c != ' ').count();
        assert!(
            ink(&rows[1]) > 0 && ink(&rows[10]) > 0,
            "a 5 m walk should span the panel top to bottom: {rows:?}"
        );
    }

    /// The memory is bounded: however long the walk, the track never holds
    /// more than [`CAPACITY`] points, and both ends survive the thinning.
    #[test]
    fn a_long_walk_thins_but_keeps_its_ends() {
        let mut map = PathMap::new();
        for i in 0..200_000 {
            map.observe(f64::from(i) * 0.02, 0.0, 0.0);
        }
        assert!(map.points.len() < CAPACITY);
        assert_eq!(map.points[0], (0.0, 0.0), "the origin end was thinned away");
        let last = map.points.last().expect("points survive");
        assert!(last.0 > 3990.0, "the fresh end was thinned away: {last:?}");
    }

    /// Drawing into a sliver must be a no-op, not a panic — the panel can be
    /// squeezed to nothing while the terminal resizes.
    #[test]
    fn a_tiny_area_draws_nothing_and_survives() {
        let mut map = PathMap::new();
        map.observe(1.0, 2.0, 0.5);
        let _ = painted(&map, 3, 1);
    }
}
