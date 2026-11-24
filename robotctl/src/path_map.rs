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

    /// The recorded track, (x, y), oldest first — for drawing it over a view that owns its own
    /// projection, the map's.
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// Where the robot is now, if it has been seen.
    pub fn here(&self) -> Option<(f64, f64, f64)> {
        self.here
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
            grid.line(dot(pair[0].0, pair[0].1), dot(pair[1].0, pair[1].1), None);
        }

        // Origin first, robot second: on the tick the robot is still at the
        // start, the robot is the thing to see.
        grid.mark(dot(0.0, 0.0), '+', Color::DarkGray);

        if let Some((x, y, yaw)) = self.here {
            if let Some(&(px, py)) = self.points.last() {
                grid.line(dot(px, py), dot(x, y), None);
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

// The map's colours, the ground-truth video's: walls red, floor seen clear the video's cyan at a
// third over a dark ground (so a room reads as a surface without drowning what is on it), maploc's
// track magenta, the robot yellow, stops grey. Unknown stays the terminal's own background.
const WALL: [u8; 3] = [255, 60, 60];
const FREE: [u8; 3] = [40, 85, 90];
const TRACK: [u8; 3] = [230, 60, 255];
const ROBOT: [u8; 3] = [255, 220, 40];
const STOP: [u8; 3] = [140, 140, 140];

/// The map `mapd` serves, top-down in the same orientation as the odometry track: the map's +x up
/// the screen, +y screen-left. Drawn in coloured pixels — two square ones per cell, the half-block
/// glyph of [`crate::cells`], like the 3D view — so walls, floor, track and robot each keep their
/// own colour instead of sharing one per braille cell.
///
/// Every map cell paints at least one pixel and walls are painted over floor, so a one-cell wall
/// survives a zoomed-out panel. The panel never grows — the world scales to fit, because the point
/// of a map is seeing all of it.
///
/// `track` is the robot's pose history **in the map frame**, never raw odometry: after a reboot or
/// a loop closure the two frames differ, and odometry drawn over the map reads as a bug.
pub fn draw_map(grid_msg: &proto::MapGridResult, track: &PathMap, area: Rect, buf: &mut Buffer) {
    let (w, h) = (area.width as usize, area.height as usize * 2);
    let (cols, rows) = (grid_msg.width as usize, grid_msg.height as usize);
    let cells = grid_msg.cells.as_bytes();
    if w < 8 || h < 8 || cols == 0 || rows == 0 || cells.len() != cols * rows {
        return;
    }
    let cell = grid_msg.res_m;
    let (span_x, span_y) = (cols as f64 * cell, rows as f64 * cell);
    // Map x runs up the screen and map y across it, so x is measured against the panel's height.
    // Half-block pixels are square, so one scale serves both.
    let scale =
        ((h - 1) as f64 / span_x.max(MIN_SPAN_M)).min((w - 1) as f64 / span_y.max(MIN_SPAN_M));
    let centre = (
        grid_msg.origin[0] + span_x / 2.0,
        grid_msg.origin[1] + span_y / 2.0,
    );
    let dot = |x: f64, y: f64| -> (isize, isize) {
        (
            (w as f64 / 2.0 - (y - centre.1) * scale).round() as isize,
            (h as f64 / 2.0 - (x - centre.0) * scale).round() as isize,
        )
    };

    let mut px = Pixels::new(w, h);
    // Floor first, then walls over it.
    for (kind, colour) in [(b'.', FREE), (b'#', WALL)] {
        for j in 0..rows {
            for i in 0..cols {
                if cells[j * cols + i] != kind {
                    continue;
                }
                let x0 = grid_msg.origin[0] + i as f64 * cell;
                let y0 = grid_msg.origin[1] + j as f64 * cell;
                px.fill(dot(x0, y0), dot(x0 + cell, y0 + cell), colour);
            }
        }
    }
    for k in &grid_msg.keyframes {
        px.set(dot(k.x, k.y), STOP);
    }
    for pair in track.points().windows(2) {
        px.line(dot(pair[0].0, pair[0].1), dot(pair[1].0, pair[1].1), TRACK);
    }
    if let Some((x, y, yaw)) = grid_msg
        .pose
        .as_ref()
        .map(|p| (p.x, p.y, p.yaw))
        .or(track.here())
    {
        // A heading ray of a fixed panel length, so it reads at every zoom, and a 3×3 body.
        let reach = 4.0 / scale;
        px.line(
            dot(x, y),
            dot(x + reach * yaw.cos(), y + reach * yaw.sin()),
            ROBOT,
        );
        let (cx, cy) = dot(x, y);
        px.fill((cx - 1, cy - 1), (cx + 1, cy + 1), ROBOT);
    }
    crate::cells::blit(area, buf, |x, y| px.get(x, y));
}

/// An RGB raster for [`draw_map`], `None` where nothing was drawn.
struct Pixels {
    w: usize,
    h: usize,
    px: Vec<Option<[u8; 3]>>,
}

impl Pixels {
    fn new(w: usize, h: usize) -> Self {
        Self {
            w,
            h,
            px: vec![None; w * h],
        }
    }

    fn get(&self, x: usize, y: usize) -> Option<[u8; 3]> {
        self.px.get(y * self.w + x).copied().flatten()
    }

    fn set(&mut self, (x, y): (isize, isize), c: [u8; 3]) {
        if x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h {
            self.px[y as usize * self.w + x as usize] = Some(c);
        }
    }

    /// The rectangle between two corners, inclusive, clipped: a cell smaller than a pixel still
    /// paints one.
    fn fill(&mut self, a: (isize, isize), b: (isize, isize), c: [u8; 3]) {
        let (x0, x1) = (a.0.min(b.0).max(0), a.0.max(b.0).min(self.w as isize - 1));
        let (y0, y1) = (a.1.min(b.1).max(0), a.1.max(b.1).min(self.h as isize - 1));
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.px[y as usize * self.w + x as usize] = Some(c);
            }
        }
    }

    /// Bresenham; endpoints may be off-panel (`set` clips), but a line whose ends are both far
    /// outside is skipped rather than walked.
    fn line(&mut self, a: (isize, isize), b: (isize, isize), c: [u8; 3]) {
        let (dx, dy) = ((b.0 - a.0).abs(), -(b.1 - a.1).abs());
        if dx.max(-dy) > (self.w + self.h) as isize * 4 {
            return;
        }
        let (sx, sy) = (
            if a.0 < b.0 { 1 } else { -1 },
            if a.1 < b.1 { 1 } else { -1 },
        );
        let (mut x, mut y, mut err) = (a.0, a.1, dx + dy);
        loop {
            self.set((x, y), c);
            if (x, y) == b {
                break;
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
}

/// A braille dot grid with per-cell colour and a few character overrides — the odometry track.
struct Grid {
    w: usize,
    h: usize,
    /// Braille dot mask per cell (`U+2800 + mask` is the glyph).
    dots: Vec<u8>,
    color: Vec<Option<Color>>,
    /// A character that replaces the braille in its cell — markers beat track.
    over: Vec<Option<(char, Color)>>,
}

const BITS: [[u8; 2]; 4] = [[0x01, 0x08], [0x02, 0x10], [0x04, 0x20], [0x40, 0x80]];

impl Grid {
    fn new(w: usize, h: usize) -> Self {
        Self {
            w,
            h,
            dots: vec![0; w * h],
            color: vec![None; w * h],
            over: vec![None; w * h],
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

    /// A wall must ink the panel as a line, floor must fill its background, and the robot must be
    /// its yellow marker — the three things a glance at the map is for.
    #[test]
    fn the_map_draws_walls_floor_and_the_robot() {
        // 10 × 10 cells of 0.1 m: a wall along the top (max x) row, floor elsewhere.
        let mut cells = String::new();
        for _y in 0..10 {
            for x in 0..10 {
                cells.push(if x == 9 { '#' } else { '.' });
            }
        }
        let grid = proto::MapGridResult {
            res_m: 0.1,
            origin: [0.0, 0.0],
            width: 10,
            height: 10,
            cells,
            keyframes: vec![],
            pose: Some(proto::MapPose {
                x: 0.5,
                y: 0.5,
                yaw: 0.0,
            }),
        };
        let area = Rect::new(0, 0, 20, 10);
        let mut buf = Buffer::empty(area);
        draw_map(&grid, &PathMap::new(), area, &mut buf);
        let rgb = |c: [u8; 3]| Color::Rgb(c[0], c[1], c[2]);
        let painted =
            |x: u16, y: u16, c: [u8; 3]| buf[(x, y)].fg == rgb(c) || buf[(x, y)].bg == rgb(c);
        // The wall is the top (max x) row of cells: red along the panel's top.
        assert!(
            (0..20).filter(|&x| painted(x, 0, WALL)).count() >= 10,
            "no wall along the top"
        );
        assert!(
            (0..10).any(|y| (0..20).any(|x| painted(x, y, ROBOT))),
            "no robot"
        );
        assert!(
            (0..10).any(|y| (0..20).any(|x| painted(x, y, FREE))),
            "no floor"
        );
        // Drawn in half blocks, like the 3D view — not braille.
        assert!((0..10).any(|y| (0..20).any(|x| buf[(x, y)].symbol() == "▀")));
    }

    fn painted(map: &PathMap, w: u16, h: u16) -> Vec<String> {
        let area = Rect::new(0, 0, w, h);
        let mut buf = Buffer::empty(area);
        map.draw(area, &mut buf);
        (0..h)
            .map(|y| (0..w).map(|x| buf[(x, y)].symbol().to_owned()).collect())
            .collect()
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

#[cfg(test)]
mod picture {
    use super::*;

    /// Not a test — draws a replay export's map (`MAP_JSON`, `replay --export`) as the monitor
    /// would, at `COLS×ROWS` cells, into `MAP_PPM`, two pixels a cell scaled up. Run with:
    ///
    ///     MAP_JSON=run.json MAP_PPM=map.ppm cargo test -p robotctl picture -- --ignored
    #[test]
    #[ignore = "writes a picture, run by hand"]
    fn picture_of_a_replay_map() {
        let v: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(std::env::var("MAP_JSON").unwrap()).unwrap(),
        )
        .unwrap();
        let g = &v["grid"];
        let rows: Vec<String> = g["rows"]
            .as_array()
            .unwrap()
            .iter()
            .map(|r| r.as_str().unwrap().to_owned())
            .collect();
        let pose = v["poses"]
            .as_array()
            .unwrap()
            .iter()
            .rev()
            .find_map(|p| p["pose"].as_array().cloned());
        let grid = proto::MapGridResult {
            res_m: g["res"].as_f64().unwrap(),
            origin: [
                g["origin"][0].as_f64().unwrap(),
                g["origin"][1].as_f64().unwrap(),
            ],
            width: g["w"].as_u64().unwrap() as u32,
            height: g["h"].as_u64().unwrap() as u32,
            cells: rows.concat(),
            keyframes: vec![],
            pose: pose.map(|p| proto::MapPose {
                x: p[0].as_f64().unwrap(),
                y: p[1].as_f64().unwrap(),
                yaw: p[2].as_f64().unwrap(),
            }),
        };
        let (cols, rws): (u16, u16) = (
            std::env::var("COLS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(120),
            std::env::var("ROWS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(40),
        );
        let area = Rect::new(0, 0, cols, rws);
        let mut buf = Buffer::empty(area);
        draw_map(&grid, &PathMap::new(), area, &mut buf);
        let (cw, ch) = (6usize, 12usize);
        let (w, h) = (cols as usize * cw, rws as usize * ch);
        let mut img = vec![[20u8, 20, 24]; w * h];
        let rgb = |c: Color| match c {
            Color::Rgb(r, g, b) => Some([r, g, b]),
            _ => None,
        };
        for y in 0..rws {
            for x in 0..cols {
                let c = &buf[(x, y)];
                let (top, bot) = match c.symbol() {
                    "▀" => (rgb(c.fg), rgb(c.bg)),
                    "▄" => (rgb(c.bg), rgb(c.fg)),
                    _ => (None, None),
                };
                for py in 0..ch {
                    for px in 0..cw {
                        let col = if py < ch / 2 { top } else { bot };
                        if let Some(col) = col {
                            img[(y as usize * ch + py) * w + x as usize * cw + px] = col;
                        }
                    }
                }
            }
        }
        let mut out = format!("P6\n{w} {h}\n255\n").into_bytes();
        for p in img {
            out.extend_from_slice(&p);
        }
        std::fs::write(std::env::var("MAP_PPM").unwrap(), out).unwrap();
    }
}
