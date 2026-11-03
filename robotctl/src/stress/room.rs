//! The room, from `mapd`'s map: where the walls are, how far each place is from them, and which
//! way a walk can go without coming closer to one than `--wall`.
//!
//! The map is the one `robotctl map load` put on every robot under test — built once, carefully,
//! on one of them — so every robot keeps off the same walls in the same frame. It replaces the
//! first guards' two approximations: the odometry square, which drifted with every fall, and the
//! head ToF, which sees only ahead and had to turn the robot round to look behind it.
//!
//! Everything here is geometry on a grid. The driver ([`super`]) decides when to ask.

use duck_ipc_proto as proto;

/// A pose in the map frame: metres, metres, radians.
pub type Pose = (f64, f64, f64);

/// `a ∘ b`: `b` expressed in `a`'s frame, carried into the frame `a` is in.
pub fn compose(a: Pose, b: Pose) -> Pose {
    let (s, c) = a.2.sin_cos();
    (
        a.0 + c * b.0 - s * b.1,
        a.1 + s * b.0 + c * b.1,
        super::guard::wrap(a.2 + b.2),
    )
}

/// The inverse of a pose: `compose(p, inverse(p))` is the identity.
pub fn inverse(p: Pose) -> Pose {
    let (s, c) = p.2.sin_cos();
    (-c * p.0 - s * p.1, s * p.0 - c * p.1, -p.2)
}

/// Headings tried when choosing which way to walk: every 15°.
const HEADINGS: usize = 24;

/// The occupancy grid, and each cell's distance to the nearest place the robot must not be.
#[derive(Debug, Clone)]
pub struct Room {
    res: f64,
    origin: [f64; 2],
    width: usize,
    height: usize,
    /// Metres from the cell's centre to the nearest wall or unknown cell's centre. Zero on one.
    clearance: Vec<f64>,
    /// The most open place in the room: the free cell furthest from everything.
    centre: (f64, f64),
}

impl Room {
    /// The room `mapd` drew. `None` when it has drawn nothing the robot could stand on.
    ///
    /// **Unknown counts as a wall.** The robot walks only on floor the map saw: a hole in a
    /// carefully built map is rare, and the edge of what was seen is usually a wall the ToF did
    /// not quite reach.
    pub fn from_grid(g: &proto::MapGridResult) -> Option<Self> {
        let (width, height) = (g.width as usize, g.height as usize);
        let cells = g.cells.as_bytes();
        if width == 0 || height == 0 || cells.len() != width * height || g.res_m <= 0.0 {
            return None;
        }
        let blocked = |i: usize| cells[i] != b'.';
        let clearance = distance_transform(width, height, &blocked)
            .into_iter()
            .map(|d2| d2.sqrt() * g.res_m)
            .collect::<Vec<_>>();
        // The most open cell, and in a room longer than it is wide that is a ridge down its middle:
        // the middle of the ridge.
        let most = (0..width * height)
            .filter(|&i| !blocked(i))
            .map(|i| clearance[i])
            .max_by(f64::total_cmp)?;
        let ridge: Vec<usize> = (0..width * height)
            .filter(|&i| !blocked(i) && clearance[i] >= most - g.res_m / 2.0)
            .collect();
        let (sx, sy) = ridge.iter().fold((0.0, 0.0), |(sx, sy), &i| {
            (sx + (i % width) as f64, sy + (i / width) as f64)
        });
        let (mx, my) = (sx / ridge.len() as f64, sy / ridge.len() as f64);
        let off = |i: usize| ((i % width) as f64 - mx).hypot((i / width) as f64 - my);
        let best = *ridge.iter().min_by(|&&a, &&b| off(a).total_cmp(&off(b)))?;
        let mut room = Self {
            res: g.res_m,
            origin: g.origin,
            width,
            height,
            clearance,
            centre: (0.0, 0.0),
        };
        room.centre = room.cell_centre(best);
        Some(room)
    }

    fn cell_centre(&self, i: usize) -> (f64, f64) {
        let (cx, cy) = (i % self.width, i / self.width);
        (
            self.origin[0] + (cx as f64 + 0.5) * self.res,
            self.origin[1] + (cy as f64 + 0.5) * self.res,
        )
    }

    /// The most open place in the room — where a lap is best centred.
    pub fn centre(&self) -> (f64, f64) {
        self.centre
    }

    /// How far `(x, y)` is from the nearest wall or unknown cell, metres. Zero off the map.
    pub fn clearance(&self, x: f64, y: f64) -> f64 {
        let cx = ((x - self.origin[0]) / self.res).floor();
        let cy = ((y - self.origin[1]) / self.res).floor();
        if cx < 0.0 || cy < 0.0 || cx >= self.width as f64 || cy >= self.height as f64 {
            return 0.0;
        }
        self.clearance[cy as usize * self.width + cx as usize]
    }

    /// How far the robot at `pose` can travel straight along `direction` (its heading for a walk
    /// forward, the opposite for a walk back), up to `want`, before it would come within `margin`
    /// of a wall.
    ///
    /// **Never closer, rather than never close.** A robot already inside the margin — put down
    /// there, or fallen into it — may walk as long as every place on the way is no closer than
    /// where it stands: away from the wall, or along it, but never into it.
    pub fn free_run(&self, pose: Pose, direction: f64, want: f64, margin: f64) -> f64 {
        let (x, y, _) = pose;
        let floor = margin.min(self.clearance(x, y));
        let step = self.res / 2.0;
        let (s, c) = direction.sin_cos();
        let mut d = step;
        while d <= want {
            if self.clearance(x + c * d, y + s * d) < floor {
                return d - step;
            }
            d += step;
        }
        want
    }

    /// The heading to walk a step from: one along which the robot can travel `travel` metres
    /// (negative: backwards) and keep `margin` off every wall.
    ///
    /// Of the headings that work, the one needing the least turn — unless the robot is further
    /// than `square` from `home`, when it is the one closest to taking it home. When none works,
    /// the one with the longest free run, and the walk's live guard ends the step where the run
    /// does.
    pub fn heading_for(
        &self,
        pose: Pose,
        travel: f64,
        margin: f64,
        home: (f64, f64),
        square: f64,
    ) -> Heading {
        let (x, y, yaw) = pose;
        let away = (x - home.0).hypot(y - home.1) > square;
        let to_home = (home.1 - y).atan2(home.0 - x);
        let back = if travel < 0.0 {
            std::f64::consts::PI
        } else {
            0.0
        };
        let want = travel.abs();
        let mut best: Option<(f64, f64, f64)> = None; // (cost, heading, run)
        let mut longest: (f64, f64) = (yaw, -1.0); // (heading, run)
        for k in 0..HEADINGS {
            // Starting from the current heading, so a tie keeps the robot as it is.
            let heading =
                super::guard::wrap(yaw + k as f64 * std::f64::consts::TAU / HEADINGS as f64);
            let direction = heading + back;
            let run = self.free_run(pose, direction, want, margin);
            if run > longest.1 {
                longest = (heading, run);
            }
            if run < want {
                continue;
            }
            let cost = if away {
                super::guard::wrap(direction - to_home).abs()
            } else {
                super::guard::wrap(heading - yaw).abs()
            };
            if best.is_none_or(|(c, _, _)| cost < c - 1e-9) {
                best = Some((cost, heading, run));
            }
        }
        match best {
            Some((_, heading, _)) => Heading {
                yaw: heading,
                run: want,
                homeward: away,
            },
            None => Heading {
                yaw: longest.0,
                run: longest.1.max(0.0),
                homeward: away,
            },
        }
    }
}

/// Which way to face for a walk, and how far it can go.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Heading {
    pub yaw: f64,
    /// The free run that way: the whole walk, or less when no heading had all of it.
    pub run: f64,
    /// Chosen to bring the robot back towards home rather than to turn least.
    pub homeward: bool,
}

/// Squared distance, in cells, from every cell to the nearest blocked one — Felzenszwalb and
/// Huttenlocher's exact transform, a column pass then a row pass. A grid with nothing blocked
/// gets a distance larger than any room.
fn distance_transform(width: usize, height: usize, blocked: &dyn Fn(usize) -> bool) -> Vec<f64> {
    const FAR: f64 = 1e12;
    let mut grid: Vec<f64> = (0..width * height)
        .map(|i| if blocked(i) { 0.0 } else { FAR })
        .collect();
    let mut column = vec![0.0; height];
    for x in 0..width {
        for y in 0..height {
            column[y] = grid[y * width + x];
        }
        let out = dt_1d(&column);
        for y in 0..height {
            grid[y * width + x] = out[y];
        }
    }
    for y in 0..height {
        let out = dt_1d(&grid[y * width..(y + 1) * width]);
        grid[y * width..(y + 1) * width].copy_from_slice(&out);
    }
    grid
}

/// The 1-D squared distance transform under the lower envelope of parabolas.
fn dt_1d(f: &[f64]) -> Vec<f64> {
    let n = f.len();
    let mut d = vec![0.0; n];
    let mut v = vec![0usize; n];
    let mut z = vec![0.0f64; n + 1];
    let mut k = 0usize;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    let sq = |q: usize| (q * q) as f64;
    let meet =
        |q: usize, p: usize| ((f[q] + sq(q)) - (f[p] + sq(p))) / (2.0 * (q as f64 - p as f64));
    for q in 1..n {
        // z[0] is −∞, so this stops at k = 0 at the latest.
        let mut s = meet(q, v[k]);
        while s <= z[k] {
            k -= 1;
            s = meet(q, v[k]);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, out) in d.iter_mut().enumerate() {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let p = v[k];
        let dq = q as f64 - p as f64;
        *out = dq * dq + f[p];
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A walled box of `w × h` cells at 5 cm, the walls one cell thick, origin at 0.
    fn boxed(w: usize, h: usize) -> proto::MapGridResult {
        let mut cells = String::new();
        for y in 0..h {
            for x in 0..w {
                let wall = x == 0 || y == 0 || x == w - 1 || y == h - 1;
                cells.push(if wall { '#' } else { '.' });
            }
        }
        proto::MapGridResult {
            res_m: 0.05,
            origin: [0.0, 0.0],
            width: w as u32,
            height: h as u32,
            cells,
            ..Default::default()
        }
    }

    #[test]
    fn the_transform_is_exact() {
        // Against brute force on a scatter of blocked cells.
        let (w, h) = (17, 11);
        let blocked = |i: usize| i.is_multiple_of(7) || i % 13 == 5;
        let got = distance_transform(w, h, &blocked);
        for (i, &got) in got.iter().enumerate() {
            let (x, y) = ((i % w) as f64, (i / w) as f64);
            let want = (0..w * h)
                .filter(|&j| blocked(j))
                .map(|j| {
                    let (bx, by) = ((j % w) as f64, (j / w) as f64);
                    (x - bx).powi(2) + (y - by).powi(2)
                })
                .fold(f64::INFINITY, f64::min);
            assert_eq!(got, want, "cell {i}");
        }
    }

    #[test]
    fn clearance_is_the_distance_to_the_nearest_wall() {
        // 2 m × 1.5 m inside the walls.
        let room = Room::from_grid(&boxed(42, 32)).unwrap();
        // 0.3 m in from the left wall's cell centre at x = 0.025, mid-height.
        let c = room.clearance(0.325, 0.775);
        assert!((c - 0.30).abs() < 0.026, "{c}");
        assert_eq!(room.clearance(-1.0, 0.5), 0.0);
        assert_eq!(room.clearance(0.02, 0.5), 0.0);
        let (cx, cy) = room.centre();
        assert!(
            (cx - 1.05).abs() < 0.06 && (cy - 0.8).abs() < 0.06,
            "{cx} {cy}"
        );
    }

    #[test]
    fn unknown_is_a_wall() {
        let mut g = boxed(42, 32);
        // An unknown cell in the middle of the floor.
        let mid = 16 * 42 + 21;
        g.cells.replace_range(mid..mid + 1, "?");
        let room = Room::from_grid(&g).unwrap();
        assert_eq!(room.clearance(21.5 * 0.05, 16.5 * 0.05), 0.0);
    }

    #[test]
    fn nothing_to_stand_on_is_no_room() {
        let g = proto::MapGridResult {
            res_m: 0.05,
            width: 3,
            height: 1,
            cells: "?#?".into(),
            ..Default::default()
        };
        assert!(Room::from_grid(&g).is_none());
        assert!(Room::from_grid(&proto::MapGridResult::default()).is_none());
    }

    #[test]
    fn a_run_stops_short_of_the_wall_by_the_margin() {
        let room = Room::from_grid(&boxed(42, 32)).unwrap();
        // From the middle, facing +x: the right wall's centre is at 2.075, so 30 cm off it is 1.775.
        let run = room.free_run((1.0, 0.775, 0.0), 0.0, 2.0, 0.30);
        assert!((run - 0.775).abs() < 0.05, "{run}");
        // Plenty of room for 60 cm.
        assert_eq!(room.free_run((1.0, 0.775, 0.0), 0.0, 0.6, 0.30), 0.6);
    }

    #[test]
    fn inside_the_margin_it_may_walk_away_but_not_closer() {
        let room = Room::from_grid(&boxed(42, 32)).unwrap();
        // 15 cm off the left wall.
        let pose = (0.175, 0.775, 0.0);
        assert_eq!(room.free_run(pose, 0.0, 0.6, 0.30), 0.6);
        assert!(room.free_run(pose, std::f64::consts::PI, 0.6, 0.30) < 0.03);
    }

    #[test]
    fn a_walk_forward_into_a_wall_turns_the_least_it_can() {
        let room = Room::from_grid(&boxed(42, 32)).unwrap();
        // 50 cm from the right wall, facing it; 60 cm forward does not fit.
        let pose = (1.575, 0.775, 0.0);
        let h = room.heading_for(pose, 0.6, 0.30, (1.2, 0.775), 0.5);
        assert_eq!(h.run, 0.6);
        assert!(!h.homeward);
        let turned = h.yaw.abs();
        assert!(turned > 0.5 && turned < std::f64::consts::PI, "{turned}");
        // Backing up from the same pose needs no turn at all.
        let h = room.heading_for(pose, -0.6, 0.30, (1.2, 0.775), 0.5);
        assert_eq!(h.yaw, 0.0);
    }

    #[test]
    fn far_from_home_the_walk_goes_home() {
        let room = Room::from_grid(&boxed(82, 82)).unwrap();
        // At (3, 1) facing +y, home at (1, 1): a walk forward faces -x, a walk back faces +x.
        let pose = (3.0, 1.0, std::f64::consts::FRAC_PI_2);
        let h = room.heading_for(pose, 0.6, 0.30, (1.0, 1.0), 0.5);
        assert!(h.homeward);
        assert!(
            guard_wrap(h.yaw - std::f64::consts::PI).abs() < 0.01,
            "{}",
            h.yaw
        );
        let h = room.heading_for(pose, -0.6, 0.30, (1.0, 1.0), 0.5);
        assert!(h.yaw.abs() < 0.01, "{}", h.yaw);
    }

    #[test]
    fn a_room_too_small_gives_the_longest_run() {
        // 50 cm square inside: nowhere has 60 cm.
        let room = Room::from_grid(&boxed(12, 12)).unwrap();
        let h = room.heading_for((0.3, 0.3, 0.0), 0.6, 0.10, (0.3, 0.3), 0.5);
        assert!(h.run < 0.6, "{}", h.run);
    }

    #[test]
    fn compose_and_inverse() {
        let a = (1.0, 2.0, 0.7);
        let b = (0.3, -0.2, -1.1);
        let back = compose(inverse(a), compose(a, b));
        assert!((back.0 - b.0).abs() < 1e-12 && (back.1 - b.1).abs() < 1e-12);
        assert!((back.2 - b.2).abs() < 1e-12);
    }

    fn guard_wrap(a: f64) -> f64 {
        super::super::guard::wrap(a)
    }
}
