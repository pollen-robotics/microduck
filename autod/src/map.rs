//! `mapd`'s map, as the brain uses it: where the odometry frame sits in the map, and the grid.
//!
//! Pure, like `brain.rs`: `main.rs` reads `map.stream` and `map.grid` and hands the results here.
//! `docs/design/autonomous.md` §6 is the design; `mapping-design.md` owns the map itself.
//!
//! **The brain keeps walking on odometry.** It arrives at 50 Hz with every state frame, and it is
//! smooth; the map pose arrives at 4 Hz and jumps when a stop is matched or a loop closes. So the
//! map is used as an *offset*: where the odometry frame sits in the map, from the map pose and the
//! odometry pose of the same instant. A goal is picked in the map and converted to odometry every
//! tick, so a correction moves the goal rather than the duck's idea of where it is.

use duck_ipc_proto as proto;

use crate::brain::wrap_angle;

/// A rigid transform of the plane, `[x, y, yaw]`: rotate by `yaw`, then translate.
pub type Se2 = [f64; 3];

pub const IDENTITY: Se2 = [0.0; 3];

/// `t` applied to a point.
pub fn apply(t: Se2, p: [f64; 2]) -> [f64; 2] {
    let (s, c) = t[2].sin_cos();
    [t[0] + c * p[0] - s * p[1], t[1] + s * p[0] + c * p[1]]
}

/// A pose `[x, y, yaw]` carried by `t`.
pub fn apply_pose(t: Se2, pose: [f64; 3]) -> [f64; 3] {
    let [x, y] = apply(t, [pose[0], pose[1]]);
    [x, y, wrap_angle(pose[2] + t[2])]
}

/// Where the odometry frame sits in the map: the `t` with `t ∘ odom = map`, from two poses of the
/// same instant.
pub fn offset(map: [f64; 3], odom: [f64; 3]) -> Se2 {
    let yaw = wrap_angle(map[2] - odom[2]);
    let (s, c) = yaw.sin_cos();
    [
        map[0] - (c * odom[0] - s * odom[1]),
        map[1] - (s * odom[0] + c * odom[1]),
        yaw,
    ]
}

/// What a map cell says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    /// Never seen, or off the grid.
    Unknown,
    /// Floor seen clear.
    Free,
    /// An obstacle.
    Wall,
}

/// `map.grid`'s answer, as a lookup.
#[derive(Debug, Clone)]
pub struct Grid {
    res: f64,
    origin: [f64; 2],
    width: usize,
    height: usize,
    cells: Vec<u8>,
}

impl Grid {
    /// `None` for a grid with nothing in it, or one whose cells do not add up.
    pub fn of(g: &proto::MapGridResult) -> Option<Self> {
        let (width, height) = (g.width as usize, g.height as usize);
        let cells = g.cells.as_bytes().to_vec();
        (g.res_m > 0.0 && width > 0 && height > 0 && cells.len() == width * height).then_some(
            Self {
                res: g.res_m,
                origin: g.origin,
                width,
                height,
                cells,
            },
        )
    }

    /// The cell holding map point `p`.
    #[cfg(test)]
    pub fn at(&self, p: [f64; 2]) -> Cell {
        let i = ((p[0] - self.origin[0]) / self.res).floor();
        let j = ((p[1] - self.origin[1]) / self.res).floor();
        if i < 0.0 || j < 0.0 || i >= self.width as f64 || j >= self.height as f64 {
            return Cell::Unknown;
        }
        match self.cells[j as usize * self.width + i as usize] {
            b'.' => Cell::Free,
            b'#' => Cell::Wall,
            _ => Cell::Unknown,
        }
    }

    /// Every cell touching the disc of radius `r` around `p`, as what it says. Cells off the grid
    /// are unknown: never seen.
    fn around(&self, p: [f64; 2], r: f64) -> impl Iterator<Item = Cell> + '_ {
        let index = |v: f64, o: f64| ((v - o) / self.res).floor() as i64;
        let (i0, i1) = (
            index(p[0] - r, self.origin[0]),
            index(p[0] + r, self.origin[0]),
        );
        let (j0, j1) = (
            index(p[1] - r, self.origin[1]),
            index(p[1] + r, self.origin[1]),
        );
        (j0..=j1).flat_map(move |j| {
            (i0..=i1).filter_map(move |i| {
                // The nearest point of the cell to `p`.
                let lo = [
                    self.origin[0] + i as f64 * self.res,
                    self.origin[1] + j as f64 * self.res,
                ];
                let nx = p[0].clamp(lo[0], lo[0] + self.res);
                let ny = p[1].clamp(lo[1], lo[1] + self.res);
                ((nx - p[0]).hypot(ny - p[1]) <= r).then(|| {
                    if i < 0 || j < 0 || i >= self.width as i64 || j >= self.height as i64 {
                        Cell::Unknown
                    } else {
                        match self.cells[j as usize * self.width + i as usize] {
                            b'.' => Cell::Free,
                            b'#' => Cell::Wall,
                            _ => Cell::Unknown,
                        }
                    }
                })
            })
        })
    }

    /// Whether any cell within `r` of `p` is a wall.
    pub fn wall_near(&self, p: [f64; 2], r: f64) -> bool {
        self.around(p, r).any(|c| c == Cell::Wall)
    }

    /// The share of cells within `r` of `p` never seen: how much there is to discover there.
    pub fn unknown_near(&self, p: [f64; 2], r: f64) -> f64 {
        let (mut unknown, mut all) = (0u32, 0u32);
        for cell in self.around(p, r) {
            all += 1;
            unknown += u32::from(cell == Cell::Unknown);
        }
        if all == 0 {
            0.0
        } else {
            f64::from(unknown) / f64::from(all)
        }
    }

    /// Whether the map has seen anything at all — an empty one says nothing about where to go.
    pub fn has_floor(&self) -> bool {
        self.cells.contains(&b'.')
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    /// A grid from rows of text, the lowest y first, `res` metres a cell, its corner at the origin.
    pub fn grid(rows: &[&str], res: f64) -> Grid {
        let width = rows[0].len();
        Grid {
            res,
            origin: [0.0, 0.0],
            width,
            height: rows.len(),
            cells: rows.concat().into_bytes(),
        }
    }

    fn inverse(t: Se2) -> Se2 {
        let (s, c) = t[2].sin_cos();
        [-(c * t[0] + s * t[1]), s * t[0] - c * t[1], -t[2]]
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        (a[0] - b[0]).abs() < 1e-9
            && (a[1] - b[1]).abs() < 1e-9
            && wrap_angle(a[2] - b[2]).abs() < 1e-9
    }

    /// The offset carries the odometry pose onto the map pose it was measured with, and carries
    /// every other odometry point the same rigid way.
    #[test]
    fn the_offset_maps_odometry_onto_the_map() {
        let odom = [1.0, 2.0, 0.3];
        let map = [-0.5, 4.0, 2.0];
        let t = offset(map, odom);
        assert!(close(apply_pose(t, odom), map));
        let back = inverse(t);
        assert!(close(apply_pose(back, map), odom));
        // A point a metre ahead in odometry is a metre ahead in the map.
        let ahead = apply(t, [odom[0] + odom[2].cos(), odom[1] + odom[2].sin()]);
        assert!((ahead[0] - (map[0] + map[2].cos())).abs() < 1e-9);
        assert!((ahead[1] - (map[1] + map[2].sin())).abs() < 1e-9);
    }

    #[test]
    fn cells_are_read_by_map_position() {
        let g = grid(&["..#", "?.."], 0.5);
        assert_eq!(g.at([0.1, 0.1]), Cell::Free);
        assert_eq!(g.at([1.2, 0.1]), Cell::Wall);
        assert_eq!(g.at([0.1, 0.6]), Cell::Unknown);
        assert_eq!(g.at([-0.1, 0.1]), Cell::Unknown, "off the grid");
        assert!(g.wall_near([0.8, 0.2], 0.3));
        assert!(!g.wall_near([0.2, 0.7], 0.3));
        assert!(g.unknown_near([0.1, 0.9], 0.3) > 0.0);
        assert!(g.has_floor());
    }

    #[test]
    fn a_grid_that_does_not_add_up_is_refused() {
        let bad = proto::MapGridResult {
            res_m: 0.05,
            width: 3,
            height: 2,
            cells: "...".into(),
            ..Default::default()
        };
        assert!(Grid::of(&bad).is_none());
    }
}
