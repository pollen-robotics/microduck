//! 2D log-odds occupancy grid, fixed-point.
//!
//! Cells store log-odds × `LOG_SCALE` as `i16` in `[LO_MIN, LO_MAX]`.
//! That keeps the per-cell footprint at 2 bytes and the inner update
//! loop fully integer — so it's cheap on the Pi Zero 2W's A53 cores
//! and avoids any FP vs FP-fastmath subtlety in the cell math.
//!
//! Float ops only show up at the world↔index boundary (where the input
//! is f32 anyway), and inside `cast_ray` (where we step along a unit
//! vector in world coordinates).

use std::fs::File;
use std::io::{self, Read, Write};
use std::path::Path;
use std::sync::Arc;

/// Fixed-point scale: stored value = round(log_odds * LOG_SCALE).
pub const LOG_SCALE: f32 = 100.0;
/// Log-odds clamps. ±4 → posterior probability ≈ [0.018, 0.982].
pub const LO_MIN: i16 = -400;
pub const LO_MAX: i16 = 400;
/// Per-update increments (matches Python `lo_hit = +0.85`, `lo_miss = -0.40`).
pub const LO_HIT: i16 = 85;
pub const LO_MISS: i16 = -40;
/// A cell counts as occupied for ray-casting / display when log-odds > 0.
pub const OCC_THRESHOLD: i16 = 0;

const SAVE_MAGIC: &[u8; 4] = b"MDLM";
const SAVE_VERSION: u32 = 1;

/// Geometry of an occupancy grid in world coordinates.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct GridConfig {
    pub x_range: (f32, f32),
    pub y_range: (f32, f32),
    pub cell: f32,
}

impl Default for GridConfig {
    fn default() -> Self {
        // Matches the Python sim defaults — sized for the 8×6 m
        // apartment with margin on all sides.
        Self {
            x_range: (-4.5, 4.5),
            y_range: (-3.5, 3.5),
            cell: 0.05,
        }
    }
}

/// 2D log-odds occupancy grid. Row-major, `[i, j]` indexed by
/// `(row=y_index, col=x_index)`.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct OccupancyGrid {
    cfg: GridConfig,
    w: usize,
    h: usize,
    log: Vec<i16>,
    /// Cached distance-to-nearest-obstacle field in metres, lazy.
    /// Re-computed by `distance_field()` after a map mutation flips the
    /// dirty flag. Stored behind an `Arc` so consumers that need to keep
    /// the field across further `&mut` access (MCL, relocalize) can
    /// clone the handle instead of copying 100–200 KB per call.
    #[serde(skip, default)]
    field: Option<Arc<Vec<f32>>>,
    #[serde(skip, default)]
    field_threshold_fp: i16,
    #[serde(skip, default = "default_dirty")]
    field_dirty: bool,
}

fn default_dirty() -> bool {
    true
}

impl OccupancyGrid {
    pub fn new(cfg: GridConfig) -> Self {
        let w = ((cfg.x_range.1 - cfg.x_range.0) / cfg.cell).ceil() as usize;
        let h = ((cfg.y_range.1 - cfg.y_range.0) / cfg.cell).ceil() as usize;
        Self {
            cfg,
            w,
            h,
            log: vec![0; w * h],
            field: None,
            field_threshold_fp: 0,
            field_dirty: true,
        }
    }

    pub fn cfg(&self) -> &GridConfig {
        &self.cfg
    }
    pub fn width(&self) -> usize {
        self.w
    }
    pub fn height(&self) -> usize {
        self.h
    }
    pub fn cell(&self) -> f32 {
        self.cfg.cell
    }
    pub fn log_raw(&self) -> &[i16] {
        &self.log
    }

    /// World coordinate → signed cell index `(i, j)`, floor-based.
    ///
    /// `floor()` (not `as i32` truncation) so points *below* the grid's
    /// lower edges map to negative indices and fail bounds checks,
    /// instead of aliasing into row/column 0. Non-finite inputs return
    /// `(i32::MIN, i32::MIN)`, which fails every bounds check.
    #[inline]
    pub fn world_to_cell(&self, x: f32, y: f32) -> (i32, i32) {
        if !x.is_finite() || !y.is_finite() {
            return (i32::MIN, i32::MIN);
        }
        let j = ((x - self.cfg.x_range.0) / self.cfg.cell).floor();
        let i = ((y - self.cfg.y_range.0) / self.cfg.cell).floor();
        // Clamp to i32 range before the cast (a huge world coordinate
        // would otherwise saturate to i32::MAX and wrap in later math).
        (
            i.clamp(i32::MIN as f32, i32::MAX as f32) as i32,
            j.clamp(i32::MIN as f32, i32::MAX as f32) as i32,
        )
    }

    /// World coordinate → grid index. Returns `None` if out of bounds.
    #[inline]
    pub fn world_to_idx(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let (i, j) = self.world_to_cell(x, y);
        if i < 0 || j < 0 || (i as usize) >= self.h || (j as usize) >= self.w {
            None
        } else {
            Some((i as usize, j as usize))
        }
    }

    #[inline]
    pub fn in_bounds(&self, i: i32, j: i32) -> bool {
        i >= 0 && j >= 0 && (i as usize) < self.h && (j as usize) < self.w
    }

    #[inline]
    pub fn log_at(&self, i: usize, j: usize) -> i16 {
        self.log[i * self.w + j]
    }

    /// True iff the fixed-point log-odds at (i, j) exceed `threshold_fp`.
    /// Lets callers ask "is this a *confident* wall?" without changing
    /// global constants. The renderer / distance-field passes a value
    /// here; the default `OCC_THRESHOLD` keeps backwards compatibility.
    #[inline]
    pub fn is_above_threshold(&self, i: usize, j: usize, threshold_fp: i16) -> bool {
        self.log[i * self.w + j] > threshold_fp
    }

    /// True if this cell has been explicitly observed as free at least once.
    /// Useful for seeding MCL particles in known-free space.
    #[inline]
    pub fn is_known_free(&self, i: usize, j: usize) -> bool {
        self.log[i * self.w + j] < -50 // ≈ 0.5 log-odds below 0
    }

    #[inline]
    pub fn is_occupied(&self, i: usize, j: usize) -> bool {
        self.log[i * self.w + j] > OCC_THRESHOLD
    }

    /// Apply one log-odds increment to a cell, clamped to `[LO_MIN, LO_MAX]`.
    #[inline]
    fn bump(&mut self, i: usize, j: usize, delta: i16) {
        let idx = i * self.w + j;
        let v = self.log[idx] as i32 + delta as i32;
        self.log[idx] = v.clamp(LO_MIN as i32, LO_MAX as i32) as i16;
    }

    /// Add a log-odds delta at the cell containing `(x, y)`. Used by
    /// `global_render` to merge submap grids into a composite map: each
    /// submap's per-cell log-odds is added to the global cell at the
    /// world coordinate that cell maps to via the submap's anchor pose.
    /// Out-of-bounds writes are silently ignored. Sum is clamped to
    /// `[LO_MIN, LO_MAX]`.
    pub fn add_log_odds_at_world(&mut self, x: f32, y: f32, delta: i16) {
        if let Some((i, j)) = self.world_to_idx(x, y) {
            self.field_dirty = true;
            self.bump(i, j, delta);
        }
    }

    /// Index-addressed variant of [`add_log_odds_at_world`](Self::add_log_odds_at_world)
    /// for callers that already resolved the cell (e.g. the global
    /// renderer's inverse-mapped compositing loop).
    pub fn add_log_odds_at(&mut self, i: usize, j: usize, delta: i16) {
        if i < self.h && j < self.w {
            self.field_dirty = true;
            self.bump(i, j, delta);
        }
    }

    /// Integrate one ray from `(x0,y0)` (sensor origin) to `(x1,y1)` (hit).
    /// Cells along the ray are marked free; the endpoint is marked occupied
    /// iff `hit_is_occupied` (e.g. ray hit a wall, not the floor).
    /// Returns whether the ray wrote at least one cell — a caller deciding
    /// whether a scan is worth remembering must not count rays the grid
    /// silently clipped away.
    pub fn integrate_ray(
        &mut self,
        x0: f32,
        y0: f32,
        x1: f32,
        y1: f32,
        hit_is_occupied: bool,
    ) -> bool {
        // A non-finite coordinate would alias to a garbage cell (NaN
        // casts to 0) and silently paint phantom walls — skip the ray.
        if !(x0.is_finite() && y0.is_finite() && x1.is_finite() && y1.is_finite()) {
            return false;
        }
        self.field_dirty = true;
        let (i0, j0) = self.world_to_cell(x0, y0);
        let (i1, j1) = self.world_to_cell(x1, y1);
        let mut touched = false;
        bresenham(i0, j0, i1, j1, |i, j, at_end| {
            if !self.in_bounds(i, j) {
                return;
            }
            touched = true;
            let (i, j) = (i as usize, j as usize);
            let delta = if at_end && hit_is_occupied {
                LO_HIT
            } else {
                LO_MISS
            };
            self.bump(i, j, delta);
        });
        touched
    }

    /// Cast a ray in the grid; return distance until the first occupied
    /// cell or the bound, capped at `max_range`. Used by the localizer to
    /// predict expected ToF returns from each particle pose.
    ///
    /// Fixed-step march with `cell/2` step. Faster than a DDA for small
    /// grids and trivially vectorizable later.
    pub fn cast_ray(&self, x: f32, y: f32, theta: f32, max_range: f32) -> f32 {
        let cx = theta.cos();
        let cy = theta.sin();
        let cell = self.cfg.cell;
        let step = cell * 0.5;
        let n_steps = (max_range / step) as i32;
        for k in 1..=n_steps {
            let r = (k as f32) * step;
            let xi = x + cx * r;
            let yi = y + cy * r;
            let (i, j) = self.world_to_cell(xi, yi);
            if !self.in_bounds(i, j) {
                return r;
            }
            if self.log[(i as usize) * self.w + (j as usize)] > OCC_THRESHOLD {
                return r;
            }
        }
        max_range
    }

    // ── Distance field (likelihood-field measurement model) ───────────

    /// Distance (in metres) from each grid cell to the nearest cell
    /// whose log-odds exceed `occ_threshold_fp`. Lazy + cached: the
    /// dirty flag is flipped on `integrate_ray`, so the first call after
    /// any map update recomputes and subsequent calls are O(1) clone-free.
    ///
    /// Algorithm: Felzenszwalb 2D Euclidean distance transform (separable
    /// 1D parabola-envelope sweep applied row-wise then column-wise) —
    /// O(W*H), no approximation. ~5 ms on the apartment grid in Rust;
    /// drives the global scan-match in `Localizer::global_relocalize_field`.
    pub fn distance_field(&mut self, occ_threshold_fp: i16) -> &[f32] {
        self.distance_field_shared(occ_threshold_fp);
        // Safe: distance_field_shared always populates `self.field`.
        self.field.as_ref().expect("field populated").as_slice()
    }

    /// Like [`distance_field`](Self::distance_field) but returns a cheap
    /// `Arc` clone of the cached field, so the caller can keep reading it
    /// while the grid is borrowed elsewhere — no 100–200 KB `to_vec()`
    /// copies on the MCL / relocalize hot paths.
    pub fn distance_field_shared(&mut self, occ_threshold_fp: i16) -> Arc<Vec<f32>> {
        let needs_recompute =
            self.field_dirty || self.field.is_none() || self.field_threshold_fp != occ_threshold_fp;
        if needs_recompute {
            self.recompute_distance_field(occ_threshold_fp);
        }
        self.field.as_ref().expect("field populated").clone()
    }

    fn recompute_distance_field(&mut self, occ_threshold_fp: i16) {
        let n = self.w * self.h;
        // Initialize: 0 at obstacle, +∞ elsewhere (squared-distance space).
        let mut buf = vec![f32::INFINITY; n];
        let mut any_occ = false;
        for idx in 0..n {
            if self.log[idx] > occ_threshold_fp {
                buf[idx] = 0.0;
                any_occ = true;
            }
        }
        let cell = self.cfg.cell;
        if !any_occ {
            // No obstacles yet — saturate to grid-diagonal-ish distance
            // so likelihoods at any pose are uniformly small.
            let cap = self.w.max(self.h) as f32 * cell;
            self.field = Some(Arc::new(vec![cap; n]));
            self.field_threshold_fp = occ_threshold_fp;
            self.field_dirty = false;
            return;
        }
        let w = self.w;
        let h = self.h;
        let cap = w.max(h);
        let mut tmp = vec![0.0_f32; cap];
        let mut line_in = vec![0.0_f32; cap];
        // Envelope scratch shared across all dt_1d calls — 2(W+H) heap
        // allocations per recompute otherwise.
        let mut scratch = DtScratch::with_capacity(cap);
        // Pass 1 — DT over each row (along x).
        for r in 0..h {
            for c in 0..w {
                line_in[c] = buf[r * w + c];
            }
            dt_1d(&line_in[..w], &mut tmp[..w], &mut scratch);
            for c in 0..w {
                buf[r * w + c] = tmp[c];
            }
        }
        // Pass 2 — DT over each column (along y).
        for c in 0..w {
            for r in 0..h {
                line_in[r] = buf[r * w + c];
            }
            dt_1d(&line_in[..h], &mut tmp[..h], &mut scratch);
            for r in 0..h {
                buf[r * w + c] = tmp[r];
            }
        }
        // buf now holds squared cell-distance; convert to metres.
        for v in buf.iter_mut() {
            *v = v.sqrt() * cell;
        }
        self.field = Some(Arc::new(buf));
        self.field_threshold_fp = occ_threshold_fp;
        self.field_dirty = false;
    }

    // ── Persistence ────────────────────────────────────────────────────

    /// Serialize to the `MDLM` binary format into any writer. Same bytes
    /// on disk (`save`) and on the wire (`wire::write_map`), so there's a
    /// single parser everywhere.
    pub fn write_to<W: Write>(&self, f: &mut W) -> io::Result<()> {
        f.write_all(SAVE_MAGIC)?;
        f.write_all(&SAVE_VERSION.to_le_bytes())?;
        f.write_all(&self.cfg.x_range.0.to_le_bytes())?;
        f.write_all(&self.cfg.x_range.1.to_le_bytes())?;
        f.write_all(&self.cfg.y_range.0.to_le_bytes())?;
        f.write_all(&self.cfg.y_range.1.to_le_bytes())?;
        f.write_all(&self.cfg.cell.to_le_bytes())?;
        f.write_all(&(self.w as u32).to_le_bytes())?;
        f.write_all(&(self.h as u32).to_le_bytes())?;
        // log-odds as raw little-endian i16. Endianness is fixed in the
        // file format so cross-arch loads don't get garbled.
        let bytes = unsafe {
            std::slice::from_raw_parts(self.log.as_ptr() as *const u8, self.log.len() * 2)
        };
        if cfg!(target_endian = "little") {
            f.write_all(bytes)?;
        } else {
            for v in &self.log {
                f.write_all(&v.to_le_bytes())?;
            }
        }
        Ok(())
    }

    /// Serialize to an in-memory buffer — for telemetry map sends, so the
    /// grid never round-trips through a temp file on flash.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(37 + self.log.len() * 2);
        self.write_to(&mut buf).expect("write to Vec cannot fail");
        buf
    }

    pub fn save<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut f = File::create(path)?;
        self.write_to(&mut f)
    }

    /// Deserialize from the `MDLM` binary format. Returns `Err` on a bad
    /// magic, unsupported version, an inconsistent geometry header, or a
    /// truncated payload — the caller decides whether to start fresh.
    pub fn read_from<R: Read>(f: &mut R) -> io::Result<Self> {
        let mut magic = [0u8; 4];
        f.read_exact(&mut magic)?;
        if &magic != SAVE_MAGIC {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "bad magic"));
        }
        let mut u32_buf = [0u8; 4];
        f.read_exact(&mut u32_buf)?;
        let version = u32::from_le_bytes(u32_buf);
        if version != SAVE_VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unsupported map version {version}"),
            ));
        }
        let mut f32_buf = [0u8; 4];
        let mut read_f32 = |f: &mut R| -> io::Result<f32> {
            f.read_exact(&mut f32_buf)?;
            Ok(f32::from_le_bytes(f32_buf))
        };
        let x0 = read_f32(f)?;
        let x1 = read_f32(f)?;
        let y0 = read_f32(f)?;
        let y1 = read_f32(f)?;
        let cell = read_f32(f)?;
        f.read_exact(&mut u32_buf)?;
        let w = u32::from_le_bytes(u32_buf) as usize;
        f.read_exact(&mut u32_buf)?;
        let h = u32::from_le_bytes(u32_buf) as usize;

        // Validate the header before trusting w*h for an allocation: the
        // dimensions must match the world ranges (same formula as `new`),
        // and stay under a sanity cap so a corrupt/truncated file can't
        // trigger a multi-GB allocation.
        const MAX_CELLS: usize = 64 * 1024 * 1024; // 128 MB of i16 — far above any real map
        let bad = !(x1 > x0
            && y1 > y0
            && cell > 0.0
            && cell.is_finite()
            && x0.is_finite()
            && y0.is_finite())
            || w == 0
            || h == 0
            || w.saturating_mul(h) > MAX_CELLS
            || w != ((x1 - x0) / cell).ceil() as usize
            || h != ((y1 - y0) / cell).ceil() as usize;
        if bad {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "inconsistent map header: {w}x{h}, \
                         x=[{x0},{x1}] y=[{y0},{y1}] cell={cell}"
                ),
            ));
        }

        let cfg = GridConfig {
            x_range: (x0, x1),
            y_range: (y0, y1),
            cell,
        };
        let mut log = vec![0i16; w * h];
        // Same little-endian-on-disk convention as save().
        let bytes =
            unsafe { std::slice::from_raw_parts_mut(log.as_mut_ptr() as *mut u8, log.len() * 2) };
        f.read_exact(bytes)?;
        if cfg!(target_endian = "big") {
            for v in log.iter_mut() {
                *v = i16::from_le_bytes(v.to_ne_bytes());
            }
        }
        Ok(Self {
            cfg,
            w,
            h,
            log,
            field: None,
            field_threshold_fp: 0,
            field_dirty: true,
        })
    }

    /// Load a previously saved map. `Err` on any corrupt/mismatched file
    /// (see [`read_from`](Self::read_from)); the `Option` wrapper is kept
    /// for callers that treat "no file" and "fresh map" alike.
    pub fn load<P: AsRef<Path>>(path: P) -> io::Result<Option<Self>> {
        let mut f = File::open(path)?;
        Ok(Some(Self::read_from(&mut f)?))
    }
}

// ── 1D Euclidean distance transform (Felzenszwalb & Huttenlocher 2004) ──

/// Reusable envelope buffers for `dt_1d` — hoisted out so a full 2D
/// recompute does 3 allocations instead of 2·(W+H).
struct DtScratch {
    v: Vec<usize>,
    z: Vec<f32>,
}

impl DtScratch {
    fn with_capacity(n: usize) -> Self {
        Self {
            v: Vec::with_capacity(n),
            z: Vec::with_capacity(n + 1),
        }
    }
}

/// Lower-envelope sweep over the parabolas `y_q(x) = (x - q)^2 + f[q]`.
/// `f` carries 0 at "target" cells and +∞ elsewhere; result is the
/// squared distance to the nearest target along the 1D axis.
///
/// Skips non-finite samples explicitly — naive Felzenszwalb computes
/// `f[q] - f[v[k]]` which is NaN when both are +∞ and silently breaks
/// the envelope. Real implementations handle the indicator-function
/// case by treating +∞ samples as "no parabola here".
fn dt_1d(f: &[f32], d: &mut [f32], scratch: &mut DtScratch) {
    let n = f.len();
    debug_assert!(d.len() >= n);
    if n == 0 {
        return;
    }

    // Lower-envelope state: parallel arrays of parabola indices `v` and
    // their boundary x-values `z`. `z` is always one longer than `v` —
    // entries z[0..v.len()] are left boundaries, z[v.len()] is +∞.
    let v = &mut scratch.v;
    let z = &mut scratch.z;
    v.clear();
    z.clear();

    for q in 0..n {
        if !f[q].is_finite() {
            continue;
        }
        // While this parabola dominates the current top-of-envelope from
        // before its left boundary, pop the latter.
        loop {
            match v.last() {
                Some(&last_q) => {
                    let s = compute_intersection(f, q, last_q);
                    let last_z = *z.last().expect("z aligned with v");
                    if s <= last_z {
                        v.pop();
                        z.pop();
                    } else {
                        z.push(s);
                        break;
                    }
                }
                None => {
                    z.push(f32::NEG_INFINITY);
                    break;
                }
            }
        }
        v.push(q);
    }
    z.push(f32::INFINITY);

    if v.is_empty() {
        // No finite samples on this line — distances stay +∞.
        for slot in d.iter_mut().take(n) {
            *slot = f32::INFINITY;
        }
        return;
    }
    let mut k = 0_usize;
    for q in 0..n {
        while z[k + 1] < q as f32 {
            k += 1;
        }
        let dq = (q as f32) - (v[k] as f32);
        d[q] = dq * dq + f[v[k]];
    }
}

#[inline]
fn compute_intersection(f: &[f32], q: usize, vk: usize) -> f32 {
    let qf = q as f32;
    let vkf = vk as f32;
    ((f[q] + qf * qf) - (f[vk] + vkf * vkf)) / (2.0 * qf - 2.0 * vkf)
}

// ── Bresenham ─────────────────────────────────────────────────────────────

/// Walk integer cells along the line `(i0,j0)..(i1,j1)`; calls `visit(i, j,
/// at_endpoint)` at each cell, including both endpoints.
pub fn bresenham<F: FnMut(i32, i32, bool)>(i0: i32, j0: i32, i1: i32, j1: i32, mut visit: F) {
    let di = (i1 - i0).abs();
    let dj = (j1 - j0).abs();
    let si = if i0 < i1 { 1 } else { -1 };
    let sj = if j0 < j1 { 1 } else { -1 };
    let (mut i, mut j) = (i0, j0);
    let mut err = di - dj;
    loop {
        let at_end = i == i1 && j == j1;
        visit(i, j, at_end);
        if at_end {
            return;
        }
        let e2 = 2 * err;
        if e2 > -dj {
            err -= dj;
            i += si;
        }
        if e2 < di {
            err += di;
            j += sj;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_field_matches_hand_computed() {
        // 5×5 grid, single obstacle at (2, 2). Distances should be the
        // Euclidean distance from each cell to that obstacle, in metres.
        let mut g = OccupancyGrid::new(GridConfig {
            x_range: (0.0, 0.5),
            y_range: (0.0, 0.5),
            cell: 0.10,
        });
        // Hammer (2, 2) until log-odds saturate above any threshold.
        for _ in 0..20 {
            // Cast a ray that ends exactly at cell (2, 2) — j=2 → x=0.25,
            // i=2 → y=0.25, with both contype/floor checks bypassed via
            // the integrate_ray hit_is_occupied=true path.
            g.integrate_ray(0.0, 0.0, 0.25, 0.25, true);
        }
        let field = g.distance_field(150 /* = 1.5 in fixed-point */).to_vec();
        let cell = g.cell();
        // Spot-check a few cells.
        for &(i, j, expected_cells) in &[
            (2_usize, 2_usize, 0.0_f32),
            (2, 3, 1.0),
            (2, 0, 2.0),
            (0, 0, (2.0_f32 * 2.0 + 2.0 * 2.0).sqrt()),
            (4, 4, (2.0_f32 * 2.0 + 2.0 * 2.0).sqrt()),
        ] {
            let got = field[i * g.width() + j];
            let want = expected_cells * cell;
            assert!(
                (got - want).abs() < 1e-4,
                "cell ({i},{j}): expected {want:.3} m, got {got:.3} m"
            );
        }
        // Cache hit: second call must be a no-op (no panic, same values).
        let field2 = g.distance_field(150);
        assert_eq!(field2.len(), 25);
    }

    #[test]
    fn distance_field_invalidates_on_mutation() {
        let mut g = OccupancyGrid::new(GridConfig {
            x_range: (0.0, 0.5),
            y_range: (0.0, 0.5),
            cell: 0.10,
        });
        for _ in 0..20 {
            g.integrate_ray(0.0, 0.0, 0.25, 0.25, true);
        }
        let d_before = g.distance_field(150)[0]; // distance from (0,0)
        // Add a much closer obstacle and confirm the cache rebuilt.
        for _ in 0..20 {
            g.integrate_ray(0.4, 0.4, 0.05, 0.05, true);
        }
        let d_after = g.distance_field(150)[0];
        assert!(
            d_after < d_before,
            "cache must invalidate after integrate_ray: before={d_before}, after={d_after}"
        );
    }

    #[test]
    fn world_to_idx_round_trip() {
        let g = OccupancyGrid::new(GridConfig::default());
        let (i, j) = g.world_to_idx(0.0, 0.0).unwrap();
        // Centre of an apartment with x in [-3, 3.5] and 5 cm cells.
        assert!(j > 0 && i > 0);
        assert!(g.world_to_idx(-100.0, 0.0).is_none());
    }

    /// Truncation regression: a point in the one-cell band just OUTSIDE
    /// the lower edges used to alias onto row/column 0 (`as i32`
    /// truncates toward zero). It must be out of bounds.
    #[test]
    fn just_below_lower_edge_is_out_of_bounds() {
        let g = OccupancyGrid::new(GridConfig {
            x_range: (-2.0, 2.0),
            y_range: (-2.0, 2.0),
            cell: 0.05,
        });
        assert!(
            g.world_to_idx(-2.03, 0.0).is_none(),
            "x = x_min - 0.03 must not alias into column 0"
        );
        assert!(
            g.world_to_idx(0.0, -2.03).is_none(),
            "y = y_min - 0.03 must not alias into row 0"
        );
        // Just inside is fine.
        assert_eq!(g.world_to_idx(-1.99, -1.99), Some((0, 0)));
        // Non-finite coordinates must not alias into cell 0 either.
        assert!(g.world_to_idx(f32::NAN, 0.0).is_none());
        assert!(g.world_to_idx(0.0, f32::INFINITY).is_none());
    }

    /// A wall integrated just outside the low-x border must not smear
    /// into column 0.
    #[test]
    fn integrate_ray_clips_below_lower_edge() {
        let mut g = OccupancyGrid::new(GridConfig {
            x_range: (-1.0, 1.0),
            y_range: (-1.0, 1.0),
            cell: 0.05,
        });
        for _ in 0..10 {
            g.integrate_ray(-0.5, 0.0, -1.03, 0.0, true); // endpoint outside
        }
        let (i, j) = g.world_to_idx(-0.99, 0.0).unwrap(); // column 0
        assert_eq!(j, 0);
        assert!(
            g.log_at(i, j) <= 0,
            "out-of-map hit smeared into edge column (log={})",
            g.log_at(i, j)
        );
    }

    #[test]
    fn to_bytes_matches_file_save() {
        let mut g = OccupancyGrid::new(GridConfig::default());
        for _ in 0..5 {
            g.integrate_ray(0.0, 0.0, 1.5, 0.7, true);
        }
        let dir = tempdir();
        let path = dir.join("map.bin");
        g.save(&path).unwrap();
        let file_bytes = std::fs::read(&path).unwrap();
        assert_eq!(g.to_bytes(), file_bytes);
        // And read_from round-trips.
        let g2 = OccupancyGrid::read_from(&mut g.to_bytes().as_slice()).unwrap();
        assert_eq!(g.log_raw(), g2.log_raw());
        std::fs::remove_dir_all(dir).ok();
    }

    /// A corrupt header (bogus w×h) must error out instead of allocating
    /// whatever the file claims.
    #[test]
    fn load_rejects_inconsistent_header() {
        let g = OccupancyGrid::new(GridConfig::default());
        let mut bytes = g.to_bytes();
        // Stomp the width field (offset 28: magic 4 + version 4 + 5×f32).
        bytes[28..32].copy_from_slice(&0x00FF_FFFF_u32.to_le_bytes());
        let r = OccupancyGrid::read_from(&mut bytes.as_slice());
        assert!(r.is_err(), "corrupt header must be rejected");
    }

    #[test]
    fn integrate_then_cast_ray_finds_obstacle() {
        let mut g = OccupancyGrid::new(GridConfig::default());
        // Repeatedly hit the same wall at (1.0, 0.0) from origin so the
        // log-odds saturate above the occupancy threshold.
        for _ in 0..10 {
            g.integrate_ray(0.0, 0.0, 1.0, 0.0, true);
        }
        let d = g.cast_ray(0.0, 0.0, 0.0, 4.0);
        assert!(
            (d - 1.0).abs() < g.cell() * 2.0,
            "expected hit near 1.0 m, got {d}"
        );
    }

    #[test]
    fn save_load_round_trip() {
        let mut g = OccupancyGrid::new(GridConfig::default());
        for _ in 0..5 {
            g.integrate_ray(0.0, 0.0, 1.5, 0.7, true);
        }
        let dir = tempdir();
        let path = dir.join("map.bin");
        g.save(&path).unwrap();
        let g2 = OccupancyGrid::load(&path).unwrap().unwrap();
        assert_eq!(g.log_raw(), g2.log_raw());
        assert_eq!(g.width(), g2.width());
        assert_eq!(g.height(), g2.height());
        std::fs::remove_dir_all(dir).ok();
    }

    fn tempdir() -> std::path::PathBuf {
        let mut p = std::env::temp_dir();
        let pid = std::process::id();
        let nano = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        p.push(format!("microduck_maploc_test_{pid}_{nano}"));
        std::fs::create_dir_all(&p).unwrap();
        p
    }
}
