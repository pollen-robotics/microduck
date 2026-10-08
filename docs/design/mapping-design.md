# Mapping — the onboard map, and where the robot is in it

`mapd` builds a floor plan of the robot's home from its head ToF and its legs, keeps it across
reboots, and knows where the robot is in it — or says it does not. It is the ground the autonomy
work stands on: going somewhere, coming back, finding a place again after a power cycle.

This page owns the mechanism. The algorithm is the `maploc` crate; the service is `mapd`; the
wire is `map.*` (`duck-ipc-proto`, API v42); the switch is `[map] enabled` in `robotd.toml`, **off by
default** until its standing cost — parsing the state stream — has been measured on a board. Off,
`mapd` still runs and answers `map.status` with why it is not mapping.

## 1. What the hardware allows

**The ToF is a cane, not a lidar.** 64 zones over 45°, and on real recordings the valid returns
(status 5/9) have a median range of 0.42–0.56 m and a 90th percentile under 1 m; status 0 makes up
21–50 % of zones, mostly in the upper rows. At the resting head pose the neck and head pitches
cancel and the sensor axis is level, about 25 cm above the floor: the lower rows meet the floor
between 0.7 and 5 m, the upper rows look over most furniture.

**Odometry is the backbone.** Contact odometry gives position from the legs and heading from the
IMU's integrated gyro, so heading drift is gyro bias rather than foot slip. Real-robot drift is
**not yet measured** (§7); the twin drifts ~0.3 % per metre.

**Compute is not the limit.** The whole mapper replays a 2.5-minute session in 0.1 s on a laptop;
on the board the steady cost is parsing the state stream. The camera is what is expensive, and v1
does not use it.

**Visual odometry is not an option on this platform** — the `microduck_vslam` work established that
a bobbing biped with no metric depth loses track constantly, and that legs doing odometry with
vision only proposing loop closures is the architecture that works. The camera's place here, when
it comes, is the same: proposing where to look, never measuring motion (§8).

So the design leans on what the duck does have: good heading, the ability to **stop and look
around**, and patience.

## 2. A stop is the unit of mapping

The robot maps only where it stands still. After `settle_s` of stillness (no twist asked for, not
sitting, held, fallen or picked up, odometry not moving) a **keyframe** opens and collects every
depth frame until the robot moves again. Frames are voted cell by cell — a cell is an obstacle when
`min_hits` frames agree and the floor was not seen there more often, each frame voting once per
cell — and the keyframe stores the result in its own frame: obstacle cells and floor-seen-clear
cells, 2.5 cm each.

A keyframe is rigid by construction: every frame in it came from one standing place. Everything
between two keyframes is one odometry edge. The first maploc grouped several stops into a submap
and corrected only at submap granularity; odometry drifts between stops, so those submaps were not
rigid, and their loop-closure witnesses contradicted each other.

The head adds width. `robotd`'s idle behaviour glances ±0.6 rad after two seconds of stillness,
so a stop sees ~120° rather than one 45° wedge; a deliberate sweep (§8) would see more.

### 2.1 Beams are judged in 3D, by height

Each return is placed in 3D — the head's pose at the frame's **capture time**, interpolated from
the state history on the shared `CLOCK_MONOTONIC` (`t_ns`), less `tof_latency_ms` — and then
judged against a height band that is the duck's own business:

| where the return is | what it is | free space it proves |
|---|---|---|
| below `floor_margin_m` (+ `floor_margin_per_m` × range) | floor | the beam's stretch below `free_height_m` |
| above `ceiling_m` | overhead — a table top | none |
| in between | an obstacle | the beam's stretch below `free_height_m`, stopping a cell short |

Two failures of the first design this fixes. Rows passing *over* a low box on their way to a far
wall carved the box's cells free and erased it. And table tops were drawn as walls, so the map said
the duck could not go where it walks every day. Free space is now claimed only where a beam ran low
enough that anything the duck could trip on would have stopped it — less free space per frame, all
of it true. The floor the robot walked on between stops is free too, and the map draws it.

## 3. Placing a stop

1. **Predicted** where odometry says, from the previous keyframe.
2. **Matched** against the map within `local_radius_m`, by exhaustive correlative search inside a
   window three odometry-σ wide since the last match that pinned each direction. The score of
   every pose in the window, read as a likelihood, gives the answer *and* its covariance.
3. **Loop-checked** against parts of the map more than `loop_min_path_m` back along the walk,
   with a wide coarse-to-fine search.
4. **Optimized**, and the map-from-odometry transform taken from the keyframe's new pose.

**A direction a scan does not pin gets no weight.** One wall seen through the keyhole fixes the
distance to the wall and says nothing about the position along it; the covariance comes out as
wide as the window in that direction, and the edge's information there is zero. The graph keeps
odometry for it. The first maploc returned a pose anyway — the corner of its search window on a
flat score — at a fixed 5 cm in every direction.

**The weights are honest.** Odometry edges grow with distance walked (separately along and across
the direction of travel) and angle turned; heading is the gyro's and is believed. The first graph
gave every odometry edge a flat 0.10 m / 0.05 rad, which handed one loop closure ~96 % of the yaw
vote: 6.7° of heading error tracked where raw odometry had 0.1°.

**The solver** is Levenberg–Marquardt with step acceptance on a skyline Cholesky — a chain with a
few long edges, so it stays linear in the walk. Match, loop and relocalization edges go through a
Cauchy kernel; an edge that still disagrees after optimization is removed, and any piece of graph
that leaves disconnected becomes its own island (§4).

**Two consecutive stops must agree** before a loop closure or a relocalization is believed. Every
single-window acceptance path the first maploc had eventually false-positived through the keyhole.

## 4. Losing and finding itself

Three things break odometry's chain: being **carried** (the pick-up detector's `picked_up`; the
feet are in the air, but the gyro's heading still holds), a **fall**, and a **reboot** (a new
odometry frame, heading arbitrary). Each leaves the mapper lost with what it still knows:

| cause | hint | search |
|---|---|---|
| carried | the heading | the whole map, ±`carry_yaw_tol_rad` |
| bumped — picked up for under `brief_pickup_s` | the pose | ±`bumped_xy_tol_m`, ±`bumped_yaw_tol_rad` |
| fell | the pose, roughly | ±`fall_xy_tol_m`, ±`fall_yaw_tol_rad` |
| boot | where it was switched off | there first, then the whole map at every heading |
| contradiction | the predicted pose | ±1 m |

**A brief pick-up is not a carry.** On the robot the pick-up detector fires as a fall begins —
measured on graphite: "picked up" for 0.36 s, put down, then the limp fall 0.06 s later. Read as a
carry, that threw the position away and searched a thin map everywhere, and the robot never found
itself. Nobody carries a duck anywhere in two seconds, so a pick-up that short is searched for
nearby. A nearby search (bumped, fell, contradiction) that fails `local_search_keyframes` stops in a
row keeps only the heading and widens to the whole map, ±`fallback_yaw_tol_rad`. Every search is
logged with the gate that refused it (`relocalization search:` in `journalctl -u mapd`).

*Contradiction* is the fourth way: two consecutive stops whose obstacles land on floor the map saw
clear, or nowhere near the walls it predicts. Neither stop touches the map.

Stops made while lost go into an **island** — chained by odometry, not attached to the map. Each new
one is searched for with the island's last `composite_keyframes` stops as one wider scan; when two
consecutive searches agree, the island is joined to the map and tracking resumes. An island that
is never recognised stays an island: it is kept, and it is not drawn.

`map.status` says which state it is in, and why (`lost`: `boot`, `carried`, `bumped`, `fell`,
`contradiction`).

## 5. What is kept

The map is the keyframes, the pose graph and the islands, in one JSON file at `[map] path`
(`/var/lib/mapd/map.json`, `StateDirectory=mapd` — the robot's, surviving updates and rollbacks).
Saved every `autosave_s` while it changes, when the pose has moved 30 cm since the last save (so the
boot hint is close), and on shutdown. Written atomically.

**A file `mapd` cannot read is set aside, never overwritten**: renamed to `map.json.bad-<seconds>`
and named in the journal, and a new map starts beside it. The first maploc started fresh on any read
error and its next autosave replaced the file — a format bump destroyed hours of mapping silently.

The occupancy grid is **derived**: rendered from the keyframes at their current poses whenever it is
asked for, so it cannot disagree with the graph after a loop closure.

## 6. The service

`mapd` is a client of `robotd` (`robot.subscribe` at `state_hz`) and `tofd` (`tof.stream`), each on
its own thread that reconnects by itself — an update restarting either is ordinary. The mapper runs
on its own thread behind a bounded queue: a relocalization search delays the map, never an answer.
`Nice=10`, and nothing depends on the unit; the robot walks without it.

| method | answers |
|---|---|
| `map.status` | the pose or why it is lost, stops, islands, loops, relocalizations, the file, `unavailable` |
| `map.stream` | the status, then `map.state` notifications at 4 Hz |
| `map.grid` | the occupancy grid at `res_m`, one character per cell (`?` `.` `#`), the stops, the pose |
| `map.wipe` | forgets the map and deletes the file; mutating |

Routed over WebRTC by `mediad`; refused over BLE. `robotctl map` (status), `robotctl map show`
(the grid in the terminal, `--pgm` for an image), `robotctl map wipe`.

**`mapd` sends nothing to `robotd`.** It does not move the head or the robot; it maps whatever the
robot does. That keeps authority where `architecture.md` §6 puts it.

## 7. Benches

- `cargo test -p maploc` — the synthetic world (`maploc::sim`): rooms of prisms with ground truth,
  odometry that drifts (stride scale, slip, gyro bias and scale), the head sweeping during stops.
  Two laps of a furnished room under harsh drift end 9 cm off where odometry is 27 cm off; carried,
  rebooted-in-place and moved-while-off robots relocalize within three stops (1–4 cm, under 1°,
  without drift); a 15 cm box is mapped and a table top is not.
- `cargo run -p maploc --release --example replay -- <session>` — a recorded `microduck_vslam`
  session (`state.jsonl` + `tof.jsonl`) through the mapper, with the map written as an image.
- `cargo test -p mapd` — the service end to end, minus its two upstream sockets.

**What is not measured yet, and decides the defaults:** real odometry drift (along, across, gyro —
the `OdometryNoise` defaults are placeholders), the ToF's real capture latency, whether the sensor
reports radial or axial distance (`range_is_axial`; up to 11 % at the corner zones), and how the
mapper does on recordings made *for* it — stops of 3–5 s with the head moving, returning to a
taped mark. The existing recordings were made while walking and rarely stopping.

## 8. Not yet

- **A deliberate head sweep** at each stop, asked of `robotd` as a behaviour rather than commanded
  by `mapd` — wider stops are better matches and faster relocalization.
- **`tofd` configured for range**: integration time, closest-target order and sharpener are left at
  ST's defaults, and σ/signal are compiled out; the sensor was arguably never asked for good data.
- **Camera place fingerprints** — a few frames per stop, a small descriptor on the idle NPU, to
  propose where a wide search should look and veto aliased candidates. Proposing, never measuring.
- **Planning** on the grid (`go there`) — a client of `map.grid` and `map.status`, in the autonomy
  layer, not here.
