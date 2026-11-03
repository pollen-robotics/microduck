# Mapping — the onboard map, and where the robot is in it

`mapd` builds a floor plan of the robot's home from its head ToF and its legs, keeps it across
reboots, and knows where the robot is in it — or says it does not. It is the ground the autonomy
work stands on: going somewhere, coming back, finding a place again after a power cycle.

This page owns the mechanism. The algorithm is the `maploc` crate; the service is `mapd`; the
wire is `map.*` (`duck-ipc-proto`, API v49; `map.mode` v50); the switch is `[map] enabled` in `robotd.toml`, **off by
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

**A stop the map already covers adds no scan.** When at least `cover_min_keyframes` keyframes
already stood within `cover_xy_m` of a stop, facing within `cover_yaw_rad`, and saw about as much,
the stop's scan is not kept. With a believed, fully pinned and clean match its pose stays in the
graph — the match and the odometry through it, as for any stop — as a node without a scan; without a
believed match it is dropped and odometry carries on past it. The map's content then grows with the
room seen, not with the time spent in it: 600 stops of laps in the synthetic room keep ~250 scans,
not 600, and end as close to the truth as keeping them all (8.1 cm vs 8.0 cm after eight laps).
Two views are kept, not one: skipping the second stop at a place left the next stop a single scan to
match against, 12 cm off, and refused. A first version chained the next keyframe to a covered stop's
fused pose instead of keeping the node; it fed the graph's own estimate back in as a measurement and
ended 33 cm off after eight laps with whole-graph solves.

*Not yet measured on the ground truth*: the recorded sessions are 4–7 minutes and no stop in them is
covered at the default. With one view enough (`cover_min_keyframes = 1`) three unmatched stops of
session 3 are dropped and a seated carry is then found 16 cm off — over the 15 cm line. A long
session is what this needs.

**A stop costs the same in any map.** Every stop used to re-solve the whole graph, and each match
edge to an old keyframe makes its row of the skyline dense back to it: measured in the synthetic
room, 8 ms a stop at 50 keyframes, 390 ms at 600 (laptop; the board is several times slower). A
stop now re-solves only its `local_optimize_keyframes` newest keyframes, the rest held where they
are; the whole graph is re-solved after a loop closure or a relocalization, every
`full_optimize_every` stops, and the moment the robot goes lost, so every relocalization searches a
fully solved map. A whole-graph solve renumbers its nodes first (reverse Cuthill–McKee): a robot
pacing one room matches old keyframes at nearly every stop, and in walk order that made the skyline
nearly dense — 55 ms an iteration at 380 nodes. With all of it, at 600 stops a stop costs ~4 ms
median, and the rare whole-graph solve ~120 ms (laptop). The
last of these was the one the regression bench asked for: without it, two of the ground-truth
session's carries were no longer recovered. `mapd` logs each stop's time and the map's size
(`stop processed`).

**A direction a scan does not pin gets no weight.** One wall seen through the keyhole fixes the
distance to the wall and says nothing about the position along it; the covariance comes out as
wide as the window in that direction, and the edge's information there is zero. The graph keeps
odometry for it. The first maploc returned a pose anyway — the corner of its search window on a
flat score — at a fixed 5 cm in every direction.

**A match is believed only within odometry's reach.** Measured on the ground-truth session
(`pollen-robotics/SLAM_JJ_Salon_with_GT`): odometry between two stops is good to a few centimetres
and a degree or two, and the whole 4-minute session stays within ~5 cm on odometry alone — while
narrow scans that fit a wall 30–50 cm away scored 95 % on walls and rank 3. A tracking match that
moves the stop further than `track_max_correction_m` / `track_max_correction_rad` from the
prediction is not believed. With the gate the matches are neutral to slightly helpful on that
session (stops within 3.3 cm median, 7.7 cm worst); without it they made the pose 7× worse.

**The weights are honest.** Odometry edges grow with distance walked (separately along and across
the direction of travel) and angle turned; heading is the gyro's and is believed. The first graph
gave every odometry edge a flat 0.10 m / 0.05 rad, which handed one loop closure ~96 % of the yaw
vote: 6.7° of heading error tracked where raw odometry had 0.1°.

**The solver** is Levenberg–Marquardt with step acceptance on a skyline Cholesky, over the nodes a
solve is allowed to move. Match, loop and relocalization edges go through a
Cauchy kernel; an edge that still disagrees after optimization is removed, and any piece of graph
that leaves disconnected becomes its own island (§4).

**Two consecutive stops must agree** before a loop closure or a relocalization is believed. Every
single-window acceptance path the first maploc had eventually false-positived through the keyhole.

## 4. Losing and finding itself

Four things break odometry's chain: being **carried** (the pick-up detector's `picked_up`; the
feet are in the air, but the gyro's heading still holds), a **fall**, a **sit**, and a **reboot**
(a new odometry frame, heading arbitrary). Each leaves the mapper lost with what it still knows:

| cause | hint | search |
|---|---|---|
| carried | the heading | the whole map, ±`carry_yaw_tol_rad` |
| sat — stood up again | the pose it sat at | ±`sat_xy_tol_m`, ±`sat_yaw_tol_rad` |
| fell | the pose, roughly | ±`fall_xy_tol_m`, ±`fall_yaw_tol_rad` |
| boot | where it was switched off | there first, then the whole map at every heading |
| contradiction | the predicted pose | ±1 m |

**A brief pick-up is not a carry.** On the robot the pick-up detector fires as a fall begins —
measured on graphite: "picked up" for 0.36 s, put down, then the limp fall 0.06 s later. Read as a
carry, that threw the position away and searched a thin map everywhere, and the robot never found
itself. Nobody carries a duck anywhere in two seconds, so a pick-up that short is a **bump**: the mapper
keeps tracking — it reports `Bumped`, not lost — and matches the next stop in a window
±`bumped_xy_tol_m` / ±`bumped_yaw_tol_rad` instead of the usual one. The ground-truth session
measured brief pick-ups putting the robot back within ~40 cm — every one held 0.4 s, where every
real carry was held 1.5–2.2 s and moved it 1.2 m or more, so `brief_pickup_s` is 1 s (at 2 s a
quick 1.25 m carry passed for a bump and the mapper tracked from the old place for a minute); going lost over one, as the first cut
of this design did, asked for a relocalization the narrow ToF rarely gets, and the robot stayed lost
for the rest of the session. A nearby search (sat, fell, contradiction) that fails `local_search_keyframes` stops in a
row keeps only the heading and widens to the whole map, ±`fallback_yaw_tol_rad`. Until then, a stop
whose nearby search is refused is searched across the whole map at the hint's heading too, on the
same stop: a robot sat down and carried away otherwise spent its first stops on a hint that was
wrong. Every search is
logged with the gate that refused it (`relocalization search:` in `journalctl -u mapd`).

**A sit is a kidnap marker.** The pick-up detector watches only while the walking and standing
networks drive, so a robot sat down, carried across the room and stood up again reports nothing —
the 2026-10-14 field test kept tracking from where it sat, and drew the new place over the old.
Standing up therefore ends the chain, with the pose it sat at as a tight hint: an unmoved robot is
confirmed there by the usual two agreeing stops, and a moved one fails that nearby search and
widens to the whole map like any other. The first maploc did this; the rewrite had dropped it. A
pick-up while seated, should the detector ever see one, is still a carry.

*Contradiction* is the fourth way: two consecutive stops whose obstacles land on floor the map saw
clear, or nowhere near the walls it predicts. Neither stop touches the map.

Stops made while lost go into an **island** — chained by odometry, not attached to the map. Each new
one is searched for with the island's last `composite_keyframes` stops as one wider scan; when two
searches of the island agree (within `agree_m`, `agree_rad`), the island is joined to the map and
tracking resumes. A stop whose search is refused does not erase the candidate before it — only a
carry does, since the island ends there.

**An ambiguous stop offers its rivals.** A search refused only because a place ≥ 25 cm away fits
nearly as well (`wide_min_margin`) keeps the best and its rivals within the margin that pass every
other gate, as hypotheses. A later stop joins the map when exactly one of its hypotheses agrees
with an earlier stop's; two consistent hypotheses — both stops seeing the same wall, say — keep it
waiting. On the ground-truth session this turned two "ambiguous" refusals of the right answer
into a join 23 s sooner.

**Pinned means pinned in metres and degrees.** A relocalization is believed when its position is
pinned to `reloc_max_sigma_xy_m` and its heading either to `reloc_max_sigma_yaw_rad` by the scan or
by a window already that narrow (`reloc_yaw_prior_rad`): the gyro holds heading through a carry, so
the carry and fallback searches look within ±8°. The first cut counted a direction as pinned only
when its spread was under 40 % of the window — which, on a whole-map search at ±30°, refused every
right answer on the ground-truth session (found 2–8 cm from the truth, "pins 2 of 3 directions"). An island that
is never recognised stays an island: it is kept, and it is not drawn.

`map.status` says which state it is in, and why (`lost`: `boot`, `carried`, `bumped`, `sat`,
`fell`, `contradiction`).

### 4.1 Localizing only

A map built carefully once can be **frozen**: `map.mode` with `localize_only`, which `robotctl
stress` asks for at the start of every lap (it is not kept — `mapd` starts mapping) and hands back
when it stops. Frozen, a stop is matched against the map exactly as for tracking and adds nothing:
the pose is corrected by an information-weighted update of odometry's prediction (the window's σ)
with the match (its information, zero in the directions it does not pin), not by a graph solve.
Nothing is optimized, so the map's poses never move. Lost, stops still chain into an island — a
relocalization needs the composite scan — and the island is dropped when the robot is found or
when a new one starts. Islands present when it is turned on are dropped; what is saved is the
frozen map, with the boot hint kept current.

**After a relocalization the window stays wide for three believed stops** (`track_max_xy_m`,
`track_max_yaw_rad`, the correction gate likewise). A mapping robot's graph goes on refining a
relocalized pose; a frozen map cannot, and in the simulator the relocalization landed 5° off at
the place it was found — which the first stop after, at the same place, agreed with — and 8° off
what the rest of the room said, so every later match was refused by the 3° gate and the robot
ended a lap 33 cm off. With the wide window it ends the lap 8 cm and 0.6° off, as close as mapping
does (`cargo test -p maploc --test sim localizing_only`).

With nothing mapped there is nothing to localize in: the switch answers that it is still mapping.

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
| `map.mode` | localize only in a frozen map, or map (§4.1); not kept across a restart; not routed over WebRTC |

Routed over WebRTC by `mediad`; refused over BLE. `robotctl map` (status), `robotctl map show`
(the grid in the terminal, `--pgm` for an image), `robotctl map wipe`, `robotctl map mode localize|map`, and `robotctl map load
FILE` — another robot's `map.json` in place of this one's (`mapd` stopped, the old map kept beside
it, the builder's last pose cleared so the boot search covers the whole map). That is how
`robotctl stress` puts one test room's map on every robot it tests.

**`mapd` sends nothing to `robotd`.** It does not move the head or the robot; it maps whatever the
robot does. That keeps authority where `architecture.md` §6 puts it.

## 7. Benches

- `cargo test -p maploc` — the synthetic world (`maploc::sim`): rooms of prisms with ground truth,
  odometry that drifts (stride scale, slip, gyro bias and scale), the head sweeping during stops.
  Two laps of a furnished room under harsh drift end 9 cm off where odometry is 27 cm off; carried,
  rebooted-in-place and moved-while-off robots relocalize within three stops (1–4 cm, under 1°,
  without drift), one sat down and stood up in place within three, one carried while seated within
  six; a 15 cm box is mapped and a table top is not.
- `cargo run -p maploc --release --example replay -- <session>` — a recorded `microduck_vslam`
  session (`state.jsonl` + `tof.jsonl`) through the mapper, with the map written as an image.
  `--export run.json` also writes what it decided, for scoring against ground truth outside this
  repository: each stop's odometry pose, live pose and final optimized pose, the tracked pose at
  10 Hz with why it was lost, every event, and the grid (`--res`, one character a cell).
- `cargo test -p mapd` — the service end to end, minus its two upstream sockets.
- **The regression gate**: `maploc_truth`'s `scripts/bench.py` replays this checkout's `maploc` on
  every ground-truth session and its kidnap suite and fails if anything got worse than the saved
  baseline — never a wrong relocalization, never a recovery lost, accuracy only within the replay's
  noise. Run it before merging any change to `maploc`; move the baseline only for a stated trade-off.

**Kidnaps from one recording.** `maploc_truth/scripts/kidnap.py` cuts the ground-truth session at
t1 and resumes it at t2, odometry's position held across the gap (heading carried on, as the gyro
would) and the gap flagged as a carry or a sit — a relocalization test whose answer the camera
knows, against a map built only from before t1. Of 12 such kidnaps, 8 are solvable (≥ 60 % of
what the robot sees after t2 was mapped before t1); the mapper finds itself in 6 of those, 3–7 cm
from the truth, a median 31 s after standing, and never in a wrong place.

**Ground truth.** `pollen-robotics/SLAM_JJ_Salon_with_GT` (private) is a 4-minute session in a
scanned room with the robot's true pose from a fixed camera, the room's true occupancy, and a
scorer (`maploc_truth`'s `mtruth evaluate` on a `replay --export`). It settled the sensor model:
with the true pose, ToF returns land on the room's obstacles 58 % of the time within 5 cm as first
configured and 81 % with **axial** range, a **60 ms** latency and the sensor **3° further down**
than the head's kinematics say (`pitch_offset_rad`) — measured on one alpha, so a per-robot number
once more are measured. Status 0 lands 25 cm off and stays excluded; 9 and 10 are as good as 5.
The simulator models the same sensor. Odometry noise is still the placeholder `OdometryNoise`;
stop to stop the real robot measured a few centimetres and ~1°, mostly from turning in place.

## 8. Not yet

- **Floor under an overhang is not walkable.** The ToF's low rows pass under a bench or a sofa on
  legs and see clear floor that the duck cannot reach; the ground-truth session counts 58 % of the
  room's obstacle cells as "free" in the map, almost all of them under furniture. Cutting a low
  row's free run at the nearest hit in its column fixes the map (obstacles called free 58 → 46 %,
  recall 67 → 76 %) but takes "saw through here" evidence from the matcher (a bump's heading 1.4°
  → 4.9° in the simulator). It belongs in the drawn grid, not in the keyframes the matcher reads.

- **A deliberate head sweep** at each stop, asked of `robotd` as a behaviour rather than commanded
  by `mapd` — wider stops are better matches and faster relocalization.
- **`tofd` configured for range**: integration time, closest-target order and sharpener are left at
  ST's defaults, and σ/signal are compiled out; the sensor was arguably never asked for good data.
- **Camera place fingerprints** — a few frames per stop, a small descriptor on the idle NPU, to
  propose where a wide search should look and veto aliased candidates. Proposing, never measuring.
- **Planning** on the grid (`go there`) — a client of `map.grid` and `map.status`, in the autonomy
  layer, not here.
