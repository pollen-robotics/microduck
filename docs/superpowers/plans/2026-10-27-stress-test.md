# `robotctl stress` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `robotctl stress` subcommand that runs a fixed lap of walks, provoked falls and
sit/stands until the battery is empty, measures every get-up, and logs it so robots compare lap
for lap.

**Architecture:** Bring `robotctl endurance` over from `battery-endurance-test`, lift its
run-agnostic plumbing (background unit, durable CSV, state feed, sampler, signals) into a shared
module, then build `stress` beside it from three pure, unit-tested pieces — the lap script, the
get-up state machine, the guards — and one driver that ties them to `robotd` and `tofd`.

**Tech Stack:** Rust (`robotctl` crate, `duck-ipc-proto`, `kinematics::tof`), Python
(`scripts/endurance-plot.py`).

**Spec:** `docs/superpowers/specs/2026-10-27-stress-test-design.md`

## Global Constraints

- Branch `stress-test` (from `main`). Commit per task; do not push.
- `stress` is a client of `robotd`: no change to `robotd`, `tofd` or the protocol.
- Fall cut: `robot.enable { on: false, toggle: false }`, then `robot.enable { on: true }` **0.5 s**
  later.
- Get-up timeout **7 s**; failed → policy off **2 s**, on, another **7 s**; failed again →
  `gave_up`, `robot.relax`, stop driving, keep sampling.
- Got up = `safety.fallen` false **held 1 s**; get-up time = enable → start of that hold. Seated
  upright counts; then `sit_toggle` to stand.
- `no_fall` = `fallen` never set within **2 s** of the re-enable.
- Lap `stress-v1`: A ×3 (fwd 2 s, fall; back 2 s, fall), B ×1 (fwd 7 s, left 2 s, back 7 s,
  right 2 s), C ×3 (sit 3 s, stand 2 s). Speeds 0.3 m/s, 1.5 rad/s.
- Guards: square ±0.5 m on odometry; wall < 0.20 m ahead on ToF, forward walking only; a cut
  step's recovery stays inside its slot; the next step starts at its scheduled time.
- Hands off at ≤ 2 % battery, as `endurance`.
- Code style: match `endurance.rs` — module doc arguing the why, `///` on every item, tests in
  `#[cfg(test)] mod tests` at the bottom of each file.
- Gate per task: `cargo fmt --all`, `cargo clippy -p robotctl --all-targets -- -D warnings`,
  `cargo test -p robotctl`.

## Review Focus

1. **No ToF** (`tofd` socket absent, or `TofStreamResult` with no sensor): one `no_tof` event,
   the run goes on on odometry — Task 4 pins `nearest_ahead` on an empty/absent frame; Task 5
   logs it once, never per frame.
2. **`fallen` flapping** during a get-up (true/false/true inside 1 s): not counted as up; the
   hold restarts — Task 3 test `a_relapse_inside_the_hold_restarts_it`.
3. **Battery hands-off during a fall sequence**: the driver must not send `enable` or
   `sit_toggle` after `HandsOff` — Task 5 checks the battery before every cut, re-enable and
   reset, and the get-up loop returns `HandsOff` like every other wait.
4. **A fall nobody provoked** (mid-walk, mid-turn, failed rise): same machine, logged
   `fallen_unprovoked`, step abandoned — Task 3 test `an_unprovoked_fall_uses_the_same_rules`,
   Task 5 checks `fallen` on every tick of every step.
5. **`robotd` restarting mid-run**: state feed and driver reconnect (inherited from `endurance`);
   a frame older than 1 s is no frame, and the get-up clock does not count "no frame" as up —
   Task 3 treats `None` as still down.

---

### Task 1: Bring `robotctl endurance` over onto `main`

**Files:**
- Create: `robotctl/src/endurance.rs`, `scripts/endurance-plot.py` (from `battery-endurance-test`)
- Modify: `robotctl/src/main.rs` (`mod endurance;`, `Namespace::Endurance`, its dispatch),
  `robotctl/src/show.rs` (`civil` → `pub(crate)`), `docs/robot/cheatsheet.md` (its section)

**Interfaces:**
- Produces: `robotctl endurance` exactly as on `battery-endurance-test`.

- [ ] **Step 1:** `git checkout battery-endurance-test -- robotctl/src/endurance.rs scripts/endurance-plot.py`.
- [ ] **Step 2:** Hand-apply only the endurance hunks of `git diff main battery-endurance-test -- robotctl/src/main.rs robotctl/src/show.rs docs/robot/cheatsheet.md`.
  That diff also *reverts* newer `main` work (`vetbook`, the UDP-pad status) — keep `main`'s side
  of those.
- [ ] **Step 3:** Fix whatever no longer compiles against `main`'s `duck-ipc-proto` (field and
  method names only; no behaviour change).
- [ ] **Step 4:** Run the gate. Expected: the six `endurance::tests` pass, nothing else changes.
- [ ] **Step 5:** Commit `robotctl: bring endurance over from battery-endurance-test`.

### Task 2: Lift the shared run plumbing into `runlog.rs`

**Files:**
- Create: `robotctl/src/runlog.rs`
- Modify: `robotctl/src/endurance.rs`, `robotctl/src/main.rs` (`mod runlog;`)

**Interfaces:**
- Produces (all `pub(crate)` in `runlog`):
  - `struct Unit { name: &'static str, runs_dir: &'static str }` with
    `fn start(&self, args: &[String]) -> Result<(), Failure>`, `fn stop(&self)`,
    `fn status(&self)`, `fn default_out(&self) -> PathBuf`, `fn active(&self) -> bool`
  - `DurableCsv`, `fsync_dir`, `csv_field`, `unix_secs`
  - `Run { t0, shared: Mutex<Shared>, events }` with `event(kind, detail)`,
    `set_activity(activity, detail)`; `Shared` gains `lap: u32`, `step: String`
  - `StateFeed` (`spawn`, `frame`, `take_mean_current_ma`), `FRAME_MAX_AGE`
  - `Sampler` (`new`, `spawn`), now writing the `lap` and `step` columns (empty for endurance)
  - `install_signal_handlers()`, `signalled()`, `signal_name()`, `brief(&Failure)`, `json(&T)`
- Endurance keeps its `Driver`, args, `Rng`, steering and tests.

- [ ] **Step 1:** Move the items above; endurance calls `Unit { name: "robotctl-endurance", runs_dir: "/var/lib/endurance" }`.
- [ ] **Step 2:** Move `csv_quotes_only_what_needs_it` and `sysfs_entries_sort_numerically` to `runlog::tests`.
- [ ] **Step 3:** Run the gate. Expected: same test count as Task 1, all pass.
- [ ] **Step 4:** Commit `robotctl: share endurance's run plumbing in runlog`.

### Task 3: The lap and the get-up, as pure logic

**Files:**
- Create: `robotctl/src/stress/lap.rs`, `robotctl/src/stress/getup.rs`, `robotctl/src/stress/mod.rs` (just `mod lap; mod getup;` for now)
- Modify: `robotctl/src/main.rs` (`mod stress;`)

**Interfaces:**
- Produces, `lap.rs`:
  - `pub const VERSION: &str = "stress-v1";`
  - `pub enum Action { Walk { vx: f64, vyaw: f64 }, Fall { vx: f64 }, Sit, Stand }`
    (`Fall` = walk at `vx` is the step before; the step itself is the cut)
  - `pub struct Step { pub name: &'static str, pub action: Action, pub seconds: f64 }`
    (`seconds` = 0.0 for `Fall`: it lasts as long as the get-up)
  - `pub fn lap(speed: f64, vyaw: f64) -> Vec<Step>` — the expanded lap, 22 steps, names
    `"A1.fwd"`, `"A1.fall_fwd"`, `"A1.back"`, `"A1.fall_back"`, …, `"B.fwd"`, `"B.left"`,
    `"B.back"`, `"B.right"`, `"C1.sit"`, `"C1.stand"`, …
- Produces, `getup.rs`:
  - `pub struct Timing { pub timeout: f64, pub reset: f64, pub hold: f64, pub no_fall: f64 }`,
    `impl Default` = 7.0 / 2.0 / 1.0 / 2.0
  - `pub enum Verdict { Waiting, NoFall, GotUp { seconds: f64, after_reset: bool }, Reset, GaveUp }`
  - `pub struct Getup` with `pub fn provoked(t: Timing) -> Self` (starts at the re-enable,
    may end in `NoFall`), `pub fn unprovoked(t: Timing) -> Self` (starts at the first `fallen`,
    never `NoFall`), `pub fn tick(&mut self, now: f64, fallen: Option<bool>) -> Verdict`
    (`now` seconds since construction; `None` = no frame, counted as down), and
    `pub fn resumed(&mut self, now: f64)` (call after the 2 s reset's re-enable; restarts the 7 s)
  - `Reset` is returned once; after `resumed`, a second timeout returns `GaveUp`.

- [ ] **Step 1: Write the failing tests**

```rust
// lap.rs
#[test] fn a_lap_has_six_falls_three_forward_three_back()
#[test] fn a_lap_walks_as_far_back_as_forward_and_turns_cancel()
    // Σ vx·seconds over Walk steps == 0.0 and Σ vyaw·seconds == 0.0
#[test] fn the_lap_is_the_spec_s_table()
    // lap(0.3, 1.5).len() == 22; steps[0] == ("A1.fwd", Walk{0.3,0}, 2.0);
    // steps[12] == ("B.fwd", Walk{0.3,0}, 7.0); steps[13] == ("B.left", Walk{0,1.5}, 2.0);
    // steps[16] == ("C1.sit", Sit, 3.0); steps[17] == ("C1.stand", Stand, 2.0); last name "C3.stand"

// getup.rs
#[test] fn up_within_the_timeout_reports_the_time_to_the_start_of_the_hold()
    // provoked; fallen true at 0.1..3.0, false from 3.0; tick at 4.0 → GotUp{seconds≈3.0, after_reset:false}
#[test] fn a_relapse_inside_the_hold_restarts_it()
    // false at 2.0, true at 2.5, false from 3.0 → GotUp at 4.0 with seconds≈3.0, Waiting at 3.5
#[test] fn still_down_at_seven_seconds_asks_for_a_reset_then_gives_up()
    // always true: tick(7.0) == Reset; resumed(9.0); tick(15.9) == Waiting; tick(16.0) == GaveUp
#[test] fn up_after_the_reset_is_marked_as_such()
#[test] fn never_down_within_two_seconds_is_no_fall()
    // provoked, always false: tick(1.9) == Waiting, tick(2.0) == NoFall
#[test] fn an_unprovoked_fall_uses_the_same_rules()
    // unprovoked, always false: never NoFall; GotUp after 1.0 with seconds≈0.0
#[test] fn no_frame_counts_as_down()
    // provoked; fallen true at 0.5, then None from 1.0..8.0 → Reset at 7.0
```

- [ ] **Step 2:** Run `cargo test -p robotctl stress::` — expected FAIL (items not defined).
- [ ] **Step 3:** Implement `lap.rs` and `getup.rs` to the interfaces above.
- [ ] **Step 4:** Run the gate. Expected PASS.
- [ ] **Step 5:** Commit `robotctl stress: the lap and the get-up machine`.

### Task 4: The guards

**Files:**
- Create: `robotctl/src/stress/guard.rs`

**Interfaces:**
- Produces:
  - `pub fn nearest_ahead(zones: &[kinematics::tof::Zone], half_width: f64) -> Option<f64>` —
    smallest `range` over `Zone::Hit { point, range }` with `point[0] > 0.0` and
    `point[1].abs() <= half_width` (corridor half-width 0.15 m, the robot's width plus margin —
    *builder's call per spec*). `Floor`, `TooClose`, `Empty` are ignored.
  - `pub fn classify(frame: &proto::TofFrame, state: &proto::RobotState, r: &kinematics::tof::Reprojector) -> Option<Vec<Zone>>`
    — the same reduction as `monitor.rs` `classified_tof` (head joints at `state.joints[5..9]`,
    `Posture` from `safety.gravity` and odometry Z > 0.02). Extract it from `monitor.rs` into a
    shared `pub(crate) fn` here and make `monitor` call it, rather than copying it.
  - `pub fn outside_square(pose: (f64, f64), home: (f64, f64), half: f64) -> bool` — `|dx| > half || |dy| > half`.
  - `pub fn toward(pose: (f64, f64, f64), target: (f64, f64)) -> (f64, f64)` — `(vx, vyaw)` to
    walk to a point: turn on the spot above 0.6 rad heading error, else full stick (reuse
    `endurance`'s `steer`; move it to `guard.rs` and have endurance import it).

- [ ] **Step 1: Write the failing tests**

```rust
#[test] fn a_hit_straight_ahead_is_the_nearest()        // Hit at (0.18, 0.0) → Some(0.18)
#[test] fn hits_off_the_corridor_and_behind_are_ignored() // (0.10, 0.30), (-0.10, 0.0) → None
#[test] fn floor_and_too_close_are_not_walls()          // [Floor, TooClose, Empty] → None
#[test] fn the_square_is_half_a_metre_each_way()         // (0.49,0.49) false, (0.51,0) true, (0,-0.51) true
#[test] fn toward_turns_first_then_walks()              // (keep endurance's steering test, moved here)
```

- [ ] **Step 2:** Run `cargo test -p robotctl stress::guard` — expected FAIL.
- [ ] **Step 3:** Implement; move `classified_tof`'s body and `steer` as stated.
- [ ] **Step 4:** Run the gate. Expected PASS, including `monitor` and `endurance` tests unchanged.
- [ ] **Step 5:** Commit `robotctl stress: the square and the wall`.

### Task 5: The driver and the subcommand

**Files:**
- Modify: `robotctl/src/stress/mod.rs`, `robotctl/src/main.rs` (`Namespace::Stress`, dispatch)
- Modify: `docs/robot/cheatsheet.md` (a `robotctl stress` section next to `endurance`'s)

**Interfaces:**
- Consumes: Task 2 `runlog::*`, Task 3 `lap::*`, `getup::*`, Task 4 `guard::*`.
- Produces:
  - `pub enum StressCommand { Status, Stop }`
  - `pub struct StressArgs` (clap, `Serialize` into `meta.json`): `--out`, `--interval` (10.0),
    `--foreground`, `--speed` (0.3), `--vyaw` (1.5), `--cut` (0.5), `--getup-timeout` (7.0),
    `--reset` (2.0), `--square` (0.5, half-side, m), `--wall` (0.20, m), `--hands-off-pct` (2.0)
  - `pub fn run(robot_socket: &Path, tof_socket: &Path, command: Option<StressCommand>, args: StressArgs) -> Result<(), Failure>`
  - `Unit { name: "robotctl-stress", runs_dir: "/var/lib/stress" }`; `meta.json` carries
    `lap: lap::VERSION` and the args.

The driver, per lap (`Shared.lap` += 1, event `lap_start`), per step (`Shared.step` = name, event
`step`), with a schedule: each step has a deadline = step start + `seconds`; the next step starts
at that deadline whatever happened inside (a get-up step has no deadline — it ends with its
`Getup` verdict).

- **Walk**: `robot.move` at 20 Hz until the deadline. On every tick: `fallen` → run an
  unprovoked `Getup` (event `fallen_unprovoked`), then skip to the next step. Forward walks also
  check `nearest_ahead < --wall` → event `wall_guard`, turn on the spot (+vyaw) until clear or the
  deadline. `outside_square` → event `square_guard`, `toward(home)` until back inside 0.8 ×
  half or the deadline.
- **Fall**: `enable{on:false}` (event `fall_triggered`), wait `--cut`, `enable{on:true}`, then
  tick a provoked `Getup` at 20 Hz with `robot.move(0,0)`. Verdicts → events: `fallen` (first
  `fallen` true), `no_fall`, `got_up` (detail = seconds, after_reset), `getup_failed` +
  `enable{on:false}`, wait `--reset`, `enable{on:true}`, `resumed` (event `long_reset`),
  `gave_up` → `robot.relax`, set activity `gave_up`, then only sample until signalled. After
  `got_up`, if the policy reads `"sit"`, `sit_toggle` and wait for a driving policy (≤ 15 s).
- **Sit / Stand**: reuse endurance's `sit` / `rise` behaviour, bounded by the step's seconds.
- Before every `enable` and `sit_toggle`: `check_battery()` (hands off at ≤ 2 %, or `robotd`
  went to `rest`/an unrequested `sit`).
- ToF: a thread like `monitor`'s `read_tof` keeping the latest frame; a frame older than 0.5 s is
  none. If the stream answers with no sensor or the socket is absent: event `no_tof` once, and
  wall checks are skipped.
- Startup: `ensure_driving` (as endurance), home = odometry pose then. Stop (signal): zero
  twist, leave standing, as endurance's `park`.

- [ ] **Step 1:** Write a unit test `the_schedule_does_not_slip_when_a_step_is_cut` on the pure
  schedule helper `fn deadlines(steps: &[Step], start: f64) -> Vec<Option<f64>>` (walk/sit/stand
  get `Some(cumulative)`; a fall gets `None` and resets the base to "when its get-up ended" —
  expose `fn next_start(prev_deadline: Option<f64>, getup_end: f64) -> f64`).
- [ ] **Step 2:** Run it, expect FAIL; implement; run, expect PASS.
- [ ] **Step 3:** Implement the driver, `run`, and `main.rs` wiring (same doc-comment shape as
  `Endurance`: background unit, root, no pad connected).
- [ ] **Step 4:** Run the gate.
- [ ] **Step 5:** Check the CLI: `cargo run -p robotctl -- stress --help` lists every flag above
  with its default; `cargo run -p robotctl -- stress status` without a run says none is going.
- [ ] **Step 6:** Write the cheatsheet section: start/status/stop, the lap table, the event
  names, and that a pad must not be connected.
- [ ] **Step 7:** Commit `robotctl stress: the driver`.

### Task 6: Plot per lap and overlay robots

**Files:**
- Modify: `scripts/endurance-plot.py`

**Interfaces:**
- Consumes: `events.csv` kinds from Task 5; `samples.csv` `lap` column; `meta.json` `lap`.
- Produces: `endurance-plot.py RUN_DIR [RUN_DIR …]`. For a stress run, `summary.md` gains a
  per-lap table (lap, falls, no_fall, got_up count, median/max get-up s, getup_failed,
  gave_up, square_guard, wall_guard, battery % at lap end, hottest servo °C) and a
  `getup.png` (get-up time per fall against lap). With several run dirs: refuse to overlay runs
  whose `meta.json` `lap` versions differ (print which), else `compare.png`: median get-up
  time and battery % against lap number, one line per robot (hostname from `meta.json`).

- [ ] **Step 1:** Make a fixture: a small fake stress run under
  `scripts/testdata/stress-run/` (meta.json, 3 laps of events, samples) and add
  `scripts/test_endurance_plot.py` with `test_per_lap_table_counts_falls_and_getups` (lap 1: 6
  falls, 5 got_up, 1 no_fall) and `test_overlay_refuses_mixed_lap_versions`.
- [ ] **Step 2:** Run `python3 -m pytest scripts/test_endurance_plot.py` — expect FAIL.
- [ ] **Step 3:** Implement; an endurance run (no `lap` in meta) still plots as before.
- [ ] **Step 4:** Run the tests and `python3 scripts/endurance-plot.py endurance/microduck-beta-20261008T093913Z`
  (existing run, untracked in the checkout) — expected: same summary as before.
- [ ] **Step 5:** Commit `endurance-plot: per-lap stress tables and robot overlays`.

### Task 7: On a robot

- [ ] **Step 1:** `cargo build --release -p robotctl --target aarch64-unknown-linux-gnu`, copy to
  a duck per `docs/robot/dev-push.md`.
- [ ] **Step 2:** In a pen, no pad connected: `sudo robotctl stress --foreground` for 2 laps;
  watch it. Expected: 6 `fall_triggered` per lap, `got_up` times logged, no `gave_up` on a
  healthy duck, the lap restarts.
- [ ] **Step 3:** Report back the events of those 2 laps (count of `no_fall` especially: if most
  cuts do not fall, the 0.5 s cut needs revisiting with Coralie, not changing silently).
