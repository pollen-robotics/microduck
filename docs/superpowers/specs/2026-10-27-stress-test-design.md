# `robotctl stress` — a reproducible fall-and-get-up endurance test

## Intent

`robotctl endurance` (branch `battery-endurance-test`) measures what ordinary use costs in
battery and heat. It walks gently, sits a lot, and when the robot falls it **pauses and waits for
a human**. That is the opposite of what we now need.

`robotctl stress` is a violent test: the robot walks most of the time, is **made to fall on
purpose**, and **gets itself up**. It runs until the battery is empty. Its purpose is to compare
robots, and versions of the same robot, on the get-up: how long it takes, how often it fails,
and how the battery and the motors hold up under that load. So it has to be **the same test on
every robot**, so that two runs can be laid side by side and their differences belong to the
robots, not to the test.

## Goals

- Little idle time: walking dominates, standing and sitting are short.
- Falls are provoked, forward and backward, at fixed points of a scripted lap.
- The get-up is the robot's own (the policy picks it up); the test only measures it and gives up
  cleanly when it cannot.
- It stays inside a 1 m² square and does not walk into walls (head ToF as a guard).
- Deterministic: the same commands in the same order on every robot; anything that differs is
  measured and logged, never compensated.
- Logs good enough to compare robots lap by lap.

## Non-goals

- Mapping, navigation, or using the ToF for anything but "don't hit the wall".
- Exploring the whole square: the lap is roughly self-cancelling and stays near its start.
- Changing `robotd`'s fall or get-up behaviour. The test drives `robotd` as a client.

## Anti-goals

- A test whose timeline drifts with each robot's reactions so laps no longer line up.
- A test that keeps thrashing a robot that cannot get up: it must stop for good.
- A test that fights `robotd`'s empty-battery shutdown.

## Starting point

New branch `stress-test`, from `main`. Reuse the skeleton of `robotctl endurance` from
`battery-endurance-test` (`robotctl/src/endurance.rs`, `scripts/endurance-plot.py`): background
transient systemd unit, `status` / `stop`, run directory under `/var/lib/…` that survives the
final reboot, `fsync`ed CSV rows, sample columns (battery, thermal zones, cpufreq, servo temps,
safety flags, odometry), and the `--hands-off-pct` handover to `robotd`'s battery-empty shutdown.
That branch is not merged as is; the parts reused are brought over and adapted to `main`'s APIs.

## The fall

Provoked exactly as with the pad's Start button while walking:

1. walk at full command (forward or backward, `robot.move` at 20 Hz);
2. `robot.enable { on: false }`: the policy stops and `robotd` ramps to the home pose; mid-stride
   the robot loses its balance;
3. **0.5 s later**, `robot.enable { on: true }`: the policy comes back while the robot is still
   falling. The off time is short on purpose — a longer one lets the ramp to the home pose
   settle the robot, which is a softer fall and an easier get-up than a real one;
4. the test then watches for the robot to be down (`safety.fallen`, debounced 0.2 s in `robotd`)
   and for it to get up.

If the robot did not fall (`safety.fallen` never set within ~2 s of the enable — *my call*), the
event is logged as `no_fall` and the lap continues. That is data, not an error: with only 0.5 s
off, some cuts will be caught by the policy, and the rate is worth comparing too.

### v2: a trip, not a cut (2026-11-03)

The cut above never exercises `robotd`'s limp-fall: it makes `driving` false, the one condition
the fall predictor needs, and when the policy returns the robot is already past the arming gate.
Every provoked fall landed stiff — the case limp-fall exists to prevent, so not a real fall.

The default fall is now `robot.trip` (robotd API v50): the leg targets held where they were for
`[stress] trip_ms` (500 ms — 250 was caught too often) with `trip_legs` both, left or right,
both in `robotctl configure` and re-read at every fall so a running test can be tuned;
`--trip-ms` / `--trip-legs` pin them for one run. The legs are held while the policy keeps stepping and the walk
command keeps arriving. The policy tries to recover; if it cannot, the robot falls with the
limp-fall live, as a caught foot would. Each fall logs `limp` when the limp-fall took it. The
trip length is tuned by hand first, `robotctl robot trip MS` while walking on the pad. The cut
stays as `--fall cut`, and the get-up's 2 s reset is still a cut. `no_fall` now counts from the
end of the trip.

## The get-up and giving up

After the enable, the robot tries to get up continuously; there is no cut between its tries.

- Upright (`safety.fallen` false) within **`--getup-timeout` = 7 s**: `got_up`, with its
  duration.
- Still down at 7 s: `getup_failed`; **policy off for 2 s**, then on again, and another 7 s
  (`long_reset`).
- Still down after that: `gave_up`; the run sends `robot.relax` (torque off — *my call*: policy
  off with torque on would push the home pose into the floor for hours, heating the servos and
  skewing the battery data), stops driving for good, and keeps logging samples until stopped or
  the battery dies. `status` says so plainly. It also calls for help — two quacks and a wheee every 15 s
  (`--call-every`) — so somebody in the next room knows to come and look.

**Got up** means `fallen` false and held for 1 s without a relapse (`fallen` clears on a single
upright sample, and a rocking robot can flip back). Upright on its seat counts as up — on
re-enable `robotd` may hand a robot that reads as seated to the sit/stand network — and the test
then sends `sit_toggle` to stand it before the lap resumes. The get-up time is measured from the
enable to the start of that 1 s hold.

**Unprovoked falls** (*my call*): a fall the test did not cause — mid-walk, mid-turn, a failed rise
from the sit — goes through the same get-up machinery (7 s, 2 s reset, 7 s, give up), logged
`fallen_unprovoked`. The interrupted step is abandoned and the lap resumes at the next step's
scheduled time.

## The lap (`stress-v1`)

A fixed script, repeated until the battery is empty. Symmetric (as much forward as backward) so
the robot stays near its start without navigating. Three blocks:

**A — falls, ×3**

| step | length |
|------|--------|
| walk forward | 2 s |
| cut 0.5 s → fall forward, get up | — |
| walk backward | 2 s |
| cut 0.5 s → fall backward, get up | — |

**B — walk, ×1**

| step | length |
|------|--------|
| walk forward | 3 s |
| walk backward | 3 s |
| turn in place left | 2 s |
| turn in place right | 2 s |

**C — sit/stand, ×3**

| step | length |
|------|--------|
| sit (`sit_toggle`), seated | 3 s |
| rise (`sit_toggle`), standing | 2 s |

Then the next lap. Six provoked falls per lap. The walk comes back before the turns and the two turns cancel, so the lap
ends where it started (*my call*: short turns in place rather than a curved walk). The sit and stand lengths count from the
`sit_toggle` that starts them.

Speeds are fixed (full stick: 0.3 m/s, 1.5 rad/s, `endurance`'s defaults). The script is data in
the code with a version string; the version goes in `meta.json` and each sample row carries the
lap number and step index, so runs compare lap for lap.

## Guards: the square and the wall

These are safety overrides, not navigation, and each is logged as an event with the lap/step it
happened in:

- **Square**: ±0.5 m from the start on odometry. If a walking step would take the robot past
  it, the step is cut short and the robot walks back toward the centre before the next step.
- **Wall**: from `tofd`'s frames, if the nearest return in the direction of travel is under
  30 cm (forward walking only — the sensor is in the head and looks forward), the step is cut
  short and the robot turns in place away from it before resuming the lap. 20 cm until the first
  runs, whose forward falls started that close and landed against the wall.
- **Looking behind** (`look_back`): the sensor looks forward and the lap backs up, so before each
  backward walk the robot turns its body a quarter left and its head a quarter left, which points
  the ToF where it is about to back into. Clear for the backing up plus `--wall`: it turns back
  and goes. A wall nearer: it carries on to a half turn, faces the wall and backs away from it.
  The look sits outside the step's slot, like a get-up, so the step keeps its length.
- **A wall on the way back to the start moves the start** (`home_moved`): the one exception to
  "never re-anchor" below. The robot never walked through a wall, so a wall between it and the
  start is drift; walking back to that start pinned the first runs to the wall until they gave up.
- No ToF (socket absent / no sensor): one line in the events, and the run continues on odometry
  only.

A cut step ends early; the recovery (walk back / turn away) is time-boxed inside what is left of
that step's slot, and the lap goes on with the next step at its scheduled time rather than
replaying it, so the lap timeline stays the same (*my call*). Get-up time is the one thing that
stretches a lap; it is measured, and the lap's own steps keep their fixed lengths.

**Odometry after a fall** (*my call*): `robotd` keeps integrating contact odometry while the robot
is down and flailing, so after a few falls the square is approximate. That is accepted: the
origin is not re-anchored (re-anchoring would let real drift accumulate unseen), the square guard
is a coarse backstop, and the ToF is the guard that actually keeps it off the walls. Run it in an
open area or a pen.

## Output

Same run directory shape as `endurance` (`meta.json`, `samples.csv`, `events.csv`), plus:

- events: `lap_start`, `step`, `fall_triggered`, `fallen`, `no_fall`, `got_up` (duration),
  `getup_failed`, `long_reset`, `gave_up`, `square_guard`, `wall_guard`, `no_tof`, `hands_off`;
- a per-lap summary (laps file or derived by the plot script): falls, got-up count, get-up times,
  failures, guards, battery % and hottest servo at lap end;
- the plot script can overlay several run directories (several robots) lap by lap: get-up time
  and battery against lap number.

The data is expensive, so a run has to survive what happens to the robot:

- **Where.** `/var/lib/robot/stress/`, on the data partition. `/var/lib` is on the root filesystem,
  which is one per A/B slot: on 2026-11-04 a fallback from B to A hid a morning of runs on the slot
  no longer booted.
- **When.** Nothing starts until a time server has set the clock. The board has no battery-backed
  clock and boots on the last time it saved; a boot with no wifi stayed there, and its runs took
  yesterday's name and wall times, wrong by an amount nothing recorded. Waiting is simpler and
  surer than correcting afterwards, and the cost is that a robot with no network does not test.
- **Whole files.** `meta.json` is replaced atomically (written in place, it was left at 0 bytes). A
  reopened run cuts a half-written row off each CSV before appending, and logs `repaired`; an
  unreadable `meta.json` is set aside, not overwritten.
- **What it ran on.** Each segment records `revision`, `release`, `slot`, `os`, `policies` (SHA-256
  by file), `config` (the whole `robotd.toml`) and `note`. Every build of a release says the same
  version, which is how mixed builds went unnoticed.
- **Why it did what it did.** At each start, every earlier boot with a run here and no saved log
  gets `journal/<boot id>.txt` in that run, before the journal rotates it away.
- **Checking.** `robotctl stress verify` and the plot script make the same checks.

## Running it

`sudo robotctl stress` (background), `robotctl stress status`, `sudo robotctl stress stop`;
`--foreground` and `--out` as in `endurance`; `--getup-timeout` (7 s). A stop (`stop`, or ctrl-C in
`--foreground`) turns the policy off, as the pad's Start does: the robot holds its home pose rather
than balancing on. Running out of battery ends it as `endurance` does. Documented in `docs/robot/cheatsheet.md`.

**At boot**, with `[stress] on_boot = true` (`robotctl configure`; off by default): for a bench
robot left to repeat the test, each charge-and-boot is the next run. `robotctl-stress-boot.timer`
fires 10 s into every boot — time to set the robot down — and runs `robotctl stress boot`, which
reads the key, waits for the clock, waits up to 120 s for robotd to be publishing state (not just its socket: robotd
answers before it publishes, and a run started then dies `fatal`) and starts the default run exactly as
`sudo robotctl stress` would. The oneshot has no `[Install]` section, like `robot-boot-check`'s, so
neither an install nor an update ever starts a run; changing the key starts or stops nothing until
the next boot.

## Left to the builder

- How much of `endurance.rs` is shared vs. copied; whether `endurance` itself comes along.
- Exactly how the ToF frame is reduced to "nearest ahead" (the head pose matters; keep the head
  still during the test).
- The column set, provided it stays a superset of what `endurance-plot.py` reads.
