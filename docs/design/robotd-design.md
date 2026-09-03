# `robotd` — the control loop

Status: draft · Date: 2026-08-20 · Owner: pierre

Implements the `robotd` row of [`architecture.md`](architecture.md) §1 and covers
[`roadmap.md`](../project/roadmap.md) M3. The prototype being absorbed is
[`apirrone/microduck_runtime`](https://github.com/apirrone/microduck_runtime), referred to
throughout as *the runtime*.

**Only the alpha variant, only the Radxa, only the v2 `imu_to_dxl` board.** v1/v1.5/v1.6, the
four other IMUs, the three cameras and the Pi are dropped, and every shipped policy is
`alpha_*`. The wheeled configuration survives as one params switch — `policy.mode = "roller"`
(§4.2) — because what it selects is a policy set and a tuning preset, not a hardware variant.

## 1. The shape of it

One process, one serial bus, one 50 Hz loop. The loop reads all sixteen devices on the bus in
a single transaction, decides fifteen joint targets, and writes them back. Everything else —
clients, health, telemetry — hangs off that loop without ever being able to block it.

### 1.1 The bus, and who owns the port

Fifteen servos and the `imu_to_dxl` board share one UART. There is no second bus and no
second port:

```text
                     robotd — control thread
                              │
                              │  duck_control::bus::DynamixelIo
                              │  serialport · TIOCEXCL
                              ▼
         /dev/ttyS2 · 1 Mbps · Dynamixel protocol v2
                              │
    ┌────────────┬────────────┴───────┬──────────────────┐
    │            │                    │                  │
  id 200       20–24                30–34             10–14
 imu_to_dxl   left leg        neck · head · mouth     right leg
  v2 board    5 servos             5 servos           5 servos
```

The IMU is `id 200` and is read in the *same* `sync_read` as the servos, because that is what
the hardware does: the v2 board sits on the Dynamixel bus and serves an on-chip SFLP
quaternion out of the same register block the servos answer at. One board, one code path, no
IMU abstraction. It is listed first in the id vector so it answers before the servo burst.

**One owner at a time, and nothing hard-enforces it.** `serialport` sets `TIOCEXCL`, which
turns a second *unprivileged* open into `EBUSY` — but `robotd.service` runs as root, because
motor control needs the character devices, and root is not stopped by that flag. So the
exclusion is arranged rather than enforced, and each other claimant is kept off the port
deliberately:

- **The control loop** owns it for as long as the daemon runs.
- **`robotd init`** opens the port itself, and as root it will succeed *while the daemon is
  running* — two writers interleaving packets on one bus, which reads as a hardware fault. So
  it wants the daemon stopped, and that is exactly why `robot.init` and `robot.relax` exist as
  IPC methods (§3.3): the daemon serves both from inside the loop, so nothing else has to open
  the bus at all. `init` is the escape hatch for a robot whose daemon is not running.
- **`serial-getty@ttyS2`** — Armbian runs a login console on UART2 by default, and an `agetty`
  holding the port makes every servo invisible to everything else. `scripts/setup-board.sh`
  masks the unit; `fuser -v /dev/ttyS2` naming `agetty` is how that was found, and it is still
  the command that answers "who has the bus".
- **The runtime**, at a coarser grain: it drives the same bus, so a board runs the runtime or
  `robotd` and never both, and the units say so with `Conflicts=` (§5.2).

### 1.2 Who talks to `robotd`

```text
   ┌──────────┐   robot.move / robot.head      ┌─────────────────────┐
   │  padd    │──(notifications, ≤50 Hz)──────►│                     │
   │ gamepad  │   robot.stop / robot.enable    │                     │
   └──────────┘───(requests, answered)────────►│                     │
                                               │   /run/robotd.sock  │
   ┌──────────┐   robot.subscribe              │   JSON-RPC 2.0      │
   │ robotctl │──────────────────────────────► │   NDJSON            │
   │ monitor  │◄──robot.state (decimated)──────│                     │
   └──────────┘                                │                     │
                                               │                     │
   ┌──────────┐   robot.health                 │                     │
   │ updaterd │──robot.safeToRestart──────────►│                     │
   │          │  robot.modelApi                └──────────┬──────────┘
   └──────────┘                                           │
        │                                                 │
        │ on_apply: systemctl restart robotd              │
        └─────────────────────────────────────────────────┘

   ┌ the two transports ────────────────────────┐
   │  mediad — WebRTC + JSON-RPC relay          │  phone, browser and LLM
   │  btd    — BLE, a subset of the same API    │  clients arrive through here
   └────────────────────────────────────────────┘
```

Every one of those speaks the same two vocabularies — intents in, state out — so `mediad`
**relays** frames rather than translating them (§3.1, §3.2). `updaterd` only ever asks
questions: nothing in the update path can command a motor.

### 1.3 The crate boundary

```text
duck-ipc-proto/  wire contract — serde only; no tokio, no http, no crypto
duck-control/    robot model · bus · IMU · RobotIo · obs · policy · safety
                 everything between reading the bus and writing it
                 no tokio, no sockets, no systemd
robotd/          the process: socket, JSON-RPC, systemd, health reporting
robotd-params/   the startup parameters: schema, defaults, validation (§4.2)
kinematics/      the MJCF model, forward kinematics, head and hand chains
odometry/        where the robot is, from foot contacts and the IMU
sounds/          the voice: synthesis, per-robot personality, the chorale's score
pet-detect/      the camera-side detector robotd polls, off the loop
robotctl/        CLI
padd/            gamepad → intents
updater/         engine + updaterd
```

Everything below `robotd/` in that list is a **library it drives**, not a service: no tokio
runtime of its own, no socket, nothing systemd starts. They are separate crates for the same
reason `duck-control` is — the compiler is what keeps daemon concerns out of them, and
`kinematics` in particular has two consumers (`odometry` and `robotd`'s head FK) that would
otherwise each grow a copy of the model.

`duck-control` holds everything between reading the bus and writing it; `robotd` is the
process around it. The compiler enforces that boundary, which is what stops daemon concerns
leaking into control code — and it means the crate can be lifted into its own repo later if
the runtime needs to consume it during the transition, without that being a rewrite.

`safety` holds the `RobotIo`, so the policy, the controller and every client can *propose*
targets and none of them can send one. That is the borrow checker, not a convention (§2.4).

### 1.4 The tick

50 Hz, one `tokio` task on its own runtime so IPC work cannot sit in front of it. Two bus
transactions per tick, plus a third once a second:

```text
read()          one sync_read  · IMU board + 15 servos · registers 124–136   (§2.1)
decide          observation → policy → targets → clamp                 (§2.2–§2.4)
write()         one sync_write · goal positions
publish         atomics always; a state frame only if someone subscribed     (§4.1)

every 1 s       slow_sensors() · registers 144–146 · voltage + temperature   (§2.1)
```

Where the data goes, once per period:

```text
  Dynamixel bus
       │  one sync_read: IMU board + 15 servos, one transaction
       ▼
   Sensors ──────────┬──────────────────────► safety.observe ──► fallen? (debounced)
   joints, IMU       │
                     ▼
              Observation::build  ◄──── Command ◄── gate(deadman) ◄── intent snapshot
                     │              (twist, head, body = nominal)
                     │  [f32; 61]
                     ▼
              Policy::infer ──── roulade > kick > ground pick > sit/rise >
                     │           stand (by |twist|, or forced) > walk
                     │  [f32; 14]   — mouth excluded
                     ▼
              home pose + scale × action ──► low-pass on head and legs
                     │
                     │  [f64; 15] proposed targets
                     ▼
        ╔═════════════════════════════════════════════╗
        ║  safety.apply   ← owns the only RobotIo      ║
        ║  · refuse non-finite                         ║
        ║  · clamp to actuator range                   ║
        ║  · no fall gate — the verdict only reports   ║
        ╚═════════════════════════════════════════════╝
                     │  sync_write goal positions
                     ▼
              Dynamixel bus
```

And the decisions around it, which the dataflow above does not show:

```text
  startup
    ├─ open the bus, waiting              an unpowered board is fixed with a switch,
    │                                     not by abandoning the loop
    ├─ Safety::new(io)                    safety takes the RobotIo; nothing else can write
    ├─ read() → hold = the pose the robot is already in    ── never move on start
    └─ policy
         disabled ─────────────► controller = None                    healthy
         loaded   ─────────────► controller = Some
         failed   ─────────────► controller = None + policy_error   unhealthy

  each tick
    read ─┬─ ok  ─► clear the consecutive-error count
          └─ err ─► count++, sensors = None   (the tick still runs)

    observe → fallen?     published; gates nothing

    driving = enabled ∧ policy loaded ∧ sensors this tick ∧ ¬limp-fall

    edges ─┬─ started driving ──► controller.reset()
           │                      else a stale last action, or a filter anchored to
           │                      where the robot was a minute ago, shows up as a lurch
           └─ stopped driving ──► hold = current pose, captured once
                                  re-reading each tick would sag under gravity

    driving ─┬─ limp-fall ─► the sequence's own targets and gain    (§2.4.1)
             ├─ yes ──────► step() → targets, gain, the active net's name
             └─ no  ──────► targets = hold, default gain, "held"

    safety.apply(targets, hold, gain)

    publish ─┬─ atomics            always      → robot.health, safeToRestart
             └─ state frame        only if subscribed   → robot.state
```

The four conditions on `driving` are each load-bearing. `sensors this tick` is the non-obvious
one: a read that failed leaves nothing to build an observation from, and inventing one would
feed the policy a robot that does not exist. `¬limp-fall` is the one that is not a refusal —
while the limp-fall sequence owns the robot (§2.4.1) the policy is deliberately not driving,
and the targets come from the sequence instead.

The loop stays a `tokio` task with `interval`. It is not being made real-time (§5.4). One
change from the runtime is worth naming: **`MissedTickBehavior::Skip`**. `Burst` fires the
backlog back to back and stacks motor commands on top of each other. `Delay` is wrong in a
less obvious way — it schedules the next tick at *now + period* after each one, so every
wakeup latency is added to the period instead of being absorbed, and the loop drifts slower
than its own configured rate. `Skip` keeps the original schedule and drops missed ticks, which
is what a control loop wants. Moving perception out to `mediad` removed most of what competed
with the loop for free.

### 1.5 The invariants

Five properties everything else is arranged around. Each is enforced somewhere, not merely
intended:

1. **Nothing above `safety` can write to a motor.** It owns the only `RobotIo`; the borrow
   checker is the enforcement (§2.4).
2. **`robotd` never moves the robot because a process started.** An update restart leaves a
   standing robot standing (§3.3).
3. **The control loop never waits on a client.** Intents are atomic loads, telemetry is
   drop-on-lag (§4.1).
4. **Health is published, never asked for.** A wedged loop reports itself unhealthy rather
   than hanging the caller (§3.4).
5. **Only what a release can be blamed for may reach the health verdict.** Battery and
   temperature are description, never a rollback input (§3.4).

## 2. The control path

### 2.1 The bus layer and `RobotIo`

A thin layer over `rustypot`: open, one combined `sync_read`, `sync_write` goal positions,
torque enable, gains, the slow sensor read, and the startup register check. Written fresh
rather than lifted, but **the numbers are borrowed from the runtime**, each with a comment
saying so:

- `RAD_PER_SEC_PER_COUNT = 0.229 × 2π/60`, and the position count↔radian conversion.
- The EEPROM registers from `check_and_fix_config`, asserted *and corrected* at startup:
  `return_delay_time=0`, `baud_rate=3`, `pwm_slope=255`, `shutdown=52`. The first is
  load-bearing — at the XL330 default of 250 that is 500 µs of turnaround per device, so
  sixteen devices cost ~8 ms per tick, 40% of the budget. A servo that was factory-reset or
  swapped in arrives at 250, so the check is what removes a whole class of "why is it slow on
  this robot". `shutdown = 52` is the error mask that latches on overload, overheating and
  input-voltage faults.
- The position P gain is written with I and D at **zero**, the runtime's `--ki`/`--kd`
  defaults. These are RAM registers, so a power cycle restores the servo's factory values, and
  the factory D is not zero: left in place it damps the servo's internal PID and the robot runs
  measurably softer at the *same* kP. Not a tuning choice anyone made, so it is pinned rather
  than exposed.

**Two bus transactions per tick, and a third once a second.** The tick reads a contiguous
block at 124–136 (pwm, current, velocity, position). Voltage and temperature sit at 144–146,
eight bytes past its end, with twelve bytes of trajectory registers nothing wants in between —
so they are sampled together once a second in their own transaction (~1 ms) rather than
widening the tick's read to 22 bytes per servo at 50 Hz. The sampling interval is the same
window the achieved rate is measured over, so one clock drives both.

Voltage is averaged across the servos: all fifteen sit on one pack, so a single reading is the
same measurement with more noise, and a device answering zero is filtered out rather than
averaged in as if the pack were half flat. Temperature is *not* averaged — the caller gets
every joint and reports the hottest one **by name**, because a knee holding a squat runs far
above the mouth and a mean over fifteen servos hides the one approaching its overheat
shutdown.

A silent servo does not produce a short answer: `rustypot`'s `sync_read` waits for every id and
fails the whole transaction if one does not reply. So both reads are all-or-nothing, and the
caller keeps its previous sample rather than treating one miss as news.

**Board temperature is a third source, and not on the bus at all.** The hottest of the SoC's
thermal zones, read from `sysfs` in the same once-a-second sample (`robotd/src/soc.rs`). It
lives in `robotd` rather than `duck-control` because it is a property of the Linux board, not
of the robot — which is also why it keeps answering when the motor bus does not, and that is
precisely when it earns its place: a board cooking behind a blocked vent and a robot with dead
servos are the same symptom until you can see both numbers. The maximum across zones rather
than one zone by name, so a board that wires its sensors differently cannot silently omit the
one that was climbing.

**IMU staleness is tracked, permanently.** "The read succeeded but the board handed back the
same sample" feeds dead orientation to the policy, is invisible unless someone counts it, and
is known to happen. The bus layer remembers the last block, counts identical successors, and
says so in the journal once a run is long enough to mean something — half a second, the same
span the SFLP decoder waits before it will call the chip's output a measurement. Warning on the
first repeated block is what teaches everyone to ignore the message; past the threshold it is
rate-limited, because a board that has stopped refreshing produces one per tick and 50 Hz of
identical warnings evicts the journal.

The seam:

```rust
trait RobotIo {
    fn read(&mut self) -> Result<Sensors>;                 // joints + IMU, one transaction
    fn write(&mut self, targets: &JointTargets) -> Result<()>;
    fn set_gain(&mut self, kp: u16) -> Result<()>;         // one write per joint, not per tick
    fn set_torque(&mut self, on: bool) -> Result<()>;      // idem
    fn slow_sensors(&mut self) -> Result<SlowSensors>;     // voltage + per-joint temperature
    fn imu_stale(&self) -> ImuStale;                       // defaulted
    fn imu_ready(&self) -> bool;                           // defaulted
}
```

`set_gain` and `set_torque` are one transaction *per joint*, which is why neither is a
per-tick call and why bring-up is a state machine rather than a flag the loop keeps applying
(§3.3). Two inherent methods sit outside the trait because only `init` uses them:
`present_positions` (a lighter read, used once at startup to adopt the pose the robot is
already in) and `interpolate_to` (a blocking linear ramp — deliberately blocking, since
nothing else should be talking to the bus while it runs).

Two implementations: `DynamixelIo` and `FakeIo` (scripted samples, optionally frozen or
failing on demand). `FakeIo` is what the test suite runs against, and it is why `cargo test`
needs no hardware, no network and no Docker.

**Neither is `cfg`-gated off macOS.** The gate was meant to keep `serialport` out of a
laptop's dependency tree, but `rustypot` and `serialport` both build cleanly there, so it
bought nothing and cost the ability to type-check the bus layer without a board — which is
exactly the code most likely to be edited by someone who does not have one. Only the entry
points that open a real port are gated, so a Mac build still refuses to pretend it has a
robot: `robotd --fake` is the laptop path, and it must be asked for explicitly rather than
fallen back to.

### 2.2 One observation builder

Every alpha policy is `obs[1,61] → actions[1,14]` — verified across walking, standing, ground
pick, ball kick and sit. So there is exactly one layout:

```
[ gyro(3) | projected_gravity(3) | joint_pos(14) | joint_vel(14) | last_action(14) | command(13) ]
                                                                    command = vel(3) + head(4) + body(6)
```

Joints exclude the mouth throughout; actions map back into 15 motor slots with index 9 left at
zero. The 51/54D legacy, 49D wheeled and 85D tracking layouts go away with the variants.

The command block, which was the only part in doubt, is settled — read out of the prototype's
`control_step` rather than guessed:

```text
48..51   vx, vy, vyaw
51..55   neck_pitch, head_pitch, head_yaw, head_roll
55..57   body x, y      — hardcoded zero, unbound in training
57..60   body z, roll, pitch
60       body yaw       — hardcoded zero, unbound in training
```

Three things about it are individually plausible and wrong:

1. **All-zero body is the nominal encoding**, not a placeholder — x, y and yaw are literally
   hardcoded zero as "unbound", and z/roll/pitch are zero unless body-pose mode is active.
2. **Head targets ride in the command and are not added on top of the policy output.** The
   prototype does both in different modes and gates the post-hoc addition behind
   `if !new_cmd_obs`, commented "head\_offsets are a COMMAND fed via the obs vector instead —
   don't double-add it here". Doing both bends the head twice.
3. **The body block is ordered `z, roll, pitch`** — not `z, pitch, roll`. Swapping the last
   two tilts the robot sideways when asked to lean forward.

### 2.3 The policy

Shaped like the runtime's, deliberately. `robotd/src/control.rs` holds the priority chain and
every numeric default from `control_step`, which it replaces:

```text
skill windows ← advance / expire (roulade window, kick timer, ground-pick phase, sit↔stand rise)
command       ← the caller's smoothed command, re-encoded for the active skill
net           ← roulade > kick > ground pick > sit/rise > stand-by-magnitude (or forced) > walk
action        ← ONNX
targets       ← home pose + action_scale × action
filters       ← first-order low-pass on head and legs
```

Two subtleties of the prototype are worth naming because they are easy to "fix" by accident.
**A kick window runs at standing tuning** — the kick's observation carries an all-zero command
and the standing transition fires on exactly that, so a kick runs at `standing_action_scale`
and the softened standing gain. Kept, because the kicks were tuned against it. **The sitstand
*rise* also runs at the standing gain** (its command is all-zero) while the *sit* does not (its
posture flag makes the twist magnitude 1). Same mechanism, same reason. One deliberate
divergence: the prototype tracks the standing action scale by saving and restoring
`action_scale` across transitions, which can leave a stale value behind after a sit→stand cycle
until the next walk; here scale and gain are recomputed from the active state every tick.

Policy files come from paths in the params file, defaulting into the release directory — so a
normal update carries the policy trained against the binary, and a dev points a path at their
own `.onnx` and iterates without cutting a release.

*Which* file fills a slot — the release's copy, a component from the Hub, or something a dev
dropped on the board — is [`policy-channel-design.md`](policy-channel-design.md)'s, along with
the commands that change it and what happens when a load fails. This section owns what a policy
is and how it is validated and run, and stops there.

Everything is validated at **load**, not at inference: observation width, action count, and
whether ONNX Runtime is present at all. Every net must be 61-input, 14-output, checked at load
rather than discovered mid-stride. The runtime also ships a 51-D family using the legacy
3-value command; those load only under its `--new-cmd-obs=false` path, and `robotd` refuses
them with `observation width is 51, expected 61`. A warm-up inference runs before the loop
starts, which both pays the first-call cost off the hot path — where it would look identical to
a missed deadline — and proves the dylib resolved.

**`ort` panics when ONNX Runtime is missing.** It `expect`s inside `setup_api`, on a lazy path
reachable from any API call, so it cannot be caught as an error. Left alone that killed the
control thread: no tick ever landed and health reported "the loop has not completed a cycle"
forever, so the daemon looked wedged rather than naming the cause — worse than the crashloop
this design rejected. `policy::ensure_runtime` therefore probes for the dylib with the same
loader and search rule `ort` uses, before `ort` is touched, so a missing library becomes an
ordinary error.

**`policy.enabled` separates "no policy wanted" from "policy broken".** The first is healthy
and is the right configuration for bench updater testing; the second is unhealthy, so the
updater rolls the release back. Collapsing them would either make a bench robot look broken or
let an unusable bundle pass the gate. `robotd --no-policy` sets it, and the gate tests use it,
since neither CI nor a laptop has ONNX Runtime installed.

ONNX Runtime is a **board prerequisite**, installed by `scripts/install.sh`, not shipped in the
release. It changes far less often than the daemon, and ~20 MB in every artifact would enlarge
every update for nothing. The trade is that a board missing it installs and starts fine and
then cannot walk — which is why health reports the searched path.

Not done, and deliberately: pre-binding the ONNX input/output tensors. The current path
allocates a 61-float vector per inference, ~244 bytes at 50 Hz. Worth measuring on the board
before optimising.

**Carried over from the runtime because it works** — head and leg low-pass filters, action
scale, voltage-adaptive scaling, the standing-transition gain change. These are tunables
(§4.2), not decisions to revisit. The low-pass alphas in particular are the values the alpha
policies are *trained* with, so they must match training or transfer degrades. The rule is not
to regress what already runs.

### 2.4 Safety

`safety` owns the only `RobotIo` write handle. No policy and no client has one, so nothing
above it *can* command a motor — the invariant is structural rather than remembered. Same
argument the updater makes about its recovery path: code that only runs once something has
already gone wrong is the code most likely to be quietly broken, so make the broken state
unrepresentable instead.

Two rules, unconditional:

- **Non-finite refusal.** A `NaN` target is not clamped, it is refused outright.
- **Range clamp.** Targets are held inside the actuator's travel, every tick, whatever the
  policy asked. This is the *actuator's* range, not a per-joint anatomical limit — it catches
  `NaN`, an absurd action scale and a garbage tensor; it will not stop a joint being driven
  somewhere mechanically unwise (§9.3).

Plus a deadman on the command: if intents stop arriving, the velocity goes to zero. **Stop is
not limp**, and the distinction matters — losing comms makes the robot *stand still*, because
standing is the safe state for a biped; losing balance is a different event, and this layer
does not answer it.

**The fall verdict is a report, not a rule.** It is computed every tick — projected gravity in
the trunk frame, debounced 0.2 s so a firm footfall is not a fall — and published, and it
**gates nothing**: a fallen robot is enabled, init'd, driven and sent skills exactly like an
upright one. That is deliberate. Being on the floor is precisely when someone needs those
calls to work, and a robot that yields the moment gravity misreads a lean is a robot that
keeps sitting down while someone handles it. It is also what lets the answer to a fall live
above this layer with no exemption to special-case: earlier revisions had a `fall_limp` gate
and a `fall_recover` auto-stand-up here, and both were removed, because a safety rule that
recovery has to bypass in order to work is not one.

#### 2.4.1 Falling is a third event

The fall verdict above answers "is the robot down". That is the right question for reporting
a robot lying on its side, and the wrong one for softening a landing: gravity past
`fall_gravity_z` held for 200 ms *is* the robot on the floor, and the window worth acting in
has closed by then.

So `limp_fall` (on by default since it was validated on a robot) runs a second, separate
detector — `duck_control::fall` — on the rate rather than the position. Projected gravity
rotates with the trunk, so `ġ = −ω × g` is exact and comes straight from the gyro in the same
12-byte IMU block; extrapolating it over ~0.3 s says where gravity is heading. It fires when
the robot is already tilted (≈26°), still tipping over rather than recovering, and predicted
past the fall threshold — debounced three ticks. Differentiating the SFLP quaternion instead
would add the filter's lag to the one number whose whole value is being early.

What it buys is not the landing itself but the stand-up after it. The standing policy gets a
still robot in a known posture up cleanly and a thrashing one up only after several attempts
at walking gain against the floor, which is where the load on the motors comes from. So the
sequence takes the fall away from the policy: limp at `gain_limp` following the joints down,
wait for the gyro to go quiet, ramp back to the standing pose over ~1 s, hand over. The
hand-back is nothing more than that — the twist has been held at zero throughout, so command
magnitude selects the standing network, and that is the stand-up.

It runs off the policy's path by construction: `driving` is false for the whole sequence
(§1.4), so the targets come from the sequence rather than the controller, and they reach the
motors through `apply` like anything else — no exemption, no back door. Nothing in `safety`
needs telling, precisely because the verdict gates nothing: the pose ramp moves a robot lying
on the floor like any other tick.

The tuning is the feature, and it is asymmetric: a false positive is a fall the robot
*caused*, which is worse than the stiff landing it was trying to avoid. The defaults sit
deliberately on the late side.

A spent pack is the other thing that moves the robot without being asked: with
`safety.battery_empty_shutdown` (on by default), reaching the empty floor on the smoothed
voltage sits the robot down and powers the board off. The EMA moves over ~10 s, so a load sag
cannot trip it — reaching 6.6 V requires a genuinely empty pack.

### 2.5 The robot model

Rust consts for alpha only: 15 joints, Dynamixel IDs `20–24 / 30–34 / 10–14`, names,
`DEFAULT_POSITION`, and the actuator range. Same tables as the runtime's `motor.rs`, minus the
three dead variants. There is exactly one robot; a second revision can be a second table.

Two of those tables are load-bearing beyond their own crate. `JOINT_NAMES` comes *from*
`duck-ipc-proto`, because the wire indexes `joints` and `targets` positionally and the two
orders cannot be allowed to drift — a `const` assertion makes "cannot" true. And
`DEFAULT_POSITION` must match `HOME_FRAME` in the training env: a policy observes joint
positions *relative* to the home pose, so a discrepancy here is a constant offset on 14
observation slots.

## 3. The API

### 3.1 Intents in

Two vocabularies — intents in, state out — and JSON-RPC's two message families map onto them
exactly:

```jsonc
// continuous: notifications, no id, no reply, last-writer-wins
{"jsonrpc":"2.0","method":"robot.move","params":{"vx":0.2,"vy":0.0,"vyaw":0.4}}
{"jsonrpc":"2.0","method":"robot.head","params":{"neck_pitch":0.35,"head_pitch":0.35,
                                                 "head_yaw":0.0,"head_roll":0.0}}

// discrete: requests, answered
{"jsonrpc":"2.0","id":7,"method":"robot.stop"}
{"jsonrpc":"2.0","id":8,"method":"robot.enable","params":{"on":true}}
```

Everything is radians, trunk frame, right-handed, signs fixed in the protocol definition. The
runtime carries `--laser-track-yaw-sign`, `--laser-track-pitch-sign`, `--laser-fk-pitch-sign`,
`--laser-fk-neck-sign` and `--imu-z-rotation-deg` because that convention was never written
down and each consumer rediscovered it empirically. Writing it into the protocol deletes the
category.

At 50 Hz, continuous intents as notifications means no response traffic. When they later
travel over WebRTC, notifications route to the unreliable `teleop` channel and requests to the
reliable `control` one — which is what `architecture.md` §5.2 asks for, falling out of the
message family rather than a rule anyone has to remember.

**Twist and head are separate slots on purpose.** A single combined slot would need
read-modify-write to update one field, and two clients — a gamepad driving the body and
something else driving the head — would silently lose each other's updates. Separate slots
make each one single-writer in practice, so last-writer-wins means what it says. Every slot is
stamped, because the loop's real question is never "what is the value" but "how old is it";
that is what the deadman reads.

`look` (gaze direction) is deferred; both gaze forms will be exposed, and arbitration between
them is last-writer-wins with no blending.

### 3.2 State out

One stream, subscribable, decimated per subscriber. It must report what was **refused**, not
just what happened — a teleop UI showing the stick forward and the robot still, with no
explanation, is unusable, and safety clamps things constantly:

```jsonc
{"method":"robot.state","params":{
  "t":1234.567,
  "move":{"requested":[0.4,0,0],"applied":[0.15,0,0],"limited_by":["max_velocity"]},
  "policy":"walk", "safety":{"fallen":false,"limp":false},
  "loop":{"hz":49.8,"missed":0},
  "battery":{"volts":7.62,"percent":64}
}}
```

**Battery carries both volts and percent**, here and in `robot.health`. The mapping — 6.6 V
empty, 8.2 V full under load, an NP-F550 — lives in `duck_control::model::battery_percent` and
travels already applied. The prototype sent volts only and the app re-derived the percentage
from constants of its own, which is how the same pack shows two different numbers on two
screens. A client drawing a battery pill should not have to know which pack this robot ships
with. There is no fuel gauge: the measurement is the servos' own supply voltage (§2.1), so it
sags under load and recovers at rest.

Same payload for `robotctl monitor` and, later, the app. This is what replaces the runtime's
180-byte frame on 9870, the JPEG stream on 9871, the UDP command socket on 9872, the maploc
ports on 9874/9875 and the web hub's `/state.json`. Adding a field today means editing four
places that can silently disagree; here it means one struct, and older clients ignore what they
do not know.

`robot.subscribe` turns a connection into a stream; the loop publishes into a bounded broadcast
and never waits on a subscriber, so a slow client gets a gap rather than applying backpressure
— the rule the updater already uses for progress. Decimation is server-side and per-subscriber,
so a 10 Hz dashboard genuinely costs the robot less than a 50 Hz digital twin.

**The acknowledgement names the policy.** `robot.subscribe` answers with `SubscribeResult`:
which networks this process was configured with, by file name, plus a sentence when nothing is
driving — disabled in params, or wanted and unloadable. That belongs in the handshake rather
than the frame because it cannot change while the process lives, and `policy` on the frame
answers a different question: which net drove *this tick*. Two releases with different gaits
both report `walk`, and "which network is this?" is the first thing anyone comparing them asks.
Putting it on the frame instead would allocate two strings per tick on the control thread for
an answer that never differs.

Two details that are easy to get wrong. **Nothing is assembled when nobody is subscribed** —
which is the normal state of a robot — because building a frame allocates on the thread that
should not be visiting the allocator without reason. And the limit names are **spelled out for
the wire** rather than derived from the Rust enum, so renaming a variant cannot silently break
a client branching on `limited_by`.

### 3.3 Bring-up: `enable`, `init`, `relax`

**`robotd` never moves the robot on its own.** On start it reads current positions, adopts them
as targets, and does not touch torque. Dynamixels hold their last commanded goal while the
process is dead, so a restart leaves the pose unchanged and there is no gap — the robot stands
through an update without noticing. Interpolating to the default pose on start would make every
update restart move a standing robot: a fall risk, and a confounder when the thing under test
is the updater.

`robot.enable` used to flip a flag and nothing else. Torque came from `robotd init`, a separate
subcommand that opens the motor bus itself — so it needed the daemon stopped, it appeared in no
documentation, and pressing Start on a fresh robot did nothing visible: the policy ran, the loop
wrote positions, and the servos ignored them. So the loop has a bring-up state, and an explicit
`robot.enable` is what advances it:

```
Limp ──enable (policy loaded, a fresh sample)──▶ Homing (torque on, 2 s ramp) ──▶ Ready ──▶ policy drives
```

**The invariant the old rule protected is unchanged: nothing here happens because a process
started.** A `robotd` restarted by an update finds `Limp`, asks for no torque, and leaves a
standing robot standing — `a_restart_asks_for_no_torque` asserts exactly that, on the absence of
any write rather than on a write of `false`. What changed is that "never touch torque" was a
broader rule than the property it was defending, and it put a manual step in front of every
drive.

Two conditions gate the bring-up, each for its own reason:

- **A loaded policy.** `enable` means "enable the policy"; powering the joints to run one that
  is disabled or would not load would stand a robot up on a broken release and then hold it.
- **A fresh sample.** The ramp starts from where the joints are. Starting from a position nobody
  read is the lurch the ramp exists to avoid.

**Being down is not one of them.** An earlier revision refused there, back when `apply` held a
fallen robot at limp gain and a ramp would have been writing a stand-up that could not happen;
it went when the verdict stopped gating anything (§2.4). Start on a robot lying on the floor is
exactly how someone asks it to stand back up — the ramp runs like any other, and the standing
policy takes it from there.

Torque is *not* dropped when the policy is disabled again: the robot holds its pose, which is
what "a standing robot stays standing" means on this side too.

`robot.init` and `robot.relax` are the same two transitions, asked for directly — because "stand
up" and "let go" are decisions of their own, and until now the first was a subcommand that opens
the motor bus itself (§1.1) and the second did not exist at all. Both are served by `robotd`, so
neither needs the daemon stopped and neither can write to the bus while the control loop is doing
the same. `init` deliberately needs no policy: standing up is reasonable to ask of a robot with no
walking network, and it is what makes the bring-up testable at all, since CI has no ONNX Runtime.

They arrive as a *request* the loop takes once per tick rather than a flag it keeps applying: one
`set_torque` is a bus transaction per joint, so a level would put sixteen writes into every tick.
The later request replaces an unread earlier one — asked to stand up and then to let go within
20 ms, the second is what was meant. And `relax` clears `enabled`, or the next tick would see a
robot that was asked to drive and stand it straight back up.

Neither is reachable over BLE. A phone button that drops the robot on the floor is not one to
offer, and standing up moves every joint at once, which wants whoever asked to be looking at the
robot.

### 3.4 Health, and what may reach the verdict

| method | answer |
|---|---|
| `robot.health` | **the loop is meeting its deadline** — from achieved rate and missed-deadline count — plus a description of the robot the verdict never consults: loop, bus, IMU, battery, servo and board temperature |
| `robot.safeToRestart` | false while the policy is enabled and the robot is moving |
| `robot.modelApi` | constant |
| `robot.remoteSessionActive` | `false` — `mediad` owns the real answer |

Health is computed by the IPC side from atomics the loop publishes — a last-tick timestamp plus
counters — never by asking the loop. That is what lets a *wedged* loop report itself unhealthy
instead of hanging the caller.

A loop running at 60% of target is alive, answers every request, and is badly broken. Making that
distinction real is why the control loop was built before anything that walks (§5.1).

**What may and may not reach the verdict.** `healthy` and `degraded` are the update system's
inputs, so only conditions a *release* can be blamed for may set them — that is what `degraded`
already exists to enforce for an unpowered bench board. Everything else on the answer is a
**description**, and no automatic decision may read it: battery, motor temperature, and the
loop/bus/IMU counters. Gating on the battery would mean a robot updated on a low pack rolls the
release back, then judges its replacement on the same low pack, and cannot be updated at all until
someone works out why. Motor temperature would do the same on a hot afternoon.

**Why they travel together anyway.** One method, because the question arrives once: a robot
behaving oddly gets asked "what is going on", and a verdict without the numbers behind it just
starts a second round of questions. The loop section carries the very figures the verdict was
computed from, so `unhealthy: control loop at 43.9 Hz` can be read next to `missed = 0` — which
distinguishes a loop being woken late from a loop doing too much, and those have different fixes.
`robotctl health` adds the software half from `updaterd` and prints both.

`safeToRestart` is false while the policy is enabled and the robot is moving: restarting motor
control mid-stride is how a robot falls over (`updater-design.md` §7.2).

### 3.5 Maintenance is a separate namespace

`init`, emergency torque-off, calibration and raw joint writes are not intents. They live in their
own namespace so the relay's per-transport allow-list can keep them off remote transports.
Signaling gating decides *who connects*; it does not say a teleoperator is also a mechanic — and
`update.*` reaching a DataChannel would mean a remote peer can trigger a rollback.

## 4. Around the loop

### 4.1 The loop reads snapshots, never waits

Intents and params are published by IPC threads and read by the loop as a single atomic load.
Nothing can apply backpressure to the loop and no request enters it synchronously. Telemetry goes
out through a bounded broadcast where a slow subscriber gets a gap — the pattern the updater's IPC
layer already uses and documents.

```text
   IPC tasks (tokio, multi-thread)        control thread (own runtime, 50 Hz)
   ═══════════════════════════════        ═══════════════════════════════════

     robot.move  ──► ┌────────────┐
     robot.head  ──► │intent slots│ ──atomic load, once per tick──►  read
                     │ twist│head │
                     └────────────┘
                     ArcSwap, stamped

     robot.init  ──► ┌────────────┐
     robot.relax ──► │power req.  │ ──taken once per tick──────────►  bring-up
     skills      ──► │skill flags │
                     └────────────┘
                     last request wins

     robot.health ◄── ┌──────────┐ ◄────────── publish ───────────  atomics
     safeToRestart    │ atomics  │              ticks, hz, missed,
                      └──────────┘              fallen, moving

     robot.state  ◄── ┌──────────┐ ◄─── send, only if subscribed ──  frame
                      │broadcast │
                      └──────────┘
                      bounded, drop-on-lag
```

**No channel runs the other way.** Health is *published*, never asked for, which is what lets a
wedged loop report itself unhealthy instead of hanging the caller.

The skill flags are booleans rather than a queue: within one 20 ms tick a second press of the same
button means nothing extra, while two *different* requests both deserve to be seen — which a
single last-writer slot would lose.

### 4.2 Params

A TOML file read at startup, **not watched** — live reload comes later. It lives outside
`releases/<ver>/` so it survives update *and* rollback, next to the updater's own config at
`/etc/robot/robotd.toml`.

Belonging to the board rather than the release is what makes a hand-edited policy path stick: the
defaults point inside `releases/<ver>/`, so an ordinary update keeps a policy alongside the
binaries trained against it, and deleting the override goes back to that. The file may be absent
entirely — an unprovisioned board comes up on the built-in defaults rather than refusing to start,
which is far easier to diagnose remotely than a daemon that will not run. The corollary, learned
the hard way: an *uncommented* value is frozen on that board forever while releases move on, which
is how a fleet ended up standing at kP 120 while the release default said 160.

Roughly ten values, not 142: control rate, gains, action scale, low-pass alphas, max velocities,
deadman timeout, policy paths and the update-gate thresholds. The flag explosion in the runtime was
mostly variants, dead skills and dead sensors, all of which are gone.

One switch is coarser than the rest. `policy.mode` — `walk` or `roller` — selects which policies
load *and* the tuning defaults, so every unset field resolves per mode and moving a robot onto
wheels is one line plus a restart. It is a preset, not a variant: the roller line is the
prototype's, rebased on the alpha defaults.

### 4.3 The gamepad is a client

`padd` reads `gilrs` and sends intents over `robotd`'s socket. Its own crate, so a gamepad stack
stays out of `robotctl` — the tool that has to work on a broken robot.

One socket hop, tens of microseconds. What it buys: the input path used by the app, the SDK and any
remote client is the one a developer exercises every day, so it cannot quietly rot. For dev,
`ssh -L /tmp/robotd.sock:/run/robotd.sock` gives pad-on-laptop, robot-on-board with no code.

On the robot it is `padd.service`, running from boot and driving whatever pad connects — safe with
no pad, because it sends nothing and the deadman holds the robot. It stays **unprivileged**, which
is the load-bearing part of "the gamepad is a client": its `input` and `robot` group membership is
all it has. Pairing a pad therefore belongs to `configd` (`architecture.md` §1), not here — bonding
a device needs root and BlueZ, and a `padd` holding either would no longer be exercising the API
the app will use.

### 4.4 Odometry, on the sample the loop already took

`odometry::Odometry::alpha()` is stepped once per tick from the joint positions and the IMU
quaternion the loop has just read, and its estimate goes out on the state stream as
`odom: { position, yaw }`. `robotctl monitor` draws it as a path map under the 3D view.

**Contact-based**, ported from the prototype runtime: one sole corner is the ground anchor, the
trunk's world pose follows from it by forward kinematics through the `kinematics` crate's model,
and the anchor moves when another corner drops below it — so a step never makes the estimate jump.
Heading is the IMU's integrated yaw, so the world frame is wherever the robot was looking at boot.
There is no magnetometer and nothing corrects drift; this is relative motion, and every consumer
has to treat it that way.

It costs the loop two chain evaluations per tick and no extra bus traffic, which is why it runs at
the loop's own rate rather than the prototype's separate 100 Hz. That cost is the reason the
"no odometry" decision in §7 was reversed rather than re-argued: the objection was never the
arithmetic, it was that nothing read the answer.

### 4.5 What else the loop drives, and why none of it has a design page

`robotd` grew four subsystems after slice 2 that are not control and not safety. They share a
shape: each hangs off the tick, none may block it, and none can reach the bus except through the
intents the loop already arbitrates.

| in the process | what it is | where the reasoning lives |
|---|---|---|
| `sound.rs` | the voice at play time — one `aplay` child, and a new sound kills the old one, because the codec's PCM is exclusive | the module header |
| `theremin.rs` | depth from `tofd` at 15 Hz → a note, a mouth opening, and a line of state, sampled by the 50 Hz loop and never waited on | the module header |
| `chorale.rs` | several ducks singing one piece: the lowest id conducts, the conductor owns the seating, `btd` carries the beacons and does no thinking | the module header |
| `pet-detect/` | a ~20 KB CNN over a 40-band log-mel window from the onboard mic, in its own worker | the crate header |

Plus `soc.rs`, which reads the board's own thermal zones out of `sysfs` — not behind `RobotIo`,
because it has to keep answering when the motor bus does not.

**None of them has a design page, and that is the rule working rather than a gap.** A service earns
one when a second reader would otherwise have to derive its contract from the code
(`../README.md`). These have exactly one implementation and one consumer each, and their decisions
are local enough to live in the module header next to what they constrain. What they need instead
is to be *operable*, and that is the cheat sheet's job: [the voice], [the chorale], [the theremin].

[the voice]: ../robot/cheatsheet.md#the-voice
[the chorale]: ../robot/cheatsheet.md#the-duck-chorale
[the theremin]: ../robot/cheatsheet.md#play-the-duck-the-tof-theremin

## 5. Why it exists, and where it came from

### 5.1 The goal, which is not "a good `robotd`"

Two things were wanted, and the second is the one that reordered the work:

1. **Iterate fast on the control core.**
2. **Actually test the updater.**

The update engine was finished and had never run on hardware. Its most important paths were
therefore unproven: `systemctl restart` in `on_apply` had never met real systemd, the 30 s
health-gate timeout was an admitted guess, and — worst — **auto-rollback is only meaningful if
`robot.health` means something.** It used to mean "the control loop ticked once", so every rollback
tested so far had been tested against a placeholder.

That is why the first increment did not walk. It existed to be a truthful health signal on a real
board (§3.4).

### 5.2 What `robotd` replaces

`robotd` replaces the runtime, but not in one step — the runtime does five separable jobs and only
one of them is `robotd`'s:

| runtime job | destination | when |
|---|---|---|
| control loop, policies, motors, IMU | `robotd` | done |
| gamepad | an intent client (`padd`) | done |
| camera, ball/laser/pet detection, JPEG | `mediad` | M5 |
| web hub, PWA, brain command socket | `mediad` / the app | M5+ |
| maploc — mapping, MCL, planning | unowned | — |

So the two run side by side for a while. They cannot run *simultaneously* — one serial bus, one
owner (§1.1) — so a board is running one or the other, and the systemd units say so with
`Conflicts=`.

The remote/app layer is not designed here. It is the `reachy_mini` architecture — WebRTC for media,
JSON-RPC 2.0 over the DataChannel for control — ported to Rust, and out of scope for this document.
Its one requirement on `robotd` is in §3.5.

### 5.3 The two slices, and what "done" meant

**Slice 1 — hold the pose.** The tick, the bus, the model, `RobotIo`, and honest health. Nothing
computed anything: `held_pose` was a constant adopted at startup. That was the point — it put the
real load on the bus at the real rate, so loop timing and health were honest, and nothing fell over
when a deliberately broken release landed. You could hammer install / rollback / power-cut cycles at
a bench all day. Done when: `robotd` held the pose for an hour with no bus errors; `robotctl update
apply` installed a release, restarted it, passed the gate, and the robot did not move; a release
built to come up unhealthy was automatically rolled back; and a power cut mid-update recovered via
the boot counter.

**Slice 2 — walk and stand.** Observations, the ONNX policy, the safety layer, intents, and the
gamepad client. Done when: it walks on a board, driven through the intent API; an update applied
with `robotctl` restarts it cleanly with the gate passing; and `--unhealthy` still rolls back.

Everything since — the skill chain, the bring-up state machine, the roller preset, limp-fall —
arrived on top of that shape rather than changing it.

### 5.4 Not regressing is the acceptance criterion

The measurement already exists: `bench_dynamixel_bus` reports achieved rate, jitter, read time, bus
time, utilisation, errors and IMU sample freshness at 50 and 100 Hz. Record today's numbers as the
baseline; `robotd` must match them. This is deliberately not an RT engineering project — no
`SCHED_FIFO`, no pinning, no `mlockall`. The loop is reliable today and the job is to keep it that
way while the code around it gets simpler.

## 6. Testing

`FakeIo` with scripted samples, no hardware:

- health goes false when deadlines are missed;
- startup adopts the current pose and never commands motion;
- safety refuses a non-finite target and clamps one past the actuator's range, and a fall
  preempts neither the policy nor the caller's gain;
- the limp-fall predictor fires on a fall and not on a footfall or a static tilt, and its pose
  ramp ends at the standing pose;
- deadman zeroes velocity when intents stop;
- **golden observation vectors** — `(inputs, expected 61-float array)` pairs exported from mjlab and
  committed. A wrong index in the observation does not fail loudly; it produces a plausible robot
  that falls over. Depends on an export from `microduck_brain` (§9.2).

Each test's comment says which failure it exists to prevent, per the repo convention.

## 7. Decisions recorded

| | |
|---|---|
| `duck-control` as a workspace crate | boundary enforced by the compiler, no second repo |
| bus layer written fresh, constants borrowed | thin code, but the tuned numbers are not re-derived |
| the IMU in the motors' `sync_read` | it is a device on the same bus; no IMU abstraction |
| Rust consts for the model | one robot exists |
| params file, not watched | establishes the file and its location; the watcher is later |
| policy path in params, default = release dir | updates carry the policy; devs override it |
| adopt current pose on start | an update must not move a standing robot |
| bring-up as a state machine, not a flag | `set_torque` is a transaction per joint |
| the fall verdict reports, it does not gate | what to do about a fall is a control decision (§2.4) |
| ~~no odometry~~ — reversed | `monitor`'s path map reads it, and it is one `kinematics` pass on a sample the loop already took (§4.4) |
| the priority chain keeps the runtime's shape | the skills were tuned against its quirks |
| gamepad as its own crate | keeps `gilrs` out of the recovery CLI |
| sim after slice 2 | hardware is the validation path; `FakeIo` covers laptop development |

## 8. Deferred, deliberately

MuJoCo backend and the `RemoteIo` protocol · the skill abstraction · policy bundle manifests and
`model_api` gating · `look`/`pose`/`do` intents · gaze IK · live params reload and the config store ·
thermal limits · rate limits · per-device IMU calibration.

**Odometry has left this list.** It was deferred because nothing read it; `robotctl monitor`'s
path map now does. §4.4.

## 9. Open

1. **Control rate on the Radxa.** 50 Hz is inherited from a Pi Zero 2W. Measurable now that boards
   exist — `bench_dynamixel_bus` and the loop's own five-minute summary report the same figures.
2. **Golden vectors** would be worth having from `microduck_brain` — as a regression check against
   the training env rather than as the source of truth they were going to be. Not a prerequisite for
   anything shipped.
3. **Per-joint limits do not exist.** Safety clamps to the *actuator's* travel, which catches `NaN`,
   a bad action scale and a garbage tensor — it will not stop a joint being driven somewhere
   mechanically unwise. The real limits are in the alpha MJCF (31 KB), not vendored here. A limit
   that looked anatomical but was not would imply protection nobody has.
4. **Where the alpha MJCF lives** if a sim/real agreement test is ever wanted — 31 KB of XML, 19 MB
   of meshes, currently in the runtime's `scripts/alpha_assets/`.
5. **The standing cost of a C dependency.** `gilrs` pulls `libudev-sys` unconditionally on Linux, so
   CI and the board cross-build install it. The same expense recurs for the next C dependency that
   has to reach the board, so prefer pure-Rust crates on that path. *Unverified on macOS:* the
   cross-build needs an aarch64 sysroot, which a Mac cannot provide, so `cargo board --bins` fails
   locally there — build the shipped set with `-p updater -p robotd -p robotctl`, or build on Linux.
