# `robotd` — design, slices 1 and 2

Status: draft · Date: 2026-07-29 · Owner: pierre

Implements the `robotd` row of [`architecture.md`](architecture.md) §1 and covers
[`roadmap.md`](roadmap.md) M3. Scoped deliberately to the first two increments; everything
beyond them is in §10.

The prototype being absorbed is
[`apirrone/microduck_runtime`](https://github.com/apirrone/microduck_runtime), referred to
throughout as *the runtime*.

## 1. The goal, which is not "a good `robotd`"

Two things are wanted, and the second is the one that reorders the work:

1. **Iterate fast on the control core.**
2. **Actually test the updater.**

The update engine is finished and has never run on hardware. Its most important paths are
therefore unproven: `systemctl restart` in `on_apply` has never met real systemd, the 30 s
health-gate timeout is an admitted guess, and — worst — **auto-rollback is only meaningful
if `robot.health` means something.** Today it means "the control loop ticked once", so
every rollback tested so far has been tested against a placeholder.

That is why slice 1 does not walk. It exists to be a truthful health signal on a real
board.

## 2. What `robotd` replaces, and over what span

`robotd` replaces the runtime, but not in one step — the runtime does five separable jobs
and only one of them is `robotd`'s:

| runtime job | destination | when |
|---|---|---|
| control loop, policies, motors, IMU | `robotd` | now |
| gamepad | an intent client | slice 2 |
| camera, ball/laser/pet detection, JPEG | `mediad` | M5 |
| web hub, PWA, brain command socket | `mediad` / the app | M5+ |
| maploc — mapping, MCL, planning | unowned | — |

So the two run side by side for a while. They cannot run *simultaneously* — one serial bus,
one owner — so a board is running one or the other, and the systemd units should say so
with `Conflicts=`.

The remote/app layer is not designed here. It is the `reachy_mini` architecture — WebRTC
for media, JSON-RPC 2.0 over the DataChannel for control — ported to Rust, and out of scope
for this document. Its one requirement on `robotd` is in §7.4.

**Only the alpha variant, only the Radxa, only the v2 `imu_to_dxl` board.** v1/v1.5/v1.6,
the four other IMUs, the three cameras, the Pi, and the wheeled configuration are all
dropped. Every shipped policy is already `alpha_*`.

## 3. Crate layout

```
duck-ipc-proto/  wire contract                                  (exists)
duck-control/    robot model · bus · RobotIo · obs · policy · safety   NEW
updater/         engine + updaterd                              (exists)
robotd/          the daemon process                             (skeleton exists)
robotctl/        CLI                                            (exists)
padd/            gamepad → intents                              NEW, slice 2
xtask/ test-support/                                            (exist)
```

`duck-control` holds everything between reading the bus and writing it. `robotd` is the
process around it: socket, JSON-RPC, systemd, health reporting. The compiler enforces that
boundary, which is what stops daemon concerns leaking into control code — and it means the
crate can be lifted into its own repo later if the runtime needs to consume it during the
transition, without that being a rewrite.

(`padd` is a placeholder name.)

## 4. Slice 1 — hold the pose

A daemon that drives the real bus at the real rate and tells the truth about itself. No
observations, no ONNX, no intents, no safety layer.

### 4.1 The tick

50 Hz, one `tokio` task on its own runtime so IPC work cannot sit in front of it:

```
read()               one sync_read: IMU board + 15 motors, one transaction
publish(state)       atomics for health; snapshot for telemetry
write(held_pose)     sync_write goal positions
```

`held_pose` is a constant, adopted at startup (§4.4). Nothing computes anything. That is
the point: it puts the real load on the bus at the real rate, so loop timing and health are
honest, and **nothing falls over when a deliberately broken release lands.** You can hammer
install / rollback / power-cut cycles at a bench all day. With a walking policy you would
spend the day picking the robot up instead.

The loop stays a `tokio` task with `interval`. It is not being made real-time. Two changes
from the runtime, both small: `MissedTickBehavior::Delay` rather than `Burst` — after an
overrun, burst fires the backlog back to back and stacks motor commands on top of each
other — and moving perception out to `mediad`, which removes most of what competes with the
loop today for free.

### 4.2 The bus

A thin layer over `rustypot`: open, one combined `sync_read`, `sync_write` goal positions,
torque enable, and the startup register check. Roughly what `bench_dynamixel_bus` already
exercises.

Written fresh rather than lifted, but **the numbers are borrowed from the runtime**, each
with a comment saying so:

- `RADS_PER_SEC_PER_COUNT = 0.229 × 2π/60`, and the position count↔radian conversion.
- The register expectations from `check_and_fix_config`, asserted and corrected at startup:
  `return_delay_time=0`, `baud_rate=3`, `pwm_slope=255`, `shutdown=52`. The first is
  load-bearing — at the XL330 default of 250, sixteen devices cost ~8 ms of pure bus
  turnaround per tick, 40% of the budget.

The IMU is folded into the same `sync_read` as the motors, because that is what the
hardware does — the v2 board sits on the Dynamixel bus and ships an on-chip SFLP quaternion,
so the host only decodes. One board, one code path, no IMU abstraction.

### 4.3 `RobotIo`

```rust
trait RobotIo {
    fn read(&mut self) -> Result<Sensors>;              // joints + IMU, one sample
    fn write(&mut self, targets: &JointTargets) -> Result<()>;
}
```

Two implementations: `DynamixelIo` and `FakeIo` (scripted samples). `FakeIo` is what the
test suite runs against, and it is why `cargo test` still needs no hardware, no network and
no Docker. A MuJoCo backend comes after slice 2.

**Neither is `cfg`-gated off macOS**, contrary to an earlier draft of this document. The
gate was meant to keep `serialport` out of a laptop's dependency tree, but `rustypot` and
`serialport` both build cleanly there, so it bought nothing and cost the ability to
type-check the bus layer without a board — which is exactly the code most likely to be
edited by someone who does not have one. Only the entry points that open a real port are
gated, so a Mac build still refuses to pretend it has a robot: `robotd --fake` is the
laptop path, and it must be asked for explicitly rather than fallen back to.

### 4.4 Startup, and what an update restart does to the robot

**`robotd` never moves the robot on its own.** On start it reads current positions, adopts
them as targets, and does not touch torque.

This is what makes update testing safe on a robot that is standing up. Dynamixels hold
their last commanded goal while the process is dead, so a restart leaves the pose unchanged
and there is no gap — the robot stands through an update without noticing. Interpolating to
the default pose on start, which is what `init` does today, would make every update restart
move a standing robot: a fall risk, and a confounder when the thing under test is the
updater.

Moving to the default pose is therefore explicit — `robotctl init`, in the maintenance
namespace (§7.4).

### 4.5 What `updaterd` finally gets

| method | slice 1 |
|---|---|
| `robot.health` | **the loop is meeting its deadline** — from achieved rate and missed-deadline count |
| `robot.safeToRestart` | `true` (a constant pose is always safe to interrupt) |
| `robot.modelApi` | unchanged constant |
| `robot.remoteSessionActive` | `false` — `mediad` owns the real answer |

Health is computed by the IPC side from atomics the loop publishes — a last-tick timestamp
plus counters — never by asking the loop. That is what lets a *wedged* loop report itself
unhealthy instead of hanging the caller, and it is the property the existing skeleton was
already built around.

A loop running at 60% of target is alive, answers every request, and is badly broken. That
distinction is the whole reason slice 1 exists.

### 4.6 Done when

On a board: `robotd` holds the pose for an hour with no bus errors; `robotctl update apply`
installs a release, restarts it, passes the gate, and the robot does not move; a release
built to come up unhealthy is **automatically rolled back**; and a power cut mid-update
recovers via the boot counter. That last three are the updater's real first test.

## 5. Slice 2 — walk and stand

### 5.1 What it adds

Observations, the ONNX policy, the walk/stand pair, the safety layer, intents, and the
gamepad client.

### 5.2 One observation builder

Every alpha policy is `obs[1,61] → actions[1,14]` — verified across walking, standing,
ground pick, ball kick and sit. So there is exactly one layout:

```
[ gyro(3) | projected_gravity(3) | joint_pos(14) | joint_vel(14) | last_action(14) | command(13) ]
                                                                    command = vel(3) + head(4) + body(6)
```

Joints exclude the mouth throughout; actions map back into 15 motor slots with index 9 left
at zero. The 51/54D legacy, 49D wheeled and 85D tracking layouts go away with the variants.

The command block, which was the only part in doubt, is now settled — read out of the
prototype's `control_step` rather than guessed:

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

### 5.3 The policy stays shaped like Antoine's

No skill abstraction. Lift `policy.rs`'s main-plus-standing path: two ONNX sessions, and
`command_magnitude ≤ 0.05` selects standing. Drop the other six modes and the priority
if-chain with them. The abstraction can arrive when the third skill does.

Policy files come from a path in the params file, defaulting into the release directory — so
a normal update carries the policy, and a dev points the path at their own `.onnx` and
iterates without cutting a release.

Everything is validated at **load**, not at inference: observation width, action count, and
whether ONNX Runtime is present at all. A warm-up inference runs before the loop starts,
which both pays the first-call cost off the hot path — where it would look identical to a
missed deadline — and proves the dylib resolved.

**`ort` panics when ONNX Runtime is missing.** It `expect`s inside `setup_api`, on a lazy
path reachable from any API call, so it cannot be caught as an error. Left alone that killed
the control thread: no tick ever landed and health reported "the loop has not completed a
cycle" forever, so the daemon looked wedged rather than naming the cause — worse than the
crashloop this design rejected. `policy::ensure_runtime` therefore probes for the dylib with
the same loader and search rule `ort` uses, before `ort` is touched, so a missing library
becomes an ordinary error.

**`policy.enabled` separates "no policy wanted" from "policy broken."** The first is healthy
and is the right configuration for bench updater testing; the second is unhealthy so the
updater rolls the release back. Collapsing them would either make a bench robot look broken
or let an unusable bundle pass the gate. `robotd --no-policy` sets it, and the gate tests use
it, since neither CI nor a laptop has ONNX Runtime installed.

ONNX Runtime is a **board prerequisite**, installed by `scripts/install.sh`, not shipped in
the release. It changes far less often than the daemon, and ~20 MB in every artifact would
enlarge every update for nothing. The trade is that a board missing it installs and starts
fine and then cannot walk — which is why health reports the searched path.

Not done, and deliberately: pre-binding the ONNX input/output tensors. The current path
allocates a 61-float vector per inference, which is ~244 bytes at 50 Hz. Worth measuring on
the board before optimising.

**Carried over from the runtime because it works** — head and leg low-pass filters, action
scale, voltage-adaptive scaling, the standing-transition gain change. These are tunables
(§8), not decisions to revisit. The rule is not to regress what already runs.

### 5.4 Safety

`safety` owns the only `RobotIo` write handle. No policy and no client has one, so nothing
above it *can* command a motor — the invariant is structural rather than remembered.

Three rules, unconditional:

- **Joint clamp** — targets clipped to the model's range every tick, whatever the policy asked.
- **Fall → limp** — projected gravity in the body frame, debounced (0.2 s in the runtime).
  In the runtime this is `--fall-detect`, a flag, evaluated inline among the gamepad handling
  and *skipped while a scripted skill is active*. Here it is always on and preempts anything.
- **Intent deadman** — if intents stop arriving, velocity goes to zero.

**Stop is not limp**, and the distinction matters: losing comms makes the robot *stand
still*, because standing is the safe state for a biped. Losing balance makes it limp. Two
events, two responses, written down rather than inferred from whichever got implemented
first.

### 5.5 Intents

Two vocabularies — intents in, state out — and JSON-RPC's two message families map onto
them exactly:

```jsonc
// continuous: notifications, no id, no reply, last-writer-wins
{"jsonrpc":"2.0","method":"robot.move","params":{"vx":0.2,"vy":0.0,"vyaw":0.4}}
{"jsonrpc":"2.0","method":"robot.head","params":{"neck_pitch":0.35,"head_pitch":0.35,
                                                 "head_yaw":0.0,"head_roll":0.0}}

// discrete: requests, answered
{"jsonrpc":"2.0","id":7,"method":"robot.stop"}
{"jsonrpc":"2.0","id":8,"method":"robot.enable","params":{"on":true}}
```

That is the whole slice-2 surface. `look` (gaze direction), `pose` and `do` come later; both
gaze forms will be exposed, and arbitration between them is last-writer-wins with no
blending.

Everything is radians, trunk frame, right-handed, signs fixed in the protocol definition.
The runtime carries `--laser-track-yaw-sign`, `--laser-track-pitch-sign`,
`--laser-fk-pitch-sign`, `--laser-fk-neck-sign` and `--imu-z-rotation-deg` because that
convention was never written down and each consumer rediscovered it empirically. Writing it
into the protocol deletes the category.

At 50 Hz, continuous intents as notifications means no response traffic. When they later
travel over WebRTC, notifications route to the unreliable `teleop` channel and requests to
the reliable `control` one — which is what §5.2 of `architecture.md` asks for, falling out of
the message family rather than a rule anyone has to remember.

### 5.6 State out

One stream, subscribable, decimated per subscriber. It must report what was **refused**, not
just what happened — a teleop UI showing the stick forward and the robot still, with no
explanation, is unusable, and safety clamps things constantly:

```jsonc
{"method":"robot.state","params":{
  "t":1234.567,
  "move":{"requested":[0.4,0,0],"applied":[0.15,0,0],"limited_by":["max_velocity"]},
  "policy":"walk", "safety":{"fallen":false,"limp":false},
  "loop":{"hz":49.8,"missed":0}
}}
```

Same payload for `robotctl monitor` and, later, the app. This is what replaces the runtime's
180-byte frame on 9870, the JPEG stream on 9871, the UDP command socket on 9872, the maploc
ports on 9874/9875 and the web hub's `/state.json`. Adding a field today means editing four
places that can silently disagree; here it means one struct, and older clients ignore what
they do not know.

### 5.7 The gamepad is a client

`padd` reads `gilrs` and sends intents over `robotd`'s socket. Its own crate, so a gamepad
stack stays out of `robotctl` — the tool that has to work on a broken robot.

One socket hop, tens of microseconds. What it buys: the input path used by the app, the SDK
and any remote client is the one a developer exercises every day, so it cannot quietly rot.
For dev, `ssh -L /tmp/robotd.sock:/run/robotd.sock` gives pad-on-laptop, robot-on-board with
no code.

### 5.8 `safeToRestart` becomes real

False while the policy is enabled and the robot is moving. Restarting motor control
mid-stride is how a robot falls over (`updater-design.md` §7.2), and slice 2 is the first
time that can happen.

### 5.9 Done when

It walks on a board, driven through the intent API; an update applied with `robotctl`
restarts it cleanly with the gate passing; and `--unhealthy` still rolls back.

**Built, not yet run on hardware.** Nothing in slice 2 has met a robot: no policy has been
loaded on a board, no observation has reached a real ONNX Runtime, and the fall and deadman
paths have only been exercised against `FakeIo`. The tests establish that the logic is
self-consistent — not that the robot walks. Note also that `padd` cannot run on the board
yet (§11.4), so the first hardware driving will be from a laptop over a forwarded socket.

## 6. The robot model

Rust consts for alpha only: 15 joints, Dynamixel IDs `20–24 / 30–34 / 10–14`, names,
`ALPHA_DEFAULT_POSITION`, and per-joint limits. Same tables as the runtime's `motor.rs`,
minus the three dead variants. There is exactly one robot; a second revision can be a second
table.

## 7. Cross-cutting

### 7.1 The control loop reads snapshots, never waits

Intents and params are published by IPC threads and read by the loop as a single atomic
load. Nothing can apply backpressure to the loop and no request enters it synchronously.
Telemetry goes out through a bounded broadcast where a slow subscriber gets a gap, never
backpressure — the pattern the updater's IPC layer already uses and documents.

### 7.2 Params

A TOML file read at startup, **not watched** — live reload comes later. It lives outside
`releases/<ver>/` so it survives update *and* rollback, next to the updater's own config at
`/etc/robot/robotd.toml`.

Roughly ten values, not 142: control rate, gains, action scale, low-pass alphas, deadzone,
max velocities, deadman timeout, policy paths. The flag explosion in the runtime was mostly
variants, dead skills and dead sensors, all of which are gone.

### 7.3 Not regressing is the acceptance criterion

The measurement already exists: `bench_dynamixel_bus` reports achieved rate, jitter, read
time, bus time, utilisation, errors and IMU sample freshness at 50 and 100 Hz. Record
today's numbers as the baseline; `robotd` must match them. This is deliberately not an RT
engineering project — no `SCHED_FIFO`, no pinning, no `mlockall`. The loop is reliable today
and the job is to keep it that way while the code around it gets simpler.

Worth keeping permanently: the IMU-freshness check. "The read succeeded but the board handed
back the same sample" feeds dead data to the policy and is known to happen.

### 7.4 Maintenance is a separate namespace

`init`, emergency torque-off, calibration and raw joint writes are not intents. They live in
their own namespace so the relay's per-transport allow-list can keep them off remote
transports. Signaling gating decides *who connects*; it does not say a teleoperator is also
a mechanic — and `update.*` reaching a DataChannel would mean a remote peer can trigger a
rollback.

## 8. Testing

`FakeIo` with scripted samples, no hardware:

- health goes false when deadlines are missed;
- startup adopts the current pose and never commands motion;
- safety clamps a policy output past a joint limit, and fall → limp preempts a running policy;
- deadman zeroes velocity when intents stop;
- **golden observation vectors** — `(inputs, expected 61-float array)` pairs exported from
  mjlab and committed. A wrong index in the observation does not fail loudly; it produces a
  plausible robot that falls over. Depends on an export from `microduck_brain`.

Each test's comment says which failure it exists to prevent, per the repo convention.

## 9. Decisions recorded

| | |
|---|---|
| `duck-control` as a workspace crate | boundary enforced by the compiler, no second repo |
| bus layer written fresh, constants borrowed | thin code, but the tuned numbers are not re-derived |
| Rust consts for the model | one robot exists |
| params file, not watched | establishes the file and its location; the watcher is later |
| policy path in params, default = release dir | updates carry the policy; devs override it |
| adopt current pose on start | an update must not move a standing robot |
| no odometry | nothing in slice 1 or 2 reads it; one 50 Hz rate instead of 50/100 |
| walk/stand keep the runtime's shape | abstraction waits for the third skill |
| gamepad as its own crate | keeps `gilrs` out of the recovery CLI |
| sim after slice 2 | hardware is the validation path; `FakeIo` covers laptop development |

## 10. Deferred, deliberately

MuJoCo backend and the `RemoteIo` protocol · the remaining six skills · the skill
abstraction · policy bundle manifests and `model_api` gating · `look`/`pose`/`do` intents ·
gaze IK · live params reload and the config store · odometry · thermal limits · rate limits ·
per-device IMU calibration.

## 11. Open

1. **Control rate on the Radxa.** 50 Hz is inherited from a Pi Zero 2W. Measurable now
   that boards exist.
2. ~~**The 61D command encoding is not settled**~~ — **resolved** (§5.2), by reading
   `control_step` rather than by asking. All three sub-questions are answered and pinned by
   tests. This turned out not to need `microduck_brain` at all.
3. **Golden vectors** would still be worth having from `microduck_brain` — as a regression
   check against the training env rather than as the source of truth they were going to be.
   No longer a prerequisite for slice 2.
4. **`padd` does not reach the board.** `gilrs` pulls `libudev-sys` unconditionally on Linux,
   which needs a pkg-config sysroot — the thing building with zig was chosen to avoid. Three
   ways out: a cross sysroot, a pure-Rust evdev backend, or building it natively on the
   board. Until one is picked it runs on a laptop against a forwarded socket, which is
   genuinely useful but is not the gamepad-on-the-robot story.
5. **Per-joint limits do not exist.** Safety clamps to the *actuator's* travel, which catches
   `NaN`, a bad action scale and a garbage tensor — it will not stop a joint being driven
   somewhere mechanically unwise. The real limits are in the alpha MJCF (31 KB), not vendored
   here. A limit that looked anatomical but was not would imply protection nobody has.
4. **Where the alpha MJCF lives** if a sim/real agreement test is ever wanted — 31 KB of
   XML, 19 MB of meshes, currently in the runtime's `scripts/alpha_assets/`.
