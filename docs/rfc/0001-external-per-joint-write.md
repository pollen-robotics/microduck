# RFC: `robot.setJoints` — external per-joint target streaming

Status: **draft / scoping** · Author: @cagataycali · Target: pollen-robotics/microduck

## Summary

Add one new intent to robotd's `robot.*` socket — `robot.setJoints` — that lets an
**off-robot** controller stream joint targets at the control rate, and a matching
**External** drive mode in which the control loop uses those targets in place of the
on-device policy. Everything still flows through the existing `duck-control` safety
layer; nothing bypasses `Safety::apply`.

## Motivation

robotd today is intent-level by design: `robot.move` / `robot.pose` / `robot.head` /
`robot.do` describe *what* to do, and the on-device `Policy` (walking + skills) turns
that into joint targets at 50 Hz. That is the right default and should stay the default.
But two real workflows have no path:

1. **Run an off-robot policy on the real robot.** A policy that is too large for the
   Pi, or one being iterated on a workstation/GPU, cannot drive the hardware without
   re-flashing a `model` component through the updater on every change. The
   teleoperation / VLA / lerobot / MHS pattern is: *controller computes joint targets,
   streams them to the robot at rate.* There is no verb for that here.
2. **Bring-up / calibration / scripted motion.** Moving one joint to a commanded angle
   for a test, a calibration sweep, or a recorded trajectory replay — all want direct
   joint targets, not an intent the policy re-interprets.

This is distinct from **deploying** a finished on-device policy, which is already served
by the updater's `model` component (`update.apply`, gated by `robot.modelApi`). This RFC
does **not** touch that path.

### Downstream unlock

`strands-robots`' native driver (`Robot("microduck", mode="real")`) is delegate-only
*because the wire has no per-joint write* — it sends intents and reads `robot.state`,
and refuses `run_policy`. `robot.setJoints` turns that refusal into a real **passthrough
mode**: the driver streams an off-robot policy's actions straight to the joints.

## Non-goals

- Not a replacement for the on-device policy or the model-update deploy path.
- Not a general teleop protocol (no bilateral force, no trajectory interpolation server —
  the controller owns interpolation and sends targets at rate).
- No new IO handle in `duck-control::control` — the Runtime still only *proposes*
  targets; `Safety` still owns the only motor write.

## Current architecture (what we build on)

Control tick (robotd/src/main.rs:1040): `read → observe (fall) → gate (deadman) → policy → safety.apply`

- **`duck_control::control::Runtime::step(sensors, command, …) -> targets[NUM_JOINTS]`**
  (control.rs) turns a `Command` (twist / head_pose / body_pose) + the active skill into
  `targets[j] = DEFAULT_POSITION[j] + action_scale · offset[j]`. It holds no IO handle.
- **`duck_control::safety::Safety::apply`** (safety.rs) is the sole writer. It already:
  refuses non-finite targets (NaN is *refused*, not clamped), clamps to actuator travel,
  runs the fall gate + limp gain, and enforces a **deadman** (`SafetyConfig.deadman`,
  default 500 ms) that zeroes the velocity when intents go stale.
- **`robotd/src/intents.rs`** tracks intent *age*; the deadman reads the twist's age, and
  a head write must not refresh the twist clock.
- `NUM_JOINTS = 15`, `MOUTH_INDEX = 9` (duck-control/src/model.rs); the policy drives 14,
  the mouth is slot 9. Targets are absolute joint angles in **radians**.

Known gap this RFC must close (safety.rs:42–46, in Pollen's own words): the range clamp is
the *actuator's travel, not a per-joint anatomical limit* — "the real joint limits live in
the MJCF, which is not vendored here." A policy trained in that MJCF stays inside those
limits implicitly; **arbitrary external targets do not**, so external write needs real
per-joint bounds.

## Design

### 1. Protocol — `duck-ipc-proto`

New method constant + `Call` variant + params struct, wired through `method()`, `parse()`,
`params()`, with a serde round-trip test in the style of the existing ones (assert the exact
wire line, e.g. field names, and `from_str(to_string(x)) == x`).

```rust
// method::
pub const ROBOT_SET_JOINTS: &str = "robot.setJoints";

// Call::
RobotSetJoints(SetJointsParams),

/// Absolute joint targets, radians, in JOINT_NAMES order (len == NUM_JOINTS).
/// A NOTIFICATION (no id) — it is a high-rate stream like robot.move.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SetJointsParams {
    pub targets: Vec<f64>,          // len must equal NUM_JOINTS; validated on parse
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gain: Option<u16>,          // optional stiffness; defaults to gain_running
}
```

Wire (notification):
```json
{"jsonrpc":"2.0","method":"robot.setJoints","params":{"targets":[0.0,-0.087, … 15 values …]}}
```

Open question: absolute radians (proposed, easiest to clamp) vs. offsets from
`DEFAULT_POSITION`. Absolute is safer — the clamp bounds are absolute.

### 2. robotd — an **External** drive mode

- Extend the drive mode (`robot.setMode`, main.rs) with `External`. `robot.setJoints` is
  **only** honoured in `External` mode and only with torque enabled; in any other mode it
  is refused with a named error (exactly as `run_policy` is refused on the strands side
  today). Entering `External` requires an explicit `robot.setMode external`.
- `intents.rs` gains an external-targets slot with its **own age clock** (independent of
  the twist deadman). A fresh `robot.setJoints` stores `targets` + `Instant::now()`.
- Control tick becomes `read → observe → gate → {External ? external_targets : policy} → safety.apply`.
  In `External` mode the policy stage is skipped and the stored external targets are used.

### 3. Safety envelope (the whole point)

`robot.setJoints` must be *no less safe* than the policy path. Additions, all inside the
existing `Safety` chokepoint:

1. **Mode + torque gate** — refuse unless `External` and enabled (above).
2. **Deadman on external targets** — reuse the deadman mechanism with the external clock:
   if no fresh `setJoints` within `SafetyConfig.deadman`, hold the last safe target and
   drop toward limp gain. A dropped/stalled off-robot controller must not leave a live
   command. (500 ms is generous for a 50 Hz stream; consider a tighter external deadman,
   e.g. 100–150 ms.)
3. **Per-tick step clamp (new)** — bound `|target[j] − previous[j]|` per tick to a max
   joint velocity, so a single bad frame can't snap a joint. The policy path is inherently
   smooth; raw external targets are not, so this is required.
4. **Per-joint anatomical limits (new — closes safety.rs:42–46)** — vendor the per-joint
   `[min,max]` from the MJCF and clamp external targets to them, not just to actuator
   travel. Without this, "range clamp" is weaker than the guarantee external write implies.
5. **NaN refusal** — already present; a non-finite target in the vector is refused outright.

### 4. strands-robots driver (downstream, separate PR)

Add a passthrough path to `MicroduckDriver`: `set_mode("external")` then stream
`robot.setJoints` from `send_action`/a policy loop. This upgrades the driver from
delegate-only to true external control and is what lets an off-robot / mid-iteration RL
policy drive the physical robot. Ships after this RFC lands.

## Testing

- **Protocol**: serde round-trip + exact-wire assertions, matching the existing
  `duck-ipc-proto` test discipline; reject `targets.len() != NUM_JOINTS` on parse.
- **Safety** (fake `RobotIo`): a NaN in the vector is refused; a target outside a joint's
  anatomical limit is clamped to it; a jump larger than the per-tick bound is rate-limited;
  external targets older than the deadman hold + limp; `setJoints` outside `External` mode
  is refused.
- **Mode**: `robot.setMode external` → `setJoints` moves a joint; `robot.setMode …` back to
  a policy mode resumes the policy with no residual external target.

## Rollout / PR plan

1. `duck-ipc-proto`: method + `Call` variant + `SetJointsParams` + round-trip test. (self-contained)
2. `duck-control`: per-joint limits + per-tick step clamp + external deadman in `Safety`; the
   `External` branch in `Runtime`. (safety-critical; most review here)
3. `robotd`: `External` mode in `robot.setMode`, external-target slot + clock in `intents.rs`,
   the loop branch, refusals.
4. Docs + a `duckctl` example that streams a sine sweep on one joint in `External` mode.
5. (separate, downstream) strands-robots driver passthrough mode.

## Open questions for Pollen

- Absolute radians vs. offsets from `DEFAULT_POSITION`? (RFC proposes absolute.)
- Is a new `External` mode preferred, or gating `setJoints` on an existing mode?
- Per-joint limits: vendor from the MJCF into `duck-control`, or a config file robotd loads?
- External deadman: reuse 500 ms, or a tighter dedicated value for rate-streamed control?
- Should the mouth (slot 9) be writable via `setJoints`, or masked like the policy path masks it?
