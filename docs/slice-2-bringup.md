# Slice 2 bring-up on hardware

Handoff notes for finishing this branch on a real robot. Everything below was observed on a
Radxa Zero 3W, not inferred.

## The one open problem

**`ort` panics instead of returning an error, which kills the control thread and makes
`robot.health` blame the wrong thing.**

That is the whole task. Slice 2's code is otherwise complete and its tests pass.

## What already works, verified on the board

Running `0.1.4` (slice 1) on a wired robot:

- 15 servos and the `imu_to_dxl` board answering on `/dev/ttyS2`
- control loop at **50.0 Hz**, `missed=3` out of 15022 ticks (0.02%)
- `robotctl health` → `healthy`
- update path exercised end to end: install, health gate, commit, and auto-rollback

So the bus, the servos, the IMU, the rate and the updater are **not** suspects. Anything that
fails now is slice 2's code or the board's ONNX Runtime.

## The failure

`sudo robotctl update apply daemon --ref slice-2-walk-stand` rolled back:

```
  HealthGate
  RollingBack
{
  "attempted": "0.1.4-dev.58.6781f98",
  "outcome": "rolled_back",
  "reason": "health check failed: not healthy within 30s:
             control loop has not completed a cycle yet",
  "reverted_to": "0.1.4"
}
```

The journal gave the real cause:

```
thread 'control' panicked at ort-2.0.0-rc.11/src/lib.rs:191:41:
Failed to load ONNX Runtime dylib: Error { code: GenericFailure, msg:
  "ort 2.0.0-rc.11 is not compatible with the ONNX Runtime binary found at
   `libonnxruntime.so`; expected version >= '1.23.x', but got '1.20.1'" }
```

### Why the health reason was useless

`RobotState::health` reports `control loop has not completed a cycle yet` when
`ticks == 0 && startup_bus_failures == 0`. A panicking control *thread* does not kill the
process, so `robotd` stays up, keeps serving its socket, and answers with exactly that — the
one message that names no cause. Read the reason string as "the loop never started and never
recorded a reason", not as "still starting".

## Two causes. One is fixed.

**1. The board had an incompatible runtime — fixed in #17 (merged).** `setup-board.sh` pinned
ONNX Runtime 1.20.1; `ort 2.0.0-rc.11` needs >= 1.23. The check is now version-aware, so
re-running the script replaces a wrong version instead of reporting "already present".

The floor and target live in `[workspace.metadata.onnxruntime]` in the root `Cargo.toml`, and
#18 generates the release's `hooks/preinstall` from them so a board below the floor is healed —
or the update aborts *before the swap* — rather than installing and then panicking.

**2. `ort` panics rather than erroring — still open. This is the task.**

`duck-control/src/policy.rs:87`, `ensure_runtime()`, probes the dylib with `libloading` before
letting `ort` touch it. Its doc comment claims:

> a probe that succeeds means its load will succeed too and the panic cannot fire

**The board falsified that.** The probe only proves the library *loads*; 1.20.1 loads fine.
`ort`'s own compatibility check then rejects the *version* and panics inside `setup_api`, which
`ensure_runtime` cannot see. So the guard closes the "missing" case and not the "wrong version"
case — and by construction it cannot close every future `ort` panic either.

## What to build

The graceful path already exists and is well designed. `robotd/src/main.rs:688` handles a
policy that fails to load: it logs `policy unavailable; holding the pose`, stores the reason in
`state.policy_error`, leaves `controller` as `None`, and **the loop still ticks at rate**.
Health then reports `policy unavailable: <reason>` (`robotd/src/main.rs:246`), which is
accurate and actionable, and the updater rolls the release back for a stated reason.

The panic bypasses all of that. So: make any `ort` panic take the existing path.

Suggested shape — `std::panic::catch_unwind` around the `ort` work inside `Policy::load`
(`duck-control/src/policy.rs:125`), converting a caught panic into a `PolicyError`. Points to
settle while doing it:

- **Catch as narrowly as possible.** Wrap the `ort` calls, not the whole function, so a genuine
  bug in our code is not silently converted into "policy unavailable".
- `Session` and friends may not be `UnwindSafe`; `AssertUnwindSafe` is probably needed. Justify
  it in a comment — the argument is that a failed `ort` init leaves nothing of ours partly
  mutated.
- **Extract the panic message** into the `PolicyError`, or the health reason loses the version
  numbers that make it actionable. The string above is exactly what someone needs.
- Consider whether `ensure_runtime`'s doc comment should keep its claim. It is now known false;
  at minimum it should say what the probe does and does not cover.
- `panic = "abort"` would defeat `catch_unwind`. Checked: there is no custom
  `[profile.release]` in the root `Cargo.toml`, so the default unwind strategy applies and
  `catch_unwind` will work. Anyone adding `panic = "abort"` later would silently break this,
  which is worth a comment where the catch happens.

### Test it deterministically

There is no need to install an old ONNX Runtime to test this. The failure is "`ort` panicked",
so a test only needs a panic on that path — the point is that a panicking policy load becomes
`PolicyError` and the loop keeps ticking, not which panic it was.

Slice 2 already has `an_unloadable_policy_holds_the_pose_and_reports_why` in
`robotd/src/main.rs`. The new test should assert the same outcome when the load *panics* rather
than returns `Err`, and that the health reason still carries the detail.

## Verifying on the board

The board needs the dev key once, or `--ref` is refused:

```bash
sudo cp team.dev.pub /etc/robot/trusted_keys/
```
```bash
sudo sed -i 's/^allow_dev_keys.*/allow_dev_keys        = true/' /etc/robot/updater.toml
sudo systemctl restart updaterd
```

`team.dev.pub` is deliberately not in the repository — `deploy/trusted_keys/README.md` explains
why. Get the public half from Pierre, or regenerate it from the secret with
`minisign -R -s <secret> -p team.dev.pub`. Once #18 is merged, `install.sh` does both steps
given `DUCK_DEV_KEY=/path/to/team.dev.pub`.

Then:

```bash
sudo robotctl update apply daemon --ref slice-2-walk-stand
```

**Re-fetch `setup-board.sh` before trusting it.** `/usr/local/sbin/robot-setup-board` is a
snapshot copied when it was last run; it never refreshes itself, so it can silently run
pre-#17 logic and report "already present" for an incompatible runtime.

Success looks like `ONNX Runtime  1.28.0` in the status block, the update committing rather
than rolling back, and:

```bash
journalctl -u robotd -b --no-pager | grep -E 'policy|control loop'
```

showing `policy loaded` followed by `control loop running driving=true`. Then re-measure the
rate — slice 2 adds inference to the same tick, and the slice 1 baseline above (50.0 Hz,
`missed=3`) is what to compare against. A large jump in `missed` is inference cost, not jitter.

## Conventions

- **Branch in a fresh clone under `/tmp`**, never in the working checkout. A stale working tree
  in a shared clone is how #13 silently reverted #12 — `git checkout -b` carried uncommitted
  changes into a new branch and `git add -A` committed them as deletions.
- Commit trailer is `Assisted-by: Claude:claude-opus-5`. Never `Co-Authored-By`.
- Scope test runs: `cargo test -p <crate>`, once. Save `--workspace` for the pre-PR check.
- Ask before making architecture decisions.
- Fix release-path bugs and cut a release; do not hand over a local workaround.

## Deliberately not done

- MuJoCo backend, and the remaining six skills.
- Per-joint limits. `duck-control/src/safety.rs` clamps to actuator travel (±π), not per-joint
  ranges; that needs the alpha MJCF vendored.
- Golden observation vectors from `microduck_brain` to pin the 61-D encoding against the
  prototype. The layout tests cover shape, not agreement with the original.
- `hooks/postinstall` — #18 ships only `preinstall`.
