# Slice 2 bring-up on hardware

Notes from running slice 2 on a real robot. Everything below was observed on a Radxa Zero 3W,
not inferred.

## Status

Slice 2 is merged, and the one problem the board found — **`ort` panicking instead of returning
an error, which killed the control thread and made `robot.health` blame the wrong thing** — is
fixed: `duck-control::policy::catching_ort_panics` turns it into a `PolicyError`, so it takes
the existing "hold the pose and report why" path.

**Both are now closed on hardware.** The fix was exercised on the board by pointing `robotd` at
a 1.20.1 runtime through `ORT_DYLIB_PATH`: the loop kept ticking and health named the version
mismatch, instead of a dead control thread reporting "has not completed a cycle yet". Slice 2
then ran with inference in the tick, and `0.2.0` — the first release containing any of it — was
installed from the stable channel.

Nothing links to this file any more. It is kept as the record of what the board actually said;
the reusable half is the recipe below, and [`cheatsheet.md`](../robot/cheatsheet.md) is where a command
someone needs again should end up.

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

**2. `ort` panics rather than erroring — fixed.**

`ensure_runtime()` probes the dylib with `libloading` before letting `ort` touch it. Its doc
comment used to claim:

> a probe that succeeds means its load will succeed too and the panic cannot fire

**The board falsified that.** The probe only proves the library *loads*; 1.20.1 loads fine.
`ort`'s own compatibility check then rejects the *version* and panics inside `setup_api`, which
`ensure_runtime` cannot see. So the guard closes the "missing" case and not the "wrong version"
case — and by construction it could not have closed every future `ort` panic either.

The fix does not try to. `catching_ort_panics` wraps the `ort` calls inside `Policy::load` and
converts any panic into `PolicyError::RuntimePanic`, carrying the panic message — the version
numbers above are the whole diagnosis, so losing them would leave a health reason nobody can
act on. That puts a panic on the path `robotd` already had for a policy that will not load: log
`policy unavailable; holding the pose`, store the reason, leave `controller` as `None`, **keep
ticking at rate**, and report `policy unavailable: <reason>` so the updater rolls the release
back for a stated cause.

Two things to know about it:

- The catch wraps the `ort` work only, not all of `load`, so a genuine bug of ours is not
  relabelled "policy unavailable". `AssertUnwindSafe` is needed because `Session` is not
  `UnwindSafe`; the sessions are moved into the `Policy` on success and dropped on failure, so
  nothing of ours is observed after a catch.
- **`panic = "abort"` would defeat it.** There is no `[profile.release]` in the root
  `Cargo.toml` today; adding one would silently restore the dead control thread.

### What the tests cover, and what they do not

Covered offline, in `duck-control` and `robotd`: a panic on the `ort` path becomes an error and
keeps its message, success passes through untouched, an unprintable payload still yields a
reason, and — via `an_unloadable_policy_holds_the_pose_and_reports_why` — a policy that will not
load leaves the loop ticking with the underlying cause in the health string.

Not covered offline: a *real* `ort` panic travelling through the control loop. Reproducing it
needs a wrong-version runtime, which is a board, and faking one inside `Policy::load` would mean
shipping a fault-injection knob in `duck-control` to test three lines. The board check below is
what closes it.

## Verifying on the board

The board needs the dev key once, or `--ref` is refused. `install.sh` does both halves —
installing the key and flipping `allow_dev_keys` — given the path to the public half:

```bash
sudo DUCK_TOKEN="$DUCK_TOKEN" DUCK_DEV_KEY=/tmp/team.dev.pub sh /tmp/install.sh
```

`team.dev.pub` is committed at `deploy/dev-key/`, outside `trusted_keys/` so nothing installs it
by default. The by-hand equivalent is in [`../deploy/README.md`](../../deploy/README.md).

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
