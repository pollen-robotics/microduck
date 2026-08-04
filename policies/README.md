# Policies

The ONNX gait policies `robotd` runs. Both are `obs[1,61] -> actions[1,14]`; `robotd` checks
that at load rather than discovering it mid-stride.

## This is a temporary home

**These belong on the Hugging Face Hub**, delivered as a `model` updater component that
versions independently of the daemon — a gait retrain should not need a daemon release, and a
daemon fix should not re-download 1.5 MB of unchanged weights. `deploy/updater.toml` already
describes that component and deliberately leaves it unconfigured until the repos exist.

They are vendored here because they were not on the Hub yet and slice 2 cannot walk without
them. Committing them makes a release self-contained, which is the property that makes the
update path testable end to end: one `robotctl update apply` turns a standing robot into a
walking one.

Removing this directory later is the whole migration: point `[policy] walk`/`stand` in
`deploy/robotd.toml` at wherever the model component installs, and drop the two `--include`
lines from `.github/workflows/{release,dev}.yml`.

## Provenance

Copied from `apirrone/microduck_runtime` at commit `567fdcd`, dereferencing the symlinks that
repository uses to give stable names to specific training runs:

| here | there | size |
| --- | --- | --- |
| `alpha_walking.onnx` | `policies/BEST_alpha_walking_flat.onnx` | 793705 |
| `alpha_stand.onnx` | `policies/BEST_alpha_stand.onnx` | 793695 |

## Why not the ones microduck_runtime loads by default

Its `src/main.rs` defaults to `policies/walking.onnx` and `policies/standing.onnx`, which
resolve into `new_policies/`. Those were vendored here briefly on the strength of being the
proven pair, and the robot rejected them:

```
policy unavailable: .../walking.onnx: observation width is 51, expected 61
```

They are not an older generation — they are a different **observation format**. 51 is
`3 gyro + 3 gravity + 42 joints + 3 command`, the legacy `[vx, vy, vtheta]` command. 61 is the
same sensors with the unified 13-value command (`[vel(3), head(4), body(6)]`) this daemon
builds. The prototype runs the legacy path by default and reaches the 61-D policies only under
`--new-cmd-obs`, so "what the working runtime loads by default" is the wrong question to ask
of it. The right one is which policies match the observation we build, and that is the
`BEST_alpha_*` family.

Worth noting the shape check earned its place here: it turned a wrong-policy mistake into one
precise line naming both widths, instead of a robot moving in ways nobody could explain.

The names here are the *roles* — what `deploy/robotd.toml` asks for — not the training runs.
That indirection is deliberate and worth keeping: swapping which run is "the walking policy"
should not mean editing config on every robot.

The prototype carries 22 MB of policies across several revisions and skills. Only these two
are copied: the rest belong to skills this daemon does not implement yet, and vendoring them
would put weight in every robot's update for capabilities it cannot use.

## Trying your own

No release needed — `deploy/robotd.toml` takes absolute paths:

```toml
[policy]
walk  = "/home/pierre/my_walk.onnx"
stand = "/home/pierre/my_stand.onnx"
```

Then `sudo systemctl restart robotd`. A policy that fails to load is reported through
`robot.health` as `policy unavailable: <reason>` while the loop keeps ticking and holding its
pose, so a bad file is visible without putting the robot on the floor.
