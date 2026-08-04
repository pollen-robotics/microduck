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

| here | there | size | used by default |
| --- | --- | --- | --- |
| `walking.onnx` | `policies/new_policies/vel_noise_walk.onnx` | 772527 | **yes** |
| `standing.onnx` | `policies/new_policies/standup_gentle_more_range.onnx` | 772527 | **yes** |
| `alpha_walking.onnx` | `policies/BEST_alpha_walking_flat.onnx` | 793705 | no |
| `alpha_stand.onnx` | `policies/BEST_alpha_stand.onnx` | 793695 | no |

Two generations, and the distinction cost a hardware round trip. `walking.onnx` and
`standing.onnx` are what `microduck_runtime` loads by default — `src/main.rs` has them as the
`--policy` / `--standing-policy` defaults — so they are the pair with a track record on a real
robot. The `alpha_*` pair was chosen here first, purely because `deploy/robotd.toml` asked for
a file of that name and the prototype had one; nothing checked which the working system
actually ran.

Everything in the prototype's `new_policies/` is exactly 772527 bytes and everything in the
`BEST_alpha_*` set is ~793700, so the two are clearly distinct generations. Both are shipped
so a board can A/B them by editing `deploy/robotd.toml` and restarting, rather than waiting on
a release — `robotd` checks the 61-input/14-output shape at load, but nothing detects "right
shape, wrong robot", so which family suits alpha is a question only the hardware answers.

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
