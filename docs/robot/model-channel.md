# Model channel contract

A policy is a separately versioned `model-*` component. It is not a daemon release: it carries
weights and model metadata only, and it cannot carry daemon binaries or install hooks. This keeps
a model trial inside the updater's ordinary signature, hash, compatibility, rollback and pinning
boundaries without giving a policy artifact a second way to alter a robot.

## Bundle

Build a bundle from a directory containing the files for one named policy slot. Files are installed
at the root of that model component's release, so a configured slot can name, for example,
`/opt/robot/model/walk/current/walk.onnx` directly.

```
cargo xtask package \
  --channel model-walk \
  --version 1.2.3 \
  --model-dir /path/to/walk-bundle \
  --model-api 1 \
  --out dist/
```

`--model-api` is required. A robot accepts the artifact only when its running daemon implements at
least that API. The package command rejects non-`model-*` channels, daemon hooks, and daemon binary
layout for model bundles. Sign the resulting artifact and manifests with a policy signing key whose
public half is already trusted by the target robot; never put a private key or passphrase in this
repository, a command history, or a model bundle.

The bundle is deliberately shallow: package the ONNX file and the small metadata needed to run it,
not recordings, checkpoints, training logs, notebooks, or dependencies. Those belong to the
training environment, not a robot update.

## Trial sequence

1. Disable policy control and leave the robot upright, not limp or fallen.
2. Run `python3 scripts/model-update-preflight.py model-walk` (add `--from` for a signed local
   artifact directory). It verifies the stopped state, health, signature, hash and compatibility
   through the updater's dry-run path; it does not swap `current`.
3. An operator applies the configured model component with the normal `robotctl update apply`
   workflow. This is the only step that may move `current` or request a reload.
4. Before re-enabling policy control, run `python3 scripts/observe-model-reload.py`. It records a
   ten-second disabled-policy window and rejects missed ticks, a safety event, or unhealthy state.
5. Inspect the local observation report and update transcript. Only then make a deliberate,
   supervised decision to re-enable policy control.

An older daemon that does not report `policy_enabled` fails step 2 rather than being assumed safe.
An incompatible or malformed model must remain non-current; do not bypass the preflight by copying
files into a component's `current` path.

## Experiment budget

Use short, local-only recordings for the first trial. The recorder and converter default to bounded
frames, storage, episode count and export size; see [local LeRobot recording](lerobot-local-recording.md).
Increase a `--max-*` limit only for a reviewed run with a stated metric and a stopping condition.
