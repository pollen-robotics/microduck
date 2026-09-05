# Local LeRobot recording

Before hardware arrives, run `bash scripts/record-lerobot-preflight.sh`. It uses synthetic data only.

On a robot, record one 30-second local episode, validate it, then inspect the report. Do not export
or train on an episode with missing frames, non-monotonic capture times, or malformed actions.
The recorder never sends a control command and never uploads data. Images remain under
`/var/lib/robot/datasets/` until an operator deliberately copies them for local development.

Recording stops at the first configured budget: 300 frames, 512 MiB of raw images, or a 1 GiB
free-space reserve by default. It also refuses a 21st local episode until the existing ones have
been reviewed or archived. These are deliberate experiment-cost guardrails, not a retention
policy: choose explicit `--max-*` values for a larger, reviewed run. The validator rejects a
dataset that exceeds its recorded frame or byte budget.

Before converting, run the exporter's `--dry-run`. It reports policy-labelled frames and the
estimated uncompressed RGB footprint, refusing more than 300 frames or 1 GiB by default. Creating
the local LeRobot dataset then requires `--confirm-export`, and its destination must be empty:

```
python3 scripts/export-lerobot-local.py /var/lib/robot/datasets/<episode> \
  --root /var/lib/robot/lerobot/<episode> --dry-run
python3 scripts/export-lerobot-local.py /var/lib/robot/datasets/<episode> \
  --root /var/lib/robot/lerobot/<episode> --confirm-export
```

## Before trying a model update

With policy control disabled, run a read-only update dry run. It refuses an armed policy, a fallen
or limp robot, an unhealthy daemon, and an older daemon that cannot report its policy state. The
dry run then uses the normal updater path to verify the candidate's signature, hash and
`model_api`, but stops before moving `current` or signalling `robotd`:

```
python3 scripts/model-update-preflight.py model-walk
# for a signed local artifact directory:
python3 scripts/model-update-preflight.py model-walk --from /path/to/signed-artifacts
```

After an operator applies the model through the normal updater flow, observe the disabled-policy
control loop for ten seconds before re-enabling policy control. This command does not signal,
reload, or change robot settings; it writes a local report and rejects a policy-enabled, fallen,
limp, unhealthy, or missed-tick observation:

```
python3 scripts/observe-model-reload.py
```
