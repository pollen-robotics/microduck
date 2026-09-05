# Local LeRobot recording

Before hardware arrives, run `bash scripts/record-lerobot-preflight.sh`. It uses synthetic data only.

On a robot, record one 30-second local episode, validate it, then inspect the report. Do not export
or train on an episode with missing frames, non-monotonic capture times, or malformed actions.
The recorder never sends a control command and never uploads data. Images remain under
`/var/lib/robot/datasets/` until an operator deliberately copies them for local development.
