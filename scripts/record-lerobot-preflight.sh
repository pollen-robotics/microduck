#!/usr/bin/env bash
# Verify the local-only data path before the first real MicroDuck recording.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
python3 "$ROOT/scripts/test-lerobot-staging.py"
printf '%s\n' 'Synthetic preflight passed. On the robot, collect a short episode with:'
printf '%s\n' '  sudo python3 scripts/record-lerobot-local.py --task "<task>" --seconds 30 --hz 5'
printf '%s\n' 'Then validate and inspect it before exporting to LeRobot:'
printf '%s\n' '  python3 scripts/validate-lerobot-staging.py /var/lib/robot/datasets/<episode>'
printf '%s\n' '  python3 scripts/report-lerobot-staging.py /var/lib/robot/datasets/<episode>'
