#!/usr/bin/env python3
"""Record a read-only safety observation around a model update or reload."""
import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path


def subscribe(socket_path, seconds):
    deadline = time.monotonic() + seconds
    frames = []
    with socket.socket(socket.AF_UNIX) as conn:
        conn.settimeout(3)
        conn.connect(socket_path)
        request = {
            "jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {"hz": 10},
        }
        conn.sendall((json.dumps(request, separators=(",", ":")) + "\n").encode())
        reader = conn.makefile("rb")
        acknowledgement = json.loads(reader.readline())
        if "error" in acknowledgement:
            raise RuntimeError(acknowledgement["error"].get("message", "subscribe refused"))
        while time.monotonic() < deadline:
            state = json.loads(reader.readline()).get("params")
            if not isinstance(state, dict):
                raise RuntimeError("robotd sent an invalid state frame")
            frames.append(state)
    if not frames:
        raise RuntimeError("robotd sent no state frames")
    return frames


def health(robotctl):
    result = subprocess.run([robotctl, "health", "--json"], text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(f"robotctl health failed: {result.stderr.strip() or result.stdout.strip()}")
    return json.loads(result.stdout)


def evaluate(frames, report, max_missed):
    violations = []
    missed = [frame.get("loop", {}).get("missed") for frame in frames]
    hz = [frame.get("loop", {}).get("hz") for frame in frames]
    if any(frame.get("policy_enabled") is not False for frame in frames):
        violations.append("policy control was enabled or could not be verified as disabled")
    if any(frame.get("safety", {}).get("fallen") for frame in frames):
        violations.append("robot reported fallen")
    if any(frame.get("safety", {}).get("limp") for frame in frames):
        violations.append("robot reported limp")
    if not all(isinstance(value, int) for value in missed):
        violations.append("control-loop missed-tick count was unavailable")
    elif max(missed) - min(missed) > max_missed:
        violations.append(f"missed ticks increased by {max(missed) - min(missed)} (limit {max_missed})")
    if not all(isinstance(value, (int, float)) for value in hz):
        violations.append("control-loop rate was unavailable")
    report.update({
        "frames": len(frames),
        "policy_enabled": False,
        "missed_start": missed[0] if missed else None,
        "missed_end": missed[-1] if missed else None,
        "min_hz": min(hz) if hz and all(isinstance(value, (int, float)) for value in hz) else None,
        "violations": violations,
    })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10, help="observation window (default: 10)")
    parser.add_argument("--max-missed", type=int, default=0,
                        help="maximum allowed increase in missed ticks (default: 0)")
    parser.add_argument("--robotctl", default="robotctl")
    parser.add_argument("--robot-socket", default="/run/robotd.sock")
    parser.add_argument("--output", type=Path, default=Path("/var/tmp/microduck-model-reload-observation.json"))
    args = parser.parse_args()
    if args.seconds <= 0 or args.max_missed < 0:
        parser.error("--seconds must be positive and --max-missed cannot be negative")

    report = {"format": "microduck-model-reload-observation-v1", "started_at_unix": time.time()}
    try:
        frames = subscribe(args.robot_socket, args.seconds)
        evaluate(frames, report, args.max_missed)
        report["health"] = health(args.robotctl)
        if report["health"].get("robot", {}).get("healthy") is not True:
            report["violations"].append("robotd health was not healthy")
    except (OSError, ValueError, RuntimeError) as error:
        report["violations"] = [str(error)]

    args.output.write_text(json.dumps(report, indent=2) + "\n")
    if report["violations"]:
        print(f"REJECTED: {args.output}", *report["violations"], sep="\n- ", file=sys.stderr)
        return 1
    print(f"OK: observed {report['frames']} disabled-policy frames; report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
