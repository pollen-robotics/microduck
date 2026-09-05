#!/usr/bin/env python3
"""Refuse an unsafe model-update dry run; never changes robot state itself."""
import argparse
import json
import socket
import subprocess
import sys


def robot_state(socket_path):
    with socket.socket(socket.AF_UNIX) as conn:
        conn.settimeout(3)
        conn.connect(socket_path)
        request = {
            "jsonrpc": "2.0", "id": 1, "method": "robot.subscribe", "params": {"hz": 1},
        }
        conn.sendall((json.dumps(request, separators=(",", ":")) + "\n").encode())
        reader = conn.makefile("rb")
        acknowledgement = json.loads(reader.readline())
        if "error" in acknowledgement:
            raise RuntimeError(acknowledgement["error"].get("message", "subscribe refused"))
        state = json.loads(reader.readline()).get("params")
        if not isinstance(state, dict):
            raise RuntimeError("robotd did not send a state frame")
        return state


def require_safe_state(state):
    # Missing is deliberately a refusal: an older daemon cannot prove this is the new, explicit
    # policy-disabled state. `held` is insufficient because a live policy can hold on zero input.
    if state.get("policy_enabled") is not False:
        raise RuntimeError("policy is enabled or this robotd cannot report policy_enabled; disable policy control first")
    safety = state.get("safety", {})
    if safety.get("fallen") or safety.get("limp"):
        raise RuntimeError("robot is fallen or limp; recover it before changing a policy")


def require_healthy(robotctl):
    result = subprocess.run([robotctl, "health", "--json"], text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(f"robotctl health failed: {result.stderr.strip() or result.stdout.strip()}")
    report = json.loads(result.stdout)
    if report.get("robot", {}).get("healthy") is not True:
        raise RuntimeError("robotd is not healthy; model update dry run is not safe")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("component", help="configured model component, for example model-walk")
    parser.add_argument("--robotctl", default="robotctl")
    parser.add_argument("--robot-socket", default="/run/robotd.sock")
    parser.add_argument("--from", dest="from_dir", help="signed local artifact directory to dry-run")
    args = parser.parse_args()
    if not args.component.startswith("model-"):
        parser.error("component must start with model-")

    try:
        require_safe_state(robot_state(args.robot_socket))
        require_healthy(args.robotctl)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"REFUSED: {error}", file=sys.stderr)
        return 2

    command = [args.robotctl, "update", "apply", args.component, "--dry-run"]
    if args.from_dir:
        command.extend(["--from", args.from_dir])
    print("preflight passed: policy is disabled and robotd is healthy; verifying signed artifact")
    return subprocess.run(command).returncode


if __name__ == "__main__":
    raise SystemExit(main())
