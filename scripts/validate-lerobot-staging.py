#!/usr/bin/env python3
"""Validate a local MicroDuck recording before converting it to LeRobot."""
import argparse, json, math, sys
from pathlib import Path

def bad(errors, text): errors.append(text)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("dataset", type=Path)
    a = p.parse_args(); root = a.dataset; errors = []; previous = -1; count = 0
    try: meta = json.loads((root / "meta.json").read_text())
    except Exception as e: print(f"invalid meta.json: {e}", file=sys.stderr); return 2
    if meta.get("format") != "microduck-lerobot-staging-v1": bad(errors, "unknown staging format")
    if meta.get("upload") != "disabled": bad(errors, "dataset is not explicitly local-only")
    guardrails = meta.get("guardrails")
    if not isinstance(guardrails, dict): bad(errors, "missing recording guardrails")
    else:
        for key in ("max_samples", "max_bytes", "min_free_bytes", "max_episodes"):
            if not isinstance(guardrails.get(key), int) or guardrails[key] <= 0:
                bad(errors, f"invalid guardrail {key}")
    max_gap = int(2_000_000 / meta.get("fps", 1))
    frame_bytes = 0
    for number, raw in enumerate((root / "samples.jsonl").read_text().splitlines(), 1):
        try: sample = json.loads(raw); camera = sample["camera"]; state = sample["state"]
        except Exception as e: bad(errors, f"line {number}: invalid JSON: {e}"); continue
        path = root / sample.get("frame", "")
        if camera.get("format") != "UYVY": bad(errors, f"line {number}: expected UYVY")
        if not path.is_file() or path.stat().st_size != camera.get("bytes"):
            bad(errors, f"line {number}: missing or truncated {path.name}")
        elif isinstance(camera.get("bytes"), int): frame_bytes += camera["bytes"]
        stamp = camera.get("captured_at_unix_us")
        if not isinstance(stamp, int) or stamp <= previous: bad(errors, f"line {number}: non-monotonic camera time")
        if previous >= 0 and isinstance(stamp, int) and stamp - previous > max_gap: bad(errors, f"line {number}: camera gap exceeds two sample periods")
        previous = stamp if isinstance(stamp, int) else previous
        action = state.get("policy_action", [])
        if action and len(action) != 14: bad(errors, f"line {number}: policy_action has {len(action)}, expected 14")
        if action and not all(isinstance(x, (int, float)) and math.isfinite(x) for x in action): bad(errors, f"line {number}: non-finite action")
        if len(state.get("joints", [])) != 15: bad(errors, f"line {number}: joints must have 15 values")
        if state.get("safety", {}).get("fallen") or state.get("safety", {}).get("limp"): bad(errors, f"line {number}: unsafe robot state")
        count += 1
    if not count: bad(errors, "no samples")
    if isinstance(guardrails, dict):
        if count > guardrails.get("max_samples", 0): bad(errors, "sample count exceeds recording cap")
        if frame_bytes > guardrails.get("max_bytes", 0): bad(errors, "frame bytes exceed recording cap")
    if errors:
        print("REJECTED", *errors, sep="\n- ", file=sys.stderr); return 1
    print(f"OK: {count} samples, local-only, ready for LeRobot conversion")
    return 0
if __name__ == "__main__": raise SystemExit(main())
