#!/usr/bin/env python3
"""Print a small quality report for a validated local MicroDuck recording."""
import argparse, json
from pathlib import Path

def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument("dataset", type=Path); a = p.parse_args()
    rows = [json.loads(x) for x in (a.dataset / "samples.jsonl").read_text().splitlines()]
    times = [x["camera"]["captured_at_unix_us"] for x in rows]
    actions = [x["state"].get("policy_action", []) for x in rows]
    labelled = [x for x in actions if len(x) == 14]
    gaps = [b-a for a,b in zip(times, times[1:])]
    print(f"samples: {len(rows)}")
    print(f"policy-labelled: {len(labelled)} ({len(labelled)/len(rows):.0%})")
    if gaps: print(f"camera interval us: min={min(gaps)} median={sorted(gaps)[len(gaps)//2]} max={max(gaps)}")
    if labelled:
        flat = [v for action in labelled for v in action]
        print(f"action range: {min(flat):.3f} .. {max(flat):.3f}")
if __name__ == "__main__": main()
