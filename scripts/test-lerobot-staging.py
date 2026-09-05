#!/usr/bin/env python3
"""Exercise the local recording validator with synthetic, non-camera data."""
import json, subprocess, sys, tempfile
from pathlib import Path

def main():
    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp); (root / "frames").mkdir()
        (root / "meta.json").write_text(json.dumps({"format":"microduck-lerobot-staging-v1", "upload":"disabled", "guardrails":{"max_samples":2, "max_bytes":8, "min_free_bytes":1, "max_episodes":1}}))
        rows = []
        for i in range(2):
            name = f"frames/{i:06d}.uyvy"; data = bytes([128, 32, 128, 32])
            (root / name).write_bytes(data)
            rows.append({"frame":name, "camera":{"format":"UYVY", "bytes":len(data), "width":2, "height":1, "captured_at_unix_us":1000+i}, "state":{"joints":[0.0]*15, "policy_action":[0.0]*14}})
        (root / "samples.jsonl").write_text("\n".join(json.dumps(x) for x in rows)+"\n")
        subprocess.run([sys.executable, str(repo / "scripts/validate-lerobot-staging.py"), str(root)], check=True)
        subprocess.run([
            sys.executable, str(repo / "scripts/export-lerobot-local.py"), str(root),
            "--root", str(root / "lerobot-output"), "--dry-run",
        ], check=True)
    print("synthetic staging test passed")
if __name__ == "__main__": main()
