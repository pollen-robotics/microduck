#!/usr/bin/env python3
"""Convert validated MicroDuck staging data to a local LeRobot dataset (no Hub upload)."""
import argparse, json, subprocess, sys
from pathlib import Path

def rgb_uyvy(raw, w, h):
    import numpy as np
    x = np.frombuffer(raw, dtype=np.uint8).reshape(h, w // 2, 4).astype(np.float32)
    u, y0, v, y1 = (x[..., i] for i in range(4))
    y = np.stack((y0, y1), -1).reshape(h, w); u = np.repeat(u, 2, -1); v = np.repeat(v, 2, -1)
    return np.clip(np.stack((y + 1.402*(v-128), y - .344136*(u-128)-.714136*(v-128), y + 1.772*(u-128)), -1), 0, 255).astype(np.uint8)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("staging", type=Path)
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--repo-id", default="local/microduck")
    p.add_argument("--max-frames", type=int, default=300,
                   help="refuse an export with more labelled frames (default: 300)")
    p.add_argument("--max-estimated-bytes", type=int, default=1024 * 1024 * 1024,
                   help="refuse when uncompressed RGB would exceed this budget (default: 1 GiB)")
    p.add_argument("--dry-run", action="store_true",
                   help="validate and print the export budget without importing LeRobot")
    p.add_argument("--confirm-export", action="store_true",
                   help="required to create the local dataset after reviewing --dry-run")
    a = p.parse_args()
    if a.max_frames <= 0 or a.max_estimated_bytes <= 0:
        p.error("export limits must be positive")
    validator = Path(__file__).with_name("validate-lerobot-staging.py")
    subprocess.run([sys.executable, str(validator), str(a.staging)], check=True)
    rows = [json.loads(x) for x in (a.staging / "samples.jsonl").read_text().splitlines()]
    labelled = [row for row in rows if len(row["state"].get("policy_action", [])) == 14]
    if not labelled:
        p.error("no policy-labelled frames; refusing an empty training dataset")
    estimated_bytes = sum(
        row["camera"]["width"] * row["camera"]["height"] * 3 for row in labelled
    )
    print(f"labelled frames: {len(labelled)}; estimated uncompressed RGB: {estimated_bytes} bytes")
    if len(labelled) > a.max_frames:
        p.error(f"labelled frame count exceeds --max-frames ({len(labelled)}/{a.max_frames})")
    if estimated_bytes > a.max_estimated_bytes:
        p.error(f"estimated RGB bytes exceed --max-estimated-bytes ({estimated_bytes}/{a.max_estimated_bytes})")
    if a.dry_run:
        print("dry run only; no LeRobot dataset was created")
        return
    if not a.confirm_export:
        p.error("run with --dry-run, then pass --confirm-export to create the local dataset")
    if a.root.exists() and any(a.root.iterdir()):
        p.error(f"destination {a.root} is not empty; choose a new local directory")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import numpy as np
    first = labelled[0]; c = first["camera"]; features = {
      "observation.image": {"dtype":"image", "shape":(c["height"], c["width"], 3), "names":["height","width","channel"]},
      "observation.state": {"dtype":"float32", "shape":(15,), "names":None},
      "action": {"dtype":"float32", "shape":(14,), "names":None}, }
    ds = LeRobotDataset.create(repo_id=a.repo_id, root=a.root, fps=json.loads((a.staging/"meta.json").read_text())["fps"], features=features, robot_type="microduck", use_videos=False)
    for row in labelled:
        s, c = row["state"], row["camera"]; action = s.get("policy_action", [])
        image = rgb_uyvy((a.staging/row["frame"]).read_bytes(), c["width"], c["height"])
        ds.add_frame({"observation.image":image, "observation.state":np.asarray(s["joints"], dtype=np.float32), "action":np.asarray(action, dtype=np.float32), "task":json.loads((a.staging/"meta.json").read_text())["task"]})
    ds.save_episode(); ds.finalize()
    print(f"wrote local LeRobot dataset to {a.root}; no upload was requested")
if __name__ == "__main__": main()
