#!/usr/bin/env python3
"""Convert validated MicroDuck staging data to a local LeRobot dataset (no Hub upload)."""
import argparse, json, subprocess
from pathlib import Path

def rgb_uyvy(raw, w, h):
    import numpy as np
    x = np.frombuffer(raw, dtype=np.uint8).reshape(h, w // 2, 4).astype(np.float32)
    u, y0, v, y1 = (x[..., i] for i in range(4))
    y = np.stack((y0, y1), -1).reshape(h, w); u = np.repeat(u, 2, -1); v = np.repeat(v, 2, -1)
    return np.clip(np.stack((y + 1.402*(v-128), y - .344136*(u-128)-.714136*(v-128), y + 1.772*(u-128)), -1), 0, 255).astype(np.uint8)

def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument("staging", type=Path); p.add_argument("--root", type=Path, required=True); p.add_argument("--repo-id", default="local/microduck"); a = p.parse_args()
    subprocess.run(["python3", "scripts/validate-lerobot-staging.py", str(a.staging)], check=True)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import numpy as np
    rows = [json.loads(x) for x in (a.staging / "samples.jsonl").read_text().splitlines()]
    first = rows[0]; c = first["camera"]; features = {
      "observation.image": {"dtype":"image", "shape":(c["height"], c["width"], 3), "names":["height","width","channel"]},
      "observation.state": {"dtype":"float32", "shape":(15,), "names":None},
      "action": {"dtype":"float32", "shape":(14,), "names":None}, }
    ds = LeRobotDataset.create(repo_id=a.repo_id, root=a.root, fps=json.loads((a.staging/"meta.json").read_text())["fps"], features=features, robot_type="microduck", use_videos=False)
    for row in rows:
        s, c = row["state"], row["camera"]; action = s.get("policy_action", [])
        if len(action) != 14: continue  # held/homing ticks have no policy label
        image = rgb_uyvy((a.staging/row["frame"]).read_bytes(), c["width"], c["height"])
        ds.add_frame({"observation.image":image, "observation.state":np.asarray(s["joints"], dtype=np.float32), "action":np.asarray(action, dtype=np.float32), "task":json.loads((a.staging/"meta.json").read_text())["task"]})
    ds.save_episode(); ds.finalize()
    print(f"wrote local LeRobot dataset to {a.root}; no upload was requested")
if __name__ == "__main__": main()
