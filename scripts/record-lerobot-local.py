#!/usr/bin/env python3
"""Record MicroDuck observations locally; never uploads or sends control commands."""
import argparse, json, os, shutil, socket, time
from pathlib import Path

MEDIA = "/run/mediad/media.sock"
ROBOT = "/run/robotd.sock"

def line(sock, value):
    sock.sendall((json.dumps(value, separators=(",", ":")) + "\n").encode())

def frame(sock):
    line(sock, {"jsonrpc":"2.0", "id":1, "method":"media.frame", "params":{}})
    f = sock.makefile("rb")
    header = json.loads(f.readline())
    if "error" in header: raise RuntimeError(header["error"]["message"])
    meta = header["result"]
    data = f.read(meta["bytes"])
    if len(data) != meta["bytes"]: raise RuntimeError("short camera frame")
    return meta, data

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--task", required=True)
    p.add_argument("--seconds", type=float, default=30)
    p.add_argument("--hz", type=float, default=5)
    p.add_argument("--root", type=Path, default=Path("/var/lib/robot/datasets"))
    p.add_argument("--max-samples", type=int, default=300,
                   help="hard cap on frames written (default: 300)")
    p.add_argument("--max-bytes", type=int, default=512 * 1024 * 1024,
                   help="hard cap on raw frame bytes written (default: 512 MiB)")
    p.add_argument("--min-free-bytes", type=int, default=1024 * 1024 * 1024,
                   help="stop before free space drops below this reserve (default: 1 GiB)")
    p.add_argument("--max-episodes", type=int, default=20,
                   help="refuse to create another episode once this many exist (default: 20)")
    a = p.parse_args()
    if not 0 < a.hz <= 10: p.error("--hz must be between 0 and 10")
    if a.seconds <= 0: p.error("--seconds must be positive")
    if min(a.max_samples, a.max_bytes, a.min_free_bytes, a.max_episodes) <= 0:
        p.error("all recording limits must be positive")
    a.root.mkdir(parents=True, exist_ok=True)
    episodes = [path for path in a.root.iterdir() if path.is_dir() and (path / "meta.json").is_file()]
    if len(episodes) >= a.max_episodes:
        p.error(f"episode cap reached ({len(episodes)}/{a.max_episodes}); review or archive recordings first")
    if shutil.disk_usage(a.root).free < a.min_free_bytes:
        p.error("free-space reserve is already below --min-free-bytes; free space before recording")
    out = a.root / time.strftime("microduck-%Y%m%d-%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    (out / "frames").mkdir()
    manifest = {"format":"microduck-lerobot-staging-v1", "task":a.task, "fps":a.hz,
                "upload":"disabled", "camera":"UYVY", "action":"policy_action",
                "guardrails":{"max_samples":a.max_samples, "max_bytes":a.max_bytes,
                              "min_free_bytes":a.min_free_bytes, "max_episodes":a.max_episodes}}
    (out / "meta.json").write_text(json.dumps(manifest, indent=2) + "\n")
    end, n, frame_bytes, stop_reason = time.monotonic() + a.seconds, 0, 0, "duration"
    with (out / "samples.jsonl").open("x") as samples:
        while time.monotonic() < end:
            if n >= a.max_samples:
                stop_reason = "max_samples"
                break
            started = time.monotonic()
            with socket.socket(socket.AF_UNIX) as media:
                media.connect(MEDIA); meta, pixels = frame(media)
            if frame_bytes + len(pixels) > a.max_bytes:
                stop_reason = "max_bytes"
                break
            if shutil.disk_usage(a.root).free - len(pixels) < a.min_free_bytes:
                stop_reason = "min_free_bytes"
                break
            name = f"frames/{n:06d}.uyvy"
            (out / name).write_bytes(pixels)
            # State is intentionally sampled after the image and carries its own monotonic t.
            # The converter pairs by the image's capture timestamp and preserves this skew.
            with socket.socket(socket.AF_UNIX) as robot:
                robot.connect(ROBOT)
                line(robot, {"jsonrpc":"2.0", "id":1, "method":"robot.subscribe", "params":{"hz":1}})
                r = robot.makefile("rb")
                r.readline()  # subscription acknowledgement
                state = json.loads(r.readline())["params"]
            samples.write(json.dumps({"frame":name, "camera":meta, "state":state}, separators=(",", ":")) + "\n")
            samples.flush(); os.fsync(samples.fileno()); n += 1; frame_bytes += len(pixels)
            time.sleep(max(0, 1/a.hz - (time.monotonic()-started)))
    manifest["samples"] = n
    manifest["frame_bytes"] = frame_bytes
    manifest["stop_reason"] = stop_reason
    (out / "meta.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"recorded {n} local samples ({frame_bytes} bytes, stopped by {stop_reason}) in {out}")

if __name__ == "__main__": main()
