#!/usr/bin/env python3
"""Record MicroDuck observations locally; never uploads or sends control commands."""
import argparse, json, os, socket, time
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
    a = p.parse_args()
    if not 0 < a.hz <= 10: p.error("--hz must be between 0 and 10")
    out = a.root / time.strftime("microduck-%Y%m%d-%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    (out / "frames").mkdir()
    manifest = {"format":"microduck-lerobot-staging-v1", "task":a.task, "fps":a.hz,
                "upload":"disabled", "camera":"UYVY", "action":"policy_action"}
    (out / "meta.json").write_text(json.dumps(manifest, indent=2) + "\n")
    end, n = time.monotonic() + a.seconds, 0
    with (out / "samples.jsonl").open("x") as samples:
        while time.monotonic() < end:
            started = time.monotonic()
            with socket.socket(socket.AF_UNIX) as media:
                media.connect(MEDIA); meta, pixels = frame(media)
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
            samples.flush(); os.fsync(samples.fileno()); n += 1
            time.sleep(max(0, 1/a.hz - (time.monotonic()-started)))
    print(f"recorded {n} local samples in {out}")

if __name__ == "__main__": main()
