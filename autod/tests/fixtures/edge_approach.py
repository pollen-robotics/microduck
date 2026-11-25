"""Regenerate edge_approach.json: python3 edge_approach.py <microduck_rl checkout> [duck-sim script]

Walks a duck-sim duck off a step and keeps what autod's obstacle model reads over the last 40 cm:
each depth frame, the robot state nearest to it in time, and how far the beak really was from the
edge, from the simulator. Nothing stops the duck: the fixture is what it saw on the way over.
"""
import json, math, os, re, socket, subprocess, sys, threading, time
from pathlib import Path

HERE = Path(__file__).parent
RL = Path(sys.argv[1])
DUCK_SIM = sys.argv[2] if len(sys.argv) > 2 else str(HERE.parents[2] / "scripts" / "duck-sim")
SCENES = RL / "src/mjlab_microduck/robot/microduck"
STATE = Path(os.environ.get("DUCK_SIM_STATE", Path.home() / ".cache/duck-sim"))
PORT = int(os.environ.get("DUCK_SIM_PORT", 7801))
BEAK_M, EDGE_M, KEPT_M = 0.18, 0.7, 0.40
HEAD = slice(5, 9)  # neck_pitch, head_pitch, head_yaw, head_roll in proto::JOINT_NAMES
# name, how far off square the duck meets the edge (deg), the step's height (m; 0 is a flat floor)
CASES = [("square_20cm", 0, 0.20), ("30deg_20cm", 30, 0.20), ("45deg_20cm", 45, 0.20),
         ("square_10cm", 0, 0.10), ("flat", 0, 0.0)]


def scene(angle_deg, drop_m):
    """The default scene with its floor sunk by `drop_m` and a platform under the duck, whose
    edge is `EDGE_M` ahead."""
    base = (SCENES / "scene_backlash.xml").read_text()
    if not drop_m:
        return base
    a, half = math.radians(angle_deg), 2.0
    cx, cy = (EDGE_M - half) * math.cos(a), (EDGE_M - half) * math.sin(a)
    floor = re.search(r'<geom name="floor"[^>]*/>', base)[0]
    sunk = re.sub(r'pos="[^"]*"', f'pos="0 0 {-drop_m}"', floor)
    platform = (f'<geom name="platform" type="box" size="{half} {half} {drop_m / 2}" '
                f'pos="{cx:.4f} {cy:.4f} {-drop_m / 2}" euler="0 0 {a:.6f}" />')
    return base.replace(floor, sunk + "\n        " + platform)


def stream(s, method, params, want, sink):
    f = s.makefile("rw")
    f.write(json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}) + "\n")
    f.flush()
    for line in f:
        try:
            m = json.loads(line)
        except ValueError:  # the line the shutdown cut
            break
        if m.get("method") == want:
            sink.append(m["params"])


def unix(name):
    s = socket.socket(socket.AF_UNIX)
    s.connect(str(STATE / name))
    return s


def record(angle_deg, drop_m):
    a = math.radians(angle_deg)
    states, frames, truth = [], [], []
    body = socket.create_connection(("127.0.0.1", PORT)).makefile("rw")
    body.write(json.dumps({"op": "hello", "protocol": 1, "joints": 15}) + "\n")
    body.flush()
    body.readline()

    def pose():
        body.write('{"op":"read"}\n')
        body.flush()
        r = json.loads(body.readline())
        w, x, y, z = r["imu"]["quat"]
        return r["trunk"], math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    socks = [unix("duck-a.sock"), unix("duck-a-tof.sock"), unix("duck-a.sock")]
    threads = [
        threading.Thread(target=stream, args=(socks[0], "robot.subscribe", {"hz": 50}, "robot.state", states)),
        threading.Thread(target=stream, args=(socks[1], "tof.stream", None, "tof.frame", frames)),
    ]
    for t in threads:
        t.start()
    out = socks[2].makefile("w")

    def move(vx, vyaw):
        out.write(json.dumps({"jsonrpc": "2.0", "method": "robot.move",
                              "params": {"vx": vx, "vy": 0.0, "vyaw": vyaw}}) + "\n")
        out.flush()

    t0 = time.time()
    while time.time() - t0 < 25:
        trunk, yaw = pose()
        beak = trunk[0] + BEAK_M * math.cos(yaw), trunk[1] + BEAK_M * math.sin(yaw)
        gap = EDGE_M - (beak[0] * math.cos(a) + beak[1] * math.sin(a)) if drop_m else None
        truth.append((time.clock_gettime_ns(time.CLOCK_MONOTONIC), gap, trunk[0], trunk[1]))
        if (gap is not None and gap < 0.0) or (gap is None and time.time() - t0 > 6):
            break
        # Along the world's x, which the scene's edge is turned against: the gait drifts sideways.
        move(0.22, max(-0.6, min(0.6, -2.0 * yaw - 4.0 * trunk[1])))
        time.sleep(0.03)
    move(0.0, 0.0)
    for s in socks:
        try:
            s.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
    for t in threads:
        t.join()

    rows = []
    for frame in frames:
        state = min(states, key=lambda s: abs(s["t_ns"] - frame["t_ns"]))
        gap = min(truth, key=lambda s: abs(s[0] - frame["t_ns"]))[1]
        if gap is not None and not 0.0 <= gap <= KEPT_M:
            continue
        odom = state["odom"]
        rows.append({
            "t_ns": frame["t_ns"],
            "at_ns": state["t_ns"],
            "distance_mm": frame["distance_mm"],
            "status": frame["status"],
            "head": [round(j, 4) for j in state["joints"][HEAD]],
            "gravity": [round(g, 4) for g in state["safety"]["gravity"]],
            "trunk_height_m": round(odom["position"][2], 4),
            "pose": [round(odom["position"][0], 4), round(odom["position"][1], 4), round(odom["yaw"], 4)],
            "beak_gap_m": None if gap is None else round(gap, 4),
        })
    (_, _, x0, y0), (_, _, x1, y1) = truth[0], truth[-1]
    print(f"   walked {math.degrees(math.atan2(y1 - y0, x1 - x0)):+.0f} deg off the world's x")
    return rows


def main():
    cases = []
    for name, angle_deg, drop_m in CASES:
        path = SCENES / "scene_edge_approach.xml"
        path.write_text(scene(angle_deg, drop_m))
        env = dict(os.environ, DUCK_SIM_VIEWER="0", DUCK_SIM_SCENE=str(path), DUCK_SIM_RL=str(RL))
        try:
            subprocess.run([DUCK_SIM, "down"], env=env, capture_output=True)
            time.sleep(3)  # the body's port takes a moment to free
            subprocess.run([DUCK_SIM, "up"], env=env, check=True, stdout=subprocess.DEVNULL)
            subprocess.run([DUCK_SIM, "ctl", "autonomous", "off"], env=env, capture_output=True)
            frames = record(angle_deg, drop_m)
        finally:
            subprocess.run([DUCK_SIM, "down"], env=env, capture_output=True)
            path.unlink()
        print(name, len(frames), "frames")
        cases.append({"name": name, "drop_m": drop_m, "frames": frames})
    (HERE / "edge_approach.json").write_text(json.dumps(cases, separators=(",", ":")))


main()
