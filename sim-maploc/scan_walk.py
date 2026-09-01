"""Stop-and-scan: short walking bursts separated by stands.

maploc's default mode only inks while the robot is still -- a stop's frames are
voted against each other before any of them reach the map. So a route that
never stops never maps. Each leg here walks for `WALK` seconds and then stands
for `STOP`, long enough for a still-window to form and the head sweep to run.
"""
import json
import math
import socket
import sys
import time

SOCK = "/tmp/dsm/a.sock"
WALK, STOP = 5.0, 6.0

s = socket.socket(socket.AF_UNIX); s.connect(SOCK)
sf = s.makefile("rw")
sf.write(json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe",
                     "params": {}}) + "\n"); sf.flush(); sf.readline()

c = socket.socket(socket.AF_UNIX); c.connect(SOCK)
cf = c.makefile("w")

b = socket.socket(); b.connect(("127.0.0.1", 7871))
bf = b.makefile("rw")
bf.write('{"op":"hello","protocol":1,"joints":15}\n'); bf.flush(); bf.readline()


def truth():
    bf.write('{"op":"read"}\n'); bf.flush()
    r = json.loads(bf.readline())
    return r["trunk"], r["trunk_z"]


def pose():
    while True:
        m = json.loads(sf.readline())
        od = (m.get("params") or {}).get("odom")
        if od:
            return od["position"][0], od["position"][1], od["yaw"]


def move(vx, vyaw):
    cf.write(json.dumps({"jsonrpc": "2.0", "method": "robot.move",
                         "params": {"vx": vx, "vy": 0.0, "vyaw": vyaw}}) + "\n")
    cf.flush()


# Short legs, all inside the corridor and the near half of the kitchen. Kept
# away from the staircase hole at x in [-0.4, 0], y in [-1.4, -0.7].
ROUTE = [(-0.30, 1.00), (-0.30, 1.90), (-0.30, 2.55), (-1.40, 2.05),
         (-2.20, 2.05), (-1.30, 1.95), (-0.30, 1.90), (-0.30, 0.60)]

deadline = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 300)
for (tx, ty) in ROUTE:
    if time.time() > deadline:
        break
    leg = time.time() + 40
    while time.time() < min(leg, deadline):
        x, y, yaw = pose()
        if math.hypot(tx - x, ty - y) < 0.25:
            break
        e = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw),
                       math.cos(math.atan2(ty - y, tx - x) - yaw))
        move(0.30, max(-0.7, min(0.7, 1.2 * e)))
        time.sleep(0.1)
        if time.time() % (WALK + STOP) > WALK:
            break
    # Stand still and let the mapper have its window.
    t_end = time.time() + STOP
    while time.time() < t_end:
        move(0.0, 0.0)
        time.sleep(0.1)
    (t, tz) = truth()
    x, y, _ = pose()
    print(f"stop: truth ({t[0]:+.2f},{t[1]:+.2f}) z={tz:.3f} "
          f"odom ({x:+.2f},{y:+.2f})", flush=True)

# Walk back to where we booted and stand there. maploc's `evaluate` bench
# scores return-to-start: tracked pose vs raw odometry at a pose it already
# knows, which separates "the mapper drifted" from "the odometry drifted".
# Without this leg that whole half of the report is empty.
print("returning to start", flush=True)
leg = time.time() + 90
while time.time() < leg:
    x, y, yaw = pose()
    if math.hypot(-x, -y) < 0.20:
        break
    e = math.atan2(math.sin(math.atan2(-y, -x) - yaw),
                   math.cos(math.atan2(-y, -x) - yaw))
    move(0.30, max(-0.7, min(0.7, 1.2 * e)))
    time.sleep(0.1)

# Stand still ~12 s: long enough for a still-window to form and settle at the
# start pose, which is what the return-to-start number is read from.
t_end = time.time() + 12
while time.time() < t_end:
    move(0.0, 0.0)
    time.sleep(0.1)
(t, tz) = truth()
x, y, _ = pose()
print(f"back at start: truth ({t[0]:+.2f},{t[1]:+.2f}) odom ({x:+.2f},{y:+.2f})",
      flush=True)
move(0.0, 0.0)
print("route done", flush=True)
