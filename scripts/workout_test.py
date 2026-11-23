#!/usr/bin/env python3
"""workout_test: look for damaged gears in a microduck's servos, with the robot lying on its back.

The robot plays a fixed workout: it raises its head with the neck base vertical, then sweeps every joint
but the neck base through most of its range at a slow, constant speed (legs in mirrored pairs, head joints
alongside), in poses where each joint carries its own links' weight and nothing touches the floor. A sound
gear train follows the commanded speed smoothly. A broken tooth, or a cable catching, makes the joint stick
and then slip ahead, at the same angle each time it passes there.

Lay the robot on its back on the floor, with room around it, then on the robot (about 3.5 minutes):

    sudo python3 workout_test.py [--save samples.json]

robotd, which owns the servo bus, stops for the test and starts again afterwards. --save keeps the
recording.

It prints one line per joint, then:
  PASS  no leg joint shows a fault.
  FAIL  a leg joint sticks and slips at the same angle on more than one pass (a damaged gear or something
        catching: check it by hand), a servo does not answer, or a joint was blocked (the test stops).
Head joints only warn: no broken head servo has been recorded yet to set their threshold.
Exit status: 0 PASS, 1 FAIL, 2 the test could not run.
"""
import argparse
import json
import math
import os
import select
import statistics
import struct
import subprocess
import sys
import termios
import time
import tomllib
from collections import defaultdict

NAMES = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch", "head_pitch",
         "head_yaw", "head_roll", "mouth", "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]
IDS = dict(zip(NAMES, [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]))
HEAD = ("neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth")
TORQUE, P_GAIN, GOAL_POS, PRESENT = 64, 84, 116, 124  # PRESENT: PWM 2, current 2, velocity 4, position 4 bytes
# robotd's position gain lets the raised head sag the neck: the neck and head run stiffer during the test.
STIFF_HEAD, STIFF_FACTOR = ("neck_pitch", "head_pitch", "head_yaw", "head_roll"), 4
HZ, MOVE_SPEED, HOLD_S = 100, 60.0, 0.25  # control rate, pose-change speed (deg/s), hold after each move
SWEEP_SPEED, PASSES = 15.0, 2  # faster sweeps miss a slipping hip pitch
STUCK_DEG, STUCK_NECK_DEG, STUCK_S = 25.0, 45.0, 0.5  # this far from its goal this long: blocked
STALL_MA, STALL_PWM, STALL_S = 900, 90.0, 0.3  # held near the 1 A limit or full drive this long: fighting something

CHECK = {"left_hip_yaw": 0, "left_hip_roll": 10, "left_hip_pitch": -45, "left_knee": 0, "left_ankle": 26,
         "neck_pitch": 20, "head_pitch": 20, "head_yaw": 0, "head_roll": 0, "mouth": 0,
         "right_hip_yaw": 0, "right_hip_roll": -10, "right_hip_pitch": 45, "right_knee": 0, "right_ankle": -26}
# The head goes up with the neck base vertical: fold the head, lift the neck, then unroll both together.
APPROACH = [{**CHECK, "head_pitch": 80}, {**CHECK, "neck_pitch": -52, "head_pitch": 80},
            {**CHECK, "neck_pitch": -82, "head_pitch": 0}]
UP = APPROACH[-1]
# (holding pose, {joint: (start, end)}), each played from UP and back to it. The head pitches down while
# the feet rise, so the two never meet.
SWEEPS = [
    ({**UP, "left_hip_roll": 0, "left_knee": -45, "left_ankle": 0, "head_pitch": 83,
      "right_hip_roll": 0, "right_knee": 45, "right_ankle": 0},
     {"left_hip_pitch": (-45, -111), "right_hip_pitch": (45, 111), "head_pitch": (83, -50)}),
    ({**UP, "left_hip_pitch": -70, "left_knee": 67, "left_ankle": 45, "right_hip_pitch": 70, "right_knee": -67,
      "right_ankle": -45},
     {"left_knee": (67, -80), "right_knee": (-67, 80), "head_yaw": (-90, 90)}),
    ({**UP, "left_hip_pitch": -55, "left_knee": -30, "left_ankle": 71, "right_hip_pitch": 55, "right_knee": 30,
      "right_ankle": -71},
     {"left_ankle": (71, -31), "right_ankle": (-71, 31), "head_roll": (-25, 25)}),
    ({**UP, "left_hip_yaw": -22, "left_hip_pitch": -85, "left_knee": -60},
     {"left_hip_yaw": (-22, 29), "mouth": (0, 22)}),
    ({**UP, "right_hip_yaw": 25, "right_hip_pitch": 85, "right_knee": 60}, {"right_hip_yaw": (25, -56)}),
    ({**UP, "left_hip_yaw": 24, "left_hip_roll": 14, "left_hip_pitch": -70, "left_knee": -30},
     {"left_hip_roll": (14, -25)}),
    ({**UP, "right_hip_yaw": -30, "right_hip_roll": -15, "right_hip_pitch": 55, "right_knee": 30},
     {"right_hip_roll": (-15, 18)}),
]

# Verdict, on the middle of each traversal of a sweep (EDGE_DEG at each end are left out, a third of a sweep
# too short for both: the joint breaks away from its hold at the same angle every time). Irregularity is the
# velocity's std over the command; a slip is a velocity over PEAK times the command. A slip that comes back
# within REPEAT_DEG on another traversal, at no more than MAX_SPOTS places (SPOT_DEG apart), with an
# irregularity over DAMAGED_IRR, is DAMAGED; below that it is a "repeat". Many evenly spaced places are a
# light joint sticking at every tooth mesh ("stick-slip"). Set on two broken leg servos and a catching cable
# (irregularity 0.69 to 3.6) against sound joints checked by hand (up to 0.41). One pass is not enough: their
# slips came back on the next pass in the same direction, not on the way back.
IRREGULAR, PEAK, REPEAT_DEG, MAX_SPOTS, SPOT_DEG, DAMAGED_IRR, EDGE_DEG = 0.4, 2.0, 5.0, 3, 2.0, 0.5, 12.0


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def packet(dxl_id, inst, params=b""):
    stuffed = params.replace(b"\xff\xff\xfd", b"\xff\xff\xfd\xfd")
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id]) + struct.pack("<H", len(stuffed) + 3) + bytes([inst]) + stuffed
    return body + struct.pack("<H", crc16(body))


class Bus:
    """Dynamixel protocol 2.0 at 1 Mbps, on the raw serial port."""

    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        cc = termios.tcgetattr(self.fd)[6]
        cc[termios.VMIN], cc[termios.VTIME] = 0, 0
        termios.tcsetattr(self.fd, termios.TCSANOW, [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0,
                                                     termios.B1000000, termios.B1000000, cc])
        self.buf = b""

    def status(self, deadline):
        """Next status packet (id, error, params), skipping anything else, or None at the deadline."""
        while True:
            i = self.buf.find(b"\xff\xff\xfd\x00")
            if i >= 0 and len(self.buf) >= i + 7:
                n = struct.unpack_from("<H", self.buf, i + 5)[0]
                if len(self.buf) >= i + 7 + n:
                    pkt, self.buf = self.buf[i:i + 7 + n], self.buf[i + 7 + n:]
                    if crc16(pkt[:-2]) == struct.unpack_from("<H", pkt, len(pkt) - 2)[0] and pkt[7] == 0x55:
                        return pkt[4], pkt[8], pkt[9:-2].replace(b"\xff\xff\xfd\xfd", b"\xff\xff\xfd")
                    continue
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            if select.select([self.fd], [], [], left)[0]:
                self.buf += os.read(self.fd, 4096)

    def send(self, pkt):
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.buf = b""
        while pkt:
            pkt = pkt[os.write(self.fd, pkt):]

    def write(self, dxl_id, addr, data):
        self.send(packet(dxl_id, 0x03, struct.pack("<H", addr) + data))
        return self.status(time.monotonic() + 0.05)

    def read(self, dxl_id, addr, n):
        self.send(packet(dxl_id, 0x02, struct.pack("<HH", addr, n)))
        s = self.status(time.monotonic() + 0.05)
        return s[2] if s and s[0] == dxl_id and len(s[2]) == n else None

    def sync_write(self, addr, n, values):
        self.send(packet(0xFE, 0x83, struct.pack("<HH", addr, n) + b"".join(bytes([i]) + v for i, v in values.items())))

    def sync_read(self, addr, n, ids):
        self.send(packet(0xFE, 0x82, struct.pack("<HH", addr, n) + bytes(ids)))
        out, deadline = {}, time.monotonic() + 0.03
        while len(out) < len(ids):
            s = self.status(deadline)
            if s is None:
                break
            if len(s[2]) == n:
                out[s[0]] = s[2]
        return out


def raw(deg):
    return round(4096 * (math.radians(deg) + math.pi) / (2 * math.pi))


def deg(raw_pos):
    return math.degrees(2 * math.pi * raw_pos / 4096 - math.pi)


class Blocked(Exception):
    pass


class Workout:
    def __init__(self, bus, goal):
        self.bus = bus
        self.goal = goal
        self.samples = []
        self.t0 = time.monotonic()
        self.far_since = {}
        self.hard_since = {}

    def tick(self, label=None, joints=()):
        """Send every goal, read every servo, keep the swept joints' rows, watch for a blocked joint."""
        self.bus.sync_write(GOAL_POS, 4, {IDS[n]: struct.pack("<i", raw(g)) for n, g in self.goal.items()})
        got = self.bus.sync_read(PRESENT, 12, list(IDS.values()))
        now = time.monotonic()
        for n, i in IDS.items():
            if i not in got:
                continue
            pwm, cur, vel, pos = struct.unpack("<hhii", got[i])
            q = deg(pos)
            if n in joints:
                self.samples.append([round(now - self.t0, 4), n, label, round(self.goal[n], 2), round(q, 2), cur,
                                     round(vel * 0.229 * 6, 1), round(pwm * 0.113, 1)])
            if abs(q - self.goal[n]) > (STUCK_NECK_DEG if n == "neck_pitch" else STUCK_DEG):
                if now - self.far_since.setdefault(n, now) > STUCK_S:
                    raise Blocked(f"{n} held {q - self.goal[n]:+.0f} deg from its goal for {STUCK_S} s")
            else:
                self.far_since.pop(n, None)
            if abs(cur) > STALL_MA or abs(pwm * 0.113) > STALL_PWM:
                if now - self.hard_since.setdefault(n, now) > STALL_S:
                    raise Blocked(f"{n} held at {cur} mA, PWM {pwm * 0.113:.0f} % for {STALL_S} s")
            else:
                self.hard_since.pop(n, None)

    def ramp(self, target, speed, label=None, joints=(), each=False):
        """Every joint in `target` moves linearly: together, the one with the most travel at `speed` deg/s,
        or with `each` every one at `speed` deg/s, the shorter ones then holding their end."""
        start = dict(self.goal)
        travel = {n: abs(target[n] - start[n]) for n in target}
        longest = max(travel.values())
        if longest == 0:
            return
        t_start = time.monotonic()
        while True:
            done = (time.monotonic() - t_start) * speed
            for n, v in target.items():
                along = travel[n] if each else longest
                s = min(1.0, done / along) if along > 0 else 1.0
                self.goal[n] = start[n] + s * (v - start[n])
            self.tick(label, joints)
            if done >= longest:
                break
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - self.t0) % (1 / HZ)))
        t_hold = time.monotonic()
        while time.monotonic() - t_hold < HOLD_S:
            self.tick(label, joints)
            time.sleep(1 / HZ)

    def move(self, target):
        """Staged change of pose: raise the hip pitches, move the rest, lower the hip pitches."""
        goal = {**self.goal, **target}
        raise_ = {"left_hip_pitch": min(self.goal["left_hip_pitch"], goal["left_hip_pitch"]),
                  "right_hip_pitch": max(self.goal["right_hip_pitch"], goal["right_hip_pitch"])}
        self.ramp(raise_, MOVE_SPEED)
        self.ramp({n: v for n, v in goal.items() if n not in raise_}, MOVE_SPEED)
        self.ramp({n: goal[n] for n in raise_}, MOVE_SPEED)

    def play(self):
        self.move(CHECK)
        for pose in APPROACH:
            self.move(pose)
        for hold, ends in SWEEPS:
            self.move({**hold, **{n: s for n, (s, _) in ends.items()}})
            for k in range(PASSES):
                self.ramp({n: e for n, (_, e) in ends.items()}, SWEEP_SPEED, f"out{k}", ends, each=True)
                self.ramp({n: s for n, (s, _) in ends.items()}, SWEEP_SPEED, f"back{k}", ends, each=True)
            self.move(UP)
        for pose in reversed(APPROACH[:-1]):
            self.move(pose)
        self.move(CHECK)


def verdicts(samples, speed):
    """[(joint, irregularity, peak, verdict, angles slipping again)] per swept joint, in sweep order."""
    rows = defaultdict(list)
    for t, joint, phase, goal, pos, cur, vel, pwm in samples:
        if phase and phase.startswith(("out", "back")):
            rows[joint].append((t, phase, goal, pos, vel))
    report = []
    for joint, rr in rows.items():
        traversals = defaultdict(list)
        for r in rr:
            traversals[r[1]].append(r)
        irr = peak = 0.0
        slips = []  # (traversal, angle) where the velocity exceeds PEAK x the command
        for phase, tr in traversals.items():
            moving = [r for r in tr if abs(r[2] - tr[-1][2]) > 0.5]  # leave out the hold at the end
            if len(moving) < 10:
                continue
            lo, hi = min(r[3] for r in moving), max(r[3] for r in moving)
            edge = EDGE_DEG if hi - lo > 2 * EDGE_DEG + 4 else (hi - lo) / 3
            mid = [r for r in moving if lo + edge < r[3] < hi - edge]
            last = -9.0
            for r in mid:
                if abs(r[4]) > PEAK * speed and r[0] - last > 0.4:
                    slips.append((phase, r[3]))
                    last = r[0]
            if len(mid) >= 10:
                v = [abs(r[4]) for r in mid]
                irr, peak = max(irr, statistics.pstdev(v) / speed), max(peak, max(v) / speed)
        repeats = sorted({round(a) for ph, a in slips if any(p2 != ph and abs(a2 - a) <= REPEAT_DEG for p2, a2 in slips)})
        spots = sum(1 for k, a in enumerate(repeats) if k == 0 or a - repeats[k - 1] > SPOT_DEG)
        verdict = ("stick-slip" if spots > MAX_SPOTS else "DAMAGED" if repeats and irr > DAMAGED_IRR else
                   "repeat" if repeats else "irregular" if irr > IRREGULAR or peak > PEAK else "ok")
        report.append((joint, irr, peak, verdict, repeats))
    return report


def die(msg):
    print(msg, file=sys.stderr)
    sys.exit(2)


def bus_port():
    port = "/dev/ttyS2"
    for path in ("/etc/robot/robotd.toml", "/var/lib/robot/robotd.toml"):  # the user layer wins
        try:
            with open(path, "rb") as f:
                port = tomllib.load(f).get("bus", {}).get("port", port)
        except FileNotFoundError:
            pass
    return port


def gravity_x():
    """The trunk's gravity x from robotd: -1 lying on its back."""
    try:
        mon = subprocess.Popen(["robotctl", "monitor", "--json", "--hz", "2"], stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, text=True)
    except FileNotFoundError:
        die("robotctl not found: is this a microduck?")
    with mon:
        try:
            if select.select([mon.stdout], [], [], 5)[0]:
                return json.loads(mon.stdout.readline())["params"]["safety"]["gravity"][0]
        except (ValueError, KeyError, IndexError, TypeError):
            pass
        finally:
            mon.kill()
    die("cannot read the robot's orientation: robotd must be running to start the test")


def workout(port):
    """Play the workout: (samples, why it stopped early or None, servos that did not answer)."""
    bus = Bus(port)
    goal, gains = {}, {}
    for n, i in IDS.items():
        p, g = bus.read(i, PRESENT + 8, 4), bus.read(i, P_GAIN, 2)
        if p is None or g is None:
            continue
        goal[n], gains[i] = deg(struct.unpack("<i", p)[0]), g  # torque goes on where it is: nothing jumps
    missing = [n for n in NAMES if n not in goal]
    if missing:
        return [], None, missing
    w = Workout(bus, goal)
    try:
        for n in STIFF_HEAD:
            bus.write(IDS[n], P_GAIN, struct.pack("<H", min(16383, STIFF_FACTOR * struct.unpack("<H", gains[IDS[n]])[0])))
        for n, i in IDS.items():
            bus.write(i, GOAL_POS, struct.pack("<i", raw(goal[n])))
            bus.write(i, TORQUE, b"\x01")
        w.play()
        stopped = None
    except Blocked as e:
        stopped = str(e)
    finally:
        for i in IDS.values():
            bus.write(i, TORQUE, b"\x00")
            bus.write(i, P_GAIN, gains[i])
    return w.samples, stopped, []


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--save", metavar="FILE", help="write the recording here, as JSON")
    args = ap.parse_args()
    if os.geteuid() != 0:
        die("run it as root: sudo python3 workout_test.py")
    port = bus_port()
    if gravity_x() > -0.85:
        die("lay the robot on its back first")
    subprocess.run(["systemctl", "stop", "robotd"], check=True)
    print("robot on its back; workout started (about 3.5 minutes)", flush=True)
    try:
        samples, stopped, missing = workout(port)
    except KeyboardInterrupt:
        die("interrupted")
    except OSError as e:
        die(f"servo bus {port}: {e}")
    finally:
        subprocess.run(["systemctl", "start", "robotd"], check=False)  # a failure must not hide the result
    if args.save:
        with open(args.save, "w") as f:
            json.dump({"started": time.time(), "speed_deg_s": SWEEP_SPEED, "passes": PASSES, "stopped": stopped,
                       "columns": ["t", "joint", "phase", "goal_deg", "pos_deg", "cur_ma", "vel_deg_s", "pwm_pct"],
                       "samples": samples}, f)
    if missing:
        print(f"FAIL: no answer from {', '.join(missing)}")
        sys.exit(1)
    report = verdicts(samples, SWEEP_SPEED)
    print(f"{'joint':16s} {'irregularity':>12s} {'peak/cmd':>9s}  result")
    for joint, irr, peak, verdict, repeats in report:
        if verdict == "DAMAGED" and joint in HEAD:
            verdict = "suspect (warning)"
        print(f"{joint:16s} {irr:12.2f} {peak:9.2f}  {verdict}" + (f", slips again at {repeats} deg" if repeats else ""))
    damaged = [j for j, _, _, v, _ in report if v == "DAMAGED" and j not in HEAD]
    if stopped:
        print(f"FAIL: stopped, {stopped} (a damaged gear, or something in the way)")
    elif damaged:
        print(f"FAIL: {', '.join(damaged)} stick and slip at the same angle: check them by hand")
    else:
        print("PASS")
    sys.exit(1 if stopped or damaged else 0)


if __name__ == "__main__":
    main()
