#!/usr/bin/env python3
"""Battery-life and heat test: run a duck through ordinary use until the battery is empty.

Runs **on the robot**, standard library only. It drives the robot through `robotd`'s socket the
way any client does — `robot.enable`, `robot.move` at 20 Hz, `robot.do sit_toggle` — and cycles
through walking, standing and sitting in random order and for random lengths, so the load is the
mix a duck sees in normal use rather than one steady state. The camera is not streamed.

**It stays home.** Walking is goal-to-goal on odometry (`robot.state`'s `odom`): goals are drawn
inside `--goal-radius` of where it started, and past `--fence` it abandons the goal and walks
back to the start. Odometry drifts — there is no absolute position — so it keeps the robot near
where *it thinks* it began; put it on a clear patch of floor with a margin.

**It runs until the board dies.** `robotd` sits the robot down and powers the board off when the
battery reaches 0% (`[safety] battery_empty_shutdown`, on by default). Every sample is flushed and
`fsync`ed as it is written, so the log is complete up to the last interval after the reboot.

Each run writes a directory:

    samples.csv   one row every --interval s: battery, board temperature per thermal zone,
                  throttle (cooling level and clock ceiling), motor temperature, motor current,
                  what the robot was doing and where odometry had it
    events.csv    activity changes, refusals, falls, fence trips, lost connections
    meta.json     arguments, host, start time

Run it detached from your ssh session, as root (intents need it), with no pad connected — a pad's
idle stick sends zero velocities that would fight this one:

    scp scripts/battery-test/endurance.py microduck@<duck>:
    ssh microduck@<duck>
    sudo systemd-run --unit=battery-test --collect python3 /home/microduck/endurance.py
    journalctl -u battery-test -f          # progress
    sudo systemctl stop battery-test       # abort: the robot is stopped and left standing

After the shutdown, charge it, boot it, and fetch the run:

    scp -r microduck@<duck>:/var/lib/battery-test/ .
    uv run scripts/battery-test/plot.py battery-test/<run>

`--dry-run` logs without moving the robot, which is also the idle-drain baseline.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import signal
import socket
import sys
import threading
import time
from pathlib import Path

ROBOT_SOCKET = "/run/robotd.sock"
THERMAL_ROOT = Path("/sys/class/thermal")
CPUFREQ_ROOT = Path("/sys/devices/system/cpu/cpufreq")

# `robotd`'s deadman zeroes the twist 500 ms after the last intent; 20 Hz is well inside it.
MOVE_HZ = 20.0


# ── the wire ────────────────────────────────────────────────────────────────────────────────


class Rpc:
    """JSON-RPC 2.0 over `robotd`'s unix socket, one object per line. One request at a time."""

    def __init__(self, timeout: float = 5.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(ROBOT_SOCKET)
        self.file = self.sock.makefile("rb")
        self.next_id = 1
        self.lock = threading.Lock()

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def _send(self, obj: dict) -> None:
        self.sock.sendall(json.dumps(obj).encode() + b"\n")

    def notify(self, method: str, params: dict | None = None) -> None:
        msg: dict = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            msg["params"] = params
        with self.lock:
            self._send(msg)

    def call(self, method: str, params: dict | None = None) -> dict:
        with self.lock:
            rid = self.next_id
            self.next_id += 1
            msg: dict = {"jsonrpc": "2.0", "id": rid, "method": method}
            if params is not None:
                msg["params"] = params
            self._send(msg)
            while True:
                line = self.file.readline()
                if not line:
                    raise ConnectionError(f"{method}: robotd closed the connection")
                reply = json.loads(line)
                if reply.get("id") != rid:
                    continue  # a stray notification; this connection never subscribes
                if "error" in reply:
                    raise RuntimeError(f"{method}: {reply['error'].get('message')}")
                return reply.get("result") or {}


class StateFeed(threading.Thread):
    """The latest `robot.state` frame, plus motor current averaged since the last sample."""

    def __init__(self, hz: int = 20):
        super().__init__(daemon=True, name="state")
        self.hz = hz
        self.lock = threading.Lock()
        self.latest: dict | None = None
        self.latest_at = 0.0
        self.current_sum = 0.0
        self.current_n = 0

    def run(self) -> None:
        while True:
            try:
                rpc = Rpc(timeout=5.0)
                rpc.call("robot.subscribe", {"hz": self.hz})
                while True:
                    line = rpc.file.readline()
                    if not line:
                        raise ConnectionError("state stream closed")
                    msg = json.loads(line)
                    if msg.get("method") != "robot.state":
                        continue
                    frame = msg.get("params") or {}
                    currents = frame.get("currents_ma") or []
                    with self.lock:
                        self.latest = frame
                        self.latest_at = time.monotonic()
                        if currents:
                            self.current_sum += sum(abs(c) for c in currents)
                            self.current_n += 1
            except (OSError, ConnectionError, ValueError, RuntimeError):
                time.sleep(2.0)

    def frame(self, max_age: float = 1.0) -> dict | None:
        with self.lock:
            if self.latest is None or time.monotonic() - self.latest_at > max_age:
                return None
            return self.latest

    def take_mean_current_ma(self) -> float | None:
        with self.lock:
            mean = self.current_sum / self.current_n if self.current_n else None
            self.current_sum, self.current_n = 0.0, 0
            return mean


# ── what the board says about itself, read directly ─────────────────────────────────────────
#
# `robot.health` carries the hottest zone and the worst throttle already. These are read here as
# well because they keep working when robotd does not, and because per-zone is what a heat test
# wants to see.


def read_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def by_index(path: Path) -> int:
    """`policy10` after `policy2`: sysfs numbers its entries, and the sort should agree."""
    digits = "".join(c for c in path.name if c.isdigit())
    return int(digits) if digits else -1


def thermal_zones() -> list[tuple[str, Path]]:
    """`(column name, temp file)` per zone. The index is in the name because types repeat."""
    zones = []
    for zone in sorted(THERMAL_ROOT.glob("thermal_zone*"), key=by_index):
        kind = (read_text(zone / "type") or "zone").replace("-", "_")
        zones.append((f"{zone.name.removeprefix('thermal_')}_{kind}", zone / "temp"))
    return zones


def cooling_devices() -> list[tuple[str, Path, Path]]:
    devs = []
    for dev in sorted(THERMAL_ROOT.glob("cooling_device*"), key=by_index):
        kind = (read_text(dev / "type") or dev.name).replace("-", "_")
        devs.append((f"{dev.name}_{kind}", dev / "cur_state", dev / "max_state"))
    return devs


def cpufreq_policies() -> list[Path]:
    return sorted(CPUFREQ_ROOT.glob("policy*"), key=by_index)


def read_int(path: Path) -> int | None:
    raw = read_text(path)
    try:
        return int(raw) if raw is not None else None
    except ValueError:
        return None


# ── the log ─────────────────────────────────────────────────────────────────────────────────


class DurableCsv:
    """A CSV appended a row at a time and `fsync`ed after each one, so a power cut keeps it."""

    def __init__(self, path: Path, fields: list[str]):
        self.fields = fields
        fresh = not path.exists() or path.stat().st_size == 0
        self.fh = open(path, "a", newline="", buffering=1)
        self.writer = csv.DictWriter(self.fh, fieldnames=fields, extrasaction="ignore")
        if fresh:
            self.writer.writeheader()
        self.lock = threading.Lock()
        self._sync()

    def _sync(self) -> None:
        self.fh.flush()
        os.fsync(self.fh.fileno())

    def write(self, row: dict) -> None:
        with self.lock:
            self.writer.writerow(row)
            self._sync()


def fsync_dir(path: Path) -> None:
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class Run:
    """Shared state between the driver and the sampler, and the two files."""

    def __init__(self, out: Path, t0_mono: float):
        self.out = out
        self.t0 = t0_mono
        self.activity = "starting"
        self.home: tuple[float, float] | None = None
        self.battery_pct: float | None = None
        self.lock = threading.Lock()
        self.events = DurableCsv(out / "events.csv", ["wall", "t_s", "kind", "detail"])

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def event(self, kind: str, detail: str = "") -> None:
        self.events.write(
            {"wall": f"{time.time():.3f}", "t_s": f"{self.elapsed():.1f}", "kind": kind, "detail": detail}
        )
        print(f"[{self.elapsed():8.1f}s] {kind}: {detail}", flush=True)

    def set_activity(self, activity: str, detail: str = "") -> None:
        with self.lock:
            self.activity = activity
        self.event("activity", f"{activity} {detail}".strip())


class Sampler(threading.Thread):
    def __init__(self, run: Run, feed: StateFeed, interval: float):
        super().__init__(daemon=True, name="sampler")
        self.run_ = run
        self.feed = feed
        self.interval = interval
        self.zones = thermal_zones()
        self.cooling = cooling_devices()
        self.policies = cpufreq_policies()
        self.robotd_up = True
        fields = [
            "wall", "t_s", "uptime_s", "robotd",
            "battery_v", "battery_pct",
            "cpu_temp_c",
            *(f"temp_{name}_c" for name, _ in self.zones),
            "throttled", "throttle_level", "throttle_max_level", "cpu_khz_ceiling", "cpu_khz_max",
            *(f"{p.name}_cur_khz" for p in self.policies),
            *(f"{p.name}_scaling_max_khz" for p in self.policies),
            *(name for name, _, _ in self.cooling),
            "load1",
            "motor_max_c", "motor_mean_c", "motor_hottest",
            "motor_current_ma",
            "loop_hz",
            "activity", "policy", "fallen", "picked_up",
            "odom_x", "odom_y", "odom_yaw", "odom_dist",
            "healthy", "reason",
        ]
        self.csv = DurableCsv(run.out / "samples.csv", fields)

    def run(self) -> None:
        next_at = time.monotonic()
        while True:
            try:
                self.sample()
            except Exception as e:  # noqa: BLE001 — a bad sample must never end the log
                self.run_.event("sampler_error", repr(e))
            next_at += self.interval
            time.sleep(max(0.0, next_at - time.monotonic()))

    def sample(self) -> None:
        row: dict = {"wall": f"{time.time():.3f}", "t_s": f"{self.run_.elapsed():.1f}"}
        uptime = read_text(Path("/proc/uptime"))
        row["uptime_s"] = uptime.split()[0] if uptime else ""
        load = read_text(Path("/proc/loadavg"))
        row["load1"] = load.split()[0] if load else ""

        for name, path in self.zones:
            milli = read_int(path)
            row[f"temp_{name}_c"] = f"{milli / 1000:.1f}" if milli else ""
        for policy in self.policies:
            row[f"{policy.name}_cur_khz"] = read_int(policy / "scaling_cur_freq") or ""
            row[f"{policy.name}_scaling_max_khz"] = read_int(policy / "scaling_max_freq") or ""
        for name, cur, _ in self.cooling:
            level = read_int(cur)
            row[name] = level if level is not None else ""

        try:
            rpc = Rpc(timeout=3.0)
            try:
                health = rpc.call("robot.health")
            finally:
                rpc.close()
            row["robotd"] = "up"
            if not self.robotd_up:
                self.run_.event("health_available", "robotd answers again")
            self.robotd_up = True
        except (OSError, ConnectionError, ValueError, RuntimeError) as e:
            health = {}
            row["robotd"] = "down"
            # Once per outage: the last minute before a power-off is all outage.
            if self.robotd_up:
                self.run_.event("health_unavailable", repr(e))
            self.robotd_up = False

        if battery := health.get("battery"):
            row["battery_v"] = f"{battery['volts']:.3f}"
            row["battery_pct"] = f"{battery['percent']:.1f}"
            with self.run_.lock:
                self.run_.battery_pct = battery["percent"]
        if (cpu := health.get("cpu_temp_c")) is not None:
            row["cpu_temp_c"] = f"{cpu:.1f}"
        if throttle := health.get("cpu_throttle"):
            row["throttle_level"] = throttle.get("level")
            row["throttle_max_level"] = throttle.get("max_level")
            row["cpu_khz_ceiling"] = throttle.get("khz")
            row["cpu_khz_max"] = throttle.get("max_khz")
            row["throttled"] = int(
                throttle.get("level", 0) > 0 or throttle.get("khz", 0) < throttle.get("max_khz", 0)
            )
        if motors := health.get("motors"):
            row["motor_max_c"] = motors.get("max_c")
            row["motor_mean_c"] = f"{motors.get('mean_c', 0):.1f}"
            row["motor_hottest"] = motors.get("hottest")
        if loop := health.get("control_loop"):
            hz = loop.get("achieved_hz")
            row["loop_hz"] = f"{hz:.1f}" if hz is not None else ""
        row["healthy"] = int(bool(health.get("healthy"))) if health else ""
        row["reason"] = health.get("reason") or ""

        current = self.feed.take_mean_current_ma()
        row["motor_current_ma"] = f"{current:.0f}" if current is not None else ""

        with self.run_.lock:
            row["activity"] = self.run_.activity
            home = self.run_.home
        if frame := self.feed.frame():
            safety = frame.get("safety") or {}
            row["policy"] = frame.get("policy", "")
            row["fallen"] = int(bool(safety.get("fallen")))
            row["picked_up"] = int(bool(safety.get("picked_up")))
            odom = frame.get("odom") or {}
            pos = odom.get("position") or [0.0, 0.0, 0.0]
            row["odom_x"] = f"{pos[0]:.3f}"
            row["odom_y"] = f"{pos[1]:.3f}"
            row["odom_yaw"] = f"{odom.get('yaw', 0.0):.3f}"
            if home:
                row["odom_dist"] = f"{math.hypot(pos[0] - home[0], pos[1] - home[1]):.3f}"

        self.csv.write(row)
        print(
            f"[{self.run_.elapsed():8.1f}s] batt {row.get('battery_pct', '?')}% "
            f"({row.get('battery_v', '?')} V)  cpu {row.get('cpu_temp_c', '?')} °C  "
            f"throttled={row.get('throttled', '?')}  {row['activity']}",
            flush=True,
        )


# ── the behaviour ───────────────────────────────────────────────────────────────────────────


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Stop(Exception):
    pass


class HandsOff(Exception):
    """The battery is nearly empty, or robotd has started its own shutdown: stop driving.

    At 0% `robotd` sits the robot down and powers off by itself. A `sit_toggle` or an enable
    sent into the middle of that is a client fighting the daemon's shutdown, so near the end
    this script lets go and only logs.
    """


class Driver:
    def __init__(self, run: Run, feed: StateFeed, args: argparse.Namespace, stop: threading.Event):
        self.run = run
        self.feed = feed
        self.args = args
        self.stop = stop
        self.rng = random.Random(args.seed)
        self.rpc: Rpc | None = None
        # Whether the robot is in a sit this script asked for. A sit it did not ask for is
        # robotd's battery shutdown, and must not be undone.
        self.our_sit = False

    # The connection is reopened whenever it drops: a robotd restart mid-test should cost a few
    # seconds of behaviour, not the run.
    def conn(self) -> Rpc:
        while self.rpc is None:
            self.check_stop()
            try:
                self.rpc = Rpc(timeout=5.0)
            except OSError as e:
                self.run.event("robotd_unreachable", repr(e))
                self.wait(5.0)
        return self.rpc

    def drop(self, why: Exception) -> None:
        self.run.event("connection_lost", repr(why))
        if self.rpc:
            self.rpc.close()
        self.rpc = None

    def call(self, method: str, params: dict | None = None) -> dict | None:
        try:
            return self.conn().call(method, params)
        except (OSError, ConnectionError, ValueError) as e:
            self.drop(e)
            return None

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        try:
            self.conn().notify("robot.move", {"vx": vx, "vy": vy, "vyaw": vyaw})
        except OSError as e:
            self.drop(e)

    def check_stop(self) -> None:
        if self.stop.is_set():
            raise Stop

    def wait(self, seconds: float) -> None:
        if self.stop.wait(seconds):
            raise Stop

    def policy(self) -> str:
        frame = self.feed.frame()
        return frame.get("policy", "") if frame else ""

    def pose(self) -> tuple[float, float, float] | None:
        frame = self.feed.frame()
        if not frame or "odom" not in frame:
            return None
        odom = frame["odom"]
        return odom["position"][0], odom["position"][1], odom["yaw"]

    def down(self) -> str | None:
        """Why the robot should not be driven right now, or None."""
        frame = self.feed.frame()
        if frame is None:
            return "no state from robotd"
        safety = frame.get("safety") or {}
        if safety.get("fallen"):
            return "fallen"
        if safety.get("picked_up"):
            return "picked up"
        return None

    def hold_still(self, seconds: float) -> None:
        """Zero twist at the intent rate. Standing is a policy driving, not an absence of intents."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.move(0.0, 0.0, 0.0)
            self.wait(1.0 / MOVE_HZ)

    def wait_until_upright(self) -> None:
        reason = self.down()
        if reason is None:
            return
        self.run.set_activity("paused", reason)
        while (reason := self.down()) is not None:
            self.move(0.0, 0.0, 0.0)
            self.wait(0.5)
        self.run.event("resumed", "upright and on the floor again")
        self.hold_still(2.0)

    def check_battery(self) -> None:
        with self.run.lock:
            pct = self.run.battery_pct
        if pct is not None and pct <= self.args.hands_off_pct:
            raise HandsOff(f"battery at {pct:.1f}%")
        label = self.policy()
        if label == "rest" or (label == "sit" and not self.our_sit):
            raise HandsOff(f"robot went to {label!r} on its own")

    def ensure_driving(self) -> None:
        self.check_battery()
        if self.policy() in ("walk", "stand"):
            return
        if self.policy() == "sit":
            self.rise()
            return
        result = self.call("robot.enable", {"on": True})
        self.run.event("enable", json.dumps(result))
        deadline = time.monotonic() + 30.0
        while self.policy() not in ("walk", "stand"):
            if time.monotonic() > deadline:
                self.run.event("enable_timeout", f"policy still {self.policy()!r}")
                return
            self.move(0.0, 0.0, 0.0)
            self.wait(0.2)
        self.hold_still(2.0)

    # ── activities ──

    def stand(self, seconds: float) -> None:
        self.run.set_activity("stand", f"{seconds:.0f}s")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.down():
                return
            self.check_battery()
            self.move(0.0, 0.0, 0.0)
            self.wait(1.0 / MOVE_HZ)

    def sit(self, seconds: float) -> None:
        self.run.set_activity("sit", f"{seconds:.0f}s")
        self.hold_still(0.5)
        result = self.call("robot.do", {"skill": "sit_toggle"})
        if not result or not result.get("accepted"):
            self.run.event("sit_refused", json.dumps(result))
            self.hold_still(seconds)
            return
        self.our_sit = True
        deadline = time.monotonic() + 15.0
        while self.policy() != "sit" and time.monotonic() < deadline:
            self.wait(0.1)
        if self.policy() != "sit":
            self.run.event("sit_timeout", f"policy {self.policy()!r}")
        # Seated: nothing to send. The seat is latched, and a twist would not move a sitting duck.
        # Checked once a second, so a shutdown that starts while seated is not stood back up.
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.wait(1.0)
            self.check_battery()
        self.rise()

    def rise(self) -> None:
        self.check_battery()
        self.our_sit = False
        result = self.call("robot.do", {"skill": "sit_toggle"})
        if not result or not result.get("accepted"):
            self.run.event("rise_refused", json.dumps(result))
        deadline = time.monotonic() + 15.0
        while self.policy() not in ("walk", "stand") and time.monotonic() < deadline:
            self.wait(0.1)
        if self.policy() not in ("walk", "stand"):
            self.run.event("rise_timeout", f"policy {self.policy()!r}")
        self.hold_still(1.0)

    def new_goal(self, x: float, y: float, home: tuple[float, float]) -> tuple[float, float]:
        """A point inside the goal disc, far enough from here that getting there is a walk."""
        r_max = self.args.goal_radius
        for _ in range(50):
            r = r_max * math.sqrt(self.rng.random())
            a = self.rng.uniform(-math.pi, math.pi)
            gx, gy = home[0] + r * math.cos(a), home[1] + r * math.sin(a)
            if math.hypot(gx - x, gy - y) >= 0.6 * r_max:
                return gx, gy
        return home

    def walk(self, seconds: float) -> None:
        self.run.set_activity("walk", f"{seconds:.0f}s")
        home = self.run.home
        assert home is not None
        a = self.args
        goal: tuple[float, float] | None = None
        returning = False
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.down():
                return
            self.check_battery()
            pose = self.pose()
            if pose is None:
                self.move(0.0, 0.0, 0.0)
                self.wait(1.0 / MOVE_HZ)
                continue
            x, y, yaw = pose
            from_home = math.hypot(x - home[0], y - home[1])
            if from_home > a.fence and not returning:
                self.run.event("fence", f"{from_home:.2f} m from start; walking back")
                goal, returning = home, True
            if goal is None:
                goal = self.new_goal(x, y, home)
            dx, dy = goal[0] - x, goal[1] - y
            dist = math.hypot(dx, dy)
            if dist < a.arrive:
                goal, returning = None, False
                # A short pause at each goal, as a duck being steered around does.
                self.hold_still(self.rng.uniform(0.5, 2.0))
                continue
            err = wrap(math.atan2(dy, dx) - yaw)
            if abs(err) > a.turn_threshold:
                # Turn on the spot toward the goal.
                vx, vyaw = 0.0, math.copysign(a.vyaw, err)
            else:
                vx = a.speed
                vyaw = max(-a.vyaw, min(a.vyaw, 2.5 * err))
            self.move(vx, 0.0, vyaw)
            self.wait(1.0 / MOVE_HZ)
        self.hold_still(0.5)

    # ── the loop ──

    def run_forever(self) -> None:
        a = self.args
        self.ensure_driving()
        deadline = time.monotonic() + 5.0
        while (pose := self.pose()) is None:
            if time.monotonic() > deadline:
                raise SystemExit("no odometry on robot.state — is this robotd new enough?")
            self.wait(0.1)
        x, y, _ = pose
        with self.run.lock:
            self.run.home = (x, y)
        self.run.event("home", f"odometry start ({x:.3f}, {y:.3f})")

        weights = {"walk": a.w_walk, "stand": a.w_stand, "sit": a.w_sit}
        spans = {"walk": a.walk_s, "stand": a.stand_s, "sit": a.sit_s}
        last = None
        while True:
            self.wait_until_upright()
            self.ensure_driving()
            # Never the same activity twice in a row: three stands in a row is one long stand.
            choices = [k for k, w in weights.items() if w > 0 and k != last] or list(weights)
            kind = self.rng.choices(choices, weights=[weights[k] for k in choices])[0]
            lo, hi = spans[kind]
            seconds = self.rng.uniform(lo, hi)
            getattr(self, kind)(seconds)
            last = kind

    def park(self) -> None:
        """On the way out: stop and leave the robot standing (rise if it was sitting)."""
        self.stop.clear()
        try:
            if self.policy() == "sit" and self.our_sit:
                self.rise()
            self.call("robot.stop")
        except Exception:  # noqa: BLE001
            pass


# ── main ────────────────────────────────────────────────────────────────────────────────────


def span(text: str) -> tuple[float, float]:
    lo, _, hi = text.partition(":")
    lo_f, hi_f = float(lo), float(hi or lo)
    if not 0 < lo_f <= hi_f:
        raise argparse.ArgumentTypeError(f"{text!r}: want MIN:MAX seconds")
    return lo_f, hi_f


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--out", type=Path, default=None,
                   help="run directory (default /var/lib/battery-test/<hostname>-<UTC time>); "
                        "an existing one is appended to")
    p.add_argument("--interval", type=float, default=10.0, help="seconds between samples (10)")
    p.add_argument("--seed", type=int, default=None, help="RNG seed for the activity schedule")
    p.add_argument("--socket", default="/run/robotd.sock", help=argparse.SUPPRESS)
    p.add_argument("--dry-run", action="store_true", help="log only; never move the robot")
    g = p.add_argument_group("behaviour")
    g.add_argument("--w-walk", type=float, default=0.40, help="share of activities that are walks")
    g.add_argument("--w-stand", type=float, default=0.35, help="share that are stands")
    g.add_argument("--w-sit", type=float, default=0.25, help="share that are sits")
    g.add_argument("--walk-s", type=span, default=(20, 60), help="walk length MIN:MAX s (20:60)")
    g.add_argument("--stand-s", type=span, default=(20, 90), help="stand length MIN:MAX s (20:90)")
    g.add_argument("--sit-s", type=span, default=(30, 120), help="sit length MIN:MAX s (30:120)")
    # Full stick, the pad's own limits (`[pad_drive]` defaults). A gentler command does not
    # always get the gait going from a stand: the policy can sit on a small one and not step.
    g.add_argument("--speed", type=float, default=0.3, help="walking speed, m/s (0.3)")
    g.add_argument("--vyaw", type=float, default=1.5, help="turn rate, rad/s (1.5)")
    g.add_argument("--goal-radius", type=float, default=0.30,
                   help="goals are drawn inside this many m of the start (0.30)")
    g.add_argument("--fence", type=float, default=0.45,
                   help="past this many m from the start, walk straight back (0.45)")
    g.add_argument("--arrive", type=float, default=0.08, help="goal reached within this, m (0.08)")
    g.add_argument("--hands-off-pct", type=float, default=2.0,
                   help="at or below this battery %%, stop driving and let robotd's own "
                        "empty-battery shutdown run undisturbed (2)")
    g.add_argument("--turn-threshold", type=float, default=0.6,
                   help="heading error above which it turns on the spot, rad (0.6)")
    args = p.parse_args()

    globals()["ROBOT_SOCKET"] = args.socket
    if not os.path.exists(args.socket):
        print(f"no {args.socket}: is robotd running?", file=sys.stderr)
        return 1

    out = args.out or Path("/var/lib/battery-test") / (
        f"{socket.gethostname()}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
    )
    out.mkdir(parents=True, exist_ok=True)
    fsync_dir(out.parent)

    meta_path = out / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {"segments": []}
    meta["segments"].append({
        "started_wall": time.time(),
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "boot_id": read_text(Path("/proc/sys/kernel/random/boot_id")),
        "host": socket.gethostname(),
        "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
    })
    meta_path.write_text(json.dumps(meta, indent=2))
    fsync_dir(out)

    run = Run(out, time.monotonic())
    run.event("start", f"logging to {out}" + (" (dry run)" if args.dry_run else ""))

    feed = StateFeed()
    feed.start()
    Sampler(run, feed, args.interval).start()

    stop = threading.Event()

    def on_signal(signum, _):
        run.event("signal", signal.Signals(signum).name)
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    if args.dry_run:
        run.set_activity("dry_run")
        stop.wait()
        return 0

    driver = Driver(run, feed, args, stop)
    try:
        driver.run_forever()
    except HandsOff as why:
        # Keep logging, touch nothing: the rows from here to the power-off are the end of the
        # curve, which is the part this test exists for.
        run.set_activity("hands_off", str(why))
        stop.wait()
        run.event("stop", "stopped while hands-off")
        return 0
    except Stop:
        pass
    driver.park()
    run.event("stop", "robot stopped and left standing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
