"""Draw a maploc occupancy grid into a MuJoCo passive viewer's `user_scn`.

The map crosses the process boundary as robotd's `robot.map` stream: JSON-RPC
`map.frame` notifications on robotd's unix socket, one per second, carrying a
base64 grid of one byte per cell (0 unknown, 1 free, 2 wall) plus the pose.
Nothing here knows about SLAM; it decodes MapFrame and emits boxes.

Two entry points:
  * `MapSource` — a background thread that subscribes to robotd and keeps the
    latest frame in a lock-protected slot.
  * `draw(user_scn, frame, ...)` — call from the thread that owns `viewer.sync()`.
"""

from __future__ import annotations

import base64
import json
import socket
import threading
from dataclasses import dataclass

import mujoco
import numpy as np

UNKNOWN, FREE, WALL = 0, 1, 2

# Where the overlay floats. The apartment's walls run z in [0, 0.50]; the free
# tiles sit just above the floor and the wall cells just under the real wall
# tops, so the overlay reads against the geometry instead of hiding inside it.
FREE_Z = 0.004
FREE_H = 0.002
WALL_Z = 0.26
WALL_H = 0.26

COLOR_FREE = (0.20, 0.55, 0.95, 0.28)
COLOR_WALL = (1.00, 0.35, 0.15, 0.55)
COLOR_POSE = (1.00, 0.90, 0.10, 0.95)
COLOR_TRAIL = (1.00, 0.20, 0.40, 0.85)


@dataclass
class MapFrame:
    seq: int
    x: float
    y: float
    yaw: float
    tracking: bool
    x_min: float
    y_min: float
    cell_m: float
    rows: int
    cols: int
    cells: np.ndarray  # uint8, shape (rows, cols), row 0 at y_min
    n_submaps: int
    n_loops: int
    windows: int
    still: bool
    seated: bool

    @staticmethod
    def from_params(p: dict) -> "MapFrame":
        rows, cols = int(p["rows"]), int(p["cols"])
        raw = base64.b64decode(p["cells"])
        if len(raw) != rows * cols:
            raise ValueError(f"cells is {len(raw)} bytes, expected {rows * cols}")
        return MapFrame(
            seq=int(p["seq"]),
            x=float(p["x"]), y=float(p["y"]), yaw=float(p["yaw"]),
            tracking=bool(p["tracking"]),
            x_min=float(p["x_min"]), y_min=float(p["y_min"]),
            cell_m=float(p["cell_m"]),
            rows=rows, cols=cols,
            cells=np.frombuffer(raw, dtype=np.uint8).reshape(rows, cols),
            n_submaps=int(p.get("n_submaps", 0)),
            n_loops=int(p.get("n_loops", 0)),
            windows=int(p.get("windows", 0)),
            still=bool(p.get("still", False)),
            seated=bool(p.get("seated", False)),
        )

    def caption(self) -> str:
        return (
            f"seq {self.seq} · {self.rows}x{self.cols} @ {self.cell_m:.2f} m · "
            f"{self.n_submaps} submaps · {self.n_loops} loops · {self.windows} windows · "
            f"pose ({self.x:+.2f}, {self.y:+.2f}, {np.degrees(self.yaw):+.0f} deg) · "
            f"{'tracking' if self.tracking else 'LOST'}"
        )


class MapSource:
    """Subscribe to robotd's `robot.map` stream on a background thread."""

    def __init__(self, sock_path: str):
        self.sock_path = sock_path
        self._lock = threading.Lock()
        self._frame: MapFrame | None = None
        self._status = "connecting"
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "MapSource":
        self.thread.start()
        return self

    def latest(self):
        with self._lock:
            return self._frame, self._status

    def _run(self) -> None:
        backoff = 0.5
        while not self._stop.is_set():
            try:
                s = socket.socket(socket.AF_UNIX)
                s.settimeout(10)
                s.connect(self.sock_path)
                f = s.makefile("rw")
                f.write(json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.map",
                                    "params": {}}) + "\n")
                f.flush()
                reply = json.loads(f.readline())
                res = reply.get("result", {})
                with self._lock:
                    self._status = (
                        f"subscribed (enabled={res.get('enabled')} mode={res.get('mode')})"
                    )
                backoff = 0.5
                s.settimeout(None)
                for line in f:
                    if self._stop.is_set():
                        break
                    try:
                        msg = json.loads(line)
                    except ValueError:
                        continue
                    if msg.get("method") != "map.frame":
                        continue
                    frame = MapFrame.from_params(msg["params"])
                    with self._lock:
                        self._frame = frame
                        self._status = "streaming"
            except Exception as error:  # noqa: BLE001 - a dead robotd is one reconnect
                with self._lock:
                    self._status = f"reconnecting ({type(error).__name__}: {error})"
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 5.0)


def _runs(row: np.ndarray, value: int):
    """Merge equal neighbours into (start, length) runs — far fewer boxes."""
    mask = row == value
    if not mask.any():
        return
    idx = np.flatnonzero(np.diff(np.concatenate(([0], mask.view(np.int8), [0]))))
    for a, b in zip(idx[0::2], idx[1::2]):
        yield int(a), int(b - a)


def _add(scn, type_, size, pos, mat, rgba) -> bool:
    if scn.ngeom >= scn.maxgeom:
        return False
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, type_, np.asarray(size, dtype=np.float64),
                        np.asarray(pos, dtype=np.float64),
                        np.asarray(mat, dtype=np.float64).reshape(9),
                        np.asarray(rgba, dtype=np.float32))
    g.category = mujoco.mjtCatBit.mjCAT_DECOR
    scn.ngeom += 1
    return True


_EYE = np.eye(3).reshape(9)


def draw(scn, frame: MapFrame, trail=None, show_free=True) -> int:
    """Rebuild `scn` from one MapFrame. Returns the geom count used.

    Must be called from the thread that owns `viewer.sync()`: `user_scn` is
    copied into the render scene by `sync()`, so mutating it from a socket
    thread races that copy.
    """
    scn.ngeom = 0
    c = frame.cell_m
    half = c / 2.0

    for r in range(frame.rows):
        y = frame.y_min + (r + 0.5) * c
        row = frame.cells[r]
        if show_free:
            for start, length in _runs(row, FREE):
                x0 = frame.x_min + start * c
                _add(scn, mujoco.mjtGeom.mjGEOM_BOX,
                     (length * half, half, FREE_H),
                     (x0 + length * half, y, FREE_Z), _EYE, COLOR_FREE)
        for start, length in _runs(row, WALL):
            x0 = frame.x_min + start * c
            _add(scn, mujoco.mjtGeom.mjGEOM_BOX,
                 (length * half, half, WALL_H),
                 (x0 + length * half, y, WALL_Z), _EYE, COLOR_WALL)

    # The tracked path, in the map frame — deliberately separate from odometry,
    # because after a loop closure the two frames differ.
    if trail:
        for (px, py) in trail[::2]:
            _add(scn, mujoco.mjtGeom.mjGEOM_SPHERE, (0.025, 0, 0),
                 (px, py, 0.05), _EYE, COLOR_TRAIL)

    # Pose estimate: a shaft along +x rotated by yaw, so heading is readable.
    ca, sa = np.cos(frame.yaw), np.sin(frame.yaw)
    mat = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]], dtype=np.float64)
    rgba = COLOR_POSE if frame.tracking else (1.0, 0.1, 0.1, 0.95)
    _add(scn, mujoco.mjtGeom.mjGEOM_ARROW, (0.035, 0.035, 0.32),
         (frame.x, frame.y, 0.62), (mat @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])).reshape(9),
         rgba)
    _add(scn, mujoco.mjtGeom.mjGEOM_SPHERE, (0.07, 0, 0),
         (frame.x, frame.y, 0.62), _EYE, rgba)
    return scn.ngeom
