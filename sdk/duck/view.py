"""What the robot can see right now, as one object.

A robot program is a loop — look, decide, act — and the three things it looks at arrive on three
different schedules: `robot.state` at whatever rate it was asked for, depth at 15 Hz, JPEG frames
at whatever `media.stream` was told. Merging those is the part every caller would otherwise write
for itself, and it is the reason the surface below `Duck.watch` exists at all.

**The merge is last-one-wins, not a queue.** A behaviour wants the freshest reading, not every
reading: a frame from two ticks ago is a frame from somewhere the robot no longer is, and a
program that falls behind should skip rather than accumulate a backlog it will act on late.

**Nothing here interprets a picture.** `frame` is the JPEG bytes as they arrived, and what is in
it is the caller's model to run — an SDK that shipped one would be a much larger dependency, and a
worse guess than whatever the caller already has.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

#: Status codes ST documents as a usable range: valid, and valid with a large pulse. The same two
#: `tof::STATUS_VALID` carries — the protocol says consumers should use `Frame::zone`'s rules
#: rather than re-deriving the thresholds, so these are those rules and not new ones.
STATUS_VALID = (5, 9)

#: "Measured, and nothing is there." Distinct from a failed measurement, which is why the wire
#: carries the status byte rather than a magic distance.
STATUS_NO_TARGET = 255


@dataclass
class View:
    """One tick's worth of everything, whatever arrived most recently.

    Every field can be `None` or empty: a robot with no camera has no frame, a duck whose ToF is
    not fitted has no depth, and the first tick of a loop may have neither yet. A behaviour that
    checks is a behaviour that survives a sensor going away, which on a real robot happens.
    """

    #: The last JPEG, exactly as the robot sent it. `None` until one arrives.
    frame: bytes | None = None

    #: The last `robot.state`: joints, odometry, the loop's rate, whether it has fallen.
    state: dict[str, Any] | None = None

    #: The last depth frame, as the wire carries it — millimetres and ST's status byte.
    depth: dict[str, Any] | None = None

    #: Seconds since the loop started, so a behaviour can time itself without a clock of its own.
    elapsed: float = 0.0

    #: Ticks so far, starting at one.
    tick: int = 0

    #: Where a zone is out of range or failed, `ranges` holds `None` in its place.
    _ranges: list[float | None] = field(default_factory=list, repr=False)

    # ── depth, interpreted ───────────────────────────────────────────────────

    @property
    def ranges(self) -> list[float | None]:
        """Every depth zone in metres, row-major, `None` where there is no usable range.

        The interpretation is `tof::Frame::zone`'s, including the part that is not obvious: a
        negative distance comes back from the sensor on a failed convergence and is not a range
        whatever the status byte says.
        """
        if self._ranges:
            return self._ranges
        if not self.depth:
            return []
        distances = self.depth.get("distance_mm") or []
        statuses = self.depth.get("status") or []
        out: list[float | None] = []
        for i, distance in enumerate(distances):
            status = statuses[i] if i < len(statuses) else STATUS_NO_TARGET
            out.append(distance / 1000.0 if status in STATUS_VALID and distance > 0 else None)
        self._ranges = out
        return out

    @property
    def nearest(self) -> float | None:
        """The closest thing the depth sensor can see, in metres, or `None` if it sees nothing.

        The one number an avoidance behaviour wants. `None` means no zone had a usable range —
        an empty room and an unfitted sensor look the same from here, which is why `depth` is
        there for a caller that needs to tell them apart.
        """
        seen = [r for r in self.ranges if r is not None]
        return min(seen) if seen else None

    # ── state, unwrapped ─────────────────────────────────────────────────────

    @property
    def fallen(self) -> bool:
        """Whether the robot is down. `False` when nothing has said yet — a behaviour should not
        act on a fall it has no evidence for."""
        return bool((self.state or {}).get("safety", {}).get("fallen", False))

    @property
    def position(self) -> tuple[float, float, float] | None:
        """Where contact odometry believes the robot is, metres, in the frame it booted in."""
        odom = (self.state or {}).get("odom")
        if not odom or "position" not in odom:
            return None
        x, y, z = odom["position"]
        return (x, y, z)

    @property
    def yaw(self) -> float | None:
        """Which way the robot is facing, radians, relative to where it booted."""
        odom = (self.state or {}).get("odom")
        return odom.get("yaw") if odom else None
