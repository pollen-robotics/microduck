"""What the robot senses right now: the latest `robot.state` and depth frame, not a queue."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

# `tof::STATUS_VALID`.
STATUS_VALID = (5, 9)
STATUS_NO_TARGET = 255


@dataclass
class View:
    """One tick. Any field can be `None`: no ToF fitted, or nothing arrived yet."""

    state: dict[str, Any] | None = None
    depth: dict[str, Any] | None = None
    elapsed: float = 0.0
    tick: int = 0

    _ranges: list[float | None] = field(default_factory=list, repr=False)

    @property
    def ranges(self) -> list[float | None]:
        """Every zone in metres, row major, `None` where `tof::Frame::zone` would say no range.

        A negative distance is a failed convergence, whatever the status byte says.
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
        """The closest range in metres, or `None`. `depth` tells an empty room from no sensor."""
        seen = [r for r in self.ranges if r is not None]
        return min(seen) if seen else None

    @property
    def fallen(self) -> bool:
        """`False` until `robot.state` says otherwise."""
        return bool((self.state or {}).get("safety", {}).get("fallen", False))

    @property
    def position(self) -> tuple[float, float, float] | None:
        """Contact odometry, metres, in the frame it booted in."""
        odom = (self.state or {}).get("odom")
        if not odom or "position" not in odom:
            return None
        x, y, z = odom["position"]
        return (x, y, z)

    @property
    def yaw(self) -> float | None:
        """Radians, relative to where it booted."""
        odom = (self.state or {}).get("odom")
        return odom.get("yaw") if odom else None
