"""The pixels half of the demo: what runs on each frame.

Separate from `app.py`, and for the same reason `mediad`'s `session.rs` is transport-agnostic: it
imports OpenCV and numpy and nothing else, so it can be exercised on a synthetic frame pair without
a WebRTC stack, a robot, or a Hugging Face token. Every one of these is a few lines whose failure
mode is a wrong picture rather than an exception, which is exactly the kind of thing to run before
a Space builds rather than after.
"""

from __future__ import annotations

import cv2
import numpy as np

# The camera is mounted a quarter turn off upright and nothing on the robot rotates pixels: a
# `videoflip` in the pipeline cost the encoder its zero-copy path and the board 22 fps. So every
# consumer turns the picture itself. A client that can read `media.video` is told the angle; this
# one cannot (see the module docstring), so it is named here.
MOUNT_ROTATION_DEGREES = 90

# The focal length in pixels at the sensor's native pitch: 3.04 mm over 1.12 µm. `mediad::camera`
# derives the published intrinsics from this and the sensor mode, and this is the same arithmetic
# for the same reason — it can be checked rather than trusted.
NATIVE_FOCAL_PX = 3.04 * 1000.0 / 1.12
PINNED_SENSOR_WIDTH = 1920


def upright(frame: np.ndarray) -> np.ndarray:
    """Turn the picture the way the robot is looking.

    **Unused on the frame-stream path, and kept because it is still right for a WebRTC one.**
    `mediad`'s `stream.rs` turns a frame upright while it converts the colours, since that is a
    per-pixel loop either way and the turn is a change of which source pixel is fetched — free,
    and what receives the frames is a model that wants them the way up it was trained on. A
    consumer pulling H.264 through WebRTC gets the picture the camera took and has to do this
    itself, which is what `media.video`'s `rotate` is for.
    """
    if MOUNT_ROTATION_DEGREES % 360 == 0:
        return frame
    # `np.rot90` counts anticlockwise and the mount is measured clockwise.
    return np.ascontiguousarray(np.rot90(frame, k=-(MOUNT_ROTATION_DEGREES // 90)))


def edges(frame: np.ndarray) -> np.ndarray:
    """Canny, on the picture rather than beside it, so the alignment is visible."""
    grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    found = cv2.Canny(grey, 80, 180)
    tinted = np.zeros_like(frame)
    tinted[found > 0] = (255, 220, 40)
    return cv2.addWeighted(frame, 0.6, tinted, 1.0, 0)


def motion(frame: np.ndarray, previous: np.ndarray | None) -> np.ndarray:
    """What changed since the last frame, which on a duck is mostly the duck moving."""
    if previous is None or previous.shape != frame.shape:
        return frame
    difference = cv2.absdiff(
        cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY),
        cv2.cvtColor(previous, cv2.COLOR_RGB2GRAY),
    )
    _, mask = cv2.threshold(difference, 18, 255, cv2.THRESH_BINARY)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    tinted = np.zeros_like(frame)
    tinted[mask > 0] = (255, 60, 60)
    return cv2.addWeighted(frame, 0.7, tinted, 0.9, 0)


def flow(frame: np.ndarray, previous: np.ndarray | None) -> np.ndarray:
    """Sparse optical flow: where the corners went, which is where the robot did not.

    Lucas–Kanade on a few hundred corners rather than a dense field: it costs a few milliseconds
    on a CPU Space, and it is the same measurement visual odometry starts from.
    """
    if previous is None or previous.shape != frame.shape:
        return frame
    grey_now = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    grey_was = cv2.cvtColor(previous, cv2.COLOR_RGB2GRAY)
    corners = cv2.goodFeaturesToTrack(grey_was, maxCorners=200, qualityLevel=0.01, minDistance=12)
    if corners is None:
        return frame
    moved, found, _ = cv2.calcOpticalFlowPyrLK(grey_was, grey_now, corners, None)
    if moved is None:
        return frame

    drawn = frame.copy()
    for new, old, ok in zip(moved, corners, found.ravel()):
        if not ok:
            continue
        (x1, y1), (x0, y0) = new.ravel(), old.ravel()
        # Sub-pixel jitter on a static scene would fill the picture with noise.
        if (x1 - x0) ** 2 + (y1 - y0) ** 2 < 4:
            continue
        cv2.arrowedLine(
            drawn, (int(x0), int(y0)), (int(x1), int(y1)), (60, 220, 255), 2, tipLength=0.35
        )
    return drawn


def geometry(frame: np.ndarray) -> np.ndarray:
    """The camera's geometry, drawn on the picture it describes.

    **Derived here, and it should not be.** `media.video` publishes `fx`, `fy`, `cx`, `cy` and
    whether they were calibrated — but this consumer cannot read the robot's control channel (see
    the module docstring), so the numbers are recomputed from the sensor's optics: the focal length
    in pixels at native pitch, scaled by how much the ISP shrank the pinned 1920×1080 mode. It
    lands on the same value the robot publishes, which is the point: the arithmetic is checkable.
    """
    height, width = frame.shape[:2]
    # The frame arrives rotated by the mount, so the sensor's width is this frame's *height*.
    sensor_axis = max(width, height)
    focal = NATIVE_FOCAL_PX * sensor_axis / PINNED_SENSOR_WIDTH
    hfov = 2.0 * np.degrees(np.arctan(sensor_axis / 2.0 / focal))

    drawn = frame.copy()
    centre = (width // 2, height // 2)
    cv2.drawMarker(drawn, centre, (255, 255, 255), cv2.MARKER_CROSS, 28, 2)
    cv2.circle(drawn, centre, 6, (255, 255, 255), 1)
    for label, position in (
        (f"fx = fy = {focal:.0f} px", 26),
        (f"principal point ~ {centre[0]}, {centre[1]}", 50),
        (f"field of view ~ {hfov:.0f} deg across the long axis", 74),
        ("derived, not calibrated - media.video is the source of truth", 98),
    ):
        cv2.putText(
            drawn, label, (12, position), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA
        )
        cv2.putText(
            drawn,
            label,
            (12, position),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return drawn


FILTERS = {
    "raw": lambda frame, previous: frame,
    "edges (Canny)": lambda frame, previous: edges(frame),
    "motion": motion,
    "optical flow": flow,
    "camera geometry": lambda frame, previous: geometry(frame),
}
