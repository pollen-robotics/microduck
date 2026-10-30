#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib>=3.9", "numpy>=1.26", "pandas>=2.1"]
# ///
"""Charts and a summary for a run of `robotctl endurance`.

    uv run scripts/endurance-plot.py <run dir> [<run dir> ...]

Writes into each run directory:

    timeline.png   battery, voltage, board temperatures, clock ceiling, motors — one panel each,
                   on a shared wall-clock axis, with measured activity/throttle intervals shaded
    breakdown.png  per-session odometry tracks and fence radii, measured time per activity,
                   battery drain rate and board temperature per activity
    summary.md     wall-clock span, sample-covered time, temperatures, throttle coverage, drain

Given several runs it also writes `compare.png` beside the first, battery against time for each.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Circle, Patch  # noqa: E402

# Categorical slots in fixed order (one per activity, never cycled), plus neutrals.
ACTIVITY_COLORS = {
    "walk": "#2a78d6",
    "stand": "#eb6834",
    "sit": "#1baf7a",
}
OTHER_COLOR = "#a8a7a1"  # paused, hands-off, starting: not part of the mix
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"
CRITICAL = "#e34948"

# A median is stable against a few stalls; five times it is a conservative "obvious gap" rule
# that still admits ordinary scheduler jitter. Fewer than three positive intervals cannot
# establish an empirical median, but a matched configured interval can still provide a fallback.
GAP_FACTOR = 5.0
MIN_CADENCE_INTERVALS = 3
# started_wall is written just before the first sample; allow a small clock/rounding skew when
# matching a sample to a start, but never use metadata alone to split a continuous t_s run.
META_CLOCK_TOLERANCE_S = 1.0
REFERENCE_FENCE_M = 0.45  # producer default, drawn only as a labelled reference circle

NUMERIC_COLUMNS = {
    "wall", "t_s", "uptime_s", "battery_v", "battery_pct", "cpu_temp_c", "throttled",
    "throttle_level", "throttle_max_level", "cpu_khz_ceiling", "cpu_khz_max", "load1",
    "motor_max_c", "motor_mean_c", "motor_current_ma", "loop_hz", "fallen", "picked_up",
    "odom_x", "odom_y", "odom_yaw", "odom_dist", "healthy",
}

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK_2,
    "axes.titlecolor": INK,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.titlelocation": "left",
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "font.size": 9,
    "lines.linewidth": 2,
    "legend.frameon": False,
})


class InputError(ValueError):
    """A run directory cannot produce a trustworthy plot."""


@dataclass
class RunData:
    run: Path
    df: pd.DataFrame
    ev: pd.DataFrame
    meta_segments: list[dict[str, Any]]
    warnings: list[str]
    events_available: bool = True


def warn(warnings: list[str], message: str) -> None:
    if message not in warnings:
        warnings.append(message)


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    return number if math.isfinite(number) else None


def numeric_column(df: pd.DataFrame, name: str) -> pd.Series:
    if name in df:
        return df[name]
    return pd.Series(np.nan, index=df.index, dtype=float)


def is_numeric_column(name: str) -> bool:
    return (name in NUMERIC_COLUMNS or name.startswith("temp_")
            or name.endswith("_cur_khz") or name.endswith("_scaling_max_khz")
            or name.startswith("cooling_device_"))


def read_meta(run: Path, warnings: list[str]) -> list[dict[str, Any]]:
    path = run / "meta.json"
    if not path.exists():
        warn(warnings, "meta.json is missing; configured fence radii are unavailable.")
        return []
    try:
        meta = json.loads(path.read_text())
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        warn(warnings, f"meta.json is unreadable ({exc}); using CSV data without segment metadata.")
        return []
    if not isinstance(meta, dict):
        warn(warnings, "meta.json is not an object; using CSV data without segment metadata.")
        return []
    raw_segments = meta.get("segments")
    if not isinstance(raw_segments, list):
        warn(warnings, "meta.json segments is missing or not an array; segment metadata is unavailable.")
        return []

    segments = []
    for index, raw in enumerate(raw_segments):
        if not isinstance(raw, dict):
            warn(warnings, f"meta.json segment {index + 1} is not an object; ignoring it.")
            continue
        started = finite_number(raw.get("started_wall"))
        if started is None:
            warn(warnings, f"meta.json segment {index + 1} has no valid started_wall; ignoring it.")
            continue
        args = raw.get("args")
        if not isinstance(args, dict):
            warn(warnings, f"meta.json segment {index + 1} has no valid args object.")
            args = {}
        fence = finite_number(args.get("fence"))
        if fence is not None and fence < 0:
            warn(warnings, f"meta.json segment {index + 1} has a negative fence radius; ignoring it.")
            fence = None
        elif "fence" in args and fence is None:
            warn(warnings, f"meta.json segment {index + 1} has an invalid fence radius; ignoring it.")
        segments.append({"index": index, "started_wall": started, "args": args, "fence": fence})

    starts = [segment["started_wall"] for segment in segments]
    if any(b <= a for a, b in zip(starts, starts[1:])):
        warn(warnings, "meta.json segment starts are not strictly increasing; refusing ambiguous metadata matches.")
        return []
    if not segments:
        warn(warnings, "meta.json has no usable segment starts; configured fence radii are unavailable.")
    return segments


def counter_pairs(df: pd.DataFrame, column: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (reset, continuous) flags for adjacent rows with two valid counter readings."""
    if column not in df or len(df) < 2:
        return np.zeros(max(len(df) - 1, 0), dtype=bool), np.zeros(max(len(df) - 1, 0), dtype=bool)
    values = df[column].to_numpy(dtype=float)
    valid = np.isfinite(values[:-1]) & np.isfinite(values[1:])
    with np.errstate(invalid="ignore"):
        reset = valid & (values[1:] < values[:-1])
    return reset, valid & ~reset


def estimate_cadence(df: pd.DataFrame, groups: np.ndarray) -> np.ndarray:
    """Median positive wall interval per provisional/final segment, or NaN when too short."""
    cadence = np.full(len(df), np.nan)
    if len(df) < 2:
        return cadence
    wall = df["wall"].to_numpy(dtype=float)
    rows = df["_row"].to_numpy(dtype=int)
    with np.errstate(invalid="ignore", over="ignore"):
        dt = np.diff(wall)
    adjacent = (np.diff(rows) == 1) & (groups[1:] == groups[:-1])
    positive = adjacent & np.isfinite(dt) & (dt > 0)
    for group in np.unique(groups):
        values = dt[positive & (groups[:-1] == group)]
        if len(values) >= MIN_CADENCE_INTERVALS:
            median = float(np.median(values))
            if math.isfinite(median) and median > 0:
                cadence[groups == group] = median
    return cadence


def assign_segments(df: pd.DataFrame, meta: list[dict[str, Any]], warnings: list[str]) -> pd.DataFrame:
    """Identify logger sessions conservatively from counters, supported by metadata starts.

    Counter decreases are direct evidence. A metadata start can add a boundary only at an
    obvious sampling gap and only when t_s does not say the logger process continued. Machine
    uptime may continue across a process restart, so it cannot veto that metadata evidence.
    Extra metadata starts without matching sample evidence never create empty sessions.
    """
    n = len(df)
    wall = df["wall"].to_numpy(dtype=float)
    t_reset, t_continuous = counter_pairs(df, "t_s")
    u_reset, _ = counter_pairs(df, "uptime_s")
    resets = t_reset | u_reset
    provisional = np.concatenate(([1], 1 + np.cumsum(resets))).astype(int)
    provisional_cadence = estimate_cadence(df, provisional)

    starts = np.array([segment["started_wall"] for segment in meta], dtype=float)

    def applicable(at: float) -> int:
        if not len(starts) or not math.isfinite(at):
            return -1
        return int(np.searchsorted(starts, at + META_CLOCK_TOLERANCE_S, side="right") - 1)

    segment = np.ones(n, dtype=int)
    meta_segment = np.full(n, -1, dtype=int)
    current_meta = applicable(wall[0]) if n else -1
    meta_floor = current_meta
    if n:
        meta_segment[0] = current_meta
    session = 1
    for i in range(1, n):
        candidate = applicable(wall[i])
        advance = candidate > meta_floor
        with np.errstate(invalid="ignore", over="ignore"):
            dt = wall[i] - wall[i - 1]
        expected = provisional_cadence[i]
        if not math.isfinite(expected) and candidate >= 0:
            configured = finite_number(meta[candidate]["args"].get("interval"))
            if configured is not None and configured > 0:
                expected = max(configured, 0.5)  # producer clamps its sampling interval to 0.5 s
        gap = math.isfinite(dt) and dt > 0 and math.isfinite(expected) and dt > GAP_FACTOR * expected
        process_continued = bool(t_continuous[i - 1])

        if resets[i - 1]:
            session += 1
            if candidate > meta_floor:
                current_meta = candidate
                meta_floor = candidate
            else:
                current_meta = -1
                meta_floor = max(meta_floor, candidate)
        elif advance:
            meta_floor = candidate
            if gap and not process_continued:
                session += 1
                current_meta = candidate
            elif process_continued:
                warn(warnings, f"meta.json start {candidate + 1} conflicts with continuous t_s; "
                               "no session boundary was invented.")
            else:
                warn(warnings, f"meta.json start {candidate + 1} lacks a matching sampling gap or counter reset; "
                               "no session boundary was invented.")
        segment[i] = session
        meta_segment[i] = current_meta

    df["segment"] = segment
    df["meta_segment"] = meta_segment
    sample_s = estimate_cadence(df, segment)
    # A three-sample session has only two intervals, too few for a median. Its matched
    # producer interval is still enough to reject an obvious stall as measured time.
    for sid in np.unique(segment):
        mask = segment == sid
        if np.isfinite(sample_s[mask]).any():
            continue
        meta_index = int(meta_segment[mask][0])
        if 0 <= meta_index < len(meta):
            configured = finite_number(meta[meta_index]["args"].get("interval"))
            if configured is not None and configured > 0:
                sample_s[mask] = max(configured, 0.5)  # producer clamps its interval to 0.5 s
    df["sample_s"] = sample_s

    t_complete = "t_s" in df and np.isfinite(df["t_s"].to_numpy(dtype=float)).all()
    u_complete = "uptime_s" in df and np.isfinite(df["uptime_s"].to_numpy(dtype=float)).all()
    if not t_complete and not u_complete:
        warn(warnings, "Neither t_s nor uptime_s is fully available; restarts cannot be ruled out "
                       "except where metadata and a sampling gap agree.")
    elif not t_complete:
        warn(warnings, "t_s is missing or incomplete; process-only restarts may be missed unless metadata identifies them.")

    if meta:
        for sid in np.unique(segment):
            if meta_segment[segment == sid][0] < 0:
                warn(warnings, f"Logger session {sid} has no matching meta.json segment; its configured fence is unavailable.")
        used = set(meta_segment[meta_segment >= 0].tolist())
        if len(used) < len(meta):
            warn(warnings, "Some meta.json starts had no matching sample boundary; no empty sessions were invented.")
    elif n and len(np.unique(segment)) > 1:
        warn(warnings, "Session boundaries were detected from counters, but meta.json segment arguments are unavailable.")

    rows = df["_row"].to_numpy(dtype=int)
    if n > 1:
        with np.errstate(invalid="ignore", over="ignore"):
            dt = np.diff(wall)
        original_adjacent = np.diff(rows) == 1
        finite_dt = np.isfinite(dt)
        if np.any(original_adjacent & finite_dt & (dt <= 0)):
            warn(warnings, "Some adjacent wall timestamps do not increase; those intervals are excluded from durations and rates.")
        same_session = original_adjacent & (segment[1:] == segment[:-1])
        cadence = df["sample_s"].to_numpy(dtype=float)[:-1]
        gaps = same_session & finite_dt & (dt > 0) & np.isfinite(cadence) & (dt > GAP_FACTOR * cadence)
        if np.any(gaps):
            warn(warnings, f"{int(gaps.sum())} obvious sampling gap(s) exceed {GAP_FACTOR:g}× the segment sampling interval "
                           "and are excluded from measured durations and rates.")
    return df


def load(run: Path) -> RunData:
    path = run / "samples.csv"
    if not path.exists():
        raise InputError(f"{run}: no samples.csv")
    try:
        df = pd.read_csv(path)
    except pd.errors.EmptyDataError as exc:
        raise InputError(f"{path}: empty samples.csv") from exc
    except (pd.errors.ParserError, UnicodeError) as exc:
        raise InputError(f"{path}: malformed samples.csv ({exc})") from exc
    except OSError as exc:
        raise InputError(f"{path}: cannot read samples.csv ({exc})") from exc
    if df.empty:
        raise InputError(f"{path}: header-only samples.csv has no samples")
    missing = [column for column in ("wall", "activity") if column not in df.columns]
    if missing:
        raise InputError(f"{path}: missing required column(s): {', '.join(missing)}")

    warnings: list[str] = []
    df = df.copy()
    df["_row"] = np.arange(len(df), dtype=int)
    wall = pd.to_numeric(df["wall"], errors="coerce").to_numpy(dtype=float)
    bad_wall = ~np.isfinite(wall)
    if bad_wall.any():
        rows = (df.index[bad_wall][:5] + 2).tolist()
        raise InputError(f"{path}: invalid/non-finite wall timestamp(s) at CSV row(s) {rows}; every sample needs a valid wall")
    activity = df["activity"]
    bad_activity = ~activity.map(lambda value: isinstance(value, str) and bool(value.strip()))
    if bad_activity.any():
        rows = (df.index[bad_activity][:5] + 2).tolist()
        raise InputError(f"{path}: missing/invalid activity at CSV row(s) {rows}; every sample needs a non-empty activity string")
    df["wall"] = wall
    df["activity"] = activity.str.strip()
    with np.errstate(over="ignore", invalid="ignore"):
        df["h"] = (df["wall"] - df["wall"].iloc[0]) / 3600.0
    if not np.isfinite(df["h"]).all():
        raise InputError(f"{path}: wall timestamp range is too large to plot")

    for column in df.columns:
        if column in {"wall", "activity", "_row", "h"} or not is_numeric_column(column):
            continue
        original = df[column]
        values = pd.to_numeric(original, errors="coerce")
        invalid = original.notna() & ~np.isfinite(values.to_numpy(dtype=float))
        if invalid.any():
            warn(warnings, f"{column}: {int(invalid.sum())} invalid/non-finite value(s) treated as unavailable.")
        df[column] = values.where(np.isfinite(values))
    if "throttled" in df:
        invalid = df["throttled"].notna() & ~df["throttled"].isin([0, 1])
        if invalid.any():
            warn(warnings, f"throttled: {int(invalid.sum())} value(s) other than 0/1 treated as unknown, not false.")
            df.loc[invalid, "throttled"] = np.nan
    if "odom_dist" in df:
        invalid = df["odom_dist"].notna() & (df["odom_dist"] < 0)
        if invalid.any():
            warn(warnings, "odom_dist: negative distance(s) treated as unavailable.")
            df.loc[invalid, "odom_dist"] = np.nan

    meta = read_meta(run, warnings)
    df = assign_segments(df, meta, warnings)
    ev, events_available = read_events(run, float(df["wall"].iloc[0]), warnings)
    return RunData(run, df, ev, meta, warnings, events_available)


def read_events(run: Path, first_wall: float, warnings: list[str]) -> tuple[pd.DataFrame, bool]:
    columns = ["wall", "kind", "detail", "h"]
    empty = pd.DataFrame(columns=columns)
    path = run / "events.csv"
    if not path.exists():
        warn(warnings, "events.csv is missing; event markers and counts are unavailable.")
        return empty, False
    try:
        ev = pd.read_csv(path)
    except pd.errors.EmptyDataError:
        warn(warnings, "events.csv is empty; event markers and counts are unavailable.")
        return empty, False
    except (pd.errors.ParserError, UnicodeError) as exc:
        warn(warnings, f"events.csv is malformed ({exc}); event markers and counts are unavailable.")
        return empty, False
    except OSError as exc:
        warn(warnings, f"events.csv cannot be read ({exc}); event markers and counts are unavailable.")
        return empty, False
    if not {"wall", "kind"}.issubset(ev.columns):
        warn(warnings, "events.csv is missing wall/kind columns; event markers and counts are unavailable.")
        return empty, False
    if "detail" not in ev:
        warn(warnings, "events.csv has no detail column; event details are unavailable.")
        ev["detail"] = ""
    if ev.empty:
        return empty, True
    wall = pd.to_numeric(ev["wall"], errors="coerce")
    valid = np.isfinite(wall) & ev["kind"].notna()
    if not valid.all():
        warn(warnings, f"events.csv: {int((~valid).sum())} invalid row(s) ignored.")
    ev = ev.loc[valid, ["wall", "kind", "detail"]].copy()
    if ev.empty:
        return empty, False
    ev["wall"] = wall.loc[valid].to_numpy(dtype=float)
    ev["kind"] = ev["kind"].astype(str)
    ev["detail"] = ev["detail"].fillna("").astype(str)
    with np.errstate(over="ignore", invalid="ignore"):
        ev["h"] = (ev["wall"] - first_wall) / 3600.0
    ev = ev.loc[np.isfinite(ev["h"])].copy()
    return (ev, True) if len(ev) else (empty, False)


def interval_mask(df: pd.DataFrame, required: tuple[str, ...] = ()) -> np.ndarray:
    """Base validity for original-adjacent pairs, before metric-specific state checks."""
    n = len(df)
    if n < 2:
        return np.zeros(0, dtype=bool)
    rows = df["_row"].to_numpy(dtype=int)
    segment = df["segment"].to_numpy(dtype=int)
    wall = df["wall"].to_numpy(dtype=float)
    with np.errstate(invalid="ignore", over="ignore"):
        dt = np.diff(wall)
    ok = ((np.diff(rows) == 1) & (segment[1:] == segment[:-1])
          & np.isfinite(dt) & (dt > 0))
    cadence = df["sample_s"].to_numpy(dtype=float)[:-1]
    with np.errstate(invalid="ignore", over="ignore"):
        ok &= ~(np.isfinite(cadence) & (dt > GAP_FACTOR * cadence))
    for column in required:
        if column not in df:
            return np.zeros(n - 1, dtype=bool)
        values = df[column].to_numpy(dtype=float)
        ok &= np.isfinite(values[:-1]) & np.isfinite(values[1:])
    return ok


def valid_intervals(df: pd.DataFrame, required: tuple[str, ...] = ()) -> pd.DataFrame:
    """Actual measured hours between original-adjacent samples in one non-gap session interval."""
    columns = ["left", "right", "start_h", "end_h", "dh", "segment"]
    mask = interval_mask(df, required)
    left = np.flatnonzero(mask)
    if not len(left):
        return pd.DataFrame(columns=columns)
    right = left + 1
    h = df["h"].to_numpy(dtype=float)
    wall = df["wall"].to_numpy(dtype=float)
    with np.errstate(over="ignore", invalid="ignore"):
        dh = (wall[right] - wall[left]) / 3600.0
    finite = np.isfinite(dh) & (dh > 0)
    left, right, dh = left[finite], right[finite], dh[finite]
    return pd.DataFrame({
        "left": left, "right": right, "start_h": h[left], "end_h": h[right],
        "dh": dh, "segment": df["segment"].to_numpy(dtype=int)[left],
    })


def activity_intervals(df: pd.DataFrame) -> pd.DataFrame:
    intervals = valid_intervals(df)
    if intervals.empty:
        intervals["activity"] = pd.Series(dtype=str)
        return intervals
    activity = df["activity"].to_numpy(dtype=str)
    left = intervals["left"].to_numpy(dtype=int)
    right = intervals["right"].to_numpy(dtype=int)
    same = activity[left] == activity[right]
    intervals = intervals.loc[same].copy()
    intervals["activity"] = activity[left[same]]
    return intervals


def throttle_intervals(df: pd.DataFrame) -> pd.DataFrame:
    """Only stable, known 0/0 or 1/1 endpoints classify an interval; transitions stay unknown."""
    intervals = valid_intervals(df, ("throttled",))
    if intervals.empty:
        intervals["throttled"] = pd.Series(dtype=int)
        return intervals
    values = df["throttled"].to_numpy(dtype=float)
    left = intervals["left"].to_numpy(dtype=int)
    right = intervals["right"].to_numpy(dtype=int)
    known = np.isin(values[left], [0, 1]) & np.isin(values[right], [0, 1])
    stable = known & (values[left] == values[right])
    intervals = intervals.loc[stable].copy()
    intervals["throttled"] = values[left[stable]].astype(int)
    return intervals


def merge_spans(intervals: pd.DataFrame, value_column: str) -> list[tuple[float, float, Any]]:
    out: list[tuple[float, float, Any]] = []
    for row in intervals.itertuples(index=False):
        value = getattr(row, value_column)
        if out and row.left == previous_right and value == out[-1][2]:
            out[-1] = (out[-1][0], row.end_h, value)
        else:
            out.append((row.start_h, row.end_h, value))
        previous_right = row.right
    return out


def spans(df: pd.DataFrame) -> list[tuple[float, float, str]]:
    return merge_spans(activity_intervals(df), "activity")


def throttle_spans(df: pd.DataFrame) -> list[tuple[float, float, int]]:
    intervals = throttle_intervals(df)
    return merge_spans(intervals.loc[intervals["throttled"] == 1], "throttled")


def temp_columns(df: pd.DataFrame) -> list[str]:
    return [column for column in df.columns
            if column.startswith("temp_") and np.isfinite(df[column].to_numpy(dtype=float)).any()]


def zone_label(col: str) -> str:
    # temp_zone0_soc_thermal_c -> "soc thermal (zone0)": the type reads better, the index
    # keeps two zones of one type apart.
    zone, _, kind = col.removeprefix("temp_").removesuffix("_c").partition("_")
    return f"{kind.replace('_', ' ')} ({zone})" if kind else zone


def activity_color(activity: str) -> str:
    return ACTIVITY_COLORS.get(activity, OTHER_COLOR)


def activity_order(df: pd.DataFrame) -> list[str]:
    present = set(df["activity"].unique())
    return [activity for activity in ACTIVITY_COLORS if activity in present] + sorted(present - set(ACTIVITY_COLORS))


def series_colors(count: int) -> list[Any]:
    if count <= len(SERIES):
        return SERIES[:count]
    cmap = matplotlib.colormaps["turbo"].resampled(count)
    return [cmap(i) for i in range(count)]


def plot_series(ax, df: pd.DataFrame, y: Any, *, x: Any = None, **kwargs) -> None:
    """Plot every finite sample, inserting NaN breaks at sessions, gaps and invalid time jumps."""
    if df.empty:
        return
    y_values = np.asarray(y, dtype=float)
    x_values = df["h"].to_numpy(dtype=float) if x is None else np.asarray(x, dtype=float)
    mask = interval_mask(df)
    xs: list[float] = []
    ys: list[float] = []
    for i in range(len(df)):
        if i and not mask[i - 1]:
            xs.append(np.nan)
            ys.append(np.nan)
        xs.append(x_values[i])
        ys.append(y_values[i])
    kwargs.setdefault("marker", ".")
    kwargs.setdefault("markersize", 2.5)
    ax.plot(xs, ys, **kwargs)


def end_label(ax, x, y, text, color=INK_2) -> None:
    ax.annotate(text, (x, y), xytext=(4, 0), textcoords="offset points", va="center",
                fontsize=8, color=color)


def shade_activities(ax, segs) -> None:
    for start, end, activity in segs:
        ax.axvspan(start, end, color=activity_color(activity), alpha=0.10, lw=0, zorder=0)


def shade_throttle(ax, df: pd.DataFrame) -> None:
    for start, end, _ in throttle_spans(df):
        ax.axvspan(start, end, color=CRITICAL, alpha=0.18, lw=0, zorder=0)


def timeline(data: RunData) -> None:
    run, df, ev = data.run, data.df, data.ev
    temps = temp_columns(df)
    has_motor = np.isfinite(numeric_column(df, "motor_max_c")).any()
    has_current = np.isfinite(numeric_column(df, "motor_current_ma")).any()
    panels = ["battery", "volts", "temps", "clock"] + (["motor"] if has_motor else []) + (
        ["current"] if has_current else [])
    heights = {"battery": 1.4, "volts": 0.9, "temps": 1.4, "clock": 0.9, "motor": 1.0, "current": 0.9}
    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(12, 2.0 * sum(heights[p] for p in panels)),
                             gridspec_kw={"height_ratios": [heights[p] for p in panels]})
    ax_of = dict(zip(panels, axes))
    segs = spans(df)
    for ax in axes:
        shade_activities(ax, segs)

    ax = ax_of["battery"]
    battery = numeric_column(df, "battery_pct")
    plot_series(ax, df, battery, color=SERIES[0])
    ax.set_ylim(0, 105)
    ax.set_ylabel("%")
    ax.set_title("Battery charge (robotd's ~10 s EMA, 0% = 6.6 V shutdown floor)")
    finite = np.isfinite(battery)
    if finite.any():
        last = df.loc[finite].iloc[-1]
        end_label(ax, last["h"], last["battery_pct"], f"{last['battery_pct']:.0f}%")

    ax = ax_of["volts"]
    plot_series(ax, df, numeric_column(df, "battery_v"), color=SERIES[0])
    ax.axhline(6.6, color=CRITICAL, lw=1, ls="--")
    ax.annotate("6.6 V empty", (df["h"].iloc[0], 6.6), xytext=(2, 3), textcoords="offset points",
                va="bottom", fontsize=8, color=CRITICAL)
    ax.set_ylabel("V")
    ax.set_title("Pack voltage")

    ax = ax_of["temps"]
    shade_throttle(ax, df)
    colors = series_colors(len(temps))
    for i, col in enumerate(temps):
        plot_series(ax, df, df[col], color=colors[i], lw=1.5, label=zone_label(col))
        if len(temps) <= len(SERIES):
            finite = np.isfinite(df[col])
            if finite.any():
                last = df.loc[finite].iloc[-1]
                end_label(ax, last["h"], last[col], zone_label(col))
    ax.set_ylabel("°C")
    ax.set_title("Board thermal zones (red bands: known throttled intervals)")
    if temps:
        if len(temps) > len(SERIES):
            ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), ncol=1, fontsize=7)
        else:
            ax.legend(loc="upper left", ncol=min(4, len(temps)))

    ax = ax_of["clock"]
    shade_throttle(ax, df)
    ceiling = numeric_column(df, "cpu_khz_ceiling")
    if np.isfinite(ceiling).any():
        plot_series(ax, df, ceiling / 1000, color=SERIES[0], label="ceiling (scaling_max)")
    hardware = numeric_column(df, "cpu_khz_max")
    if np.isfinite(hardware).any():
        plot_series(ax, df, hardware / 1000, color=INK_2, lw=1, ls="--", label="hardware max")
    cur_cols = [column for column in df.columns if column.endswith("_cur_khz")
                and np.isfinite(df[column].to_numpy(dtype=float)).any()]
    if cur_cols:
        plot_series(ax, df, df[cur_cols].max(axis=1) / 1000, color=SERIES[1], lw=1, alpha=0.8,
                    label="current clock (fastest cluster)")
    ax.set_ylabel("MHz")
    ax.set_title("CPU clock")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="lower left", ncol=3)

    if "motor" in ax_of:
        ax = ax_of["motor"]
        plot_series(ax, df, numeric_column(df, "motor_max_c"), color=SERIES[1], label="hottest servo")
        mean = numeric_column(df, "motor_mean_c")
        if np.isfinite(mean).any():
            plot_series(ax, df, mean, color=SERIES[0], label="mean")
        ax.set_ylabel("°C")
        ax.set_title("Servo temperature")
        ax.legend(loc="upper left", ncol=2)

    if "current" in ax_of:
        ax = ax_of["current"]
        plot_series(ax, df, numeric_column(df, "motor_current_ma") / 1000, color=SERIES[0], lw=1)
        ax.set_ylabel("A")
        ax.set_title("Servo current, sum of |joint current|, mean over each sample")

    if len(ev):
        marked = ev["kind"].isin(["fence", "paused", "sit_refused", "rise_refused", "connection_lost"])
        for _, event in ev.loc[marked].iterrows():
            axes[0].axvline(event["h"], color=INK_2, lw=0.6, ls=":")
        paused = (ev["kind"] == "activity") & ev["detail"].str.startswith("paused")
        for _, event in ev.loc[paused].iterrows():
            axes[0].axvline(event["h"], color=CRITICAL, lw=1)

    axes[-1].set_xlabel("hours since first sample (wall clock; gaps retained)")
    present = [activity for activity in ACTIVITY_COLORS if (df["activity"] == activity).any()]
    handles = [Patch(color=activity_color(activity), alpha=0.35, label=activity) for activity in present]
    handles.append(Patch(color=OTHER_COLOR, alpha=0.35, label="other / paused"))
    handles.append(Patch(color=CRITICAL, alpha=0.3, label="known throttled"))
    fig.legend(handles=handles, loc="upper right", ncol=len(handles), bbox_to_anchor=(0.99, 0.995))
    fig.suptitle(run.name, x=0.01, ha="left", fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 0.86 if len(temps) > len(SERIES) else 1, 0.97))
    fig.savefig(run / "timeline.png", dpi=130)
    plt.close(fig)


def drain_by_activity(df: pd.DataFrame) -> pd.DataFrame:
    """%/h from original-adjacent, same-session, same-activity pairs with finite battery endpoints.

    Missing battery readings are never compressed away. The EMA lags ~10 s behind load, so
    short runs bias per-activity rates toward the mean; rates are not a chemistry-independent
    measure of charge. Zero/recovering battery has no finite positive runtime estimate.
    """
    activities = activity_order(df)
    intervals = valid_intervals(df, ("battery_pct",))
    if not intervals.empty:
        activity = df["activity"].to_numpy(dtype=str)
        left = intervals["left"].to_numpy(dtype=int)
        right = intervals["right"].to_numpy(dtype=int)
        same = activity[left] == activity[right]
        intervals = intervals.loc[same].copy()
        intervals["activity"] = activity[left[same]]
        battery = df["battery_pct"].to_numpy(dtype=float)
        with np.errstate(invalid="ignore", over="ignore"):
            intervals["dp"] = battery[left[same]] - battery[right[same]]
        intervals = intervals.loc[np.isfinite(intervals["dp"])]
        grouped = intervals.groupby("activity").agg(hours=("dh", "sum"), pct=("dp", "sum"))
    else:
        grouped = pd.DataFrame(columns=["hours", "pct"])

    rows = []
    for activity in activities:
        hours = float(grouped.loc[activity, "hours"]) if activity in grouped.index else 0.0
        pct = float(grouped.loc[activity, "pct"]) if activity in grouped.index else np.nan
        if not math.isfinite(hours):
            hours = np.nan
        if not math.isfinite(pct):
            pct = np.nan
        rate = pct / hours if math.isfinite(hours) and hours > 0 and math.isfinite(pct) else np.nan
        if not math.isfinite(rate):
            rate = np.nan
        runtime = 100.0 / rate if math.isfinite(rate) and rate > 0 else np.nan
        if not math.isfinite(runtime):
            runtime = np.nan
        rows.append({"activity": activity, "hours": hours, "pct": pct,
                     "pct_per_h": rate, "runtime_h_if_only_this": runtime})
    result = pd.DataFrame(rows, columns=["activity", "hours", "pct", "pct_per_h", "runtime_h_if_only_this"])
    return result.set_index("activity")


def odometry_tracks(df: pd.DataFrame, meta: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Rebase each logger session independently; never expose one appended track as continuous."""
    tracks = []
    for sid, group in df.groupby("segment", sort=True):
        x = numeric_column(group, "odom_x").to_numpy(dtype=float)
        y = numeric_column(group, "odom_y").to_numpy(dtype=float)
        valid = np.isfinite(x) & np.isfinite(y)
        od = group.loc[valid].copy()
        if od.empty:
            continue
        with np.errstate(invalid="ignore", over="ignore"):
            od["x"] = x[valid] - x[valid][0]
            od["y"] = y[valid] - y[valid][0]
        finite = np.isfinite(od["x"]) & np.isfinite(od["y"])
        od = od.loc[finite].copy()
        if od.empty:
            continue
        meta_index = int(group["meta_segment"].iloc[0])
        fence = meta[meta_index]["fence"] if 0 <= meta_index < len(meta) else None
        distance = numeric_column(od, "odom_dist")
        distance = distance.where(np.isfinite(distance) & (distance >= 0))
        if distance.notna().any():
            max_distance = float(distance.max())
            basis = "logger home (odom_dist)"
        else:
            max_distance = float(np.hypot(od["x"], od["y"]).max())
            basis = "first plotted position"
        tracks.append({"segment": int(sid), "data": od, "fence": fence,
                       "max_distance": max_distance, "distance_basis": basis})
    return tracks


def fmt(value: Any, spec: str = ".2f", default: str = "N/A") -> str:
    number = finite_number(value)
    return format(number, spec) if number is not None else default


def breakdown(data: RunData) -> pd.DataFrame:
    run, df = data.run, data.df
    fig, ((a1, a2), (a3, a4)) = plt.subplots(2, 2, figsize=(12, 9))

    tracks = odometry_tracks(df, data.meta_segments)
    if tracks:
        colors = series_colors(len(tracks))
        limit = REFERENCE_FENCE_M
        for i, track in enumerate(tracks):
            od = track["data"]
            color = colors[i]
            plot_series(a1, od, od["y"], x=od["x"], color=color, lw=0.8, alpha=0.65,
                        label=f"session {track['segment']}")
            a1.scatter(od["x"], od["y"], color=color, s=10, lw=0, zorder=3)
            limit = max(limit, float(np.hypot(od["x"], od["y"]).max()))
            if track["fence"] is not None:
                a1.add_patch(Circle((0, 0), track["fence"], fill=False, color=color, lw=1.2, ls=":"))
                a1.plot([], [], color=color, ls=":",
                        label=f"session {track['segment']} configured radius {track['fence']:g} m at rebased origin")
                limit = max(limit, track["fence"])
        a1.add_patch(Circle((0, 0), REFERENCE_FENCE_M, fill=False, color=INK_2, lw=1, ls="--", alpha=0.6))
        a1.plot([], [], color=INK_2, ls="--", label=f"{REFERENCE_FENCE_M:g} m reference only")
        a1.plot([0], [0], marker="+", color=INK, ms=10, ls="")
        a1.set_xlim(-limit * 1.15, limit * 1.15)
        a1.set_ylim(-limit * 1.15, limit * 1.15)
        a1.set_aspect("equal")
        a1.set_xlabel("x, m (rebased per session)")
        a1.set_ylabel("y, m")
        max_distance = max(track["max_distance"] for track in tracks)
        a1.set_title(f"Session-relative odometry (max {max_distance:.2f} m)")
        a1.legend(loc="upper left", fontsize=7, frameon=True, framealpha=0.9,
                  ncol=2 if len(tracks) > 4 else 1)
    else:
        a1.text(0.5, 0.5, "No valid odometry positions", ha="center", va="center", transform=a1.transAxes)
        a1.set_title("Odometry by session")

    durations = activity_intervals(df).groupby("activity")["dh"].sum() * 60
    order = [activity for activity in activity_order(df) if activity in durations.index]
    if order:
        values = durations.reindex(order).to_numpy()
        a2.barh(order, values, color=[activity_color(activity) for activity in order], height=0.6)
        for i, value in enumerate(values):
            a2.annotate(f"{value:.1f} min", (value, i), xytext=(4, 0), textcoords="offset points",
                        va="center", color=INK_2)
        a2.set_xlim(0, max(float(values.max()) * 1.25, 0.1))
    else:
        a2.text(0.5, 0.5, "No valid same-activity intervals", ha="center", va="center", transform=a2.transAxes)
    a2.set_xlabel("measured minutes")
    a2.set_title("Measured time in each activity")
    a2.grid(axis="y", visible=False)

    drain = drain_by_activity(df)
    rates = drain["pct_per_h"]
    finite_rates = rates[np.isfinite(rates)]
    if len(finite_rates):
        positions = np.arange(len(drain))
        values = rates.to_numpy(dtype=float)
        finite = np.isfinite(values)
        colors = [activity_color(activity) for activity in drain.index]
        a3.bar(positions[finite], values[finite], color=[color for color, ok in zip(colors, finite) if ok], width=0.6)
        a3.set_xticks(positions, drain.index)
        for i, (_, row) in enumerate(drain.iterrows()):
            rate = row["pct_per_h"]
            if not math.isfinite(rate):
                a3.annotate("N/A", (i, 0), xytext=(0, 4), textcoords="offset points",
                            ha="center", va="bottom", color=INK_2)
                continue
            runtime = row["runtime_h_if_only_this"]
            runtime_text = f"≈{runtime:.1f} h alone" if math.isfinite(runtime) else "runtime N/A"
            offset = 4 if rate >= 0 else -4
            va = "bottom" if rate >= 0 else "top"
            a3.annotate(f"{rate:.1f} %/h\n{runtime_text}", (i, rate), xytext=(0, offset),
                        textcoords="offset points", ha="center", va=va, color=INK_2)
        low = min(0.0, float(finite_rates.min()))
        high = max(0.0, float(finite_rates.max()))
        padding = max((high - low) * 0.25, 1.0)
        a3.set_ylim(low - (padding if low < 0 else 0), high + padding)
        a3.axhline(0, color=INK_2, lw=0.8)
    else:
        a3.text(0.5, 0.5, "No valid battery drain intervals", ha="center", va="center", transform=a3.transAxes)
        a3.set_ylim(-1, 1)
    a3.set_ylabel("% per hour")
    a3.set_title("Battery drain by activity")
    a3.grid(axis="x", visible=False)

    col = "cpu_temp_c"
    acts = []
    samples = []
    if col in df:
        for activity in activity_order(df):
            values = df.loc[df["activity"] == activity, col].dropna().to_numpy()
            if len(values):
                acts.append(activity)
                samples.append(values)
    if samples:
        bp = a4.boxplot(samples, tick_labels=acts, patch_artist=True, widths=0.5,
                        medianprops={"color": INK}, flierprops={"markersize": 3})
        for patch, activity in zip(bp["boxes"], acts):
            patch.set_facecolor(activity_color(activity))
            patch.set_alpha(0.6)
            patch.set_edgecolor(activity_color(activity))
        a4.set_ylabel("°C")
    else:
        a4.text(0.5, 0.5, "No board temperature samples", ha="center", va="center", transform=a4.transAxes)
    a4.set_title("Hottest board zone by activity")
    a4.grid(axis="x", visible=False)

    fig.suptitle(run.name, x=0.01, ha="left", fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(run / "breakdown.png", dpi=130)
    plt.close(fig)
    return drain


def summary(data: RunData, drain: pd.DataFrame) -> str:
    run, df, ev = data.run, data.df, data.ev
    wall = df["wall"].to_numpy(dtype=float)
    wall_span = float((wall.max() - wall.min()) / 3600.0)
    covered_h = float(valid_intervals(df)["dh"].sum())
    sessions = int(df["segment"].nunique())
    lines = [
        f"# {run.name}",
        "",
        f"- Wall-clock span: **{wall_span:.2f} h** (includes restart downtime and sampling gaps; not active time)",
        f"- Sample-covered time: **{covered_h:.2f} h** ({len(df)} samples; valid adjacent intervals only)",
        f"- Identified logger sessions: **{sessions}**",
    ]
    cadences = []
    for sid, group in df.groupby("segment", sort=True):
        value = group["sample_s"].iloc[0]
        cadences.append(f"session {sid}: {value:.1f} s" if math.isfinite(value) else f"session {sid}: insufficient samples")
    lines.append("- Sampling interval used for gap detection: " + "; ".join(cadences))

    battery = numeric_column(df, "battery_pct")
    finite_battery = np.isfinite(battery)
    if finite_battery.any():
        first = df.loc[finite_battery].iloc[0]
        last = df.loc[finite_battery].iloc[-1]
        span = abs(float(last["wall"] - first["wall"])) / 3600.0
        lines.append(
            f"- Battery endpoints: {first['battery_pct']:.0f}% ({fmt(first.get('battery_v'), '.2f')} V) → "
            f"{last['battery_pct']:.0f}% ({fmt(last.get('battery_v'), '.2f')} V) "
            f"over {span:.2f} h wall-clock span (not a drain measurement)"
        )
    if "robotd" in df:
        down = df.loc[df["robotd"] == "down"]
        if len(down):
            lines.append(f"- robotd stopped answering at {down['h'].iloc[0]:.2f} h "
                         "(the empty-battery shutdown, if the battery read ~0% just before)")
    if "cpu_temp_c" in df and np.isfinite(df["cpu_temp_c"]).any():
        index = df["cpu_temp_c"].idxmax()
        lines.append(f"- Hottest board zone: **{df.loc[index, 'cpu_temp_c']:.1f} °C** at {df.loc[index, 'h']:.2f} h "
                     f"({df.loc[index, 'activity']}); median {df['cpu_temp_c'].median():.1f} °C")
    for col in temp_columns(df):
        lines.append(f"  - {zone_label(col)}: max {df[col].max():.1f} °C, median {df[col].median():.1f} °C")

    throttle = throttle_intervals(df)
    known_h = float(throttle["dh"].sum())
    if known_h > 0:
        throttled_h = float(throttle.loc[throttle["throttled"] == 1, "dh"].sum())
        unknown_h = max(covered_h - known_h, 0.0)
        first = throttle.loc[throttle["throttled"] == 1, "start_h"]
        line = (f"- Throttled: **{throttled_h * 60:.1f} min** measured over {known_h:.2f} h known stable "
                f"throttle coverage ({100 * throttled_h / known_h:.1f}% of that coverage); "
                f"{unknown_h:.2f} h of sample-covered time is unclassified")
        if len(first):
            line += f"; first throttled interval at {first.iloc[0]:.2f} h"
        lines.append(line)
    else:
        lines.append("- Throttle telemetry: unavailable (no valid stable throttle intervals).")

    ceiling = numeric_column(df, "cpu_khz_ceiling")
    if np.isfinite(ceiling).any():
        hardware = numeric_column(df, "cpu_khz_max")
        hardware_text = f" of {hardware.max() / 1000:.0f} MHz" if np.isfinite(hardware).any() else ""
        lines.append(f"- Lowest clock ceiling: {ceiling.min() / 1000:.0f} MHz{hardware_text}")
    motor = numeric_column(df, "motor_max_c")
    if np.isfinite(motor).any():
        index = motor.idxmax()
        hottest = df.loc[index, "motor_hottest"] if "motor_hottest" in df else None
        if not isinstance(hottest, str) or not hottest.strip() or hottest.lower() == "nan":
            hottest = "unknown servo"
        lines.append(f"- Hottest servo: {motor.loc[index]:.0f} °C ({hottest}) at {df.loc[index, 'h']:.2f} h")

    tracks = odometry_tracks(df, data.meta_segments)
    if tracks:
        for track in tracks:
            fence = f"{track['fence']:g} m" if track["fence"] is not None else "unavailable"
            lines.append(f"- Odometry session {track['segment']}: max {track['max_distance']:.2f} m "
                         f"session-relative distance from {track['distance_basis']}; configured fence radius: {fence}")
        lines.append("- Odometry distances are session-relative, not a continuous full-run distance.")
        fence_trips = str(int((ev["kind"] == "fence").sum())) if data.events_available else "unavailable"
        lines.append(f"- Fence trips: {fence_trips}")
    if data.events_available and len(ev):
        n_paused = int(((ev["kind"] == "activity") & ev["detail"].str.startswith("paused")).sum())
        if n_paused:
            lines.append(f"- Paused {n_paused}× (fallen or picked up) — see events.csv")

    if len(drain):
        lines += ["", "| activity | battery-covered h | drain %/h | runtime if only this |", "|---|---:|---:|---:|"]
        for activity, row in drain.iterrows():
            rate = fmt(row["pct_per_h"], ".1f")
            runtime = fmt(row["runtime_h_if_only_this"], ".1f")
            runtime_text = f"{runtime} h" if runtime != "N/A" else "N/A"
            lines.append(f"| {activity} | {fmt(row['hours'])} | {rate} | {runtime_text} |")
        lines += ["", "Drain uses only adjacent finite battery readings in the same session and activity. "
                      "It is read off robotd's percentage, which is linear in voltage between 6.6 V and full — "
                      "not in charge — so per-activity rates depend on where in the curve each activity happened. "
                      "Compare them within a run, not across chemistries."]
    if data.warnings:
        lines += ["", "## Data quality", ""]
        lines += [f"- {message}" for message in data.warnings]
    text = "\n".join(lines) + "\n"
    (run / "summary.md").write_text(text, encoding="utf-8")
    return text


def compare(runs: list[RunData], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 4.5))
    colors = series_colors(len(runs))
    for i, data in enumerate(runs):
        battery = numeric_column(data.df, "battery_pct")
        plot_series(ax, data.df, battery, color=colors[i], label=data.run.name)
        finite = np.isfinite(battery)
        if finite.any():
            last = data.df.loc[finite].iloc[-1]
            end_label(ax, last["h"], last["battery_pct"], f"{last['h']:.1f} h")
    ax.set_xlabel("hours since first sample (wall clock; gaps retained)")
    ax.set_ylabel("%")
    ax.set_ylim(0, 105)
    ax.set_title("Battery charge, run by run")
    if runs:
        columns = max(1, math.ceil(len(runs) / 12))
        ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), ncol=columns, fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", nargs="+", type=Path, help="run directories written by robotctl endurance")
    args = parser.parse_args(argv)

    # Validate every input before writing the first run's outputs.
    loaded = []
    errors = []
    for run in args.runs:
        try:
            loaded.append(load(run))
        except InputError as exc:
            errors.append(str(exc))
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    try:
        for data in loaded:
            for message in data.warnings:
                print(f"{data.run}: warning: {message}", file=sys.stderr)
            timeline(data)
            drain = breakdown(data)
            print(summary(data, drain))
            print(f"→ {data.run / 'timeline.png'}\n→ {data.run / 'breakdown.png'}\n→ {data.run / 'summary.md'}\n")
        if len(loaded) > 1:
            out = args.runs[0].parent / "compare.png"
            compare(loaded, out)
            print(f"→ {out}")
    except OSError as exc:
        print(f"cannot write plot outputs: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
