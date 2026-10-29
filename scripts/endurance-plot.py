#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib>=3.8", "numpy>=1.26", "pandas>=2.1"]
# ///
"""Charts and a summary for a run of `robotctl endurance` or `robotctl stress`.

    uv run scripts/endurance-plot.py <run dir> [<run dir> ...]

Writes into each run directory:

    timeline.png   battery, voltage, board temperatures, clock ceiling, motors — one panel each,
                   on a shared time axis, with what the robot was doing shaded behind them
    breakdown.png  the odometry track against the 0.5 m circle, time per activity, drain rate
                   and board temperature per activity
    summary.md     the numbers: runtime, peak temperatures, time throttled, drain per activity
    currents.png   each servo's mean current over the run and its highest peak (when the run
                   logged per-joint current)

For a stress run it also writes:

    getup.png      every get-up's time against the lap it happened in
    report.pdf     the same, to read or pass on: an overview (headline numbers, how each fall
                   ended, get-up times), the per-lap table, the timeline, servo currents and every
                   incident
    summary.md     … plus a table per lap: falls, get-ups, failures, guards, battery, hottest servo

Given several runs it also writes `compare.png` beside the first, battery against time for each —
or, for stress runs, median get-up time and battery against lap number, one line per robot. Stress
runs of different laps (`meta.json` `lap`) are not the same test and are refused together.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Circle, Patch  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

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


def load(run: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    df = pd.read_csv(run / "samples.csv")
    # Wall clock, not `t_s`: a run resumed after a crash restarts `t_s` at zero.
    df["h"] = (df["wall"] - df["wall"].iloc[0]) / 3600.0
    ev_path = run / "events.csv"
    ev = pd.read_csv(ev_path) if ev_path.exists() else pd.DataFrame(columns=["wall", "kind", "detail"])
    if len(ev):
        ev["h"] = (ev["wall"] - df["wall"].iloc[0]) / 3600.0
    return df, ev


def temp_columns(df: pd.DataFrame) -> list[str]:
    return [c for c in df.columns if c.startswith("temp_") and df[c].notna().any()]


def zone_label(col: str) -> str:
    # temp_zone0_soc_thermal_c -> "soc thermal (zone0)": the type reads better, the index
    # keeps two zones of one type apart.
    zone, _, kind = col.removeprefix("temp_").removesuffix("_c").partition("_")
    return f"{kind.replace('_', ' ')} ({zone})" if kind else zone


def activity_color(a: str) -> str:
    return ACTIVITY_COLORS.get(a, OTHER_COLOR)


def spans(df: pd.DataFrame) -> list[tuple[float, float, str]]:
    """Contiguous runs of one activity, as (start h, end h, activity)."""
    out = []
    act = df["activity"].fillna("").to_numpy()
    h = df["h"].to_numpy()
    start = 0
    for i in range(1, len(df) + 1):
        if i == len(df) or act[i] != act[start]:
            end = h[i] if i < len(df) else h[-1]
            out.append((h[start], end, act[start]))
            start = i
    return out


def shade_activities(ax, segs) -> None:
    for a, b, act in segs:
        ax.axvspan(a, b, color=activity_color(act), alpha=0.10, lw=0, zorder=0)


def shade_throttle(ax, df: pd.DataFrame) -> None:
    if "throttled" not in df:
        return
    t = df["throttled"].fillna(0).astype(int).to_numpy()
    h = df["h"].to_numpy()
    i = 0
    while i < len(t):
        if t[i]:
            j = i
            while j + 1 < len(t) and t[j + 1]:
                j += 1
            ax.axvspan(h[i], h[min(j + 1, len(h) - 1)], color=CRITICAL, alpha=0.18, lw=0, zorder=0)
            i = j + 1
        else:
            i += 1


def end_label(ax, x, y, text, color=INK_2) -> None:
    ax.annotate(text, (x, y), xytext=(4, 0), textcoords="offset points", va="center",
                fontsize=8, color=color)


def timeline_figure(run: Path, df: pd.DataFrame, ev: pd.DataFrame):
    temps = temp_columns(df)
    has_motor = df.get("motor_max_c", pd.Series(dtype=float)).notna().any()
    has_current = df.get("motor_current_ma", pd.Series(dtype=float)).notna().any()
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
    ax.plot(df["h"], df["battery_pct"], color=SERIES[0])
    ax.set_ylim(0, 105)
    ax.set_ylabel("%")
    ax.set_title("Battery charge (robotd's ~10 s EMA, 0% = 6.6 V shutdown floor)")
    last = df.dropna(subset=["battery_pct"]).iloc[-1:] if df["battery_pct"].notna().any() else None
    if last is not None and len(last):
        end_label(ax, last["h"].iloc[0], last["battery_pct"].iloc[0], f"{last['battery_pct'].iloc[0]:.0f}%")

    ax = ax_of["volts"]
    ax.plot(df["h"], df["battery_v"], color=SERIES[0])
    ax.axhline(6.6, color=CRITICAL, lw=1, ls="--")
    ax.annotate("6.6 V empty", (df["h"].iloc[0], 6.6), xytext=(2, 3), textcoords="offset points",
                va="bottom", fontsize=8, color=CRITICAL)
    ax.set_ylabel("V")
    ax.set_title("Pack voltage")

    ax = ax_of["temps"]
    shade_throttle(ax, df)
    for i, col in enumerate(temps[: len(SERIES)]):
        ax.plot(df["h"], df[col], color=SERIES[i], lw=1.5, label=zone_label(col))
        s = df[col].dropna()
        if len(s):
            end_label(ax, df.loc[s.index[-1], "h"], s.iloc[-1], zone_label(col))
    ax.set_ylabel("°C")
    ax.set_title("Board thermal zones (red bands: clock throttled)")
    if len(temps) > 1:
        ax.legend(loc="upper left", ncol=min(4, len(temps)))

    ax = ax_of["clock"]
    shade_throttle(ax, df)
    if df.get("cpu_khz_ceiling", pd.Series(dtype=float)).notna().any():
        ax.plot(df["h"], df["cpu_khz_ceiling"] / 1000, color=SERIES[0], label="ceiling (scaling_max)")
        ax.plot(df["h"], df["cpu_khz_max"] / 1000, color=INK_2, lw=1, ls="--", label="hardware max")
    cur_cols = [c for c in df.columns if c.endswith("_cur_khz")]
    if cur_cols:
        ax.plot(df["h"], df[cur_cols].max(axis=1) / 1000, color=SERIES[1], lw=1, alpha=0.8,
                label="current clock (fastest cluster)")
    ax.set_ylabel("MHz")
    ax.set_title("CPU clock")
    ax.legend(loc="lower left", ncol=3)

    if "motor" in ax_of:
        ax = ax_of["motor"]
        ax.plot(df["h"], df["motor_max_c"], color=SERIES[1], label="hottest servo")
        ax.plot(df["h"], df["motor_mean_c"], color=SERIES[0], label="mean")
        ax.set_ylabel("°C")
        ax.set_title("Servo temperature")
        ax.legend(loc="upper left", ncol=2)

    if "current" in ax_of:
        ax = ax_of["current"]
        if df.get("motor_current_peak_ma", pd.Series(dtype=float)).notna().any():
            ax.plot(df["h"], df["motor_current_peak_ma"] / 1000, color=SERIES[1], lw=0.8,
                    alpha=0.7, label="peak")
        ax.plot(df["h"], df["motor_current_ma"] / 1000, color=SERIES[0], lw=1, label="mean")
        ax.set_ylabel("A")
        ax.set_title("Servo current, sum of |joint current| over each sample")
        if "motor_current_peak_ma" in df:
            ax.legend(loc="upper left", ncol=2)

    for _, e in ev[ev["kind"].isin(["fence", "paused", "sit_refused", "rise_refused", "connection_lost"])].iterrows():
        axes[0].axvline(e["h"], color=INK_2, lw=0.6, ls=":")
    for _, e in ev[ev["kind"] == "activity"].iterrows():
        if str(e["detail"]).startswith("paused"):
            axes[0].axvline(e["h"], color=CRITICAL, lw=1)

    axes[-1].set_xlabel("hours since start")
    present = [a for a in ACTIVITY_COLORS if (df["activity"] == a).any()]
    handles = [Patch(color=activity_color(a), alpha=0.35, label=a) for a in present]
    handles.append(Patch(color=OTHER_COLOR, alpha=0.35, label="other / paused"))
    handles.append(Patch(color=CRITICAL, alpha=0.3, label="throttled"))
    fig.legend(handles=handles, loc="upper right", ncol=len(handles), bbox_to_anchor=(0.99, 0.995))
    fig.suptitle(run.name, x=0.01, ha="left", fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def timeline(run: Path, df: pd.DataFrame, ev: pd.DataFrame) -> None:
    fig = timeline_figure(run, df, ev)
    fig.savefig(run / "timeline.png", dpi=130)
    plt.close(fig)


def motor_currents(df: pd.DataFrame) -> pd.DataFrame:
    """Per joint: mean current over the run, the highest peak, and when and doing what it hit.

    Empty for a run logged before per-joint current (only `motor_current_ma`, the sum)."""
    joints = [c.removesuffix("_current_peak_ma") for c in df.columns
              if c.endswith("_current_peak_ma") and c != "motor_current_peak_ma"
              and df[c].notna().any()]
    rows = []
    for j in joints:
        peak = df[f"{j}_current_peak_ma"]
        i = peak.idxmax()
        rows.append({"joint": j, "mean_a": df[f"{j}_current_ma"].mean() / 1000,
                     "peak_a": peak.max() / 1000, "peak_h": df.loc[i, "h"],
                     "peak_activity": str(df.loc[i, "step"] if "step" in df and
                                          not pd.isna(df.loc[i, "step"]) else df.loc[i, "activity"])})
    return pd.DataFrame(rows, columns=["joint", "mean_a", "peak_a", "peak_h", "peak_activity"]
                        ).set_index("joint")


def currents_figure(run: Path, cur: pd.DataFrame, figsize=(10, 6)):
    fig, ax = plt.subplots(figsize=figsize)
    y = np.arange(len(cur))[::-1]
    ax.barh(y, cur["peak_a"], color=SERIES[1], alpha=0.35, height=0.7, label="highest peak")
    ax.barh(y, cur["mean_a"], color=SERIES[0], height=0.7, label="mean over the run")
    for yi, (_, r) in zip(y, cur.iterrows()):
        ax.annotate(f"{r['mean_a']:.2f} / {r['peak_a']:.2f} A", (r["peak_a"], yi), xytext=(4, 0),
                    textcoords="offset points", va="center", fontsize=8, color=INK_2)
    ax.set_yticks(y, [j.replace("_", " ") for j in cur.index])
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("A")
    ax.set_xlim(0, cur["peak_a"].max() * 1.25)
    ax.set_title(f"Servo current per joint — {run.name}")
    ax.legend(loc="lower right")
    fig.tight_layout()
    return fig


def drain_by_activity(df: pd.DataFrame) -> pd.DataFrame:
    """%/h drained per activity, from consecutive samples that share one.

    The EMA lags ~10 s behind the load, so each activity's first samples carry some of the one
    before. Over hours that averages out; over one run with few transitions it biases toward
    the mean, which makes the per-activity rates a lower bound on their spread.
    """
    d = df[["h", "battery_pct", "activity"]].dropna().copy()
    d["dh"] = d["h"].diff().shift(-1)
    d["dp"] = -d["battery_pct"].diff().shift(-1)
    d["same"] = d["activity"] == d["activity"].shift(-1)
    d = d[d["same"] & (d["dh"] > 0)]
    g = d.groupby("activity").agg(hours=("dh", "sum"), pct=("dp", "sum"))
    g["pct_per_h"] = g["pct"] / g["hours"]
    g["runtime_h_if_only_this"] = 100.0 / g["pct_per_h"]
    return g


def breakdown(run: Path, df: pd.DataFrame, ev: pd.DataFrame, meta: dict) -> pd.DataFrame:
    fig, ((a1, a2), (a3, a4)) = plt.subplots(2, 2, figsize=(12, 9))

    # Odometry track, relative to the start.
    od = df.dropna(subset=["odom_x", "odom_y"])
    fence = 0.5
    if len(od):
        x0, y0 = od["odom_x"].iloc[0], od["odom_y"].iloc[0]
        x, y = od["odom_x"] - x0, od["odom_y"] - y0
        a1.scatter(x, y, c=od["h"], cmap="Blues", s=10, lw=0)
        a1.plot(x, y, color=SERIES[0], lw=0.6, alpha=0.4)
        a1.plot([0], [0], marker="+", color=INK, ms=10)
        a1.add_patch(Circle((0, 0), fence, fill=False, color=CRITICAL, lw=1, ls="--"))
        args = (meta.get("segments") or [{}])[-1].get("args", {})
        if "fence" in args:
            a1.add_patch(Circle((0, 0), args["fence"], fill=False, color=INK_2, lw=1, ls=":"))
        lim = max(fence, float(np.hypot(x, y).max())) * 1.15
        a1.set_xlim(-lim, lim)
        a1.set_ylim(-lim, lim)
        a1.set_aspect("equal")
        a1.set_xlabel("x, m (odometry)")
        a1.set_ylabel("y, m")
        maxd = od["odom_dist"].max()
        a1.set_title(f"Odometry track — max {maxd:.2f} m from start (red: 0.5 m)")

    # Time per activity.
    # Same order as every other panel, top to bottom; the bookkeeping states are left out.
    per = df.groupby("activity")["h"].count() * df["h"].diff().median()
    per = per.reindex([a for a in reversed(ACTIVITY_COLORS) if a in per.index])
    a2.barh(per.index, per.values * 60, color=[activity_color(a) for a in per.index], height=0.6)
    for i, v in enumerate(per.values * 60):
        a2.annotate(f"{v:.0f} min", (v, i), xytext=(4, 0), textcoords="offset points", va="center", color=INK_2)
    a2.set_xlabel("minutes")
    a2.set_title("Time in each activity")
    a2.grid(axis="y", visible=False)

    # Drain rate per activity.
    g = drain_by_activity(df)
    g = g.reindex([a for a in ACTIVITY_COLORS if a in g.index])
    if len(g):
        a3.bar(g.index, g["pct_per_h"], color=[activity_color(a) for a in g.index], width=0.6)
        for i, (_, row) in enumerate(g.iterrows()):
            a3.annotate(f"{row['pct_per_h']:.0f} %/h\n≈{row['runtime_h_if_only_this']:.1f} h alone",
                        (i, row["pct_per_h"]), xytext=(0, 4), textcoords="offset points",
                        ha="center", va="bottom", color=INK_2)
        a3.set_ylabel("% per hour")
        a3.set_ylim(0, g["pct_per_h"].max() * 1.3)
    a3.set_title("Battery drain by activity")
    a3.grid(axis="x", visible=False)

    # Board temperature by activity.
    col = "cpu_temp_c"
    acts = [a for a in ACTIVITY_COLORS if (df["activity"] == a).any()]
    data = [df.loc[df["activity"] == a, col].dropna().to_numpy() for a in acts]
    if any(len(d) for d in data):
        bp = a4.boxplot(data, tick_labels=acts, patch_artist=True, widths=0.5,
                        medianprops={"color": INK}, flierprops={"markersize": 3})
        for patch, a in zip(bp["boxes"], acts):
            patch.set_facecolor(activity_color(a))
            patch.set_alpha(0.6)
            patch.set_edgecolor(activity_color(a))
        a4.set_ylabel("°C")
    a4.set_title("Hottest board zone by activity")
    a4.grid(axis="x", visible=False)

    fig.suptitle(run.name, x=0.01, ha="left", fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(run / "breakdown.png", dpi=130)
    plt.close(fig)
    return g


def summary(run: Path, df: pd.DataFrame, ev: pd.DataFrame, drain: pd.DataFrame) -> str:
    batt = df.dropna(subset=["battery_pct"])
    total_h = df["h"].iloc[-1]
    dt = df["h"].diff().median()
    throttled_h = df["throttled"].fillna(0).astype(int).sum() * dt if "throttled" in df else 0
    first_throttle = df.loc[df.get("throttled", pd.Series(dtype=float)).fillna(0).astype(int) > 0, "h"]
    lines = [
        f"# {run.name}",
        "",
        f"- Logged: **{total_h:.2f} h** ({len(df)} samples, every {dt * 3600:.0f} s)",
    ]
    if len(batt):
        lines.append(
            f"- Battery: {batt['battery_pct'].iloc[0]:.0f}% ({batt['battery_v'].iloc[0]:.2f} V) → "
            f"{batt['battery_pct'].iloc[-1]:.0f}% ({batt['battery_v'].iloc[-1]:.2f} V) "
            f"over {batt['h'].iloc[-1] - batt['h'].iloc[0]:.2f} h"
        )
    down = df[df["robotd"] == "down"]
    if len(down):
        lines.append(f"- robotd stopped answering at {down['h'].iloc[0]:.2f} h "
                     f"(the empty-battery shutdown, if the battery read ~0% just before)")
    if "cpu_temp_c" in df and df["cpu_temp_c"].notna().any():
        i = df["cpu_temp_c"].idxmax()
        lines.append(f"- Hottest board zone: **{df.loc[i, 'cpu_temp_c']:.1f} °C** at {df.loc[i, 'h']:.2f} h "
                     f"({df.loc[i, 'activity']}); median {df['cpu_temp_c'].median():.1f} °C")
    for col in temp_columns(df):
        lines.append(f"  - {zone_label(col)}: max {df[col].max():.1f} °C, median {df[col].median():.1f} °C")
    lines.append(f"- Throttled: **{throttled_h * 60:.1f} min** ({100 * throttled_h / max(total_h, 1e-9):.1f}% of the run)"
                 + (f", first at {first_throttle.iloc[0]:.2f} h" if len(first_throttle) else ""))
    if "cpu_khz_ceiling" in df and df["cpu_khz_ceiling"].notna().any():
        lines.append(f"- Lowest clock ceiling: {df['cpu_khz_ceiling'].min() / 1000:.0f} MHz "
                     f"of {df['cpu_khz_max'].max() / 1000:.0f} MHz")
    if "motor_max_c" in df and df["motor_max_c"].notna().any():
        i = df["motor_max_c"].idxmax()
        lines.append(f"- Hottest servo: {df.loc[i, 'motor_max_c']:.0f} °C ({df.loc[i, 'motor_hottest']}) "
                     f"at {df.loc[i, 'h']:.2f} h")
    if "motor_current_peak_ma" in df and df["motor_current_peak_ma"].notna().any():
        i = df["motor_current_peak_ma"].idxmax()
        lines.append(f"- Servo current, all joints: mean {df['motor_current_ma'].mean() / 1000:.2f} A, "
                     f"peak {df.loc[i, 'motor_current_peak_ma'] / 1000:.2f} A at {df.loc[i, 'h']:.2f} h")
    if "odom_dist" in df and df["odom_dist"].notna().any():
        lines.append(f"- Odometry: max {df['odom_dist'].max():.2f} m from start, "
                     f"{(ev['kind'] == 'fence').sum()} fence trips")
    n_paused = ev["detail"].astype(str).str.startswith("paused").sum() if len(ev) else 0
    if n_paused:
        lines.append(f"- Paused {n_paused}× (fallen or picked up) — see events.csv")
    if len(drain):
        lines += ["", "| activity | hours | drain %/h | runtime if only this |", "|---|---:|---:|---:|"]
        for act, row in drain.iterrows():
            lines.append(f"| {act} | {row['hours']:.2f} | {row['pct_per_h']:.1f} | {row['runtime_h_if_only_this']:.1f} h |")
        lines += ["", "Drain is read off robotd's percentage, which is linear in voltage between 6.6 V "
                      "and full — not in charge — so per-activity rates depend on where in the curve "
                      "each activity happened. Compare them within a run, not across chemistries."]
    cur = motor_currents(df)
    if len(cur):
        lines += ["", "| joint | mean A | peak A | peak at | doing |", "|---|---:|---:|---:|---|"]
        for j, r in cur.iterrows():
            lines.append(f"| {j} | {r['mean_a']:.2f} | {r['peak_a']:.2f} | {r['peak_h']:.2f} h | {r['peak_activity']} |")
        lines += ["", "Peaks are read off every control tick, means are of each sample's mean."]
    text = "\n".join(lines) + "\n"
    (run / "summary.md").write_text(text)
    return text


def compare(runs: list[tuple[Path, pd.DataFrame]], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for i, (run, df) in enumerate(runs[: len(SERIES)]):
        ax.plot(df["h"], df["battery_pct"], color=SERIES[i], label=run.name)
        s = df.dropna(subset=["battery_pct"])
        if len(s):
            end_label(ax, s["h"].iloc[-1], s["battery_pct"].iloc[-1], f"{s['h'].iloc[-1]:.1f} h")
    ax.set_xlabel("hours since start")
    ax.set_ylabel("%")
    ax.set_ylim(0, 105)
    ax.set_title("Battery charge, run by run")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ── stress runs ─────────────────────────────────────────────────────────────────────────────

LAP_COUNTS = ["fall_triggered", "no_fall", "got_up", "getup_failed", "long_reset", "gave_up",
              "fallen_unprovoked", "square_guard", "wall_guard"]


def read_meta(run: Path) -> dict:
    path = run / "meta.json"
    return json.loads(path.read_text()) if path.exists() else {}


def lap_version(meta: dict) -> str | None:
    """The lap a stress run ran, from its latest segment; None for an endurance run."""
    segments = meta.get("segments") or [{}]
    return segments[-1].get("lap")


def mixed_laps(runs: list[Path]) -> str | None:
    """Why these runs cannot be overlaid lap for lap, or None when they can."""
    versions = {run: lap_version(read_meta(run)) for run in runs}
    stress = {run: v for run, v in versions.items() if v}
    if len(set(stress.values())) <= 1:
        return None
    first = next(iter(stress.values()))
    odd = ", ".join(f"{run.name} ({v})" for run, v in stress.items() if v != first)
    return f"not the same lap: the first stress run ran {first}, but {odd}"


def getup_seconds(detail: str) -> float:
    m = re.match(r"\s*([0-9.]+)", str(detail))
    return float(m.group(1)) if m else float("nan")


def stress_laps(df: pd.DataFrame, ev: pd.DataFrame) -> pd.DataFrame:
    """One row per lap: what happened in it and what the battery and servos were at its end.
    Empty for a run with no laps."""
    if not len(ev) or not (ev["kind"] == "lap_start").any():
        return pd.DataFrame()
    ev = ev.sort_values("wall").copy()
    ev["lap"] = (ev["kind"] == "lap_start").cumsum()
    ev = ev[ev["lap"] > 0]
    rows = []
    for lap, e in ev.groupby("lap"):
        row = {"lap": int(lap)}
        for kind in LAP_COUNTS:
            row["falls" if kind == "fall_triggered" else kind] = int((e["kind"] == kind).sum())
        times = e.loc[e["kind"] == "got_up", "detail"].map(getup_seconds)
        row["getup_median_s"] = times.median() if len(times) else float("nan")
        row["getup_max_s"] = times.max() if len(times) else float("nan")
        s = df[df["lap"] == lap] if "lap" in df else df.iloc[0:0]
        b = s["battery_pct"].dropna() if "battery_pct" in s else pd.Series(dtype=float)
        row["battery_pct"] = b.iloc[-1] if len(b) else float("nan")
        m = s["motor_max_c"].dropna() if "motor_max_c" in s else pd.Series(dtype=float)
        row["motor_max_c"] = m.max() if len(m) else float("nan")
        rows.append(row)
    return pd.DataFrame(rows).set_index("lap")


def stress_summary(laps: pd.DataFrame, version: str) -> str:
    def num(v, fmt):
        return "–" if pd.isna(v) else format(v, fmt)

    tot = laps.sum(numeric_only=True)
    times = laps["getup_median_s"].dropna()
    lines = [
        "",
        f"## Stress: {len(laps)} laps of {version}",
        "",
        f"- Falls provoked: **{int(tot['falls'])}**; no fall {int(tot['no_fall'])}, "
        f"got up {int(tot['got_up'])}, after a reset {int(tot['long_reset'])}, "
        f"gave up {int(tot['gave_up'])}",
        f"- Falls nobody asked for: {int(tot['fallen_unprovoked'])}; guards: square "
        f"{int(tot['square_guard'])}, wall {int(tot['wall_guard'])}",
    ]
    if len(times):
        lines.append(f"- Median get-up per lap: {times.iloc[0]:.2f} s on lap 1, "
                     f"{times.iloc[-1]:.2f} s on the last")
    lines += ["", "| lap | falls | no fall | got up | median s | max s | failed | gave up | "
                  "square | wall | battery % | hottest servo °C |",
              "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for lap, r in laps.iterrows():
        n = {k: int(r[k]) for k in ["falls", "no_fall", "got_up", "getup_failed", "gave_up",
                                    "square_guard", "wall_guard"]}
        lines.append(
            f"| {lap} | {n['falls']} | {n['no_fall']} | {n['got_up']} | "
            f"{num(r['getup_median_s'], '.2f')} | {num(r['getup_max_s'], '.2f')} | "
            f"{n['getup_failed']} | {n['gave_up']} | {n['square_guard']} | {n['wall_guard']} | "
            f"{num(r['battery_pct'], '.0f')} | {num(r['motor_max_c'], '.0f')} |")
    return "\n".join(lines) + "\n"


def getup_chart(run: Path, ev: pd.DataFrame) -> None:
    ev = ev.sort_values("wall").copy()
    ev["lap"] = (ev["kind"] == "lap_start").cumsum()
    up = ev[ev["kind"] == "got_up"]
    fig, ax = plt.subplots(figsize=(10, 4))
    reset = up["detail"].astype(str).str.contains("reset")
    ax.scatter(up.loc[~reset, "lap"], up.loc[~reset, "detail"].map(getup_seconds),
               s=14, color=SERIES[0], label="got up")
    ax.scatter(up.loc[reset, "lap"], up.loc[reset, "detail"].map(getup_seconds),
               s=22, color=CRITICAL, label="got up after the reset")
    ax.set_xlabel("lap")
    ax.set_ylabel("s")
    ax.set_ylim(bottom=0)
    ax.set_title("Get-up time, fall by fall")
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(run / "getup.png", dpi=130)
    plt.close(fig)


def compare_laps(runs: list[tuple[Path, pd.DataFrame]], out: Path) -> None:
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for i, (run, laps) in enumerate(runs[: len(SERIES)]):
        top.plot(laps.index, laps["getup_median_s"], color=SERIES[i], label=run.name)
        bottom.plot(laps.index, laps["battery_pct"], color=SERIES[i], label=run.name)
    top.set_ylabel("s")
    top.set_ylim(bottom=0)
    top.set_title("Median get-up time per lap")
    top.legend(loc="upper left")
    bottom.set_ylabel("%")
    bottom.set_ylim(0, 105)
    bottom.set_xlabel("lap")
    bottom.set_title("Battery at the end of each lap")
    bottom.xaxis.set_major_locator(MaxNLocator(integer=True))
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


OUTCOMES = {  # how a fall ended, in the order the bars stack; gray is not an outcome to count
    "up": ("got up", SERIES[0]),
    "up_after_reset": ("got up after the reset", SERIES[3]),
    "gave_up": ("gave up", CRITICAL),
    "no_fall": ("policy caught it", OTHER_COLOR),
}
INCIDENTS = ["getup_failed", "long_reset", "gave_up", "fallen_unprovoked", "odometry_jump",
             "robotd_unreachable", "connection_lost", "no_tof", "enable_timeout", "rise_timeout",
             "sit_timeout", "fatal", "signal", "stop"]
A4 = (11.69, 8.27)


def fall_outcomes(ev: pd.DataFrame) -> pd.DataFrame:
    """One row per fall, provoked or not, with the lap it was in, how it ended and in how long."""
    ev = ev.sort_values("wall")
    lap, open_fall, rows = 0, None, []
    for e in ev.itertuples():
        if e.kind == "lap_start":
            lap += 1
        elif e.kind in ("fall_triggered", "fallen_unprovoked"):
            open_fall = {"lap": lap, "h": e.h, "provoked": e.kind == "fall_triggered"}
        elif open_fall and e.kind in ("got_up", "no_fall", "gave_up"):
            reset = e.kind == "got_up" and "reset" in str(e.detail)
            outcome = {"got_up": "up_after_reset" if reset else "up"}.get(e.kind, e.kind)
            seconds = getup_seconds(e.detail) if e.kind == "got_up" else float("nan")
            rows.append({**open_fall, "outcome": outcome, "seconds": seconds})
            open_fall = None
    return pd.DataFrame(rows, columns=["lap", "h", "provoked", "outcome", "seconds"])


def _tile(ax, x, w, value, label, color=INK) -> None:
    ax.add_patch(plt.Rectangle((x, 0.05), w, 0.9, transform=ax.transAxes, facecolor="white",
                               edgecolor=GRID, linewidth=1))
    ax.text(x + 0.012, 0.55, value, transform=ax.transAxes, fontsize=20, fontweight="bold",
            color=color, va="center")
    ax.text(x + 0.012, 0.2, label, transform=ax.transAxes, fontsize=8.5, color=INK_2, va="center")


def _page(pdf, title: str, subtitle: str = ""):
    fig = plt.figure(figsize=A4)
    fig.text(0.04, 0.95, title, fontsize=16, fontweight="bold", color=INK, va="top")
    if subtitle:
        fig.text(0.04, 0.905, subtitle, fontsize=9.5, color=INK_2, va="top")
    return fig


def _table(ax, header: list[str], rows: list[list[str]], widths: list[float] | None = None,
           left: int = 0) -> None:
    """A table filling `ax` from the top; the first `left` columns are text, aligned left."""
    ax.axis("off")
    t = ax.table(cellText=rows, colLabels=header, loc="upper left", cellLoc="right",
                 colLoc="right", colWidths=widths)
    t.auto_set_font_size(False)
    t.set_fontsize(8.5)
    t.scale(1, 1.35)
    for (r, c), cell in t.get_celld().items():
        if c < left:
            cell.get_text().set_horizontalalignment("left")
            cell.PAD = 0.03
        cell.set_edgecolor(GRID)
        cell.set_linewidth(0.6)
        if r == 0:
            cell.set_text_props(fontweight="bold", color=INK)
            cell.set_facecolor("#f1f0ec")
        else:
            cell.set_facecolor("white" if r % 2 else SURFACE)
            cell.get_text().set_color(INK)


def stress_report(run: Path, df: pd.DataFrame, ev: pd.DataFrame, laps: pd.DataFrame,
                  meta: dict) -> Path:
    """`report.pdf`: an overview page, the per-lap table, the timeline and the incident log."""
    from matplotlib.backends.backend_pdf import PdfPages

    out = run / "report.pdf"
    falls = fall_outcomes(ev)
    provoked = falls[falls["provoked"]]
    seg = (meta.get("segments") or [{}])[0]
    minutes = (df["wall"].iloc[-1] - df["wall"].iloc[0]) / 60 if len(df) else 0
    batt = df["battery_pct"].dropna() if "battery_pct" in df else pd.Series(dtype=float)
    subtitle = "  ·  ".join(x for x in [
        str(seg.get("host") or run.name), str(seg.get("started_utc") or ""),
        f"lap {lap_version(meta) or '?'}", f"{len(laps)} laps", f"{minutes:.0f} min logged",
        f"battery {batt.iloc[0]:.0f}% → {batt.iloc[-1]:.0f}%" if len(batt) else ""] if x)
    count = lambda o, f=provoked: int((f["outcome"] == o).sum())  # noqa: E731
    fell = len(provoked) - count("no_fall")
    times = falls.loc[falls["outcome"] == "up", "seconds"]

    with PdfPages(out) as pdf:
        # 1 — overview
        fig = _page(pdf, f"Stress test — {run.name}", subtitle)
        tiles = fig.add_axes([0.04, 0.72, 0.92, 0.14])
        tiles.axis("off")
        w, gap = 0.155, 0.014
        for i, (value, label, color) in enumerate([
            (f"{len(provoked)}", "falls provoked", INK),
            (f"{100 * fell / len(provoked):.0f}%" if len(provoked) else "–",
             "of the cuts made it fall", INK),
            (f"{times.median():.2f} s" if len(times) else "–", "median get-up (first try)", INK),
            (f"{count('up_after_reset', falls)}", "needed the 2 s reset", INK),
            (f"{count('gave_up', falls)}", "gave up", CRITICAL if count("gave_up", falls) else INK),
            (f"{int((~falls['provoked']).sum())}", "falls nobody asked for", INK),
        ]):
            _tile(tiles, i * (w + gap), w, value, label, color)

        bars = fig.add_axes([0.06, 0.16, 0.4, 0.46])
        per_lap = (provoked.groupby(["lap", "outcome"]).size().unstack(fill_value=0)
                   .reindex(columns=list(OUTCOMES), fill_value=0))
        bottom = np.zeros(len(per_lap))
        for key, (label, color) in OUTCOMES.items():
            bars.bar(per_lap.index, per_lap[key], bottom=bottom, color=color, label=label,
                     width=0.7, edgecolor=SURFACE, linewidth=2)
            bottom += per_lap[key].to_numpy()
        bars.set_title("How each provoked fall ended, lap by lap")
        bars.set_xlabel("lap")
        bars.set_ylabel("falls")
        bars.xaxis.set_major_locator(MaxNLocator(integer=True))
        bars.yaxis.set_major_locator(MaxNLocator(integer=True))
        bars.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=4)

        dots = fig.add_axes([0.55, 0.16, 0.41, 0.46])
        for key in ["up", "up_after_reset"]:
            f = falls[falls["outcome"] == key]
            dots.scatter(f["lap"], f["seconds"], s=22, color=OUTCOMES[key][1],
                         edgecolor=SURFACE, linewidth=1, label=OUTCOMES[key][0], zorder=3)
        med = falls[falls["outcome"] == "up"].groupby("lap")["seconds"].median()
        dots.plot(med.index, med.to_numpy(), color=INK_2, linewidth=1.5, label="median, first try")
        dots.set_title("Get-up time, every fall")
        dots.set_xlabel("lap")
        dots.set_ylabel("seconds")
        dots.set_ylim(bottom=0)
        dots.xaxis.set_major_locator(MaxNLocator(integer=True))
        dots.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=3)
        pdf.savefig(fig)
        plt.close(fig)

        # 2 — per lap
        fig = _page(pdf, "Lap by lap", "falls: provoked  ·  got up: every fall, provoked or "
                    "not  ·  failed: still down at 7 s  ·  unasked: falls nobody provoked  ·  "
                    "square, wall: guard trips")

        def num(v, fmt):
            return "–" if pd.isna(v) else format(v, fmt)

        rows = [[str(lap), str(int(r["falls"])), str(int(r["no_fall"])), str(int(r["got_up"])),
                 num(r["getup_median_s"], ".2f"), num(r["getup_max_s"], ".2f"),
                 str(int(r["getup_failed"])), str(int(r["gave_up"])),
                 str(int(r["fallen_unprovoked"])), str(int(r["square_guard"])),
                 str(int(r["wall_guard"])), num(r["battery_pct"], ".0f"),
                 num(r["motor_max_c"], ".0f")] for lap, r in laps.iterrows()]
        header = ["lap", "falls", "no fall", "got up", "median s", "max s", "failed",
                  "gave up", "unasked", "square", "wall", "battery %", "servo °C"]
        for i in range(0, max(len(rows), 1), 28):
            if i:
                fig = _page(pdf, "Lap by lap (continued)")
            _table(fig.add_axes([0.04, 0.05, 0.92, 0.82]), header, rows[i:i + 28])
            pdf.savefig(fig)
            plt.close(fig)

        # 3 — timeline, drawn again rather than pasted from the PNG, so it stays sharp
        fig = timeline_figure(run, df, ev)
        pdf.savefig(fig)
        plt.close(fig)

        # 4 — servo currents
        cur = motor_currents(df)
        if len(cur):
            fig = currents_figure(run, cur, figsize=A4)
            pdf.savefig(fig)
            plt.close(fig)

        # 5 — incidents
        ev = ev.sort_values("wall").copy()
        ev["lap"] = (ev["kind"] == "lap_start").cumsum()
        steps = ev["detail"].where(ev["kind"] == "activity").astype(str).str.split().str[-1]
        ev["step"] = steps.ffill().fillna("")
        inc = ev[ev["kind"].isin(INCIDENTS)]
        rows = [[f"{(r.wall - ev['wall'].iloc[0]) / 60:.1f}", str(r.lap or ""), r.step, r.kind,
                 str(r.detail)[:70] if not pd.isna(r.detail) else ""] for r in inc.itertuples()]
        if not rows:
            rows = [["", "", "", "nothing went wrong", ""]]
        for i in range(0, len(rows), 30):
            fig = _page(pdf, "Incidents" + (" (continued)" if i else ""),
                        "" if i else "everything that was not the lap going to plan")
            _table(fig.add_axes([0.04, 0.05, 0.92, 0.82]),
                   ["min", "lap", "step", "event", "detail"], rows[i:i + 30],
                   [0.06, 0.05, 0.12, 0.17, 0.6], left=5)
            pdf.savefig(fig)
            plt.close(fig)
        d = pdf.infodict()
        d["Title"] = f"Stress test — {run.name}"
        d["Subject"] = subtitle
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("runs", nargs="+", type=Path, help="run directories written by robotctl endurance or stress")
    args = p.parse_args()
    for run in args.runs:
        if not (run / "samples.csv").exists():
            print(f"{run}: no samples.csv", file=sys.stderr)
            return 1
    if why := mixed_laps(args.runs):
        print(why, file=sys.stderr)
        return 1
    loaded, lapped = [], []
    for run in args.runs:
        df, ev = load(run)
        meta = read_meta(run)
        timeline(run, df, ev)
        drain = breakdown(run, df, ev, meta)
        text = summary(run, df, ev, drain)
        made = [run / "timeline.png", run / "breakdown.png"]
        cur = motor_currents(df)
        if len(cur):
            fig = currents_figure(run, cur)
            fig.savefig(run / "currents.png", dpi=130)
            plt.close(fig)
            made.append(run / "currents.png")
        laps = stress_laps(df, ev)
        if len(laps):
            text += stress_summary(laps, lap_version(meta) or "?")
            (run / "summary.md").write_text(text)
            getup_chart(run, ev)
            made += [run / "getup.png", stress_report(run, df, ev, laps, meta)]
            lapped.append((run, laps))
        print(text)
        print("".join(f"→ {m}\n" for m in made + [run / "summary.md"]))
        loaded.append((run, df))
    if len(loaded) > 1:
        out = args.runs[0].parent / "compare.png"
        if len(lapped) == len(loaded):
            compare_laps(lapped, out)
        else:
            compare(loaded, out)
        print(f"→ {out}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
