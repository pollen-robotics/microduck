#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib>=3.8", "numpy>=1.26", "pandas>=2.1"]
# ///
"""Charts and a summary for a run of `robotctl endurance`.

    uv run scripts/endurance-plot.py <run dir> [<run dir> ...]

Writes into each run directory:

    timeline.png   battery, voltage, board temperatures, clock ceiling, motors — one panel each,
                   on a shared time axis, with what the robot was doing shaded behind them
    breakdown.png  the odometry track against the 0.5 m circle, time per activity, drain rate
                   and board temperature per activity
    summary.md     the numbers: runtime, peak temperatures, time throttled, drain per activity

Given several runs it also writes `compare.png` beside the first, battery against time for each.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

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


def timeline(run: Path, df: pd.DataFrame, ev: pd.DataFrame) -> None:
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
        ax.plot(df["h"], df["motor_current_ma"] / 1000, color=SERIES[0], lw=1)
        ax.set_ylabel("A")
        ax.set_title("Servo current, sum of |joint current|, mean over each sample")

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
    fig.savefig(run / "timeline.png", dpi=130)
    plt.close(fig)


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


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("runs", nargs="+", type=Path, help="run directories written by robotctl endurance")
    args = p.parse_args()
    loaded = []
    for run in args.runs:
        if not (run / "samples.csv").exists():
            print(f"{run}: no samples.csv", file=sys.stderr)
            return 1
        df, ev = load(run)
        meta = json.loads((run / "meta.json").read_text()) if (run / "meta.json").exists() else {}
        timeline(run, df, ev)
        drain = breakdown(run, df, ev, meta)
        print(summary(run, df, ev, drain))
        print(f"→ {run / 'timeline.png'}\n→ {run / 'breakdown.png'}\n→ {run / 'summary.md'}\n")
        loaded.append((run, df))
    if len(loaded) > 1:
        out = args.runs[0].parent / "compare.png"
        compare(loaded, out)
        print(f"→ {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
