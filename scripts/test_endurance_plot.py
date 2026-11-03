"""Tests for `endurance-plot.py`'s stress-run tables.

    uv run --with pytest --with pandas --with numpy --with matplotlib pytest scripts/test_endurance_plot.py
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("endurance_plot", Path(__file__).with_name("endurance-plot.py"))
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


def write_run(run: Path, lap_version: str | None, events: list[tuple[float, str, str]]) -> Path:
    run.mkdir(parents=True)
    seg = {"started_wall": 1000.0, "host": run.name}
    if lap_version:
        seg["lap"] = lap_version
    (run / "meta.json").write_text(json.dumps({"segments": [seg]}))
    rows = ["wall,t_s,robotd,battery_v,battery_pct,throttled,motor_max_c,motor_mean_c,"
            "motor_hottest,activity,lap,step"]
    for i in range(12):
        lap = 1 + i // 4
        rows.append(f"{1000 + 10 * i},{10 * i},up,8.0,{97 - i},0,{30 + i},{25 + i},knee,walk,"
                    f"{lap},A1.fwd")
    (run / "samples.csv").write_text("\n".join(rows) + "\n")
    lines = ["wall,t_s,kind,detail"] + [f"{1000 + t},{t},{k},{d}" for t, k, d in events]
    (run / "events.csv").write_text("\n".join(lines) + "\n")
    return run


def lap_events(t0: float, getups: list[str | None]) -> list[tuple[float, str, str]]:
    """One lap: a cut per entry, then `got_up` with that detail, or `no_fall` for None."""
    ev = [(t0, "lap_start", "lap n (stress-v1)")]
    for i, detail in enumerate(getups):
        t = t0 + 1 + 3 * i
        ev.append((t, "fall_triggered", "forward"))
        if detail is None:
            ev.append((t + 2, "no_fall", "the policy caught it"))
        else:
            ev.append((t + 0.3, "fallen", ""))
            ev.append((t + 2, "got_up", detail))
    return ev


def test_per_lap_table_counts_falls_and_getups(tmp_path):
    events = lap_events(0, ["1.50 s", "2.00 s", None, "2.50 s", "9.10 s after the reset", "1.00 s"])
    events.insert(-2, (14.0, "getup_failed", "still down after 7 s"))
    events += lap_events(40, ["1.00 s"] * 6)
    events.append((60.0, "wall_guard", "0.15 m ahead"))
    run = write_run(tmp_path / "duck-a", "stress-v1", events)
    df, ev = plot.load(run)

    laps = plot.stress_laps(df, ev)

    one = laps.loc[1]
    assert one["falls"] == 6
    assert one["got_up"] == 5
    assert one["no_fall"] == 1
    assert one["getup_failed"] == 1
    assert one["getup_median_s"] == pytest.approx(2.0)
    assert one["getup_max_s"] == pytest.approx(9.1)
    two = laps.loc[2]
    assert two["got_up"] == 6
    assert two["wall_guard"] == 1
    assert two["gave_up"] == 0


def test_a_run_without_laps_has_no_lap_table(tmp_path):
    run = write_run(tmp_path / "duck-a", None, [(5, "fence", "0.5 m")])
    df, ev = plot.load(run)
    assert plot.lap_version(plot.read_meta(run)) is None
    assert plot.stress_laps(df, ev).empty


def test_overlay_refuses_mixed_lap_versions(tmp_path):
    a = write_run(tmp_path / "duck-a", "stress-v1", lap_events(0, ["1.00 s"]))
    b = write_run(tmp_path / "duck-b", "stress-v2", lap_events(0, ["1.00 s"]))
    c = write_run(tmp_path / "duck-c", "stress-v1", lap_events(0, ["1.00 s"]))

    why = plot.mixed_laps([a, b, c])

    assert why is not None
    assert "duck-b" in why and "stress-v2" in why
    assert plot.mixed_laps([a, c]) is None


def test_each_fall_is_paired_with_how_it_ended(tmp_path):
    events = lap_events(0, ["1.50 s", None, "9.70 s after the reset"])
    events.append((12.0, "fallen_unprovoked", ""))
    events.append((13.0, "got_up", "2.00 s"))
    events.append((20.0, "fall_triggered", "backward"))
    events.append((35.0, "gave_up", "still down after the reset; torque off"))
    run = write_run(tmp_path / "duck-a", "stress-v1", events)
    _, ev = plot.load(run)

    falls = plot.fall_outcomes(ev)

    assert list(falls["outcome"]) == ["up", "no_fall", "up_after_reset", "up", "gave_up"]
    assert list(falls["provoked"]) == [True, True, True, False, True]
    assert falls["seconds"].iloc[2] == pytest.approx(9.7)
    assert set(falls["lap"]) == {1}


def test_a_stress_run_gets_a_pdf_report(tmp_path):
    events = lap_events(0, ["1.50 s", None, "9.70 s after the reset"])
    events.insert(1, (0.5, "activity", "walk A1.fwd"))
    events.append((30.0, "getup_failed", "still down after 7 s"))
    run = write_run(tmp_path / "duck-a", "stress-v1", events)
    df, ev = plot.load(run)

    out = plot.stress_report(run, df, ev, plot.stress_laps(df, ev), plot.read_meta(run))

    assert out == run / "report.pdf"
    assert out.read_bytes()[:5] == b"%PDF-"
    assert out.read_bytes().count(b"/Type /Page\n") + out.read_bytes().count(b"/Type /Page ") >= 4


def test_concatenate_runs_them_back_to_back_with_laps_numbered_on(tmp_path):
    parent = tmp_path / "series"
    # Three laps of samples each (see write_run); the second run starts an hour later, on the
    # same wall clock, and must follow the first with the gap cut out.
    a = write_run(parent / "duck-a", "stress-v1",
                  lap_events(0, ["1.00 s"] * 6) + lap_events(40, ["2.00 s"] * 6))
    b = write_run(parent / "duck-b", "stress-v1", lap_events(0, ["3.00 s"] * 6))
    for f in ("samples.csv", "events.csv"):
        text = (b / f).read_text().splitlines()
        rows = [text[0]] + [str(float(r.split(",", 1)[0]) + 3600) + "," + r.split(",", 1)[1] for r in text[1:]]
        (b / f).write_text("\n".join(rows) + "\n")

    out = plot.concatenate(parent)
    df, ev = plot.load(out)
    assert out == parent / "concatenated"
    # 12 samples each, 10 s apart: the second run starts 10 s after the first's last sample.
    assert df["wall"].is_monotonic_increasing
    assert df["wall"].iloc[12] - df["wall"].iloc[11] == 10
    assert list(ev.loc[ev["kind"] == "session", "detail"]) == ["duck-a", "duck-b"]

    laps = plot.stress_laps(df, ev)
    assert list(laps.index) == [1, 2, 3]
    assert laps.loc[3, "getup_median_s"] == 3.0
    # Run b's samples were laps 1-3 of its own; lap 3 of the series is its lap 1.
    assert set(df.loc[12:, "lap"]) == {3, 4, 5}
    # Each run drains 97 → 86 % walking; the jump back to 97 at the join is a charge, not a drain.
    drain = plot.drain_by_activity(df)
    assert drain.loc["walk", "pct"] == 22
    meta = json.loads((out / "meta.json").read_text())
    assert meta["concatenated"] == ["duck-a", "duck-b"]
    assert len(meta["segments"]) == 2


def test_concatenate_refuses_different_laps(tmp_path):
    parent = tmp_path / "series"
    write_run(parent / "duck-a", "stress-v1", lap_events(0, ["1.00 s"]))
    write_run(parent / "duck-b", "stress-v2", lap_events(0, ["1.00 s"]))
    with pytest.raises(SystemExit):
        plot.concatenate(parent)


def test_the_timeline_runs_on_the_runs_own_timer(tmp_path):
    run = write_run(tmp_path / "duck-a", "stress-v1", lap_events(0, ["1.00 s"]))
    # The board syncs its clock six samples in: the wall clock leaps an hour, t_s does not.
    lines = (run / "samples.csv").read_text().splitlines()
    out = [lines[0]]
    for i, row in enumerate(lines[1:]):
        wall, rest = row.split(",", 1)
        out.append(f"{float(wall) + (3600 if i >= 6 else 0)},{rest}")
    (run / "samples.csv").write_text("\n".join(out) + "\n")
    df, _ = plot.load(run)
    assert df["h"].diff().max() * 3600 == pytest.approx(10)


def test_a_resumed_run_follows_the_piece_before_it():
    samples = plot.pd.Series([0.0, 10.0, 20.0, 0.0, 10.0])
    events = plot.pd.Series([0.0, 25.0, 0.0, 12.0])
    s, e = plot.run_hours(samples, events)
    # The first piece ends at 25 s (its last event), so the second starts there.
    assert list(s * 3600) == pytest.approx([0, 10, 20, 25, 35])
    assert list(e * 3600) == pytest.approx([0, 25, 25, 37])
