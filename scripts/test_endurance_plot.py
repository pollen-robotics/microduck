"""Synthetic regression tests for scripts/endurance-plot.py; no robot or hardware required."""

from __future__ import annotations

import csv
import importlib.util
import io
import json
import math
import sys
import tempfile
import unittest
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

import numpy as np

SCRIPT = Path(__file__).with_name("endurance-plot.py")
SPEC = importlib.util.spec_from_file_location("endurance_plot", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
plotter = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = plotter
SPEC.loader.exec_module(plotter)
plt = plotter.plt


def session_rows(walls, activity="walk", *, battery=None, throttled=None, t0=0.0, uptime0=1000.0, **extra):
    rows = []
    for i, wall in enumerate(walls):
        row = {
            "wall": float(wall),
            "t_s": t0 + float(wall) - float(walls[0]),
            "uptime_s": uptime0 + float(wall) - float(walls[0]),
            "activity": activity[i] if isinstance(activity, list) else activity,
        }
        if battery is not None:
            row["battery_pct"] = battery[i]
        if throttled is not None:
            row["throttled"] = throttled[i]
        for key, value in extra.items():
            row[key] = value[i] if isinstance(value, list) else value
        rows.append(row)
    return rows


def meta_for(starts, fences=None, interval=10.0):
    segments = []
    for i, start in enumerate(starts):
        args = {"interval": interval}
        if fences is not None:
            args["fence"] = fences[i]
        segments.append({"started_wall": start, "args": args})
    return {"segments": segments}


def write_run(root: Path, name: str, rows, *, meta=None, events=None, columns=None, raw_samples=None):
    run = root / name
    run.mkdir(parents=True, exist_ok=True)
    path = run / "samples.csv"
    if raw_samples is not None:
        path.write_text(raw_samples)
    else:
        if columns is None:
            columns = list(dict.fromkeys(key for row in rows for key in row))
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
    if meta is not None:
        (run / "meta.json").write_text(json.dumps(meta))
    if events is not None:
        if isinstance(events, str):
            (run / "events.csv").write_text(events)
        else:
            with (run / "events.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=["wall", "t_s", "kind", "detail"])
                writer.writeheader()
                writer.writerows(events)
    return run


def call_main(runs):
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        status = plotter.main([str(run) for run in runs])
    return status, out.getvalue(), err.getvalue()


@contextmanager
def captured_figure(function, *args, **kwargs):
    with mock.patch.object(plt, "close") as close:
        result = function(*args, **kwargs)
    figure = close.call_args.args[0]
    try:
        yield figure, result
    finally:
        plt.close(figure)


class LoadingAndSegmentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def test_normal_single_session_uses_wall_time_and_one_segment(self):
        run = write_run(self.root, "normal", session_rows([1000, 1010, 1020, 1030], battery=[90, 89, 88, 87]),
                        meta=meta_for([1000], [0.45]))
        data = plotter.load(run)
        self.assertEqual(data.df["segment"].tolist(), [1, 1, 1, 1])
        self.assertEqual(data.df["meta_segment"].tolist(), [0, 0, 0, 0])
        np.testing.assert_allclose(data.df["h"], [0, 10 / 3600, 20 / 3600, 30 / 3600])
        self.assertEqual(len(plotter.valid_intervals(data.df)), 3)
        self.assertAlmostEqual(data.df["sample_s"].iloc[0], 10)

    def test_appended_logger_session_is_detected_when_t_s_resets(self):
        rows = session_rows([0, 10, 20]) + session_rows([120, 130, 140], t0=0, uptime0=1120)
        run = write_run(self.root, "restart", rows, meta=meta_for([0, 120], [0.3, 0.6]))
        data = plotter.load(run)
        self.assertEqual(data.df["segment"].tolist(), [1, 1, 1, 2, 2, 2])
        self.assertEqual(data.df["meta_segment"].tolist(), [0, 0, 0, 1, 1, 1])
        intervals = plotter.valid_intervals(data.df)
        self.assertNotIn(2, intervals["left"].tolist())  # the downtime pair must never be measured
        self.assertAlmostEqual(intervals["dh"].sum() * 3600, 40)

    def test_machine_reboot_is_detected_from_uptime_even_without_t_s(self):
        rows = session_rows([0, 10, 120, 130], uptime0=500)
        for row in rows:
            del row["t_s"]
        rows[2]["uptime_s"] = 5
        rows[3]["uptime_s"] = 15
        run = write_run(self.root, "reboot", rows, meta=meta_for([0, 120]))
        data = plotter.load(run)
        self.assertEqual(data.df["segment"].tolist(), [1, 1, 2, 2])
        self.assertNotIn(1, plotter.valid_intervals(data.df)["left"].tolist())

    def test_metadata_can_support_a_boundary_only_at_a_gap_when_counters_are_missing(self):
        rows = session_rows([0, 10, 1000, 1010])
        for row in rows:
            del row["t_s"]
            del row["uptime_s"]
        run = write_run(self.root, "meta-gap", rows, meta=meta_for([0, 1000]))
        data = plotter.load(run)
        self.assertEqual(data.df["segment"].tolist(), [1, 1, 2, 2])
        self.assertTrue(any("cannot be ruled out" in message for message in data.warnings))

    def test_continuous_uptime_does_not_veto_a_metadata_supported_process_restart(self):
        rows = session_rows([0, 10, 1000, 1010], uptime0=5000)
        for row in rows:
            del row["t_s"]  # machine uptime continues, but the logger process may still restart
        run = write_run(self.root, "process-only", rows, meta=meta_for([0, 1000], [0.3, 0.6]))
        data = plotter.load(run)
        self.assertEqual(data.df["segment"].tolist(), [1, 1, 2, 2])
        self.assertEqual(data.df["meta_segment"].tolist(), [0, 0, 1, 1])
        self.assertNotIn(1, plotter.valid_intervals(data.df)["left"].tolist())

    def test_metadata_start_alone_does_not_invent_a_boundary(self):
        rows = session_rows([0, 10, 20, 30])
        for row in rows:
            del row["t_s"]
            del row["uptime_s"]
        run = write_run(self.root, "meta-only", rows, meta=meta_for([0, 20]))
        data = plotter.load(run)
        self.assertEqual(data.df["segment"].nunique(), 1)
        self.assertTrue(any("no session boundary was invented" in message for message in data.warnings))

    def test_continuous_counters_override_a_conflicting_metadata_start(self):
        run = write_run(self.root, "conflict", session_rows([0, 10, 20, 1000, 1010, 1020, 1030]),
                        meta=meta_for([0, 1000]))
        data = plotter.load(run)
        # t_s/uptime_s continue through the wall gap, so the extra start is not sample evidence.
        self.assertEqual(data.df["segment"].nunique(), 1)
        self.assertTrue(any("conflicts with continuous" in message for message in data.warnings))

    def test_malformed_optional_metadata_warns_but_valid_csv_still_loads(self):
        variants = [
            "not json",
            "[]",
            json.dumps({"segments": {}}),
            json.dumps({"segments": [None, {"started_wall": "bad", "args": {"fence": 0.3}}]}),
            json.dumps({"segments": [{"started_wall": 10, "args": {"fence": 0.3}},
                                      {"started_wall": 5, "args": {"fence": 0.6}}]}),
        ]
        for index, raw in enumerate(variants):
            with self.subTest(index=index):
                run = write_run(self.root, f"bad-meta-{index}", session_rows([0, 10, 20]))
                (run / "meta.json").write_text(raw)
                data = plotter.load(run)
                self.assertEqual(len(data.df), 3)
                self.assertTrue(data.warnings)
                self.assertEqual(data.meta_segments, [])

    def test_invalid_required_input_fails_with_a_useful_error(self):
        cases = {
            "missing": None,
            "empty": "",
            "header": "wall,activity\n",
            "missing-column": "wall,battery_pct\n0,90\n",
            "bad-wall": "wall,activity\nnope,walk\n",
            "infinite-wall": "wall,activity\ninf,walk\n",
            "bad-activity": "wall,activity\n0,\n",
            "numeric-activity": "wall,activity\n0,123\n",
        }
        for name, raw in cases.items():
            with self.subTest(name=name):
                run = self.root / name
                run.mkdir()
                if raw is not None:
                    (run / "samples.csv").write_text(raw)
                with self.assertRaises(plotter.InputError) as caught:
                    plotter.load(run)
                self.assertIn(str(run), str(caught.exception))

    def test_missing_and_invalid_optional_telemetry_remain_available_or_unknown(self):
        rows = session_rows([0, 10, 20], battery=[90, "bad", 88], throttled=[1, 2, 0],
                            cpu_temp_c=[40, "bad", 42])
        run = write_run(self.root, "optional", rows)
        data = plotter.load(run)
        self.assertTrue(math.isnan(data.df["battery_pct"].iloc[1]))
        self.assertTrue(math.isnan(data.df["throttled"].iloc[1]))
        self.assertTrue(math.isnan(data.df["cpu_temp_c"].iloc[1]))
        self.assertTrue(any("other than 0/1" in message for message in data.warnings))
        status, _, err = call_main([run])
        self.assertEqual(status, 0, err)
        self.assertTrue((run / "timeline.png").exists())

    def test_missing_empty_and_malformed_events_do_not_block_plotting(self):
        variants = [None, "", "wall,t_s,kind,detail\n", "wall,kind\n0,fence\n", 'wall,kind,detail\n0,"unterminated']
        for index, events in enumerate(variants):
            with self.subTest(index=index):
                run = write_run(self.root, f"events-{index}", session_rows([0, 10, 20]), events=events)
                data = plotter.load(run)
                self.assertEqual(len(data.df), 3)
                if index in (0, 1, 4):  # missing, empty, or unparsable: no event counts are available
                    self.assertFalse(data.events_available)
                elif index == 3:  # missing detail is warned about, but fence kinds remain usable
                    self.assertTrue(data.events_available)
                    self.assertEqual(int((data.ev["kind"] == "fence").sum()), 1)
                if index != 2:  # a valid header-only events file legitimately has zero events
                    self.assertTrue(any("events.csv" in message for message in data.warnings))
                status, _, err = call_main([run])
                self.assertEqual(status, 0, err)

    def test_backwards_wall_intervals_are_excluded_and_warned_about(self):
        rows = session_rows([0, 10, 5, 20], battery=[90, 89, 88, 87])
        run = write_run(self.root, "backwards", rows)
        data = plotter.load(run)
        intervals = plotter.valid_intervals(data.df)
        self.assertEqual(intervals["left"].tolist(), [0, 2])
        self.assertTrue((intervals["dh"] > 0).all())
        self.assertTrue(any("do not increase" in message for message in data.warnings))
        drain = plotter.drain_by_activity(data.df)
        self.assertGreater(drain.loc["walk", "pct_per_h"], 0)
        self.assertTrue(math.isfinite(drain.loc["walk", "pct_per_h"]))

    def test_single_sample_has_zero_measured_time_and_no_nan_summary(self):
        run = write_run(self.root, "single", session_rows([42], battery=[90], throttled=[1]))
        status, out, err = call_main([run])
        self.assertEqual(status, 0, err)
        data = plotter.load(run)
        self.assertTrue(plotter.valid_intervals(data.df).empty)
        self.assertTrue(plotter.activity_intervals(data.df).empty)
        self.assertEqual(plotter.spans(data.df), [])
        self.assertIn("Sample-covered time: **0.00 h**", out)
        self.assertNotRegex(out.lower(), r"\bnan\b|\binf\b|infinity")
        self.assertTrue((run / "timeline.png").exists())
        self.assertTrue((run / "breakdown.png").exists())

    def test_minimal_required_columns_only_still_plot(self):
        run = write_run(self.root, "minimal", [{"wall": 0, "activity": "walk"}])
        status, out, err = call_main([run])
        self.assertEqual(status, 0, err)
        self.assertIn("Throttle telemetry: unavailable", out)
        self.assertTrue((run / "timeline.png").exists())
        self.assertTrue((run / "breakdown.png").exists())
        self.assertTrue((run / "summary.md").exists())


class IntervalMetricTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def load_rows(self, rows, **kwargs):
        return plotter.load(write_run(self.root, "metrics", rows, **kwargs))

    def test_activity_duration_uses_actual_irregular_intervals_not_count_times_median(self):
        data = self.load_rows(session_rows([0, 1, 3, 6]))
        durations = plotter.activity_intervals(data.df).groupby("activity")["dh"].sum() * 3600
        self.assertAlmostEqual(durations["walk"], 6)
        self.assertNotAlmostEqual(durations["walk"], 4 * 2)  # old count × median would overcount

    def test_restart_gap_is_not_attributed_to_an_activity_or_shaded(self):
        rows = session_rows([0, 10, 20]) + session_rows([3600, 3610, 3620])
        data = self.load_rows(rows, meta=meta_for([0, 3600]))
        durations = plotter.activity_intervals(data.df).groupby("activity")["dh"].sum() * 3600
        self.assertAlmostEqual(durations["walk"], 40)
        for start, end, _ in plotter.spans(data.df):
            self.assertTrue(end <= 20 / 3600 or start >= 3600 / 3600)
            self.assertLess(end - start, 3000 / 3600)

    def test_obvious_sampling_gap_is_excluded_by_per_segment_median_rule(self):
        data = self.load_rows(session_rows([0, 10, 20, 30, 330, 340, 350, 360]))
        intervals = plotter.valid_intervals(data.df)
        self.assertNotIn(3, intervals["left"].tolist())
        self.assertAlmostEqual(intervals["dh"].sum() * 3600, 60)
        self.assertTrue(any("obvious sampling gap" in message for message in data.warnings))

    def test_short_session_uses_configured_interval_to_exclude_long_gap(self):
        # Two intervals are too few for an empirical median; without the metadata fallback
        # this 990 s stall would be counted as walk time.
        data = self.load_rows(session_rows([0, 10, 1000]), meta=meta_for([0], interval=10.0))
        self.assertEqual(data.df["segment"].nunique(), 1)
        self.assertAlmostEqual(data.df["sample_s"].iloc[0], 10.0)
        intervals = plotter.valid_intervals(data.df)
        self.assertEqual(intervals["left"].tolist(), [0])
        self.assertAlmostEqual(intervals["dh"].sum() * 3600, 10)
        durations = plotter.activity_intervals(data.df).groupby("activity")["dh"].sum() * 3600
        self.assertAlmostEqual(durations["walk"], 10)
        self.assertTrue(any("obvious sampling gap" in message for message in data.warnings))

    def test_unknown_activity_transition_interval_is_not_assigned(self):
        data = self.load_rows(session_rows([0, 10, 20, 30], activity=["walk", "walk", "stand", "stand"]))
        intervals = plotter.activity_intervals(data.df)
        self.assertEqual(intervals["left"].tolist(), [0, 2])
        self.assertAlmostEqual(intervals["dh"].sum() * 3600, 20)

    def test_missing_battery_between_valid_readings_is_not_bridged(self):
        data = self.load_rows(session_rows([0, 10, 20], battery=[100, np.nan, 90]))
        drain = plotter.drain_by_activity(data.df)
        self.assertEqual(drain.loc["walk", "hours"], 0)
        self.assertTrue(math.isnan(drain.loc["walk", "pct_per_h"]))
        self.assertTrue(math.isnan(drain.loc["walk", "runtime_h_if_only_this"]))

    def test_battery_drain_handles_positive_zero_and_negative_rates(self):
        rows = (session_rows([0, 10], "walk", battery=[90, 89])
                + session_rows([20, 30], "stand", battery=[80, 80])
                + session_rows([40, 50], "sit", battery=[70, 71]))
        data = self.load_rows(rows)
        drain = plotter.drain_by_activity(data.df)
        self.assertGreater(drain.loc["walk", "pct_per_h"], 0)
        self.assertTrue(math.isfinite(drain.loc["walk", "runtime_h_if_only_this"]))
        self.assertEqual(drain.loc["stand", "pct_per_h"], 0)
        self.assertTrue(math.isnan(drain.loc["stand", "runtime_h_if_only_this"]))
        self.assertLess(drain.loc["sit", "pct_per_h"], 0)
        self.assertTrue(math.isnan(drain.loc["sit", "runtime_h_if_only_this"]))

    def test_battery_drain_does_not_cross_sessions_or_activity_changes(self):
        rows = (session_rows([0, 10], battery=[90, 80])
                + session_rows([1000, 1010], battery=[70, 60]))
        data = self.load_rows(rows, meta=meta_for([0, 1000]))
        drain = plotter.drain_by_activity(data.df)
        self.assertAlmostEqual(drain.loc["walk", "hours"] * 3600, 20)
        self.assertAlmostEqual(drain.loc["walk", "pct"], 20)
        changed = self.load_rows(session_rows([0, 10, 20], activity=["walk", "stand", "stand"],
                                              battery=[90, 80, 70]))
        self.assertEqual(plotter.drain_by_activity(changed.df).loc["walk", "hours"], 0)

    def test_unknown_throttle_is_never_converted_to_false(self):
        data = self.load_rows(session_rows([0, 10, 20, 30], throttled=[1, np.nan, 0, 2]))
        self.assertTrue(math.isnan(data.df["throttled"].iloc[1]))
        self.assertTrue(math.isnan(data.df["throttled"].iloc[3]))
        self.assertTrue(plotter.throttle_intervals(data.df).empty)
        self.assertEqual(plotter.throttle_spans(data.df), [])

    def test_throttle_duration_and_shading_share_stable_known_intervals(self):
        data = self.load_rows(session_rows([0, 10, 20, 30, 40, 50], throttled=[1, 1, 0, 0, np.nan, 1]))
        intervals = plotter.throttle_intervals(data.df)
        self.assertEqual(intervals["left"].tolist(), [0, 2])
        self.assertAlmostEqual(intervals["dh"].sum() * 3600, 20)
        self.assertAlmostEqual(intervals.loc[intervals["throttled"] == 1, "dh"].sum() * 3600, 10)
        self.assertEqual(plotter.throttle_spans(data.df), [(0.0, 10 / 3600, 1)])

    def test_no_usable_throttle_telemetry_is_reported_unavailable_not_zero(self):
        data = self.load_rows(session_rows([0, 10, 20]))
        text = plotter.summary(data, plotter.drain_by_activity(data.df))
        self.assertIn("Throttle telemetry: unavailable", text)
        self.assertNotIn("Throttled: **0.0 min**", text)

    def test_known_zero_throttle_is_reported_with_coverage(self):
        data = self.load_rows(session_rows([0, 10, 20], throttled=[0, 0, 0]))
        text = plotter.summary(data, plotter.drain_by_activity(data.df))
        self.assertIn("Throttled: **0.0 min**", text)
        self.assertIn("known stable throttle coverage", text)


class OdometryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def test_odometry_is_rebased_per_session_and_fences_follow_metadata(self):
        rows = (session_rows([0, 10], odom_x=[10, 11], odom_y=[20, 21], odom_dist=[0, 1.414])
                + session_rows([1000, 1010], odom_x=[100, 102], odom_y=[200, 203], odom_dist=[0, 3.606]))
        run = write_run(self.root, "odom", rows, meta=meta_for([0, 1000], [0.3, 0.6]))
        data = plotter.load(run)
        tracks = plotter.odometry_tracks(data.df, data.meta_segments)
        self.assertEqual(len(tracks), 2)
        np.testing.assert_allclose(tracks[0]["data"][["x", "y"]].to_numpy(), [[0, 0], [1, 1]])
        np.testing.assert_allclose(tracks[1]["data"][["x", "y"]].to_numpy(), [[0, 0], [2, 3]])
        self.assertEqual([track["fence"] for track in tracks], [0.3, 0.6])
        self.assertAlmostEqual(tracks[0]["max_distance"], 1.414)
        self.assertAlmostEqual(tracks[1]["max_distance"], 3.606)
        with captured_figure(plotter.breakdown, data) as (figure, _):
            circles = [patch.radius for patch in figure.axes[0].patches if isinstance(patch, plotter.Circle)]
            self.assertIn(0.3, circles)
            self.assertIn(0.6, circles)
            self.assertIn(0.45, circles)  # labelled reference, not a claimed configured fence
        text = plotter.summary(data, plotter.drain_by_activity(data.df))
        self.assertIn("session-relative, not a continuous full-run distance", text)
        self.assertIn("configured fence radius: 0.3 m", text)
        self.assertIn("configured fence radius: 0.6 m", text)

    def test_unmatched_session_does_not_reuse_the_last_configured_fence(self):
        rows = (session_rows([0, 10], odom_x=[0, 0.1], odom_y=[0, 0.1])
                + session_rows([1000, 1010], odom_x=[5, 5.1], odom_y=[5, 5.1]))
        run = write_run(self.root, "unmatched", rows, meta=meta_for([0], [0.3]))
        data = plotter.load(run)
        tracks = plotter.odometry_tracks(data.df, data.meta_segments)
        self.assertEqual(tracks[0]["fence"], 0.3)
        self.assertIsNone(tracks[1]["fence"])
        self.assertTrue(any("configured fence is unavailable" in message for message in data.warnings))

    def test_invalid_metadata_does_not_invent_a_custom_fence(self):
        run = write_run(self.root, "no-fence", session_rows([0, 10], odom_x=[1, 2], odom_y=[1, 2]),
                        meta={"segments": [{"started_wall": 0, "args": {"fence": "bad"}}]})
        data = plotter.load(run)
        tracks = plotter.odometry_tracks(data.df, data.meta_segments)
        self.assertIsNone(tracks[0]["fence"])
        self.assertTrue(any("invalid fence radius" in message for message in data.warnings))


class PlotOutputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def assert_png(self, path):
        self.assertTrue(path.exists(), path)
        self.assertEqual(path.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
        image = plt.imread(path)
        self.assertGreater(image.shape[0], 0)
        self.assertGreater(image.shape[1], 0)
        self.assertTrue(np.isfinite(image).all())

    def test_end_to_end_single_and_multiple_runs_write_expected_valid_outputs(self):
        first = write_run(self.root, "first", session_rows([0, 10, 20], battery=[90, 89, 88],
                                                           cpu_temp_c=[40, 41, 42]),
                          meta=meta_for([0], [0.45]), events=[{"wall": 0, "t_s": 0, "kind": "start", "detail": "go"}])
        second = write_run(self.root, "second", session_rows([0, 10, 20], battery=[80, 79, 78]))
        status, out, err = call_main([first])
        self.assertEqual(status, 0, err)
        for name in ("timeline.png", "breakdown.png"):
            self.assert_png(first / name)
        self.assertTrue((first / "summary.md").exists())
        self.assertFalse((self.root / "compare.png").exists())
        status, out, err = call_main([first, second])
        self.assertEqual(status, 0, err)
        self.assert_png(self.root / "compare.png")
        self.assertIn("Wall-clock span", out)
        self.assertIn("Sample-covered time", out)

    def test_all_inputs_are_validated_before_any_output_is_written(self):
        good = write_run(self.root, "good", session_rows([0, 10, 20]))
        bad = write_run(self.root, "bad", [], columns=["wall", "activity"])
        status, _, err = call_main([good, bad])
        self.assertEqual(status, 1)
        self.assertIn("header-only", err)
        self.assertNotIn("Traceback", err)
        self.assertFalse((good / "timeline.png").exists())
        self.assertFalse((good / "breakdown.png").exists())
        self.assertFalse((good / "summary.md").exists())
        self.assertFalse((self.root / "compare.png").exists())

    def test_plot_lines_break_at_session_boundaries_and_keep_single_point_markers(self):
        rows = session_rows([0, 10]) + session_rows([1000, 1010])
        data = plotter.load(write_run(self.root, "breaks", rows, meta=meta_for([0, 1000])))
        figure, ax = plt.subplots()
        try:
            plotter.plot_series(ax, data.df, plotter.numeric_column(data.df, "battery_pct"), color="blue")
            line = ax.lines[0]
            self.assertTrue(np.isnan(line.get_ydata()).any())
            self.assertEqual(line.get_marker(), ".")  # isolated samples remain visible
            self.assertEqual(np.isfinite(line.get_xdata()).sum(), len(data.df))
        finally:
            plt.close(figure)

    def test_timeline_shading_never_extends_through_restart_downtime(self):
        rows = session_rows([0, 10, 20], throttled=[1, 1, 1]) + session_rows([1000, 1010, 1020], throttled=[1, 1, 1])
        data = plotter.load(write_run(self.root, "shade", rows, meta=meta_for([0, 1000])))
        with captured_figure(plotter.timeline, data) as (figure, _):
            temp_axis = figure.axes[2]
            spans = [patch.get_x() for patch in temp_axis.patches]
            widths = [patch.get_width() for patch in temp_axis.patches]
            self.assertTrue(all(start + width <= 20 / 3600 + 1e-12 or start >= 1000 / 3600 - 1e-12
                                for start, width in zip(spans, widths)))
            self.assertTrue(all(width < 900 / 3600 for width in widths))
        self.assert_png(data.run / "timeline.png")

    def test_more_than_eight_runs_are_all_in_compare_output(self):
        runs = []
        for i in range(10):
            run = write_run(self.root, f"run-{i}", session_rows([0, 10, 20], battery=[90 - i, 89 - i, 88 - i]))
            runs.append(plotter.load(run))
        out = self.root / "compare.png"
        with captured_figure(plotter.compare, runs, out) as (figure, _):
            axis = figure.axes[0]
            self.assertEqual(len(axis.lines), 10)
            self.assertEqual(axis.get_legend_handles_labels()[1], [f"run-{i}" for i in range(10)])
            colors = {tuple(line.get_color()) if isinstance(line.get_color(), tuple) else line.get_color()
                      for line in axis.lines}
            self.assertGreaterEqual(len(colors), 10)
        self.assert_png(out)

    def test_more_than_eight_temperature_zones_are_all_plotted(self):
        extra = {f"temp_zone{i}_soc_thermal_c": [40 + i, 41 + i, 42 + i] for i in range(10)}
        run = write_run(self.root, "thermal", session_rows([0, 10, 20], **extra))
        data = plotter.load(run)
        with captured_figure(plotter.timeline, data) as (figure, _):
            temp_axis = figure.axes[2]
            self.assertEqual(len(temp_axis.lines), 10)
            self.assertEqual(len(temp_axis.get_legend_handles_labels()[1]), 10)
        self.assert_png(run / "timeline.png")

    def test_drain_chart_includes_zero_and_places_negative_annotations_below_bars(self):
        rows = (session_rows([0, 10], "walk", battery=[90, 89])
                + session_rows([20, 30], "stand", battery=[80, 80])
                + session_rows([40, 50], "sit", battery=[70, 71]))
        data = plotter.load(write_run(self.root, "drain-chart", rows))
        with captured_figure(plotter.breakdown, data) as (figure, _):
            axis = figure.axes[2]
            low, high = axis.get_ylim()
            self.assertLessEqual(low, 0)
            self.assertGreaterEqual(high, 0)
            negative = [text for text in axis.texts if "-360.0 %/h" in text.get_text()]
            self.assertEqual(len(negative), 1)
            self.assertEqual(negative[0].get_va(), "top")
            self.assertIn("runtime N/A", negative[0].get_text())
        text = plotter.summary(data, plotter.drain_by_activity(data.df))
        self.assertIn("| stand | 0.00 | 0.0 | N/A |", text)
        self.assertIn("| sit | 0.00 | -360.0 | N/A |", text)

    def test_embedded_matplotlib_minimum_matches_tick_labels_api(self):
        self.assertIn('"matplotlib>=3.9"', SCRIPT.read_text())


if __name__ == "__main__":
    unittest.main()
