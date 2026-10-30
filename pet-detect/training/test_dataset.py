from __future__ import annotations

import json
import tempfile
import unittest
import wave
from pathlib import Path

from dataset import (
    Recording,
    average_precision,
    best_f1_threshold,
    classification_metrics,
    dataset_fingerprint,
    discover_recordings,
    split_recordings,
)


def recording(label: int, session: str, name: str) -> Recording:
    return Recording(
        path=Path(name),
        relative_path=name,
        label=label,
        session=session,
        sha256=name,
    )


def write_wav(path: Path, *, frames: int = 1_600, sample_rate: int = 16_000) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(b"\0\0" * frames)


class SplitTests(unittest.TestCase):
    def setUp(self) -> None:
        self.recordings = [
            recording(0, "normal-room-a", "normal/a-1.wav"),
            recording(0, "normal-room-a", "normal/a-2.wav"),
            recording(0, "normal-room-b", "normal/b.wav"),
            recording(0, "normal-room-c", "normal/c.wav"),
            recording(1, "pet-room-a", "petting/a.wav"),
            recording(1, "pet-room-b", "petting/b.wav"),
            recording(1, "pet-room-c", "petting/c.wav"),
        ]

    def test_sessions_never_cross_the_split(self) -> None:
        split = split_recordings(self.recordings, validation_fraction=0.25, seed=42)

        train_sessions = {item.session for item in split.train}
        validation_sessions = {item.session for item in split.validation}
        self.assertFalse(train_sessions & validation_sessions)
        self.assertEqual({item.label for item in split.train}, {0, 1})
        self.assertEqual({item.label for item in split.validation}, {0, 1})

    def test_split_is_reproducible(self) -> None:
        first = split_recordings(self.recordings, validation_fraction=0.25, seed=17)
        second = split_recordings(self.recordings, validation_fraction=0.25, seed=17)

        self.assertEqual(first, second)

    def test_each_class_needs_two_sessions(self) -> None:
        too_small = [
            recording(0, "normal-a", "normal/a.wav"),
            recording(0, "normal-b", "normal/b.wav"),
            recording(1, "only-pet", "petting/a.wav"),
        ]

        with self.assertRaisesRegex(ValueError, "petting needs at least two"):
            split_recordings(too_small, validation_fraction=0.2, seed=42)


class ManifestTests(unittest.TestCase):
    def test_manifest_records_provenance_and_validates_audio(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_wav(root / "clips" / "pet.wav")
            entry = {
                "path": "clips/pet.wav",
                "label": "petting",
                "session": "kitchen-morning-01",
                "robot": "duck-a",
                "environment": "kitchen morning",
                "license": "CC-BY-4.0",
            }
            (root / "manifest.jsonl").write_text(
                json.dumps(entry) + "\n", encoding="utf-8"
            )

            recordings, explicit = discover_recordings(root)

            self.assertTrue(explicit)
            self.assertEqual(recordings[0].session, "kitchen-morning-01")
            self.assertEqual(len(recordings[0].sha256), 64)
            self.assertEqual(len(dataset_fingerprint(recordings)), 64)

    def test_manifest_path_cannot_escape_dataset(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entry = {"path": "../private.wav", "label": "normal", "session": "outside"}
            (root / "manifest.jsonl").write_text(
                json.dumps(entry) + "\n", encoding="utf-8"
            )

            with self.assertRaisesRegex(ValueError, "escapes the dataset"):
                discover_recordings(root)

    def test_manifest_rejects_aliases_of_the_same_recording(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_wav(root / "clip.wav")
            entries = [
                {"path": "clip.wav", "label": "normal", "session": "first"},
                {
                    "path": "nested/../clip.wav",
                    "label": "normal",
                    "session": "second",
                },
            ]
            (root / "manifest.jsonl").write_text(
                "".join(json.dumps(entry) + "\n" for entry in entries),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "appears more than once"):
                discover_recordings(root)

    def test_manifest_allows_audio_the_rust_extractor_can_resample(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_wav(root / "wrong-rate.wav", sample_rate=48_000)
            entry = {
                "path": "wrong-rate.wav",
                "label": "normal",
                "session": "wrong-rate",
            }
            (root / "manifest.jsonl").write_text(
                json.dumps(entry) + "\n", encoding="utf-8"
            )

            recordings, _ = discover_recordings(root)

            self.assertEqual(recordings[0].relative_path, "wrong-rate.wav")

    def test_manifest_rejects_an_empty_recording(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "empty.wav").write_bytes(b"")
            entry = {
                "path": "empty.wav",
                "label": "normal",
                "session": "empty",
            }
            (root / "manifest.jsonl").write_text(
                json.dumps(entry) + "\n", encoding="utf-8"
            )

            with self.assertRaisesRegex(ValueError, "RIFF/WAVE"):
                discover_recordings(root)

    def test_mixed_label_session_stays_on_one_side(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_wav(root / "normal.wav")
            write_wav(root / "pet.wav")
            write_wav(root / "other-normal.wav")
            write_wav(root / "other-pet.wav")
            entries = [
                {"path": "normal.wav", "label": "normal", "session": "same"},
                {"path": "pet.wav", "label": "petting", "session": "same"},
                {
                    "path": "other-normal.wav",
                    "label": "normal",
                    "session": "normal-only",
                },
                {
                    "path": "other-pet.wav",
                    "label": "petting",
                    "session": "petting-only",
                },
            ]
            (root / "manifest.jsonl").write_text(
                "".join(json.dumps(entry) + "\n" for entry in entries),
                encoding="utf-8",
            )

            recordings, _ = discover_recordings(root)
            split = split_recordings(recordings, validation_fraction=0.34, seed=42)

            train_paths = {item.relative_path for item in split.train}
            validation_paths = {item.relative_path for item in split.validation}
            self.assertTrue(
                {"normal.wav", "pet.wav"} <= train_paths
                or {"normal.wav", "pet.wav"} <= validation_paths
            )


class MetricsTests(unittest.TestCase):
    def test_best_threshold_prefers_high_precision_tie(self) -> None:
        metrics = best_f1_threshold(
            [0, 0, 1, 1],
            [0.10, 0.80, 0.80, 0.90],
        )

        self.assertEqual(metrics.threshold, 0.8)
        self.assertAlmostEqual(metrics.f1, 0.8)
        self.assertEqual(metrics.false_positive, 1)
        self.assertEqual(metrics.false_negative, 0)

    def test_average_precision_is_one_for_a_perfect_ranking(self) -> None:
        self.assertEqual(average_precision([0, 1, 0, 1], [0.1, 0.8, 0.2, 0.9]), 1.0)

    def test_threshold_sweep_matches_brute_force_with_ties_and_boundaries(self) -> None:
        labels = [0, 1, 0, 1, 1, 0]
        scores = [-0.1, 0.0, 0.4, 0.4, 1.0, 1.1]
        candidates = {0.0, 1.0, *scores}
        expected = max(
            (
                classification_metrics(labels, scores, threshold=threshold)
                for threshold in candidates
            ),
            key=lambda item: (item.f1, item.precision, item.threshold),
        )

        self.assertEqual(best_f1_threshold(labels, scores), expected)


if __name__ == "__main__":
    unittest.main()
