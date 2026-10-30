"""Dataset discovery, validation, splitting, and metrics for pet detection.

This module intentionally uses only the Python standard library.  CI can test the
parts that protect evaluation integrity without installing the training stack.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import random
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

LABELS = {"normal": 0, "petting": 1}
LABEL_NAMES = {value: key for key, value in LABELS.items()}
MANIFEST_FIELDS = {
    "path",
    "label",
    "session",
    "robot",
    "environment",
    "license",
}


@dataclass(frozen=True)
class Recording:
    path: Path
    relative_path: str
    label: int
    session: str
    robot: str | None = None
    environment: str | None = None
    license: str | None = None
    sha256: str = ""

    def provenance(self) -> dict[str, object]:
        result = asdict(self)
        result.pop("path")
        result["label"] = LABEL_NAMES[self.label]
        return result


@dataclass(frozen=True)
class DatasetSplit:
    train: tuple[Recording, ...]
    validation: tuple[Recording, ...]


@dataclass(frozen=True)
class Metrics:
    threshold: float
    accuracy: float
    precision: float
    recall: float
    f1: float
    true_positive: int
    true_negative: int
    false_positive: int
    false_negative: int

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _text(entry: dict[str, object], field: str, *, required: bool) -> str | None:
    value = entry.get(field)
    if value is None and not required:
        return None
    if not isinstance(value, str) or not value.strip():
        qualifier = "required" if required else "must be a non-empty string"
        raise ValueError(f"manifest field {field!r} is {qualifier}")
    return value.strip()


def _validate_wav(path: Path) -> None:
    # Parse only the RIFF envelope here. The Rust extractor remains the authority on
    # sample formats, rates, and channel layouts so this check cannot reject audio it can
    # resample (including float WAV, which Python's wave module did not support until 3.12).
    file_size = path.stat().st_size
    with path.open("rb") as source:
        header = source.read(12)
        if len(header) != 12 or header[:4] != b"RIFF" or header[8:] != b"WAVE":
            raise ValueError(f"{path}: expected a RIFF/WAVE recording")

        has_format = False
        data_size = 0
        while source.tell() + 8 <= file_size:
            chunk_header = source.read(8)
            chunk_id = chunk_header[:4]
            chunk_size = int.from_bytes(chunk_header[4:], "little")
            chunk_start = source.tell()
            chunk_end = chunk_start + chunk_size
            padded_end = chunk_end + (chunk_size % 2)
            if padded_end > file_size:
                raise ValueError(f"{path}: truncated {chunk_id!r} WAV chunk")
            if chunk_id == b"fmt ":
                has_format = chunk_size >= 16
            elif chunk_id == b"data":
                data_size += chunk_size
            source.seek(padded_end)

    if not has_format:
        raise ValueError(f"{path}: WAV has no valid format chunk")
    if data_size == 0:
        raise ValueError(f"{path}: WAV has no samples")


def _recording_from_entry(
    root: Path,
    entry: dict[str, object],
    *,
    line: int,
) -> Recording:
    unknown = set(entry) - MANIFEST_FIELDS
    if unknown:
        names = ", ".join(sorted(unknown))
        raise ValueError(f"manifest line {line}: unknown field(s): {names}")

    relative_text = _text(entry, "path", required=True)
    label_name = _text(entry, "label", required=True)
    session = _text(entry, "session", required=True)
    assert relative_text is not None and label_name is not None and session is not None
    if label_name not in LABELS:
        raise ValueError(
            f"manifest line {line}: label must be one of {sorted(LABELS)}, got {label_name!r}"
        )

    relative = Path(relative_text)
    if relative.is_absolute():
        raise ValueError(
            f"manifest line {line}: path must be relative: {relative_text!r}"
        )
    root_resolved = root.resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root_resolved):
        raise ValueError(
            f"manifest line {line}: path escapes the dataset: {relative_text!r}"
        )
    if path.suffix.lower() != ".wav":
        raise ValueError(
            f"manifest line {line}: recording must be a .wav: {relative_text!r}"
        )
    if not path.is_file():
        raise ValueError(
            f"manifest line {line}: recording does not exist: {relative_text!r}"
        )
    _validate_wav(path)

    return Recording(
        path=path,
        relative_path=relative.as_posix(),
        label=LABELS[label_name],
        session=session,
        robot=_text(entry, "robot", required=False),
        environment=_text(entry, "environment", required=False),
        license=_text(entry, "license", required=False),
        sha256=_sha256(path),
    )


def _manifest_recordings(root: Path, manifest: Path) -> list[Recording]:
    recordings: list[Recording] = []
    for line_number, raw in enumerate(
        manifest.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not raw.strip():
            continue
        try:
            entry = json.loads(raw)
        except json.JSONDecodeError as error:
            raise ValueError(
                f"manifest line {line_number}: invalid JSON: {error.msg}"
            ) from error
        if not isinstance(entry, dict):
            # A malformed manifest is a data validation error, regardless of which
            # JSON value type happened to appear on this line.
            raise ValueError(  # noqa: TRY004
                f"manifest line {line_number}: expected a JSON object"
            )
        recordings.append(_recording_from_entry(root, entry, line=line_number))
    if not recordings:
        raise ValueError(f"{manifest}: manifest contains no recordings")
    return recordings


def _legacy_recordings(root: Path) -> list[Recording]:
    recordings: list[Recording] = []
    for label_name, label in LABELS.items():
        for path in sorted((root / label_name).glob("*.wav")):
            _validate_wav(path)
            relative = path.relative_to(root).as_posix()
            recordings.append(
                Recording(
                    path=path.resolve(),
                    relative_path=relative,
                    label=label,
                    # A recording is the smallest safe grouping when old data has no manifest.
                    session=relative.removesuffix(".wav"),
                    sha256=_sha256(path),
                )
            )
    if not recordings:
        raise ValueError(
            f"no recordings under {root}; add manifest.jsonl or normal/*.wav and petting/*.wav"
        )
    return recordings


def discover_recordings(root: Path) -> tuple[list[Recording], bool]:
    """Return validated recordings and whether an explicit manifest described them."""
    root = root.resolve()
    manifest = root / "manifest.jsonl"
    recordings = (
        _manifest_recordings(root, manifest)
        if manifest.is_file()
        else _legacy_recordings(root)
    )

    paths: set[Path] = set()
    for recording in recordings:
        # Manifest spellings such as `clip.wav` and `nested/../clip.wav` must not
        # let one capture acquire two session IDs and cross the train/validation split.
        if recording.path in paths:
            raise ValueError(
                f"recording appears more than once: {recording.relative_path}"
            )
        paths.add(recording.path)
    return recordings, manifest.is_file()


def split_recordings(
    recordings: Sequence[Recording],
    *,
    validation_fraction: float,
    seed: int,
) -> DatasetSplit:
    """Stratify by label while keeping every session on exactly one side."""
    if not 0.0 < validation_fraction < 1.0:
        raise ValueError("validation_fraction must be between 0 and 1")

    session_members: dict[str, list[Recording]] = {}
    for recording in recordings:
        session_members.setdefault(recording.session, []).append(recording)

    sessions_by_label = {
        label: {
            session
            for session, members in session_members.items()
            if any(member.label == label for member in members)
        }
        for label in LABEL_NAMES
    }
    for label, sessions in sessions_by_label.items():
        if len(sessions) < 2:
            raise ValueError(
                f"{LABEL_NAMES[label]} needs at least two independent sessions so validation "
                "does not reuse training audio"
            )

    session_names = sorted(session_members)
    target_count = min(
        len(session_names) - 1,
        max(1, math.ceil(len(session_names) * validation_fraction)),
    )
    rng = random.Random(seed)
    shuffled = session_names.copy()
    rng.shuffle(shuffled)
    tie_break = {session: index for index, session in enumerate(shuffled)}
    totals = {
        label: sum(recording.label == label for recording in recordings)
        for label in LABEL_NAMES
    }

    def valid(selection: set[str]) -> bool:
        remaining = set(session_names) - selection
        return all(
            selection & sessions_by_label[label]
            and remaining & sessions_by_label[label]
            for label in LABEL_NAMES
        )

    def score(selection: set[str]) -> tuple[float, tuple[int, ...]]:
        distance = 0.0
        for label in LABEL_NAMES:
            selected = sum(
                recording.label == label
                for session in selection
                for recording in session_members[session]
            )
            distance += (selected / totals[label] - validation_fraction) ** 2
        return distance, tuple(sorted(tie_break[session] for session in selection))

    # One mixed-label session can seed validation alone. Otherwise two sessions are the
    # minimum needed to put both classes in validation while leaving both in training.
    candidates: list[set[str]] = []
    for size in (1, 2):
        for names in itertools.combinations(session_names, size):
            selection = set(names)
            if valid(selection):
                candidates.append(selection)
        if candidates:
            break
    if not candidates:
        raise ValueError(
            "could not construct a validation split containing both labels"
        )
    validation_sessions = min(candidates, key=score)

    while len(validation_sessions) < target_count:
        additions = [
            validation_sessions | {session}
            for session in session_names
            if session not in validation_sessions
            and valid(validation_sessions | {session})
        ]
        if not additions:
            break
        validation_sessions = min(additions, key=score)

    train = tuple(
        sorted(
            (
                recording
                for recording in recordings
                if recording.session not in validation_sessions
            ),
            key=lambda recording: recording.relative_path,
        )
    )
    validation = tuple(
        sorted(
            (
                recording
                for recording in recordings
                if recording.session in validation_sessions
            ),
            key=lambda recording: recording.relative_path,
        )
    )
    if {item.session for item in train} & {item.session for item in validation}:
        raise AssertionError("a session reached both sides of the split")
    return DatasetSplit(train=train, validation=validation)


def classification_metrics(
    labels: Sequence[int], scores: Sequence[float], *, threshold: float
) -> Metrics:
    if len(labels) != len(scores) or not labels:
        raise ValueError("labels and scores must be non-empty and have equal length")
    if any(label not in LABEL_NAMES for label in labels):
        raise ValueError("labels must be zero or one")

    predicted = [int(score >= threshold) for score in scores]
    tp = sum(
        label == 1 and prediction == 1 for label, prediction in zip(labels, predicted)
    )
    tn = sum(
        label == 0 and prediction == 0 for label, prediction in zip(labels, predicted)
    )
    fp = sum(
        label == 0 and prediction == 1 for label, prediction in zip(labels, predicted)
    )
    fn = sum(
        label == 1 and prediction == 0 for label, prediction in zip(labels, predicted)
    )
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    return Metrics(
        threshold=threshold,
        accuracy=(tp + tn) / len(labels),
        precision=precision,
        recall=recall,
        f1=f1,
        true_positive=tp,
        true_negative=tn,
        false_positive=fp,
        false_negative=fn,
    )


def best_f1_threshold(labels: Sequence[int], scores: Sequence[float]) -> Metrics:
    """Pick the most conservative threshold among equal-F1 operating points."""
    if len(labels) != len(scores) or not labels:
        raise ValueError("labels and scores must be non-empty and have equal length")
    if any(label not in LABEL_NAMES for label in labels):
        raise ValueError("labels must be zero or one")

    ranked = sorted(
        ((float(score), label) for score, label in zip(scores, labels)),
        reverse=True,
    )
    candidates = sorted({0.0, 1.0, *(score for score, _ in ranked)}, reverse=True)
    positives = sum(labels)
    negatives = len(labels) - positives
    true_positive = 0
    false_positive = 0
    index = 0
    best: Metrics | None = None

    # Descending thresholds only add predictions. Updating the confusion counts once per
    # window avoids rescanning the full validation set for every distinct score.
    for threshold in candidates:
        while index < len(ranked) and ranked[index][0] >= threshold:
            if ranked[index][1] == 1:
                true_positive += 1
            else:
                false_positive += 1
            index += 1
        false_negative = positives - true_positive
        true_negative = negatives - false_positive
        precision = (
            true_positive / (true_positive + false_positive)
            if true_positive + false_positive
            else 0.0
        )
        recall = true_positive / positives if positives else 0.0
        f1 = (
            2.0 * precision * recall / (precision + recall)
            if precision + recall
            else 0.0
        )
        current = Metrics(
            threshold=threshold,
            accuracy=(true_positive + true_negative) / len(labels),
            precision=precision,
            recall=recall,
            f1=f1,
            true_positive=true_positive,
            true_negative=true_negative,
            false_positive=false_positive,
            false_negative=false_negative,
        )
        if best is None or (current.f1, current.precision, current.threshold) > (
            best.f1,
            best.precision,
            best.threshold,
        ):
            best = current

    assert best is not None
    return best


def average_precision(labels: Sequence[int], scores: Sequence[float]) -> float:
    """Area under the stepwise precision-recall curve (average precision)."""
    if len(labels) != len(scores) or not labels:
        raise ValueError("labels and scores must be non-empty and have equal length")
    if any(label not in LABEL_NAMES for label in labels):
        raise ValueError("labels must be zero or one")
    positives = sum(label == 1 for label in labels)
    if positives == 0:
        raise ValueError("average precision needs at least one positive label")
    ranked = sorted(zip(scores, labels), key=lambda pair: pair[0], reverse=True)
    true_positive = 0
    seen = 0
    area = 0.0
    index = 0
    while index < len(ranked):
        score = ranked[index][0]
        group_positive = 0
        while index < len(ranked) and ranked[index][0] == score:
            group_positive += ranked[index][1] == 1
            seen += 1
            index += 1
        true_positive += group_positive
        area += (group_positive / positives) * (true_positive / seen)
    return area


def dataset_fingerprint(recordings: Iterable[Recording]) -> str:
    rows = sorted(
        (recording.provenance() for recording in recordings),
        key=lambda row: str(row["relative_path"]),
    )
    encoded = json.dumps(rows, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()
