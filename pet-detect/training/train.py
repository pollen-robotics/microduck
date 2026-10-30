#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "torch>=2.2",
#   "numpy",
#   "onnx",
#   "onnxruntime",
#   "onnxscript",
# ]
# ///
"""Train and audit the petting classifier.

Features come from the Rust runtime implementation. Recordings are split before
feature extraction so adjacent windows from one capture can never appear on both
sides of validation.

Run from the repository root:
    cargo build --release -p pet-detect --bin pet-features
    uv run pet-detect/training/train.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path

# Required by deterministic CUDA matrix multiplication. It is harmless on CPU.
os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")

import numpy as np
import onnx
import onnxruntime
import torch
from dataset import (
    LABEL_NAMES,
    Recording,
    average_precision,
    best_f1_threshold,
    classification_metrics,
    dataset_fingerprint,
    discover_recordings,
    split_recordings,
)
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

PACKAGE = Path(__file__).resolve().parent.parent
WORKSPACE = PACKAGE.parent
DEFAULT_DATA = PACKAGE / "data"
DEFAULT_MODEL = PACKAGE / "models" / "pet_detect.onnx"
DEFAULT_REPORT = PACKAGE / "models" / "pet_detect.report.json"
FEATURES_BIN = (
    WORKSPACE
    / "target"
    / "release"
    / ("pet-features.exe" if os.name == "nt" else "pet-features")
)

# Must match src/lib.rs.
N_MELS = 40
WINDOW_FRAMES = 100
BLOCK_BYTES = N_MELS * WINDOW_FRAMES * 4  # f32 LE


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--epochs", type=int, default=1_000)
    parser.add_argument("--patience", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.epochs < 1:
        parser.error("--epochs must be positive")
    if args.patience < 1:
        parser.error("--patience must be positive")
    if args.batch_size < 1:
        parser.error("--batch-size must be positive")
    return args


def extract_blocks(wav: Path) -> np.ndarray:
    """Run the Rust feature extractor and return [N, mel, time] f32."""
    result = subprocess.run(
        [str(FEATURES_BIN), str(wav)],
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.decode(errors="replace").strip()
        raise RuntimeError(
            f"{wav}: feature extraction failed: {detail or result.returncode}"
        )
    raw = result.stdout
    if len(raw) % BLOCK_BYTES != 0:
        raise RuntimeError(
            f"{wav}: feature byte count {len(raw)} is not divisible by {BLOCK_BYTES}"
        )
    count = len(raw) // BLOCK_BYTES
    if count == 0:
        raise RuntimeError(
            f"{wav}: recording is too short to produce one feature window"
        )
    return (
        np.frombuffer(raw, dtype=np.float32)
        .reshape(count, N_MELS, WINDOW_FRAMES)
        .copy()
    )


def load_recordings(
    recordings: tuple[Recording, ...],
    *,
    name: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    features: list[np.ndarray] = []
    labels: list[np.ndarray] = []
    sessions: list[np.ndarray] = []
    for recording in recordings:
        blocks = extract_blocks(recording.path)
        features.append(blocks[:, None, :, :])
        labels.append(np.full(blocks.shape[0], recording.label, dtype=np.int64))
        sessions.append(np.full(blocks.shape[0], recording.session, dtype=object))
        print(
            f"  {name}: {recording.relative_path}: {blocks.shape[0]} windows "
            f"({LABEL_NAMES[recording.label]}, session={recording.session})"
        )
    return (
        np.concatenate(features),
        np.concatenate(labels),
        np.concatenate(sessions),
    )


class TinyAudioCNN(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, padding=1),
            nn.BatchNorm2d(8),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(8, 16, kernel_size=3, padding=1),
            nn.BatchNorm2d(16),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(),
            nn.Linear(16, 2),
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.net(inputs)


class ProbabilityModel(nn.Module):
    """Keep the deployed ONNX contract as probabilities while training on logits."""

    def __init__(self, model: TinyAudioCNN) -> None:
        super().__init__()
        self.model = model

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return torch.softmax(self.model(inputs), dim=1)


def deterministic(seed: int) -> None:
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True)


def validation_result(
    model: TinyAudioCNN,
    features: torch.Tensor,
    labels: torch.Tensor,
    class_weights: torch.Tensor,
) -> tuple[float, np.ndarray]:
    model.eval()
    with torch.no_grad():
        logits = model(features)
        loss = nn.functional.cross_entropy(logits, labels, weight=class_weights)
        scores = torch.softmax(logits, dim=1)[:, 1].cpu().numpy()
    return float(loss.item()), scores


def train(
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_validation: np.ndarray,
    y_validation: np.ndarray,
    *,
    epochs: int,
    patience: int,
    batch_size: int,
    seed: int,
) -> tuple[TinyAudioCNN, dict[str, object], np.ndarray]:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = TinyAudioCNN().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

    counts = np.bincount(y_train, minlength=2)
    if np.any(counts == 0):
        raise RuntimeError("training split must contain normal and petting windows")
    weights = y_train.size / (2.0 * counts)
    class_weights = torch.tensor(weights, dtype=torch.float32, device=device)

    generator = torch.Generator().manual_seed(seed)
    loader = DataLoader(
        TensorDataset(torch.from_numpy(x_train), torch.from_numpy(y_train)),
        batch_size=batch_size,
        shuffle=True,
        generator=generator,
    )
    validation_features = torch.from_numpy(x_validation).to(device)
    validation_labels = torch.from_numpy(y_validation).to(device)

    best_loss = float("inf")
    best_epoch = 0
    best_state: dict[str, torch.Tensor] | None = None
    stale_epochs = 0
    epochs_run = 0

    for epoch in range(1, epochs + 1):
        model.train()
        total = 0.0
        for features, labels in loader:
            features, labels = features.to(device), labels.to(device)
            optimizer.zero_grad()
            loss = nn.functional.cross_entropy(
                model(features), labels, weight=class_weights
            )
            loss.backward()
            optimizer.step()
            total += loss.item() * features.shape[0]

        validation_loss, validation_scores = validation_result(
            model,
            validation_features,
            validation_labels,
            class_weights,
        )
        metrics = classification_metrics(
            y_validation.tolist(), validation_scores.tolist(), threshold=0.5
        )
        epochs_run = epoch
        print(
            f"epoch {epoch:04d} train_loss={total / len(loader.dataset):.4f} "
            f"val_loss={validation_loss:.4f} val_f1@0.5={metrics.f1:.3f}"
        )

        if validation_loss < best_loss - 1e-6:
            best_loss = validation_loss
            best_epoch = epoch
            best_state = {
                name: value.detach().cpu().clone()
                for name, value in model.state_dict().items()
            }
            stale_epochs = 0
        else:
            stale_epochs += 1
            if stale_epochs >= patience:
                print(
                    f"early stop: validation loss did not improve for {patience} epochs"
                )
                break

    assert best_state is not None
    model.load_state_dict(best_state)
    model.to(device)
    final_loss, final_scores = validation_result(
        model,
        validation_features,
        validation_labels,
        class_weights,
    )
    return (
        model,
        {
            "device": str(device),
            "epochs_requested": epochs,
            "epochs_run": epochs_run,
            "best_epoch": best_epoch,
            "best_validation_loss": final_loss,
            "patience": patience,
            "batch_size": batch_size,
            "class_windows": {
                LABEL_NAMES[label]: int(count) for label, count in enumerate(counts)
            },
        },
        final_scores,
    )


def export_and_check(
    model: TinyAudioCNN,
    output: Path,
    sample: np.ndarray,
) -> dict[str, object]:
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep the wrapper itself in evaluation mode. Exporters restore the wrapper's
    # original state, which would otherwise put its BatchNorm child back into train mode
    # before the PyTorch/ONNX parity comparison below.
    exported = ProbabilityModel(model.cpu()).eval()
    dummy = torch.zeros(1, 1, N_MELS, WINDOW_FRAMES)
    torch.onnx.export(
        exported,
        dummy,
        output,
        input_names=["input"],
        output_names=["probs"],
        dynamic_axes={"input": {0: "batch"}, "probs": {0: "batch"}},
        opset_version=18,
    )

    sidecar = output.with_suffix(".onnx.data")
    if sidecar.exists():
        onnx_model = onnx.load(str(output), load_external_data=True)
        onnx.save_model(onnx_model, str(output), save_as_external_data=False)
        sidecar.unlink()
    onnx.checker.check_model(onnx.load(str(output)))

    sample = sample[: min(32, sample.shape[0])]
    with torch.no_grad():
        torch_probabilities = exported(torch.from_numpy(sample)).numpy()
    session = onnxruntime.InferenceSession(
        str(output), providers=["CPUExecutionProvider"]
    )
    onnx_probabilities = session.run(None, {"input": sample})[0]
    difference = np.abs(torch_probabilities - onnx_probabilities)
    maximum = float(difference.max())
    mean = float(difference.mean())
    if not np.allclose(torch_probabilities, onnx_probabilities, rtol=1e-5, atol=1e-6):
        raise RuntimeError(
            f"ONNX parity check failed: max_abs={maximum:.8g}, mean_abs={mean:.8g}"
        )
    return {"max_absolute_error": maximum, "mean_absolute_error": mean}


def source_revision() -> dict[str, object]:
    def git(*args: str) -> str | None:
        result = subprocess.run(
            ["git", *args],
            cwd=WORKSPACE,
            text=True,
            capture_output=True,
            check=False,
        )
        return result.stdout.strip() if result.returncode == 0 else None

    status = git("status", "--porcelain", "--untracked-files=no")
    return {"commit": git("rev-parse", "HEAD"), "dirty": bool(status)}


def session_report(
    labels: np.ndarray,
    scores: np.ndarray,
    sessions: np.ndarray,
    *,
    threshold: float,
) -> list[dict[str, object]]:
    report: list[dict[str, object]] = []
    for session in sorted(set(sessions.tolist())):
        selected = sessions == session
        session_labels = labels[selected]
        session_scores = scores[selected]
        correct = (session_scores >= threshold) == session_labels
        report.append(
            {
                "session": session,
                "windows": int(selected.sum()),
                "class_windows": {
                    LABEL_NAMES[label]: int((session_labels == label).sum())
                    for label in LABEL_NAMES
                },
                "mean_petting_probability": float(session_scores.mean()),
                "min_petting_probability": float(session_scores.min()),
                "max_petting_probability": float(session_scores.max()),
                "accuracy": float(correct.mean()),
            }
        )
    return report


def main() -> None:
    args = arguments()
    if not FEATURES_BIN.exists():
        raise SystemExit(
            f"missing {FEATURES_BIN} — run "
            "`cargo build --release -p pet-detect --bin pet-features` first"
        )
    deterministic(args.seed)

    recordings, has_manifest = discover_recordings(args.data)
    if not has_manifest:
        print(
            "warning: no data/manifest.jsonl; treating each WAV as one session. "
            "Add a manifest to record robot, environment, license, and capture grouping."
        )
    split = split_recordings(
        recordings,
        validation_fraction=args.validation_fraction,
        seed=args.seed,
    )
    print(
        f"recordings: train={len(split.train)} validation={len(split.validation)} "
        f"seed={args.seed}"
    )
    x_train, y_train, _ = load_recordings(split.train, name="train")
    x_validation, y_validation, validation_sessions = load_recordings(
        split.validation, name="validation"
    )
    print(f"windows: train={len(y_train)} validation={len(y_validation)}")

    model, training_report, validation_scores = train(
        x_train,
        y_train,
        x_validation,
        y_validation,
        epochs=args.epochs,
        patience=args.patience,
        batch_size=args.batch_size,
        seed=args.seed,
    )
    calibrated = best_f1_threshold(y_validation.tolist(), validation_scores.tolist())
    parity = export_and_check(model, args.output, x_validation)

    model_hash = hashlib.sha256(args.output.read_bytes()).hexdigest()
    report = {
        "schema_version": 1,
        # timezone.utc keeps the script compatible with its declared Python 3.10 floor.
        "created_at": datetime.now(timezone.utc).isoformat(),  # noqa: UP017
        "source": source_revision(),
        "environment": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "torch": torch.__version__,
            "torch_cuda": torch.version.cuda,
            "onnx": onnx.__version__,
            "onnxruntime": onnxruntime.__version__,
            "feature_extractor_sha256": hashlib.sha256(
                FEATURES_BIN.read_bytes()
            ).hexdigest(),
        },
        "dataset": {
            "explicit_manifest": has_manifest,
            "fingerprint": dataset_fingerprint(recordings),
            "recordings": [recording.provenance() for recording in recordings],
        },
        "split": {
            "seed": args.seed,
            "validation_fraction": args.validation_fraction,
            "train_sessions": sorted({item.session for item in split.train}),
            "validation_sessions": sorted({item.session for item in split.validation}),
            "train_windows": int(y_train.size),
            "validation_windows": int(y_validation.size),
        },
        "training": training_report,
        "validation": {
            "average_precision": average_precision(
                y_validation.tolist(), validation_scores.tolist()
            ),
            "calibrated": calibrated.to_dict(),
            "at_0_5": classification_metrics(
                y_validation.tolist(), validation_scores.tolist(), threshold=0.5
            ).to_dict(),
            "at_runtime_enter_0_95": classification_metrics(
                y_validation.tolist(), validation_scores.tolist(), threshold=0.95
            ).to_dict(),
            "at_runtime_exit_0_85": classification_metrics(
                y_validation.tolist(), validation_scores.tolist(), threshold=0.85
            ).to_dict(),
            "sessions": session_report(
                y_validation,
                validation_scores,
                validation_sessions,
                threshold=calibrated.threshold,
            ),
        },
        "export": {
            "path": args.output.name,
            "sha256": model_hash,
            "onnx_parity": parity,
        },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"wrote {args.output} and {args.report}; "
        f"validation f1={calibrated.f1:.3f} threshold={calibrated.threshold:.4f}"
    )


if __name__ == "__main__":
    main()
