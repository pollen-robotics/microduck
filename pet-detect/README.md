# pet-detect

A tiny audio classifier that hears when the robot's head is being scratched (the onboard
mic sits there). ~20 KB CNN over 40-band log-mel windows, sub-millisecond inference.

Ported from `apirrone/microduck_pet_detect`; the arecord worker and the ambient sound
sentry from the prototype's `pet_worker.rs` live here too, so everything that listens to
the mic is one crate. `robotd` runs the worker (see `[audio]` in `deploy/robotd.toml`) and
coos when petting starts.

- **Library**: `PettingDetector` (streaming, hysteresis) and `worker::PetHandle`
  (arecord subprocess + event channels).
- **`pet-detect`** (binary): pipe `arecord -D plughw:aic3104,0 -f S16_LE -r 16000 -c 1 -t raw`
  into it and watch the probability while you scratch the head.
- **`pet-features`** (binary): dumps log-mel features of a WAV — the training side of the
  train/infer parity contract.
- **`models/pet_detect.onnx`**: the trained model, shipped in the release as
  `models/pet_detect.onnx`.

## Retraining

`training/train.py` (PyTorch) extracts features THROUGH the `pet-features` binary — the
same code path the robot runs, so there is no train/infer drift. Record data on the robot:

```bash
arecord -D plughw:aic3104,0 -f S16_LE -r 16000 -c 1 -d 30 /tmp/petting_01.wav
```

Record several independent sessions for each class. The trainer splits whole sessions, not the
overlapping one-second windows extracted from them: putting adjacent windows on opposite sides
would make validation measure memorisation. [`data/README.md`](data/README.md) defines the
recommended manifest, provenance fields, privacy checks, and hard negatives. The legacy
`data/petting/*.wav` and `data/normal/*.wav` layout remains accepted, with each WAV treated as one
session. Recordings are ignored by Git.

Then run:

```bash
cargo build --release -p pet-detect --bin pet-features
uv run pet-detect/training/train.py
```

Training is deterministic for a seed, balances the two classes, stops on held-out loss, restores
the best epoch, and writes both `models/pet_detect.onnx` and `models/pet_detect.report.json`. The
report records every input hash, the session split, per-session results, the F1-calibrated operating
point, performance at the shipped 0.95 enter threshold, and PyTorch-to-ONNX parity.

Review the report before changing the model or thresholds. A model update should include the report
and explain any session where performance regressed; dataset recordings belong in a separately
versioned dataset, not this repository.
