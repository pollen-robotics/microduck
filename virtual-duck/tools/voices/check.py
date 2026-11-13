"""check.py <voice> [--out DIR] — renders one voice's whole bank and measures it.

Every tag × variant in the low, middle and high registers, plus the song, is written as WAV to
DIR (default: target/duck-voices/<voice> at the repository root) with `report.txt` and
`metrics.json`, and checked two ways:

- each sound against the comfort targets, `comfort.TARGETS`;
- the loudness of each through a phone speaker, relative to the median hello, question and chirp
  of its register, against `comfort.LOUDNESS_OFFSETS`, within ±2 dB — so that a talk-back run of
  chirps, pecks and coos ending in a question comes out even.

A voice is a module here with RATE, STYLE, TAGS, render(tag, variant, pitch) and
sing(chorale, pitch, seconds). Exits non-zero if anything is outside its target.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import wave
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import comfort  # noqa: E402
import score  # noqa: E402

REGISTERS = (("low", 0.8), ("mid", 1.0), ("high", 1.25))
# The verse the app ships: render_voices.py says why it is no longer.
SONG_S = 20.0


def load_voice(name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    voice = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voice)
    assert voice.RATE == comfort.RATE, f"{name} renders at {voice.RATE} Hz, the measures assume {comfort.RATE}"
    return voice


def as_written(x) -> np.ndarray:
    """The sound as a 16-bit file will hold it."""
    return (np.clip(np.asarray(x, dtype=np.float64), -1, 1) * 32767).astype("<i2")


def write_wav(path: Path, samples: np.ndarray, rate: int) -> None:
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(samples.tobytes())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("voice")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    voice = load_voice(args.voice)
    out = args.out or HERE.parents[2] / "target" / "duck-voices" / args.voice
    (out / "bank").mkdir(parents=True, exist_ok=True)

    report, measured, loud = [], {}, {}
    for reg, pitch in REGISTERS:
        for tag, n in voice.TAGS.items():
            for v in range(n):
                name = f"{reg}_{tag}_{v}"
                raw = voice.render(tag, v, pitch)
                q = as_written(raw)
                write_wav(out / "bank" / f"{name}.wav", q, voice.RATE)
                x = np.clip(np.asarray(raw, dtype=np.float64), -1, 1)
                m = comfort.measure(x)
                measured[name] = m
                loud.setdefault(reg, []).append((name, tag, comfort.phone_loudness(q.astype(np.float64) / 32768)))
                bad = comfort.problems(m)
                report.append(
                    f"{name:22s} {m['seconds']:5.2f}s pk{m['peak_db']:6.1f} rms{m['rms_db']:6.1f} sharp{m['sharpness']:5.2f} "
                    f"rumble{m['rumble']:5.2f} rough{m['roughness']:5.2f} att{m['attack_ms']:5.1f} cen{m['centroid']:5d} "
                    f"f0{m.get('pitch_hz', 0):5d}  {'OK' if not bad else 'BAD: ' + '; '.join(bad)}"
                )
    song = np.clip(np.asarray(voice.sing(score.chorale(), 1.0, SONG_S), dtype=np.float64), -1, 1)
    write_wav(out / "bank" / "song_mid.wav", as_written(song), voice.RATE)
    m = comfort.measure(song)
    measured["song_mid"] = m
    song_ok = not comfort.problems({k: v for k, v in m.items() if k in ("peak_db", "sharpness", "rumble", "clicks")})
    report.append(f"{'song_mid':22s} {m['seconds']:5.2f}s pk{m['peak_db']:6.1f} rms{m['rms_db']:6.1f} "
                  f"sharp{m['sharpness']:5.2f} rumble{m['rumble']:5.2f}  {'OK' if song_ok else 'check'}")
    (out / "metrics.json").write_text(json.dumps(measured, indent=1))
    (out / "report.txt").write_text("\n".join(report) + "\n")
    bad = sum(1 for line in report if "BAD" in line) + (not song_ok)
    print(f"{len(report)} sounds rendered, {bad} outside the comfort targets; report in {out / 'report.txt'}")

    misses, refs = 0, {}
    for reg, items in loud.items():
        refs[reg] = float(np.median([l for _, t, l in items if t in ("greet", "inquire", "chirp")]))
        for name, tag, level in items:
            want = comfort.LOUDNESS_OFFSETS.get(tag, 0)
            rel = level - refs[reg]
            if abs(rel - want) > 2.0:
                misses += 1
                print(f"{name:20s} {rel:+5.1f} dB (want {want:+d} ±2)")
    lows, highs = (float(np.median([l for *_, l in loud[r]])) for r in ("low", "high"))
    print(f"registers: low {lows - refs['mid']:+.1f} dB, high {highs - refs['mid']:+.1f} dB vs mid (want within ±3)")
    print(f"{misses} sounds outside their loudness target; mid reference {refs['mid']:.1f} dB")
    return 1 if bad or misses else 0


if __name__ == "__main__":
    sys.exit(main())
