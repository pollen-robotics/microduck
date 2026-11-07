#!/usr/bin/env python3
"""Render the duck's voices for the app: three styles, each in a low, a middle and a high register.

The voices are synthesized here, with numpy and no recordings, by the modules in `tools/voices/`:
Babble (the default: duck-speak, quick vowel syllables on a pentatonic scale, in the manner of a
game's animalese), Duckling (a duckling's peeps) and Squeaky (a bath-toy squeak). Each has a few
variants of every call the duck makes — hello, a question, a happy chirp, a content coo, a nibble, a
startled eek, a whee — and sings the robots' own chorale, `sounds/scores/wistful.duckscore`, in its
voice. `tools/voices/check.py` measures a voice against what makes a sound pleasant or grating on a
phone speaker; `tools/voices/comfort.py` says what each measure stands for.

Within a voice every call is already levelled against the others, as heard through a phone
speaker, and the three voices come out within about two decibels of each other, so changing voice
hardly changes how loud the duck is. The song is levelled here, register by register, to sit just
under the duck's talking: a verse sung at its own peak level would be heard several decibels
quieter than a chirp.

Every sound is encoded to Ogg Vorbis at 24 kHz. Writes `voices/<style>-<register>/<tag>/*.ogg` and
`voices.json`, the index the app reads: the styles, which sounds exist, how long each lasts, and its
loudness every 20 ms, which is what opens the beak in time with the sound.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "voices"))
import comfort  # noqa: E402
import score  # noqa: E402

# The first is the default, the one a new duck hatches with.
STYLES = ("babble", "duckling", "squeaky")
REGISTERS = (("low", 0.8), ("mid", 1.0), ("high", 1.25))
# The verse the duck sings, with its fade. SoundPool decodes a sample into about a megabyte, and cuts
# off whatever does not fit — at 24 kHz, 16-bit, anything past 21 s — so the song stops short of that.
SONG_S = 20.0
# How far under the talking the song sits, and the highest it may peak.
SONG_UNDER_TALK_DB = 2.0
SONG_PEAK_DB = -3.0
ENVELOPE_HZ = 50


def load(style: str):
    spec = importlib.util.spec_from_file_location(style, HERE / "voices" / f"{style}.py")
    voice = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voice)
    return voice


def envelope(x: np.ndarray, rate: int) -> list[float]:
    """Loudness per 20 ms, mapped onto 0–1 from -45 dBFS to -5."""
    hop = rate // ENVELOPE_HZ
    frames = max(1, len(x) // hop)
    padded = np.pad(x, (0, max(0, frames * hop - len(x))))
    rms = np.sqrt((padded[: frames * hop].reshape(frames, hop) ** 2).mean(axis=1) + 1e-12)
    return np.round(np.clip((20 * np.log10(rms) + 45) / 40, 0, 1), 2).tolist()


def sustained_loudness(x: np.ndarray) -> float:
    """How loud a song is heard on a phone: the 90th percentile of its 50 ms loudness, the measure
    calls are levelled by taken over its louder moments rather than its single loudest."""
    y = comfort.phone_weighted(x)
    win = int(0.05 * comfort.RATE)
    power = np.convolve(y ** 2, np.ones(win) / win, mode="valid")
    return float(10 * np.log10(np.percentile(power, 90) + 1e-20))


def level_song(song: np.ndarray, calls: dict[str, list[np.ndarray]]) -> np.ndarray:
    """The song brought to just under the talking of its register, its peak kept in bounds."""
    talk = float(np.median([comfort.phone_loudness(x) for t in ("greet", "inquire", "chirp") for x in calls[t]]))
    gain = talk - SONG_UNDER_TALK_DB - sustained_loudness(song)
    peak = 20 * np.log10(np.abs(song).max() + 1e-12)
    return song * 10 ** (min(gain, SONG_PEAK_DB - peak) / 20)


def encode(x: np.ndarray, rate: int, ogg: Path, scratch: Path) -> None:
    wav = scratch / "sound.wav"
    with wave.open(str(wav), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())
    ogg.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-y", "-threads", "1", "-i", str(wav),
         "-ac", "1", "-ar", "24000", "-c:a", "libvorbis", "-q:a", "4", str(ogg)],
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    chorale = score.chorale()
    voices_dir = args.out / "voices"
    shutil.rmtree(voices_dir, ignore_errors=True)
    index: dict = {"envelope_hz": ENVELOPE_HZ, "styles": [], "registers": [r for r, _ in REGISTERS], "voices": {}}
    with tempfile.TemporaryDirectory() as scratch:
        for style in STYLES:
            voice = load(style)
            index["styles"].append({"id": style, "name": voice.STYLE["name"], "description": voice.STYLE["description"]})
            for register, pitch in REGISTERS:
                key = f"{style}-{register}"
                bank = {tag: [np.asarray(voice.render(tag, v, pitch), dtype=np.float64) for v in range(n)]
                        for tag, n in voice.TAGS.items()}
                bank["song"] = [level_song(np.asarray(voice.sing(chorale, pitch, SONG_S), dtype=np.float64), bank)]
                entry: dict = {}
                for tag, sounds in bank.items():
                    for v, x in enumerate(sounds):
                        name = "song.ogg" if tag == "song" else f"{tag}_{v}.ogg"
                        encode(x, comfort.RATE, voices_dir / key / tag / name, Path(scratch))
                        entry.setdefault(tag, []).append(
                            {"file": name, "duration": round(len(x) / comfort.RATE, 3), "envelope": envelope(x, comfort.RATE)}
                        )
                index["voices"][key] = entry
                mid = float(np.median([comfort.phone_loudness(x) for t in ("greet", "inquire", "chirp") for x in bank[t]]))
                print(f"{key}: talk {mid:.1f} dB on a phone, song {sustained_loudness(bank['song'][0]):.1f}")
    (args.out / "voices.json").write_text(json.dumps(index, separators=(",", ":")) + "\n")
    size = sum(f.stat().st_size for f in voices_dir.rglob("*.ogg"))
    count = sum(len(v) for v in index["voices"][f"{STYLES[0]}-mid"].values())
    print(f"{voices_dir}: {len(index['voices'])} voices × about {count} sounds, {size / 1e6:.1f} MB")


if __name__ == "__main__":
    sys.exit(main())
