#!/usr/bin/env python3
"""Regenerate robotd/emotions/ from a clone of microduck_emotions.

    git clone https://github.com/pollen-robotics/microduck_emotions /tmp/microduck_emotions
    python3 scripts/vendor-emotions.py /tmp/microduck_emotions robotd/emotions

Writes one <name>.json and <name>.wav per emotion, cut to what the real duck plays: the keyframes
inside the sound's window, re-timed to start at 0. The set ships inside every daemon release and
robotd loads it from there (robotd/src/emotion.rs), so this is how a new or changed emotion reaches
the robots: rerun, commit, release.
"""

import json
import pathlib
import shutil
import sys

src = pathlib.Path(sys.argv[1]).expanduser()
out = pathlib.Path(sys.argv[2]).expanduser()
out.mkdir(parents=True, exist_ok=True)
for e in json.loads((src / "emotions.json").read_text())["emotions"]:
    if e["name"] == "play_dead":
        continue  # its dead pose needs a joint-level call the runtime does not have yet
    start, end = e["sound_at_s"], e["sound_at_s"] + e["duration_s"]
    keys = [
        dict(k, t=round(k["t"] - start, 3))
        for k in json.loads((src / e["keyframes"]).read_text())["keyframes"]
        if start - 1e-6 <= k["t"] <= end + 1e-6
    ]
    if e["name"] == "curious":  # the real duck keeps only the two tilts
        for k in keys:
            k["neck"] = k["head_pitch"] = 0.0
    (out / f"{e['name']}.json").write_text(json.dumps({"keyframes": keys}))
    shutil.copy(src / e["sounds"][0], out / f"{e['name']}.wav")
