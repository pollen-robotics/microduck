"""The duck chorale as the voices sing it: `sounds/scores/wistful.duckscore`, the score the robots
sing together, read into a plain list of events.

    {"name", "bpm", "voices_low_to_high": [bass, tenor, alto, soprano], "events": [...]}

Events are `chord` (four voices hold notes), `build` (the voices come in one after another,
`stagger` beats apart, from the bottom or `from_top`), `solo` (one `part` sings `notes` while the
others hum `under`) and `rest`. Notes are scientific pitch names, C4 being middle C; `-` is a voice
that is silent.
"""

from __future__ import annotations

import re
from pathlib import Path

SCORE = Path(__file__).resolve().parents[3] / "sounds" / "scores" / "wistful.duckscore"


def chorale(path: Path = SCORE) -> dict:
    events, bpm, vowel, dyn = [], 58.0, "oo", "mf"
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        t = line.split()
        if t[0] == "bpm:":
            bpm = float(t[1])
        elif t[0] == "vowel":
            vowel = t[1]
        elif t[0] == "dynamic":
            dyn = t[1]
        elif t[0] == "chord":
            events.append({"type": "chord", "beats": float(t[1]), "voices": t[2:6], "vowel": vowel, "dynamic": dyn})
        elif t[0] == "build":
            beats, stagger = float(t[1]), float(t[3])
            top = t[4] == "top"
            voices = t[5:9] if top else t[4:8]
            events.append({"type": "build", "beats": beats, "stagger": stagger, "from_top": top, "voices": voices,
                           "vowel": vowel, "dynamic": dyn})
        elif t[0] == "rest":
            events.append({"type": "rest", "beats": float(t[1])})
        elif t[0] == "solo":
            part, under, rest = t[1], t[3:7], t[7:]
            hum = "mm"
            if rest[0] == "hum":
                hum, rest = rest[1], rest[2:]
            assert rest[0] == "sing", line
            notes = []
            for n in rest[1:]:
                m = re.match(r"([A-G][#b]?\d)(?::([\d.]+))?(?:/(\w+))?", n)
                notes.append({"note": m[1], "beats": float(m[2] or 1), "vowel": m[3] or vowel})
            events.append({"type": "solo", "part": part, "under": under, "hum": hum, "notes": notes, "dynamic": dyn})
    return {"name": path.stem, "bpm": bpm, "voices_low_to_high": ["bass", "tenor", "alto", "soprano"],
            "note_names": "scientific pitch, C4 = middle C = MIDI 60; '-' = that voice is silent", "events": events}
