"""The gallery: every duck whose owner said yes to listing it, as one row each.

A share asks the owner whether to list the duck (`updater/src/vetbook.rs` in the microduck
repository). Yes leaves two marks on the dataset: the `microduck-vetbook-gallery` tag on its card,
which is how the Space finds the books without reading every vet book on the Hub, and
`share.listed` in `vetbook.json`, which is what it believes. Anybody can tag a dataset; only the
robot writes `listed`, so a tagged book without it stays out.

`entry` is arithmetic on a book, like `cards.py`, so it can be tested on its own; the fetching is
`app.py`'s.
"""

from __future__ import annotations

import re

import cards

TAG = "microduck-vetbook-gallery"

# `<user>/<duck>-vetbook`, the only datasets a share writes.
REPO = re.compile(r"^([A-Za-z0-9][A-Za-z0-9._-]{0,95})/([A-Za-z0-9][A-Za-z0-9._-]{0,95})-vetbook$")


def split(repo: str) -> tuple[str, str] | None:
    """`("apirrone", "graphite-bang")` from `apirrone/graphite-bang-vetbook`, or `None`."""
    m = REPO.match(repo)
    return (m.group(1), m.group(2)) if m else None


def entry(book: dict, user: str, duck: str) -> dict | None:
    """The gallery's row for a book, or `None` when its owner did not ask for it to be listed."""
    share = book.get("share") or {}
    if share.get("listed") is not True:
        return None
    c = cards.card_for(book, user, duck)
    life = book.get("life") or {}
    return {
        "user": user,
        "duck": duck,
        "robot": c.robot,
        # The dataset's namespace, not `share.owner`: it is the account that wrote the book.
        "owner": user,
        "mood": c.status.key,
        "mood_title": c.status.title,
        "mood_ink": c.status.ink,
        "personality": c.personality,
        # Every badge, rarest first: the tiles show a few, the filter wants them all.
        "badges": [{"key": b.key, "glyph": b.glyph, "name": b.name, "colour": b.colour} for b in c.badges],
        "shared_at": int(cards._num(share.get("shared_at"))),
        # The card this share drew, at its own versioned address, so a browser keeps it until the
        # duck is shared again.
        "image": f"/d/{user}/{duck}/card.png?v={int(cards._num(share.get('shared_at')))}",
        "day": c.day,
        "walked": f"{c.walked} {c.walked_unit}",
        "awake": c.awake,
        # Raw numbers, for sorting and filtering in the browser.
        "stats": {
            "distance_m": round(cards._num(life.get("distance_m")), 1),
            "awake_h": round(c.awake_hours, 2),
            "cuddles": c.cuddles,
            "pets": c.pets,
            "falls": c.falls,
            "songs": c.songs,
            "spins": c.spins,
            "tricks": c.tricks_known,
            "servos_replaced": c.servos_replaced,
            "age_days": c.day,
            "top_speed_kmh": round(c.records.get("top_speed_kmh", 0.0), 2),
            "photos": int(c.records.get("photos", 0)),
        },
    }
