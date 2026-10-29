---
title: microduck vet book
emoji: 🦆
colorFrom: pink
colorTo: purple
sdk: docker
app_port: 7860
pinned: false
short_description: Shareable cards for microducks, drawn from their vet books.
---

# microduck vet book

Every microduck keeps a vet book: hours awake, metres walked, cuddles, falls, tricks, and every
servo it has been through. On the robot, `robotctl vetbook` shows it, and `s` shares it:
`updaterd` publishes it to a public dataset on the owner's Hugging Face account,
`<user>/<duck>-vetbook`, and this Space turns it into a card.

- `/`: the gallery — every duck whose owner said yes to listing it when sharing. Search by name or
  owner, sort by any stat, filter by mood and badge; a tile opens the duck's card, its owner's name
  their Hugging Face profile. A listed book carries the `microduck-vetbook-gallery` tag (how the
  gallery finds it) *and* `share.listed: true` (what the gallery believes — anybody can tag a
  dataset). `gallery.py` turns a book into a row; the list refreshes from the Hub every 5 minutes.
- `/api/ducks`: the gallery's rows as JSON.
- `/d/<user>/<duck>`: the page to share, with buttons to post it. Its preview image is the card,
  so a pasted link unfurls as the card.
- `/d/<user>/<duck>/card.png`: the card, 1200 × 630. Each share draws one of eight at random;
  `?card=notebook|purikura|profile|receipt|passport|gameboy|polaroid|fifa` picks one, and
  `?season=halloween|winter|valentine|none` previews a seasonal skin.
- `/d/<user>/<duck>/story.png`: the same duck as a 1080 × 1920 story, for Instagram and TikTok.

What a card says — the mood, the badges, the fun facts, the personality and star sign, the palette
and the season — is `traits.py`, arithmetic on the book; `python test_traits.py` checks it. Each
design is a `card_<name>.html` extending `card.html`.

The source lives in the microduck repository under `spaces/vetbook/`, next to the code that
publishes the book (`updater/src/vetbook.rs`), and is deployed with `scripts/publish-space.sh vetbook`.

To work on the cards without sharing anything, point the app at local books and the system's
Chrome:

```
VETBOOK_LOCAL=./books VETBOOK_CHROME=chrome uvicorn app:app --port 7861
# reads ./books/<user>/<duck>-vetbook/vetbook.json
```
