# NFC tags

A tag touched to the duck's head pairs a gamepad or switches the robot's mode, with nothing typed.
The robot writes its own tags too. This page owns all of it: which tags there are, what each one
does, which tags the reader takes, how to write one, and how to see what happened.

`nfcd` is the daemon behind it. It is always running, polls the reader five times a second, and
picks a USB reader up within thirty seconds of it being plugged in. The robot's other daemons do
the actual work: `configd` pairs the pad and `robotd` switches the mode. `nfcd` asks them over their
sockets exactly as `robotctl` would.

## The tags

Every tag holds **one text record**, and the text is the whole message: surrounding spaces are
ignored, case is not. A tag that merely *contains* one of these texts among others does nothing.

| Text on the tag | What the robot does | Same as |
|---|---|---|
| `microduck_gamepad_pairing` | pairs the pad that is in pairing mode | `robotctl pad pair` |
| `walk_mode` | laser tracking and soccer off, then walk mode | holding D-pad up |
| `roller_skating_mode` | laser tracking and soccer off, then roller mode | holding D-pad up |
| `laser_mode` | soccer off, walk mode, then laser tracking on | `robotctl laser-track on` |
| `soccer_mode` | laser tracking off, walk mode, then soccer mode on | `robotctl soccer on` |
| `autonomous_mode` | read and quacked at; nothing yet | — |

The robot **quacks** (a short chirp) as soon as it has read any of them, so the person holding the
tag knows it was read, whatever follows.

**A tag acts once per touch.** One left on the reader is not read again and again: lift it (about
half a second away from the reader) and touch it again to repeat.

### Pairing a pad

Put the pad in pairing mode — [`pair-a-gamepad.md`](pair-a-gamepad.md#put-the-pad-in-pairing-mode)
has the button for each pad — then touch the pairing tag to the head.

- The robot **quacks** when it has read the tag, and **greets** — a different sound, often a double
  "wak-wak" — once the pad is paired. A refusal gets no second sound.
- **A pad already connected wins.** If one is driving, a touch does nothing, so a tag brushed against
  the robot never hands the bond to someone else's pad.
- **The tag names no pad.** The robot takes the one in pairing mode, and refuses if it finds two, as
  `robotctl pad pair` does. Every pairing tag is therefore the same. A tag with an address after the
  text, as the first ones had, pairs nothing.
- **One attempt per touch.** If it did not pair, lift the tag and touch again.

When a pad will not bond at all, that is the pad and the radio, not the tag:
[`pair-a-gamepad.md`](pair-a-gamepad.md#when-a-pad-will-not-bond-at-all).

### Switching mode

**A mode tag is the mode the robot is then in**, not something added to the last one. Laser
tracking and soccer are switches on top of walk mode rather than modes of their own, so their tags
put the robot in walk first, and every other mode tag switches them off.

- **`walk_mode`, `roller_skating_mode`** switch the policy as holding D-pad up does.
- **`laser_mode`** follows a laser dot on the floor with the head and walks to it. A pushed stick on
  the pad wins while it is pushed. The cheat sheet's
  [Follow a laser dot](cheatsheet.md#follow-a-laser-dot) has the readout and the tuning.
- **`soccer_mode`** is soccer **by hand**: **LB / RB kick** with the left and right foot, and
  **D-pad down** sets the robot playing by itself — it looks for the purple ball, walks to it and
  kicks it. Any button takes it back to soccer by hand. The tag deliberately does not start
  autonomous play: a robot that sets off after a ball the moment a card touches it surprises
  whoever is holding it. The cheat sheet's [Play soccer](cheatsheet.md#play-soccer) has the rest.
- **`autonomous_mode`** is recognised, so a tag written today keeps working, and does nothing yet.

A `robotd` too old to know laser tracking or soccer refuses that switch; the journal says so and the
mode itself still changes.

The table and where a new mode's behaviour goes are in `nfc/src/modes.rs`.

## Which tags work

- **NTAG** (NFC Forum Type 2, ISO 14443A) — NTAG213, 215, 216. The one to buy: every reader reads it
  and the robot writes it.
- **ISO 15693** (NFC Forum Type 5: ICODE SLIX, ST25TV) — read by the beta board's reader only, not
  by the USB reader, and not written by the robot. One that is not NDEF-formatted yet needs its
  maker's app first (NXP TagWriter, ST25 NFC Tap).
- **MIFARE Classic** (SAK `08`, "NXP MIFARE Classic 1K" in a phone app) — **not read.** It says
  so once per touch in the journal, `tag cannot be read`. Use an NTAG.

To know what a tag is, NFC Tools on a phone shows it under *Read*: the tag type, the technology
(ISO 14443-3A for an NTAG), and the SAK (`0x00` for an NTAG).

The reader's range is a centimetre or two. A shell or case between the tag and the head is often
the difference between a touch that works and one that does nothing: touch the tag flat against
the head, and hold it still for half a second.

## Writing a tag

**From the robot itself** — on the robot, or from a laptop over Bluetooth:

```bash
robotctl nfc-flash "walk_mode"
duckctl --name <robot-name> nfc-flash "walk_mode"
```

1. The robot asks for a tag with the sound it makes when it loses its pad, the questioning quack.
2. Touch an NTAG to the head within **20 seconds** (`--timeout` changes that, up to 120). A tag
   already on the reader is written too.
3. The robot **quacks** when the tag took, and the command prints `<uid> now reads "walk_mode"` once
   it has read it back.

Quote the text whenever it has a space or a character the shell reads (`*`, `?`, `!`, `$`, `&`,
`#`); for the texts above the quotes change nothing.

While a write is waiting, a touched tag is written rather than acted on, and the tag just written
is not then read as a touch: writing a pairing tag does not pair, writing a mode tag does not
switch the mode.

**Only an NTAG is written.** A Type 5 tag, a MIFARE Classic, a tag locked read-only, or a text
longer than the tag holds is refused with why, and nothing on it changes. Only the message area is
written — never the UID, the lock bytes or the configuration pages — so a write cannot lock a tag.
A tag lifted half way through reads as no NDEF; write it again.

`duckctl nfc-flash` needs a `duckctl` and a daemon that both know `nfc.write`; on a robot whose
`nfcd` is older, nothing is listening on `/run/nfcd/nfc.sock` and the command says so.

**From a phone** — NFC Tools: *Write* → *Add a record* → *Text*, type the text, *Write*, and hold
the tag to the phone. If the phone says *erreur d'écriture* / *write error*, hold the tag still
against the phone's NFC spot (usually near the camera) until it confirms; a tag moved during the
write is the usual cause.

## What happened

```bash
sudo journalctl -u nfcd -b -f
```

`sudo`, because the journal of a system service is not readable by the operator account otherwise.
Then touch the tag and read the line that appears:

| Line | Meaning |
|---|---|
| `reader up — touch a tag` | the reader is found; nothing touched yet |
| `no NFC reader; retrying` | no reader on this robot, or not plugged in |
| `pairing tag touched — looking for a pad in pairing mode` | then `paired`, or why not |
| `a pad is already connected — nothing to do` | a pad was driving; disconnect it first |
| `mode tag touched`, then `mode switched` | the mode changed |
| `the robot refused the mode` | `robotd` said no, and says why |
| `this mode does nothing yet` | `autonomous_mode` |
| `neither a pairing nor a mode tag` | the tag's text is something else — the line shows it |
| `tag holds no readable NDEF` | a blank tag, or one lifted during a write |
| `tag cannot be read` | a MIFARE Classic |
| `tag not read; trying again` | the tag moved or sits at the edge of the field |
| `tag written` / `tag not written` | an `nfc-flash`, and why not |

**Nothing at all appears** when the tag is touched: the reader did not see it. Take the shell off or
press the tag flat against the head; check the tag is an NTAG; then check the reader line above.
