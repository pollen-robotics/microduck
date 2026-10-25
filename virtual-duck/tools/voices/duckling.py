"""Duckling: the voice of a baby duck, for the virtual Microduck.

Every sound is built from one primitive, the *peep*: a round tone whose pitch follows a short
smooth contour and whose loudness follows a soft, click-free envelope. That is what a real
duckling does -- its calls carry their meaning in the pitch arc and the rhythm. The register is
brought down from a real chick's 3-5 kHz to about 350-1650 Hz, where a phone speaker is at its
sweetest.

A near-pure sine reads as a whistle or a beep on a phone, and its low notes all but vanish
there, so every peep has a body: a 2nd harmonic 3.5-8 dB under the fundamental, a 3rd 7-28 dB
under (fullest where it lands on the soft beak formant near 1.75 kHz that colours the
overtones), and a trace of 4th and 5th. Low notes get a little more body still (2nd 3.5-6 dB
down, 3rd 8-10 dB), since a phone hardly plays a fundamental under ~500 Hz, and the coo, warmest
of all, keeps its 2nd only 2.5-5 dB down. Everything is rolled off from about 3.3 kHz, so on the
highest notes, whose 3rd would land above 4 kHz, the 3rd stays faint (21-28 dB down): warm, not
buzzy or shrill. The 2nd alone rolls off from 4.8 kHz instead, so the top notes (1.3-1.65 kHz)
keep it about 7.5 dB down rather than thinning back to a whistle; it never reaches 4 kHz.

In tune: the part of every call that is *held* -- the plateau a peep scoops into, the note a
glide arrives on, the centre a trill wobbles around -- sits exactly on one D-major pentatonic
scale (no random detuning), so any string of sounds the app chains together (talk-back) lands in
tune with itself. Scoops, flicks and falls are the glides between those notes.

Level: each sound is set by how loud it is *through a phone speaker* (peak 50 ms loudness, phone
response + A-weighting), not by RMS or peak, so a high chirp and a low coo sit evenly side by side.
Pecks sit 4 dB under the chirps, coos 3 dB under, wheee 1 dB under: the coo's full body reads
0.5-2 dB louder than this measure to a loudness model that sums across the ear's bands (with
their spread), so its 3 dB is heard as about the 2 a talk-back run wants. Within a peep, the
scoops and falls under the held note are a touch softer than the note itself, as in a voice.

Meaning by contour:
  greet    two peeps, the second higher and longer, settling down onto the note below:
           "pee-PEEP." -- arrived, never a question
  inquire  a peep, then a longer one (or the longer one alone) that dips briefly and swells up to
           finish high, on the note it is heard on: "pip... pweee?"
  chirp    one bright arched peep, an upward flick, or a quick "pi-pip"
  coo      a warm, slow-down trill on D5 or E5 that sighs down a step or two: a sleepy "prrrr-oo"
  peck     a tiny nibble: a fast-decaying "pik" that drops 4-5 semitones as it strikes, so the
           fall is heard and not just its tail -- the quietest sound
  alarm    a clear upward jump to the top of the range, then a tumble down: "e-EEP!" (round)
  wheee    a long slide-whistle arc up and back down, with vibrato blooming at the top

Singing (`sing`): a little choir of four ducklings, each a pair of slightly detuned voices
gliding legato between the chorale's notes, an octave up. The tune sits on top; the other three
sing at 0.65 of its level, and any sung note under ~640 Hz leans on its harmonics, so on a phone
the bass, the tenor and the dark "oo" the piece opens on are heard as a choir, not a solo.

Only numpy. Deterministic: every random choice is seeded from (tag, variant) alone, and `pitch`
only scales frequencies, so the low/mid/high voices are the same gesture in another register.
"""

import zlib

import numpy as np

RATE = 24000
STYLE = {
    "name": "Duckling",
    "description": "Soft, round baby-duck peeps with a warm little body, whose sweet pitch glides "
                   "carry the meaning, all tuned to one pentatonic scale so its chatter is music.",
}
TAGS = {"greet": 5, "inquire": 5, "chirp": 8, "coo": 6, "peck": 8, "alarm": 4, "wheee": 4}

# Home note D5. Scale degrees (semitones) of D major pentatonic: D E F# A B.
HOME_HZ = 587.33
PENTA = np.array([-10, -8, -5, -3, 0, 2, 4, 7, 9, 12, 14, 16])

# Loudness through a phone speaker (peak 50 ms, dB) for greet/inquire/chirp/alarm at pitch 1.0;
# the others sit under it by LOUDNESS_OFFSET_DB, and a higher (smaller) voice carries a little
# more, LOUDNESS_PER_OCTAVE. The peak and the RMS are then held under their ceilings.
LOUDNESS_DB = -14.8
LOUDNESS_PER_OCTAVE = 3.0
# The coo is set 3 dB down rather than 2: its body is the fullest in the bank, and a loudness
# model that sums across the ear's bands hears it 0.5-2 dB louder than this measure does.
LOUDNESS_OFFSET_DB = {"greet": 0.0, "inquire": 0.0, "chirp": 0.0, "alarm": 0.0, "wheee": -1.0,
                      "coo": -3.0, "peck": -4.0}
PEAK_FLOOR_DB = -11.9
PEAK_CEIL_DB = -6.1
RMS_CEIL_DB = -16.2


# ---------------------------------------------------------------- building blocks

def _rng(*key):
    return np.random.default_rng(zlib.crc32(":".join(str(k) for k in key).encode()))


def _below(semi):
    """The pentatonic degree just under `semi`."""
    return float(PENTA[PENTA < semi - 1e-6].max())


def _lowpass(f, fc=3300.0, order=8):
    """Gentle absolute-frequency roll-off applied to every partial: no shrillness at any pitch."""
    return 1.0 / np.sqrt(1.0 + (f / fc) ** order)


# The 2nd partial is rolled off from higher up: on the top notes (fundamental 1.3-1.65 kHz) it is
# what keeps the voice from thinning to a whistle, and it stays under 4 kHz across the range.
H2_CORNER_HZ = 4800.0


def _formant(f):
    """A soft bump near 1.75 kHz -- the hint of a beak, and presence on a phone, without buzz."""
    return 1.0 + 0.55 * np.exp(-((f - 1750.0) / 450.0) ** 2)


# (harmonic, base weight, + per unit bright, + per unit warm, phase). The phases (0, pi, pi, 0, 0)
# keep the crest of the stack within a hair of a pure sine's, so the body costs no headroom.
_PARTIALS = (
    (1, 1.0, 0.0, 0.0, 0.0),
    (2, 0.38, 0.10, 0.14, np.pi),
    (3, 0.17, 0.05, 0.08, np.pi),
    (4, 0.06, 0.03, 0.02, 0.0),
    (5, 0.02, 0.015, 0.0, 0.0),
)
LOW_BODY = 1.2  # extra warmth at and under ~380 Hz, fading out by 640 Hz
TILT_DB = 0.3  # per semitone under the note a peep is held on


def _voice(f, amp, bright=0.3, warm=0.0, phase=0.0):
    """Additive voice with a body. `amp` is the true amplitude envelope: the harmonic stack is
    normalised per sample, so timbre can change with pitch while loudness stays as drawn."""
    f = np.asarray(f, dtype=np.float64)
    n = len(f)
    bright = np.broadcast_to(np.asarray(bright, dtype=np.float64), (n,))
    warm = np.broadcast_to(np.asarray(warm, dtype=np.float64), (n,))
    # Low notes get more body still: a phone barely plays a fundamental under ~500 Hz, so there
    # the harmonics carry the note (the ear still hears the fundamental's pitch).
    warm = warm + LOW_BODY * np.clip((640.0 - f) / 260.0, 0.0, 1.0)
    ph = 2 * np.pi * np.cumsum(f) / RATE + phase
    out = np.zeros(n)
    power = np.zeros(n)
    for k, h0, hb, hw, kph in _PARTIALS:
        # the beak formant colours the overtones; the fundamental is left plain
        lp = _lowpass(k * f, H2_CORNER_HZ) if k == 2 else _lowpass(k * f)
        g = (h0 + hb * bright + hw * warm) * lp * (_formant(k * f) if k > 1 else 1.0)
        out += g * np.cos(k * ph + kph)
        power += g * g
    return amp * out / np.sqrt(power)


def _contour(n, pts):
    """Semitone track through (time 0..1, semitone) keypoints, eased (cosine) between them."""
    t = np.linspace(0.0, 1.0, n)
    tp = np.array([p[0] for p in pts], dtype=np.float64)
    sp = np.array([p[1] for p in pts], dtype=np.float64)
    i = np.clip(np.searchsorted(tp, t, side="right") - 1, 0, len(tp) - 2)
    u = np.clip((t - tp[i]) / np.maximum(tp[i + 1] - tp[i], 1e-9), 0.0, 1.0)
    u = 0.5 - 0.5 * np.cos(np.pi * u)
    return sp[i] + (sp[i + 1] - sp[i]) * u


def _env(n, att, rel, decay=0.0, hold=0.0, tau=0.0):
    """Raised-cosine attack and release; in between an optional slow sag (`decay`, 0..1),
    starting after `hold` of the body has passed, or (`tau`, seconds) a fast exponential fall
    from the end of the attack -- a struck, nibbled sound."""
    e = np.ones(n)
    a = max(2, min(n // 2, int(att * RATE)))
    r = max(2, min(n - a, int(rel * RATE)))
    e[:a] = np.sin(0.5 * np.pi * np.arange(a) / a) ** 2
    e[n - r:] *= np.cos(0.5 * np.pi * np.arange(1, r + 1) / r) ** 2
    if decay:
        t = np.linspace(0.0, 1.0, n)
        sag = np.clip((t - hold) / max(1e-9, 1.0 - hold), 0.0, 1.0)
        e *= 1.0 - decay * (0.5 - 0.5 * np.cos(np.pi * sag))
    if tau:
        e[a:] *= np.exp(-np.arange(n - a) / (tau * RATE))
    return e


def _peep(dur, pts, base_hz, att=0.010, rel=0.045, decay=0.2, bright=0.35, warm=0.0,
          vib_semi=0.0, vib_hz=6.0, vib_from=0.3, level=1.0, phase=0.0, tau=0.0, held=None):
    """One peep: duration, pitch keypoints (relative semitones), base frequency. `held` is the
    note the peep is heard on (default: its highest keypoint)."""
    n = int(dur * RATE)
    semi = _contour(n, pts)
    # A little louder when higher, as a voice is: the scoops, dips and falls under the held note
    # are softer, and the note itself carries the call.
    held = max(p[1] for p in pts) if held is None else held
    tilt = 10 ** (-TILT_DB * np.clip(held - semi, 0.0, None) / 20)
    if vib_semi:
        t = np.arange(n) / RATE
        ramp = np.clip((np.linspace(0, 1, n) - vib_from) / max(1e-9, 1 - vib_from), 0, 1)
        semi = semi + vib_semi * ramp * np.sin(2 * np.pi * vib_hz * t)
    f = base_hz * 2.0 ** (semi / 12.0)
    env = _env(n, att, rel, decay, tau=tau) * tilt
    # Louder means a touch brighter, as in a real voice.
    return _voice(f, level * env, bright=bright * (0.55 + 0.45 * env), warm=warm, phase=phase)


def _place(parts):
    """[(start_seconds, signal), ...] -> one buffer."""
    end = max(int(round(s * RATE)) + len(x) for s, x in parts)
    out = np.zeros(end + 1)
    for s, x in parts:
        i = int(round(s * RATE))
        out[i:i + len(x)] += x
    return out


def _phone_loudness(x):
    """Peak short-term (50 ms) loudness of x as heard through a phone speaker, dB: little below
    ~400 Hz (2nd-order), a mild presence lift around 3 kHz, rolling off above ~8 kHz, then
    A-weighted (unity at 1 kHz)."""
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / RATE) + 1e-9
    w = (f / 400) ** 2 / (1 + (f / 400) ** 2)
    w *= 1 / np.sqrt(1 + (f / 8000) ** 4)
    w *= 1 + 0.4 * np.exp(-((np.log2(f / 3000)) ** 2) / 0.5)
    f2 = f ** 2
    a = (12194 ** 2 * f2 ** 2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2))
                                  * (f2 + 12194 ** 2))
    a /= a[np.argmin(np.abs(f - 1000))]
    y = np.fft.irfft(np.fft.rfft(x) * w * a, n)
    win = int(0.05 * RATE)
    if n < win:
        return float(10 * np.log10((y ** 2).mean() + 1e-20))
    p = np.convolve(y ** 2, np.ones(win) / win, mode="valid")
    return float(10 * np.log10(p.max() + 1e-20))


def _finish(x, tag, pitch):
    """Set the level by loudness on a phone, then keep the peak inside its window and the RMS
    under its ceiling."""
    x = np.asarray(x, dtype=np.float64)
    x = x - np.linspace(x[0], x[-1], len(x))  # ends exactly at zero
    target = LOUDNESS_DB + LOUDNESS_OFFSET_DB[tag] + LOUDNESS_PER_OCTAVE * np.log2(pitch)
    gain_db = target - _phone_loudness(x)
    peak_db = 20 * np.log10(np.abs(x).max() + 1e-12)
    rms_db = 20 * np.log10(np.sqrt((x ** 2).mean()) + 1e-12)
    gain_db = max(gain_db, PEAK_FLOOR_DB - peak_db)  # never so soft it is lost
    gain_db = min(gain_db, PEAK_CEIL_DB - peak_db, RMS_CEIL_DB - rms_db)
    return (x * 10 ** (gain_db / 20)).astype(np.float32)


# ---------------------------------------------------------------- the seven calls

def _greet(r, v, base):
    """Two happy peeps, the second higher and longer, settling onto the note below: "pee-PEEP." """
    lo, hi = [(0, 7), (2, 9), (0, 4), (4, 9), (2, 7)][v % 5]
    d1 = r.uniform(0.11, 0.14)
    gap = r.uniform(0.045, 0.065)
    d2 = r.uniform(0.22, 0.27)
    p1 = _peep(d1, [(0, lo - 1.5), (0.25, lo), (0.7, lo), (1, lo - 1.0)], base,
               att=0.009, rel=0.05, decay=0.25, bright=0.4)
    # scoop up into the high note, hold it, then settle down a step: an arrival, not a question
    p2 = _peep(d2, [(0, hi - 2.0), (0.17, hi), (0.6, hi), (0.9, _below(hi)), (1, _below(hi))], base,
               att=0.012, rel=0.09, decay=0.35, bright=0.45, vib_semi=0.1, vib_hz=6.5,
               vib_from=0.3, level=0.93)
    return _place([(0, p1), (d1 + gap, p2)])


def _inquire(r, v, base):
    """Curious and asking: ends on an upward turn, the note it rises to the one it is heard on."""
    start, top = [(0, 7), (2, 9), (4, 12), (4, 9), (2, 7)][v % 5]
    low = _below(start)
    # The rise is the question, so it carries the call: the dip under it is brief and a touch
    # softer (held=top), and the top note is reached before the release begins, not inside it.
    # That accent is also what keeps a long, even call clear of the RMS ceiling. (The single
    # "pweee?" sags more, so its end swells no louder than its onset: it still starts at once.)
    if v % 2 == 0:
        # "pip... pweee?" -- a short peep then a longer one that dips and rises.
        d1 = r.uniform(0.085, 0.11)
        gap = r.uniform(0.06, 0.09)
        d2 = r.uniform(0.25, 0.31)
        p1 = _peep(d1, [(0, start - 1.0), (0.25, start), (0.65, start), (1, start - 0.8)], base,
                   att=0.008, rel=0.04, decay=0.25, bright=0.35)
        p2 = _peep(d2, [(0, start), (0.16, low), (0.26, low), (0.80, top), (1, top)], base,
                   att=0.014, rel=0.06, decay=0.1, bright=0.4, vib_semi=0.08, vib_hz=6.0,
                   vib_from=0.75, level=0.92, held=top)
        return _place([(0, p1), (d1 + gap, p2)])
    # one "pweee?" that leans down, then turns up
    d = r.uniform(0.34, 0.42)
    return _peep(d, [(0, start - 0.8), (0.08, start), (0.24, low), (0.32, low), (0.82, top), (1, top)],
                 base, att=0.016, rel=0.07, decay=0.25, bright=0.4, vib_semi=0.08, vib_hz=6.2,
                 vib_from=0.75, held=top)


def _chirp(r, v, base):
    """Happy little peep."""
    # (shape, target note, where the arch arrives) -- every variant its own little word
    kind, top, apex = [(0, 7, 0.28), (1, 7, 0), (0, 12, 0.35), (2, 7, 0), (0, 9, 0.25), (1, 9, 0),
                       (0, 9, 0.4), (2, 9, 0)][v % 8]
    if kind == 0:  # arched "peep!"
        d = r.uniform(0.13, 0.17)
        land = [_below(top), _below(_below(top)), _below(top)][v % 3]
        a = apex + r.uniform(-0.03, 0.03)
        return _peep(d, [(0, top - 4), (a, top), (a + 0.3, top), (1, land)],
                     base, att=0.008, rel=0.05, decay=0.3, bright=0.45)
    if kind == 1:  # upward flick "wip!"
        d = r.uniform(0.11, 0.14)
        return _peep(d, [(0, top - 5), (0.5, top), (1, top)], base,
                     att=0.008, rel=0.04, decay=0.15, bright=0.45)
    # "pi-pip": two quick peeps, the second a step higher
    d1 = r.uniform(0.065, 0.08)
    d2 = r.uniform(0.08, 0.10)
    gap = r.uniform(0.025, 0.035)
    lo = _below(top)
    p1 = _peep(d1, [(0, lo - 1.5), (0.3, lo), (0.75, lo), (1, lo - 0.7)], base,
               att=0.007, rel=0.03, decay=0.2, bright=0.4)
    p2 = _peep(d2, [(0, top - 1.5), (0.3, top), (0.72, top), (1, top - 1.2)], base,
               att=0.007, rel=0.04, decay=0.25, bright=0.45, level=0.95)
    return _place([(0, p1), (d1 + gap, p2)])


def _coo(r, v, base):
    """Warm sleepy trill: rounded, slowing down, then sighing down onto a lower scale note."""
    # held on D5 or E5 (raised a fourth, where a phone carries it); sighs one or two steps down
    centre, steps = [(0, 1), (2, 1), (0, 2), (2, 2), (2, 1), (0, 1)][v % 6]
    land = _below(centre) if steps == 1 else _below(_below(centre))
    land -= centre
    d = r.uniform(0.62, 0.82)
    n = int(d * RATE)
    u = np.linspace(0, 1, n)
    # trill rate eases from ~12 Hz to ~7 Hz; depth fades as it drifts off. The trill wobbles
    # symmetrically about the held note, so the note it is heard on is the scale note.
    rate = r.uniform(11.0, 13.0) * (1 - 0.42 * u)
    trill_ph = 2 * np.pi * np.cumsum(rate) / RATE
    depth = r.uniform(0.45, 0.6) * (1 - 0.6 * u)
    sigh = r.uniform(0.7, 0.76)
    semi = centre + _contour(n, [(0, -0.5), (0.07, 0.0), (sigh, 0.0), (sigh + 0.09, land),
                                 (1, land)]) \
        + depth * np.sin(trill_ph)
    f = base * 2.0 ** (semi / 12.0)
    env = _env(n, 0.03, 0.2, decay=0.45, hold=0.12)
    env *= 1 - 0.12 * (1 - u) * (0.5 - 0.5 * np.cos(trill_ph))  # soft pulse of each trill beat
    return _voice(f, env, bright=0.12, warm=0.7)


def _peck(r, v, base):
    """A tiny nibble: struck, decaying fast, dropping 4-5 semitones onto a scale note."""
    kind, top = [(0, 7), (0, 4), (1, 9), (0, 9), (1, 7), (0, 12), (0, 2), (1, 4)][v % 8]

    def nib(d, note, level=1.0, tau=0.028):
        drop = note - _below(_below(note))  # two scale steps down: 4 or 5 semitones
        # the fall starts as the strike peaks and is half done within 6 dB of it, so the nibble is
        # heard to drop, not just its fading tail
        return _peep(d, [(0, note), (0.08, note), (0.40, note - drop), (1, note - drop)], base,
                     att=0.0065, rel=0.025, decay=0.0, tau=tau, bright=0.25, warm=0.2,
                     level=level)

    if kind == 1:  # "pi-pik": two tiny nibbles, the second a little softer
        d1 = r.uniform(0.055, 0.065)
        gap = r.uniform(0.022, 0.03)
        return _place([(0, nib(d1, top, tau=0.02)),
                       (d1 + gap, nib(r.uniform(0.065, 0.075), top, level=0.85, tau=0.022))])
    return nib(r.uniform(0.08, 0.095), top, tau=r.uniform(0.026, 0.03))


def _alarm(r, v, base):
    """Startled "e-EEP!": a clear jump up to the top of the range, then a tumble. Round, not shrill."""
    start, top, land = [(4, 12, 7), (7, 14, 9), (2, 12, 4), (0, 9, 2)][v % 4]
    d = r.uniform(0.24, 0.30)
    main = _peep(d, [(0, start), (0.05, start), (0.15, top), (0.42, top), (1, land)], base,
                 att=0.006, rel=0.07, decay=0.35, bright=0.5, vib_semi=0.12, vib_hz=8.0,
                 vib_from=0.25)
    if v % 2 == 1:
        # a little echoing "ip" after the eep
        tail = _peep(0.075, [(0, land), (0.4, land), (1, _below(land))], base,
                     att=0.006, rel=0.035, decay=0.3, bright=0.35, level=0.55)
        return _place([(0, main), (d + 0.035, tail)])
    return main


def _wheee(r, v, base):
    """Thrown: a playful slide-whistle arc up and back down."""
    d = r.uniform(0.95, 1.3)
    lo, top, land = [(-3, 12, 4), (-5, 12, 2), (-3, 9, 4), (0, 12, 7)][v % 4]
    n = int(d * RATE)
    semi = _contour(n, [(0, lo), (0.08, lo), (0.52, top), (0.72, top), (0.93, land), (1, land)])
    t = np.arange(n) / RATE
    u = np.linspace(0, 1, n)
    bloom = np.exp(-((u - 0.62) / 0.2) ** 2)
    semi = semi + (0.06 + 0.3 * bloom) * np.sin(2 * np.pi * r.uniform(5.6, 6.4) * t)
    f = base * 2.0 ** (semi / 12.0)
    env = _env(n, 0.018, 0.22)
    # launched full, carried over the top, easing off on landing
    env *= _contour(n, [(0, 1.0), (0.72, 1.0), (1, 0.6)])
    return _voice(f, env, bright=0.4 * (0.6 + 0.4 * env))


_CALLS = {"greet": _greet, "inquire": _inquire, "chirp": _chirp, "coo": _coo, "peck": _peck,
          "alarm": _alarm, "wheee": _wheee}


def render(tag: str, variant: int, pitch: float) -> np.ndarray:
    v = int(variant) % TAGS[tag]
    r = _rng("duckling", tag, v)
    return _finish(_CALLS[tag](r, v, HOME_HZ * float(pitch)), tag, float(pitch))


# ---------------------------------------------------------------- the chorale

_STEP = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}
_VOWEL = {  # harmonic weights (2nd, 3rd, 4th) for each sung vowel
    "oo": (0.10, 0.03, 0.008),
    "ah": (0.26, 0.11, 0.035),
    "mm": (0.05, 0.012, 0.003),
}
_DYN = {"pp": 0.45, "p": 0.55, "mp": 0.65, "mf": 0.75, "f": 0.88, "ff": 1.0}
# Added to the 2nd, 3rd and 4th harmonics of any sung note under ~640 Hz (in full by 380 Hz, the
# shape of LOW_BODY): a phone hardly plays those fundamentals, so a low voice -- the bass, the
# tenor, and the dark "oo" the opening builds on -- is heard through its harmonics or not at all.
_SUNG_LOW_LIFT = (0.25, 0.10, 0.03)


def _midi(name):
    name = name.strip()
    step = _STEP[name[0].upper()]
    i = 1
    while i < len(name) and name[i] in "#b":
        step += 1 if name[i] == "#" else -1
        i += 1
    return 12 * (int(name[i:]) + 1) + step


def _fftconv(x, k):
    """'same'-mode convolution via FFT."""
    n = len(x) + len(k) - 1
    m = 1 << (n - 1).bit_length()
    y = np.fft.irfft(np.fft.rfft(x, m) * np.fft.rfft(k, m), m)[:n]
    s = (len(k) - 1) // 2
    return y[s:s + len(x)]


def _smooth(x, ms):
    k = np.hanning(max(3, int(ms * RATE / 1000)) + 2)[1:-1]
    return _fftconv(x, k / k.sum())


def _score(chorale, total):
    """Per voice: list of (t0, t1, midi, vowel, level, hum)."""
    beat = 60.0 / float(chorale.get("bpm", 60))
    names = chorale.get("voices_low_to_high", ["bass", "tenor", "alto", "soprano"])
    segs = [[] for _ in names]
    t = 0.0
    for ev in chorale["events"]:
        if t >= total:
            break
        kind = ev.get("type")
        lvl = _DYN.get(ev.get("dynamic", "mf"), 0.75)
        if kind == "rest":
            t += ev["beats"] * beat
            continue
        if kind in ("build", "chord"):
            dur = ev["beats"] * beat
            order = list(range(len(names)))
            if ev.get("from_top"):
                order = order[::-1]
            for pos, vi in enumerate(order):
                note = ev["voices"][vi]
                if note == "-":
                    continue
                start = t + (pos * ev.get("stagger", 0) * beat if kind == "build" else 0.0)
                segs[vi].append((start, t + dur, _midi(note), ev.get("vowel", "ah"), lvl, False))
            t += dur
        elif kind == "solo":
            part = names.index(ev["part"]) if ev.get("part") in names else len(names) - 1
            dur = sum(nn["beats"] for nn in ev["notes"]) * beat
            for vi, note in enumerate(ev.get("under", [])):
                if note == "-" or vi == part:
                    continue
                segs[vi].append((t, t + dur, _midi(note), ev.get("hum", "mm"), lvl * 0.8, True))
            tt = t
            for nn in ev["notes"]:
                d = nn["beats"] * beat
                segs[part].append((tt, tt + d, _midi(nn["note"]), nn.get("vowel", "ah"), lvl, False))
                tt += d
            t += dur
    return segs


def _sing_voice(segs, n, pitch, role, r):
    """One duckling holding one part, legato, with glides between notes."""
    if not segs:
        return np.zeros(n)
    midi = np.full(n, np.nan)
    level = np.zeros(n)
    h = np.zeros((3, n))
    age = np.zeros(n)
    scoop = np.zeros(n)
    dips = np.zeros(n)
    prev_end, prev_midi = -1.0, None
    for (t0, t1, m, vowel, lvl, hum) in segs:
        i0, i1 = int(t0 * RATE), min(n, int(t1 * RATE))
        if i0 >= n or i1 <= i0:
            continue
        midi[i0:i1] = m
        level[i0:i1] = lvl * (0.62 if hum else 1.0)
        h[:, i0:i1] = np.array(_VOWEL.get(vowel, _VOWEL["ah"]))[:, None]
        fresh = abs(t0 - prev_end) > 0.02  # entering after silence
        if fresh or prev_midi != m:
            age[i0:i1] = np.arange(i1 - i0) / RATE
        else:  # same note continues: keep aging, re-articulate the syllable softly
            age[i0:i1] = age[i0 - 1] + 1.0 / RATE + np.arange(i1 - i0) / RATE
        if fresh:  # a little duckling scoop into the note
            k = np.arange(min(i1 - i0, int(0.12 * RATE))) / RATE
            scoop[i0:i0 + len(k)] = -0.9 * np.exp(-k / 0.035)
        else:
            w = int(0.05 * RATE)
            a, b = max(0, i0 - w), min(n, i0 + w)
            dips[a:b] = np.maximum(dips[a:b], np.hanning(2 * w)[: b - a] * (0.22 if prev_midi == m else 0.12))
        prev_end, prev_midi = t1, m
    # fill silences with the neighbouring note so the glide has somewhere to come from
    valid = ~np.isnan(midi)
    idx = np.where(valid, np.arange(n), 0)
    np.maximum.accumulate(idx, out=idx)
    first = np.argmax(valid)
    midi = np.where(valid, midi, midi[idx])
    midi[:first] = midi[first]
    semi = _smooth(midi, 70) + _smooth(scoop, 12) + (12.0 * 1.0)  # up an octave: duckling register
    t = np.arange(n) / RATE
    vib_depth = {"soprano": 0.13, "alto": 0.07, "tenor": 0.06, "bass": 0.05}[role]
    vib = vib_depth * np.clip(age / 0.45, 0, 1) * np.sin(2 * np.pi * r.uniform(5.0, 5.6) * t + r.uniform(0, 6.28))
    drift = 0.04 * np.sin(2 * np.pi * r.uniform(0.15, 0.3) * t + r.uniform(0, 6.28))
    amp = _smooth(level * (1 - dips), 90)
    h = np.vstack([_smooth(row, 60) for row in h])
    out = np.zeros(n)
    for detune, ph in ((-0.035, r.uniform(0, 6.28)), (0.035, r.uniform(0, 6.28))):
        s = semi + detune + vib * (1 + 0.15 * detune / 0.035) + drift
        f = 440.0 * pitch * 2.0 ** ((s - 69.0) / 12.0)
        phase = 2 * np.pi * np.cumsum(f) / RATE + ph
        num = np.sin(phase)
        pw = np.ones(n)
        extra = 1.6 if role == "bass" else 1.0  # bass leans on harmonics: a phone cannot play its root
        low = np.clip((640.0 - f) / 260.0, 0.0, 1.0)
        for k in (2, 3, 4):
            g = (extra * h[k - 2] + _SUNG_LOW_LIFT[k - 2] * low) * _lowpass(k * f) * _formant(k * f)
            num += g * np.sin(k * phase)
            pw += g * g
        out += 0.5 * num / np.sqrt(pw)
    return out * amp


def _reverb(x, seconds=1.5, rt60=1.3, wet=0.2):
    r = _rng("duckling", "hall")
    m = int(seconds * RATE)
    t = np.arange(m) / RATE
    ir = r.standard_normal(m) * np.exp(-6.9 * t / rt60)
    ir = np.convolve(ir, np.hanning(9) / np.hanning(9).sum(), mode="same")  # dark tail, no hiss
    ir[: int(0.018 * RATE)] = 0.0  # pre-delay
    ir /= np.sqrt((ir ** 2).sum())
    y = np.fft.irfft(np.fft.rfft(x, len(x) + m) * np.fft.rfft(ir, len(x) + m), len(x) + m)[: len(x)]
    return (1 - wet) * x + wet * y


def sing(chorale: dict, pitch: float, seconds: float) -> np.ndarray:
    n = int(seconds * RATE)
    segs = _score(chorale, seconds)
    names = chorale.get("voices_low_to_high", ["bass", "tenor", "alto", "soprano"])
    # the tune on top, the three under it close enough behind to be heard as a choir on a phone
    gains = {"bass": 0.65, "tenor": 0.65, "alto": 0.65, "soprano": 1.0}
    r = _rng("duckling", "choir")
    mix = np.zeros(n)
    for vi, name in enumerate(names):
        role = name if name in gains else ("soprano" if vi == len(names) - 1 else "alto")
        mix += gains[role] * _sing_voice(segs[vi], n, float(pitch), role, r)
    mix = _reverb(mix)
    # gentle fade in, and a long fade at the end
    fi = int(0.03 * RATE)
    mix[:fi] *= np.sin(0.5 * np.pi * np.arange(fi) / fi) ** 2
    fo = min(n // 2, int(2.5 * RATE))
    mix[n - fo:] *= np.cos(0.5 * np.pi * np.arange(1, fo + 1) / fo) ** 2
    mix = np.asarray(mix, dtype=np.float64)
    return (mix * 10 ** (-7.0 / 20) / (np.abs(mix).max() + 1e-12)).astype(np.float32)
