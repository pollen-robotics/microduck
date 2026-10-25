"""Squeaky: the duck's voice as a rubber bath-toy squeak, made kind to a phone speaker.

A real squeaker is a reed: air forced through a slot makes a tone whose pitch climbs as the
squeeze builds and drops as it lets go, with a nasal "ee" and a hiss of air. This keeps the climb
and the drop and the nasal colour and leaves out the hiss and the edge:

* Every sound is additive -- whole harmonics of one fundamental, so there is no noise at all and
  nothing inharmonic to grate. Each harmonic's level comes from a spectral envelope: a fixed
  "rubber body" resonance near 1.25 kHz that makes every sound the same toy, three vowel formants
  that move inside a syllable (so it says "wee", "hi", "hoh" rather than beeping), a steep roll-off
  above 2.8 kHz on the high squeaks and 3.3 kHz on the rest (no shrillness) and below 270 Hz
  (nothing a phone cannot play). The "ee" is kept soft at 2-5 kHz, where a squeak toy tires the ear.
* The harmonic levels are normalised to one total at every instant, so a pitch bend never makes a
  sound swell or thin out by accident; each finished sound is then levelled by how loud it is heard
  through a phone speaker, so a run of talk-back is even. Where the -12 dBFS peak floor would hold
  a high squeak well above its level, its envelope is made peakier instead: the highest wheee arc
  eases off toward its top, and a double nibble's bites ease off sooner after the click.
* Pitch lives on C major pentatonic (C5 D5 E5 G5 A5, coos on G4 A4 C5), so any run of sounds the app
  strings together is in tune with itself, and the song is in the same key. The loud, held part of
  each squeak sits on its note, and second syllables, hops and question rises land on the scale.
* Each squeak has the toy's shape: a fast scoop up into the note (the squeeze), a rounded peak, a
  drop as it lets go, and a shallow rubbery warble well under the rate the ear hears as buzz. A hello
  arrives and falls; only a question ends rising, and it swells into that rise, so the end of a
  talk-back run carries -- louder than a hello, but, that lead allowed for, no brighter at 2-5 kHz:
  the two-syllable questions' last "ee" leans toward "e", and the brightest rises stop a step short. A startle holds a soft low "u" for
  20-25 ms at half the level of its "EEK", then leaps most of an octave onto it 30-35 ms in:
  "u-EEK", not a scoop.

`render` is deterministic per (tag, variant, pitch): the variant seeds the small re-rolls, and
pitch only transposes, so the low, middle and high ducks say the same thing in their own register.
"""

import zlib

import numpy as np

RATE = 24000

STYLE = {
    "name": "Squeaky",
    "description": "A rubber bath-toy duck: bouncy pitch-bent squeaks, a squishy coo and a squeaky wheee, "
    "built from pure harmonics with no air hiss or harsh edge.",
}

TAGS = {"greet": 6, "inquire": 6, "chirp": 8, "coo": 6, "peck": 8, "alarm": 5, "wheee": 5}

C5 = 523.251
TWO_PI = 2.0 * np.pi

# Vowel formants: (centre Hz, bandwidth Hz, gain). Bandwidths are wide on purpose -- the harmonics
# of a 500-1100 Hz voice are far apart, and a narrow peak would make a glide flicker.
# The upper formants of "i" and "e" are held well down: 2-5 kHz is where a squeak toy turns from
# cute to tiring over a day of listening, and a phone speaker already lifts it.
VOWELS = {
    "i": ((320, 180, 0.45), (2250, 420, 0.60), (2950, 460, 0.30)),
    "e": ((480, 200, 0.70), (1950, 400, 0.70), (2650, 460, 0.30)),
    "a": ((780, 260, 1.00), (1250, 300, 0.80), (2550, 460, 0.30)),
    "o": ((520, 220, 1.00), (900, 260, 0.70), (2450, 460, 0.20)),
    "u": ((360, 180, 1.00), (780, 240, 0.50), (2350, 460, 0.12)),
    "m": ((300, 180, 1.00), (1100, 320, 0.12), (2300, 460, 0.04)),
}
BODY = (1250.0, 900.0, 0.40)   # the toy's own resonance, under every vowel
FLOOR = 0.06
LP_HZ, HP_HZ = 3300.0, 270.0
LP_BRIGHT = 2800.0             # the high, "i"-heavy squeaks (greet, chirp, alarm, wheee) roll off sooner

PENTA = (0, 2, 4, 7, 9)        # C major pentatonic, semitones above C
_SCALE = [12 * o + pc for o in range(-3, 4) for pc in PENTA]

# Loudness as heard on a phone, relative to a greet or chirp. A question leads a little, so it reads
# as the end of a talk-back run; pecks sit under the chatter and a coo is a touch softer.
# The question gets its lead from its shape, not from more level overall: its loudest moment is the
# rise at the end, so the RMS ceiling never caps it, even in the low register. REF_DB sits as
# high as that allows, because the high register's bright squeaks need the level to keep their peaks
# above the -12 dBFS floor; the alarm's accent settles at once partly so its peak clears that floor
# while it is heard no louder than a chirp.
REF_DB = -17.3
LOUDNESS = {"greet": 0.0, "chirp": 0.0, "alarm": 0.0, "inquire": 1.25, "wheee": -1.0, "coo": -2.0, "peck": -4.0}
PEAK_MAX, PEAK_MIN = -6.5, -11.85
RMS_MAX, RMS_MIN = -16.15, -25.85


# ---------------------------------------------------------------------------------- engine

def _hz(semis):
    return C5 * 2.0 ** (semis / 12.0)


def _rng(tag, variant):
    return np.random.default_rng(zlib.crc32(f"squeaky:{tag}:{variant}".encode()))


def _deg(note, k):
    """Semitones from `note` (on the scale) to the pentatonic note `k` degrees away: every second
    syllable, hop and question lands back on the scale."""
    i = _SCALE.index(int(note))
    return _SCALE[i + k] - int(note)


def _interp(t, keys):
    """Cosine-eased interpolation through (time, value) keys: every bend starts and lands softly."""
    ks = np.array([k[0] for k in keys], dtype=np.float64)
    vs = np.array([k[1] for k in keys], dtype=np.float64)
    if len(ks) == 1:
        return np.full_like(t, vs[0])
    idx = np.clip(np.searchsorted(ks, t, side="right") - 1, 0, len(ks) - 2)
    t0, t1 = ks[idx], ks[idx + 1]
    u = np.clip((t - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)
    u = 0.5 - 0.5 * np.cos(np.pi * u)
    return vs[idx] + (vs[idx + 1] - vs[idx]) * u


def _vowel(v):
    if isinstance(v, str):
        return np.array(VOWELS[v], dtype=np.float64)
    a, b, w = v                                   # a blend: (vowel, vowel, weight of the second)
    return _mix(np.array(VOWELS[a], float), np.array(VOWELS[b], float), w)


def _mix(pa, pb, w):
    out = pa * (1 - w) + pb * w
    out[:, 0] = np.exp(np.log(pa[:, 0]) * (1 - w) + np.log(pb[:, 0]) * w)
    return out


def _formant_track(t, keys):
    """(3 formants, [F, B, G], n) moving through (time, vowel) keys; centres move in log frequency."""
    times = [k[0] for k in keys]
    params = np.array([_vowel(k[1]) for k in keys])          # (m, 3, 3)
    out = np.empty((3, 3, len(t)))
    for i in range(3):
        for j in range(3):
            vals = params[:, i, j]
            if j == 0:
                out[i, j] = np.exp(_interp(t, list(zip(times, np.log(vals)))))
            else:
                out[i, j] = _interp(t, list(zip(times, vals)))
    return out


def _synth(f0, amp, form, tilt, lp=LP_HZ, chunk=24000):
    """Additive synthesis: harmonics of f0, each weighted by the spectral envelope, normalised to
    unit total power at every sample, times the amplitude envelope."""
    n = len(f0)
    phase = TWO_PI * np.cumsum(f0) / RATE
    kmax = int(max(1, min(24, 11000.0 / f0.max(), 5200.0 / max(f0.min(), 60.0) + 1)))
    k = np.arange(1, kmax + 1, dtype=np.float64)[:, None]
    out = np.zeros(n)
    bf, bb, bg = BODY
    for s in range(0, n, chunk):
        sl = slice(s, min(n, s + chunk))
        f = k * f0[None, sl]
        env = FLOOR + bg / (1.0 + ((f - bf) / (0.5 * bb)) ** 2)
        for i in range(3):
            env = env + form[i, 2, sl] / (1.0 + ((f - form[i, 0, sl]) / (0.5 * form[i, 1, sl])) ** 2)
        env = env * k ** (-tilt[None, sl])
        env = env / np.sqrt(1.0 + (f / lp) ** 10)
        env = env / np.sqrt(1.0 + (HP_HZ / f) ** 8)
        env = env / (np.sqrt((env ** 2).sum(axis=0, keepdims=True)) + 1e-12)
        out[sl] = amp[sl] * (env * np.sin(k * phase[None, sl])).sum(axis=0)
    return out


class _Line:
    """A sound built syllable by syllable: control tracks appended, then synthesised in one pass so
    phase runs on through gaps and slurs."""

    def __init__(self, base_hz, lp=LP_HZ):
        self.base, self.lp = base_hz, lp
        self.f0, self.amp, self.form, self.tilt = [], [], [], []

    def syllable(self, dur, pitch, amp, vowels, tilt=1.0, scoop=None, warble=None, vibrato=None):
        """pitch/amp/vowels are (seconds, value) keys; pitch in semitones from the base.
        scoop = (semitones, tau s): the squeeze, an exponential slide up into the note.
        warble = (Hz, cents, phase): the rubbery wobble. vibrato = (Hz, cents, delay s)."""
        n = int(round(dur * RATE))
        t = np.arange(n) / RATE
        semis = _interp(t, pitch)
        if scoop:
            semis = semis + scoop[0] * np.exp(-t / scoop[1])
        if warble:
            semis = semis + warble[1] / 100.0 * np.sin(TWO_PI * warble[0] * t + warble[2])
        if vibrato:
            grow = np.clip(t / max(vibrato[2], 1e-3), 0, 1) ** 2
            semis = semis + grow * vibrato[1] / 100.0 * np.sin(TWO_PI * vibrato[0] * t)
        self.f0.append(self.base * 2.0 ** (semis / 12.0))
        self.amp.append(_interp(t, amp))
        self.form.append(_formant_track(t, vowels))
        self.tilt.append(np.full(n, float(tilt)))
        return self

    def gap(self, dur):
        n = int(round(dur * RATE))
        self.f0.append(np.full(n, self.f0[-1][-1]))
        self.amp.append(np.zeros(n))
        self.form.append(np.repeat(self.form[-1][:, :, -1:], n, axis=2))
        self.tilt.append(np.full(n, self.tilt[-1][-1]))
        return self

    def render(self):
        f0 = np.concatenate(self.f0)
        amp = np.concatenate(self.amp)
        form = np.concatenate(self.form, axis=2)
        tilt = np.concatenate(self.tilt)
        return _synth(f0, amp, form, tilt, self.lp)


def _squeak_amp(dur, att=0.012, peak=1.0, settle=0.82, tail=0.70, rel=0.045):
    """The squeak's envelope: a quick swell to an accent, a settle, a held body, a rounded let-go."""
    a2 = min(att + 0.03, dur * 0.45)
    r0 = max(a2 + 0.005, dur - rel)
    return [(0, 0), (att, peak), (a2, peak * settle), (r0, peak * tail), (dur, 0)]


def _heard(x):
    """Peak short-term (50 ms) loudness of x through a typical phone speaker, dB: little below
    ~400 Hz, a presence lift near 3 kHz, roll-off above 8 kHz, then A-weighted. Sounds are levelled
    by this, not by peak or RMS, because a low coo and a bright chirp with the same RMS are not
    heard as equally loud on a phone, and talk-back strings them together."""
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / RATE) + 1e-9
    w = (f / 400) ** 2 / (1 + (f / 400) ** 2)
    w = w / np.sqrt(1 + (f / 8000) ** 4)
    w = w * (1 + 0.4 * np.exp(-((np.log2(f / 3000)) ** 2) / 0.5))
    f2 = f ** 2
    a = (12194 ** 2 * f2 ** 2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2))
                                  * (f2 + 12194 ** 2))
    w = w * a / a[np.argmin(np.abs(f - 1000))]
    y = np.fft.irfft(np.fft.rfft(x) * w, n)
    win = int(0.05 * RATE)
    if n < win:
        return float(10 * np.log10(np.mean(y ** 2) + 1e-20))
    p = np.convolve(y ** 2, np.ones(win) / win, mode="valid")
    return float(10 * np.log10(p.max() + 1e-20))


def _finish(x, tag):
    """Level to the tag's loudness as heard on a phone, inside safe peak and RMS bounds."""
    x = np.asarray(x, dtype=np.float64)
    x = x - np.linspace(x[0], x[-1], len(x))                 # exact zeros at both ends
    rms = 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-12)
    pk = 20 * np.log10(np.abs(x).max() + 1e-12)
    g = REF_DB + LOUDNESS[tag] - _heard(x)
    g = min(g, PEAK_MAX - pk, RMS_MAX - rms)
    g = max(g, PEAK_MIN - pk, RMS_MIN - rms)
    x = x * 10 ** (g / 20)
    n = int(0.0015 * RATE)
    ramp = np.linspace(0, 1, n)
    x[:n] *= ramp
    x[-n:] *= ramp[::-1]
    return x


# ---------------------------------------------------------------------------------- the sounds

def _chirp(v, r, p):
    """A happy squeak, 0.1-0.22 s: one bounce of the toy, four shapes. The loud, held middle of every
    shape sits on its scale note -- the "wip" flick too, which lands on a short held top -- and the
    bends happen in the attack and the let-go."""
    note = (4, 7, 2, 9, 4, 7, 0, 9)[v % 8]
    d = (0.15, 0.13, 0.19, 0.12, 0.21, 0.16, 0.18, 0.14)[v % 8] * r.uniform(0.94, 1.06)
    line = _Line(_hz(note + r.uniform(-0.15, 0.15)) * p, lp=LP_BRIGHT)
    warble = (r.uniform(10, 13), r.uniform(10, 18), r.uniform(0, TWO_PI))
    shape = v % 4
    if shape == 0:      # "eep!" -- up, round, down
        pk = r.uniform(0.3, 0.6)
        line.syllable(d, [(0, -0.4), (0.45 * d, pk), (0.62 * d, 0.0), (d, -r.uniform(2.5, 3.5))], _squeak_amp(d),
                      [(0, "u"), (0.25 * d, "i"), (d, "e")], tilt=1.1, scoop=(-4.0, 0.012), warble=warble)
    elif shape == 1:    # "wip!" -- an upward flick onto a short held top, then a small lift as it lets go
        up = 0.021                                  # the flick lands on the note just before the accent
        top = up + 0.25 * d                         # end of the held top; the lift is in the let-go
        line.syllable(d, [(0, -4.0), (up, 0.0), (top, 0.0), (d, r.uniform(1.2, 2.0))],
                      [(0, 0), (0.010, 0.38), (up + 0.003, 1.0), (top, 0.8), (d, 0)],
                      [(0, "u"), (up, "i"), (top, "e"), (d, "e")], tilt=1.2, warble=warble)
    elif shape == 2:    # "ee-ip" -- a double bounce, the second a scale step higher
        step = _deg(note, 2 if v % 3 == 2 else 1)
        amp = [(0, 0), (0.012, 1.0), (0.38 * d, 0.78), (0.5 * d, 0.3), (0.62 * d, 0.88),
               (d - 0.035, 0.62), (d, 0)]
        line.syllable(d, [(0, -0.2), (0.42 * d, 0.2), (0.56 * d, step + 0.3), (0.8 * d, step), (d, step - 1.2)], amp,
                      [(0, "e"), (0.4 * d, "i"), (0.5 * d, ("i", "u", 0.6)), (0.62 * d, "i"), (d, "e")],
                      tilt=1.1, scoop=(-3.5, 0.010), warble=warble)
    else:               # "squee-k" -- a peak then the let-go
        line.syllable(d, [(0, -0.3), (0.3 * d, 0.5), (0.55 * d, 0.0), (d, -r.uniform(2.5, 3.5))],
                      _squeak_amp(d, tail=0.6),
                      [(0, "e"), (0.3 * d, "i"), (d, "o")], tilt=1.1, scoop=(-4.5, 0.012), warble=warble)
    return _finish(line.render(), "chirp")


def _peck(v, r, p):
    """Eating a seed: a tiny soft "pik", 0.08-0.18 s. Rounder and quieter than a chirp, the click of
    the bite on the note and the drop after it. A double nibble's bites are shorter and ease off
    sooner after the click, so in the high register the -12 dBFS peak floor does not hold the G5 one
    well above its level."""
    note = (0, 4, 2, 7, -3, 4, 0, 2)[v % 8]
    line = _Line(_hz(note + r.uniform(-0.2, 0.2)) * p)
    bites = 2 if v in (3, 6) else 1
    settle = 0.024 if bites == 1 else 0.018         # when the click has eased to the bite's body
    for b in range(bites):
        d = (0.085 if bites == 1 else 0.07) * r.uniform(0.9, 1.15)
        drop = r.uniform(2.0, 3.5)
        lift = 0.0 if b == 0 else float(_deg(note, 1))
        line.syllable(d, [(0, lift + 0.4), (0.3 * d, lift), (d, lift - drop)],
                      [(0, 0), (0.008, 1.0 if b == 0 else 0.8), (settle, 0.62), (d * 0.7, 0.25), (d, 0)],
                      [(0, "i"), (0.4 * d, "e"), (d, "u")], tilt=1.4, scoop=(-2.0, 0.006))
        if b + 1 < bites:
            line.gap(r.uniform(0.018, 0.028))
    return _finish(line.render(), "peck")


def _coo(v, r, p):
    """Content, being petted: a warm squishy "hoh", lower and rounder, 0.5-0.85 s -- an open "o/a"
    so it is warm rather than muffled."""
    note = (-5, -3, 0, -5, -3, 0)[v % 6]
    d = (0.62, 0.72, 0.55, 0.80, 0.66, 0.70)[v % 6] * r.uniform(0.95, 1.05)
    line = _Line(_hz(note + r.uniform(-0.15, 0.15)) * p)
    vib = (r.uniform(4.8, 5.6), r.uniform(14, 20), 0.18)
    oa, ob = ("o", "a", 0.55), ("o", "a", 0.35)
    if v % 2 == 0:      # "hoooh" -- one long contented sigh that sinks at the end
        pitch = [(0, 0), (0.4 * d, 0.4), (0.72 * d, -0.4), (d, -r.uniform(2.0, 3.0))]
        amp = [(0, 0), (0.035, 1.0), (0.25 * d, 0.9), (0.7 * d, 0.72), (d, 0)]
        vowels = [(0, "u"), (0.15 * d, oa), (0.7 * d, ob), (d, "o")]
    else:               # "hoh-oh" -- squished twice, the rubber relaxing
        pitch = [(0, 0), (0.38 * d, 0.3), (0.48 * d, -1.0), (0.6 * d, 0.6), (d, -r.uniform(2.0, 3.0))]
        amp = [(0, 0), (0.035, 1.0), (0.36 * d, 0.85), (0.48 * d, 0.58), (0.6 * d, 0.84), (d, 0)]
        vowels = [(0, "u"), (0.15 * d, oa), (0.48 * d, "o"), (0.62 * d, oa), (d, "o")]
    line.syllable(d, pitch, amp, vowels, tilt=1.05, scoop=(-2.5, 0.035), vibrato=vib)
    return _finish(line.render(), "coo")


def _greet(v, r, p):
    """Hello: two (or three) bouncy squeaks stepping up -- "hi-HI!", 0.35-0.55 s. The last one
    arrives and falls, so it is a hello and never a question."""
    a, b = ((0, 7), (4, 9), (2, 7), (0, 4), (4, 12), (2, 9))[v % 6]
    line = _Line(_hz(a + r.uniform(-0.15, 0.15)) * p, lp=LP_BRIGHT)
    warble = (r.uniform(10, 12.5), r.uniform(8, 14), r.uniform(0, TWO_PI))
    d1 = r.uniform(0.10, 0.125)
    line.syllable(d1, [(0, 0), (0.6 * d1, 0.3), (d1, 0.1)], _squeak_amp(d1, rel=0.03, tail=0.75),
                  [(0, "e"), (0.4 * d1, "i"), (d1, "i")], tilt=1.15, scoop=(-3.0, 0.012), warble=warble)
    if v % 6 == 4:      # "hi-hi-HI!"
        line.gap(r.uniform(0.025, 0.035))
        d1b = r.uniform(0.085, 0.10)
        mid = 7 - a                                     # E5, G5, C6
        line.syllable(d1b, [(0, mid), (0.6 * d1b, mid + 0.3), (d1b, mid + 0.1)],
                      _squeak_amp(d1b, peak=0.9, rel=0.03, tail=0.75),
                      [(0, "e"), (0.4 * d1b, "i"), (d1b, "i")], tilt=1.15, scoop=(-2.5, 0.010), warble=warble)
    line.gap(r.uniform(0.025, 0.04))
    d2 = r.uniform(0.2, 0.26) if v % 6 != 4 else r.uniform(0.18, 0.21)
    s = b - a
    line.syllable(d2, [(0, s), (0.3 * d2, s + 0.4), (0.7 * d2, s), (d2, s - 1.0)],
                  _squeak_amp(d2, peak=0.93, settle=0.88, tail=0.7, rel=0.07),
                  [(0, "a"), (0.3 * d2, "i"), (0.75 * d2, "e"), (d2, ("e", "o", 0.5))],
                  tilt=1.1, scoop=(-2.5, 0.016), warble=warble)
    return _finish(line.render(), "greet")


def _inquire(v, r, p):
    """A question, asking for food or a fuss: low "hm?" then a squeak that holds, dips and rises --
    "mm-mee?". The held part sits on the note; the rise lands on the scale a few steps up, and the
    squeak swells into it, so the rise is the loudest thing heard and the "hm" stays a hum.
    It leads a hello or chirp by 1.25 dB, but its 2-5 kHz, where a squeak tires the ear, is no
    stronger than the brightest of them once that lead is taken off: the two-syllable rise ends on
    an "ee" leaning toward "e", and the rises that put a harmonic on the "ee"'s bright spot near
    2.5 kHz (E5 to C6, C5 to A5) stop a scale step lower."""
    note = (0, -3, 2, 0, -3, 2)[v % 6]
    line = _Line(_hz(note + r.uniform(-0.15, 0.15)) * p)
    warble = (r.uniform(9, 12), r.uniform(8, 14), r.uniform(0, TWO_PI))
    degs = (3, 4, 2, 3, 3, 3)[v % 6]                # the rise ends on A5 G5 A5 G5 G5 A5 (middle register)
    if v % 2 == 0:      # two syllables: "hm? mee-ee?"
        d1 = r.uniform(0.10, 0.12)
        line.syllable(d1, [(0, 0.2), (d1, -0.4)], _squeak_amp(d1, att=0.014, rel=0.035, tail=0.7),
                      [(0, "u"), (d1, "o")], tilt=1.2, scoop=(-2.5, 0.014), warble=warble)
        line.gap(r.uniform(0.03, 0.045))
        d2 = r.uniform(0.24, 0.3)
        st = float(_deg(note, 1))
        rise = float(_deg(note + int(st), degs))
        line.syllable(d2, [(0, st), (0.3 * d2, st), (0.45 * d2, st - 0.7), (0.78 * d2, st + rise * 0.6),
                           (d2, st + rise)],
                      [(0, 0), (0.014, 0.92), (0.044, 0.82), (0.32 * d2, 0.76), (0.45 * d2, 0.62),
                       (d2 - 0.055, 0.92), (d2, 0)],
                      [(0, "e"), (0.4 * d2, "e"), (d2, ("i", "e", 0.4))], tilt=0.95, scoop=(-2.0, 0.014),
                      warble=warble)
    else:               # one syllable that holds, dips, then lifts into the question
        d = r.uniform(0.38, 0.46)
        rise = float(_deg(note, degs))
        line.syllable(d, [(0, 0), (0.3 * d, 0.1), (0.47 * d, -1.0), (0.8 * d, rise * 0.6), (d, rise)],
                      [(0, 0), (0.014, 1.0), (0.05, 0.84), (0.36 * d, 0.74), (0.47 * d, 0.48),
                       (0.62 * d, 0.8), (d - 0.055, 0.94), (d, 0)],
                      [(0, "u"), (0.3 * d, "o"), (0.47 * d, "u"), (0.62 * d, "e"), (d, "i")],
                      tilt=1.0, scoop=(-3.0, 0.014), warble=warble)
    return _finish(line.render(), "inquire")


def _alarm(v, r, p):
    """Startled: a surprised "u-EEK!" -- a soft low "u" held for 20-25 ms, a 10 ms leap of most of an
    octave that lands on the EEK 30-35 ms in, a jolt that settles at once, then a tumble down --
    surprise, never fright. The "u" is long and loud enough to be heard as a note of its own, so
    the leap reads as one rather than as a scoop up into the EEK."""
    note = (9, 7, 9, 7, 4)[v % 5]
    d = (0.34, 0.30, 0.38, 0.32, 0.36)[v % 5] * r.uniform(0.95, 1.05)
    line = _Line(_hz(note + r.uniform(-0.15, 0.15)) * p, lp=LP_BRIGHT)
    tremble = (r.uniform(8.0, 9.5), r.uniform(22, 32), r.uniform(0, TWO_PI))
    lo = float(_deg(note, -4 if v % 5 != 1 else -5))   # the run-up note: 9-12 semitones below
    g = r.uniform(0.020, 0.025)                         # how long the run-up note is held
    j = g + 0.010                                       # the leap is over by here, and the EEK is at its loudest
    run = [(0, 0), (0.008, 0.5), (g, 0.5), (j, 1.0)]    # the run-up is held 6 dB under the EEK
    shape = v % 3
    if shape == 0:      # leap, hold, tumble
        pitch = [(0, lo), (g, lo), (j, 0.6), (j + 0.03, 0.0), (0.6 * d, 0.2), (d, -6.0)]
        amp = run + [(j + 0.022, 0.74), (d - 0.08, 0.62), (d, 0)]
        vowels = [(0, "u"), (g, "e"), (j, "i"), (0.6 * d, "i"), (d, "o")]
    elif shape == 1:    # leap and tremble, then a sigh down a third
        pitch = [(0, lo), (g, lo), (j, 0.5), (j + 0.035, 0.0), (0.6 * d, -0.3), (d - 0.03, -4.8), (d, -5.3)]
        amp = run + [(j + 0.022, 0.76), (d - 0.07, 0.68), (d, 0)]
        vowels = [(0, "u"), (g, "e"), (j, "i"), (0.6 * d, "i"), (d, "e")]
    else:               # "u-EE-ek!" -- two jolts, the first the bigger
        pitch = [(0, lo), (g, lo), (j, 0.5), (0.45 * d, 0.0), (0.55 * d, -2.0), (0.62 * d, -1.0), (d, -6.0)]
        amp = run + [(j + 0.022, 0.76), (0.44 * d, 0.72), (0.52 * d, 0.42), (0.6 * d, 0.76), (d - 0.06, 0.58), (d, 0)]
        vowels = [(0, "u"), (g, "e"), (j, "i"), (0.5 * d, "e"), (0.6 * d, "i"), (d, "o")]
    line.syllable(d, pitch, amp, vowels, tilt=1.2, warble=tremble)
    return _finish(line.render(), "alarm")


def _wheee(v, r, p):
    """Thrown or jumping: a squeaky glide up and over, "wheee-oo", 0.95-1.35 s. A glide grows louder
    on a phone as it climbs, so the highest arc (14 semitones) eases off toward its top: it is heard
    about as loud at the top as on the way up, and its peak comes early, on the climb."""
    start = (-5, -3, -5, -3, -5)[v % 5]
    d = (1.15, 1.25, 1.30, 1.05, 1.20)[v % 5] * r.uniform(0.96, 1.04)
    line = _Line(_hz(start + r.uniform(-0.15, 0.15)) * p, lp=LP_BRIGHT)
    vib = (r.uniform(5.4, 6.2), r.uniform(26, 36), 0.35 * d)
    top = (12.0, 12.0, 11.0, 14.0, 12.0)[v % 5]     # semitones above the start: G4 -> G5 etc.
    end = top - r.choice([5.0, 7.0])
    shape = v % 5
    if shape in (0, 3):     # one arc, up and over; the 14-semitone arc's top is 0.72, the octave's 0.86
        pitch = [(0, 0), (0.1 * d, 1.5), (0.5 * d, top), (0.62 * d, top + 0.4), (d, end)]
        amp = [(0, 0), (0.03, 1.0), (0.15 * d, 0.9), (0.62 * d, 0.86 - 0.07 * (top - 12.0)), (0.85 * d, 0.6), (d, 0)]
        vowels = [(0, "u"), (0.3 * d, "i"), (0.7 * d, "i"), (d, "o")]
    elif shape == 1:        # up, then two hops on the way down: "wheee-hee-hee"
        h1, h2 = 0.62 * d, 0.8 * d
        pitch = [(0, 0), (0.1 * d, 1.5), (0.5 * d, top), (h1 - 0.03, top), (h1 + 0.04, top - 2.0),
                 (h2 - 0.03, top - 2.0), (h2 + 0.04, top - 4.0), (d, end - 1.0)]
        amp = [(0, 0), (0.03, 1.0), (0.15 * d, 0.9), (h1 - 0.04, 0.85), (h1, 0.45), (h1 + 0.05, 0.8),
               (h2 - 0.04, 0.72), (h2, 0.4), (h2 + 0.05, 0.68), (d, 0)]
        vowels = [(0, "u"), (0.3 * d, "i"), (h1, "e"), (h1 + 0.05, "i"), (h2, "e"), (h2 + 0.05, "i"), (d, "o")]
    elif shape == 2:        # a roller-coaster: up, a dip, up again, down
        pitch = [(0, 0), (0.3 * d, top - 3.0), (0.45 * d, top - 5.0), (0.65 * d, top + 1.0), (d, end)]
        amp = [(0, 0), (0.03, 1.0), (0.2 * d, 0.9), (0.45 * d, 0.78), (0.65 * d, 0.88), (0.88 * d, 0.55), (d, 0)]
        vowels = [(0, "u"), (0.28 * d, "i"), (0.45 * d, "e"), (0.62 * d, "i"), (d, "o")]
    else:                   # a long rise, then a slow float down
        pitch = [(0, 0), (0.4 * d, top), (0.5 * d, top + 0.3), (d, end - 2.0)]
        amp = [(0, 0), (0.03, 1.0), (0.2 * d, 0.92), (0.5 * d, 0.85), (0.8 * d, 0.6), (d, 0)]
        vowels = [(0, "u"), (0.32 * d, "i"), (0.6 * d, "i"), (0.85 * d, "e"), (d, "u")]
    line.syllable(d, pitch, amp, vowels, tilt=1.15, scoop=(-3.0, 0.03), vibrato=vib)
    return _finish(line.render(), "wheee")


_SOUNDS = {"greet": _greet, "inquire": _inquire, "chirp": _chirp, "coo": _coo,
           "peck": _peck, "alarm": _alarm, "wheee": _wheee}


def render(tag: str, variant: int, pitch: float) -> np.ndarray:
    """One sound: `tag` from TAGS, `variant` 0..TAGS[tag]-1, `pitch` 0.8 low / 1.0 middle / 1.25 high."""
    n = TAGS[tag]
    return _SOUNDS[tag](variant % n, _rng(tag, variant % n), float(pitch))


# ---------------------------------------------------------------------------------- the song

_PC = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}
_SONG_VOWEL = {"oo": "u", "ah": ("a", "o", 0.35), "mm": "m", "ee": "i", "oh": "o"}
_DYN = {"pp": 0.45, "p": 0.55, "mp": 0.65, "mf": 0.75, "f": 0.92, "ff": 1.0}
# bass, tenor, alto, soprano: the soprano carries the tune, the rest sit softly underneath.
_GAIN = (0.36, 0.32, 0.30, 1.0)
_VIB = ((4.9, 12), (5.3, 14), (5.6, 15), (5.4, 22))
_TILT = (1.45, 1.4, 1.35, 0.75)
_TRANSPOSE = 12                     # an octave up: the soprano sings E5-G5, right where a duck lives


def _midi(name):
    pc = _PC[name[0].upper()]
    rest = name[1:]
    while rest and rest[0] in "#b":
        pc += 1 if rest[0] == "#" else -1
        rest = rest[1:]
    return 12 * (int(rest) + 1) + pc


def _smooth(x, n):
    """Box filter applied twice (a triangle), O(n) with cumulative sums; zero beyond the ends."""
    if n < 2:
        return x
    for _ in range(2):
        pad = np.concatenate([np.zeros(n), x, np.zeros(n)])
        c = np.cumsum(np.concatenate([[0.0], pad]))
        y = (c[n:] - c[:-n]) / n
        x = y[n // 2: n // 2 + len(x)]
    return x


def _smooth_hold(x, n):
    """Like _smooth but the ends are held, for tracks that have no meaning at zero (pitch, formants)."""
    if n < 2:
        return x
    for _ in range(2):
        pad = np.concatenate([np.full(n, x[0]), x, np.full(n, x[-1])])
        c = np.cumsum(np.concatenate([[0.0], pad]))
        y = (c[n:] - c[:-n]) / n
        x = y[n // 2: n // 2 + len(x)]
    return x


def _score(chorale):
    """Per voice, a list of (start s, end s, midi, vowel, level) from the chorale's events."""
    spb = 60.0 / float(chorale["bpm"])
    names = chorale.get("voices_low_to_high", ["bass", "tenor", "alto", "soprano"])
    voices = [[] for _ in names]
    t = 0.0
    for ev in chorale["events"]:
        kind = ev["type"]
        beats = ev.get("beats")
        if beats is None:                    # a solo lasts as long as its notes
            beats = sum(float(nt["beats"]) for nt in ev.get("notes", []))
        dur = float(beats) * spb
        dyn = _DYN.get(ev.get("dynamic", "mf"), 0.75)
        if kind in ("build", "chord"):
            vowel = _SONG_VOWEL.get(ev.get("vowel", "ah"), "a")
            order = list(range(len(names)))
            if kind == "build" and ev.get("from_top"):
                order = order[::-1]
            stagger = float(ev.get("stagger", 0.0)) * spb if kind == "build" else 0.0
            for i, vi in enumerate(order):
                note = ev["voices"][vi]
                if note != "-":
                    voices[vi].append((t + i * stagger, t + dur, _midi(note), vowel, dyn))
        elif kind == "solo":
            part = names.index(ev["part"])
            hum = _SONG_VOWEL.get(ev.get("hum", "mm"), "m")
            for vi, note in enumerate(ev.get("under", [])):
                if vi != part and note != "-":
                    voices[vi].append((t, t + dur, _midi(note), hum, dyn * 0.7))
            tt = t
            for nt in ev["notes"]:
                d = float(nt["beats"]) * spb
                if nt["note"] != "-":
                    voices[part].append((tt, tt + d, _midi(nt["note"]),
                                         _SONG_VOWEL.get(nt.get("vowel", "ah"), "a"), dyn))
                tt += d
        t += dur
    return voices


def _voice(segs, n, pitch, vi, seed):
    """One duck of the choir: a legato line with soft glides, held notes tied across chords, a small
    dip to re-articulate a change of note, a squeaky little scoop into each soprano note, and
    vibrato that blooms after each onset."""
    r = np.random.default_rng(seed)
    t = np.arange(n) / RATE
    semis = np.full(n, np.nan)
    level = np.zeros(n)
    form = np.zeros((3, 3, n))
    onsets = []
    prev_end, prev_midi = None, None
    for (t0, t1, midi, vowel, lvl) in sorted(segs, key=lambda s: s[0]):
        a, b = int(round(t0 * RATE)), min(n, int(round(t1 * RATE)))
        if a >= n or b <= a:
            continue
        semis[a:b] = midi + _TRANSPOSE - 69
        level[a:b] = lvl
        params = _vowel(vowel)
        if vi == 3:                     # the lead duck sings a little nasal, so the tune carries
            params = _mix(params, _vowel("e"), 0.45)
        form[:, :, a:b] = params[:, :, None]
        joined = prev_end is not None and abs(prev_end - t0) < 1e-6
        if not (joined and prev_midi == midi):
            onsets.append((t0, joined))
        prev_end, prev_midi = t1, midi
    if not np.isfinite(semis).any():
        return np.zeros(n)
    idx = np.where(np.isfinite(semis), np.arange(n), -1)       # hold pitch and vowel through silence
    idx = np.maximum.accumulate(idx)
    first = int(np.argmax(np.isfinite(semis)))
    idx[idx < 0] = first
    semis = semis[idx]
    form = form[:, :, idx]
    semis = _smooth_hold(semis, int(0.045 * RATE))             # ~90 ms portamento
    for i in range(3):
        for j in range(3):
            form[i, j] = _smooth_hold(form[i, j], int(0.06 * RATE))
    amp = _smooth(level, int(0.04 * RATE))
    since = np.full(n, 10.0)
    scoop = np.zeros(n)
    for (t0, joined) in onsets:
        a = int(round(t0 * RATE))
        dt = t[a:] - t0
        since[a:] = np.minimum(since[a:], dt)
        if vi == 3:                                               # the squeak toy's scoop
            scoop[a:] += -0.8 * np.exp(-dt / 0.045)
        if joined:                                                # re-articulate, gently
            w = np.clip(1 - np.abs(t - t0) / 0.06, 0, 1)
            amp *= 1 - (0.28 if vi == 3 else 0.15) * (0.5 - 0.5 * np.cos(np.pi * w))
    semis = semis + _smooth(scoop, int(0.012 * RATE))
    rate, cents = _VIB[vi]
    bloom = np.clip(since / 0.45, 0, 1) ** 2
    wander = 0.75 + 0.25 * np.sin(TWO_PI * 0.21 * t + r.uniform(0, TWO_PI))
    semis = semis + bloom * wander * cents / 100.0 * np.sin(TWO_PI * rate * t + r.uniform(0, TWO_PI))
    f0 = 440.0 * 2.0 ** (semis / 12.0) * pitch
    return _synth(f0, amp, form, np.full(n, _TILT[vi])) * _GAIN[vi]


def _room(x):
    """A small soft room: four damped combs and two allpasses, each vectorised a delay at a time."""
    def comb(x, d, g):
        yp = np.zeros(len(x) + 1)                 # yp[i + 1] = y[i], so y[-1] reads as silence
        for s in range(0, len(x), d):
            e = min(len(x), s + d)
            fb = yp[s - d + 1: e - d + 1] + yp[s - d: e - d] if s >= d else 0.0   # y[n-d] + y[n-d-1]
            yp[s + 1: e + 1] = x[s:e] + g * 0.5 * fb
        return yp[1:]

    def allpass(x, d, g):
        y = np.zeros(len(x))
        for s in range(0, len(x), d):
            e = min(len(x), s + d)
            xd = x[s - d: e - d] if s >= d else 0.0
            yd = y[s - d: e - d] if s >= d else 0.0
            y[s:e] = -g * x[s:e] + xd + g * yd
        return y

    wet = sum(comb(x, d, g) for d, g in ((797, 0.72), (911, 0.70), (1031, 0.68), (1123, 0.66))) / 4.0
    wet = allpass(allpass(wet, 263, 0.6), 89, 0.6)
    return x + 0.22 * wet


def sing(chorale: dict, pitch: float, seconds: float) -> np.ndarray:
    """The first `seconds` of the chorale as a little choir of rubber ducks, fading out where it
    stops -- at `seconds`, or at the chorale's own end if that comes first."""
    n = int(round(seconds * RATE))
    voices = _score(chorale)
    dry = np.zeros(n)
    for vi, segs in enumerate(voices):
        dry += _voice(segs, n, float(pitch), vi, 1000 + vi)
    last = max((s[1] for segs in voices for s in segs), default=0.0)
    end = max(1, min(n, int(round(last * RATE))))
    fade = min(end, int(2.5 * RATE))
    env = np.ones(n)
    env[end - fade:end] = (0.5 + 0.5 * np.cos(np.linspace(0, np.pi, fade))) ** 1.5
    env[end:] = 0.0
    mix = _room(dry * env)
    fade_in = int(0.03 * RATE)
    mix[:fade_in] *= 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, fade_in))
    tail = min(n, int(0.25 * RATE))                       # close the room's tail without a click
    mix[n - tail:] *= 0.5 + 0.5 * np.cos(np.linspace(0, np.pi, tail))
    mix[-1] = 0.0
    pk = np.abs(mix).max() + 1e-12
    return mix * (10 ** (-7.5 / 20) / pk)
