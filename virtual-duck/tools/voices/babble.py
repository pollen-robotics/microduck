"""Babble: the duck talks in animalese.

Every sound is a short word of quick syllables. Each syllable is a soft consonant onset gliding
into a vowel (a/e/i/o/u), sung on the A major pentatonic scale, so any two sounds strung
together stay in key and a talk-back run sounds like a sentence, not a pile of samples.

The voice is additive formant synthesis: every harmonic of the fundamental is weighted by a
three-formant vowel filter, a fixed "beak" resonance near 1.8 kHz, a gentle source tilt and a
soft low-pass, so nothing buzzes and nothing sits above the phone speaker's comfortable band.
The beak is what makes it a duck rather than a generic creature: it sits under every vowel, and
every word ends by closing into it, the small quack-like tail of a duck's call. From a little
before the release the formants close toward a nasal "ng" (F1 down, F2 pinched up into the
beak), the beak rises and the nose's antiresonance hollows out the band just below it. That is
complete by the time the word has faded 6 dB, so the tail is heard while the word is still loud:
there the beak band (1.4-2.2 kHz) stands about 6 dB prouder of the band below it than it would
without. Then, except on a question, the pitch sags a little. A coo closes more softly.

A hello jumps up and comes down onto its note while still loud, three to four semitones under its
peak before it is 12 dB down, so it is heard arrived and never as a question; a question's last
syllable glides up and stays up.

Syllables are articulated: each consonant is a voiced transition (formant loci plus a dip in
level of 10-15 dB), and the vowels keep their own formants even at high pitch, so a word reads
as syllables and not as one smeared glide. Up high, where a harmonic every 700-1100 Hz cannot
tell the back vowels apart, a word that changes vowel sets a front one against a back one. The
only noise is a whisper of breath on "h", shaped by the vowel and kept below 3 kHz, so there is
no hiss or click.

Levels are set by loudness as heard through a phone speaker, not by peak: every sound is brought
to one heard loudness (wheee a touch under, coo a little more; pecks 3.5 dB under since they
repeat all through a meal, which their brevity makes about 4.5 heard; a question 1 dB under and a
chirp half a dB over, since a question's long rising end is heard louder than a chirp's blip at
the same level; blips shorter than the ear's quarter-second get up to 1 dB back), with peaks
kept between -11.8 and -6.3 dBFS and RMS under -16.3 dB. A low round vowel and a bright chirp at
the same peak are heard many dB apart, so peak levelling made the chirps quiet; this way a
talk-back run of chirps, pecks and coos ending in a hello or a question comes out even.

Only numpy. RATE, STYLE, TAGS, render() and sing() are the interface.
"""
import zlib

import numpy as np

RATE = 24000
STYLE = {
    "name": "Babble",
    "description": "Animalese-style duck-speak: quick crisp vowel syllables with a nasal beak twang, "
                   "sung on a major pentatonic scale with expressive hello, question and joy contours.",
}
TAGS = {"greet": 6, "inquire": 6, "chirp": 8, "coo": 6, "peck": 8, "alarm": 5, "wheee": 5}

TONIC = 440.0                      # A4 at pitch 1.0; tunes sit on A major pentatonic
DO, RE, MI, SO, LA, DO2 = 0, 2, 4, 7, 9, 12

# Small-creature vowels: formants F1, F2, F3 (Hz) and their levels g1, g2, g3 (parallel formants).
VOWEL = {
    "a": (780.0, 1300.0, 2600.0, 1.0, 0.70, 0.20),
    "e": (500.0, 1950.0, 2700.0, 1.0, 0.55, 0.23),
    "i": (330.0, 2350.0, 2950.0, 1.0, 0.50, 0.22),
    "o": (500.0, 880.0, 2550.0, 1.0, 0.60, 0.10),
    "u": (350.0, 800.0, 2400.0, 1.0, 0.35, 0.07),
    "m": (270.0, 1050.0, 2350.0, 1.0, 0.15, 0.05),  # closed-mouth hum
}
BW = (150.0, 220.0, 320.0)
BW_PITCH = 0.22                # formants widen a little with pitch (x f0) so a glide stays steady
BEAK = (1800.0, 520.0, 0.35)   # the fixed nasal resonance of the beak: the "duck" colour under every vowel
# Where a word's end closes to: a nasal "ng", the mouth shut (F1 down) and F2 pinched up into the beak.
NASAL = (300.0, 1800.0, 2500.0, 1.0, 0.30, 0.06)
NASAL_BLEND = 0.5              # how far the formants close toward NASAL at a word's end (x the word's nasal)
BEAK_BOOST = 2.0               # how much the beak rises at a word's end (x the word's nasal)
NASAL_ZERO = (1200.0, 450.0, 0.8)   # the nose's antiresonance at a word's end: centre, width (Hz), depth
TAIL_LEAD = 0.06               # the tail starts this long (s) before the release ...
TAIL_FROM = 0.45               # ... but not before this share of the last syllable, so its vowel is heard first
TAIL_FULL = 0.5                # and is fully in by this share of the release: where the word is 6 dB down
FLOOR = 0.04

# Consonant onsets: formant locus (None = the vowel's own), level floor relative to the
# syllable, transition time. All voiced or a short closure.
CONS = {
    "":  (None, 0.30, 0.022),
    "h": (None, 0.18, 0.030),
    "m": ((280.0, 1100.0, 2400.0, 1.0, 0.15, 0.05), 0.15, 0.028),
    "n": ((280.0, 1600.0, 2600.0, 1.0, 0.20, 0.08), 0.15, 0.025),
    "b": ((300.0, 900.0, 2300.0), 0.12, 0.020),
    "p": ((300.0, 900.0, 2300.0), 0.06, 0.016),
    "d": ((300.0, 1700.0, 2700.0), 0.12, 0.020),
    "t": ((300.0, 1700.0, 2700.0), 0.06, 0.016),
    "g": ((300.0, 2000.0, 2300.0), 0.12, 0.022),
    "k": ((300.0, 2100.0, 2400.0), 0.06, 0.018),
    "w": ((330.0, 750.0, 2300.0), 0.20, 0.032),
    "y": ((300.0, 2450.0, 3000.0), 0.22, 0.030),
    "l": ((360.0, 1150.0, 2500.0), 0.20, 0.026),
}

_PHASES = np.pi * np.arange(1, 66) ** 2 / 9.0   # fixed, spread harmonic phases (low crest)


# ----------------------------------------------------------------------------- helpers

def _ss(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _points(pts, u):
    """Smoothly interpolate evenly spaced control points over u in [0, 1]."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 1:
        return np.full_like(u, pts[0])
    pos = np.clip(u, 0.0, 1.0) * (len(pts) - 1)
    i = np.minimum(pos.astype(int), len(pts) - 2)
    return pts[i] + (pts[i + 1] - pts[i]) * _ss(pos - i)


def _smooth(x, ms):
    n = int(RATE * ms / 1000.0)
    if n < 3:
        return x
    k = np.hanning(n + 2)[1:-1]
    k /= k.sum()
    pad = np.pad(x, (n, n), mode="edge")
    return np.convolve(pad, k, mode="same")[n:-n]


def _locus(loc, Fv):
    """A consonant locus as six formant rows; loci without their own levels borrow the vowel's."""
    if loc is None:
        return Fv
    loc = np.asarray(loc, dtype=float)
    return loc if len(loc) == 6 else np.concatenate([loc, Fv[3:]])


def _seed(*parts):
    return zlib.crc32("/".join(str(p) for p in parts).encode())


def _synth(f0, F, bright, tilt=0.5, beak=None, hp=None, zero=None, chunk=24000):
    """Unit-RMS formant voice. f0: (N,) Hz; F: (6, N) formant freqs then levels; bright: low-pass
    corner; beak: the beak resonance's level, per sample or one number (default BEAK's); hp: a
    gentle high-pass corner (Hz) for voices whose fundamental sits below what a phone can play;
    zero: the depth (0..1) of NASAL_ZERO's antiresonance, per sample or one number (default none)."""
    N = len(f0)
    phase = 2.0 * np.pi * np.cumsum(f0) / RATE
    bright = np.broadcast_to(np.asarray(bright, dtype=float), (N,))
    bf, bb, bg = BEAK
    beak = np.broadcast_to(np.asarray(bg if beak is None else beak, dtype=float), (N,))
    zf, zb, _ = NASAL_ZERO
    out = np.zeros(N)
    for s in range(0, N, chunk):
        sl = slice(s, min(N, s + chunk))
        f = f0[sl][None, :]
        K = int(min(64, max(4, 7500.0 / max(f.min(), 120.0))))
        k = np.arange(1, K + 1, dtype=float)[:, None]
        fk = k * f
        G = FLOOR + beak[sl][None, :] / np.sqrt(1.0 + ((fk * fk - bf * bf) / (bb * fk)) ** 2)
        for i in range(3):
            Fi = F[i, sl][None, :]
            Bi = np.maximum(BW[i], BW_PITCH * f)                 # a little wider at high pitch
            G = G + F[3 + i, sl][None, :] / np.sqrt(1.0 + ((fk * fk - Fi * Fi) / (Bi * fk)) ** 2)
        if zero is not None:
            z = np.broadcast_to(np.asarray(zero, dtype=float), (N,))[sl][None, :]
            G = G * (1.0 - z / np.sqrt(1.0 + ((fk * fk - zf * zf) / (zb * fk)) ** 2))
        H = G * k ** (-tilt) / (1.0 + (fk / bright[sl][None, :]) ** 4)
        H = H * (1.0 - _ss((fk - 6000.0) / 1500.0))          # nothing above ~7.5 kHz
        if hp:
            H = H * (fk / hp) ** 2 / (1.0 + (fk / hp) ** 2)
        H = H / (np.sqrt((H * H).sum(axis=0) / 2.0) + 1e-12)  # constant loudness per instant
        out[sl] = (H * np.sin(k * phase[sl][None, :] + _PHASES[:K, None])).sum(axis=0)
    return out


# ----------------------------------------------------------------------------- levels

LOUD_REF = -16.0   # heard loudness of a hello, a question, a chirp or an alarm (dB, phone-weighted)
LOUD_PER_OCTAVE = 4.0   # a higher voice is a little brighter in the ear, a lower one a little softer
# Per tag, re LOUD_REF. A question's long rising end is heard about 2 phon louder than a chirp at the
# same 50 ms level, so it sits 1 dB under and the chirps half a dB over; a peck is so short that the
# ear sums less of it, so 3.5 dB under by this measure is heard about 4.5 under.
LOUD = {"greet": 0.0, "inquire": -1.0, "chirp": 0.5, "alarm": 0.0, "wheee": -1.0, "coo": -2.0, "peck": -3.5}


def _heard(x):
    """How loud x is through a phone speaker: little below ~400 Hz, a presence lift near 3 kHz,
    nothing much above 8 kHz, then roughly A-weighted; the loudest 50 ms, in dB. Peaks say little
    about this: a low round coo and a bright chirp at the same peak are heard many dB apart."""
    n = len(x)
    f = np.fft.rfftfreq(n, 1.0 / RATE) + 1e-9
    w = (f / 400.0) ** 2 / (1.0 + (f / 400.0) ** 2) / np.sqrt(1.0 + (f / 8000.0) ** 4)
    w = w * (1.0 + 0.4 * np.exp(-np.log2(f / 3000.0) ** 2 / 0.5))
    f2 = f * f
    a = 12194.0 ** 2 * f2 * f2 / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    a /= 12194.0 ** 2 * 1e12 / ((1e6 + 20.6 ** 2) * np.sqrt((1e6 + 107.7 ** 2) * (1e6 + 737.9 ** 2)) * (1e6 + 12194.0 ** 2))
    y = np.fft.irfft(np.fft.rfft(x) * w * a, n)
    win = int(0.05 * RATE)
    if n < win:
        return float(10 * np.log10(np.mean(y * y) + 1e-20))
    return float(10 * np.log10(np.convolve(y * y, np.ones(win) / win, mode="valid").max() + 1e-20))


def _brevity(x):
    """The ear sums loudness over about a quarter of a second, so a blip whose loud part is
    shorter than that is heard a little softer than its level says: up to 1 dB back for it."""
    w = int(0.01 * RATE)
    e = np.convolve(x * x, np.ones(w) / w, mode="same")
    T = np.count_nonzero(e > 0.1 * e.max()) / RATE
    return float(np.clip(1.5 * np.log2(0.25 / max(T, 1e-3)), 0.0, 1.0))


def _level(x, loud=0.0, peak_hi=-6.3, peak_lo=-11.8, rms_hi=-16.3, rms_lo=-25.5):
    """Bring x to the heard loudness LOUD_REF + loud, within comfortable peak and RMS bounds."""
    pk = np.abs(x).max() + 1e-12
    rms = np.sqrt(np.mean(x * x)) + 1e-12
    g = 10 ** ((LOUD_REF + loud + _brevity(x) - _heard(x)) / 20.0)
    g_hi = min(10 ** (peak_hi / 20.0) / pk, 10 ** (rms_hi / 20.0) / rms)
    g_lo = max(10 ** (peak_lo / 20.0) / pk, 10 ** (rms_lo / 20.0) / rms)
    return x * min(max(g, g_lo), g_hi)


def _edge_fade(x, ms=3.0):
    n = min(int(RATE * ms / 1000.0), len(x) // 4)
    if n > 1:
        r = 0.5 - 0.5 * np.cos(np.linspace(0.0, np.pi, n))
        x[:n] *= r
        x[-n:] *= r[::-1]
    x[0] = 0.0
    x[-1] = 0.0
    return x


# ----------------------------------------------------------------------------- words

def _word(sylls, pitch, rng, bright=3400.0, tilt=0.5, scoop=0.5, vib_rate=5.8, loud=0.0,
          droop=0.7, nasal=1.0, onset=None, hp=None):
    """Render a list of syllables into one continuous voiced word.

    Each syllable: c (onset consonant), v (vowel), p (pitch points, semitones from the tonic),
    d (seconds), a (level), e (level points across the syllable), coda, vib (depth, semitones),
    rel (release seconds, last syllable only).
    Word options: loud (heard level re LOUD_REF, dB); droop (semitones the pitch sags at the
    word's end) and nasal (how far the end closes into the beak); onset (seconds for the first
    syllable to reach full level, overriding its consonant's slower rise: a crisp nibble).
    """
    ns = [max(1, int(round(s["d"] * RATE))) for s in sylls]
    N = sum(ns)
    semis = np.zeros(N)
    amp = np.zeros(N)
    vdepth = np.zeros(N)
    nas = np.zeros(N)
    sag = np.zeros(N)
    F = np.zeros((6, N))
    breaths = []
    pos = 0
    for i, s in enumerate(sylls):
        n = ns[i]
        tau = np.arange(n) / RATE
        d = n / RATE
        u = tau / d
        c = s.get("c", "")
        loc, floor, tr = CONS[c]
        first, last = i == 0, i == len(sylls) - 1
        A = s.get("a", 1.0)

        Fv = np.array(VOWEL[s["v"]])
        Floc = _locus(loc, Fv)
        w = _ss(tau / tr)
        if c == "h":
            breaths.append((pos, A, Fv))
        Ft = Floc[:, None] + (Fv - Floc)[:, None] * w[None, :]

        if first:
            if onset is not None:
                on = _ss(tau / onset)
            elif c == "":
                on = _ss(tau / 0.014)
            elif c == "h":
                on = _ss(tau / 0.028)
            else:
                on = _ss(tau / 0.007) * (floor + (1.0 - floor) * w)
        else:
            on = floor + (1.0 - floor) * w
        env = A * _points(s.get("e", [1.0, 0.9]), u) * on

        coda = s.get("coda")
        tail = 0.0
        if coda:
            cloc, cfloor, ctr = CONS[coda]
            tl = min(0.45 * d, ctr * 1.4)
            b = _ss((tau - (d - tl)) / tl)
            if cloc is not None:
                Ft = Ft * (1 - b) + _locus(cloc, Fv)[:, None] * b
            env = env * (1 - b) + (0.0 if last else A * cfloor) * b
            tail = tl
        elif not last:
            nxt = sylls[i + 1]
            _, nfloor, ntr = CONS[nxt.get("c", "")]
            tl = min(0.35 * d, max(ntr, 0.02))
            b = _ss((tau - (d - tl)) / tl)
            env = env * (1 - b) + nxt.get("a", 1.0) * nfloor * b
        if last and not coda:
            rel = s.get("rel", min(0.5 * d, 0.10))
            b = _ss((tau - (d - rel)) / rel)
            env = env * (1 - b)
            tail = rel
        if last:
            # The duck's tail: the end of the word closes into the beak. It starts before the
            # release and is complete half way down it, so it is heard while the word is still
            # loud rather than in the last whisper of the decay. The pitch sags a little only in
            # the release itself, once the word has landed on its note.
            t0 = max(d - tail - TAIL_LEAD, TAIL_FROM * d)
            t1 = max(d - (1.0 - TAIL_FULL) * tail, t0 + 0.02)
            nas[pos:pos + n] = _ss((tau - t0) / (t1 - t0))
            Ld = min(0.5 * d, float(np.clip(0.8 * tail, 0.04, 0.12)))
            sag[pos:pos + n] = _ss((tau - (d - Ld)) / Ld)

        p = _points(s["p"], u) - s.get("scoop", scoop) * (1.0 - _ss(tau / 0.03))
        semis[pos:pos + n] = p
        amp[pos:pos + n] = env
        F[:, pos:pos + n] = Ft
        vdepth[pos:pos + n] = s.get("vib", 0.0) * _ss((tau - 0.09) / 0.12)
        pos += n

    t = np.arange(N) / RATE
    semis = semis - droop * sag
    semis = _smooth(semis, 16)
    semis = semis + vdepth * np.sin(2 * np.pi * vib_rate * t + rng.uniform(0, 2 * np.pi))
    semis = semis + 0.05 * np.sin(2 * np.pi * 1.7 * t + rng.uniform(0, 6.3))
    jit = _smooth(rng.standard_normal(N), 35)                 # a living voice never holds dead still
    semis = semis + 0.035 * jit / (jit.std() + 1e-12)
    k = NASAL_BLEND * nasal * nas
    F = F * (1.0 - k) + np.array(NASAL)[:, None] * k
    F = np.vstack([_smooth(F[j], 6) for j in range(6)])
    F[:3] *= pitch ** 0.25
    amp = _smooth(amp, 2.5)
    f0 = TONIC * pitch * 2.0 ** (semis / 12.0)
    nas = _smooth(nas, 6)
    beak = BEAK[2] * (1.0 + BEAK_BOOST * nasal * nas)
    zero = NASAL_ZERO[2] * min(nasal, 1.0) * nas

    x = _synth(f0, F, bright, tilt, beak, hp, zero)
    # Even out how the waveform shape (vowel, pitch) changes its short-term peak, so the level
    # the word was designed with is the level you hear.
    e = np.convolve(np.abs(x), np.ones(48) / 48, mode="same")
    win = min(96, N)
    P = np.lib.stride_tricks.sliding_window_view(np.pad(e, (win // 2, win - win // 2 - 1), mode="edge"), win).max(axis=1)
    P = _smooth(P, 20)
    x = amp * x / (P + 1e-9)
    for p0, A, Fv in breaths:
        x = _breath(x, p0, A, Fv, pitch, rng)
    return _edge_fade(_level(x, loud + LOUD_PER_OCTAVE * np.log2(pitch)))


def _breath(x, p0, A, Fv, pitch, rng, level=0.08, dur=0.045):
    """A whisper of breath for "h": noise shaped by the coming vowel, soft and below 3 kHz."""
    L = min(int(dur * RATE), len(x) - p0)
    if L < 16:
        return x
    pad = 256
    X = np.fft.rfft(rng.standard_normal(L + 2 * pad))
    fr = np.fft.rfftfreq(L + 2 * pad, 1.0 / RATE) + 1e-3
    G = np.zeros_like(fr)
    for i, b in enumerate((300.0, 400.0, 500.0)):
        Fi = Fv[i] * pitch ** 0.25
        G += Fv[3 + i] / np.sqrt(1.0 + ((fr * fr - Fi * Fi) / (b * fr)) ** 2)
    G *= (fr / 600.0) ** 2 / (1.0 + (fr / 600.0) ** 2) / (1.0 + (fr / 2800.0) ** 4)
    nz = np.fft.irfft(X * G, L + 2 * pad)[pad:pad + L]
    nz /= np.sqrt(np.mean(nz * nz)) + 1e-12
    tt = np.arange(L) / RATE
    env = _ss(tt / 0.008) * (1.0 - _ss((tt - 0.012) / (dur - 0.014)))
    x = x.copy()
    x[p0:p0 + L] += level * A * env * nz
    return x


def _jitter(sylls, rng, dur=0.06, pit=0.12):
    off = rng.uniform(-pit, pit)
    out = []
    for s in sylls:
        s = dict(s)
        s["d"] = s["d"] * (1.0 + rng.uniform(-dur, dur))
        s["p"] = [p + off for p in s["p"]]
        out.append(s)
    return out


def S(c, v, p, d, a=1.0, **kw):
    s = {"c": c, "v": v, "p": p, "d": d, "a": a}
    s.update(kw)
    return s


# Per-tag word options; a bank entry's own options are added on top.
TAG_OPTS = {
    "greet": {"droop": 0.9},                            # a hello settles: the tail sags a touch more
    "inquire": {"droop": 0.0, "nasal": 0.9},            # a question ends up: the tail only closes, never sags
    "chirp": {"bright": 2800, "droop": 0.4},
    "coo": {"bright": 2500, "tilt": 0.85, "droop": 0.8, "hp": 380, "nasal": 0.7},  # warmer, rounder, closing softly;
                                                        # eased off below what a phone plays
    "peck": {"bright": 2400, "tilt": 0.7, "onset": 0.0075, "droop": 0.4},  # a crisp little nibble, darker than speech
    "alarm": {"bright": 3100, "vib_rate": 9.0, "droop": 0.5},              # a flustered flutter, kept below shrill
    "wheee": {"bright": 2800},
}

# Each bank entry: (syllables, word options). Pitches are pentatonic degrees in semitones.
BANK = {
    # Hello: a quick pick-up, then a jump up that rings and comes down while still loud: a third
    # onto LA from the octave, or a tone onto SO trailing lower. It lands on its note, so it is
    # heard arrived, never asking (3.3-4.1 semitones under its peak by the time it is 12 dB down).
    "greet": [
        ([S("h", "i", [SO, SO + .4], .085), S("y", "a", [DO2, DO2 + .5, LA, LA], .21, .96, e=[1, .85, .7], vib=.1)], {}),
        ([S("h", "e", [MI, MI + .4], .085), S("l", "o", [LA, LA + .5, SO, SO - 1], .22, .95, e=[1, .85, .7], vib=.1)], {}),
        ([S("h", "i", [SO, SO + .3], .08), S("y", "i", [DO2, DO2 + .6, LA, LA], .20, .95, e=[1, .85, .7], vib=.1)], {}),
        ([S("w", "a", [MI, MI + .4], .085), S("y", "a", [LA, LA + .6, SO, SO - 1], .21, .95, e=[1, .85, .7], vib=.1)], {}),
        ([S("h", "i", [MI, MI + .3], .075), S("y", "a", [SO, SO + .2], .075, .95, e=[1, .85]),
          S("y", "a", [DO2, DO2 + .5, LA, LA], .19, .95, e=[1, .85, .7], vib=.1)], {}),
        ([S("y", "a", [SO, SO + .3], .085), S("h", "i", [DO2, DO2 + .7, LA, LA], .20, .95, e=[1, .85, .7], vib=.1)], {}),
    ],
    # Question: a plain first syllable, then the last one glides up and stays up.
    "inquire": [
        ([S("w", "a", [MI, MI + .3], .09, e=[1, .85]), S("n", "i", [MI, SO, DO2], .21, .95, e=[.95, 1, .9], rel=.07)], {}),
        ([S("", "u", [RE, RE + .3], .09, e=[1, .85]), S("n", "e", [RE, MI, LA + .5], .21, .95, e=[.95, 1, .9], rel=.07)], {}),
        ([S("m", "e", [RE, RE + .3], .085, e=[1, .85]), S("l", "a", [MI, SO, LA + 1], .21, .95, e=[.95, 1, .9], rel=.07)], {}),
        ([S("n", "i", [MI, MI + .2], .08, e=[1, .85]), S("n", "i", [MI, SO, DO2], .20, .95, e=[.95, 1, .9], rel=.07)], {}),
        ([S("w", "a", [SO, SO], .075, e=[1, .85]), S("w", "a", [MI, MI], .075, .92, e=[1, .85]),
          S("w", "i", [MI, SO, DO2], .20, .95, e=[.95, 1, .9], rel=.07)], {}),
        ([S("m", "o", [RE, RE + .2], .09, e=[1, .85]), S("w", "e", [DO, MI, LA], .21, .95, e=[.95, 1, .9], rel=.07)], {}),
    ],
    # Happy little chirps: high, quick, bouncy. Up here the back vowels a/o/u blur into one another
    # (a harmonic every 700-1100 Hz says little about where F1 and F2 sit), so a word that changes
    # vowel sets a front one (i/e) against a back one (a/o).
    "chirp": [
        ([S("w", "i", [SO, DO2, DO2 - .6], .13, e=[1, .75])], {}),
        ([S("p", "i", [LA, LA + .4], .07, e=[1, .75]), S("p", "i", [DO2, DO2 + .4], .085, .95, e=[1, .7])], {}),
        ([S("y", "i", [DO2, DO2 + .5, LA + 1], .11, e=[1, .8], coda="p")], {}),
        ([S("d", "i", [LA, LA + .4], .07, e=[1, .8]), S("l", "i", [DO2, DO2 + .4, DO2 - .5], .10, .95, e=[1, .7])], {}),
        ([S("w", "i", [MI, LA, DO2], .16, e=[1, .8])], {}),
        ([S("b", "e", [DO2, DO2 + .5, LA + 1], .10, e=[1, .7])], {}),
        ([S("g", "i", [SO, SO + .5], .065, e=[1, .8]), S("l", "a", [LA, LA + .8, LA], .095, .95, e=[1, .7])], {}),
        ([S("y", "a", [LA, DO2 - .5, LA], .14, e=[1, .75])], {}),
    ],
    # Contented coo: rounded vowels, lower, gentle vibrato, a slow settling fall.
    "coo": [
        ([S("m", "u", [MI, MI + .4, RE], .30, e=[1, .9]), S("w", "o", [RE, RE + .2, DO], .36, .88, vib=.15, rel=.16)], {}),
        ([S("", "o", [RE, MI, MI - .4], .25, e=[1, .9]), S("w", "u", [RE, DO], .40, .88, vib=.15, rel=.18)], {}),
        ([S("m", "o", [DO, MI, MI, RE], .62, e=[1, .95, .85], vib=.15, rel=.2)], {}),
        ([S("n", "u", [MI, MI], .20, e=[1, .92]), S("n", "u", [RE, RE], .20, .9),
          S("w", "o", [DO, DO, DO - .5], .32, .86, vib=.12, rel=.16)], {}),
        ([S("w", "u", [SO, MI], .28, e=[1, .9]), S("", "o", [RE, DO], .42, .88, vib=.15, rel=.2)], {}),
        ([S("h", "o", [MI, MI + .6, MI], .28, e=[1, .9]), S("m", "u", [RE, RE, DO], .40, .86, vib=.14, rel=.2)], {}),
    ],
    # A tiny nibble: one plucked syllable with a small downward bend, softer and darker than speech.
    # Each variant sits on a different note, so a meal of pecks becomes a little tune.
    "peck": [
        ([S("n", "i", [DO2, LA + 1], .085, e=[1, .5], coda="p")], {}),
        ([S("b", "e", [LA, SO + 1], .08, e=[1, .45], rel=.04)], {}),
        ([S("n", "o", [SO, MI + 1.5], .085, e=[1, .5], coda="m")], {}),
        ([S("n", "i", [LA, LA], .06, e=[1, .7]), S("n", "i", [DO2, DO2 - 1], .075, .9, e=[1, .5], coda="p")], {}),
        ([S("d", "o", [SO, SO - 1], .08, e=[1, .45], rel=.04)], {}),
        ([S("b", "i", [MI, MI - .7], .075, e=[1, .5], coda="p")], {}),
        ([S("m", "e", [LA, LA - 1], .08, e=[1, .5], rel=.04)], {}),
        ([S("p", "i", [SO, SO + .3], .055, e=[1, .7]), S("p", "i", [MI, MI - .5], .07, .9, e=[1, .5], rel=.035)], {}),
    ],
    # Startled "eek!": a sudden 5-7 semitone scoop up, a flustered flutter, then a tumble down.
    # Rounded, not shrill.
    "alarm": [
        ([S("", "i", [SO - 1.5, DO2 + .5, DO2 + .3, DO2, LA, MI + .5], .32, e=[1, .95, .85, .7], vib=.35, scoop=1.0, rel=.1)], {}),
        ([S("w", "a", [SO - 1.5, DO2 + 1, DO2 + .5, LA, MI], .30, e=[1, .9, .8, .65], vib=.3, rel=.1)], {}),
        ([S("", "i", [SO - 1.5, DO2 + .6, DO2 + .4], .12, coda="p"), S("", "i", [LA + .5, LA, MI + 1], .16, .92, rel=.08)], {}),
        ([S("y", "a", [SO, SO + .3], .08, e=[1, .85]), S("y", "a", [DO2 + .5, DO2 + 1.2, LA + .5, MI + .5], .22, .95, vib=.3, rel=.09)], {}),
        ([S("", "o", [MI, LA + 1.5, LA + .7, SO, RE], .30, e=[1, .9, .8, .65], vib=.3, scoop=1.0, rel=.1)], {}),
    ],
    # Wheee: a quick swoop up, floating at the top like the arc of a jump, then a little drop.
    # High like the chirps, so a change of vowel is front against back: "ya-hee", "wee-ho".
    "wheee": [
        ([S("w", "i", [DO, SO, DO2 + .8, DO2, DO2, DO2 - .4], .95, e=[1, 1, 1, .95, .9], vib=.25, rel=.22)], {"vib_rate": 6.2}),
        ([S("w", "i", [RE, SO, DO2 + .5, DO2 + .3], .55, e=[1, 1]), S("w", "u", [LA, MI, DO], .5, .9, vib=.2, rel=.22)],
         {"vib_rate": 6.0}),
        ([S("y", "a", [MI, SO, LA], .32, e=[1, .95]), S("h", "i", [DO2, DO2 + .4, DO2, LA], .68, .95, vib=.25, rel=.22)],
         {"vib_rate": 6.0}),
        ([S("w", "i", [DO, DO + .3], .12), S("y", "i", [MI, MI + .3], .12, .97), S("y", "i", [SO, SO + .3], .12, .96),
          S("y", "i", [LA, LA + .3], .12, .95), S("y", "i", [DO2, DO2 + .5, DO2], .55, .95, vib=.25, rel=.22)],
         {"vib_rate": 6.0}),
        ([S("w", "i", [MI, LA, DO2 + .5, DO2], .5, e=[1, 1]), S("h", "o", [LA, DO2, DO2 + .4, LA], .55, .92, vib=.22, rel=.22)],
         {"vib_rate": 6.0}),
    ],
}

for _tag, _n in TAGS.items():
    assert len(BANK[_tag]) == _n, _tag


def render(tag, variant, pitch=1.0):
    """One sound: tag in TAGS, variant in range(TAGS[tag]), pitch 0.8 (low) .. 1.25 (high)."""
    sylls, opts = BANK[tag][variant % len(BANK[tag])]
    rng = np.random.default_rng(_seed("babble", tag, variant))
    sylls = _jitter(sylls, rng)
    return _word(sylls, float(pitch), rng, loud=LOUD[tag], **{**TAG_OPTS[tag], **opts})


# ----------------------------------------------------------------------------- the chorale

_VOWMAP = {"oo": "u", "ah": "a", "mm": "m", "ee": "i", "eh": "e", "oh": "o"}
_DYN = {"pp": 0.45, "p": 0.55, "mp": 0.66, "mf": 0.76, "f": 0.88, "ff": 1.0}
_LEAD_CONS = ["l", "n", "y", "l", "w", "n", "l", "y"]


def _midi(name):
    base = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}[name[0].upper()]
    rest, acc = name[1:], 0
    while rest and rest[0] in "#b":
        acc += 1 if rest[0] == "#" else -1
        rest = rest[1:]
    return 12 * (int(rest) + 1) + base + acc


def _score(chorale, seconds):
    """Per voice (bass..soprano): list of (t0, t1, midi, vowel, level, hum); and the chorale's length."""
    spb = 60.0 / float(chorale.get("bpm", 60.0))
    names = chorale.get("voices_low_to_high", ["bass", "tenor", "alto", "soprano"])
    segs = [[] for _ in names]
    t = 0.0
    for ev in chorale["events"]:
        typ = ev.get("type")
        beats = ev.get("beats")
        if beats is None and typ == "solo":                     # a solo lasts as long as its notes
            beats = sum(float(nt.get("beats", 0)) for nt in ev.get("notes", []))
        dur = float(beats or 0) * spb
        lev = _DYN.get(ev.get("dynamic", "mf"), 0.76)
        vow = _VOWMAP.get(ev.get("vowel", "ah"), "a")
        if typ == "build":
            order = list(range(len(names)))
            if ev.get("from_top"):
                order.reverse()
            for rank, vi in enumerate(order):
                nm = ev["voices"][vi]
                if nm != "-":
                    t0 = t + rank * float(ev.get("stagger", 0.5)) * spb
                    if t0 < t + dur:
                        segs[vi].append((t0, t + dur, _midi(nm), vow, lev, False))
        elif typ == "chord":
            for vi, nm in enumerate(ev["voices"]):
                if nm != "-":
                    segs[vi].append((t, t + dur, _midi(nm), vow, lev, False))
        elif typ == "solo":
            part = names.index(ev["part"])
            for vi, nm in enumerate(ev.get("under", [])):
                if vi != part and nm != "-":
                    segs[vi].append((t, t + dur, _midi(nm), "m", lev * 0.8, True))
            tt = t
            for nt in ev["notes"]:
                d = float(nt["beats"]) * spb
                segs[part].append((tt, tt + d, _midi(nt["note"]), _VOWMAP.get(nt.get("vowel", "ah"), "a"), lev, False))
                tt += d
        t += dur
        if t > seconds + 2.0:
            break
    return segs, t


def _ffill(row, default):
    row = row.copy()
    ok = ~np.isnan(row)
    if not ok.any():
        return np.full_like(row, default)
    idx = np.where(ok, np.arange(len(row)), 0)
    np.maximum.accumulate(idx, out=idx)
    row = row[idx]
    row[: int(np.argmax(ok))] = row[int(np.argmax(ok))]
    return row


def _line(segs, N, pitch, lead, rng):
    """One voice of the choir as a legato line."""
    semis = np.full(N, np.nan)
    amp = np.zeros(N)
    F = np.full((6, N), np.nan)
    bright = np.full(N, 3300.0 if lead else 2500.0)
    tsince = np.zeros(N)
    dip = np.ones(N)
    loci = []  # (a, b, locus, bump)
    ci = 0
    prev_end = None
    segs = sorted(segs)
    for j, (t0, t1, midi, vow, lev, hum) in enumerate(segs):
        i0, i1 = int(t0 * RATE), min(N, int(t1 * RATE))
        if i0 >= N or i1 <= i0:
            continue
        n = i1 - i0
        tau = np.arange(n) / RATE
        legato_in = prev_end is not None and abs(prev_end - t0) < 1e-6
        legato_out = j + 1 < len(segs) and abs(segs[j + 1][0] - t1) < 1e-6
        semis[i0:i1] = midi + 12
        F[:, i0:i1] = np.array(VOWEL[vow])[:, None]
        if hum:
            bright[i0:i1] = 1700.0
        tsince[i0:i1] = tau
        env = np.full(n, lev)
        if lead and n > RATE * 0.5:                       # a gentle swell on long notes
            env *= 1.0 + 0.07 * np.sin(np.pi * tau / (n / RATE))
        if not legato_in:
            env *= _ss(tau / (0.07 if lead else 0.12))
        if not legato_out:
            rel = min(0.18, 0.4 * n / RATE)
            env *= 1.0 - _ss((tau - (n / RATE - rel)) / rel)
        amp[i0:i1] = env
        # A soft consonant on every new lead note ("la", "na", "ya") makes the tune sing in
        # syllables; the lower voices only lean a little into each change of note.
        if lead and not hum:
            c = "w" if vow == "u" else _LEAD_CONS[ci % len(_LEAD_CONS)]
            ci += 1
            loc, floor, _ = CONS[c]
            w = int(0.09 * RATE)
            a = max(0, i0 - int(0.015 * RATE))
            b = min(N, a + w)
            bump = np.hanning(w + 2)[1:-1][: b - a]
            if legato_in:
                dip[a:b] *= 1.0 - (1.0 - max(floor, 0.5)) * bump
            loci.append((a, b, loc, bump))
        elif legato_in and segs[j - 1][2] != midi:
            w = int(0.08 * RATE)
            a = max(0, i0 - int(0.03 * RATE))
            b = min(N, a + w)
            dip[a:b] *= 1.0 - 0.15 * np.hanning(w + 2)[1:-1][: b - a]
        prev_end = t1
    if np.all(np.isnan(semis)):
        return np.zeros(N)
    semis = _ffill(semis, 60.0)
    F = np.vstack([_ffill(F[k], VOWEL["u"][k]) for k in range(6)])
    for a, b, loc, bump in loci:
        L = _locus(loc, F[:, a])
        F[:, a:b] = F[:, a:b] * (1 - bump) + L[:, None] * bump
    semis = _smooth(semis, 45 if lead else 60)
    t = np.arange(N) / RATE
    depth = (0.2 if lead else 0.08) * _ss((tsince - 0.25) / 0.35)
    semis = semis + depth * np.sin(2 * np.pi * rng.uniform(4.9, 5.6) * t + rng.uniform(0, 6.3))
    semis = semis + rng.uniform(-0.05, 0.05)                 # a few cents apart: a choir, not a chord patch
    F = np.vstack([_smooth(F[k], 30) for k in range(6)])
    F[:3] *= (1.0 if lead else 0.95) * pitch ** 0.25
    amp = _smooth(amp * dip, 25)
    bright = _smooth(bright, 40)
    f0 = 440.0 * 2.0 ** ((semis - 69.0) / 12.0) * pitch
    return amp * _synth(f0, F, bright, tilt=0.55 if lead else 0.9)


def sing(chorale, pitch=1.0, seconds=24.0):
    """The robot's chorale as a little duck choir: soprano carries the tune in syllables, the others hum underneath."""
    segs, total = _score(chorale, seconds)
    cut = seconds < total                                    # cut short: fade out; whole song: let it end
    seconds = seconds if cut else total + 0.25
    N = int(round(seconds * RATE))
    rng = np.random.default_rng(_seed("babble", "choir"))
    mix = np.zeros(N)
    gains = [0.42, 0.34, 0.38, 1.0]                          # bass, tenor, alto, soprano
    for vi, sg in enumerate(segs):
        lead = vi == len(segs) - 1
        g = gains[vi] if vi < len(gains) else 0.35
        mix += g * _line(sg, N, float(pitch), lead, rng)
    t = np.arange(N) / RATE
    fade = min(2.2, 0.15 * seconds) if cut else 0.25
    mix *= 1.0 - _ss((t - (seconds - fade)) / fade)
    mix = 10 ** (-8.0 / 20.0) * mix / (np.abs(mix).max() + 1e-12)
    return _edge_fade(mix, 10.0)
