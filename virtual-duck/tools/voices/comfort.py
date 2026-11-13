"""How a duck sound measures, since nobody can listen to one from a script.

Proxies for what makes a sound pleasant or grating through a phone's speaker:

  sharpness  share of energy above 4 kHz (hiss, shrillness)        lower is softer
  rumble     share below 250 Hz (a phone speaker cannot play it)   wasted energy, buzz
  roughness  envelope modulation 30-150 Hz relative to the mean    buzzy, raspy texture
  attack     10%->90% rise of the envelope, ms                     <3 ms clicks, >60 ms sluggish
  clicks     sample jump at the very start/end                     pops
  centroid   spectral centre, Hz                                   where the sound sits
  pitch_hz   dominant fundamental (autocorrelation), Hz

and, for how loud each sound is next to the others in a run of talk-back, [phone_loudness]: the
peak short-term loudness through a model of a phone speaker. Peak level says little about that —
a low round vowel and a bright chirp at the same peak are heard many dB apart.
"""
import numpy as np

RATE = 24000

def envelope(x, ms=5):
    n = max(1, int(RATE * ms / 1000))
    return np.convolve(np.abs(x), np.ones(n) / n, mode="same")

def measure(x):
    x = np.asarray(x, dtype=np.float64)
    out = {"seconds": round(len(x) / RATE, 3)}
    out["peak_db"] = round(20 * np.log10(np.abs(x).max() + 1e-12), 1)
    out["rms_db"] = round(20 * np.log10(np.sqrt((x ** 2).mean()) + 1e-12), 1)
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), 1 / RATE)
    total = spec.sum() + 1e-20
    out["sharpness"] = round(float(spec[f > 4000].sum() / total), 3)
    out["rumble"] = round(float(spec[f < 250].sum() / total), 3)
    out["centroid"] = int((spec * f).sum() / total)
    env = envelope(x, 2)
    peak = env.max() + 1e-12
    i10 = int(np.argmax(env > 0.1 * peak)); i90 = int(np.argmax(env > 0.9 * peak))
    out["attack_ms"] = round((i90 - i10) / RATE * 1000, 1)
    out["clicks"] = round(float(max(abs(x[0]), abs(x[-1])) / (np.abs(x).max() + 1e-12)), 3)
    e = envelope(x, 1)
    es = np.abs(np.fft.rfft(e - e.mean())) ** 2
    ef = np.fft.rfftfreq(len(e), 1 / RATE)
    out["roughness"] = round(float(es[(ef > 30) & (ef < 150)].sum() / (es.sum() + 1e-20)), 3)
    seg = x[np.argmax(env):][: int(0.05 * RATE)]
    if len(seg) > 400:
        ac = np.correlate(seg, seg, mode="full")[len(seg) - 1:]
        lo, hi = int(RATE / 1200), int(RATE / 80)
        lag = lo + int(np.argmax(ac[lo:hi]))
        out["pitch_hz"] = int(RATE / lag)
    return out

TARGETS = {
    "peak_db": (-12, -6), "rms_db": (-26, -16), "sharpness": (0, 0.06), "rumble": (0, 0.15),
    "roughness": (0, 0.35), "attack_ms": (3, 60), "clicks": (0, 0.02),
}

def problems(m):
    bad = []
    for k, (lo, hi) in TARGETS.items():
        if k in m and not (lo <= m[k] <= hi):
            bad.append(f"{k}={m[k]} (want {lo}..{hi})")
    return bad


# ---------------------------------------------------------------- loudness through a phone speaker

def phone_weighted(x):
    """x as heard through a typical phone speaker: little below ~400 Hz (2nd-order roll-off), a mild
    presence lift around 2-4 kHz, rolling off above ~8 kHz; then roughly A-weighted. Done in the
    frequency domain on the whole sound, which is exact enough for short clips."""
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1 / RATE) + 1e-9
    speaker = (f / 400) ** 2 / (1 + (f / 400) ** 2)            # 2nd-order high-pass
    speaker *= 1 / np.sqrt(1 + (f / 8000) ** 4)                 # roll-off
    speaker *= 1 + 0.4 * np.exp(-((np.log2(f / 3000)) ** 2) / 0.5)  # presence
    f2 = f ** 2                                                  # A-weighting
    a = (12194 ** 2 * f2 ** 2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194 ** 2))
    a /= a[np.argmin(np.abs(f - 1000))]
    return np.fft.irfft(spec * speaker * a, n)

def phone_loudness(x):
    """Peak short-term (50 ms) loudness of x through a phone speaker, dB. What decides whether a sound
    is heard as loud or quiet next to the others in a run of talk-back."""
    y = phone_weighted(x)
    win = int(0.05 * RATE)
    if len(y) < win:
        return float(20 * np.log10(np.sqrt((y ** 2).mean()) + 1e-12))
    p = np.convolve(y ** 2, np.ones(win) / win, mode="valid")
    return float(10 * np.log10(p.max() + 1e-20))

# Per tag, relative to the bank's median chirp/greet/inquire: pecks a little under, coo a touch under.
LOUDNESS_OFFSETS = {"greet": 0, "inquire": 0, "chirp": 0, "alarm": 0, "wheee": -1, "coo": -2, "peck": -4}

