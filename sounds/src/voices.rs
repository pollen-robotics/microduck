//! Recipes for each tag — the Python `voices.py`, line for line where it matters.
//!
//! Recipes paint with the personality's traits — pitch center, register, glide bias,
//! harmonic tilt/formant, quackiness, warble — so the *same* recipe on two different seeds
//! gives two recognisably different ducks.

use crate::personality::Personality;
use crate::rng::Rng;
use crate::synth as s;
use crate::synth::SR;

/// Attack time in seconds, modulated by `attack_sharpness`. `snappy = 1` for percussive
/// recipes; lower for soft recipes that should still be soft on snappy ducks.
fn attack(p: &Personality, dur: f64, snappy: f64) -> f64 {
    let soft = 0.04 * dur;
    let sharp = 0.003 * dur;
    (soft + (sharp - soft) * p.attack_sharpness * snappy).max(0.001)
}

/// Shared core: harmonic osc + vibrato + jitter + (optional) AM buzz + breath.
fn voice(
    p: &Personality,
    t: &[f32],
    freq: &[f32],
    rng: &mut Rng,
    am_scale: f64,
    breath_scale: f64,
) -> Vec<f32> {
    let vib = s::vibrato(
        t,
        p.vibrato_rate_hz,
        p.vibrato_depth,
        rng.uniform(0.0, std::f64::consts::TAU),
    );
    let jit = s::jitter(t, p.jitter_depth, rng);
    let f: Vec<f32> = freq
        .iter()
        .zip(&vib)
        .zip(&jit)
        .map(|((&f, &v), &j)| f * v * j)
        .collect();
    let phase = s::phase_from_freq(&f);
    let mut body = s::harmonic_osc(&phase, &p.harmonics());

    // Quackiness gates the AM buzz: pure-tone ducks have ~no AM.
    let am_d = p.am_depth * am_scale * p.quackiness;
    if am_d > 0.01 {
        for (b, &x) in body.iter_mut().zip(t) {
            let am = 1.0
                - am_d * (0.5 + 0.5 * (std::f64::consts::TAU * p.am_rate_hz * f64::from(x)).sin());
            *b *= am as f32;
        }
    }

    let breath = p.breath * breath_scale;
    if breath > 0.0 {
        for (b, n) in body.iter_mut().zip(s::pink_noise(t.len(), rng)) {
            *b += breath as f32 * n;
        }
    }
    body
}

pub fn alarm(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("alarm", variant);
    let dur = (0.20 + 0.12 * rng.random()) / p.speed;
    let t = s::t_axis(dur);
    // Raised relative to this duck's center, but stays in honk range; spread controls how
    // high it climbs.
    let f0 = p.pitch_center_hz * (1.25 + 0.35 * p.pitch_spread) * (0.94 + 0.12 * rng.random());
    let peak_mul = 1.15 + 0.25 * p.pitch_spread + 0.10 * rng.random();
    let fall_mul = 0.75 + 0.20 * (1.0 - p.pitch_spread);
    let freq = s::lerp(
        &t,
        &[(0.0, f0), (0.05 * dur, f0 * peak_mul), (dur, f0 * fall_mul)],
    );
    let env = s::expdecay(&t, attack(p, dur, 1.0), dur * (0.40 + 0.20 * rng.random()));
    let mut sig: Vec<f32> = voice(p, &t, &freq, &mut rng, 0.5, 1.0)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect();
    // Crackle scales with brightness — bright ducks rasp, soft ducks just yelp.
    let crackle = (0.04 + 0.10 * p.brightness) as f32;
    for (v, e) in sig.iter_mut().zip(&env) {
        *v += crackle * rng.standard_normal() * e;
    }
    s::normalise(&mut sig, -3.0);
    sig
}

fn greet_syllable(p: &Personality, rng: &mut Rng, dur_scale: f64, f0_scale: f64) -> Vec<f32> {
    let dur = (0.32 + 0.25 * rng.random()) * dur_scale / p.speed;
    let t = s::t_axis(dur);
    let f0 = p.pitch_center_hz * (0.9 + 0.15 * rng.random()) * f0_scale;
    // glide_bias flips the contour: positive ducks bend up, negative down.
    let bias = p.glide_bias;
    let bend = 0.10 + 0.15 * p.pitch_spread;
    let start = f0 * (1.0 - bias * bend * 0.5);
    let mid = f0 * (1.0 + bias * bend);
    let end = f0 * (1.0 - bias * bend * 0.3) * (0.92 + 0.08 * rng.random());
    let freq = s::lerp(&t, &[(0.0, start), (0.18 * dur, mid), (dur, end)]);
    let env = s::expdecay(&t, attack(p, dur, 0.5), dur * 0.7);
    voice(p, &t, &freq, rng, 1.0, 1.0)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect()
}

pub fn greet(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("greet", variant);
    let mut sig = greet_syllable(p, &mut rng, 1.0, 1.0);
    // Some greets are double quacks — "wak-wak"; a mix of one- and two-syllable calls reads
    // as much more alive than a single shape.
    if rng.random() < 0.4 {
        let gap_n = ((0.05 + 0.06 * rng.random()) / p.speed * f64::from(SR)) as usize;
        sig.extend(std::iter::repeat_n(0.0, gap_n));
        let f0_scale = 0.95 + 0.06 * rng.random();
        sig.extend(greet_syllable(p, &mut rng, 0.8, f0_scale));
    }
    s::normalise(&mut sig, -3.0);
    sig
}

pub fn inquire(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("inquire", variant);
    let dur = (0.42 + 0.25 * rng.random()) / p.speed;
    let t = s::t_axis(dur);
    let f0 = p.pitch_center_hz * (0.88 + 0.10 * rng.random());
    // Always rises (it's a question), but how much depends on spread + bias.
    let rise = 1.15 + 0.50 * p.pitch_spread + 0.20 * p.glide_bias.max(0.0) + 0.10 * rng.random();
    let freq = s::lerp(
        &t,
        &[(0.0, f0 * 0.92), (0.30 * dur, f0 * 0.95), (dur, f0 * rise)],
    );
    let env = s::bell(
        &t,
        dur * (0.06 + 0.10 * (1.0 - p.attack_sharpness)),
        dur * 0.32,
    );
    let mut sig: Vec<f32> = voice(p, &t, &freq, &mut rng, 0.6, 1.0)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect();
    s::normalise(&mut sig, -3.0);
    sig
}

pub fn peck(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("peck", variant);
    let dur = (0.16 + 0.12 * rng.random()) / p.speed;
    let t = s::t_axis(dur);
    // Always low for this duck; bigger ducks (low register) get even lower pecks.
    let f0 = p.pitch_center_hz * (0.45 + 0.20 * rng.random());
    let freq = s::lerp(&t, &[(0.0, f0 * 1.5), (0.04 * dur, f0), (dur, f0 * 0.80)]);
    let env = s::expdecay(&t, attack(p, dur, 1.0), dur * 0.35);
    let mut body: Vec<f32> = voice(p, &t, &freq, &mut rng, 0.3, 0.5)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect();
    // Click amount depends on attack_sharpness — snappy ducks have a sharper "tock".
    let click_len = ((0.003 + 0.006 * p.attack_sharpness) * f64::from(SR)) as usize;
    let click_gain = (0.4 + 0.4 * p.attack_sharpness) as f32;
    for (b, c) in body.iter_mut().zip(s::click(t.len(), &mut rng, click_len)) {
        *b += click_gain * c;
    }
    s::normalise(&mut body, -3.0);
    body
}

fn chirp_syllable(
    p: &Personality,
    rng: &mut Rng,
    f0: f64,
    dur: f64,
    contour: &[(f64, f64)],
) -> Vec<f32> {
    let t = s::t_axis(dur);
    // Warble: rapid trill, depth & rate are personality-driven.
    let warble = s::vibrato(
        &t,
        p.warble_hz,
        p.warble_depth,
        rng.uniform(0.0, std::f64::consts::TAU),
    );
    let shape = s::lerp(&t, contour);
    let freq: Vec<f32> = shape
        .iter()
        .zip(&warble)
        .map(|(&c, &w)| (f0 as f32) * c * w)
        .collect();
    let env = s::expdecay(&t, attack(p, dur, 0.7), dur * 0.55);
    // Softer than greet: gut quackiness and brightness.
    let mut p_soft = *p;
    p_soft.quackiness *= 0.2;
    p_soft.brightness *= 0.5;
    p_soft.formant_gain *= 0.5;
    voice(&p_soft, &t, &freq, rng, 1.0, 0.4)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect()
}

/// Mouth-trigger sound. Variants cycle four distinct shapes — rise, fall, trill, double —
/// so random picks are heard as different calls, not re-rolls of the same blip.
pub fn chirp(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("chirp", variant);
    let shape = variant % 4;
    let f0 = p.pitch_center_hz * (0.95 + 0.75 * rng.random());
    let mut sig = match shape {
        0 => {
            // Rising blip.
            let dur = (0.10 + 0.10 * rng.random()) / p.speed;
            let peak = 1.12 + 0.10 * rng.random();
            chirp_syllable(
                p,
                &mut rng,
                f0,
                dur,
                &[(0.0, 0.88), (0.5 * dur, peak), (dur, 1.05)],
            )
        }
        1 => {
            // Falling blip.
            let dur = (0.10 + 0.10 * rng.random()) / p.speed;
            let start = 1.12 + 0.10 * rng.random();
            chirp_syllable(
                p,
                &mut rng,
                f0,
                dur,
                &[(0.0, start), (0.3 * dur, 1.0), (dur, 0.78)],
            )
        }
        2 => {
            // Trill — longer, warble cranked up.
            let dur = (0.22 + 0.14 * rng.random()) / p.speed;
            let mut p_trill = *p;
            p_trill.warble_depth = p.warble_depth.max(0.7) * 1.6;
            chirp_syllable(
                &p_trill,
                &mut rng,
                f0,
                dur,
                &[(0.0, 1.0), (0.5 * dur, 1.06), (dur, 0.94)],
            )
        }
        _ => {
            // Double "wek-wek".
            let dur = (0.08 + 0.05 * rng.random()) / p.speed;
            let mut sig = chirp_syllable(
                p,
                &mut rng,
                f0,
                dur,
                &[(0.0, 0.95), (0.4 * dur, 1.10), (dur, 0.88)],
            );
            let gap_n = ((0.03 + 0.03 * rng.random()) / p.speed * f64::from(SR)) as usize;
            sig.extend(std::iter::repeat_n(0.0, gap_n));
            let dur_b = dur * 0.9;
            let f0_b = f0 * (0.92 + 0.10 * rng.random());
            sig.extend(chirp_syllable(
                p,
                &mut rng,
                f0_b,
                dur_b,
                &[(0.0, 0.95), (0.4 * dur_b, 1.10), (dur_b, 0.88)],
            ));
            sig
        }
    };
    s::normalise(&mut sig, -6.0);
    sig
}

pub fn coo(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("coo", variant);
    let dur = (0.85 + 0.55 * rng.random()) / p.speed;
    let t = s::t_axis(dur);
    // Well below center — drowsy ducks drop further.
    let f0 = p.pitch_center_hz
        * (0.42 + 0.15 * (1.0 - p.attack_sharpness))
        * (0.94 + 0.12 * rng.random());
    let drift_a = 1.0 + 0.05 * rng.random() + 0.04 * p.glide_bias;
    let freq = s::lerp(
        &t,
        &[
            (0.0, f0 * 0.94),
            (dur * 0.5, f0 * drift_a),
            (dur, f0 * 0.90),
        ],
    );
    let env = s::bell(
        &t,
        dur * (0.18 + 0.10 * (1.0 - p.attack_sharpness)),
        dur * 0.30,
    );
    // Breathier, much slower modulation, no buzz.
    let mut p_soft = *p;
    p_soft.breath = p.breath.max(0.12) + 0.10;
    p_soft.quackiness = p.quackiness * 0.25;
    p_soft.am_rate_hz = p.am_rate_hz * 0.30;
    p_soft.vibrato_rate_hz = p.vibrato_rate_hz * 0.45;
    p_soft.vibrato_depth = p.vibrato_depth * 0.7;
    let mut sig: Vec<f32> = voice(&p_soft, &t, &freq, &mut rng, 1.0, 1.0)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect();
    s::normalise(&mut sig, -5.0);
    sig
}

/// (start, loop, end) for the held-trigger joy ride on roller blades.
///
/// Rendered as ONE continuous master and sliced, so jitter / breath / wobble carry across
/// the start→loop cut with no seam. The loop file's tail is then crossfaded onto the sample
/// just before the loop start, so playing it back-to-back wraps without a click. The end
/// segment has its own onset (a little flick up, then the fall) because the player may
/// leave the loop at any point.
pub fn wheee_segments(p: &Personality, variant: u32) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let mut rng = p.variant_rng("wheee", variant);
    let d_start = (0.80 + 0.30 * rng.random()) / p.speed;
    let d_loop = (1.60 + 0.60 * rng.random()) / p.speed;
    let d_end = (0.55 + 0.25 * rng.random()) / p.speed;
    let total = d_start + d_loop + d_end;
    let t = s::t_axis(total);
    let (t1, t2) = (d_start, d_start + d_loop);

    let f0 = p.pitch_center_hz * (0.95 + 0.10 * rng.random());
    // How high the ride goes — spread-y ducks scream higher.
    let top = 1.6 + 0.5 * p.pitch_spread + 0.25 * rng.random();
    let base = s::lerp(
        &t,
        &[
            (0.0, f0 * 0.85),
            (0.15 * d_start, f0),
            (t1, f0 * top),
            (t2, f0 * top),
            (t2 + 0.25 * d_end, f0 * top * 1.04),
            (total, f0 * 0.60),
        ],
    );
    // Excitement wobble, swelling as the ride picks up speed, steady in the loop.
    let wob_hz = 4.5 + 3.0 * rng.random();
    let swell = s::lerp(&t, &[(0.0, 0.15), (t1, 1.0), (total, 1.0)]);
    let freq: Vec<f32> = base
        .iter()
        .zip(&swell)
        .zip(&t)
        .map(|((&f, &sw), &x)| {
            let wob = (std::f64::consts::TAU * wob_hz * f64::from(x)).sin();
            f * 2.0f64.powf(0.5 * f64::from(sw) * wob / 12.0) as f32
        })
        .collect();

    let env = s::lerp(
        &t,
        &[
            (0.0, 0.0),
            (0.06f64.min(0.5 * d_start), 1.0),
            (t2 + 0.40 * d_end, 1.0),
            (total, 0.0),
        ],
    );
    // The wobble replaces vibrato; less buzz so the glide stays clean.
    let mut p_joy = *p;
    p_joy.vibrato_depth = p.vibrato_depth * 0.5;
    p_joy.quackiness = p.quackiness * 0.5;
    let mut sig: Vec<f32> = voice(&p_joy, &t, &freq, &mut rng, 0.5, 0.5)
        .iter()
        .zip(&env)
        .map(|(&v, &e)| v * e)
        .collect();
    // Normalise the master, THEN slice — segment levels must match.
    s::normalise(&mut sig, -4.0);

    let n1 = (t1 * f64::from(SR)) as usize;
    let n2 = (t2 * f64::from(SR)) as usize;
    let start = sig[..n1].to_vec();
    let mut loop_seg = sig[n1..n2].to_vec();
    let end = sig[n2..].to_vec();
    // Crossfade the loop tail onto the master just before the loop start, so loop[last]
    // flows into loop[0] exactly like the start flowed into it.
    let nx = ((0.08 * f64::from(SR)) as usize)
        .min(loop_seg.len() / 2)
        .min(n1);
    let len = loop_seg.len();
    for i in 0..nx {
        let w = i as f32 / (nx.max(1) as f32 - 1.0).max(1.0);
        let tail = len - nx + i;
        loop_seg[tail] = (1.0 - w) * loop_seg[tail] + w * sig[n1 - nx + i];
    }
    (start, loop_seg, end)
}

/// One full ride — start, two loop passes (so the seam is auditable), end. This is what
/// `render` produces; the daemon streams the segments and repeats the loop while the left
/// trigger is held.
pub fn wheee(p: &Personality, variant: u32) -> Vec<f32> {
    let (start, loop_seg, end) = wheee_segments(p, variant);
    let mut out = start;
    out.extend_from_slice(&loop_seg);
    out.extend_from_slice(&loop_seg);
    out.extend(end);
    out
}

/// A tiny cute retch, "bwep": a falling syllable with a wet throat flutter, then a little bubble
/// pops. The robot plays it as it lets go of the e-ink display it held in its beak.
pub fn bwep(p: &Personality, variant: u32) -> Vec<f32> {
    let mut rng = p.variant_rng("bwep", variant);
    let dur = (0.18 + 0.10 * rng.random()) / p.speed;
    let t = s::t_axis(dur);
    let peak = 1.25 + 0.20 * rng.random();
    let fall = 0.62 + 0.15 * rng.random();
    let jit = s::jitter(&t, 0.6, &mut rng);
    let freq: Vec<f32> = s::lerp(&t, &[(0.0, 1.15), (0.22 * dur, peak), (dur, fall)])
        .iter()
        .zip(&jit)
        .map(|(&c, &j)| (p.pitch_center_hz as f32) * c * j)
        .collect();
    let phase = s::phase_from_freq(&freq);
    let mut sig = s::harmonic_osc(&phase, &p.harmonics());
    // The gurgle: a deep, quickening flutter — the throat giving way.
    let gurgle_hz = 24.0 + 8.0 * rng.random();
    let wob = rng.uniform(0.0, std::f64::consts::TAU);
    let env = s::expdecay(&t, 0.008, dur * 0.4);
    let noise = s::pink_noise(t.len(), &mut rng);
    for (i, v) in sig.iter_mut().enumerate() {
        let x = f64::from(t[i]);
        let flutter = 1.0
            - 0.6
                * (0.5
                    + 0.5 * (std::f64::consts::TAU * gurgle_hz * x * (1.0 + 0.6 * x) + wob).sin());
        *v = (*v * flutter as f32 + 0.1 * noise[i]) * env[i];
    }
    // The bubble: a sine whose pitch drops fast, "plop", just after.
    let plop_dur = 0.07 + 0.03 * rng.random();
    let pt = s::t_axis(plop_dur);
    let hi = 750.0 + 300.0 * rng.random();
    let pfreq = s::lerp(
        &pt,
        &[
            (0.0, hi),
            (plop_dur * 0.5, hi * 0.28),
            (plop_dur, hi * 0.25),
        ],
    );
    let penv = s::expdecay(&pt, 0.002, plop_dur * 0.3);
    let start = sig.len() + ((0.01 + 0.03 * rng.random()) * f64::from(SR)) as usize;
    sig.resize(start + pt.len(), 0.0);
    for (i, (ph, e)) in s::phase_from_freq(&pfreq).iter().zip(&penv).enumerate() {
        sig[start + i] += 0.5 * ph.sin() * e;
    }
    s::normalise(&mut sig, -6.0);
    sig
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every recipe must produce a finite, normalised, non-trivial buffer for a spread of
    /// seeds and variants — the render-all path plays every one of these on a robot.
    #[test]
    fn every_recipe_renders_sane_audio() {
        for seed in [0u32, 100, 3_405_691_582] {
            let p = Personality::from_seed(seed);
            for (tag, f) in crate::TAGS {
                for variant in 0..crate::variant_count(tag) {
                    let buf = f(&p, variant);
                    assert!(buf.len() > 1000, "{tag} v{variant} seed {seed} too short");
                    let peak = buf.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
                    assert!(
                        buf.iter().all(|v| v.is_finite()),
                        "{tag} v{variant} seed {seed} not finite"
                    );
                    assert!(
                        (0.3..=1.0).contains(&peak),
                        "{tag} v{variant} seed {seed} peak {peak}"
                    );
                }
            }
        }
    }

    /// The wheee loop must wrap seamlessly: the crossfade makes the sample after the loop's
    /// last equal the loop's first neighbourhood — check the discontinuity is small.
    #[test]
    fn the_wheee_loop_wraps_without_a_click() {
        let p = Personality::from_seed(100);
        let (_, loop_seg, _) = wheee_segments(&p, 0);
        let step = (loop_seg[0] - loop_seg[loop_seg.len() - 1]).abs();
        // Adjacent samples of a −4 dBFS 48 kHz voice move far less than 0.2 full-scale;
        // an unfaded cut would routinely exceed it.
        assert!(step < 0.2, "loop seam step {step}");
    }
}
