//! The particle filter's RNG, owned here for the same reason `sounds` owns
//! its own: determinism is a feature. A relocalize run that reproduces
//! bit-for-bit from a seed can be replayed against a recorded session and
//! debugged; one riding `rand::StdRng` (which reserves the right to change
//! algorithm) cannot. xoshiro256++ seeded through splitmix64 (public domain,
//! Blackman & Vigna); normals are Box–Muller.

pub struct Rng {
    s: [u64; 4],
    /// Box–Muller produces pairs; the spare is handed out on the next call.
    spare_normal: Option<f32>,
}

impl Rng {
    /// splitmix64 expands the seed into the four xoshiro words, so small
    /// seeds still start well-mixed.
    pub fn from_seed(seed: u64) -> Self {
        let mut x = seed;
        let mut next = || {
            x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = x;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        };
        Self {
            s: [next(), next(), next(), next()],
            spare_normal: None,
        }
    }

    fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let result = s[0].wrapping_add(s[3]).rotate_left(23).wrapping_add(s[0]);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        result
    }

    /// Uniform in [0, 1), from the top 24 bits — all the mantissa f32 holds.
    pub fn f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 * (1.0 / (1u64 << 24) as f32)
    }

    /// Uniform integer in [0, n). `n` must be non-zero.
    pub fn index(&mut self, n: usize) -> usize {
        debug_assert!(n > 0);
        // The modulo bias at particle-filter scales (n ≤ a few thousand
        // against 2^64) is unmeasurable.
        (self.next_u64() % n as u64) as usize
    }

    /// Zero-mean Gaussian with the given σ, Box–Muller.
    pub fn normal(&mut self, sigma: f32) -> f32 {
        if let Some(z) = self.spare_normal.take() {
            return z * sigma;
        }
        // r in (0, 1]: 1 − f32() cannot be zero, so ln() stays finite.
        let r = (1.0 - f64::from(self.f32())).max(f64::MIN_POSITIVE);
        let theta = std::f64::consts::TAU * f64::from(self.f32());
        let mag = (-2.0 * r.ln()).sqrt();
        self.spare_normal = Some((mag * theta.sin()) as f32);
        (mag * theta.cos()) as f32 * sigma
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The stream is pinned: a replayed relocalize must reproduce exactly.
    #[test]
    fn the_stream_is_deterministic() {
        let mut a = Rng::from_seed(0xC0FFEE);
        let mut b = Rng::from_seed(0xC0FFEE);
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn distributions_are_sane() {
        let mut rng = Rng::from_seed(7);
        let n = 100_000;
        let mean: f32 = (0..n).map(|_| rng.f32()).sum::<f32>() / n as f32;
        assert!((mean - 0.5).abs() < 0.01, "uniform mean {mean}");

        let (mut sum, mut sum2) = (0.0f64, 0.0f64);
        for _ in 0..n {
            let v = f64::from(rng.normal(2.0));
            sum += v;
            sum2 += v * v;
        }
        let m = sum / f64::from(n);
        let sd = (sum2 / f64::from(n) - m * m).sqrt();
        assert!(m.abs() < 0.05, "normal mean {m}");
        assert!((sd - 2.0).abs() < 0.05, "normal sd {sd}");

        let mut counts = [0usize; 5];
        for _ in 0..n {
            counts[rng.index(5)] += 1;
        }
        assert!(counts.iter().all(|&c| c > n as usize / 5 - 2000));
    }
}
