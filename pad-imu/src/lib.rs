//! A gamepad's inertial unit, read into an attitude.
//!
//! Some pads carry a six-axis IMU. The "Pro Controller" Switch clones do, and `padd`'s tap hands
//! its samples out as [`proto::PadReport::Imu`] — raw kernel units at several hundred a second.
//! This crate turns that stream into an orientation two programs need the same way: `padd`, to
//! pose the robot's head from the pad in your hands, and `robotctl monitor`, to draw the pad
//! tilting on screen. One filter, so the head and the picture cannot disagree.
//!
//! ## What it computes
//!
//! **Orientation**, as a quaternion from the pad's body frame to the world, by integrating the
//! gyro and pulling the result back toward the accelerometer's gravity — the smallest useful
//! complementary filter. Gravity fixes pitch and roll; yaw comes from the gyro alone and drifts,
//! which is honest: nothing on a pad can observe heading. Whoever uses yaw has to re-zero it now
//! and then, and [`relative`] is how.
//!
//! **Gyro bias**, because the clone's gyro is not calibrated. At rest on a table its Z rate reads
//! about 12 °/s, and integrated as-is the attitude would turn a full circle every thirty seconds
//! while the pad lay still. So the rate is watched for stillness — a short window in which the
//! three rates barely move and the accelerometer reads one g — and the mean over such a window is
//! taken as the bias. Until the first still window the raw rate is used and [`Imu::bias_dps`] says
//! so.
//!
//! ## Axes
//!
//! `hid-nintendo` reports the accelerometer on `ABS_X/Y/Z` and the gyro on `ABS_RX/RY/RZ`, with
//! `+Z` up when the pad lies flat: the clone reads about `+1 g` there at rest. The body frame is
//! **+X the pad's front** — the edge with the triggers, away from the player — and **+Y the pad's
//! left**, which follows from a right-handed frame with Z up. Confirmed against the drawn pad on
//! the robot, 2026-09-09.
//!
//! ## What it costs
//!
//! A few dozen multiplications per sample. Six hundred a second are a rounding error next to
//! reading them off the kernel.

use duck_ipc_proto as proto;

/// How hard the accelerometer pulls the orientation back toward gravity, per second.
///
/// Two: a tilt error decays with a half-second time constant, which is slow enough that shaking
/// the pad does not make the attitude flinch and fast enough that it is level a second after being
/// set down.
const GRAVITY_GAIN: f32 = 2.0;

/// Accelerometer magnitudes accepted as "this is gravity", in g.
///
/// Outside this the pad is being moved, and the accelerometer measures the hand as much as the
/// earth. The orientation then runs on the gyro alone until the pad settles.
const GRAVITY_BAND: std::ops::RangeInclusive<f32> = 0.85..=1.15;

/// How long the rates have to hold still before a bias is taken from them, seconds.
const STILL_WINDOW_S: f64 = 0.5;

/// How far the rates may wander across a still window and still count as still, °/s.
///
/// Above the clone's noise (about one °/s peak to peak at rest) and well below the slowest turn a
/// hand makes on purpose.
const STILL_SPREAD_DPS: f32 = 3.0;

/// Longest gap between two samples the integrator will bridge, seconds.
///
/// Beyond it the pad was silent — a dropped batch, a paused reader — and integrating one stale
/// rate over the whole gap would throw the attitude. The sample is taken; the interval is not.
const MAX_DT_S: f32 = 0.05;

/// One pad IMU, as accumulated from the tap.
pub struct Imu {
    device: proto::PadImuDevice,
    samples: u64,
    socket_dropped: u64,
    /// The last sample's kernel timestamp, for the interval.
    last_us: Option<u64>,
    /// Sample rate, hertz, as an average over the recent intervals.
    rate_hz: Option<f32>,
    /// The newest sample in physical units.
    accel_g: [f32; 3],
    gyro_dps: [f32; 3],
    bias: Bias,
    /// Body → world.
    q: [f32; 4],
    /// Has the orientation been seeded from gravity yet?
    seeded: bool,
}

/// The gyro's rest offset, and the still window being watched for the next one.
struct Bias {
    /// The bias in use, °/s. `None` until the first still window completes.
    value: Option<[f32; 3]>,
    /// The window under way: when it opened, what it has seen.
    since_us: Option<u64>,
    min: [f32; 3],
    max: [f32; 3],
    sum: [f64; 3],
    count: u32,
}

impl Bias {
    fn new() -> Self {
        Self {
            value: None,
            since_us: None,
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            sum: [0.0; 3],
            count: 0,
        }
    }

    /// Offer one raw rate. Only samples taken while the accelerometer reads gravity count: a pad
    /// in motion can have a quiet gyro for a moment and still not be at rest.
    fn observe(&mut self, raw_dps: [f32; 3], gravity_ok: bool, at_us: u64) {
        if !gravity_ok {
            self.reset_window();
            return;
        }
        let since = *self.since_us.get_or_insert(at_us);
        for (axis, rate) in raw_dps.iter().enumerate() {
            self.min[axis] = self.min[axis].min(*rate);
            self.max[axis] = self.max[axis].max(*rate);
            self.sum[axis] += f64::from(*rate);
        }
        self.count += 1;

        let spread = (0..3)
            .map(|axis| self.max[axis] - self.min[axis])
            .fold(0.0f32, f32::max);
        if spread > STILL_SPREAD_DPS {
            // Moved. Start over from here rather than waiting the window out.
            self.reset_window();
            return;
        }
        if (at_us.saturating_sub(since)) as f64 >= STILL_WINDOW_S * 1e6 && self.count > 0 {
            let mean = [
                (self.sum[0] / f64::from(self.count)) as f32,
                (self.sum[1] / f64::from(self.count)) as f32,
                (self.sum[2] / f64::from(self.count)) as f32,
            ];
            // Blend with what is already known rather than jumping: two still windows a minute
            // apart disagreeing by a tenth of a degree per second should not make the attitude
            // twitch each time.
            self.value = Some(match self.value {
                Some(old) => [
                    0.5 * (old[0] + mean[0]),
                    0.5 * (old[1] + mean[1]),
                    0.5 * (old[2] + mean[2]),
                ],
                None => mean,
            });
            self.reset_window();
        }
    }

    fn reset_window(&mut self) {
        self.since_us = None;
        self.min = [f32::INFINITY; 3];
        self.max = [f32::NEG_INFINITY; 3];
        self.sum = [0.0; 3];
        self.count = 0;
    }
}

impl Imu {
    pub fn new(device: proto::PadImuDevice) -> Self {
        Self {
            device,
            samples: 0,
            socket_dropped: 0,
            last_us: None,
            rate_hz: None,
            accel_g: [0.0; 3],
            gyro_dps: [0.0; 3],
            bias: Bias::new(),
            q: [1.0, 0.0, 0.0, 0.0],
            seeded: false,
        }
    }

    pub fn device(&self) -> &proto::PadImuDevice {
        &self.device
    }

    pub fn samples(&self) -> u64 {
        self.samples
    }

    pub fn socket_dropped(&self) -> u64 {
        self.socket_dropped
    }

    pub fn rate_hz(&self) -> Option<f32> {
        self.rate_hz
    }

    /// The newest acceleration, g.
    pub fn accel_g(&self) -> [f32; 3] {
        self.accel_g
    }

    /// The newest angular rate with the bias removed, °/s.
    pub fn gyro_dps(&self) -> [f32; 3] {
        self.gyro_dps
    }

    /// The gyro bias in use, °/s — `None` while it is still being learned.
    pub fn bias_dps(&self) -> Option<[f32; 3]> {
        self.bias.value
    }

    /// Take one batch, sample by sample and in order.
    pub fn absorb(&mut self, batch: &proto::PadImuBatch) {
        self.socket_dropped += batch.socket_dropped;
        for sample in &batch.samples {
            self.sample(sample);
        }
    }

    fn sample(&mut self, sample: &proto::PadImuSample) {
        self.samples += 1;

        let accel_scale = scale(self.device.accel_per_g);
        let gyro_scale = scale(self.device.gyro_per_dps);
        let accel = [
            sample.accel[0] as f32 * accel_scale,
            sample.accel[1] as f32 * accel_scale,
            sample.accel[2] as f32 * accel_scale,
        ];
        let raw_gyro = [
            sample.gyro[0] as f32 * gyro_scale,
            sample.gyro[1] as f32 * gyro_scale,
            sample.gyro[2] as f32 * gyro_scale,
        ];
        self.accel_g = accel;

        let magnitude = norm(accel);
        let gravity_ok = GRAVITY_BAND.contains(&magnitude);
        self.bias.observe(raw_gyro, gravity_ok, sample.at_us);
        let bias = self.bias.value.unwrap_or([0.0; 3]);
        let gyro = [
            raw_gyro[0] - bias[0],
            raw_gyro[1] - bias[1],
            raw_gyro[2] - bias[2],
        ];
        self.gyro_dps = gyro;

        // The interval, from the kernel's clock: a gap too long to trust is measured (for the
        // rate) but not integrated.
        let dt = match self.last_us {
            Some(last) if sample.at_us > last => (sample.at_us - last) as f32 * 1e-6,
            _ => 0.0,
        };
        self.last_us = Some(sample.at_us);
        if dt > 0.0 && dt <= MAX_DT_S {
            let hz = 1.0 / dt;
            self.rate_hz = Some(match self.rate_hz {
                Some(rate) => rate + 0.02 * (hz - rate),
                None => hz,
            });
        }

        if !self.seeded {
            if gravity_ok {
                self.q = from_gravity(accel);
                self.seeded = true;
            }
            return;
        }
        if dt <= 0.0 || dt > MAX_DT_S {
            return;
        }

        // Gyro: rotate the body by ω·dt, in the body frame.
        let rad = std::f32::consts::PI / 180.0;
        let step = [gyro[0] * rad * dt, gyro[1] * rad * dt, gyro[2] * rad * dt];
        self.q = mul(self.q, from_rotvec(step));

        // Gravity: nudge the body so that where it thinks "up" is moves onto where the
        // accelerometer says it is. With `q ← q ⊗ δ`, the new prediction is `δᵀ·predicted`, so δ
        // has to carry *measured* onto *predicted* — hence the order of the cross product; the
        // other way round runs away rather than converging. Applied a fraction at a time, and
        // skipped while the pad is being moved, when the accelerometer is not measuring gravity.
        if gravity_ok {
            let measured = normalized(accel);
            let predicted = rotate(conj(self.q), [0.0, 0.0, 1.0]);
            let error = cross(measured, predicted);
            let k = GRAVITY_GAIN * dt;
            self.q = mul(
                self.q,
                from_rotvec([error[0] * k, error[1] * k, error[2] * k]),
            );
        }
        self.q = normalized4(self.q);
    }

    /// Pitch, roll and yaw of the body in the world, degrees — [`euler_deg`] of the attitude.
    /// Yaw is the gyro's word alone and drifts; the other two are held by gravity.
    pub fn euler_deg(&self) -> [f32; 3] {
        euler_deg(self.q)
    }

    /// The attitude itself, body → world, `w` first. `None` until the first sample with a
    /// believable gravity has seeded it — before that there is no attitude, only a default.
    pub fn quaternion(&self) -> Option<[f32; 4]> {
        self.seeded.then_some(self.q)
    }
}

// ── quaternion arithmetic, w first ───────────────────────────────────────────────────────

/// The rotation from `reference` to `now`, in the body frame of `reference`: what a pad has done
/// since the moment `reference` was taken. This is how a consumer of yaw beats the drift — take a
/// fresh reference when the person says "this is centre", and read everything after it relative.
pub fn relative(reference: [f32; 4], now: [f32; 4]) -> [f32; 4] {
    normalized4(mul(conj(reference), now))
}

/// Pitch, roll and yaw of a body → world rotation, degrees.
///
/// Aerospace order — yaw about Z, then pitch about the body's Y (nose up positive), then roll
/// about its X (left side up positive) — read off the rotation matrix.
pub fn euler_deg(q: [f32; 4]) -> [f32; 3] {
    let m = matrix(q);
    // m carries body axes as columns in world coordinates: m[i][j] = row i, column j.
    let pitch = (-m[2][0]).clamp(-1.0, 1.0).asin();
    let roll = m[2][1].atan2(m[2][2]);
    let yaw = m[1][0].atan2(m[0][0]);
    let deg = 180.0 / std::f32::consts::PI;
    [pitch * deg, roll * deg, yaw * deg]
}

/// The rotation matrix of a unit quaternion, body axes as columns.
pub fn matrix(q: [f32; 4]) -> [[f32; 3]; 3] {
    let [w, x, y, z] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// Euclidean length of a vector.
pub fn norm(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Units per physical unit → physical units per raw unit. A driver that declared no resolution
/// leaves the numbers raw, which is at least not wrong.
fn scale(per_unit: i32) -> f32 {
    if per_unit > 0 {
        1.0 / per_unit as f32
    } else {
        1.0
    }
}

fn normalized(v: [f32; 3]) -> [f32; 3] {
    let n = norm(v);
    if n < 1e-9 {
        return [0.0, 0.0, 1.0];
    }
    [v[0] / n, v[1] / n, v[2] / n]
}

fn normalized4(q: [f32; 4]) -> [f32; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-9 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn conj(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// The rotation of a small rotation vector (axis × angle, radians).
fn from_rotvec(r: [f32; 3]) -> [f32; 4] {
    let angle = norm(r);
    if angle < 1e-9 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let (s, c) = (angle / 2.0).sin_cos();
    [c, r[0] / angle * s, r[1] / angle * s, r[2] / angle * s]
}

/// Rotate a vector by a quaternion.
fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let p = mul(mul(q, [0.0, v[0], v[1], v[2]]), conj(q));
    [p[1], p[2], p[3]]
}

/// The body → world rotation that carries the measured up (the accelerometer at rest) onto the
/// world's up, with no yaw. The seed, and what the filter would converge to if the gyro said
/// nothing.
fn from_gravity(accel: [f32; 3]) -> [f32; 4] {
    let up_body = normalized(accel);
    let up_world = [0.0, 0.0, 1.0];
    let axis = cross(up_body, up_world);
    let s = norm(axis);
    let c = up_body[2];
    if s < 1e-6 {
        return if c > 0.0 {
            [1.0, 0.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0, 0.0] // upside down: any horizontal axis will do
        };
    }
    let angle = s.atan2(c);
    from_rotvec([
        axis[0] / s * angle,
        axis[1] / s * angle,
        axis[2] / s * angle,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn a_device() -> proto::PadImuDevice {
        proto::PadImuDevice {
            name: "Nintendo Switch Pro Controller IMU".to_owned(),
            node: "/dev/input/event5".to_owned(),
            accel_per_g: 4096,
            gyro_per_dps: 14247,
            accel_max: 32767,
            gyro_max: 32_767_000,
        }
    }

    /// One sample, in physical units, `at_us` on the clock.
    fn sample(seq: u64, at_us: u64, accel_g: [f32; 3], gyro_dps: [f32; 3]) -> proto::PadImuSample {
        proto::PadImuSample {
            seq,
            at_us,
            accel: [
                (accel_g[0] * 4096.0) as i32,
                (accel_g[1] * 4096.0) as i32,
                (accel_g[2] * 4096.0) as i32,
            ],
            gyro: [
                (gyro_dps[0] * 14247.0) as i32,
                (gyro_dps[1] * 14247.0) as i32,
                (gyro_dps[2] * 14247.0) as i32,
            ],
        }
    }

    fn one(sample: proto::PadImuSample) -> proto::PadImuBatch {
        proto::PadImuBatch {
            samples: vec![sample],
            socket_dropped: 0,
        }
    }

    /// Feed `seconds` of a steady reading at 200 Hz.
    fn steady(imu: &mut Imu, seconds: f32, accel_g: [f32; 3], gyro_dps: [f32; 3]) {
        let n = (seconds * 200.0) as u64;
        let start = imu.last_us.unwrap_or(0);
        for i in 1..=n {
            imu.absorb(&one(sample(i, start + i * 5_000, accel_g, gyro_dps)));
        }
    }

    /// Flat on the table: level, and the clone's 12 °/s of Z bias is learned rather than
    /// integrated — the attitude must not spin while the real pad lies still.
    #[test]
    fn a_pad_at_rest_reads_level_and_learns_its_bias() {
        let mut imu = Imu::new(a_device());
        assert!(imu.quaternion().is_none(), "no attitude before a sample");
        steady(&mut imu, 3.0, [0.0, 0.0, 1.0], [1.8, -0.6, 12.0]);

        let bias = imu.bias_dps().expect("three seconds still is a bias");
        assert!((bias[2] - 12.0).abs() < 0.1, "{bias:?}");
        let [pitch, roll, yaw] = imu.euler_deg();
        assert!(
            pitch.abs() < 0.5 && roll.abs() < 0.5,
            "level: {pitch} {roll}"
        );
        // Yaw drifted for the half second before the bias was known and not after.
        assert!(yaw.abs() < 12.0 * 0.6, "yaw stopped drifting: {yaw}");
        let rate = imu.rate_hz().expect("a rate");
        assert!((rate - 200.0).abs() < 2.0, "{rate}");
        assert!(imu.quaternion().is_some());
    }

    /// Nose up: gravity moves toward −X on the body, and the filter reads that as positive pitch.
    /// The same rotation seeded from gravity and reached by integrating the gyro must agree.
    #[test]
    fn a_tilt_reads_as_pitch_whether_seeded_or_integrated() {
        let tilt = 30.0f32.to_radians();
        let tilted = [-tilt.sin(), 0.0, tilt.cos()];

        let mut seeded = Imu::new(a_device());
        steady(&mut seeded, 0.1, tilted, [0.0; 3]);
        let [pitch, roll, _] = seeded.euler_deg();
        assert!((pitch - 30.0).abs() < 1.0, "seeded pitch {pitch}");
        assert!(roll.abs() < 1.0, "seeded roll {roll}");

        // Level first, learn the (zero) bias, then pitch up at 60 °/s for half a second with the
        // accelerometer saying "moving" so only the gyro speaks.
        let mut turned = Imu::new(a_device());
        steady(&mut turned, 2.0, [0.0, 0.0, 1.0], [0.0; 3]);
        steady(&mut turned, 0.5, [0.0, 0.0, 1.5], [0.0, 60.0, 0.0]);
        let [pitch, _, _] = turned.euler_deg();
        assert!((pitch - 30.0).abs() < 1.5, "integrated pitch {pitch}");

        // And once it is still again, gravity agrees and holds it there.
        steady(&mut turned, 2.0, tilted, [0.0; 3]);
        let [pitch, _, _] = turned.euler_deg();
        assert!((pitch - 30.0).abs() < 1.0, "held pitch {pitch}");
    }

    /// A pad being waved has an accelerometer full of hand: it must not pull the orientation, and
    /// it must not be mistaken for stillness by the bias learner.
    #[test]
    fn motion_neither_corrects_the_attitude_nor_teaches_a_bias() {
        let mut imu = Imu::new(a_device());
        steady(&mut imu, 0.1, [0.0, 0.0, 1.0], [0.0; 3]);
        assert!(imu.bias_dps().is_none());
        // Quiet gyro, but two g on the accelerometer: not at rest.
        steady(&mut imu, 2.0, [0.0, 0.0, 2.0], [5.0, 0.0, 0.0]);
        assert!(imu.bias_dps().is_none(), "a swinging pad taught a bias");
        let [pitch, roll, _] = imu.euler_deg();
        // Integrated 5 °/s of roll for two seconds with nothing to say otherwise.
        assert!((roll - 10.0).abs() < 1.0, "roll {roll}");
        assert!(pitch.abs() < 1.0, "pitch {pitch}");
    }

    /// A gap the integrator will not bridge is measured, not integrated.
    #[test]
    fn a_long_gap_is_not_integrated() {
        let mut imu = Imu::new(a_device());
        steady(&mut imu, 2.0, [0.0, 0.0, 1.0], [0.0; 3]);
        let before = imu.euler_deg();
        let last = imu.last_us.unwrap();
        imu.absorb(&one(sample(
            9_999,
            last + 3_000_000,
            [0.0, 0.0, 1.0],
            [90.0, 0.0, 0.0],
        )));
        let after = imu.euler_deg();
        assert!((after[1] - before[1]).abs() < 0.5, "{before:?} → {after:?}");
    }

    /// Re-zeroing: the attitude relative to a reference taken a moment ago is what the pad did
    /// since — a yaw the gyro has drifted by is gone the moment a new reference is taken, and a
    /// turn made after it reads in full.
    #[test]
    fn a_relative_attitude_starts_from_zero_and_reads_the_turn_since() {
        let mut imu = Imu::new(a_device());
        steady(&mut imu, 2.0, [0.0, 0.0, 1.0], [0.0; 3]);
        // Drift: 40 °/s of yaw for a second, unlearned because the pad is "moving".
        steady(&mut imu, 1.0, [0.0, 0.0, 1.4], [0.0, 0.0, 40.0]);
        let drifted = imu.quaternion().unwrap();
        assert!(
            euler_deg(drifted)[2].abs() > 30.0,
            "{:?}",
            euler_deg(drifted)
        );

        let reference = drifted;
        let zeroed = euler_deg(relative(reference, imu.quaternion().unwrap()));
        assert!(zeroed.iter().all(|a| a.abs() < 0.01), "{zeroed:?}");

        // Turn left 20° (about +Z) after the reference, and that is all that shows.
        steady(&mut imu, 0.5, [0.0, 0.0, 1.4], [0.0, 0.0, 40.0]);
        let since = euler_deg(relative(reference, imu.quaternion().unwrap()));
        assert!((since[2] - 20.0).abs() < 1.0, "{since:?}");
        assert!(since[0].abs() < 1.0 && since[1].abs() < 1.0, "{since:?}");
    }
}
