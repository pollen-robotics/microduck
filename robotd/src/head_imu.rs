//! The beta board's head IMU: an LSM6DSV16X on the face board's I²C bus, served as
//! `head_imu.stream`.
//!
//! On `zero3` the head IMU is a BMI088 on the HAT, read by `tofd` because it shares the ToF's
//! bus, and fused on the CPU (`tof/src/imu.rs`). None of that holds here: the ToF is on SPI,
//! the IMU has a bus of its own, and this chip fuses orientation itself. So it is read by
//! `robotd`, which already owns the kinematics that place it (`frames.head_imu`).
//!
//! **Why it is cheap enough to be on by default.** What the BMI088 cost (~4% of a core at
//! 100 Hz, `docs/project/tof-on-demand.md`) was almost all I²C transactions — two per sample —
//! plus 100 wakeups a second; the Madgwick fusion was 0.3 points. Here the chip runs its SFLP
//! fusion and batches gyro, accelerometer and the game-rotation quaternion into its FIFO at
//! [`SAMPLE_HZ`], and this thread wakes [`WAKE_HZ`] times a second to read the FIFO level and
//! then every queued record in **one** burst: two transactions for ~4 samples. Measured on the
//! board (2026-10-06): one read from `FIFO_DATA_OUT_TAG` returns consecutive 7-byte records,
//! the address wrapping from `0x7E` back to `0x78`, with gyro, accel and SFLP tags interleaved.
//!
//! The bring-up is `imu_to_dxl`'s (the same chip on the body's power board): wait, software
//! reset, reboot the memory content to reload the factory trimming, configure, read back. A
//! chip configured before its trimming has loaded reads half the true gyro rate while its
//! quaternion is right — the "drunk robot" of `imu_to_dxl` fw 6.
//!
//! Frames are [`proto::HeadImuFrame`], the shape `tofd` serves on `zero3`: gyro and accel in the
//! chip's own axes, quaternion scalar-first sensor→world (gravity down, yaw arbitrary — a game
//! vector has no magnetometer).

use std::sync::Mutex;

use duck_ipc_proto as proto;

/// The rate the chip batches samples at, and so the rate frames are published at. The body IMU
/// runs at the control loop's 50 Hz; the SFLP cannot (its rates are 15, 30, 60, 120, 240 and
/// 480 Hz), so this is the nearest above it. Gyro and accel are batched at the same rate so each
/// quaternion comes with the gyro and accel sample it was computed alongside.
pub const SAMPLE_HZ: u8 = 60;

/// How often the reader drains the FIFO. Four samples a wakeup at [`SAMPLE_HZ`].
pub const WAKE_HZ: u32 = 15;

/// How far a subscriber may fall behind before it loses frames: ~2 s at [`SAMPLE_HZ`].
pub const FRAME_BUFFER: usize = 128;

/// The face board's IMU bus on the beta (`&i2c3` in rk3566-microduck-beta.dts), then the
/// others, so a board revision that moves it is still found.
#[cfg(target_os = "linux")]
const BUSES: &[&str] = &[
    "/dev/i2c-3",
    "/dev/i2c-4",
    "/dev/i2c-1",
    "/dev/i2c-2",
    "/dev/i2c-0",
];
/// SA0 low, then high.
#[cfg(target_os = "linux")]
const ADDRESSES: &[u16] = &[0x6a, 0x6b];

// ---- Registers (ST DS13510; values as in imu_to_dxl firmware/include/lsm6dsv16x.h) ----------
const FUNC_CFG_ACCESS: u8 = 0x01;
const FIFO_CTRL3: u8 = 0x09;
const FIFO_CTRL4: u8 = 0x0a;
const WHO_AM_I: u8 = 0x0f;
const CTRL1: u8 = 0x10;
const CTRL2: u8 = 0x11;
const CTRL3: u8 = 0x12;
const CTRL6: u8 = 0x15;
const CTRL8: u8 = 0x17;
const CTRL9: u8 = 0x18;
const FIFO_STATUS1: u8 = 0x1b;
const OUT_TEMP_L: u8 = 0x20;
const FIFO_DATA_OUT_TAG: u8 = 0x78;
// Embedded-functions bank, visible while FUNC_CFG_ACCESS bit 7 is set.
const EMB_FUNC_EN_A: u8 = 0x04;
const EMB_FUNC_FIFO_EN_A: u8 = 0x44;
const SFLP_ODR: u8 = 0x5e;
const EMB_FUNC_INIT_A: u8 = 0x66;
/// SFLP_ODR[5:3]: 010 is 60 Hz (the default 011 is 120). The other bits are reserved and kept.
const SFLP_ODR_MASK: u8 = 0b0011_1000;
const SFLP_ODR_60HZ: u8 = 0b010 << 3;
const SFLP_GAME_BIT: u8 = 1 << 1;

const WHO_AM_I_VALUE: u8 = 0x70;
const CTRL3_BOOT: u8 = 1 << 7;
const CTRL3_SW_RESET: u8 = 1 << 0;

/// The user-bank configuration, written in this order and read back. Same scales and filters as
/// the body's board, so a sample means the same thing on both: gyro ±500 dps, accel ±4 g with
/// LPF2 at ODR/20 (it shapes only the raw accel; SFLP taps the chain before it).
const CONFIG: &[(u8, u8)] = &[
    (CTRL3, 0x44),      // BDU + IF_INC (burst auto-increment)
    (CTRL6, 0x02),      // gyro ±500 dps
    (CTRL8, 0x41),      // accel ±4 g, LPF2 ODR/20
    (CTRL9, 0x08),      // LPF2_XL_EN
    (CTRL1, 0x06),      // accel 120 Hz, high performance (SFLP needs the sensors >= its 60 Hz)
    (CTRL2, 0x06),      // gyro  120 Hz, high performance
    (FIFO_CTRL3, 0x55), // batch gyro and accel at 60 Hz
    (FIFO_CTRL4, 0x06), // continuous: the oldest records go if the reader falls behind
];

// FIFO record tags (FIFO_DATA_OUT_TAG[7:3]).
const TAG_GYRO: u8 = 0x01;
const TAG_ACCEL: u8 = 0x02;
const TAG_SFLP_GAME: u8 = 0x13;
const RECORD: usize = 7;

const GYRO_RAD_PER_LSB: f32 = 0.0175 * std::f32::consts::PI / 180.0;
const ACCEL_MS2_PER_LSB: f32 = 0.000122 * 9.806_65;

/// The chip's sensor→head mount on the beta's face board, so the head frame is `x` forward, `y`
/// left, `z` up with the head level: head = `[−z, −x, +y]` of the sensor's. +90° about X, then
/// −90° about Z. Found on a beta (2026-10-06): standing straight the chip's `+y` pointed up,
/// and a nod — pitch — then read as roll until the second turn put its `−z` forward.
pub const MOUNT: [f32; 4] = [0.5, 0.5, -0.5, -0.5];

/// Rotate `v` by the unit quaternion `q` (scalar-first).
fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [w, x, y, z] = q;
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

/// Hamilton product `a ⊗ b`, scalar-first.
fn mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// A sample in the sensor's axes, put in the head's: vectors rotated by [`MOUNT`], and the
/// orientation sensor→world turned into head→world (`q ⊗ MOUNT⁻¹`), as the body IMU's decoder
/// does with its own mount.
fn to_head(gyro: [f32; 3], accel: [f32; 3], quat: [f32; 4]) -> Sample {
    let inverse = [MOUNT[0], -MOUNT[1], -MOUNT[2], -MOUNT[3]];
    Sample {
        gyro: rotate(MOUNT, gyro),
        accel: rotate(MOUNT, accel),
        quat: mul(quat, inverse),
    }
}

/// One fused sample out of the FIFO, before it is stamped.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub quat: [f32; 4],
}

/// Turn a burst of FIFO records into samples, one per SFLP quaternion.
///
/// The FIFO interleaves the three kinds at the same batch rate, so a quaternion is paired with
/// the most recent gyro and accel record before it. A quaternion that arrives before either (the
/// first record after a reset, or a burst that starts mid-triple) is paired with whatever the
/// previous burst left in `last` — which is why `last` outlives one call.
pub fn decode(burst: &[u8], last: &mut ([f32; 3], [f32; 3])) -> Vec<Sample> {
    let mut out = Vec::new();
    // A trailing partial record (a burst cut short) is dropped with the remainder.
    let (records, _) = burst.as_chunks::<RECORD>();
    for record in records {
        let tag = record[0] >> 3;
        let axes = |scale: f32| {
            [0, 1, 2].map(|i| {
                f32::from(i16::from_le_bytes([record[1 + 2 * i], record[2 + 2 * i]])) * scale
            })
        };
        match tag {
            TAG_GYRO => last.0 = axes(GYRO_RAD_PER_LSB),
            TAG_ACCEL => last.1 = axes(ACCEL_MS2_PER_LSB),
            TAG_SFLP_GAME => {
                let [x, y, z] = [0, 1, 2].map(|i| {
                    f16_to_f32(u16::from_le_bytes([record[1 + 2 * i], record[2 + 2 * i]]))
                });
                // The chip sends x, y, z of a unit quaternion with w >= 0; w is what is left.
                let w = (1.0 - x * x - y * y - z * z).max(0.0).sqrt();
                out.push(to_head(last.0, last.1, [w, x, y, z]));
            }
            // Timestamps, temperature and the rest are not batched; skip anything else.
            _ => {}
        }
    }
    out
}

/// IEEE 754 half precision, the format the SFLP batches its quaternion in.
pub fn f16_to_f32(h: u16) -> f32 {
    let sign = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = i32::from((h >> 10) & 0x1f);
    let frac = f32::from(h & 0x3ff);
    match exp {
        0 => sign * frac * 2f32.powi(-24),
        31 if frac == 0.0 => sign * f32::INFINITY,
        31 => f32::NAN,
        _ => sign * (1.0 + frac / 1024.0) * 2f32.powi(exp - 15),
    }
}

/// The chip temperature, from `OUT_TEMP_L/H`: 256 LSB/°C around 25 °C.
pub fn temperature_c(raw: [u8; 2]) -> f32 {
    25.0 + f32::from(i16::from_le_bytes(raw)) / 256.0
}

/// Spread `n` samples read at `read_ns` back over the time they were taken: the newest at the
/// read, each earlier one a sample period before it.
pub fn stamps(read_ns: u64, n: usize) -> impl Iterator<Item = u64> {
    let period = 1_000_000_000 / u64::from(SAMPLE_HZ);
    (0..n).map(move |i| read_ns.saturating_sub(period * (n - 1 - i) as u64))
}

/// What a `head_imu.stream` subscriber is told before the frames: which chip, or why none.
pub struct HeadImuStatus {
    inner: Mutex<(Option<String>, Option<String>)>,
}

impl HeadImuStatus {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new((None, Some("no reading yet".to_owned()))),
        }
    }

    pub fn found(&self, sensor: &str) {
        *self.inner.lock().unwrap() = (Some(sensor.to_owned()), None);
    }

    pub fn lost(&self, why: String) {
        *self.inner.lock().unwrap() = (None, Some(why));
    }

    /// Switched off in the file — named, so nobody goes looking at a cable.
    pub fn off(&self) {
        self.lost(
            "the head IMU is off — `[head_imu] enabled = true` in robotd.toml, then restart robotd"
                .to_owned(),
        );
    }

    pub fn result(&self) -> proto::HeadImuStreamResult {
        let inner = self.inner.lock().unwrap();
        proto::HeadImuStreamResult {
            accepted: true,
            sensor: inner.0.clone(),
            unavailable: inner.1.clone(),
            hz: SAMPLE_HZ,
        }
    }
}

#[cfg(target_os = "linux")]
pub use reader::run;

/// Off Linux there is no I²C bus; the status says so, which every subscriber already handles.
#[cfg(not(target_os = "linux"))]
pub fn run(
    status: &HeadImuStatus,
    _frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    _shutdown: &std::sync::atomic::AtomicBool,
) {
    status.lost("the head IMU is on an I2C bus, which exists only on Linux".to_owned());
}

#[cfg(target_os = "linux")]
mod reader {
    use std::fs::{File, OpenOptions};
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::{Duration, Instant};

    use super::*;

    /// `I2C_SLAVE` from linux/i2c-dev.h.
    const I2C_SLAVE: libc::c_ulong = 0x0703;
    /// More than a wakeup ever queues (4 at 30 Hz), less than the FIFO's 512 words, so a backlog
    /// after a stall drains over a few wakeups rather than in one long transfer.
    const MAX_WORDS: usize = 48;
    const TEMP_EVERY: u64 = WAKE_HZ as u64;
    const RETRY_MIN: Duration = Duration::from_millis(500);
    const RETRY_MAX: Duration = Duration::from_secs(30);

    struct Chip {
        file: File,
    }

    impl Chip {
        fn open(bus: &str, address: u16) -> std::io::Result<Self> {
            let file = OpenOptions::new().read(true).write(true).open(bus)?;
            // SAFETY: I2C_SLAVE takes the address by value; the fd is owned and open.
            if unsafe { libc::ioctl(file.as_raw_fd(), I2C_SLAVE, libc::c_ulong::from(address)) } < 0
            {
                return Err(std::io::Error::last_os_error());
            }
            Ok(Self { file })
        }

        fn write(&mut self, reg: u8, value: u8) -> std::io::Result<()> {
            self.file.write_all(&[reg, value])
        }

        /// Set the register pointer, then read `buf.len()` bytes from it (IF_INC advances it).
        fn read(&mut self, reg: u8, buf: &mut [u8]) -> std::io::Result<()> {
            self.file.write_all(&[reg])?;
            self.file.read_exact(buf)
        }

        fn read1(&mut self, reg: u8) -> std::io::Result<u8> {
            let mut b = [0u8];
            self.read(reg, &mut b)?;
            Ok(b[0])
        }

        fn set_bits(&mut self, reg: u8, mask: u8) -> std::io::Result<()> {
            let v = self.read1(reg)?;
            self.write(reg, v | mask)
        }

        /// `imu_to_dxl`'s bring-up: reset, reload the trimming, configure, read back, start SFLP.
        fn bring_up(&mut self) -> std::io::Result<()> {
            self.write(CTRL3, CTRL3_SW_RESET)?;
            for _ in 0..20 {
                std::thread::sleep(Duration::from_millis(1));
                if self.read1(CTRL3)? & CTRL3_SW_RESET == 0 {
                    break;
                }
            }
            self.write(CTRL3, CTRL3_BOOT | 0x44)?;
            std::thread::sleep(Duration::from_millis(30));
            for &(reg, value) in CONFIG {
                self.write(reg, value)?;
            }
            for &(reg, value) in CONFIG {
                let got = self.read1(reg)?;
                if got != value {
                    return Err(std::io::Error::other(format!(
                        "register {reg:#04x} reads {got:#04x}, wrote {value:#04x}"
                    )));
                }
            }
            self.write(FUNC_CFG_ACCESS, 0x80)?;
            let sflp = (|| {
                let odr = self.read1(SFLP_ODR)?;
                self.write(SFLP_ODR, (odr & !SFLP_ODR_MASK) | SFLP_ODR_60HZ)?;
                self.set_bits(EMB_FUNC_EN_A, SFLP_GAME_BIT)?;
                self.set_bits(EMB_FUNC_FIFO_EN_A, SFLP_GAME_BIT)?;
                self.set_bits(EMB_FUNC_INIT_A, SFLP_GAME_BIT)
            })();
            // Back to the user bank whatever happened, or every later read lands in the wrong one.
            self.write(FUNC_CFG_ACCESS, 0x00)?;
            sflp
        }
    }

    fn find() -> Result<(Chip, String), String> {
        let mut last = "no I2C bus to look on".to_owned();
        for bus in BUSES {
            if !std::path::Path::new(bus).exists() {
                continue;
            }
            for &address in ADDRESSES {
                match Chip::open(bus, address).and_then(|mut c| c.read1(WHO_AM_I).map(|w| (c, w))) {
                    Ok((chip, WHO_AM_I_VALUE)) => {
                        return Ok((chip, format!("{bus} {address:#04x}")));
                    }
                    Ok((_, other)) => {
                        last = format!(
                            "{bus} {address:#04x}: WHO_AM_I {other:#04x}, not an LSM6DSV16X"
                        )
                    }
                    Err(e) => last = format!("{bus} {address:#04x}: {e}"),
                }
            }
        }
        Err(last)
    }

    /// Read the head IMU until shutdown, broadcasting frames. Reopens with backoff on any error.
    pub fn run(
        status: &HeadImuStatus,
        frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
        shutdown: &AtomicBool,
    ) {
        let started = Instant::now();
        let period = Duration::from_secs_f64(1.0 / f64::from(WAKE_HZ));
        let mut backoff = RETRY_MIN;
        let mut seq = 0u64;
        while !shutdown.load(Ordering::Acquire) {
            let (mut chip, at) = match find() {
                Ok(found) => found,
                Err(why) => {
                    status.lost(format!("no LSM6DSV16X answered: {why}"));
                    sleep_unless_shutdown(backoff, shutdown);
                    backoff = (backoff * 2).min(RETRY_MAX);
                    continue;
                }
            };
            if let Err(e) = chip.bring_up() {
                status.lost(format!("LSM6DSV16X at {at} did not configure: {e}"));
                tracing::warn!(error = %e, at, "head IMU bring-up failed; will retry");
                sleep_unless_shutdown(backoff, shutdown);
                backoff = (backoff * 2).min(RETRY_MAX);
                continue;
            }
            tracing::info!(
                at,
                hz = SAMPLE_HZ,
                "head IMU found: LSM6DSV16X, fused on the chip"
            );
            status.found("LSM6DSV16X");
            backoff = RETRY_MIN;

            let mut last = ([0.0; 3], [0.0; 3]);
            let mut temp_c = 0.0f32;
            let mut wakeups = 0u64;
            let mut burst = vec![0u8; MAX_WORDS * RECORD];
            let error = loop {
                if shutdown.load(Ordering::Acquire) {
                    return;
                }
                let tick = Instant::now();
                let mut level = [0u8; 2];
                if let Err(e) = chip.read(FIFO_STATUS1, &mut level) {
                    break e;
                }
                let words =
                    (usize::from(level[0]) | (usize::from(level[1] & 0x01) << 8)).min(MAX_WORDS);
                if words > 0 {
                    let bytes = &mut burst[..words * RECORD];
                    if let Err(e) = chip.read(FIFO_DATA_OUT_TAG, bytes) {
                        break e;
                    }
                    let read_ns = proto::clock::monotonic_ns();
                    if wakeups.is_multiple_of(TEMP_EVERY) {
                        let mut raw = [0u8; 2];
                        if chip.read(OUT_TEMP_L, &mut raw).is_ok() {
                            temp_c = temperature_c(raw);
                        }
                    }
                    let samples = decode(bytes, &mut last);
                    let n = samples.len();
                    for (sample, t_ns) in samples.into_iter().zip(stamps(read_ns, n)) {
                        seq += 1;
                        let _ = frames.send(proto::HeadImuFrame {
                            seq,
                            at_us: started.elapsed().as_micros() as u64,
                            t_ns,
                            gyro: sample.gyro,
                            accel: sample.accel,
                            quat: sample.quat,
                            temp_c,
                            // The FIFO read has no INT1 event to time, like every polling frame.
                            timing: None,
                        });
                    }
                }
                wakeups += 1;
                let elapsed = tick.elapsed();
                if elapsed < period {
                    sleep_unless_shutdown(period - elapsed, shutdown);
                }
            };
            status.lost(format!("read failed: {error}"));
            tracing::warn!(error = %error, "head IMU read failed; reopening");
            sleep_unless_shutdown(backoff, shutdown);
            backoff = (backoff * 2).min(RETRY_MAX);
        }
    }

    fn sleep_unless_shutdown(dur: Duration, shutdown: &AtomicBool) {
        let slice = Duration::from_millis(50);
        let mut left = dur;
        while left > Duration::ZERO && !shutdown.load(Ordering::Acquire) {
            let step = left.min(slice);
            std::thread::sleep(step);
            left = left.saturating_sub(step);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(tag: u8, data: [u8; 6]) -> [u8; 7] {
        let mut r = [0u8; 7];
        r[0] = tag << 3;
        r[1..].copy_from_slice(&data);
        r
    }

    /// The records read off the board on 2026-10-06, tags and all: gyro, accel, then the SFLP
    /// quaternion that came out of the same burst.
    #[test]
    fn a_burst_from_the_board_decodes_to_one_sample() {
        let mut burst = Vec::new();
        burst.extend(record(TAG_GYRO, [0xf1, 0x06, 0xb8, 0x0b, 0x1f, 0xee]));
        burst.extend(record(TAG_ACCEL, [0xf7, 0xff, 0x14, 0x14, 0x05, 0xff]));
        // x = 0.7231, y = 0.0022, z = 0.0016: the first quaternion read off the board.
        burst.extend(record(TAG_SFLP_GAME, [0xc9, 0x39, 0x7b, 0x18, 0x8a, 0x16]));
        let mut last = ([0.0; 3], [0.0; 3]);
        let samples = decode(&burst, &mut last);
        assert_eq!(samples.len(), 1);
        let s = samples[0];
        // Sensor x is head -y: 0x06f1 = 1777 LSB * 17.5 mdps = 31.1 dps = 0.543 rad/s.
        assert!((s.gyro[1] + 0.5428).abs() < 1e-3, "{:?}", s.gyro);
        // Sensor y is head z: 0x1414 = 5140 LSB * 0.122 mg = 0.627 g = 6.15 m/s².
        assert!((s.accel[2] - 6.150).abs() < 1e-2, "{:?}", s.accel);
        // With the head level the world's up, seen from the head, is +z within a few degrees.
        // (Yaw is a game vector's own and arbitrary, so only the tilt is checked.)
        let [w, x, y, z] = s.quat;
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        assert!((norm - 1.0).abs() < 1e-3, "unit quaternion, got {norm}");
        let tilt = (1.0 - 2.0 * (x * x + y * y))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        assert!(
            tilt < 4.0,
            "a level head reads near-identity, got {tilt}° ({:?})",
            s.quat
        );
    }

    /// A burst that starts with a quaternion pairs it with the previous burst's gyro and accel.
    #[test]
    fn a_quaternion_first_in_a_burst_uses_the_last_burst_sensors() {
        let mut last = ([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]);
        let burst = record(TAG_SFLP_GAME, [0, 0, 0, 0, 0, 0]);
        let samples = decode(&burst, &mut last);
        // [x, y, z] of the sensor is [-z, -x, y] of the head.
        let close = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(p, q)| (p - q).abs() < 1e-5);
        assert!(
            close(samples[0].gyro, [-3.0, -1.0, 2.0]),
            "{:?}",
            samples[0].gyro
        );
        assert!(
            close(samples[0].accel, [-6.0, -4.0, 5.0]),
            "{:?}",
            samples[0].accel
        );
    }

    #[test]
    fn unknown_tags_and_a_partial_record_are_skipped() {
        let mut burst = record(0x04, [9; 6]).to_vec(); // timestamp tag
        burst.extend([0x13 << 3, 1, 2]); // cut short
        let mut last = ([0.0; 3], [0.0; 3]);
        assert!(decode(&burst, &mut last).is_empty());
    }

    /// The mount itself: the chip's +y (up on a level head) is head +z, and its -z is forward.
    #[test]
    fn the_mount_puts_the_chips_y_up() {
        let up = to_head([0.0; 3], [0.0, 9.8, 0.0], [1.0, 0.0, 0.0, 0.0]).accel;
        assert!(
            (up[2] - 9.8).abs() < 1e-4 && up[0].abs() < 1e-4 && up[1].abs() < 1e-4,
            "{up:?}"
        );
        let fwd = to_head([0.0, 0.0, -1.0], [0.0; 3], [1.0, 0.0, 0.0, 0.0]).gyro;
        assert!((fwd[0] - 1.0).abs() < 1e-5, "{fwd:?}");
    }

    #[test]
    fn half_floats() {
        assert_eq!(f16_to_f32(0x3c00), 1.0);
        assert_eq!(f16_to_f32(0xbc00), -1.0);
        assert_eq!(f16_to_f32(0x3800), 0.5);
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert!((f16_to_f32(0x0001) - 5.96e-8).abs() < 1e-9, "subnormal");
        assert!(f16_to_f32(0x7c00).is_infinite());
        assert!(f16_to_f32(0x7e00).is_nan());
    }

    #[test]
    fn temperature() {
        assert_eq!(temperature_c([0, 0]), 25.0);
        assert_eq!(temperature_c([0x00, 0x01]), 26.0);
    }

    /// The newest sample is stamped at the read; the others a sample period apart before it.
    #[test]
    fn samples_are_spread_back_over_the_burst() {
        let period = 1_000_000_000 / u64::from(SAMPLE_HZ);
        let t: Vec<u64> = stamps(10_000_000_000, 3).collect();
        assert_eq!(
            t,
            vec![
                10_000_000_000 - 2 * period,
                10_000_000_000 - period,
                10_000_000_000
            ]
        );
        assert_eq!(
            stamps(5, 2).next(),
            Some(0),
            "never before the clock's zero"
        );
    }

    /// The configuration matches the body's board where it matters to a reader: same scales
    /// and filters, so a sample means the same on both IMUs.
    #[test]
    fn same_scales_as_the_body_board() {
        let value = |reg| CONFIG.iter().find(|(r, _)| *r == reg).map(|(_, v)| *v);
        assert_eq!(value(CTRL6), Some(0x02), "gyro ±500 dps");
        assert_eq!(value(CTRL8), Some(0x41), "accel ±4 g, LPF2 ODR/20");
        assert_eq!(
            value(CTRL3),
            Some(0x44),
            "BDU + IF_INC, which the burst read relies on"
        );
    }

    #[test]
    fn the_status_names_the_switch_when_off() {
        let status = HeadImuStatus::new();
        status.off();
        let r = status.result();
        assert!(r.accepted && r.sensor.is_none());
        assert!(r.unavailable.unwrap().contains("[head_imu] enabled"));
        status.found("LSM6DSV16X");
        assert_eq!(status.result().sensor.as_deref(), Some("LSM6DSV16X"));
        assert_eq!(status.result().hz, SAMPLE_HZ);
    }
}
