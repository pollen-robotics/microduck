//! The head IMU (BMI088 on the HAT), read by `tofd` because `tofd` owns this I²C bus.
//!
//! The BMI088 sits on the same bus as the ToF (accel `0x19`, gyro `0x68` per the HAT schematic —
//! next to the ToF's `0x29` and the audio codec's `0x18`, no collision). The bus is accessed one
//! transaction at a time (each carries its slave address), so a second `i2cdev` handle for the IMU
//! coexists with the ToF driver's handle; the kernel serialises transactions at the adapter.
//!
//! Runs on its own std thread, not the ToF thread: the ToF blocks for seconds uploading firmware
//! and backs off for up to a minute when no sensor is fitted, and the IMU stream must not stall
//! behind that. Same shape as the ToF loop otherwise — open, read at `hz`, broadcast frames,
//! retry with backoff on error — so a BMI088 fitted later needs no reconnect.
//!
//! **Linux only, and quietly so**, like the vendored ULDs `build.rs` skips off a board: the bus is
//! `/dev/i2c-*` and the driver is `linux-embedded-hal`, so on a developer's Mac there is no sensor
//! to open and `imu_loop` says as much instead of being compiled. `tofd` still builds and still
//! serves depth from `--fake` or `--sim` there, which is what a laptop runs it for.
//!
//! Orientation is a Madgwick fusion (the `bmi088` crate's `Bmi088Ahrs`); `gyro`/`accel` are the
//! raw sensor axes. Placing the sample in the head frame (the IMU is rigid to the camera) is a
//! `kinematics` job for the consumer, not this daemon's.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use duck_ipc_proto as proto;

#[cfg(target_os = "linux")]
use std::path::PathBuf;
#[cfg(target_os = "linux")]
use std::sync::atomic::Ordering;
#[cfg(target_os = "linux")]
use std::time::{Duration, Instant};

#[cfg(target_os = "linux")]
use bmi088::{Bmi088, Bmi088Ahrs, Config};
#[cfg(target_os = "linux")]
use linux_embedded_hal::I2cdev;

#[cfg(target_os = "linux")]
use crate::BUS_CANDIDATES;

/// Madgwick convergence rate. 0.1 is the crate's recommended starting point: fast enough to track
/// a walking head, slow enough not to chase gyro noise.
#[cfg(target_os = "linux")]
const BETA: f64 = 0.1;

/// Reopen backoff after an I²C error, same reasoning as the ToF loop: a bus glitch and a missing
/// chip look alike from here, and one backoff serves both without hammering a shared bus.
#[cfg(target_os = "linux")]
const RETRY_MIN: Duration = Duration::from_millis(500);
#[cfg(target_os = "linux")]
const RETRY_MAX: Duration = Duration::from_secs(30);

/// How far an IMU subscriber may fall behind before it loses samples. At 100 Hz this is ~2.5 s.
pub const FRAME_BUFFER: usize = 256;

/// Read `temp_c` this often (every Nth sample); it barely moves and costs a bus read.
#[cfg(target_os = "linux")]
const TEMP_EVERY: u64 = 100;

/// What the `head_imu.stream` answer reports — whether a BMI088 was found and at what rate.
#[derive(Clone)]
pub struct ImuStatus {
    hz: u8,
    inner: Arc<std::sync::Mutex<Inner>>,
}

#[derive(Default)]
struct Inner {
    sensor: Option<String>,
    unavailable: Option<String>,
}

impl ImuStatus {
    pub fn new(hz: u8) -> Self {
        Self {
            hz,
            inner: Arc::new(std::sync::Mutex::new(Inner {
                unavailable: Some("no reading yet".to_owned()),
                ..Inner::default()
            })),
        }
    }

    #[cfg(target_os = "linux")]
    fn found(&self, sensor: &str) {
        let mut inner = self.inner.lock().unwrap();
        inner.sensor = Some(sensor.to_owned());
        inner.unavailable = None;
    }

    /// Switched off in the config, rather than absent or broken.
    ///
    /// A separate sentence from [`Self::lost`] on purpose: every other reason this stream has
    /// nothing is a board to go and look at, and this one is a line in `robotd.toml`. A
    /// subscriber that cannot tell them apart sends somebody to check a cable.
    pub fn off(&self) {
        self.lost(
            "the head IMU is off — `[head_imu] enabled = true` in robotd.toml, then restart tofd"
                .to_owned(),
        );
    }

    fn lost(&self, why: String) {
        let mut inner = self.inner.lock().unwrap();
        inner.sensor = None;
        inner.unavailable = Some(why);
    }

    pub fn result(&self) -> proto::HeadImuStreamResult {
        let inner = self.inner.lock().unwrap();
        proto::HeadImuStreamResult {
            accepted: true,
            sensor: inner.sensor.clone(),
            unavailable: inner.unavailable.clone(),
            hz: self.hz,
        }
    }
}

/// Read the BMI088 forever, broadcasting [`proto::HeadImuFrame`]. Returns only at shutdown.
#[cfg(target_os = "linux")]
pub fn imu_loop(
    bus: Option<&Path>,
    hz: u8,
    status: &ImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    let started = Instant::now();
    let period = Duration::from_secs_f64(1.0 / f64::from(hz.max(1)));
    let mut seq = 0u64;
    let mut backoff = RETRY_MIN;

    while !shutdown.load(Ordering::Acquire) {
        let mut ahrs = match open_imu(bus) {
            Ok(a) => {
                tracing::info!("head IMU found: BMI088");
                status.found("BMI088");
                backoff = RETRY_MIN;
                a
            }
            Err(e) => {
                status.lost(e.to_string());
                tracing::warn!(error = %e, backoff_ms = backoff.as_millis(), "no head IMU; will retry");
                sleep_unless_shutdown(backoff, shutdown);
                backoff = (backoff * 2).min(RETRY_MAX);
                continue;
            }
        };

        // Read until an error, then fall out to reopen. `last` gives Madgwick its dt.
        let mut last = Instant::now();
        let mut temp_c = 0.0f32;
        while !shutdown.load(Ordering::Acquire) {
            let tick = Instant::now();
            let dt = (tick - last).as_secs_f32().clamp(1e-4, 0.2);
            last = tick;
            // `update_all` rather than `update`: both read the accelerometer and the gyroscope,
            // and only this one hands the accelerometer sample back. Asking for it afterwards —
            // which is what this loop used to do — read the same six registers a second time.
            //
            // **Not for the CPU.** A transaction is worth about 9 µs of the ~440 µs a sample
            // costs on this board (`bench_imu` at 100 Hz: 4.53% for three reads against 4.46%
            // for two), so this buys nothing measurable and the idle cost of this thread is
            // somewhere else entirely. What it buys is that the published `accel` is the sample
            // the quaternion was computed from, rather than one read ~200 µs later, and that a
            // read can fail in one place instead of two — either chip failing reopens rather
            // than publishing a zero acceleration, which a consumer cannot tell from free-fall.
            match ahrs.update_all(dt) {
                Ok((accel, gyro, quat)) => {
                    if seq.is_multiple_of(TEMP_EVERY)
                        && let Ok(t) = ahrs.imu().read_temperature()
                    {
                        temp_c = t;
                    }
                    seq += 1;
                    let _ = frames.send(proto::HeadImuFrame {
                        seq,
                        at_us: started.elapsed().as_micros() as u64,
                        t_ns: proto::clock::monotonic_ns(),
                        gyro,
                        accel,
                        quat,
                        temp_c,
                    });
                }
                Err(e) => {
                    status.lost(format!("read failed: {e:?}"));
                    tracing::warn!("head IMU read failed; reopening");
                    break;
                }
            }
            let elapsed = tick.elapsed();
            if elapsed < period {
                sleep_unless_shutdown(period - elapsed, shutdown);
            }
        }

        // A read failure fell straight back into `open_imu`: a chip that answers its ID but
        // cannot stream was reopened in a tight loop, warning each time, on the bus the audio
        // codec shares. Same backoff as the open-failure path above.
        sleep_unless_shutdown(backoff, shutdown);
        backoff = (backoff * 2).min(RETRY_MAX);
    }
}

/// Off Linux there is no `/dev/i2c-*` to open, so this returns at once and the status says why —
/// the same answer `head_imu.stream` gives on a board whose HAT has no BMI088 fitted, which is the
/// shape every consumer already handles.
#[cfg(not(target_os = "linux"))]
pub fn imu_loop(
    _bus: Option<&Path>,
    _hz: u8,
    status: &ImuStatus,
    _frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    _shutdown: &Arc<AtomicBool>,
) {
    status.lost("the head IMU is on an I2C bus, which exists only on Linux".to_owned());
}

/// Open the BMI088 on the first bus that answers. The `bmi088` crate hardwires accel `0x19` /
/// gyro `0x68` (the HAT's addresses), so there is nothing to sweep — a failure to read the
/// chip-id in `Bmi088::new` is the "not fitted / bus glitch" signal.
#[cfg(target_os = "linux")]
fn open_imu(bus: Option<&Path>) -> anyhow::Result<Bmi088Ahrs<I2cdev>> {
    let buses: Vec<PathBuf> = match bus {
        Some(bus) => vec![bus.to_path_buf()],
        None => BUS_CANDIDATES.iter().map(PathBuf::from).collect(),
    };
    let mut last = None;
    for bus in &buses {
        if !bus.exists() {
            last = Some(anyhow::anyhow!("{} does not exist", bus.display()));
            continue;
        }
        let i2c = match I2cdev::new(bus) {
            Ok(i2c) => i2c,
            Err(e) => {
                last = Some(anyhow::anyhow!("open {}: {e}", bus.display()));
                continue;
            }
        };
        match Bmi088::new(i2c, Config::default()) {
            Ok(imu) => {
                tracing::info!(bus = %bus.display(), "BMI088 answered");
                return Ok(Bmi088Ahrs::new(imu, BETA));
            }
            Err(e) => last = Some(anyhow::anyhow!("BMI088 init on {}: {e:?}", bus.display())),
        }
    }
    Err(last.unwrap_or_else(|| anyhow::anyhow!("no bus to look on")))
}

#[cfg(target_os = "linux")]
fn sleep_unless_shutdown(dur: Duration, shutdown: &Arc<AtomicBool>) {
    // Slice the sleep so shutdown is prompt even during a long backoff.
    let slice = Duration::from_millis(50);
    let mut left = dur;
    while left > Duration::ZERO && !shutdown.load(Ordering::Acquire) {
        let step = left.min(slice);
        std::thread::sleep(step);
        left = left.saturating_sub(step);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three answers a subscriber can get, and the one this switch adds.
    ///
    /// "Off" has to be distinguishable from "not fitted" in the sentence itself, because they
    /// are the same silence: one is a line in a file and the other is a board to go and look
    /// at. So the reason names the key.
    #[test]
    fn switched_off_reads_differently_from_absent() {
        let status = ImuStatus::new(100);

        let fresh = status.result();
        assert!(fresh.sensor.is_none());
        assert_eq!(fresh.hz, 100);

        status.off();
        let off = status.result();
        let why = off.unavailable.expect("a reason");
        assert!(why.contains("[head_imu] enabled"), "{why}");
        assert!(off.sensor.is_none());
        // Still `accepted`: the subscription is fine, there is simply nothing coming. A refusal
        // would send a client into a reconnect loop over a setting.
        assert!(off.accepted);

        status.lost("nothing answered on any bus".to_owned());
        let absent = status.result();
        let why = absent.unavailable.expect("a reason");
        assert!(!why.contains("[head_imu]"), "{why}");
    }
}
