//! Linux GPIO v2 INT1 acquisition. Validate event/read timing before advancing fusion;
//! the ordinary polling path remains in `imu.rs`.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use ahrs::{Ahrs, Madgwick};
use anyhow::{Context, Result, anyhow, bail};
use bmi088::{AccBandwidth, Bmi088, Config, GyroBandwidth};
use duck_ipc_proto as proto;
use embedded_hal::i2c::I2c;
use gpiocdev::Request;
use gpiocdev::line::{EdgeDetection, EventClock};
use linux_embedded_hal::I2cdev;
use nalgebra::Vector3;

use super::{BETA, ImuStatus, Int1Line, RETRY_MAX, RETRY_MIN, TEMP_EVERY, sleep_unless_shutdown};
use crate::BUS_CANDIDATES;
use crate::imu_timing::{CaptureClock, DataReady};

const ACC: u8 = 0x19;
const GYRO: u8 = 0x68;
const WAIT_SLICE: Duration = Duration::from_millis(50);
const SILENCE_LIMIT: Duration = Duration::from_millis(500);
const MAX_DRAIN: usize = 128;

/// Record and restore the configuration this acquisition session changes, including on an
/// initialisation error. Restore the prior state before releasing our GPIO request.
struct SensorConfig {
    bus: I2cdev,
    original: Vec<(u8, u8, u8)>,
}

impl SensorConfig {
    fn open(path: &Path) -> Result<Self> {
        let device = path
            .canonicalize()
            .with_context(|| format!("resolve {}", path.display()))?;
        if let Some(number) = device
            .file_name()
            .and_then(|name| name.to_str())
            .and_then(|name| name.strip_prefix("i2c-"))
        {
            for address in [ACC, GYRO] {
                let driver = PathBuf::from(format!(
                    "/sys/bus/i2c/devices/{number}-{address:04x}/driver"
                ));
                if driver.exists() {
                    bail!(
                        "BMI088 {address:#x} is already bound to a kernel driver on {}; use its IIO acquisition path",
                        path.display()
                    );
                }
            }
        }
        let mut bus = I2cdev::new(path).with_context(|| format!("open {}", path.display()))?;
        if read_register(&mut bus, ACC, 0x00)? != 0x1e
            || read_register(&mut bus, GYRO, 0x00)? != 0x0f
        {
            bail!("BMI088 chip IDs did not match on {}", path.display());
        }
        let mut original = Vec::new();
        for (address, register) in [
            (ACC, 0x58),
            (ACC, 0x53),
            (ACC, 0x40),
            (ACC, 0x41),
            (GYRO, 0x0f),
            (GYRO, 0x10),
            (GYRO, 0x11),
            (ACC, 0x7d),
            (ACC, 0x7c),
        ] {
            original.push((
                address,
                register,
                read_register(&mut bus, address, register)?,
            ));
        }
        Ok(Self { bus, original })
    }

    fn wake(&mut self) -> Result<()> {
        // ACC_PWR_CONF is 0x7c, not 0x7d (ACC_PWR_CTRL). The pinned driver's duplicate
        // address would leave a cold accelerometer suspended and therefore generate no INT1.
        write_register(&mut self.bus, ACC, 0x7c, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));
        write_register(&mut self.bus, ACC, 0x7d, 0x04)?;
        write_register(&mut self.bus, GYRO, 0x11, 0x00)?;
        std::thread::sleep(Duration::from_millis(5));
        Ok(())
    }

    fn enable_int1(&mut self) -> Result<()> {
        let io = read_register(&mut self.bus, ACC, 0x53)?;
        let map = read_register(&mut self.bus, ACC, 0x58)?;
        let mapping = int1_data_ready_map(map);
        // Output enabled, push-pull, active-high. INT1 must carry DRDY alone: an old FIFO
        // mapping on the same pin would make the event timestamp ambiguous. Keep INT2 routing.
        write_register(&mut self.bus, ACC, 0x53, (io & !0x1e) | 0x0a)?;
        write_register(&mut self.bus, ACC, 0x58, mapping)?;
        if read_register(&mut self.bus, ACC, 0x53)? != (io & !0x1e) | 0x0a
            || read_register(&mut self.bus, ACC, 0x58)? != mapping
        {
            bail!("BMI088 INT1 configuration did not read back");
        }
        Ok(())
    }
}

impl Drop for SensorConfig {
    fn drop(&mut self) {
        for &(address, register, value) in &self.original {
            if let Err(error) = write_register(&mut self.bus, address, register, value) {
                tracing::warn!(address, register, %error, "could not restore head IMU register");
            }
        }
    }
}

fn read_register(bus: &mut I2cdev, address: u8, register: u8) -> Result<u8> {
    let mut value = [0];
    bus.write_read(address, &[register], &mut value)
        .map_err(|error| anyhow!("BMI088 read {address:#x}:{register:#x}: {error:?}"))?;
    Ok(value[0])
}

fn write_register(bus: &mut I2cdev, address: u8, register: u8, value: u8) -> Result<()> {
    bus.write(address, &[register, value])
        .map_err(|error| anyhow!("BMI088 write {address:#x}:{register:#x}: {error:?}"))
}

fn int1_data_ready_map(previous: u8) -> u8 {
    (previous & !0x07) | 0x04
}

fn sensor_configuration(hz: u8) -> Result<Config> {
    let (acc_bandwidth, gyro_bandwidth) = match hz {
        25 => (AccBandwidth::Hz25, GyroBandwidth::Hz100),
        50 => (AccBandwidth::Hz50, GyroBandwidth::Hz100),
        100 => (AccBandwidth::Hz100, GyroBandwidth::Hz100),
        200 => (AccBandwidth::Hz200, GyroBandwidth::Hz200),
        _ => bail!("INT1 mode requires --imu-hz 25, 50, 100 or 200; got {hz}"),
    };
    Ok(Config {
        acc_bandwidth,
        gyro_bandwidth,
        ..Config::default()
    })
}

struct Session {
    sensor: Bmi088<I2cdev>,
    // Restore the prior sensor state before the GPIO request is released.
    _configuration: SensorConfig,
    events: Request,
    pending: Option<DataReady>,
}

impl Session {
    fn open(bus: Option<&Path>, hz: u8, int1: &Int1Line) -> Result<Self> {
        let config = sensor_configuration(hz)?;
        let events = Request::builder()
            .on_chip(&int1.chip)
            .with_line(int1.line)
            .as_input()
            .with_edge_detection(EdgeDetection::RisingEdge)
            .with_event_clock(EventClock::Monotonic)
            .with_consumer("microduck-head-imu")
            .with_kernel_event_buffer_size(64)
            .request()
            .with_context(|| {
                format!(
                    "request BMI088 INT1 at {}:{} (GPIO v2)",
                    int1.chip.display(),
                    int1.line
                )
            })?;

        let paths: Vec<PathBuf> = bus.map_or_else(
            || BUS_CANDIDATES.iter().map(PathBuf::from).collect(),
            |path| vec![path.to_path_buf()],
        );
        let mut last_error = None;
        for path in paths {
            match SensorConfig::open(&path) {
                Ok(mut configuration) => {
                    configuration.wake()?;
                    let sensor = Bmi088::new(I2cdev::new(&path)?, config)
                        .map_err(|error| anyhow!("BMI088 initialise: {error:?}"))?;
                    std::thread::sleep(Duration::from_millis(5));
                    configuration.enable_int1()?;
                    tracing::info!(bus = %path.display(), gpiochip = %int1.chip.display(), line = int1.line, hz, "head IMU INT1 acquisition ready");
                    return Ok(Self {
                        sensor,
                        _configuration: configuration,
                        events,
                        pending: None,
                    });
                }
                Err(error) => last_error = Some(error),
            }
        }
        Err(last_error.unwrap_or_else(|| anyhow!("no bus to look on")))
    }

    /// Keep both the first and last event: after a read the first detects an overwrite,
    /// while the last is the only event whose register contents may still be present.
    fn drain(&self) -> Result<Option<(DataReady, DataReady)>> {
        let mut first = None;
        let mut last = None;
        for _ in 0..MAX_DRAIN {
            if !self.events.has_edge_event()? {
                return Ok(first.zip(last));
            }
            let edge = self.events.read_edge_event()?;
            let event = DataReady {
                t_ns: edge.timestamp_ns,
                seq: edge.line_seqno,
            };
            if first.is_none() {
                first = Some(event);
            }
            last = Some(event);
        }
        bail!("BMI088 INT1 events exceed the bounded drain; check wiring and configured rate")
    }

    fn next(&mut self) -> Result<Option<DataReady>> {
        let pending = self.pending.take();
        if pending.is_none() && !self.events.wait_edge_event(WAIT_SLICE)? {
            return Ok(None);
        }
        Ok(self.drain()?.map(|(_, last)| last).or(pending))
    }
}

pub fn run(
    bus: Option<&Path>,
    hz: u8,
    int1: &Int1Line,
    status: &ImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    let started = Instant::now();
    let mut sequence = 0;
    let mut backoff = RETRY_MIN;
    while !shutdown.load(Ordering::Acquire) {
        let result = Session::open(bus, hz, int1).and_then(|mut session| {
            status.found("BMI088");
            backoff = RETRY_MIN;
            capture(&mut session, hz, &mut sequence, started, frames, shutdown)
        });
        if let Err(error) = result {
            status.lost(error.to_string());
            tracing::warn!(%error, backoff_ms = backoff.as_millis(), "head IMU interrupt acquisition unavailable; retrying");
            sleep_unless_shutdown(backoff, shutdown);
            backoff = (backoff * 2).min(RETRY_MAX);
        }
    }
}

fn capture(
    session: &mut Session,
    hz: u8,
    sequence: &mut u64,
    started: Instant,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &Arc<AtomicBool>,
) -> Result<()> {
    let mut clock = CaptureClock::new(hz);
    let mut filter = Madgwick::new(1.0 / f64::from(hz), BETA);
    let mut last_edge = Instant::now();
    let mut temperature = 0.0;
    while !shutdown.load(Ordering::Acquire) {
        let Some(event) = session.next()? else {
            if last_edge.elapsed() > SILENCE_LIMIT {
                bail!("BMI088 INT1 produced no data-ready edge for 500 ms");
            }
            continue;
        };
        last_edge = Instant::now();
        let read_started_ns = proto::clock::monotonic_ns();
        if !clock.is_fresh(event, read_started_ns) {
            continue;
        }
        let (ax, ay, az) = session
            .sensor
            .read_accelerometer()
            .map_err(|error| anyhow!("BMI088 acceleration: {error:?}"))?;
        let (gx, gy, gz) = session
            .sensor
            .read_gyroscope()
            .map_err(|error| anyhow!("BMI088 gyro: {error:?}"))?;
        let read_finished_ns = proto::clock::monotonic_ns();
        let pending = session.drain()?;
        session.pending = pending.map(|(_, last)| last);
        let next_edge = pending.map(|(first, _)| first);
        let Some((timing, dt)) = clock.finish(event, read_started_ns, read_finished_ns, next_edge)
        else {
            continue;
        };
        // Same Madgwick state/gain and units as Bmi088Ahrs::update_all, but only accepted
        // register reads may advance it. Acceleration is normalised by update_imu internally.
        filter = Madgwick::new_with_quat(f64::from(dt), BETA, filter.quat);
        let q = match filter.update_imu(
            &Vector3::new(f64::from(gx), f64::from(gy), f64::from(gz)),
            &Vector3::new(f64::from(ax), f64::from(ay), f64::from(az)),
        ) {
            Ok(q) => *q,
            Err(_) => filter.quat,
        }
        .into_inner();
        if sequence.is_multiple_of(TEMP_EVERY)
            && let Ok(value) = session.sensor.read_temperature()
        {
            temperature = value;
        }
        *sequence += 1;
        let _ = frames.send(proto::HeadImuFrame {
            seq: *sequence,
            at_us: started.elapsed().as_micros() as u64,
            t_ns: proto::clock::monotonic_ns(),
            gyro: [gx, gy, gz],
            accel: [ax * 9.80665, ay * 9.80665, az * 9.80665],
            quat: [q.w as f32, q.i as f32, q.j as f32, q.k as f32],
            temp_c: temperature,
            timing: Some(timing),
        });
    }
    tracing::info!("head IMU interrupt acquisition stopped");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_events_must_not_be_reported_as_acceleration_data_ready() {
        // INT1 FIFO-full/watermark plus INT2 mappings left by an earlier session.
        // Merely OR-ing DRDY in would let FIFO events masquerade as sample times.
        assert_eq!(int1_data_ready_map(0x77), 0x74);
        assert_eq!(int1_data_ready_map(0x03), 0x04);
        assert_eq!(int1_data_ready_map(0x00), 0x04);
    }
}
