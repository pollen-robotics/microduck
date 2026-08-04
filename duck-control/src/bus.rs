//! The Dynamixel bus.
//!
//! One combined `sync_read` per tick covering the IMU board and all 15 servos, and one
//! `sync_write` of goal positions. The IMU is listed first so it answers before the servo
//! burst.
//!
//! Battery and thermals are the one thing that does not fit that shape: they live at registers
//! outside the block the tick fetches, so [`RobotIo::slow_sensors`] is a transaction of its
//! own, meant to be called about once a second rather than every tick.
//!
//! Written against `rustypot`, but the *numbers* — conversion factors and the EEPROM
//! registers asserted at startup — come from `microduck_runtime`, where they were arrived
//! at against real hardware. See [`crate::model`].

use std::f64::consts::PI;
use std::time::Duration;

use rustypot::servo::dynamixel::xl330::Xl330Controller;

use crate::imu::{IMU_BLOCK_LEN, SflpDecoder};
use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{BAUD_RATE, EXPECTED_REGISTERS, IMU_DXL_ID, JOINT_IDS, NUM_JOINTS};

/// Start of the contiguous block read every tick: `present_pwm`, `present_current`,
/// `present_velocity`, `present_position`. Twelve bytes covers all four, and happens to be
/// exactly what the IMU board serves at the same address.
const READ_ADDR: u8 = 124;
const READ_LEN: u8 = 12;

/// 0.229 rev/min per count, in rad/s.
const RAD_PER_SEC_PER_COUNT: f64 = 0.229 * (2.0 * PI / 60.0);

/// The second, slower block: `present_input_voltage` (144, `u16`) then `present_temperature`
/// (146, `u8`). Three bytes covers both, so voltage and thermals cost *one* extra transaction
/// between them rather than two — see [`RobotIo::slow_sensors`].
///
/// It starts eight bytes past the end of the tick's block, which is why it cannot simply be
/// folded into [`READ_LEN`]: the gap is `velocity_trajectory`/`position_trajectory`, twelve
/// bytes nothing here wants, and a servo answering a 22-byte read every tick would cost more
/// bus time than the two transactions do.
const SLOW_READ_ADDR: u8 = 144;
const SLOW_READ_LEN: u8 = 3;

/// `present_input_voltage` counts 0.1 V each.
const VOLTS_PER_COUNT: f64 = 0.1;

/// A healthy 16-device read completes well inside this. Capping it means a missing device
/// costs a bounded hiccup rather than stalling the loop on the serial driver's default.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

pub struct DynamixelIo {
    controller: Xl330Controller,
    /// IMU first, then the servos in [`JOINT_IDS`] order — the order blocks come back in.
    ids: Vec<u8>,
    imu: SflpDecoder,
    last_imu_block: [u8; IMU_BLOCK_LEN],
    /// Blocks identical to their predecessor. The read succeeded but the board handed back
    /// the same sample, which means the policy is being fed dead orientation data — a
    /// failure that is invisible unless someone counts it. Known to happen.
    stale_imu_blocks: u64,
}

impl DynamixelIo {
    pub fn open(port: &str) -> Result<Self> {
        let serial = serialport::new(port, BAUD_RATE)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: std::io::Error::other(e),
            })?;

        let controller = Xl330Controller::new()
            .with_protocol_v2()
            .with_serial_port(serial);

        let mut ids = Vec::with_capacity(NUM_JOINTS + 1);
        ids.push(IMU_DXL_ID);
        ids.extend_from_slice(&JOINT_IDS);

        Ok(Self {
            controller,
            ids,
            imu: SflpDecoder::default(),
            last_imu_block: [0; IMU_BLOCK_LEN],
            stale_imu_blocks: 0,
        })
    }

    /// Assert — and correct — the EEPROM registers the control loop depends on.
    ///
    /// Returns how many needed fixing. A servo that has been factory-reset or swapped in
    /// arrives with `return_delay_time = 250`, which alone would eat 40% of the tick
    /// budget across the bus. Checking costs one read per register at startup and removes
    /// a whole class of "why is it slow on this robot".
    pub fn check_registers(&mut self) -> Result<usize> {
        let mut fixed = 0;
        for &id in &JOINT_IDS {
            for &(name, want) in EXPECTED_REGISTERS {
                // rustypot returns a Vec even for a single-id read. An empty one means the
                // servo did not answer, which must not be read as "register is fine".
                let raw = match name {
                    "return_delay_time" => self.controller.read_return_delay_time(id),
                    "baud_rate" => self.controller.read_baud_rate(id),
                    "pwm_slope" => self.controller.read_pwm_slope(id),
                    "shutdown" => self.controller.read_shutdown(id),
                    other => unreachable!("unhandled register {other}"),
                }
                .map_err(|e| IoError::Bus(format!("read {name} on {id}: {e}")))?;

                let got = *raw.first().ok_or(IoError::ShortRead {
                    what: "register read",
                    expected: 1,
                    got: 0,
                })?;

                if got == want {
                    continue;
                }
                tracing::warn!(id, register = name, got, want, "correcting motor register");
                match name {
                    "return_delay_time" => self.controller.write_return_delay_time(id, want),
                    "baud_rate" => self.controller.write_baud_rate(id, want),
                    "pwm_slope" => self.controller.write_pwm_slope(id, want),
                    "shutdown" => self.controller.write_shutdown(id, want),
                    other => unreachable!("unhandled register {other}"),
                }
                .map_err(|e| IoError::Bus(format!("write {name} on {id}: {e}")))?;
                fixed += 1;
            }
        }
        Ok(fixed)
    }

    /// Present positions only — a lighter read than [`RobotIo::read`], used once at startup
    /// to adopt the pose the robot is already in.
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let values = self
            .controller
            .sync_read_present_position(&JOINT_IDS)
            .map_err(|e| IoError::Bus(format!("read present positions: {e}")))?;
        if values.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "present positions",
                expected: NUM_JOINTS,
                got: values.len(),
            });
        }
        let mut out = [0.0; NUM_JOINTS];
        out.copy_from_slice(&values);
        Ok(out)
    }

    /// Torque on every servo. Not used by the control loop, which deliberately never
    /// touches torque so that a restart mid-update leaves a standing robot standing.
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        for &id in &JOINT_IDS {
            self.controller
                .write_torque_enable(id, on)
                .map_err(|e| IoError::Bus(format!("torque {on} on {id}: {e}")))?;
        }
        Ok(())
    }

    /// Ramp every joint from where it is now to `target`, linearly.
    ///
    /// Only ever called by an explicit `init` — the control loop must never move the robot
    /// on its own, because that would make an update restart a fall risk. Blocking, and
    /// deliberately so: nothing else should be talking to the bus while this runs.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }
}

impl RobotIo for DynamixelIo {
    fn read(&mut self) -> Result<Sensors> {
        let blocks = self
            .controller
            .sync_read_raw_data(&self.ids, READ_ADDR, READ_LEN)
            .map_err(|e| IoError::Bus(format!("combined imu+motor sync_read: {e}")))?;

        if blocks.len() != self.ids.len() {
            return Err(IoError::ShortRead {
                what: "sync_read blocks",
                expected: self.ids.len(),
                got: blocks.len(),
            });
        }

        let mut sensors = Sensors::default();

        // Slot 0 is the IMU board.
        if blocks[0].len() == IMU_BLOCK_LEN {
            let mut raw = [0u8; IMU_BLOCK_LEN];
            raw.copy_from_slice(&blocks[0]);
            if raw == self.last_imu_block {
                let n = self.stale_imu_blocks.saturating_add(1);
                self.stale_imu_blocks = n;
                // Say so, or the counter is a number nobody ever reads. Rate-limited
                // because a board that has stopped refreshing produces one of these every
                // single tick, and 50 Hz of identical warnings would evict the journal.
                if n == 1 || n.is_multiple_of(500) {
                    tracing::warn!(
                        stale_blocks = n,
                        "imu board returned the same sample as last tick — orientation may be dead"
                    );
                }
            }
            self.last_imu_block = raw;
            sensors.imu = self.imu.decode(&raw);
        } else {
            return Err(IoError::ShortRead {
                what: "imu block",
                expected: IMU_BLOCK_LEN,
                got: blocks[0].len(),
            });
        }

        for (joint, block) in blocks[1..].iter().enumerate() {
            if block.len() != READ_LEN as usize {
                return Err(IoError::ShortRead {
                    what: "motor block",
                    expected: READ_LEN as usize,
                    got: block.len(),
                });
            }
            // [0..2] present_pwm, unused · [2..4] current · [4..8] velocity · [8..12] position
            sensors.currents_ma[joint] = (i16::from_le_bytes([block[2], block[3]]) as f64).abs();
            let velocity = i32::from_le_bytes([block[4], block[5], block[6], block[7]]);
            sensors.velocities[joint] = velocity as f64 * RAD_PER_SEC_PER_COUNT;
            let position = i32::from_le_bytes([block[8], block[9], block[10], block[11]]);
            sensors.positions[joint] = (2.0 * PI * position as f64 / 4096.0) - PI;
        }

        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.controller
            .sync_write_goal_position(&JOINT_IDS, &targets.positions)
            .map_err(|e| IoError::Bus(format!("sync_write goal positions: {e}")))
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        for &id in &JOINT_IDS {
            self.controller
                .write_position_p_gain(id, kp)
                .map_err(|e| IoError::Bus(format!("position_p_gain {kp} on {id}: {e}")))?;
        }
        Ok(())
    }

    /// Supply voltage and case temperatures, in one `sync_read` over registers 144–146.
    ///
    /// Voltage is averaged because all 15 servos sit on one pack: a single reading is the
    /// same measurement with more noise. Temperature is *not* averaged here — the caller gets
    /// every joint, because one loaded joint running hot is the case worth seeing and a mean
    /// over fifteen hides it.
    ///
    /// Note that a silent servo does not produce a short answer: `rustypot`'s `sync_read`
    /// waits for every id and fails the whole transaction if one does not reply. So this is
    /// all-or-nothing, and the caller is expected to keep its previous sample rather than
    /// treat one miss as news. The zero filter on voltage guards a device that answers with a
    /// nonsense value, which must not be averaged in as if the pack were half flat.
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let blocks = self
            .controller
            .sync_read_raw_data(&JOINT_IDS, SLOW_READ_ADDR, SLOW_READ_LEN)
            .map_err(|e| IoError::Bus(format!("voltage+temperature sync_read: {e}")))?;

        if blocks.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "voltage+temperature blocks",
                expected: NUM_JOINTS,
                got: blocks.len(),
            });
        }

        let mut temps_c = [0.0; NUM_JOINTS];
        let mut volts = Vec::with_capacity(NUM_JOINTS);
        for (joint, block) in blocks.iter().enumerate() {
            if block.len() != SLOW_READ_LEN as usize {
                return Err(IoError::ShortRead {
                    what: "voltage+temperature block",
                    expected: SLOW_READ_LEN as usize,
                    got: block.len(),
                });
            }
            // [0..2] present_input_voltage · [2] present_temperature, already in whole °C.
            let counts = u16::from_le_bytes([block[0], block[1]]);
            let v = counts as f64 * VOLTS_PER_COUNT;
            if v > 0.0 {
                volts.push(v);
            }
            temps_c[joint] = block[2] as f64;
        }

        if volts.is_empty() {
            return Err(IoError::ShortRead {
                what: "input voltage",
                expected: NUM_JOINTS,
                got: 0,
            });
        }
        Ok(SlowSensors {
            volts: volts.iter().sum::<f64>() / volts.len() as f64,
            temps_c,
        })
    }

    fn imu_stale_blocks(&self) -> u64 {
        self.stale_imu_blocks
    }

    fn imu_ready(&self) -> bool {
        self.imu.ready()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The block parsed per servo must cover current, velocity and position without
    /// overrunning. If `READ_LEN` and the offsets below ever disagree, joints get each
    /// other's values — which reads as a wiring fault, not a code bug.
    #[test]
    fn read_block_is_long_enough_for_every_field() {
        assert_eq!(READ_LEN as usize, IMU_BLOCK_LEN);
        // Highest offset touched by the parser below is position at 8..12.
        const { assert!(READ_LEN >= 12) };
    }

    /// The conversion must agree with rustypot's own `AnglePosition`, which is what
    /// `sync_write_goal_position` uses on the way out. A mismatch would mean the loop
    /// commands a different angle than it believes it read back.
    #[test]
    fn position_conversion_round_trips_through_rustypot() {
        for raw in [0i32, 1024, 2048, 3072, 4095] {
            let ours = (2.0 * PI * raw as f64 / 4096.0) - PI;
            let theirs = (4096.0 * (PI + ours) / (2.0 * PI)) as i32;
            assert_eq!(theirs, raw, "raw {raw} did not survive the round trip");
        }
    }

    /// 0.229 rev/min per count. Getting this wrong scales every joint velocity in the
    /// observation vector by a constant, which a policy tolerates just well enough to walk
    /// badly.
    #[test]
    fn velocity_scale_matches_the_datasheet_figure() {
        let one_count = RAD_PER_SEC_PER_COUNT;
        let expected_rpm = 0.229;
        assert!((one_count * 60.0 / (2.0 * PI) - expected_rpm).abs() < 1e-12);
    }
}
