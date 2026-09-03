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
use crate::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
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

/// Run of consecutive stale reads at which the journal says something.
///
/// 25 reads is half a second at 50 Hz — the same span [`SflpDecoder::ready`] waits for before
/// it will call the chip's output a measurement, and far longer than any ordinary hiccup. Below
/// it the tracker stays quiet on purpose: warning on the very first repeated block is what
/// taught everyone to ignore this message. Kept in step with `ImuHealth::FROZEN_RUN`, which is
/// where the same threshold is applied to the health report — this crate is the hardware layer
/// and deliberately does not depend on the IPC vocabulary, so the number lives in both places.
const STALE_RUN_WARN: u64 = 25;

/// Detects an IMU board that answers without refreshing, by remembering the last block.
///
/// Split out from the read path so it can be tested without a serial port: the fault it
/// describes is one nothing else on the robot reports, and it would otherwise be verifiable
/// only against broken hardware.
#[derive(Debug, Default)]
struct StaleImuTracker {
    /// `None` until the first block. A fixed initial value cannot work here: it would have to
    /// be all zeros, and an all-zero block is exactly what a board whose SFLP table is still
    /// empty sends — scoring a stale read against a predecessor that never existed.
    last: Option<[u8; IMU_BLOCK_LEN]>,
    stale: ImuStale,
}

impl StaleImuTracker {
    /// Records one block and returns the length of the run it belongs to — 0 when the block is
    /// fresh, which is the overwhelmingly common answer.
    fn observe(&mut self, block: &[u8; IMU_BLOCK_LEN]) -> u64 {
        if self.last.replace(*block) == Some(*block) {
            self.stale.total = self.stale.total.saturating_add(1);
            self.stale.run = self.stale.run.saturating_add(1);
        } else {
            self.stale.run = 0;
        }
        self.stale.run
    }
}

pub struct DynamixelIo {
    controller: Xl330Controller,
    /// IMU first, then the servos in [`JOINT_IDS`] order — the order blocks come back in.
    ids: Vec<u8>,
    imu: SflpDecoder,
    /// Blocks identical to their predecessor. The read succeeded but the board handed back
    /// the same sample, which means the policy is being fed dead orientation data — a
    /// failure that is invisible unless someone counts it. Known to happen.
    stale_imu: StaleImuTracker,
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
            stale_imu: StaleImuTracker::default(),
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

    /// Torque on every servo.
    ///
    /// One transaction per joint, so this is not something to call per tick — the control loop calls
    /// it once, when someone enables the policy on a limp robot. See [`RobotIo::set_torque`] for what
    /// has *not* changed: nothing touches torque because a process started.
    ///
    /// **Every servo is written, whatever the others said.** Fourteen acknowledged transactions
    /// in a row, and one dropped ack used to end the loop there — which for `on = false` on the
    /// way to a power-off meant a robot that sat down and switched off with half its legs still
    /// locked, because the tick that saw the error was the last one that could have retried.
    /// Writing the rest costs the same as it would have, and the error names every joint that
    /// did not answer so the caller can decide whether to ask again.
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        let mut failed = Vec::new();
        for &id in &JOINT_IDS {
            if let Err(e) = self.controller.write_torque_enable(id, on) {
                failed.push(format!("torque {on} on {id}: {e}"));
            }
        }
        if failed.is_empty() {
            Ok(())
        } else {
            Err(IoError::Bus(failed.join("; ")))
        }
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
            // Say so, or the counters are numbers nobody ever reads — but only once the run
            // is long enough to mean something. Rate-limited past that because a board which
            // has stopped refreshing produces one of these every single tick, and 50 Hz of
            // identical warnings would evict the journal.
            let run = self.stale_imu.observe(&raw);
            if run == STALE_RUN_WARN || (run > STALE_RUN_WARN && run.is_multiple_of(500)) {
                tracing::warn!(
                    consecutive = run,
                    total = self.stale_imu.stale.total,
                    "imu board has returned the same sample {run} reads running — orientation is frozen"
                );
            }
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

    fn set_torque(&mut self, on: bool) -> Result<()> {
        // The inherent method, which predates the trait and is still what `robotd init` uses.
        DynamixelIo::set_torque(self, on)
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        // I and D are written too, at zero — the prototype's `--ki`/`--kd` defaults, which
        // its startup writes to every motor. These are RAM registers, so every power-up
        // restores the servo's factory values, and the factory D gain is not zero: left in
        // place it damps the servo's internal PID, and the robot runs measurably softer
        // than the prototype at the *same* kP. That is not a tuning choice anyone made, so
        // it is pinned here rather than exposed as a knob.
        const KI: u16 = 0;
        const KD: u16 = 0;
        for &id in &JOINT_IDS {
            self.controller
                .write_position_p_gain(id, kp)
                .map_err(|e| IoError::Bus(format!("position_p_gain {kp} on {id}: {e}")))?;
            self.controller
                .write_position_i_gain(id, KI)
                .map_err(|e| IoError::Bus(format!("position_i_gain {KI} on {id}: {e}")))?;
            self.controller
                .write_position_d_gain(id, KD)
                .map_err(|e| IoError::Bus(format!("position_d_gain {KD} on {id}: {e}")))?;
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

    fn imu_stale(&self) -> ImuStale {
        self.stale_imu.stale
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

    fn block(n: u8) -> [u8; IMU_BLOCK_LEN] {
        [n; IMU_BLOCK_LEN]
    }

    /// The first block has no predecessor, so it cannot be a repeat of one. This is not a
    /// hypothetical: the natural initial value is all zeros, and an all-zero block is what a
    /// board sends before SFLP has written its table — which used to score a stale read on the
    /// very first tick of every boot, and put a permanent 1 in a counter rendered as an alarm.
    #[test]
    fn the_first_block_is_never_stale() {
        let mut t = StaleImuTracker::default();
        assert_eq!(t.observe(&block(0)), 0);
        assert_eq!(t.stale.total, 0);
    }

    /// Fresh blocks must leave both counters alone. The whole point of the run is that it means
    /// "right now", so anything that does not repeat has to clear it.
    #[test]
    fn fresh_blocks_count_for_nothing() {
        let mut t = StaleImuTracker::default();
        for n in 0..10 {
            assert_eq!(t.observe(&block(n)), 0);
        }
        assert_eq!(t.stale, ImuStale { total: 0, run: 0 });
    }

    /// A hiccup: two identical blocks, then the board recovers. The total remembers it — that
    /// is what makes "9 over 40 minutes" sayable — while the run goes back to zero, because
    /// orientation is live again and nothing should be shouting.
    #[test]
    fn a_hiccup_is_remembered_in_the_total_but_not_the_run() {
        let mut t = StaleImuTracker::default();
        t.observe(&block(1));
        assert_eq!(t.observe(&block(1)), 1, "the repeat is the first of a run");
        assert_eq!(t.observe(&block(2)), 0, "a fresh block ends the run");
        assert_eq!(t.stale, ImuStale { total: 1, run: 0 });
    }

    /// A board that has stopped refreshing repeats forever, and the run is what separates that
    /// from the hiccup above. It has to reach the threshold the journal and the health report
    /// both key off, or a genuinely dead IMU is never reported at all.
    #[test]
    fn a_dead_board_runs_past_the_warning_threshold() {
        let mut t = StaleImuTracker::default();
        t.observe(&block(7));
        for _ in 0..STALE_RUN_WARN {
            t.observe(&block(7));
        }
        assert_eq!(t.stale.run, STALE_RUN_WARN);
        assert_eq!(t.stale.total, STALE_RUN_WARN);
    }

    /// Runs accumulate into the same total across separate episodes: the total is "how often
    /// has this ever happened", not "how bad is it now".
    #[test]
    fn separate_episodes_add_up() {
        let mut t = StaleImuTracker::default();
        for n in 0..3u8 {
            t.observe(&block(n));
            t.observe(&block(n));
            t.observe(&block(n));
        }
        assert_eq!(t.stale.total, 6, "two repeats in each of three episodes");
        assert_eq!(t.stale.run, 2, "the last episode was still going");
    }
}
