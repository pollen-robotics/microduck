//! The robot, as data.
//!
//! One variant — **alpha** — because that is the only robot that exists. Every shipped
//! policy is `alpha_*`; v1/v1.5/v1.6 are history. A second revision becomes a second set
//! of tables, which is honest until there is a second robot to generalise from.
//!
//! The numeric values here are lifted from `microduck_runtime`'s `motor.rs`, where they
//! were measured against hardware rather than derived. Re-deriving them from a datasheet
//! is exactly the kind of change that looks right and walks wrong.

/// Left leg (5) · neck/head/mouth (5) · right leg (5).
pub const NUM_JOINTS: usize = 15;

/// Dynamixel IDs, indexed as [`JOINT_NAMES`] — from the protocol crate too, since
/// `robot.health` names a missing servo by its ID and a client has to say which joint it is.
pub use duck_ipc_proto::JOINT_IDS;

/// Joint names, from the protocol crate — the wire indexes `joints` and `targets`
/// positionally, so that order and this one cannot be allowed to drift apart. The
/// assertion below is what makes "cannot" true.
pub use duck_ipc_proto::JOINT_NAMES;

const _: () = assert!(JOINT_NAMES.len() == NUM_JOINTS);
const _: () = assert!(JOINT_IDS.len() == NUM_JOINTS);

/// The mouth is absent from every alpha policy — they are all 61-D observation, 14-action,
/// and the action vector skips this index. Named so that omission is deliberate rather
/// than an off-by-one someone has to rediscover.
pub const MOUTH_INDEX: usize = 9;

/// Home pose. The trunk sits ~5 mm further forward than the v1.5 pose so the CoM is over
/// the ankle axis; the old pose biased the robot backwards.
///
/// Must match `HOME_FRAME` in the training env — a policy is trained against these angles
/// and observes joint positions *relative* to them, so a discrepancy here is a constant
/// offset on 14 observation slots.
pub const DEFAULT_POSITION: [f64; NUM_JOINTS] = [
    0.0,     // left_hip_yaw
    -0.0873, // left_hip_roll
    -0.4579, // left_hip_pitch
    -0.0049, // left_knee
    0.4530,  // left_ankle
    0.3491,  // neck_pitch
    0.3491,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // mouth
    0.0,     // right_hip_yaw
    0.0873,  // right_hip_roll
    0.4579,  // right_hip_pitch
    0.0049,  // right_knee
    -0.4530, // right_ankle
];

/// Rest pose: where a seated robot settles with its torque off. Recorded on graphite-bang
/// sitting stably untorqued (2026-10-03), averaged over a second of samples.
///
/// The shutdown eases the joints here from the policy's seat before cutting torque. The seat
/// the sitstand network holds is not where the robot comes to rest once nothing holds it, so
/// cutting torque straight from the seat let the robot sag forward. Arriving here first, it
/// is already where it would fall to.
pub const REST_POSITION: [f64; NUM_JOINTS] = [
    0.3129,  // left_hip_yaw
    0.4050,  // left_hip_roll
    -0.2338, // left_hip_pitch
    1.5493,  // left_knee
    0.5522,  // left_ankle
    0.9572,  // neck_pitch
    1.4787,  // head_pitch
    -0.0129, // head_yaw
    -0.0522, // head_roll
    -0.0121, // mouth
    0.0353,  // right_hip_yaw
    -0.5660, // right_hip_roll
    0.2853,  // right_hip_pitch
    -1.4696, // right_knee
    -0.4096, // right_ankle
];

/// Mouth travel, radians: closed and fully open. The alpha reuses the v1.6 range,
/// −5°..+30°, from `microduck_runtime`'s `variant.rs`.
///
/// The mouth is not part of any policy — every alpha network is 14 actions with this joint
/// skipped — so these two numbers and [`mouth_target`] are the whole of mouth control.
pub const MOUTH_CLOSED: f64 = -5.0 * std::f64::consts::PI / 180.0;
pub const MOUTH_OPEN: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Joint angle for a mouth opening fraction. 0 is closed, 1 is fully open; anything outside
/// is clamped rather than fed to a servo as an out-of-travel target.
pub fn mouth_target(open: f64) -> f64 {
    let open = if open.is_finite() {
        open.clamp(0.0, 1.0)
    } else {
        0.0
    };
    MOUTH_CLOSED + open * (MOUTH_OPEN - MOUTH_CLOSED)
}

/// The `imu_to_dxl` v2 board's Dynamixel ID. It rides the motor bus and is read in the
/// same transaction as the servos ([`crate::bus`]).
pub const IMU_DXL_ID: u8 = 200;

pub const BAUD_RATE: u32 = 1_000_000;

/// What a servo answers as out of the box: ID 1 at 57 600 baud. Both are deliberately unused
/// on this bus — no joint is ID 1 and nothing runs at that speed — which is what lets a
/// replacement be told apart from every servo already fitted ([`crate::bus`]).
pub const FACTORY_ID: u8 = 1;
pub const FACTORY_BAUD_RATE: u32 = 57_600;

/// EEPROM registers asserted (and corrected) at startup.
///
/// `return_delay_time` is the load-bearing one: the XL330 ships at 250, which is 500 µs of
/// turnaround *per device*. Across 16 devices that is 8 ms per tick — 40% of a 20 ms budget
/// — spent waiting for servos to get around to answering. The rest are here because the
/// runtime found them worth pinning.
///
/// `shutdown = 52` is `0b110100` — overload, electrical shock, overheating — with the
/// input-voltage bit **clear**, where the factory's 53 sets it. That bit is what clears torque
/// once the supply passes the servo's `Max Voltage Limit`, which nothing here writes and which
/// therefore stays at its default 7.0 V. A charged 2S pack sits above that, so the clear bit is
/// the reason fifteen servos do not latch themselves off a fully charged battery. Read as
/// "latches on input-voltage faults" it says the opposite of what it does.
pub const EXPECTED_REGISTERS: &[(&str, u8)] = &[
    ("return_delay_time", 0),
    ("baud_rate", 3), // 3 = 1 Mbps, must agree with BAUD_RATE
    ("pwm_slope", 255),
    ("shutdown", 52),
];

/// `current_limit`, mA, asserted alongside [`EXPECTED_REGISTERS`] and kept apart because it is a
/// `u16`. The XL330 ships at 1750. 1000 is what a microduck walks the same ONNX with, and what the
/// gear-breakage bench ran ROBOTIS's impact guard firmware at; that firmware is the one that
/// enforces it in position control, the factory's only does so in the current-based modes.
pub const CURRENT_LIMIT_MA: u16 = 1000;

/// `firmware_version` of ROBOTIS's impact guard firmware (V3): a servo that hits something brakes
/// and then holds softly, which is what keeps the gear teeth whole. Its settings are
/// [`ImpactGuard`]; register 66 says what the guard is doing
/// ([`crate::io::RobotIo::impact_guard`]).
pub const IMPACT_GUARD_FIRMWARE: u8 = 3;

/// The impact guard's six settings on [`IMPACT_GUARD_FIRMWARE`], in the registers' own units.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImpactGuard {
    /// V Min, rad/s: an impact counts only while the servo moves toward its goal faster than this.
    pub v_min: u8,
    /// Brake Time, ms: how long the H-bridge is shorted after an impact.
    pub brake_ms: u8,
    /// Deceleration, rad/s²: what counts as an impact.
    pub decel: u16,
    /// i Hold, mA: the current the hold allows after the brake, until the goal comes back past
    /// the position.
    pub i_hold_ma: u16,
    /// V Fast, rad/s: where the PWM limit has fallen to `pwm_fast`. Never 0: the firmware divides
    /// by it.
    pub v_fast: u8,
    /// PWM Fast, % of PWM Limit: the limit at `v_fast` and above.
    pub pwm_fast: u8,
}

impl ImpactGuard {
    /// ROBOTIS's factory values, which are the gear-breakage bench's: brake 50 ms on a
    /// deceleration past 400 rad/s² toward the goal, then hold at 0.5 A; and a PWM limit falling
    /// from PWM Limit at rest to 50 % at 9 rad/s.
    pub const FACTORY: Self = Self {
        v_min: 1,
        brake_ms: 50,
        decel: 400,
        i_hold_ma: 500,
        v_fast: 9,
        pwm_fast: 50,
    };

    /// The factory firmware with only Current Limit: a deceleration no impact reaches, a hold
    /// current the motor never draws, and PWM Fast at the 100 % PWM Limit this robot keeps.
    pub const OFF: Self = Self {
        decel: 32767,
        i_hold_ma: 32767,
        pwm_fast: 100,
        ..Self::FACTORY
    };

    /// The two EEPROM blocks the settings fill, as (address, bytes): 40–43 (V Min, Brake Time,
    /// Deceleration `u16`) and 56–59 (i Hold `u16`, V Fast, PWM Fast).
    pub fn blocks(self) -> [(u8, [u8; 4]); 2] {
        let [d0, d1] = self.decel.to_le_bytes();
        let [h0, h1] = self.i_hold_ma.to_le_bytes();
        [
            (40, [self.v_min, self.brake_ms, d0, d1]),
            (56, [h0, h1, self.v_fast, self.pwm_fast]),
        ]
    }
}

/// `homing_offset` on the knees, in raw position counts (0.088° each, so −45°).
///
/// Every other joint is at 0. The value is `microduck_runtime`'s `setup_motor_rpi.py`, the
/// flashing rig every servo on a built robot went through, so [`DEFAULT_POSITION`] and every
/// policy assume it. A servo straight from the box arrives at 0, so a knee adopted without this
/// reads and drives 45° off, with nothing in the journal to say why.
pub const KNEE_HOMING_OFFSET: i32 = -512;

/// Right and left knee: the only joints whose `homing_offset` is not 0.
pub const KNEE_IDS: [u8; 2] = [13, 23];

/// The `homing_offset` a servo should hold, asserted alongside [`EXPECTED_REGISTERS`]. It is
/// kept apart from them because it is an `i32` and differs per joint.
pub fn homing_offset(id: u8) -> i32 {
    if KNEE_IDS.contains(&id) {
        KNEE_HOMING_OFFSET
    } else {
        0
    }
}

/// Index of a joint by name. Linear scan over 15 entries, used at startup and in tests.
pub fn joint_index(name: &str) -> Option<usize> {
    JOINT_NAMES.iter().position(|n| *n == name)
}

// ── battery ──────────────────────────────────────────────────────────────────
//
// There is no fuel gauge and no ADC. The only measurement available is what the servos
// report as their own supply (`crate::bus::DynamixelIo::bus_voltage`), which is the pack
// seen through the bus — so it sags under load and recovers when the robot stands still.
// That is why the span below is *usable-under-load*, not the cell chemistry's range.
//
// The span is a statement about the *rail*, and it holds because that rail is the pack: the
// servos are fed the 2S battery, not a regulated 5 V. A servo on a bench supply reads its own
// supply voltage like any other, so 5 V lands under `BATTERY_EMPTY_V` and maps to 0% — the
// mapping working as defined rather than a fault to chase. Anything that regulates the servo
// rail has to move these two constants with it.

/// Off a full charge, under load. NP-F550, 2S Li-ion.
pub const BATTERY_FULL_V: f64 = 8.2;

/// The sag floor: below this the robot starts struggling, well before the pack's own
/// protection trips. Empty for our purposes, not empty for the cells'.
pub const BATTERY_EMPTY_V: f64 = 6.6;

/// Fraction of a pack, 0–100, for a bus voltage.
///
/// Linear, and the numbers come from `microduck_runtime`'s `check_battery`, where they were
/// arrived at by running a duck flat. It lives here rather than in `robotd` so there is one
/// mapping: the prototype had it in a CLI *and* re-derived in the app, which is how two
/// screens end up disagreeing about the same pack.
///
/// A non-finite or non-positive reading is 0 — those mean "no answer from the bus", and the
/// caller is expected to report that as unknown rather than to display this number.
pub fn battery_percent(volts: f64) -> f64 {
    if !volts.is_finite() || volts <= 0.0 {
        return 0.0;
    }
    ((volts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)).clamp(0.0, 1.0) * 100.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three tables are indexed by the same integer everywhere in the crate. If they
    /// ever diverge in length, every lookup silently reads the wrong joint.
    #[test]
    fn tables_agree_on_length() {
        assert_eq!(JOINT_IDS.len(), NUM_JOINTS);
        assert_eq!(JOINT_NAMES.len(), NUM_JOINTS);
        assert_eq!(DEFAULT_POSITION.len(), NUM_JOINTS);
    }

    /// A duplicated Dynamixel ID makes a `sync_read` return blocks that cannot be matched
    /// back to joints, and a `sync_write` command two joints at once. Both fail in ways
    /// that look like a wiring fault.
    #[test]
    fn ids_are_unique() {
        let mut seen = JOINT_IDS;
        seen.sort_unstable();
        seen.windows(2)
            .for_each(|w| assert_ne!(w[0], w[1], "duplicate Dynamixel ID {}", w[0]));
    }

    /// The factory must be byte for byte what ROBOTIS ships, the settings the bench validated;
    /// off must be unreachable rather than merely high, and keep V Fast off zero, which the
    /// firmware divides by.
    #[test]
    fn impact_guard_factory_is_robotis_and_off_never_fires() {
        assert_eq!(
            ImpactGuard::FACTORY.blocks(),
            [(40, [1, 50, 0x90, 0x01]), (56, [0xF4, 0x01, 9, 50])]
        );
        let [(_, low), (_, high)] = ImpactGuard::OFF.blocks();
        assert_eq!(u16::from_le_bytes([low[2], low[3]]), 32767, "deceleration");
        assert_eq!(u16::from_le_bytes([high[0], high[1]]), 32767, "i hold");
        assert_eq!(high[3], 100, "pwm fast: no taper");
        assert_ne!(high[2], 0, "v fast");
    }

    /// Every setting lands at its own address, the two `u16`s little-endian.
    #[test]
    fn impact_guard_settings_fill_their_registers() {
        let guard = ImpactGuard {
            v_min: 2,
            brake_ms: 30,
            decel: 1000,
            i_hold_ma: 800,
            v_fast: 8,
            pwm_fast: 70,
        };
        assert_eq!(
            guard.blocks(),
            [(40, [2, 30, 0xE8, 0x03]), (56, [0x20, 0x03, 8, 70])]
        );
    }

    /// The IMU board shares the bus with the servos, so its ID must not collide with one.
    #[test]
    fn imu_id_does_not_collide_with_a_joint() {
        assert!(!JOINT_IDS.contains(&IMU_DXL_ID));
    }

    /// The replacement path finds a new servo by the ID it ships with. If a joint ever took
    /// ID 1, a fresh servo would be indistinguishable from it — and flashing "the missing
    /// joint" onto ID 1 would re-address a servo that was never missing.
    #[test]
    fn factory_defaults_are_unused_on_the_bus() {
        assert!(!JOINT_IDS.contains(&FACTORY_ID));
        assert_ne!(IMU_DXL_ID, FACTORY_ID);
        assert_ne!(FACTORY_BAUD_RATE, BAUD_RATE);
    }

    /// `shutdown` is the one register whose *bits* are the decision rather than the number:
    /// the input-voltage bit is cleared on purpose, and that is what lets the pack's range run
    /// across a servo rated to 6.0 V. The factory default is 53 — one bit away — so a later
    /// edit that "restores the default" should meet a test rather than a comment.
    #[test]
    fn the_shutdown_mask_clears_the_input_voltage_bit() {
        let want = EXPECTED_REGISTERS
            .iter()
            .find(|(n, _)| *n == "shutdown")
            .map(|&(_, v)| v)
            .expect("shutdown is an expected register");
        const OVERLOAD: u8 = 1 << 5;
        const ELECTRICAL_SHOCK: u8 = 1 << 4;
        const OVERHEATING: u8 = 1 << 2;
        const INPUT_VOLTAGE: u8 = 1 << 0;
        assert_eq!(
            want,
            OVERLOAD | ELECTRICAL_SHOCK | OVERHEATING,
            "the three faults worth latching on, with input-voltage clear"
        );
        assert_eq!(want & INPUT_VOLTAGE, 0);
    }

    /// The knee IDs are positional facts about [`JOINT_IDS`]; a renumbering that moved them
    /// would put the offset on a hip.
    #[test]
    fn only_the_knees_have_a_homing_offset() {
        for (i, &id) in JOINT_IDS.iter().enumerate() {
            let want = if JOINT_NAMES[i].ends_with("_knee") {
                KNEE_HOMING_OFFSET
            } else {
                0
            };
            assert_eq!(homing_offset(id), want, "{}", JOINT_NAMES[i]);
        }
    }

    /// `MOUTH_INDEX` is used to skip a slot when mapping 14 policy actions onto 15 joints.
    /// Pointing it at the wrong joint would shift every action after it by one.
    #[test]
    fn mouth_index_names_the_mouth() {
        assert_eq!(JOINT_NAMES[MOUTH_INDEX], "mouth");
        assert_eq!(joint_index("mouth"), Some(MOUTH_INDEX));
    }

    /// The ends of the span and the middle of it. Getting the direction wrong here would
    /// report a full pack as flat, which is the kind of thing nobody double-checks.
    #[test]
    fn battery_percent_spans_the_usable_range() {
        assert_eq!(battery_percent(BATTERY_FULL_V), 100.0);
        assert_eq!(battery_percent(BATTERY_EMPTY_V), 0.0);
        assert!((battery_percent(7.4) - 50.0).abs() < 0.001);
    }

    /// Voltages outside the span are ordinary — a fresh pack reads over 8.2 V off the
    /// charger, and a robot being run into the ground reads under 6.6 V. Neither may
    /// produce a percentage outside 0–100 for a caller to display.
    #[test]
    fn battery_percent_clamps_rather_than_extrapolating() {
        assert_eq!(battery_percent(9.5), 100.0);
        assert_eq!(battery_percent(5.0), 0.0);
    }

    /// A bus that did not answer arrives here as 0.0, and NaN is what a mean over an empty
    /// set produces. Both must be 0 rather than a wild number a caller might print.
    #[test]
    fn battery_percent_treats_no_reading_as_zero() {
        assert_eq!(battery_percent(0.0), 0.0);
        assert_eq!(battery_percent(f64::NAN), 0.0);
        assert_eq!(battery_percent(-1.0), 0.0);
    }

    /// The mouth range is the prototype's: −5° closed, +30° open. A fraction outside 0..1
    /// (or a NaN from a broken client) must clamp rather than command a servo past travel.
    #[test]
    fn mouth_target_spans_the_prototype_range() {
        assert!((mouth_target(0.0) - (-5.0f64.to_radians())).abs() < 1e-12);
        assert!((mouth_target(1.0) - 30.0f64.to_radians()).abs() < 1e-12);
        assert_eq!(mouth_target(-3.0), mouth_target(0.0));
        assert_eq!(mouth_target(7.0), mouth_target(1.0));
        assert_eq!(mouth_target(f64::NAN), mouth_target(0.0));
    }

    /// The legs are mirrored: the roll/pitch/ankle pairs are equal and opposite. A sign
    /// typo in the home pose is invisible by inspection and makes the robot stand crooked.
    #[test]
    fn home_pose_legs_are_mirrored() {
        for (left, right) in [
            ("left_hip_roll", "right_hip_roll"),
            ("left_hip_pitch", "right_hip_pitch"),
            ("left_knee", "right_knee"),
            ("left_ankle", "right_ankle"),
        ] {
            let l = DEFAULT_POSITION[joint_index(left).unwrap()];
            let r = DEFAULT_POSITION[joint_index(right).unwrap()];
            assert!(
                (l + r).abs() < 1e-9,
                "{left} ({l}) and {right} ({r}) should be equal and opposite"
            );
        }
    }
}
