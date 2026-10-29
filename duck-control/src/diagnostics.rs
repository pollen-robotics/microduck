//! Selective native operations for the exclusive raw-maintenance controller.
use crate::io::{IoError, Result};
use crate::model::{JOINT_IDS, JOINT_NAMES, homing_offset};
pub use duck_ipc_proto::diagnostics::{Motor, Pid};

pub fn index(id: u8) -> Result<usize> {
    JOINT_IDS
        .iter()
        .position(|v| *v == id)
        .ok_or_else(|| IoError::Bus("unknown motor ID".into()))
}
pub fn empty_motor(id: u8) -> Result<Motor> {
    let i = index(id)?;
    Ok(Motor {
        joint: JOINT_NAMES[i].into(),
        id,
        observed_at_ns: 0,
        model_number: None,
        firmware_version: None,
        hardware_error: None,
        voltage_v: None,
        temperature_c: None,
        current_limit: None,
        pwm_limit: None,
        velocity_limit: None,
        shutdown: None,
        torque: None,
        position_counts: None,
        goal_rad: None,
        pid: None,
        homing_offset_counts: None,
        expected_homing_offset_counts: homing_offset(id),
        before_startup_fix: None,
        operating_mode: None,
        drive_mode: None,
        min_position_rad: None,
        max_position_rad: None,
        profile_velocity: None,
        profile_acceleration: None,
        error: None,
    })
}
pub fn position_valid(q: f64) -> bool {
    q.is_finite() && (-std::f64::consts::PI..=std::f64::consts::PI * 4094.0 / 4096.0).contains(&q)
}
pub fn pid_valid(p: Pid) -> bool {
    p.p <= 16383 && p.i <= 16383 && p.d <= 16383
}

/// Decode one XL330 control-table read. Offsets and conversions are shared by every motor.
pub fn decode_registers(id: u8, b: &[u8], before_startup_fix: Option<i32>) -> Result<Motor> {
    if b.len() != 148 {
        return Err(IoError::ShortRead {
            what: "diagnostic registers",
            expected: 148,
            got: b.len(),
        });
    }
    let u16at = |n: usize| u16::from_le_bytes([b[n], b[n + 1]]);
    let i32at = |n: usize| i32::from_le_bytes([b[n], b[n + 1], b[n + 2], b[n + 3]]);
    let rad =
        |n: usize| 2.0 * std::f64::consts::PI * i32at(n) as f64 / 4096.0 - std::f64::consts::PI;
    let mut m = empty_motor(id)?;
    m.model_number = Some(u16at(0));
    m.firmware_version = Some(b[6]);
    m.hardware_error = Some(b[70]);
    m.voltage_v = Some(u16at(144) as f64 * 0.1);
    m.temperature_c = Some(b[146]);
    m.current_limit = Some(u16at(38));
    m.pwm_limit = Some(u16at(36));
    m.velocity_limit = Some(i32at(44) as u32);
    m.shutdown = Some(b[63]);
    m.torque = Some(b[64] != 0);
    m.position_counts = Some(i32at(132));
    m.goal_rad = Some(rad(116));
    m.pid = Some(Pid {
        p: u16at(84),
        i: u16at(82),
        d: u16at(80),
    });
    m.homing_offset_counts = Some(i32at(20));
    m.before_startup_fix = before_startup_fix;
    m.operating_mode = Some(b[11]);
    m.drive_mode = Some(b[10]);
    m.min_position_rad = Some(rad(52));
    m.max_position_rad = Some(rad(48));
    m.profile_velocity = Some(i32at(112) as u32);
    m.profile_acceleration = Some(i32at(108) as u32);
    Ok(m)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn register_offsets_and_coordinate_conversion_are_exact() {
        let mut b = [0u8; 148];
        b[0..2].copy_from_slice(&1190u16.to_le_bytes());
        b[6] = 42;
        b[11] = 3;
        b[10] = 1;
        b[63] = 52;
        b[64] = 1;
        b[70] = 4;
        for (offset, n) in [
            (20, -512i32),
            (44, 100),
            (48, 4095),
            (52, 0),
            (108, 50),
            (112, 60),
            (116, 2048),
            (132, 1024),
        ] {
            b[offset..offset + 4].copy_from_slice(&n.to_le_bytes());
        }
        for (offset, n) in [
            (36, 885u16),
            (38, 1000),
            (80, 3),
            (82, 2),
            (84, 200),
            (144, 74),
        ] {
            b[offset..offset + 2].copy_from_slice(&n.to_le_bytes());
        }
        b[146] = 31;
        for id in JOINT_IDS {
            let m = decode_registers(id, &b, Some(0)).unwrap();
            assert_eq!(m.id, id);
            assert_eq!(m.pid, Some(Pid { p: 200, i: 2, d: 3 }));
            assert_eq!(m.goal_rad, Some(0.0));
            assert_eq!(m.position_counts, Some(1024));
            assert_eq!(m.homing_offset_counts, Some(-512));
            assert_eq!(m.before_startup_fix, Some(0));
            assert_eq!(m.model_number, Some(1190));
            assert_eq!(m.hardware_error, Some(4));
            assert_eq!(m.voltage_v, Some(7.4));
            assert_eq!(m.profile_acceleration, Some(50));
            assert_eq!(m.profile_velocity, Some(60));
        }
        assert!(decode_registers(34, &b[..147], None).is_err());
    }
}
