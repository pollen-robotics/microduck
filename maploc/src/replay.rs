//! Offline reader for `.mdlg` session logs.
//!
//! Mirrors `tools/replay_session.py` but skips the TCP indirection — feeds
//! decoded ToF frames and digital-twin packets directly into the v2 dev
//! loop:
//!
//! ```ignore
//! use microduck_maploc::replay::{SessionReplayer, Record};
//! for record in SessionReplayer::open("session.mdlg")? {
//!     match record? {
//!         Record::Tof(t)  => /* t.ranges_m, t.status, t.ts_us */ ,
//!         Record::Twin(d) => /* d.odom_x, d.odom_yaw, d.quat_wxyz, ... */ ,
//!     }
//! }
//! ```
//!
//! Wire format (little-endian, kept stable):
//!
//! ```text
//! header (16 B):
//!   magic         : 4 bytes  "MDLG"
//!   version       : u32      currently 1
//!   epoch_unix_ms : u64      capture start (ms since Unix epoch)
//!
//! record (until EOF):
//!   ts_us         : u64      microseconds since recorder start
//!   stream_id     : u8       0 = ToF, 1 = digital twin
//!   size          : u32      payload size
//!   payload       : u8[size] verbatim TCP wire bytes
//! ```
//!
//! The ToF payload format is the one emitted by `tof_streamer.py`:
//!
//! ```text
//! f64 ts_sender_s | u8 rows | u8 cols | u8[2] reserved
//!                 | f32[rows*cols] ranges_m  (NaN = invalid)
//!                 | u8 [rows*cols] target_status
//! ```
//!
//! The digital-twin payload is the packet documented in
//! `microduck_runtime/src/main.rs` (twin streaming block): 172 B legacy,
//! 180 B with the appended contact-anchor fields. Both decode.

use std::fs::File;
use std::io::{self, BufReader, ErrorKind, Read};
use std::path::Path;

const MAGIC: &[u8; 4] = b"MDLG";
/// Version 1 is the prototype's format (twin packets); version 2 is what
/// [`crate::record::SessionRecorder`] writes (odom records). Both read.
const VERSIONS: [u32; 2] = [1, 2];

const STREAM_TOF: u8 = 0;
const STREAM_TWIN: u8 = 1;
const STREAM_ODOM: u8 = 2;

const TOF_ROWS: usize = 8;
const TOF_COLS: usize = 8;

/// Legacy digital-twin packet: 8 B timestamp + 41 × f32. Newer runtimes
/// append 2 × f32 (contact-anchor world x, y) for 180 B total; both
/// sizes are accepted and the anchor surfaces as `Option`.
const TWIN_PACKET_SIZE_V1: usize = 8 + 41 * 4; // 172 B
const TWIN_PACKET_SIZE_V2: usize = 8 + 43 * 4; // 180 B (adds contact anchor)

#[derive(Debug, Clone)]
pub struct TofRecord {
    /// Recorder-side timestamp (µs since session start).
    pub ts_us: u64,
    /// Sender (Pi monotonic) timestamp from the payload header.
    pub sender_ts_s: f64,
    /// Per-zone slant ranges, metres. NaN means the chip flagged it
    /// invalid (status code outside the valid set).
    pub ranges_m: [[f32; TOF_COLS]; TOF_ROWS],
    /// Raw VL53L5CX target_status per zone. 5 / 6 = valid; everything
    /// else = various failure modes. The streamer already NaNs the
    /// corresponding `ranges_m`, but we surface the byte too in case
    /// downstream tooling wants to slice differently.
    pub status: [[u8; TOF_COLS]; TOF_ROWS],
}

#[derive(Debug, Clone, Copy)]
pub struct TwinRecord {
    /// Recorder-side timestamp (µs since session start).
    pub ts_us: u64,
    /// Sender timestamp from the packet header (runtime monotonic).
    pub sender_ts_s: f64,
    /// IMU quaternion `[w, x, y, z]`, body→world.
    pub quat_wxyz: [f32; 4],
    /// 15 joint positions, runtime motor order.
    pub joints: [f32; 15],
    /// 15 motor currents, mA.
    pub motor_currents_ma: [f32; 15],
    pub odom_x: f32,
    pub odom_y: f32,
    /// Ball world XYZ; any component may be NaN if not detected.
    pub ball_xyz: [f32; 3],
    pub odom_z: f32,
    pub odom_yaw: f32,
    /// Contact-odometry foot-anchor world (x, y). `None` on legacy
    /// 172 B recordings that predate the field; may be NaN when no
    /// odometry engine was running.
    pub contact_anchor: Option<(f32, f32)>,
}

/// A version-2 robot-state record — what robotd's mapping worker consumed
/// on one control-loop tick. See [`crate::record`] for the layout.
#[derive(Debug, Clone, Copy)]
pub struct OdomRecord {
    /// Recorder-side timestamp (µs since session start).
    pub ts_us: u64,
    pub odom_x: f32,
    pub odom_y: f32,
    pub odom_yaw: f32,
    /// Projected gravity, body frame.
    pub gravity: [f32; 3],
    /// Trunk height above the floor, metres.
    pub trunk_z: f32,
    /// neck_pitch, head_pitch, head_yaw, head_roll.
    pub head: [f32; 4],
    /// The control loop's verdict (scripted move mid-flight or walking).
    pub moving: bool,
    /// Seated — the ground-truth protocol's kidnap marker: a carried robot
    /// is sat first, and odometry cannot see a carry but cannot miss a sit.
    pub sitting: bool,
    /// Fallen over — a fall can displace and rotate the robot.
    pub fallen: bool,
}

#[derive(Debug, Clone)]
pub enum Record {
    Tof(TofRecord),
    Twin(TwinRecord),
    Odom(OdomRecord),
}

impl Record {
    pub fn ts_us(&self) -> u64 {
        match self {
            Record::Tof(t) => t.ts_us,
            Record::Twin(t) => t.ts_us,
            Record::Odom(o) => o.ts_us,
        }
    }
}

pub struct SessionReplayer {
    reader: BufReader<File>,
    epoch_unix_ms: u64,
}

impl SessionReplayer {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let file = File::open(path)?;
        let mut reader = BufReader::new(file);
        let mut magic = [0u8; 4];
        reader.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("not an mdlg file (magic = {:?})", magic),
            ));
        }
        let version = read_u32(&mut reader)?;
        if !VERSIONS.contains(&version) {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "unsupported mdlg version {} (this build reads {:?})",
                    version, VERSIONS
                ),
            ));
        }
        let epoch_unix_ms = read_u64(&mut reader)?;
        Ok(Self {
            reader,
            epoch_unix_ms,
        })
    }

    pub fn epoch_unix_ms(&self) -> u64 {
        self.epoch_unix_ms
    }
}

impl Iterator for SessionReplayer {
    type Item = io::Result<Record>;

    fn next(&mut self) -> Option<Self::Item> {
        // Try to read the next record header. UnexpectedEof here means
        // end-of-stream, which is the normal way to stop iterating.
        let ts_us = match read_u64(&mut self.reader) {
            Ok(v) => v,
            Err(e) if e.kind() == ErrorKind::UnexpectedEof => return None,
            Err(e) => return Some(Err(e)),
        };
        let stream_id = match read_u8(&mut self.reader) {
            Ok(v) => v,
            Err(e) => return Some(Err(e)),
        };
        let size = match read_u32(&mut self.reader) {
            Ok(v) => v,
            Err(e) => return Some(Err(e)),
        };
        let mut payload = vec![0u8; size as usize];
        if let Err(e) = self.reader.read_exact(&mut payload) {
            return Some(Err(e));
        }
        match stream_id {
            STREAM_TOF => Some(decode_tof(ts_us, &payload).map(Record::Tof)),
            STREAM_TWIN => Some(decode_twin(ts_us, &payload).map(Record::Twin)),
            STREAM_ODOM => Some(decode_odom(ts_us, &payload).map(Record::Odom)),
            other => Some(Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("unknown stream_id {}", other),
            ))),
        }
    }
}

// ── Decoders ─────────────────────────────────────────────────────────────────

fn decode_tof(ts_us: u64, payload: &[u8]) -> io::Result<TofRecord> {
    if payload.len() < 12 {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            "ToF payload too short for header",
        ));
    }
    let sender_ts_s = read_f64_le(&payload[0..8]);
    let rows = payload[8] as usize;
    let cols = payload[9] as usize;
    if rows != TOF_ROWS || cols != TOF_COLS {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!("expected 8x8 ToF, got {}x{}", rows, cols),
        ));
    }
    let n = rows * cols;
    let need = 12 + n * 4 + n;
    if payload.len() < need {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!("ToF payload {} < expected {}", payload.len(), need),
        ));
    }
    let mut ranges_m = [[0.0_f32; TOF_COLS]; TOF_ROWS];
    let mut status = [[0u8; TOF_COLS]; TOF_ROWS];
    let dist_off = 12;
    let stat_off = dist_off + n * 4;
    for r in 0..TOF_ROWS {
        for c in 0..TOF_COLS {
            let off = dist_off + (r * TOF_COLS + c) * 4;
            ranges_m[r][c] = read_f32_le(&payload[off..off + 4]);
            status[r][c] = payload[stat_off + r * TOF_COLS + c];
        }
    }
    Ok(TofRecord {
        ts_us,
        sender_ts_s,
        ranges_m,
        status,
    })
}

fn decode_twin(ts_us: u64, payload: &[u8]) -> io::Result<TwinRecord> {
    if payload.len() != TWIN_PACKET_SIZE_V1 && payload.len() != TWIN_PACKET_SIZE_V2 {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "twin payload {} != expected {} or {}",
                payload.len(),
                TWIN_PACKET_SIZE_V1,
                TWIN_PACKET_SIZE_V2
            ),
        ));
    }
    let sender_ts_s = read_f64_le(&payload[0..8]);
    let f = |idx: usize| -> f32 {
        let off = 8 + idx * 4;
        read_f32_le(&payload[off..off + 4])
    };
    let quat_wxyz = [f(0), f(1), f(2), f(3)];
    let mut joints = [0.0_f32; 15];
    for i in 0..15 {
        joints[i] = f(4 + i);
    }
    let mut motor_currents_ma = [0.0_f32; 15];
    for i in 0..15 {
        motor_currents_ma[i] = f(19 + i);
    }
    let odom_x = f(34);
    let odom_y = f(35);
    let ball_xyz = [f(36), f(37), f(38)];
    let odom_z = f(39);
    let odom_yaw = f(40);
    let contact_anchor = if payload.len() >= TWIN_PACKET_SIZE_V2 {
        Some((f(41), f(42)))
    } else {
        None
    };
    Ok(TwinRecord {
        ts_us,
        sender_ts_s,
        quat_wxyz,
        joints,
        motor_currents_ma,
        odom_x,
        odom_y,
        ball_xyz,
        odom_z,
        odom_yaw,
        contact_anchor,
    })
}

fn decode_odom(ts_us: u64, payload: &[u8]) -> io::Result<OdomRecord> {
    const SIZE: usize = 11 * 4 + 1;
    if payload.len() != SIZE {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!("odom payload {} != expected {}", payload.len(), SIZE),
        ));
    }
    let f = |idx: usize| -> f32 { read_f32_le(&payload[idx * 4..idx * 4 + 4]) };
    let flags = payload[SIZE - 1];
    Ok(OdomRecord {
        ts_us,
        odom_x: f(0),
        odom_y: f(1),
        odom_yaw: f(2),
        gravity: [f(3), f(4), f(5)],
        trunk_z: f(6),
        head: [f(7), f(8), f(9), f(10)],
        moving: flags & crate::record::FLAG_MOVING != 0,
        sitting: flags & crate::record::FLAG_SITTING != 0,
        fallen: flags & crate::record::FLAG_FALLEN != 0,
    })
}

// ── Byte helpers ─────────────────────────────────────────────────────────────

fn read_u8<R: Read>(r: &mut R) -> io::Result<u8> {
    let mut b = [0u8; 1];
    r.read_exact(&mut b)?;
    Ok(b[0])
}

fn read_u32<R: Read>(r: &mut R) -> io::Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

fn read_u64<R: Read>(r: &mut R) -> io::Result<u64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}

fn read_f32_le(b: &[u8]) -> f32 {
    f32::from_le_bytes(b.try_into().unwrap())
}

fn read_f64_le(b: &[u8]) -> f64 {
    f64::from_le_bytes(b.try_into().unwrap())
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn write_synthetic(path: &Path) {
        let mut f = std::fs::File::create(path).unwrap();
        // Header.
        f.write_all(MAGIC).unwrap();
        f.write_all(&1u32.to_le_bytes()).unwrap(); // a v1 prototype capture
        f.write_all(&123_456_789u64.to_le_bytes()).unwrap();
        // ToF record at ts_us = 1000.
        let mut tof_payload = Vec::new();
        tof_payload.extend(&1.2345_f64.to_le_bytes());
        tof_payload.push(8); // rows
        tof_payload.push(8); // cols
        tof_payload.extend(&[0u8, 0u8]); // reserved
        for r in 0..8 {
            for c in 0..8 {
                let v = (r * 8 + c) as f32 / 100.0;
                tof_payload.extend(&v.to_le_bytes());
            }
        }
        for r in 0..8 {
            for c in 0..8 {
                tof_payload.push((r * 8 + c) as u8);
            }
        }
        f.write_all(&1000u64.to_le_bytes()).unwrap();
        f.write_all(&[STREAM_TOF]).unwrap();
        f.write_all(&(tof_payload.len() as u32).to_le_bytes())
            .unwrap();
        f.write_all(&tof_payload).unwrap();
        // Legacy 172 B twin record at ts_us = 2000.
        let mut twin = Vec::with_capacity(TWIN_PACKET_SIZE_V1);
        twin.extend(&2.71_f64.to_le_bytes());
        for i in 0..41 {
            twin.extend(&((i as f32) * 0.1).to_le_bytes());
        }
        f.write_all(&2000u64.to_le_bytes()).unwrap();
        f.write_all(&[STREAM_TWIN]).unwrap();
        f.write_all(&(twin.len() as u32).to_le_bytes()).unwrap();
        f.write_all(&twin).unwrap();
        // Current 180 B twin record (appended contact anchor) at 3000.
        let mut twin2 = Vec::with_capacity(TWIN_PACKET_SIZE_V2);
        twin2.extend(&2.72_f64.to_le_bytes());
        for i in 0..43 {
            twin2.extend(&((i as f32) * 0.1).to_le_bytes());
        }
        f.write_all(&3000u64.to_le_bytes()).unwrap();
        f.write_all(&[STREAM_TWIN]).unwrap();
        f.write_all(&(twin2.len() as u32).to_le_bytes()).unwrap();
        f.write_all(&twin2).unwrap();
    }

    #[test]
    fn round_trip_decode() {
        let dir = tempdir().unwrap();
        let path = dir.join("test.mdlg");
        write_synthetic(&path);
        let mut r = SessionReplayer::open(&path).unwrap();
        assert_eq!(r.epoch_unix_ms(), 123_456_789);
        let rec1 = r.next().unwrap().unwrap();
        if let Record::Tof(t) = rec1 {
            assert_eq!(t.ts_us, 1000);
            assert!((t.sender_ts_s - 1.2345).abs() < 1e-9);
            assert_eq!(t.status[0][0], 0);
            assert_eq!(t.status[7][7], 63);
            assert!((t.ranges_m[0][0] - 0.0).abs() < 1e-6);
            assert!((t.ranges_m[7][7] - 0.63).abs() < 1e-5);
        } else {
            panic!("expected Tof");
        }
        let rec2 = r.next().unwrap().unwrap();
        if let Record::Twin(d) = rec2 {
            assert_eq!(d.ts_us, 2000);
            assert!((d.sender_ts_s - 2.71).abs() < 1e-9);
            // Quat is the first 4 floats.
            assert!((d.quat_wxyz[0] - 0.0).abs() < 1e-6);
            assert!((d.quat_wxyz[3] - 0.3).abs() < 1e-5);
            // odom_yaw is the last float: index 40 → 4.0.
            assert!((d.odom_yaw - 4.0).abs() < 1e-5);
            // Legacy packet carries no contact anchor.
            assert!(d.contact_anchor.is_none());
        } else {
            panic!("expected Twin");
        }
        let rec3 = r.next().unwrap().unwrap();
        if let Record::Twin(d) = rec3 {
            assert_eq!(d.ts_us, 3000);
            assert!((d.odom_yaw - 4.0).abs() < 1e-5);
            let (ax, ay) = d.contact_anchor.expect("v2 packet carries anchor");
            assert!((ax - 4.1).abs() < 1e-5);
            assert!((ay - 4.2).abs() < 1e-5);
        } else {
            panic!("expected Twin (v2)");
        }
        assert!(r.next().is_none());
    }

    fn tempdir() -> io::Result<std::path::PathBuf> {
        let p = std::env::temp_dir().join(format!("mdlg_test_{}", std::process::id(),));
        std::fs::create_dir_all(&p)?;
        Ok(p)
    }
}
