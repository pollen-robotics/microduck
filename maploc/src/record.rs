//! Writer for `.mdlg` session logs — the ground-truth recorder robotd uses.
//!
//! Version 2 of the format documented in [`crate::replay`]: the same
//! header and record framing, but the robot-state stream carries what
//! robotd's mapping worker actually consumes (odometry, projected gravity,
//! trunk height, head joints, the moving/sitting verdicts) instead of the
//! prototype's digital-twin packet. A recorded session replays through the
//! `evaluate` example byte-for-byte the way the live worker saw it —
//! which is the whole point: a field failure becomes a bench case.
//!
//! Stream ids: 0 = ToF (same payload as v1), 2 = odom (v2 only). The v1
//! twin stream (1) is never written by this recorder.
//!
//! Odom payload (45 B, little-endian):
//!
//! ```text
//! f32 odom_x, odom_y, odom_yaw
//! f32 gravity_x, gravity_y, gravity_z   (projected gravity, body frame)
//! f32 trunk_z                            (metres above the floor)
//! f32 head[4]                            (neck_pitch, head_pitch, head_yaw, head_roll)
//! u8  flags                              bit 0 = moving, bit 1 = sitting, bit 2 = fallen
//! ```

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

pub const MAGIC: &[u8; 4] = b"MDLG";
pub const VERSION: u32 = 2;

pub const STREAM_TOF: u8 = 0;
pub const STREAM_ODOM: u8 = 2;

pub const FLAG_MOVING: u8 = 1;
pub const FLAG_SITTING: u8 = 2;
pub const FLAG_FALLEN: u8 = 4;

/// Appends records to one `.mdlg` file. Buffered; call [`Self::flush`] on
/// whatever cadence losing the tail to a crash stops being acceptable.
pub struct SessionRecorder {
    w: BufWriter<File>,
    started: Instant,
}

impl SessionRecorder {
    pub fn create(path: impl AsRef<Path>) -> io::Result<Self> {
        let file = File::create(path)?;
        let mut w = BufWriter::new(file);
        let epoch_unix_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        w.write_all(MAGIC)?;
        w.write_all(&VERSION.to_le_bytes())?;
        w.write_all(&epoch_unix_ms.to_le_bytes())?;
        Ok(Self {
            w,
            started: Instant::now(),
        })
    }

    fn header(&mut self, stream_id: u8, size: u32) -> io::Result<()> {
        let ts_us = self.started.elapsed().as_micros() as u64;
        self.w.write_all(&ts_us.to_le_bytes())?;
        self.w.write_all(&[stream_id])?;
        self.w.write_all(&size.to_le_bytes())
    }

    /// One depth frame, raw: every zone's range in metres alongside its
    /// status byte — filtering is the reader's decision, so a recording
    /// outlives today's idea of which statuses are trustworthy.
    pub fn tof(
        &mut self,
        sender_ts_s: f64,
        rows: u8,
        cols: u8,
        distance_mm: &[i16],
        status: &[u8],
    ) -> io::Result<()> {
        let n = rows as usize * cols as usize;
        if distance_mm.len() != n || status.len() != n {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "ToF zone count does not match rows × cols",
            ));
        }
        let size = 12 + n * 4 + n;
        self.header(STREAM_TOF, size as u32)?;
        self.w.write_all(&sender_ts_s.to_le_bytes())?;
        self.w.write_all(&[rows, cols, 0, 0])?;
        for &mm in distance_mm {
            self.w.write_all(&(f32::from(mm) / 1000.0).to_le_bytes())?;
        }
        self.w.write_all(status)
    }

    /// One control-loop tick's worth of robot state.
    #[allow(clippy::too_many_arguments)] // it is the record layout, spelled out
    pub fn odom(
        &mut self,
        odom: (f32, f32, f32),
        gravity: [f32; 3],
        trunk_z: f32,
        head: [f32; 4],
        moving: bool,
        sitting: bool,
        fallen: bool,
    ) -> io::Result<()> {
        self.header(STREAM_ODOM, 11 * 4 + 1)?;
        for v in [
            odom.0, odom.1, odom.2, gravity[0], gravity[1], gravity[2], trunk_z, head[0], head[1],
            head[2], head[3],
        ] {
            self.w.write_all(&v.to_le_bytes())?;
        }
        let mut flags = 0u8;
        if moving {
            flags |= FLAG_MOVING;
        }
        if sitting {
            flags |= FLAG_SITTING;
        }
        if fallen {
            flags |= FLAG_FALLEN;
        }
        self.w.write_all(&[flags])
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.w.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::replay::{Record, SessionReplayer};

    #[test]
    fn a_recording_round_trips_through_the_reader() {
        let path = std::env::temp_dir().join(format!("mdlg_record_{}.mdlg", std::process::id()));
        {
            let mut rec = SessionRecorder::create(&path).expect("create");
            rec.odom(
                (1.0, -2.0, 0.5),
                [0.0, 0.1, -9.8],
                0.21,
                [0.1, 0.2, 0.3, 0.4],
                false,
                true,
                false,
            )
            .expect("odom");
            let mm: Vec<i16> = (0..64).collect();
            let status = [5u8; 64];
            rec.tof(1.5, 8, 8, &mm, &status).expect("tof");
            rec.flush().expect("flush");
        }
        let mut r = SessionReplayer::open(&path).expect("open");
        match r.next().expect("first").expect("ok") {
            Record::Odom(o) => {
                assert_eq!((o.odom_x, o.odom_y, o.odom_yaw), (1.0, -2.0, 0.5));
                assert_eq!(o.gravity, [0.0, 0.1, -9.8]);
                assert_eq!(o.trunk_z, 0.21);
                assert_eq!(o.head, [0.1, 0.2, 0.3, 0.4]);
                assert!(!o.moving);
                assert!(o.sitting);
                assert!(!o.fallen);
            }
            other => panic!("expected Odom, got {other:?}"),
        }
        match r.next().expect("second").expect("ok") {
            Record::Tof(t) => {
                assert!((t.ranges_m[0][1] - 0.001).abs() < 1e-6);
                assert!((t.ranges_m[7][7] - 0.063).abs() < 1e-6);
                assert_eq!(t.status[3][3], 5);
            }
            other => panic!("expected Tof, got {other:?}"),
        }
        assert!(r.next().is_none());
        std::fs::remove_file(&path).ok();
    }
}
