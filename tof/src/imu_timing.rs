//! Host checks for stale/overlapping register reads. A rejected read must not advance
//! fusion time; physical sensor delay and GPIO capture latency are not calibrated here.

use duck_ipc_proto::HeadImuTiming;

#[derive(Debug, Clone, Copy)]
pub struct DataReady {
    pub t_ns: u64,
    pub seq: u32,
}

pub struct CaptureClock {
    period_ns: u64,
    last_accepted_ns: Option<u64>,
}

impl CaptureClock {
    pub fn new(hz: u8) -> Self {
        Self {
            period_ns: 1_000_000_000 / u64::from(hz.max(1)),
            last_accepted_ns: None,
        }
    }

    pub fn is_fresh(&self, event: DataReady, read_started_ns: u64) -> bool {
        event.t_ns != 0
            && event.t_ns <= read_started_ns
            && read_started_ns - event.t_ns < self.period_ns
            && self.last_accepted_ns.is_none_or(|last| event.t_ns > last)
    }

    /// Reject stale events and observed edge overlap before advancing fusion time.
    ///
    /// Do not advance `last_accepted_ns` on rejection: otherwise a skipped I²C read would
    /// consume the integration interval without contributing an actual measurement.
    pub fn finish(
        &mut self,
        event: DataReady,
        read_started_ns: u64,
        read_finished_ns: u64,
        next_edge: Option<DataReady>,
    ) -> Option<(HeadImuTiming, f32)> {
        if !self.is_fresh(event, read_started_ns)
            || read_finished_ns < read_started_ns
            || read_finished_ns - event.t_ns >= self.period_ns
            || next_edge.is_some_and(|next| next.t_ns <= read_finished_ns)
        {
            return None;
        }
        let interval_ns = self
            .last_accepted_ns
            .map_or(self.period_ns, |last| event.t_ns - last);
        self.last_accepted_ns = Some(event.t_ns);
        Some((
            HeadImuTiming {
                accel_data_ready_ns: event.t_ns,
                accel_event_seq: event.seq,
                read_started_ns,
                read_finished_ns,
            },
            (interval_ns as f64 / 1e9).clamp(1e-4, 0.2) as f32,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(ms: u64, seq: u32) -> DataReady {
        DataReady {
            t_ns: ms * 1_000_000,
            seq,
        }
    }

    #[test]
    fn user_space_wakeup_and_temperature_work_do_not_retimestamp_the_sample() {
        let mut clock = CaptureClock::new(100);
        assert_eq!(clock.period_ns, 10_000_000);
        let (timing, dt) = clock
            .finish(edge(100, 1), 103_000_000, 104_000_000, None)
            .unwrap();
        assert_eq!(timing.accel_data_ready_ns, 100_000_000);
        assert_eq!(timing.read_finished_ns, 104_000_000);
        assert!((dt - 0.01).abs() < 1e-6);
    }

    #[test]
    fn an_old_queued_event_must_not_date_newer_register_contents() {
        let mut clock = CaptureClock::new(100);
        assert!(!clock.is_fresh(edge(100, 1), 110_000_000));
        assert!(
            clock
                .finish(edge(100, 1), 109_000_000, 110_000_000, None)
                .is_none()
        );
    }

    #[test]
    fn a_new_sample_during_the_read_is_ambiguous_and_does_not_consume_fusion_dt() {
        let mut clock = CaptureClock::new(100);
        clock
            .finish(edge(100, 1), 101_000_000, 102_000_000, None)
            .unwrap();
        assert!(
            clock
                .finish(edge(110, 2), 111_000_000, 115_000_000, Some(edge(114, 3)))
                .is_none()
        );
        let (_, dt) = clock
            .finish(edge(120, 4), 121_000_000, 122_000_000, None)
            .unwrap();
        assert!((dt - 0.02).abs() < 1e-6);
    }

    #[test]
    fn a_future_edge_can_be_saved_for_the_next_read_without_invalidating_this_one() {
        let mut clock = CaptureClock::new(100);
        assert!(
            clock
                .finish(edge(100, 1), 101_000_000, 102_000_000, Some(edge(110, 2)))
                .is_some()
        );
    }

    #[test]
    fn duplicate_or_wrong_clock_events_are_not_published() {
        let mut clock = CaptureClock::new(100);
        clock
            .finish(edge(100, 1), 101_000_000, 102_000_000, None)
            .unwrap();
        assert!(
            clock
                .finish(edge(100, 1), 103_000_000, 104_000_000, None)
                .is_none()
        );
        assert!(
            clock
                .finish(edge(200, 2), 110_000_000, 111_000_000, None)
                .is_none()
        );
        assert!(
            clock
                .finish(edge(0, 0), 1_000_000, 2_000_000, None)
                .is_none()
        );
    }
}
