//! The camera LED, which `architecture.md` §7 asks for: visible whenever the picture is leaving
//! the robot. Which LED it is, and that this daemon owns it, is §3.2.
//!
//! "Leaving" is either way out: a WebRTC peer being encoded for, or a `media.stream` running. Not
//! a `media.frame` snapshot — that one goes to a process on the robot itself — and not the
//! detector, which looks at frames without sending them anywhere.
//!
//! Polled rather than signalled. The peer count changes on GStreamer threads and the stream on a
//! request, and a quarter of a second is well inside what a person notices of an LED coming on,
//! while a callback from each of those places would be three places to get wrong.

use std::sync::atomic::Ordering;
use std::time::Duration;

use duck_ipc_proto::led::{Led, Light, name};

/// How often the LED catches up.
const PERIOD: Duration = Duration::from_millis(250);

/// Whether the picture is leaving the robot.
pub fn recording(peers: u32, streaming: bool) -> bool {
    peers > 0 || streaming
}

/// Keep the camera LED in step with `peers` and `streaming` for as long as the daemon runs.
/// Returns at once on a board without one.
pub async fn run(
    peers: std::sync::Arc<std::sync::atomic::AtomicU32>,
    streaming: impl Fn() -> bool,
) {
    let Some(mut led) = Led::open(name::CAMERA) else {
        tracing::debug!("no camera LED on this board");
        return;
    };
    let mut ticker = tokio::time::interval(PERIOD);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        ticker.tick().await;
        let on = recording(peers.load(Ordering::Relaxed), streaming());
        if let Err(e) = led.set(Light::from(on)) {
            tracing::warn!(error = %e, "cannot switch the camera LED");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn either_way_out_lights_it() {
        assert!(!recording(0, false));
        assert!(recording(1, false));
        assert!(recording(0, true));
    }
}
