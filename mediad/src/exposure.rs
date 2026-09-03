//! Software auto-exposure.
//!
//! **Because the 3A engine sets the exposure once and then stops.** Rockchip's `rkaiq_3A_server`
//! does white balance, the colour matrix, gamma and noise reduction, and `scripts/setup-rkaiq.sh`
//! installs it with AE enabled. Its AE does write the sensor — measured on a robot, on a boot where
//! the engine started before `mediad` and caught the stream-start event, `mediad` wrote
//! `exposure=600 analogue_gain=1024` and the sensor was found at `1589 / 1536`, which is the
//! engine's answer for the room and not ours. What it does not do is *keep* doing it: a hand-written
//! `exposure=300 analogue_gain=256` on that same healthy boot — a much darker picture — held for 25
//! seconds with no correction. One convergence at stream start is not auto-exposure. A robot that
//! walks from a window into a corridor keeps the window's exposure.
//!
//! And when the engine misses the stream-start event — it waits for one and does not notice one that
//! already fired, which is the other half of this branch — even that one shot does not happen and
//! the picture stays at whatever `mediad` pinned. That is the "3A stopped working, a reboot sometimes
//! fixes it" shape. Fixing the ordering brings the one shot back; it does not make it a loop, which
//! is why both halves are here. With this module, exposure stops depending on the order two units
//! started in at all.
//!
//! So this is the loop, ported from the prototype (`microduck_runtime`'s `camera.rs`), where it ran
//! for months: sample luma twice a second, compare it against a setpoint, and steer the sensor with
//! a damped multiplicative step.
//!
//! Two things differ from the prototype, both because `mediad` is a better place to do this:
//!
//! * **Luma comes off the tee's raw branch**, which already exists for the duck detector and the
//!   `get_frame` surface. The prototype decoded one of its own JPEGs; here the frame is UYVY and
//!   the luma is every other byte, so a mean costs a subsampled walk and no decode.
//! * **Nothing opens the camera a second time to measure.** The prototype's first version sampled
//!   the ISP self path with a parallel `v4l2-ctl`, which contended with the capture at the driver
//!   level and killed the pipeline intermittently. Reading the frames we already have cannot.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::pipeline::{CAPTURE_FORMAT, Frame, Frames};

/// How often to look. Twice a second: fast enough to follow a robot walking from a window into a
/// corridor, slow enough that the damped step never rings.
const INTERVAL: Duration = Duration::from_millis(500);

/// How many quiet ticks before the current values are written again anyway. 20 is ten seconds.
///
/// **Because this loop is not the only thing that writes the sensor.** `v4l2src` applies
/// `--exposure`/`--analogue-gain` through `extra-controls` whenever the device is opened, including
/// a re-open nobody here initiated, and a fresh IQ file or a re-enabled engine could put rkaiq back
/// in the same business. Skipping a write because we already wrote that value assumes what we wrote
/// is still there, which is exactly the assumption that leaves a camera dark and a log saying it
/// converged. So the assumption is re-checked on a slow heartbeat — at 43 ms of CPU a call, ten
/// seconds apart is under half a percent of a core, against the tenth of a core that writing every
/// tick cost.
const REASSERT_TICKS: u32 = 20;

/// How often to say out loud what the loop is seeing. 20 is ten seconds.
///
/// **At info, not debug, because the question "is auto-exposure alive?" should not need a drop-in.**
/// A loop that has settled writes nothing, and from the sensor that is indistinguishable from a loop
/// metering a frozen frame or one that never started — the difference is the measured luma, which
/// only this line carries. Ten seconds of it is six lines a minute against a journal that already
/// carries a capture-rate line.
const REPORT_TICKS: u32 = 20;

/// Mean-luma setpoint, 8-bit and *after* the ISP's gamma curve.
const TARGET_Y: f64 = 90.0;

/// Relative error tolerated without a step. Without a deadband the exposure jitters continuously on
/// sensor noise alone, which is visible as a picture that breathes.
const DEADBAND: f64 = 0.12;

/// The IMX219 in the mode [`crate::pipeline`] pins (1920x1080, one line ≈ 19.05 µs).
///
/// **The two shutter caps are the whole reason this steers three controls rather than one.**
/// Brightness is spent in noise order: shutter up to the soft cap first (cheapest and cleanest, and
/// 11 ms is short enough that a walking robot's picture is not smeared), then sensor analogue gain
/// (clean amplification), then shutter up to the hard cap, and ISP digital gain — the noisiest —
/// only when there is nothing else left.
///
/// `HARD_LINES` is a real ceiling, not a preference: the driver responds to an exposure longer than
/// the frame length by *stretching the frame time* to fit it rather than clamping, so asking for
/// 3500 lines silently halves the frame rate. The mode is 1766 lines total; measured on the
/// prototype, 1762 lines held 29.96 fps and 3500 collapsed to 15.1.
const SOFT_LINES: f64 = 600.0; // ≈ 11.4 ms
const HARD_LINES: f64 = 1200.0; // ≈ 22.9 ms
/// Sensor analogue gain ceiling, in multiples. The register is this × 256.
const MAX_ANALOGUE: f64 = 11.0;
/// ISP digital gain ceiling, in multiples. Well under what the sensor will accept, because past
/// this the picture is brighter and no more legible.
const MAX_DIGITAL: f64 = 16.0;

/// What the loop writes to the sensor, in the sensor's own units.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Controls {
    /// Exposure in sensor lines.
    pub exposure: u32,
    /// Analogue gain, where 256 is 1x.
    pub analogue_gain: u32,
    /// ISP digital gain, where 256 is 1x.
    pub gain: u32,
}

/// The loop's state: one brightness budget, split across three controls.
#[derive(Debug, Clone, Copy)]
pub struct Ae {
    exposure: f64,
    analogue: f64,
    digital: f64,
    /// What was last handed out, so the same values are never written twice.
    written: Option<Controls>,
}

impl Ae {
    /// Starting from what `mediad` already wrote to the sensor, so the first step is relative to the
    /// picture on screen rather than to a number this module invented.
    pub fn starting_at(exposure_lines: u32, analogue_gain_reg: u32) -> Self {
        Self {
            exposure: (exposure_lines as f64).clamp(4.0, HARD_LINES),
            analogue: (analogue_gain_reg as f64 / 256.0).clamp(1.0, MAX_ANALOGUE),
            digital: 1.0,
            written: None,
        }
    }

    /// One step towards the setpoint. `None` inside the deadband, and `None` when the step lands on
    /// the values already written.
    ///
    /// **That second case is not an optimisation.** A room darker than the sensor can reach — the
    /// shutter at 1200 lines, analogue gain at 11x, digital gain at its ceiling — leaves the ratio
    /// permanently outside the deadband, because the setpoint is unreachable rather than merely far
    /// away. Without this the loop writes the same three numbers twice a second for as long as the
    /// robot is in that room, and each write is a `v4l2-ctl` process: measured on the board at 43 ms
    /// of CPU a call, which at the throttled 408 MHz is most of a tenth of a core spent achieving
    /// nothing.
    ///
    /// The step is multiplicative on the *product* of the three controls, because that product is
    /// what luma is proportional to; the split back into three is where the noise ordering lives.
    /// `ratio^0.6` rather than `ratio` damps it: the controls are linear in light and the measured
    /// luma is not, so a full correction against a gamma-compressed measurement overshoots and the
    /// loop hunts.
    pub fn step(&mut self, mean_luma: f64) -> Option<Controls> {
        let ratio = (TARGET_Y / mean_luma.max(1.0)).clamp(0.25, 4.0);
        if (1.0 - ratio).abs() < DEADBAND {
            return None;
        }

        let budget = (self.exposure * self.analogue * self.digital * ratio.powf(0.6))
            .clamp(4.0, HARD_LINES * MAX_ANALOGUE * MAX_DIGITAL);

        self.exposure = budget.min(SOFT_LINES);
        self.analogue = (budget / self.exposure).clamp(1.0, MAX_ANALOGUE);
        self.exposure = (budget / self.analogue).clamp(self.exposure, HARD_LINES);
        self.digital = (budget / (self.exposure * self.analogue)).clamp(1.0, MAX_DIGITAL);

        let next = self.controls();
        if self.written == Some(next) {
            return None;
        }
        self.written = Some(next);
        Some(next)
    }

    /// The values as they stand, for the periodic re-assert — and they count as written, so the
    /// heartbeat restarts rather than firing every tick from here on.
    pub fn current(&mut self) -> Controls {
        let controls = self.controls();
        self.written = Some(controls);
        controls
    }

    fn controls(&self) -> Controls {
        Controls {
            exposure: self.exposure as u32,
            analogue_gain: (self.analogue * 256.0) as u32,
            gain: (self.digital * 256.0) as u32,
        }
    }
}

/// Mean luma of one captured frame, 0–255.
///
/// UYVY puts luma in every odd byte, so this is a subsampled walk over the buffer and no decode:
/// every eighth pixel, which is 11k samples of a 1280x720 frame and enough for a mean to three
/// figures. `None` for any other format rather than a wrong answer — a mean computed over the wrong
/// byte layout would still look like a plausible brightness, and the loop would chase it.
pub fn mean_luma(frame: &Frame) -> Option<f64> {
    if frame.format != CAPTURE_FORMAT {
        return None;
    }
    let mut sum = 0u64;
    let mut count = 0u64;
    let mut index = 1; // U Y V Y: the first luma byte
    while index < frame.data.len() {
        sum += frame.data[index] as u64;
        count += 1;
        index += 16; // every 8th pixel
    }
    (count > 0).then(|| sum as f64 / count as f64)
}

/// Set when the daemon is going away, so the thread does not outlive the pipeline it steers.
#[derive(Clone, Default)]
pub struct Stop(Arc<AtomicBool>);

impl Stop {
    fn stopped(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Start the loop. Returns the handle that stops it.
///
/// `device` is the *capture* node rather than the sensor subdev, because rkisp proxies the sensor's
/// controls through it — which is also how the starting exposure gets there, as `v4l2src`'s
/// `extra-controls`. One node to open, and it is the one we already know we can.
pub fn spawn(device: String, frames: Frames, exposure_lines: u32, analogue_gain_reg: u32) -> Stop {
    let stop = Stop::default();
    let mine = stop.clone();

    tracing::info!(
        %device,
        target_luma = TARGET_Y,
        shutter_lines = format!("{SOFT_LINES:.0}/{HARD_LINES:.0}"),
        analogue_max = MAX_ANALOGUE,
        "software auto-exposure"
    );

    std::thread::Builder::new()
        .name("auto-exposure".into())
        .spawn(move || {
            let mut ae = Ae::starting_at(exposure_lines, analogue_gain_reg);
            let mut proven = false;
            let mut waiting_logged = false;
            // Ticks since the last write, for the re-assert heartbeat.
            let mut quiet = 0u32;
            let mut format_logged = false;
            let mut since_report = 0u32;

            while !mine.stopped() {
                std::thread::sleep(INTERVAL);
                if mine.stopped() {
                    break;
                }

                // Asked for, so what gets metered is a frame captured now rather than one left
                // over from the previous tick — [`Frames`] has why the branch works this way. The
                // wait is bounded and lands on the same "no frame" path a camera that has not
                // started yet does.
                let metered = frames.inspect_next(mean_luma);
                if metered == Some(None) && !format_logged {
                    format_logged = true;
                    // The one way a frame can arrive and be unreadable: it is not what the capture
                    // caps say. Loud, because the loop then meters nothing for ever and the only
                    // other symptom is a camera that never changes exposure.
                    tracing::error!(
                        expected = CAPTURE_FORMAT,
                        "the raw frames are not in the format this can meter; auto-exposure is inert"
                    );
                }
                let Some(mean) = metered.flatten() else {
                    if !waiting_logged {
                        waiting_logged = true;
                        tracing::debug!("no frame to meter yet");
                    }
                    continue;
                };
                tracing::debug!(mean_luma = format!("{mean:.1}"), target = TARGET_Y, "metered");
                since_report += 1;
                if since_report >= REPORT_TICKS {
                    since_report = 0;
                    let at = ae.controls();
                    tracing::info!(
                        mean_luma = format!("{mean:.1}"),
                        target = TARGET_Y,
                        exposure = at.exposure,
                        analogue_gain = at.analogue_gain,
                        gain = at.gain,
                        // Named, because "the numbers are not moving" has two very different causes
                        // and this is the one that tells them apart.
                        at_ceiling = at.exposure as f64 >= HARD_LINES
                            && at.analogue_gain as f64 >= MAX_ANALOGUE * 256.0,
                        "metering"
                    );
                }

                let controls = match ae.step(mean) {
                    Some(controls) => controls,
                    // Nothing to change — but say it to the sensor again every so often, in case
                    // something else has been talking to it. See `REASSERT_TICKS`.
                    None if quiet >= REASSERT_TICKS => ae.current(),
                    None => {
                        quiet += 1;
                        continue;
                    }
                };
                quiet = 0;

                match write(&device, controls) {
                    Err(e) if proven => {
                        // At debug, not an error twice a second for as long as the daemon runs: the
                        // first one already said what is wrong.
                        tracing::debug!(error = %e, "exposure write failed");
                    }
                    Err(e) => {
                        proven = true;
                        tracing::error!(
                            %device, error = %e,
                            "auto-exposure cannot write the sensor; the picture stays at one \
                             brightness. `v4l2-ctl -d {device} --list-ctrls` says which controls \
                             this node carries."
                        );
                    }
                    Ok(()) => {
                        if worth_reading_back(proven, controls.exposure, exposure_lines) {
                            proven = true;
                            match read_exposure(&device) {
                                Some(landed) if landed == controls.exposure => tracing::info!(
                                    exposure = landed,
                                    "auto-exposure is driving the sensor"
                                ),
                                landed => tracing::error!(
                                    %device,
                                    asked = controls.exposure,
                                    landed = landed.unwrap_or_default(),
                                    "the exposure write reported success and the sensor did not \
                                     take it — the picture will stay at one brightness"
                                ),
                            }
                        }
                        tracing::debug!(
                            mean_luma = format!("{mean:.0}"),
                            exposure = controls.exposure,
                            analogue_gain = controls.analogue_gain,
                            gain = controls.gain,
                            "exposure step"
                        );
                    }
                }
            }
            tracing::debug!("auto-exposure stopped");
        })
        .map(|_| ())
        .unwrap_or_else(|e| {
            tracing::error!(error = %e, "no auto-exposure thread; the picture stays at one exposure")
        });

    stop
}

/// `v4l2-ctl` rather than the ioctl, for the prototype's reason: the struct layout of
/// `VIDIOC_S_EXT_CTRLS` is three nested types we would have to pin by hand, and getting one offset
/// wrong is a write that succeeds and changes nothing. The tool is in `v4l-utils`, which the
/// capture path already needs for `media-ctl`.
///
/// **Two calls, not one.** `exposure` and `analogue_gain` belong to the sensor and `gain` to the
/// ISP, and `v4l2-ctl` fails a whole `--set-ctrl` if any name in it is unknown on the node. One
/// call would mean a board that spells digital gain differently loses its shutter as well, which is
/// the entire picture rather than the last stop of brightness.
fn write(device: &str, controls: Controls) -> std::io::Result<()> {
    let Controls {
        exposure,
        analogue_gain,
        gain,
    } = controls;
    set(
        device,
        &format!("exposure={exposure},analogue_gain={analogue_gain}"),
    )?;
    // Only when it is doing something: at 1x this is a no-op write, and skipping it means a node
    // with no digital gain never reports an error at all.
    if gain > 256 {
        set(device, &format!("gain={gain}"))?;
    }
    Ok(())
}

fn set(device: &str, controls: &str) -> std::io::Result<()> {
    let output = std::process::Command::new("v4l2-ctl")
        .args(["-d", device, &format!("--set-ctrl={controls}")])
        .output()?;
    if output.status.success() {
        return Ok(());
    }
    Err(std::io::Error::other(format!(
        "{controls}: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    )))
}

/// Whether a successful write is worth reading back from the sensor.
///
/// **Once, and not for a step that asks for the value already there.** Every way the write path can
/// fail is silent, and all of them leave the sensor at the value `mediad` pinned at startup — so a
/// read-back that finds the pin proves nothing if the pin is also what we asked for. The check would
/// pass on the strength of somebody else's write. That is not hypothetical: the first step on a
/// robot landed on exactly the 600 lines `mediad` had pinned, and reported success.
fn worth_reading_back(proven: bool, asked: u32, pinned: u32) -> bool {
    !proven && asked != pinned
}

/// What the sensor says its exposure is now, for the one check above.
fn read_exposure(device: &str) -> Option<u32> {
    let output = std::process::Command::new("v4l2-ctl")
        .args(["-d", device, "--get-ctrl=exposure"])
        .output()
        .ok()?;
    // "exposure: 600"
    String::from_utf8_lossy(&output.stdout)
        .split_once(':')
        .and_then(|(_, value)| value.trim().parse().ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(luma: u8) -> Frame {
        // UYVY, so half the bytes are chroma; they are set to something that would be a *different*
        // mean if the walk read them by mistake.
        let mut data = vec![0u8; 1280 * 720 * 2];
        for (index, byte) in data.iter_mut().enumerate() {
            *byte = if index % 2 == 1 { luma } else { 200 };
        }
        Frame {
            width: 1280,
            height: 720,
            format: CAPTURE_FORMAT,
            data,
        }
    }

    #[test]
    fn luma_comes_from_the_luma_bytes() {
        assert_eq!(mean_luma(&frame(64)), Some(64.0));
    }

    #[test]
    fn a_format_we_cannot_read_is_not_guessed_at() {
        let mut other = frame(64);
        other.format = "NV12";
        assert_eq!(mean_luma(&other), None);
    }

    #[test]
    fn a_picture_at_the_setpoint_is_left_alone() {
        let mut ae = Ae::starting_at(600, 1024);
        assert_eq!(ae.step(TARGET_Y), None);
        // And within the deadband, either side.
        assert_eq!(ae.step(TARGET_Y * 1.1), None);
        assert_eq!(ae.step(TARGET_Y / 1.1), None);
    }

    #[test]
    fn a_dark_picture_gets_more_light_and_a_bright_one_less() {
        let brightness = |c: Controls| c.exposure as f64 * c.analogue_gain as f64 * c.gain as f64;

        let mut ae = Ae::starting_at(600, 1024);
        let before = ae.controls();
        let darker = ae.step(20.0).expect("a step out of the deadband");
        assert!(brightness(darker) > brightness(before));

        let mut ae = Ae::starting_at(600, 1024);
        let before = ae.controls();
        let brighter = ae.step(220.0).expect("a step out of the deadband");
        assert!(brightness(brighter) < brightness(before));
    }

    #[test]
    fn light_is_spent_on_the_shutter_before_the_gains() {
        // From the dimmest state there is, the first thing that grows is the shutter, and neither
        // gain moves off 1x until the soft cap is reached.
        let mut ae = Ae {
            exposure: 4.0,
            analogue: 1.0,
            digital: 1.0,
            written: None,
        };
        let step = ae.step(2.0).expect("a step");
        assert!(step.exposure > 4);
        assert!(step.exposure as f64 <= SOFT_LINES);
        assert_eq!((step.analogue_gain, step.gain), (256, 256));
    }

    #[test]
    fn a_room_darker_than_the_sensor_can_reach_stops_being_written_to() {
        // The bug this catches, seen on a robot in a dark room: pinned at every ceiling, the ratio
        // stays outside the deadband for ever because the setpoint is unreachable — so the loop kept
        // writing the same three numbers twice a second, and each write is a process.
        let mut ae = Ae::starting_at(600, 1024);
        let mut steps = 0;
        for _ in 0..200 {
            if ae.step(1.0).is_some() {
                steps += 1;
            }
        }
        assert!(
            steps < 10,
            "still writing after settling at the ceiling: {steps} writes"
        );
        // And it is at the ceiling, not merely quiet.
        let pinned = ae.controls();
        assert_eq!(pinned.exposure as f64, HARD_LINES);
        assert_eq!(pinned.analogue_gain as f64, MAX_ANALOGUE * 256.0);

        // Light returns: the next step must write again rather than stay stuck on "unchanged".
        assert!(
            ae.step(TARGET_Y * 4.0).is_some(),
            "a bright scene must move it"
        );
    }

    #[test]
    fn the_re_assert_writes_the_same_values_and_restarts_the_heartbeat() {
        let mut ae = Ae::starting_at(600, 1024);
        for _ in 0..50 {
            ae.step(1.0);
        }
        let quiet = ae.controls();
        assert_eq!(ae.step(1.0), None, "settled");
        // What the heartbeat sends: the values as they stand, not a recomputation.
        assert_eq!(ae.current(), quiet);
        // And it counts as written, so the next quiet tick is quiet again rather than a second
        // write.
        assert_eq!(ae.step(1.0), None);
    }

    #[test]
    fn the_shutter_never_asks_for_a_longer_frame_than_the_sensor_has() {
        // Pitch dark, for as long as it takes: the shutter must stop at the hard cap, or the driver
        // stretches the frame time and the stream halves its rate.
        let mut ae = Ae::starting_at(600, 1024);
        for _ in 0..200 {
            ae.step(1.0);
        }
        let pinned = ae.controls();
        assert!(pinned.exposure as f64 <= HARD_LINES, "{pinned:?}");
        assert!(
            pinned.analogue_gain as f64 <= MAX_ANALOGUE * 256.0,
            "{pinned:?}"
        );
        assert!(pinned.gain as f64 <= MAX_DIGITAL * 256.0, "{pinned:?}");
    }

    #[test]
    fn the_sensor_is_only_believed_about_a_value_it_was_not_already_at() {
        // The pin is 600. A step that asks for 600 tells us nothing about whether the write landed.
        assert!(!worth_reading_back(false, 600, 600));
        assert!(worth_reading_back(false, 601, 600));
        // And once, not every step.
        assert!(!worth_reading_back(true, 900, 600));
    }

    #[test]
    fn the_loop_converges_rather_than_ringing() {
        // A crude model of the sensor: luma is proportional to the product of the controls, through
        // the gamma curve the damping term exists to account for. The picture starts eight times
        // too dark; what matters is that it settles and stays settled.
        let reference = 600.0 * 4.0; // the starting product, which we call 90/8 of luma
        let luma = |c: Controls| {
            let product =
                c.exposure as f64 * (c.analogue_gain as f64 / 256.0) * (c.gain as f64 / 256.0);
            (TARGET_Y / 8.0) * (product / reference).powf(1.0 / 0.45)
        };

        let mut ae = Ae::starting_at(600, 1024);
        let mut seen = ae.controls();
        let mut steps = 0;
        for _ in 0..60 {
            match ae.step(luma(seen)) {
                Some(next) => {
                    seen = next;
                    steps += 1;
                }
                None => break,
            }
        }
        let settled = luma(seen);
        assert!(
            (settled - TARGET_Y).abs() / TARGET_Y < DEADBAND,
            "settled at {settled:.0} after {steps} steps"
        );
        assert!(steps < 30, "took {steps} steps to settle");
    }
}
