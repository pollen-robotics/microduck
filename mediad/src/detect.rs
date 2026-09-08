//! Looking for other ducks in the frames already on the tee.
//!
//! `architecture.md` §2 wants perception next to the sensor — deriving features rather than shipping
//! pixels to `robotd` — and §5.3 wants a frame on demand. The raw branch of the tee has existed for
//! both since the pipeline was written; this is the first thing to read it.
//!
//! **A thread, not a task.** Inference is 60 ms of blocking work per frame and the tokio runtime
//! here serves WebRTC signalling; a detector that occupied one of its workers for a tenth of every
//! second would make session setup stutter for no reason anybody could find.
//!
//! **Paced, and the pace is a thermal number.** Flat out on a Radxa Zero 3 the detector reaches
//! 95 °C and the CPU throttles to 408 MHz — a robot that walks badly to see well. Two looks a
//! second is plenty for "is there a duck over there" and costs about a tenth of one core.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use duck_detect::{Detection, Turn, decode, letterbox_from_uyvy};
use tokio::sync::broadcast;

use crate::pipeline::Frames;

/// What one look found, and what it cost.
#[derive(Debug, Clone, PartialEq)]
pub struct Sighting {
    /// The frame the boxes are in, so a consumer can scale them to whatever it is drawing on.
    pub width: u32,
    pub height: u32,
    pub found: Vec<Detection>,
    /// Inference plus decode, in milliseconds — on the frame, not averaged.
    pub took_ms: f64,
}

/// The published stream of sightings, and the counters `robot.health` style reporting wants.
#[derive(Clone)]
pub struct Detector {
    pub sightings: broadcast::Sender<Arc<Sighting>>,
    looks: Arc<AtomicU64>,
    seen: Arc<AtomicU64>,
}

impl Detector {
    /// How many times it has looked, and how many of those found something.
    pub fn counters(&self) -> (u64, u64) {
        (
            self.looks.load(Ordering::Relaxed),
            self.seen.load(Ordering::Relaxed),
        )
    }
}

/// How often to say out loud what the detector is seeing. 20 looks is ten seconds at 2 Hz.
///
/// **At info, and for [`crate::exposure`]'s `REPORT_TICKS` reason: the question "is the detector
/// alive?" should not need a browser.** A detector that finds nothing writes nothing, and from
/// outside the process that is indistinguishable from three different states — a thread that died,
/// a tee that went quiet, and a detector working perfectly in a room with no ducks in it. Only this
/// line separates them, and `starved` is what separates the middle one: looks that had no frame to
/// look at, which is otherwise a bare `continue`.
const REPORT_LOOKS: u64 = 20;

/// Which runtime is doing the work.
///
/// Chosen by the model's own extension rather than by a config switch: a `.rknn` only runs on the
/// NPU and an `.onnx` only runs on the CPU, so asking somebody to say both is asking them to
/// contradict themselves.
enum Backend {
    Npu(duck_detect::rknn::Model),
    Cpu(duck_detect::onnx::Model),
}

impl Backend {
    fn open(path: &Path) -> Result<Self> {
        let rknn = path.extension().is_some_and(|ext| ext == "rknn");
        if rknn {
            let model = duck_detect::rknn::Model::open(path)?;
            tracing::info!(
                model = %path.display(),
                api = %model.api_version,
                driver = %model.driver_version,
                "duck detector on the npu"
            );
            Ok(Self::Npu(model))
        } else {
            let model = duck_detect::onnx::Model::open(path)?;
            tracing::info!(
                model = %path.display(),
                "duck detector on the cpu — an .onnx model, or an .rknn the npu would not take"
            );
            Ok(Self::Cpu(model))
        }
    }

    fn input(&self) -> (usize, usize, usize) {
        match self {
            Self::Npu(model) => model.input,
            Self::Cpu(model) => model.input,
        }
    }

    fn infer(&mut self, frame: &[u8], out: &mut Vec<f32>) -> Result<()> {
        match self {
            Self::Npu(model) => model.infer(frame, out),
            Self::Cpu(model) => model.infer(frame, out),
        }
    }
}

/// Start looking, on its own thread. The handle is dropped by the caller to stop it.
///
/// Errors from *loading* are returned, because a detector that was asked for and cannot start is
/// worth refusing loudly. Errors from *inference* are logged and the loop carries on: a bad frame is
/// no reason for a robot to stop being able to see the next one.
pub fn spawn_first(
    models: &[std::path::PathBuf],
    frames: Frames,
    hz: f64,
    threshold: f32,
    turn: Turn,
) -> Result<Detector> {
    let mut refused = Vec::new();
    for model in models {
        match spawn(model, frames.clone(), hz, threshold, turn) {
            Ok(detector) => return Ok(detector),
            Err(error) => {
                // Said at `warn` rather than swallowed: falling back to the CPU is a decision worth
                // seeing in a journal, because it is the difference between 60 ms and 60 ms of
                // *somebody else's* CPU.
                tracing::warn!(
                    model = %model.display(),
                    error = %format!("{error:#}"),
                    "that model will not load; trying the next"
                );
                refused.push(format!("{}: {error:#}", model.display()));
            }
        }
    }
    anyhow::bail!(
        "no model would load ({}). For the NPU: sudo /usr/local/sbin/robot-setup-npu",
        refused.join("; ")
    )
}

/// Start looking with one specific model.
pub fn spawn(
    model: &Path,
    frames: Frames,
    hz: f64,
    threshold: f32,
    turn: Turn,
) -> Result<Detector> {
    let mut backend = Backend::open(model)?;
    let (height, width, channels) = backend.input();
    anyhow::ensure!(
        channels == 3 && width == height,
        "this detector wants a square RGB model, got {width}×{height}×{channels}"
    );

    let (sightings, _) = broadcast::channel(8);
    let detector = Detector {
        sightings: sightings.clone(),
        looks: Arc::new(AtomicU64::new(0)),
        seen: Arc::new(AtomicU64::new(0)),
    };
    let looks = Arc::clone(&detector.looks);
    let seen = Arc::clone(&detector.seen);
    let period = Duration::from_secs_f64(1.0 / hz.max(0.1));

    std::thread::Builder::new()
        .name("duck-detect".into())
        .spawn(move || {
            let mut square = Vec::new();
            let mut raw = Vec::new();
            let mut next = Instant::now();
            let mut last_error: Option<String> = None;
            // Since the last report, not for ever: what matters is whether the tee is quiet *now*.
            let mut ticks = 0u64;
            let mut starved = 0u64;
            let mut took_ms = 0.0f64;
            let (mut reported_looks, mut reported_seen) = (0u64, 0u64);

            loop {
                // Paced from the deadline rather than by sleeping a period after the work, so a
                // slow inference eats its own slot instead of drifting the whole loop later.
                let now = Instant::now();
                if next > now {
                    std::thread::sleep(next - now);
                }
                next += period;

                // The sender is the only thing keeping this thread alive: when the pipeline goes
                // away, so does the receiver count — but a broadcast with no receivers is not an
                // error, so the loop ends when the *sender* is dropped, which happens when the
                // `Detector` does.
                if sightings.receiver_count() == 0 && Arc::strong_count(&looks) == 1 {
                    tracing::debug!("nothing holds the detector; stopping");
                    return;
                }

                ticks += 1;
                if ticks >= REPORT_LOOKS {
                    // **Every count on this line is since the last one.** A heartbeat answers "is
                    // it seeing a duck *now*", and a cumulative `seen` cannot: one found twenty
                    // minutes ago and one found this second both read `seen=1`, for ever. The
                    // totals are on the end, named as totals, for the different question of how
                    // long this detector has been up.
                    let (looked, found) =
                        (looks.load(Ordering::Relaxed), seen.load(Ordering::Relaxed));
                    tracing::info!(
                        looks = looked - reported_looks,
                        seen = found - reported_seen,
                        starved,
                        took_ms = format!("{took_ms:.1}"),
                        total_looks = looked,
                        total_seen = found,
                        "duck detector"
                    );
                    (reported_looks, reported_seen) = (looked, found);
                    ticks = 0;
                    starved = 0;
                }

                // Asked for rather than taken: the tee only copies a frame somebody wants, and this
                // is the want — see [`crate::pipeline::Frames`]. `starved` now counts a tee that
                // did not answer within the timeout, which is the same fault it always meant and a
                // narrower one than "there was nothing there when I looked".
                let Some(frame) = frames.next_frame() else {
                    starved += 1;
                    continue;
                };
                if frame.format != crate::pipeline::CAPTURE_FORMAT {
                    tracing::warn!(
                        format = frame.format,
                        "the tee is not carrying {} any more; the detector only knows that one",
                        crate::pipeline::CAPTURE_FORMAT
                    );
                    return;
                }

                let started = Instant::now();
                // One pass from the tee's 4:2:2 straight into the model's square. Converting the
                // whole 720×1280 frame and shrinking it afterwards cost 345 ms of a 407 ms look.
                let fit = letterbox_from_uyvy(
                    &frame.data,
                    frame.width as usize,
                    frame.height as usize,
                    width,
                    turn,
                    &mut square,
                );
                match backend.infer(&square, &mut raw) {
                    Ok(()) => {
                        let found = decode(&raw, fit, threshold, 0.5);
                        looks.fetch_add(1, Ordering::Relaxed);
                        if !found.is_empty() {
                            seen.fetch_add(1, Ordering::Relaxed);
                        }
                        last_error = None;
                        // Ignored on purpose: nobody watching is the ordinary state of a robot with
                        // no console open, and it is not a reason to stop looking.
                        // Upright, because that is the space the boxes are in — a consumer
                        // scaling them against the *camera's* dimensions would have them sideways.
                        let (upright_w, upright_h) =
                            turn.upright(frame.width as usize, frame.height as usize);
                        took_ms = started.elapsed().as_secs_f64() * 1e3;
                        let _ = sightings.send(Arc::new(Sighting {
                            width: upright_w as u32,
                            height: upright_h as u32,
                            found,
                            took_ms,
                        }));
                    }
                    Err(error) => {
                        // Once per distinct message: a failure that repeats at 2 Hz would be 7000
                        // identical lines an hour, which is how a journal stops being read.
                        let text = format!("{error:#}");
                        if last_error.as_deref() != Some(text.as_str()) {
                            tracing::error!(error = %text, "inference failed");
                            last_error = Some(text);
                        }
                    }
                }
            }
        })
        .context("cannot start the detector thread")?;

    Ok(detector)
}

/// What the video is, told to a peer once when its channel opens.
///
/// **The rotation is the point of this.** Nothing rotates pixels any more, so the stream a browser
/// receives is the picture the camera took — sideways, on a robot whose camera is mounted a quarter
/// turn off. The page has to be told by how much, because it cannot infer it: a 180° mount looks
/// exactly like an upright one from the aspect ratio alone.
pub fn video_notification(width: u32, height: u32, rotate_degrees: u32) -> String {
    format!(
        r#"{{"jsonrpc":"2.0","method":"media.video","params":{{"width":{width},"height":{height},"rotate":{rotate_degrees}}}}}"#
    )
}

/// One sighting as a JSON-RPC notification, for the console's control channel.
///
/// A notification, with no id: the page already treats an id-less line as something that streams —
/// `robot.state` is the other one — so this needs no new mechanism at either end. Boxes are in the
/// frame's own pixels and the frame's size comes with them, because the page is drawing on a video
/// element whose size is the browser's business, not ours.
pub fn notification(sighting: &Sighting) -> String {
    let boxes: Vec<String> = sighting
        .found
        .iter()
        .map(|detection| {
            format!(
                r#"{{"x0":{:.1},"y0":{:.1},"x1":{:.1},"y1":{:.1},"score":{:.3}}}"#,
                detection.box_[0],
                detection.box_[1],
                detection.box_[2],
                detection.box_[3],
                detection.score
            )
        })
        .collect();
    // Hand-built rather than through serde: it is five numbers per box at 2 Hz, the shape is fixed,
    // and this crate has no serde dependency to add for it.
    format!(
        r#"{{"jsonrpc":"2.0","method":"media.detections","params":{{"width":{},"height":{},"took_ms":{:.1},"boxes":[{}]}}}}"#,
        sighting.width,
        sighting.height,
        sighting.took_ms,
        boxes.join(",")
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Grey stays grey, and the channels do not swap.
    ///
    /// The notification the console reads, in the shape it reads it.
    ///
    /// Hand-built JSON, so this is the only thing standing between a working overlay and a page
    /// that silently draws nothing: no id (it streams), boxes in the frame's own pixels, and the
    /// frame's size beside them so the page can scale without knowing anything about the camera.
    #[test]
    fn a_sighting_serialises_as_a_notification() {
        let sighting = Sighting {
            width: 720,
            height: 1280,
            found: vec![
                Detection {
                    score: 1.38,
                    box_: [10.0, 20.0, 110.0, 220.0],
                },
                Detection {
                    score: 0.51,
                    box_: [300.5, 400.25, 350.0, 500.0],
                },
            ],
            took_ms: 63.55,
        };
        let line = notification(&sighting);
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("valid json");

        assert_eq!(parsed["jsonrpc"], "2.0");
        assert_eq!(parsed["method"], "media.detections");
        assert!(parsed.get("id").is_none(), "a notification carries no id");
        assert_eq!(parsed["params"]["width"], 720);
        assert_eq!(parsed["params"]["height"], 1280);
        let boxes = parsed["params"]["boxes"].as_array().expect("boxes");
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0]["x0"], 10.0);
        assert_eq!(boxes[0]["y1"], 220.0);
        assert_eq!(boxes[1]["score"], 0.51);
    }

    /// Nothing found is still a message: the page has to clear its boxes.
    ///
    /// A detector that goes quiet when it sees nothing leaves the last duck drawn on screen for
    /// ever, which looks exactly like a duck that is still there.
    #[test]
    fn an_empty_sighting_is_still_sent() {
        let line = notification(&Sighting {
            width: 720,
            height: 1280,
            found: Vec::new(),
            took_ms: 60.0,
        });
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("valid json");
        assert_eq!(parsed["params"]["boxes"].as_array().map(Vec::len), Some(0));
    }
}
