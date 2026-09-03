//! The GStreamer pipeline, and the datachannel the control channel rides on.
//!
//! Linux only, the way `padd`'s evdev tap is: the daemon runs on the robot, and everything else in
//! this crate — [`crate::route`], [`crate::session`], [`crate::upstream`] — is portable and stays
//! testable on a laptop. Gating here rather than behind a feature keeps `cargo test` honest on both.
//!
//! ## Shape
//!
//! ```text
//!                                                ┌─ queue ─ webrtcsink ─ setup → mpph264enc
//! videotestsrc | camera ─ NV12 ─ videoflip ─ tee ┤          │
//!                                                └─ queue ─ │  run-signalling-server=true
//!                                       (leaky)     │
//!                                          │        └─ consumer-added → "control" channel
//!                                      appsink
//!                                   a frame on request
//! ```
//!
//! **The tee is on raw NV12, before the encoder**, and that placement is the point of it.
//! `architecture.md` §5.3 wants a frame on demand for a server-side program — "it wants a frame
//! every second or two plus a state blob", not a 30 fps H.264 track to decode — and §2 wants
//! perception next to the sensor, deriving features rather than shipping pixels to `robotd`. Both
//! need pixels, and taking them off the encoded branch would mean decoding what we just encoded.
//!
//! NV12 because that is what the rkisp capture path emits and what `mpph264enc` takes, so nothing
//! *converts* anywhere: no `videoconvert`, and no RGA pass, between capture and either consumer.
//!
//! **Nothing converts and nothing rotates.** The head camera is mounted a quarter turn off, and for
//! one afternoon this pipeline fixed that with a `videoflip` before the tee — which broke
//! `mpph264enc`'s zero-copy path to the 2D engine and had MPP converting every frame in software:
//! 97 °C, the CPU throttled to 408 MHz, 8 fps out of a 30 fps camera. Rotation is now the
//! consumer's business, because for both consumers it is free — a CSS transform in the browser, and
//! a resample the detector was doing anyway. [`Rotation`] has the numbers.
//!
//! **Each branch has its own `queue`, and the raw one is leaky.** A `tee` without queues runs its
//! branches on one thread, so a slow consumer stalls the others — here that would mean a
//! perception consumer pausing the video track. The raw branch drops old frames rather than
//! applying backpressure, which is the semantics `architecture.md` §2 asks for: the *latest*
//! snapshot, never a queue of stale ones. A stalled reader costs frames, never the encoder.
//!
//! **And the raw branch only copies a frame somebody asked for.** Every buffer reaches the appsink
//! and nearly all of them are dropped there unread — see [`crate::pipeline::Frames`] for what that saves and why the
//! request has to be answered by the *next* capture rather than the last one.
//!
//! **`webrtcsink` runs the signalling server in this process** (`run-signalling-server`, with
//! `signalling-server-host` and `-port`), so the separate `gst-webrtc-signalling-server` binary
//! never has to be built or shipped — what we ship from that upstream is a `.so`.
//! `remote-webrtc.md` §3.
//!
//! **`webrtcsink` owns the encoder, and is handed raw video.** It briefly did not — the pipeline
//! was `mpph264enc ! h264parse ! webrtcsink`, which worked and quietly gave up two things: with
//! pre-encoded input `webrtcsink` cannot reach the encoder, so its congestion control cannot adapt
//! the bitrate to the link, and a peer's PLI cannot produce a keyframe, leaving a viewer that lost
//! one broken until the next periodic GOP.
//!
//! That costs a software `videoconvert ! videoscale` in front of any encoder `webrtcsink` does not
//! recognise, and it does not recognise `mpph264enc` — so **the plugin we ship carries a patch**
//! adding that arm. Without the patch this arrangement is slower than pre-encoding rather than
//! faster; the two belong together. See `patches/` in `pollen-robotics/microduck-gst-plugins`.
//!
//! The encoder settings survive through `encoder-setup` — see [`wire_encoder_setup`], which is
//! also where a fallback to software encoding gets noticed.
//!
//! ## A test pattern before a camera
//!
//! The default source is `videotestsrc`. That is not a placeholder for want of a better idea: it is
//! the source that works on a board with no camera attached, which is most of them, and it makes
//! the whole session — signalling, negotiation, the datachannel, the control API — exercisable
//! without the capture path existing. The camera arrives as a different source element behind the
//! same encoder, and `media-bringup.md` records why capture cannot simply be `v4l2src`.
//!
//! ## What is not verified
//!
//! **Nothing in a signal handler here may panic.** These closures are invoked from C, so a panic
//! does not unwind — it aborts the process, and the journal shows `thread caused non-unwinding
//! panic` with a backtrace through `g_closure_invoke` and nothing about what was actually wrong.
//! The first board run died exactly that way, from `tokio::spawn` on a GStreamer thread that has
//! no runtime. So: the runtime handle is captured where one exists and spawned onto explicitly,
//! and every signal is checked to exist before it is connected or emitted — `emit_by_name` and
//! `connect` both panic on an absent name.
//!
//! What is left of that risk is a signature that exists but differs, which shows up as a warning
//! naming the arity rather than as an abort.

use std::sync::{Arc, Mutex};

use anyhow::{Context, Result, anyhow, bail};
use duck_ipc_proto as proto;
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_video as gst_video;
use gstreamer_webrtc as gst_webrtc;
use tokio::sync::mpsc;

/// How far the picture is turned *in the pipeline* — which, by default, is not at all.
///
/// **This defaulted to a quarter turn for one afternoon and cost 145% of a core.** `mpph264enc`
/// hands the UYVY→NV12 conversion to the SoC's 2D engine and pays nothing for it; `videoflip`'s
/// output is a buffer the RGA refuses — `10000 is unsupport format`, then `RGA_BLIT fail: Bad
/// address` on a `rect[0, 0, 720, 1280]` — so MPP fell back to converting **every frame in
/// software**. Measured on the robot: 97 °C, the CPU throttled from 1.8 GHz to 408 MHz, 1565 frames
/// lost by `v4l2src` in one session, and 8 fps out of a 30 fps camera. The boot before that change
/// had zero RGA failures and zero lost frames.
///
/// So the pipeline no longer rotates pixels. The camera is still mounted a quarter turn off, and the
/// consumer that has to care rotates for itself, for free: the console does it with a CSS transform
/// on the GPU, and a perception consumer folds the turn into the resampling it already does.
/// [`Settings::rotation`] stays for a consumer that genuinely needs an upright *encoded* stream —
/// `--flip-in-pipeline` — and now that its cost is written down, that is a choice rather than a
/// default.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rotation {
    /// Leave the frame as the sensor delivered it.
    None,
    /// A quarter turn clockwise: what this robot's mount needs.
    Cw90,
    Cw180,
    /// A quarter turn anticlockwise, for a head assembled the other way round.
    Cw270,
}

impl Rotation {
    /// From degrees clockwise, which is how the flag is written.
    pub fn from_degrees(degrees: u32) -> Result<Self> {
        match degrees {
            0 => Ok(Self::None),
            90 => Ok(Self::Cw90),
            180 => Ok(Self::Cw180),
            270 => Ok(Self::Cw270),
            other => Err(anyhow!(
                "rotation must be 0, 90, 180 or 270 degrees clockwise, not {other}"
            )),
        }
    }

    /// `videoflip`'s `video-direction`, or `None` where there is nothing to do.
    ///
    /// `90r` is clockwise and `90l` anticlockwise, which is GStreamer's naming and not ours.
    fn video_direction(self) -> Option<&'static str> {
        match self {
            Self::None => None,
            Self::Cw90 => Some("90r"),
            Self::Cw180 => Some("180"),
            Self::Cw270 => Some("90l"),
        }
    }

    /// The frame size after the turn. A quarter turn swaps the axes; a half turn does not.
    ///
    /// Everything that reads a raw frame depends on this being right: [`Frame`] carries the
    /// dimensions the buffer is in, and a consumer handed 1280x720 for a 720x1280 buffer reads
    /// the picture diagonally rather than failing.
    fn output(self, width: u32, height: u32) -> (u32, u32) {
        match self {
            Self::Cw90 | Self::Cw270 => (height, width),
            Self::None | Self::Cw180 => (width, height),
        }
    }
}

/// Where the signalling server listens, and what the video is.
///
/// One value rather than six positional arguments. `start` had reached eight of them, and two of
/// those are a `width` and a `height` of the same type: a call site that swapped them would compile
/// and produce a portrait stream. Named fields make that unrepresentable, and a seventh setting
/// stops changing the signature.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Settings {
    /// Where the signalling server binds. All interfaces on the robot — `remote-webrtc.md` §3.
    pub host: String,
    /// The signalling server's port, which is *not* the console's: [`crate::web`] owns that one.
    pub port: u32,
    /// Starting video bitrate, bits per second. Congestion control moves it from here — unless
    /// it is `disabled`, which is what makes this the rate rather than a starting point.
    pub bitrate: u32,
    /// Whether the send rate adapts to the link, and by what. `robotd_params::CongestionControl`
    /// has the trade, and it is a CPU one as much as a network one.
    pub congestion_control: robotd_params::CongestionControl,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    /// How far the *pipeline* turns the picture; see [`Rotation`]. Almost always `None` — the
    /// capture geometry above is then also what leaves the tee.
    pub rotation: Rotation,
}

/// Where the video comes from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Source {
    /// A test pattern. Works with no camera attached, which is what makes a session testable
    /// before the capture path exists.
    Test,
    /// The head camera, through the rkisp capture path.
    Camera(Camera),
}

/// The head camera, and the two things it will not work without.
///
/// **These are a starting exposure, not a policy.** A capture with the driver's boot defaults
/// comes out black, so something has to write the sensor before the first frame — that is what
/// these are for. From there [`crate::exposure`] meters the picture and takes over, because
/// Rockchip's 3A engine converges once at stream start and then stops — and does not manage even
/// that if it missed the stream-start event. With `--no-auto-exposure` these are all there is and
/// the picture stays at one brightness. Values are in the sensor's own units: exposure in lines (~19 µs each)
/// and analogue gain where 256 is 1x, up to 2816 for 11x.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Camera {
    pub device: String,
    pub exposure: u32,
    pub analogue_gain: u32,
}

/// One raw frame off the tee, as the last one seen.
#[derive(Debug, Clone)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    /// The GStreamer format name — [`CAPTURE_FORMAT`], carried rather than assumed so a consumer
    /// reading this cannot silently misinterpret the bytes if the capture format changes again.
    pub format: &'static str,
    /// Tightly packed as the caps describe it, in `format`.
    pub data: Vec<u8>,
}

/// A frame off the tee, handed over on request.
///
/// **Asked for, not published.** Copying a frame out of the tee costs the whole frame — 1.84 MB at
/// 720p30, the quality every robot ships at — and the branch's readers want two a second between
/// them: the auto-exposure loop meters one every 500 ms, and the duck detector looks twice a second
/// when it is switched on at all. Capturing all thirty meant **55 MB/s of memcpy and a 1.8 MB
/// allocation thirty times a second, from boot, on every robot**, for twenty-eight frames nobody
/// ever read. A reader now says when it wants one, and the appsink callback's cost on every other
/// frame is a relaxed load and dropping the sample.
///
/// **A reader waits for the next frame rather than taking the last one.** The demand has to be
/// answered by a capture that happens *after* it, or the saving would come straight back out of the
/// picture: a reader polling every 500 ms would meter a frame captured for the previous poll, and an
/// exposure loop steering on half-second-old luma is a loop that hunts. So the request marks the
/// next frame, and the reader blocks until it lands — [`FRAME_TIMEOUT`] bounds that for a camera
/// that has stopped delivering.
///
/// One captured frame answers every reader waiting for one: the flag is a single bit and the
/// delivery wakes all of them, so the detector and the exposure loop asking at the same moment cost
/// one copy between them rather than two.
#[derive(Clone, Default)]
pub struct Frames(Arc<Shared>);

/// The reader/callback rendezvous. [`Frames`] is the handle; this is what both ends touch.
#[derive(Default)]
struct Shared {
    latest: Mutex<Latest>,
    /// Signalled when [`Latest::generation`] moves, which is the only thing a reader waits on.
    delivered: std::sync::Condvar,
    /// A reader is waiting for the next frame. Read once per buffer and `Relaxed` on purpose:
    /// nothing is published through it — the frame itself goes through `latest`, whose mutex
    /// carries the ordering — so this only ever has to be eventually true, and it is the one thing
    /// the callback does on a frame nobody asked for.
    wanted: std::sync::atomic::AtomicBool,
}

#[derive(Default)]
struct Latest {
    frame: Option<Frame>,
    /// Bumped on every delivery, so a reader can tell the frame it asked for from the one that was
    /// already there. A counter rather than an `Option::take`, because two readers waiting on the
    /// same capture must both see it.
    generation: u64,
}

/// How long a reader waits for the frame it asked for before giving up on it.
///
/// Fifteen frame periods at 720p30 and seven at 720p15 — far longer than any capture hiccup, and
/// short enough that a reader on a camera that has stopped goes back round its own loop and says so
/// rather than parking for ever. Both readers already have a "no frame" path; this is what reaches
/// it.
pub const FRAME_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(500);

impl Frames {
    /// Ask for the next frame and read it in place, without copying it.
    ///
    /// For a reader that wants a number out of a frame rather than the frame — the auto-exposure
    /// loop wants a mean, and copying 1.84 MB twice a second to average 11k bytes of it is a memcpy
    /// nobody needs.
    ///
    /// `None` when no frame arrived within [`FRAME_TIMEOUT`].
    pub fn inspect_next<T>(&self, read: impl FnOnce(&Frame) -> T) -> Option<T> {
        self.next_capture(|frame| frame.map(read))
    }

    /// Ask for the next frame and take a copy of it. `None` when none arrived within
    /// [`FRAME_TIMEOUT`].
    pub fn next_frame(&self) -> Option<Frame> {
        self.next_capture(|frame| frame.cloned())
    }

    /// Register the demand, wait for the capture that answers it, and read what landed.
    ///
    /// The generation is read *before* the flag is set: a capture that lands between the two is one
    /// this reader asked for as far as it can tell, and waiting out a whole extra frame to be
    /// pedantic about it would only make the answer staler.
    fn next_capture<T>(&self, read: impl FnOnce(Option<&Frame>) -> T) -> T {
        let latest = self.0.latest.lock().expect("frame lock");
        let seen = latest.generation;
        self.0
            .wanted
            .store(true, std::sync::atomic::Ordering::Relaxed);
        let (latest, _) = self
            .0
            .delivered
            .wait_timeout_while(latest, FRAME_TIMEOUT, |latest| latest.generation == seen)
            .expect("frame lock");
        // On a timeout this is whatever the last delivery left — `None` on a camera that has never
        // produced one, and a stale frame on one that has stopped. Reported as the timeout it is
        // rather than as that frame: a reader told "here is a frame" cannot tell the difference,
        // and the exposure loop steering on a frame from a minute ago is the failure this whole
        // rendezvous exists to prevent.
        if latest.generation == seen {
            return read(None);
        }
        read(latest.frame.as_ref())
    }

    /// Whether a reader is waiting, taking the request if one is. The callback's whole cost on a
    /// frame nobody asked for.
    fn take_request(&self) -> bool {
        self.0
            .wanted
            .swap(false, std::sync::atomic::Ordering::Relaxed)
    }

    /// Hand the captured frame to whoever is waiting for it.
    fn deliver(&self, frame: Frame) {
        let mut latest = self.0.latest.lock().expect("frame lock");
        latest.frame = Some(frame);
        latest.generation = latest.generation.wrapping_add(1);
        drop(latest);
        // Every waiter, not one: a single capture is the answer to every request outstanding when
        // it was taken.
        self.0.delivered.notify_all();
    }
}

/// What a peer's control channel needs to talk to [`crate::session::run`].
pub struct Channel {
    /// Lines the peer sent.
    pub inbound: mpsc::Receiver<String>,
    /// Lines to send the peer.
    pub outbound: mpsc::Sender<String>,
}

/// Build and start the pipeline. Returns it, plus a stream of control channels — one per peer.
///
/// The pipeline is returned rather than kept here so the caller owns its lifetime: dropping it
/// stops the session, which is what a shutdown should do.
pub fn start(
    source: Source,
    producer: &crate::producer::Producer,
    settings: &Settings,
) -> Result<(gst::Pipeline, mpsc::Receiver<Channel>, Frames)> {
    let &Settings {
        port,
        bitrate,
        congestion_control,
        width,
        height,
        fps,
        rotation,
        ..
    } = settings;

    // What leaves the pipeline, which is what every consumer downstream of the tee sees. Only the
    // capture side uses `width`/`height` from here on.
    let (out_width, out_height) = rotation.output(width, height);
    let host = settings.host.as_str();

    // `GST_DEBUG` has to be in the environment before `init`, which is when GStreamer parses it.
    set_gstreamer_log_threshold();

    gst::init().context("gstreamer would not initialise")?;

    // And the log functions have to be swapped *after* it, which is the fix for INFO and below
    // never arriving — see [`bridge_gstreamer_log`].
    bridge_gstreamer_log();

    // **A GStreamer signal handler runs on a GStreamer thread, which is not inside the tokio
    // runtime.** `tokio::spawn` there panics with "there is no reactor running", and a panic
    // crossing the C closure boundary is a non-unwinding abort — the whole daemon dies with
    // SIGABRT from inside `g_closure_invoke`, which is exactly what the first board run did. So
    // the handle is captured here, where there *is* a runtime, and the handler spawns onto it.
    let runtime = tokio::runtime::Handle::try_current().context(
        "pipeline::start must be called from inside a tokio runtime: the datachannel writer is \
         spawned onto it from a GStreamer signal thread, which has no runtime of its own",
    )?;

    let pipeline = gst::Pipeline::new();

    let src = match &source {
        Source::Test => {
            let src = make("videotestsrc")?;
            // `is-live` so the pipeline behaves like a camera does rather than racing ahead of
            // the clock. A camera is live by construction and needs no such property.
            src.set_property("is-live", true);
            src
        }
        Source::Camera(camera) => camera_source(camera, fps)?,
    };

    // Pinned rather than negotiated, because both branches of the tee depend on the answer, and a
    // raw consumer that has to guess the format is one that gets it wrong the first time the
    // source changes.
    //
    // **`UYVY` rather than `NV12`, and that is a measurement rather than a preference.** rkisp
    // offers a two-plane, non-contiguous `NM12` alongside single-plane formats, and asking for
    // GStreamer `NV12` selects `NM12` — which `v4l2src` cannot push at full rate on this driver
    // whatever the buffer depth. 300 frames of 720p off the ISP main path:
    //
    // | caps | 2 buffers | 4+ buffers |
    // |---|---|---|
    // | `NV12` (selects `NM12`, 2 planes) | 19.5 fps | 19.6 fps |
    // | `UYVY` (1 plane) | 19.7 fps | **29.3 fps** |
    //
    // The buffer depth alone is not enough and neither is the format — see
    // [`raise_capture_buffers`] for the other half. `v4l2-ctl` reaches 29.2 fps with either
    // format, so this is `v4l2src`'s multi-plane path rather than the driver.
    //
    // `mpph264enc` lists `UYVY` on its sink pad and converts on the SoC's 2D accelerator, so the
    // 4:2:2 to 4:2:0 step costs no CPU — the RGA was already doing one operation per frame.
    let caps = gst::Caps::builder("video/x-raw")
        .field("format", CAPTURE_FORMAT)
        .field("width", width as i32)
        .field("height", height as i32)
        .field("framerate", gst::Fraction::new(fps as i32, 1))
        .build();
    let capsfilter = gst::ElementFactory::make("capsfilter")
        .property("caps", &caps)
        .build()
        .map_err(|_| anyhow!("no capsfilter element; gstreamer core is incomplete"))?;

    // ── the turn, when somebody asks for it ─────────────────────────────────
    //
    // `videoflip` from `gstreamer1.0-plugins-good`, and **off unless asked**: see [`Rotation`] for
    // the measurement. It does not merely cost a CPU pass of its own — it takes the encoder's
    // zero-copy path down with it, because the RGA will not touch what it produces, so the true
    // price is a software colour conversion of every frame as well.
    //
    // Left in for the consumer that cannot rotate for itself and can afford this. If that ever
    // becomes the common case, the fix is an RGA element in the plugin set (the 2D engine can
    // rotate for nothing), not this.
    let flip = match rotation.video_direction() {
        Some(direction) => Some(
            gst::ElementFactory::make("videoflip")
                .property_from_str("video-direction", direction)
                .build()
                .map_err(|_| {
                    anyhow!(
                        "no videoflip element, so the picture cannot be turned the right way up. \
                         It comes from gstreamer1.0-plugins-good: \
                         sudo /usr/local/sbin/robot-setup-gstreamer"
                    )
                })?,
        ),
        None => None,
    };

    let tee = make("tee")?;

    // ── the video branch ────────────────────────────────────────────────────
    //
    // Its own queue, so this branch runs on its own thread. Without one, `tee` pushes to both
    // branches from a single thread and whichever is slower holds up the other.
    let video_queue = make("queue")?;

    // **Raw video in, and `webrtcsink` owns the encoder.** This used to be
    // `mpph264enc ! h264parse ! webrtcsink`, which worked and gave up two things quietly: with
    // pre-encoded input `webrtcsink` cannot reach the encoder, so its congestion control cannot
    // adapt the bitrate to the link, and a peer's PLI cannot produce a keyframe — a viewer that
    // loses one stays broken until the next periodic GOP.
    //
    // Handing it raw video costs a software `videoconvert ! videoscale` in front of whatever
    // encoder it picks, unless it knows the encoder. It does not know `mpph264enc`, so the
    // plugin we ship carries a patch adding that arm — see `patches/` in
    // pollen-robotics/microduck-gst-plugins. Without it this is *slower* than pre-encoding, not
    // faster, so the two changes belong together.
    //
    // Which encoder it picks is by rank: `mpph264enc` registers at primary+1 (257), above
    // `x264enc`. Worth confirming with `GST_DEBUG=webrtcsink:4` rather than trusting, because the
    // failure mode is a robot quietly encoding in software.
    let sink = gst::ElementFactory::make("webrtcsink")
        .build()
        .map_err(|_| {
            anyhow!(
                "no webrtcsink. It comes from gst-plugins-rs, which Debian packages in no suite — \
             setup-gstreamer.sh installs it from the microduck-gst-plugins release, and \
             GST_PLUGIN_PATH must include /usr/local/lib/gstreamer-1.0."
            )
        })?;
    sink.set_property("run-signalling-server", true);
    sink.set_property("signalling-server-host", host);
    sink.set_property("signalling-server-port", port);

    // Who this robot is, handed to every peer in the signalling server's `list` answer — so a
    // client knows which robot it found before it negotiates anything. [`crate::producer`] is what
    // goes in it and why. A structure name is required and is not what a peer reads; `meta` is
    // webrtcsink's own word for the property.
    //
    // **Checked before it is set, for the reason every signal in this file is checked**:
    // `set_property` panics on a name the element does not have, and a panic here is a daemon that
    // will not start — costing the video and the control channel to gain a producer's name. This is
    // the newest thing this function touches, so it is the one most likely to be wrong about a
    // spelling, and a producer that is merely anonymous is a far better failure.
    if sink.has_property("meta") {
        let mut meta = gst::Structure::builder("meta");
        for (field, value) in producer.fields() {
            meta = meta.field(field, value);
        }
        sink.set_property("meta", meta.build());
    } else {
        tracing::warn!(
            "webrtcsink has no `meta` property on these plugins, so peers see a producer id and \
             nothing else. Everything else is unaffected."
        );
    }

    // Offer H.264 and nothing else. Left alone `webrtcsink` proposes everything it can encode:
    // `mppvp8enc`, `mpph265enc` and `mpph264enc` on the VPU, but `vp9enc` and `av1enc` in
    // *software*. A browser preferring AV1 would have this robot software-encoding AV1 on four
    // Cortex-A55s, which is not a degraded stream but a dead control loop.
    //
    // **No `profile` field here, deliberately.** `webrtcsink` reads one off these caps and does
    // `H264_PROFILES_COMPAT.iter().position(..).expect("Unsupported H264 profile")` — a panic, in
    // a plugin, for a value it does not know. Omitting the field skips that path, and the profile
    // is set on the encoder itself in `wire_encoder_setup` where it belongs.
    //
    // This restriction was held back for a while because H.264 was missing from the offer, and
    // restricting to a codec that fails discovery leaves *no* codecs. The cause was
    // `mpph264enc`'s pad template omitting `constrained-baseline`, which is the one profile
    // `webrtcsink`'s discovery pass demands; the plugins release carries a patch for it from `v3`.
    // If a robot on older plugins reaches here it now fails loudly — no producer at all — rather
    // than quietly serving VP8.
    sink.set_property("video-caps", gst::Caps::builder("video/x-h264").build());

    // The starting bitrate. `webrtcsink` moves it from here as congestion control learns the
    // link — which is the whole point of letting it own the encoder, so this is a starting
    // point rather than the setting it was when we encoded ourselves. Unless the estimator is
    // off, and then nothing moves it and this is the rate.
    sink.set_property("start-bitrate", bitrate);

    set_congestion_control(&sink, congestion_control);

    // The encoder settings, applied through the hook that exists for it.
    //
    // Handing `webrtcsink` the encoder would otherwise *lose* them, which would make this change a
    // regression rather than an improvement: `profile` defaults to High and `header-mode` to
    // first-frame, and both matter — see `wire_encoder_setup`.
    wire_encoder_setup(&sink)?;

    let consumers: Consumers = Arc::new(std::sync::atomic::AtomicU32::new(0));
    let (channels_tx, channels_rx) = mpsc::channel::<Channel>(4);
    wire_consumers(&sink, channels_tx, runtime, consumers.clone())?;

    // ── the raw branch ──────────────────────────────────────────────────────
    //
    // Leaky downstream and one buffer deep: when the reader is behind, the *oldest* frame is
    // dropped and the newest kept. That is last-value-wins, and it is what keeps a slow perception
    // consumer from ever becoming the video track's problem.
    let raw_queue = gst::ElementFactory::make("queue")
        .property("max-size-buffers", 1u32)
        .property("max-size-bytes", 0u32)
        .property("max-size-time", 0u64)
        .property_from_str("leaky", "downstream")
        .build()
        .map_err(|_| anyhow!("no queue element; gstreamer core is incomplete"))?;

    // The turn happens before the tee, so this branch carries the rotated geometry. Built from
    // `out_width`/`out_height` rather than reusing `caps`: handing the appsink the capture caps
    // would fail negotiation on a quarter turn, and silently describe the wrong shape on a half
    // one.
    let out_caps = gst::Caps::builder("video/x-raw")
        .field("format", CAPTURE_FORMAT)
        .field("width", out_width as i32)
        .field("height", out_height as i32)
        .field("framerate", gst::Fraction::new(fps as i32, 1))
        .build();

    let frames = Frames::default();
    let appsink = gst_app::AppSink::builder()
        .caps(&out_caps)
        // `sync=false` so this branch never waits on the clock: a snapshot wants the newest frame
        // as soon as it exists, and pacing it would only add latency to a consumer that is not
        // rendering anything.
        .sync(false)
        .max_buffers(1)
        .drop(true)
        .build();
    wire_frames(&appsink, frames.clone(), out_width, out_height);

    if let Some(flip) = flip.as_ref() {
        pipeline
            .add(flip)
            .context("could not add videoflip to the pipeline")?;
    }

    pipeline
        .add_many([
            &src,
            &capsfilter,
            &tee,
            &video_queue,
            &sink,
            &raw_queue,
            appsink.upcast_ref(),
        ])
        .context("could not add elements to the pipeline")?;

    // On the capsfilter's src pad, which is the last point before the tee splits the stream —
    // so this counts every frame the driver delivered, with nothing lossy in between.
    meter_capture_rate(
        &capsfilter
            .static_pad("src")
            .context("capsfilter has no src pad, which cannot happen")?,
        width,
        height,
        fps,
        consumers.clone(),
    )?;

    match flip.as_ref() {
        Some(flip) => gst::Element::link_many([&src, &capsfilter, flip, &tee]),
        None => gst::Element::link_many([&src, &capsfilter, &tee]),
    }
    .context(
        "could not link the source to the tee. A caps failure here means the source cannot \
         produce NV12 at the requested size and rate.",
    )?;
    gst::Element::link_many([&video_queue, &sink])
        .context("could not link the video queue to webrtcsink")?;
    gst::Element::link_many([&raw_queue, appsink.upcast_ref()])
        .context("could not link the raw branch to its appsink")?;

    // `tee`'s source pads are request pads: they do not exist until asked for, which is why these
    // two links are separate from the `link_many` chains above.
    link_tee_branch(&tee, &video_queue).context("could not attach the video branch to the tee")?;
    link_tee_branch(&tee, &raw_queue).context("could not attach the raw branch to the tee")?;

    // **Watch the bus, or every media failure is silent.**
    //
    // This was learned the hard way. `webrtcsink` drops a codec whose discovery pipeline fails
    // with nothing more than `gst::warning!` — "We don't consider this fatal, as long as we end up
    // with one potential codec" — and a consumer pipeline that dies posts an ERROR to the bus.
    // Neither reaches `tracing`, so the journal showed a session starting, a session ending, and
    // no reason for either. Two rounds of guessing went into diagnosing something GStreamer was
    // already saying out loud.
    watch_bus(&pipeline);

    pipeline
        .set_state(gst::State::Playing)
        .context("the pipeline would not start")?;

    tracing::info!(
        host,
        port,
        ?source,
        width,
        height,
        out_width,
        out_height,
        ?rotation,
        fps,
        "signalling server listening"
    );
    Ok((pipeline, channels_rx, frames))
}

/// Request a source pad from the tee and link it to a branch's sink pad.
fn link_tee_branch(tee: &gst::Element, branch: &gst::Element) -> Result<()> {
    let src_pad = tee
        .request_pad_simple("src_%u")
        .ok_or_else(|| anyhow!("the tee would not give a source pad"))?;
    let sink_pad = branch
        .static_pad("sink")
        .ok_or_else(|| anyhow!("the branch has no sink pad"))?;
    src_pad
        .link(&sink_pad)
        .map_err(|e| anyhow!("linking a tee branch failed: {e:?}"))?;
    Ok(())
}

/// Answer a reader's request for a frame out of the raw branch, and drop every other buffer.
fn wire_frames(appsink: &gst_app::AppSink, frames: Frames, width: u32, height: u32) {
    appsink.set_callbacks(
        gst_app::AppSinkCallbacks::builder()
            .new_sample(move |sink| {
                // Pulled whatever happens, and *before* the request is looked at. A sample left
                // unpulled sits in the appsink's queue holding one of the capture pool's buffers,
                // and [`CAPTURE_BUFFERS`] is four with three the cliff — declining to copy a frame
                // must not cost the pipeline a buffer to decline it with. Dropped at the end of
                // this scope instead, which returns the buffer to the pool.
                let sample = sink.pull_sample().map_err(|_| gst::FlowError::Eos)?;
                if !frames.take_request() {
                    return Ok(gst::FlowSuccess::Ok);
                }
                let Some(buffer) = sample.buffer() else {
                    return Ok(gst::FlowSuccess::Ok);
                };
                // `UYVY` is a single plane, so this maps without merging anything — unlike the
                // `NM12` this used to carry, where mapping silently copied two non-contiguous
                // planes into one block. The `to_vec` below is still a copy, and still the only
                // one on this branch.
                let Ok(map) = buffer.map_readable() else {
                    // A buffer that will not map is not worth failing the pipeline over — the next
                    // one is a frame away, and this branch is advisory by design. The request has
                    // been taken by now, so the reader waits out its timeout rather than being
                    // answered with nothing; a map that fails twice running is a pipeline in
                    // trouble, not a frame to retry for.
                    tracing::debug!("a raw frame would not map");
                    return Ok(gst::FlowSuccess::Ok);
                };
                frames.deliver(Frame {
                    width,
                    height,
                    format: CAPTURE_FORMAT,
                    data: map.as_slice().to_vec(),
                });

                Ok(gst::FlowSuccess::Ok)
            })
            .build(),
    );
}

/// Put a default `GST_DEBUG` in the environment, before `gst::init` reads it.
///
/// Honoured if already set, so raising a category still works the usual way. Unset, `WARNING` is
/// enough to catch a codec being dropped or an element refusing, and quiet enough for a journal.
fn set_gstreamer_log_threshold() {
    if std::env::var_os("GST_DEBUG").is_none() {
        // SAFETY: single-threaded here — this runs before `gst::init` and before any task is
        // spawned, which is the only point at which setting an env var is sound.
        unsafe { std::env::set_var("GST_DEBUG", "*:WARNING") };
    }
}

/// Send GStreamer's own log into `tracing`, so the journal shows what it says.
///
/// **The bus is not enough.** `webrtcsink` drops a codec whose discovery pipeline fails with a
/// `gst::warning!` and nothing else — "We don't consider this fatal, as long as we end up with one
/// potential codec for each input stream" — and that goes to GStreamer's debug log, not the bus. So
/// a robot offering VP8 instead of H.264 said nothing at all about why, and it had been saying it
/// the whole time to a log nobody was reading.
///
/// **Called after `gst::init`, and that is the point of splitting this in two.** It ran before
/// `init` originally, on the reasoning that anything said earlier would be lost. That cost more
/// than it saved: `WARNING` and `ERROR` arrived but `INFO` and below never did, whatever
/// `GST_DEBUG` said — so `GST_DEBUG=v4l2bufferpool:4` produced nothing at all, and two capture
/// questions that `gst_v4l2_object_decide_allocation` answers in its own `GST_INFO` log had to be
/// inferred from frame rates instead. Both inferences were wrong.
///
/// What is lost by moving it is a handful of registry-scan lines from before `init`, which said
/// nothing anyone wanted.
///
/// The effective threshold is reported once at startup: a logger that cannot say what it will and
/// will not forward is what made this expensive.
fn bridge_gstreamer_log() {
    // Otherwise every message is printed to stderr by GStreamer *and* logged by us, which in a
    // journal is the same line twice with different formatting.
    gst::log::remove_default_log_function();

    // This is called from arbitrary GStreamer threads and from C, so — as everywhere in this file
    // — it must not panic. It formats and forwards, and nothing else.
    gst::log::add_log_function(|category, level, file, _function, line, object, message| {
        let text = message.get().unwrap_or_default();
        let src = object
            .map(|o| o.to_string())
            .unwrap_or_else(|| "-".to_string());
        let cat = category.name();
        match level {
            gst::DebugLevel::Error => {
                tracing::error!(target: "gst", %cat, %src, %file, line, "{text}")
            }
            gst::DebugLevel::Warning => {
                tracing::warn!(target: "gst", %cat, %src, %file, line, "{text}")
            }
            gst::DebugLevel::Fixme | gst::DebugLevel::Info => {
                tracing::info!(target: "gst", %cat, %src, "{text}")
            }
            _ => tracing::debug!(target: "gst", %cat, %src, "{text}"),
        }
    });

    // Not the same question as what `GST_DEBUG` says: a per-category threshold only takes effect
    // if the global minimum lets the message reach a log function at all. Printed so the next
    // person raising a category can see whether it took.
    tracing::info!(
        gst_debug = %std::env::var("GST_DEBUG").unwrap_or_else(|_| "(unset)".into()),
        default_threshold = ?gst::log::get_default_threshold(),
        "gstreamer log bridged"
    );
}

/// Forward what the pipeline says about itself into the journal.
///
/// A dedicated thread rather than `bus.add_watch`, which needs a GLib main loop this daemon does
/// not run, and rather than a tokio task, because `timed_pop` blocks.
fn watch_bus(pipeline: &gst::Pipeline) {
    let Some(bus) = pipeline.bus() else {
        tracing::warn!("the pipeline has no bus; media failures will be silent");
        return;
    };
    std::thread::Builder::new()
        .name("gst-bus".into())
        .spawn(move || {
            // `None` blocks until a message arrives; the loop ends when the bus is flushed on
            // teardown, which is the daemon exiting.
            while let Some(msg) = bus.timed_pop(gst::ClockTime::NONE) {
                let src = msg
                    .src()
                    .map(|s| s.path_string().to_string())
                    .unwrap_or_else(|| "?".into());
                match msg.view() {
                    gst::MessageView::Error(e) => {
                        // `debug` carries the element's own detail, which is usually the part that
                        // names the actual cause — a caps mismatch, a device that would not open.
                        tracing::error!(
                            %src,
                            error = %e.error(),
                            detail = e.debug().unwrap_or_default().as_str(),
                            "pipeline error"
                        );
                    }
                    gst::MessageView::Warning(w) => {
                        tracing::warn!(
                            %src,
                            warning = %w.error(),
                            detail = w.debug().unwrap_or_default().as_str(),
                            "pipeline warning"
                        );
                    }
                    // Everything else is state changes and stream status at a rate nobody wants in
                    // a journal — visible with GST_DEBUG when it is wanted.
                    _ => {}
                }
            }
            tracing::debug!("bus watch ended");
        })
        .map(|_| ())
        .unwrap_or_else(
            |e| tracing::warn!(error = %e, "no bus watch thread; failures will be silent"),
        );
}

fn make(name: &str) -> Result<gst::Element> {
    gst::ElementFactory::make(name)
        .build()
        .map_err(|_| anyhow!("no {name} element; a GStreamer package is missing"))
}

/// The head camera as a `v4l2src`, with the one adjustment this driver needs.
///
/// `v4l2src` rather than a hand-written V4L2 loop. The case for our own capture was that this
/// driver drops every third frame, and that raw bytes through `fdsrc` need
/// `rawvideoparse blocksize=…`, which is silently wrong the moment stride padding appears. Both
/// belong to the *subprocess* shape: `v4l2src` attaches a `GstVideoMeta` describing the real
/// layout, and the frame loss has a cause with a small fix — see [`raise_capture_buffers`].
fn camera_source(camera: &Camera, fps: u32) -> Result<gst::Element> {
    pin_sensor_mode(fps)?;

    // Exposure and gain go through `extra-controls` rather than a `v4l2-ctl` call, so they are
    // applied by whoever opens the device — including after a re-open we did not initiate.
    let controls = gst::Structure::builder("c")
        .field("exposure", camera.exposure as i32)
        .field("analogue_gain", camera.analogue_gain as i32)
        .build();

    let src = gst::ElementFactory::make("v4l2src")
        .property("device", &camera.device)
        .property("extra-controls", &controls)
        .build()
        .map_err(|_| {
            anyhow!(
                "no v4l2src element; it comes from gstreamer1.0-plugins-good, which \
                 setup-gstreamer.sh installs"
            )
        })?;

    raise_capture_buffers(&src)?;

    tracing::info!(
        device = %camera.device,
        exposure = camera.exposure,
        analogue_gain = camera.analogue_gain,
        "head camera"
    );
    Ok(src)
}

/// What the tee carries, and what both branches therefore see.
///
/// Single-plane on purpose: `v4l2src` cannot drive rkisp's two-plane `NM12` at full rate, and
/// asking for GStreamer `NV12` is what selects it. The table in [`start`] has the numbers.
pub const CAPTURE_FORMAT: &str = "UYVY";

/// How many capture buffers to ask for. Three is the cliff; four leaves one spare.
///
/// Measured with `v4l2-ctl --stream-mmap=N`, 300 frames of 1280x720 NV12 off rkisp:
///
/// | buffers | 2 | 3 | 4 | 6 |
/// |---|---|---|---|---|
/// | seconds | 15.2 | 10.3 | 10.3 | 10.3 |
///
/// 19.7 fps against 29.2 from a 30 fps sensor, and `v4l2src` lands on two.
const CAPTURE_BUFFERS: u32 = 4;

/// Get `v4l2src` off two capture buffers, which costs a third of the frames.
///
/// `gst_v4l2_object_decide_allocation` computes the pool depth three different ways, and only one
/// of them is enough:
///
/// ```text
/// can_share_own_pool = (has_video_meta || !obj->need_video_meta);
/// ...
/// if (pushing_from_our_pool) {
///     own_min = min + obj->min_buffers + 2;
///     if (!update) own_min += 2;              /* `update` == the query carried a pool */
/// } else {
///     own_min = MAX (obj->min_buffers + 1, GST_V4L2_MIN_BUFFERS (obj));
/// }
/// ```
///
/// rkisp implements neither `V4L2_CID_MIN_BUFFERS_FOR_CAPTURE` (so `obj->min_buffers` is 0) nor a
/// contiguous `NV12` — it offers the two-plane `NM12`, which `v4l2src` prefers and which only a
/// `GstVideoMeta` can describe. Measured on the board, 300 frames of 720p:
///
/// | chain | `own_min` | fps |
/// |---|---|---|
/// | `UYVY ! queue ! fakesink` | `0 + 0 + 2 + 2` | 29.3 |
/// | `UYVY ! videoconvert ! fakesink` | `0 + 0 + 2` | 19.7 |
/// | `UYVY ! mpph264enc` | `0 + 0 + 2` | 19.7 |
/// | `NV12 ! fakesink` | else branch, `MAX(1, 2)` | 19.7 |
///
/// Three is the cliff, so two costs a third of every second. Two things are therefore needed:
///
/// 1. **The meta**, or `can_share_own_pool` is false and the else branch ignores everything the
///    query says. That also means a copy of every frame into a generic pool.
/// 2. **A first pool whose `min` is not zero**, because any downstream element that proposes a
///    pool sets `update` and forfeits the `+ 2`. `GstVideoEncoder::propose_allocation` proposes
///    exactly that — a pool with `min = 0` — so `mpph264enc` downstream is enough to do it.
///
/// **And (2) has to happen after downstream answers.** `propose_allocation` implementations
/// overwrite pool 0 rather than appending, so a `min` written on the way out is replaced by the
/// encoder's zero on the way back. A pad probe fires in both directions, so this rewrites pool 0
/// every time it sees the query and the last word is ours. That is the bug that made three
/// earlier versions of this function look like they were being ignored.
fn raise_capture_buffers(src: &gst::Element) -> Result<()> {
    let pad = src
        .static_pad("src")
        .context("v4l2src has no src pad, which cannot happen")?;

    // The query passes here twice — once outbound, once with downstream's answer — and the second
    // pass is the one that matters. Logged for the first few, because "our rewrite was overwritten"
    // and "our rewrite stuck and was ignored" are different bugs and the frame rate cannot tell
    // them apart. Four passes is two negotiations' worth.
    let passes = std::sync::atomic::AtomicU32::new(0);

    pad.add_probe(gst::PadProbeType::QUERY_DOWNSTREAM, move |_, info| {
        if let Some(gst::PadProbeData::Query(query)) = info.data.as_mut()
            && let gst::QueryViewMut::Allocation(allocation) = query.view_mut()
        {
            let pass = passes.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let before: Vec<(u32, u32, u32)> = allocation
                .allocation_pools()
                .map(|(_, size, min, max)| (size, min, max))
                .collect();
            let meta_before = allocation
                .find_allocation_meta::<gst_video::VideoMeta>()
                .is_some();

            if allocation
                .find_allocation_meta::<gst_video::VideoMeta>()
                .is_none()
            {
                allocation.add_allocation_meta::<gst_video::VideoMeta>(None);
            }

            match allocation.allocation_pools().next() {
                // Size 0 is fine: `decide_allocation` overwrites it with the driver's own frame
                // size in every io-mode this can reach. Max 0 means unlimited.
                None => {
                    allocation.add_allocation_pool(None::<&gst::BufferPool>, 0, CAPTURE_BUFFERS, 0)
                }
                Some((pool, size, min, max)) if min < CAPTURE_BUFFERS => {
                    allocation.set_nth_allocation_pool(
                        0,
                        pool.as_ref(),
                        size,
                        CAPTURE_BUFFERS,
                        max,
                    );
                }
                Some(_) => {}
            }

            if pass < 4 {
                let after: Vec<(u32, u32, u32)> = allocation
                    .allocation_pools()
                    .map(|(_, size, min, max)| (size, min, max))
                    .collect();
                tracing::info!(
                    pass,
                    meta_before,
                    pools_before = ?before,
                    pools_after = ?after,
                    "capture allocation query"
                );
            }
        }
        gst::PadProbeReturn::Ok
    })
    .context("could not add the allocation probe to v4l2src")?;
    Ok(())
}

/// Switch the IMX219 out of its boot mode, which caps capture at 21 fps.
///
/// The sensor boots in 3280x2464 and the rkisp scaler will happily give us 1280x720 from it — at
/// the full-res frame rate. 1920x1080 is the mode that runs at 30, and the ISP scales down from
/// there, so nothing else in the pipeline changes with it.
///
/// This shells out to `media-ctl` once at startup, because the switch is a subdev ioctl on an
/// entity whose name embeds its I2C bus and address (`m00_b_imx219 2-0010`) and therefore has to
/// be discovered from the topology rather than named. Doing it here rather than in the unit means
/// a run with `[media] camera` off needs no camera at all.
fn pin_sensor_mode(fps: u32) -> Result<()> {
    let (media, entity) = find_sensor()?;

    let format = format!("\"{entity}\":0[fmt:SRGGB10_1X10/1920x1080]");
    let output = std::process::Command::new("media-ctl")
        .args(["-d", &media, "--set-v4l2", &format])
        .output()
        .context("could not run media-ctl; it comes from v4l-utils")?;

    if !output.status.success() {
        // Not fatal: capture still works, just slower. Said loudly because a third of the frames
        // going missing looks like a network problem from the far end.
        tracing::warn!(
            %media, %entity,
            why = %String::from_utf8_lossy(&output.stderr).trim(),
            "media-ctl would not set the 1920x1080 sensor mode — capture stays in the boot \
             mode, which caps it at 21 fps"
        );
    } else {
        tracing::info!(%media, %entity, target_fps = fps, "sensor mode 1920x1080");
    }
    Ok(())
}

/// The media device and entity name of the IMX219, from the topology.
///
/// Matched on a substring rather than a fixed name: the entity is `m00_b_imx219 2-0010`, which
/// embeds the I2C bus and address, and those move with the overlay.
///
/// **Every way this fails says which one it was.** An earlier version returned `Option` and
/// reported "no imx219 entity" for all of them, which sent the first real run chasing the
/// overlay when the actual cause was `media-ctl` being denied `/dev/media0`. The three cases want
/// three different fixes and look identical from the outside.
fn find_sensor() -> Result<(String, String)> {
    let mut nodes = 0;
    let mut failures = Vec::new();

    for index in 0..8 {
        let media = format!("/dev/media{index}");
        if !std::path::Path::new(&media).exists() {
            continue;
        }
        nodes += 1;

        let output = match std::process::Command::new("media-ctl")
            .args(["-d", &media, "-p"])
            .output()
        {
            Ok(output) => output,
            Err(err) => {
                failures.push(format!("{media}: cannot run media-ctl ({err})"));
                continue;
            }
        };
        if !output.status.success() {
            let why = String::from_utf8_lossy(&output.stderr);
            failures.push(format!("{media}: {}", why.trim()));
            continue;
        }

        for line in String::from_utf8_lossy(&output.stdout).lines() {
            // "- entity 76: m00_b_imx219 2-0010 (1 pad, 1 link, 0 routes)"
            let line = line.trim_start();
            if !line.starts_with("- entity") || !line.contains("imx219") {
                continue;
            }
            let Some((_, rest)) = line.split_once(": ") else {
                continue;
            };
            let name = rest.split(" (").next().unwrap_or(rest).trim();
            if !name.is_empty() {
                return Ok((media, name.to_string()));
            }
        }
    }

    if nodes == 0 {
        bail!(
            "no /dev/media* at all, so no camera is attached as far as the kernel is concerned.\n  \
             The overlay is enabled by setup-board.sh's configure_camera and needs a reboot; \
             Armbian ships it unprefixed while the board sets overlay_prefix=rk3568, so a boot \
             with no camera and no complaint is the expected shape of that bug."
        );
    }
    if !failures.is_empty() {
        bail!(
            "found {nodes} media device(s) and could not read the topology of any:\n  {}\n  \
             /dev/media* is root:video, so this is what running outside the `video` group looks \
             like. The unit grants it with SupplementaryGroups=, which `sudo -u` does not apply — \
             use `systemctl` or `systemd-run -p SupplementaryGroups=video`.",
            failures.join("\n  ")
        );
    }
    bail!(
        "read {nodes} media device(s) and none has an imx219 entity. The overlay loaded something, \
         so DUCK_CAMERA_OVERLAY may name the wrong module for this camera."
    )
}

/// Tell `webrtcsink` whether to adapt the send rate to the link, and by what.
///
/// **Set rather than inherited.** `gcc` is the element's own default, so naming it changes nothing
/// today — which is the point: what a plugin we ship from a pinned release defaults to is not a
/// decision this robot should discover it inherited on the day upstream changes it.
///
/// **And set defensively, because this is the one value in this function that comes from a config
/// file rather than a literal.** `set_property_from_str` panics both on a property the element
/// lacks and on a nickname its enum does not know, and a panic here is a daemon that will not
/// start — costing the video *and* the control channel to gain a setting. So the property is
/// looked up and the nickname resolved through the enum's own class; either failing leaves the
/// element on its default and says so, which is a far better failure than no robot at all. Same
/// reasoning as the `meta` property above.
fn set_congestion_control(sink: &gst::Element, mode: robotd_params::CongestionControl) {
    let Some(pspec) = sink.find_property("congestion-control") else {
        tracing::warn!(
            "webrtcsink has no congestion-control property on these plugins, so the send rate \
             adapts however this build defaults. Everything else is unaffected."
        );
        return;
    };
    let Some(value) = glib::EnumClass::with_type(pspec.value_type())
        .and_then(|class| class.to_value_by_nick(mode.nick()))
    else {
        tracing::warn!(
            nick = mode.nick(),
            "webrtcsink's congestion-control has no such value on these plugins; leaving its own \
             default"
        );
        return;
    };
    sink.set_property_from_value("congestion-control", &value);
    tracing::info!(congestion_control = mode.nick(), "send rate");
}

/// Configure each encoder `webrtcsink` builds, before it runs.
///
/// `webrtcsink` emits `encoder-setup` once per encoder — per consumer, plus one for the discovery
/// pass it uses to work out caps — with the element in hand. It is the only place these can be set
/// now that it owns the encoder rather than us.
///
/// Both settings are measured, and both fail in ways that do not look like encoder settings:
///
/// - **`profile=baseline`** produces a stream `h264parse` reports as `constrained-baseline`, which
///   is WebRTC's interoperable floor (`profile-level-id 42e01f`). The default is High: current
///   browsers negotiate it, older peers do not.
/// - **`header-mode=each-idr`** repeats SPS/PPS on every IDR. The default puts them in the first
///   frame only, so a peer that joins late — or loses that one packet — never decodes anything.
///
/// Returns `false`, so `webrtcsink` still applies its own configuration on top: it owns the
/// bitrate now, and congestion control moving it is the reason for this whole arrangement.
fn wire_encoder_setup(sink: &gst::Element) -> Result<()> {
    if glib::subclass::signal::SignalId::lookup("encoder-setup", sink.type_()).is_none() {
        return Err(anyhow!(
            "webrtcsink has no encoder-setup signal; without it the encoder cannot be configured \
             and the stream would be High profile with SPS/PPS only in its first frame"
        ));
    }

    sink.connect("encoder-setup", false, move |values| {
        // (webrtcsink, consumer_id, stream_name, encoder).
        let Some(encoder) = values.get(3).and_then(|v| v.get::<gst::Element>().ok()) else {
            tracing::warn!(
                arity = values.len(),
                "encoder-setup did not carry an encoder; it will run unconfigured"
            );
            return Some(false.to_value());
        };
        let name = encoder
            .factory()
            .map(|f| f.name().to_string())
            .unwrap_or_default();

        // `"discovery"` for the startup pass in which `webrtcsink` builds one encoder per codec it
        // could offer, purely to learn its caps. A real peer id otherwise.
        let consumer = values
            .get(1)
            .and_then(|v| v.get::<String>().ok())
            .unwrap_or_default();
        let discovering = consumer == "discovery";

        // Only `mpph264enc` has these properties, and setting a property an element lacks panics —
        // which in a signal handler aborts. So this is keyed on the factory rather than attempted
        // hopefully.
        if name == "mpph264enc" {
            encoder.set_property_from_str("profile", "baseline");
            encoder.set_property_from_str("header-mode", "each-idr");
            if !discovering {
                tracing::info!(encoder = %name, %consumer, "hardware H.264, configured for WebRTC");
            }
        } else if !discovering {
            // Only meaningful for a real consumer. During discovery this fires once per codec —
            // including `mppvp8enc` and `mpph265enc`, which are *hardware* — so warning there
            // called two VPU encoders software on every startup, crying wolf about the one thing
            // it exists to catch.
            //
            // For a real peer it is worth saying loudly: `video-caps` restricts the offer to
            // H.264, so anything else arriving here means that restriction stopped working, and
            // something is encoding on the cores `robotd`'s control loop shares.
            tracing::warn!(
                encoder = %name, %consumer,
                "a consumer negotiated something other than hardware H.264"
            );
        }
        Some(false.to_value())
    });
    Ok(())
}

/// Live count of what the consumers see, so [`meter_capture_rate`] can report it.
///
/// An `AtomicU32` rather than a lock: it is written from `consumer-added`/`consumer-removed` on
/// GStreamer threads and read from the capture probe on another, and neither may block the other.
type Consumers = Arc<std::sync::atomic::AtomicU32>;

/// Count frames where they enter the pipeline, not where they leave it, and publish what we see.
///
/// **Placement is the whole point.** This lived on the tee's raw branch first, which sits behind a
/// deliberately leaky one-buffer queue — so it measured what survived that queue. The other rates
/// available are misleading in the same direction: `rkvenc` interrupts count what the *encoder*
/// consumed (behind `webrtcsink`'s own queue and its `videorate drop-only`), and `v4l2src`'s
/// `lost frames detected` warning counts gaps in the driver's sequence numbers, which stays silent
/// when the source is merely slow. Every wrong turn in this bring-up came from one of those.
///
/// On the pad *before* the tee there is nothing between here and the driver.
///
/// **Driver-level drops come from the buffer offset**, where `v4l2src` leaves the V4L2 sequence
/// number. A gap there is a frame the driver captured and we never got, which is the number worth
/// reporting — as opposed to what our own leaky queue discards, which is a choice.
fn meter_capture_rate(
    pad: &gst::Pad,
    width: u32,
    height: u32,
    fps: u32,
    consumers: Consumers,
) -> Result<()> {
    let target = fps as f64;
    struct Meter {
        window_frames: u64,
        window_start: std::time::Instant,
        frames: u64,
        dropped: u64,
        last_offset: Option<u64>,
        healthy: Option<bool>,
    }
    let meter = Mutex::new(Meter {
        window_frames: 0,
        window_start: std::time::Instant::now(),
        frames: 0,
        dropped: 0,
        last_offset: None,
        healthy: None,
    });

    pad.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        // Must not panic: this is a GStreamer thread, and a panic crossing the C closure boundary
        // aborts the process rather than unwinding.
        let offset = match info.data {
            Some(gst::PadProbeData::Buffer(ref buffer)) => buffer.offset(),
            _ => gst::ClockTime::NONE.map_or(u64::MAX, |_| u64::MAX),
        };

        if let Ok(mut meter) = meter.lock() {
            meter.frames += 1;
            meter.window_frames += 1;

            // `u64::MAX` is `GST_BUFFER_OFFSET_NONE`, which is what a source that does not set one
            // leaves behind — `videotestsrc`, for instance. No offset, no gap detection.
            if offset != u64::MAX {
                if let Some(last) = meter.last_offset
                    && offset > last + 1
                {
                    meter.dropped += offset - last - 1;
                }
                meter.last_offset = Some(offset);
            }

            let elapsed = meter.window_start.elapsed();
            if elapsed >= std::time::Duration::from_secs(1) {
                let measured = meter.window_frames as f64 / elapsed.as_secs_f64();
                meter.window_frames = 0;
                meter.window_start = std::time::Instant::now();

                let stats = proto::CameraStats {
                    fps: (measured * 10.0).round() / 10.0,
                    target_fps: fps,
                    width,
                    height,
                    format: CAPTURE_FORMAT.to_owned(),
                    frames: meter.frames,
                    dropped: meter.dropped,
                    consumers: consumers.load(std::sync::atomic::Ordering::Relaxed),
                };
                // Ignored on purpose: a robot that cannot describe its camera still has one, and
                // this runs every second — a warning here would be a warning every second.
                let _ = proto::publish_camera_stats(&stats);

                // 90% rather than equality: a sensor's clock is not the CPU's, and a frame landing
                // either side of a window boundary is not a fault. Logged only on a crossing,
                // because a line a second forever gets grepped out.
                let healthy = measured >= target * 0.9;
                if meter.healthy.is_none_or(|previous| previous != healthy) {
                    if healthy {
                        tracing::info!(fps = %format!("{measured:.1}"), target = fps, "capture rate");
                    } else {
                        tracing::warn!(
                            fps = %format!("{measured:.1}"), target = fps,
                            dropped = meter.dropped,
                            "capture is below its target rate"
                        );
                    }
                    meter.healthy = Some(healthy);
                }
            }
        }
        gst::PadProbeReturn::Ok
    })
    .context("could not add the capture-rate probe")?;
    Ok(())
}

/// Give every consumer a `control` datachannel, and hand its ends to the caller.
///
/// The robot creates the channel rather than waiting for the peer to, which is what
/// `reachy_mini`'s working equivalent does. It means a peer that connects and creates nothing
/// still gets a control surface.
fn wire_consumers(
    sink: &gst::Element,
    channels: mpsc::Sender<Channel>,
    runtime: tokio::runtime::Handle,
    consumers: Consumers,
) -> Result<()> {
    // Counted here rather than inferred from the log, so `robotctl health` can say whether anyone
    // is actually watching. `consumer-removed` is guarded the same way `consumer-added` is: a
    // signal that has moved upstream should degrade the count, not abort the daemon.
    if glib::subclass::signal::SignalId::lookup("consumer-removed", sink.type_()).is_some() {
        let leaving = consumers.clone();
        sink.connect("consumer-removed", false, move |_| {
            // `fetch_update` rather than `fetch_sub`, so a spurious removal cannot wrap the count
            // around to four billion viewers.
            let _ = leaving.fetch_update(
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
                |current| Some(current.saturating_sub(1)),
            );
            None
        });
    } else {
        tracing::warn!(
            "webrtcsink has no consumer-removed signal; the consumer count will only ever rise"
        );
    }

    let arriving = consumers;
    let channels = Arc::new(channels);
    sink.connect("consumer-added", false, move |values| {
        arriving.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        // (webrtcsink, peer_id, webrtcbin). A signature change upstream shows up here as a
        // warning naming what arrived, rather than a panic in a signal handler.
        let Some(webrtcbin) = values.get(2).and_then(|v| v.get::<gst::Element>().ok()) else {
            tracing::warn!(
                arity = values.len(),
                "consumer-added did not carry a webrtcbin; cannot open a control channel"
            );
            return None;
        };
        let peer = values
            .get(1)
            .and_then(|v| v.get::<String>().ok())
            .unwrap_or_else(|| "?".into());

        match open_control_channel(&webrtcbin, &peer, &runtime) {
            Ok(channel) => {
                // A full queue means nobody is accepting sessions, which is a bug rather than
                // backpressure — say so instead of blocking a GStreamer signal handler.
                if channels.try_send(channel).is_err() {
                    tracing::error!(peer, "no room for another control channel");
                }
            }
            Err(e) => tracing::error!(peer, error = %e, "could not open a control channel"),
        }
        None
    });
    Ok(())
}

/// Create the `control` datachannel on one peer's `webrtcbin` and bridge it to channels.
fn open_control_channel(
    webrtcbin: &gst::Element,
    peer: &str,
    runtime: &tokio::runtime::Handle,
) -> Result<Channel> {
    // `emit_by_name` panics when the signal is absent or its signature differs — and a panic here
    // aborts the process rather than unwinding, because this runs inside a C closure. Checked
    // first so an upstream change becomes a logged refusal to open a control channel, with the
    // video track still working.
    for signal in ["create-data-channel"] {
        if glib::subclass::signal::SignalId::lookup(signal, webrtcbin.type_()).is_none() {
            return Err(anyhow!(
                "webrtcbin has no {signal} signal; gst-plugins-rs may have changed it"
            ));
        }
    }
    // Reliable and ordered, which is the default and is what §2 wants for `control` —
    // `remote-webrtc.md` §6 covers why the first version opens only this one.
    // Typed as `WebRTCDataChannel` rather than `glib::Object`, and that is load-bearing rather
    // than tidy: a `GstObject` is `Send`, a bare `glib::Object` is not, so the writer task below
    // does not compile against the untyped form.
    let channel = webrtcbin
        .emit_by_name::<Option<gst_webrtc::WebRTCDataChannel>>(
            "create-data-channel",
            &[&"control", &None::<gst::Structure>],
        )
        .ok_or_else(|| anyhow!("webrtcbin returned no data channel"))?;

    // Same reasoning for the channel's own signals: `connect` and `emit_by_name` both panic when a
    // name is absent, and both run where a panic aborts. Checked together so the failure is one
    // clear message rather than whichever fires first.
    for signal in ["on-message-string", "send-string"] {
        if glib::subclass::signal::SignalId::lookup(signal, channel.type_()).is_none() {
            return Err(anyhow!(
                "the data channel has no {signal} signal; gst-plugins-rs may have changed it"
            ));
        }
    }

    let (inbound_tx, inbound) = mpsc::channel::<String>(64);
    let (outbound, mut outbound_rx) = mpsc::channel::<String>(64);

    let peer_label = peer.to_owned();
    channel.connect("on-message-string", false, move |values| {
        if let Some(line) = values.get(1).and_then(|v| v.get::<String>().ok()) {
            // Dropping a control frame is bad, but blocking a GStreamer signal handler is worse:
            // it would stall the whole pipeline, media included.
            if inbound_tx.try_send(line).is_err() {
                tracing::warn!(peer = %peer_label, "dropped a control frame; the session is behind");
            }
        }
        None
    });

    // The writer half. `send-string` is called from this task rather than from the session, so
    // nothing in the session has to know about GStreamer.
    let writer = channel.clone();
    let peer_label = peer.to_owned();
    runtime.spawn(async move {
        while let Some(line) = outbound_rx.recv().await {
            writer.emit_by_name::<()>("send-string", &[&line]);
        }
        tracing::debug!(peer = %peer_label, "control channel writer ended");
    });

    tracing::info!(peer, "control channel open");
    Ok(Channel { inbound, outbound })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame whose every byte is `tag`, so a test can say *which* capture came back.
    fn frame(tag: u8) -> Frame {
        Frame {
            width: 4,
            height: 2,
            format: CAPTURE_FORMAT,
            data: vec![tag; 16],
        }
    }

    /// **The point of the whole rendezvous.** With nobody waiting, the callback's answer is no,
    /// and no is what stops 1.84 MB being copied thirty times a second for readers that want two.
    ///
    /// A regression here is silent: the picture is identical, the detector still detects, and the
    /// only symptom is a robot that runs hotter than it needs to.
    #[test]
    fn a_frame_nobody_asked_for_is_not_captured() {
        let frames = Frames::default();
        assert!(!frames.take_request());
        assert!(!frames.take_request());
    }

    /// One capture answers every reader waiting for one. The request is a bit rather than a queue,
    /// so the exposure loop and the detector asking in the same frame period cost one copy.
    #[test]
    fn two_readers_asking_at_once_cost_one_capture() {
        let frames = Frames::default();
        frames
            .0
            .wanted
            .store(true, std::sync::atomic::Ordering::Relaxed);
        frames
            .0
            .wanted
            .store(true, std::sync::atomic::Ordering::Relaxed);

        assert!(frames.take_request());
        assert!(
            !frames.take_request(),
            "the second frame must not be captured too"
        );
    }

    /// **A reader gets the frame captured after it asked, not the one already lying there.**
    ///
    /// This is what makes demand-driven capture safe for the exposure loop. Handing back the last
    /// delivery would be cheaper still and would mean metering a frame from the previous tick — up
    /// to half a second of dead reckoning on a control loop whose whole job is to track the light
    /// in the room.
    #[test]
    fn a_reader_gets_the_capture_that_answered_it() {
        let frames = Frames::default();
        // What a previous reader's request left behind.
        frames.deliver(frame(1));

        let reader = {
            let frames = frames.clone();
            std::thread::spawn(move || frames.next_frame())
        };

        // Stand in for the appsink: wait for the request to show up, then answer it.
        let deadline = std::time::Instant::now() + FRAME_TIMEOUT;
        while !frames.take_request() {
            assert!(
                std::time::Instant::now() < deadline,
                "the reader never asked for a frame"
            );
            std::thread::yield_now();
        }
        frames.deliver(frame(2));

        let got = reader.join().expect("reader thread").expect("a frame");
        assert_eq!(
            got.data,
            vec![2; 16],
            "got the stale frame, not the fresh one"
        );
    }

    /// A camera that has stopped times out rather than handing back the frame it stopped on.
    ///
    /// The reader cannot tell a stale frame from a current one, so the staleness has to be the
    /// answer. Both readers already have a "no frame" path — this is what reaches it.
    #[test]
    fn a_capture_that_never_comes_is_not_a_stale_frame() {
        let frames = Frames::default();
        frames.deliver(frame(1));

        let waited = std::time::Instant::now();
        assert!(
            frames.next_frame().is_none(),
            "a timeout must not be answered with the last frame"
        );
        assert!(
            waited.elapsed() >= FRAME_TIMEOUT,
            "it gave up before the timeout was out"
        );
    }

    /// A quarter turn swaps the frame's axes; a half turn does not.
    ///
    /// [`Frame`] carries the dimensions its buffer is in, and the raw branch is handed these
    /// rather than reading them back off the caps. Get this wrong and a consumer reads a 720x1280
    /// picture as 1280x720 — which is not a failure, it is a diagonal smear, and the kind of thing
    /// that gets blamed on the camera.
    #[test]
    fn a_quarter_turn_swaps_the_frame_size() {
        assert_eq!(Rotation::None.output(1280, 720), (1280, 720));
        assert_eq!(Rotation::Cw90.output(1280, 720), (720, 1280));
        assert_eq!(Rotation::Cw180.output(1280, 720), (1280, 720));
        assert_eq!(Rotation::Cw270.output(1280, 720), (720, 1280));
    }

    /// The mount is a quarter turn clockwise, and `90r` is GStreamer's name for that.
    ///
    /// Named the wrong way round, the picture is upside down twice over: the console's drag-to-look
    /// maps a gaze off the same geometry, so a 180° error there sends the robot looking away from
    /// where the operator pointed rather than merely showing a sideways picture.
    #[test]
    fn clockwise_is_90r_and_identity_is_nothing_at_all() {
        assert_eq!(Rotation::Cw90.video_direction(), Some("90r"));
        assert_eq!(Rotation::Cw270.video_direction(), Some("90l"));
        assert_eq!(Rotation::Cw180.video_direction(), Some("180"));
        // Not `Some("identity")`: no element is built at all, so the pass costs nothing.
        assert_eq!(Rotation::None.video_direction(), None);
    }

    /// Only the four right angles, and a wrong one is refused rather than rounded.
    #[test]
    fn only_right_angles_are_accepted() {
        for (degrees, expected) in [
            (0, Rotation::None),
            (90, Rotation::Cw90),
            (180, Rotation::Cw180),
            (270, Rotation::Cw270),
        ] {
            assert_eq!(Rotation::from_degrees(degrees).unwrap(), expected);
        }
        for bad in [45, 89, 91, 360, 1] {
            let error = Rotation::from_degrees(bad).unwrap_err().to_string();
            assert!(error.contains("0, 90, 180 or 270"), "{error}");
        }
    }
}
