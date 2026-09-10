//! `mediad` — camera, mic, WebRTC, and the remote gateway.
//!
//! Runs the signalling server in this process, streams video to whoever connects, and gives each
//! peer a `control` datachannel that is a pipe to the robot API. `docs/design/remote-webrtc.md` is
//! the design.
//!
//! ## What it does not do
//!
//! **It does not authenticate.** Anyone who reaches the signalling port can drive the robot and
//! see its camera. That is a decision, not an omission — §4 has the reasoning, and the short
//! version is that the pairing PIN is a shared `000000`, so a gate would add a step to every
//! connection and prove nothing. The bridge that makes a robot reachable from outside the LAN
//! authenticates on both sides before a session arrives.
//!
//! **It is not on the recovery path.** If `mediad` will not start, the robot still walks, still
//! takes an update, and is still reachable over Bluetooth. That is why it may depend on a plugin
//! from a release asset and a device node's group while `updaterd` may not.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "Camera, mic, WebRTC — and the remote gateway", version)]
struct Args {
    /// Where to bind the signalling server.
    ///
    /// All interfaces by default, and that is the point: loopback-only would mean a peer on the
    /// LAN cannot reach it at all and every session would have to go through a bridge, which
    /// defeats having a local mode. See `remote-webrtc.md` §3.
    #[arg(long, default_value = "0.0.0.0")]
    host: String,

    /// The signalling server's port. 8443 is what `webrtcsink`'s own signaller defaults to, so a
    /// client built against it needs no argument.
    #[arg(long, default_value_t = 8443)]
    port: u32,

    /// The rendezvous service this robot registers with, so it can be reached from off its LAN.
    ///
    /// Defaults to the Space the mini's fleet uses. A flag rather than a config key because there
    /// is nothing to choose on a real robot — what it is for is pointing a board at a fake, or at
    /// a self-hosted copy on the day somebody wants one. `docs/design/remote-access-design.md` §4.
    #[arg(long, default_value = mediad::relay::DEFAULT_RENDEZVOUS)]
    rendezvous_url: String,

    /// The account credential `updaterd` writes, which the relay needs to prove whose robot this
    /// is. Absent means nobody has signed this robot in, and remote access is simply off.
    #[arg(long, default_value = mediad::relay::DEFAULT_TOKEN_PATH)]
    token: PathBuf,

    /// Where short-lived TURN credentials come from.
    ///
    /// Hugging Face hosts this proxy and mints Cloudflare credentials for the account the token
    /// belongs to, which is why offering a relay needs no new secret on the robot. A flag for
    /// pointing a board at a fake; there is nothing to choose on a real one.
    #[arg(long, default_value = mediad::turn::DEFAULT_TURN_ENDPOINT)]
    turn_url: String,

    /// Do not register with the rendezvous service, whatever the token file says.
    ///
    /// For a board that is signed in and being worked on: a duck registering from a bench while
    /// somebody drives the same account's robot elsewhere is a producer in a list nobody wants,
    /// and evicting it means finding this flag afterwards.
    #[arg(long)]
    no_remote: bool,

    /// Where the console is served. `http://<robot>:8080/`, and nothing else to run.
    ///
    /// **Two ports, and only this one is ever typed.** `webrtcsink` owns the listener on `--port`
    /// and takes only a host and a port about it, so the page cannot be a route on it — see
    /// `webrtc-console.md` §1.3, which also says where this ends up: one port, our own signalling
    /// server, and a certificate, on the day a microphone or a browser gamepad is wanted.
    #[arg(long, default_value_t = 8080)]
    web_port: u16,

    /// Params file. Defaults to `/etc/robot/robotd.toml`, which may be absent — a board with no
    /// file streams its camera at the built-in defaults. A path given here must exist.
    ///
    /// **The same file `robotd` reads, and `[media]` is this daemon's section of it.** What the
    /// stream looks like — camera or test pattern, frame size, rate, bitrate — used to be flags
    /// on this unit's `ExecStart` line, which the release installer rewrites: changing one meant
    /// a systemd drop-in, and nobody reaches for a drop-in to answer "why is the video soft?".
    /// `robotctl configure` edits that file, so it now edits this.
    #[arg(long)]
    config: Option<PathBuf>,

    /// Which capture node. rkisp exposes several; `video0` is the main path.
    #[arg(long, default_value = "/dev/video0")]
    camera_device: String,

    /// Sensor exposure in lines (~19 µs each) and analogue gain, where 256 is 1x.
    ///
    /// The starting values only: with the driver's boot values the picture is black rather than
    /// merely dark, so something must write the sensor before the first frame. On a board where
    /// `scripts/setup-rkaiq.sh` installed the 3A engine, it converges exposure from here; on one
    /// where it did not, these are what the camera keeps. The defaults are the prototype's.
    #[arg(long, default_value_t = 600)]
    exposure: u32,

    #[arg(long, default_value_t = 1024)]
    analogue_gain: u32,

    /// How far the camera is mounted from upright, clockwise: 0, 90, 180 or 270.
    ///
    /// **90 by default, because the head camera is mounted a quarter turn off**, and this is the one
    /// place that fact is written down. It no longer means "rotate the pixels": it is told to
    /// whoever displays the video, and they rotate for free — the console with a CSS transform on
    /// the GPU. Rotating here cost 145% of a core and 22 fps; `pipeline::Rotation` has the numbers.
    ///
    /// True of a simulated camera too: the one in MuJoCo is rolled to match the mount, so a frame
    /// from a duck in the twin needs the same quarter turn as a frame from a duck on the desk.
    #[arg(long)]
    rotate: Option<u32>,

    /// Leave the exposure where `--exposure` and `--analogue-gain` put it, instead of metering.
    ///
    /// **The software loop is on by default because the board's 3A engine only does this once.**
    /// `rkaiq_3A_server` owns white balance, gamma and noise reduction, and its AE converges the
    /// sensor at stream start and then stops responding — and skips even that on a boot where it
    /// missed the stream-start event, which is the "3A stopped working" shape. `mediad::exposure`
    /// is the loop. Turn it off for a fixed exposure: a calibration capture, or a board whose
    /// engine really does keep converging.
    #[arg(long)]
    no_auto_exposure: bool,

    /// Take frames from a duck in MuJoCo at `host:port` instead of a camera.
    ///
    /// The geometry has to match the simulator's camera — set `[media] quality` (the rung `mediad`
    /// streams; `mediad` has no `--width`/`--height` of its own) to the resolution and rate the body
    /// renders at — because the frames arrive raw and length-prefixed with no handshake, and a
    /// mismatch is a picture nobody can read rather than an error the pipeline can recover from.
    /// `mediad` says so and refuses the frame if the sizes disagree.
    /// Takes precedence over `[media] camera`, which is a fact about a robot and not about this.
    #[arg(long)]
    sim_camera: Option<String>,

    /// Rotate in the pipeline as well, so the *encoded stream* comes out upright.
    ///
    /// **Off by default because it is expensive in a way that does not look like rotation.** It
    /// breaks `mpph264enc`'s zero-copy path to the SoC's 2D engine, so MPP converts every frame in
    /// software: measured at 97 °C, the CPU throttled to 408 MHz and 8 fps out of a 30 fps camera.
    /// Worth it only for a consumer that cannot rotate for itself.
    #[arg(long)]
    flip_in_pipeline: bool,

    /// Where the daemons listen, when not at `proto::socket`'s paths.
    ///
    /// On a robot the defaults are right and none of these is ever typed. They exist for the twin
    /// (`scripts/duck-sim`), where every duck's `robotd` and `tofd` listen under a per-duck state
    /// directory — without them a peer's `control` channel reaches a `mediad` whose routes all end
    /// at `/run/*.sock`, and every call but `media.video` answers "not answering".
    #[arg(long)]
    robot_socket: Option<std::path::PathBuf>,
    #[arg(long)]
    tof_socket: Option<std::path::PathBuf>,
    #[arg(long)]
    config_socket: Option<std::path::PathBuf>,
    #[arg(long)]
    pad_socket: Option<std::path::PathBuf>,
    #[arg(long)]
    updater_socket: Option<std::path::PathBuf>,
}

// Gated with the `main` that calls it: off Linux there is no pipeline, so there is nothing to
// point at a socket and `-D warnings` would call this dead.
#[cfg(target_os = "linux")]
impl Args {
    fn sockets(&self) -> mediad::upstream::Sockets {
        let mut s = mediad::upstream::Sockets::default();
        if let Some(p) = &self.robot_socket {
            s.robot = p.clone();
        }
        if let Some(p) = &self.tof_socket {
            s.tof = p.clone();
        }
        if let Some(p) = &self.config_socket {
            s.config = p.clone();
        }
        if let Some(p) = &self.pad_socket {
            s.pad = p.clone();
        }
        if let Some(p) = &self.updater_socket {
            s.updater = p.clone();
        }
        s
    }
}

#[cfg(target_os = "linux")]
fn main() -> ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    // Before anything that can fail, so a journal that reports a startup failure also reports
    // which build failed. Every other daemon does this for the same reason.
    duck_ipc_proto::log_startup_identity!("mediad");

    let runtime = match tokio::runtime::Runtime::new() {
        Ok(runtime) => runtime,
        Err(e) => {
            tracing::error!(error = %e, "no tokio runtime");
            return ExitCode::FAILURE;
        }
    };

    // Refused before anything starts: a bad angle is a typo on a command line, and the daemon
    // should say so rather than opening a camera first.
    // Validated even when the pipeline will not use it, because it is still what every consumer is
    // told about the mount — a typo should not reach the console as a rotation nobody can apply.
    // 90 whatever the source. The head camera is mounted a quarter turn off and every consumer is
    // told so — and the *simulated* camera is rolled the same way on purpose, so that a frame from a
    // duck in MuJoCo needs the same turn as a frame from a duck on the desk. Overridable, because a
    // scene could mount it differently, but there is one default and it is the robot's.
    let rotate = args.rotate.unwrap_or(90);
    let mount = match mediad::pipeline::Rotation::from_degrees(rotate) {
        Ok(rotation) => rotation,
        Err(e) => {
            tracing::error!(error = %e, "mediad cannot start");
            return ExitCode::FAILURE;
        }
    };
    // What the stream is and what it looks for, from `[media]` and `[detect]` — see
    // `--config` and `mediad::config`. One file, one read: `[detect]` is `mediad`'s section
    // too, and a second config file for the second daemon that wants one is how a fleet ends
    // up with settings nobody can find.
    let explicit = args.config.is_some();
    let config = args
        .config
        .clone()
        .unwrap_or_else(mediad::config::default_path);
    let params = mediad::config::load(&config, explicit);
    let (media, detect) = (params.media, params.detect);
    tracing::info!(
        camera = media.camera,
        quality = media.quality.label(),
        width = media.quality.width(),
        height = media.quality.height(),
        fps = media.quality.fps(),
        bitrate = media.bitrate_resolved(),
        congestion_control = media.congestion_control.nick(),
        "streaming"
    );
    // The same angle the detector needs, in its own vocabulary: it folds the turn into the
    // resampling it already does, which is why nothing in the pipeline has to.
    let turn = match duck_detect::Turn::from_degrees(rotate) {
        Some(turn) => turn,
        None => {
            tracing::error!(degrees = rotate, "mediad cannot start");
            return ExitCode::FAILURE;
        }
    };

    let rotation = if args.flip_in_pipeline {
        tracing::warn!(
            degrees = rotate,
            "--flip-in-pipeline: rotating in the pipeline costs the encoder its zero-copy path"
        );
        mount
    } else {
        mediad::pipeline::Rotation::None
    };

    runtime.block_on(async move {
        // The console, before the pipeline: it is the page that says a robot's pipeline would not
        // start, so it should be up first — and it needs nothing from GStreamer.
        //
        // **A page that cannot be served does not cost the video.** A refused bind is almost always
        // a port already in use, which `Restart=always` cannot fix by trying again; a robot that
        // streams and answers control calls with no console is much better than one that does
        // neither. So this is logged at error and the daemon carries on.
        // Made here rather than beside the relay, because the console's listener now carries the
        // agent socket too and both want the same late-arriving media.
        let (video_tx, video_rx) =
            tokio::sync::watch::channel::<Option<mediad::session::Media>>(None);

        let page = mediad::web::page(args.port);
        let (web_host, web_port) = (args.host.clone(), args.web_port);
        let agent = mediad::agent::State {
            sockets: args.sockets(),
            video: video_rx.clone(),
        };
        tokio::spawn(async move {
            if let Err(e) = mediad::web::serve(&web_host, web_port, page, agent).await {
                tracing::error!(
                    error = %format!("{e:#}"),
                    "the console is not being served; video and control are unaffected"
                );
            }
        });

        // Before the pipeline, because `webrtcsink`'s `meta` is set as the element is built and a
        // producer that registered without a name would keep it until this daemon restarts. Costs a
        // unix-socket round trip on a boot where `configd` may not be up yet, which is why it is
        // bounded and why a failure is a warning rather than an exit.
        let sockets = args.sockets();
        let producer =
            mediad::producer::Producer::learn(sockets.clone(), duck_ipc_proto::build_info!()).await;
        tracing::info!(
            name = producer.name.as_deref().unwrap_or("unknown"),
            release = %producer.release,
            api_version = producer.api_version,
            "producing as"
        );

        // Relay candidates, so a consumer on a network that cannot punch a hole to this robot
        // still reaches it. Spawned whatever the account state — it is inert without a token and
        // starts on its own when a login lands — and *before* the pipeline, because the first
        // consumer's offer is built as the pipeline comes up.
        let relays = mediad::turn::Relays::empty();
        tokio::spawn(mediad::turn::maintain(
            std::sync::Arc::clone(&relays),
            args.token.clone(),
            args.turn_url.clone(),
        ));

        // What a control lane can say about this robot's own media: the picture's geometry, and
        // the frame streamer. Empty until the pipeline is up — `sensor_mode()` is only truthful
        // once something has tried to set it, and there are no frames to encode before then — and
        // the relay below is spawned before that on purpose, so the answer has to be able to
        // arrive late rather than be a value passed in now.
        // The outward half of remote access, and it is deliberately *after* the producer is
        // learned: the name a client sees in the service's listing comes from the same place the
        // local `meta` gets it, and a relay that registered first would publish an unnamed robot
        // until the next restart.
        //
        // Spawned whatever happens next. It is inert without a token, it holds no lock, and a
        // pipeline that fails to build should not take remote access down with it — a robot that
        // appears in its owner's list and cannot stream is still a robot somebody can reach to
        // find out why.
        if args.no_remote {
            tracing::info!("--no-remote: this robot will not register with the rendezvous service");
        } else {
            match mediad::relay::Meta::of(&producer, None) {
                None => tracing::warn!(
                    "no serial and no machine id, so this robot has no stable identity to \
                     register with; remote access is off"
                ),
                Some(meta) => {
                    if let Some(relay) =
                        mediad::relay::Relay::new(&args.rendezvous_url, &args.token, meta)
                    {
                        // The bridge is a *consumer* of the signalling server this same process
                        // runs, so it has to be told the port `--port` chose rather than assuming
                        // the default — a robot moved off 8443 would otherwise register happily
                        // and fail every session.
                        tokio::spawn(
                            relay
                                .with_local_signalling(format!("ws://127.0.0.1:{}", args.port))
                                .with_video(video_rx.clone())
                                .run(),
                        );
                    }
                }
            }
        }

        let source = if let Some(addr) = args.sim_camera.clone() {
            mediad::pipeline::Source::Sim(addr)
        } else if media.camera {
            mediad::pipeline::Source::Camera(mediad::pipeline::Camera {
                device: args.camera_device.clone(),
                exposure: args.exposure,
                analogue_gain: args.analogue_gain,
            })
        } else {
            mediad::pipeline::Source::Test
        };

        // Frame size and rate are still pinned rather than negotiated — both branches of the tee
        // depend on the answer, so a consumer that had to guess would get it wrong the first time
        // the source changed. What changed is only where the numbers come from: one named quality
        // in the config file rather than three flags nobody could set. `robotd_params::Quality`
        // says why the three move together.
        let settings = mediad::pipeline::Settings {
            host: args.host.clone(),
            port: args.port,
            bitrate: media.bitrate_resolved(),
            congestion_control: media.congestion_control,
            width: media.quality.width(),
            height: media.quality.height(),
            fps: media.quality.fps(),
            rotation,
        };

        // `frames` is the raw tap off the tee: the auto-exposure loop meters it, and the
        // `get_frame` surface in `architecture.md` §5.3 is what the rest of it is for. The branch
        // runs from the start rather than being added later, because a tee inserted into a live
        // pipeline is a different and much harder problem than a tee that was always there.
        let (_pipeline, mut channels, frames, stream_branch) = match mediad::pipeline::start(
            source.clone(),
            &producer,
            &settings,
            std::sync::Arc::clone(&relays),
        ) {
            Ok(started) => started,
            Err(e) => {
                // The message names which step failed and what usually causes it — a missing
                // plugin, a missing library, or a device node nobody can open. Those look
                // identical from a log line that only says "failed".
                tracing::error!(error = %format!("{e:#}"), "mediad cannot start");
                return ExitCode::FAILURE;
            }
        };

        // After the pipeline, because it meters the pipeline's own frames — and only with a real
        // camera, since a test pattern has no sensor to write and the loop would spend the daemon's
        // life reporting that it cannot.
        //
        // `_exposure` is the handle that stops the thread; it lives as long as this scope, which is
        // as long as the daemon.
        let _exposure = match (&source, args.no_auto_exposure) {
            (mediad::pipeline::Source::Camera(camera), false) => Some(mediad::exposure::spawn(
                camera.device.clone(),
                frames.clone(),
                camera.exposure,
                camera.analogue_gain,
            )),
            (mediad::pipeline::Source::Camera(_), true) => {
                tracing::info!(
                    exposure = args.exposure,
                    analogue_gain = args.analogue_gain,
                    "--no-auto-exposure: the picture stays at the starting exposure"
                );
                None
            }
            (mediad::pipeline::Source::Test, _) => None,
            // A simulated camera has no sensor to write, and its brightness is the renderer's.
            (mediad::pipeline::Source::Sim(_), _) => None,
        };

        // **The duck detector, from the same config file as everything else.** `[detect]` lives in
        // robotd.toml because that is the file `robotctl configure` edits and a robot has one place
        // for its switches — even though it is this daemon that reads that section.
        //
        // A detector that was asked for and cannot start is a warning, not a failure: the camera,
        // the console and the control channel are all still worth having, and "mediad refused to
        // boot because a model file moved" is a bad trade.
        let models = detect.models();
        let detector = if models.is_empty() {
            tracing::info!("duck detector off ([detect] enabled = false, or no model)");
            None
        } else {
            // The frames on the tee are as the camera took them — unless the pipeline was asked
            // to flip, in which case they are upright already and the sampler must not turn them
            // again.
            let sampler_turn = if args.flip_in_pipeline {
                duck_detect::Turn::None
            } else {
                turn
            };
            match mediad::detect::spawn_first(
                &models,
                frames.clone(),
                detect.hz,
                detect.threshold,
                sampler_turn,
            ) {
                Ok(detector) => Some(detector),
                Err(error) => {
                    tracing::warn!(
                        error = %format!("{error:#}"),
                        "the duck detector will not start; carrying on without it"
                    );
                    None
                }
            }
        };

        // What every peer is told about the picture. The geometry is the *encoded* frame — the
        // pipeline does not rotate, so it is the capture geometry — and the rotation is the mount.
        // The camera's geometry, for a consumer that has to turn pixels into directions. Read
        // *after* the pipeline is up, because which sensor mode is in force is only known once
        // something tried to set it — and a mode nobody knows the field of view of publishes
        // nothing rather than a plausible wrong number. `mediad::camera` has the arithmetic.
        let intrinsics = if args.sim_camera.is_some() {
            // The MuJoCo twin renders a known field of view, so publish its exact geometry — twin
            // recordings then self-describe (no `--calib` needed on the duckslam side).
            mediad::camera::Intrinsics::sim(media.quality.width(), media.quality.height())
        } else {
            mediad::camera::Intrinsics::published(
                media.intrinsics.as_ref(),
                mediad::pipeline::sensor_mode(),
                media.quality.width(),
                media.quality.height(),
            )
        };
        match &intrinsics {
            Some(geometry) => tracing::info!(
                fx = geometry.fx,
                fy = geometry.fy,
                cx = geometry.cx,
                cy = geometry.cy,
                calibrated = geometry.calibrated,
                "camera geometry"
            ),
            None => tracing::info!(
                "no camera geometry to publish: the sensor is not in a mode whose field of view \
                 is known, so a consumer is told nothing rather than something wrong"
            ),
        }

        let video = mediad::session::Video {
            width: media.quality.width(),
            height: media.quality.height(),
            rotate,
            intrinsics,
        };

        // Frames out to a WebSocket this robot dials, when something asks for them. Built here
        // because it needs the tee — and given the same `turn` the detector's sampler gets, for
        // the same reason: a pipeline that was asked to flip has already turned the frames, and
        // turning them twice is a picture on its side with nothing to say why.
        let streamer = std::sync::Arc::new(mediad::stream::Streamer::new(
            mediad::stream::Encoders {
                // The same `turn` the detector's sampler gets, and for the same reason: a pipeline
                // asked to flip has already turned the frames, and turning them twice is a
                // picture on its side with nothing to say why.
                jpeg: mediad::stream::jpeg_encoder(
                    frames.clone(),
                    if args.flip_in_pipeline {
                        duck_detect::Turn::None
                    } else {
                        turn
                    },
                ),
                // The H.264 branch turns nothing: it is downstream of the same tee, so the flip —
                // or its absence — is already in the pixels it encodes.
                h264: stream_branch.clone().map(mediad::stream::h264_encoder),
                gate: stream_branch.clone().map(|branch| {
                    std::sync::Arc::new(move |open: bool| {
                        if open {
                            branch.open();
                        } else {
                            branch.close();
                        }
                    }) as std::sync::Arc<dyn Fn(bool) + Send + Sync>
                }),
            },
            producer.clone(),
            // The resolved mount angle, not the flag: `--rotate` is an `Option` now and the
            // default lives in one place at the top of `main`.
            rotate,
            &args.token,
        ));

        let media = mediad::session::Media {
            video: video.clone(),
            streamer: std::sync::Arc::clone(&streamer),
        };

        // The relay has been up since before the pipeline; this is the point its control lanes can
        // start answering for the robot's own media.
        let _ = video_tx.send(Some(media.clone()));

        // One session per peer, each with its own connections to the services it talks to. Per
        // peer rather than shared, so one peer's minutes-long update cannot silence another's
        // telemetry — which is the same reason a session keeps one connection per lane.
        while let Some(channel) = channels.recv().await {
            let (replies_tx, mut replies_rx) = tokio::sync::mpsc::channel::<String>(256);
            let pool = mediad::upstream::Pool::new(sockets.clone(), replies_tx);

            let to_peer = channel.outbound.clone();
            tokio::spawn(async move {
                while let Some(line) = replies_rx.recv().await {
                    if to_peer.send(line).await.is_err() {
                        break;
                    }
                }
            });
            // Pushed as a courtesy for a client that only listens — and it may well arrive before
            // the peer's datachannel is open, in which case it is dropped. The console *asks*
            // (`media.video`), which is why that path exists and this one is best-effort.
            {
                let to_peer = channel.outbound.clone();
                let line = mediad::session::video_notification(&video);
                tokio::spawn(async move {
                    let _ = to_peer.send(line).await;
                });
            }

            // Detections go to the peer as notifications, on the same channel the console already
            // reads `robot.state` from — no polling, and one subscription per peer so a slow
            // consumer cannot hold up the detector.
            if let Some(detector) = detector.as_ref() {
                let mut sightings = detector.sightings.subscribe();
                let to_peer = channel.outbound.clone();
                tokio::spawn(async move {
                    loop {
                        match sightings.recv().await {
                            Ok(sighting) => {
                                if to_peer
                                    .send(mediad::detect::notification(&sighting))
                                    .await
                                    .is_err()
                                {
                                    break;
                                }
                            }
                            // Lagged: the peer is slower than the detector, and only the newest
                            // sighting is worth having. Skipping is what the bounded channel is for.
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                        }
                    }
                });
            }

            tokio::spawn(mediad::session::run(
                channel.inbound,
                channel.outbound,
                pool,
                // Cloned per session: it carries the camera's intrinsics and a handle to the
                // frame streamer. Always `Some` here — a datachannel exists because the pipeline
                // handed over a consumer, so by definition there is media behind it.
                Some(media.clone()),
            ));
        }

        // The pipeline outlived its consumers, which means `webrtcsink` stopped producing them.
        tracing::warn!("no longer accepting peers");
        ExitCode::FAILURE
    })
}

/// `mediad` is a Linux daemon: it drives GStreamer against a Rockchip VPU and a V4L2 capture path.
/// The rest of the crate is portable and its tests run anywhere, which is why this is a stub rather
/// than a `cfg` on the whole crate.
#[cfg(not(target_os = "linux"))]
fn main() -> ExitCode {
    let _ = Args::parse();
    eprintln!("mediad runs on the robot; this host is not Linux");
    ExitCode::FAILURE
}
