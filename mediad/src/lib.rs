//! `mediad` — camera, mic, WebRTC, and the remote gateway.
//!
//! What exists so far is the control channel, end to end but for the channel itself:
//!
//! - [`route`] — which calls a WebRTC peer may make. `remote-webrtc.md` §5.
//! - [`upstream`] — connections to the five services that own the answers, one per (service, lane).
//! - [`session`] — the pipe. Lines in, lines out, replies never parsed.
//!
//! [`session::run`] is transport-agnostic on purpose: it takes lines and gives lines, so it is
//! testable without a WebRTC peer and would serve a WebSocket surface (§11) unchanged.
//!
//! - [`config`] — `[media]` in `robotd.toml`: what the stream is, edited with `robotctl configure`.
//! - [`web`] — the console page, served by the daemon it drives. `webrtc-console.md` §1.
//! - [`producer`] — who this robot says it is, before a peer negotiates anything. §5.
//!
//! [`pipeline`] is the rest, and the only part that is not portable: `webrtcsink` with the
//! signalling server in this process, `mpph264enc` in front of it, and a `control` datachannel per
//! peer wired to [`session::run`].

/// The WebSocket a program drives the robot over, as opposed to the datachannel a person does.
/// `architecture.md` §5.3 has the argument; the surface is the same one either way.
pub mod agent;
/// What the camera's geometry is — the intrinsics a consumer needs to turn pixels into
/// directions, and which sensor mode they belong to.
pub mod camera;
pub mod config;
/// The account credential `updaterd` writes, read by the two things here that need it.
pub mod producer;
/// The outward connection to the rendezvous service — what makes a duck reachable from off its
/// own LAN. `docs/design/remote-access-design.md` §3.
pub mod relay;
pub mod route;
pub mod session;
mod snapshot;
pub mod stream;
/// Relay candidates, so a robot behind a router is reachable from a network that cannot punch a
/// hole to it. `docs/design/remote-access-design.md` §6.
pub mod turn;
pub mod upstream;
pub mod web;

/// The GStreamer pipeline and the datachannel.
///
/// Assumed on Linux, which is the robot's OS. Elsewhere it is `--features gstreamer` and a
/// GStreamer install — the crate manifest says what that buys and what it costs. The capture half
/// inside is Linux either way; the rest is portable GStreamer.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub mod pipeline;

/// Auto-exposure, in software, because the board's 3A engine does not do it.
///
/// **Linux only, and unlike [`pipeline`] that is not a packaging decision**: this writes V4L2
/// controls to a camera through `ioctl`. Nothing off a robot wants it — there is no sensor behind a
/// simulated source or a test pattern to meter, and `main` arms it for a real camera and nothing
/// else.
#[cfg(target_os = "linux")]
pub mod exposure;

/// Looking for other ducks in the frames on the tee.
///
/// Gated with [`pipeline`] rather than with [`exposure`]: it reads the tee's raw branch, so it
/// needs the pipeline to exist and needs nothing else. `duck-detect` and `ort` are portable, so a
/// duck in the twin can be pointed at a model like any other.
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub mod detect;

/// The local, on-demand raw-frame endpoint. Gated with the pipeline, because it reads the frame
/// rendezvous that lives there; the WebRTC control channel deliberately does not carry
/// camera-sized replies. `npu-bringup.md` §"media.frame".
#[cfg(any(target_os = "linux", feature = "gstreamer"))]
pub mod frame;
