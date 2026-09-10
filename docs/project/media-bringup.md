# Media bring-up on hardware

What a Radxa Zero 3W does about video. Everything below was observed on a board, not inferred —
where something is still an assumption it says so.

## What this settles

`mediad` needs hardware H.264: software encode is not a slower option, it is not an option.
`jpegenc` alone cannot hold 30 fps at 640x480 on this SoC
(`microduck_runtime/src/camera.rs:500`), and H.264 costs more per frame than JPEG, on four
Cortex-A55s that `robotd`'s 50 Hz control loop already shares.

**The VPU encodes H.264, the bitstream is valid, and the encoder is reached through Rockchip's
MPP rather than V4L2.** Two GStreamer plugins have to be built from source to use any of it, and
neither is blocked by anything unknown.

| | |
|---|---|
| VPU encodes 720p H.264 | yes — 60 frames, 428 KB, via `mpi_enc_test` |
| Bitstream valid on this kernel | yes — clean `avdec_h264` decode, High profile level 4, 4:2:0 8-bit |
| Reached through | `/dev/mpp_service` (Rockchip MPP). **Not** V4L2 M2M |
| GStreamer plugin ABI across 1.x | not a risk — a 1.14-built plugin registers cleanly in 1.26.2 |
| `mpph264enc` element | must be built (§ [What has to be built](#what-has-to-be-built)) |
| `webrtcsink` / `webrtcsrc` | must be built, separately |

## What the board has

Nothing media-related is installed on a provisioned board. `scripts/setup-gstreamer.sh` installs
it and reports what the hardware can do; that script is the executable form of this page, and it
is where a command someone needs again should end up.

GStreamer comes from **plain Debian trixie** — `apt-cache policy` shows `deb.debian.org` and
`security.debian.org` with no Armbian multimedia overlay, so the archive's versions apply
exactly:

| package | version |
|---|---|
| `gstreamer1.0-plugins-bad` (has `webrtcbin`) | 1.26.2-3+deb13u3 |
| `libgstreamer-plugins-bad1.0-dev` (has `gstreamer-webrtc-1.0.pc`) | 1.26.2-3+deb13u3 |
| `gstreamer1.0-nice` | 0.1.22-1 |
| `gstreamer1.0-plugins-rs` | **does not exist in any Debian suite** |

The kernel is `6.1.115-vendor-rk35xx`. That matters twice: the camera's MIPI-CSI ISP capture
driver exists only on Armbian's vendor branch, and so do the VPU nodes. `setup-board.sh` already
installs that kernel — for the audio codec's I²S tree, not for video — so the prerequisite was
met before anyone asked for it. A stray `apt upgrade` that pulls the `current` kernel and
repoints `/boot` takes both away.

## The camera needs an overlay, mirrored under a prefix

A plugged-in CSI camera produces **no `/dev/video*` and nothing in dmesg** until its device-tree
overlay is enabled — which reads exactly like a camera that is not connected.

Enabling it has a trap worth stating on its own. Armbian ships the overlay as
`radxa-zero3-rpi-camera-v2.dtbo` with **no `rk3568-` prefix**, while the board runs
`overlay_prefix=rk3568`. So an `overlays=` word resolves to
`rk3568-radxa-zero3-rpi-camera-v2.dtbo`, the loader finds nothing, and the board boots happily
with no camera — the same silent failure `configure_overlay` exists to prevent for `uart2-m0`.
The file has to be **mirrored under the prefixed name first**, and only then named in `overlays=`.
`microduck_runtime/install.sh` hit this and does the same thing.

`configure_camera` in `setup-board.sh` does both, into the *vendor* kernel's overlay directory —
the MIPI-CSI capture driver exists only on that branch, which is the second reason the vendor
kernel is not optional. `DUCK_CAMERA_OVERLAY` picks another module; Armbian ships one per sensor,
and for this board that is `radxa-zero3-rpi-camera-v2` (Pi Cam v2 / IMX219) or
`radxa-zero3-rpi-camera-v1.3` (Pi Cam v1.3 / OV5647). Only the first has ever been used here.

## The encoder is MPP, not V4L2

`v4l2h264enc` is absent and `/dev/video*` is empty. Neither is a fault:

- On a Rockchip BSP kernel the VPU is exposed as `/dev/mpp_service`, not as a V4L2 M2M encoder.
  `v4l2h264enc` is registered by `gstreamer1.0-plugins-good` only when it finds an encoder node,
  so its absence is the expected shape here rather than a missing package.
- No `/dev/video*` at all is also exactly what an **unattached camera** looks like — the rkisp
  capture nodes appear only once a sensor is probed.

This is worth stating plainly because it is the branch point. Had the kernel exposed a V4L2
encoder, hardware H.264 would have needed nothing out of tree at all.

### The permission trap

`/dev/mpp_service` arrives as `crw------- root root`, mode 0600. A non-root process cannot open
it — and **`mpi_enc_test` against it writes an empty file and exits 0**. No error, no log line.
A zero exit status is therefore evidence of nothing; the file size is the evidence.

`mediad` will run as its own user, like every other daemon here — `tofd` rides into `i2c`,
`padd` into `input`, `btd` into `bluetooth` — so the VPU needs the same treatment: a udev rule
giving the node a group, and `SupplementaryGroups=` on the unit.
`scripts/setup-gstreamer.sh` installs that rule (`99-robot-mpp.rules`, group `video`, mode
0660), following the shape of `configure_tof`'s i2c rule in `setup-board.sh`.

`video` rather than `robot`: `robot` gates the IPC sockets *we* define
([`app-path-design.md`](../design/app-path-design.md) §, the socket-mode-plus-group layering). A
kernel device node is not ours to redefine, and `video` is the distro convention for this device
class, so a developer with `gst-launch` gets in the same way `mediad` does.

## What Radxa's pool provides

Rockchip MPP is not in Debian. Radxa publish it as a GitHub Pages apt repo, and the packages are
taken as **direct `.deb` downloads** rather than by adding the repo to `sources.list` — which is
the route `microduck_runtime/radxa_setup/setup_rkaiq.sh` already uses on this board for
`rkaiq_3A_server`. Base: `https://radxa-repo.github.io/bullseye/pool/main`.

| package | version | why |
|---|---|---|
| `m/mpp/librockchip-mpp1` | 1.5.0-1 | the MPP userspace library |
| `m/mpp/librockchip-vpu0` | 1.5.0-1 | `rockchip-mpp-demos` depends on it at exactly this version |
| `m/mpp/rockchip-mpp-demos` | 1.5.0-1 | `mpi_enc_test` — proves the VPU with no GStreamer involved |
| `m/mpp/librockchip-mpp-dev` | 1.5.0-1 | headers, to build the encoder plugin |
| `libr/librga/librga2` | 2.2.0-1 | Rockchip 2D accelerator; the rockchip plugin depends on it |
| `libr/librga/librga-dev` | 2.2.0-1 | headers, same build |
| `g/gstreamer1.0-rockchip/gstreamer1.0-rockchip1` | 1.14-4 | the MPP GStreamer plugin — see below |

These are bullseye builds and they configure cleanly against glibc 2.41 on trixie.

**`dpkg -i` resolves nothing**, because these do not come from a configured apt source. Every
missing dependency is an unconfigured package rather than an install that repairs itself, so each
set has to name its full closure. That cost three round trips to learn.

### Radxa's prebuilt plugin is *not* decode-only, and this page said it was

`gstreamer1.0-rockchip1_1.14-4` installs and registers cleanly in GStreamer 1.26.2, showing
exactly `mppvideodec` and `mppjpegdec`. That reads as "no encoders", and this page claimed it —
wrongly. `strings` on its `.so` lists `mpph264enc`, `mpph265enc`, `mppjpegenc` and `mppvp8enc`.
They are all there.

**The permission trap is the whole explanation, and it produced four separate misleading results
before that was clear:**

| what it looked like | what was actually true |
|---|---|
| `mpi_enc_test` wrote nothing and **exited 0** | the node was unopenable; a zero exit says nothing |
| Radxa's deb was decode-only | it has every encoder |
| a third-party 1.14-8 deb still showed no `mpph264enc` | same cause again |
| our own CI build lists only the two decoders | a container has no `/dev/mpp_service` either — expected, not a failure |

An MPP plugin **registers its decoders unconditionally, and probes MPP before registering its
encoders.** With `/dev/mpp_service` at `0600 root:root` the probe fails silently, so the encoders
are omitted from a plugin that contains them perfectly well.

So a plugin listing only decoders is evidence about the *device node*, not about the plugin, and
`gst-inspect-1.0 mpph264enc` means nothing until the udev rule is in place.

One thing that install did prove, and it stands: **a plugin built against GStreamer 1.14 registers
without complaint in 1.26.2.** Plugin ABI was the stated reason to fear a source build, and it is
not the risk.

## What has to be built

Two plugins, for two unrelated reasons. Neither substitutes for the other.

| plugin | source | gives | why it cannot be installed |
|---|---|---|---|
| `gstreamer-rockchip` | [`JeffyCN/mirrors`](https://github.com/JeffyCN/mirrors) branch `gstreamer-rockchip`, meson | `mpph264enc` — the hardware encoder | Debian has no Rockchip encoder at all. Radxa's build *does* have them, so this one is about a pin we control, dropping `libx11-6`, and riding along with the plugin below |
| `gst-plugin-webrtc` | [`gst-plugins-rs`](https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs) at 0.15.3, cargo-c | `webrtcsink`, `webrtcsrc` | `gst-plugins-rs` is packaged in **no** Debian suite |

0.15.3 rather than the 0.14.5 the `reachy_mini` SDK documents: 0.14.5 is the floor that matters —
below it a `webrtcsink` deadlock fix between remote description and ICE handling is missing, which
presents as a client spinning forever on "connecting" — and 0.15.3 is simply newer. Both series
declare a GStreamer `v1_22` feature floor and the robot runs 1.26.2, so the newer one costs
nothing.

`webrtcbin` **is** installed, from `gstreamer1.0-plugins-bad`. So a WebRTC session is reachable
today without the second build — at the cost of implementing the signalling protocol ourselves.
`webrtcsink` is preferred because its signalling protocol is what a relay proxies, which is what
makes a central signaling server reusable.

### Why not a prebuilt one

The hardware, the kernel driver and MPP's userspace library all work with **nothing compiled** —
`mpi_enc_test` came out of a deb and encoded 720p H.264 on the first try. What is missing is only
the *GStreamer binding*: a plugin that wraps `librockchip-mpp` as an element a pipeline can use.
`mpi_enc_test` is a standalone program; GStreamer has no idea it exists. The same shape as ONNX
Runtime here — `libonnxruntime.so` is installed from a tarball with nothing compiled, and `ort` is
the binding that makes it reachable.

Prebuilt bindings:

| source | what it has |
|---|---|
| Radxa `bullseye` pool | `gstreamer1.0-rockchip1_1.14-4` — installed and inspected: `mppvideodec` + `mppjpegdec` only |
| Radxa `rk3588s2-bookworm` pool | the same `1.14-4`, byte-identical |
| [`numbqq/gstreamer-rockchip-debs`](https://github.com/numbqq/gstreamer-rockchip-debs) | `1.14-8` — **has every encoder** |

The last one is worth trying before building anything. Its `bookworm/arm64/<board>/` entries are
symlinks into `jammy/arm64/`, so it is an Ubuntu 22.04 build, from
`rockchip-linux/gstreamer-rockchip` (now 404) with Jeffy Chen as maintainer — the same upstream
Radxa built, at a revision with the encoders enabled. `mpph264enc`, `mpph265enc`, `mppjpegenc`
and `mppvp8enc` are all present in the `.so`.

Its `DT_NEEDED` is satisfied by what a board already has after the debs above:
`librockchip_mpp.so.1`, `librga.so.2`, `libgstreamer-1.0.so.0`, `libgstvideo`,
`libgstallocators`, `libgstpbutils`, `libdrm2`, `libglib2.0-0`, `libx11-6`, `libc6 >= 2.33`
against glibc 2.41. Nothing in it is RK3588-specific — the SoC differences live inside MPP, not
the plugin — and `Depends` bounds GStreamer only from below (`>= 1.14`).

Ours are built in [`microduck-gst-plugins`](https://github.com/pollen-robotics/microduck-gst-plugins)
— a repository of its own, deliberately:

- **Not on the board.** An RK3566 compiles the Rust half far too slowly to wait for.
- **Not cross-compiled either.** The daemon cross-builds with `cargo-zigbuild`, and
  `scripts/ci-cross-deps.sh` says outright that its one C dependency "is the cost of that one
  exception, and it is worth reading before adding another". GStreamer would be a much larger
  second one, and both routes — x86 multiarch, or a sysroot with a meson cross file — link against
  an approximation of the target.
- **Natively, on an arm64 runner in a `debian:trixie` container**, which is the robot's own
  userland. Nothing is approximated. arm64 runners are free on public repositories.
- **Public** for a second reason that matters more: the download happens during provisioning and
  from the updater's `preinstall` hook, which runs with a cleared environment and **no token**. The
  same arrangement the daemon already relies on for ONNX Runtime.

It builds both plugins, pins upstream by commit or tag in one `pins.env`, disables `rkximage` and
`kmssrc` (the X11 and KMS sinks in the same tree — a headless robot needs neither, and they are why
the prebuilt Radxa deb depends on `libx11-6`), and publishes a tarball plus its sha256 with a
`MANIFEST` naming the exact upstream ref per plugin. That manifest is the thing the third-party deb
could not answer.

Two traps it guards, both found by reading the trees rather than by failing:
`gst/rockchipmpp/meson.build` ends in `if not mpp_dep.found() → subdir_done()`, so a missing
`librockchip-mpp-dev` makes meson **skip the plugin and succeed**; and `dpkg -i` resolves nothing
for direct `.deb` downloads, so the Radxa closure is installed in one call.

**`mediad.service` must set `GST_PLUGIN_PATH`.** The plugins install to
`/usr/local/lib/gstreamer-1.0`, which GStreamer does **not** search by default — its built-in
path is the distro's `/usr/lib/aarch64-linux-gnu/gstreamer-1.0`, and that directory is
deliberately avoided so an `apt` operation cannot replace or remove them. So the unit needs

```
Environment=GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0
```

alongside the `SupplementaryGroups=video` the VPU node needs. Both are easy to forget and both
present the same way at runtime: the encoder simply does not exist, with nothing saying why.

`scripts/setup-gstreamer.sh` consumes it at a **pinned** version — never "latest". Two
provisioning runs a day apart producing different plugins, with nothing recording which, is an
unreproducible media bug waiting to happen. The pin lives in
`[workspace.metadata.gst-plugins]` in `Cargo.toml`, the script carries the literal because it is
fetched standalone with `curl`, and an `xtask` test asserts they agree — the same arrangement, and
the same reason, as `ONNX_VERSION`.

**Trying the third-party deb is not the same as depending on it.** It is one person's per-board dump with no
provenance we control, and it vanishes if that repository does. What it buys cheaply is the answer
to the only real question about the build — whether this plugin works against *our* MPP and
GStreamer versions — and if it does, building the same source ourselves is de-risked rather than
unnecessary. A pinned build of our own is still where this should end up.

**The build could be avoided entirely** by calling MPP's C API from `mediad` over Rust FFI, which
`mpi_enc_test` proves works. That trades a meson build for hand-written and hand-maintained
bindings to a vendor library, which is the worse side of the trade — but it is a real option, not
a dead end, if the plugin turns out to fight GStreamer 1.26.

### The upstream

`rockchip-linux/gstreamer-rockchip` is gone (404). `JeffyCN/mirrors@gstreamer-rockchip` is the
live mirror — last commit 2026-05-21 — and `gst/rockchipmpp` holds `gstmpph264enc.c`,
`gstmpph265enc.c`, `gstmppjpegenc.c`, `gstmppvp8enc.c`. Fork soup exists around it; whichever
fork and tag is used has to be pinned and recorded, for the same reason
`gst-plugin-webrtc` is pinned to ≥ 0.14.5 (below).

### The pin on `gst-plugin-webrtc`

**0.14.5 or newer, not 0.14.4.** The earlier tags miss a `webrtcsink` deadlock fix between remote
description and ICE handling, which presents as a client spinning forever on "connecting".
`reachy_mini`'s SDK install doc records it; note that `reachy-mini-desktop-app` vendors 0.14.4
and is therefore on the wrong side of that line.

Pollen already vendor this plugin for **x86_64** — built natively with `cargo cinstall`,
stripped, committed per-arch, and consumed by CI pinned to a commit ref plus a sha256. An
`aarch64/` sibling in the same place may be less total work than a second pipeline here.

### Where a built plugin belongs

In the daemon release payload, with `GST_PLUGIN_PATH` pointing into `current` — not in apt, and
not in `/opt` on each machine.

The plugin version and `mediad`'s code are entangled: the 0.14.5 story above is exactly a case
where a plugin version determines whether the daemon needs a workaround. A skew is a `mediad`
bug, so it wants `mediad`'s lifecycle — atomic swap, rollback, health gate. `librockchip-mpp` is
the opposite: a system library paired with the *kernel*, wanted by anything touching the VPU,
and belongs in the package manager.

## Measured, and not

**Measured on the board:** GStreamer 1.26.2 and its origin; `webrtcbin` present;
`webrtcsink`/`webrtcsrc` absent; `v4l2h264enc` absent and no `/dev/video*`; `/dev/mpp_service`
present at 0600 root:root; `mpi_enc_test` silently writing nothing as non-root and 428 KB as
root; that bitstream decoding clean as High/4.0; the Radxa debs' dependency closure; the rockchip
plugin loading in 1.26.2 with two decode elements.

**The encode path is closed, on hardware, end to end.** In order:

1. `v1` fetched from the public release, sha256 verified, installed to
   `/usr/local/lib/gstreamer-1.0`.
2. `gst-inspect-1.0 mpph264enc` answers `provided-by
   /usr/local/lib/gstreamer-1.0/libgstrockchipmpp.so` — our build. The third-party deb was
   removed first so the answer could be attributed to something.
3. It **encodes**: `videotestsrc ! mpph264enc profile=baseline header-mode=each-idr bps=2000000 !
   h264parse ! filesink` produced 476 KB for 60 frames of 720p in **0.44 s wall**, source
   generation and pipeline setup included — comfortably faster than realtime. The result decodes
   clean through `avdec_h264`.
4. It works **without root**: after a udev rule puts `/dev/mpp_service` at `660 root:video` and
   the user joins `video`, `gst-inspect` answers as that user. Which is the case `mediad` is in,
   and every check before this one had been under `sudo`.

**The whole media chain is closed on hardware.** Sensor to a WebRTC-negotiable stream:

| step | evidence |
|---|---|
| overlay applied | `csi2-dphy0` probed, `rkisp` up, ten `/dev/videoN` |
| sensor identified | `imx219 2-0010: Model ID 0x0219, Lot ID 0x5a8e73, Chip ID 0x0773` |
| capture node | `/dev/video0`, card name `rkisp_mainpath`, formats to 3280x2464 |
| frames | 13,824,000 bytes for `--stream-count=10` at 720p NV12 — exact |
| hardware encode | `v4l2-ctl … --stream-to=-` into `fdsrc ! rawvideoparse ! mpph264enc` |
| the stream | decodes clean; `h264parse` reports **1280x720 constrained-baseline** |

The node numbers are not stable across boots, so the capture node is found by matching the card
name `rkisp_mainpath` under `/sys/class/video4linux/*/name` — as `camera.rs:219` does.

### Three device nodes need the `video` group, not one

This cost three separate debugging rounds, each with a failure that named something else:

| node | symptom when root-only |
|---|---|
| `/dev/mpp_service` | `mpi_enc_test` writes nothing and **exits 0**; `mpph264enc` is not registered at all |
| `/dev/rga` | the element exists, the pipeline starts, then `Try to use uninit rgaCtx=(nil)` and pages of `rga call blit fail` |
| `/dev/video0` | arrives `root:video` already, so it is the one that does not bite |

`setup-gstreamer.sh` installs one udev rule covering the first two.
**`mediad.service` needs `SupplementaryGroups=video`** — with `Environment=GST_PLUGIN_PATH`, that
is two lines standing between a working pipeline and four different confusing failures.

### The rotation that cost 22 fps

The camera is mounted a quarter turn off, and the obvious fix — `videoflip` before the tee, so every
consumer gets an upright picture — is the wrong one, measured on a robot:

| | RGA failures | frames lost by `v4l2src` | fps | SoC |
|---|---|---|---|---|
| without the flip | 0 | 0 | ~30 | fine |
| with the flip | 5522 in one session | 1565 | 7–8 | 97 °C, CPU at 408 MHz |

`mpph264enc` hands the UYVY→NV12 conversion to the SoC's 2D engine and pays nothing for it.
`videoflip`'s output is a buffer the RGA refuses — `10000 is unsupport format`, then `RGA_BLIT fail:
Bad address` on a `rect[0, 0, 720, 1280]` — so MPP falls back to converting **every frame in
software**. That saturates the CPU, the SoC hits its thermal limit, everything throttles to 408 MHz,
and the camera drops frames it cannot deliver. The rotation itself is the smaller half of the bill.

So nothing in the pipeline rotates. The mount is *reported* (`media.video`, once per control
channel) and whoever displays the picture turns it: the console does it with a CSS transform, which
is free, and a perception consumer folds it into the resampling it already does. `--flip-in-pipeline`
puts the old behaviour back for a consumer that cannot rotate for itself and can afford this.

The proper fix, if an upright stream is ever genuinely needed, is an RGA element in the plugin set:
the 2D engine rotates for nothing, which is why the encoder can afford to use it.
### The 3A engine has to be waiting before the stream starts

`rkaiq_3A_server` attaches to the ISP and then waits for a **stream start** event — and it misses one
that already happened. Restart it while `mediad` is streaming and it sits on

```
DBG: /dev/media0: wait stream start event...
```

for ever: no stats loop, so no auto exposure and no white balance, and a green picture. A reboot
"fixes" it only because a reboot happens to order the two correctly.

Which made it a regression with no obvious cause, because **every `robotctl update apply` restarted
the engine underneath a running stream**: the pre-install hook runs `setup-rkaiq.sh`, and that script
restarts `rkaiq_3A`. It even printed "restart the camera stream for it to take effect" — advice
nobody follows and nothing enforced.

So the drop-in that script installs carries the invariant now:

```ini
ExecStartPost=-/bin/systemctl --no-block try-restart mediad.service
```

Whenever the engine starts, the stream is bounced behind it, so the event it waits for is one that
has not happened yet. `try-restart` leaves a board with no `mediad` alone, and `--no-block` is what
keeps a unit waiting on another unit's job from deadlocking systemd. To rescue a board by hand, the
order is the whole trick:

```bash
sudo systemctl stop mediad && sudo systemctl restart rkaiq_3A && sleep 2 && sudo systemctl start mediad
```

### Nothing was supervising the engine

The vendor unit is `Type=forking` over `/etc/init.d/rkaiq_3A.sh`, whose `start_3A` is

```sh
/usr/bin/rkaiq_3A_server 2>&1 | logger -t rkaiq &
```

— a pipeline, backgrounded, and then the script exits. systemd's forking heuristic expects one
forked child and finds nothing it can claim, so on a board with the engine running:

```
$ systemctl is-active rkaiq_3A
active
$ systemctl show rkaiq_3A -p MainPID
MainPID=0
```

Both processes are reparented to init, though they do stay in the unit's cgroup, which is the one
thing keeping `systemctl stop` honest — the final cgroup kill is what collects them, not the unit's
own `ExecStop`. What is left broken:

- **The unit reads `active` whether the engine is alive or dead**, and nothing restarts it when it
  segfaults — which is the failure this platform actually has, the day the shim stops matching the
  kernel.
- **`stop_3A` is `killall rkaiq_3A_server`**, so the unit's stop reaches any engine on the box rather
  than the one it started.
- **`reload` is `stop_3A; start_3A` inside the unit**, so it never reaches the cgroup kill and leaks
  the `logger` from the previous run every time.

So the drop-in runs the engine directly: `ExecStart` cleared and set to `rkaiq_3A_server`, `ExecStop`
cleared so the vendor `killall` never runs, and the engine's output to the journal under its own unit
instead of through a `logger` process — `journalctl -u rkaiq_3A` rather than `journalctl -t rkaiq`.
`Type=simple` is right because the engine does not daemonise, and the evidence is that `logger`: it
was alive alongside the server, which only holds if the server keeps the write end of that pipe open
in the foreground.

**`Restart=` needs a cap here, and the cap is the interesting half.** The `ExecStartPost` above
bounces `mediad` on every start, and with `Type=simple` it fires as soon as the process is spawned —
before a segfault at startup is known. Uncapped, a board with a mismatched shim would bounce the
camera stream every few seconds for ever, which is worse than an engine that simply stays dead. So
`StartLimitBurst=3` in three minutes: `mediad` is bounced three times, then systemd gives up and
leaves the unit `failed` — which is at least a state `systemctl status` can report, where `active`
with `MainPID=0` was not. `on-failure` rather than `always`, because the segfault exits non-zero and
a clean exit from this engine has never been observed.

**A failed engine is still not visible in `robotctl health`.** `configd::units::MANAGED` is the seven
units a daemon release ships and `install.sh` manages, and the 3A engine is a vendor unit this
project provisions rather than ships, so it is deliberately not among them. Now that the unit can
reach `failed` at all, putting it there is worth considering — a green picture is a real symptom with
no reading on that report — but it means widening what that list claims to cover.

### rkaiq's auto-exposure fires once, and only if it caught the stream

`scripts/setup-rkaiq.sh` first shipped leaving rkaiq's AE **enabled**, on this reasoning: the
prototype disabled it only to stop the engine fighting a runtime that owned exposure itself, and
nothing owned exposure in `mediad`, so the engine should own it. It does write the sensor. It does
not keep writing it.

Measured on a robot, on a boot where the units started in the right order — `rkaiq_3A` at 17:24:01,
`mediad` at 17:24:11, `wait stream start event success` at 17:24:17:

| what | value |
|---|---|
| `mediad` wrote, at 17:24:17 | `exposure=600 analogue_gain=1024` |
| `/dev/v4l-subdev3`, minutes later | `exposure: 1589  analogue_gain: 1536` |
| a hand-written `exposure=300 analogue_gain=256`, then 25 s of watching | `300 / 256`, uncorrected |

So the engine converged once, to an answer that is plainly its own and not ours, and then stopped:
a picture made four times darker underneath it drew no response at all. One convergence at stream
start is not auto-exposure — a robot that walks from a window into a corridor keeps the window's
exposure.

**And on a boot where the engine missed the stream-start event — the section above — even the one
shot does not happen.** That was measured first and read wrongly: the sensor sat at exactly
`600 / 1024` for as long as it was watched and a manual write stuck, which looked like an AE that had
never worked, when it was an AE that had never been given a stream. The two states are
indistinguishable from a single reading of the sensor, which is why the two fixes belong together:
the ordering fix is what makes the difference between them observable at all.

`mediad::exposure` is what closes the loop, ported from the prototype's `ae_loop`: mean luma off the
tee's raw branch twice a second against a setpoint of 90, and a damped multiplicative step split
across three controls in noise order — shutter to 600 lines (≈11 ms, short enough not to smear a
walking robot), then analogue gain to 11×, then shutter to 1200 lines, then ISP digital gain. The
hard shutter cap is a real ceiling: the driver answers an exposure longer than the frame length by
*stretching the frame time* rather than clamping, so 3500 lines silently gives 15 fps.

Two things it does differently, because `mediad` is a better place for it than the prototype was.
Luma comes from the frames already tapped off the tee for the duck detector, so there is no JPEG
decode and nothing opens the camera a second time — the prototype's first version metered by
sampling the ISP self path with a parallel `v4l2-ctl`, which contended with its own capture at the
driver level and killed the pipeline intermittently. And the first write is read back: every way this
can fail (a node without the control, a denial, no `v4l2-ctl`) otherwise leaves a camera at one
exposure, which is indistinguishable from the bug it fixes.

`setup-rkaiq.sh` now asserts `CommCtrl.Enable = 0`. Not because the engine's AE does nothing, but
because what it does lands at stream start — exactly when mediad's loop is converging from its own
starting values, which is two writers racing for one control.

**One thing `v4l2-ctl` will do to you while measuring any of this:** a single unknown control name
fails the whole `--set-ctrl` or `--get-ctrl`. `--get-ctrl=exposure,analogue_gain,digital_gain`
returns `unknown control 'digital_gain'` and no exposure, which reads as a node that carries neither.
`mediad::exposure` writes the sensor pair and the ISP's digital gain in two calls for that reason.

### Two things known to be unfinished

**The bitrate came out ~50× under target.** 15,553 bytes for 3.3 s of capture against
`bps=2000000` is about 37 kbps. Either the scene was static enough for CBR to collapse — plausible,
the ISP was on raw defaults and the image is green and noisy until `scripts/setup-rkaiq.sh` runs
(ported from the prototype; provisioning and the pre-install hook both run it now) — or
capture is delivering well below 30 fps. The frame count has not been measured. If it is the
latter, the sensor mode is the suspect: the IMX219 boots in 3280x2464 and the prototype pins the
mode with `media-ctl` before every capture (`camera.rs:277`).

**`rawvideoparse blocksize=1382400` is a bring-up shortcut, not a design.** It works because
`v4l2-ctl` emits tightly-packed NV12 at a size we computed, and it is silently wrong the moment
stride padding appears at another resolution — `camera.rs` notes exactly that. `mediad` doing its
own V4L2 mmap loop into `appsrc` gets the real stride from the driver instead of assuming it.

## Two things the pipeline will have to decide

**Capture cannot use `v4l2src`.** The rkisp driver hands it a 2-buffer pool and it requeues too
slowly, dropping every third frame — ~20 fps from a 30 fps sensor, with "lost frames detected".
`v4l2-ctl --stream-mmap` sustains the full rate, so `microduck_runtime` captures with it and
pipes raw frames into a `fdsrc` pipeline (`camera.rs:487`). `mediad` needs either that subprocess
shape or its own V4L2 mmap loop feeding `appsrc`.

**Four `mpph264enc` properties are pipeline decisions, not defaults to inherit.** Read off the
element on the board:

| property | default | what `mediad` should set | why |
|---|---|---|---|
| `profile` | `high` | **`baseline`** | WebRTC's interoperable floor is Constrained Baseline (`profile-level-id 42e01f`). Current browsers negotiate High; older peers do not. Setting `baseline` produces a stream `h264parse` reports as `constrained-baseline` — verified, not assumed, since the enum only says "baseline" |
| `header-mode` | `first-frame` | **`each-idr`** | SPS/PPS in the first frame *only* means a peer that joins later — or loses that packet — never decodes anything. `reachy_mini`'s Pi pipeline sets exactly this on `v4l2h264enc` via `repeat_sequence_header=1`; same requirement, different spelling |
| `rotation` | `0` | **`180`** on the alpha | the IMX219 is mounted upside down. `microduck_runtime` fixes it with `videoflip method=rotate-180` — a full CPU pass over every frame, on the SoC `robotd` shares. The encoder does it in hardware for nothing |
| `bps` | `0` (auto) | an explicit target | `rc-mode` already defaults to `cbr`, which is what a lossy link wants; the bitrate should not be left to "auto calculate" |

Two things that turn out to need no decision:

- **There is no B-frame knob at all**, so §5.5's "no B-frames" requirement is satisfied by
  construction rather than by configuration.
- **The sink pad accepts `NV12`**, which is exactly what the rkisp capture path emits. No
  `videoconvert`, and no RGA colour conversion, between capture and encode.

Keyframes should come from `min-force-key-unit-interval` rather than a periodic `gop`: WebRTC
drives them from the peer's PLI, and `gop` defaults to one IDR per second whether anybody needed
one or not.

### The constraint flags were worth reading, and the template was worse than the flags

This page used to end by asking someone to verify one thing rather than assume it: the `profile`
enum says `baseline` (66) while WebRTC negotiates *Constrained* Baseline, and a Baseline stream
that avoids FMO, ASO and redundant slices is what a constrained-baseline decoder expects. Read off
the board:

```
gst-launch-1.0 -v videotestsrc num-buffers=60 ! video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! mpph264enc profile=baseline ! h264parse ! fakesink
```

`h264parse` negotiates `profile=(string)constrained-baseline` on its src pad. So the stream is
right: `profile=baseline` turns CABAC and the 8x8 transform off, MPP emits no FMO, ASO or
redundant slices, and the SPS carries `profile_idc=66` with `constraint_set1_flag`.

**What was wrong was the pad template, and it cost a day.** `mpph264enc`'s src template listed
`profile = { baseline, main, high }` and omitted `constrained-baseline` — the one profile WebRTC
asks for. That only matters once the encoder is inside `webrtcsink` rather than in front of it,
which is why nothing saw it here first:

1. `webrtcsink`'s codec discovery builds its encoding chain with no output caps, so `force_profile`
   is true and it inserts a capsfilter demanding `profile=constrained-baseline`.
2. `h264parse` strips `alignment`, `stream-format` and `parsed` from a caps query but **not
   `profile`**, so the demand reaches the encoder's src pad.
3. Empty intersection with the template. `GstVideoEncoder`'s sink getcaps returns nothing, and the
   failure surfaces four elements upstream as `videorate` reporting it "could not transform NV12 …
   in anything we support".
4. Discovery drops H.264 with a `gst::warning!`, VP8 is negotiated instead, and the session dies
   in `rtpvp8pay`. **No error anywhere names the profile.**

Two lessons rather than one. The plugins repository now carries a one-word patch widening that
template, released as `v3`. And `mediad` had to bridge GStreamer's log and the pipeline bus into
the journal before any of this was visible — every media failure up to that point had been silent,
including a session ending mid-negotiation.

### Pre-encoding is no longer the shape

This page used to close by noting that `webrtcsink` accepts pre-encoded H.264 on its sink pad, so
`appsrc ! mpph264enc ! h264parse ! webrtcsink` keeps the encoder out of negotiation entirely. True,
and it worked first — but it costs two things that are hard to add back. `webrtcsink` cannot reach
an encoder it does not own, so congestion control cannot adapt the bitrate to the link, and a
peer's PLI cannot produce a keyframe: a viewer that loses one stays broken until the next periodic
GOP.

So `mediad` hands it raw NV12 and lets it build the encoder, configuring each one through the
`encoder-setup` signal — which is where the table above now applies. The cost is that the encoder
*does* reach negotiation, which is how the template gap above was found.
