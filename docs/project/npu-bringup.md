# The NPU, and the duck detector on it

The RK3566 has a small INT8 NPU — 0.8 TOPS, one core. This is the record of putting a trained duck
detector on it: what to run, what to expect, and what is still missing before a behaviour can use it.

The model is trained in [duck_detector](https://github.com/pollen-robotics/duck_detector) and comes
here as a quantised `.rknn`. First model, for reference: `yolo11n` at 320×320, one class, 150 frames
from three sessions, mAP50 0.976 on a held-out session — and 3.9 MB after INT8 quantisation, which
kept 2 of 2 detections at 95% box overlap against the float model on the desk.

## Where the model comes from

**The Hub, the way the policies do.** `duck_detector` publishes every run to
[`pollen-robotics/microduck-duck-detector`](https://huggingface.co/pollen-robotics/microduck-duck-detector)
— `duck_detect.rknn` for the NPU and `duck_detect.onnx` for the CPU fallback, at the repo root under
fixed names, one tag per run. Nothing in this repository carries the weights: `mediad` reads them
from `/opt/robot/detector/current`, and what fills that is

| | |
|---|---|
| `scripts/seed-detector.sh` | run by the release's postinstall hook; installs the pin in `[workspace.metadata.detector]` on a board that has nothing, and never touches a set it did not install |
| `robotctl duck-detector check` | what is installed against what the repo offers |
| `sudo robotctl duck-detector update [--version <tag>]` | installs a revision and restarts `mediad` onto it |

It is `seed-policies.sh` and `robotctl policy check/update` with a different root and a fixed file
list, served by the same `updaterd` calls (`detector.check`, `detector.install`), and
`docs/design/policy-channel-design.md` §9 has the reasoning that carries over: the pin is a floor,
nothing partial goes live, a retrain is a tag rather than a daemon release.

Two things worth knowing. The model repo **shares its name with the dataset repo**; the robot only
ever addresses the model (`…/resolve/<rev>/…`, `api/models/…`), and the dataset lives under
`datasets/`, so nothing on the robot can land on a frame by accident. And `update`'s "newest" is
decided by **version tags** (`v2` sorts above `v1`; a name like `experimental` never counts), so a
run meant for robots wants a `vN` tag — the first run was tagged `duck-v1`, which is what the pin
names and is fine to install by name, but is not a version `check` can rank.

## What is here

| | |
|---|---|
| `duck-detect` | The letterbox, the runtime binding, and the decode — plus `duck-bench`. |
| `scripts/setup-npu.sh` | Enables the NPU node, installs `librknnrt.so`, and reports on the driver. |

Two decisions worth knowing before reading either:

**`dlopen`, not link.** `librknnrt.so` is a vendor blob in no Debian suite, and a crate that linked
it could not be cross-compiled in CI. `robotd` reaches ONNX Runtime the same way. The cost is
`duck-detect/src/rknn.rs`; the benefit is that `cargo board --bins` still works on a laptop.

**The runtime dequantises.** A quantised model's output tensor is int8 with a scale and a zero
point. `rknn_outputs_get` will convert to float if asked, and it is asked — the alternative is
carrying the scale into the decoder and getting it wrong once, quietly.

## Running the benchmark

The driver is the gate: it is part of the vendor kernel, mainline has none, and nothing in userspace
can work around its absence.

**An ordinary `robotctl update` does this.** `hooks/preinstall` runs the release's own copy beside
`setup-gstreamer.sh` and `setup-rkaiq.sh`, never fatally, with its report in the update log — so a
board provisioned before the NPU existed is fixed by an update rather than by somebody remembering
a command. Running it by hand is a retry:

```bash
sudo sh /opt/robot/daemon/current/scripts/setup-npu.sh
```

**Expect the first update carrying this to ask for a reboot.** Armbian ships `npu@fde40000` as
`status = "disabled"` on every Radxa Zero 3, so a stock board has the hardware, the kernel and the
driver and still no NPU. The script writes the overlay that fixes it and says so; the node binds on
the next boot. `--no-enable-node` installs only the runtime, and `dmesg | grep rknpu` is how you
confirm afterwards.

Run the release copy rather than `/usr/local/sbin/robot-setup-npu`: the overlay source lives beside
the script, and the copy left in `/usr/local/sbin` has nothing beside it on a first run.

Then, from a clone on your machine:

```bash
cargo board --bins -p duck-detect
scp target/aarch64-unknown-linux-gnu/release/duck-bench microduck@<robot>:/var/tmp/
scp <the>.rknn microduck@<robot>:/var/tmp/duck.rknn
scp -r datasets/raw/<a-session> microduck@<robot>:/var/tmp/frames
```

`duck-bench` is not in a release: it is a measuring tool, and packaging it would put it on every
robot for the benefit of two people. It goes over `scp` until there is a behaviour that needs the
detector, at which point what ships is the detector inside `mediad`, not this.

```bash
/var/tmp/duck-bench --model /var/tmp/duck.rknn --frames /var/tmp/frames
```

It answers three questions in the order they matter:

1. **Does it run?** A runtime that will not load, a model built for another platform, or a driver
   older than the runtime all fail here rather than inside a daemon.
2. **Does it still see ducks?** It reports detections per frame, because a model that runs and
   detects nothing looks exactly like one that works.
3. **What does it cost?** Latency percentiles and the CPU *this process* burned — the reason to use
   the NPU is to leave `robotd`'s 50 Hz loop alone, and that is a claim to measure.

`--threshold` is the flag to reach for first. **The quantised model's scores are on their own
scale** — the float model's 0.5 is not this model's 0.5 — so a run that detects nothing is more
likely a threshold than a broken conversion. Try `0.2` before believing the worst.

## Numbers

From a Radxa Zero 3, `duck-bench` at the paced 2 Hz, 30 frames over 3 passes:

| | measured | notes |
|---|---|---|
| driver / runtime | 0.9.8 / 2.3.2 | `setup-npu.sh` prints both |
| latency p50 / p95 | 25.7 ms / 58.4 ms | inference plus decode, not JPEG decoding |
| cpu per frame | 20.7 ms | see below — this is not all inference |
| detections | | against frames a person has already labelled |
| soc temp | 63 °C | at the end of a paced run |

**The CPU figure is not the NPU's cost, and the way it is reported invites reading it as one.**
The latency column times `infer` + `decode`; the CPU column is the whole loop's process CPU divided
by frames, so it also carries `letterbox_rgb` — a 1280×720 → 320×320 resample that runs on the CPU
and is not in the latency at all. Whether the remainder means `rknn_run` busy-waits (charging NPU
wait to the CPU) is not yet known. At 2 Hz it is 4% of one core either way; before anyone quotes
that as the price of perception, the two should be measured apart.

## What is still missing

`mediad` has a raw tee branch that exists precisely for this — `architecture.md` §5.3. Two ways
forward, and they are not exclusive:

- **`media.frame`** — **done.** A call that answers with one frame, on `mediad`'s own unix socket
  (`/run/mediad/media.sock`), group-readable like the other observation sockets. Useful for far
  more than perception (a snapshot in the console, a still for a bug report), and it means capturing
  a dataset no longer has to stop `mediad` to take the camera. It answers a JSON-RPC header naming
  a byte count, then those bytes: a raw frame is ~1.8 MiB, which is not something to base64 into a
  control reply. It asks the tee for the *next* frame rather than taking a cached one, so a reader
  cannot be handed the frame a stopped camera stopped on.
- **The detector inside `mediad`**: subscribe to the raw branch, run the model at a few Hz, and
  publish detections on the state stream. This is where it ends up — perception next to the sensor,
  deriving features rather than shipping pixels — and it is what a behaviour would consume.

Once detections exist as state, the behaviours in `docs/ideas/autonomous_behavior.md` that currently
key on Bluetooth ("a duck is *nearby*") can key on sight ("a duck is *there*"): approaching,
following, facing, and a chorale where the ducks look at each other while they sing.

### Taking a snapshot

On the robot, `robotctl frame --output frame.uyvy` saves one fresh packed UYVY frame and prints
its JSON metadata (width, height, bytes and capture timestamp) to stderr. Use those dimensions
when converting it, for example `ffmpeg -f rawvideo -pixel_format uyvy422 -video_size 1280x720
-i frame.uyvy -frames:v 1 frame.png` for a 1280×720 capture; do not assume that geometry after
changing the camera mode or rotation. The file is written only after the full response arrives.

From a browser on the robot's LAN, open `http://<robot>:8080/frame`, or save it with
`curl --fail http://<robot>:8080/frame -o frame.png`. This returns a PNG at the capture geometry,
with `Cache-Control: no-store`. A stopped or unavailable camera returns HTTP 503, never the
last good picture. PNG preserves the RGB conversion without JPEG compression; it is not a
byte-for-byte replacement for the raw UYVY data. This has the same LAN access boundary as the
existing console and camera stream; there is no additional authentication on this route.

The local endpoint supports `hello`, then `media.frame` on the same connection. Its socket is
configurable with `mediad --frame-socket <path>` and `robotctl --media-socket <path> frame`.
Failure to claim the socket aborts startup; an existing file or live listener is left intact.
There are at most 16 local connections, each limited to five seconds, and at most four HTTP
snapshot jobs. HTTP capture has a three-second deadline and response metadata is capped at
4 KiB. Invalid geometry and payloads larger than 16 MiB are refused by both clients.

`media.frame` intentionally has no `Call`/service-lane route: its JSON header is followed by a
binary tail. Neither the WebRTC control datachannel nor `duckctl`'s current BLE transport
carries snapshots. Use the local socket or the console's HTTP route; remote video transport is
separate. A future shared socket-group helper can replace the existing duplicated ownership
code without coupling this feature to a multi-daemon refactor.
