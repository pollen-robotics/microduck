# The NPU, and the duck detector on it

The RK3566 has a small INT8 NPU — 0.8 TOPS, one core. This is the record of putting a trained duck
detector on it: what to run, what to expect, and what is still missing before a behaviour can use it.

The model is trained in [duck_detector](https://github.com/pollen-robotics/duck_detector) and comes
here as a quantised `.rknn`. First model, for reference: `yolo11n` at 320×320, one class, 150 frames
from three sessions, mAP50 0.976 on a held-out session — and 3.9 MB after INT8 quantisation, which
kept 2 of 2 detections at 95% box overlap against the float model on the desk.

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

`media.frame` now exposes the raw **UYVY** tee branch locally at `/run/mediad/media.sock`.
`robotctl media frame --output frame.uyvy` asks for the latest frame without stopping `mediad` or
back-pressuring its encoder. The response starts with a JSON-RPC header (geometry, format, byte
count and capture time), followed by exactly that many binary bytes. It is deliberately not a
WebRTC method: a frame is about 1.8 MiB at the default geometry, whereas the control channel has to
stay prompt. A recorder can therefore join camera observations to robot state on the board without
fighting the daemon or base64-encoding pixels.

What remains is publishing the detector's output on the robot state stream, so a behaviour can
consume it without opening a second observation channel. The detector already subscribes to the
raw branch and runs at a paced rate; publishing keeps perception next to the sensor, deriving
features rather than shipping pixels.

Once detections exist as state, the behaviours in `docs/ideas/autonomous_behavior.md` that currently
key on Bluetooth ("a duck is *nearby*") can key on sight ("a duck is *there*"): approaching,
following, facing, and a chorale where the ducks look at each other while they sing.
