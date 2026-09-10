# Recurrent ONNX policies

The runtime accepts feed-forward policies and the explicit-state LSTM export produced by
mjlab/rsl_rl. Both consume the existing 61D observation and produce the same 14 raw joint
actions at 50 Hz. Observation normalization must be baked into the exported model.
The training critic is not deployed. This adds no sensors or action filtering.

## Model contract

All tensors are float32. Inputs and outputs are matched by name, independently of order.

| Model | Inputs | Outputs |
| --- | --- | --- |
| Feed-forward | `obs [1, 61]` | One output `[1, 14]` (existing output names remain accepted) |
| LSTM | `obs [1, 61]`, `h_in [layers, 1, hidden]`, `c_in [layers, 1, hidden]` | `actions [1, 14]`, `h_out [layers, 1, hidden]`, `c_out [layers, 1, hidden]` |

Batch dimensions may be symbolic; inference always uses one robot. Layers and hidden
width must be positive static dimensions, identical across the four state tensors.
Each state is limited to 1,048,576 elements. Unsupported names, ranks, types, widths and
extra tensors fail at load. Warm-up also rejects invalid or non-finite outputs.
GRU and other recurrent export signatures are not implemented.

Publish recurrent policies with **`model_api: 2`** so older daemons refuse them before
installation. This daemon also accepts existing API 1 feed-forward models. The manifest
schema, observation width and action width are unchanged.

## Memory lifetime

Each loaded policy owns preallocated hidden and cell tensors. Every successful inference
copies both new states into those buffers. Failed inference clears memory; non-finite
outputs are rejected before either state is committed. Warm-up state is discarded.

Memory starts at zero on initial activation, on switching networks (including switching
back), explicit disable, controller reset after a pause or fall recovery, and a chained
skill restarting. Brief sensor dropouts follow the existing resume grace period.
Command changes within an active network retain memory.

Replacing an inactive slot preserves the active network's memory only when its ONNX file
has the same SHA-256 digest. A changed active model starts fresh, even at the same file path.
Use self-contained ONNX exports for this guarantee: externally referenced weight files
are not part of the ONNX file digest. An explicit controller reset always clears all slots.

Existing action scaling and optional runtime filters retain their configuration. For a
policy trained without action filtering, disable those filters in its deployment tuning;
adding recurrence does not make a mismatched filter configuration valid.

## Offline rehearsal and board timing

The example uses the production Rust inference path and never opens the motor bus:

```sh
export ORT_DYLIB_PATH=/path/to/libonnxruntime.so
cargo run --release -p duck-control --example policy-rehearsal -- policy.onnx > rehearsal.json
```

It warms up the actor, clears state, then measures 1,000 inferences with nominal gravity,
zero commands and previous-action feedback. JSON includes p50/p95/p99/max latency, the
number exceeding 20 ms, and the actions. Run the built example **on the Radxa** to establish
onboard timing; development-host timings do not establish the robot's timing budget.
These timings exclude sensor reads, motor writes and scheduling delays.

For deterministic multi-step comparison against PyTorch or Python ONNX Runtime, supply a
JSON array of records as the second argument:

```json
[{"obs": ["replace with exactly 61 numbers"], "reset": true}]
```

Each record supplies the complete observation, including previous actions. Set `reset` at
episode boundaries. Compare the returned action sequence to the reference with the same
observations and resets. This checks inference and memory handling, not physical sim2real
transfer. Export trained checkpoints through the training repository's normalized exporter.

## Tests

```sh
cargo test -p duck-control
# Requires ONNX Runtime >= 1.23:
cargo test -p duck-control --test recurrent_policy -- --ignored
cargo test -p robotd recurrent_memory_resets_with_controller_feedback -- --ignored
```

Tiny checked-in fixtures use an actual ONNX LSTM operator. They exercise memory continuity,
resets, policy switches, fallback selection, warm-up, hot swaps, dynamic batch dimensions,
invalid contracts and non-finite state. Regenerate with
`python duck-control/tests/fixtures/generate.py` in an environment with `onnx` and `numpy`.
