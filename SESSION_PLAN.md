# Session plan — dance to music (72h)

**Task:** Complete `docs/ideas/dance-to-music.md`: onboard beat tracker + standing overlay, then train a dedicated dance gait on the Tailscale cluster (Spark GB10 / 5090). Do not train on this Mac.

**Started:** 2026-09-02T22:53:00-04:00
**Deadline:** 2026-09-05T22:53:00-04:00 (72 hours)
**Status:** IN_PROGRESS — robot-side landed; Spark PPO **finished** 15k (`TRAIN_EXIT 0`); ONNX exported

## Success criteria

1. Off-hardware `cargo test` covers the idea’s test list (tracker lock/unlock, EOF/gap, self-audio fixture, pose box, overlay preemption, worker without pet ONNX, shared PCM).
2. `[audio] listen` starts capture without a pet model; `[audio] dance` is opt-in, off by default; no `robot.dance` RPC; obs width stays 61.
3. In-process overlay maps beat phase → crouch waveform inside the stand box (chorale-sway style), omitted on pad / leftover `pose.active` / fall / skill / self-audio.
4. Attended phase-0B client streams the same crouch waveform and clears pose/head/mouth on exit.
5. A dance gait is smoke-tested then trained on Spark or the 5090 (not this Mac); ONNX export keeps `obs[1,61] → actions[1,14]`.
6. Session report at the deadline with host, PIDs, logs, and remaining work.

## Scope

**In**
- Phases 0–2 of the idea (scripted pose, tracker, in-process overlay).
- Phase 3 training: new mjlab task conditioning on beat phase/energy in the unbound command slots (`body_x`, `body_y`, `body_yaw`).
- Isolated Spark job directory; no killing of unrelated LLM / rfc / docker workloads.

**Out**
- Training on this Mac.
- Audio in the 61-D observation.
- A dance daemon or `robot.dance` mode.
- Stopping Spark model services or rfc pipelines to free GPU.
- Dancing while walking, on the roller, or with a publishing pad.

## Host choice (live, 2026-09-02 23:00)

| Host | GPU | Free | Decision |
| --- | --- | --- | --- |
| Spark `100.69.30.38` | GB10, ~128 GiB unified | **88 GiB free, 0% util** | **Train here** — isolated job dir, do not stop docker/LLM units |
| windows1 WSL `100.77.103.59` | RTX 5090 | `nvidia-smi` not on PATH in this login | Fallback if Spark fills or Warp/CUDA-13 fails |
| This Mac | — | — | Orchestration, `cargo test` only |
