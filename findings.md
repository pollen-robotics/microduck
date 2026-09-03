# Findings

## Requirements
- Complete `docs/ideas/dance-to-music.md`.
- Train the phase-3 policy on Spark or the 5090 via tailscale-compute-cluster.
- Do not train on this Mac.
- Work autonomously for 72 hours.

## Architecture (from the idea + code)

Hear a beat, send intents the **existing** stand network already knows. Do not put audio in `obs[1,61]`.

```
arecord 16 kHz mono (one capture)
  ├ pet CNN (opt-in, may be absent)
  ├ ambient sentry (ignores music)
  └ beat tracker (new)
        → BeatState cache (t, locked, bpm, phase, onset_seq, energy)
              → mapper overlay on this tick's command (not PoseIntent)
                    → stand policy, unchanged
```

## Key code
- Mic worker: `pet-detect/src/worker.rs` — optional CNN; beat + sentry on the same PCM
- Spawn: `robotd/src/main.rs` if `listen || dance || (pet_detect && model)`
- Overlay: tick-local `command.body` after EMA, omitted via `OverlayGate`
- Pose box: z −0.025..+0.010 m, angles ±0.2618
- Unbound command slots hardcoded 0 in the *stand* net unless `audio.dance_gait`: then
  `body_x=sin(2πφ)`, `body_y=cos(2πφ)`, `body_yaw=energy` and z/roll/pitch stay 0.
  Only with the dance ONNX loaded as `[policy] stand`.
- Self-audio = `Sound::emitting()` (ride or one-shot child)

## GPU
- Spark job dir: `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/`
- Smoke: 64 envs × 5 iters, exit 0 (2026-09-02 23:38 ET)
- Full train: tmux `dance-train-1bf7770a`, python PID ~116161, 4096 envs, **15k iters finished** `TRAIN_EXIT 0` at 2026-09-03 15:14 ET (~15.4 h wall). Final reward ~64, ep len 600, `nan_state` 0.
- Checkpoint: `.../microduck_dance/2026-09-02_23-39-19_microduck_dance/model_14999.pt`
- ONNX: `docs/ideas/dance-gait/dance_gait_iter14999.onnx` (`obs[1,61] → actions[1,14]`, normalizer baked)
- Preview: `docs/ideas/dance-gait/dance_gait_iter14999.mp4`
- Log: `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/logs/train.log`
- Do not `docker compose down`, do not stop litellm/rfc/docker
- windows1 WSL user `micro`; fallback if Spark fills

## Mapping
```
# alpha stand overlay (audio.dance, dance_gait=false)
z = -A_z(energy) * 0.5 * (1 + cos(2π phase))   # most negative at onset, 0 at energy 0
A_z(1) ≤ 0.025 m
roll/pitch 0 at first

# dance gait (audio.dance + audio.dance_gait, stand=dance ONNX)
body_x = sin(2π φ); body_y = cos(2π φ); body_yaw = energy; z/roll/pitch = 0
```

## Resources
- Idea: `docs/ideas/dance-to-music.md`
- RL: https://github.com/pollen-robotics/microduck_rl (develop)
- CLAUDE.md playbook: 64-env 5-iter smoke before any long run
