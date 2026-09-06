# Session report — dance to music (72h)

**Started:** 2026-09-02 22:53 ET  
**Deadline:** 2026-09-05 22:53 ET  
**Closed:** 2026-09-06 09:09 ET (monitor loop stopped; training itself finished days earlier)

Orchestration was this Mac. All PPO ran on Spark `mstaub@100.69.30.38` (GB10). No training on the Mac. Litellm, docker, and other agents' jobs were not stopped.

## What landed

1. **Onboard listener (phases 0–3).** Spectral-flux beat tracker in `pet-detect`, tick-local crouch overlay in `robotd` inside the stand pose box. `[audio] listen` / `[audio] dance`. No `robot.dance` RPC. Observation stays `obs[1,61] → actions[1,14]`. Optional `beat` on `robot.state`.
2. **Standing dance gait.** Spark job `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/`. Tmux `dance-train-1bf7770a`. Smoke 64×5 exit 0. PPO 4096 envs, 15k iters, `TRAIN_EXIT 0` 2026-09-03 15:14 ET. ONNX `docs/ideas/dance-gait/dance_gait_iter14999.onnx`.
3. **Locomoting hop-dance.** Spark job `/home/mstaub/jobs/feat-dancing-skill-loco-1bf7770a/`. Scratch 15k then hop fine-tunes to `model_23997.pt` (`FIX2_TRAIN_EXIT 0` 2026-09-05 18:53 ET). ONNX `docs/ideas/dance-gait/dance_loco_iter_fix2.onnx`. Do not load this as `[policy] stand`.
4. **Play-time mixer.** Phrase mix 20/28/27/25 inplace/translate/sidestep/spin. Organic play clock (groove-warped φ, BPM/energy wander, hesitations, twist slew) without a retrain. Latest clip `docs/ideas/dance-gait/dance_loco_20s_organic.mp4`.
5. **Robot path (uncommitted).** Optional `[policy] dance`, `DanceDirector`, `API_VERSION` 19. Overlay drop still uses client twist. Uncommitted on purpose (no commit unless asked).

## Hosts, jobs, logs

| What | Where |
| --- | --- |
| Standing train log | `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/logs/train.log` |
| Standing ckpt | `.../microduck_dance/2026-09-02_23-39-19_microduck_dance/model_14999.pt` |
| Loco/hop2 log | `/home/mstaub/jobs/feat-dancing-skill-loco-1bf7770a/logs/fix2_train.log` |
| Hop2 ckpt | `.../2026-09-05_15-44-58_microduck_dance_loco/model_23997.pt` |
| Monitor loop | Mac PID 89535, every 20 min, **killed** 2026-09-06 09:09 ET |
| Spark tmux | standing + hop2 sessions **gone** after `TRAIN_EXIT` / `FIX2_PIPELINE_DONE` |

## Remaining work

- Hardware checklist on a real duck: `[policy] dance` = fix2 ONNX, stand stays `alpha_stand`, `[audio] dance = true`. Speaker in the room, own quack, dead `arecord`.
- Commit the uncommitted robotd / director wiring when asked.
- If the organic clip still reads as a stomp, nudge play groove/slew; do not scratch another 15k for that.
