# Progress Log

## Session: 2026-09-02 → 2026-09-03

### Phase 0: Routing
- **Status:** complete
- Spark `100.69.30.38` GB10, isolated job dir `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/`

### Phase 1: Tracker + mapper
- **Status:** complete
- `pet-detect` beat + mapper tests green

### Phase 2: Worker + config
- **Status:** complete
- Mic worker runs with no pet ONNX; `[audio] listen` / `[audio] dance`
- `cargo test -p pet-detect --lib`: 14 passed
- `cargo test -p robotd-params --lib`: 26 passed

### Phase 3: Overlay + client
- **Status:** complete
- In-process overlay in `robotd` (chorale-sway style)
- Optional `beat` on `robot.state` (`API_VERSION` 18)
- `cargo run -p robotd --example dance-pose`

### Phase 4: Spark training
- **Status:** complete
- Smoke 64×5: **exit 0** at 2026-09-02 23:38 ET
- Full PPO: tmux `dance-train-1bf7770a`, PID 116161, 4096 envs, 15k iters
- Tick 1 (2026-09-03 00:03 ET): iter **390/15000**, mean reward **71.9**, ep len **600** (cap), `fell_over` ~0, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 68% @ 70°C, ETA ~14.8 h
- Tick 2 (2026-09-03 00:23 ET): iter **714/15000**, mean reward **70.2**, ep len **595**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 67% @ 67°C, ETA ~14.5 h
- Tick 3 (2026-09-03 00:43 ET): iter **1038/15000**, mean reward **68.2**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.89, GPU 66% @ 65°C, ETA ~14.2 h
- Tick 4 (2026-09-03 01:03 ET): iter **1359/15000**, mean reward **70.0**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 67% @ 66°C, ETA ~13.9 h
- Tick 5 (2026-09-03 01:23 ET): iter **1680/15000**, mean reward **63.4** (dip from 70; still high, ep len **590**, falls ~0.13), `nan_state` 0, `dance_z_tracking` ~1.86, GPU 73% @ 68°C, ETA ~13.6 h
- Tick 6 (2026-09-03 01:43 ET): iter **2010/15000**, mean reward **64.6**, ep len **598**. Dip explained: curriculum `action_rate_weight` −0.1→**−1.0**, `head_pose_range` 0.07→**1.4**. Falls ~0.3, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 65% @ 66°C, ETA ~13.2 h
- Tick 7 (2026-09-03 02:03 ET): iter **2331/15000**, mean reward **62.3**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 75% @ 67°C, ETA ~12.9 h. Curriculum still at `action_rate_weight` −1.0 / `head_pose_range` 1.4.
- Tick 8 (2026-09-03 02:23 ET): iter **2655/15000**, mean reward **60.9**, ep len **594**, `fell_over` ~0.08, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 68% @ 67°C, ETA ~12.6 h
- Tick 9 (2026-09-03 02:43 ET): iter **2979/15000**, mean reward **61.5**, ep len **591**, `fell_over` ~0.25, `nan_state` 0, `dance_z_tracking` ~1.86, GPU 49% @ 68°C, ETA ~12.2 h
- Tick 10 (2026-09-03 03:03 ET): iter **3300/15000**, mean reward **61.6**, ep len **596**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 63% @ 66°C, ETA ~11.9 h
- Tick 11 (2026-09-03 03:23 ET): iter **3618/15000**, mean reward **61.8**, ep len **592**, `fell_over` ~0.21, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 68% @ 68°C, ETA ~11.6 h
- Tick 12 (2026-09-03 03:43 ET): iter **3939/15000**, mean reward **63.4**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 64% @ 67°C, ETA ~11.3 h
- Tick 13 (2026-09-03 04:03 ET): iter **4266/15000**, mean reward **62.7**, ep len **594**, `fell_over` ~0.13, `nan_state` 0, `dance_z_tracking` ~1.89, GPU 49% @ 66°C, ETA ~11.0 h
- Tick 14 (2026-09-03 04:23 ET): iter **4587/15000**, mean reward **63.0**, ep len **598**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 44% @ 65°C, ETA ~10.6 h
- Tick 15 (2026-09-03 04:43 ET): iter **4905/15000**, mean reward **63.3**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 51% @ 65°C, ETA ~10.3 h
- Tick 16 (2026-09-03 05:03 ET): iter **5229/15000**, mean reward **61.9**, ep len **587**, `fell_over` ~0.13, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 62% @ 66°C, ETA ~10.0 h
- Tick 17 (2026-09-03 05:23 ET): iter **5556/15000**, mean reward **63.5**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 64% @ 65°C, ETA ~9.6 h
- Tick 18 (2026-09-03 05:43 ET): iter **5871/15000**, mean reward **62.5**, ep len **600**, `fell_over` ~0.08, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 58% @ 67°C, ETA ~9.3 h
- Tick 19 (2026-09-03 06:03 ET): iter **6189/15000**, mean reward **63.2**, ep len **600**, `fell_over` ~0.08, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 65% @ 66°C, ETA ~9.0 h
- Tick 20 (2026-09-03 06:23 ET): iter **6510/15000**, mean reward **62.4**, ep len **593**, `fell_over` ~0.17, `nan_state` 0, `dance_z_tracking` ~1.89, GPU 66% @ 66°C, ETA ~8.7 h
- Tick 21 (2026-09-03 06:43 ET): iter **6834/15000**, mean reward **63.9**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 75% @ 68°C, ETA ~8.4 h
- Tick 22 (2026-09-03 07:03 ET): iter **7155/15000**, mean reward **63.3**, ep len **596**, `fell_over` ~0.13, `nan_state` 0, `dance_z_tracking` ~1.89, GPU 55% @ 66°C, ETA ~8.0 h
- Tick 23 (2026-09-03 07:23 ET): iter **7482/15000**, mean reward **63.2**, ep len **595**, `fell_over` ~0.13, `nan_state` 0, `dance_z_tracking` ~1.88, GPU 68% @ 68°C, ETA ~7.7 h
- Tick 24 (2026-09-03 07:43 ET): iter **7803/15000**, mean reward **63.5**, ep len **597**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.92, GPU 78% @ 68°C, ETA ~7.4 h
- Tick 25 (2026-09-03 08:03 ET): iter **8121/15000**, mean reward **64.2**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.92, GPU 75% @ 67°C, ETA ~7.0 h
- Tick 26 (2026-09-03 08:23 ET): iter **8442/15000**, mean reward **63.0**, ep len **597**, `fell_over` ~0.13, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 79% @ 67°C, ETA ~6.7 h
- Tick 27 (2026-09-03 08:43 ET): iter **8763/15000**, mean reward **63.7**, ep len **597**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 76% @ 65°C, ETA ~6.4 h
- Tick 28 (2026-09-03 09:03 ET): iter **9081/15000**, mean reward **62.5**, ep len **598**, `fell_over` ~0.08, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 60% @ 64°C, ETA ~6.1 h
- Tick 37 (2026-09-03 ~12:16 ET): iter **11931/15000** (~80%), mean reward **63.8**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 61% @ 68°C, ETA ~3.1 h — **still running, not finished**
- Tick 38 (2026-09-03 12:23 ET): iter **12246/15000**, mean reward **64.8**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 58% @ 67°C, ETA ~2.8 h
- Tick 39 (2026-09-03 12:43 ET): iter **12567/15000**, mean reward **63.8**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 72% @ 71°C, ETA ~2.5 h
- Tick 40 (2026-09-03 13:03 ET): iter **12891/15000**, mean reward **64.6**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 66% @ 67°C, ETA ~2.2 h
- Tick 42 (2026-09-03 13:43 ET): iter **13548/15000**, mean reward **64.0**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 71% @ 69°C, ETA ~1.5 h
- Tick 43 (2026-09-03 14:03 ET): iter **13854/15000**, mean reward **62.2**, ep len **590**, `fell_over` ~0.17, `nan_state` 0, `dance_z_tracking` ~1.87, GPU 67% @ 72°C, ETA ~1.2 h
- Tick 44 (2026-09-03 14:23 ET): iter **14172/15000**, mean reward **64.3**, ep len **600**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` ~1.89, GPU 74% @ 67°C, ETA ~51 min
- Tick 45 (2026-09-03 14:43 ET): iter **14511/15000**, mean reward **63.3**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.91, GPU 75% @ 64°C, ETA ~30 min
- Tick 46 (2026-09-03 15:03 ET): iter **14820/15000**, mean reward **62.8**, ep len **598**, `fell_over` ~0.13, `nan_state` 0, `dance_z_tracking` ~1.90, GPU 65% @ 65°C, ETA ~11 min
- Tick 47 (2026-09-03 15:23 ET): **finished** iter **14999/15000**, `TRAIN_EXIT 0` at 2026-09-03 15:14 ET. Final mean reward **64.1**, ep len **600**, `fell_over` ~0.04, `nan_state` 0, `dance_z_tracking` ~1.92. Checkpoint `model_14999.pt`.
- Tick 48 (2026-09-03 15:43 ET): train still stopped (`TRAIN_EXIT 0`); ONNX + `model_14999.pt` present; `cargo test -p pet-detect --lib` **14 passed**. Remaining: hardware on a duck.
- Tick 49 (2026-09-03 16:03 ET): still stopped; artifacts present. Deploying the dance ONNX needs filling `body_x/y/yaw` from beat **only** under that net (stand weights stay zeros there).
- Continued (2026-09-03 16:18 ET): wired `audio.dance_gait` — beat → `body_x=sin(2πφ)`, `body_y=cos(2πφ)`, `body_yaw=energy`; alpha stand overlay unchanged. Tests: duck-control obs 9, pet-detect 15, robotd-params 26, robotd 96+7, robotctl 142.
- Tick 50 (2026-09-03 16:27 ET): train still `TRAIN_EXIT 0`; ONNX present. Remaining: hardware on a duck.
- Tick 51 (2026-09-03 16:43 ET): still stopped. 20 s MuJoCo clip at 100 BPM: `docs/ideas/dance-gait/dance_20s_100bpm.mp4`
- Export: `uv run scripts/export.py` → `obs[1,61] → actions[1,14]` ONNX at `docs/ideas/dance-gait/dance_gait_iter14999.onnx`
- Preview: `docs/ideas/dance-gait/dance_gait_iter14999.mp4`
- Log: `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/logs/train.log`

## Test Results
| Test | Expected | Actual | Status |
|------|----------|--------|--------|
| pet-detect lib | pass | 15 passed | pass |
| robotd-params | pass | 26 passed | pass |
| duck-ipc-proto | pass | 49 passed | pass |
| robotd | pass | 96 + 7 updater_gate | pass |
| robotctl | pass | 142 passed | pass |
| Spark smoke 64×5 | exit 0 | exit 0 | pass |
| Spark PPO 15k | TRAIN_EXIT 0, nan_state 0 | TRAIN_EXIT 0, reward ~64 | pass |
| ONNX export | obs[1,61]→actions[1,14] | that shape | pass |
| dance gait obs slots | x/y/yaw at 55/56/60 | pass | pass |

## 5-Question Reboot Check
| Question | Answer |
|----------|--------|
| Where am I? | 15k PPO done; dance gait command slots wired behind `audio.dance_gait` |
| Where am I going? | Hardware checklist on a real duck |
| What's the goal? | Duck bobs on heard beat; dance gait trained off this Mac |
| What have I learned? | Alpha stand must keep x/y/yaw at 0; dance net reads those three |
| What have I done? | Tracker, mapper, overlay, 15k train, ONNX, `dance_gait` encoding |
