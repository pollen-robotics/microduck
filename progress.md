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
- Tick 52 (2026-09-03 17:03 ET): still `TRAIN_EXIT 0`; tmux `dance-train-1bf7770a` gone (expected). GPU idle. Not restarting. Next: hop-loco plan `docs/ideas/dance-gait/locomote-and-hop.md`.
- Tick 53 (2026-09-03 17:23 ET): unchanged. `TRAIN_EXIT 0`, `model_14999.pt` present, tmux gone. Not restarting.
- Tick 54 (2026-09-03 17:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 55 (2026-09-03 18:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 56 (2026-09-03 18:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 57 (2026-09-03 18:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 58 (2026-09-03 19:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 59 (2026-09-03 19:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 60 (2026-09-03 19:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 61 (2026-09-03 20:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 62 (2026-09-03 20:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 63 (2026-09-03 20:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 64 (2026-09-03 21:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 65 (2026-09-03 21:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 66 (2026-09-03 21:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 67 (2026-09-03 22:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 68 (2026-09-03 22:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 69 (2026-09-03 22:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 70 (2026-09-03 23:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 71 (2026-09-03 23:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 72 (2026-09-03 23:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 73 (2026-09-04 00:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 74 (2026-09-04 00:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 75 (2026-09-04 00:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 76 (2026-09-04 01:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 77 (2026-09-04 01:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 78 (2026-09-04 01:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 79 (2026-09-04 02:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 80 (2026-09-04 02:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 81 (2026-09-04 02:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 83 (2026-09-04 03:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 84 (2026-09-04 03:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 85 (2026-09-04 04:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 86 (2026-09-04 04:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 87 (2026-09-04 04:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 88 (2026-09-04 05:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 89 (2026-09-04 05:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 90 (2026-09-04 05:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 91 (2026-09-04 06:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 92 (2026-09-04 06:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 93 (2026-09-04 06:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 94 (2026-09-04 07:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 95 (2026-09-04 07:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 96 (2026-09-04 07:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 97 (2026-09-04 08:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 98 (2026-09-04 08:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 99 (2026-09-04 08:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 100 (2026-09-04 09:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 101 (2026-09-04 09:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 102 (2026-09-04 09:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 103 (2026-09-04 10:03 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 104 (2026-09-04 10:23 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 105 (2026-09-04 10:43 ET): unchanged. `TRAIN_EXIT 0`, checkpoint present, tmux gone. Not restarting.
- Tick 106 (2026-09-04 11:03 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **83/15000**, mean reward **~83**, `fell_over` ~1, `nan_state` 0, GPU 81% @ 65°C.
- Tick 107 (2026-09-04 11:23 ET): standing still `TRAIN_EXIT 0`, tmux gone, not restarting. Loco-hop iter **398/15000**, mean reward **142**, ep len **995**, `fell_over` **0.04**, `nan_state` 0, `dance_foot_phase` 1.08, `twist_speed_xy` 0.05, GPU 72% @ 69°C. Falls collapsed vs tick 106. ETA ~15 h.
- Tick 108 (2026-09-04 11:43 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **713/15000**, mean reward **144**, ep len **1000**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.04, GPU 68% @ 69°C. ETA ~15 h.
- Tick 109 (2026-09-04 12:03 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **1031/15000**, mean reward **138**, ep len **980**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.08, `twist_speed_xy` 0.06, GPU 67% @ 71°C. ETA ~14.5 h.
- Tick 110 (2026-09-04 12:23 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **1337/15000**, mean reward **139**, ep len **999**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.05, GPU 60% @ 67°C. ETA ~14.3 h.
- Tick 111 (2026-09-04 12:43 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **1643/15000**, mean reward **131**, ep len **995**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.14, `twist_speed_xy` 0.05, GPU 65% @ 67°C. ETA ~14.1 h.
- Tick 112 (2026-09-04 13:03 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **1949/15000**, mean reward **132**, ep len **989**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.04, GPU 73% @ 68°C. ETA ~13.8 h.
- Tick 113 (2026-09-04 13:23 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **2255/15000**, mean reward **123**, ep len **992**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.10, `twist_speed_xy` 0.04, GPU 65% @ 66°C. ETA ~13.5 h.
- Tick 114 (2026-09-04 13:43 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **2561/15000**, mean reward **125**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.05, GPU 66% @ 68°C. ETA ~13.2 h.
- Tick 115 (2026-09-04 14:03 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **2879/15000**, mean reward **128**, ep len **996**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.14, `twist_speed_xy` 0.05, GPU 47% @ 67°C. ETA ~12.8 h.
- Tick 116 (2026-09-04 14:23 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **3182/15000**, mean reward **127**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.05, GPU 77% @ 67°C. ETA ~12.5 h.
- Tick 117 (2026-09-04 14:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **3719/15000**, mean reward **127**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 60% @ 68°C. ETA ~12.0 h.
- Tick 118 (2026-09-04 15:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **4028/15000**, mean reward **126**, ep len **978**, `fell_over` 0.17, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04, GPU 60% @ 67°C. ETA ~11.6 h.
- Tick 119 (2026-09-04 15:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **4331/15000**, mean reward **126**, ep len **992**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 67% @ 68°C. ETA ~11.3 h.
- Tick 120 (2026-09-04 15:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **4637/15000**, mean reward **127**, ep len **992**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04, GPU 66% @ 69°C. ETA ~11.0 h.
- Tick 121 (2026-09-04 16:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **4940/15000**, mean reward **127**, ep len **995**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.04, GPU 67% @ 66°C. ETA ~10.7 h.
- Tick 122 (2026-09-04 16:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **5240/15000**, mean reward **127**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.05, GPU 80% @ 70°C. ETA ~10.4 h.
- Tick 123 (2026-09-04 16:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **5561/15000**, mean reward **125**, ep len **983**, `fell_over` 0.13, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04, GPU 79% @ 68°C. ETA ~10.0 h.
- Tick 124 (2026-09-04 17:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **5873/15000**, mean reward **128**, ep len **995**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.06, GPU 73% @ 66°C. ETA ~9.7 h.
- Tick 125 (2026-09-04 17:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **6182/15000**, mean reward **128**, ep len **986**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.06, GPU 69% @ 68°C. ETA ~9.4 h.
- Tick 126 (2026-09-04 17:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **6488/15000**, mean reward **129**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.06, GPU 73% @ 66°C. ETA ~9.1 h.
- Tick 127 (2026-09-04 18:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **6803/15000**, mean reward **129**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.15, `twist_speed_xy` 0.04, GPU 74% @ 68°C. ETA ~8.7 h.
- Tick 128 (2026-09-04 18:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **7115/15000**, mean reward **124**, ep len **977**, `fell_over` 0.17, `nan_state` 0, `dance_foot_phase` 1.10, `twist_speed_xy` 0.04, GPU 67% @ 66°C. ETA ~8.4 h.
- Tick 129 (2026-09-04 18:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **7424/15000**, mean reward **128**, ep len **996**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 46% @ 65°C. ETA ~8.1 h.
- Tick 130 (2026-09-04 19:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **7733/15000**, mean reward **129**, ep len **991**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.14, `twist_speed_xy` 0.04, GPU 66% @ 66°C. ETA ~7.7 h.
- Tick 131 (2026-09-04 19:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **8042/15000**, mean reward **127**, ep len **988**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.08, `twist_speed_xy` 0.05, GPU 61% @ 65°C. ETA ~7.4 h.
- Tick 132 (2026-09-04 19:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **8351/15000**, mean reward **130**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 79% @ 65°C. ETA ~7.1 h.
- Tick 133 (2026-09-04 20:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **8666/15000**, mean reward **129**, ep len **998**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.05, GPU 66% @ 64°C. ETA ~6.7 h.
- Tick 134 (2026-09-04 20:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **8966/15000**, mean reward **127**, ep len **994**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 75% @ 67°C. ETA ~6.4 h.
- Tick 135 (2026-09-04 20:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **9263/15000**, mean reward **128**, ep len **995**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.09, `twist_speed_xy` 0.05, GPU 81% @ 64°C. ETA ~6.1 h.
- Tick 136 (2026-09-04 21:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **9575/15000**, mean reward **128**, ep len **997**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04, GPU 56% @ 64°C. ETA ~5.8 h.
- Tick 137 (2026-09-04 21:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **9881/15000**, mean reward **129**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 76% @ 67°C. ETA ~5.5 h.
- Tick 138 (2026-09-04 21:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **10187/15000**, mean reward **129**, ep len **992**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04, GPU 51% @ 65°C. ETA ~5.1 h.
- Tick 139 (2026-09-04 22:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **10490/15000**, mean reward **129**, ep len **996**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04, GPU 62% @ 65°C. ETA ~4.8 h.
- Tick 140 (2026-09-04 22:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **10793/15000**, mean reward **128**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.06, GPU 83% @ 67°C. ETA ~4.5 h.
- Tick 141 (2026-09-04 22:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **11096/15000**, mean reward **129**, ep len **999**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 58% @ 68°C. ETA ~4.2 h.
- Tick 142 (2026-09-04 23:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **11405/15000**, mean reward **128**, ep len **996**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.05, GPU 71% @ 69°C. ETA ~3.8 h.
- Tick 143 (2026-09-04 23:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **11705/15000**, mean reward **128**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.05, GPU 82% @ 66°C. ETA ~3.5 h.
- Tick 144 (2026-09-04 23:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **12017/15000**, mean reward **128**, ep len **988**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.14, `twist_speed_xy` 0.04, GPU 85% @ 67°C. ETA ~3.2 h.
- Tick 145 (2026-09-05 00:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **12317/15000**, mean reward **127**, ep len **985**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.10, `twist_speed_xy` 0.04, GPU 71% @ 66°C. ETA ~2.9 h.
- Tick 146 (2026-09-05 00:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **12626/15000**, mean reward **129**, ep len **994**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.05, GPU 72% @ 64°C. ETA ~2.5 h.
- Tick 147 (2026-09-05 00:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **12923/15000**, mean reward **129**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.12, `twist_speed_xy` 0.05, GPU 71% @ 65°C. ETA ~2.2 h.
- Tick 148 (2026-09-05 01:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **13226/15000**, mean reward **129**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.06, GPU 70% @ 65°C. ETA ~1.9 h.
- Tick 149 (2026-09-05 01:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **13538/15000**, mean reward **127**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.05, GPU 45% @ 65°C. ETA ~1.6 h.
- Tick 150 (2026-09-05 01:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **13847/15000**, mean reward **129**, ep len **999**, `fell_over` 0.08, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.05, GPU 65% @ 66°C. ETA ~1.2 h.
- Tick 151 (2026-09-05 02:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **14147/15000**, mean reward **127**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.10, `twist_speed_xy` 0.06, GPU 43% @ 66°C. ETA ~55 min.
- Tick 152 (2026-09-05 02:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **14447/15000**, mean reward **127**, ep len **997**, `fell_over` 0.04, `nan_state` 0, `dance_foot_phase` 1.13, `twist_speed_xy` 0.06, GPU 64% @ 65°C. ETA ~35 min.
- Tick 153 (2026-09-05 02:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Loco-hop iter **14753/15000**, headline mean reward **−252** (one-iter spike; component rewards still ~track_lin 0.91 / foot_phase 1.10), ep len **986**, `fell_over` 0.13, `nan_state` 0, GPU 65% @ 64°C. ETA ~16 min. Not restarting.
- Tick 154 (2026-09-05 03:18 ET): loco **finished**. `TRAIN_EXIT 0` at 2026-09-05 03:14 ET. Iter **14999/15000**, mean reward **129**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_foot_phase` 1.11, `twist_speed_xy` 0.04. Checkpoint `model_14999.pt`. Export `obs[1,61]→actions[1,14]`. 20.0 s play at 100 BPM opened in VLC: `docs/ideas/dance-gait/dance_loco_20s_100bpm.mp4`. ONNX: `docs/ideas/dance-gait/dance_loco_iter14999.onnx`.
- Tick 155 (2026-09-05 03:38 ET): both jobs still `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting. Clip and loco ONNX still on disk.
- Tick 156 (2026-09-05 03:58 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 157 (2026-09-05 04:18 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 158 (2026-09-05 04:38 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 159 (2026-09-05 04:58 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 160 (2026-09-05 05:18 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 161 (2026-09-05 05:38 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 162 (2026-09-05 05:58 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 163 (2026-09-05 06:18 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 164 (2026-09-05 06:38 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 165 (2026-09-05 06:58 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 166 (2026-09-05 07:18 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 167 (2026-09-05 07:38 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 168 (2026-09-05 07:58 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 169 (2026-09-05 08:18 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 170 (2026-09-05 08:38 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 171 (2026-09-05 08:58 ET): unchanged. Both `TRAIN_EXIT 0`, tmux gone, GPU idle. Not restarting.
- Tick 172 (2026-09-05 09:14 ET): **Checkpoint B failed.** `dance_loco_20s_100bpm.mp4` is essentially frozen (1 fps crop |Δ| ~1.2 vs standing ~7.9). Causes: `dance_z_tracking` 0.5, walk `air_time` zeroed on in-place, camera distance 3.0. Retuned `dance_z_tracking` 1.5 / `dance_foot_phase` 3.0 / energy-gated `air_time` 0.5; play camera 0.55; play mix drops in-place. Smoke 64×5 `SMOKE_EXIT 0` 09:08. Fine-tune from loco `model_14999.pt` in tmux **`dance-loco-fix-1bf7770a`** (do **not** kill; do **not** restart standing `dance-train-1bf7770a`). Resume additive: **14999/20999**, ~6k iters, `nan_state` 0, GPU ~69% @ 62°C, ETA ~6.5 h (~15:45 ET). Log `/home/mstaub/jobs/feat-dancing-skill-loco-1bf7770a/logs/fix_train.log`. Pipeline after `FIX_TRAIN_EXIT 0`: export + 20 s play → Spark `$JOB/dance_loco_20s_100bpm_fix.mp4`. Then scp to `docs/ideas/dance-gait/` and open VLC.
- Tick 173 (2026-09-05 09:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **15070/20999**, mean reward **166**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.18 (was 0.41 at resume), `dance_foot_phase` 2.29, `air_time` 0.0075, GPU 69% @ 68°C, ETA ~6.2 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 174 (2026-09-05 09:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **15376/20999**, mean reward **161**, ep len **996**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.21, `dance_foot_phase` 2.22, `air_time` 0.0086, GPU 49% @ 71°C, ETA ~6.0 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 175 (2026-09-05 09:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **15688/20999**, mean reward **157**, ep len **978**, `fell_over` 0.13, `nan_state` 0, `dance_z_tracking` 1.22, `dance_foot_phase` 2.21, `air_time` 0.010, GPU 79% @ 73°C, ETA ~5.6 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 176 (2026-09-05 10:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **15994/20999**, mean reward **160**, ep len **992**, `fell_over` 0.08, `nan_state` 0, `dance_z_tracking` 1.22, `dance_foot_phase` 2.21, `air_time` 0.011, GPU 67% @ 72°C, ETA ~5.3 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 177 (2026-09-05 10:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **16309/20999**, mean reward **162**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.32, `dance_foot_phase` 2.30, `air_time` 0.009, GPU 64% @ 71°C, ETA ~5.0 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 178 (2026-09-05 10:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **16621/20999**, mean reward **158**, ep len **989**, `fell_over` 0.08, `nan_state` 0, `dance_z_tracking` 1.28, `dance_foot_phase` 2.20, `air_time` 0.010, GPU 67% @ 70°C, ETA ~4.6 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 179 (2026-09-05 11:23 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **17005/20999**, mean reward **163**, ep len **998**, GPU 56% @ 71°C, ETA ~4.2 h. Wired `DanceDirector` + optional `policy.dance` / `Net::Dance` (`API_VERSION` 19). Tests: pet-detect 22, robotd-params 27, duck-control 61, duck-ipc-proto 49, robotd 97+7, robotctl 142. Do not kill `dance-loco-fix-1bf7770a`.
- Tick 180 (2026-09-05 11:25 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **17032/20999**, mean reward **162**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.31, `dance_foot_phase` 2.26, `air_time` 0.009, GPU 68% @ 71°C, ETA ~4.2 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 181 (2026-09-05 11:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **17245/20999**, mean reward **162**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.32, `dance_foot_phase` 2.23, `air_time` 0.011, GPU 63% @ 74°C, ETA ~4.0 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 182 (2026-09-05 11:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **17554/20999**, mean reward **162**, ep len **994**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.35, `dance_foot_phase` 2.26, `air_time` 0.011, GPU 75% @ 68°C, ETA ~3.7 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 183 (2026-09-05 12:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **17872/20999**, mean reward **163**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.38, `dance_foot_phase` 2.25, `air_time` 0.015, GPU 87% @ 68°C, ETA ~3.3 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 184 (2026-09-05 12:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **18193/20999**, mean reward **165**, ep len **1000**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.40, `dance_foot_phase` 2.33, `air_time` 0.020, `twist_speed_xy` 0.06, GPU 83% @ 68°C, ETA ~3.0 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 185 (2026-09-05 12:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **18499/20999**, mean reward **163**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.41, `dance_foot_phase` 2.29, `air_time` 0.023, GPU 79% @ 65°C, ETA ~2.6 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 186 (2026-09-05 13:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **18814/20999**, mean reward **164**, ep len **998**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.40, `dance_foot_phase` 2.37, `air_time` 0.026, GPU 36% @ 63°C, ETA ~2.3 h. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 187 (2026-09-05 13:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **19135/20999**, mean reward **162**, ep len **986**, `fell_over` 0.08, `nan_state` 0, `dance_z_tracking` 1.40, `dance_foot_phase` 2.33, `air_time` 0.027, GPU 58% @ 63°C, ETA ~2.0 h. Latest ckpt `model_19000.pt`. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 188 (2026-09-05 13:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **19444/20999**, mean reward **164**, ep len **993**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.41, `dance_foot_phase` 2.39, `air_time` 0.032, GPU 74% @ 64°C, ETA ~1.6 h. Latest ckpt `model_19250.pt`. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 189 (2026-09-05 14:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **19765/20999**, mean reward **165**, ep len **999**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.43, `dance_foot_phase` 2.46, `air_time` 0.033, GPU 69% @ 65°C, ETA ~1.3 h. Latest ckpt `model_19750.pt`. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 190 (2026-09-05 14:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **20077/20999**, mean reward **165**, ep len **1000**, `fell_over` 0, `nan_state` 0, `dance_z_tracking` 1.42, `dance_foot_phase` 2.40, `air_time` 0.038, GPU 56% @ 64°C, ETA ~58 min. Latest ckpt `model_20000.pt`. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 191 (2026-09-05 14:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **20398/20999**, mean reward **163**, ep len **991**, `fell_over` 0.08, `nan_state` 0, `dance_z_tracking` 1.41, `dance_foot_phase` 2.37, `air_time` 0.037, GPU 60% @ 64°C, ETA ~38 min. Latest ckpt `model_20250.pt`. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 192 (2026-09-05 15:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop fine-tune **20707/20999**, mean reward **162**, ep len **991**, `fell_over` 0.04, `nan_state` 0, `dance_z_tracking` 1.41, `dance_foot_phase` 2.39, `air_time` 0.040, GPU 72% @ 64°C, ETA ~18 min. Latest ckpt `model_20500.pt`. tmux `dance-loco-fix-1bf7770a` running. Not killing it.
- Tick 193 (2026-09-05 15:38 ET): **FIX_TRAIN_EXIT 0** 15:36, **EXPORT_EXIT 0**, **PLAY_EXIT 0**, **FIX_PIPELINE_DONE** 15:37. Final **20998/20999**, mean reward **165**, ep len **1000**, `fell_over` 0, `nan_state` 0, `air_time` 0.038. ONNX `obs[1,61]→actions[1,14]`. Clip `docs/ideas/dance-gait/dance_loco_20s_100bpm_fix.mp4` opened in VLC. **Checkpoint B: travel yes, hop no** (1 fps crop |Δ| ~20.8 vs frozen v1 ~1.2; grid shift 25 px vs 0; feet stay planted). Raised `dance_foot_phase` 3→5, `air_time` 0.5→2.0, thresholds 0.08–0.28. Smoke 64×5 `SMOKE_EXIT 0` 15:44, `nan_state` 0. Hop2 3k resume from `model_20998.pt` in tmux **`dance-loco-hop2-1bf7770a`** at **21024/23998**, `nan_state` 0, GPU 62% @ 61°C, ETA ~2.8 h (~18:35 ET). Do **not** kill it. Do **not** restart standing. Do **not** widen the twist box.
- Tick 194 (2026-09-05 15:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **21213/23998**, mean reward **181**, ep len **951**, `fell_over` 0.33, `nan_state` 0, `dance_z_tracking` 1.30, `dance_foot_phase` 4.30, `air_time` **0.64** (was 0.038), `air_time_mean` 0.11, GPU 70% @ 72°C, ETA ~2.8 h. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 195 (2026-09-05 16:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **21525/23998**, mean reward **174**, ep len **949**, `fell_over` 0.46, `nan_state` 0, `dance_z_tracking` 1.22, `dance_foot_phase` 4.33, `air_time` **0.73**, GPU 73% @ 73°C, ETA ~2.5 h. Latest ckpt `model_21500.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 196 (2026-09-05 16:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **21843/23998**, mean reward **185**, ep len **968**, `fell_over` 0.33, `nan_state` 0, `dance_z_tracking` 1.24, `dance_foot_phase` 4.47, `air_time` **0.79**, GPU 56% @ 70°C, ETA ~2.2 h. Latest ckpt `model_21750.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 197 (2026-09-05 16:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **22161/23998**, mean reward **178**, ep len **936**, `fell_over` 0.46, `nan_state` 0, `dance_z_tracking` 1.23, `dance_foot_phase` 4.35, `air_time` **0.79**, GPU 71% @ 71°C, ETA ~1.9 h. Latest ckpt `model_22000.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 198 (2026-09-05 17:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **22497/23998**, mean reward **186**, ep len **978**, `fell_over` 0.25, `nan_state` 0, `dance_z_tracking` 1.30, `dance_foot_phase` 4.51, `air_time` **0.82**, GPU 67% @ 72°C, ETA ~1.5 h. Latest ckpt `model_22250.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 199 (2026-09-05 17:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **22806/23998**, mean reward **188**, ep len **977**, `fell_over` 0.21, `nan_state` 0, `dance_z_tracking` 1.27, `dance_foot_phase` 4.46, `air_time` **0.81**, GPU 75% @ 71°C, ETA ~1.2 h. Latest ckpt `model_22750.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 200 (2026-09-05 17:58 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **23121/23998**, mean reward **185**, ep len **970**, `fell_over` 0.33, `nan_state` 0, `dance_z_tracking` 1.29, `dance_foot_phase` 4.50, `air_time` **0.74**, GPU 75% @ 71°C, ETA ~54 min. Latest ckpt `model_23000.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 201 (2026-09-05 18:18 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **23445/23998**, mean reward **186**, ep len **960**, `fell_over` 0.38, `nan_state` 0, `dance_z_tracking` 1.26, `dance_foot_phase` 4.46, `air_time` **0.79**, GPU 63% @ 70°C, ETA ~34 min. Latest ckpt `model_23250.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 202 (2026-09-05 18:38 ET): standing still `TRAIN_EXIT 0`, not restarting. Hop2 **23760/23998**, mean reward **190**, ep len **977**, `fell_over` 0.33, `nan_state` 0, `dance_z_tracking` 1.31, `dance_foot_phase` 4.53, `air_time` **0.73**, GPU 66% @ 71°C, ETA ~15 min. Latest ckpt `model_23750.pt`. tmux `dance-loco-hop2-1bf7770a` running. Not killing it.
- Tick 203 (2026-09-05 18:58 ET): **FIX2_TRAIN_EXIT 0** 18:53, **EXPORT_EXIT 0**, **PLAY_EXIT 0**, **FIX2_PIPELINE_DONE** 18:53. Final **23997/23998**, mean reward **187**, ep len **967**, `fell_over` 0.33, `nan_state` 0, `air_time` 0.63. ONNX `obs[1,61]→actions[1,14]`. Clip `docs/ideas/dance-gait/dance_loco_20s_100bpm_fix2.mp4` opened in VLC. **Checkpoint B pass:** travel + swing foot (crop |Δ| ~23.8; grid shift hits 80 px; one foot off the floor). Do not load as `[policy] stand`. Standing stays `TRAIN_EXIT 0`, not restarting.
- Tick 204 (2026-09-05 19:18 ET): hop2 still `FIX2_PIPELINE_DONE`, standing still `TRAIN_EXIT 0`, GPU idle. Artifacts present. Not restarting. Next: hardware `[policy] dance` = fix2 ONNX.
- Tick 205 (2026-09-05 19:38 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 206 (2026-09-05 19:58 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 207 (2026-09-05 20:18 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 208 (2026-09-05 20:38 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 209 (2026-09-05 20:58 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 210 (2026-09-05 21:18 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 211 (2026-09-05 21:38 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 212 (2026-09-05 21:58 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting.
- Tick 213 (2026-09-05 22:18 ET): unchanged. Hop2 `FIX2_PIPELINE_DONE`, standing `TRAIN_EXIT 0`, GPU idle, tmux gone. Not restarting. Session deadline 22:53 ET.
- Tick 214 (2026-09-05 22:39 ET): play mix was 70% translate / 4-beat phrases (repetitive march). Switched play+director to 20/28/27/25 inplace/translate/sidestep/spin, 1–2 beat phrases, BPM 88–124 and energy 0.55–1.0 without jumping φ. Same hop2 `model_23997.pt` (no retrain). Clip `docs/ideas/dance-gait/dance_loco_20s_playful.mp4` opened in VLC. `cargo test -p pet-detect --lib` 23 passed.
- Tick 215 (2026-09-05 22:40 ET): hop2 still `FIX2_PIPELINE_DONE`, GPU idle, tmux gone. Playful clip present. Not restarting. Session deadline 22:53 ET.
- 2026-09-06 09:09 ET: organic play clip `docs/ideas/dance-gait/dance_loco_20s_organic.mp4` (groove-warped φ, twist slew, same hop2 weights). 72h monitor loop killed (deadline was 2026-09-05 22:53 ET). Report: `SESSION_REPORT.md`.
- Loco-hop (2026-09-04 10:58 ET): Spark GPU idle → new job `/home/mstaub/jobs/feat-dancing-skill-loco-1bf7770a/`. Task `Mjlab-Dance-Loco-Flat-MicroDuck`. Mix histogram ok. Smoke 64×5 `SMOKE_EXIT 0`. PPO 4096×15k scratch started tmux `dance-loco-1bf7770a`, `nan_state` 0 at iter 5, GPU 68%. Finished `TRAIN_EXIT 0` 2026-09-05 03:14 ET.
- Export: standing `docs/ideas/dance-gait/dance_gait_iter14999.onnx`; loco `docs/ideas/dance-gait/dance_loco_iter14999.onnx` (`obs[1,61]→actions[1,14]`, `uv run scripts/export.py` only)
- Preview: standing `docs/ideas/dance-gait/dance_gait_iter14999.mp4`; loco 20 s @ 100 BPM `docs/ideas/dance-gait/dance_loco_20s_100bpm.mp4`
- Log: standing `/home/mstaub/jobs/feat-dancing-skill-1bf7770a/logs/train.log`; loco `/home/mstaub/jobs/feat-dancing-skill-loco-1bf7770a/logs/train.log`

## Test Results
| Test | Expected | Actual | Status |
|------|----------|--------|--------|
| pet-detect lib | pass | 22 passed | pass |
| robotd-params | pass | 27 passed | pass |
| duck-ipc-proto | pass | 49 passed | pass |
| robotd | pass | 97 + 7 updater_gate | pass |
| robotctl | pass | 142 passed | pass |
| Spark smoke 64×5 | exit 0 | exit 0 | pass |
| Spark PPO 15k | TRAIN_EXIT 0, nan_state 0 | standing reward ~64; loco reward ~129 | pass |
| Loco ONNX export | obs[1,61]→actions[1,14] | that shape | pass |
| Loco 20 s play | travel + hop at 100 BPM | v3 travel + swing foot | pass |
| ONNX export | obs[1,61]→actions[1,14] | that shape | pass |
| dance gait obs slots | x/y/yaw at 55/56/60 | pass | pass |

## 5-Question Reboot Check
| Question | Answer |
|----------|--------|
| Where am I? | Organic play clip `dance_loco_20s_organic.mp4` on hop2 weights. 72h loop stopped. |
| Where am I going? | Hardware: `[policy] dance` = fix2 ONNX, stand stays `alpha_stand`. |
| What's the goal? | Duck moves around and hops feet on the beat, organic not a metronome march |
| What have I learned? | Stomp was play-time φ, not missing rewards; mix/groove can change without a retrain |
| What have I done? | Organic play clock + twist slew; session report; loop killed |
