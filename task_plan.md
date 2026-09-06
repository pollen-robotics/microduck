# Task Plan: Dance to music

## Goal
A standing Microduck bobs on a heard beat inside the stand pose box, with no obs-width change; a dedicated dance gait is trained on Spark.

## Current Phase
Phase 5 — closed: 72h loop stopped 2026-09-06; hardware checklist still needs a duck

## Phases

### Phase 0: Session + GPU routing
- [x] Read idea + skills
- [x] Probe Spark / 5090
- [x] Choose Spark
- **Status:** complete

### Phase 1: Beat tracker + mapper (TDD)
- [x] `pet-detect` BeatState + spectral-flux tracker tests
- [x] Mapper waveform + overlay-gate tests
- **Status:** complete

### Phase 2: Mic worker without pet ONNX
- [x] Optional detector; beat frames; EOF unlock; shared PCM test double
- [x] `[audio] listen` + `[audio] dance` params/registry
- **Status:** complete

### Phase 3: robotd overlay + attended client
- [x] Tick-local overlay (chorale-sway style)
- [x] Optional `beat` on `robot.state` (API_VERSION 18)
- [x] `dance-pose` example for phase 0B
- **Status:** complete

### Phase 4: Remote dance-gait training
- [x] Clone microduck_rl on Spark in an isolated job dir
- [x] New env: beat phase/energy in unbound command slots
- [x] 64-env smoke (exit 0)
- [x] 4096-env PPO to 15k iters (`TRAIN_EXIT 0`, 2026-09-03 15:14 ET)
- [x] Export ONNX (`obs[1,61] → actions[1,14]`) via `scripts/export.py`
- **Status:** complete

### Phase 5: 72h loop
- [x] Monitor training through 15k
- [x] Wire dance gait command slots (`audio.dance_gait`) without touching alpha stand zeros
- [x] Stop 72h loop after deadline; write `SESSION_REPORT.md`
- [ ] Hardware checklist (speaker in the room; own quack; dead arecord) — needs a duck
- **Status:** complete except hardware (needs a duck)

## Decisions Made
| Decision | Rationale |
|----------|-----------|
| `[audio] listen` + `[audio] dance` | One capture reason, separate mapper opt-in; pet ONNX not required |
| Tracker in `pet-detect`, overlay in `robotd` | PCM already lives in the pet worker; overlay must see twist_age / pose.active |
| Beat in unbound command slots for phase-3 policy | Width stays 61; stand policy still sees zeros there |
| Train on Spark, not 5090 first | 88 GiB free vs 5090 `nvidia-smi` missing in this WSL login |
| No commits unless asked | User rule |

## Errors Encountered
| Error | Attempt | Resolution |
|-------|---------|------------|
| windows1 user `mikestaub` | 1 | Real user is `micro@100.77.103.59` |
