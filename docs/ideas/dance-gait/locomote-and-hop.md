# Implementation plan: locomoting hop-dance

Status: **in progress** · Date: 2026-09-05 · v1 15k + clip **failed visual QA** (frozen). 6k reward-retune fine-tune running (`dance-loco-fix-1bf7770a`, 14999/20999).

The committed dance gait (`Mjlab-Dance-Flat-MicroDuck`, `dance_gait_iter14999.onnx`)
is a **standing crouch**. Twist is trained at `(0,0,0)`, walking rewards were
stripped, and at runtime the net is loaded as `[policy] stand`. Any real
`|twist|` above the 0.05 standing threshold switches to `alpha_walking` and
drops the dance weights. A tracker quarantine on `|gated.twist| > 1e-3` would
also kill the lock the moment we started stepping.

This plan makes the duck **move around on the beat**: small random travel,
weight hopping foot-to-foot, still `obs[1,61] → actions[1,14]`, still no
`robot.dance` RPC, still one `arecord`. Train on Spark, not this Mac.

## Done looks like

A 20 s MuJoCo clip at 100 BPM where, without a pad:

1. The trunk translates and/or turns (not a stationary bob).
2. Support alternates: left planted on one half-cycle, right on the other.
3. Motion changes character every four beats (not a straight-line march).
4. Falls stay rare (`fell_over` on the order of the standing run, not a collapse).

On a duck, same four, plus the existing checklist: speaker moves it, own quack
does not, dead `arecord` stops it, pad / skill / `fallen` still win.

## Why the current net cannot do this

| Fact | Consequence |
|---|---|
| `lin_vel_* = (0,0)`, `rel_standing_envs = 1` | Policy never saw a travel command |
| `air_time` / `foot_clearance` popped | No incentive to pick a foot up |
| Loaded as **stand** | `|twist| > 0.05` selects **walk** |
| Quarantine if `|gated.twist| > 1e-3` | Injecting travel into the pad twist would unlock the tracker |

Do not stuff travel into `alpha_stand` or `alpha_walking`. This is a new
network, same observation layout.

## Architecture (locked)

```
arecord 16 kHz mono
  └ beat tracker → BeatState { locked, bpm, phase, energy, t, onset_seq }
        ├ body_x,y,yaw  = sin/cos/energy     (unchanged)
        └ twist         = DanceDirector        (new, tick-local)
              → dance ONNX  (new policy slot, Net::Dance)
```

**Dance director** (in-process, like the mapper): while the overlay gate
applies **and** `policy.dance` is loaded, overwrite **this tick's
`PolicyCommand.twist`** with a small sampled `(vx, vy, vyaw)`. Do **not**
write `robot.move`. Do **not** touch pad `twist_ema` or `gated.twist`. Do
**not** set `pose.active`. Resample on **4-beat phrases** (`onset_seq / 4`
increases), not on a wall-clock independent of the music. First tick of a
new lock resamples immediately. Energy 0 snaps twist to 0 without waiting
for a phrase boundary. Overlay drop resets the director (next lock is a
new phrase).

Hold the current phrase twist in director state. Toward that target, apply
the existing `cmd_alpha` on a **separate** `dance_twist_ema` so a phrase
change is a ramp the policy can track. On overlay drop, zero that EMA; it
must never leak into pad `twist_ema`.

**Net selection:** optional `[policy] dance` (no release default; unset =
`None`). `Controller::step` takes `dance_active: bool` **before**
`will_stand`. If `dance_active` and `Sit::Up`, infer `Net::Dance` with the
director twist and beat body slots, even when `|director twist| > 0.05`.
Skills, sit, limp-fall, pad (`twist_fresh`), leftover `pose.active` still
outrank dance — same `OverlayGate` as today. `body_active` stays
`snapshot.pose.active` (false while overlay applies), so the sit/stand
path must **not** zero director twist.

**Scale / gain:** explicit `Net::Dance` arm: `standing_action_scale` (1.0,
what v1 was trained with) and **walk** `gain`, not `standing_gain_ratio`
0.8. Do not let Dance fall through `_ if standing_tuned` (that is 0.8) or
the walk `_` arm (that is scale 0.9).

**Flags:** `[audio] dance` still starts capture + mapper. `[audio]
dance_gait` still means "fill `body_x/y/yaw` while the **stand** net is
running" (the v1 ONNX-as-stand path). `Net::Dance` **always** gets
`map_beat_gait`, ignoring `dance_gait` — otherwise an operator can run the
loco net with zeros in the slots it was trained on. Do not load the loco
ONNX as `policy.stand` or `policy.walk`.

**Z overlay stays:** `[audio] dance` without `policy.dance` still bobs
`alpha_stand` in place. `OverlayGate.overlay_applies` requires
`has_standing || has_dance`. Crouch path needs `has_standing`. Loco path
needs `has_dance`. Missing dance slot (`None`) ⇒ crouch overlay, no
crash. A **configured** dance path that fails to load fails the whole
`Policy::load`, same as a missing kick — unhealthy, roll it back.

**Quarantine / walking flag:** `|gated.twist|` and `snapshot.twist_age`
from clients only. Director twist is invisible to the tracker, to
`will_stand`, and to `twist_ema`.

### Command box (training and director — one table)

Keep BPM **60–140**. Travel stays in a *dance* box, not the walk box:

| Slot | Range | Notes |
|---|---|---|
| `vx` | −0.10 … +0.20 m/s | Prefer forward; allow a little reverse |
| `vy` | −0.12 … +0.12 m/s | Sidestep |
| `vyaw` | −0.8 … +0.8 rad/s | Slow spin |
| energy 0 | twist = 0 | Idle, same as today |

Phrase mix when energy > 0 (resample at phrase start). **The Python env
duplicates this table**; a comment in both files says so. Do not sample
uniform `lin_vel_*` independently — that over-represents reverse+spin and
will not match the robot.

- 15% in-place (`twist = 0`) — hop without travelling
- 55% translate: `vx` in box, `vy` in box, `vyaw` in **±0.25** (small)
- 20% sidestep-only: `vx = 0`, `vy` in box, `vyaw` in ±0.25
- 10% spin-in-place: `vx = 0`, `vy = 0`, `|vyaw|` in **0.4 … 0.8** (sign random)

Idle in train: keep `ZERO_ENERGY_PROB = 0.3`. That is on top of the 15%
in-place phrases, not instead of them. Set `rel_standing_envs = 0` so the
velocity command term does not add a second standing fraction.

Play cfg: energy 0.55–1.0, BPM 88–124 (resample every 2.4–4 s, **do not
jump φ**), mix 20/28/27/25 inplace/translate/sidestep/spin, phrase length
**1–2 beats**. Drop `push_robot` in play only. The hop2 policy tracks
twist, so this mix does not need a retrain.

Phrase length on the robot is **1–2 beats** (sampled per phrase):
- Runtime: resample when `onset_seq - phrase_start >= phrase_len`
- Train: 2–4 wraps (unchanged for the next fine-tune)

At 100 BPM that is 2.4 s; at 60 BPM 4 s; at 140 BPM ~1.7 s.

### Hop (not a jump)

"Hop between feet" means **alternate support on the beat**, not an aerial
jump. XL330s and a 12 s standing policy do not make a reliable ballistic
hop; asking for flight first will teach falling.

Phase lock (same φ as `body_x/y`):

- `φ ∈ [0, 0.5)`: left stance, right swing
- `φ ∈ [0.5, 1)`: right stance, left swing
- Double support in a window `|φ| < 0.08` or `|φ − 0.5| < 0.08` is allowed
  (no penalty for both contacts there)
- Both feet airborne is **penalized** in v1
- At energy 0: both feet down, no step incentive

Keep a reduced crouch: trunk z still dips on onset (`dance_z_tracking`),
but weaker than today so it does not fight the step.

### Starting reward weights

Checkpoint B retunes these; do not start from walk defaults blindly.
Confirm foot body names on Spark from the walk `air_time` `SceneEntityCfg`
(`left_foot` / `right_foot` sites exist in this tree's kinematics).

| Term | Weight | Role |
|---|---|---|
| `dance_foot_phase` | +3.0 | Stance foot in contact, swing foot light; skip the swap window. Raised after Checkpoint B (v1 clip did not hop). |
| `dance_both_airborne` | −2.0 | Flight forbidden |
| `track_linear_velocity` | +1.0 | Against **director** twist |
| `track_angular_velocity` | +0.5 | Against director `vyaw` |
| `air_time` | +0.5 | Energy-gated; does **not** require |twist| (in-place hop must count). 0.05–0.22 s |
| `foot_slip` | −0.4 | Stance should not skate |
| `dance_z_tracking` | +1.5 | Was 0.5 in v1 (clip had no bob). Not 2.0 — leave room for steps. |
| upright / action_rate / similar | keep | From the velocity base |

Do **not** restore `foot_clearance` / `foot_swing_height` at walk weight —
those want a commuting gait.

`dance_foot_phase` sketch (energy > 0, outside the swap window):

```
want_left  = φ < 0.5
left_ok    = want_left  ? contact(left)  : ~contact(left)
right_ok   = want_left  ? ~contact(right) : contact(right)
reward     = 0.5 * (left_ok + right_ok)
```

At energy 0: reward both contacts, no swing incentive.

## Locked decisions (do not re-litigate)

1. **Scratch train.** `model_14999` saw twist ≡ 0; its baked obs normalizer
   has degenerate twist stats, so a non-zero command is immediately OOD.
   Fine-tune is an optional experiment only; abandon it if smoke `fell_over`
   explodes or feet never lift. Default job is from scratch.
2. **Director at runtime**, not baked into the policy. Phrase mix can change
   without a retrain. The policy must **track** the twist it is given.
3. **Variable 1–2 beat phrases** on the robot / play clip. Train still
   uses 2–4. Mix table on the robot is the livelier play table; the
   policy tracks twist so this does not need a retrain.
4. **`policy.dance` has no release default.** Operator points a path.
5. **No `robot.dance` RPC.** No audio in the 61-D obs. No training on this Mac.
6. **Bigger travel later.** Raise the box only after Checkpoint B is a hop,
   not a walk clone.
7. **Wire `SubscribeResult.dance`** (filename, `skip_serializing_if` none)
   and bump `API_VERSION` to 19. `RobotState.policy` is already a free
   string; the tick label is `"dance"`.
8. **`heading_command = False`.** `vyaw` is explicit.
9. **`symmetry_cfg = None`.** Phase-locked feet are not left/right symmetric
   at a given φ.

## Not in this plan

- Audio in the 61-D observation
- `robot.dance` RPC or a new daemon
- Training on this Mac
- Replacing `alpha_walking` for ordinary pad drive
- Aerial / jumping curriculum
- Group chorale sync
- Hardware ToF "don't dance off the table" (operator puts it on the floor)
- Shipping the ONNX in the release bundle (operator path is enough)

## Tasks

### Task 1 — Director + gate, tests only

**Description:** `DanceDirector` samples phrase twists from the mix table
and returns them only when the caller passes `applies = true`. It does not
know about pads or pose — Task 2 owns the gate. Seeded RNG so tests are
deterministic.

```rust
pub struct DanceDirector { /* rng, phrase_twist, last_phrase */ }
impl DanceDirector {
    pub fn new(seed: u64) -> Self;
    pub fn tick(&mut self, beat: &BeatState, applies: bool) -> [f64; 3];
    pub fn reset(&mut self);
}
```

**Acceptance:**
- [x] Phrase RNG is deterministic under a seed; 10k samples match the mix
      table within a few points (χ² / histogram, not one lucky draw)
- [x] Unit test: `applies = false` ⇒ `[0,0,0]`, and a later `true` starts a
      fresh phrase
- [x] Unit test: resample lands when `onset_seq / 4` increases, not on
      wall-clock; first lock tick resamples
- [x] Unit test: `energy == 0` ⇒ twist 0 even mid-phrase
- [x] Samples stay inside the command box

**Verification:** `cargo test -p pet-detect --lib`

**Files:** `pet-detect/src/director.rs` (new), `pet-detect/src/lib.rs`

**Depends:** none · **Scope:** S

### Task 2 — `Net::Dance` slot

**Description:** Optional `policy.dance` path through params, load, infer,
and the scheduler. If overlay applies and the session exists,
`dance_active = true` and `Net::Dance` with director twist + beat body
slots. `overlay_applies` is `has_standing || has_dance`. Crouch overlay
unchanged when dance is `None`.

Must-touch (registry completeness will fail if any is skipped):
`PolicyParams.dance`, `ResolvedPolicy` (`path(&self.dance, None)`),
registry key, `PolicyPaths` / `Policy` / `Net::Dance` / `has_dance()`,
`Controller::step(..., dance_active)`, scale/gain arm, `PolicyNames` +
`SubscribeResult.dance`, `API_VERSION` 19, `robotctl` configure + monitor,
`deploy/robotd.toml` comments, `robotd/src/main.rs` (director EMA,
`command.twist` overwrite **after** pad EMA, quarantine still on
`gated.twist`).

**Acceptance:**
- [x] Unset `policy.dance` ⇒ overlay falls back to stand crouch, no crash
- [x] Configured unloadable dance path ⇒ `Policy::load` fails (unhealthy)
- [x] `|director twist| > 0.05` still infers dance, not walk
- [x] Pad-fresh twist infers walk (or stand), never dance
- [x] Skill / sit / limp-fall still take the existing nets
- [x] `Net::Dance` always fills gait body slots; `audio.dance_gait` without
      `policy.dance` still does not stuff travel into stand
- [x] `OverlayGate` quarantine still uses client twist only
- [x] `body_active` / `pose.active` still zeros twist and forces stand; it
      cannot be true on the same tick as `dance_active`
- [x] Tick label `"dance"`; subscribe lists the dance filename when set

**Verification:** `cargo test -p duck-control --lib`,
`cargo test -p robotd-params --lib`, `cargo test -p duck-ipc-proto --lib`,
`cargo test -p robotd --tests`, `cargo test -p robotctl --lib`

**Files:** listed above

**Depends:** Task 1 · **Scope:** M

### Checkpoint A

- [ ] Pad still walks on `alpha_walking` (needs a duck)
- [x] In-place stand bob still works without a dance ONNX (overlay on `has_standing`)
- [x] No quarantine from director twist (`gated.twist` only)
- [x] `cargo test` green on this Mac (no ONNX Runtime required beyond what
      existing tests already assume)

### Task 3 — Spark env: travel + foot phase

**Description:** New task id `Mjlab-Dance-Loco-Flat-MicroDuck` (do not
silently replace the standing recipe). Copy `microduck_dance_env_cfg.py`.
Custom twist command that **implements the mix table** and resamples every
4 phase wraps. Add `dance_foot_phase` and `dance_both_airborne`. Restore
velocity tracking and light `air_time` / `foot_slip`. Weaken
`dance_z_tracking` to 0.5. Episode 20 s. Keep obs 61. `rel_standing_envs
= 0`. `heading_command = False`. Joint action scale 1.0.
`symmetry_cfg = None`. Keep `push_robot` in train, drop it in play.
Keep `ZERO_ENERGY_PROB = 0.3` in train.

**Acceptance:**
- [ ] `body_command` still 6-D `[sin, cos, 0, 0, 0, energy]`
- [ ] Twist command non-zero on a majority of high-energy steps
- [ ] Histogram of resampled phrases matches the mix table
- [ ] 64 env × 5 iter smoke exit 0 on Spark

**Verification:** Spark `run_smoke.sh` equivalent, isolated job dir. Do not
stop docker/LLM.

**Files:** Spark `microduck_rl` clone + copy recipe into
`docs/ideas/dance-gait/microduck_dance_loco_env_cfg.py`

**Depends:** none (can overlap Task 1–2) · **Scope:** M

### Task 4 — PPO on Spark

**Description:** New job dir
`/home/mstaub/jobs/feat-dancing-skill-loco-<session>/`. 4096 envs, **from
scratch**. 15k iters or until mean ep length is near cap and `fell_over`
is not exploding. `WANDB_MODE=offline`. tmux name distinct from
`dance-train-1bf7770a` (do not attach to the finished standing job).

**Acceptance:**
- [x] `nan_state` stays 0
- [ ] Mean `|xy|` speed clearly above the standing run (not ~0) — train `twist_speed_xy` ~0.04–0.06; visual QA in Checkpoint B
- [x] Foot-phase reward fires (non-zero, not stuck at both-down)
- [x] Export via `uv run scripts/export.py` only → `obs[1,61] → actions[1,14]`

**Verification:** train log + ONNX shape probe. 20 s play video at 100 BPM
with phrase resampling on: `docs/ideas/dance-gait/dance_loco_20s_100bpm.mp4`.

**Depends:** Task 3 · **Scope:** M (wall-clock ~15 h, not a code XL)

### Checkpoint B

- [x] Clip shows travel + alternate feet
- [ ] If the policy moonwalks or both-feet-slides, do not export; retune
      foot-phase / slip weights and re-smoke before another 15k
- [ ] If it is a small walk with no hop, raise `dance_foot_phase` / `air_time`
      before widening the twist box

v1 `dance_loco_20s_100bpm.mp4` **failed** (2026-09-05): no bob, no hop, no
travel. Raised `dance_z_tracking` 0.5→1.5, `dance_foot_phase` 1.5→3.0,
replaced walk `air_time` with energy-gated `dance_air_time` (no twist
threshold), play camera 0.55. Fine-tune from `model_14999.pt`, **not** a
second 15k scratch. Do not widen vx/vy/vyaw yet.

v2 `dance_loco_20s_100bpm_fix.mp4` (2026-09-05 15:37 ET): **travel yes, hop
no.** Center-crop 1 fps |Δ| ~20.8 vs v1 ~1.2; floor grid shifts up to 25 px
(v1 was 0). Both feet stay planted with a beat bob (`air_time` ~0.038).
Raised `dance_foot_phase` 3.0→5.0, `air_time` 0.5→2.0, thresholds
0.08–0.28. Another 3k from `model_20998.pt` (`dance-loco-hop2-1bf7770a`).
Still do not widen vx/vy/vyaw.

v3 `dance_loco_20s_100bpm_fix2.mp4` (2026-09-05 18:53 ET): **travel + swing
foot.** Center-crop |Δ| ~23.8; grid shifts hit the 80 px lag cap; 25 fps
left/right contact split (v2 was lockstep). Frames show one orange foot
off the floor. ONNX `dance_loco_iter_fix2.onnx` `obs[1,61]→actions[1,14]`.
Do not load it as `[policy] stand`.

### Task 5 — Robotd loads the new ONNX

**Description:** Point `policy.dance` at the export. Keep `policy.stand` as
`alpha_stand`. `[audio] dance = true` plus dance file ⇒ director + dance
net. Toml comments state the two-file rule.

```toml
[policy]
stand = "alpha_stand.onnx"   # stays the crouch / idle net
dance = "/path/to/dance_loco.onnx"

[audio]
dance = true
# dance_gait is irrelevant on this path; Net::Dance always gets sin/cos/energy
```

**Acceptance:**
- [ ] Toml comments state the two-file rule (stand stays stand)
- [ ] Gait encoding still used for body x/y/yaw on `Net::Dance`

**Verification:** unit tests on net choice; no board required

**Depends:** Task 2, Task 4 · **Scope:** S

### Task 6 — Hardware (when a duck is available)

Speaker in the room, floor not a table edge. Same omit rules. Expect small
steps, not a walk commute.

**Depends:** Task 5 · **Scope:** S (manual)

## Training host

Spark `mstaub@100.69.30.38`, new job dir as in Task 4. uv is
`/home/mstaub/.local/bin/uv`. Do not train on this Mac. Do not `docker
compose down`. Do not kill unrelated LLM / rfc jobs. Fallback: windows1
WSL `micro@100.77.103.59`.

Export only via `uv run scripts/export.py` (bakes the obs normalizer).

## Risks

| Risk | Mitigation |
|---|---|
| Fine-tune from stand-bob collapses when twist appears | Do not default to it; scratch train |
| Tracker quarantine on director twist | Quarantine uses client twist only (Task 1–2) |
| Dance net selected for pad walking | Overlay omits on `twist_fresh`; pad uses walk |
| `|director twist|` selects walk | `dance_active` before `will_stand` |
| Director twist in `twist_ema` poisons pad deadman | Separate `dance_twist_ema`; never write `gated.twist` |
| `body_active` zeros director twist | Overlay omits on `pose.active`; do not set it |
| "Hop" interpreted as jump | v1 forbids flight; stance/swing only |
| Walks off a table | Floor-only hardware; limp-fall stays on |
| Phrase RNG looks like a march | Mix table + sidestep/spin; play cfg still resamples |
| Train mix ≠ robot mix | One table, histogram tests, Python comment |
| Degenerate obs normalizer from 14999 | Scratch train |

## Ready checklist

- [x] Travel cannot ride `policy.stand` / `will_stand`
- [x] Tracker cannot see director twist
- [x] Hop is stance/swing, not flight
- [x] Train and runtime share phrase length and mix
- [x] Scratch vs fine-tune decided
- [x] Params / proto / registry / robotctl file list is complete
- [x] Hardware omit rules unchanged
- [x] No `robot.dance` RPC, no audio-in-obs, no Mac training
