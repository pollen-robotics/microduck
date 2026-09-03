# The autonomous behavior stack — what it inherits, and ideas waiting for it

The brain is the biggest untracked gap in the [parity audit] (§03): the runtime's
`autonomous.rs` exists nowhere in the daemon and no design doc owns it yet. This file is the
holding pen — what the port has to cover, and the ideas from the theremin/chorale work
(2026-08) that should land as *behaviors in that stack* rather than as more ad-hoc modes.

[parity audit]: https://claude.ai/code/artifact/4ea45b32-a298-42ea-bcb4-1d2a7a567948

## To port from the runtime (audit §03)

A 16-state machine — Chill, LookAround, Wander, TurnInPlace, Zoomies, Startle, Stretch,
Ruffle, Preen, Sneeze, Dance, GroundPick, Nap, BallPlay, Petted, Held — built on:

- an **energy/mood model** driving state choice
- **novelty-grid exploration memory** for Wander
- **ToF obstacle avoidance** with freshness gating
- contrast-based **startle**; **sound reactions** (noise vs voice) with self-audio /
  self-motion gating
- **ball play** (approach / line up / kick), a **nap cycle**, **petting reactions**
- the in-process gamepad↔autonomous toggle (DpadRight 2 s), `--autonomous-max-speed`

Its inputs all exist in the daemon now: ambient sound events (step 2 — logged, no consumer),
depth frames (step 3), classified trunk-frame obstacle points (step 5), and the voice tags
(step 2, "ready and waiting for the brain step"). `Held` depended on pickup detection, which
is deprecated — decide whether to revive it or drop the state.

## New inputs the brain gets for free (2026-08 work)

Things the theremin/chorale steps built that the runtime's brain never had:

- **Nearby ducks, by stable id** — the chorale beacon (`ChoraleBeacon`), which survives BLE
  address rotation (see `docs/../memory`: never key on the address)
- **A shared beat** with no clock sync (`sounds::chorale::beat`) — ±20 ms across ducks
- **RSSI per advertisement** — free coarse distance (near / far / approaching)
- **~245 spare bytes** of extended-advertising payload
- **A live synth voice** (`sounds::Stream`): pitch/level/vowel at runtime, not just bank wavs
- **Hand distance from the ToF** (`kinematics::hand::Tracker`)

## Behavior ideas, roughly by charm-per-line

**Social (BLE presence — the new territory):**

- **Recognition & greeting** — keep a persisted list of duck ids met; `greet` a stranger,
  a warmer sound for a friend, a `peck`/sigh when a friend's beacon goes stale. Friendship
  as a met-count that changes the greeting over time. Reads as *memory*; needs no sync.
- **Lonely / content** — a duck alone calls out occasionally; a duck with company doesn't.
  An input to the mood model, not a state.
- **Excitement on approach** — RSSI rising for a known id → visible anticipation.
- **Marco Polo** — one duck hidden, another guides you by quacking faster as RSSI grows.
- **Follow-the-leader** — RSSI holds spacing, ToF handles the duck directly ahead.
- **Applause / social feedback** — one duck does a roulade, nearby ducks react.
- **Telephone** — a message hops duck to duck through the spare payload, mutating.
- **Voting** — preferences in the beacon payload; majority picks the group's next behavior.

**Beat-synced motion (cash in the sync work where it shows — no speaker involved):**

- **Group head-bob / sway** — all ducks in phase on the shared beat; visual sync tolerates
  ~50 ms where audio wanted 20. Group pose on the downbeat of a bar. Conga line.
- **Dance** (already a runtime state) becomes *synchronized* dance when company is present.
  One duck dancing to **heard** music is a different clock (the onboard mic, not the
  BLE beat) and lives in [`dance-to-music.md`](dance-to-music.md) until the brain can
  choose it as a state rather than as another `robotd` mode.

**Musical, beyond the chorale:**

- **Call and response** — antiphonal phrases; the gap between phrases hides sync error, so
  it is *easier* than the chorale and more duck-like.
- **A round** — same melody, deliberate multi-bar offsets; the offsets absorb the error.
- **One role each** — drone + rhythmic peck + melody. Small speakers do texture better
  than harmony.

## Duck detector (camera + NPU) — wanted, waiting on mediad

A tiny single-class detector for *our own duck*: precise **bearing** for gaze and following,
which neither ToF nor BLE can give. The RK3566 has a 0.8 TOPS INT8 NPU (`rknpu2` /
`rknn-toolkit2`); a YOLOv8n/11n-class model at 320 input should run ~20–40 ms → 15–30 Hz,
leaving the CPUs alone. Range math: IMX219 ~62° HFOV, a 25 cm duck is ~25 px at 3 m — room
scale, which is the interaction envelope.

- **Gate first:** is the NPU driver on the board? (`dmesg | grep -i rknpu`) — vendor-kernel
  `rknpu2`, not mainline. And `/dev/rga` for free NV12 resize. Ask mediad to tee raw NV12
  from the ISP mainpath; the detector must not decode the streaming MJPEG.
- **Data is the project, not the model.** Duck's-eye-view footage (robot height, robot
  camera) auto-labeled by a big open-vocab model, distilled into the tiny one; synthetic
  renders from the Open Duck CAD for the tail; hard negatives (rubber ducks, white prints).
- **Fusion:** vision cannot tell identical ducks apart — camera = direction, ToF = distance,
  BLE beacon = identity + presence. Follow-the-leader uses all three; "look at each other
  when doing stuff" = beacon says *when*, detector says *where*, `robot.look` does the rest.
- Same architecture as `tofd`/`pet-detect`: a perception worker outside `robotd`, safe to
  kill. Fix the placeholder IMX219 intrinsics during the mediad port (audit §04 TODO).

## Shape notes for the port

- Presence, mood, and the shared beat are **inputs to one brain**, not modes beside it —
  the chorale/theremin grew as explicit modes because there was no brain to hang them on;
  fold them in as states/inputs when it lands ("Petted" is to pet-detect what "Sing" is to
  a heard beacon).
- **The chorale ends up as a spontaneous event, not a command.** `robotctl chorale` is
  bench scaffolding: once ducks run autonomously, a group of them together should
  *sometimes* break into song on their own — a low random chance gated on company being
  present (and plausibly on mood/energy), the way Zoomies or Dance fire, not something a
  user starts. Rare on purpose: a surprise duet is a delight, a jukebox is not. The
  mechanism barely changes — an idle-beacon duck already knows who is nearby, so "decide to
  sing" is one more transition; `[chorale] accept` stays as the consent gate for whether a
  duck may ever join in.
- The chorale's consent rule generalizes: **anything social is opt-in and off = invisible**
  (`[chorale] accept` today; probably one `[social]` switch tomorrow).
- Recognition/greeting is the right *first* behavior: highest charm per line, no sync, and
  it exercises the BLE discovery layer — the part still under hardware suspicion.
