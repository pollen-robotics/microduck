# Dance to music the duck can hear

Status: idea · Date: 2026-09-02 · Not a design decision

Holding pen for teaching a Microduck to move in time with music that reaches its
onboard microphone. It is not a design doc: nothing here is decided, and a
`robot.dance` mode must not grow out of this file the way the theremin and the
chorale grew — those exist as explicit modes because the autonomous brain
([`autonomous_behavior.md`](autonomous_behavior.md), roadmap M9) was not there
to hang them on.

**The claim.** The robot already has a dancer. The stand policy maps a small
body-pose command into balanced bob and sway. What it does not have is a
listener that fills that command from the mic. Do not put audio in the gait
observation. Hear a beat, send the intents the stand network already knows.

## Why not retrain the walk

Every shipped policy is `obs[1,61] → actions[1,14]`, checked at load
(`duck-control/src/obs.rs`, `policies/README.md`). The 61 floats are gyro,
projected gravity, 14 joint positions and velocities (mouth excluded), the
previous action, and a 13-value command:

```
vx vy vyaw | neck_pitch head_pitch head_yaw head_roll | body_x body_y body_z body_roll body_pitch body_yaw
```

`body_x`, `body_y` and `body_yaw` are hardcoded zero: unbound in training, so
an all-zero body command is the nominal encoding. The three that *are* bound
are `body_z`, `body_roll`, `body_pitch`, and they are exactly what
`robot.pose` writes.

Widening that vector to carry a spectrogram, or even four extra beat features,
invalidates every locomotion policy under `policies/` (`obs[1,61] → actions[1,14]`)
and every corresponding run in
[microduck_rl](https://github.com/pollen-robotics/microduck_rl). Pet and duck
detectors are a different shape and are not this contract. Sim2real for
audio is a second project on top: the capsule is on the head, it hears servos,
and the duck's own speaker leaks into it. Walking already conditions on a
command (`vx`); a later dance gait should condition on beat phase the same
way, not on a waveform.

Architecture already says this for vision (`docs/design/architecture.md` §2.4):
perception sits next to the sensor and publishes **features**, tens of bytes,
cached last-value-wins. The control loop never blocks on a mic.

## What is already on the robot

| Piece | Where | What it gives dance |
|---|---|---|
| Stand policy + body pose | `policies/alpha_stand.onnx`, `robot.pose` | Bob, crouch, roll, pitch inside the trained box |
| Head and mouth intents | `robot.head`, `robot.mouth` | Head flicks and beak; the mouth is not in any policy |
| Gamepad BodyPose mode | `padd` **B** | The same mapping, driven by sticks — a metronome test with no code |
| Onboard mic | TLV320AIC3104, `plughw:aic3104,0` | 16 kHz mono; exclusive ALSA capture |
| Mic worker | `pet-detect/src/worker.rs`, driven by `robotd` | The only process that may hold the capture device |
| Petting CNN | `pet-detect` | Template for on-robot audio ML (log-mel → tiny ONNX, train/infer parity via `pet-features`) |
| Ambient sentry | same worker | RMS envelope; **swallows music on purpose** |
| Chorale beat follower | `sounds/src/chorale/beat.rs` | Phase lock to a pulse, ±20 ms; the pulse today is a BLE beacon, not audio |
| Prototype Dance state | runtime `autonomous.rs`, not ported | A brain state, not a gait. Group head-bob is the later, multi-duck version |

`body_active` (`robotd/src/control.rs`) is the standing-pose contract: while a
client holds `robot.pose` with `active: true`, the twist is zeroed and the
stand network is forced even if magnitude selection would have walked. That is
the prototype's B-button mode, and it is the motion regime this idea uses.

Trained pose box, documented on `PoseParams` and mirrored by `padd`:

- `z`: −0.025 m (crouch) … +0.010 m (rise)
- `roll`, `pitch`: ±0.26 rad (~15°)

Out of that box the policy is out of distribution. The robot does not clamp
pose intents; the client must stay inside.

## Constraints that will bite

**The sentry is not a beat tracker.** `SoundEvent::Noise` / `Voice` fire on
short loudness bursts. Runs longer than ~3 s — vacuum, music — raise the
adaptive floor and emit nothing (`pet-detect/src/worker.rs`). Reusing the
sentry for "is there a song" is fighting its design. Add a parallel tracker on
the same PCM.

**Capture is single-client.** Everything that analyses the mic shares the
`arecord` subprocess (`S16_LE`, 16 kHz, mono). A second `arecord` on
`plughw:aic3104,0` starves the worker (or never starts). Laptop demos must use
the **laptop's** mic, or consume features the onboard worker already
publishes — never a second capture on the duck. Recording training audio on
the robot means **stopping the worker first** (or tapping PCM it already
owns). Do not copy the pet-detect README's `arecord` line while `robotd` holds
the device.

**The mic worker today is petting-shaped.** `[audio] pet_detect` defaults to
unset/false because always-on cooing wore thin. Startup currently requires
that flag **and** an on-disk pet ONNX; `PetHandle` constructs the classifier
and every PCM block runs through it. Dance must not inherit that coupling.
Capture + beat tracking have to run with **no pet model and no ONNX**, while
pet inference and the coo stay gated on the petting opt-in. Open question 1
is the config surface; this paragraph is the invariant.

**Pose and mouth are unstamped; head is stamped but has no expiry.** Twist
has a deadman (~500 ms, `safety.deadman_ms`). Pose does not: `active: false`
is the only "nobody is posing". Head is stored as `Stamped` but `snapshot`
discards its age, so it does not expire. Socket EOF does not clear intents
(`robotd`'s connection loop just ends). A client that dies while `active:
true` leaves the last crouch, and that crouch **keeps forcing stand and
zeroing twist**, so a publishing pad that appears later cannot walk. Mouth
and head stay wherever they were.

Phases 0–1 are **attended demos**: trap exit (including SIGINT) and send
`active: false` **and** head/mouth back to nominal. That is not crash-safe.
`robot.enable` off commands the joints home and stops the policy **with no
ramp**, but it does **not** clear the pose/head/mouth slots — re-enable would
restore the crouch and force stand again. Unwise while fallen or obstructed;
not an emergency stop. `robot.relax` is torque off — the robot collapses if
nothing is holding it. Functional recovery is `active: false` plus nominal
head/mouth, then enable if you want to drive.
Do not grow a pose deadman in this idea unless a design pass
changes the prototype contract. An in-process mapper is a tick-local overlay
(see Architecture): a daemon crash drops it with the process; an `arecord`
death must still unlock (see BeatState).

**Deadman only watches twist.** A client that only sends `robot.pose` does not
refresh the twist clock. Stale twist zeroes velocity, which is what standing
dance wants, but that client still holds `pose.active`. Phase 2 must treat
`snapshot.pose.active` as a live owner: omit the overlay. `padd` on pad-gone
sends **nothing**, including no `active: false`, so a BodyPose unplug without
B leaves the slot held until someone clears it. `robot.enable` is a discrete
latch; it stays on until something disables it. Enable on a limp robot with a
loaded policy already turns torque on and ramps home (`robotd` bring-up on
enable). `robot.init` is the no-policy bench path, not a prerequisite for
dancing. The dance client does not need to stream `robot.move`.

**Last-writer-wins with a publishing pad.** `padd` ships and runs from boot,
but with no gamepad connected it sends **nothing** (deadman holds the robot).
A connected pad in Drive streams `robot.move` every tick, including a zero
stick. In Head and BodyPose it still refreshes a **zero** twist every tick,
so `|twist|` is not a live-pad detector. Yield on **fresh twist age** (any
writer, including zeros) — in-process, `snapshot.twist_age` inside the
deadman window. An external client must not treat a cached
`movement.limited_by` containing `"deadman"` as permission to keep posing:
state frames can stop (no sensors → no `robot.state`), and the last frame
would lie. External mapping is attended phases 0–1 only, and must clear
pose/head/mouth on stream loss or a stale `RobotState` timestamp. Stop
`padd` for scripted demos (`docs/robot/cheatsheet-dev.md`) — deterministic.
`padd` up with **no pad connected** already sends nothing. Do not invent a
fourth authority; take whatever M9 / architecture §9 decides later.

**Roller has no stand network.** `standing_disabled` is set in roller mode.
Dance-to-music is walk-mode, on legs, standing. Do not attempt it on wheels.

**Skills preempt stand.** Roulade, kick, ground pick, sit/rise win the
network selector. A dance mapper must not overlay pose during those windows, and
must not be surprised when a kick eats a beat. The stand policy must actually
be loaded (`has_standing`); without it `body_active` cannot force `Net::Stand`
and a zeroed twist runs the walk network.

**Fall is a report, not limp.** `Safety::fallen` is published and preempts
nothing (`duck-control/src/safety.rs`): a fallen robot keeps being driven.
Limp-fall is a separate predictor in `robotd` that goes soft before impact,
then poses back to standing. **On by default** (`[safety] limp_fall`; set
false for the prototype's report-only behaviour). Eligibility exclusions:
already down, skills, … The mapper must handle limp-fall as the **default**
path, not an opt-in. Do not write "fall → limp" as if `Safety::fallen` did
it. The mapper still omits the overlay (external: `active: false`) when
`fallen` is true **or** limp-fall is in sequence, so a crouch is not handed
back to stand after the robot is upright again.

**Self-audio and self-motion gate the tracker, not just the mapper.** The
speaker and the mic share a head. Servos are loud. A lock acquired while
walking, kicking, limp-falling, or while the voice/theremin/chorale owns the
PCM is a lock on the duck, not on the song. Quarantine the tracker in those
windows (same idea as the sentry's petting hangover): `locked = false`,
reset the period estimate, ignore PCM for a hangover after the noise stops,
then require a fresh stationary / post-audio lock before the mapper may pose.
Gate on **the duck emitting sound** (voice, theremin, chorale) leaking into
the mic, plus a hangover — not on who owns the capture device. The runtime
brain already named this: "sound reactions with self-audio /
self-motion gating." Dance is a standing, quiet-robot behaviour.

**Smoothing.** Body pose is EMA-smoothed with `cmd_alpha` (default 0.2) in
the control loop (`robotd/src/main.rs`, `body_ema`), not `head_alpha` — that
one is the head only. At 50 Hz, α = 0.2 is about 100 ms to approach a step
change. Square-wave crouches at 180 BPM will come out attenuated. 60–140 BPM
is the readable range. The chorale's ±20 ms budget is for *audio* ensemble,
not for a bob. Beat-tracker hop plus this EMA is the latency that matters;
do not spend it chasing sample-accurate onsets.

**Unrequested motion.** A duck that starts swaying because a speaker came on
is the same class of surprise as a chorale that starts because another duck
walked in. `[chorale] accept` is false by default and off means invisible.
Dance needs the same consent shape: opt-in, off by default, no motion and no
extra mic work when off.

**Do not add a daemon mode.** M9's explicit warning: presence, mood, and beat
are inputs to one brain, not modes beside it. The on-robot mapper is a
*reaction* (petting → coo), gated by opt-in, until the brain can choose Dance
as a state. A `robot.dance` RPC that arms an instrument is the theremin
pattern again.

**`mediad` does not capture this mic today.** `docs/design/architecture.md`
lists `mediad` as owning camera/mic in the service table; the capture that
exists is `arecord` inside `robotd`'s pet-detect worker. A future `alsasrc`
in the WebRTC pipeline would collide with that exclusive device. Dance must
not be a third opener. If streaming ever takes the PCM, the worker (petting +
sentry + beat) has to become its consumer, not a competitor.

## Architecture

```
                    room music
                        │
                        ▼
              TLV320 / arecord 16 kHz mono
              (one capture; worker may run without the pet CNN)
                        │
        ┌───────────────┼────────────────┐
        ▼               ▼                ▼
   petting CNN     ambient sentry    beat tracker  ← new, parallel
   (opt-in, may     (existing,         on the same PCM
    be absent)      ignores music)
                        │
                        ▼
                  BeatState cache
                  last-value-wins **with a source time**
                  never on the 50 Hz critical path
                        │
                        ▼
              mapper (opt-in, standing, not busy, no live pad, pose inactive)
                        │
                        ▼
     phases 0–1: robot.pose / head / mouth (attended client)
     phase 2: this tick's command, chorale-sway style — not the shared slots
                        │
                        ▼
              stand policy, unchanged
```

Two placements for the mapper, in order:

1. **External client** (like `padd`). Owns hearing (laptop mic) or, later,
   reads a published `BeatState`. Writes the same intents any other client
   writes. This is the demo and the test harness. It does not touch `robotd`'s
   tick. Attended: no pad publishing (stop `padd`, or leave it up with no
   pad), enable, stream pose, clean up with `active: false` plus nominal
   head/mouth.
2. **In-process overlay** inside `robotd`, consuming the worker's channel
   the way petting already does. Same place the chorale adds head sway: this
   tick's `command.body` / head / mouth, **not** `intents.set_pose` (or
   set-head / set-mouth). Shared slots stay with clients. Unattended (phase
   2). Still not a mode: a flag, a cache, a map onto this tick. Omit the
   overlay on lost/stale lock, `fallen`, limp-fall, skill, a live twist
   (`twist_age` inside the deadman window), **or** `snapshot.pose.active`.

**Preemption is load-bearing, not polite.** `body_active` zeroes the twist and
forces stand. A mapper that holds `active: true` on the **shared** pose slot
while a pad is streaming would steal the walk — including Head/BodyPose,
where the streamed twist is zero — and writing `active: false` into that
slot on yield would clobber BodyPose. Yield when twist is **fresh** (not
merely large) **or** `pose.active` is true: the in-process mapper **omits
the overlay** so the snapshot is unchanged; it does not write the shared
slots. A publishing pad blocks dance; so does leftover BodyPose after the
pad is gone. `padd` running with no pad connected does not, **unless** the
pose slot is still `active`. External 0–1 cleanup still sends
`active: false` because that client *is* the slot writer.

## BeatState

Keep it small enough to log and to test. Recompute every ~20–32 ms, same
order as the sentry's 512-sample frame.

| Field | Role |
|---|---|
| `t` | Monotonic source time (or a frame sequence). Mapper treats missing/stale as unlocked |
| `locked` | True only after a stable period estimate **and** a fresh `t` |
| `bpm` | Held period, not re-fit every frame (the chorale lesson: slope-at-the-edge is a worse clock) |
| `phase` | 0..1 through the current beat; **0 is a beat onset**, not a bar downbeat |
| `onset_seq` | Monotonic counter, incremented on an onset; not a sticky boolean |
| `energy` | Short-window RMS above a slow floor, 0..1 |
| `band_low` / `band_mid` | Optional; kick vs the rest. Skip until a mapping uses them |

`arecord` failure today is a silent retry/backoff inside the worker; nothing
invalidates consumers. Last-value-wins without `t` would keep `locked: true`
and an unstamped pose through a dead capture. Required semantics: EOF,
spawn failure, or a gap longer than one beat → `locked = false`, tracker
reset, overlay omitted (external mapper: `active: false`). Relock only after
a fresh period estimate on new PCM. Onset is an edge (`onset_seq` changed
since last tick), not a cached flag.

Classical DSP first: spectral flux or high-frequency content for onsets,
autocorrelation or comb-filter for period, then a phase follower in the
spirit of `sounds::chorale::beat::Follower` — hold the period, average phase.
A second CNN, pet-detect-style, is only justified if this mic's motor hum and
plastic shell defeat flux. If that happens, record on-robot **with the mic
worker stopped** (or from a tap it exposes), same format as the worker
(`arecord -D plughw:aic3104,0 -f S16_LE -r 16000 -c 1`), extract through a
binary so train and infer cannot drift, and do not train on clean studio
audio.

Do not vendor songs. Click tracks and synthetic pulses are the test corpus.

## Mapping (inside the trained box)

While: walk mode, policy enabled, stand network in charge, not `fallen`, not
in limp-fall, not in a skill, not playing the theremin or chorale, opt-in on,
`BeatState.locked` with a fresh `t`, no live driver (`twist_age` stale **and**
`pose.active` false).

Phase 0 of the beat (an onset) is the crouch. A zero-centered sine is 0 there,
so it cannot be "crouch on the onset." Use a waveform that is **most negative
at phase 0** and **0 at energy 0**:

```
# crouch at beat onset (phase 0); energy 0 = nominal (z = 0)
z     = -A_z(energy) * 0.5 * (1 + cos(2π * phase))
roll  =  A_r(energy) * sin(2π * phase)          # optional sway; same every beat
pitch =  0 at first
head  =  short pulses when onset_seq advances; yaw/roll, not pitch-into-floor
mouth =  energy in the low band, or an onset_seq edge; this is the only mouth mover
```

`phase = 0` is a **beat onset**, not a bar downbeat. There is no bar counter
in `BeatState`; do not plan choreography on beats 2 and 4 until one exists.
`A_z(1)` ≤ 0.025 m (the crouch side). The rise side of this waveform is 0,
not +1 cm, which fits the asymmetric box without clipping. Do not center a
sine on z = 0. Start with z only. Add roll when the bob reads. Head and
mouth last. If lock drops, `t` goes stale, or the tracker is quarantined:
external mapper sends `active: false` (snap to nominal, the B-button exit)
and head/mouth to 0; in-process omits the overlay and leaves client slots
alone.

Cap amplitudes so `|z|` and `|roll|`/`|pitch|` never exceed the box, including
the energy scale. The bound is on the **body command that reaches the policy
this tick**, not only the dance term. `padd` uses `BODY_MAX_Z_UP = 0.010`,
`BODY_MAX_Z_DOWN = 0.025`, `BODY_MAX_ANGLE = 0.2618`.

## Phases

### 0 — Motion path, no hearing

Two separate checks. Do not interleave them.

**A. Visual budget, `padd` running, a pad connected.** Enable the policy
(`robot.enable`; that homes from limp if a policy is loaded). Stand still,
press **B**, bob the sticks to a metronome inside the asymmetric box. Exit
BodyPose (B again — `padd` sends `active: false`). **Unplug the pad** (or
stop `padd`). Centering the sticks does **not** age the twist: a connected
pad keeps publishing `robot.move` at zero. Then wait past the deadman
(~500 ms). `robot.init` only if you want to stand **without** a policy.

**B. Scripted waveform, `padd` stopped.** `sudo systemctl stop padd`. Then a
script on the robot (or a laptop with
`ssh -L /tmp/robotd.sock:/run/robotd.sock`) sends the **same crouch waveform**
as the mapping section, at a fixed BPM, **not** a symmetric sine about z = 0.
The socket is root:robot mode 0660; the process writing it needs that group,
or root.

Notifications, no `id`, one JSON object per line (NDJSON — the newline is
the frame), as `padd` already does. Example at the bottom of a crouch (phase
0, mid-amplitude):

```json
{"jsonrpc":"2.0","method":"robot.pose","params":{"z":-0.012,"roll":0.0,"pitch":0.0,"active":true}}
```

Enable is a request (needs an answer):

```json
{"jsonrpc":"2.0","id":1,"method":"robot.enable","params":{"on":true,"toggle":false}}
```

On a clean exit, send `active: false`, zero head, zero mouth. If the process
is killed, the same three notifications are the functional recovery — not
`robot.enable` off, which only homes until you enable again and then
restores the abandoned crouch. No firmware change. If this does not look
like dancing, a new policy will not save it.

### 1 — Laptop hears, duck moves

Same as 0, plus a beat tracker on the laptop (aubio, librosa, or the DSP
sketched above), room speaker, **laptop** mic. Same socket, same pose stream.
Still attended. For the scripted path, `padd` stopped (or no pad publishing).
This still does not prove the **duck** can hear — it proves hearing → motion
with a clean capsule and no motor noise. That is the point: cheap iteration
before the hard mic.

### 2 — The duck hears

Tracker in the mic worker, on the existing PCM. Worker starts if petting
**or** dance (or `[audio] listen`) is on, **without** requiring the pet ONNX.
Self-audio / self-motion quarantine. Standing only. Opt-in. Fresh `t` or no
dance. **Map in-process** as a tick-local overlay on this tick's command
(chorale-sway style), not by writing `PoseIntent`. The phase-1 external
client is attended and crash-unsafe; it is not how phase 2 ships.
`BeatState` on `robot.state` is still optional for diagnostics, not for an
unattended mapper. A publishing pad still wins, and so does a held pose
slot: omit the overlay on fresh `twist_age` **or** `pose.active`, shared
slots unchanged. Unplug without B leaves `active: true`; dance stays off
until that slot is cleared. The tracker must not carry a lock across
walking, skills, limp-fall, or self-audio.

Done for "it dances to music it can hear" when a Bluetooth speaker in the room
moves the duck, the duck's own quack does not, a dead `arecord` stops the
bob, and walking/skills/`fallen` still behave.

### 3 — Only if sway is not enough

A dedicated dance gait in `microduck_rl`: condition on beat phase and energy
in the **command** block, the way walking conditions on `vx`. The three
unbound slots (`body_x`, `body_y`, `body_yaw`) are the obvious place if the
width must stay 61 — but only a newly trained network may read them; the
current stand policy was trained with those at zero, so stuffing a beat in
there under today's weights is out-of-distribution. Play a click or song in
sim, reward phase-locked motion plus balance, randomize BPM. Export ONNX.
Imitation from teleop is a poor fit: this biped has to balance, and the
training stack is PPO in MuJoCo, not demonstration datasets.

Group sync is a different input: the chorale beacon's beat, already shared
across ducks. That is M9's "Dance becomes synchronized when company is
present." Do not couple it to the mic tracker.

## Safety, authority, privacy

- Joint targets still go through `safety.apply`: non-finite refused, range
  clamped to **actuator travel** (±π), not anatomical limits. Deadman zeroes
  **twist only**. `fallen` is a report. Limp-fall is **on by default** and
  separate from that report.
  Pose commands are **not** clamped by the robot; staying in the trained box
  is the mapper's job, the same way `padd` source-bounds BodyPose. `safety.apply`
  is last-resort protection against NaN and wrapping the servo, not a dance
  sandbox.
- Phase 0B scripted path: stop `padd` (deterministic). Phase 2 is an
  in-process overlay and **omits** it on a publishing pad (shared slots
  unchanged); idle `padd` with no pad need not be stopped.
- Mic for dance is a camera-class privacy choice in a home. Opt-in, off by
  default. There is no software-controlled LED
  (`docs/design/remote-webrtc.md` §11); do not pretend dance is hidden by one.
- Do not dance while `audio.enabled` is false.

## Tests that make this real

Off-hardware, so they run in `cargo test --workspace`:

- Tracker locks onto a synthetic click train at a few BPMs in the 60–140
  range, reports phase 0 on the beat onset within a generous visual bound, and
  drops `locked` when the clicks stop.
- Capture EOF / a gap longer than one beat clears `locked` and bumps `t`
  so a mapper cannot keep posing on a stale sample.
- Tracker does not lock onto a duck-voice WAV (self-audio fixture).
- Mapper output never leaves the pose box, including at energy = 1; z is
  ≤ 0 and most negative at phase 0; energy 0 is z = 0. The check is the
  command that would reach the policy this tick.
- External mapper emits `active: false` (and nominal head/mouth) on lost
  lock, stale `t`, tracker quarantine (motion / self-audio), and `fallen`.
  In-process overlay is omitted on those plus fresh `twist_age` **or**
  `pose.active`; shared pose/head/mouth slots are unchanged (a BodyPose pad
  must still win). Stale twist + `pose.active` is a required regression:
  overlay off, leftover crouch not summed with the dance term.
- Tracker drops `locked` across a walking/skill/self-audio window and does
  not relock until a hangover of quiet standing PCM.
- Worker can pump PCM and beat frames with **no** pet model loaded.
- Worker still shares one capture: petting events and beat frames from the
  same PCM in a test double, not two `arecord`s.

On-hardware, once, as a checklist rather than CI:

- Phase 0 crouch-waveform bob on a real stand policy, `padd` stopped.
- Kill the script mid-crouch; `active: false` plus nominal head/mouth returns
  the body. Confirm `robot.enable` off then on **restores** an abandoned
  crouch if pose was not cleared. Confirm a publishing pad cannot walk
  through an abandoned `active: true`.
- Phase 2: speaker in the room, duck standing, petting still optional, a
  quack does not start a dance, `fallen` / a publishing pad omit the overlay
  (pad BodyPose still wins), a yanked capture stops the bob.

## Non-goals

- Audio in the 61-D observation.
- A `robot.dance` mode or a new daemon.
- Dancing while walking, on the roller, during a skill, or while a pad is
  publishing twists.
- Playing the song from the duck's speaker so it can "hear itself."
- Matching a specific choreography or genre classifier.
- Shipping music files.
- Replacing the chorale; they are different clocks (BLE beat vs heard beat).
- Changing the pose/mouth unstamped contract, or giving head an expiry
  (that is a design doc).

## Open questions

1. **Worker lifetime.** One `[audio] listen` that covers capture, sentry, and
   beat, with petting-coo and dance-map as separate opt-ins — or keep
   `pet_detect` and add `dance`. Either way, capture must not require the pet
   ONNX. The first avoids a second reason to hold ALSA.
2. **Where `BeatState` is visible.** In-process only, or on `robot.state`
   (and then BLE/WebRTC), including `t` so an external client can apply the
   same stale-lock rule. The latter makes phase 1's client work with the
   duck's ear without folding the mapper into `robotd`.
3. **Preemption.** Pad vs script vs future brain. Today it is
   last-writer-wins on shared slots, plus "fresh twist **or** `pose.active`
   ⇒ omit the overlay." Phase 2 must not write those slots. Dance should not
   invent a fourth authority; it should take whatever M9 / architecture §9
   decides. Dancing while a pad is publishing is not a phase 0–2 goal.
4. **Whether phase 3 is ever worth it.** Decide after phase 2 is boring on
   hardware, not before.

## Done when

A duck standing in a room with a speaker, no pad publishing, bobs on the beat of
music that reaches its own mic, stays inside the stand policy's pose box,
ignores its own voice, stops when capture dies or the music stops, and gives
the body back — with no change to observation width and no new control-loop
mode.
