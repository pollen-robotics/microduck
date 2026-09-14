# Per-mode policy slots — `roller` and `crouch`

Status: draft · Date: 2026-09-14 · Owner: coralie

Giving the roller its own slots in `[policy]`, so "the network this robot drives with" is two
decisions rather than one key that means something different depending on a mode set elsewhere in
the same file.

Companion to [`policy-channel-design.md`](policy-channel-design.md), which owns where a slot's
file comes from and the commands that change it — this page owns only *which slot* a mode reads.
Nothing here touches the channel or the manifest format. The wire gains one additive field on a
report and nothing else (§5.1).

## 1. What is wrong today

`[policy]` has seven path slots and `robotd-params` resolves them against the mode's defaults
(`PolicyParams::resolved_with`). The mode picks the *default*, and only the default:

```rust
walk: path(&self.walk, Some(walk_default))
```

`path()` returns an explicit value before it ever looks at the default. So the moment somebody
writes `walk = …`, that file is what loads — in both modes. The mode's contribution disappears
exactly when a person has expressed a preference.

Concretely, on a robot that has been given a retrained gait:

```toml
[policy]
mode = "walk"
walk = "/var/lib/robot/policies/me/gait_v4.onnx"
```

Hold DPad-Up. The robot quacks twice, homes, and comes back up on wheels driving `gait_v4.onnx` —
a walking network, on a roller. `roller.onnx` is never consulted, because the key that would have
fallen back to it is set. Nothing warns; the config names the file it is running and the file is
loadable, so every check passes.

The same is true of `ground_pick`, where the two modes are not even the same *gesture*: in walk
mode that slot is the A-button pick, in roller mode it is the crouch.

There is no way to say "this robot walks with X and rolls with Y", because there is nowhere to
write the second half.

**The set already knows how to do this.** `SetPolicy.mode` exists, and `SetManifest::ground_pick`
takes a mode and returns that mode's entry — which is how the crouch gets its own 5 s cycle after
a board ran it at 3 s for a while (`robotd-params/src/lib.rs`,
`the_set_declares_each_modes_ground_pick`). The published set differentiates. Only the layer above
it, the list of a person's own decisions, cannot.

## 2. Four slots, each named for its mode

Two new slots, no renames:

| slot | mode | default, under `/opt/robot/policies/current/` |
|---|---|---|
| `walk` | walk | `velstand.onnx` |
| `ground_pick` | walk | `alpha_ground_pick.onnx` |
| **`roller`** | roller | `roller.onnx` |
| **`crouch`** | roller | `roller_crouch.onnx` |

`stand`, `sitstand`, `kick_left`, `kick_right` and `roulade` stay single-valued and shared. That is
not an omission: the two modes genuinely run the same sit/stand, the same kicks and the same
roulade. `stand` is unset in both modes since set v5 — velstand stands on its own at zero command,
and the roller skips standing transitions entirely — so it is a slot with no per-mode difference to
express.

`Slot::ALL` goes from seven to nine. The slot name is the mode, which is what removes the question
`robotctl policy load walk …` cannot currently answer: *which mode did you mean?* There is no
`--mode` flag to add and no nested table to parse, because `[policy.walk]` is not available as a
table anyway — `walk` is already a key, and TOML will not have both.

```toml
[policy]
mode        = "roller"
walk        = "/var/lib/robot/policies/me/gait_v4.onnx"    # legs
ground_pick = "/var/lib/robot/policies/me/pick_v2.onnx"    # legs
roller      = "/var/lib/robot/policies/me/roller_v3.onnx"  # wheels
crouch      = "/var/lib/robot/policies/me/crouch_v2.onnx"  # wheels
crouch_period = 2.5
```

## 3. The mode dispatches before it defaults

`resolved_with` chooses the *pair* — key and default together — instead of choosing a default for a
fixed key:

```rust
let (locomotion, locomotion_default, pick, pick_default, period, scale, gain_ratio) =
    match self.mode {
        Mode::Walk   => (&self.walk,   "velstand.onnx", &self.ground_pick,
                         Some("alpha_ground_pick.onnx"), self.ground_pick_period,
                         self.ground_pick_action_scale, self.ground_pick_gain_ratio),
        Mode::Roller => (&self.roller, "roller.onnx",   &self.crouch,
                         Some("roller_crouch.onnx"),     self.crouch_period,
                         self.crouch_action_scale,       self.crouch_gain_ratio),
    };
```

**In roller mode the `walk` and `ground_pick*` keys are not read at all, and in walk mode the
`roller` and `crouch*` keys are not read at all.** No cross-mode fallback: `crouch_period` does not
fall back to `ground_pick_period` when unset, it falls back to the roller's own layers. A fallback
between the two would reintroduce the exact bleed this page exists to remove, one level down and
harder to see.

Within a mode the existing three layers are untouched — the file's key, then that mode's entry in
the set manifest, then the literal:

| resolved value | key | manifest | literal |
|---|---|---|---|
| walk-mode pick cycle | `ground_pick_period` | `manifest.ground_pick(Walk)` | 4.0 |
| roller crouch cycle | `crouch_period` | `manifest.ground_pick(Roller)` | 3.0 |

`walk` keeps its one exception, and only in walk mode: the `"none"` sentinel does not apply to it,
because a robot with no locomotion network has nothing to run. `roller` inherits that exception for
roller mode, for the same reason and by the same route — `drop_unloadable_overrides` clears it and
reports degraded rather than letting `resolved` fall over.

### 3.1 Resolving a slot that is not this mode's

`resolved_with` answers for the mode the robot is in. Two callers need the other question —
*what would this slot load, in the mode it belongs to?* — and both are in §5 and §6. Rather than
let each work it out, one helper on `PolicyParams`:

```rust
/// What `slot` would load, resolved in the mode that slot belongs to rather than the mode the
/// robot is in. `None` for a slot switched off with the `"none"` sentinel.
pub fn resolved_slot_with(&self, slot: Slot, manifest: Option<&SetManifest>) -> Option<PathBuf>

/// The same, against the set installed on this board.
pub fn resolved_slot(&self, slot: Slot) -> Option<PathBuf>
```

For the five shared slots this is what `resolved()` already says. For `walk`/`ground_pick` it
resolves against walk-mode defaults and for `roller`/`crouch` against roller-mode defaults,
whatever `mode` currently is.

One question, two functions, and the pair is still the only new public surface here. The `_with`
form is the one callers use: `set_manifest()` re-parses a JSON file off disk on every call, and
every caller of this asks about all nine slots in a loop — the bare form in that loop is nine
parses of the same file, at boot and again on every report republish. The bare form stays because
it is the pairing `resolved()`/`resolved_with()` already established, and a caller holding no
manifest should not have to learn about the file to ask about one slot. There are two of these
and not three because two callers working the same awkward question out separately is how they
come to disagree.

## 4. The tuning of the crouch

Three new keys, mirroring the ground pick's three exactly:

| new key | twin | type | note |
|---|---|---|---|
| `crouch_period` | `ground_pick_period` | `Option<f64>` | literal 3.0 |
| `crouch_action_scale` | `ground_pick_action_scale` | `Option<f64>` | literal 0.8 |
| `crouch_gain_ratio` | `ground_pick_gain_ratio` | `f64` | literal 1.0 |

`crouch_gain_ratio` is a bare `f64` rather than an `Option` because its twin is, and the twin has
no per-mode resolution to preserve — 1.0 in both modes today. Matching the sibling's shape is worth
more here than a distinction nothing reads.

`action_scale`, `standing_action_scale`, `gain`, `head_lowpass` and `legs_lowpass` stay shared and
keep resolving per mode from their defaults as they do now. They were not part of the ask, and
`action_scale` in particular already resolves 0.9/0.8 per mode with no reported friction.

## 5. What does not change

**`ResolvedPolicy`** keeps every field name it has, `walk` and `ground_pick` included. Its own
doc comment is the argument: *nothing downstream should ever have to ask "walk or roller?" to know
the action scale.* Renaming its fields to `locomotion`/`pick` would push the mode back into the
consumers this type exists to shield. The blast radius stops at `resolved_with`.

That one decision is what keeps the rest of this list short:

- **`robot.loadPolicy`** — `LoadPolicyParams` carries a slot *name*, so two more names need no
  protocol change; `PolicyNames`, which names what is driving, is built from `ResolvedPolicy` and
  sees nothing new. (`robot.policies` is the exception, and it is §5.1.) Its one behaviour change
  is the `"none"` refusal: the method refuses `walk = "none"` because a robot with no locomotion
  network has nothing to run, and under §3 that is true of `roller` on wheels for exactly the
  same reason — so it refuses both, with its own sentence for each, rather than accepting a
  `policy load roller none` it would then ignore until the next boot dropped it.
- **`xtask`'s `policies_robotd_expects`** — the list of files that must exist on a board. It
  already loops over both modes and unions what `ResolvedPolicy::slot()` answers for each, and
  the reason it keeps returning `roller.onnx` and `roller_crouch.onnx` from the roller pass is
  the new mode gate: in that pass `slot(Roller)` and `slot(Crouch)` are the ones that answer, and
  they answer with the roller's defaults. (`ResolvedPolicy` keeping its field names is why it
  still compiles untouched; it is not why the answer is right.) Left alone deliberately; it looks
  like it wants updating and does not.
- **`change_disturbs`** (`robotd/src/main.rs`) compares two `ResolvedPolicy` values, so
  `Driving::Walk => before.walk != after.walk` already covers "the `roller` key changed while the
  roller was driving".
- **`updater::policy`** writes through `Slot::config_key()`, so a fetched policy lands in
  `policy.roller` with no new branch.
- **`robotctl configure`** — its *rows* need no change: they derive from the registry and learn
  the five keys at compile time, which is what `the_registry_covers_every_key_exactly` enforces.
  Its *hints* did. `Model::resolved_hint` (`robotd-params/src/edit.rs`) mapped registry keys onto
  a single `resolved()`, which answers for the mode the robot is in — so on a wheeled robot the
  `policy.walk` row hinted `roller.onnx`, pre-filled the edit box with it, and took whatever the
  operator then typed into a key roller mode does not read. That is §1 again, through the one
  tool this list claimed was untouched. The eight mode-specific policy keys resolve through
  [§3.1](#31-resolving-a-slot-that-is-not-this-modes) instead, each in its own mode, and they are
  the only hints in that function that no longer move when `mode` does.

### 5.1 The one report that does change

`slot_report` (`robotd/src/main.rs`) is what `robot.policies` and `robotctl policy list` are made
of: one row per slot, with the resolved path, its origin, whether config overrode it, and any
error. It loops `Slot::ALL` and asks `ResolvedPolicy::slot()`, which only knows the current mode.

Left alone, a robot in walk mode with `roller` set would report the row `roller` with **no path,
origin `None`, and `overridden: true`** — which reads as "switched off" for a slot that is in fact
configured and will load the moment somebody holds DPad-Up. A report that is wrong about a slot is
worse than a report that omits it; this one is how a person checks what their robot is running.

So `slot_report` resolves through [§3.1](#31-resolving-a-slot-that-is-not-this-modes) instead, and
every row carries a real path. `proto::PolicySlot` gains one field:

```rust
/// Which drive mode reads this slot — `None` for the five that both do. A client shows the
/// four mode-specific rows without having to know which names those are.
pub mode: Option<String>,
```

That is additive on the wire, and it is the only protocol change in this design. Without it a
client has to hardcode `["walk", "ground_pick", "roller", "crouch"]` to render the list
intelligibly, and a tenth slot later would silently render wrong.

## 6. A slot belonging to the mode you are not in

`drop_unloadable_overrides` (`robotd/src/main.rs`) walks `Slot::ALL` at startup, asks
`ResolvedPolicy::slot()` for the path, validates the ONNX, and on failure clears the override and
reports degraded. With four mode-specific slots, `cfg.slot(Slot::Roller)` in walk mode has no
obvious answer — `ResolvedPolicy` only carries the mode it resolved.

**Decision: validate all nine paths at boot regardless of mode.** The check is the config's
override, not the resolved value, so it does not need `ResolvedPolicy` to answer: read the raw
`PolicyParams` slot, resolve it to a path the way `resolved_with` would for *that slot's* mode, and
validate it.

The alternative — return `None` for the other mode's slots and skip them — moves the discovery of a
broken path from boot to the DPad-Up that loads it, which is the one moment it takes something
down, usually with the robot on a table. The whole point of this function is that one bad line in a
file must not brick the board, and a line that only bricks it on Tuesday is still that.

Cost: two extra ONNX loads at startup on a robot that has overridden both modes. A robot that has
overridden neither — the normal case — pays nothing, because the loop already skips unset slots.

## 7. Migration

Additive at the schema. The five keys are new, and `Params` has `#[serde(default)]`, so every
existing `robotd.toml` parses unchanged and every robot with unset slots — the normal case —
resolves exactly as it does today.

Not additive at the resolution. **A robot already on `mode = "roller"` stops reading five keys it
used to read**, because §3 dispatches on the mode before it defaults and there is no cross-mode
fallback:

| key it stops reading on wheels | what the robot comes back with | re-apply it as |
|---|---|---|
| `walk` | `roller.onnx`, or `roller` if that is set | `roller` |
| `ground_pick` | `roller_crouch.onnx`, or `crouch` if that is set | `crouch` |
| `ground_pick_period` | the set's roller entry, then 3.0 | `crouch_period` |
| `ground_pick_action_scale` | the set's roller entry, then 0.8 | `crouch_action_scale` |
| `ground_pick_gain_ratio` | **1.0** | `crouch_gain_ratio` |

The first row is the bug and is why this page exists: that robot was driving a walking network on
wheels, which is not a configuration anyone chose, it is the failure in §1. No compat shim —
silently honouring `walk` in roller mode would preserve the thing being fixed. `roller` is where
that robot's intent goes if it really wanted a custom roller network.

**The last row is the sharp one: it is the only one that changes how the hardware behaves rather
than which file loads.** `ground_pick_gain_ratio` is a bare `f64` that both modes read today, so a
roller deliberately tuned to `ground_pick_gain_ratio = 0.6` for a softer crouch comes back after
this change at `crouch_gain_ratio`'s literal 1.0 — servos noticeably stiffer through every crouch,
on real hardware, with nothing pointing at the release that did it. The other four rows surface
somewhere: a changed path shows in `policy list` and a changed period is visible in the motion. A
gain ratio has no report row, no health line and nothing in the log, because from the daemon's
side nothing failed — a key the current mode does not read is not an error, and warning about one
would mean warning on every wheeled robot that has `walk` set on purpose.

So, plainly: **a roller owner who tuned any `ground_pick_*` key must re-apply it under the
`crouch_*` name.** That table is the whole migration. `robotctl configure` lists both families of
keys with what each resolves to, and `robotctl policy list` shows all nine slots, so what a robot
has set under either name is one command away.

## 8. Tests

TDD, in this order. Each step's tests fail before its implementation exists:

1. **Resolution** (`robotd-params`) — roller mode reads `roller`/`crouch` and ignores `walk`/
   `ground_pick`; walk mode the reverse; `crouch_period` unset does *not* pick up a set
   `ground_pick_period`; the set manifest's per-mode entry still beats the literal and still loses
   to the key. The existing `resolved()` tests are the mould.
2. **Schema** — the five fields, defaults, `deny_unknown_fields` still holding.
3. **`Slot`** — nine variants; `every_slot_is_a_registry_key` and
   `the_registry_covers_every_key_exactly` go red until the registry entries exist, which is the
   point of having them.
4. **`resolved_slot`** (§3.1) — a slot resolves in its own mode, not the robot's: `resolved_slot`
   for `Slot::Roller` names `roller.onnx` on a robot whose `mode` is `walk`, and the five shared
   slots agree with `resolved()` in both modes.
5. **`drop_unloadable_overrides`** — a bad `roller` path is cleared and reported while the robot is
   in walk mode, via `drop_unloadable_overrides_with` so it runs on a machine with no ONNX Runtime.
6. **`slot_report`** (§5.1) — nine rows; a robot in walk mode with `roller` set reports that row
   with a path and `overridden: true`, never a path-less row, and each mode-specific row carries
   its `mode`.

**One existing test changes on purpose.** `a_config_key_overrides_the_sets_ground_pick_timing`
builds `PolicyParams { mode: Roller, ground_pick_period: Some(6.0), … }` and expects 6.0. Under §3
that key is not read in roller mode. It becomes `crouch_period: Some(6.0)`, and the assertion it is
making — *the file is a list of decisions, a key still beats the set* — is preserved exactly. This
is the one diff in this change that will look like a regression and is not.

## 9. What this amends elsewhere

- `deploy/robotd.toml` — the commented reference gains the five keys and the roller paragraph stops
  describing the shared-slot behaviour.
- `duck-ipc-proto` — `PolicySlot::mode`, additive (§5.1).
- `robotctl policy list` — nine rows; the four mode-specific ones want marking as such, from the
  new field rather than from a hardcoded list of names.
- [`policy-channel-design.md`](policy-channel-design.md) §2 — its slot list is seven names; it
  becomes nine.
- `docs/policy-manifest.md` — the `slot` field's row names the slots a `policy load <slot>` line can
  take.

Written against `f8ba931`, so the walk-mode defaults here are set v5's: `velstand.onnx` with no
standing network.
