# Per-Mode Policy Slots Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give roller mode its own `[policy]` slots — `roller`, `crouch`, and the crouch's three tuning keys — so a robot can be configured to walk with one network and roll with another.

**Architecture:** `PolicyParams` gains five fields. `resolved_with` dispatches on `mode` to choose *which keys it reads*, not just which defaults it falls back to — walk mode never reads `roller`/`crouch*`, roller mode never reads `walk`/`ground_pick*`, and there is no cross-mode fallback between them. `ResolvedPolicy` keeps every field name it has, so the change stops at `resolved_with` and nothing downstream learns about modes. A new `PolicyParams::resolved_slot_with` answers "what would this slot load, in the mode it belongs to?" for the two callers that must reason about a slot the robot is not currently using.

**Tech Stack:** Rust 2021, `serde` + `toml`, workspace crates `robotd-params`, `robotd`, `duck-ipc-proto`. Tests are `#[cfg(test)] mod tests` inline in `lib.rs`/`main.rs`, addressed with `super::`.

**Spec:** `docs/design/per-mode-policy-slots-design.md`

## Global Constraints

- **Written against `f8ba931`.** Walk-mode defaults are set v5's: locomotion `velstand.onnx`, `stand` unset in both modes, `sitstand` `alpha_sitstand.onnx` in both.
- **No cross-mode fallback.** `crouch_period` unset resolves to the roller's manifest entry then the roller literal `3.0`. It must never read `ground_pick_period`. Same for every other pair.
- **`ResolvedPolicy` field names do not change.** Not `walk` → `locomotion`, not `ground_pick` → `pick`. Its doc comment is the reason: nothing downstream should have to ask "walk or roller?".
- **Mode literals, exact:** locomotion default `velstand.onnx` (walk) / `roller.onnx` (roller); pick default `alpha_ground_pick.onnx` (walk) / `roller_crouch.onnx` (roller); period `4.0` / `3.0`; action scale `1.0` / `0.8`; gain ratio `1.0` / `1.0`.
- **`set_manifest()` is not cached** — it reads and parses `/opt/robot/policies/current/manifest.json` on every call. Any new API that resolves must follow the existing `resolved` / `resolved_with` pair so a caller in a loop reads the manifest once.
- **Per-crate test runs**, per `CONTRIBUTING.md`: `cargo test -p <crate>`. Format with `cargo fmt --all` before each commit.
- **Commit trailer**, every commit in this plan:
  `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`

---

### Task 1: The five keys, and the mode dispatch

The schema and the resolution, together with the registry entries they require. These cannot be split: `the_registry_covers_every_key_exactly` walks serde's own field list, so the moment the fields exist without registry entries the crate is red.

**Files:**
- Modify: `robotd-params/src/lib.rs` — `PolicyParams` fields (~line 845), `Default for PolicyParams` (~line 1734), `resolved_with` (~line 1560)
- Modify: `robotd-params/src/registry.rs` — five entries in the `[policy]` block (~line 121)
- Modify: `deploy/robotd.toml` — the commented reference (~line 92)
- Test: `robotd-params/src/lib.rs`, inline `mod tests`

**Interfaces:**
- Consumes: nothing — first task.
- Produces: `PolicyParams.roller: Option<PathBuf>`, `.crouch: Option<PathBuf>`, `.crouch_period: Option<f64>`, `.crouch_action_scale: Option<f64>`, `.crouch_gain_ratio: f64`. Registry keys `policy.roller`, `policy.crouch` (both `Kind::OptionalPath`), `policy.crouch_period`, `policy.crouch_action_scale` (both `Kind::OptionalFloat`), `policy.crouch_gain_ratio` (`Kind::Float`). Private `fn mode_slots(&self) -> ModeSlots<'_>` on `PolicyParams`, reading `self.mode`.

- [ ] **Step 1: Write the failing tests**

Add to `mod tests` in `robotd-params/src/lib.rs`:

```rust
/// **A mode reads its own keys and only its own.** `[policy]` had one `walk` key for both
/// modes, so a robot given a retrained gait and then switched to the roller drove a walking
/// network on wheels — the mode picked the default and an explicit value beat the default.
/// Four slots named for their mode is the fix, and this is the half of it that matters: the
/// keys of the mode you are not in are not read at all.
#[test]
fn each_mode_reads_only_its_own_slots() {
    let both = super::PolicyParams {
        walk: Some(std::path::PathBuf::from("/legs/gait.onnx")),
        ground_pick: Some(std::path::PathBuf::from("/legs/pick.onnx")),
        roller: Some(std::path::PathBuf::from("/wheels/roller.onnx")),
        crouch: Some(std::path::PathBuf::from("/wheels/crouch.onnx")),
        ..Default::default()
    };

    let walking = both.clone().resolved_with(None);
    assert_eq!(walking.walk, std::path::PathBuf::from("/legs/gait.onnx"));
    assert_eq!(
        walking.ground_pick,
        Some(std::path::PathBuf::from("/legs/pick.onnx"))
    );

    let rolling = super::PolicyParams {
        mode: super::Mode::Roller,
        ..both
    }
    .resolved_with(None);
    assert_eq!(
        rolling.walk,
        std::path::PathBuf::from("/wheels/roller.onnx"),
        "the roller drives its own network, not the one in the walk key"
    );
    assert_eq!(
        rolling.ground_pick,
        Some(std::path::PathBuf::from("/wheels/crouch.onnx")),
        "and the crouch, not the legs' ground pick"
    );
}

/// The bug in one assertion: a robot configured for legs, switched to wheels, must come back
/// up on the roller's own default rather than the gait somebody loaded for walking.
#[test]
fn a_walk_override_does_not_follow_the_robot_onto_wheels() {
    let rolling = super::PolicyParams {
        mode: super::Mode::Roller,
        walk: Some(std::path::PathBuf::from("/legs/gait_v4.onnx")),
        ..Default::default()
    }
    .resolved_with(None);
    assert_eq!(
        rolling.walk,
        std::path::PathBuf::from(super::POLICY_DIR).join("roller.onnx"),
        "roller.onnx, not gait_v4.onnx"
    );
}

/// **No cross-mode fallback on the tuning either.** `crouch_period` unset resolves through the
/// roller's own layers — the set's roller entry, then the roller literal — and never picks up a
/// `ground_pick_period` written for the legs. A fallback between the two would be the same bleed
/// one level down and harder to see.
#[test]
fn the_crouch_tuning_never_reads_the_ground_picks() {
    let rolling = super::PolicyParams {
        mode: super::Mode::Roller,
        ground_pick_period: Some(9.0),
        ground_pick_action_scale: Some(0.1),
        ..Default::default()
    }
    .resolved_with(None);
    assert_eq!(rolling.ground_pick_period, 3.0, "the roller literal");
    assert_eq!(rolling.ground_pick_action_scale, 0.8, "the roller literal");

    let tuned = super::PolicyParams {
        mode: super::Mode::Roller,
        crouch_period: Some(2.5),
        crouch_action_scale: Some(0.6),
        ..Default::default()
    }
    .resolved_with(None);
    assert_eq!(tuned.ground_pick_period, 2.5);
    assert_eq!(tuned.ground_pick_action_scale, 0.6);
}

/// Walk mode is the mirror: the wheels' keys are inert.
#[test]
fn the_roller_keys_are_inert_while_walking() {
    let walking = super::PolicyParams {
        roller: Some(std::path::PathBuf::from("/wheels/roller.onnx")),
        crouch: Some(std::path::PathBuf::from("/wheels/crouch.onnx")),
        crouch_period: Some(2.5),
        ..Default::default()
    }
    .resolved_with(None);
    assert_eq!(
        walking.walk,
        std::path::PathBuf::from(super::POLICY_DIR).join("velstand.onnx")
    );
    assert_eq!(
        walking.ground_pick,
        Some(std::path::PathBuf::from(super::POLICY_DIR).join("alpha_ground_pick.onnx"))
    );
    assert_eq!(walking.ground_pick_period, 4.0, "the walk literal");
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p robotd-params each_mode_reads_only_its_own_slots`
Expected: FAIL to compile — `struct 'PolicyParams' has no field named 'roller'`.

- [ ] **Step 3: Add the five fields**

In `robotd-params/src/lib.rs`, after `pub roulade: Option<PathBuf>,` (~line 856):

```rust
    /// The roller's locomotion network — roller mode's `walk`.
    ///
    /// A separate key rather than a per-mode meaning for `walk`, because the mode used to pick
    /// only the *default*: an explicit `walk = …` loaded on wheels too, so a robot given a
    /// retrained gait and switched to the roller drove a walking network with nothing to warn
    /// about it. The slot name is the mode. Unset resolves to `roller.onnx` in the set.
    pub roller: Option<PathBuf>,
    /// The roller's ground pick — the crouch. Roller mode's `ground_pick`.
    ///
    /// Not the same gesture as the legs' pick, which is why it is not the same key: on wheels
    /// the A button crouches. Unset resolves to `roller_crouch.onnx` in the set.
    pub crouch: Option<PathBuf>,
```

And after `pub ground_pick_gain_ratio: f64,` (~line 880):

```rust
    /// One crouch cycle, seconds. Roller mode's `ground_pick_period`. Absent resolves through
    /// the roller's own layers — the set's roller-tagged entry, then 3.0 — and **never** reads
    /// `ground_pick_period`, which belongs to the legs.
    pub crouch_period: Option<f64>,
    /// Action scale while the crouch runs. Roller mode's `ground_pick_action_scale`. Absent:
    /// the set's roller entry, else 0.8.
    pub crouch_action_scale: Option<f64>,
    /// Gain multiplier during the crouch. Roller mode's `ground_pick_gain_ratio`.
    ///
    /// A bare `f64` rather than an `Option` because its twin is one, and the twin has no
    /// per-mode resolution to preserve — 1.0 in both modes. Matching the sibling's shape is
    /// worth more than a distinction nothing reads.
    pub crouch_gain_ratio: f64,
```

In `Default for PolicyParams` (~line 1734), add alongside the existing entries:

```rust
            roller: None,
            crouch: None,
```

and

```rust
            crouch_period: None,
            crouch_action_scale: None,
            crouch_gain_ratio: 1.0,
```

- [ ] **Step 4: Add the registry entries**

In `robotd-params/src/registry.rs`, in the `[policy]` block: put `roller` immediately after the `policy.walk` entry and `crouch` immediately after `policy.ground_pick`, so the registry reads in the same order the shipped file does.

```rust
    entry(
        "policy.roller",
        Kind::OptionalPath,
        "Roller-mode locomotion; unset = this robot's own. Walk mode does not read it",
    ),
```

```rust
    entry(
        "policy.crouch",
        Kind::OptionalPath,
        "Roller-mode ground pick (the crouch); unset = this robot's own",
    ),
```

And after the three `policy.ground_pick_*` tuning entries:

```rust
    entry(
        "policy.crouch_period",
        Kind::OptionalFloat,
        "One crouch cycle, seconds; unset resolves from the set then 3.0",
    ),
    entry(
        "policy.crouch_action_scale",
        Kind::OptionalFloat,
        "Action scale during the crouch; unset resolves from the set then 0.8",
    ),
    entry(
        "policy.crouch_gain_ratio",
        Kind::Float,
        "Gain multiplier during the crouch",
    ),
```

- [ ] **Step 5: Dispatch the mode in `resolved_with`**

In `robotd-params/src/lib.rs`, add above `impl PolicyParams`'s `resolved_with`:

```rust
/// The half of `[policy]` that belongs to one drive mode: which keys it reads, and what they
/// fall back to when unset.
///
/// It exists so the mode is consulted **once**, before anything is resolved, rather than at each
/// `unwrap_or` — which is what let an explicit `walk = …` load on wheels. Walking reads
/// `walk`/`ground_pick*`; the roller reads `roller`/`crouch*`; neither ever reads the other's.
/// See `docs/design/per-mode-policy-slots-design.md` §3.
struct ModeSlots<'a> {
    locomotion: &'a Option<PathBuf>,
    locomotion_default: &'static str,
    pick: &'a Option<PathBuf>,
    pick_default: Option<&'static str>,
    period: Option<f64>,
    period_default: f64,
    action_scale: Option<f64>,
    action_scale_default: f64,
    gain_ratio: f64,
}
```

Then, as a method on `PolicyParams`:

```rust
    /// The keys and defaults belonging to the mode this robot is in.
    ///
    /// No `mode` parameter: `resolved_slot_with` asks about another mode by cloning and setting
    /// `mode`, so a parameter here would be flexibility nothing uses.
    fn mode_slots(&self) -> ModeSlots<'_> {
        match self.mode {
            Mode::Walk => ModeSlots {
                locomotion: &self.walk,
                // The velstand gait (set v5) walks on a twist and stands still at zero command.
                locomotion_default: "velstand.onnx",
                pick: &self.ground_pick,
                pick_default: Some("alpha_ground_pick.onnx"),
                period: self.ground_pick_period,
                period_default: 4.0,
                action_scale: self.ground_pick_action_scale,
                action_scale_default: 1.0,
                gain_ratio: self.ground_pick_gain_ratio,
            },
            Mode::Roller => ModeSlots {
                locomotion: &self.roller,
                locomotion_default: "roller.onnx",
                pick: &self.crouch,
                pick_default: Some("roller_crouch.onnx"),
                period: self.crouch_period,
                period_default: 3.0,
                action_scale: self.crouch_action_scale,
                action_scale_default: 0.8,
                gain_ratio: self.crouch_gain_ratio,
            },
        }
    }
```

In `resolved_with`, replace the four-element match with the two shared slots plus the dispatch:

```rust
        // `stand` and `sitstand` are the same in both modes today — velstand stands on its own
        // at zero command, and the roller skips standing transitions — but they stay a match
        // rather than two constants, because that is the shape a divergence goes back into.
        let (stand, sitstand) = match self.mode {
            Mode::Walk => (None, Some("alpha_sitstand.onnx")),
            Mode::Roller => (None, Some("alpha_sitstand.onnx")),
        };
        let slots = self.mode_slots();
```

Then change the four consumers inside the `ResolvedPolicy { … }` literal:

```rust
            walk: path(slots.locomotion, Some(slots.locomotion_default))
                .unwrap_or_else(|| PathBuf::from(POLICY_DIR).join(slots.locomotion_default)),
            stand: path(&self.stand, stand),
            sitstand: path(&self.sitstand, sitstand),
            ground_pick: path(slots.pick, slots.pick_default),
```

```rust
            ground_pick_period: slots
                .period
                .or(pick.map(|t| t.period_s))
                .unwrap_or(slots.period_default),
```

```rust
            ground_pick_action_scale: slots
                .action_scale
                .or(pick.and_then(|t| t.action_scale))
                .unwrap_or(slots.action_scale_default),
```

```rust
            ground_pick_gain_ratio: slots.gain_ratio,
```

Leave `action_scale` (0.9/0.8) exactly as it is — it is shared, and not part of this change.

- [ ] **Step 6: Fix the one existing test this breaks on purpose**

`a_config_key_overrides_the_sets_ground_pick_timing` builds `PolicyParams { mode: Roller, ground_pick_period: Some(6.0), ground_pick_action_scale: Some(0.7), … }` and expects 6.0 and 0.7. Those keys are not read in roller mode any more. Rename the two fields in the fixture — `crouch_period: Some(6.0)`, `crouch_action_scale: Some(0.7)` — and leave every assertion untouched. What it asserts, *the file is a list of decisions and a key still beats the set*, is preserved exactly.

Add one line to its doc comment so the next reader knows why it names the crouch keys:

```rust
    /// The file is a list of decisions: a `[policy]` key still beats the set. Roller mode's keys
    /// are `crouch_*`; `ground_pick_*` are the legs' and are not read here.
```

- [ ] **Step 7: Update the shipped reference**

In `deploy/robotd.toml`, in the `[policy]` block: the paragraph describing the slots currently tells a roller owner that the mode changes which policies load. Replace the roller sentences with the four-slot table, and add the new keys commented out beside their twins.

```toml
# Drive mode: "walk" (legs, the default) or "roller". The mode picks which *slots* are read,
# not just their defaults: walking reads walk/ground_pick and the ground_pick_* tuning, the
# roller reads roller/crouch and the crouch_* tuning, and neither reads the other's. So a robot
# can be given a gait for its legs and a different network for its wheels, and switching modes
# is one line plus a restart (or a held DPad-Up, which switches live without writing config).
mode = "walk"
```

```toml
# Roller-mode slots. Same rules as the walk-mode pair above — unset means the set's own file,
# "none" switches an optional slot off, `robotctl policy load roller <file>` writes the key.
# roller = "/opt/robot/policies/current/roller.onnx"        locomotion on wheels
# crouch = "/opt/robot/policies/current/roller_crouch.onnx" the A-button crouch

# The crouch's timing, as ground_pick_* is the legs' pick's. Unset resolves from the installed
# set's roller-tagged entry, then from 3.0 / 0.8 / 1.0.
# crouch_period = 3.0
# crouch_action_scale = 0.8
# crouch_gain_ratio = 1.0
```

- [ ] **Step 8: Run the tests to verify they pass**

Run: `cargo fmt --all && cargo test -p robotd-params`
Expected: PASS, all of it. `the_registry_covers_every_key_exactly` and `choices_match_the_types` must be green — they are what proves the registry learned the five keys.

- [ ] **Step 9: Commit**

```bash
git add robotd-params/src/lib.rs robotd-params/src/registry.rs deploy/robotd.toml
git commit -m "$(cat <<'EOF'
The roller reads its own slots, and walking never reads them

`[policy]` had one `walk` key and one `ground_pick` key, and the mode chose
only their defaults — so an explicit `walk = …` loaded on wheels too. Five
new keys, and `resolved_with` now picks which keys to read before it resolves
anything.

No cross-mode fallback: `crouch_period` unset goes to the set's roller entry
then 3.0, never to `ground_pick_period`.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: `Slot::Roller` and `Slot::Crouch`

Nine slots, so `robotctl policy load roller <repo>`, `robot.loadPolicy`, `updater::policy` and the `configure` editor all reach the new keys with no code of their own.

**Files:**
- Modify: `robotd-params/src/lib.rs` — `Slot` (~line 1360), `Slot::ALL`, `as_str`, `ResolvedPolicy::slot` (~line 1464), `PolicyParams::slot`/`set_slot` (~line 1529)
- Modify: `docs/design/policy-channel-design.md` — §2's slot list
- Modify: `docs/policy-manifest.md` — the `slot` row
- Test: `robotd-params/src/lib.rs`, inline `mod tests`

**Interfaces:**
- Consumes: Task 1's registry keys `policy.roller` and `policy.crouch` — `every_slot_is_a_registry_key` fails without them.
- Produces: `Slot::Roller` (`as_str` → `"roller"`), `Slot::Crouch` (`as_str` → `"crouch"`), `Slot::ALL` of length 9.

- [ ] **Step 1: Write the failing tests**

```rust
/// **A slot the robot is not in is not running.** `ResolvedPolicy` carries one mode's answer,
/// so the other mode's slots report empty — and `PolicyParams::slot` still reports the key,
/// because the config says what it says regardless of which mode is live.
#[test]
fn the_other_modes_slots_resolve_empty() {
    let walking = super::PolicyParams {
        roller: Some(std::path::PathBuf::from("/wheels/roller.onnx")),
        crouch: Some(std::path::PathBuf::from("/wheels/crouch.onnx")),
        ..Default::default()
    };
    let cfg = walking.resolved_with(None);

    assert!(cfg.slot(super::Slot::Walk).is_some());
    assert!(cfg.slot(super::Slot::GroundPick).is_some());
    assert_eq!(cfg.slot(super::Slot::Roller), None, "not running on legs");
    assert_eq!(cfg.slot(super::Slot::Crouch), None, "not running on legs");

    assert!(
        walking.slot(super::Slot::Roller).is_some(),
        "the key is set, whatever mode the robot is in"
    );
}

/// The mirror, and the one that matters: on wheels, `walk` and `ground_pick` are the empty ones.
#[test]
fn on_wheels_the_legs_slots_resolve_empty() {
    let cfg = super::PolicyParams {
        mode: super::Mode::Roller,
        ..Default::default()
    }
    .resolved_with(None);

    assert_eq!(cfg.slot(super::Slot::Walk), None);
    assert_eq!(cfg.slot(super::Slot::GroundPick), None);
    assert!(
        cfg.slot(super::Slot::Roller)
            .is_some_and(|p| p.ends_with("roller.onnx"))
    );
    assert!(
        cfg.slot(super::Slot::Crouch)
            .is_some_and(|p| p.ends_with("roller_crouch.onnx"))
    );
}

/// `set_slot` reaches the new fields, which is what `robot.loadPolicy` writes through.
#[test]
fn set_slot_writes_the_roller_keys() {
    let mut p = super::PolicyParams::default();
    p.set_slot(
        super::Slot::Roller,
        Some(std::path::PathBuf::from("/wheels/v3.onnx")),
    );
    p.set_slot(
        super::Slot::Crouch,
        Some(std::path::PathBuf::from("/wheels/crouch_v2.onnx")),
    );
    assert_eq!(
        p.roller.as_deref(),
        Some(std::path::Path::new("/wheels/v3.onnx"))
    );
    assert_eq!(
        p.crouch.as_deref(),
        Some(std::path::Path::new("/wheels/crouch_v2.onnx"))
    );
}

/// The names a person types, and the ones a client sends.
#[test]
fn the_new_slots_parse_by_name() {
    assert_eq!(super::Slot::parse("roller"), Some(super::Slot::Roller));
    assert_eq!(super::Slot::parse("crouch"), Some(super::Slot::Crouch));
    assert_eq!(super::Slot::ALL.len(), 9);
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p robotd-params the_other_modes_slots_resolve_empty`
Expected: FAIL to compile — `no variant named 'Roller' found for enum 'Slot'`.

- [ ] **Step 3: Add the two variants**

In `robotd-params/src/lib.rs`, in `enum Slot`, after `Walk` and after `GroundPick` respectively, so the enum reads in the file's own order:

```rust
pub enum Slot {
    Walk,
    Roller,
    Stand,
    SitStand,
    GroundPick,
    Crouch,
    KickLeft,
    KickRight,
    Roulade,
}
```

`Slot::ALL` takes the same order and its length becomes 9:

```rust
    pub const ALL: [Slot; 9] = [
        Slot::Walk,
        Slot::Roller,
        Slot::Stand,
        Slot::SitStand,
        Slot::GroundPick,
        Slot::Crouch,
        Slot::KickLeft,
        Slot::KickRight,
        Slot::Roulade,
    ];
```

`as_str` gains two arms:

```rust
            Slot::Roller => "roller",
            Slot::Crouch => "crouch",
```

`Slot::SKILLS` is unchanged — the new two are not one-shot skills.

- [ ] **Step 4: Teach the three `match`es the new slots**

`PolicyParams::slot` (~line 1529):

```rust
            Slot::Roller => &self.roller,
            Slot::Crouch => &self.crouch,
```

`PolicyParams::set_slot` (~line 1543):

```rust
            Slot::Roller => &mut self.roller,
            Slot::Crouch => &mut self.crouch,
```

`ResolvedPolicy::slot` (~line 1464) is the one with a decision in it. A `ResolvedPolicy` is one mode's answer, so a slot belonging to the other mode is not running:

```rust
    /// The file that will actually be loaded into one slot, after mode defaults are applied.
    /// `None` means the slot is empty — a capability this robot does not have.
    ///
    /// **The four mode-specific slots answer only for the mode this resolved.** On legs,
    /// `Roller` and `Crouch` are empty because nothing is driving them; on wheels, `Walk` and
    /// `GroundPick` are. That is the honest answer to "what is in this slot right now" — for
    /// "what *would* this slot load", which is a different question, see
    /// [`PolicyParams::resolved_slot_with`].
    pub fn slot(&self, slot: Slot) -> Option<&std::path::Path> {
        match slot {
            Slot::Walk => (self.mode == Mode::Walk).then_some(self.walk.as_path()),
            Slot::Roller => (self.mode == Mode::Roller).then_some(self.walk.as_path()),
            Slot::Stand => self.stand.as_deref(),
            Slot::SitStand => self.sitstand.as_deref(),
            Slot::GroundPick => (self.mode == Mode::Walk)
                .then(|| self.ground_pick.as_deref())
                .flatten(),
            Slot::Crouch => (self.mode == Mode::Roller)
                .then(|| self.ground_pick.as_deref())
                .flatten(),
            Slot::KickLeft => self.kick_left.as_deref(),
            Slot::KickRight => self.kick_right.as_deref(),
            Slot::Roulade => self.roulade.as_deref(),
        }
    }
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo fmt --all && cargo test -p robotd-params`
Expected: PASS. `every_slot_is_a_registry_key` is the one to watch — it walks all nine and demands each be a registry key of kind `OptionalPath`, which Task 1 provided.

- [ ] **Step 6: Check the whole workspace still builds**

Run: `cargo test --workspace`
Expected: `robotd` may fail to compile on non-exhaustive `match` over `Slot`. That is Task 4's work; if `robotd` fails **only** on missing `Slot::Roller`/`Slot::Crouch` arms, that is expected here. Any other crate failing is not — stop and report it.

- [ ] **Step 7: Update the two docs**

`docs/design/policy-channel-design.md` §2, first line of the section — the slot list is seven names and becomes nine:

```markdown
A slot — `walk`, `roller`, `stand`, `sitstand`, `ground_pick`, `crouch`, `kick_left`,
`kick_right`, `roulade` — is filled from exactly one of three origins:
```

Add a sentence at the end of that paragraph:

```markdown
`walk`/`ground_pick` are walk mode's and `roller`/`crouch` are roller mode's; the other five are
shared. Which mode reads which is
[`per-mode-policy-slots-design.md`](per-mode-policy-slots-design.md), not this page — here they
are nine slots filled the same way.
```

`docs/policy-manifest.md`, the `slot` row of the field table:

```markdown
| `slot` | str | display | for a perpetual gait: the slot it is for (`walk`, `roller`, `stand`, …), so `policy load <slot> <repo>` is the install line |
```

- [ ] **Step 8: Commit**

```bash
git add robotd-params/src/lib.rs docs/design/policy-channel-design.md docs/policy-manifest.md
git commit -m "$(cat <<'EOF'
Nine slots: `roller` and `crouch` are names a person can load

`Slot` is what turns the string a client sends into the key that gets written,
so two variants is all `robotctl policy load roller <repo>`, `robot.loadPolicy`
and `updater::policy` need to reach the new keys.

`ResolvedPolicy::slot` answers for the mode it resolved: on legs the roller's
slots are empty, because nothing is driving them.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: `resolved_slot_with` — what a slot would load, in its own mode

Two callers need the question `ResolvedPolicy::slot` deliberately does not answer. One helper, so they cannot come to disagree.

**Files:**
- Modify: `robotd-params/src/lib.rs` — `impl Slot` (~line 1383), `impl PolicyParams` (~line 1556)
- Test: `robotd-params/src/lib.rs`, inline `mod tests`

**Interfaces:**
- Consumes: `Slot::Roller`, `Slot::Crouch` from Task 2; `PolicyParams::mode_slots` from Task 1.
- Produces:
  - `Slot::mode(self) -> Option<Mode>` — `Some(Mode::Walk)` for `Walk`/`GroundPick`, `Some(Mode::Roller)` for `Roller`/`Crouch`, `None` for the five shared.
  - `PolicyParams::resolved_slot_with(&self, slot: Slot, manifest: Option<&SetManifest>) -> Option<PathBuf>`
  - `PolicyParams::resolved_slot(&self, slot: Slot) -> Option<PathBuf>` — the convenience that reads the manifest itself.

- [ ] **Step 1: Write the failing tests**

```rust
/// **A slot resolves in the mode it belongs to, not the one the robot is in.** Two callers need
/// this — the boot-time validation and the slot report — and both are about a slot that is not
/// currently driving. `ResolvedPolicy::slot` answers "what is running"; this answers "what
/// would load", which is the question you need to catch a broken path before it is loaded.
#[test]
fn a_slot_resolves_in_its_own_mode() {
    let walking = super::PolicyParams::default();
    assert_eq!(walking.mode, super::Mode::Walk);

    assert!(
        walking
            .resolved_slot_with(super::Slot::Roller, None)
            .is_some_and(|p| p.ends_with("roller.onnx")),
        "the roller's default, on a robot standing on its legs"
    );
    assert!(
        walking
            .resolved_slot_with(super::Slot::Crouch, None)
            .is_some_and(|p| p.ends_with("roller_crouch.onnx"))
    );
    assert!(
        walking
            .resolved_slot_with(super::Slot::Walk, None)
            .is_some_and(|p| p.ends_with("velstand.onnx"))
    );
}

/// An override is honoured whichever mode is live — that is the point, since the caller is
/// checking a file it is not about to load.
#[test]
fn resolved_slot_honours_an_override_from_the_other_mode() {
    let walking = super::PolicyParams {
        roller: Some(std::path::PathBuf::from("/wheels/v3.onnx")),
        ..Default::default()
    };
    assert_eq!(
        walking.resolved_slot_with(super::Slot::Roller, None),
        Some(std::path::PathBuf::from("/wheels/v3.onnx"))
    );
}

/// The five shared slots answer the same in both modes, and agree with `resolved()`.
#[test]
fn a_shared_slot_resolves_the_same_in_both_modes() {
    for mode in [super::Mode::Walk, super::Mode::Roller] {
        let p = super::PolicyParams {
            mode,
            ..Default::default()
        };
        let cfg = p.resolved_with(None);
        for slot in [
            super::Slot::Stand,
            super::Slot::SitStand,
            super::Slot::KickLeft,
            super::Slot::KickRight,
            super::Slot::Roulade,
        ] {
            assert_eq!(
                p.resolved_slot_with(slot, None).as_deref(),
                cfg.slot(slot),
                "{slot} disagrees with resolved() in {mode:?}"
            );
        }
    }
}

/// The `"none"` sentinel still switches an optional slot off, whichever mode asked.
#[test]
fn resolved_slot_respects_the_none_sentinel() {
    let p = super::PolicyParams {
        crouch: Some(std::path::PathBuf::from("none")),
        ..Default::default()
    };
    assert_eq!(p.resolved_slot_with(super::Slot::Crouch, None), None);
}

/// Which mode owns which slot.
#[test]
fn slots_know_their_mode() {
    assert_eq!(super::Slot::Walk.mode(), Some(super::Mode::Walk));
    assert_eq!(super::Slot::GroundPick.mode(), Some(super::Mode::Walk));
    assert_eq!(super::Slot::Roller.mode(), Some(super::Mode::Roller));
    assert_eq!(super::Slot::Crouch.mode(), Some(super::Mode::Roller));
    assert_eq!(super::Slot::Stand.mode(), None, "shared");
    assert_eq!(super::Slot::Roulade.mode(), None, "shared");
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p robotd-params a_slot_resolves_in_its_own_mode`
Expected: FAIL to compile — `no method named 'resolved_slot_with' found`.

- [ ] **Step 3: Add `Slot::mode`**

In `impl Slot`:

```rust
    /// Which drive mode reads this slot, or `None` for the five both modes share.
    ///
    /// The four mode-specific slots are the reason this change exists: the slot name is the
    /// mode, so nothing has to carry a mode alongside a slot to know which is which.
    pub fn mode(self) -> Option<Mode> {
        match self {
            Slot::Walk | Slot::GroundPick => Some(Mode::Walk),
            Slot::Roller | Slot::Crouch => Some(Mode::Roller),
            Slot::Stand
            | Slot::SitStand
            | Slot::KickLeft
            | Slot::KickRight
            | Slot::Roulade => None,
        }
    }
```

- [ ] **Step 4: Add `resolved_slot_with` and `resolved_slot`**

In `impl PolicyParams`, beside `resolved` / `resolved_with`:

```rust
    /// What `slot` would load, resolved in the mode that slot belongs to rather than the mode
    /// the robot is in. `None` for a slot switched off with the `"none"` sentinel.
    ///
    /// [`ResolvedPolicy::slot`] answers "what is running in this slot", and for a slot of the
    /// mode the robot is not in the honest answer is nothing. This is the other question, and
    /// two callers need it: the boot-time check that clears a path that will not load, and the
    /// slot report. A broken `roller = …` found at boot is a degraded health line; found at the
    /// held DPad-Up that loads it, it is a robot going down, usually on a table.
    ///
    /// Takes the manifest rather than reading it, because [`set_manifest`] parses the file on
    /// every call and both callers ask about all nine slots in a loop.
    pub fn resolved_slot_with(
        &self,
        slot: Slot,
        manifest: Option<&SetManifest>,
    ) -> Option<PathBuf> {
        let mode = slot.mode().unwrap_or(self.mode);
        if mode == self.mode {
            return self
                .resolved_with(manifest)
                .slot(slot)
                .map(std::path::Path::to_path_buf);
        }
        let mut as_if = self.clone();
        as_if.mode = mode;
        as_if
            .resolved_with(manifest)
            .slot(slot)
            .map(std::path::Path::to_path_buf)
    }

    /// [`Self::resolved_slot_with`] against the installed set. Prefer the `_with` form in a
    /// loop — this one re-reads the manifest each call.
    pub fn resolved_slot(&self, slot: Slot) -> Option<PathBuf> {
        self.resolved_slot_with(slot, set_manifest().as_ref())
    }
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo fmt --all && cargo test -p robotd-params`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add robotd-params/src/lib.rs
git commit -m "$(cat <<'EOF'
`resolved_slot`: what a slot would load, in the mode it belongs to

`ResolvedPolicy::slot` answers what is running, and for the mode the robot is
not in that is nothing. Two callers need the other question, and asking it
separately in each is how they come to disagree.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: Validate all nine paths at boot

**Files:**
- Modify: `robotd/src/main.rs` — `drop_unloadable_overrides_with` (~line 473)
- Test: `robotd/src/main.rs`, inline `mod tests`

**Interfaces:**
- Consumes: `PolicyParams::resolved_slot_with`, `Slot::mode` from Task 3.
- Produces: nothing new; behaviour change only.

- [ ] **Step 1: Write the failing test**

```rust
/// **A path that will not load is found at boot, not at the DPad-Up that loads it.** The
/// roller's slots are not driving while the robot is on its legs, and skipping them would move
/// the discovery of a typo to the one moment it takes something down. Two extra ONNX loads at
/// startup, and only on a robot that has overridden both modes.
#[test]
fn a_broken_roller_path_is_dropped_while_walking() {
    let mut policy = params::PolicyParams {
        mode: params::Mode::Walk,
        roller: Some(PathBuf::from("/wheels/gone.onnx")),
        ..Default::default()
    };
    let mut errors = SlotErrors::default();

    drop_unloadable_overrides_with(&mut policy, &mut errors, |path| {
        Err(shape_error(&path.display().to_string()))
    });

    assert!(
        policy.slot(Slot::Roller).is_none(),
        "the override is dropped, so the roller comes back on its own default"
    );
    assert!(
        errors.get(Slot::Roller).is_some(),
        "and health says why, rather than the robot going down on the next mode switch"
    );
}

/// The mirror: the legs' slots are checked while the robot is on wheels.
#[test]
fn a_broken_walk_path_is_dropped_while_rolling() {
    let mut policy = params::PolicyParams {
        mode: params::Mode::Roller,
        walk: Some(PathBuf::from("/legs/gone.onnx")),
        ..Default::default()
    };
    let mut errors = SlotErrors::default();

    drop_unloadable_overrides_with(&mut policy, &mut errors, |path| {
        Err(shape_error(&path.display().to_string()))
    });

    assert!(policy.slot(Slot::Walk).is_none());
    assert!(errors.get(Slot::Walk).is_some());
}

/// `roller = "none"` is the same refusal `walk = "none"` gets, for the same reason: a robot
/// with no locomotion network on wheels has nothing to run.
#[test]
fn the_roller_slot_cannot_be_switched_off() {
    let mut policy = params::PolicyParams {
        mode: params::Mode::Roller,
        roller: Some(PathBuf::from("none")),
        ..Default::default()
    };
    let mut errors = SlotErrors::default();

    drop_unloadable_overrides_with(&mut policy, &mut errors, |_| Ok(()));

    assert!(policy.slot(Slot::Roller).is_none(), "the line is dropped");
    assert!(errors.get(Slot::Roller).is_some());
}
```

`shape_error` is the existing helper in this test module (`robotd/src/main.rs:6807`) — a `PolicyError::Shape` for a given path, which is what the neighbouring `an_unloadable_override_falls_back_to_the_default` uses. `SlotErrors` derives `Default`, and `PathBuf` and `params` are already in scope there.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p robotd a_broken_roller_path_is_dropped_while_walking`
Expected: FAIL — the override survives, because `cfg.slot(Slot::Roller)` is `None` on legs after Task 2 and the loop `continue`s past it.

- [ ] **Step 3: Extend the locomotion sentinel check to both modes**

The existing block special-cases `Slot::Walk`. Both locomotion slots need it, and each names its own key:

```rust
    // A `walk = "none"` already in the file, from a `policy load walk none` this daemon used to
    // accept. `resolved` now falls back rather than panicking, so the robot walks — but it walks
    // with something the config does not name, which is exactly what this reports. The roller's
    // locomotion slot is the same rule: a robot on wheels with no network has nothing to run.
    for slot in [Slot::Walk, Slot::Roller] {
        if policy_params
            .slot(slot)
            .as_deref()
            .is_some_and(params::is_none_sentinel)
        {
            tracing::error!(
                slot = slot.as_str(),
                "[policy] {} = \"none\" cannot be honoured; using this robot's own",
                slot.as_str()
            );
            errors.set(
                slot,
                "the locomotion policy cannot be switched off; using this robot's own".to_owned(),
            );
            policy_params.set_slot(slot, None);
        }
    }
```

- [ ] **Step 4: Resolve each slot in its own mode**

Replace the loop body's resolution. The manifest moves out of the loop — `set_manifest()` parses the file on every call, and the old code called `resolved()` once per slot:

```rust
    let manifest = params::set_manifest();
    for slot in Slot::ALL {
        if policy_params.slot(slot).is_none() {
            continue;
        }
        // Resolved in the slot's own mode, not the robot's: a path that will not load is worth
        // finding at boot even when it belongs to the mode nobody is in. The alternative is
        // finding it at the held DPad-Up that loads it.
        // A slot set to the literal "none" resolves to nothing, and disabling a capability on
        // purpose is not a fault to fall back from.
        let Some(path) = policy_params.resolved_slot_with(slot, manifest.as_ref()) else {
            continue;
        };
        let Err(e) = validate(&path) else {
            continue;
        };
        if e.path().is_none() {
            tracing::error!(error = %e, "cannot validate policies at all; leaving config alone");
            return;
        }
        tracing::error!(
            slot = slot.as_str(),
            path = %path.display(),
            error = %e,
            "override will not load; falling back to this slot's default"
        );
        errors.set(slot, e.to_string());
        policy_params.set_slot(slot, None);
    }
```

Note `validate(&path)` — `resolved_slot_with` returns an owned `PathBuf`, where `cfg.slot()` returned a borrow.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo fmt --all && cargo test -p robotd`
Expected: PASS, including the pre-existing `the_line_is_dropped` and `the_override_is_dropped` tests around `Slot::Walk`.

- [ ] **Step 6: Commit**

```bash
git add robotd/src/main.rs
git commit -m "$(cat <<'EOF'
Check every slot at boot, including the mode nobody is in

A typo in `roller = …` used to be found at the held DPad-Up that loads it,
which is the one moment it takes the robot down, usually on a table. Two extra
ONNX loads at startup, and only on a robot that has overridden both modes.

The manifest is read once rather than once per slot while we are here.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: The slot report tells the truth about nine slots

**Files:**
- Modify: `duck-ipc-proto/src/lib.rs` — `PolicySlot` (~line 2334)
- Modify: `robotd/src/main.rs` — `slot_report` (~line 427)
- Modify: `robotctl/src/main.rs` — the `policy list` renderer
- Test: `robotd/src/main.rs`, inline `mod tests`

**Interfaces:**
- Consumes: `PolicyParams::resolved_slot_with`, `Slot::mode` from Task 3.
- Produces: `proto::PolicySlot.mode: Option<String>`.

- [ ] **Step 1: Write the failing test**

```rust
/// **A configured slot never reports as empty.** Left resolving against the current mode, a
/// robot on its legs with `roller` set reported that row with no path, no origin and
/// `overridden: true` — which reads as "switched off" for a slot that is configured and will
/// load at the next mode switch. This report is how a person checks what their robot runs.
#[test]
fn the_report_names_a_file_for_every_configured_slot() {
    let policy = params::PolicyParams {
        mode: params::Mode::Walk,
        roller: Some(std::path::PathBuf::from("/wheels/v3.onnx")),
        ..Default::default()
    };
    let cfg = policy.resolved_with(None);
    let rows = slot_report(&policy, &cfg, &SlotErrors::default());

    assert_eq!(rows.len(), 9);
    let roller = rows.iter().find(|r| r.slot == "roller").expect("a row");
    assert_eq!(
        roller.path.as_deref(),
        Some("/wheels/v3.onnx"),
        "configured, and it says so"
    );
    assert!(roller.overridden);
    assert_eq!(roller.mode.as_deref(), Some("roller"));

    let walk = rows.iter().find(|r| r.slot == "walk").expect("a row");
    assert_eq!(walk.mode.as_deref(), Some("walk"));
    assert!(walk.path.is_some(), "the default, unoverridden");

    let sitstand = rows.iter().find(|r| r.slot == "sitstand").expect("a row");
    assert_eq!(sitstand.mode, None, "shared by both modes");
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p robotd the_report_names_a_file_for_every_configured_slot`
Expected: FAIL to compile — `struct 'PolicySlot' has no field named 'mode'`.

- [ ] **Step 3: Add `PolicySlot::mode`**

In `duck-ipc-proto/src/lib.rs`, in `PolicySlot`, after `slot`:

```rust
    /// Which drive mode reads this slot — `"walk"`, `"roller"`, or absent for the five both
    /// modes share.
    ///
    /// Carried so a client can render the mode-specific rows as such without hardcoding which
    /// names those are, which would render wrong the next time a slot is added. A string rather
    /// than an enum, for the same reason `origin` is one: a client that predates a mode reports
    /// it verbatim instead of failing to parse a robot's answer.
    pub mode: Option<String>,
```

The struct is `#[derive(Default)]` with `#[serde(default)]`, so this is additive on the wire and an older client's payload still deserializes.

- [ ] **Step 4: Resolve the report in each slot's own mode**

In `robotd/src/main.rs`, `slot_report`:

```rust
fn slot_report(
    policy_params: &params::PolicyParams,
    cfg: &params::ResolvedPolicy,
    errors: &SlotErrors,
) -> Vec<proto::PolicySlot> {
    let _ = cfg;
    // Once, not once per slot: `set_manifest` parses the file on every call.
    let manifest = params::set_manifest();
    Slot::ALL
        .into_iter()
        .map(|slot| {
            // In the slot's own mode, so a configured slot of the mode the robot is not in
            // names its file instead of reporting as empty.
            let path = policy_params.resolved_slot_with(slot, manifest.as_ref());
            proto::PolicySlot {
                slot: slot.as_str().to_owned(),
                mode: slot.mode().map(|m| m.as_str().to_owned()),
                path: path.as_ref().map(|p| p.display().to_string()),
                origin: path.as_ref().map(|p| origin_of(p).to_owned()),
                overridden: policy_params.slot(slot).is_some(),
                error: errors.get(slot).map(str::to_owned),
            }
        })
        .collect()
}
```

If `cfg` ends up with no remaining use, drop the parameter and its argument at the call site rather than keeping `let _ = cfg;`. Check the call site before deciding.

- [ ] **Step 5: Mark the mode-specific rows in `robotctl policy list`**

`render_policies` (`robotctl/src/main.rs:3760`) builds the table. It already computes the `SLOT`
column width from `s.slot.len()` and writes the header `"\n   {:width$}  {:9}  POLICY"`. Suffix the
name with its mode, and widen from the suffixed string so the columns still line up:

```rust
    let slot_name = |s: &proto::PolicySlot| match s.mode.as_deref() {
        // Named rather than inferred: the four mode-specific slots are the point of the
        // column, and a hardcoded list of which names those are would go stale the next time
        // a slot is added.
        Some(mode) => format!("{} ({mode})", s.slot),
        None => s.slot.clone(),
    };
    let width = policies
        .slots
        .iter()
        .map(|s| slot_name(s).len())
        .max()
        .unwrap_or(4)
        .max(4);
```

Then use `slot_name(slot)` in place of `slot.slot` in the row-writing loop below the header.

Leave `--json` alone: it serializes `PolicySlot` and gains the field for free.

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cargo fmt --all && cargo test -p duck-ipc-proto && cargo test -p robotd && cargo test -p robotctl`
Expected: PASS.

- [ ] **Step 7: Run the whole workspace**

Run: `cargo test --workspace`
Expected: PASS, everything. This is the first point at which the change is complete, so a failure here is a real one — do not move past it.

- [ ] **Step 8: Commit**

```bash
git add duck-ipc-proto/src/lib.rs robotd/src/main.rs robotctl/src/main.rs
git commit -m "$(cat <<'EOF'
The slot report names a file for every configured slot

Resolved against the current mode, a robot on its legs with `roller` set
reported that row with no path and `overridden: true` — which reads as
"switched off" for a slot that is configured and loads at the next mode
switch. This report is how a person checks what their robot is running.

`PolicySlot.mode` so a client can mark the mode-specific rows without
hardcoding which names those are.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

## Verification

After Task 5, confirm the whole thing end to end rather than trusting the task-level runs:

- [ ] `cargo fmt --all --check` — clean
- [ ] `cargo test --workspace` — green
- [ ] `cargo clippy --workspace --all-targets` with `RUSTFLAGS="-D warnings"` — clean
- [ ] `cargo test -p robotd-params the_registry_covers_every_key_exactly every_slot_is_a_registry_key` — the two tests that pin the registry complete against serde's own field list
- [ ] Read back `docs/design/per-mode-policy-slots-design.md` §§2-6 against the code and fix the doc if the implementation diverged — the spec is the thing the next person reads

Not verifiable on a dev machine, and worth saying in the PR: none of this has run on a board. The behaviour that matters — a held DPad-Up bringing the robot up on `roller.onnx` instead of a gait somebody loaded for walking — needs a duck.
