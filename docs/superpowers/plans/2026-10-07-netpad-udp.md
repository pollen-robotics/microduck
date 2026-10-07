# netpadd — gamepad over UDP — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A new daemon, `netpadd`, receives gamepad state over UDP from a program on the LAN and
drives the robot with exactly `padd`'s mapping, at the lowest latency the link allows, with no
buffering.

**Architecture:**
- `padd`'s mapping moves into a pure library crate, `pad-map`: pad state goes in, `proto::Call`s
  come out. `padd` (gilrs) and `netpadd` (UDP) both feed it.
- `netpadd` is a single tokio task on a `current_thread` runtime, around a pure receive state
  machine. It never queues: it keeps the newest state only, finds button edges per datagram, and
  caps sends at `max_hz`.
- `[netpad] enabled` in `robotd.toml` decides which of the two daemons runs, through
  `ExecCondition=` in both units.

**Tech Stack:** Rust 2024 workspace, tokio, serde/toml (`robotd-params`), gilrs (`padd`, `duckctl`),
systemd units.

**Spec:** `docs/superpowers/specs/2026-10-07-netpad-udp-design.md`. Read it first; this plan argues
from it.

## Global Constraints

- **Branch** `udp-pad`, from `main`. Commit at the end of every task; do not push.
- **Toolchain:** `cargo +1.94.1 check --workspace` is the check. Also run `cargo fmt --all` and
  `RUSTFLAGS="-D warnings" cargo clippy -p <crate> --all-targets` on each crate touched.
- **Wire:** 28 bytes little-endian: magic `b"DKPD"` · version `1` · flags u8 · seq u32 · t_ms u32 ·
  i16 ×6 (`lx ly rx ry lt rt`) · buttons u16. Button bits: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Start,
  7 Select, 8 Up, 9 Down, 10 Left, 11 Right.
- **`[netpad]` keys:**

  | key | default | rule |
  |---|---|---|
  | `enabled` | `false` | |
  | `port` | `4210` | |
  | `max_hz` | `30` | 10–100, refused outside |
  | `timeout_ms` | `250` | must be `< [safety] deadman_ms` |

- **No version-difference workarounds.** A skew is logged and served. Only a malformed or
  unknown-version packet is dropped (CLAUDE.md).
- **Comments and docs** match the surrounding code's density and voice: say *why*, in full
  sentences. Docs own mechanisms one page each (`docs/README.md`).
- **`padd`'s behaviour must not change.** Task 2 is a pure refactor.

## Review Focus

1. **A client restarts within `timeout_ms` with a lower `seq`.** It starts from a random `seq`, so
   its packets look stale until the timeout passes. Expected: driving resumes within `timeout_ms`,
   and it is never frozen for good. *(Task 4: `a_restarted_client_is_taken_after_the_timeout`.)*
2. **A button already held when a client (re)appears.** Expected: no skill fires and no D-pad mode
   switch happens until it is pressed again. *(Task 4: `the_first_datagram_makes_no_edges`.)*
3. **Start or Select held across a dropout.** Expected: the hold does not resume at full length, so
   a power-off nobody asked for never fires. *(Task 4: `going_away_reports_gone_once`, plus Task 2's
   moved `a_hold_does_not_survive_the_pad_going_away`.)*
4. **A second sender on the LAN while the first is driving.** Expected: it is ignored until the
   first one goes quiet. *(Task 4: `another_peer_is_ignored_while_the_first_is_alive`.)*
5. **Extreme axis values (`i16::MIN`) and garbage datagrams.** Expected: values are clamped to ±1,
   garbage is dropped, and nothing panics. *(Task 3: `i16_min_clamps_to_minus_one` and
   `garbage_is_refused_not_panicked_on`.)*

---

## File structure

| path | responsibility |
|---|---|
| `robotd-params/src/lib.rs` | `NetpadParams`, its validation, its place in `Params` |
| `robotd-params/src/registry.rs` | `netpad.*` editor entries |
| `deploy/robotd.toml` | the commented `[netpad]` section |
| `pad-map/src/lib.rs` | crate doc, re-exports, `udp_selected` |
| `pad-map/src/buttons.rs` | `Buttons` bitset (bit order = wire order) |
| `pad-map/src/hold.rs` | `HoldButton`, `HoldAction` (moved from `padd`) |
| `pad-map/src/continuous.rs` | `Continuous`, `HEARTBEAT`; pure, so it returns bytes and does no I/O |
| `pad-map/src/mapper.rs` | `PadFrame`, `Config`, `Out`, `Mapper`, `Mode`, `DriveLimits`, `head_from_pad`, `mode_exit_calls`, `report` |
| `pad-map/src/bindings.rs` | `read_bindings` (moved from `padd`) |
| `pad-map/src/wire.rs` | `Packet` encode/decode |
| `padd/src/main.rs` | gilrs → `PadFrame` per tick, sync I/O, `--should-run` |
| `netpadd/src/receiver.rs` | the pure receive state machine |
| `netpadd/src/robotd.rs` | the async `robotd` client, every call timeout-bounded |
| `netpadd/src/main.rs` | args, `--should-run`, the tokio loop |
| `netpadd/systemd/netpadd.service`, `netpadd/systemd/sysusers.d/netpadd.conf` | the unit and its user |
| `duckctl/src/udp_pad.rs` | the reference client |
| `docs/robot/udp-pad.md` | the owning doc page |

---

### Task 1: `[netpad]` in `robotd-params`

**Files:**
- Modify: `robotd-params/src/lib.rs`
  - the `Params` struct (~l.66–109)
  - the `ParamsError` enum (~l.2216)
  - `validate` (~l.2300)
  - the tests module
- Modify: `robotd-params/src/registry.rs`: add entries after `[pad_drive]` (~l.534)
- Modify: `deploy/robotd.toml`: add a section after `[pad_drive]`

**Interfaces:**
- Produces: `robotd_params::NetpadParams { enabled: bool, port: u16, max_hz: u32, timeout_ms: u64 }`
  with `Default`, reached as `Params::netpad`, and `ParamsError::Netpad { path, reason: String }`.

- [ ] **Step 1: Write the failing tests** at the end of `robotd-params/src/lib.rs`'s `mod tests`,
  next to `a_positive_pad_drive_min_is_refused`. They reuse that test's `write` helper.

```rust
    /// The cap is a divisor and a promise about latency: zero divides by it, and a rate above
    /// what a pad link carries only spends the robot's socket.
    #[test]
    fn netpad_max_hz_outside_ten_to_a_hundred_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        for bad in ["0", "101"] {
            let path = write(dir.path(), &format!("[netpad]\nmax_hz = {bad}\n"));
            let error = Params::load(&path, true).expect_err("refused").to_string();
            assert!(error.contains("netpad.max_hz"), "{error}");
        }
        let path = write(dir.path(), "[netpad]\nmax_hz = 100\n");
        assert_eq!(Params::load(&path, true).expect("valid").netpad.max_hz, 100);
    }

    /// A client timeout at or past the deadman would keep re-sending a dead client's last
    /// stick until robotd stops the robot anyway — the timeout would protect nothing.
    #[test]
    fn netpad_timeout_must_be_below_the_deadman() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[netpad]\ntimeout_ms = 500\n");
        let error = Params::load(&path, true).expect_err("refused").to_string();
        assert!(error.contains("netpad.timeout_ms"), "{error}");
        assert!(error.contains("safety.deadman_ms"), "{error}");

        let path = write(dir.path(), "[netpad]\ntimeout_ms = 0\n");
        assert!(Params::load(&path, true).is_err(), "zero is always timed out");

        let path = write(
            dir.path(),
            "[netpad]\ntimeout_ms = 500\n[safety]\ndeadman_ms = 800\n",
        );
        assert_eq!(Params::load(&path, true).expect("valid").netpad.timeout_ms, 500);
    }

    #[test]
    fn netpad_defaults_are_off_on_4210_at_30_hz() {
        let n = NetpadParams::default();
        assert_eq!((n.enabled, n.port, n.max_hz, n.timeout_ms), (false, 4210, 30, 250));
    }
```

- [ ] **Step 2: Run the tests to see them fail**

Run: `cargo test -p robotd-params netpad`
Expected: a compile error, `cannot find type NetpadParams` / `no field netpad`.

- [ ] **Step 3: Implement it**

  Add this to `Params`, after `pad_drive`:

```rust
    /// The pad over UDP: `netpadd` reads this, and so does `padd`, to know whether to stand down.
    pub netpad: NetpadParams,
```

  Add this after `PadDriveParams`' impl block:

```rust
/// A gamepad over UDP, from a program on the LAN — `netpadd`. `docs/robot/udp-pad.md` owns it.
///
/// `enabled` is the switch between the two pad daemons, not a feature flag on one: both units ask
/// it in `ExecCondition=`, so exactly one of `padd` and `netpadd` runs, and an update's restart of
/// every shipped unit reaches the same answer as a boot does.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct NetpadParams {
    /// UDP is the pad source; `padd` stands down.
    pub enabled: bool,
    /// The UDP port, bound on IPv4 0.0.0.0.
    pub port: u16,
    /// At most this many intent frames a second reach robotd. A change is sent at once when the
    /// last send is at least one interval old, otherwise at the next slot — never queued.
    pub max_hz: u32,
    /// Silence after which the client counts as gone, and nothing more is sent.
    pub timeout_ms: u64,
}

impl Default for NetpadParams {
    fn default() -> Self {
        Self {
            enabled: false,
            port: 4210,
            max_hz: 30,
            timeout_ms: 250,
        }
    }
}
```

  Add this variant to `ParamsError`:

```rust
    #[error("{path}: {reason}")]
    Netpad { path: String, reason: String },
```

  Add this in `validate`, before the pickup check:

```rust
        // Refused rather than clamped, like `control.hz`: the editor should say which value it
        // was not going to guess at.
        if !(10..=100).contains(&self.netpad.max_hz) {
            return Err(ParamsError::Netpad {
                path: path.display().to_string(),
                reason: format!("netpad.max_hz ({}) must be 10–100", self.netpad.max_hz),
            });
        }
        if self.netpad.timeout_ms == 0 || self.netpad.timeout_ms >= self.safety.deadman_ms {
            return Err(ParamsError::Netpad {
                path: path.display().to_string(),
                reason: format!(
                    "netpad.timeout_ms ({}) must be above 0 and below safety.deadman_ms ({}) — \
                     past the deadman it would re-send a dead client's last stick",
                    self.netpad.timeout_ms, self.safety.deadman_ms
                ),
            });
        }
```

  Add this to `registry.rs`, after the last `pad_drive` entry:

```rust
    // ── [netpad] ─────────────────────────────────────────────────────────────
    //
    // The pad over UDP. `enabled` picks between `padd` and `netpadd`; restart both after.
    entry(
        "netpad.enabled",
        Kind::Bool,
        "Drive from UDP (netpadd) instead of the Bluetooth pad (padd) — restart both after",
    ),
    entry("netpad.port", Kind::Integer, "UDP port netpadd listens on"),
    entry(
        "netpad.max_hz",
        Kind::Integer,
        "Most intent frames a second sent to robotd, 1–100",
    ),
    entry(
        "netpad.timeout_ms",
        Kind::Integer,
        "Silence after which the UDP client counts as gone — below safety.deadman_ms",
    ),
```

  Add this to `deploy/robotd.toml`, after the `[pad_drive]` block:

```toml

# ── Pad over UDP ────────────────────────────────────────────────────────────────────────────
#
# Read by `padd` and `netpadd`. On, `netpadd` takes gamepad state from a program on the LAN —
# `duckctl udp-pad`, or your own; docs/robot/udp-pad.md has the wire format — and `padd` stands
# down. Restart both after changing it: `sudo systemctl restart padd netpadd`.
# [netpad]
# enabled = false
# port = 4210
# max_hz = 30          # intent frames a second to robotd, at most
# timeout_ms = 250     # silence that counts as the client gone; below [safety] deadman_ms
```

- [ ] **Step 4: Run the whole crate's tests**

Run: `cargo test -p robotd-params`
Expected: everything passes, including `the_registry_covers_every_key_exactly`,
`the_shipped_example_matches_the_defaults` and `the_shipped_example_sets_nothing_away_from_default`.
If the shipped-example test cannot read commented keys, compare with how `[pad_drive]` is written
and match it exactly.

- [ ] **Step 5: Commit**

```bash
git add robotd-params deploy/robotd.toml
git commit -m "robotd-params: [netpad], the pad over UDP"
```

---

### Task 2: Move `padd`'s mapping into `pad-map`, behaviour unchanged

This is a refactor. Line numbers below refer to `padd/src/main.rs` at `main` (`ea4db6d`); use
`git show main:padd/src/main.rs` to read them. "Move verbatim" means cut and paste, then change only
visibility (`pub`/`pub(crate)`) and paths.

**Files:**
- Create: `pad-map/Cargo.toml`
- Create: `pad-map/src/lib.rs`, `buttons.rs`, `hold.rs`, `continuous.rs`, `mapper.rs`, `bindings.rs`
- Modify: `Cargo.toml` (workspace `members` and `default-members`: add `"pad-map"`, alphabetically
  after `"pad-imu"`)
- Modify: `padd/Cargo.toml` (add `pad-map = { path = "../pad-map" }`)
- Modify: `padd/src/main.rs`

**Interfaces:**
- Produces, all at `pad_map::`:

```rust
pub struct Buttons(pub u16);            // A..RIGHT consts, NONE, KNOWN = 0x0FFF
pub struct PadFrame {
    pub left_x: f64, pub left_y: f64, pub right_x: f64, pub right_y: f64, // raw -1..1, up +
    pub lt: f64, pub rt: f64,                                               // 0..1
    pub held: Buttons, pub pressed: Buttons, pub released: Buttons,
    pub attitude: Option<[f32; 4]>, pub has_imu: bool,
}
pub struct Config {
    pub bindings: robotd_params::PadParams,
    pub imu_head: robotd_params::PadImuHeadControlParams,
    pub drive: robotd_params::PadDriveParams,
    pub deadzone: f64, pub max_head: f64, pub roller: bool,
}
pub enum Out { Notify(proto::Call), Request(proto::Call) }
pub struct Mapper;   // ::new(), .tick(&PadFrame, &Config, Instant, &mut Vec<Out>, &mut Vec<proto::Call>), .pad_gone()
pub struct Continuous; // ::default(), .next(&[proto::Call], Instant) -> Option<&[u8]>
pub const HEARTBEAT: Duration;          // 100 ms
pub fn report(call: &proto::Call, response: Option<&proto::Response>);
pub fn read_bindings(path: &Path) -> (PadParams, PadImuHeadControlParams, PadDriveParams);
pub fn udp_selected(config: &Path) -> bool;
```

- [ ] **Step 1: Create the crate skeleton**

  `pad-map/Cargo.toml`:

```toml
[package]
name = "pad-map"
version.workspace = true
edition.workspace = true
license.workspace = true
description = "What a gamepad means to a duck: pad state in, intents out — shared by padd and netpadd"

[dependencies]
duck-ipc-proto = { path = "../duck-ipc-proto" }
robotd-params = { path = "../robotd-params" }
pad-imu = { path = "../pad-imu" }
serde_json.workspace = true
tracing.workspace = true

[dev-dependencies]
tempfile = "3.27.0"
```

  These are the same `.workspace = true` fields `padd/Cargo.toml` uses.

  `pad-map/src/lib.rs`:

```rust
//! What a gamepad means to a duck: pad state in, intents out.
//!
//! Moved out of `padd` so that `netpadd` — the same pad, arriving over UDP — drives the robot
//! with exactly the same mapping rather than a copy of it that drifts. It is pure: no socket, no
//! clock of its own, no config file read behind the caller's back. The two daemons own the I/O;
//! this owns what a button means. `padd`'s crate doc still describes the mapping itself.

mod bindings;
mod buttons;
mod continuous;
mod hold;
mod mapper;
pub mod wire;

pub use bindings::read_bindings;
pub use buttons::Buttons;
pub use continuous::{Continuous, HEARTBEAT};
pub use hold::{HoldAction, HoldButton};
pub use mapper::{Config, Mapper, Out, PadFrame, report};

/// Whether `[netpad] enabled` hands the pad to `netpadd`. The one question both units'
/// `ExecCondition=` ask.
///
/// A file that cannot be read answers **no**: a bad config must never leave somebody without the
/// Bluetooth pad, which is the rule `padd` already lives by.
pub fn udp_selected(config: &std::path::Path) -> bool {
    match robotd_params::Params::load(config, false) {
        Ok(params) => params.netpad.enabled,
        Err(e) => {
            tracing::warn!(error = %e, "cannot read the config; the Bluetooth pad keeps the robot");
            false
        }
    }
}
```

  Create `pad-map/src/wire.rs` with only `//! The pad on the wire — Task 3.` for now.

- [ ] **Step 2: `Buttons`.** Create `pad-map/src/buttons.rs`:

```rust
//! The pad's buttons as a bitset. The bit order **is** the wire's (`wire.rs`), so a datagram's
//! `buttons` field is a `Buttons` without translation.

use std::ops::{BitOr, BitOrAssign};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Buttons(pub u16);

impl Buttons {
    pub const NONE: Self = Self(0);
    pub const A: Self = Self(1 << 0);
    pub const B: Self = Self(1 << 1);
    pub const X: Self = Self(1 << 2);
    pub const Y: Self = Self(1 << 3);
    pub const LB: Self = Self(1 << 4);
    pub const RB: Self = Self(1 << 5);
    pub const START: Self = Self(1 << 6);
    pub const SELECT: Self = Self(1 << 7);
    pub const UP: Self = Self(1 << 8);
    pub const DOWN: Self = Self(1 << 9);
    pub const LEFT: Self = Self(1 << 10);
    pub const RIGHT: Self = Self(1 << 11);
    /// Every bit that means something. The rest are reserved.
    pub const KNOWN: u16 = 0x0FFF;

    /// The six bindable buttons, by their `[pad]` config name, in `[pad]`'s order.
    pub const BINDABLE: [(Self, &'static str); 6] = [
        (Self::A, "a"),
        (Self::B, "b"),
        (Self::X, "x"),
        (Self::Y, "y"),
        (Self::LB, "lb"),
        (Self::RB, "rb"),
    ];

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0 && other.0 != 0
    }

    /// Bits set in `self` that were not set in `before` — the presses between two states.
    pub fn newly_set(self, before: Self) -> Self {
        Self(self.0 & !before.0)
    }
}

impl BitOr for Buttons {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for Buttons {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edges_are_the_difference_in_each_direction() {
        let before = Buttons::A | Buttons::START;
        let now = Buttons::A | Buttons::B;
        assert_eq!(now.newly_set(before), Buttons::B, "pressed");
        assert_eq!(before.newly_set(now), Buttons::START, "released");
        assert!(!Buttons::NONE.contains(Buttons::NONE), "nothing is not a button");
    }
}
```

- [ ] **Step 3: `HoldButton`.** Create `pad-map/src/hold.rs`. Move these from `padd/src/main.rs`
  verbatim:
  - `HoldButton` and `HoldAction`, with their docs (l.267–354)
  - the constants `HOME_HOLD`, `REST_HOLD` and `SHUTDOWN_HOLD` (l.248–264)

  Then make the types, `tick`, `reset` and the constants `pub`. Add
  `use std::time::{Duration, Instant};`.

- [ ] **Step 4: `Continuous`, made pure.** Create `pad-map/src/continuous.rs`:
  - Move `HEARTBEAT` and its doc (l.1037–1052) verbatim, and make it `pub`.
  - Move the `Continuous` struct and `may_hold` verbatim.
  - Replace `send` with the version below. `Continuous` no longer does I/O; the caller writes.

```rust
impl Continuous {
    /// This tick's intents as the bytes to put on robotd's socket, or `None` when they are the
    /// ones already there and may be held back (see [`Continuous::may_hold`]).
    ///
    /// Returning `Some` records the frame as sent. The caller writes it or dies trying: both
    /// daemons exit on a failed write, so there is no "returned but not sent" state to undo.
    /// One buffer for the whole frame: the calls describe a single instant, and a peer that read
    /// half of one would act on a head pose without the velocity that came with it.
    pub fn next(&mut self, calls: &[proto::Call], now: Instant) -> Option<&[u8]> {
        self.line.clear();
        for call in calls {
            // Encoding plain data into a Vec cannot fail.
            serde_json::to_writer(&mut self.line, &proto::Request::notify(call))
                .expect("a Call serialises");
            self.line.push(b'\n');
        }
        if self.line == self.last && self.may_hold(calls, now) {
            return None;
        }
        // Swapped rather than cloned: the displaced buffer is next tick's scratch.
        std::mem::swap(&mut self.last, &mut self.line);
        self.at = Some(now);
        Some(&self.last)
    }
}
```

  Move the six `Continuous` tests from `padd`'s `mod tests` here:
  - `an_unchanged_stationary_frame_is_not_resent`
  - `a_frame_that_asks_for_motion_is_always_sent`
  - `a_stationary_frame_goes_out_again_on_the_heartbeat`
  - `a_changed_frame_is_sent_at_once`
  - `a_two_call_frame_is_one_write_and_two_lines`
  - `an_untouched_pad_in_head_mode_holds_both_intents`

  Rewrite each the same way: `continuous.send(&mut ours, &calls, t).expect(..)` followed by
  `drain(&mut theirs)` becomes a single `continuous.next(&calls, t)`. `None` stands for "nothing was
  sent", and `Some(bytes)` is asserted on as `String::from_utf8_lossy(bytes)`. For example:

```rust
    #[test]
    fn an_unchanged_stationary_frame_is_not_resent() {
        let mut continuous = Continuous::default();
        let at = Instant::now();
        let first = continuous.next(&still(), at).map(|b| String::from_utf8_lossy(b).into_owned());
        assert!(first.expect("the first frame must go out").contains("robot.move"));
        for tick in 1..5 {
            let now = at + Duration::from_millis(20 * tick);
            assert!(continuous.next(&still(), now).is_none(), "an idle pad must say nothing");
        }
    }
```

  `a_two_call_frame_is_one_write_and_two_lines` becomes: the returned bytes hold exactly two `\n`,
  so one buffer means one write.

- [ ] **Step 5: `bindings.rs`.** Move `read_bindings` (l.~357–395) verbatim, make it `pub`, and fix
  its `use`s. Move `BINDINGS_POLL` with it and make that `pub` too: both daemons poll at the same
  rate.

- [ ] **Step 6: `mapper.rs`.** Move these verbatim, keeping their docs:
  - `Mode`, `poses_head` and `mode_exit_calls`
  - the `BODY_MAX_*` and `ROLLER_*` constants
  - `head_from_pad` (l.~380–404)
  - `DriveLimits` and its `walk` (l.983–1019)

  Then add the types and the tick below. The tick body is padd's l.642–970 with every I/O call
  turned into a push onto `out`. It must emit **in the same order** padd sent.

```rust
use std::time::Instant;

use duck_ipc_proto as proto;

use crate::buttons::Buttons;
use crate::hold::{HOME_HOLD, HoldAction, HoldButton, REST_HOLD, SHUTDOWN_HOLD};

/// One instant of a pad, from whichever daemon read it. Sticks are raw — the deadzone is
/// applied here, so both sources get the same one — and `pressed`/`released` are the edges since
/// the previous frame, which the source is responsible for not losing.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PadFrame {
    pub left_x: f64,
    pub left_y: f64,
    pub right_x: f64,
    pub right_y: f64,
    pub lt: f64,
    pub rt: f64,
    pub held: Buttons,
    pub pressed: Buttons,
    pub released: Buttons,
    /// The pad's IMU attitude, when `[pad_imu_head_control]` is on and the pad has said
    /// something believable. Always `None` from `netpadd`.
    pub attitude: Option<[f32; 4]>,
    /// Whether the pad has an IMU at all, for the "no attitude yet" warning.
    pub has_imu: bool,
}

/// Everything the mapping reads that is not the pad.
#[derive(Debug, Clone)]
pub struct Config {
    pub bindings: robotd_params::PadParams,
    pub imu_head: robotd_params::PadImuHeadControlParams,
    pub drive: robotd_params::PadDriveParams,
    pub deadzone: f64,
    pub max_head: f64,
    pub roller: bool,
}

/// What to send, in order. A `Request` is answered — "refused, and here is why" is a real
/// outcome — and the daemon hands the answer to [`report`]; a `Notify` is not.
#[derive(Debug, Clone, PartialEq)]
pub enum Out {
    Notify(proto::Call),
    Request(proto::Call),
}

/// The mapping's memory between frames.
#[derive(Debug)]
pub struct Mapper {
    mode: Mode,
    imu_reference: Option<[f32; 4]>,
    start: HoldButton,
    select: HoldButton,
    prev_rt: f64,
    prev_lt: f64,
    up: bool,
}

impl Default for Mapper {
    fn default() -> Self {
        Self::new()
    }
}

impl Mapper {
    pub fn new() -> Self {
        Self {
            mode: Mode::Drive,
            imu_reference: None,
            start: HoldButton::default(),
            select: HoldButton::default(),
            prev_rt: 0.0,
            prev_lt: 0.0,
            up: false,
        }
    }

    /// The pad went away. A hold in flight was measured against it — see [`HoldButton::reset`] —
    /// and an IMU reference means nothing to the next pad.
    pub fn pad_gone(&mut self) {
        self.start.reset();
        self.select.reset();
        self.imu_reference = None;
    }

    /// One frame: the discrete intents into `out`, in the order they must be sent, and the
    /// continuous ones into `frame` for [`crate::Continuous`]. Both are cleared first.
    pub fn tick(
        &mut self,
        pad: &PadFrame,
        cfg: &Config,
        now: Instant,
        out: &mut Vec<Out>,
        frame: &mut Vec<proto::Call>,
    ) {
        out.clear();
        frame.clear();
        // ... body: see the step text below ...
    }
}
```

  Fill in the body of `tick` by porting padd l.627–970 in order, with these substitutions:
  - `let attitude = …` (l.630–634) becomes `let attitude = pad.attitude;`. padd now computes it
    before building the frame.
  - The "IMU head control off" check (l.635–640) stays as it is.
  - `wanted_mode` comes from `pad.pressed`. Check `Buttons::UP`, `RIGHT`, `LEFT` and `DOWN` in that
    order; the last one pressed wins, as gilrs's event order made it in padd:

```rust
        let mut wanted_mode = None;
        for (button, mode) in [
            (Buttons::UP, Mode::Head),
            (Buttons::RIGHT, Mode::HeadDrive),
            (Buttons::LEFT, Mode::Drive),
            (Buttons::DOWN, Mode::BodyPose),
        ] {
            if pad.pressed.contains(button) {
                wanted_mode = Some(mode);
            }
        }
```

  - The skill list `pressed` comes from
    `Buttons::BINDABLE.iter().filter(|(b, _)| pad.pressed.contains(*b)).map(|(_, n)| *n)`.
  - `start.tick(pad.is_pressed(Button::Start), start_released, tick, &[HOME_HOLD])` becomes
    `self.start.tick(pad.held.contains(Buttons::START), pad.released.contains(Buttons::START), now, &[HOME_HOLD])`.
    Select gets the same treatment with `Buttons::SELECT`.
  - `request(&mut stream, &mut next_id, &call)` becomes `out.push(Out::Request(call))`.
    `notify(&mut stream, &call)` becomes `out.push(Out::Notify(call))`.
  - The `Tap if !up` arm sets `self.up = true` right after pushing `RobotInit`. padd set it only on
    `Ok`, and on `Err` it exited, so the state seen by the next tick is the same.
  - The enable-toggle answer log (l.679–687) moves to `report`, below.
  - `pad.is_pressed(Button::West)` becomes `pad.held.contains(Buttons::X)`.
  - `deadzone(pad.value(Axis::LeftStickX))` becomes `dz(pad.left_x)`, where
    `let dz = |v: f64| if v.abs() < cfg.deadzone { 0.0 } else { v };`.
  - `trigger(Button::RightTrigger2)` becomes `pad.rt`, and the left trigger becomes `pad.lt`.
  - `args.max_head` becomes `cfg.max_head`. `imu_head_cfg` becomes `cfg.imu_head`. `drive` becomes
    `&cfg.drive`. `roller` becomes `cfg.roller`.
  - `tap.as_ref().is_some_and(|tap| tap.has_imu())` becomes `pad.has_imu`.
  - The final `continuous.send(…)` is **not** here; the daemon does it with `frame`.

  Then add `report`, which moves the answer logging out of padd's `request` (l.1138–1153) and the
  enable outcome (l.679–687):

```rust
/// Log what the robot said to a [`Out::Request`], the way `padd` always has: a refusal or a
/// not-accepted at `warn`, and the policy toggle's outcome, because the robot owns that state and
/// its answer is the only account of it.
pub fn report(call: &proto::Call, response: Option<&proto::Response>) {
    let Some(response) = response else { return };
    if let Some(error) = &response.error {
        tracing::warn!(code = error.code, message = %error.message, "refused");
        return;
    }
    let result = response.result_as::<proto::IntentResult>().ok();
    if let Some(result) = &result
        && !result.accepted
    {
        tracing::warn!(reason = ?result.reason, "not accepted");
    }
    if let proto::Call::RobotEnable(proto::EnableParams { toggle: true, .. }) = call {
        let outcome = result.and_then(|r| r.reason).unwrap_or_else(|| "toggled".to_owned());
        tracing::warn!(%outcome, "policy");
    }
}
```

  Move these mapping tests from padd's `mod tests` into `mapper.rs`, `hold.rs` and `bindings.rs`,
  whichever holds what they test:
  - `the_pad_attitude_at_the_press_reads_as_centre`
  - `the_head_follows_the_pad_with_the_sticks_signs_gain_and_limit`
  - `select_rests_on_a_release_between_two_and_four_and_powers_off_at_four`
  - `start_taps_toggle_and_a_long_hold_goes_home_only`
  - `a_hold_does_not_survive_the_pad_going_away`
  - `a_pad_dropout_after_a_threshold_spends_the_rest_of_the_hold`
  - `leaving_a_mode_puts_back_what_it_moved`
  - `head_and_move_walks_and_turns_from_the_left_stick`

  `a_rate_this_loop_cannot_run_at_is_refused_rather_than_divided_by` tests padd's `Args` and stays
  in padd.

- [ ] **Step 7: Add one end-to-end mapper test** in `mapper.rs`. It is the contract `netpadd` relies
  on:

```rust
    fn cfg() -> Config {
        Config {
            bindings: robotd_params::PadParams::default(),
            imu_head: robotd_params::PadImuHeadControlParams::default(),
            drive: robotd_params::PadDriveParams::default(),
            deadzone: 0.1,
            max_head: 2.5,
            roller: false,
        }
    }

    /// The whole path a frame takes: a D-pad edge changes mode once, a held one does not change
    /// it again, a stick inside the deadzone walks nothing, and A's edge runs A's binding.
    #[test]
    fn a_frame_becomes_the_intents_padd_always_sent() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());

        let pad = PadFrame { left_y: 1.0, ..Default::default() };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert_eq!(frame, vec![proto::Call::RobotMove(proto::MoveParams { vx: 0.3, vy: 0.0, vyaw: 0.0 })]);

        let pad = PadFrame { left_y: 0.05, pressed: Buttons::A, held: Buttons::A, ..Default::default() };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert_eq!(frame, vec![proto::Call::RobotMove(proto::MoveParams::default())], "deadzone");
        let skill = cfg.bindings.skill("a").unwrap_or_default().to_owned();
        assert!(
            skill.is_empty()
                || out.contains(&Out::Request(proto::Call::RobotDo(proto::DoParams { skill }))),
            "{out:?}"
        );

        let pad = PadFrame { pressed: Buttons::UP, held: Buttons::UP, ..Default::default() };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert!(frame.iter().any(|c| matches!(c, proto::Call::RobotHead(_))), "head mode");
        let pad = PadFrame { held: Buttons::UP, ..Default::default() };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert!(frame.iter().any(|c| matches!(c, proto::Call::RobotHead(_))), "still head mode");
    }
```

  `Call` and `MoveParams` already derive `PartialEq`.

- [ ] **Step 8: Rewire `padd/src/main.rs` onto the lib.**
  - Delete everything that moved, and add `use pad_map::{Buttons, Config, Continuous, Mapper, Out, PadFrame};`.
  - In the loop, after the gilrs event drain (l.551–582), build edges as `Buttons` instead of
    `pressed`, `wanted_mode` and the `*_released` flags. A gilrs `Button` maps to a `Buttons` as
    follows:

```rust
/// The gilrs name for each wire bit. gilrs calls the *bumpers* `LeftTrigger`/`RightTrigger` and
/// the analogue triggers `LeftTrigger2`/`RightTrigger2` — getting that backwards binds a skill to
/// a control nobody presses.
const GILRS: [(Button, Buttons); 12] = [
    (Button::South, Buttons::A),
    (Button::East, Buttons::B),
    (Button::West, Buttons::X),
    (Button::North, Buttons::Y),
    (Button::LeftTrigger, Buttons::LB),
    (Button::RightTrigger, Buttons::RB),
    (Button::Start, Buttons::START),
    (Button::Select, Buttons::SELECT),
    (Button::DPadUp, Buttons::UP),
    (Button::DPadDown, Buttons::DOWN),
    (Button::DPadLeft, Buttons::LEFT),
    (Button::DPadRight, Buttons::RIGHT),
];

fn bit(button: Button) -> Buttons {
    GILRS.iter().find(|(b, _)| *b == button).map_or(Buttons::NONE, |(_, bit)| *bit)
}
```

  - The drain loop collects `pressed |= bit(b)` on `ButtonPressed` and `released |= bit(b)` on
    `ButtonReleased`.
  - `held` is `GILRS.iter().filter(|(b, _)| pad.is_pressed(*b)).fold(Buttons::NONE, |acc, (_, bit)| acc | *bit)`.
  - The `PadFrame` fields come from `pad.value(Axis::…)` as `f64`, from `trigger(...)`, from the
    attitude computed as at l.630–634, and from `has_imu: tap.as_ref().is_some_and(|t| t.has_imu())`.
  - On "no pad" (l.587–610), call `mapper.pad_gone()` where `start.reset(); select.reset(); imu_reference = None;` was.
  - Then send:

```rust
        mapper.tick(&frame_in, &config, tick, &mut out, &mut frame);
        for o in out.drain(..) {
            let result = match &o {
                Out::Notify(call) => notify(&mut stream, call).map(|()| None),
                Out::Request(call) => request(&mut stream, &mut next_id, call),
            };
            match result {
                Ok(response) => {
                    if let Out::Request(call) = &o {
                        pad_map::report(call, response.as_ref());
                    }
                }
                Err(e) => {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }
        if let Some(bytes) = continuous.next(&frame, tick)
            && let Err(e) = stream.write_all(bytes).and_then(|()| stream.flush())
        {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }
```

  - `request` loses its logging, which is now `report`'s. It returns `Option<proto::Response>`
    exactly as before.
  - `config` is rebuilt whenever bindings reload or `roller` changes: `Config { bindings, imu_head, drive, deadzone: args.deadzone, max_head: args.max_head, roller }`.
  - The tap calls (`watch`, `imu_control`, `idle`) stay in padd as they are.
  - The startup `tracing::warn!("driving — …")` stays in padd.

- [ ] **Step 9: Run every pad test, then the lint**

Run: `cargo test -p pad-map -p padd && RUSTFLAGS="-D warnings" cargo clippy -p pad-map -p padd --all-targets`
Expected: every moved test passes under its old name, and clippy is clean.

- [ ] **Step 10: Check behaviour by hand, if a pad is at hand.** Run padd on the laptop against the
  simulated duck:

```bash
scripts/duck-sim            # in another terminal; note the robotd socket under ~/.cache/duck-sim/
cargo run -p padd -- --socket ~/.cache/duck-sim/<name>.sock --tap-socket /tmp/padd-tap.sock
```

  Expected: Start stands the duck up, a second Start walks, the sticks drive, the D-pad changes mode,
  and Select held for 2 s rests it. If no pad is available, say so in the task report; do not claim
  it was checked.

- [ ] **Step 11: Commit**

```bash
git add Cargo.toml Cargo.lock pad-map padd
git commit -m "pad-map: padd's mapping as a library, unchanged"
```

---

### Task 3: The wire format, `pad_map::wire`

**Files:**
- Modify: `pad-map/src/wire.rs`

**Interfaces:**
- Consumes: `Buttons`, `PadFrame` (Task 2).
- Produces:

```rust
pub const MAGIC: [u8; 4] = *b"DKPD";
pub const VERSION: u8 = 1;
pub const LEN: usize = 28;
pub const DEFAULT_PORT: u16 = 4210;
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Packet { pub seq: u32, pub t_ms: u32, pub axes: [i16; 6], pub buttons: u16 }
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError { Short(usize), Magic, Version(u8) }
impl Packet {
    pub fn encode(&self) -> [u8; LEN];
    pub fn decode(bytes: &[u8]) -> Result<Packet, WireError>;
    pub fn held(&self) -> Buttons;          // known bits only
    pub fn reserved(&self) -> u16;          // the bits that are not
    pub fn frame(&self, pressed: Buttons, released: Buttons) -> PadFrame;
    pub fn from_axes(seq: u32, t_ms: u32, sticks: [f64; 4], triggers: [f64; 2], held: Buttons) -> Packet;
}
```

- [ ] **Step 1: Write the failing tests** in `wire.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Packet {
        Packet { seq: 0xDEAD_BEEF, t_ms: 42, axes: [32767, -32767, 0, 100, 32767, 0], buttons: 0b1_0100_0001 }
    }

    #[test]
    fn a_packet_round_trips_at_28_bytes() {
        let bytes = sample().encode();
        assert_eq!(bytes.len(), 28);
        assert_eq!(&bytes[..4], b"DKPD");
        assert_eq!(bytes[4], 1);
        assert_eq!(Packet::decode(&bytes), Ok(sample()));
    }

    #[test]
    fn the_layout_is_little_endian_at_the_documented_offsets() {
        let b = sample().encode();
        assert_eq!(u32::from_le_bytes(b[6..10].try_into().unwrap()), 0xDEAD_BEEF);
        assert_eq!(u32::from_le_bytes(b[10..14].try_into().unwrap()), 42);
        assert_eq!(i16::from_le_bytes([b[14], b[15]]), 32767);
        assert_eq!(u16::from_le_bytes([b[26], b[27]]), 0b1_0100_0001);
    }

    #[test]
    fn garbage_is_refused_not_panicked_on() {
        assert_eq!(Packet::decode(&[]), Err(WireError::Short(0)));
        assert_eq!(Packet::decode(&[0u8; 27]), Err(WireError::Short(27)));
        let mut b = sample().encode();
        b[0] = b'X';
        assert_eq!(Packet::decode(&b), Err(WireError::Magic));
        let mut b = sample().encode();
        b[4] = 2;
        assert_eq!(Packet::decode(&b), Err(WireError::Version(2)));
    }

    /// Fields can be appended without a version bump: a v1 reader takes the 28 it knows.
    #[test]
    fn a_longer_v1_packet_is_read_and_its_tail_ignored() {
        let mut long = sample().encode().to_vec();
        long.extend_from_slice(&[9, 9, 9, 9]);
        assert_eq!(Packet::decode(&long), Ok(sample()));
    }

    #[test]
    fn i16_min_clamps_to_minus_one() {
        let p = Packet { axes: [i16::MIN, 0, 0, 0, i16::MIN, 0], ..Default::default() };
        let f = p.frame(Buttons::NONE, Buttons::NONE);
        assert_eq!(f.left_x, -1.0);
        assert_eq!(f.lt, 0.0, "a trigger is never negative");
    }

    #[test]
    fn reserved_bits_are_split_from_buttons() {
        let p = Packet { buttons: 0xF001, ..Default::default() };
        assert_eq!(p.held(), Buttons::A);
        assert_eq!(p.reserved(), 0xF000);
    }

    #[test]
    fn from_axes_inverts_frame() {
        let p = Packet::from_axes(7, 9, [0.5, -1.0, 0.0, 1.0], [1.0, 0.25], Buttons::START);
        let f = p.frame(Buttons::NONE, Buttons::NONE);
        assert!((f.left_x - 0.5).abs() < 1e-4 && f.left_y == -1.0 && f.right_y == 1.0);
        assert!((f.lt - 1.0).abs() < 1e-4 && (f.rt - 0.25).abs() < 1e-4);
        assert_eq!(f.held, Buttons::START);
        assert!(f.attitude.is_none() && !f.has_imu);
    }
}
```

- [ ] **Step 2: Run the tests to see them fail**

Run: `cargo test -p pad-map wire`
Expected: compile errors for the missing items.

- [ ] **Step 3: Implement it**

```rust
//! The pad on the wire: one UDP datagram per state, 28 bytes, little-endian.
//! `docs/robot/udp-pad.md` owns the format; this is its only implementation.
//!
//! Every datagram is the pad's *whole* state, never a delta, so a lost one costs nothing — the
//! next replaces it — and nothing ever has to be retransmitted or reordered.

use crate::{Buttons, PadFrame};

pub const MAGIC: [u8; 4] = *b"DKPD";
pub const VERSION: u8 = 1;
pub const LEN: usize = 28;
pub const DEFAULT_PORT: u16 = 4210;

const FULL: f64 = 32767.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Packet {
    /// +1 per datagram, wrapping. The receiver drops anything not newer than what it has.
    pub seq: u32,
    /// The sender's own clock, ms. For jitter in logs only; never compared with the robot's.
    pub t_ms: u32,
    /// lx ly rx ry (±32767 is ±1, up positive), lt rt (0..32767 is 0..1).
    pub axes: [i16; 6],
    pub buttons: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    Short(usize),
    Magic,
    Version(u8),
}

impl std::fmt::Display for WireError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Short(n) => write!(f, "{n} bytes, a pad datagram is {LEN}"),
            Self::Magic => write!(f, "not a pad datagram (magic)"),
            Self::Version(v) => write!(f, "pad datagram version {v}, this build reads {VERSION}"),
        }
    }
}

impl Packet {
    pub fn encode(&self) -> [u8; LEN] {
        let mut b = [0u8; LEN];
        b[..4].copy_from_slice(&MAGIC);
        b[4] = VERSION;
        b[6..10].copy_from_slice(&self.seq.to_le_bytes());
        b[10..14].copy_from_slice(&self.t_ms.to_le_bytes());
        for (i, a) in self.axes.iter().enumerate() {
            b[14 + 2 * i..16 + 2 * i].copy_from_slice(&a.to_le_bytes());
        }
        b[26..28].copy_from_slice(&self.buttons.to_le_bytes());
        b
    }

    /// Longer than [`LEN`] is fine — appended fields this build does not know are ignored.
    pub fn decode(b: &[u8]) -> Result<Self, WireError> {
        if b.len() < LEN {
            return Err(WireError::Short(b.len()));
        }
        if b[..4] != MAGIC {
            return Err(WireError::Magic);
        }
        if b[4] != VERSION {
            return Err(WireError::Version(b[4]));
        }
        let u32_at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let mut axes = [0i16; 6];
        for (i, a) in axes.iter_mut().enumerate() {
            *a = i16::from_le_bytes([b[14 + 2 * i], b[15 + 2 * i]]);
        }
        Ok(Self { seq: u32_at(6), t_ms: u32_at(10), axes, buttons: u16::from_le_bytes([b[26], b[27]]) })
    }

    pub fn held(&self) -> Buttons {
        Buttons(self.buttons & Buttons::KNOWN)
    }

    pub fn reserved(&self) -> u16 {
        self.buttons & !Buttons::KNOWN
    }

    pub fn frame(&self, pressed: Buttons, released: Buttons) -> PadFrame {
        let stick = |v: i16| (f64::from(v) / FULL).clamp(-1.0, 1.0);
        let trigger = |v: i16| (f64::from(v) / FULL).clamp(0.0, 1.0);
        let a = self.axes;
        PadFrame {
            left_x: stick(a[0]),
            left_y: stick(a[1]),
            right_x: stick(a[2]),
            right_y: stick(a[3]),
            lt: trigger(a[4]),
            rt: trigger(a[5]),
            held: self.held(),
            pressed,
            released,
            attitude: None,
            has_imu: false,
        }
    }

    /// For a sender: sticks `[lx, ly, rx, ry]` in -1..1, triggers `[lt, rt]` in 0..1.
    pub fn from_axes(seq: u32, t_ms: u32, sticks: [f64; 4], triggers: [f64; 2], held: Buttons) -> Self {
        let q = |v: f64, lo: f64| (v.clamp(lo, 1.0) * FULL).round() as i16;
        Self {
            seq,
            t_ms,
            axes: [
                q(sticks[0], -1.0),
                q(sticks[1], -1.0),
                q(sticks[2], -1.0),
                q(sticks[3], -1.0),
                q(triggers[0], 0.0),
                q(triggers[1], 0.0),
            ],
            buttons: held.0,
        }
    }
}
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `cargo test -p pad-map wire`
Expected: 7 pass.

- [ ] **Step 5: Commit**

```bash
git add pad-map/src/wire.rs
git commit -m "pad-map: the pad on the wire, 28 bytes over UDP"
```

---

### Task 4: The receive state machine, `netpadd/src/receiver.rs`

**Files:**
- Create: `netpadd/Cargo.toml`, `netpadd/src/main.rs` (a placeholder `fn main() {}` for this task;
  Task 5 replaces it), `netpadd/src/receiver.rs`
- Modify: `Cargo.toml` (add `"netpadd"` to `members` and `default-members`, alphabetically)

**Interfaces:**
- Consumes: `pad_map::wire::{Packet, WireError}`, `pad_map::{Buttons, PadFrame, HEARTBEAT}`.
- Produces:

```rust
pub struct Timing { pub min_interval: Duration, pub timeout: Duration, pub fallback: Duration }
pub enum Accept { Connected, Taken, Stale, OtherPeer, Malformed(WireError) }
pub enum Step { Tick(PadFrame), Gone, Idle }
pub struct Receiver;
impl Receiver {
    pub fn new(timing: Timing) -> Self;
    pub fn on_datagram(&mut self, now: Instant, from: SocketAddr, bytes: &[u8]) -> Accept;
    pub fn poll(&mut self, now: Instant) -> Step;
    pub fn deadline(&self) -> Option<Instant>;
    pub fn reserved_bits_seen(&self) -> u16;
}
```

- [ ] **Step 1: Create the crate.** `netpadd/Cargo.toml`:

```toml
[package]
name = "netpadd"
version.workspace = true
edition.workspace = true
license.workspace = true
description = "A gamepad over UDP, from a program on the LAN, as an intent client"

[dependencies]
duck-ipc-proto = { path = "../duck-ipc-proto" }
pad-map = { path = "../pad-map" }
robotd-params = { path = "../robotd-params" }
clap = { workspace = true, features = ["derive"] }
serde_json.workspace = true
tokio = { workspace = true, features = ["rt", "macros", "net", "io-util", "time"] }
tracing.workspace = true
tracing-subscriber = { workspace = true, features = ["env-filter"] }

[dev-dependencies]
tokio = { workspace = true, features = ["rt", "macros", "net", "io-util", "time", "test-util"] }
tempfile = "3.27.0"
```

- [ ] **Step 2: Write the failing tests** in `receiver.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use pad_map::wire::Packet;

    const A: &str = "10.0.0.2:5000";
    const B: &str = "10.0.0.3:5000";

    fn rx() -> Receiver {
        Receiver::new(Timing {
            min_interval: Duration::from_millis(33),
            timeout: Duration::from_millis(250),
            fallback: Duration::from_millis(100),
        })
    }
    fn addr(s: &str) -> SocketAddr {
        s.parse().unwrap()
    }
    fn pkt(seq: u32, buttons: Buttons, lx: f64) -> [u8; 28] {
        Packet::from_axes(seq, 0, [lx, 0.0, 0.0, 0.0], [0.0, 0.0], buttons).encode()
    }
    fn ms(t0: Instant, n: u64) -> Instant {
        t0 + Duration::from_millis(n)
    }
    fn tick(step: Step) -> PadFrame {
        match step {
            Step::Tick(f) => f,
            other => panic!("expected a tick, got {other:?}"),
        }
    }

    #[test]
    fn a_first_datagram_is_sent_at_once() {
        let (mut r, t) = (rx(), Instant::now());
        assert_eq!(r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.5)), Accept::Connected);
        assert!((tick(r.poll(t)).left_x - 0.5).abs() < 1e-4);
    }

    /// The cap: a second change inside one interval waits for the slot, and the slot sends the
    /// newest state, not the one in between.
    #[test]
    fn changes_inside_an_interval_merge_into_the_newest_at_the_slot() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.1));
        tick(r.poll(t));
        r.on_datagram(ms(t, 5), addr(A), &pkt(2, Buttons::NONE, 0.2));
        r.on_datagram(ms(t, 10), addr(A), &pkt(3, Buttons::NONE, 0.3));
        assert_eq!(r.poll(ms(t, 10)), Step::Idle, "inside the interval");
        assert_eq!(r.deadline(), Some(ms(t, 33)));
        assert!((tick(r.poll(ms(t, 33))).left_x - 0.3).abs() < 1e-4, "the newest");
    }

    /// Once the last send is an interval old, a change goes out on arrival — no waiting for a
    /// periodic slot. This is the latency the design exists for.
    #[test]
    fn a_change_after_a_quiet_interval_is_sent_on_arrival() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        tick(r.poll(t));
        r.on_datagram(ms(t, 60), addr(A), &pkt(2, Buttons::NONE, 0.9));
        assert!((tick(r.poll(ms(t, 60))).left_x - 0.9).abs() < 1e-4);
    }

    #[test]
    fn a_late_datagram_is_dropped() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, 0.5));
        assert_eq!(r.on_datagram(t, addr(A), &pkt(4, Buttons::NONE, -0.5)), Accept::Stale);
        assert_eq!(r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, -0.5)), Accept::Stale);
        assert!((tick(r.poll(t)).left_x - 0.5).abs() < 1e-4);
    }

    #[test]
    fn seq_wraps() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(u32::MAX, Buttons::NONE, 0.0));
        assert_eq!(r.on_datagram(t, addr(A), &pkt(0, Buttons::NONE, 0.0)), Accept::Taken);
        assert_eq!(r.on_datagram(t, addr(A), &pkt(u32::MAX, Buttons::NONE, 0.0)), Accept::Stale);
    }

    /// A press and a release that both land between two sends still make a tap.
    #[test]
    fn a_press_and_release_inside_one_interval_are_both_seen() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        tick(r.poll(t));
        r.on_datagram(ms(t, 5), addr(A), &pkt(2, Buttons::START, 0.0));
        r.on_datagram(ms(t, 10), addr(A), &pkt(3, Buttons::NONE, 0.0));
        let f = tick(r.poll(ms(t, 33)));
        assert!(f.pressed.contains(Buttons::START) && f.released.contains(Buttons::START));
        assert!(!f.held.contains(Buttons::START));
        let f = tick(r.poll(ms(t, 140)));
        assert_eq!((f.pressed, f.released), (Buttons::NONE, Buttons::NONE), "edges are consumed");
    }

    #[test]
    fn the_first_datagram_makes_no_edges() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::A | Buttons::UP, 0.0));
        let f = tick(r.poll(t));
        assert_eq!(f.pressed, Buttons::NONE);
        assert!(f.held.contains(Buttons::A));
    }

    #[test]
    fn another_peer_is_ignored_while_the_first_is_alive() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.5));
        assert_eq!(r.on_datagram(ms(t, 10), addr(B), &pkt(99, Buttons::NONE, -1.0)), Accept::OtherPeer);
        assert!((tick(r.poll(ms(t, 10))).left_x - 0.5).abs() < 1e-4);
    }

    #[test]
    fn going_away_reports_gone_once_and_frees_the_lock() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::START, 0.5));
        tick(r.poll(t));
        tick(r.poll(ms(t, 100)));
        tick(r.poll(ms(t, 200)));
        assert_eq!(r.poll(ms(t, 250)), Step::Gone);
        assert_eq!(r.poll(ms(t, 300)), Step::Idle);
        assert_eq!(r.deadline(), None, "nothing to wake for");
        assert_eq!(r.on_datagram(ms(t, 400), addr(B), &pkt(1, Buttons::NONE, 0.0)), Accept::Connected);
    }

    #[test]
    fn a_restarted_client_is_taken_after_the_timeout() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1000, Buttons::NONE, 0.0));
        tick(r.poll(t));
        assert_eq!(r.on_datagram(ms(t, 50), addr(A), &pkt(3, Buttons::NONE, 0.0)), Accept::Stale);
        assert_eq!(r.poll(ms(t, 250)), Step::Gone);
        assert_eq!(r.on_datagram(ms(t, 260), addr(A), &pkt(4, Buttons::NONE, 0.0)), Accept::Connected);
    }

    /// Lost datagrams do not stop the mapping: hold timing and the heartbeat keep their clock.
    #[test]
    fn the_fallback_ticks_with_no_new_datagram() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        tick(r.poll(t));
        assert_eq!(r.poll(ms(t, 99)), Step::Idle);
        assert_eq!(r.deadline(), Some(ms(t, 100)));
        tick(r.poll(ms(t, 100)));
    }

    #[test]
    fn malformed_is_reported_and_changes_nothing() {
        let (mut r, t) = (rx(), Instant::now());
        assert!(matches!(r.on_datagram(t, addr(A), b"hello"), Accept::Malformed(_)));
        assert_eq!(r.poll(t), Step::Idle);
        assert_eq!(r.on_datagram(t, addr(B), &pkt(1, Buttons::NONE, 0.0)), Accept::Connected, "no lock taken");
    }
}
```

  `Step` needs `#[derive(Debug, PartialEq)]`; `PadFrame` already derives `PartialEq`. `Accept`
  needs `#[derive(Debug, PartialEq, Eq)]`.

- [ ] **Step 3: Run the tests to see them fail**

Run: `cargo test -p netpadd receiver`
Expected: compile errors.

- [ ] **Step 4: Implement it**

```rust
//! What arrives on the socket, turned into "send this frame now" — with no I/O and no clock of
//! its own, so every rule below is a unit test rather than a timing experiment.
//!
//! Nothing is queued. The newest accepted state replaces the last one; the edges between them are
//! accumulated per datagram so none is lost to merging; a send happens on arrival when the last
//! one is at least `min_interval` old, else at that slot. The spec's §3 is the argument.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use pad_map::wire::{Packet, WireError};
use pad_map::{Buttons, PadFrame};

#[derive(Debug, Clone, Copy)]
pub struct Timing {
    /// `1 / [netpad] max_hz`.
    pub min_interval: Duration,
    /// `[netpad] timeout_ms`.
    pub timeout: Duration,
    /// How often the mapping runs with no new datagram — `pad_map::HEARTBEAT`.
    pub fallback: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Accept {
    /// Accepted, and it is the first since the client was last gone.
    Connected,
    Taken,
    /// Not newer than what is held.
    Stale,
    /// Another sender holds the lock.
    OtherPeer,
    Malformed(WireError),
}

#[derive(Debug, PartialEq)]
pub enum Step {
    /// Run the mapping on this.
    Tick(PadFrame),
    /// The client just went quiet: send nothing from now, and reset the holds.
    Gone,
    Idle,
}

#[derive(Debug)]
pub struct Receiver {
    timing: Timing,
    /// The client, while it is alive. `None` means nobody is driving.
    peer: Option<SocketAddr>,
    latest: Packet,
    last_rx: Instant,
    last_tick: Option<Instant>,
    /// Something was accepted since the last tick.
    fresh: bool,
    pressed: Buttons,
    released: Buttons,
    reserved_seen: u16,
}

impl Receiver {
    pub fn new(timing: Timing) -> Self {
        Self {
            timing,
            peer: None,
            latest: Packet::default(),
            last_rx: Instant::now(),
            last_tick: None,
            fresh: false,
            pressed: Buttons::NONE,
            released: Buttons::NONE,
            reserved_seen: 0,
        }
    }

    pub fn on_datagram(&mut self, now: Instant, from: SocketAddr, bytes: &[u8]) -> Accept {
        let packet = match Packet::decode(bytes) {
            Ok(p) => p,
            Err(e) => return Accept::Malformed(e),
        };
        self.reserved_seen |= packet.reserved();
        let connected = match self.peer {
            Some(peer) if peer != from => return Accept::OtherPeer,
            Some(_) => {
                // Newer, in wrapping order: within half the sequence space ahead.
                if (packet.seq.wrapping_sub(self.latest.seq) as i32) <= 0 {
                    return Accept::Stale;
                }
                let (now_held, before) = (packet.held(), self.latest.held());
                self.pressed |= now_held.newly_set(before);
                self.released |= before.newly_set(now_held);
                false
            }
            None => {
                // A new client, or the old one back: what it holds was pressed before we saw it,
                // and firing it now would run a skill nobody just asked for.
                self.peer = Some(from);
                self.pressed = Buttons::NONE;
                self.released = Buttons::NONE;
                self.last_tick = None;
                true
            }
        };
        self.latest = packet;
        self.last_rx = now;
        self.fresh = true;
        if connected { Accept::Connected } else { Accept::Taken }
    }

    pub fn poll(&mut self, now: Instant) -> Step {
        if self.peer.is_none() {
            return Step::Idle;
        }
        if now.duration_since(self.last_rx) >= self.timing.timeout {
            self.peer = None;
            self.fresh = false;
            return Step::Gone;
        }
        let due = match self.last_tick {
            None => true,
            Some(at) => {
                let since = now.duration_since(at);
                (self.fresh && since >= self.timing.min_interval) || since >= self.timing.fallback
            }
        };
        if !due {
            return Step::Idle;
        }
        let frame = self.latest.frame(self.pressed, self.released);
        self.pressed = Buttons::NONE;
        self.released = Buttons::NONE;
        self.fresh = false;
        self.last_tick = Some(now);
        Step::Tick(frame)
    }

    /// When `poll` next has something to say, or `None` when nobody is driving.
    pub fn deadline(&self) -> Option<Instant> {
        self.peer?;
        let timeout = self.last_rx + self.timing.timeout;
        let next = match self.last_tick {
            None => return Some(self.last_rx),
            Some(at) if self.fresh => at + self.timing.min_interval,
            Some(at) => at + self.timing.fallback,
        };
        Some(next.min(timeout))
    }

    /// Button bits this build does not know, OR-ed over everything accepted — for one log line.
    pub fn reserved_bits_seen(&self) -> u16 {
        self.reserved_seen
    }
}
```

- [ ] **Step 5: Run the tests to see them pass**

Run: `cargo test -p netpadd receiver`
Expected: 12 pass. If `the_fallback_ticks_with_no_new_datagram` fails on the deadline, check that
`fresh` is cleared on tick.

- [ ] **Step 6: Commit**

```bash
git add Cargo.toml Cargo.lock netpadd
git commit -m "netpadd: the receive state machine — newest state only, edges per datagram"
```

---

### Task 5: The `netpadd` daemon — tokio loop, `robotd` client, unit selection

**Files:**
- Create: `netpadd/src/robotd.rs`
- Replace: `netpadd/src/main.rs`
- Modify: `padd/src/main.rs` (add the `--should-run` flag and early exit)

**Interfaces:**
- Consumes: `Receiver`, `Timing`, `Step`, `Accept` (Task 4); `Mapper`, `Config`, `Out`,
  `Continuous`, `HEARTBEAT`, `BINDINGS_POLL`, `read_bindings`, `report` and `udp_selected`
  (Task 2); `NetpadParams` (Task 1).
- Produces: the `netpadd` binary, and `netpadd --should-run` / `padd --should-run`. Each exits 0 when
  its daemon should run and 1 otherwise.

- [ ] **Step 1: Write the failing integration test** at the bottom of `netpadd/src/main.rs`, under
  `#[cfg(test)]`. A fake `robotd` and a real UDP socket on localhost exercise the whole loop:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use pad_map::wire::Packet;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    /// A fake robotd: answers every request with an accepted IntentResult, and `robot.mode`
    /// with walk; collects every method name it is sent.
    async fn fake_robotd(path: std::path::PathBuf) -> tokio::sync::mpsc::UnboundedReceiver<String> {
        let listener = tokio::net::UnixListener::bind(&path).unwrap();
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let (read, mut write) = stream.into_split();
            let mut lines = BufReader::new(read).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                let v: serde_json::Value = serde_json::from_str(&line).unwrap();
                let method = v["method"].as_str().unwrap_or_default().to_owned();
                if let Some(id) = v.get("id") {
                    let result = if method == "robot.mode" {
                        serde_json::json!({"mode": "walk"})
                    } else {
                        serde_json::json!({"accepted": true})
                    };
                    let reply = serde_json::json!({"jsonrpc": "2.0", "id": id, "result": result});
                    write.write_all(format!("{reply}\n").as_bytes()).await.unwrap();
                }
                let _ = tx.send(method);
            }
        });
        rx
    }

    #[tokio::test]
    async fn a_datagram_reaches_robotd_as_a_move_and_silence_stops_it() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("robotd.sock");
        let mut seen = fake_robotd(sock.clone()).await;

        let mut params = robotd_params::Params::default();
        params.netpad.port = 0; // any free port; `serve` reports which
        let (port_tx, port_rx) = tokio::sync::oneshot::channel();
        let args = Args::for_test(sock, dir.path().join("none.toml"));
        tokio::spawn(serve(args, params, Some(port_tx)));
        let port = port_rx.await.unwrap();

        let client = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let p = Packet::from_axes(1, 0, [0.0, 1.0, 0.0, 0.0], [0.0, 0.0], pad_map::Buttons::NONE);
        client.send_to(&p.encode(), ("127.0.0.1", port)).await.unwrap();

        let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(2);
        let mut methods = Vec::new();
        while !methods.iter().any(|m: &String| m == "robot.move") {
            let m = tokio::time::timeout_at(deadline, seen.recv()).await.expect("a move in 2 s");
            methods.push(m.unwrap());
        }
        assert_eq!(methods[0], "robot.mode", "the roller question comes first");

        // Silence: after the timeout nothing more is sent.
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
        while seen.try_recv().is_ok() {}
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        let late: Vec<_> = std::iter::from_fn(|| seen.try_recv().ok())
            .filter(|m| m != "robot.mode")
            .collect();
        assert!(late.is_empty(), "sent after the client went quiet: {late:?}");
    }
}
```

- [ ] **Step 2: Run it to see it fail**

Run: `cargo test -p netpadd a_datagram_reaches_robotd`
Expected: compile errors (`serve`, `Args::for_test` missing).

- [ ] **Step 3: Write `robotd.rs`**

```rust
//! robotd's socket, from inside the one tokio task: every write and every answer bounded by a
//! timeout, because a robotd that stops reading must not freeze the receive loop and let the
//! socket fill behind it. Any failure is the caller's cue to exit; systemd brings us back.

use std::path::Path;
use std::time::Duration;

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

/// How long an answered request may take. robotd answers intents from memory.
const ANSWER: Duration = Duration::from_secs(1);

pub struct Robotd {
    stream: BufReader<UnixStream>,
    next_id: u64,
    /// One send interval: a write that cannot land in that has missed its slot anyway.
    write_timeout: Duration,
}

fn timed_out(what: &str) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::TimedOut, format!("robotd {what} timed out"))
}

impl Robotd {
    pub async fn connect(path: &Path, write_timeout: Duration) -> std::io::Result<Self> {
        let stream = UnixStream::connect(path).await?;
        Ok(Self { stream: BufReader::new(stream), next_id: 1, write_timeout })
    }

    pub async fn write(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        tokio::time::timeout(self.write_timeout, self.stream.get_mut().write_all(bytes))
            .await
            .map_err(|_| timed_out("write"))?
    }

    pub async fn notify(&mut self, call: &proto::Call) -> std::io::Result<()> {
        let mut line = serde_json::to_vec(&proto::Request::notify(call))?;
        line.push(b'\n');
        self.write(&line).await
    }

    /// Notifications have no answers, so the only lines coming back are these, in order.
    pub async fn request(&mut self, call: &proto::Call) -> std::io::Result<Option<proto::Response>> {
        let id = proto::Id::Number(self.next_id);
        self.next_id += 1;
        let mut line = serde_json::to_vec(&proto::Request::call(id, call))?;
        line.push(b'\n');
        self.write(&line).await?;
        let mut answer = String::new();
        let n = tokio::time::timeout(ANSWER, self.stream.read_line(&mut answer))
            .await
            .map_err(|_| timed_out("answer"))??;
        if n == 0 {
            return Err(std::io::ErrorKind::UnexpectedEof.into());
        }
        match serde_json::from_str::<proto::Response>(&answer) {
            Ok(r) => Ok(Some(r)),
            Err(e) => {
                tracing::warn!(error = %e, raw = %answer.trim(), "unparsable answer");
                Ok(None)
            }
        }
    }

    /// Whether this robot is on wheels. `None` for an answer that did not say.
    pub async fn ask_roller(&mut self) -> std::io::Result<Option<bool>> {
        Ok(self
            .request(&proto::Call::RobotMode)
            .await?
            .and_then(|a| a.result_as::<proto::ModeResult>().ok())
            .map(|m| m.mode == "roller"))
    }
}
```

- [ ] **Step 4: Write `main.rs`**

```rust
//! `netpadd` — a gamepad over UDP, as an intent client.
//!
//! A program on the LAN sends the pad's whole state, 28 bytes a datagram
//! (`docs/robot/udp-pad.md` owns the format), and this drives the robot with exactly `padd`'s
//! mapping — `pad-map`, shared. It is `padd` with a different pad: no privileged access, the same
//! socket, the same deadman semantics. A client that goes quiet is a pad that went away: nothing
//! more is sent, and robotd's deadman holds the robot.
//!
//! Only one of the two runs. `[netpad] enabled` decides, and both units ask it in
//! `ExecCondition=` (`--should-run`) — not `Conflicts=`, which an update's restart of every
//! shipped unit would turn into "Bluetooth wins" (`restart-order.md` §1).
//!
//! One tokio task on a current-thread runtime owns the socket, robotd's connection and the
//! receiver: no locks, no channels, no work-stealing jitter. `receiver.rs` decides; this file
//! only waits and writes.

mod receiver;
mod robotd;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Parser;
use pad_map::{Config, Continuous, HEARTBEAT, Mapper, Out};
use receiver::{Accept, Receiver, Step, Timing};

#[derive(Parser, Debug)]
#[command(name = "netpadd", about = "Drive the robot from gamepad state sent over UDP", version)]
struct Args {
    /// `robotd`'s socket.
    #[arg(long, default_value = "/run/robotd.sock")]
    socket: PathBuf,
    /// Where `[netpad]` and the button bindings are read from.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,
    /// Exit 0 if `[netpad] enabled` says this daemon should run, 1 if not — for `ExecCondition=`.
    #[arg(long)]
    should_run: bool,
    /// Deflection below this counts as centre — `padd`'s value.
    #[arg(long, default_value_t = 0.1)]
    deadzone: f64,
    /// Full-deflection head travel, radians — `padd`'s value.
    #[arg(long, default_value_t = 2.5)]
    max_head: f64,
}

#[cfg(test)]
impl Args {
    fn for_test(socket: PathBuf, config: PathBuf) -> Self {
        Self { socket, config, should_run: false, deadzone: 0.1, max_head: 2.5 }
    }
}

/// Malformed datagrams are logged with their count at most this often: a LAN can hold a noisy
/// neighbour, and one line per packet would bury the journal.
const MALFORMED_LOG: Duration = Duration::from_secs(10);

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    if args.should_run {
        return if pad_map::udp_selected(&args.config) {
            std::process::ExitCode::SUCCESS
        } else {
            std::process::ExitCode::from(1)
        };
    }

    duck_ipc_proto::log_startup_identity!("netpadd");

    // Strict here, unlike the bindings: a `[netpad]` this daemon cannot read is one it cannot
    // honour, and running on a guessed port would be worse than saying so.
    let params = match robotd_params::Params::load(&args.config, false) {
        Ok(p) => p,
        Err(e) => {
            tracing::error!(error = %e, "cannot read the config");
            return std::process::ExitCode::FAILURE;
        }
    };
    let runtime = match tokio::runtime::Builder::new_current_thread().enable_all().build() {
        Ok(r) => r,
        Err(e) => {
            tracing::error!(error = %e, "no runtime");
            return std::process::ExitCode::FAILURE;
        }
    };
    runtime.block_on(serve(args, params, None))
}

async fn serve(
    args: Args,
    params: robotd_params::Params,
    bound: Option<tokio::sync::oneshot::Sender<u16>>,
) -> std::process::ExitCode {
    let np = params.netpad.clone();
    let min_interval = Duration::from_secs_f64(1.0 / f64::from(np.max_hz));

    let mut robot = match robotd::Robotd::connect(&args.socket, min_interval).await {
        Ok(r) => r,
        Err(e) => {
            tracing::error!(error = %e, socket = %args.socket.display(), "cannot reach robotd");
            return std::process::ExitCode::FAILURE;
        }
    };
    let socket = match tokio::net::UdpSocket::bind(("0.0.0.0", np.port)).await {
        Ok(s) => s,
        Err(e) => {
            tracing::error!(error = %e, port = np.port, "cannot bind the pad port");
            return std::process::ExitCode::FAILURE;
        }
    };
    let port = socket.local_addr().map(|a| a.port()).unwrap_or(np.port);
    if let Some(tx) = bound {
        let _ = tx.send(port);
    }

    let roller = match robot.ask_roller().await {
        Ok(r) => r.unwrap_or(false),
        Err(e) => {
            tracing::error!(error = %e, "mode request failed");
            return std::process::ExitCode::FAILURE;
        }
    };
    let (bindings, imu_head, drive) = pad_map::read_bindings(&args.config);
    let mut config = Config {
        bindings,
        imu_head,
        drive,
        deadzone: args.deadzone,
        max_head: args.max_head,
        roller,
    };
    tracing::warn!(port, max_hz = np.max_hz, timeout_ms = np.timeout_ms, roller, "listening for a pad over UDP");

    let mut rx = Receiver::new(Timing {
        min_interval,
        timeout: Duration::from_millis(np.timeout_ms),
        fallback: HEARTBEAT,
    });
    let (mut mapper, mut continuous) = (Mapper::new(), Continuous::default());
    let (mut out, mut frame) = (Vec::new(), Vec::new());
    let mut buf = [0u8; 512];
    let mut housekeeping = Instant::now() + pad_map::BINDINGS_POLL;
    let mut config_at = std::fs::metadata(&args.config).and_then(|m| m.modified()).ok();
    let (mut malformed, mut malformed_logged) = (0u64, None::<Instant>);
    let mut reserved_logged = false;

    loop {
        let wake = rx.deadline().map_or(housekeeping, |d| d.min(housekeeping));
        tokio::select! {
            biased;
            got = socket.recv_from(&mut buf) => {
                // Handle this one, then everything else already waiting, in arrival order: edges
                // need every datagram, and the receiver keeps only the newest state.
                let mut next = got.ok();
                while let Some((n, from)) = next {
                    let now = Instant::now();
                    match rx.on_datagram(now, from, &buf[..n]) {
                        Accept::Connected => tracing::warn!(%from, "pad client connected — driving"),
                        Accept::Malformed(e) => {
                            malformed += 1;
                            if malformed_logged.is_none_or(|at| now.duration_since(at) >= MALFORMED_LOG) {
                                tracing::warn!(%from, error = %e, malformed, "dropping datagrams that are not pad state");
                                malformed_logged = Some(now);
                            }
                        }
                        Accept::Taken | Accept::Stale | Accept::OtherPeer => {}
                    }
                    next = socket.try_recv_from(&mut buf).ok();
                }
                if !reserved_logged && rx.reserved_bits_seen() != 0 {
                    tracing::warn!(bits = format!("{:#06x}", rx.reserved_bits_seen()), "client sends button bits this build does not know; ignored");
                    reserved_logged = true;
                }
            }
            _ = tokio::time::sleep_until(wake.into()) => {}
        }

        let now = Instant::now();
        if now >= housekeeping {
            housekeeping = now + pad_map::BINDINGS_POLL;
            let at = std::fs::metadata(&args.config).and_then(|m| m.modified()).ok();
            if at != config_at {
                config_at = at;
                (config.bindings, config.imu_head, config.drive) = pad_map::read_bindings(&args.config);
                tracing::warn!("button bindings reloaded");
            }
            match robot.ask_roller().await {
                Ok(Some(r)) if r != config.roller => {
                    config.roller = r;
                    tracing::warn!(roller = r, "drive mode changed — stick shaping follows");
                }
                Ok(_) => {}
                Err(e) => {
                    tracing::error!(error = %e, "mode request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        match rx.poll(now) {
            Step::Idle => {}
            Step::Gone => {
                tracing::warn!("pad client gone — sending nothing; robotd's deadman holds the robot");
                mapper.pad_gone();
            }
            Step::Tick(pad) => {
                mapper.tick(&pad, &config, now, &mut out, &mut frame);
                for o in out.drain(..) {
                    let result = match &o {
                        Out::Notify(call) => robot.notify(call).await.map(|()| None),
                        Out::Request(call) => robot.request(call).await,
                    };
                    match (result, &o) {
                        (Ok(response), Out::Request(call)) => pad_map::report(call, response.as_ref()),
                        (Ok(_), Out::Notify(_)) => {}
                        (Err(e), _) => {
                            tracing::error!(error = %e, "send failed");
                            return std::process::ExitCode::FAILURE;
                        }
                    }
                }
                if let Some(bytes) = continuous.next(&frame, now)
                    && let Err(e) = robot.write(bytes).await
                {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }
    }
}
```

  Make `BINDINGS_POLL` `pub` in `pad_map` and re-export it from `lib.rs`
  (`pub use bindings::{BINDINGS_POLL, read_bindings};`), if Task 2 did not already.

- [ ] **Step 5: `padd --should-run`.** In `padd/src/main.rs`, add this to `Args`:

```rust
    /// Exit 0 if this daemon should run — `[netpad] enabled` is off — and 1 if `netpadd` has the
    /// pad. For `ExecCondition=`; nothing else is started.
    #[arg(long)]
    should_run: bool,
```

  Add this to `main`, right after the tracing init and before `log_startup_identity!`:

```rust
    if args.should_run {
        return if pad_map::udp_selected(&args.config) {
            std::process::ExitCode::from(1)
        } else {
            std::process::ExitCode::SUCCESS
        };
    }
```

  Add this test in `pad-map/src/lib.rs`:

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn the_selector_reads_netpad_enabled_and_a_bad_file_keeps_bluetooth() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("robotd.toml");
        std::fs::write(&p, "[netpad]\nenabled = true\n").unwrap();
        assert!(super::udp_selected(&p));
        std::fs::write(&p, "[netpad]\nenabled = false\n").unwrap();
        assert!(!super::udp_selected(&p));
        std::fs::write(&p, "[netpad\n").unwrap();
        assert!(!super::udp_selected(&p), "unreadable → padd");
        assert!(!super::udp_selected(&dir.path().join("missing.toml")), "absent → defaults → padd");
    }
}
```

- [ ] **Step 6: Run the tests and the lint**

Run: `cargo test -p netpadd -p pad-map -p padd && RUSTFLAGS="-D warnings" cargo clippy -p netpadd -p pad-map -p padd --all-targets`
Expected: all pass, including `a_datagram_reaches_robotd_as_a_move_and_silence_stops_it`.

- [ ] **Step 7: Commit**

```bash
git add netpadd padd pad-map Cargo.lock
git commit -m "netpadd: the daemon — one tokio task, UDP in, intents out"
```

---

### Task 6: Units, packaging and the update path

**Files:**
- Create: `netpadd/systemd/netpadd.service`, `netpadd/systemd/sysusers.d/netpadd.conf`
- Modify: `padd/systemd/padd.service` (add `ExecCondition=` after `Type=exec`)
- Modify: `deploy/updater.toml` (`required_files`: add `"bin/netpadd"`)
- Modify: `.github/workflows/dev.yml`, `.github/workflows/_build-release.yml` and
  `scripts/dev-push.sh`: stage the binary, unit and sysusers file wherever padd's are
  (`grep -n padd` in each file)
- Modify: `scripts/install.sh` l.391 and l.640: add `netpadd.service` to both unit loops, next to
  `padd.service`
- Modify: `docs/design/restart-order.md`: the unit table (§1, l.~25) and the `units_to_restart`
  list (l.~76)

- [ ] **Step 1: Run the packaging tests first to see the gap**

Run: `cargo test -p xtask`
Expected: it passes now. The tests discover units from `install.sh`, sysusers files from the repo,
and `required_files` from the recipes. After Step 2 they should fail and name each place that still
lacks `netpadd`. That is the checklist.

- [ ] **Step 2: Write the unit.** `netpadd/systemd/netpadd.service`:

```ini
# netpadd — a gamepad over UDP, as an intent client. See netpadd/src/main.rs.
#
# Install to /etc/systemd/system/netpadd.service.
#
# Enabled at boot like padd, and exactly one of the two runs: both ask `[netpad] enabled` in
# ExecCondition=, which is the whole selection. Not Conflicts=: the updater restarts every shipped
# unit in alphabetical order (docs/design/restart-order.md §1), so padd would restart after this one
# and stop it, and every update would hand the robot back to Bluetooth. A skipped condition is not a
# failure, so Restart=always does not loop on it and the update's restart step still succeeds.

[Unit]
Description=Robot gamepad over UDP, intent client
Documentation=file:///opt/robot/daemon/current/docs/robot/udp-pad.md
# As padd: after robotd, never requiring it. A robotd that is not there yet is retried in 5 s.
After=robotd.service local-fs.target network.target

[Service]
Type=exec
ExecCondition=/opt/robot/daemon/current/bin/netpadd --should-run
ExecStart=/opt/robot/daemon/current/bin/netpadd

# Unprivileged, for padd's reason: an intent client with no special access. It needs robotd's
# 0660 socket and nothing else — no `input`, since the pad is on the other end of the network.
User=netpadd
Group=netpadd
SupplementaryGroups=robot

Restart=always
RestartSec=5s

NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
PrivateDevices=yes
ProtectKernelTunables=yes
ProtectKernelModules=yes
ProtectControlGroups=yes
RestrictSUIDSGID=yes
# robotd's unix socket and the pad's IPv4 UDP port. No AF_NETLINK: there is no device to watch.
RestrictAddressFamilies=AF_UNIX AF_INET
RestrictNamespaces=yes
LockPersonality=yes
MemoryDenyWriteExecute=yes
SystemCallFilter=@system-service
SystemCallErrorNumber=EPERM
CapabilityBoundingSet=

Environment=RUST_LOG=info
StandardOutput=journal
StandardError=journal

# /run/netpadd/identity.json, for `robotctl health` and the updater's startup check — see padd.service.
RuntimeDirectory=netpadd

[Install]
WantedBy=multi-user.target
```

  `netpadd/systemd/sysusers.d/netpadd.conf`:

```
# Creates the `netpadd` system user and group. Install to /usr/lib/sysusers.d/netpadd.conf.
#
# An ordinary intent client, like padd: its only access is the `robot` group, granted in
# netpadd.service, which reaches robotd's 0660 socket. netpadd.service fails to start without it.
u netpadd - "Robot gamepad over UDP" - -
```

  Add this to `padd/systemd/padd.service`, under `Type=exec`:

```ini
# Stands down when `[netpad] enabled` gives the pad to netpadd — see netpadd.service for why this
# is a condition and not Conflicts=. An unreadable config answers "run": a bad file never leaves
# somebody without the Bluetooth pad.
ExecCondition=/opt/robot/daemon/current/bin/padd --should-run
```

- [ ] **Step 3: Update every packaging site** that the xtask tests and `grep -n padd` name. Mirror
  padd's lines with `netpadd` in place of `padd`:
  - `cp …/release/netpadd staged/`
  - `--include "netpadd/systemd/netpadd.service=systemd/netpadd.service"`
  - `--include "netpadd/systemd/sysusers.d/netpadd.conf=systemd/sysusers.d/netpadd.conf"`

  Also add `"bin/netpadd"` to `deploy/updater.toml` and `netpadd.service` to both `install.sh`
  loops.

- [ ] **Step 4: Check whether anything on the board filters inbound UDP.**

Run: `grep -rn "nft\|iptables\|ufw\|firewall" scripts deploy hooks | grep -v "^Binary"`
Expected: either nothing, or a rule set. Record what you found in the task report. If a filter
exists, open `port` 4210/udp in it. If there is none, say so; do not add one.

- [ ] **Step 5: Run the packaging tests and the check**

Run: `cargo test -p xtask && cargo +1.94.1 check --workspace`
Expected: both pass.

- [ ] **Step 6: Exercise the units for real, under systemd, in the simulated duck**

```bash
scripts/duck-sim boot && scripts/duck-sim shell
# on the duck:
systemctl is-active padd netpadd        # active / inactive (condition)
sudo robotctl configure                  # set netpad.enabled = true, save
sudo systemctl restart padd netpadd
systemctl is-active padd netpadd        # inactive / active
systemctl show -p ConditionResult padd   # ConditionResult=no
sudo systemctl restart netpadd padd      # the update's order, roughly — netpadd must stay active
systemctl is-active netpadd
```

  Expected: as in the comments. If `duck-sim boot` is unavailable (it needs sudo and nspawn), say
  so in the report and leave this step unchecked.

- [ ] **Step 7: Commit**

```bash
git add netpadd/systemd padd/systemd deploy/updater.toml .github scripts docs/design/restart-order.md
git commit -m "netpadd: unit, user and packaging; padd stands down on [netpad] enabled"
```

---

### Task 7: `duckctl udp-pad`, the reference client

**Files:**
- Create: `duckctl/src/udp_pad.rs`
- Modify: `duckctl/Cargo.toml`: add `pad-map = { path = "../pad-map" }` and `gilrs = "0.11"`
- Modify: `duckctl/src/main.rs`:
  - add `mod udp_pad;`
  - add the `Command::UdpPad` variant (after `Pad(Pad)`, ~l.1126)
  - dispatch it at the top of `run()`, **before** the Bluetooth `Manager::new()` (~l.1440), because
    this command never touches Bluetooth

**Interfaces:**
- Consumes: `pad_map::wire::{Packet, DEFAULT_PORT}` and `pad_map::Buttons`.
- Produces: `udp_pad::run(host: &str, hz: u32) -> Result<(), Box<dyn std::error::Error>>` and a pure
  `udp_pad::Pacer`.

- [ ] **Step 1: Write the failing tests** for the pacing. The I/O is a thin loop around them.

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn sends_are_capped_at_the_interval_whether_or_not_anything_changed() {
        let t = Instant::now();
        let mut p = Pacer::new(Duration::from_millis(33));
        assert!(p.due(t), "first");
        assert!(!p.due(t + Duration::from_millis(10)), "capped");
        assert!(p.due(t + Duration::from_millis(33)), "change at the slot");
        assert!(!p.due(t + Duration::from_millis(50)), "steady, not yet");
        assert!(p.due(t + Duration::from_millis(66)), "keepalive");
    }

    #[test]
    fn host_without_port_gets_the_default() {
        assert_eq!(target("10.0.0.5"), "10.0.0.5:4210");
        assert_eq!(target("10.0.0.5:9000"), "10.0.0.5:9000");
        assert_eq!(target("duck.local"), "duck.local:4210");
    }
}
```

- [ ] **Step 2: Run them to see them fail**

Run: `cargo test -p duckctl udp_pad`
Expected: compile errors.

- [ ] **Step 3: Implement it**

```rust
//! `duckctl udp-pad` — the pad plugged into this machine, sent to a duck's `netpadd`.
//!
//! The reference sender for the format `docs/robot/udp-pad.md` owns. It sends the pad's whole
//! state the moment anything changes, at most `hz` times a second, and a keepalive every 1/hz
//! when nothing does. No pad, nothing sent — the robot times the client out and its deadman holds
//! it, exactly as when a Bluetooth pad goes away.

use std::hash::{BuildHasher, Hasher};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use gilrs::{Axis, Button, Gilrs};
use pad_map::Buttons;
use pad_map::wire::{DEFAULT_PORT, Packet};

/// When to send: a change and a keepalive are both capped at the interval. What a change buys
/// is that the loop wakes for it at once rather than at the next keepalive (see `run`).
pub struct Pacer {
    interval: Duration,
    last: Option<Instant>,
}

impl Pacer {
    pub fn new(interval: Duration) -> Self {
        Self { interval, last: None }
    }

    /// Whether to send now; records it when yes. A change and a keepalive are both capped at
    /// the interval — the difference is that the loop wakes for a change at once (see `run`).
    pub fn due(&mut self, now: Instant) -> bool {
        let due = self.last.is_none_or(|at| now.duration_since(at) >= self.interval);
        if due {
            self.last = Some(now);
        }
        due
    }

    /// When the next send may happen.
    pub fn next(&self) -> Option<Instant> {
        self.last.map(|at| at + self.interval)
    }
}

fn target(host: &str) -> String {
    if host.rsplit_once(':').is_some_and(|(_, p)| p.parse::<u16>().is_ok()) {
        host.to_owned()
    } else {
        format!("{host}:{DEFAULT_PORT}")
    }
}

const GILRS: [(Button, Buttons); 12] = [
    (Button::South, Buttons::A),
    (Button::East, Buttons::B),
    (Button::West, Buttons::X),
    (Button::North, Buttons::Y),
    (Button::LeftTrigger, Buttons::LB),
    (Button::RightTrigger, Buttons::RB),
    (Button::Start, Buttons::START),
    (Button::Select, Buttons::SELECT),
    (Button::DPadUp, Buttons::UP),
    (Button::DPadDown, Buttons::DOWN),
    (Button::DPadLeft, Buttons::LEFT),
    (Button::DPadRight, Buttons::RIGHT),
];

/// How often the pad is read. Faster than any `hz` worth sending, so a change is seen within a
/// millisecond and the cap, not the polling, is what decides when it goes.
const POLL: Duration = Duration::from_millis(1);

pub fn run(host: &str, hz: u32) -> Result<(), Box<dyn std::error::Error>> {
    let mut gilrs = Gilrs::new().map_err(|e| format!("no gamepad subsystem: {e}"))?;
    let to = target(host);
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.connect(&to)?;
    let mut pacer = Pacer::new(Duration::from_secs_f64(1.0 / f64::from(hz)));
    // A random start, so a restarted sender is never mistaken for a stale one for long.
    let mut seq = std::collections::hash_map::RandomState::new().build_hasher().finish() as u32;
    let started = Instant::now();
    let (mut last, mut sent) = (None::<Packet>, 0u64);
    eprintln!("sending the first pad on this machine to {to}, at most {hz} Hz — Ctrl-C to stop");

    loop {
        while gilrs.next_event().is_some() {}
        let Some((_, pad)) = gilrs.gamepads().next() else {
            last = None;
            std::thread::sleep(Duration::from_millis(200));
            continue;
        };
        let held = GILRS.iter().filter(|(b, _)| pad.is_pressed(*b)).fold(Buttons::NONE, |a, (_, bit)| a | *bit);
        let trigger = |b: Button| f64::from(pad.button_data(b).map_or(0.0, |d| d.value()));
        let now = Instant::now();
        let mut packet = Packet::from_axes(
            0,
            now.duration_since(started).as_millis() as u32,
            [
                f64::from(pad.value(Axis::LeftStickX)),
                f64::from(pad.value(Axis::LeftStickY)),
                f64::from(pad.value(Axis::RightStickX)),
                f64::from(pad.value(Axis::RightStickY)),
            ],
            [trigger(Button::LeftTrigger2), trigger(Button::RightTrigger2)],
            held,
        );
        let changed = last.is_none_or(|l| l.axes != packet.axes || l.buttons != packet.buttons);
        if pacer.due(now) {
            seq = seq.wrapping_add(1);
            packet.seq = seq;
            // A refused send (no route yet, the duck rebooting) is not fatal: the next one tries again.
            if socket.send(&packet.encode()).is_ok() {
                sent += 1;
            }
            last = Some(packet);
            if sent % 300 == 1 {
                eprintln!("{sent} sent");
            }
        }
        let wake = if changed { now + POLL } else { pacer.next().unwrap_or(now + POLL).min(now + POLL * 5) };
        if let Some(d) = wake.checked_duration_since(Instant::now()) {
            std::thread::sleep(d);
        }
    }
}
```

  Add this to `Command` in `main.rs`:

```rust
    /// Drive a duck running `netpadd` from the pad plugged into this machine, over UDP on the LAN.
    ///
    /// No Bluetooth: give the duck's address (`duckctl ip` prints it). The duck needs
    /// `[netpad] enabled = true` — docs/robot/udp-pad.md.
    UdpPad {
        /// The duck's address, optionally with `:port` (default 4210).
        host: String,
        /// Most datagrams a second.
        #[arg(long, default_value_t = 30, value_parser = clap::value_parser!(u32).range(10..=100))]
        hz: u32,
    },
```

  Add this at the top of `run()`, right after `let cli = Cli::parse();`:

```rust
    // Before anything Bluetooth: this command talks UDP to an address it was given.
    if let Command::UdpPad { host, hz } = &cli.command {
        return udp_pad::run(host, *hz);
    }
```

  Then add `Command::UdpPad { .. } => unreachable!("handled before discovery")` to every later
  `match cli.command` that the compiler says is now non-exhaustive.

- [ ] **Step 4: Run the tests and the lint**

Run: `cargo test -p duckctl udp_pad && RUSTFLAGS="-D warnings" cargo clippy -p duckctl --all-targets`
Expected: 2 tests pass, and clippy is clean.

- [ ] **Step 5: End to end against the simulated duck**

```bash
scripts/duck-sim                                  # terminal 1
printf '[netpad]\nenabled = true\n' > /tmp/netpad.toml
cargo run -p netpadd -- --socket ~/.cache/duck-sim/<name>.sock --config /tmp/netpad.toml   # terminal 2
cargo run -p duckctl -- udp-pad 127.0.0.1         # terminal 3, a pad plugged in
```

  Expected:
  - Start stands the duck up, a second Start walks, and the left stick drives it.
  - Stopping `duckctl` logs `pad client gone` in netpadd within 250 ms, and the duck stops.
  - Restarting `duckctl` logs `connected` and driving resumes.

  If no pad is available, run the scripted check below instead and say which one ran:

```bash
python3 - <<'EOF'
import socket, struct, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def pkt(seq, ly=0, buttons=0):
    return b"DKPD" + struct.pack("<BBII6hH", 1, 0, seq, 0, 0, ly, 0, 0, 0, 0, buttons)
seq = 1
for b in (64, 0, 64, 0):                  # Start, release, Start, release — stand, then walk
    s.sendto(pkt(seq, 0, b), ("127.0.0.1", 4210)); seq += 1; time.sleep(1.5)
for _ in range(90):                        # 3 s forward at 30 Hz, two of them reordered
    s.sendto(pkt(seq, 32767), ("127.0.0.1", 4210)); seq += 1; time.sleep(1/30)
s.sendto(pkt(seq - 40, -32767), ("127.0.0.1", 4210))   # a stale "backwards": must be ignored
# then silence: the duck must stop
EOF
```

- [ ] **Step 6: Commit**

```bash
git add duckctl Cargo.lock
git commit -m "duckctl udp-pad: the pad on this machine, to a duck's netpadd"
```

---

### Task 8: Docs

**Files:**
- Create: `docs/robot/udp-pad.md`
- Modify: `docs/README.md` (`robot/` table: a row after `pair-a-gamepad.md`)
- Modify: `docs/design/architecture.md`: in the service table near `padd` (l.~68 and the table at
  l.~87), one sentence plus a link
- Modify: `docs/robot/pair-a-gamepad.md`: one sentence plus a link, near the top
- Modify: `docs/robot/duckctl.md`: a `udp-pad` entry in the same shape as its neighbours
- Modify: `padd/src/main.rs` crate doc: one sentence pointing to `netpadd` and `[netpad]`

- [ ] **Step 1: Write `docs/robot/udp-pad.md`.** It owns these sections, in this order:
  1. **What it is:** one paragraph. A program on the LAN sends pad state, and the robot applies
     `padd`'s mapping. For a client outside the LAN, link `../design/remote-webrtc.md` and say that
     is the path there.
  2. **Turning it on:**
     - `sudo robotctl configure`, then `netpad.enabled = true`
     - `sudo systemctl restart padd netpadd`
     - how to check: `systemctl is-active padd netpadd`
     - turning it off again
  3. **Driving from a laptop:** `duckctl udp-pad $(duckctl ip)`.
  4. **Writing your own sender.**
     - The wire table from the spec §2, which is now owned here.
     - The Python sender from Task 7 Step 5, as an example.
     - The sender's rules:
       - send the whole state every time
       - start `seq` at a random value and add 1 per datagram
       - send on change, plus a keepalive every 1/hz
       - stop sending to let go
  5. **What the robot does:**
     - the newest state only, and the `max_hz` cap
     - peer lock
     - silence past `timeout_ms` counts as the client gone, and the deadman holds the robot
     - the first datagram fires nothing
  6. **Trust:** no authentication. Anyone on the LAN can take the robot once the current client is
     quiet. This is a LAN tool.

  Write it in the house voice: say why, and use full sentences. `pair-a-gamepad.md` is the model to
  follow.

- [ ] **Step 2: Remove the wire table from the spec**, replacing it with a link to
  `docs/robot/udp-pad.md`, so the format has one owner.

- [ ] **Step 3: Check the links**

Run: `grep -rn "udp-pad.md" docs padd netpadd duckctl | head` and open each target path.
Expected: every link resolves.

- [ ] **Step 4: Final full check**

Run: `cargo fmt --all --check && cargo +1.94.1 check --workspace && cargo test --workspace`
Expected: everything passes. Report the test count.

- [ ] **Step 5: Commit**

```bash
git add docs padd/src/main.rs
git commit -m "docs: the pad over UDP — udp-pad.md owns it"
```
