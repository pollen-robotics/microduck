# netpadd — a gamepad over UDP

Status: design, approved in conversation 2026-10-07. Branch `udp-pad`, from `main`.

## Goal

Drive a duck from a program on the **same LAN** that sends gamepad state — sticks, triggers,
buttons — over UDP. The robot applies exactly the mapping a Bluetooth pad gets from `padd`.

Success means:

- **Lowest latency the link allows.** A stick change is forwarded to `robotd` as soon as it arrives,
  limited only by a configurable rate cap (`max_hz`, default 30).
- **No buffering anywhere.** Only the newest state counts, and nothing is queued or retransmitted.
  A late packet is dropped.
- **Robust.** A lost packet costs nothing. A silent client looks exactly like a pad going away, so
  `robotd`'s deadman holds the robot. One client at a time.

Out of scope: clients outside the LAN (WebRTC is the path there — `remote-webrtc.md`), authentication,
the pad's IMU, and the raw input tap (`robotctl monitor`'s pad block).

## Shape

```
 laptop                                   robot
 ┌──────────────────┐   UDP, 28 B   ┌──────────────────────────────┐  unix socket  ┌────────┐
 │ duckctl udp-pad  │ ────────────▶ │ netpadd                      │ ────────────▶ │ robotd │
 │ (gilrs, ≤ hz)    │               │  wire → PadFrame → pad-map   │   intents     └────────┘
 └──────────────────┘               └──────────────────────────────┘
                                     padd: not running (ExecCondition)
```

Three pieces:

1. **`pad-map`** — a new library crate holding `padd`'s mapping, moved out of `padd/src/main.rs`.
2. **`netpadd`** — a new daemon: a UDP listener feeding `pad-map`, and an intent client of
   `/run/robotd.sock`, like `padd`.
3. **`duckctl udp-pad <host>`** — the reference client: reads a pad on the laptop and sends its state.

## 1. `pad-map`: the mapping, shared

Everything in `padd` that turns pad state into intents moves into `pad-map`, unchanged in
behaviour:

- the D-pad modes and `mode_exit_calls`
- `HoldButton` and the Start/Select hold thresholds
- `DriveLimits`, which covers `[pad_drive]` and roller shaping
- the deadzone, `[pad]` button bindings, mouth and sounds
- body pose and `Continuous` (holding back identical frames, `HEARTBEAT`)

It is **pure**: input frames come in and `proto::Call`s go out. Holding the socket, reading the
config, asking `robot.mode` and logging stay in each daemon. The input is a source-neutral snapshot:

```rust
pub struct PadFrame {
    pub left: [f64; 2], pub right: [f64; 2],   // raw, -1..1, up positive (gilrs convention)
    pub lt: f64, pub rt: f64,                   // 0..1
    pub held: Buttons,                          // what is down now
    pub pressed: Buttons, pub released: Buttons, // edges since the previous frame
    pub attitude: Option<[f32; 4]>,             // pad IMU; always None from netpadd
}
```

`padd` keeps gilrs, the IMU and the tap, and builds a `PadFrame` per tick from them. Its existing
tests move with the code they test. **`padd`'s behaviour does not change**, and the refactor is a
separate commit that touches nothing else.

## 2. Wire format

One datagram per state. Every field is little-endian, and the whole packet is 28 bytes:

| offset | size | field | |
|---|---|---|---|
| 0 | 4 | magic | `b"DKPD"` |
| 4 | 1 | version | `1` |
| 5 | 1 | flags | reserved, sent 0, ignored |
| 6 | 4 | seq | u32, +1 per datagram, wraps |
| 10 | 4 | t_ms | u32, the client's own clock; logged for jitter only, never compared with the robot's |
| 14 | 12 | axes | i16 ×6: lx ly rx ry (±32767 → ±1), lt rt (0..32767 → 0..1) |
| 26 | 2 | buttons | u16: 0 A · 1 B · 2 X · 3 Y · 4 LB · 5 RB · 6 Start · 7 Select · 8 Up · 9 Down · 10 Left · 11 Right |

- **The client sends raw values.** `pad-map` applies the deadzone, as it does for `padd`.
- **Wrong magic, a packet shorter than 28 bytes, or an unknown version:** the packet is dropped and
  counted, and logged with the running count at most once every 10 s. A version-1 packet *longer* than 28 bytes is accepted and its
  tail is ignored, so fields can be added without a version bump. Set reserved button bits are
  ignored and logged once.

## 3. The receive path

`netpadd` runs on tokio: a `current_thread` runtime and **one task** that owns the
`tokio::net::UdpSocket`, the connection to `robotd` (`tokio::net::UnixStream`) and the receive state.
There are no locks and no channels: one task means a single owner, and a single thread means no
scheduling jitter from a work-stealing pool. The decision logic is a pure state machine
(`(time, source, bytes)` in, "send now / next deadline" out), so the tokio loop is thin glue and the
tests run without a runtime.

1. **The loop waits in `tokio::select!` (`biased`, socket first).** One arm is `recv_from`; the
   other is `sleep_until` the next deadline: the next send slot, the periodic tick, or the client
   timeout.
2. **When a datagram arrives, it drains the socket with `try_recv_from` until `WouldBlock`.** Each
   datagram is then handled in arrival order:
   - **Peer lock.** If a peer is locked and the source address is a different one, the datagram is
     dropped. Otherwise the source becomes the locked peer.
   - **Freshness.** The datagram is dropped unless `seq.wrapping_sub(last_seq) as i32 > 0`. After a
     timeout, any `seq` is accepted, so a client that restarts is picked up again.
   - **Edges.** The first datagram after a timeout makes no edges: its buttons are taken as
     already held, so a reconnecting client never fires the skill on a button it was holding.
     After that, the new `buttons` are diffed against the last accepted state, and the result is
     OR-ed into the pending `pressed` / `released` sets. Edges are therefore found per datagram, not
     per tick, and a press and release that both land between two sends are not lost.
   - **The latest state replaces the held state.**
3. **Send.** `pad-map` runs once and its calls go to `robotd`. This happens when the state changed
   or edges are pending and at least `1/max_hz` has passed since the last send. Otherwise it happens
   at the slot. In-between states are merged, and nothing accumulates.
4. **Fallback tick.** While the client is alive, `pad-map` also runs every `HEARTBEAT` (100 ms,
   `padd`'s own) even with no new datagram, so hold timing and the stationary heartbeat continue
   across lost packets. It is deliberately slower than `1/max_hz`: a periodic tick at the cap would
   keep every slot occupied and make a real change wait for the next one. The client's keepalive
   carries the steady state at `hz`.
5. **Timeout.** If no datagram is accepted for `timeout_ms`, `netpadd` sends **nothing**, as `padd`
   does with no pad. `robotd`'s deadman (`[safety] deadman_ms`, 500) holds the robot, holds in
   flight are reset (`HoldButton::reset`), and the peer lock is released. The transition is logged
   once at `warn` in each direction.

**Writes to `robotd`** are bounded by `tokio::time::timeout` at one send interval. A `robotd` that
stops reading must not freeze the receive loop and let datagrams pile up behind it. A timed-out or
failed write ends the process: `Restart=always` brings `netpadd` back after 5 s, exactly as `padd`
does when `robotd` goes away. Exiting also drops every belief about the robot (`up`, the mode). The `robot.mode` question asked once a second (roller shaping) is also
timeout-bounded, and its reply is read on the same connection.

`timeout_ms` must be below `deadman_ms`, or `netpadd` would keep re-sending a dead client's last
state up to the deadman. A config that breaks this is refused at load and the error names both keys.

`SO_RCVBUF` is left at the kernel default. Draining on every wake already means at most one wake's
worth of datagrams is ever read, and a smaller buffer only adds drops under a burst.

## 4. Configuration — `robotctl configure`

New section `[netpad]` in `robotd-params`, with entries in the registry (its completeness test
enforces that) and commented defaults in `deploy/robotd.toml`:

| key | default | |
|---|---|---|
| `enabled` | `false` | UDP is the pad source and `padd` stands down |
| `port` | `4210` | UDP port, bound on IPv4 `0.0.0.0` |
| `max_hz` | `30` | cap on how often intents are sent to `robotd`; 10–100, refused outside |
| `timeout_ms` | `250` | silence after which the client counts as gone; must be < `[safety] deadman_ms` |

All four are read at startup. Changing one takes a restart of `padd` and `netpadd` (§5).

## 5. Units, and why not `Conflicts=`

`padd` and `netpadd` must never both drive. The obvious tool, `Conflicts=`, breaks on update: the
updater runs `systemctl restart` on every shipped unit with an `[Install]` section
(`restart-order.md` §1), in alphabetical order. That restarts `netpadd` and then `padd`, and starting
`padd` would stop `netpadd`, so every update would silently hand the robot back to Bluetooth.

Instead, **the selector is the config key**, and each unit asks it before starting:

- **`padd.service`** gains `ExecCondition=/opt/robot/daemon/current/bin/padd --should-run`. It
  exits 1, so systemd skips the unit, when `[netpad] enabled = true`, and 0 otherwise.
- **`netpadd.service`** has `ExecCondition=… netpadd --should-run`, the inverse.

A skipped condition is not a failure, so `Restart=always` does not loop and the update's restart
step still succeeds. Boot, update and manual restarts all reach the same answer from the same key. A
config that cannot be read means `padd` runs: the existing rule is that a bad file never leaves
somebody without a pad.

`netpadd.service` copies `padd.service`'s hardening, with these differences:

- `User=netpadd`, with only the `robot` group added, and no `input`.
- `RestrictAddressFamilies=AF_UNIX AF_INET`. It binds IPv4 `0.0.0.0` only.
- `RuntimeDirectory=netpadd` for `identity.json`, so `robotctl health` and the updater's startup
  check see it the way they see `padd`.

The plan must also:

- add `bin/netpadd` to the required binaries in `deploy/updater.toml`
- add `netpadd` to `restart-order.md`'s unit table
- check whether anything on the board filters inbound UDP on `port`

## 6. `duckctl udp-pad <host>[:port]`

- **What it sends.** It reads the first gilrs pad on the laptop and packs the full state (§2). It
  sends immediately when anything changes, and otherwise sends a keepalive every `1/hz`.
- **Rate.** `--hz` (default 30) caps both kinds of send.
- **Sequence numbers.** `seq` starts at a random value, so a restart cannot look like a stale
  sender.
- **No pad on the laptop:** it sends nothing, and the robot times out.
- **Output.** It prints one status line: packets sent, and the current mode guessed from D-pad
  edges.

## 7. Testing

- **`pad-map`:** `padd`'s existing tests, moved. They pass unchanged before and after the refactor.
- **Wire:** a round trip; rejection of bad magic, short packets and unknown versions; an over-long
  v1 packet accepted.
- **Receive logic, as the pure state machine of §3 (no runtime needed):**
  - `seq` ordering, wrap at `u32::MAX`, and acceptance of any `seq` after a timeout
  - peer lock and its release
  - press and release inside one send interval producing a tap
  - the rate cap: no two sends closer than `1/max_hz`, and the latest state sent at the slot
  - timeout: nothing sent, and holds reset
- **Config:** range checks, `timeout_ms < deadman_ms`, and the registry completeness test.
- **End to end against the simulated duck (`scripts/duck-sim`):** `duckctl udp-pad` → `netpadd` →
  `robotd`, plus a scripted sender that drops, reorders and stops, checking that the robot walks,
  stops on silence, and that a reordered packet never moves it backwards in time.

## 8. Docs

- **Owning page:** `docs/robot/udp-pad.md`, covering enabling it, the wire format and the client.
  It is listed in `docs/README.md` under `robot/`. The wire table in §2 moves there, and the crate
  doc of `netpadd` links to it rather than repeating it.
- **One sentence plus a link** in `architecture.md`'s service table, in `pair-a-gamepad.md` ("or
  over UDP"), and in `duckctl.md`.
