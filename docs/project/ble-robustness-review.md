# The BLE stack's robustness — review, 2026-11-19

A read-only review of everything between the phone's radio and the robot's daemons, done
2026-11-10 and re-checked against these revisions on 2026-11-19:

| Repo | Revision |
|---|---|
| `microduck` (this repo) | `56424efe` |
| `microduck_yocto` | `6371aa6` |
| `microduck-app` | `d2bd2f7` (0.3.0) |

Nothing was built or run for it. **Verified** means traced in the code; **suspected** means the
claim rests on BlueZ, radio or scheduler behaviour that was not observed. The mechanisms
themselves belong to [`app-path-design.md`](../design/app-path-design.md) (`btd`, `configd`),
[`mobile-app.md`](../design/mobile-app.md) (the app) and
[`pad-minimal-pairing.md`](pad-minimal-pairing.md) (pads).

The happy path holds. What is weak is **recovery** — a radio that stalls, a session that ends
badly, a daemon behind the link that hangs — and almost none of the code that touches BlueZ or a
radio is tested.

## The layers

**OS (`microduck_yocto`).** The kernel builds in BT, HCIUART (H4), UHID and RFKILL. BlueZ is 5.86
with a stock `main.conf` (`Privacy=off`, `ControllerMode=dual`) and the stock `org.bluez` D-Bus
policy.

- **zero3** — AIC8800 over UART at `/dev/ttyS1`, attached by `aic8800-bt-attach.service`
  (`hciattach -n -s 1500000 /dev/ttyS1 any 1500000 flow`, `Restart=on-failure`, 2 s).
- **beta** — WQ9201S. `wq_wlan` downloads the BT firmware over SDIO; `wq9201s-bt-attach` waits
  for `wlan0`, then makes three `btattach -P h4 -S 1500000` attempts. The unit has no `Restart=`,
  deliberately: each start power-cycles the chip through rfkill and wipes the firmware.
- **Bonds** live in `/var/lib/bluetooth`, bind-mounted from `/var/lib/robot/bluetooth` on the data
  partition, so they survive A/B updates and rollback. `main.conf` is per-slot, so a
  `Privacy=device` set by hand is lost on every update while the bonds made under it survive.

**`btd`** — unprivileged, bluer 0.17 on a current-thread runtime, `Restart=always`.

- One service, one characteristic used as a pipe: read returns the API version, write carries
  requests, notify carries replies. Newline framing, inbound lines capped at 8 KiB, outbound
  chunks of `mtu - 3` clamped to 20..512.
- A legacy advert (100–150 ms, IPv4 address and busy bit in manufacturer data) and an extended
  twin while connectable. Name and address are reconciled with `configd` every 5 s.
- One session per StartNotify. The PIN gate (3 attempts) or, when the `ble-psk` credential
  exists, Noise-PSK through `duck-auth`.
- Routes to `robotd`, `updaterd`, `configd` and `nfcd` over unix sockets, one connection per
  lane; connect 3 s, write 5 s, one reconnect on EPIPE.

**`configd`** — root, zbus. Pad discovery, bonding (LE and classic), the default agent while a pad
bonds, re-paging classic pads (12 s doubling to 120 s). It polls the object tree rather than
listening for signals.

**`padd`** — reads the pad through gilrs on evdev at 50 Hz and sends intents to `robotd`. It has
no BlueZ access.

**The app** — `tauri-plugin-blec` 0.14 (btleplug); everything radio-side is in Rust.

- Unfiltered scan, as §3.3 asks. Connect budget 25 s, opened once after a scan on failure.
- JSON-RPC id correlation, lane budgets of 15 / 60 / 180 s, one transparent retry on a link
  error.
- Setup code (Noise, kept in the Keychain/Keystore) or the legacy PIN, then `hello`. On Android a
  system bond (`tauri-plugin-duck-bond`) precedes every connect.

## Findings

### Can move the robot unintentionally, or leave it unreachable

**1. The app replays non-idempotent calls after a drop.** Verified.
`microduck-app/src-tauri/src/session.rs:410-415` retries on any `CallError::Link`, including the
link closing *after* the request was written. `robot_do`, `set_skill`, `net_connect`,
`os_apply` and `system_reboot` can run twice. A reboot drops the link every time, so it is always
reported as "could not connect to the duck". Retry only when the write itself failed.

**2. A stalled pad radio keeps the robot walking.** Verified.
`padd/src/main.rs:547` sends gilrs's last stick values every tick with no check on their age. The
deadman fires only once the kernel tears the link down — up to about 20 s on classic, set by the
link parameters on LE. `padd/src/tap.rs` already timestamps every report, so "no report for N ms
while a stick is off-centre" is cheap to add.

**3. beta: Bluetooth can die silently, and only a reboot brings it back.** Verified.
`wq9201s-bt-attach:34-35` ends with a bare `wait; exit 0`, which returns 0 whatever `btattach`
exited with. If `btattach` dies after a good attach, the unit goes inactive, not failed, and
nothing watches it. A manual `systemctl restart wq9201s-bt` re-runs `rfkill unblock all`, which by
the unit's own comment power-cycles the chip and wipes the firmware, so it probably cannot recover
either.

**4. PIN mode: a subscriber that never writes keeps the robot busy until `btd` restarts.**
Verified in `btd`; BlueZ omitting StopNotify is suspected.
`watch_session_peer` arms only once a write has recorded the peer (`btd/src/bluez.rs:441-445`).
A phone that subscribes and is killed before its first write, with no StopNotify from BlueZ, keeps
the slot filled and the advert non-connectable. Secure mode's 15 s handshake deadline covers this;
plain `session::run` has no idle limit.

**5. Every session end takes BLE down for 5–15 s.** Verified.
Any session end tears down the whole registration — GATT application, adverts, agent, chorale —
and `serve` sleeps `ADAPTER_RETRY` (5 s) before bring-up (`btd/src/bluez.rs:221-236`). Bring-up
then waits on `configd` for name and address (5 s each at worst) before advertising. A quick
reconnect from the app fails inside that window. Suspected on top: re-claiming the default agent
can take it from `configd` in the middle of a pad bond.

**6. A bluetoothd restart that keeps hci0 leaves the robot dark.** Suspected.
`watch_adapter` checks only `is_powered`. A new bluetoothd answers `true` on the same object path,
the advert handle is still `Some`, and nothing re-registers. On zero3 a bluetoothd restart takes
hci0 with it, which hides this; a crash-restart on another image would not.

### Degrades, or tells the user the wrong thing

**7. App: one failed `hello` can make every later call reconnect.** Ordering verified; the
callback firing is suspected. `ensure_ready` clears `dropped` before closing the stale link
(`session.rs:464-466`). If that close runs the old link's disconnect callback, the shared flag is
set again for the new link, and with status polling every 3 s this becomes a steady reconnect
loop. Likely trigger: `btd` answering right after a reboot, before the daemon behind `hello` is up.
Separately, every `hello` error — drop, timeout, an error reply — is reported as "too old, run
`duckctl os apply`" (`session.rs:587`).

**8. App: link state and error text.** Verified.
- A failed open sends `Connecting` and never `Down`, so the pill stays on "connecting…" and the
  heartbeat, which runs only while `up`, stops probing.
- `isPinError` (`src/robot/api.ts:60-70`) classifies blec's write timeout and Android's
  "Bluetooth pairing failed:" as PIN errors, so an out-of-range robot shows the PIN form.
- On iOS a denied Bluetooth permission reads as `Unknown`; the list is simply empty, with no
  pointer to Settings.

**9. Secure mode drops a session that is only listening.** Verified.
The 60 s lease (`btd/src/secure.rs`) is renewed only by inbound data. A phone that starts
`update.apply` and watches `update.subscribe` without sending is cut off mid-update, then meets
finding 5.

**10. `btd`: no reply timeout, serial dispatch, and dropped responses.** Verified.
A hung upstream stalls every lane of the session, because `dispatch` is awaited inline. When the
512-line reply queue is full, `btd/src/upstream.rs` drops a JSON-RPC response as silently as a
progress line, and the client waits out its own budget. Reader tasks outlive their session until
the daemon closes its side.

**11. `btd`: chorale tasks leak on every session end.** Verified.
The listening `JoinHandle` is dropped rather than aborted (`btd/src/chorale.rs:207,237-241`). In
fallback mode that leaves an active discovery scan running, and with finding 5 they accumulate.
`spawn_follow` also leaves one task per non-duck device seen.

**12. `configd`: the stale-bond rule can delete a working LE pad's bond.** Suspected.
`is_stale_bond` reads "bonded, not connected, RSSI this sweep" as "in pairing mode"
(`configd/src/bluez.rs`). For LE, BlueZ may set RSSI from any advert. Under Fault 3 of
[`pad-minimal-pairing.md`](pad-minimal-pairing.md) — phone connected, Xbox pad advertising but
unable to connect — adding a second pad would remove the Xbox pad's bond. Settle with `btmon` and
`bluetoothctl info` on a bonded, blocked pad.

**13. `robotctl pad pair` leaves `btd` stopped after Ctrl-C.** Verified.
On weird-ble boards `BtdPaused` stops `btd` (`robotctl/src/main.rs:4130`) and restarts it in
`Drop` (`:4209`), but the `pad` command installs no SIGINT handler, so `Drop` never runs and
`Restart=always` does not apply to an explicit stop. The phone path stays down until a reboot.
App- and NFC-initiated `pad.pair` never get the pause at all.

**14. Smaller.**
- A `ble-psk` that fails to load makes `btd` exit and crash-loop, with no PIN fallback — by design
  (§5.4), at the cost of the recovery transport (`btd/src/main.rs`).
- A failed data-partition mount takes `bluetooth.service`, and so BLE, down with it
  (`microduck_yocto/.../var-lib-bluetooth.mount`).
- `microduck-slot-health` checks only that an `hci*` exists, not that it is UP with a non-zero
  address, and only once per boot.
- The app's write chunk is `mtu - 3` with no 512 clamp (`link.rs:355`); Android's 517 MTU gives
  514-byte chunks. Latent while requests stay under 514 bytes. `framing::notification_payload` is
  the fix.
- In secure mode a full `system.logs` reply (`MAX_LOG_BYTES` 48 KiB) exceeds the 64 KiB frame once
  sealed and base64-encoded; `btd`'s secure session returns without a word, and the app waits
  15 s.
- Secure-mode handshake failures are bare `return`s with no log line (`btd/src/secure.rs`).
- `robots.json` is written with a plain `fs::write`; a torn write forgets every robot
  (`remembered.rs`).
- `nfcd`'s `PAIR_ANSWER` is 40 s (`nfc/src/main.rs:81`), shorter than `configd`'s worst case of
  about 105 s, so a pad can bond after `nfcd` has given up.
- `configd` leaks its agent object if `AgentManagerProxy::new` fails after `object_server().at()`;
  the next pairing keeps the old, wrongly scoped agent until `configd` restarts.
- `padd`'s `request()` has no read timeout; a wedged `robotd` freezes it, pad-loss sit included.

### Not measured, and possibly the largest

- **Whether the beta's BD address is stable across boots.** The attach script checks only for
  non-zero. §8.6 already records an address drift on another board; if it moves, every bond is
  orphaned on reboot.
- **Whether `rfkill unblock all` on beta can race `wq_wlan`'s firmware download** at boot. It works
  today by timing.
- **A soak.** hci0 over hours of uptime, a UART error, a controller hardware error, a bluetoothd
  crash on the Yocto image — none has been run.

## Tests

Tested, host-only: `btd`'s session, routing, upstream, PIN and secure handshake over real unix
sockets with fake daemons; `duck-ble` framing and adverts; `configd`'s pad heuristics and pure
helpers; the app's session over blec's mock backend (auth, drop then re-auth, wrong PIN, concurrent
opens, the version floor).

Untested: all of `btd/src/bluez.rs` (session slot, notify pump, reset, peer watch, reconcile,
advert fallback), `btd/src/agent.rs`, the chorale's task lifecycle, every D-Bus path in `configd`,
`padd`'s main loop and pad loss, the Android bond plugin, the app's frontend error classification,
and anything on a real radio. `btd`'s `Link` seam would let the slot and pump logic be tested
without one, if they were factored out of `bluez.rs`.

## Where the docs disagree with the code

- [`app-path-design.md`](../design/app-path-design.md) §5.5 (after its first paragraph) and §8.1
  say `--require-pairing` is off by default and `--insecure-no-pairing` is ignored. Pairing is on
  by default and `--insecure-no-pairing` is live (`btd/src/main.rs`).
- §5.4 calls secure mode opt-in; `ImportCredential=ble-psk` turns it on for every robot with a
  setup code. The header of `btd/src/secure.rs` says the same stale thing.
- §3.2 and §3.7 do not mention that a session end re-registers the whole stack, nor
  `watch_session_peer`.
- `configd/src/bluez.rs:75` says `setup-board.sh` sets `Privacy=device`; it does so only with
  `--weird-ble`, and the Yocto image sets nothing.
- [`pair-a-gamepad.md`](../robot/pair-a-gamepad.md) says `robotd` zeroes the velocity past 500 ms
  (lines 295, 327) and, between them, that the deadman never fires while the radio stalls.
- The app refuses robots below API v49, against "skew is served, never refused";
  [`mobile-app.md`](../design/mobile-app.md) §6 still gives the floor as v31.
