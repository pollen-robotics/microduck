# Roadmap

Status: draft · Date: 2026-07-28 · Owner: pierre

Companion to [`architecture.md`](architecture.md) (what we're building) and
[`updater-design.md`](updater-design.md) (how it ships). This is *order and sequencing*
— it will change; the design docs shouldn't.

## Where we are

| | |
|---|---|
| `updater/` | engine, verification, store, journal, hooks, preflight, GitHub/HF/local sources, IPC server, systemd unit — **done** |
| `duck-control/` | robot model · bus · IMU · `RobotIo` · observations · ONNX policy · safety — **slices 1–2 done**, untested on a board. A library: no tokio, no sockets, no systemd |
| `duck-ipc-proto/` | wire contract for `update.*` and `robot.*` — **done**; serde/serde_json/semver only, so nothing on the recovery path pulls the engine's tree |
| `robotd/` | a real 50 Hz loop driving walk/stand through the safety layer, intents, health from deadline adherence and policy state — **slices 1–2 done**, untested on a board; no kinematics |
| `padd/` | gamepad → intents, as an ordinary socket client — **done**, ships in the release; needs libudev, installed by CI and the board cross-build |
| `robotctl/` | CLI over the update socket — **done** for the `update` namespace; depends on `duck-ipc-proto`, not `updater` |
| `xtask/` | package · sign · promote — **done**, byte-identical promotion verified |
| `.github/` | ci · release · promote — **ci passing**; release/promote still unrun (needs secrets + the `release` environment) |
| bootstrap | `updaterd install` + `scripts/install.sh` — a robot installs its first release through the **ordinary engine**, so there is no bootstrap-only code path to drift |
| `deploy/` | shipped `updater.toml`, `robotd.toml`, trust anchor, journald retention drop-in |
| `scripts/` | `install.sh` provisioning · `board-test.sh` — **passing in CI**: 13 checks on emulated aarch64, Debian 13 (Trixie) |
| tests | **350 passing**, including the health gate, the battery+thermal readout and the policy/safety path against a real `robotd` process |
| missing | `mediad`, `btd`, `robot-config`, app, SDK |
| never run on hardware | every claim above is from CI and a laptop. Slice 1's whole purpose is to change that |

## The framing

**The hard part is productisation, not capability.** `microduck_runtime` already walks,
runs gait policies, does perception and mapping. What doesn't exist is a robot you can
hand to a stranger: app-driven updates, safety authority, privacy, provisioning,
recovery. Porting existing capability into the new architecture is laborious but
*known-feasible*; the unknowns are all on the productisation side.

**The updater is finished and instrumentally useless.** It has nothing real to ship. So
the first milestone is whatever gives it cargo — and, now that a team is arriving, gives
*them* a way to share work.

## What changed the order: the team arrives in ~2 weeks

Others will work on `robotd`/`mediad` and **share builds through the updater**. That
makes two things urgent that would otherwise have waited:

1. **Dev-channel installs** — install a specific branch or commit on a board, without
   cutting a release. This is now ahead of `btd`: teammates will use `robotctl`, not the
   phone app.
2. ~~**A repo and a dev signing key**~~ — **done.** `pollen-robotics/microduck_daemon`
   (private), CI green on first fix; `team.dev` key generated. Still outstanding before a
   real release: the signing secrets and the `release` environment gate (`ci-setup.md`).

`btd` and the app path slip behind both. They matter for *customers*, not for the team.

## Milestones

Each has a test that says "done", because milestones without one drift.

### M1 — Close the loop  ·  **done**

The updater got something real to gate against, and the team got a shared crate boundary.

- **`robotd` skeleton** — heartbeat plus the four `robot.*` methods `updaterd` calls. Its
  state is atomics, not a mutex: a robot whose control loop is wedged must still be able to
  answer "I am not healthy", and needing the loop's lock to answer would hang in exactly the
  case that matters. `--unhealthy` / `--busy` exercise rollback on a bench robot.
- **`duck-ipc-proto` extracted** — `robotd` and `robotctl` depend on it and not on `updater`,
  so nothing on the recovery path links the engine's http/tar/crypto tree.
- **The health gate is real** — `on_apply` restarts `robotd`, `health` is a socket probe, and
  a test fails if either regresses to its inert bootstrap value.
- **One source of truth for the robotd socket** — `robot_socket` at the top level of the
  config; `--robot-socket` is a documented dev override.
- **Logging and version reporting** — every daemon's first line is its own identity (version,
  revision, exe path) at `warn`, so it survives `RUST_LOG=warn`; `robotctl version` reports
  running *and* installed per service, because `updaterd` never restarts itself and so
  legitimately lags until reboot.
- **First-install bootstrap** — `updaterd install` + `scripts/install.sh`, through the
  ordinary engine.

**Done:** `robotd/tests/updater_gate.rs` gates an update against a real `robotd` process over
a real socket and commits; `robotd --unhealthy` reverts the content behind `current`.

`robot-config` was dropped from this milestone: M1's test is about the health gate, and a
heartbeat daemon needs CLI flags, not a shared config store. It lands when something reads it.

### M2 — Dev channel  ·  **done**

Install a branch on a board without cutting a release:

```
sudo robotctl update apply daemon --ref my-branch
```

- **`Target::Ref`** and `manifest_at_ref` on the source trait. `--ref` conflicts with
  `--version` rather than one silently winning.
- **`dev.yml`** — every branch push publishes `<crate>-dev.<run>.<sha7>` to the moving tag
  `daemon-dev-<branch>`, signed with `team.dev`.
- **`xtask package` accepts a prerelease of the crate version** without
  `--allow-version-drift`, so the escape hatch stays reserved for what it was built for.
- **Refs work on `local_dir`** too, which makes the path testable offline and is the sideload
  story. A ref becomes a filename there, so separators and `..` are refused.

Two properties make this safe on every push, both enforced away from the workflow:

- A dev build **cannot become `latest`** — the version is a semver prerelease, and
  `version_from_tag` refuses to read a dev tag as a release version.
- A dev build **cannot install on a customer robot** — `allow_dev_keys` is false there, and a
  trusted key only counts as a dev key if its filename ends `.dev.pub`.

A ref bypasses the downgrade guard by design: a prerelease always sorts below the release a
board is on, so guarding it would refuse every branch install. A plain `apply` returns the
board to the release stream, since `latest` resolves to the highest *stable* version.

**Done:** verified against the real repository — `dev.yml` published, `--ref main` installed
over the network, and a customer-robot config refused the same build.

**Open, and it blocks M4:** a private repo's release assets need a token, and a customer robot
has none. See `updater-design.md` §6.1.

### M3 — `robotd` for real  ·  hardware first, in two slices

Designed in [`robotd-design.md`](robotd-design.md). `robotd` **replaces**
`microduck_runtime`, by extracting its control core into `duck-control` rather than
reimplementing it — so the prototype keeps running while the daemon grows, and parity
arrives as a consequence of the extraction instead of as a race against a moving target.
Only the alpha variant on the Radxa survives; the other three variants, four IMUs and two
boards are dropped.

**Hardware first, sim after.** An earlier draft of this milestone said the reverse. It was
wrong on the facts: there are boards, and correctness gets settled on them. The simulator's
job is a clean laptop dev environment, not a validation oracle, so it lands after slice 2
and never becomes a second definition of what the robot is. Tests run against a `FakeIo`
backend — no hardware, no network, no Docker, no Python.

**Slice 1 — hold the pose · done, pending a board.** A real 50 Hz loop on the Dynamixel
bus, holding whatever pose it starts in. No policy. It exists so `robot.health` means *the
loop is meeting its deadline* rather than *it ticked once* — until now the updater's
auto-rollback has been gating on a placeholder. Holding a pose is also what makes it safe to
hammer install/rollback/power-cut cycles at a bench for a day.

**Slice 2 — walk and stand.** One 61-D observation builder (every alpha policy is
`obs[1,61] → actions[1,14]`), the main-plus-standing policy shaped as it is in the runtime,
`move`/`head`/`stop`/`enable` intents, and a gamepad client that goes through them.

**Safety authority belongs here, not in M6** — `architecture.md` §6 designs it and nothing
implements it. It lands in slice 2, holding the only write handle to the bus, so no policy
and no client *can* command a motor around it. Joint clamp, fall → limp, and an intent
deadman; thermal waits for a measured threshold rather than a guessed one.

**Done when:** it walks on a board driven through the intent API, an update applied with
`robotctl` restarts it cleanly with the gate passing, and a release built to come up
unhealthy is automatically rolled back.

### M4 — Hardware bring-up

M3 on the Radxa with real motors and IMU. This is where the genuinely unknown numbers
appear: control-loop jitter on a non-RT kernel, ONNX inference rate on Cortex-A55, eMMC
write timing, thermals, battery. Also the first real test of `systemctl restart` in
`on_apply`, and of the health-gate timeouts — 30s is currently a guess.

Also the first chance to settle the **log retention** question, which cannot be answered
off-board (`deploy/README.md`):

- `findmnt /var/log` — if Armbian's RAM-log has it on tmpfs, journald's `Storage=persistent`
  is a directory in memory: it survives a clean `reboot` and loses recent logs on a power
  cut, which is how a robot is actually switched off. Decide explicitly: disable the RAM log
  and accept eMMC writes, or keep it and rely on the update history (which does not go
  through `/var/log`).
- `journalctl --list-boots` after a real reboot — two or more entries, or the drop-in is not
  doing what it claims.

**Done when:** it walks on hardware, an update applied via `robotctl` restarts `robotd`
cleanly with the gate passing, and `journalctl -u robotd -b -1` returns the previous boot's
logs after a power cut.

### M5 — `mediad`, WebRTC, SDK

Camera/mic, encode, perception, the remote gateway. Privacy lands here and not later:
per-session consent and a visible streaming indicator are cheap now and expensive to
bolt on. The SDK's WebSocket + snapshot path (§5.3) is what makes "an LLM drives the
robot" easy.

**Done when:** telepresence works from outside the LAN, and a server-side script can
fetch a frame and send an intent in a few dozen lines.

### M6 — Ship readiness

`btd` + the app update path, provisioning (device identity, calibration, key
installation), recovery mode (§8.2's last link), manifest staleness reporting (§8.4.2),
and the authority arbitration finished.

**Done when:** a non-developer updates the robot from the phone, and a deliberately
bricked release recovers without a laptop.

## Organisation

**One repo, one workspace.** `robotd`, `mediad`, `btd` join as siblings. They co-version
because they all ship in the same `daemon` artifact — one version line is correct, and
models version separately already.

**Crate layout as it should end up:**

```
duck-ipc-proto/ wire types — serde/serde_json/semver only; btd/robotd/robotctl depend
                on this, never on updater
robot-config/   config store: file + flock + inotify        (not built yet)
updater/        engine + updaterd
robotctl/       CLI
robotd/         control, kinematics, gait, safety           (skeleton)
mediad/         camera, encode, perception, WebRTC gateway  (not built yet)
btd/            BLE transport adapter                       (not built yet)
xtask/          build/publish tooling — never ships
```

**Docs per concern, not per service.** `architecture.md` is the cross-cutting contract;
a service gets its own design doc only when it earns one (`updater-design.md` is the
model). Resist one giant document.

**Two channels of work for newcomers.** A teammate on `robotd` should be able to: clone,
`cargo test`, run against sim, push a branch, and install it on a board via `--ref`.
That's the whole onboarding path, and M1+M2 are exactly what make it true.

## Decisions that shape work rather than follow it

1. ~~**Signing key custody**~~ — **done.** Three encrypted release keys plus an
   unencrypted dev key in `~/.duck-keys`, all round-trip verified. Releases are signed in
   CI behind an approval gate; only `release-1` goes into secrets. See
   [`ci-setup.md`](ci-setup.md).
2. **Safety authority** (§6) — pulled into M3 for the reason above.
3. **Provisioning** — deciding nothing now is fine, but §5.7's per-device state
   (calibration, identity) needs a home before the first robot ships, and it constrains
   M6.
4. **Privacy** — consent + indicator in M5, not M6.

## Not doing, on purpose

Recorded so they stay decided: A/B image updates, OS/kernel OTA, fleet
dashboards/telemetry, delta updates, staged rollouts, hardware capability matrix,
competing model alternatives per slot (§17), peripheral firmware OTA (§11.1).
