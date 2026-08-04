# Robot Daemon Update System — Design

Status: draft · Date: 2026-07-22 · Owner: pierre

## 1. Context

The robot runs a Rust daemon on a Raspberry-Pi-class board (Raspberry Pi OS /
Debian). The daemon handles motor control, kinematics, a small ML gait model,
and streams video/audio over HTTP/WebRTC. A Bluetooth (BLE) service exposes
configuration and provisioning. Robots ship to non-developer clients; updates
must be triggerable from a companion mobile app with a few taps.

We need a way to update the shipped software safely in the field.

Sequencing and milestones live in [`roadmap.md`](roadmap.md).

The surrounding system — service split, IPC, state ownership, the robot API and
remote/WebRTC access — is covered in [`architecture.md`](architecture.md). This
document is the update system only.

## 2. Goals / Non-goals

**Goals**
- Update the **daemon binary** and the **ML gait model + config** *independently*
  (they will effectively never ship together).
- Support **several models** (walk, jump, stand, …) that version and update
  **independently** and are all loaded at once, each gated by daemon compatibility
  (§5.5).
- Trigger + monitor updates from the mobile app.
- **Force-upgrade off known-bad releases** via a minimum-version floor.
- Never brick a robot: atomic apply, automatic rollback, signed artifacts.
- Keep it simple; no fleet backend.
- Be **reusable across the team's other robots** with minimal adaptation.
- Support a per-component **post-install hook** for migrations / one-off steps.

**Non-goals (for now)**
- OTA of the OS / kernel / drivers (handled out-of-band, see §11).
- Fleet management: staged rollouts, per-device telemetry, dashboards.
  Model is "latest version, all robots."
- Delta/diff updates. Full artifacts are small enough at our cadence.

## 3. Key decisions & rationale

| Decision | Choice | Why |
|---|---|---|
| Update granularity | Application-level (not A/B image) | Only daemon + model change; OS is static. A/B (RAUC/Mender) would be over-engineering. |
| Transport | Robot **pulls** from CDN over its own wifi | Robot has internet; phone is trigger + progress only. Big payloads never touch BLE. |
| Hosting | **GitHub Releases** (daemon) + **HF Hub** (model) | Zero backend; each is a natural home for its artifact. Engine treats "source" as pluggable. |
| Signing | **minisign** (CI signs, robot verifies) | Battle-tested format, tiny verify crate, simple key management. |
| Channels | One per independently-versioned thing: `daemon`, plus one per model | Different cadences; models reload without a full restart. |
| Rollback | Versioned dirs + atomic symlink swap + health gate | Simple, robust, no partitioning needed. |
| Reuse | Config-driven generic engine | Same engine everywhere; each robot declares its components. |

Cadence expectation *for client-facing releases*: a daemon release every ~2–3
weeks for the first months, then every ~2–3 months. The model updates on its own
separate rhythm.

But **internal** iteration runs far hotter (the prototype saw ~8 tags in 3 days),
so the system must serve two rhythms: a fast internal `staging` stream and a slow,
curated `stable` stream (§16.3). That's what makes the staging channel and
scripted rollback load-bearing rather than optional polish.

### 3.1 Prerequisite: the robot needs its own internet access

The transport decision above has a setup consequence worth stating outright:
**an update requires the robot to have working internet on its own network
interface.** Wifi provisioning is therefore a prerequisite of updating, not an
independent feature — the bootstrap order is *pair over BLE → provision wifi →
update* — and a robot that has never been provisioned cannot be updated at all.

The phone's own connectivity is not a substitute. Realistic sustained BLE
application throughput is on the order of 10–30 kB/s once connection intervals
and iOS's pacing are accounted for, which puts a daemon artifact in the tens of
minutes and a model bundle in the hours. That is why payloads never traverse BLE
(§13): the limit is the link, not the protocol, so no amount of framing work
recovers it.

What still works with no network at all: `rollback` and `reset-to-golden` (§8.2)
operate purely on releases already on disk. They need no manifest and no source,
and move no bytes, so they fit inside BLE trivially and `btd` serves them even
with `robotd` dead (§4.1). **An unprovisioned or offline robot is therefore
recoverable, just not upgradeable** — the right side of that trade to fail on.

Delivering an artifact over the phone instead is possible as a fallback, but it
is not the normal path and still needs a real link (wifi or USB, never BLE).
See §17.

## 4. Architecture

`systemd` is the supervisor (lifecycle, restart-on-crash, ordering, watchdog).

```
systemd
├── robotd     motor control · kinematics · gait ML · sensor loop   [RT-ish core]
├── mediad     video/audio capture · WebRTC/HTTP                     [isolated: media crash ≠ dead motors]
├── btd        BLE GATT server (bluer): wifi provisioning, naming, update trigger/progress
└── updaterd   the update engine (this document)
                    ↕ local IPC (unix socket) with robotd + btd
```

**Structural rule:** the updater is a *separate unit* from what it updates. A
process cannot cleanly replace its own running binary, and the updater must
survive a daemon crash to perform rollback.

**Corollary — `updaterd` must be resident, and must exclude itself from the
restart set.** `updaterd` and `btd` both ship *inside* the daemon artifact, so a
naive "restart everything" would kill the executor mid-swap or mid-health-gate.
`on_apply` therefore restarts `robotd` and `mediad` only; `updaterd` and `btd`
pick up the new binary at the next boot or an explicit later restart.

This is also why the update logic cannot live in `btd`: as a client of the
update, `btd` cannot be the thing performing it — it would kill itself partway
through. `btd` stays a thin transport, and the resident `updaterd` owns the state
machine, the single-flight lock, boot recovery, and any periodic/mandatory check
(§8.1) where no client is present at all.

`btd` is a thin relay: it forwards "check" / "apply" to `updaterd` and streams
progress back to the phone. It never moves the payload itself.

### 4.1 Invariant: `btd` and `updaterd` survive a dead `robotd`

**`btd` and `updaterd` must start, run, and remain fully usable even if `robotd`
is crashed, crash-looping, or absent.** They are the recovery path — if they can
only work when the robot is healthy, then the one situation where a client
actually needs them is the situation where they don't work. This is the single
most important structural property for not bricking a robot in the field.

Concretely:
- **No systemd dependency on `robotd`.** No `Requires=`/`After=robotd`, no shared
  fate. `btd` and `updaterd` come up on boot independently and stay up.
- **No hard IPC dependency.** Every call into `robotd` (the "safe to restart?"
  check, health probes, model reload) is *optional and timeout-bounded*. A dead
  socket is a normal, expected answer — never a hang, never a panic, never a
  refusal to serve the phone.
- **Degraded-mode semantics.** With `robotd` down, the safe-to-restart check
  trivially passes (nothing is moving) and `updaterd` proceeds. Health gating
  falls back to "does the new `robotd` come up and report healthy" — which is
  exactly the check that matters when recovering from a bad release.
- **`btd` reports the truth.** The app must be able to see "daemon unhealthy,
  version X, last update failed" and offer *update* / *rollback* / *reset to
  golden* (§8.2) from that state. A robot that can't walk must still be able to
  explain itself and accept a fix.
- **Minimal dependency surface.** These two are the last line of defence, so keep
  them boring: few dependencies, no ML runtime, no media stack, no GPU/camera
  access. A bug in the interesting code must not be able to take down the
  recovery path.
- **Separately updatable, conservatively.** `btd`/`updaterd` ship in the daemon
  artifact, so a bad release could in principle break them too — which is what
  the golden release (§8.2) and boot-counter revert (§8) exist to catch. Treat
  changes to these two as higher-risk than changes to `robotd`.

Corollary for `updaterd`: it should never *require* `btd` either. A local
`robotctl` (§15) over the unix socket must be able to drive recovery even if BLE
itself is broken.

## 5. Update model

### 5.1 Channels

Two logical channels, each with its own manifest and version line:

- `daemon` — the `robotd` / `mediad` / `btd` binaries + supporting files.
- `model`  — gait model weights + behavior/config bundle.

The two channels live on **different hosts**, which the engine handles via a
pluggable `source` per component:
- `daemon` → **GitHub Releases**, tag `daemon-v1.4.2`, assets = artifact +
  `.minisig` + manifest.
- `model` → **HF Hub** repo, versioned by git revision/tag; files resolved at
  `https://huggingface.co/ORG/MODEL/resolve/<rev>/<file>`. "Latest" = a moving
  tag/branch or the newest tag. We still `minisign` the model artifact ourselves
  and store the `.minisig` alongside — HF does not sign for us.

### 5.2 Artifact

One compressed tarball per release, e.g. `daemon-1.4.2.tar.zst`, containing:

```
bin/robotd  bin/mediad  bin/btd        # (daemon channel)
model/gait.onnx  config/…              # (model channel)
version.toml                           # semver, min_hw_rev, schema_version
hooks/postinstall                      # optional, executable, versioned+signed
```

### 5.3 Manifest

A small JSON published as a release asset (and referenced by a stable URL, see
§6). Signed with minisign.

```json
{
  "channel": "daemon",
  "version": "1.4.2",
  "url": "https://github.com/ORG/REPO/releases/download/daemon-v1.4.2/daemon-1.4.2.tar.zst",
  "sha256": "…",
  "sig_url": "https://github.com/ORG/REPO/releases/download/daemon-v1.4.2/daemon-1.4.2.tar.zst.minisig",
  "min_hw_rev": 3,
  "schema_version": 2,
  "min_supported": "1.5.1",
  "changelog": "…"
}
```

`schema_version` is **migration context, not a compatibility gate**. Gating on it
would be self-defeating: the engine judging a manifest is always the *previous*
release's engine, so refusing `schema_version > supported` would make every schema
bump undeliverable — including the release carrying the engine that understands it.
The post-install hook performs the migration; the engine passes the numbers through
(§9).

`min_supported` is the **minimum-version floor** (§8.1). Included in the schema
from v1 even if unused at first — it cannot be retrofitted onto robots that
never learned to look for it.

There is deliberately **no** `not_valid_after` field yet: manifest expiry is the
defence against freeze attacks, and the decision to take on its operational cost
is deferred in §8.4.2. Adding it later is additive (absent = no expiry), unlike
`min_supported`, so it does not need reserving now. `model` manifests additionally carry
`model_api` (§5.5).

### 5.4 Signing / trust

- CI signs both the **artifact** and the **manifest** with the private minisign
  key (kept in CI secrets, offline master ideally).
- The robot ships with a **set of trusted minisign public keys**
  (`/etc/robot/trusted_keys/`), not a single key. A signature is valid if it
  verifies against *any* trusted key. This is the trust anchor — no PKI needed.
- **Key rotation:** shipping a *set* from day one means a lost/compromised key
  is survivable — publish an update signed by an existing key that adds the new
  key and (later) retires the old one. A single baked-in key would make a lost
  key an unrecoverable dead update path.
- Optionally reserve a distinct **dev key** in the set, gated behind a flag, for
  the team's local sideload path (§15).
- Verification order on the robot: verify manifest signature → download artifact
  → verify sha256 → verify artifact signature. **No unsigned bytes are ever
  executed or extracted to a live path.**

#### Key custody

Generate keys with `cargo xtask keygen`, which refuses to write into the repository and
writes secret keys `0600`.

| key | encrypted | private half lives | in trusted set of |
|---|---|---|---|
| `release-1.pub/.key` | yes | password manager **and** CI secrets | every robot |
| `release-2.pub/.key` | yes, **different passphrase** | password manager only — **never CI** | every robot |
| `release-3.pub/.key` | yes, **different passphrase** | offline; ideally never on a networked machine | every robot |
| `team.dev.pub/.key` | no | team store + CI | developer boards only |

Three things this table encodes, each easy to get wrong:

1. **The spare must ship from the first image.** A robot verifies against the key set
   baked into it, so a replacement cannot be introduced over the air later — a lost sole
   key means re-flashing every robot by hand.
2. **The spare must differ in passphrase *and* exposure.** Its purpose is surviving a
   compromise of the first; `release.key` is in CI by necessity, so `release-2.key` must
   not be. Sharing a passphrase collapses both into one.
3. **Passphrases are generated, not chosen.** They are never typed — they live in a
   password manager and a secret store — so memorability buys nothing, while minisign's
   scrypt parameters (32 MiB, opslimit 2²⁰) are only a modest brake on an offline attack
   against a leaked key file. ≥128 bits, e.g. `openssl rand -base64 24`.

Releases are signed **in CI**, not locally — a deliberate choice, with the approval gate
that compensates for it documented in [`ci-setup.md`](ci-setup.md).

The dev key is deliberately unencrypted: CI signs non-interactively, and a passphrase
stored beside the key it protects adds little. Its real protection is structural — it is
absent from customer robots' trusted sets, and gated there by `allow_dev_keys = false`.

### 5.5 Models: one component each

There are several models — walk, jump, stand, ground-pick — **each versioned
independently**, and all loaded at once rather than one selected from many.

That is exactly what a *component* is: a thing with its own version line. So each
model is its own entry in `updater.toml`, and no special store layout is needed:

```toml
[component.model-walk]
install_dir = "/opt/robot/model/walk"
source      = { type = "hf_hub", repo = "ORG/gait-walk", revision = "main" }
on_apply    = { action = "reload", unit = "robotd", signal = "SIGHUP" }

[component.model-jump]
install_dir = "/opt/robot/model/jump"
source      = { type = "hf_hub", repo = "ORG/gait-jump", revision = "main" }
on_apply    = { action = "reload", unit = "robotd", signal = "SIGHUP" }
```

Each then gets, for free and independently: its own rollback target, golden release,
pin, boot-counter trial, and known-bad history. `robotctl update apply model-walk`
updates one model without touching the others. Trying an older version of one is
`robotctl update select model-walk 1.1.0` — `select` repoints at any installed
release without downloading.

Models use `reload` (SIGHUP → re-`mmap` weights) rather than `restart`, so a weights
swap never drops motor control. Per-component `on_apply` is what makes that natural,
and a core reason models and the daemon stay separate components.

**A "library layout" was tried and removed.** An earlier draft had a `Layout::Library`
mode (`library/<ver>/` + an `active` symlink) meant to hold several named bundles with
one selected. It could not actually do that — it varied only two directory names, so
it was identical in capability to the normal layout while *looking* like a feature.
Worse, "many installed, one active" is not the shape the models have: they are all
active simultaneously. Deleted.

If a *single* slot ever needs competing alternatives (two different walk models, user
picks one), that is a genuine `(name, version)` store key and a real change — see §17.
Nothing in the current design pretends to support it.

**Compatibility (kept light).** The daemon advertises a **model API version**
(`model_api`, e.g. `1`) describing the sensor-input / actuator-output contract it
implements. Each model manifest declares the `model_api` it requires. Rule: a model
loads only if `model.model_api <= daemon.model_api`.

The API version bumps when the interface changes — e.g. a model starts consuming a
**new sensor input** or emitting a **new output**. This means:
- A model needing API `2` is refused on a daemon that only speaks `1` → the app
  prompts to update the daemon first.
- Old models keep working on newer daemons (backward compatible), so nothing breaks on
  a daemon upgrade.

An unreachable `robotd` makes this *unknown* rather than incompatible, which is a
reason to wait for a model but not for the daemon — see §5.4's `Compatibility` note.

### 5.6 Hardware: single target

**Current target: Radxa Zero 3** (RK3566, Cortex-A55 → `aarch64`) running **Armbian
26.2.x** with the **Debian 13 (Trixie)** userland, Rockchip BSP kernel 6.1.115.
Provisional — the closest board to hand rather than a final choice — but it is what the
build and test tooling aims at.

Build and verification: `scripts/board-test.sh` cross-compiles with `cargo-zigbuild`
(`zig cc` supplies the cross sysroot and the C compiler `zstd` needs) and runs the
binaries on real ARM64 Linux in containers.

The glibc floor is pinned at **2.31** (yielding an actual floor of 2.30) rather than
Trixie's 2.41, and the script runs the binaries on **Trixie only** — the userland we ship.
Armbian offers others for this board; testing configurations nobody runs costs job time to
defend a claim we do not need, and `BOARD_IMAGES=` overrides it for a one-off check.

The pin is not about those alternatives. It defends against the *build host*: unpinned, the
build links against whatever glibc the developer's machine or the CI runner has, which is
newer than the board's — it links cleanly and then fails to load on the robot, with nothing
in the build output pointing at the cause. 2.31 is simply well below the target, which
costs nothing.

The kernel version is not a constraint: `flock`, `SO_PEERCRED` and `statvfs` all long
predate 6.1.

What the container cannot cover, and hardware must: a real `systemctl restart`
(`on_apply` has never run against systemd), eMMC write behaviour and timing, `statvfs`
against the real filesystem, and whether the health-gate timeouts suit a robot that
takes tens of seconds to stand up.

**v1 targets one well-specified hardware configuration.** No board / variant /
IMU / camera matrix, no per-device hardware profile, no artifact selection logic.
The prototype's hardware variance is an artifact of exploration, not a
requirement to carry forward.

What we keep is deliberately minimal:
- `min_hw_rev` in the manifest (§5.3) as a single forward-compatibility guard, so
  a future board revision can refuse an incompatible artifact. One integer, no
  matrix.
- A **stable device ID** for the update log (§8.3) and any future phone-home. The
  SoC serial (`/proc/device-tree/serial-number`) works and survives reflashes —
  no provisioning step needed to obtain it.

If a second hardware target ever appears, the manifest gains constraints and the
engine gains a match step. Not before — this is exactly the speculative
complexity to avoid.

### 5.7 Robot-specific state must survive updates

The requirement that *does* carry over: some files belong to **this** robot, and
an update or rollback must never destroy them. Three classes, treated
differently:

| Class | Examples | Lifecycle |
|---|---|---|
| **Shipped** (replaced) | binaries, policy bundles, default config, static systemd units | Under `releases/<ver>/`; swapped atomically |
| **Robot-specific** (never touched) | calibration data, per-device generated assets, learned/persisted state (maps, personality, habits) | Outside release dirs; **preserved across update *and* rollback** |
| **User preferences** (preserved, migrated) | robot name, wifi credentials, active model selection, app-set tunables | Own config file; migrated by hooks (§9) on `schema_version` bumps |

Two rules:
1. Release dirs are **disposable**. Anything that must survive lives in
   `/etc/robot/` or `/var/lib/robot/`. A rollback must not lose user data or
   force the client to recalibrate.
2. **Runtime config lives in a config file, not in the systemd unit.** Units are
   shipped and static; they read a config file the updater never overwrites.
   (The prototype bakes CLI flags into a generated unit at install time, so
   reinstalling silently resets settings — a trap worth naming.)

Corollary: regenerating per-device assets must be keyed off a stable seed and
done only when genuinely needed, never unconditionally on every update.

## 6. Hosting on GitHub Releases

- Artifact + `.minisig` + `manifest.json` are assets on each release.
- "What's the latest?" resolves via one of:
  - **Stable redirect URL** `…/releases/latest/download/<asset>` — but GitHub's
    "latest" is repo-wide, so this only works cleanly with **one channel per repo**.
  - **GitHub API** query for the newest tag matching a channel prefix
    (`daemon-v*`) — works with a single shared repo. Preferred if we keep both
    channels in one repo.
- CI side: `cargo-dist` can build and publish signed artifacts to GitHub
  Releases; we host manifests as additional assets.

### 6.1 ⚠ A private repository cannot serve the fleet

**Unresolved, and it constrains M4.** `pollen-robotics/microduck_daemon` is private, and a
private repo's `releases/download/<tag>/<asset>` URL returns **404 — with or without a
token**. Verified directly:

| URL | private repo |
|---|---|
| `https://github.com/<repo>/releases/download/<tag>/<asset>` | 404, authenticated or not |
| `https://api.github.com/repos/<repo>/releases/assets/<id>` + `Accept: application/octet-stream` | 200 with a token |

The engine now resolves every asset through the API endpoint, so a **developer's board**
works: `GITHUB_TOKEN` is in the environment and `--ref` installs a branch build. Verified end
to end against this repo.

**A customer robot has no token, and should not.** A fleet-wide credential baked into an
image is a credential that leaks and cannot be rotated without reflashing — the same problem
the signing keys are tiered to avoid. So as things stand, robots in the field cannot download
anything. The options, none of them chosen yet:

| option | keeps zero-backend | notes |
|---|---|---|
| A **public repo holding only release artifacts** | yes | signatures are what make an artifact safe, not obscurity — a public artifact repo leaks build metadata and nothing else. Source stays private. |
| Make this repo public | yes | product source; presumably not. |
| An object store or CDN with a plain HTTP source | mostly | one more thing to own and pay for; the engine's source trait already abstracts it. |
| A read-only token in the image | yes | rejected reasoning above: an unrotatable fleet credential. |

The first is the conventional answer and costs nothing but a second repository. It does not
change the engine — only `repo` in `updater.toml` and where `release.yml` publishes.

Nothing about this blocks M2 or M3: dev boards have tokens and sim needs no downloads. It
blocks the first robot that has to update itself without a developer present, which is M4.

## 7. `updaterd` state machine

```
receive trigger (from btd)  ── check | apply(version)
        │
        ▼
PREFLIGHT (§7.2):  single-flight lock · clock/NTP sane · robot stopped · disk space free
        │  (any fail → report + exit, no changes)
        ▼
fetch manifest ──► verify manifest signature ──► compare version, check min_hw_rev / model_api / pin / downgrade
        │                                                    │ (nothing to do / incompatible)
        ▼                                                    └─► report + exit
download artifact ──► verify sha256 ──► verify artifact signature
        │
        ▼
extract to releases/<ver>.tmp/  ──► [pre_install hook]
        │
        ▼
atomic symlink swap:  current → releases/<ver>        (rename(2), same fs)
        │
        ▼
[post_install hook]  ──► apply: restart (daemon) | reload+SIGHUP (model)
        │
        ▼
HEALTH GATE (poll robotd over unix socket, timeout per component)
        │
   ┌────┴─────────────────────────┐
 healthy                        not healthy / hook failed / timeout
   │                                │
 prune old (keep_previous)      ROLLBACK: swap symlink back, re-apply, mark failed
   │                                │
 report success                 report failure
```

Any non-zero hook exit, failed health probe, or timeout is treated identically:
**abort and roll back.**

### 7.1 On-disk layout

```
/opt/robot/daemon/
├── releases/1.4.1/     ← previous (kept for rollback)
├── releases/1.4.2/     ← new
└── current → releases/1.4.2     ← systemd units point at current/
```

Atomicity is a single `rename(2)` of the symlink on the same filesystem. No
half-written state is ever live.

### 7.2 Preflight preconditions

Checked before any download or change; any failure aborts cleanly with no
side effects:

- **Single-flight lock.** `updaterd` holds a lock file / socket so two triggers
  (or a retry) cannot collide. An update in progress reports "busy," never
  starts a second.
- **Phone-independence.** The robot pulls, so the update runs to completion even
  if BLE drops. Status is persisted and replayed to the phone on reconnect —
  the phone is never required to stay connected.
- **Clock sanity.** A Pi/eMMC board has no battery-backed RTC; a wrong clock
  fails HTTPS cert-date validation before any download. Require NTP sync (or a
  sane-clock check + bounded retry) as a precondition. minisign itself is
  time-independent.
- **Robot stopped.** We assume the app only offers updates while the robot is
  stopped/parked (motors safe). `updaterd` still does a light "safe to restart?"
  query to `robotd` and refuses if not — cheap insurance against restarting
  motor control mid-motion.
- **No active remote session.** Restarting `robotd`/`mediad` mid-telepresence is a
  bad surprise; refuse, or warn and require explicit confirmation. See
  [`architecture.md`](architecture.md) §5.
- **Disk space.** Storage is eMMC (finite, wear less of a concern than SD, but
  space still is). Verify free space for download + extract + `keep_previous`
  before starting.

## 8. Health gate & rollback

- After apply, the new `robotd` must clear a **healthy flag** within a
  per-component timeout (motors ack, model loads, media pipeline inits).
- Belt-and-suspenders for hard hangs (not just clean failures):
  - `systemd` `WatchdogSec` on `robotd` so a hang trips recovery.
  - A **boot-counter** file `updaterd` inspects on start: if the last update
    never reached "healthy" across 2 boots, revert unconditionally. Covers both
    "started but sick" and "won't start at all."
- `keep_previous` (default 1) retained release dirs bound disk usage while
  always leaving a known-good target to roll back to.

### 8.1 Minimum-version floor / kill switch

**Implemented**, including the part that makes it work: `check_interval` in config
makes the resident `updaterd` poll each source on a timer, and `auto_apply` decides what
that timer may install **without waiting for a client**. A scheduled pass is skipped
entirely while another update is in flight, and the first check is delayed after boot (the
network is often not up, and a fleet restarting together would arrive as a herd).

Leaving `check_interval` unset disables polling — and with it the floor, since a robot
nobody taps update on never learns a floor exists. `updaterd` warns at startup when
that is the case, and warns distinctly when `auto_apply` is set but has no timer to run it.

### 8.1.1 `auto_apply` — unattended update policy

| | |
|---|---|
| `off` | never; availability is logged, a mandatory release loudly |
| `mandatory` | **default** — only a release the floor marks mandatory. Ordinary ones wait for a client |
| `all` | every available release |

One ordered setting rather than a flag per urgency, so a config cannot express "apply
ordinary updates automatically but not mandatory ones" — auto-updating everything except
the releases published to rescue a broken fleet.

`mandatory` is the default because the alternative is a fleet stuck on a release we have
withdrawn. Ordinary releases wait because *when a robot restarts is its owner's decision*,
which is the decision the whole app-driven update flow exists to give them.

`all` is the canary and bench setting, and it is what makes §16.2's Tier 2 configurable:
lab robots that track `staging` and install each candidate. Shipping it to client robots
would take the restart decision away from them.

Whatever the policy, an unattended apply is an **ordinary** apply. It runs the same
preflight — so a robot that is walking or has a remote session refuses and retries at the
next interval rather than restarting under someone's hands — and the same health gate, so
a release that does not come up is rolled back. That is also why no maintenance window is
needed: `safeToRestart` is a better answer to "is now a bad time" than a clock is, and it
runs *before* any network access, so a busy robot costs nothing to skip.

**This belongs in the daemon, not in cron.** A systemd timer or crontab calling `robotctl
update apply` looks equivalent and is not: explicit applies deliberately bypass the
known-bad guard below, because an operator retrying a release may have fixed the cause.
An external timer would inherit that bypass and lose the protection — the wrong half of
each — and reintroduce exactly the loop the next paragraph exists to prevent.

**The unattended path refuses a candidate this robot already rolled back from**, whatever
the policy and even when the release is mandatory. Without that guard, a broken release is
a fleet-wide trap with no exit: check says available → apply → gate fails → roll back →
wait `check_interval` → repeat. Each cycle re-downloads the artifact, rewrites the eMMC and
restarts `robotd`, so every robot is both unusable and wearing out, on battery, with no
client involved to notice. Nothing in the loop converges.

The guard is `known_bad`, derived from the journal's *latest* outcome per version, so it
self-clears if the release ever does succeed. It applies **only** to the unattended path:
an explicit `robotctl update apply` still retries, because an operator may have fixed the
cause and refusing them would remove the obvious way to check. When the guard fires it
logs at `error` — a robot stuck below a mandatory floor needs a fixed release, and that
should be loud.

The test for this counts attempts in the journal rather than checking the live version: after
every cycle the symlink is back on the good release, so the robot's *state* looks correct even
while the loop runs.


The manifest's `min_supported` lets us force robots off a known-bad release
even under "latest-for-all" (where a robot otherwise only updates when tapped):

- On check/start, if the running version `< min_supported`, the update is
  **mandatory** — `updaterd` applies it without waiting for a tap (and the
  running bad version can be refused).
- A blunter **kill switch** (a signed "version X revoked" statement) is the same
  mechanism escalated. Being signed means an attacker cannot *forge* one — but see
  §8.4: a signature says an artifact is **ours**, not that it is **current**, so
  signing alone does not prevent replay of an old genuine manifest.
- Ship the field in the schema now (§5.3); it can't be retrofitted later.
- `min_supported` only takes effect once a robot successfully fetches a manifest
  carrying it. It is therefore a *remediation* tool, not a defence: a robot cut
  off from updates never learns the floor exists.

### 8.2 Golden release & recovery

- Keep one known-good **golden** release that is *never pruned* (separate from
  the `keep_previous` rotation).
- If both current and previous fail to come up healthy, fall back to golden and,
  failing that, a minimal **recovery mode** that can still reach the network and
  re-fetch. The robot must always be able to phone home rather than require an
  RMA / truck roll.

### 8.3 On-device update log

Persist the last N update attempts (timestamp, channel, from→to version,
outcome, error) to disk, retrievable over BLE/HTTP. This is the first thing
support needs when a client reports "update failed."

### 8.4 What signatures do and don't buy: downgrade and freeze

A minisign signature proves an artifact **came from us and wasn't modified**. It
says nothing about *when* it was published or whether it is still current. Two
attacks survive a perfectly valid signature, and they are the standard pair for
any signed-artifact scheme:

| | What it is | Status |
|---|---|---|
| **Downgrade / rollback** | Serve an *older, genuinely signed* manifest so robots walk backwards onto a version we withdrew | **Fixed** |
| **Freeze** | Serve the *current* manifest forever, so robots never learn a fix exists | **Open** (§8.4.2) |

Both are reachable by anyone who controls what the robot fetches: a stale or
reverted CDN/mirror, a cached proxy, DNS interception, or a hostile local network.
Neither requires a stolen key.

#### 8.4.1 Downgrade — closed

`updaterd` refuses a candidate older than what is installed when the target is
"whatever is latest":

- `Target::Latest` → refused with `WOULD_DOWNGRADE`. `check` reports it as
  incompatible rather than offering it as an available update.
- `Target::Exact` → **allowed**. That is how a targeted revert works, and it is a
  deliberate operator action which a mirror cannot induce.
- `rollback` and `reset-to-golden` move backwards by design and use already-installed
  releases, so they never consult a manifest at all.

The asymmetry is the point: the guard blocks what an *attacker* can cause while
leaving what an *operator* can choose.

#### 8.4.2 Freeze — open, needs a decision

Nothing currently distinguishes "there is no update" from "someone is preventing
me from seeing one". A robot pinned at a vulnerable version indefinitely is the
realistic harm — the same version it is already running, so no integrity check
fires.

Options, cheapest first:

1. **Signed manifest expiry.** Add `not_valid_after` (a timestamp) to the
   manifest; refuse a manifest older than that and surface "update metadata is
   stale" in the app. Cost: publishing becomes time-bound — CI must re-sign the
   `stable` manifest on a schedule even when no release changes, or every robot
   starts warning. Also depends on the robot's clock, which is exactly what
   §7.2's clock check exists to distrust.
2. **Monotonic manifest counter.** A `sequence` that only ever increases;
   robots record the highest seen and refuse anything lower. Detects *rollback* of
   metadata robustly without any clock dependency, but does **not** detect freeze
   (a replayed current manifest has the expected sequence).
3. **Report staleness rather than enforce it.** Record "last successful manifest
   fetch" and surface it in `status`/the app: "last checked 47 days ago". Detects
   nothing cryptographically, but makes a frozen robot *visible* to its owner and
   to support. No publishing burden, no clock dependency.
4. **Phone-home** (§17). A robot that reports in lets *us* see which robots have
   stopped checking — the only option that detects freeze centrally rather than
   per-robot. Requires backend.

**Recommendation: (3) now, (1) when there is a release cadence to hang it on.**
Staleness reporting is nearly free, has no failure mode of its own, and converts a
silent attack into a visible one. Expiry is the real defence but its operational
cost — a re-signing schedule that, if missed, warns the entire fleet — is not worth
paying before the publishing pipeline is routine. (2) is cheap but solves a problem
§8.4.1 already covers at the version level.

**Explicitly accepted for v1:** a robot whose network is hostile can be prevented
from updating. It cannot be made to *downgrade*, install an artifact we did not
sign, or install one that fails its health gate. Those are the properties we
actually rely on.

## 9. Post-install hooks

Same idea as dpkg `postinst`, but the hook ships **inside the (signed) tarball**,
so no unsigned code ever runs. Rust binary or shell script — the engine just
`exec`s it.

**Contract**
- Runs *after* the symlink swap, *before* `apply`. (A `pre_install` hook, if
  present, runs before the swap.)
- Non-zero exit ⇒ failed update ⇒ rollback.
- Environment provided (absent values are **omitted**, not set empty, so a hook can
  tell "first install" from "unknown"):
  - `UPDATE_COMPONENT` / `UPDATE_CHANNEL` — the component name; both are set, same value
  - `UPDATE_NEW_VERSION`, and `UPDATE_OLD_VERSION` when there is a previous release
  - `UPDATE_RELEASE_DIR` — the release being installed, `<install_dir>/releases/<ver>/`.
    **This is what a migration usually wants**, and it is also the hook's working
    directory.
  - `UPDATE_INSTALL_DIR` — the component *root*, e.g. `/opt/robot/daemon`
  - `UPDATE_NEW_SCHEMA_VERSION`, and `UPDATE_OLD_SCHEMA_VERSION` when known
- Hooks run with a **cleared environment** plus a fixed `PATH`, so behaviour doesn't
  depend on how systemd happened to invoke `updaterd`.
- Output is captured and truncated; on failure it is included in the error and
  therefore in the update log. On success it is currently **discarded** — see §17.
- Typical uses: config-schema migrations, `udev`/permission tweaks, data
  conversions across `schema_version` bumps.

## 10. Reusable, config-driven engine

The engine is identical across robots; each robot ships a config declaring its
components. Adapting to a new robot = new config + new signing key + (maybe) a
new health probe.

The authoritative, parse-tested example is
[`updater/updater.example.toml`](../updater/updater.example.toml) — a unit test
parses it, so it cannot drift from the code. Abridged here:

```toml
# /etc/robot/updater.toml
trusted_keys_dir = "/etc/robot/trusted_keys"   # a *set* of keys (§5.4)
hw_rev           = 1
state_dir        = "/var/lib/robot/updater"    # must be outside every install_dir

[component.daemon]
install_dir   = "/opt/robot/daemon"
keep_previous = 1
golden        = "1.0.0"

[component.daemon.source]
type       = "github_releases"
repo       = "ORG/robot-daemon"
tag_prefix = "daemon-v"

[component.daemon.on_apply]
action = "restart"
units  = ["robotd", "mediad"]                  # never updaterd or btd — see §4

[component.daemon.health]
probe   = "socket"
path    = "/run/robotd.sock"
timeout = "30s"

# One component per model — each versions independently (§5.5).
[component.model-walk]
install_dir   = "/opt/robot/model/walk"
keep_previous = 3

[component.model-walk.source]
type     = "hf_hub"
repo     = "ORG/gait-walk"
revision = "main"

[component.model-walk.on_apply]
action = "reload"                              # never drops motor control
unit   = "robotd"
signal = "SIGHUP"
```

Config is validated on load and `deny_unknown_fields` is on, so a typo fails
loudly rather than silently disabling the setting it was meant to change. Notably
refused: `state_dir` inside an `install_dir` (a swap would destroy the update log),
two components sharing an `install_dir`, relative paths, and `keep_previous = 0`
with no golden (no rollback target).


Note the model uses `reload` (SIGHUP → re-mmap weights) rather than `restart`,
so motor control is never dropped for a model swap. Per-component `on_apply` is
what makes this natural and is a core reason the two channels stay separate.

Shared across robots: engine, minisign pipeline, phone/BLE trigger protocol,
rollback logic. Per-robot: the config file, the signing key, health probes,
hooks. It's a small binary + a schema, not a framework.

## 11. OS updates (deliberately out of scope for OTA)

Skipping A/B has one real constraint: we do not OTA the kernel/system libs.
Mitigations:
- Enable `unattended-upgrades` scoped to the Debian **security** pocket only,
  for CVE patches. Set-and-forget.
- Anything bigger (kernel bump, new system lib the daemon needs) = **re-flash at
  service time**.
- Minimize runtime deps on the base OS: static-ish linking, bundle what we can
  inside the tarball, so a stock OS never blocks a daemon update.

### 11.1 Peripheral / MCU firmware — out of scope

Sub-board and servo firmware is flashed **at production**, not in the field. Not
an OTA component; no provision made for it. (Should that ever change, it would be
a component whose `on_apply` flashes over the motor bus — materially riskier,
since rollback means re-flashing rather than a symlink swap.)

## 12. Alternative considered: apt / dpkg (own signed repo)

Still worth keeping on the table, especially for the **daemon** channel on Debian:

**Pros**
- Signing (GPG), atomic install, versioning, and **maintainer scripts**
  (`preinst`/`postinst`/`prerm`/`postrm`) — the post-install hook we want — all
  for free and battle-tested.
- Standard Debian tooling; easy to reason about; `apt` handles dependencies.

**Cons for our case**
- No health-gated rollback and no clean "roll back to previous version"
  out of the box; we'd build that on top anyway.
- Awkward for shipping the ML model as a plain versioned asset.
- Doesn't carry the phone-trigger / progress IPC — still needs wrapping.
- **Portability:** won't exist on a future non-Debian robot (Yocto, etc.),
  which cuts against the "common tool across robots" goal.

**When apt would make sense:** if we decide the fleet is Debian-only for the
foreseeable future and we're willing to give up automatic health-gated rollback
(or drive rollback via pinning `apt install pkg=old`), an own signed apt repo is
a legitimately simpler path for the daemon binary. The model would still want a
side channel. Recommendation: start with the custom Rust engine for portability
+ rollback; revisit apt if the Rust engine's maintenance cost ever outweighs its
benefits and the fleet stays Debian-only.

## 13. Phone / BLE trigger protocol (sketch)

`btd` exposes two GATT characteristics per channel (or one with a channel field):
- **Trigger** (write): `{ "cmd": "check" | "apply", "channel": "daemon", "version"?: "…" }`
- **Status** (read/notify): `{ "channel", "state": idle|checking|downloading|applying|healthy|failed|rolled_back, "progress": 0-100, "version", "error"? }`

`btd` forwards to `updaterd` over the unix socket and mirrors its status back.
Payloads never traverse BLE.

## 14. Implementation sketch

`updaterd`, a small Rust binary (~a few hundred lines of logic):
- `reqwest` — download (with resume/retry) + GitHub API for latest-tag lookup.
- `minisign-verify` — signature verification.
- `tar` + `zstd` — extract.
- `serde` / `toml` — config; `serde_json` — manifests.
- Atomic symlink swap via `rename(2)`; health poll over the existing unix-socket IPC.
- systemd integration: `Type=notify`, `WatchdogSec`, unit templating.

CI (per channel):
1. Build artifact (optionally via `cargo-dist`).
2. `minisign -S` the artifact and the manifest.
3. Create a GitHub Release tagged `daemon-vX` / `model-vX` with the artifact,
   `.minisig`, and `manifest.json` as assets.

## 15. Good neighbors (in scope)

Cheap additions that slot into the same IPC / engine and pay off:

- **Reusable health self-test module.** One probe (motors ack, model loads,
  media inits) called by two callers: at boot, and post-update as the health
  gate. Single source of truth for "is the robot OK."
- **`robotctl` local CLI.** A thin client over `updaterd`'s unix socket — same
  role as `btd`, different transport; it holds no update logic of its own. For
  support, field recovery, and CI.
  Commands are namespaced (`robotctl update …`) so later namespaces are additive:
  `check`, `apply [--version|--dry-run]`, `rollback`, `reset-to-golden`,
  `select` (how a model bundle is switched), `pin`, `status`, `log`, `watch`.
  Only the `update` namespace is implemented.
- **Dev sideload path.** Accept an artifact signed with a distinct **dev key**
  (present in the trusted set but gated behind a flag / dev build) so the team
  can flash local builds without touching prod signing (see §5.4).
- **BLE provisioning security.** Adjacent but important: wifi credentials pass
  over BLE during setup. That characteristic must be paired + encrypted, or it's
  a credential leak. Update artifacts are signed so a spoofed *trigger* is
  low-risk, but *provisioning* writes must be authenticated. Confirm `btd`
  already enforces this.

## 16. Release testing & confidence

**Motivation.** On a previous robot, releases were validated by hand — manually
revert to a known version, manually apply the new one, manually check. Too much
room for human error. Design goal here: **make the update path scriptable and
auto-tested so shipping to clients is a *decision*, not a manual procedure.**

Key framing: the bulk of that human-error surface (revert → apply → verify)
lives in the update *mechanism*, which is pure software and testable in CI with
**no hardware**. Only genuinely hardware-dependent behavior needs a real board.

### 16.1 Design for testability

- **Real code path against test artifacts.** `updaterd` reads its source/manifest
  from config, so tests point it at a local dir or a `staging` channel and drive
  the *exact production code* — no mock updater that drifts from reality.
- **Scriptable, idempotent primitives** (via `robotctl`, §15):
  `apply --version X`, `rollback`, `pin <version>` / `unpin`, `model select`,
  **`reset-to-golden`**. The previous pain was the absence of a clean "go to
  exactly this version" command — these are it. All **non-interactive**: no
  prompts, nothing for an operator to answer wrongly.
- **`--dry-run`:** run fetch / verify / compat / space checks and stop before
  the swap.
- **Fault-injection hooks:** flags to force a failing health probe, a corrupt
  artifact, a non-zero hook, or a kill mid-swap — so rollback is *tested*, not
  assumed.

### 16.2 Two test tiers

**Tier 1 — mechanism tests (CI, no hardware, fast).** Run on every PR; this
alone removes most of the manual-revert risk. Cover automatically:

- signature valid / invalid / wrong-key → accept / reject
- hash mismatch → reject
- `min_hw_rev` / `model_api` incompatible → refuse with clear status
- older-but-validly-signed manifest offered as "latest" → refused as a downgrade
  (§8.4.1), while an explicit `--version` downgrade is still allowed
- `rollback` never lands on the release that just failed, nor on one the log
  records as rolled back
- boot-counter trials are per-component: one component's transition must not
  consume another's budget
- **robot-specific state preserved** across update *and* rollback: calibration,
  generated assets, learned state, user prefs all intact (§5.7)
- healthy update → commit + prune
- failing health probe → auto-rollback; robot healthy on the *old* version
- hook non-zero exit → rollback
- **kill -9 mid-swap / simulated power loss** → consistent state on restart,
  never a half-live release
- disk full → abort before any change
- concurrent triggers → single-flight; second reports busy
- **`robotd` absent / crash-looping** → `btd` and `updaterd` still serve; update,
  rollback and reset-to-golden all complete (§4.1). Include a "recover from a
  deliberately broken release" test — it exercises the path that matters most.
- `robotd` IPC hangs (socket open, no reply) → timeout, not a stall
- version matrix: upgrade A→B, skip A→C, rollback B→A, migration across a
  `schema_version` bump

Tier 1 drives the engine with an in-process `FakeRobot` implementing `RobotClient`. That
is the right way to test the engine's *decisions* — a fake can be unhealthy, hung or
absent on demand, none of which is easy to stage for real. But it never serialises
anything, so it cannot see the wire at all.

**Tier 1b — against a real `robotd` process** (`robotd/tests/updater_gate.rs`, still CI,
still no hardware) closes that. It spawns the actual binary and lets the real
`SocketRobotClient` talk to it over a real unix socket: every method the engine calls must
come back in the shape it parses, an update must gate and commit, and `robotd --unhealthy`
must revert the *content* behind `current`, not merely the symlink.

Nothing else covers this. The protocol crate's own round-trip tests cannot detect skew,
because both sides share the struct; a `dispatch`-level unit test in `robotd` catches a
changed reply shape cheaply, but only a real process exercises the socket itself.

**Tier 2 — on-device acceptance (canary robots).** Hardware-dependent behavior
(gait, motors, media) needs real boards. Keep a few **lab/canary robots** that
track `staging`, auto-update on each candidate, run a scripted acceptance test
(motor sweep, gait smoke test, media init, health self-test), and report
pass/fail + the update log. Repeatable because reset-to-golden and
`apply --version` are scriptable.

### 16.3 Channels & promotion

**Implemented** as three GitHub Actions workflows plus a `cargo xtask` publisher
(`.github/workflows/`, `xtask/`):

| | |
|---|---|
| `ci.yml` | fmt, clippy, tests, plus `board-test.sh` — the only job that proves the binaries run on aarch64 Linux |
| `release.yml` | on a `daemon-staging-v*` tag: cross-build, package, sign, **verify with the robot's own code path**, publish a prerelease |
| `promote.yml` | manual: re-sign a *stable* manifest pointing at the staging artifact |

The publisher is a Rust `xtask` rather than a shell script for one reason: it reuses the
exact `minisign`, `tar`, `zstd` and `sha2` crates the updater verifies with. A shell
version would depend on separately-installed binaries whose behaviour could drift from
what the robot accepts — the last place a difference should be able to hide. It also
links the *full* `minisign` crate (which can sign) while the daemon links only
`minisign-verify` (which cannot).

Three properties worth stating, because each is asserted rather than assumed:

- **Promotion never rebuilds.** The stable manifest carries the staging `sha256` and
  points at the staging artifact URL, so the bytes clients receive are the bytes the
  canary validated; a test asserts the digest is unchanged.
- **Artifacts are reproducible.** Fixed mtimes in the tar mean the same inputs produce
  the same archive, so a rebuild can be compared against what shipped; a test asserts two
  packages of the same inputs hash identically.
- **`release.yml` verifies before publishing.** It installs the release through the real
  engine (`updaterd install --from`, over a `LocalDir` source) and asserts the binaries land
  executable. If the updater cannot accept a release, nobody can download it.

Two guards exist because both mistakes are easy: `package` refuses a `--version` that
doesn't match `Cargo.toml` (tagging without bumping), and `promote` refuses a version
that doesn't match the staging manifest.

- Channels: `staging` → `stable`. CI publishes candidates to `staging`.
- Canary robots auto-pull `staging`, run Tier-2, report.
- On green, **promote**: repoint `stable` at the *same bytes* already validated —
  re-sign the `stable` manifest to reference the identical tarball + hash. No
  rebuild, no re-flash, no hand-copying files. Promotion is one command / one
  tap and ships exactly what was tested.
- Bad `stable` → the same move in reverse: repoint `stable` at the prior
  known-good manifest; `min_supported` (§8.1) then pulls robots forward once the
  fix lands.

### 16.4 Provenance

`version.toml` and the update log record each artifact's git SHA + hash, so "the
exact version a client is running" is always reproducible in the lab.

This closes the loop that hurt last time: **humans decide *whether* to promote;
the machine does revert / apply / verify identically every time.**

### 16.5 Bootstrap state — over for the health gate, not for the payload

The `daemon` artifact still ships less than it eventually will: `bin/updaterd`,
`bin/robotctl`, `bin/robotd`, `version.toml`. There is no `mediad` or `btd` yet.

That is not circular: §4.1 establishes that `updaterd` and `btd` ship *inside* the daemon
artifact, so updating the updater with the updater is the real eventual flow — and the
riskiest one. Exercising it first is the point of §9's build order.

Two settings in `updater.example.toml` were deliberately inert until something existed to
gate against. Both are now live:

| | was | now | still pending |
|---|---|---|---|
| `on_apply` | `none` | `restart` with `["robotd"]` | `mediad` joins the list when it exists |
| `health` | `none` | `socket`, 30s | timeout is a guess until M4 measures a real boot |

`mediad` stays out for the original reason: naming a unit that is not installed makes
`systemctl restart` fail, which fails the update, which rolls it back — so listing it
today would revert every update.

**`health = none` was the weakest the design gets**: it commits as soon as the swap
succeeds, so there was no auto-rollback at all, and the boot counter was the only
remaining recovery mechanism. Leaving that state is what M1 was for.

The unit test that pinned the example config to the inert values now asserts the
opposite: `on_apply` must restart `robotd` and must **not** restart `updaterd` or `btd`
(§4.1), and `health` must be a socket probe. A regression to `probe = "none"` would
silently disable auto-rollback while looking like a one-word diff, which is exactly the
kind of change that needs a test standing in front of it.

**What is still untested:** the `systemctl restart` in `on_apply` has never run against
real systemd — there is none on a dev laptop, and stubbing it would test the stub. The
health gate itself *is* tested against a real `robotd` process over a real socket
(`robotd/tests/updater_gate.rs`), so what remains unproven is specifically the restart
step and the 30s timeout. Both land in M4 on the Radxa.

## 17. Open questions / future

- **Config ownership.** Config currently rides with the model bundle. Does any
  config belong to the daemon channel instead, or become its own tiny channel?
  Schema migrations are handled by hooks (§9) regardless.
- **Minimal "did it succeed" phone-home.** Fleet mgmt is deferred, but a single
  success/failure ping per update would let us catch a bad release early and is
  what makes `min_supported` (§8.1) actionable in practice. Worth it?
- Staged rollouts / telemetry / dashboards: explicitly deferred; revisit if
  fleet grows.
- Delta updates: deferred; artifacts are small at current cadence.

Known gaps between this document and the implementation, deliberately open:

- **Competing alternatives within one model slot are not supported.** Each model is a
  component with one version line (§5.5), which covers "walk, jump and stand each
  update independently". It does *not* cover "two different walk models installed, user
  picks one". That needs a `(name, version)` store key, which ripples through ~14 files
  — `Store`'s keyed methods, `known_bad`, `PendingUpdate`, `Pins`, `golden`, `pinned`,
  and the wire types (so an `API_VERSION` bump). It also turns four things per-bundle
  rather than global: prune counts, rollback targets, "latest", and golden. Deferred
  until it's known to be needed; guessing at those four is how the deleted
  `Layout::Library` came about.
- **No way to discover installable models.** The app can list what *is* installed, but
  no `Source` can answer "what could be installed" — adding a model means editing
  config today. Wants a `Source::list_*` operation.
- **No recovery mode.** §8.2's chain is `current → previous → golden`, all three
  implemented including escalation past a missing or known-bad previous. The final
  "minimal recovery mode that can still re-fetch" does not exist.
- **Successful hook output is discarded.** Only failure text reaches the log, via the
  error. Recording successful output too would help debug a migration that "worked"
  but did the wrong thing.
- **One `RobotClient` serves every component.** A `HealthCheck::Socket { path }` is
  honoured for *which probe to run*, but the socket path itself comes from whatever
  `main` constructed. Fine while both components probe `robotd`; a trap if a
  component ever needs a different peer. Wants a per-component client.

Still open:
- **Phone-delivered artifacts as a fallback** — for a robot with no usable
  internet: never provisioned, captive portal, blocked CDN, offline demo site.
  The engine side is nearly free, because `LocalDir` (§15, §16.1) already applies
  a directory of manifest + artifact + both `.minisig`s through the real
  verification path, and signing makes the delivery transport untrusted by
  construction. The cost is entirely in the link, which must be wifi or USB
  (§3.1). Two shapes, and they solve different problems: the robot hosts an AP
  for the phone to join (needs no robot credentials, but the phone must fetch the
  release over cellular *before* joining, and both mobile OSes resist a network
  with no internet), or — where wifi works but the CDN does not — the phone
  pushes over the LAN, much cheaper but it adds a network-facing ingest listener.
  That listener's blast radius is bounded by signature verification (an unsigned
  upload cannot install; worst case is filling the disk), but it wants a token
  and a deliberate bind regardless (`architecture.md` §2.2). Backup plan, not v1.
- **Manifest staleness reporting** (§8.4.2) — surface "last successful check N
  days ago" in `status` and the app, converting a freeze attack from silent to
  visible. Cheap; recommended. Signed manifest expiry is the real defence but
  carries a re-signing schedule, deferred until publishing is routine.
- **Config ownership** — does config ride with the model bundle, the daemon, or
  become its own tiny channel? (Hooks handle migrations either way, §9.)
- **Minimal success/failure phone-home** — one ping per update would let us catch
  a bad release early, and is what makes `min_supported` (§8.1) actionable in
  practice. Worth it, or too much for v1?
- **Behaviour/brain layer** — if the higher-level behaviour layer (drives, mood,
  habits) ever ships as its own artifact, it becomes a third channel with its own
  compatibility constraint. Its learned state is also squarely §5.7 material.

Explicitly **not** doing: hardware variant matrix (§5.6), peripheral firmware OTA
(§11.1), staged rollouts / telemetry, delta updates.

Decided since first draft: split hosting (GitHub + HF Hub); `min_supported` floor
in schema; model *bundles with named slots* + `model_api` compatibility; multiple
trusted signing keys; single hardware target; robot-specific state preservation
(§5.7); config in a file rather than the systemd unit.
