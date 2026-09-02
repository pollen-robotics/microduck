# What restarts, and when

The authoritative sequence for every path that moves the `current` symlink, and for boot. Written
because "which daemon restarted, and at what point" has been answered three different ways from
three different documents, and the answer decides how a skew is diagnosed.

Everything here is read from the code, and each step names the function that owns it. Where a
narrative comment elsewhere disagrees with this page, the comment is the bug.

## 1. The seven daemons, and the unit that is not one

A release ships seven daemons — `robotd`, `configd`, `btd`, `padd`, `mediad`, `tofd`, `updaterd` —
each `ExecStart`ing a path under `/opt/robot/daemon/current/bin/`, so each is stale the instant the
symlink moves, and each is either restarted by the update or restarted after it.

It also ships `robot-boot-check.service`, which is **not** one of them and which an update must never
restart: it asks whether the release that booted came up and hands over to `robot-rescue` if not, so
running it mid-update points a rollback check at daemons that are legitimately mid-restart. It carries
no `[Install]` section, and that is what keeps it out — see §1.1.

| unit | restarted mid-update | restarted ~5 s after the reply |
|---|---|---|
| `robotd` | yes | — |
| `configd` | yes | — |
| `padd` | yes | — |
| `mediad` | yes | — |
| `tofd` | yes | — |
| `updaterd` | **never** — it is the process performing the update | yes |
| `btd` | **never** — it may be the transport the update arrived over | yes |

Both halves are one decision, and a test (`every_unit_held_back_is_restarted_once_the_answer_is_out`)
asserts the two lists in `engine.rs` are the same set: `NEVER_RESTART` and `RESTART_AFTER_REPLYING`.

Nothing is held back until a reboot. That was true before `RESTART_AFTER_REPLYING` existed and is the
single most common stale claim about this system.

And because scheduling a restart is not evidence that one happened, the next `updaterd` start checks
each unit against the active release and restarts anything stale — §5.

### 1.1 What counts as a unit an update starts

A `systemd/*.service` file with an `[Install]` section. One without is triggered by something else — a
timer, or another unit pulling it in — so its lifecycle is not the update's to drive, and
`engine.rs::has_install_section` is where that is decided.

The rule rather than a list of names, because there were already two places applying it and they
disagreed: `hooks/postinstall` declines to `enable --now` a unit with no `[Install]`, saying so in as
many words — *"`enable --now` on it would run a rollback check in the middle of the update that
installed it, with daemons legitimately mid-restart"* — while the engine read every `*.service` and
restarted it a moment later anyway. Keyed on the section, the two agree by construction and the next
unit like it needs nobody to remember.

An unreadable unit file counts as one to restart. That is more likely a permissions problem than a
deliberately triggerless unit, and a restart failing loudly beats one skipped quietly.

### How the set is computed

Two sets, one function apart. `engine.rs::units_shipped` is everything the release provides:

```
(the release's own systemd/*.service stems)  ∪  (on_apply.units from /etc/robot/updater.toml)
```

sorted and deduplicated. `engine.rs::units_to_restart` is that, minus `NEVER_RESTART`:

```
units_shipped(…)  −  {updaterd, btd}
```

An update restarts `units_to_restart`; the startup check in §5 reads `units_shipped`, because the two
units an update cannot touch are exactly the two it exists to watch.

On today's release `units_to_restart` is exactly:

```
configd, mediad, padd, robotd, tofd
```

in that order — alphabetical, so the order is identical on every board and in every test. Nothing
was added to `deploy/updater.toml` to put `mediad` and `tofd` in that list, and nothing needed to
be: both ship a unit with an `[Install]` section, which is the whole rule.

Two consequences of deriving it from the release rather than from the board:

- `padd` is restarted even though `deploy/updater.toml` never mentions it. The config list is
  **additive**, not authoritative; it only needs to name units the release does *not* ship.
- A board whose `updater.toml` predates a daemon still restarts that daemon. That file belongs to the
  operator and `install.sh` preserves it, which is how `configd` went unrestarted for a while
  (`../project/install-path-gap.md` §4).

Each unit is restarted by its own `systemctl restart <unit>` invocation, never batched — a single
`systemctl restart a b` fails as a whole when either unit is unknown, *without* restarting the one
that exists. A unit systemd reports as `LoadState=not-found` is skipped with a warning; a unit that
exists and fails to restart **fails the update and rolls it back**.

## 2. `robotctl update apply` / `update.apply` over BLE

`engine.rs::apply` → `apply_inner` → `stage_and_swap` → `post_swap`. Nothing before step 8 touches a
live path.

| # | step | restarts anything? |
|---|---|---|
| 1 | preflight: clock, robot stopped, no live session | no |
| 2 | fetch manifest, verify its signature, channel / pin / downgrade / compatibility checks | no |
| 3 | preflight again, now for disk space (the requirement comes from the manifest) | no |
| 4 | download to `releases/.staging-<ver>/dl/` | no |
| 5 | verify sha256, then verify the artifact signature | no |
| 6 | extract to `releases/.staging-<ver>/root/`, write `.updater-manifest.json` | no |
| 7 | **`hooks/preinstall`** (cwd = the staged tree) — installs ONNX Runtime if below the floor, then runs the release's `scripts/setup-gstreamer.sh` for `mediad`'s stack | no |

**The hook script comes from the incoming release; the code that runs it does not.** `updaterd`
never restarts itself mid-update (§1), so every step in this table is executed by the `updaterd`
that was already running — the *previous* release's. A change to how hooks are run, logged or
bounded therefore takes effect one apply later, on the first apply after `updaterd` has restarted
onto the release carrying it. This cost a confused round of "the hook ran and logged nothing" once;
`cat /run/updaterd/identity.json` is what tells you which build is about to run your hook.

| 8 | `rename` the staged tree to `releases/<ver>/` | no |
| 9 | arm the boot counter (`pending.json`), *before* the swap | no |
| 10 | swap `current` → `releases/<ver>` | no |
| 11 | **`hooks/postinstall`** (cwd = `releases/<ver>`) | **starts newly shipped units** |
| 12 | `on_apply` — `systemctl restart` each unit from §1, one at a time | **configd, mediad, padd, robotd, tofd** |
| 13 | `releases/<ver>/bin/updaterd --self-test` | no |
| 14 | health gate: poll `robotd` over its socket, every 500 ms, up to 30 s | no |
| 15 | confirm the boot counter, prune old releases | no |
| 16 | release the update lock, schedule the deferred restarts | **schedules updaterd, btd** |
| 17 | the reply goes out to `robotctl` / the app | — |
| 18 | ~5 s after step 16 | **updaterd, btd** |

Any failure from step 11 onward rolls back: swap `current` back, re-run `on_apply` against the
previous release, confirm the boot counter, journal it. Steps 1–10 fail with the old release still
live and nothing to undo.

**Step 2 can also end it, and not always without restarting anything.** If the manifest names the
release already installed, the operation stops there and reports `already_current` — nothing is
downloaded and nothing is swapped. Before replying it does §5's reading over that release's units and
names in `stale` any that are running something else, and those are scheduled exactly as steps 16–18
schedule the deferred pair. So `apply` on an up-to-date robot is inert, and `apply` on a robot whose
daemons disagree with what is installed is not. `select` on the release a board already has active does
the same, for the same reason.

### Step 11 in detail — `hooks/postinstall`

The hook ships inside the signed artifact and runs with the release directory as its cwd. In order:

1. `install` every `systemd/sysusers.d/*.conf` to `/usr/lib/sysusers.d/`, then run
   `systemd-sysusers`. Accounts before units, because a unit naming a missing `User=` fails to start
   and reads as a broken daemon.
2. `install` every `systemd/*.service` to `/etc/systemd/system/`, overwriting. **All of them**,
   including `updaterd.service` and `btd.service` — the hook has no exclusion list, and does not
   need one.
3. `systemctl daemon-reload`.
4. `systemctl enable --now` each unit that has an `[Install]` section (§1.1); one without is
   installed and left alone, and a `.timer` is enabled without `--now`.

Step 4 is why the exclusions in §1 still hold: `--now` means *start*, and starting an already-running
unit is a no-op. It does not restart `updaterd` or `btd`. What it does do is start a unit the board
has never had — which is the whole point of the hook, and means **a newly introduced daemon starts
twice on the release that introduces it**: once here, then again at step 12.

Every failure inside the hook is a warning except a failed `install` of a unit or sysusers file, which
exits non-zero and therefore rolls the update back. A service that will not start is deliberately not
fatal: `btd` legitimately fails on a board whose radio has not appeared yet.

### Step 13 — the self-test

`updaterd` does not restart itself during the update, so a replacement binary that cannot start would
otherwise be discovered at the next boot, after the commit, with recovery living inside the process
that is failing. Instead the *new* `bin/updaterd` is run with `--self-test`: it loads
`/etc/robot/updater.toml`, constructs the engine, and exits before touching any state. 10 s ceiling.

A failure is `Error::SelfTest` carrying the binary's last line of stderr, and rolls back. A release
with no `bin/updaterd` (a model component) passes trivially. Skipped entirely unless `on_apply` is a
restart, which is what keeps it out of the bootstrap install's way.

### Steps 16–18 — the deferred restarts

`engine.rs::schedule_deferred_restarts`, for `updaterd` then `btd`:

```
systemd-run --on-active=5s --timer-property=AccuracySec=100ms -- systemctl restart <unit>
```

Four details that are load-bearing:

- **A transient unit, not a child process.** `systemctl restart updaterd` kills `updaterd`'s whole
  cgroup, and a child of `updaterd` is *in* that cgroup — it would be killed partway through
  restarting its own parent. A `systemd-run` timer lives outside it.
- **The update lock is dropped first** (step 16, before the spawn). A fork duplicates every open
  descriptor, so spawning while holding the lock hands a copy to the child.
- **On `Applied`, and on `already_current` with stale units** — `engine.rs::restarts_owed` decides
  which, and it is the only thing that decides it. The first owes the fixed pair; the second owes
  whatever §2 found running the wrong release, which is usually `updaterd` itself. A rollback owes
  nothing: it leaves the resident `updaterd` already matching `current`, so restarting there would be
  churn with nothing to fix.
- **Every unit goes through the timer**, including ones an update would restart in place at step 12. A
  second, immediate path for `configd` and `robotd` would buy five seconds and cost a mechanism.
- **Failures are logged, never returned.** An update that succeeded is not reported as failed because
  a restart could not be scheduled; the cost of that is a daemon on an old binary until the next boot.

The timer is scheduled *before* the reply is written (the engine runs inside the `update.apply` call),
so the 5 s is measured from step 16, not from when the client sees the answer. A client sees its
outcome and then a dropped connection — for BLE, an ordinary reconnect.

### What the restarted `updaterd` then does

It is a normal start, so it runs §4's startup sequence in full: `clean_staging`, `record_boot`, the
reconciliation of §5 — which is where a `btd` restart that did not happen gets caught — then serve.
Two things follow that are easy to miss:

- **The boot counter counts `updaterd` starts, not literal boots.** A successful apply confirms its
  own trial at step 15, so there is nothing outstanding to advance — but a trial left armed by an
  *earlier* interrupted update is advanced by this restart.
- **The periodic-check clock resets.** `INITIAL_CHECK_DELAY` is 60 s from process start, and
  `check_interval` counts from there.

## 3. The other transitions

`select`, `rollback` and `reset-to-golden` share `engine.rs::transition_to`: arm, swap, `on_apply`,
health gate, revert on failure. They do **not** run hooks and do **not** self-test — no artifact was
extracted, so there is nothing new to install or prove.

| | hooks | `on_apply` (§1) | self-test | deferred updaterd/btd restart |
|---|---|---|---|---|
| `update apply` | yes | yes | yes | **yes** |
| `update select <ver>` | no | yes | no | **yes** |
| `update rollback` | no | yes | no | **no** |
| `update reset-to-golden` | no | yes | no | **no** |
| auto-rollback inside a failed apply | no | yes | no | **no** |
| boot-counter revert at startup | no | yes | no | **no** |

The two `no`s at the bottom are correct by construction: in a failed apply and in a boot-counter
revert, `updaterd` and `btd` were never restarted, so they are still the binaries belonging to the
release being returned to.

The `no` for an explicit `rollback` / `reset-to-golden` is a different case and worth knowing:
if the release being left was applied successfully at some earlier point, `updaterd` and `btd` *were*
restarted onto it, and an explicit revert leaves them there. `robotd`, `configd` and `padd` move back;
those two stay **ahead** of the active release.

That resolves itself at the next `updaterd` start rather than at a reboot: §5 compares in both
directions, so `btd` running a release newer than `current` is stale by the same test and gets
restarted. `updaterd` is the one process that does not self-heal there, by design.

Because `select` and the reverts skip `hooks/postinstall`, they also never *remove* a unit file. A
downgrade to a release predating a daemon leaves that daemon's unit installed, pointing at a binary
the older release does not contain; the unit fails, and since it is in the restart set, the failed
restart fails the transition (`../project/install-path-gap.md`).

## 4. At boot

systemd starts the daemons from `multi-user.target`, and `robot-boot-check.timer` arms the
recovery check for 180 seconds in. There is **no ordering between the daemons**
beyond `padd` after `robotd` (advisory — `padd` exits and retries every 5 s if the socket is absent)
and `btd` after `dbus`/`bluetooth`. Nothing waits on `updaterd`, and `updaterd` waits on nothing but
`network-online.target`, which is `Wants` rather than `Requires`.

On this board `hci0` does not exist until roughly 73 s after power-on, so `btd` retries for an
adapter rather than failing.

`updaterd`'s own startup, in order (`main.rs::serve`):

1. Log the startup identity line at `warn` — version, revision, **`exe` path**, pid. The `exe` path is
   what tells you which release directory the process actually came from.
2. Load `/etc/robot/updater.toml` and the trusted keys. Either failing is fatal.
3. Construct the engine.
4. `--self-test` returns here, before any state is touched.
5. `Engine::recover_on_start`, **before the socket is served**, so a robot that booted into a bad
   release has already begun reverting by the time anything can ask it to do something else:
   - `clean_staging` for every component: delete `releases/.staging-*`.
   - `record_rescue`: if `<state_dir>/rescued` exists, the boot recovery net swapped `current` to
     golden while nothing was running. Copy it into the update log as a rollback, **clear the trial
     for that component**, and delete the file. Before `record_boot` on purpose: the rescue already
     went further than the trial would have, and a trial left armed would move `current` back off
     golden a boot or two later. Deleting the file is also what releases the rescue's loop guard.
   - `refresh_golden_links`: publish each component's configured golden as a `golden` symlink beside
     `current`, so the rescue can find it with one `readlink` and no parser.
   - `record_boot`: increment `boots` on every armed trial in `pending.json`.
   - For each trial with `boots >= 2` (`MAX_BOOT_ATTEMPTS`): revert to `previous`, escalating to
     `golden` when `previous` is absent, missing from disk, or itself recorded as rolled back. That
     means swap the symlink, confirm the trial, and re-run `on_apply` — **so a boot-counter revert
     restarts `configd`, `padd` and `robotd`.** It runs no hooks and schedules nothing, so the
     `updaterd` and `btd` processes systemd has just launched are left running from the release being
     abandoned. Step 6 then catches `btd`, since it runs after this and compares against the release
     the revert made active; `updaterd` is reported and left, and is stale until the next boot.
   - Journal the outcome. A failure here is logged and serving continues: refusing to serve would
     remove the only way to fix it.
6. `Engine::reconcile_running_units` — §5. **After** recovery, because recovery can change which
   release is active, and before it a unit would be compared against a version the robot is in the
   middle of abandoning. This step can restart units.
7. `--check-only` returns here. Note that it *performs* both recovery and the reconciliation of step 6
   — it is an operator tool, not a probe, and it can revert a release and restart daemons.
8. Serve `/run/updaterd.sock`.
9. If `check_interval` is set, spawn the scheduler: first pass 60 s after start, then every interval.
   `auto_apply` decides what it may install with no client attached.

A trial only exists if an apply was interrupted between step 9 and step 15 of §2 — a power cut, a
`kill -9`, or a cancelled RPC after the swap. A committed update leaves nothing armed. And since
`exhausted` is `boots >= 2` while `arm` writes `boots = 0`, the *first* `updaterd` start after the
interruption logs "update still on trial" and the *second* reverts.

**Every step above presupposes an `updaterd` that starts.** One more thing happens at boot for the
case where none of it runs: `robot-boot-check.timer` fires 180 s in, asks whether `robotd`, `configd`,
`btd` and `updaterd` came up, and hands over to a `/bin/sh` rescue that swaps `current` to golden and
reboots if they did not. It is deliberately outside every process on this page —
[`boot-recovery-net.md`](boot-recovery-net.md) owns it, including why it is a deadline rather than
`OnFailure=` on these units.

## 5. The startup reconciliation — did the restarts happen?

`systemd-run` returning success means a transient timer was created. It does not mean the restart ran,
and it does not mean the new binary started; §2's step 16 swallows those failures on purpose. That
leaves one silent way for a robot to end up running a release it did not install, and it lands on the
two units nothing else watches. `updater/src/reconcile.rs` closes it.

### What it reads

Each daemon publishes its own identity at startup — `duck_ipc_proto::publish_identity`, called from
the `log_startup_identity!` macro every daemon opens with:

```
/run/<service>/identity.json    { service, version, revision, built_at, exe, pid }
```

The `exe` field comes from `/proc/self/exe`, so it resolves through the `current` symlink and names
the release directory the process was actually launched from. `Identity::release()` parses the
`releases/<ver>` component out of it.

Self-published rather than read from outside: a process knows its own version and revision and can
read its own `exe` with no privilege, where reading another user's needs root. `RuntimeDirectory=<service>`
in each unit creates the directory owned by that unit's `User=`, survives `ProtectSystem=strict`, and
is **removed by systemd when the unit stops** — so a stopped daemon cannot leave behind an identity
claiming to be running.

### What it decides

Per component, for each unit in `units_shipped` (§1) — so `updaterd` and `btd` are included, which is
the point — compare the published release against the component's active release. `verdict_for` is a
pure function; every syscall is kept outside it.

| verdict | when | action |
|---|---|---|
| `Current` | published release == active | nothing, silently |
| `Restarted` | they differ | `systemctl restart <unit>`, at `warn` |
| `ReportedOnly` | they differ **and the unit is `updaterd`** | logged, never acted on |
| `RestartFailed` | the restart was attempted and failed | logged at `error` |
| `Unknown` | no identity file | nothing |

Three deliberate non-actions:

- **`updaterd` never restarts itself here.** A successor that disagreed about which release is active
  would restart, disagree again, and loop — in the one process that owns recovery, so nothing would be
  left to break the cycle. It is safe to only report: the process has just started, so anything stale
  about it was decided before this code ran. It is repaired from the other side instead — see below.
- **A stopped unit is left stopped.** No identity file means either stopped or too old to publish, and
  both read the same here. Starting it would override, on every `updaterd` start, whoever stopped it.
- **A daemon that published nothing is not treated as stale.** Restarting a robot's daemons for being
  old is a decision nobody asked for; the next update makes them able to answer.

The comparison is direction-agnostic — any difference is stale — which is what makes it cover a
release left *ahead* of `current` by an explicit rollback, not only one left behind.

It runs on every `updaterd` start, so at boot it is a no-op: everything started from the same symlink.
The cost is one file read per unit.

### The same reading, from `apply`

`reconcile::stale_units` is `verdict_for` over the same identity files with the acting left out, and
`apply` and `select` call it on their already-current paths (§2). It exists because of the exception
above: a stale `updaterd` is the one skew this module refuses to repair, and the only thing that ever
looks at it afterwards is a person running `apply` because a daemon seems wrong. That used to answer
`already_current` and schedule nothing, which read as confirmation that there was nothing to fix.

Two properties keep it from being the loop §5 guards against. It fires on a request rather than on
every start, and it schedules through `systemd-run` rather than driving `systemctl` from inside the
process being restarted. It reports as well as acts: the units are named in the `already_current`
outcome, so a client — `robotctl`, or the app — sees which daemons were not running the release the
robot has installed.

Both stale verdicts count as stale here (`Restarted` and `ReportedOnly`); the difference between them
is about who may perform the restart, not about what is wrong. Everything else the table says is
unchanged: a stopped unit and a daemon that published nothing are still not stale.

## 6. First install

`scripts/install.sh` on a bare board, in order (`main`):

1. Write `/etc/robot/updater.toml` and the trusted keys.
2. `bootstrap_first_release`: fetch a standalone `updaterd` binary and run
   `updaterd install [--from <dir>]`. That is the ordinary `Engine::apply` with two settings forced,
   because on a board with no release they are facts rather than policy: `on_apply = none` (the units
   live inside the release being installed) and `health = none` (there is no `robotd` to probe).
   So §2 runs with steps 12, 13 and 14 skipped — but **step 11 still runs**, and
   `hooks/postinstall` is what installs the units and `enable --now`s every one with an
   `[Install]` section. The daemons first start there, from inside the hook. Step 16 is not skipped either: the bootstrap install schedules
   the `updaterd` and `btd` restarts for 5 s later, while `install.sh` is still running.
3. `install_units`: copy the units again, `daemon-reload`, then `enable --now` `updaterd`,
   `robotd`, `configd`, `btd`, `padd`, `mediad` in that order — `configd` before `btd` because `btd`
   asks `configd` for the pairing PIN, and `mediad` last because its unit is `After=` three of the
   others. `btd`, `padd` and `mediad` may fail without failing the install: a robot with no radio, no
   gamepad or no camera still updates and walks. Then `robot-boot-check.timer` is enabled *without*
   `--now`. Redundant with the hook and harmless; it is also the path for a release older than the
   hook. A unit the release ships that this function does not know is installed, reported, and left
   alone. `tofd` is named as known but gets no `enable_unit` of its own: the hook enabled it a step
   earlier and nothing depends on it, so there is no ordering for this function to have an opinion
   about.
4. `install_token_dropin`: write the `GITHUB_TOKEN` drop-in, `daemon-reload`, and
   `systemctl try-restart updaterd` — `daemon-reload` alone would leave the *running* `updaterd`
   without the token, which is every board.

`DUCK_FORCE_REINSTALL=1` adds `stop_for_reinstall` before step 2: `systemctl stop` on `padd`,
`tofd`, `btd`, `configd`, `robotd`, `updaterd`, in that order. Nothing is live while the swap happens, and there is
no health gate behind it.

## 7. Diagnosing a skew

Two version numbers are legitimately different at once, and which pair it is decides the diagnosis.

| observation | means |
|---|---|
| `updaterd` or `btd` behind the installed release, for a few seconds after an update | expected — the deferred restart has not fired yet |
| `btd`, `robotd`, `configd`, `padd`, `mediad` or `tofd` disagreeing with `current` at all, persistently | the restart did not take effect *and* §5 did not fix it — so either `updaterd` has not restarted since, or its restart failed (journal, at `error`) |
| `updaterd` behind it, persistently | the deferred restart never landed, and §5 will not fix this one. `robotctl update apply daemon` repairs it — it reports `already_current` with `updaterd` in `stale` and schedules the restart. `systemctl restart updaterd` is the same fix by hand; either way the journal has why it did not land |
| any daemon reporting `restarting` | it exited and systemd is bringing it back. For a few seconds after an update, that is the restart itself. Persistently, the daemon cannot start at all — `mediad` with the camera flex unplugged is the case this row exists for — and §5 leaves it alone, since a daemon that is not running has no identity to compare. `journalctl -u <name> -b` has why it exits |
| any daemon reporting `failed` | it could not start and systemd has stopped retrying. Same journal, and unlike `stopped` nobody chose it |
| any daemon reporting `build unknown` | it is running, and published no identity: a build predating the mechanism, or the instant between `exec` and its first line. §5 leaves it alone. One update makes an old one answerable |

Ask the robot rather than inferring:

```bash
robotctl health
```

The `units` block prints one line per daemon with the release its process was launched from, and warns
when that disagrees with what is installed.

By hand, the same answer from the file the daemon published:

```bash
cat /run/configd/identity.json; readlink /opt/robot/daemon/current
```
