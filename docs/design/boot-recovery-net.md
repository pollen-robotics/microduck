# The boot recovery net

What puts a robot back on a known-good release when the one it booted cannot start its daemons.

This page owns the boot deadline and its predicate, the membership rule for the units it watches,
`robot-rescue`, the `golden` symlink, and the loop guard. [`restart-order.md`](restart-order.md) owns
the sequence this sits inside — including where `updaterd` reads the breadcrumb at startup — and
[`updater-design.md`](updater-design.md) owns the store, the health gate and the boot counter.

## The gap it closes

`updaterd` never restarts itself during an update, because it is the process performing it. Three
layers narrow what that costs, and all three run on a board that is fully up:

- `self_test_updaterd` runs the release's own `updaterd --self-test` before the commit, so a
  wrong-architecture binary, a missing library or a rejected `updater.toml` fails the update while
  the old release is still live;
- the deferred restarts put `updaterd` and `btd` on the new binaries five seconds after the update
  replies;
- the startup reconciliation checks, from the successor's side, that those restarts happened.

A binary that starts on a running board can still fail at **boot**, where the network is not up, a
peripheral is not enumerated, and unit ordering differs. And the reconciliation runs *at `updaterd`
startup*, so it presupposes an `updaterd` that starts.

That last point is the whole reason this exists. `BootCounter::record_boot` runs from
`Engine::recover_on_start`, inside `updaterd`, and counts `updaterd` **starts** rather than literal
boots — so an `updaterd` that does not start does not count, and the never-brick guarantee closes
through the one process that might be the casualty. Left there, a board has a good release on disk, a
trial armed to revert to it, and nothing running that can act on either.

The boot counter handles a *different* failure: a release that boots and runs but never reports
healthy. This one handles a release whose daemons do not come up at all.

## The parts

| | |
|---|---|
| `robot-boot-check.timer` | fires a second into every boot |
| `robot-boot-check.service` | the oneshot: watches until the release came up or did not, hands over if not |
| `scripts/robot-boot-check` | the predicate. Installed at `/usr/local/sbin/robot-boot-check` |
| `scripts/robot-rescue` | swaps `current` to golden, records it, reboots. `/usr/local/sbin/robot-rescue` |
| `<install_dir>/golden` | symlink to the release with a standing guarantee, published by `updaterd` |
| `<state_dir>/rescued` | breadcrumb: the last attempt, and the loop guard |

Both scripts are `/bin/sh` and both are **copied** into `/usr/local/sbin` by `scripts/install.sh` and
`hooks/postinstall` — the same decision, for the same reason, as the unit files those two copy rather
than reading through `current`. A rescue that shipped as a binary in the release payload would be
broken by exactly the releases it exists to survive: the wrong architecture, a missing shared
library, a panic on startup. A rescue read through `current` would route the recovery through the
thing being recovered.

## The trigger: a watch, not `OnFailure=`

`OnFailure=` looks like the natural fit — systemd fires it exactly when a unit exhausts its restarts
and enters `failed`, with no polling and no resident watchdog. It does not fit *these* units, which
are configured not to reach that state:

| unit | policy | `RestartSec` |
|---|---|---|
| `btd` | `Restart=always` | 5 s |
| `padd` | `Restart=always` | 5 s |
| `configd` | `Restart=always` | 2 s |
| `robotd` | `Restart=always` | 2 s |
| `updaterd` | `Restart=on-failure` | 10 s |

`Restart=always` is deliberate in each case and argued in the unit file. No unit overrides
`StartLimitIntervalSec`/`StartLimitBurst`, so systemd's default of five starts in ten seconds applies
— and a `btd` restarting every 5 s never comes near it. It crash-loops indefinitely without ever
entering `failed`, which is the state `OnFailure=` waits for. Making `OnFailure=` fire would mean
retuning those units so the daemons *give up*: degrading the normal case to enable the rescue case.

Watching inverts the question, needs no unit changes, and catches the forever-restarter. The timer
starts the script a second into the boot, and it looks at the members every 2 s until the first of:

- **a member did not come up** (the predicate below): hand over at once. Restart counts only grow, so
  waiting cannot change the answer;
- **every member has been `active` for 10 s straight**: the release came up. A crash-looper re-enters
  `active` on each restart (`ActiveEnterTimestampMonotonic` moves), so it never stays long enough to
  count;
- **the deadline**, `[update_gate] boot_deadline_s` in `robotd.toml`, 60 s by default: the predicate
  decides on what is there, and a member that never settled — stopped by hand, never started — is left
  alone, as below.

On a Yocto board every member is up by 16 s, so an ordinary boot is decided at about 25 s. The
deadline is not a wait; it is when a boot that never settles is judged, and it is 60 because that
judgement is by restart count: `updaterd` restarts every 10 s and passes three about 45 s into a boot,
so a deadline below that passes a release whose `updaterd` cannot start. The script clamps the value to
20–300 rather than anything refusing it, because every daemon reading `robotd.toml` exits on a value
`Params::load` refuses, and a robotd that does not start is what this rolls a release back for.

A daemon waiting for its hardware is `active`, so it settles like any other: a slow peripheral does
not hold the verdict back, and a slow boot is not a failed one.

### The predicate

Two conditions, both positive evidence that systemd tried to run a daemon and could not keep it
running. Either one, on any member:

- `ActiveState` is `failed`;
- `NRestarts` is 3 or more.

Everything else is left alone, and the asymmetry is the point. A unit an operator stopped is
`inactive` with no restarts, and must not cost the board its release — the same call the startup
reconciliation makes when it leaves a stopped unit stopped. A unit the release predates is not loaded
and answers nothing, which reads the same way. A daemon still waiting for hardware is `active`. And
one crash that recovered leaves one or two restarts, not three. **A false negative costs an operator a
diagnosis; a false positive costs a good release.**

Three restarts rather than one because a genuine loop passes three inside the deadline, while a daemon
that died once on a transient does not.

`--dry-run` exits 3 where it would have handed over, so a caller can ask the question without the
rescue: the OS's own boot gate on a Yocto image, `microduck-slot-health`, which decides whether RAUC
marks the slot good.

### Three ways it refuses to fire

- **`Conflicts=shutdown.target`** on the oneshot. A daemon killed on the way down can be recorded as
  `failed`; without this, a `systemctl poweroff` at the wrong moment eventually reboots a robot into
  golden.
- **An uptime guard.** Past ten minutes, something other than the timer started it, and the answer is
  that the question is not this script's business.
- **No `[Install]` section** on the oneshot, which both installers read as "do not enable this".
  Without it, `hooks/postinstall` running `enable --now` over the units it just wrote would run a
  rollback check in the middle of the update that installed it — with daemons legitimately
  mid-restart, which is exactly what a rollback check reads as a release that cannot start. Timers are
  enabled but never started for the same reason: an `OnBootSec=` timer started past its deadline fires
  immediately.

## The membership rule

The objection to "any daemon fails → roll back" is that a daemon may fail for reasons a rollback
cannot fix — a missing radio, an unpowered motor bus — and reverting a good release over a hardware
fault is worse than doing nothing.

That resolves into a rule for admitting a unit to the set: **a unit may join only if it waits for its
hardware, absent or misbehaving, rather than exiting.** Then a member that is down means the *binary*
is broken — wrong architecture, missing library, panic on startup, a config it rejects — which is
what a rollback fixes.

| daemon | in the set | when its dependency is absent |
|---|---|---|
| `robotd` | yes | waits for the motor bus, "waiting, not commanding anything" |
| `updaterd` | yes | serves regardless; it is the recovery path and must be reachable when things are broken |
| `btd` | yes | retries for an adapter every 5 s, and re-runs the whole bring-up if one appears and then stops answering |
| `configd` | yes | serves `system.*` and `pad.*`, and reports `net.state=unavailable` |
| `padd` | no | exits cleanly with no robotd socket, by design |

`btd` earns its place on merit rather than symmetry: with no network, BLE is the only way in, so a
robot that boots with no `btd` and no known wifi cannot be reached at all. `configd` earns its place
the same way — `system.pin` is where `btd` gets the PIN a phone authenticates with, so a `configd`
that is down takes the phone path with it.

If a future daemon exits on missing hardware, either fix the daemon or leave it out of the set. Do not
weaken the rule.

## The rescue

`robot-rescue` reads two symlinks and parses nothing.

That is not economy. The likeliest thing this rescues is a release whose `updaterd` rejects the
board's `updater.toml` — the operator's file, preserved across installs, while a release is free to
change what it expects. A rescue that parses that same file dies of the disease it is treating.

Golden is a version *in* that file (`ComponentConfig::golden`), so `Engine::refresh_golden_links`
publishes it as a `golden` symlink beside `current` on every `updaterd` start: one `readlink`, nothing
to reject. A cache rather than a second source of truth, and it fails safe — if `updaterd` stops
starting, the link still names the release that was golden when it was last written, which is a
release that was known good. A configured golden that is *not installed* publishes no link at all,
because a dangling one would tell the rescue it has a target when it does not.

**Golden, not previous:** when what broke is the recovery path itself, previous may be broken too.
Golden is the one release with a standing guarantee, and `prune` never removes it.

It declines, saying which it is, when there is no golden link, when golden names a release that is not
installed, and when `current` is already golden. That last one matters most: the daemons are then
failing on the release carrying the standing guarantee, so this is not a release fault, and swapping
would change nothing except adding a reboot — which is how a hardware fault becomes a reboot loop.

The swap renames a temporary symlink over `current`, matching `Store::link_to`, because `ln -sfn`
unlinks and recreates and leaves a window where a daemon cannot exec. `rename(2)` does not follow
symlinks but `mv`'s path resolution does, so a plain `mv tmp current` moves the link *into*
`releases/<version>` and leaves `current` untouched; GNU spells the fix `-T` and BSD spells it `-h`.

`--reboot` is opt-in. Every unit execs through `current`, so the swap does nothing until they restart,
and a robot that is standing when its daemons die is a robot that falls — whoever is holding it
decides. The boot check passes the flag; a human at a console gets the command printed instead.

## The loop guard, and what opens it

Two independent things stop "swap and reboot" happening twice on its own:

1. the refusal when `current` is already golden, which covers the ordinary case, because a successful
   rescue leaves exactly that state;
2. the breadcrumb, which covers the case the first misses — an update moved `current` away from golden
   and failed again.

`Engine::record_rescue` clears the breadcrumb on the next `updaterd` start, after copying it into the
update log. So the guard opens exactly when the board proves it can run its update daemon again, and
stays shut when it cannot: a robot sitting still with a clear journal beats one rebooting forever.
`robot-rescue --force` is the way past it by hand.

That clearing runs **before** the boot counter, and the ordering is load-bearing. An armed trial would
revert to `previous`; the rescue already went further, to golden. Left armed, `record_boot` would
advance the trial and eventually move `current` *off* the release the rescue chose — the recovery net
and the never-brick guarantee undoing each other, one boot apart.

The breadcrumb is `key=value` rather than JSON, though `updaterd` parses it:

```text
at=1786453421
install_dir=/opt/robot/daemon
from=1.2.0
to=1.0.0
because=boot check: robotd.service (failed, 7 restarts)
```

The writer is a shell script running on a board where things are already going wrong, and quoting
JSON correctly in `sh` is a way to produce a record nothing can read. `journal::Breadcrumb` is lenient
for the same reason — every field optional — because a breadcrumb that cannot be parsed is a guard
that never opens. `install_dir` is how the entry is attributed to a component without a second place
that has to agree with `updater.toml` about the name `daemon`.

## Evidence

A silent rollback is how a day goes to "it works on my board", so the swap leaves three traces:

- **the journal**, from both scripts, via `logger` when it is present and stderr always;
- **the update log**, as `Outcome::RolledBack` with the failing unit named in its reason — which is
  what `robotctl update log` shows, and it is permanent, unlike the breadcrumb that is cleared;
- **`robotctl version`**, indirectly but usefully: a board running an older release than the one
  installed is exactly what it reports on.

## What is not tested

The decisions are: `xtask/tests/rescue.rs` asks both scripts every question against a temporary tree,
with a stub `systemctl` supplying unit states and recording whether a reboot was asked for, and
`scripts/board-test.sh` covers the installer half on real ARM64 Linux.

**systemd is not.** Whether the timer fires at `OnBootSec=1`, whether `Conflicts=shutdown.target`
keeps the oneshot off the way down, and whether `NRestarts` and `ActiveEnterTimestampMonotonic` read
the way the script assumes on a crash-looping unit are all unverified. Nor is the watch itself: each
test is decided on its first look, because the stubbed uptime does not move. Nothing short of real systemd — `systemd-nspawn`, or a
privileged container with it as pid 1 — can answer any of them, and
[`install-path-gap.md`](../project/install-path-gap.md) is where the case for that lives. This is a
second argument for it: the recovery net is the code that must work when everything else does not, and
the code least amenable to a test.

Until then this mechanism is verified on a bench board or not at all, and the way to do that is to
give a board a release whose `robotd` cannot start.

None of it has run on hardware, where the interesting path needs a board whose release cannot start.
