# Cheat sheet

`robotctl`, which runs on the robot. Every command here was taken from `--help` on the branch that
ships it, not from memory.

Read-only commands need no privilege. Anything that **changes** the robot needs `sudo` (or a user in
`--allow-user`/`--allow-group` for `configd`, `allow_uids`/`allow_gids` in `updater.toml` for
`updaterd`).

Branch builds, release candidates and the restart traps after an update are in
[`cheatsheet-dev.md`](cheatsheet-dev.md) — they need a dev board. The same robot over Bluetooth from
a laptop, with no network and no ssh, is [`duck-btctl.md`](duck-btctl.md).

## On the robot — `robotctl`

### The first thing to run

```
robotctl version
```

What every daemon is *running* against what is *installed*, plus warnings when they disagree. Run
this before believing any other diagnosis — a daemon serving old code after an update looks exactly
like a bug in the fix you just shipped. See "After an update" below.

```
robotctl health
```

Hardware and software in one report. Exits non-zero when the robot is unhealthy or unreachable, so
it can gate a script — a hot motor or a pinned component is reported, not judged, and does not
affect the exit code. `--json` for a support bundle.

### Watching the loop

```
robotctl monitor
```

What a client asked for beside what was actually applied, with the reason named when they differ —
safety clamps things constantly, and "the stick is forward and the robot is still" is unreadable
without that. A limit is spelled out rather than named: `deadman — no intent arrived recently,
velocity zeroed`.

Also on the frame: every joint measured against what it was commanded, the IMU's projected gravity
and the fall verdict drawn from it, and the achieved loop rate as a trace so a stutter that has
already recovered is still visible. Projected gravity is the only IMU quantity on this stream —
upright is about `[0, 0, -1]`, and it is what `fallen` is decided from. The stale-read counters and
the rest of the sensing live in `robotctl health`.

The bottom border names the policy that is loaded — the `.onnx` files, and whether a standing
network is configured at all — because `walk` is a mode two releases with different gaits both
report. A robot with no policy says so, and one whose policy would not load says that instead,
which the stream's `held` cannot distinguish.

`q` quits; `↑`/`↓` scroll the joint list on a window too short for all of it; `u` switches the
angles between degrees and radians; `p` opens the pad's raw input stream — every evdev report from
the gamepad, with the gaps between them, which is the only place a stalled radio is visible
([pair a gamepad](pair-a-gamepad.md#when-it-drops-while-you-are-driving)). Angles are degrees on screen — joints, head and the yaw rate.
Redirected or piped it prints one line per tick instead, so `> run.log` and `| grep FALLEN`
behave, and those numbers stay radians whatever the screen is set to. The joint vectors are in
`--json`, which carries the whole state, one object per line:

```
robotctl monitor --json --hz 50 > run.jsonl
```

### Power to the joints (`robotd`)

```
sudo robotctl robot init
```

```
sudo robotctl robot relax --yes
```

`init` powers the joints and ramps to the home pose over about two seconds — **it moves every joint**,
so have the robot on its stand. It needs no policy, and it is what the gamepad's Start does on its way
to driving, so by hand it is a bench thing.

`relax` cuts power and **the robot collapses** if nothing holds it, which is why it wants `--yes`. It
is the only way back to limp short of pulling the plug: pressing Start again stops the policy and
keeps the robot standing, and `robot.stop` zeroes the velocity while still standing.

Both go through `robotd`, which owns the motor bus. `robotd init` — the subcommand — still exists for
a robot whose daemon is not running, and it needs the daemon stopped, because two writers on one UART
corrupt each other's replies:

```
sudo systemctl stop robotd && sudo /opt/robot/daemon/current/bin/robotd init && sudo systemctl start robotd
```

A fallen robot refuses `init`: the fall gate holds it limp on purpose. Stand it up by hand first.

### Gamepad (`configd`)

```
robotctl pad status
```

```
sudo robotctl pad pair
```

```
sudo robotctl pad pair 78:86:2E:BB:13:28
```

```
sudo robotctl pad forget 78:86:2E:BB:13:28
```

Pairing is once per pad and has a page of its own —
[`pair-a-gamepad.md`](pair-a-gamepad.md): which button puts a pad in pairing mode, adding a second
pad without forgetting the first, and what to do when it will not bond (`Privacy = device` is the
answer more often than anything else).

`padd.service` runs from boot and drives whatever pad connects, so pairing is the only step. On the
pad: **Start** toggles the policy — nothing moves until it is on — **Y**/triangle switches the sticks
between body and head, **B**/circle stops.

`pad status` answers two questions separately, because a connected pad and a dead driver look
identical from the outside:

```
pad     Xbox Wireless Controller 78:86:2E:BB:13:28  connected
padd    active — driving whatever pad connects
```

To drive with non-default limits, stop the service first or two processes fight over the sticks:

```
sudo systemctl stop padd
```

```
sudo -u padd /opt/robot/daemon/current/bin/padd --max-linear 0.25
```

When the link itself is the suspect, watch it live — `robotctl monitor`, then `p`. For a verdict
over a window instead, copy the measurement over from a clone of this repo:

```
scp scripts/pad-link-test.sh radxa@<board>:/tmp/
```

Drops already in `padd`'s journal — no pad needed, and it answers immediately:

```
sudo sh /tmp/pad-link-test.sh --history
```

Or measure it now, keeping the sticks moving for the whole two minutes:

```
sudo sh /tmp/pad-link-test.sh
```

It counts drops against the kernel's own reason for each, and times the gaps between the pad's
input reports — the failure `padd` cannot see, where the link stays up and the robot walks on a
stale command. [`pair-a-gamepad.md`](pair-a-gamepad.md#when-it-drops-while-you-are-driving) reads
the numbers.

When two boards behave differently with the same pad, the difference is in the stack under it:

```
scp scripts/pad-stack-report.sh radxa@<board>:/tmp/
```

```
sudo sh /tmp/pad-stack-report.sh
```

Kernel, BlueZ, controller firmware, LE or BR/EDR, and the pad's own firmware revision — printed and
saved to `/tmp/pad-stack-<host>-<when>.log`. `--fingerprint` prints only the values that must match
between two boards, for `diff`.
[`pair-a-gamepad.md`](pair-a-gamepad.md#is-this-board-running-the-same-stack-as-that-one) has the
comparison.

### Wifi (`configd`)

```
robotctl net status
```

```
robotctl net scan
```

```
sudo robotctl net connect <ssid> --psk <passphrase>
```

```
sudo robotctl net connect <ssid> --psk-stdin
```

```
sudo robotctl net forget <ssid>
```

`--psk-stdin` keeps the passphrase out of `ps`, which shows a `--psk` argument to every user on the
box for the lifetime of the command. Prefer it on anything shared.

Joining a network **disconnects the robot from the one it is on**, so an ssh session over wifi will
drop. That is the operation working. A scan takes a few seconds — it waits for the radio to sweep
rather than returning the previous scan's results.

### Identity and power (`configd`)

```
robotctl system info
```

```
robotctl system pin
```

```
sudo robotctl system set-name <name>
```

```
sudo robotctl system set-pin <six-digits>
```

```
sudo robotctl system reboot
```

Out of the box a robot calls itself `duck-` plus four characters derived from its own serial, so two
boards flashed from the same image still look different in a phone's Bluetooth list. Renaming takes
effect over Bluetooth within a few seconds — no restart — but a phone has to scan again to see it.

The PIN is what a phone authenticates with over Bluetooth. The factory default is `000000`, which
authenticates anyone who has read this repository.

### Updates (`updaterd`)

```
robotctl update status
```

```
robotctl update check daemon
```

```
sudo robotctl update apply daemon
```

```
sudo robotctl update rollback daemon
```

```
robotctl update log
```

```
robotctl update watch
```

The component is `daemon` — one component covering every binary. `apply daemon` installs what the
stable channel offers; branch builds and release candidates need
[`cheatsheet-dev.md`](cheatsheet-dev.md).

### Switching without a download

To something the board already has unpacked. No network involved:

```
sudo robotctl update select daemon 0.1.4
```

```
sudo robotctl update rollback daemon
```

```
sudo robotctl update reset-to-golden daemon
```

`select` activates an installed release, `rollback` goes to the previously installed one, and
`reset-to-golden` goes to the never-pruned known-good one.

And refusing to move at all:

```
sudo robotctl update pin daemon 0.1.4
```

```
sudo robotctl update pin daemon
```

The second form unpins.

### When `updaterd` itself will not start

Everything above goes through `updaterd`, so none of it works when `updaterd` is the daemon that is
down. Check which one it is:

```
systemctl status updaterd robotd btd configd
```

Then go back to golden without it:

```
sudo robot-rescue --dry-run
```

```
sudo robot-rescue --reboot
```

`--dry-run` says what it would do and changes nothing. Without `--reboot` it swaps the release and
prints the reboot command rather than running it: every daemon execs through `current`, so nothing
picks up the swap until it restarts, and a robot that is standing should be caught first.

It declines, and says why, when no golden is configured or when `current` is already golden — if the
daemons are failing on golden itself, a rollback is not the answer and the journal is:

```
journalctl -b -u robotd -u updaterd -u btd -u configd
```

### The robot may have done this already

Three minutes into every boot, a timer asks whether the release brought its daemons up, and falls back
to golden if it did not. So a robot that rebooted on its own and is running an older release than you
installed has probably rescued itself. What it did:

```
robotctl update log
```

The entry reads as a rollback, with the daemon that failed named in its reason. To see the decision
being made rather than its result:

```
journalctl -b -u robot-boot-check
```

```
sudo robot-boot-check --dry-run
```

It acts once. A second rescue is refused while the first is still on record — `updaterd` clears that
when it next starts, so being refused means the daemons did not come up on golden either, and the
answer is the journal rather than another reboot. Past it, if you have read the journal and decided:

```
sudo robot-rescue --force --reboot
```

### Three things that are easy to get wrong

**`rollback` needs a predecessor, but an update creates one.** A freshly provisioned board has
exactly one release, so `rollback` right then has nothing older to go to and says so. Auto-rollback
is *not* affected: applying a release unpacks it alongside the current one and only then moves
`current`, so by the time the health gate runs there are two, and the release you came from is the
target. `rollback_target` picks the highest installed version below `current` that the journal has
not already recorded as bad — so a board with one release is fully protected from the moment it
takes its first update.

The one genuinely unprotected install is the bootstrap itself, which has nothing before it by
definition. `golden` would cover that, and it is deliberately unset until 1.0.0 exists — so
`reset-to-golden` reports honestly that none is configured rather than doing something surprising.

**`version` shows the live release per component, not the release store.** It will never list two
versions, however many are unpacked. Ask the store directly:

```
ls -l /opt/robot/daemon/releases/ /opt/robot/daemon/current
```

**`apply --version` needs the release to still exist upstream; `select` does not.** Releases
carrying known-bad builds get deleted from GitHub, so `apply --version 0.1.3` fails on purpose,
while `select 0.1.3` still works on a board that already unpacked it. The asymmetry is deliberate:
no new board can acquire a broken release, and a board that has one keeps its escape hatch.

### Installing with no network

Sideloading, factory install, or rescuing a board whose `updaterd` is too old to accept the release
that fixes being too old. See [`install-dev.md`](install-dev.md) — it is `updaterd install --from`,
and the `--force` variant has conditions worth reading before you use it.

### Logs

```
journalctl -u configd -b --no-pager | tail -40
```

```
journalctl -u btd -f
```

Swap in `robotd` or `updaterd`. `-f` follows; `-b` is this boot only.

The startup line carries version, git revision and the release directory the process was launched
from, at `warn`, so it survives any log level.

The update history is separate from the journal on purpose — `fsync`ed per entry under
`/var/lib/robot/updater/` — so it survives a robot whose logs were volatile:

```
robotctl update log
```

### Tab completion

`install.sh` sets this up in `/etc/bash_completion.d/`, as a loader that asks the binary for its own
completions — so they follow the installed release instead of going stale when an update adds a
command. For a shell it did not cover, or for a build you are running straight out of `target/`:

```
eval "$(robotctl completions bash)"
```

`zsh`, `fish`, `elvish` and `powershell` work in place of `bash`.

