# Cheat sheet

Every command here was taken from `--help` on the branch that ships it, not from memory. Two tools:

- **`robotctl`** runs *on the robot*, over unix sockets. The full surface.
- **`btctl`** runs *on a laptop*, over Bluetooth LE. A deliberately small subset — it is a test
  stand-in for the phone app, not a product.

Read-only commands need no privilege. Anything that **changes** the robot needs `sudo` (or a user in
`--allow-user`/`--allow-group` for `configd`, `allow_uids`/`allow_gids` in `updater.toml` for
`updaterd`).

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

`q` quits; `↑`/`↓` scroll the joint list on a window too short for all of it. Redirected or piped
it prints one line per tick instead, so `> run.log` and `| grep FALLEN` behave. The joint vectors
are in `--json`, which carries the whole state, one object per line:

```
robotctl monitor --json --hz 50 > run.jsonl
```

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
sudo robotctl update apply --ref <branch> daemon
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

The component is `daemon` — one component covering every binary.

**`apply daemon` with no `--ref` installs the latest *stable* release, which on a dev board is
usually a downgrade.** It is not "install the newest thing"; it is "install what the stable channel
offers". Right after a branch merges, that stable release is still older than everything you have been
testing — and if it predates a daemon that now has a unit file on the board, its `ExecStart` points at
a binary the older release does not contain, the restart fails, and the update rolls back. That is the
gate working, but the command that caused it looked like the obvious one.

So on a dev board, name what you want:

```
sudo robotctl update apply --ref main daemon
```

```
sudo robotctl update apply --ref <branch> daemon
```

`--ref` installs what that branch last built on CI, which is the whole dev workflow. `--version` pins
an exact release. Give one of them unless you genuinely mean "go to stable".

To install a **release candidate** — what `release.yml` published to staging and nobody has promoted
yet, which is what a canary robot should run before a promotion:

```
sudo robotctl update apply --staging daemon
```

```
sudo robotctl update apply --staging --version 0.3.0 daemon
```

A candidate is signed with the release key like any release and carries the version it will be
promoted under. What makes it unreachable without the flag is that it is flagged as a prerelease, and
a plain `apply` skips those so no robot drifts onto a build nobody has validated. `--staging` is that
filter's only opt-in, it applies to the one command, and it leaves nothing switched on afterwards.

A merge does not publish instantly: CI has to build `main` before `--ref main` resolves to it.
`gh run list --branch main` says whether it is done.

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

## After an update — the part that bites

Three things are true at once, and together they cost an afternoon if you do not know them:

- **`btd` is never restarted by an update.** Deliberate: restarting it drops the BLE connection
  carrying the update's own progress stream. So a `btd` fix needs a manual restart or a reboot.
- **`configd` used to be restarted only if the board's `/etc/robot/updater.toml` listed it** — that
  file belongs to the operator and is preserved across installs, so a board set up before `configd`
  existed kept `units = ["robotd"]` and silently ran the old binary. Fixed: the restart set now comes
  from the units the release ships. A board running an older `updaterd` still has the old behaviour
  until it restarts, because `updaterd` never restarts itself.
- **`robotctl update apply` then reports `already_current` and does nothing**, so the obvious recovery
  command is a no-op.

The symptom is a fix that is definitely installed and definitely not working. `robotctl version` says
so in as many words. To check by hand:

```
pgrep -a configd
```

```
sudo readlink /proc/$(pgrep -x configd)/exe; readlink /opt/robot/daemon/current
```

The command line always says `current/bin/configd` because that is what it was exec'd with; the
`/proc/<pid>/exe` link is the release it is *actually* running from. If those disagree, the process
predates the swap.

```
sudo systemctl restart configd
```

```
sudo systemctl restart btd
```

Editing the board's `updater.toml` is no longer needed — the restart set is derived from the release.
The one thing that still requires a manual step is `updaterd` itself, which never restarts itself, so
the fix above only takes effect once it has:

```
sudo systemctl restart updaterd
```

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

## From a laptop — `btctl`

Built from a clone of this repo. It is an *example*, not a binary, which is why every invocation says
`--example`:

```
cargo run -q -p btd --example btctl -- --name <robot-name> info
```

Or install it once, at the cost of it being a snapshot that does not follow the branch:

```
cargo install --path btd --example btctl
```

```
btctl --name <robot-name> info
```

### Commands

```
btctl scan
```

```
btctl --name <robot-name> info
```

```
btctl --name <robot-name> status
```

```
btctl --name <robot-name> health
```

```
btctl --name <robot-name> wifi status
```

```
btctl --name <robot-name> wifi scan
```

```
btctl --name <robot-name> wifi connect <ssid> --psk <passphrase>
```

```
btctl --name <robot-name> wifi forget <ssid>
```

```
btctl --name <robot-name> name <new-name>
```

```
btctl --name <robot-name> reboot
```

### Global options

- `--name <robot-name>` — connect by advertised name. Without it, the first robot found wins. Worth
  giving always: it skips a slow fallback tier that tries every already-connected peripheral on the
  Mac, earbuds included.
- `--pin <six-digits>` — defaults to `000000`. `robotctl system pin` on the robot shows the real one.
- `--verbose` — print every line sent and received. The first thing to add when something hangs.

### Anything not wrapped above

```
btctl --name <robot-name> call <method> '<json-params>'
```

```
btctl --name <robot-name> call update.check '{"component":"daemon"}'
```

Useful for the refusal boundary, which is worth knowing: motor control, `update.select`,
`update.pin`, `system.pairingPin` and `updaterd`'s private questions to `robotd` are **refused by
`btd` itself** and never reach a daemon. They come back as error code 14, "not available over
Bluetooth". That is a security boundary, not a missing feature — `docs/app-path-design.md` §3.1.

## Building and checking

```
cargo test -p <crate>
```

```
cargo fmt --all
```

`configd`'s NetworkManager client and `btd`'s BlueZ client are **Linux-only**, so a host build and a
green test run say nothing about them. Lint against the board's target or the breakage ships:

```
RUSTFLAGS="-D warnings" cargo clippy -p configd --all-targets --target aarch64-unknown-linux-gnu
```
