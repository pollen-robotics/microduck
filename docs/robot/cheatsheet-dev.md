# Cheat sheet — dev board

The commands that only make sense on a board set up by
[`install-dev.md`](install-dev.md). Everything a robot needs day to day is in
[`cheatsheet.md`](cheatsheet.md).

## The dev channel

Install what a branch last built on CI:

```
sudo robotctl update apply --ref <branch> daemon
```

```
sudo robotctl update apply --ref main daemon
```

`--version` pins an exact release instead. Give one of them unless you genuinely mean "go to
stable".

**`apply daemon` with no `--ref` installs the latest *stable* release, which on a dev board is
usually a downgrade.** It is not "install the newest thing"; it is "install what the stable channel
offers". Right after a branch merges, that stable release is still older than everything you have
been testing — and if it predates a daemon that now has a unit file on the board, its `ExecStart`
points at a binary the older release does not contain, the restart fails, and the update rolls back.
That is the gate working, but the command that caused it looked like the obvious one.

The tag `daemon-dev-<branch>` moves with the branch, so there is no version number to copy. The
version *inside* stays unique per build — `0.1.0-dev.42.c719ec8` — so two builds of the same branch
are never confusable. `--ref main` is how a board goes back to mainline without leaving the dev
channel; a plain `apply daemon` leaves it, since a prerelease sorts below its release and there is
no separate opt-out step.

A merge does not publish instantly: CI has to build `main` before `--ref main` resolves to it.

```
gh run list --branch main
```

## Release candidates

What `release.yml` published to staging and nobody has promoted yet — what a canary robot should run
before a promotion:

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

## After an update — the part that bites

- **`robotd`, `configd` and `padd` restart during the update. `updaterd` and `btd` restart 5 seconds
  after it replies** — the first cannot restart itself mid-update, and the second may be carrying the
  reply. So a `btd` fix is live a few seconds later, with no manual step. Reconnect and it is there.
- **If one of those two restarts does not happen, the next `updaterd` start fixes it.** Except
  `updaterd` itself, which reports the disagreement rather than restarting itself. Run the apply again
  for that one: it answers `already_current`, names the daemon that is not running it, and schedules
  the restart. `sudo systemctl restart updaterd` does the same by hand.
- **A board running an `updaterd` older than 0.4.0 has none of that** and keeps both on the old binary
  until you restart them. One update fixes it, and only the update after that behaves.
- **`robotctl update apply` reports `already_current` and installs nothing** if you ask for the version
  a board already has — but it is no longer inert. It checks which daemons are running that release and
  restarts the ones that are not, naming them in `stale`. So it *is* the command to reach for when a
  fix looks absent: either it fixes it, or `stale` is empty and the fix was never in that release.

The symptom is a fix that is definitely installed and definitely not working. Ask which release each
daemon is running:

```
robotctl health
```

The `units` block prints one line per daemon with the release its process was launched from, and a
warning naming the restart when that disagrees with what is installed. `build unknown` means that
daemon is running and published no identity — an old build, most likely; restart it and it will
answer. `restarting` means it exited and systemd is bringing it back: ordinary for a few seconds
after an update, and a daemon that cannot start at all if it stays that way.

If a daemon is genuinely stale, restart it — this should not be necessary, so it is worth reading the
journal for why it was:

```
sudo systemctl restart configd
```

`updaterd` is the one that never fixes itself:

```
sudo systemctl restart updaterd
```

Editing the board's `updater.toml` is not needed — the restart set comes from the units the release
ships. `../design/restart-order.md` is the full sequence, step by step.

## From a laptop — build here, install on the board

Skipping CI entirely: build on your machine and install over ssh, in about a minute.

```bash
scripts/dev-push.sh radxa@<board>
```

The result is an ordinary gated update, so the restart traps above still apply.
[`dev-push.md`](dev-push.md) has the setup, the container build, `--dry-run`, the first push to a
board below 0.5.0, and what to do when it fails.

## From a laptop — drive with a pad in your hands

The pad in your hands, the robot on the bench, and nothing installed on either: `padd` is an
ordinary client, so it can run from a clone against a forwarded socket.

Stop the one on the robot first, or two processes fight over the sticks:

```bash
sudo systemctl stop padd
```

Forward the socket and leave it open:

```bash
ssh -L /tmp/robotd.sock:/run/robotd.sock radxa@192.168.1.42
```

Then from this clone, in another terminal:

```bash
cargo run -p padd -- --socket /tmp/robotd.sock
```

`systemctl start padd` puts the robot's own back. This is also where `padd`'s flags are worth
having — `--max-linear` (m/s), `--max-angular` (rad/s), `--max-head` (radians) and `--deadzone`,
which exists because analogue sticks rarely rest at exactly zero and the robot creeps without it.
The unit runs with the defaults, so trying other values means running the binary yourself, here or
on the board:

```bash
sudo -u padd /opt/robot/daemon/current/bin/padd --max-linear 0.25
```

## From a laptop — `duckctl`

Reaching the robot over Bluetooth LE, with no network and no ssh:
[`duckctl.md`](duckctl.md) has every command.
