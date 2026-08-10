# Deployment: OS-level configuration

Status: draft · Date: 2026-07-28 · Owner: pierre

Config that belongs to the *robot image* rather than to any one service. Service units live
next to their service (`updater/systemd/`, `robotd/systemd/`); anything robot-wide is here.

| | |
|---|---|
| `updater.toml` | the config a client robot ships with, installed to `/etc/robot/updater.toml` |
| `trusted_keys/` | release public keys — the trust anchor, installed to `/etc/robot/trusted_keys/` |
| `journald.conf.d/10-robot.conf` | journal persistence and size caps |

Note on that last one, now **measured** rather than assumed: `/var/log` on this image is a zram
device, so `Storage=persistent` really is a directory in memory. It survives a clean reboot and
loses recent logs on a power cut, which is how a robot is actually switched off. The update history
under `/var/lib` is therefore the only durable record — which is what `architecture.md` §8.2
designed it to be.

> To just get a dev board working, [`docs/robot/install-dev.md`](../docs/robot/install-dev.md) is the short
> procedure. Below is the trust chain, what ends up where, and where logs go.

## Quickstart

Three ways in, in order of how much you have to type. Everything after this section is the same
thing with the reasons attached — read it when something disagrees with you, not before.

### Dev board, repository private — this is today

One command from a clone, covered step by step in
[`docs/robot/install-dev.md`](../docs/robot/install-dev.md):

```bash
export DUCK_TOKEN=github_pat_replace_with_your_token
```

```bash
./scripts/provision-board.sh radxa@192.168.1.42
```

`--no-dev-key` for a board that should only take releases, `--ref BRANCH` to provision from a
branch, `--local` to send this clone's `provision.sh` rather than fetching it, which is what makes
testing an unpushed change to it possible.

`team.dev.pub` is committed at [`dev-key/`](dev-key/), not in `trusted_keys/` — carrying it to a
board *is* the opt-in, so it must not ship with every robot. The script sends the committed copy;
`--dev-key PATH` overrides it.

### On the board, without a clone

Three commands, the first from your machine, and what `provision-board.sh` is doing on your
behalf above:

```bash
scp deploy/dev-key/team.dev.pub radxa@192.168.1.42:/tmp/
```

```bash
export DUCK_TOKEN=github_pat_replace_with_your_token
```

```bash
curl -fsSL -H "Authorization: Bearer $DUCK_TOKEN" https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/main/scripts/provision.sh -o /tmp/provision.sh && sudo DUCK_TOKEN="$DUCK_TOKEN" DUCK_DEV_KEY=/tmp/team.dev.pub sh /tmp/provision.sh
```

`provision.sh` runs `setup-board.sh`, `migrate-network.sh` and `install.sh` in order, warns for
ten seconds, reboots — your SSH session ends there — and finishes on its own. It copies the dev
key out of `/tmp` first, because `/tmp` does not survive that reboot; a key left there would
produce a board that provisions cleanly and is silently *not* a dev board, surfacing weeks later
as `--ref` being refused, which reads like a broken release.

Log back in and:

```bash
robotctl health
```

If it is still working when you get back, watch it:

```bash
sudo tail -f /var/lib/robot/provision.log
```

That log is the record of the half nobody watched, and it is a file rather than the journal on
purpose: journald persistence is configured by a drop-in inside the release being installed, so
during this exact window the journal can still be RAM-only. It ends with `DEV BOARD` only when
the key really installed, so which kind of board you ended up with is a thing you can check
rather than a thing you have to remember:

```bash
grep -c 'DEV BOARD' /var/lib/robot/provision.log
```

No `newgrp robot` on either path, and that is deliberate rather than an omission: the `robot`
group is created before the reboot, so the session you log back into already has it.

### Regular user, repository public

No token and no dev key.

```bash
curl -fsSL https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/main/scripts/provision.sh -o /tmp/provision.sh && sudo sh /tmp/provision.sh
```

```bash
robotctl health
```

Downloaded rather than piped even here: the second half runs after a reboot, so there has to be
a file left on disk for it to be. It refuses a pipe rather than stranding you halfway.

### Doing it by hand

`DUCK_NO_REBOOT=1` makes `provision.sh` stop before the reboot and tell you what to run, which
is the shape to use when you want to see each step's status block go past:

```bash
sudo DUCK_NO_REBOOT=1 DUCK_TOKEN="$DUCK_TOKEN" sh /tmp/provision.sh
```

```bash
sudo reboot
```

```bash
sudo /usr/local/sbin/robot-provision
```

The three scripts also stay independently runnable, and `provision.sh` is a thin orchestrator
with no logic of its own. One at a time the order is `setup-board.sh`, `migrate-network.sh`,
reboot, both again, `install.sh` — and then `newgrp robot`, because on that path nothing created
the group before the reboot.

## What those commands actually do

Three scripts, kept apart because they answer to different things. `setup-board.sh` is OS-level
bring-up — device-tree overlays, ONNX Runtime — which changes rarely and needs a reboot.
`install.sh` installs a signed daemon release, which happens on every update; conflating the two
would mean every update re-litigating boot configuration. `migrate-network.sh` is the one that
will not last: it exists only because Armbian's stock image ships netplan, and it gets deleted
rather than maintained the day we build an image with NetworkManager in it.

**A board does not arrive with NetworkManager**, and that is why the middle one is not optional.
Armbian's headless image runs netplan + `systemd-networkd` + `wpa_supplicant`, while `configd`
drives NM over D-Bus — so until the migration runs, `robotctl net status` reports `Unavailable`
and nothing over Bluetooth can configure wifi. Why NM rather than what the image ships is in
[`../docs/design/app-path-design.md`](../docs/design/app-path-design.md) §2.

`provision.sh` runs them in order and holds the state that has to cross the reboot — the token,
a dev-key path, and the boot id it uses to tell whether you have actually rebooted. It has no
provisioning logic of its own, on purpose: three scripts with different lifetimes should not
become one script whose parts cannot be removed separately.

`provision-board.sh` is a further layer out and runs on your machine, not the robot. It exists
for the seam in the middle: `provision.sh` reboots and finishes on its own, which is right, but
from outside that looks like an ssh session dying followed by a guess about when to log back in.
It waits for the board, streams the log, and ends on the health report — so provisioning is one
command with continuous output rather than three with a gap.

It does take the reboot, which the scripts it calls deliberately never do. That is not a
contradiction: they are single-purpose and can be run on a robot that is doing something else,
so neither can know what a reboot would interrupt. This one is only ever run on a board being
provisioned, where the reboot is the next step rather than an interruption.

The cost is that the second half runs unattended, and the thing that can go wrong there is a
loop rather than a failure — so two guards. The resume unit disables itself *before* doing any
work, so there is at most one automatic attempt: a phase 2 that dies leaves a board to look at
rather than a board retrying into the same wall. And `migrate-network.sh` is only re-run when
NetworkManager already owns wifi, meaning the cutover took and the run is just to retire the
backstop. If the backstop fired and restored netplan, re-cutting over unattended would re-arm
it, fail the same way, reboot, and go round again; it says so in the log and leaves wifi alone.
The unit file stays on disk, disabled, as a record of what ran.

The token is needed three times, and only the first two end with provisioning: fetching these
scripts, fetching the release, and then permanently — `updaterd` reads `GITHUB_TOKEN` from a
systemd drop-in on every later update check. Passing `DUCK_TOKEN` through is what makes updates
work *after* provisioning, not just during it, and `provision.sh` ends by saying which of the two
copies it removed and which one stayed. A board with no token installs fine and can then never
fetch an update, which is most of what `updaterd` is for.

It also creates the `robot` group in its first phase, which is the only reason the flow above
has no `newgrp robot` in it. `install.sh` does the same thing correctly and too late — by the
time it runs, your shell started before the group existed, and a process's groups are fixed when
it starts. Moving it ahead of the reboot means the login session you return to already has it.

**The token, and why a wrong URL is the wrong diagnosis.** `raw.githubusercontent.com` answers
**404**, not 401, for a private path with no credentials, so a missing header looks exactly like
a typo. There are two separate tokens in play: the one in your shell, which fetches scripts, and
the one `updaterd` needs to reach release assets — [that one](#-while-the-repository-is-private-a-robot-needs-a-token)
is a systemd drop-in and outlives the shell. The two meet when `provision.sh` reaches
`install.sh`: passing `DUCK_TOKEN` through is what writes the drop-in.

**`setup-board.sh`** is idempotent and never reboots on its own. The one thing it fixes that is
otherwise very hard to diagnose: Armbian ships `overlay_prefix=rk35xx`, but the RK3566 shares
device-tree overlays with the RK3568 and they are named `rk3568-*.dtbo`. With the wrong prefix
the loader finds nothing, the board boots happily, and there is simply no `/dev/ttyS2`.
`armbian-config`'s overlay editor crashes for the same reason, so the file is patched directly.

⚠ A kernel upgrade that repoints `/boot/{Image,dtb,uInitrd}` can undo it. A board that stops
seeing its motors after an `apt upgrade` needs this re-run.

⚠ Its `kernel console` status line reads `until the reboot` when the script has already fixed it
and only the running kernel is stale — `/proc/cmdline` cannot change without a reboot. Anything
else on that line is a real finding.

**`migrate-network.sh`** runs **twice**, either side of the reboot — `provision.sh` does both,
and it is worth knowing why the second matters. The first run arms a
boot-time backstop that restores netplan if `wlan0` gets no IPv4 address, so a bad cutover costs
a reboot instead of a serial cable, and the second retires it. Left armed, any later boot where
wifi is merely slow reverts the board. It takes the SSID and key from netplan itself —
`/run/netplan/wpa-wlan0.conf` if netplan generated one, otherwise the `access-points:` stanza in
`/etc/netplan/*.yaml` — so an SSH session over that same wifi survives. If it can read neither it
changes **nothing** and prints the `nmcli` commands to create the profile by hand.

**`install.sh`** needs `curl` and coreutils and nothing else — `tar` and `zstd` are linked into
`updaterd`. Idempotent, and it never overwrites an existing `/etc/robot/updater.toml`. It is two
commands rather than `curl … | sudo sh` while the repo is private, because the header goes on the
fetch and `sudo` does not pass a variable through on its own; a pipe carries neither. Everything
it takes is an environment variable, since it is normally run through a pipe where flags are
awkward:

| | |
|---|---|
| `DUCK_TOKEN` | token for a private repo — the fetch *and* the release assets |
| `DUCK_REPO` | the repository, for a fork or a test repo |
| `DUCK_REF` | the branch the scripts and trusted keys are read from; pin to a tag for a reproducible run |
| `DUCK_CONFIG_REF` | where `updater.toml` comes from. Defaults to the tag of the release being installed, and `DUCK_REF` does not change it — a config field is only understood by binaries from its own version onwards, so pairing a branch's config with the last stable binary is how `updaterd` ends up refusing to start. Set this only to test a config change with a build that understands it. |
| `DUCK_DEV_KEY` | path to `team.dev.pub` — makes this a dev board (below) |
| `DUCK_FORCE_REINSTALL` | reinstall over a live release using the release's own `updaterd` |

**How it installs itself.** An update needs the updater, and the updater arrives in an update.
The way out is one bare `updaterd` binary, published as the `updaterd-bootstrap-aarch64` asset
because a fresh board has no `zstd` to open a `.tar.zst` with. It then runs the *ordinary*
engine — same verification, extraction, atomic swap, journal entry — so the store comes out in
exactly the state the resident daemon expects and no bootstrap-only path can drift from how
later updates behave. `on_apply` and `health` are forced off for the duration, because the units
live inside the release being installed; `updaterd install` refuses to run once a release *is*
live, so that can never silently disable auto-rollback on a working robot. `--from <dir>`
installs from local files instead: the offline and factory path, and what CI uses to verify a
release before publishing it.

### Making it a dev board, so `--ref <branch>` works

A board refuses branch builds twice over: `allow_dev_keys` is false, and a trusted key only
counts as a dev key if its filename ends `.dev.pub`. Both halves are needed, they are independent
checks, and doing one without the other leaves a board that still refuses branch builds — with a
signature error that reads like a corrupt release. `DUCK_DEV_KEY` does both.

It validates before changing anything: the file must exist, must look like a minisign public key,
and `updater.toml` must already have an `allow_dev_keys` line to flip — that key is top-level, so
appending one would land it inside whichever `[table]` came last. It installs the key as
`team.dev.pub` whatever the source was called, because the `.dev.` infix is what classifies it; a
key landing under any other name is trusted as a *release* key, and branch builds would then be
accepted as reviewed.

`team.dev.pub` is committed at [`dev-key/`](dev-key/), deliberately outside `trusted_keys/` —
[`dev-key/README.md`](dev-key/README.md) explains why that is safe.

The closing report says `DEV BOARD` when this is on, and prints the two commands to undo it.
Never do it to a robot you ship.


### ⚠ While the repository is private, a robot needs a token

A private repo's release assets are unreachable without credentials — the
`releases/download/...` URL 404s even with one, so the engine resolves assets through the
release API instead. `updaterd` reads `GITHUB_TOKEN` from its environment, which on a board
means a systemd drop-in, not a shell export.

That is fine on a developer's board and **not** fine on a customer robot: a fleet-wide
credential in an image is one that leaks and cannot be rotated without reflashing, which is
the failure the tiered signing keys exist to avoid.

`install.sh` therefore writes the drop-in **only when `DUCK_TOKEN` was supplied** — mode 600,
and it says so loudly. A customer robot installs from a public artifact repository and passes
no token, so it never reaches that path. Without it `updaterd` would be installed, running,
and unable to fetch a single update, which is most of what it is for.

Artifact hosting is therefore an open decision, not a settled one —
[`../docs/design/updater-design.md`](../docs/design/updater-design.md) §6.1 has the options. The cheap one
is a second, public repository holding only signed artifacts: signatures are what make an
artifact safe to serve, not obscurity, and the source stays private.

### The trust chain

1. TLS to `raw.githubusercontent.com` for `install.sh`, `updater.toml` and the public keys.
   These cannot come from a release: nothing can be verified until the keys are present.
2. TLS to `github.com` for the bootstrap `updaterd`. **Not yet verified.**
3. That binary verifies the manifest and the artifact against the keys from (1), and
   refuses anything they do not sign.
4. The installer then compares the bootstrap binary's `sha256` against
   `current/bin/updaterd`, which came out of the verified artifact. Equal digests mean the
   binary from (2) was genuine. `release.yml` asserts the two are the same bytes, so a
   mismatch is a real finding rather than a packaging quirk.

Everything else — both unit files, the journald drop-in, `robot.conf` — is taken out of the
*installed* release rather than fetched from the repository, so it is the copy a signature
was checked against.

The residual trust is GitHub itself, which is also where the script came from; step (4)
narrows that window rather than removing it. An install that wants none of it should use
`--from` against files carried in by hand.

### Unattended updates

`updaterd` is already a resident process with a timer — `check_interval` in
`updater.toml` — so there is nothing to add to cron. What the timer is allowed to install
is `auto_apply`:

| | |
|---|---|
| `off` | never; availability is logged, a mandatory release loudly |
| `mandatory` | **the shipped default** — only a release whose `min_supported` floor says the running version must not be used |
| `all` | every available release |

A canary or bench robot that should track `staging` and install each candidate:

```bash
sudo sed -i 's/^auto_apply = .*/auto_apply = "all"/' /etc/robot/updater.toml
```

```bash
sudo sed -i 's/^tag_prefix     = .*/tag_prefix     = "daemon-staging-v"/' /etc/robot/updater.toml
```

```bash
sudo systemctl restart updaterd && journalctl -u updaterd -b | tail -20
```

The first check is 60s after start, then every `check_interval`. `auto_apply = "all"` logs
at `warn` on startup, so "why did this robot restart when nobody asked it to" is answerable
from the journal at any log level.

Don't reach for cron or a systemd timer instead. `robotctl update apply` deliberately
bypasses the known-bad guard — an operator retrying a release may have fixed the cause — so
a timer driving it would inherit the bypass and lose the protection, and one bad release
becomes an endless apply/rollback loop that re-downloads and rewrites the eMMC every
interval. `updater-design.md` §8.1.1 has the detail.

No maintenance window is needed, and that is deliberate. An unattended apply is an ordinary
apply: the preflight asks `robotd` whether it is safe to restart and whether a remote
session is live, *before* any network access, so a robot that is walking or streaming
refuses and retries at the next interval. `safeToRestart` is a better answer to "is now a
bad time" than a clock.

### What ends up where

```
/etc/robot/updater.toml                 config; never touched by an update
/etc/robot/trusted_keys/release-*.pub   trust anchor
/opt/robot/daemon/releases/<version>/   the release tree
/opt/robot/daemon/current -> releases/<version>
/etc/systemd/system/{updaterd,robotd}.service   copied out of the release
/usr/lib/sysusers.d/robot.conf          creates the `robot` group
/var/lib/robot/updater/                 lock, update log, boot counter
/usr/local/bin/robotctl -> current/bin/robotctl
/usr/local/sbin/robot-provision         provisioning, resumed after its reboot
/var/lib/robot/provision.log            what the unattended half did
/etc/systemd/system/robot-provision.service   disabled once provisioning finished
```

Unit files are **copied** rather than symlinked through `current`: read through the
symlink they would change under systemd's feet on every update, and after a rollback
systemd's view would depend on which release happened to be live at the last
`daemon-reload`. `robotctl` *is* a symlink, because it is a tool an operator invokes
rather than a file systemd caches.

Mutating operations are root-only: `allow_uids`/`allow_gids` are deliberately empty in
`updater.toml`. Membership of the `robot` group gets a process as far as *talking* to
`updaterd` — status and logs — and no further. `btd`'s uid joins the allow-list when it
exists, because "may relay an update request from the app" is a narrower claim than "is in
the robot group".

⚠ **Running `install.sh` directly, the first `robotctl health` fails and the install is fine.**
Both sockets are `root:robot` mode 0660 and `install.sh` puts the operator in `robot` — but a
process's groups are fixed when it starts, so the shell that ran the install is not in the group
it just gained. One command, in that same shell:

```bash
newgrp robot
```

There is no API to add a group to a running process, not even for root, so nothing the installer
does can fix the shell that launched it — which is why `provision.sh` creates the group in its
first phase instead, and the reboot does the work. On the direct path, `install.sh` prints that
command in its closing report and `robotctl` names it on the failure rather than sending you to
`systemctl status`, which would show two perfectly healthy daemons.

## Where logs go, and what survives a reboot

Every daemon logs to **stderr**, which systemd captures into the journal. Level is
`RUST_LOG`, set in each unit (`info`).

Two records, with deliberately different durability:

| | where | survives reboot | survives power loss | capped by |
|---|---|---|---|---|
| service logs | journald | only if configured (below) | **no** — `/var/log` is zram, below | `SystemMaxUse=200M` |
| **update history** | `/var/lib/robot/updater/update-log.jsonl` | **yes** | **yes** | 200 entries |

The update history is not in the journal on purpose. It lives in the engine's `state_dir`
under `/var/lib`, every entry is `fsync`ed as it is appended, and rewrites go through an
atomic temp-file-plus-rename with the parent directory `fsync`ed
(`updater/src/journal.rs`, `updater/src/fsutil.rs`). So "what did this robot install, and
what happened to it" survives even a robot whose journal is volatile, and it is readable
with `robotctl update log` or straight off the disk as JSON lines. That property is
verified by tests, not assumed.

Service logs need the drop-in in this directory. Install it, then:

```bash
sudo systemctl restart systemd-journald
```

Then confirm more than one boot is retained — this is the actual acceptance check, and it
only means anything *after* a real reboot:

```bash
journalctl --list-boots
```

Two or more lines means the previous boot is reachable. One line, or
`no persistent journal was found`, means logs are still RAM-only.

Read a specific service, previous boot:

```bash
journalctl -u robotd -b -1
```

### The RAM-log caveat — measured, and decided against durability

`/var/log` on this image is a **zram device**, so `Storage=persistent` gets journald a directory
that is itself in memory. Confirmed on the board rather than inferred:

```bash
findmnt /var/log
```

It survives a clean `reboot`, because shutdown writes back, and loses recent logs on a power cut
— which is how a robot is actually switched off. So service logs are best-effort by
construction, and the durable record is the update history under `/var/lib`, which is `fsync`ed
per entry and never goes through `/var/log` at all.

That is the intended arrangement rather than a gap — `architecture.md` §8.2 designed the update
log to be the thing that survives. The alternative, if a board ever needs durable service logs,
is one command and a known cost:

```bash
sudo systemctl disable --now armbian-zram-config
```

Logs then land on the eMMC and wear it. With `info` levels and the size caps above the volume is
small, which is why those levels were chosen — but it is a per-board decision, not a default to
change fleet-wide.

## Versions, for support

The question is always "what was running?", and on this robot it has **two answers at once**:
`updaterd` never restarts itself during an update (`updater-design.md` §4.1), so the running
binary legitimately lags the installed release until the next reboot. Anything reporting a single
version number is therefore misleading.

`robotctl version` reports both and flags the disagreement. It deliberately works when `updaterd`
is **down**, reporting that as a line rather than exiting — that is when someone is most likely
to run it. `--json` gives the same content for a support bundle.

Four independent places a version is recoverable, so losing one is not fatal:

1. **The startup log line**, first thing each daemon writes, at `warn` so it survives
   `RUST_LOG=warn`: version, revision, `exe` path, pid. The `exe` path is what tells you which
   release directory the running process actually came from.
2. **`robotctl version`**, over IPC.
3. **`--version`** on every binary, for when nothing is running.
4. **`version.toml`** inside each release directory, and `robotctl update list`.

`revision` is compiled in from `DUCK_REVISION` at build time (CI sets it; a laptop build honestly
reports `rev unknown, not a CI build`). Compile time, never git at runtime — a shipped robot has
no repository.
