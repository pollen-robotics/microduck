# Build here, install on the board

The loop between changing a line and watching the robot run it, with no push, no CI run and no
tag. One command from a clone of this repo builds for the board, signs the result and installs it
over ssh — about a minute on an incremental build, against several for a push plus a CI run.

`scripts/dev-push.sh` is that command. Everything below is a flag on it.

## Once, before the first push

Three things, and then never again.

**The board has to be a dev board.** The artifact is signed with the team dev key, so a customer
robot refuses it exactly as it refuses `--ref`. [`install-dev.md`](install-dev.md) is how a board
becomes one.

**The dev signing key** goes at `~/.duck-keys/team.dev.key` — the secret half of the key CI signs
branch builds with, which a team member has. Set `DUCK_DEV_SECRET_KEY` if yours lives elsewhere.

**A toolchain that can build for the board.** Either install the cross-compiler:

```bash
cargo install cargo-zigbuild --locked
```

```bash
brew install zig
```

Or install nothing and pass `--docker` to every `dev-push.sh` below, which needs only a running
Docker daemon. That path is slower to start and is the one to reach for before you have a board at all —
see [build in a container](#build-in-a-container-instead).

## The loop

Name the robot, and let the push find it:

```bash
scripts/dev-push.sh --name duck-c51b
```

Or once per shell:

```bash
export DUCK_ROBOT=duck-c51b
```

```bash
scripts/dev-push.sh
```

The name is the one `duckctl scan` lists and `robotctl system set-name` sets — the same
`DUCK_ROBOT` that tool reads ([`duckctl.md`](duckctl.md)). Its address is
asked for over Bluetooth — the robot's own `net.status` answers with it — and then cached, so only
a push that cannot reach the cached address goes back to the radio. That is what makes a new DHCP
lease, a reflash or a different network cost nothing to follow.

The ssh user is `radxa`. If yours is not:

```bash
export DUCK_BOARD_USER=pierre
```

An address still works, and skips the radio entirely:

```bash
scripts/dev-push.sh radxa@192.168.1.42
```

```bash
export DUCK_BOARD=radxa@192.168.1.42
```

It cross-compiles the workspace, packages the same artifact a release does, signs it with the dev
key, copies it to `~/duck-sideload` on the board, and applies it there through
`robotctl update apply --from`. Then it waits for the daemons to report the new release:

```
==> building 0.5.1-dev.local.1763400000.g7fc1444 for the board (zigbuild)
==> packaging
==> signing with /Users/you/.duck-keys/team.dev.key
==> copying to radxa@192.168.1.42:/home/radxa/duck-sideload
==> applying on radxa@192.168.1.42
==> 0.5.1-dev.local.1763400000.g7fc1444 is live on radxa@192.168.1.42
==> checking every daemon is running it
    current -> 0.5.1-dev.local.1763400000.g7fc1444
    [ok] robotd
    [ok] configd
    [ok] padd
    [ok] updaterd
    [ok] btd
    [ok] mediad
    [ok] tofd
==> every daemon on radxa@192.168.1.42 is running 0.5.1-dev.local.1763400000.g7fc1444
```

`updaterd` and `btd` restart five seconds after the apply replies, so those two lines take a moment
to arrive. A `[--] padd published nothing` is not a failure — it is also what a stopped or
deliberately disabled daemon looks like, and what `mediad` looks like on a board with no camera.

A `[--] tofd published nothing` means a release from before `tofd` published an identity at all —
one more push fixes it — rather than a sensor that is missing: `tofd` runs whether or not one is
fitted.

On the board, that version is what `robotctl version` reports:

```bash
robotctl version
```

Two pushes of the same dirty tree never collide — the version carries the push's timestamp, not
just the commit. The tree is expected to be dirty here.

**This is an ordinary update.** The signature, the artifact hash, compatibility, the health gate
and auto-rollback all run: a build that does not come up is reverted and the board is back on what
it was running. Going back on purpose is the ordinary command too:

```bash
sudo robotctl update rollback daemon
```

## Watching what you just pushed

From a second terminal, before the push, so the restart shows up in it:

```bash
ssh radxa@192.168.1.42 'journalctl -f -u robotd -u configd -u btd -u padd'
```

`-u updaterd` on its own is the update itself — each phase, the health gate, and the restarts it
schedules:

```bash
ssh radxa@192.168.1.42 'journalctl -f -u updaterd'
```

A panic in a daemon lands there with a full backtrace: nothing is stripped from these binaries, so
the frames have names.

```bash
ssh radxa@192.168.1.42 robotctl health
```

Whether the control loop is up, and which release each daemon is running.
[`cheatsheet.md`](cheatsheet.md) has the rest of what to ask a robot.

### More than `info`

Each unit sets `RUST_LOG=info`. To see a daemon's `debug!` lines, override it with a drop-in — on
the board, `robotd` here:

```bash
sudo mkdir -p /etc/systemd/system/robotd.service.d
```

```bash
sudo tee /etc/systemd/system/robotd.service.d/log.conf > /dev/null <<'EOF'
[Service]
Environment=RUST_LOG=debug
EOF
```

```bash
sudo systemctl daemon-reload && sudo systemctl restart robotd
```

The drop-in lives in `/etc`, so it outlives every later push. Take it back off when you are done:

```bash
sudo rm /etc/systemd/system/robotd.service.d/log.conf
```

```bash
sudo systemctl daemon-reload && sudo systemctl restart robotd
```

## Verify without installing

```bash
scripts/dev-push.sh --dry-run radxa@192.168.1.42
```

Builds, signs, copies, and then does everything the real apply does except the swap: signature,
hash, compatibility, the extraction, and the check that no installed unit is left pointing at a
binary this build does not contain. It stops before `current` moves, so the board keeps running what
it was running and no daemon restarts.

## Build in a container instead

```bash
scripts/dev-push.sh --docker radxa@192.168.1.42
```

No zig, no `cargo-zigbuild`, and no board to copy libudev from. On an Apple Silicon Mac the
container's target is the host, so nothing is cross-compiled; on an x86 laptop it runs under
emulation and the script says so. Same artifact, and `--dry-run` and `--bootstrap` work the same.

The two modes keep separate `target/` directories, so switching between them costs one full
rebuild. The default is faster day to day and is what CI uses.

## Re-install what is already on the board

The push leaves the artifact in `~/duck-sideload` on the board, so it can be installed again without
building anything:

```bash
sudo robotctl update apply daemon --from ~/duck-sideload
```

The same command takes any directory holding a release — a USB stick, for instance. Each push
replaces that directory rather than adding to it.

## Compile for the board without a robot

To check that a change builds for the target — the C-linked crates, the Linux-only paths — without
a board in reach:

```bash
cargo board --bins
```

`cargo board` is `cargo zigbuild` with the board's target and glibc floor, defined in
`.cargo/config.toml`. It needs the same zig toolchain as the default push, and `padd` needs the
libudev copy that the first push fetches from a board. `--docker` needs neither.

## The first push to a board below 0.5.0

`apply --from` needs API version 7, which first ships in 0.5.0. A board running anything earlier
has an `updaterd` that cannot be asked to use it, and refuses the call rather than quietly
installing from its configured source instead. Deliver that release once the ungated way:

```bash
scripts/dev-push.sh --bootstrap radxa@192.168.1.42
```

That stops `robotd` and gives up the health gate for that one install. Every push after it is the
ordinary command.

## When it does not work

**`no dev signing key at ...`** — the board verifies this artifact like any release, so it has to
be signed. Get `team.dev.key` from a team member, or point `DUCK_DEV_SECRET_KEY` at it.

**`cargo-zigbuild is not installed`** — install it and zig, or use `--docker`.

**`no libudev.so.1 on <board>`** — the first push copies that library off the board to link `padd`
against. A board that is not up yet cannot provide it; `--docker` does not need it.

**Linker errors about libudev, after reflashing the board** — the copy is cached. Drop it and the
next push fetches a fresh one:

```bash
rm -rf ~/.cache/duck-cross/aarch64
```

**`apply failed (exit 2)`, with `robotctl` and `updaterd` reporting an API mismatch** — the board's
installed release predates `apply --from`. Use `--bootstrap` once.

**`preflight check failed: SideloadDir: ... is not there for updaterd`** — the release is under
`/tmp` or `/var/tmp`, which happens if `DUCK_SIDELOAD_DIR` or a hand-written `--from` points there.
`updaterd.service` sets `PrivateTmp=yes`, so the daemon has its own `/tmp` and `/var/tmp` and
neither is the one your shell copied into. Any other path works; the default, `~/duck-sideload`,
is one. On a board whose release predates that check, the same mistake reads as
`no manifest for version <version> in <dir>` — for a directory whose `ls` lists exactly that
manifest.

**`verification failed: signature did not verify against any of N usable trusted key(s)`** — reads
like a corrupt release, and usually means the board is not a dev board: the dev key never landed, or
`allow_dev_keys` is off, either of which leaves that key out of the usable set. On the board:

```bash
grep -c 'DEV BOARD' /var/lib/robot/provision.log
```

`0` means the key is missing; [`install-dev.md`](install-dev.md) has both halves of the fix.

**`could not reach <name> over Bluetooth`** — the robot has to be advertising and in range for
the name path to resolve an address.

```bash
duckctl scan
```

Nothing listed is a robot that is off, out of range, or already connected to a phone. Give the
address instead and the radio is not involved:

```bash
scripts/dev-push.sh radxa@192.168.1.42
```

**`<name> answered over Bluetooth but has no wifi address`** — it is up but not on a network, so
there is nothing to ssh to. Join one over the same radio:

```bash
duckctl --name duck-c51b wifi connect <ssid> --psk <passphrase>
```

**`still <address>, which ssh could not reach`** — the address never moved, so whatever ssh is
unhappy about is something else. A reflashed board is the usual one; see the host keys below.

**ssh refuses to connect after a reflash** — the board regenerated its host keys.

```bash
./scripts/provision-board.sh radxa@192.168.1.42 --forget-host-key
```

**`the release is live but not everything is running it`** — the swap happened and the health gate
passed, but a daemon is still on the old binary, which reads as a fix that did not work. The script
names which ones. On the board:

```bash
robotctl health
```

The `units` block names the release each daemon is running.

```bash
journalctl -u updaterd -b | tail
```

Look for `restart scheduled`, or the reason there was none.

```bash
sudo systemctl restart updaterd
```

This should not have been necessary, so it is worth reading the journal for why it was.
[`cheatsheet-dev.md`](cheatsheet-dev.md) has the restart traps in full, and
[`../design/restart-order.md`](../design/restart-order.md) is the sequence step by step.

## Settings

| | |
|---|---|
| `DUCK_ROBOT` | The robot, by name. Its address is found over Bluetooth and cached. |
| `DUCK_BOARD_USER` | The ssh user on the board, for the name path. Default `radxa`. `duckctl ssh` reads it too. |
| `DUCK_PIN` | The robot's pairing PIN, if it is not the factory `000000`. Read by `duckctl`. |
| `DUCK_BOARD_CACHE` | Where resolved addresses are cached. Default `~/.cache/duck/boards`. |
| `DUCK_BOARD` | The board, by address, instead of an argument. `radxa@192.168.1.42`. |
| `DUCK_DEV_SECRET_KEY` | The dev signing key. Default `~/.duck-keys/team.dev.key`. |
| `DUCK_SIDELOAD_DIR` | Where the artifact lands on the board. Default `~/duck-sideload` there. Never under `/tmp` or `/var/tmp`: `updaterd` has private copies of both and would read those. |
| `DUCK_CROSS_SYSROOT` | The cached libudev copy. Default `~/.cache/duck-cross/aarch64`. |

## What this deliberately does not do

Nothing a release does for provenance. The version carries a timestamp rather than a tag, the
artifact is signed with a key customer robots refuse, and nothing is published — so nobody else can
install what you just ran. Cutting a release is still a tag and `release.yml`
([`../../README.md`](../../README.md)).
