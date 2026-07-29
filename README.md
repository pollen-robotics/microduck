# microduck daemon

The software that runs on the robot, and the machinery that ships it there.

A bipedal robot goes to people who are not developers, so the hard problem is not making it
walk — that already works in the prototype — but making it something you can hand to a
stranger: updates it can apply itself without bricking, a safety authority that outranks
every client, and recovery that works when the robot is already broken.

This repo is the daemons plus the update system. Start with
[`docs/architecture.md`](docs/architecture.md) for how the services fit together, and
[`docs/roadmap.md`](docs/roadmap.md) for what exists today versus what is designed.

## Getting started

Needs Rust **1.89+** (stable) and nothing else. macOS and Linux both work for development;
the robot is aarch64 Linux.

```bash
cargo test --workspace
```

272 tests, no hardware, no network, no Docker. If they pass, your checkout is sound.

The fastest way to actually *see* the update engine work is the playground, which drives
the real engine — real signatures, real atomic swaps, real rollback — against a fake remote
in a temp directory, with no daemon and no robot:

```bash
cargo run -p updater --example playground -- init /tmp/pg
```
```bash
cargo run -p updater --example playground -- publish /tmp/pg 1.0.0
```
```bash
cargo run -p updater --example playground -- apply /tmp/pg
```
```bash
cargo run -p updater --example playground -- status /tmp/pg
```

Then break it on purpose, which is the interesting half — install a release that comes up
unhealthy and watch it revert:

```bash
cargo run -p updater --example playground -- publish /tmp/pg 1.1.0
```
```bash
cargo run -p updater --example playground -- apply /tmp/pg --unhealthy
```

`--fault abort_after_swap` simulates power loss mid-update; run `recover` twice afterwards
to watch the boot counter undo a release that never confirmed healthy.

## The services

| | |
|---|---|
| `robotd` | motor control, kinematics, gait model, **safety authority**. Currently holds the pose it starts in: a real 50 Hz loop on the Dynamixel bus, plus the four `robot.*` methods the updater needs. Walking arrives in slice 2 ([`docs/robotd-design.md`](docs/robotd-design.md)). |
| `duck-control` | the control core — robot model, bus, sensing. A library, not a service: no tokio, no sockets, no systemd. |
| `updaterd` | the update engine. Resident, and deliberately independent of `robotd` — it is the recovery path, so it must work when the robot does not. |
| `mediad` | camera, audio, WebRTC gateway. **Not built yet.** |
| `btd` | BLE: wifi provisioning, naming, update trigger from the phone. **Not built yet.** |

They talk over unix sockets, JSON-RPC 2.0 one object per line. The contract lives in
`duck-ipc-proto`, which depends on serde and semver and nothing else — so `btd` and `robotd`
never inherit the update engine's http/tar/crypto tree.

```
duck-ipc-proto/ the wire contract
duck-control/   the control core: robot model · Dynamixel bus · IMU · the RobotIo seam
updater/        engine + updaterd
robotd/         control daemon
robotctl/       the local CLI
xtask/          package · sign · promote — build tooling, never shipped
deploy/         what a robot is configured with: updater.toml, robotd.toml, trust anchor, journald
scripts/        install.sh (provisioning) · board-test.sh (aarch64 checks)
docs/           architecture · update design · robotd design · roadmap · CI setup
```

## Working on the robot

Everything below assumes a **dev board**, never a customer robot.

What is running, and what is installed — these are different questions, because `updaterd`
never restarts itself during an update and so legitimately lags the installed release until
the next reboot:

```bash
robotctl version
```

```bash
robotctl update status
```

```bash
sudo robotctl update apply daemon
```

```bash
sudo robotctl update rollback daemon
```

Logs go to the journal. The startup line carries version, git revision and the release
directory the process was launched from, at `warn`, so it survives any log level:

```bash
journalctl -u robotd -b
```

The update history is separate from the journal on purpose — `fsync`ed per entry under
`/var/lib/robot/updater/` — so it survives a robot whose logs were volatile:

```bash
robotctl update log
```

Provisioning a board from scratch, and the log-retention caveats on Armbian, are in
[`deploy/README.md`](deploy/README.md).

### Testing your branch on a board

Push the branch. CI cross-compiles it, signs it with the team dev key, and publishes a
prerelease at the moving tag `daemon-dev-<branch>`. Then, on the board:

```bash
sudo robotctl update apply daemon --ref my-branch
```

```bash
robotctl version
```

```bash
sudo robotctl update rollback daemon
```

No version numbers to copy: the tag moves to whatever that branch last built, while the
version inside (`0.1.0-dev.42.c719ec8`) stays unique per build so two builds are never
confusable. A plain `sudo robotctl update apply daemon` puts the board back on the release
stream, because a prerelease sorts below its release — there is no "leave the dev channel"
step.

Nothing is relaxed for a dev build: same signature and hash verification, same health gate,
same auto-rollback. The difference is the key, and that is what keeps these builds off
customer robots — they refuse a dev key twice over (`allow_dev_keys = false`, and a trusted
key only counts as a dev key if its filename ends `.dev.pub`).

**A board has to opt in once**, which is also what stops this working on a robot that
shouldn't take dev builds:

```bash
sudo cp team.dev.pub /etc/robot/trusted_keys/
```

```bash
sudo sed -i 's/^allow_dev_keys.*/allow_dev_keys        = true/' /etc/robot/updater.toml
```

While this repository is **private**, the board also needs a GitHub token — a private repo's
release assets are unreachable without one, and `updaterd` reads `GITHUB_TOKEN` from its
environment, so exporting it in your shell does not reach the daemon. Use a drop-in with a
fine-grained, read-only token:

```bash
sudo mkdir -p /etc/systemd/system/updaterd.service.d
```

Substitute your own token in the next block — it is the only placeholder here:

```bash
sudo tee /etc/systemd/system/updaterd.service.d/token.conf > /dev/null <<'EOF'
[Service]
Environment=GITHUB_TOKEN=ghp_replace_with_your_token
EOF
```

```bash
sudo chmod 600 /etc/systemd/system/updaterd.service.d/token.conf
```

`chmod 600` because a drop-in is world-readable by default, and this one holds a credential.

```bash
sudo systemctl daemon-reload && sudo systemctl restart updaterd
```

A token on a *developer's* board is fine. A token on a customer robot is not, and is why
artifact hosting is still an open question — see `docs/updater-design.md` §6.1.

## Releasing

Releases are signed **in CI**, never locally, behind an approval gate. Cutting one is a
tag; promoting one re-signs a manifest over the *same bytes* the canary validated, with no
rebuild:

```bash
git tag daemon-staging-v0.2.0 && git push --tags
```

```bash
gh workflow run promote --field version=0.2.0
```

[`docs/ci-setup.md`](docs/ci-setup.md) covers key custody, the secrets, and rotation.

## Conventions

- **Comments say why, not what.** The reason a thing is the way it is outlives the code.
- **Every non-obvious decision gets a test**, and the test's comment says which failure it
  exists to prevent. The rollback paths especially: they only ever run when something else
  has already gone wrong, so they are the code most likely to be quietly broken.
- **Reach for an existing crate** before writing it yourself. Dependency count is not the
  thing being optimised; maintenance is.
- Commit trailers use `Assisted-by:`, not `Co-Authored-By:`, for AI assistance.

## Status

Honest version, kept current in [`docs/roadmap.md`](docs/roadmap.md):

- **Works and is tested:** the update engine end to end — verification, atomic swap, health
  gate, auto-rollback, boot-counter recovery, first-install bootstrap, release packaging and
  signing. Releases are cut and signed in CI; a real one has been published and installed
  through the engine.
- **The dev channel works.** Every branch push publishes a signed build installable with
  `--ref`, verified against the real repository, and refused by a customer-robot config.
- **Open:** artifact hosting. This repo is private, and a robot without a token cannot
  download from it (§6.1). Dev boards have tokens; the fleet will need a public
  artifact-only repository or an object store. Blocks hardware bring-up, not development.
- **`robotd` holds a pose.** A real 50 Hz loop on the Dynamixel bus, and `robot.health`
  now means *the loop is meeting its deadline* rather than *it ticked once* — which is what
  makes the updater's auto-rollback gate on something real. Walking is slice 2.
- **Not started:** `mediad`, `btd`, the phone app, the SDK, safety authority.
- **Runs on aarch64 Linux, emulated.** `scripts/board-test.sh` runs in CI: it
  cross-compiles for the board and executes 13 checks — rollback, tampered-artifact
  refusal, boot-counter recovery, socket modes, peer-credential authorization — on
  Debian 13 (Trixie), the userland we ship. `BOARD_IMAGES=` runs it against another.
- **Never run on real hardware.** No board yet, so nothing here says anything about motor
  timing, control-loop jitter on a non-RT kernel, thermals or eMMC behaviour. Two specifics:
  `systemctl restart` in `on_apply` has never met real systemd (containers have none), and
  the 30s health-gate timeout is a guess until someone measures a real boot.
