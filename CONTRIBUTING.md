# Contributing

For working on the daemons themselves. To use a robot rather than change it, see the
[README](README.md).

## Building and testing

Needs Rust **1.89+** (stable) and nothing else. macOS and Linux both work for development; the
robot is aarch64 Linux.

```bash
cargo test --workspace
```

458 tests, no hardware, no network, no Docker. If they pass, your checkout is sound.

Those tests are also where the engine's failure paths are: a bad signature, a release that comes
up unhealthy, a post-install hook that fails, power loss between the swap and the health gate.
Each drives the real engine with the fault injected rather than a mock of it, so
`updater/tests/apply.rs` is the honest answer to "what does this actually guarantee" — more so
than anything you could run by hand.

One crate at a time, and formatting:

```bash
cargo test -p <crate>
```

```bash
cargo fmt --all
```

`configd`'s NetworkManager client and `btd`'s BlueZ client are **Linux-only**, so a host build and
a green test run say nothing about them. Lint against the board's target or the breakage ships:

```bash
RUSTFLAGS="-D warnings" cargo clippy -p configd --all-targets --target aarch64-unknown-linux-gnu
```

`scripts/board-test.sh` runs in CI against the userland we ship: it cross-compiles for the board
and executes 13 checks — rollback, tampered-artifact refusal, boot-counter recovery, socket
modes, peer-credential authorization — on Debian 13 (Trixie). `BOARD_IMAGES=` runs it against
another.

## The layout

```
duck-ipc-proto/ the wire contract
duck-control/   the control core: model · bus · IMU · observations · policy · safety
padd/           gamepad → intents — an ordinary socket client, no privileged access
updater/        engine + updaterd
robotd/         control daemon
configd/        wifi · robot name · pairing PIN · reboot · gamepad pairing
btd/            the BLE front door, plus btctl (a laptop client, never shipped)
robotctl/       the local CLI
xtask/          package · sign · promote — build tooling, never shipped
deploy/         what a robot is configured with: updater.toml, robotd.toml, trust anchor, journald
scripts/        provision-board.sh (from your machine) · provision.sh → setup-board.sh ·
                migrate-network.sh · install.sh (on the board) · board-test.sh (CI)
docs/           robot/ (using one) · design/ (how it works) · project/ (roadmap, records)
```

Services talk over unix sockets, JSON-RPC 2.0 one object per line. The contract lives in
`duck-ipc-proto`, which depends on serde and semver and nothing else — so `btd` and `robotd`
never inherit the update engine's http/tar/crypto tree.

[`docs/design/architecture.md`](docs/design/architecture.md) §1 has what each service is and why it is its own
process. [`docs/project/roadmap.md`](docs/project/roadmap.md) has what actually works today.

## Conventions

- **Comments say why, not what.** The reason a thing is the way it is outlives the code.
- **Every non-obvious decision gets a test**, and the test's comment says which failure it
  exists to prevent. The rollback paths especially: they only ever run when something else has
  already gone wrong, so they are the code most likely to be quietly broken.
- **Reach for an existing crate** before writing it yourself. Dependency count is not the thing
  being optimised; maintenance is.
- Commit trailers use `Assisted-by:`, not `Co-Authored-By:`, for AI assistance.

## Releasing

Releases are signed **in CI**, never locally. The entry point is the GitHub releases page, and the
tag decides what happens:

| you create | what CI does |
|---|---|
| a **pre-release** tagged `daemon-staging-v0.4.0` | builds, signs, verifies through the real update engine, publishes to **staging** |
| a **release** tagged `daemon-v0.4.0` | **promotes** staging 0.4.0 if it exists — the same bytes, re-signed — otherwise builds 0.4.0 directly |

Pushing either tag from a terminal does the same thing:

```bash
git tag daemon-staging-v0.4.0 && git push --tags
```

The canaried path is two steps on purpose: publish the pre-release, install it on a robot, then
create the release. Creating a release with no staging build to promote is allowed and says so in its
own notes — verified in CI, never run on a robot.

Bump the workspace version first. `xtask package` refuses a tag that disagrees with `Cargo.toml`,
which is what stops a robot reporting a version it is not running.

`gh workflow run promote --field version=0.4.0` is the same promotion without a release to create
first, and is where `min_supported` lives.

[`docs/project/ci-setup.md`](docs/project/ci-setup.md) covers key custody, the secrets, and rotation.
