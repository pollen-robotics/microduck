# The team dev key

`team.dev.pub` — the public half of the key CI signs branch builds with, so
`robotctl update apply --ref <branch>` works on a dev board.

**`updaterd` compiles this file in** (`updater::verify::BUILTIN_DEV_KEY`), always as a dev-only
key: every robot carries it, and it counts only on a robot made a dev board with
`sudo robotctl configure --allow-dev-daemon on`. Nobody copies it onto a board for that.
[`updater-design.md`](../../docs/design/updater-design.md) §5.4 owns the mechanism.

**Not in [`../trusted_keys/`](../trusted_keys/), deliberately.** Every file there is copied onto
every robot by `scripts/install.sh` and trusted as a *release* key, so a robot would install
anything anyone on the team builds. Provisioning still sends this file (`provision-board.sh`,
`--no-dev-key` declines), which is the path for an `updaterd` that predates the built-in key.

**Committing a public key gives nothing away.** Signing a build needs the private half, which
lives in `~/.duck-keys` and never leaves it.

Regenerating it, if the private half is ever lost — the new public half reaches dev boards with
the release built from it, since it is compiled into `updaterd`:

```bash
minisign -R -s ~/.duck-keys/team.dev.key -p team.dev.pub
```
