# The team dev key

`team.dev.pub` — the public half of the key CI signs branch builds with, so
`robotctl update apply --ref <branch>` works on a board that trusts it.

**Not in [`../trusted_keys/`](../trusted_keys/), deliberately.** That directory is copied onto
*every* robot by `scripts/install.sh`, and a robot that trusts this key installs anything anyone
on the team builds. This one is here instead, where nothing installs it by default —
`provision-board.sh` sends it, and `--no-dev-key` declines.

**Committing a public key gives nothing away.** Signing a build needs the private half, which
lives in `~/.duck-keys` and never leaves it. A board still only accepts dev builds if two
separate things are true — the key is in that board's `trusted_keys_dir`, *and*
`allow_dev_keys = true` in its `updater.toml`. Both are off on a customer robot, and neither is
changed by this file existing.

It was previously kept out of the repository entirely. That protected nothing the two flags above
do not already protect, and cost every new developer a round trip asking someone for a file.

Regenerating it, if the private half is ever lost — every existing dev board then needs the new
public half installed by hand, because a board only trusts what is already in its
`trusted_keys_dir`:

```bash
minisign -R -s ~/.duck-keys/team.dev.key -p team.dev.pub
```
