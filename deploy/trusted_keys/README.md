# Trusted public keys

The robot's trust anchor. `scripts/install.sh` copies these into
`/etc/robot/trusted_keys/`, which is `trusted_keys_dir` in
[`../updater.toml`](../updater.toml). An artifact is acceptable if its signature verifies
against **any** key here.

| | |
|---|---|
| `release-1.pub` | signs every release and promotion today |
| `release-2.pub` | first rotation target if CI or `release-1` is compromised |
| `release-3.pub` | last resort; its private half should never touch a networked machine |

**All three ship from the first image, and that is the whole point.** A robot can verify
only against the set baked into it, so a robot carrying one key whose private half is
later lost or leaked cannot be given a replacement over the air — it has to be re-flashed
by hand. Shipping the spares now is free; retrofitting them is impossible. See
[`../../docs/project/ci-setup.md`](../../docs/project/ci-setup.md) for custody.

**Public keys are not secrets.** Committing them is correct and intended: they are exactly
what has to be public for anyone to verify what we sign. The private halves live in
`~/.duck-keys` and a password manager, and only `release-1`'s is in CI.

**`team.dev.pub` is deliberately not in this directory.** Every file here is trusted as a
release key on every robot, and a robot that trusts the dev key that way installs anything anyone
on the team builds. `updaterd` carries the dev key built in instead, usable only on a robot made
a dev board with `sudo robotctl configure --allow-dev-daemon on`
([`updater-design.md`](../../docs/design/updater-design.md) §5.4). It is committed at
[`../dev-key/`](../dev-key/).

## Adding a rotation key

Generate it, drop the `.pub` here, and it reaches robots on their next install. Note the
asymmetry: a *new* key only becomes trusted on robots imaged after it was added, so
rotation protects the fleet going forward and cannot rescue robots already in the field.
That is why the spares exist up front.

```bash
cargo xtask keygen --kind release --name release-4 --out ~/.duck-keys
```
