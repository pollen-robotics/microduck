# CI setup

Status: draft · Date: 2026-07-28 · Owner: pierre

One-time setup for the release pipeline. See [`updater-design.md`](../design/updater-design.md)
§5.4 for key custody and §16.3 for the staging → stable model.

## Decision: two keys, two triggers, and no gate on this plan

**Decided 2026-07-29.** Branch pushes are signed with `team.dev`; tagged releases and
promotions are signed with `release-1`. Both keys live in CI.

| trigger | workflow | key | reaches a customer robot |
|---|---|---|---|
| push to any branch | `dev.yml` | `team.dev` (repo secret) | **no** — `allow_dev_keys = false` there, and the trusted filename must end `.dev.pub` |
| tag `daemon-staging-v*` | `release.yml` | `release-1` (`release` env secret) | not until promoted — published as a prerelease |
| manual promotion | `promote.yml` | `release-1` | **yes** |

This split is clean in a way an earlier proposal was not: keys never cross *within* the
staging → stable path, so promotion still re-signs a manifest over identical bytes (§16.3).
An artifact signed by `team.dev` could not be promoted, because `promote` points `sig_url`
at the staging artifact's existing signature — which is why dev builds stay dev builds
rather than becoming release candidates.

### What was intended, and why it is not there

The plan was to gate `release-1` behind the `release` environment's required-reviewers rule.
It cannot be created:

```
HTTP 422: Failed to create the environment protection rule.
Please ensure the billing plan supports the required reviewers protection rule.
```

Tag protection was checked as a substitute and is also unavailable:

```
403: Upgrade to GitHub Pro or make this repository public to enable this feature.
```

Required reviewers, deployment branch policies, branch protection and rulesets are all
Team/Pro features on a *private* repository, and `pollen-robotics` is on the free plan. The
`release` environment exists with zero protection rules.

### The accepted risk, stated plainly

**Anyone with push access can read `release-1`.** Scoping it to the `release` environment
stops a workflow that does not declare that environment from seeing it, but any collaborator
can author one that does. "Used only for releases" is therefore a convention among people who
already trust each other, not an access control — the workflow file is not a boundary.

This was accepted deliberately: the team is small and mutually trusted, no robot has left the
building, and the alternative (signing every release by hand) buys nothing today against a
threat that does not yet exist.

**Revisit when either becomes true**, because the cost changes sharply and the failure is the
one this design cannot undo — a leaked key means shipping a `release-2`-signed update to every
robot, and any robot that misses it trusts the compromised key forever:

- a robot is in someone's home, or
- someone with push access is not someone you would hand the signing key to directly.

The fix at that point is upgrading the org to GitHub Team, which keeps this split and adds the
gate; or moving `release-1` signing back to a laptop.

## The tiering (unchanged, and still the thing that bounds damage)

Whatever is decided above, what limits the cost of a compromise is which key is reachable
from where:

| key | in CI | role |
|---|---|---|
| `release-1` | **not currently** — see above | signs every release and promotion |
| `release-2` | no | first rotation target if CI or `release-1` is compromised |
| `release-3` | no, ideally never on a networked machine | last resort |
| `team.dev` | intended, dev workflow only | branch builds; cannot touch a customer robot, because `allow_dev_keys` is false there |

All **public** keys go into every robot image from the start — a robot can only verify
against the set baked into it, so this is the only chance to make rotation possible
without physically re-flashing.

## Secrets and variables

GitHub Secrets are **write-only**: once set, nobody — including you — can read them back.
They are a *deployment copy*, never storage. The password manager remains the system of
record; losing it means the key is gone and every robot trusting it can never be signed
for again.

**Scope them to the `release` environment, not to the repository.** A repository secret is
readable by every workflow job in the repo; an environment secret is readable only by a job
declaring that environment. On this plan that difference stops an unrelated workflow from
seeing the key, and nothing more (see above) — but it is strictly better and costs nothing:

```bash
gh secret set MINISIGN_SECRET_KEY --env release < ~/.duck-keys/release-1.key
```

```bash
gh secret set MINISIGN_PASSWORD --env release
```

The second prompts, so the passphrase never lands in shell history or a transcript.

**Secrets** (encrypted, not readable back). Current state:

| name | scope | value | set |
|---|---|---|---|
| `MINISIGN_SECRET_KEY` | `release` env | `~/.duck-keys/release-1.key`, both lines | ✅ |
| `MINISIGN_PASSWORD` | `release` env | the passphrase for `release-1` | ✅ |
| `MINISIGN_DEV_SECRET_KEY` | **repo** | `~/.duck-keys/team.dev.key` | ✅ |

`MINISIGN_DEV_SECRET_KEY` is repo-scoped on purpose: every branch push signs with it, so
gating it behind an environment would mean the dev workflow declaring one meant for
`release-1`. It needs no passphrase secret — a dev key is unencrypted so CI can sign
non-interactively, which `xtask keycheck` confirms and calls correct for a dev key and wrong
for a release key.

**Variables** (plain, readable — a public key is not a secret):

| name | value |
|---|---|
| `MINISIGN_PUBLIC_KEY` | the key line of `~/.duck-keys/release-1.pub` |

The public key is used by `release.yml` to verify a release through the robot's own code
path before publishing it. Keeping it as a *variable* rather than a secret is
deliberate: treating a public key as secret invites confusion about which half is which.

Do **not** add `release-2` or `release-3`. Their entire value is being absent from here.

## The `release` environment

Both `release.yml` and `promote.yml` declare `environment: release`. Create it under
Settings → Environments and add **required reviewers**.

Without it, anyone who can push a `daemon-staging-v*` tag can sign for the whole fleet.
With it, reaching the signing key needs a second person's approval — which recovers most
of what local signing would have given, at the cost of one click per release.

Fork pull requests never receive secrets, so the key is unreachable from contributor PRs
regardless.

## Where the key is handled

Exactly one step per workflow writes the key to disk, and it is removed immediately:

```
umask 077
printf '%s' "$MINISIGN_SECRET_KEY" > "$RUNNER_TEMP/secret.key"
cargo run -p xtask -- sign --dir dist --key "$RUNNER_TEMP/secret.key"
shred -u "$RUNNER_TEMP/secret.key" || rm -f "$RUNNER_TEMP/secret.key"
```

Written to a file rather than passed as an argument, because a key on a command line is
visible in the process list to anything else on the runner.

`release.yml`'s verification step deliberately needs **no** key: `xtask package` emits a
second manifest with a bare-filename URL (for `LocalDir`), and `xtask sign` signs both in
one pass. Re-signing to verify would mean handling the signing key twice in one job for
no benefit.

## Cutting a release

```
git tag daemon-staging-v0.2.0 && git push --tags
```

`release.yml` then cross-builds for aarch64, packages, signs, verifies through the real
engine, and publishes a **prerelease**. Nothing reaches robots on `stable` yet.

After a canary robot has run the on-device checks, promote:

```
gh workflow run promote --field version=0.2.0
```

`promote.yml` re-signs a stable manifest carrying the **same artifact bytes** staging
validated — no rebuild; the sha256 is checked during promotion and again on the robot.
The stable release ends up self-contained (manifest, signature, artifact, bootstrap
binary), so the staging release is deleted once promotion succeeds. Add
`--field min_supported=0.2.0` only when remediating a bad release (§8.1); it forces
robots below that version to update without waiting for a client.

## Rotating a key

If `release-1` or CI is compromised:

1. Replace `MINISIGN_SECRET_KEY` / `MINISIGN_PASSWORD` with `release-2`'s.
2. Publish a release signed by `release-2`. Robots already trust it — that is why both
   public keys shipped from the first image.
3. Remove `release-1.pub` from `trusted_keys_dir` in a subsequent release, so the
   compromised key stops being accepted.
4. Generate a replacement third key so a spare still exists:
   `cargo xtask keygen --kind release --name release-4 --out ~/.duck-keys`

Step 3 lags step 2 on purpose: revoking the old key before every robot has taken the
new-signed release would strand any robot that missed it.
