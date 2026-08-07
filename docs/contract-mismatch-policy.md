# What to do when the other side speaks a contract you do not

Status: **decision wanted, nothing implemented** · Date: 2026-08-07 · Owner: pierre

Three open questions, all the same shape: two things on one board disagree about a contract, and the
code has to choose between refusing and coping. None is a bug report — each is a policy call that has
been made implicitly, in three different directions, by three pieces of code that never compared
notes.

Lifted out of `install-path-gap.md` (§2.2 and the `203/EXEC` consequence) plus one found while
fixing the health-reply parsing.

## 1. `hello` refuses a client that is merely *older*

`updater/src/ipc.rs:493` is an exact `!=` on `API_VERSION`. Any disagreement in either direction is
`PROTOCOL_MISMATCH`.

`robotctl` is a symlink into `current`, so it follows the installed release. `updaterd` did not, until
it began restarting itself after an update — so a release that changed `API_VERSION` guaranteed the
two would disagree, and the command that stopped working was `robotctl update apply`, which is
exactly the one you would use to get out of it. Observed by ordinary use, not contrivance: install
branch A, install branch B while waiting for CI, and every call fails. `API_VERSION` has since
reached 4, and the failure was seen again at v4-against-v3.

The handshake at least *named both versions*, which is more than the health-parse failure managed.

**The proposal.** Refuse only when the client is **newer** than the daemon. A v4 daemon serves a v2
client perfectly when v4 only *added* methods; refusing that direction costs the ability to recover
and buys nothing. Client-newer-than-daemon stays a hard failure — there the client may ask for
something that genuinely is not there.

That change makes `API_VERSION` mean what it should: "the newest contract I understand", not "the only
contract I will speak". Additive protocol growth stops being a breaking change for every client on the
box.

**What needs deciding.** The proposal rests on a premise nothing enforces: **that `API_VERSION` bumps
are additive.** Today the constant is a single number with no stated rule, and this change would quietly
promise backward compatibility on every past and future bump. Either the rule gets written down and a
non-additive change gets a different mechanism, or accepting older clients is a promise the protocol
cannot keep.

Urgency dropped a lot when `updaterd` started restarting itself — the window is now seconds rather than
until someone reboots. This stays proposed on design grounds.

## 2. An unreadable `safeToRestart` is treated as safe

Found while adding `Health::Incompatible`, and it is the same class in the neighbouring method.

`SocketRobotClient::safe_to_restart` maps a reply it cannot parse to `SafeToRestart::Unreachable`
(`updater/src/robot.rs`), and its comment gives the reason:

> An answer we cannot parse is treated as unreachable rather than guessed at: guessing "safe" could
> restart a walking robot.

But `SafeToRestart::permits_restart` is `!matches!(self, No(_))` — so `Unreachable` *is* safe. The
comment describes the opposite of what the code does. The code guesses "safe", which is the outcome
the comment says must be avoided.

Both halves are individually defensible, which is why this is a decision and not a patch:

- `Unreachable` meaning safe is **correct for its own case** and documented as such: a `robotd` that
  does not answer is a `robotd` whose control loop is not running, so nothing is moving, and that is
  precisely when an update is the fix. Making absence an error would block recovery on the robots that
  need it most.
- A robot that *answers* is a different situation. It may well be walking. Reading its "no, I am
  mid-task" as "sure, go ahead" because a field was renamed is the failure the comment was written to
  prevent.

**The proposal.** Split them, the way `Health::Incompatible` splits an unreadable health reply from an
absent daemon: a parse failure on a reply that *arrived* becomes its own variant, and it does not
permit a restart.

**What needs deciding.** Whether an unreadable answer should block an update, given that the whole
design principle here is that `updaterd` must never require the thing it is recovering
(`architecture.md` §1.1). Blocking is the safe reading for a walking robot and the dangerous one for a
robot that needs fixing. A possible middle: block, but let the existing telepresence-style bypass
override it, so the decision is a human's rather than a parser's.

## 3. Nothing refuses a downgrade that orphans a unit

`hooks/postinstall` installs the units a release ships and, by design, leaves them behind on a
rollback — the next successful update reinstalls whatever it ships, so recording what was added is not
worth it.

That reasoning holds for a rollback and not for a **downgrade to a release that predates a daemon**:
the unit stays, its `ExecStart` names a binary the older release does not contain, and the daemon
fails with `203/EXEC`. Now that such a daemon is in the derived restart set, the failed restart fails
the *update*, which reverts.

Observed exactly this way: `apply daemon` on a board running a dev build resolved to stable `0.2.0`,
which predates `configd`; `configd.service` could not start; the engine rolled back and said so.

The outcome is right. A board should not silently downgrade below the release that introduced a daemon
it is running. But nothing *states* that rule — it is emergent — and the error names a systemd failure
rather than the cause.

**The proposal.** Preflight refuses a target that lacks a binary some installed unit execs, so the
refusal arrives before the swap and names the real reason.

**What needs deciding.**

- **Where the check reads its unit list from.** `/etc/systemd/system/*.service` is what is actually
  installed and includes units this release never shipped, put there by hand or by an unrelated
  package. Reading the *previous release's* `systemd/` directory is narrower and misses precisely the
  orphan case, since the orphan outlived the release that installed it.
- **Whether it is a refusal or a warning.** A refusal makes some downgrades impossible, and a
  deliberate downgrade past a daemon boundary is a legitimate operator action — it is how you get off
  a bad release that introduced one. If it refuses, it needs an override, and the override needs to
  be discoverable from the refusal text.
- **Interaction with `Target::Exact` and `Target::Ref`.** The existing `WouldDowngrade` guard fires on
  `Latest` alone, deliberately: `Exact` is how a targeted revert works, and `Ref` always looks like a
  downgrade because a dev build is a semver prerelease. `Ref` is how this was observed, so a check
  that inherits the same exemptions would not catch the case that motivated it.

## Recommendation

Item 2 first: it is the only one where the current behaviour is unsafe rather than merely unhelpful,
and the code contradicts its own comment, so one of the two is wrong today whatever gets decided.

Then item 3, which needs the least new machinery and turns an emergent rollback into a stated rule.

Item 1 last. It is the most defensible change and now the least urgent, and it should not land before
the additivity rule it depends on is written down.

## Not implemented here

No code, on purpose. Item 1 changes what the daemon accepts from every client, item 2 changes when an
update is allowed to proceed on a moving robot, and item 3 can make a legitimate downgrade impossible.
All three want agreement before an implementation exists to argue with.
