# The install path has no test

Status: open · Date: 2026-08-05, revised 2026-08-07 · Owner: pierre

Four bugs in a row reached a board, all in the install path, none caught by 418 tests or by
`board-test.sh`. This records why, and what would actually close it. Written the same day, while
the reasons are still concrete.

A second section covers two findings that *look* like the same thing and are not: version skew
between a daemon that is already running and a release that has just been installed. Neither would
be caught by any amount of install testing, and they need their own fixes — kept here rather than in
their own document because anyone investigating one will arrive believing it is the other.

**Revision note.** All four bugs are fixed. What is still open is the *testing* gap the title names —
no test installs a real artifact — plus four smaller decisions marked below. One change since the
first draft moves enough of this document to call out here: `updaterd` and `btd` are now restarted a
few seconds *after* the update replies (`RESTART_AFTER_REPLYING`), and the release's own `updaterd` is
proved to start before the commit (`updaterd --self-test`). Two claims below were written against the
old behaviour and are now false — both are corrected in place, and the second section's premise in
particular no longer holds.

## What got through

All four landed within a day, all while installing `btd` and `configd` onto a dev board. (The count
read "three" until §4 was added below it.)

**1. `on_apply` restarts several units in one command.** `systemctl restart robotd configd` fails as
a whole if *either* unit is unknown — and fails without restarting the one that exists. A release
introducing a new daemon therefore could not restart anything, because the unit file arrives *with*
that release and nothing installed it. The rollback reason was
`not healthy within 30s: unreachable`, which names neither the unit nor the command.

Latent rather than fatal, since a board keeps its own `/etc/robot/updater.toml`.

Fixed twice, and the second one is the real fix. First, defensively: units are restarted one at a
time and a unit systemd does not know is skipped. Then properly: **`hooks/postinstall` installs the
release's units.** The engine has always had a post-install hook point — `extract → [pre_install] →
swap → [post_install] → apply → health gate` — which runs after `current` moves, so `ExecStart`
resolves, and before the restart, so `on_apply` finds a unit that exists. Nothing used it; only
`scripts/install.sh` ever copied units out of a release, so every new service needed a manual step on
every board, forever. That the mechanism existed and was unused was pointed out in review, not found
by me.

**2. The artifact did not contain the new units.** The packaging workflows name every shipped file
with an explicit `--include`, and the units were added to `install.sh` without being added there.
Found by hand: `ls current/systemd/` on the board.

**3. The artifact did not contain the new binaries.** The Package step copies binaries with an
explicit `cp` per binary. `cargo board --bins` built `btd` and `configd`; nothing staged them.
`btd.service` failed with `203/EXEC`, which reads as a broken daemon rather than an incomplete
artifact.

Bug 3 is the same class as bug 2, in the same file, **two commits after a test was added to stop
bug 2 recurring** — that test checked units and not the binaries they exec. Each fix was correct
and each was too narrow, which is the pattern worth attacking rather than the bugs.

**4. `on_apply`'s restart list lives on the board, so a new daemon is never restarted.**  ·
**measured** The most expensive one, and the one already written down here as "latent rather than
fatal".

`[component.daemon.on_apply].units` is read from `/etc/robot/updater.toml` — the *operator's* file,
which `install.sh` deliberately preserves. `configd` was added to the shipped `deploy/updater.toml`
in the branch that introduced it, so a board set up before that keeps `units = ["robotd"]` forever.
Every `configd` release therefore swapped the binary and left the old process running.

What made it cost two hours rather than two minutes is the shape of the failure:

- the update reports **success** — the swap happened, the health gate passed, nothing failed;
- `robotctl update apply` then reports **`already_current`** and does nothing, so the obvious
  recovery command is a no-op;
- the daemon keeps answering, on old code, so it looks like the fix was wrong rather than absent.
  Four wifi fixes were verified as broken against binaries that were never running.

`robotctl version` *does* diagnose this, in as many words: "configd is running X but the installed
daemon release is Y … either the restart did not happen, or it failed". Nobody ran it. A diagnostic
that exists and is not reached for is worth as much as one that does not exist, which is an argument
for the update itself noticing rather than for more diagnostics.

**Fixed** (`engine.rs::units_to_restart`): the restart set is derived from the release's
`systemd/*.service` files, unioned with whatever the config names, minus `updaterd` and `btd` — which
moved from a config convention enforced by a test into a `NEVER_RESTART` constant in code, because
they are properties of what those daemons are rather than operator choices. A board whose
`updater.toml` predates a daemon now restarts it anyway, and needs no edit. What follows is the
reasoning, kept because it is the argument for where such lists belong.

**Derive the restart set from the release, not from the board.** A release ships
`systemd/*.service`, so it already states which units it provides — the same realisation that made
`hooks/postinstall` the right answer for *installing* them. The engine can restart what the release
ships, minus the two documented exclusions (`updaterd`, which is performing the update; `btd`, which
may be the transport it was requested over). Then a release that adds a daemon restarts that daemon
on every board, with no operator edit and no way for a board's config to be silently out of date.

Two smaller options, **both still open**, worth noting because they are not alternatives to the above
so much as companions: `apply` could gain a `--force` that re-runs the hooks and the restart on an
already-current release (there is precedent — `install --force` exists for the same class of
chicken-and-egg), and `already_current` could compare *running* revisions rather than only the
installed one, so it stops being a no-op precisely when something is wrong.

Neither is urgent now that the restart set is derived from the release: the situation they exist to
dig you out of is much harder to reach. `already_current` still compares only the installed version.

**Fixed, and this paragraph used to say otherwise.** `btd` is excluded from the *in-flight* restart
because restarting it drops the BLE connection carrying the update's own progress stream — that part
was and remains right. The conclusion drawn from it was not: this said a `btd` fix "needs a manual
restart or a reboot … a real gap in a phone-driven flow", and it does not any more. The exclusion
expires the moment the outcome is on the wire, so `btd` and `updaterd` both restart five seconds later
via `systemd-run --on-active=5s` (`RESTART_AFTER_REPLYING`). A client sees its outcome and then a
dropped connection, which for BLE is an ordinary reconnect.

What survives is the *observability* half: `robotctl version` still cannot see `btd`, because `btd`
serves no socket, so there is nothing to ask. That matters less now that nothing silently keeps `btd`
on old code, but it is still the one daemon whose running revision cannot be checked.

### A consequence worth knowing: units outlive the release that installed them

`hooks/postinstall` installs the units a release ships and, by design, leaves them behind on a
rollback — the alternative is recording what was added so a revert can undo it, and the hook's own
comment argues that is not worth it because the next successful update reinstalls whatever it ships.

That reasoning holds for a rollback. It does not hold for a **downgrade to a release that predates a
daemon**: the unit stays, its `ExecStart` names a binary the older release does not contain, and the
daemon fails with `203/EXEC`. Once that daemon is also in `on_apply`'s restart set, the failed restart
fails the *update*, which reverts.

Observed exactly this way: `robotctl update apply daemon` on a board running a dev build resolved to
stable `0.2.0`, which predates `configd`; `configd.service` could not start; the engine rolled back
and said so. The outcome is right — a board should not silently downgrade below the release that
introduced a daemon it is now running — but nothing states the rule, and the error names a systemd
failure rather than the cause.

Worth deciding rather than leaving as emergent behaviour, and **still open**: whether preflight should
refuse a target that lacks a binary some installed unit execs, so the refusal arrives before the swap
and names the real reason. Today the only downgrade guard is `Error::WouldDowngrade`, which fires on
`Latest` alone — `Exact` and `Ref` bypass it deliberately, and `Ref` is precisely how this was
observed.

## Why the existing tests could not have caught them

Not an accusation of the tests; they cover what they claim. The point is what nothing covers.

| | covers | does not |
|---|---|---|
| `board-test.sh` | binaries executed on real aarch64 Linux from the *build directory*: engine behaviour, socket modes, `SO_PEERCRED`, the layered authorisation, `setup-board.sh` against a stubbed `systemctl` | unpacking an artifact, installing units, starting services |
| `xtask` tests | the workflow YAML vs `install.sh`, and unit `ExecStart` vs staged binaries | whether the *built artifact* matches either — it reads source files |
| `updater` tests | engine, journal, verification, rollback, with fakes | `install.sh` at all |
| `shipped_config_is_safe_for_a_client_robot` | `deploy/updater.toml`'s content | that the config is installable |

So: **no test takes a real artifact and installs it.** Every check either runs a binary that was
never packaged, or reads a source file that describes packaging without observing it. The two
`xtask` tests I added are strictly better than nothing and still the weaker form — they assert that
two files agree with each other, not that the thing they produce is correct.

## What would close it

Roughly in order of cost.

**A. Assert the artifact's contents.** Run `xtask package` in a test, then inspect the tarball:
every unit named by `install.sh` is present, and every unit's `ExecStart` binary is present. Cheap,
and strictly stronger than the two current tests because it observes the artifact instead of the
YAML that builds it. **Would have caught bugs 2 and 3.**

Still open in the form described. There are now *three* source-reading tests rather than two — a
`every_hook_in_the_repo_is_packaged` was added so `hooks/postinstall` cannot silently stop shipping —
but all three still assert that `.github/workflows/*.yml` agrees with `scripts/install.sh`. They cover
the drift class; none of them observes a tarball.

**B. Install the artifact in a container, with `systemctl` stubbed.** Extend `board-test.sh`: unpack
into a fake root, run `install.sh` *and* `hooks/postinstall` against it, and assert what landed
where — `/etc/systemd/system/*.service`, `/usr/lib/sysusers.d/`, the `robotctl` symlink, the state
directory. `setup-board.sh` is already tested this way, so the pattern and the stub exist. Catches
bugs 2 and 3 *and* file-placement regressions in `install.sh`, which nothing tests today.

Still open, and untouched. `board-test.sh` has a comment reading "the first install, which is the path
`scripts/install.sh` takes on a bare board" — the line beneath it runs `updaterd install --from`, not
`scripts/install.sh`. Nothing in the repository executes `scripts/install.sh` or the real
`hooks/postinstall`; the engine's hook tests use a stub hook built by `test-support`.

The postinstall hook makes this more valuable, not less: it is now a second thing that places files
on a board, it runs unattended on every update rather than once by hand, and its failures are inside
the update gate. A hook that installs a unit wrongly is worse than an installer that does, because
nobody is watching when it runs.

**C. Real systemd in a container.** `systemd-nspawn`, or a privileged container with systemd as
pid 1. Full fidelity: units actually start, `on_apply` actually restarts, the health gate actually
gates. **Would have caught bug 1** — the only one A and B miss. Real work: cgroup and privilege
setup in CI, and slow.

**D. A board as a self-hosted runner.** Highest fidelity, and the only thing that ever tests the
motor bus, the radio and the timings. Ops cost, and a single point of failure for CI.

## Suggested first step

**A, then B.** Together they cover two of the three bugs and remove the "files agree with each
other" weakness, for a fraction of C's cost. B is the better value of the two because it exercises
`install.sh`, which is 500 lines that nothing currently runs.

C is worth revisiting when `on_apply` grows again — it is the only option that tests the restart and
the gate, and it is now also the only way to observe the postinstall hook doing its real job
(enabling and starting a unit), which no stub can show.

D is a separate conversation, and probably follows M4 rather than preceding it.

## A second, separate problem: version skew on the dev channel

Found in the same session and easily confused with the above, but no amount of install testing
would catch either — in both cases the artifact was complete and correct.

The shape was always the same, and **the sentence that used to open this section is no longer true.**
It read: "`updaterd` never restarts itself during an update (§4.1), so it keeps running the old binary
until the next reboot, while everything else on the box moves to the new release immediately." That was
the root cause of both instances below, and it is fixed — `updaterd` now restarts itself five seconds
after the update replies. The skew window is seconds, not "until someone reboots".

Read the two cases with that in mind. Neither is a live incident any more; what remains is that both
were *diagnosed* badly, and one of the two fixes is still worth having on its own merits:

- Case 1's `serde(default)` fix is **not** made redundant by the shorter window. Version skew is only
  the commonest way to hit it; any newer-parser-older-sender pair does, and the fields that caused it
  still have no defaults.
- Case 2's handshake fix is **mostly** made redundant, and drops from "the recovery command itself
  stops working" to "a five-second window during which it does". It stays proposed on design grounds,
  not urgency.

Two instances, both real, both cost about an hour.

### 1. A required field added to `HealthResult`

`main` added `consecutive_stale_blocks` to the IMU section and released 0.2.0 from it. A branch that
had merged `main` *before* that did not send the field. `robotd` came up perfectly — served the
socket, loaded both policies, ran the loop at 50 Hz — and the resident 0.2.0 `updaterd` rejected its
reply for a missing field. `RobotIo::health` maps an unparseable answer to `Health::Unreachable`, so
the gate reported `not healthy within 30s: unreachable` about a robot that was entirely healthy, and
reverted a good release.

Fixes, both small, and **both still open**:

- **`#[serde(default)]` on new `HealthResult` fields**, so a newer `updaterd` can still parse an
  older `robotd`. Every `--ref` install of a branch predating a health-field addition hits this
  otherwise, which is the entire dev workflow.

  Half-true today, and the wrong half is the one that bit. Every *top-level* `HealthResult` field
  carries `#[serde(default)]`, including `imu`. The field that actually caused this —
  `consecutive_stale_blocks` on the nested `ImuHealth` — does not, and neither do `LoopHealth`'s or
  `BusHealth`'s. So the incident reproduces verbatim the next time a field is added to a nested health
  struct, which is exactly how it happened the first time.
- **A distinct `Health::Incompatible`** rather than reusing `Unreachable`, so the reason reads
  "answered in a shape this updaterd does not understand" instead of implying the robot is down.
  Pure diagnostics, and it would have found the above in a minute.

  Not done. `Health` still has four variants and `SocketRobotClient::health` still collapses an
  unparseable reply to `Unreachable`. (`Error::Incompatible` exists and is a different thing — it is
  about a *release* being refused, not a reply being unreadable.)

### 2. `API_VERSION` skew between `robotctl` and `updaterd`

`robotctl` is a symlink into `current`, so it follows the installed release. `updaterd` does not. So
the moment a release changes `API_VERSION`, the two are **guaranteed** to disagree until `updaterd`
restarts — and the command that stops working is `robotctl update apply`, which is exactly the one
you would use to get out of it.

Demonstrated by ordinary use rather than by contrivance: install branch A, install branch B while
waiting for CI, and every `robotctl` call fails with `client speaks API v2, daemon speaks v3`. The
escape is to invoke a `robotctl` from a release directory whose version matches the running daemon,
or to restart `updaterd` — neither of which is discoverable from the error.

The handshake at least *caught* it and named both versions, which is more than case 1 managed.

Proposed fix, **still open**, and the reason it is not already done is that it is a protocol-policy
decision rather than a bug:

- **`hello` should refuse only when the client is *newer* than the daemon.** A v3 daemon can serve a
  v2 client perfectly, because v3 only *added* methods — refusing that direction costs the ability
  to recover and buys nothing. Client-newer-than-daemon must stay a hard failure: there the client
  may ask for something that genuinely is not there.

  The handshake is still an exact `!=`. `API_VERSION` has since reached 5 — v5 added `pad.*`, which
  is purely additive and still bumps — and the failure was observed at v4-against-v3, so this keeps
  recurring on every additive bump, just for seconds at a time now instead of until a reboot.

That change would also make `API_VERSION` mean what it should — "the newest contract I understand"
rather than "the only contract I will speak" — and additive protocol growth would stop being a
breaking change for every client on the box.

### Why these are not install-path bugs

Nothing in options A–D above would have caught either. The artifact was correct both times; what was
wrong was the *pair* of versions running at one moment, which only exists on a machine that has been
updated. Option C (real systemd in a container) would catch the *health-gate* consequence of case 1
if the container also ran an older `updaterd`, but constructing that skew deliberately is a different
kind of test — closer to a compatibility matrix than to an install test, and worth keeping separate.
