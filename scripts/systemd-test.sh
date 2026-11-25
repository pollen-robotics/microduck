#!/bin/sh
# Drive a real update against real systemd, on demand.
#
# Usage:  scripts/systemd-test.sh
#
# Needs Docker and nothing else. Not run by CI, deliberately — see the bottom of this comment.
#
# **What only this can answer.** Everything else in the tree tests the engine's decisions against a
# stub `systemctl`: which units it would restart, in what order, and what it does when one is
# missing. None of it can see whether a restart *happened*, because none of it has systemd. Three
# mechanisms are therefore untested everywhere else, and all three are what a board depends on after
# every update (docs/design/restart-order.md):
#
#   1. `on_apply` really restarts the units a release ships;
#   2. the deferred `systemd-run --on-active=5s` transient timer really fires, and really replaces
#      `updaterd` — a child process could not, because it would sit in the cgroup being killed;
#   3. `hooks/postinstall` really installs, enables and starts a unit a board has never had.
#
# And one failure only systemd can produce: a unit that installs cleanly and **cannot start**. That
# is bug 1 of `install-path-gap.md`, whose symptom on a board was `not healthy within 30s:
# unreachable` — a message naming neither the unit nor the command.
#
# **Docker rather than `systemd-nspawn`.** The plan in `install-path-gap.md` said nspawn on a Linux
# box, and rejected privileged containers *in CI*. That rejection stands; this is not CI. What
# changed the choice is the plan's own argument against putting this last: a check that can only run
# on a machine nobody develops on stops being read. Docker Desktop runs a privileged arm64 container
# with systemd as pid 1 on the laptop this is written on, which nspawn cannot, and the fidelity that
# matters here — real units, real cgroups, real transient timers — is identical.
#
# **Why not in CI.** It needs `--privileged` and the host's cgroup filesystem, which is a much
# larger thing to hand a workflow than any check here is worth, and the `board` job is already the
# slowest thing in the loop. This is the one to run when `on_apply`, the deferred restarts or the
# hook change — which is rarely, and is exactly when a stub stops being evidence.
set -eu

cd "$(dirname "$0")/.."

WORK=target/systemd-test
FIXTURE="$WORK/fixture"
# Where the tree is mounted in the container. Every path in the generated updater.toml is written
# for this, not for the host.
MOUNT=/fixture
IMAGE=duck-systemd-test
NAME=duck-systemd-test

command -v docker >/dev/null || {
    echo "docker is required: this needs systemd as pid 1, which a container provides and this" >&2
    echo "host does not." >&2
    exit 1
}

pass() { echo "    [ok] $*"; }
fail() { echo "    [FAIL] $*"; exit 1; }

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

# ── the binaries ──
#
# Built inside the same userland the container runs, reusing the image `dev-push.sh --docker`
# already defines. On an arm64 host that is a native build and the target is the host.
echo "==> building updaterd and robotctl for the container"
docker build -q -t duck-dev-build -f scripts/dev-build.Dockerfile scripts/ >/dev/null
docker run --rm --platform linux/arm64 \
    -v "$PWD:/src" -w /src \
    -v duck-dev-cargo-registry:/usr/local/cargo/registry \
    -e CARGO_TARGET_DIR=/src/target/docker \
    duck-dev-build \
    cargo build --release -p updater -p robotctl --bins >/dev/null
BIN=target/docker/release

# ── the fixture ──
#
# One to start on, one to move to, then one per injection.
echo "==> minting the signed releases"
rm -rf "$FIXTURE"
mkdir -p "$WORK"
cargo run -q -p test-support --example systemd-fixture -- "$FIXTURE" \
    --updaterd "$BIN/updaterd" \
    --robotctl "$BIN/robotctl" \
    --postinstall hooks/postinstall \
    --prefix "$MOUNT" \
    1.0.0 1.1.0 1.2.0 1.3.0:sysusers 1.4.0:missing-user 1.5.0:broken-unit | sed 's/^/    /'

# `local_dir` serves the newest version it can see, so what is offered is decided by what has been
# copied in. One at a time, and the harness controls when.
cp "$FIXTURE"/r/1.0.0/* "$FIXTURE/published/"

# ── the container ──
#
# systemd as pid 1 needs the host's cgroup tree and privileges. `/lib/systemd/systemd` rather than
# `/sbin/init`, which Debian's slim images do not provide.
echo "==> booting systemd"
docker build -q -t "$IMAGE" -f scripts/systemd-test.Dockerfile scripts/ >/dev/null
cleanup
docker run -d --name "$NAME" --privileged --cgroupns=host \
    -v /sys/fs/cgroup:/sys/fs/cgroup:rw \
    -v "$PWD/$FIXTURE:$MOUNT" \
    "$IMAGE" /lib/systemd/systemd >/dev/null

waited=0
until state="$(docker exec "$NAME" systemctl is-system-running 2>/dev/null || true)"
      [ "$state" = running ] || [ "$state" = degraded ]; do
    waited=$((waited + 1))
    [ "$waited" -lt 30 ] || fail "systemd did not come up: ${state:-no answer}"
    sleep 1
done
pass "systemd is pid 1 (${state})"

in_container() { docker exec "$NAME" sh -c "$1"; }

# A test container is not a robot anybody uses: keep its updaterd out of the usage counts.
in_container "mkdir -p /etc/systemd/system/updaterd.service.d && printf '[Service]\nEnvironment=MICRODUCK_TELEMETRY=0\n' > /etc/systemd/system/updaterd.service.d/telemetry.conf"

# ── the first install, which is a board's bootstrap ──
#
# Run from the tree rather than from a release, because on a bare board nothing is installed yet.
# `install` forces `on_apply` and the gate off — but the hook still runs, and installing the units is
# the hook's job, so this is also the first observation: does postinstall bring up a daemon a board
# has never had?
echo "==> installing 1.0.0 (the bootstrap path)"
in_container "$MOUNT/bin/updaterd --config $MOUNT/updater.toml install --from $MOUNT/published" \
    > "$WORK/install.log" 2>&1 || { sed 's/^/    /' "$WORK/install.log"; fail "install failed"; }

in_container "test -f /etc/systemd/system/fake-robotd.service" \
    || fail "postinstall did not install a unit the board had never seen"
in_container "systemctl is-active --quiet fake-robotd" \
    || fail "postinstall installed fake-robotd but it is not running"
in_container "systemctl is-active --quiet updaterd" \
    || fail "postinstall installed updaterd.service but it is not running"
in_container "test -f /run/btd/identity.json" \
    || fail "the btd stand-in did not publish an identity, so nothing here can be observed"
pass "postinstall installed, enabled and started units on a board that had none"

live() { docker exec "$NAME" readlink "$MOUNT/opt/daemon/current" | sed 's|releases/||'; }
main_pid() { docker exec "$NAME" systemctl show -p MainPID --value "$1" | tr -d '\r'; }

[ "$(live)" = 1.0.0 ] || fail "current is $(live), expected 1.0.0"
pass "1.0.0 is live"

# `robotctl` from the live release, so the client and the daemon are always the same build — an
# `API_VERSION` bump would otherwise refuse the call, which is the contract and not the subject here.
apply() {
    in_container "$MOUNT/opt/daemon/current/bin/robotctl update apply daemon" > "$1" 2>&1 || true
}

# ── the update, through the daemon ──
robotd_before="$(main_pid fake-robotd)"
updaterd_before="$(main_pid updaterd)"

cp "$FIXTURE"/r/1.1.0/* "$FIXTURE/published/"
echo "==> applying 1.1.0 through the running updaterd"
apply "$WORK/apply.log"
grep -q '"outcome": "applied"' "$WORK/apply.log" \
    || { sed 's/^/    /' "$WORK/apply.log"; fail "apply did not report success"; }

[ "$(live)" = 1.1.0 ] || fail "current is $(live) after applying 1.1.0"
pass "1.1.0 is live"

# 1. `on_apply` restarted what the release ships. The gate above already implies the unit came back;
#    the PID is what says it was replaced rather than left alone.
robotd_after="$(main_pid fake-robotd)"
[ -n "$robotd_before" ] && [ "$robotd_before" != "$robotd_after" ] \
    || fail "fake-robotd was not restarted (pid $robotd_before throughout)"
pass "on_apply restarted the daemon the release ships (pid $robotd_before -> $robotd_after)"

# …and did not restart the one nothing but a timer may start.
#
# `recovery-check.service` is shaped like the boot recovery net's oneshot: no `[Install]`, triggered by
# a timer, and it asks whether the release that booted came up. An update restarting it points a
# rollback check at daemons that are legitimately mid-restart, and `robot-rescue` can act on that by
# swapping to golden and rebooting. `hooks/postinstall` already declines to `enable --now` it; the
# engine used to read every `*.service` and restart it a moment later regardless.
in_container "test -f /run/recovery-check.ran" \
    && { in_container "cat /run/recovery-check.ran"; fail "the update ran a unit only a timer may start"; }
pass "and left the recovery check alone, which only a timer may start"

# 2. The deferred restart. Scheduled five seconds after the reply, through a transient unit, so this
#    is the one observation a child process could not make: `updaterd` is replaced *by systemd*
#    after the operation it was performing has finished.
echo "==> waiting for the deferred updaterd restart"
waited=0
until [ "$(main_pid updaterd)" != "$updaterd_before" ]; do
    waited=$((waited + 1))
    [ "$waited" -lt 40 ] || fail "updaterd was never restarted (pid $updaterd_before after ${waited}s)"
    sleep 1
done
pass "the transient timer replaced updaterd after ${waited}s (pid $updaterd_before -> $(main_pid updaterd))"

# And the successor is the *new* binary, which is the point of restarting it at all. Its own
# published identity is the answer, because that is what the startup reconciliation reads.
waited=0
until in_container "grep -q 'releases/1.1.0/bin/updaterd' /run/updaterd/identity.json 2>/dev/null"; do
    waited=$((waited + 1))
    [ "$waited" -lt 20 ] || fail "the restarted updaterd is not running 1.1.0"
    sleep 1
done
pass "the restarted updaterd is running 1.1.0"

# And its startup reconciliation found nothing to fix. Asserted as an absence, because the positive
# line is only logged when *every* unit is accounted for and `fake-robotd` is a `sleep` that publishes
# no identity — which reads as `Unknown`, and being left alone is exactly right for it.
if in_container "journalctl -u updaterd -b --no-pager | grep -q 'still running the release it had'"
then
    fail "the successor found a unit stale, so a restart during the update did not take"
fi
pass "its startup reconciliation restarted nothing, as it should"

# ── injection: `systemd-run` cannot be run ──
#
# Scheduling the deferred restarts is best-effort by design: the update is committed and journalled by
# then, so an update that worked must not be reported as failed because a timer could not be created.
# That is only affordable because it is not the last word — the next `updaterd` start compares each
# unit against the active release and restarts what is stale (`updater/src/reconcile.rs`). Both halves
# of that are asserted here, and neither is observable without systemd.
#
# Shadowed in `/usr/local/sbin`, which comes first in the PATH systemd gives a unit, so the engine
# resolves this one instead of the real `systemd-run`.
echo "==> applying 1.2.0 with systemd-run broken"
in_container "printf '#!/bin/sh\nexit 1\n' > /usr/local/sbin/systemd-run && chmod 755 /usr/local/sbin/systemd-run"
btd_before="$(main_pid btd)"
updaterd_before="$(main_pid updaterd)"

cp "$FIXTURE"/r/1.2.0/* "$FIXTURE/published/"
apply "$WORK/no-timer.log"
grep -q '"outcome": "applied"' "$WORK/no-timer.log" \
    || { sed 's/^/    /' "$WORK/no-timer.log"; fail "a restart that could not be scheduled failed the update"; }
pass "the update still succeeded — an unschedulable restart is not an update failure"

# Matched on the half both arms share: a `systemd-run` that cannot be *executed* and one that exits
# non-zero are different log lines, and which of them a shadow produces is not the point.
in_container "journalctl -u updaterd -b --no-pager | grep -q 'until the next updaterd start notices'" \
    || fail "nothing in the journal says the restart could not be scheduled"
pass "the journal says the restart could not be scheduled"

# Nothing restarted them, so both are still on 1.1.0 — which is the silent state this exists to end.
sleep 8
[ "$(main_pid btd)" = "$btd_before" ] \
    || fail "btd restarted anyway, so this is not testing what it claims"
[ "$(main_pid updaterd)" = "$updaterd_before" ] \
    || fail "updaterd restarted anyway, so this is not testing what it claims"
in_container "grep -q 'releases/1.1.0/bin/fake-btd' /run/btd/identity.json" \
    || fail "btd is not on the old release, so there is nothing for the successor to fix"
pass "btd and updaterd are left on 1.1.0, as a missed timer leaves them"

# The successor's job. Restarted by hand here; on a board this is the next boot, or the next update.
in_container "rm -f /usr/local/sbin/systemd-run"
echo "==> restarting updaterd, whose successor should notice"
in_container "systemctl restart updaterd"
waited=0
until in_container "grep -q 'releases/1.2.0/bin/fake-btd' /run/btd/identity.json 2>/dev/null"; do
    waited=$((waited + 1))
    [ "$waited" -lt 30 ] || fail "the successor did not restart the stale btd"
    sleep 1
done
pass "its reconciliation restarted the stale btd onto 1.2.0 after ${waited}s"

# And it did not restart *itself*. There is nothing for it to fix here — a restarted `updaterd`
# re-execs through `current`, so the successor is the active release by construction — but the
# reconciliation runs over a list that includes its own unit, and a self-restart there would be a loop
# in the one process that owns recovery. The absence of a second restart is the assertion.
self_pid="$(main_pid updaterd)"
sleep 3
[ "$(main_pid updaterd)" = "$self_pid" ] \
    || fail "updaterd restarted itself from its own startup path, which is a loop"
pass "and did not restart itself, which in that process would be a loop with no way out"

# ── injection: a unit whose `User=` the release brings with it ──
#
# `hooks/postinstall` installs `sysusers.d` files and runs `systemd-sysusers` *before* the units, and
# its comment calls that ordering load-bearing: a unit naming a `User=` that does not exist fails to
# start, and the failure reads as a broken daemon rather than a missing account. Nothing observed it.
cp "$FIXTURE"/r/1.3.0/* "$FIXTURE/published/"
echo "==> applying 1.3.0, which ships a unit and the account it needs"
apply "$WORK/sysusers.log"
grep -q '"outcome": "applied"' "$WORK/sysusers.log" \
    || { sed 's/^/    /' "$WORK/sysusers.log"; fail "a release bringing its own user did not apply"; }
in_container "id duck-test >/dev/null 2>&1" \
    || fail "postinstall did not create the account the release ships"
in_container "systemctl is-active --quiet needs-a-user" \
    || fail "the unit is not running, so accounts did not come before units"
pass "accounts before units: the shipped user exists and its unit is running"

# ── injection: the same unit with nothing creating its user ──
#
# The other half of the ordering claim. `enable --now` failing in the hook is only a warning — a
# service that cannot start must not fail an otherwise good update — but the restart in `on_apply`
# right afterwards is not, and that is the distinction `restart_one` draws deliberately.
cp "$FIXTURE"/r/1.4.0/* "$FIXTURE/published/"
echo "==> applying 1.4.0, whose unit names a user nothing creates"
apply "$WORK/ghost.log"
grep -q '"outcome": "applied"' "$WORK/ghost.log" \
    && fail "a unit that cannot start reported success"
grep -q "needs-a-user" "$WORK/ghost.log" \
    || { sed 's/^/    /' "$WORK/ghost.log"; fail "the reason does not name the unit"; }
pass "the update failed and named the unit rather than the account"

# The release did revert: `rollback_to` swaps `current` before it re-runs the apply action, so the
# tree is right even when what follows is not.
[ "$(live)" = 1.3.0 ] || fail "current is $(live) after the failed update, expected 1.3.0"
pass "current went back to 1.3.0"

# The revert happened *and* one unit did not come back with it. Two facts, reported as two — this was
# a single `rollback failed after a failed update`, which asserted the one thing that was false (the
# robot had not been put back) and buried the one thing that was true (a unit refused to restart).
#
# `hooks/postinstall` overwrote 1.3.0's good unit file with 1.4.0's broken one and by design does not
# put it back, so the revert's own restart of a unit with that name inherits the same failure. The
# hook's reasoning is unchanged and still holds: one service is failing until the next release fixes
# it. What changed is that this no longer reads as the recovery having failed.
grep -q "rollback failed" "$WORK/ghost.log" \
    && { sed 's/^/    /' "$WORK/ghost.log"; fail "a revert that happened is still reported as a failed rollback"; }
grep -q "the release was reverted" "$WORK/ghost.log" \
    || { sed 's/^/    /' "$WORK/ghost.log"; fail "the outcome does not report the revert"; }
grep -q "restarting needs-a-user failed" "$WORK/ghost.log" \
    || { sed 's/^/    /' "$WORK/ghost.log"; fail "the outcome does not name the unit that would not come back"; }

# And the claim it must NOT make. Every daemon here runs `Restart=always`, so a unit refusing a
# restart on command is routinely back seconds later; a board on the bench printed "so something on
# this robot is down" directly under its own `robot healthy` line. The outcome now reports the
# refusal and names the command that answers what it cannot.
grep -q "is down" "$WORK/ghost.log" \
    && { sed 's/^/    /' "$WORK/ghost.log"; fail "the outcome still asserts an outage it did not observe"; }
grep -q "robotctl health" "$WORK/ghost.log" \
    || { sed 's/^/    /' "$WORK/ghost.log"; fail "the outcome does not say how to find out whether it came back"; }
pass "reported as reverted, naming the unit, without claiming an outage it never checked for"

# The rest of the robot is still up, which is what keeps this a gap rather than an outage.
in_container "systemctl is-active --quiet fake-robotd" \
    || fail "fake-robotd is down as well"
pass "the daemons that could restart did, so only the broken unit is down"

# Cleared by hand, so the next case starts from a board whose units match its release. On a real board
# this is what the next good release does on its own.
in_container "rm -f /etc/systemd/system/needs-a-user.service && systemctl daemon-reload"

# ── the injection: a unit that installs and cannot start ──
#
# Bug 1's class. The unit arrives with the release, postinstall installs and enables it, `enable
# --now` fails and is only a warning — and then `on_apply` restarts it, which is not. The update must
# fail, roll back, and say which unit.
cp "$FIXTURE"/r/1.5.0/* "$FIXTURE/published/"
echo "==> applying 1.5.0, which ships a unit that cannot start"
# Exit status is deliberately not the check. A rollback is a *successful* call that reports an
# unsuccessful outcome — `robotctl` exits 0 having told you what happened, and reading the status
# instead would assert a different contract than the one that matters here.
apply "$WORK/broken.log"
grep -q rolled_back "$WORK/broken.log" \
    || { sed 's/^/    /' "$WORK/broken.log"; fail "the update did not roll back"; }
grep -q '"outcome": "applied"' "$WORK/broken.log" \
    && fail "the update reported success despite a unit that cannot start"
pass "the update rolled back rather than reporting success"

# 1.3.0, not 1.4.0: that one never became live, so it was never what this rolled back from.
[ "$(live)" = 1.3.0 ] || fail "rolled back to $(live), expected 1.3.0"
pass "rolled back to 1.3.0"

grep -qi "broken" "$WORK/broken.log" \
    || { sed 's/^/    /' "$WORK/broken.log"; fail "the reason does not name the unit that failed"; }
pass "the reason names the unit, not 'unreachable'"

in_container "systemctl is-active --quiet fake-robotd" \
    || fail "fake-robotd is down after the rollback"
pass "the daemon is running again after the rollback"

echo
echo "==> systemd checks passed"
