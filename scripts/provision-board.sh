#!/bin/sh
# Provision a board from your own machine, in one command.
#
#   export DUCK_TOKEN=...              # only while the repository is private
#   ./scripts/provision-board.sh radxa@192.168.1.42
#
# The target is `[user@]host`, and the host can be a name or an address. An address is the
# normal case on this hardware: mDNS on the Radxa image is unreliable, so `radxa-zero3.local`
# resolves when it feels like it and a DHCP lease is the thing you can count on.
#
# The only script in this directory that runs on the *operator's* machine rather than on a
# robot. Everything it does, it does over ssh; nothing here is installed anywhere.
#
# What it is for is the seam in the middle. `provision.sh` reboots the board and finishes on
# its own, which is right, but from the outside that looks like an ssh session dying followed
# by an unknown interval and a guess about when to log back in. This waits for the board to
# come back, streams the log the unattended half writes, and ends on `robotctl health` — so
# provisioning is one command with continuous output instead of three with a gap.
#
#   --ref BRANCH      provision from a branch instead of main
#   --forget-host-key drop this host's key from known_hosts first. Reflashing the card
#                     regenerates the board's host keys, so the same address then presents a
#                     different one and ssh refuses outright — see `probe`.
#   --local           send this clone's scripts/provision.sh instead of having the board fetch
#                     it. What makes testing an unpushed branch possible.
#   --no-dev-key      do not install the team dev key, for a board that should only take
#                     releases. The default is to send this clone's
#                     deploy/dev-key/team.dev.pub.
#   --dev-key PATH    somewhere else to find it.
#
# Needs `ssh` and `scp`, an account on the board that can `sudo`, and nothing else. It expects
# to be able to prompt for the sudo password, so it allocates a terminal for that one command.
set -eu

# Committed, so a new developer needs nothing from anybody to provision a dev board. `--dev-key`
# overrides it for a key handed over out of band.
DEV_KEY_DEFAULT="$(dirname "$0")/../deploy/dev-key/team.dev.pub"

HOST=""
# The host without any `user@`, which is what known_hosts is keyed on.
HOST_ONLY=""
FORGET_KEY=""
REF=""
DEV_KEY="$DEV_KEY_DEFAULT"
NO_DEV_KEY=""
USE_LOCAL=""

# How long to wait for the board to come back after its reboot. Generous on purpose, and sized
# for the worst legitimate case rather than the normal one: a first boot after an overlay change
# is already the slowest this board will ever be, and a wifi cutover that does not take costs a
# further 90s of backstop grace plus a second boot. Giving up before that reports a failure that
# is really impatience — and giving up at all is cosmetic here, since the board finishes on its
# own either way.
BOOT_TIMEOUT=300

# Board-side paths. Duplicated from provision.sh rather than derived, because this script is
# copied to a laptop and run from anywhere — there is nothing to source.
STATE=/var/lib/robot/provision.env
LOG=/var/lib/robot/provision.log

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --ref)        REF="${2:?--ref needs a branch}"; shift 2 ;;
        --forget-host-key) FORGET_KEY=1; shift ;;
        --dev-key)    DEV_KEY="${2:?--dev-key needs a path}"; shift 2 ;;
        --no-dev-key) NO_DEV_KEY=1; shift ;;
        --local)      USE_LOCAL=1; shift ;;
        -h|--help)    usage 0 ;;
        -*)           die "unknown option: $1" ;;
        *)            [ -z "$HOST" ] || die "one board at a time"; HOST="$1"; shift ;;
    esac
done

[ -n "$HOST" ] || usage 2

command -v ssh >/dev/null 2>&1 || die "ssh is required"
command -v scp >/dev/null 2>&1 || die "scp is required"

# `[user@]host` split, because two things need the host on its own: known_hosts is keyed on it,
# and an IPv6 literal has to be bracketed for scp while ssh wants it bare.
case "$HOST" in
    *@*) HOST_ONLY="${HOST#*@}" ;;
    *)   HOST_ONLY="$HOST" ;;
esac
[ -n "$HOST_ONLY" ] || die "no host in '${HOST}' — expected [user@]host"

# scp's target syntax is not ssh's: `host:path` is ambiguous for an IPv6 literal, which has
# colons of its own, so that one case needs brackets. Detected by the colon rather than by
# trying to parse an address — a hostname or an IPv4 address has none.
scp_target() {
    # $1 remote path
    case "$HOST_ONLY" in
        *:*)
            if [ "$HOST" = "$HOST_ONLY" ]; then
                printf '[%s]:%s' "$HOST_ONLY" "$1"
            else
                printf '%s@[%s]:%s' "${HOST%@*}" "$HOST_ONLY" "$1"
            fi
            ;;
        *) printf '%s:%s' "$HOST" "$1" ;;
    esac
}

# Non-interactive ssh, for the polling and the file checks.
#
# Every option here is load-bearing against a board that is rebooting:
#
#   BatchMode           a board that has gone away fails in seconds instead of sitting on a
#                       password prompt nobody is watching.
#   ConnectTimeout      bounds the TCP handshake — and *only* that, which is the trap below.
#   ServerAlive*        bounds everything after it. A half-started network stack accepts the
#                       handshake and then stops talking, and without this ssh waits for that
#                       forever. That is not hypothetical: it is what made "waiting up to 180s"
#                       hang indefinitely on the first real board, because the loop never got
#                       back to its own clock to notice the time.
#   ControlPath=none    an ssh_config with multiplexing turned on leaves a master socket
#                       pointing at a connection the reboot killed, and every later call queues
#                       behind it. Not our config to fix, so it is opted out of.
rsh() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 \
        -o ServerAliveInterval=3 -o ServerAliveCountMax=2 \
        -o ControlPath=none -o StrictHostKeyChecking=accept-new "$HOST" "$@"
}

# Is the board answering? True/false within $1 seconds, whatever ssh decides to do.
#
# The belt to ServerAlive's braces. ssh has more ways to block than there are options to stop
# it — DNS, an authentication step, a sluggish sshd on a booting board — and the one thing this
# loop must never do is stop counting. `timeout(1)` would be the obvious tool and is not on
# macOS, so the watchdog is written out.
# A subshell body with its stderr closed, because the kill below makes the shell announce
# `Terminated: 15  rsh true` on the terminal — a job-control notice that reads like a failure
# in the middle of a wait that is working exactly as intended.
alive() (
    rsh true >/dev/null 2>&1 &
    _probe_pid=$!
    _probe_n=0
    while kill -0 "$_probe_pid" 2>/dev/null; do
        if [ "$_probe_n" -ge "$1" ]; then
            kill -TERM "$_probe_pid" 2>/dev/null || true
            sleep 1
            kill -KILL "$_probe_pid" 2>/dev/null || true
            wait "$_probe_pid" 2>/dev/null || true
            return 1
        fi
        sleep 1
        _probe_n=$((_probe_n + 1))
    done
    wait "$_probe_pid"
) 2>/dev/null

# True while the board still has provisioning left to do. `provision.sh` removes the state file
# when it finishes, which makes "are we done" a question with a file for an answer rather than
# a log line to pattern-match.
still_provisioning() {
    rsh "test -f ${STATE}" >/dev/null 2>&1
}

# ── checks that are cheaper to fail now than halfway ─────────────────────────

if [ -n "$FORGET_KEY" ]; then
    say "dropping ${HOST_ONLY} from known_hosts"
    ssh-keygen -R "$HOST_ONLY" >/dev/null 2>&1 || true
fi

say "checking ${HOST}"

# The probe's *output* is the diagnosis, so it is captured rather than discarded. Four failures
# are common here and they need four different answers; "cannot ssh to the board" sends you
# looking at whichever one you thought of first.
if ! _probe="$(rsh true 2>&1)"; then
    case "$_probe" in
        *"REMOTE HOST IDENTIFICATION HAS CHANGED"*|*"Host key verification failed"*)
            die "${HOST_ONLY} is presenting a different host key than the one on record.
  Reflashing the card regenerates the board's host keys, so this is the expected outcome of
  provisioning the same address twice — and StrictHostKeyChecking=accept-new does not cover it,
  because the host is not new, its key is. Drop the old one:
    ssh-keygen -R ${HOST_ONLY}
  Or re-run this with --forget-host-key, which does that first." ;;
        *"Permission denied"*)
            die "${HOST} refused the key.
  This needs to reconnect by itself after the board reboots, which a password prompt cannot
  survive, so key access is not optional here:
    ssh-copy-id ${HOST}" ;;
        *"Could not resolve"*|*"Name or service not known"*|*"nodename nor servname"*)
            # The advice splits on what was actually passed: telling someone who gave an
            # address to "use the address instead" is the kind of message that makes a tool
            # feel like it is not listening.
            case "$HOST_ONLY" in
                *[!0-9.]*)
                    die "cannot resolve '${HOST_ONLY}'.
  mDNS on this image is unreliable — a .local name resolves when it feels like it — so a name
  is not the thing to depend on here. Use the address from your router's DHCP leases, or
  find it with:  ping -c1 ${HOST_ONLY}" ;;
                *)
                    die "cannot resolve '${HOST_ONLY}', which looks like an address rather than
  a name — so this is likely a typo in it rather than a naming problem." ;;
            esac ;;
        *"Connection refused"*|*"timed out"*|*"No route to host"*|*"Network is unreachable"*)
            die "no answer from ${HOST_ONLY}.
  Either it is not up yet, or that is not its address any more — a DHCP lease moves, and on a
  reflashed card it very often does." ;;
        *)
            die "cannot ssh to ${HOST}:
${_probe}" ;;
    esac
fi

if [ -z "${DUCK_TOKEN:-}" ]; then
    warn "DUCK_TOKEN is not set. While the repository is private every fetch on the board
  needs it, and GitHub answers 404 rather than 401, so it will look like a wrong URL.
  Continuing in case the repository is public by now."
fi

if [ -n "$NO_DEV_KEY" ]; then
    DEV_KEY=""
elif [ ! -f "$DEV_KEY" ]; then
    die "${DEV_KEY} is not a readable file. It ships with the repository, so a clone should
  always have it — pass --dev-key PATH for a key from somewhere else, or --no-dev-key for a
  board that should only take releases."
fi

# ── put what the board needs where the board can reach it ────────────────────

if [ -n "$DEV_KEY" ]; then
    say "sending the dev key"
    scp -q -o StrictHostKeyChecking=accept-new "$DEV_KEY" "$(scp_target /tmp/team.dev.pub)" \
        || die "could not copy ${DEV_KEY} to ${HOST}"
fi

# The local copy is the whole point of `--local`: it provisions a board with a `provision.sh`
# that has not been pushed anywhere, which is the only way to test a change to it without
# merging first. Everything the script then fetches still comes from --ref, so a full test of a
# branch is `--local --ref that-branch`.
if [ -n "$USE_LOCAL" ]; then
    _local="$(dirname "$0")/provision.sh"
    [ -f "$_local" ] || die "--local needs ${_local}, and it is not there.
  Run this from a clone, or drop --local and let the board fetch it from ${REF:-main}."
    say "sending this clone's provision.sh"
    scp -q "$_local" "$(scp_target /tmp/provision.sh)" || die "could not copy provision.sh"
else
    _raw="https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/${REF:-main}/scripts/provision.sh"
    say "having the board fetch provision.sh from ${REF:-main}"
    # Fetched by the board rather than by this machine and copied over: the board is the one
    # that has to be able to reach GitHub with that token, and finding out here would prove
    # the wrong thing.
    rsh "curl -fsSL ${DUCK_TOKEN:+-H \"Authorization: Bearer ${DUCK_TOKEN}\"} '${_raw}' -o /tmp/provision.sh" \
        || die "the board could not fetch provision.sh from ${REF:-main}.
  A private repository answers 404 rather than 401, so this is either a missing DUCK_TOKEN, a
  token without Contents:Read on the repository, or a branch name that does not exist."
fi

# ── phase 1, which ends in a reboot that takes the connection with it ────────

say "starting provisioning — the board will reboot and this will wait for it"
echo

_env="DUCK_TOKEN='${DUCK_TOKEN:-}'"
[ -z "$REF" ]     || _env="${_env} DUCK_REF='${REF}'"
[ -z "$DEV_KEY" ] || _env="${_env} DUCK_DEV_KEY=/tmp/team.dev.pub"

# `-t` so sudo can prompt for a password, and the exit status deliberately ignored: this
# command ends by rebooting the machine it is running on, so ssh reporting a dropped connection
# is the *expected* outcome. Whether it worked is decided below, by looking at the board.
ssh -t -o StrictHostKeyChecking=accept-new "$HOST" \
    "sudo env ${_env} sh /tmp/provision.sh" || true

echo
say "waiting for ${HOST} to come back (up to ${BOOT_TIMEOUT}s)"

echo "  (the board finishes on its own — this is only watching)"

# The board is mid-reboot, so it may still answer for a moment. Wait for it to go before waiting
# for it to return, or this races and declares success against the dying session.
_start="$(date +%s)"
while [ "$(( $(date +%s) - _start ))" -lt 20 ]; do
    alive 5 || break
    sleep 2
done

# Wall clock, not a sum of sleeps. A probe that takes longer than expected must eat into the
# budget rather than extend it — the sum-of-sleeps version could not time out at all while a
# probe was blocked, which is precisely the failure this loop is here to survive.
_start="$(date +%s)"
_elapsed=0
until alive 10; do
    _elapsed="$(( $(date +%s) - _start ))"
    if [ "$_elapsed" -ge "$BOOT_TIMEOUT" ]; then
        die "no answer from ${HOST} after ${_elapsed}s.
  That does not mean provisioning failed. The board resumes by itself at boot, so this is a
  viewer that lost sight of it — the board may well be finishing right now. Look directly:
    ssh -t ${HOST} 'sudo tail -f ${LOG}'
  If it is genuinely unreachable: a failed wifi cutover makes the backstop restore netplan and
  reboot, which costs a second boot, and NetworkManager may come back on a different DHCP lease
  than netplan had — so check for a new address before concluding the board is down."
    fi
    printf '.'
    sleep 3
done
echo
say "back after ~$(( $(date +%s) - _start ))s"

# ── phase 2, which is running unattended on the board ────────────────────────

# Polled rather than `tail -f`: the connection has to survive a service that may still be
# starting, and a poll that reconnects each time cannot be left holding a dead channel. Only
# new bytes are printed, so this reads like a stream.
_seen=0
_quiet=0

# How the log gets read. Plain, or through a non-interactive sudo for a board provisioned before
# the log became group-readable. Decided once, on the first read, rather than guessed every time.
LOG_READ=""

# Work out whether this log is readable at all, and how. Its own step because the failure it
# replaces was silence: an unreadable log made the watcher print nothing, forever, next to a
# board that was provisioning perfectly well.
choose_log_reader() {
    if rsh "test -r ${LOG}" >/dev/null 2>&1; then
        LOG_READ=""
        return 0
    fi
    # `sudo -n`, never bare sudo: over BatchMode ssh there is no terminal to prompt on, so a
    # sudo that wants a password hangs or fails with its error swallowed. This is the exact
    # shape of the original bug.
    if rsh "sudo -n test -r ${LOG}" >/dev/null 2>&1; then
        LOG_READ="sudo -n "
        return 0
    fi
    return 1
}

# Print whatever has been appended since the last call. Returns 1 when there was nothing, which
# is what the stall detection counts.
drain_log() {
    _size="$(rsh "${LOG_READ}sh -c 'test -f ${LOG} && wc -c < ${LOG} || echo 0'" 2>/dev/null || echo "$_seen")"
    # Digits only: a stray line from ssh or sudo in that output would otherwise reach the
    # arithmetic below and abort the script for a cosmetic reason.
    _size="$(printf '%s' "$_size" | tr -dc '0-9')"
    [ -n "$_size" ] || _size=$_seen

    if [ "$_size" -gt "$_seen" ]; then
        rsh "${LOG_READ}tail -c +$((_seen + 1)) ${LOG}" 2>/dev/null || true
        _seen=$_size
        return 0
    fi
    return 1
}

if ! choose_log_reader; then
    warn "cannot read ${LOG} on ${HOST}, so there is nothing to stream here — not because
  provisioning failed, but because this account cannot read that file and sudo cannot prompt
  over a non-interactive connection. The board carries on regardless. Watch it yourself:
    ssh -t ${HOST} 'sudo tail -f ${LOG}'
  A board provisioned by a current provision.sh makes that log readable by the robot group,
  which you are in after the reboot; an older one left it root-only."
fi

while :; do
    if drain_log; then
        _quiet=0
    else
        _quiet=$((_quiet + 3))
    fi

    if ! still_provisioning; then
        # One more read before leaving. `provision.sh` writes its closing lines and *then*
        # removes the state file, so a loop that breaks the moment the file is gone drops the
        # last thing it said — including which token ended up where, and whether the board came
        # out a dev board. Which is the part worth reading.
        drain_log || true
        break
    fi

    # A board that has stopped writing and still has a state file has either failed or is
    # waiting on something slow. Say so rather than looking identical to progress.
    if [ "$_quiet" -ge 120 ]; then
        warn "nothing new in ${LOG} for two minutes and provisioning has not finished.
  Still waiting, but worth a look:  ssh ${HOST} 'systemctl status robot-provision'"
        _quiet=0
    fi
    sleep 3
done

echo
say "provisioning finished"

# The health report is the point of all of it, and it is also the thing most likely to have
# something to say — a bench board with no servos powered reports unhealthy, correctly.
rsh "robotctl health" || warn "robotctl health did not report cleanly. On a board with no
  servos powered that is the honest answer, not a failed install. The full log is at:
    ssh -t ${HOST} 'sudo cat ${LOG}'"
