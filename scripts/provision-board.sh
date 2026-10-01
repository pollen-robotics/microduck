#!/bin/sh
# Provision a board from your own machine, in one command.
#
#   export DUCK_TOKEN=...              # only while the repository is private
#   ./scripts/provision-board.sh microduck@192.168.1.42
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
#   --ref BRANCH      provision from a branch: its scripts run the bring-up, and its build of
#                     the daemon is installed on top of the stable release. Provisioning FAILS
#                     if that build cannot be installed or is rolled back — a dev board quietly
#                     running stable when a branch was asked for is worse than a clear stop.
#                     Needs the branch build to exist, so give CI its minute or two first.
#   --name NAME       name the robot, at the end of provisioning: `--name Ducky`. Optional —
#                     without it the board names itself `duck-<four hex>` from its SoC serial,
#                     which is already unique per board. Changeable later at any time with
#                     `robotctl system set-name`, so this saves a command rather than deciding
#                     anything permanent.
#   --forget-host-key drop this host's key from known_hosts first. Reflashing the card
#                     regenerates the board's host keys, so the same address then presents a
#                     different one and ssh refuses outright — see `probe`.
#   --local           send this clone's scripts/provision.sh instead of having the board fetch
#                     it. What makes testing an unpushed branch possible.
#   --no-dev-key      do not install the team dev key, for a board that should only take
#                     releases. The default is to send this clone's
#                     deploy/dev-key/team.dev.pub.
#   --dev-key PATH    somewhere else to find it.
#   --no-ble          do not use Bluetooth to re-find the board. See below for what that costs.
#   --pause-btd-on-pair
#                     for a Radxa Zero 3W that pairs a gamepad only while `btd` is out of the way.
#                     Leaves /var/lib/robot/weird-ble, which makes a pairing session stop `btd`
#                     and power-cycle the adapter for the pairing window — and **does not touch
#                     `Privacy`**. This is what a board wants when a pad pairs and then flaps with
#                     `PIN or Key Missing`: that is `Privacy = device` on a board that needed only
#                     the pause. Try this before `--weird-ble`.
#   --no-gstreamer    skip the GStreamer stack `mediad` needs. Installed by default, with a
#                     report of what this board can encode. It needs no reboot, so it can also
#                     be added or re-run later:  sudo /usr/local/sbin/robot-setup-gstreamer
#   --no-rkaiq        skip the camera's 3A engine (white balance, colour, denoise; exposure is
#                     mediad's own software loop, not this).
#                     Installed by default. Without it the camera runs on raw ISP defaults:
#                     green and noisy. Re-run later with:
#                     sudo /usr/local/sbin/robot-setup-rkaiq
#   --weird-ble       for a Radxa Zero 3W whose Bluetooth cannot bond a gamepad at all, even with
#                     `btd` paused. Implies `--pause-btd-on-pair`, and additionally sets
#                     `Privacy = device`.
#                     **The default in docs/robot/install-dev.md**, because about half these
#                     boards need it and nothing measurable says which — and a board that needed
#                     it and did not get it presents as a pad that will not pair, for no visible
#                     reason. Drop it on a board proven to bond a pad without it; see that page
#                     for how to check, and `configure_bluetooth` in scripts/setup-board.sh for
#                     the measurement behind it.
#
# Needs `ssh` and `scp`, an account on the board that can `sudo`, and nothing else. It expects
# to be able to prompt for the sudo password, so it allocates a terminal for that one command.
#
# ## When the board comes back at a different address
#
# A DHCP lease moves, and the cutover in the middle of provisioning is exactly when it does: the
# board leaves on netplan's lease and comes back on NetworkManager's. From out here that is
# indistinguishable from a board that never booted at all — the address you were given simply
# stops answering, and every remaining step is addressed to it.
#
# So the wait is a race. ssh keeps polling the address you gave, and in parallel a Bluetooth probe
# asks the robot itself what address it ended up with — `net.status`, the same call the phone app
# makes. Whichever answers first wins, and an answer over Bluetooth is adopted for everything
# after it.
#
# On the same address ssh wins essentially always: `hci0` takes about seventy seconds to appear on
# this board and sshd does not, so the probe is still building when the ordinary case is already
# over. That is the point. It costs nothing in the normal case and it is the only thing that can
# answer in the abnormal one.
#
# **The probe is only trusted when exactly one robot answers to the name this board advertises**,
# which is read off the board before the reboot while ssh still works. Adopting an address means
# ssh'ing somewhere new, and doing that to a robot chosen by scan order is worse than waiting.
#
# It needs `cargo` and this clone, because `duckctl` is an example rather than an installed
# binary. Without either, the wait is exactly what it was before and says so. Three more things it
# cannot do:
#
#   - A board being provisioned for the first time has no `btd` until the install reaches it, so
#     Bluetooth cannot answer for the first few minutes.
#   - A robot with a per-robot pairing PIN needs it in `DUCK_PIN`; the probe otherwise offers the
#     factory default.
#   - `net.status` reports the *wifi* interface, so a board you reach over ethernet is not covered.
#     Which is the right interface to read — the cutover this exists for is wifi's — but it means an
#     ethernet lease that moves is still a lease nothing here can find.
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
NO_BLE=""
WEIRD_BLE=""
NO_GSTREAMER=""
NO_RKAIQ=""
PAUSE_BTD=""
# The name to give the robot. Not `BLE_NAME` below, which points the other way: the name this board
# already answers to, used to find it again after the reboot.
ROBOT_NAME=""

# ── the Bluetooth fallback ───────────────────────────────────────────────────
#
# `duckctl` is an example rather than a binary — deliberately, so `btleplug` never reaches a
# robot — which means the only way to run it is `cargo` against this clone.
BLE_MANIFEST="$(dirname "$0")/../Cargo.toml"

# The name the robot advertises, read off the board before the reboot.
#
# Empty means the fallback is off: a probe with no name talks to whichever robot the scan reported
# first, and the whole point of the fallback is to hand ssh a new address to trust.
BLE_NAME=""

# Whether `btd` was already running before the reboot. Does not gate the probe — it is one line in
# a message, so a first-time board is told why Bluetooth has nothing to say yet rather than left to
# wonder whether the probe is broken.
BLE_LIVE=""

# The PIN the probe offers. The factory default, which is what a board being provisioned has;
# `DUCK_PIN` is for a robot that has been given its own.
BLE_PIN="${DUCK_PIN:-000000}"

# Scratch for the probe: its verdict, and its output kept for the diagnosis.
BLE_DIR=""
# The running probe, empty when none is.
BLE_PID=""
# What the probe last said, printed once rather than every three seconds.
BLE_SAID=""

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

# The unit that runs phase 2 on the board, asked whether it failed. Must match `UNIT_NAME` in
# provision.sh; a mismatch would make a failed phase 2 look like one still working, which is the
# hang this exists to end.
UNIT_NAME=robot-provision.service

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# The header is the help, so print it whole: every comment line from the shebang to the first line
# that is not one.
#
# A line range is what this was, and it had already gone stale — `2,22p` stopped mid-sentence
# inside `--forget-host-key` and hid the three options after it, because the range does not move
# when a paragraph is added above it. Nothing to keep in step this way.
usage() {
    awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --ref)        REF="${2:?--ref needs a branch}"; shift 2 ;;
        --name)       ROBOT_NAME="${2:?--name needs a name}"; shift 2 ;;
        --forget-host-key) FORGET_KEY=1; shift ;;
        --dev-key)    DEV_KEY="${2:?--dev-key needs a path}"; shift 2 ;;
        --no-dev-key) NO_DEV_KEY=1; shift ;;
        --no-ble)     NO_BLE=1; shift ;;
        --no-gstreamer) NO_GSTREAMER=1; shift ;;
        --no-rkaiq)   NO_RKAIQ=1; shift ;;
        --weird-ble)  WEIRD_BLE=1; shift ;;
        --pause-btd-on-pair) PAUSE_BTD=1; shift ;;
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

# Does the board still have provisioning left to do? `provision.sh` removes the state file when it
# finishes, which makes "are we done" a question with a file for an answer rather than a log line
# to pattern-match.
#
#   0  still provisioning
#   1  finished
#   2  cannot tell, because the board is not answering
#
# Three answers rather than two, and the third is a bug this fixes. `rsh` fails when the *board*
# went away just as surely as when the file is gone, so one test read `finished` for a board that
# had merely moved to a new lease mid-install — and the watcher announced provisioning complete and
# then a health check that could not connect. Which of the two it is decides whether to stop or to
# go looking, so it cannot be collapsed.
#
# **Only `test`'s own 1 means the file is gone.** ssh exits 255 when *it* fails, and it fails for
# one round-trip at a time on a board busy unpacking packages: a 5 s connect timeout and a 6 s
# keepalive budget are both reachable while apt runs triggers on 1 GB of memory. Reading any
# non-zero as "absent" let one slow answer, followed by a successful `alive`, announce provisioning
# finished mid-apt — and then run `robotctl health` on a board that did not have robotctl yet.
still_provisioning() {
    _state=0
    rsh "test -f ${STATE}" >/dev/null 2>&1 || _state=$?
    if [ "$_state" != 0 ] && [ "$_state" != 1 ]; then
        # No answer about the file at all. Ask again next time round if the board is there;
        # otherwise it is the lost board the verdict 2 exists for.
        if alive 10; then
            return 0
        fi
        return 2
    fi
    if [ "$_state" = 0 ]; then
        # The state file outlives a phase 2 that *failed*: `provision.sh` removes it only on the way
        # out cleanly, so its presence alone cannot tell a board still working from one that stopped
        # with an error. Ask systemd, which knows.
        #
        # Returned as its own verdict rather than folded into "finished", because the two need
        # different words: one is a robot ready to use, the other is a board that needs looking at.
        if rsh "systemctl is-failed --quiet ${UNIT_NAME}" >/dev/null 2>&1; then
            return 3
        fi
        return 0
    fi
    # The file is gone, or the board is. One more question tells them apart, and it is only ever
    # asked once — at the end, or at the failure.
    if alive 10; then
        return 1
    fi
    return 2
}

# Point everything at a new address for the same board.
#
# The `[user@]` is carried across rather than rebuilt: a board whose account is not the default
# would otherwise become unreachable at the moment this was trying to save it.
adopt_address() {
    case "$HOST" in
        *@*) HOST="${HOST%@*}@$1" ;;
        *)   HOST="$1" ;;
    esac
    HOST_ONLY="$1"

    # A recycled lease is the one way an adopted address cannot work: another board held it, so
    # known_hosts has *its* key on record and ssh refuses this one outright. `alive` discards output,
    # so without this the wait would simply go quiet until the budget ran out and blame the board.
    _adopted="$(rsh true 2>&1 || true)"
    case "$_adopted" in
        *"REMOTE HOST IDENTIFICATION HAS CHANGED"*|*"Host key verification failed"*)
            die "the robot says it is at ${1}, and ssh refuses that address: known_hosts has a
  different board's key on record for it. A DHCP lease that has been round the houses is enough
  to do this. Drop it and re-run:
    ssh-keygen -R ${1}" ;;
    esac
}

# ── the Bluetooth probe ──────────────────────────────────────────────────────

# Print something in the middle of a line of progress dots, once per distinct message.
#
# Repeats are dropped rather than rate-limited: a probe that keeps returning the same answer every
# twenty seconds has nothing new to say, and the alternative is a screen of one repeated line with
# the useful message scrolled off the top.
#
# Closing the dot line is this function's job rather than every caller's, which it can do because
# POSIX `sh` has no locals and `_dots` is therefore shared with the loop printing them.
ble_say() {
    [ "$BLE_SAID" != "$1" ] || return 0
    BLE_SAID="$1"
    [ "${_dots:-0}" = 0 ] || echo
    _dots=0
    printf '\033[1m  bluetooth:\033[0m %s\n' "$1"
}

# Learn the name the robot advertises, while ssh still works. Empty when it cannot be known.
#
# Two sources, because a board being provisioned is in one of two states and they have different
# authorities:
#
#   - A board that already runs the daemon is asked. That answer includes a rename someone stored
#     through `system.setName`, which no derivation can know about.
#   - A board that does not is derived from, exactly as `configd/src/identity.rs` will when it
#     starts: `duck-` and the first two bytes of the SHA-256 of the SoC serial. Computed on the
#     board, so nothing is needed on this machine — macOS has `shasum` and not `sha256sum`, and
#     that is a silly reason for a fallback not to work.
#
# A board with no readable serial is deliberately left empty rather than guessed at. `configd` falls
# back to the hostname there, and every board flashed from one image has the same one — so the name
# would match all of them, which is the case the probe must not act on.
learn_ble_name() {
    if _name="$(rsh "robotctl system info --json 2>/dev/null" 2>/dev/null)"; then
        _name="$(printf '%s' "$_name" | sed -n 's/.*"name":"\([^"]*\)".*/\1/p')"
        if [ -n "$_name" ]; then
            printf '%s' "$_name"
            return 0
        fi
    fi

    # Mirrors `serial_at` clause for clause, because a name that disagrees with the robot's is worse
    # than no name at all:
    #
    #   tr '\0' '\n' | head -n1   a devicetree property holds NUL-terminated strings and may hold
    #                             several, so the first NUL ends the value — `split('\0').next()`,
    #                             not "delete the NULs", which would hash two values joined.
    #   sed            the `.trim()`.
    #   *[![:graph:]]* `is_ascii_graphic`, which excludes the space: a property that exists but is
    #                  blank or full of control characters is no identity, and printing nothing here
    #                  is how this agrees.
    #
    # Nothing is printed unless the digest came out, so a board without `sha256sum` yields no name
    # rather than the name `duck-`, which every such board would then share.
    rsh "serial=\$(tr '\\0' '\\n' < /proc/device-tree/serial-number 2>/dev/null | head -n1 \
             | sed 's/^[[:space:]]*//; s/[[:space:]]*\$//'); \
         case \"\$serial\" in \
           '' | *[![:graph:]]*) exit 0 ;; \
         esac; \
         digest=\$(printf '%s' \"\$serial\" | sha256sum 2>/dev/null | cut -c1-4); \
         [ \${#digest} -eq 4 ] || exit 0; \
         printf 'duck-%s' \"\$digest\"" 2>/dev/null || true
}

# Start a probe in the background, unless one is already running.
#
# Backgrounded and polled rather than waited on, because it has to lose gracefully: the whole design
# is that ssh usually answers first, and a probe still holding a radio open must not delay the run
# that already succeeded. The verdict is written to a temporary name and renamed, so a poll can
# never read half a line.
ble_probe_start() {
    [ -n "$BLE_NAME" ] || return 0
    [ -z "$BLE_PID" ] || return 0

    {
        if cargo run -q --manifest-path "$BLE_MANIFEST" -p duckctl -- \
            --name "$BLE_NAME" --pin "$BLE_PIN" wifi status \
            >"${BLE_DIR}/out" 2>"${BLE_DIR}/err"
        then
            _found="$(sed -n 's/.*"ip4"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' \
                "${BLE_DIR}/out" | head -n1)"
            if [ -n "$_found" ]; then
                printf 'ip %s\n' "$_found" > "${BLE_DIR}/verdict.tmp"
            else
                # The robot answered and reported no IPv4 address, which is not a failed probe —
                # it is the diagnosis, and a far better one than silence.
                printf 'no-address\n' > "${BLE_DIR}/verdict.tmp"
            fi
        else
            printf 'failed\n' > "${BLE_DIR}/verdict.tmp"
        fi
        mv -f "${BLE_DIR}/verdict.tmp" "${BLE_DIR}/verdict"
    } >/dev/null 2>&1 &

    BLE_PID=$!
}

# The probe's verdict if it has one, consumed so each is acted on once. 1 while it is still working.
#
# Prints and nothing else. Reaping the job and clearing `BLE_PID` belong to the caller, because this
# is read through a command substitution — a subshell — and an assignment made in here would be
# discarded with it. That is not a hypothetical tidiness point: `BLE_PID` would have stayed set for
# the rest of the run, `ble_probe_start` would have returned early on every call, and the fallback
# would have had exactly one attempt at a board whose `btd` needs a minute to come up.
ble_verdict() {
    [ -n "$BLE_DIR" ] || return 1
    [ -f "${BLE_DIR}/verdict" ] || return 1

    cat "${BLE_DIR}/verdict"
    rm -f "${BLE_DIR}/verdict"
}

# Why the last probe failed, as `duckctl` put it.
#
# Its own words rather than a paraphrase, and not matched on either: it explains a name nobody
# answers to, two robots answering to one, a missing adapter and a refused PIN, each differently and
# better than this script could. Matching on those strings would be one more thing to keep in step.
ble_failure() {
    [ -n "$BLE_DIR" ] || return 0
    [ -f "${BLE_DIR}/err" ] || return 0
    # Compiler output on a first run is many lines of nothing anyone wants; the last line is the
    # error itself.
    grep -v '^ *$' "${BLE_DIR}/err" 2>/dev/null | tail -n1 || true
}

ble_stop() {
    if [ -n "$BLE_PID" ]; then
        kill -TERM "$BLE_PID" 2>/dev/null || true
        wait "$BLE_PID" 2>/dev/null || true
        BLE_PID=""
    fi
    # A verdict nobody read is about a wait that is over. Left behind, it would be the first thing
    # the *next* wait saw — an address from before the board rebooted again.
    [ -z "$BLE_DIR" ] || rm -f "${BLE_DIR}/verdict" "${BLE_DIR}/verdict.tmp"
}

# How long the last wait took, for whoever has to report it.
WAITED=0

# Wait until $HOST answers, adopting a new address if the robot reports one over Bluetooth. 0 when
# the board answers — possibly somewhere new — and 1 when the budget ran out. Sets $WAITED.
#
# Wall clock, not a sum of sleeps. A probe that takes longer than expected must eat into the budget
# rather than extend it: the sum-of-sleeps version could not time out at all while a probe was
# blocked, which is precisely the failure this loop is here to survive.
wait_for_board() {
    _budget="$1"
    _began="$(date +%s)"
    _dots=0
    BLE_SAID=""
    ble_probe_start

    until alive 10; do
        _verdict="$(ble_verdict || true)"
        if [ -n "$_verdict" ]; then
            # Reaped here rather than in `ble_verdict`, which cannot: see its comment.
            [ -z "$BLE_PID" ] || wait "$BLE_PID" 2>/dev/null || true
            BLE_PID=""
        fi

        case "$_verdict" in
            "ip "*)
                _reported="${_verdict#ip }"
                if [ "$_reported" = "$HOST_ONLY" ]; then
                    # Nothing to adopt, and still worth saying: the board is up and answering, so
                    # what this is waiting for is sshd rather than a boot.
                    ble_say "the robot is up and reports ${_reported}, the address already in use,
             so this is waiting on sshd rather than on the board"
                else
                    ble_say "the robot reports ${_reported}, and ${HOST_ONLY} was its old lease.
             Everything after this is addressed there."
                    adopt_address "$_reported"
                fi
                ;;
            no-address)
                ble_say "the robot is up and has no IPv4 address, so it never got a lease.
             Phase 2 downloads a release, so it cannot finish in this state — but NetworkManager
             may still be trying, so this keeps waiting."
                ;;
            failed)
                ble_say "nothing usable yet — $(ble_failure)"
                ;;
        esac

        # Re-armed only on a verdict, so there is one probe at a time and one radio in use. A probe
        # that has just failed is worth repeating: the commonest reason by far is a board whose
        # `btd` has not finished starting.
        if [ -n "$_verdict" ]; then
            ble_probe_start
        fi

        WAITED="$(( $(date +%s) - _began ))"
        if [ "$WAITED" -ge "$_budget" ]; then
            [ "$_dots" = 0 ] || echo
            ble_stop
            return 1
        fi
        printf '.'
        _dots=1
        sleep 3
    done

    WAITED="$(( $(date +%s) - _began ))"
    [ "$_dots" = 0 ] || echo
    ble_stop
    return 0
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

# ── arrange the Bluetooth fallback, while ssh still works ────────────────────
#
# All of it happens here, before anything has been changed, because every part of it needs a working
# ssh connection — and after the reboot is precisely when there may not be one. A fallback that has
# to reach the board to arm itself is no fallback at all.
if [ -n "$NO_BLE" ]; then
    say "not using Bluetooth to re-find the board (--no-ble)"
elif ! command -v cargo >/dev/null 2>&1; then
    warn "no cargo on this machine, so Bluetooth cannot be used to re-find the board if its
  address changes across the reboot. Not fatal — the board finishes on its own either way, and
  the wait below is what it always was. Install Rust to get the fallback."
elif [ ! -f "$BLE_MANIFEST" ]; then
    warn "${BLE_MANIFEST} is not there, so this is not being run from a clone and duckctl
  cannot be built — no Bluetooth fallback if the board's address changes across the reboot."
else
    BLE_NAME="$(learn_ble_name)"
    if [ -z "$BLE_NAME" ]; then
        warn "cannot tell what name this board will advertise over Bluetooth, so there is no
  fallback if its address changes across the reboot. A board whose bootloader leaves
  /proc/device-tree/serial-number empty is named after its hostname, which is the same on every
  board flashed from one image — so a probe could not tell this robot from the next one, and
  guessing is worse than waiting."
    else
        BLE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/provision-ble.XXXXXX")"

        # Both the probe and the scratch go, whichever way this script ends. A radio left held open
        # by an orphaned `cargo run` is the one piece of litter here that a later run would notice.
        #
        # Only EXIT does the cleaning, and the signals just exit — which is what makes Ctrl-C still
        # *stop* this. A handler on INT that returns rather than exiting resumes the script at the
        # next line, so the ten-second grace `provision.sh` prints before rebooting would have
        # become uninterruptible from here: the one place Ctrl-C is documented to work.
        trap 'ble_stop; [ -z "$BLE_DIR" ] || rm -rf "$BLE_DIR"' EXIT
        trap 'exit 130' INT
        trap 'exit 143' TERM

        if rsh "systemctl is-active --quiet btd" >/dev/null 2>&1; then
            BLE_LIVE=1
            say "this board advertises as '${BLE_NAME}', and btd is running — so Bluetooth can
    re-find it if its address changes"
        else
            say "this board will advertise as '${BLE_NAME}'. btd is not running on it yet, so
    Bluetooth can only answer once the install reaches it — a few minutes into phase 2"
        fi
    fi
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
    _raw="https://raw.githubusercontent.com/pollen-robotics/microduck/${REF:-main}/scripts/provision.sh"
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
[ -z "$WEIRD_BLE" ] || _env="${_env} DUCK_WEIRD_BLE=1"
[ -z "$PAUSE_BTD" ]  || _env="${_env} DUCK_PAUSE_BTD=1"
[ -z "$NO_GSTREAMER" ] || _env="${_env} DUCK_GSTREAMER=0"
[ -z "$NO_RKAIQ" ]     || _env="${_env} DUCK_RKAIQ=0"

# The name is a flag rather than one more `DUCK_*`, because on the board it goes no further than
# `robotctl system set-name`. Single-quoted with any quote of its own escaped: a name is free text,
# and this whole string is handed to a shell on the board — `Pierre's duck` would otherwise arrive
# as a syntax error rather than as a name.
_args=""
if [ -n "$ROBOT_NAME" ]; then
    _args=" --name '$(printf '%s' "$ROBOT_NAME" | sed "s/'/'\\\\''/g")'"
fi

# `-t` so sudo can prompt for a password, and the exit status deliberately ignored: this
# command ends by rebooting the machine it is running on, so ssh reporting a dropped connection
# is the *expected* outcome. Whether it worked is decided below, by looking at the board.
ssh -t -o StrictHostKeyChecking=accept-new "$HOST" \
    "sudo env ${_env} sh /tmp/provision.sh${_args}" || true

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

_was="$HOST_ONLY"
if ! wait_for_board "$BOOT_TIMEOUT"; then
    # Why the fallback did not save this, which is the part the operator cannot see. Said only when
    # it explains something: with a live `btd` and a name, the probe already reported its own
    # failure while the dots were going past.
    _why=""
    if [ -z "$BLE_NAME" ]; then
        _why="
  Bluetooth was not used here, and a new address is exactly what it would have found."
    elif [ -z "$BLE_LIVE" ]; then
        _why="
  Bluetooth had nothing to report, which is expected on a board being provisioned for the first
  time: btd is installed by phase 2, so there was nothing to ask until the install reached it."
    fi

    die "no answer from ${HOST} after ${WAITED}s.
  That does not mean provisioning failed. The board resumes by itself at boot, so this is a
  viewer that lost sight of it — the board may well be finishing right now. Look directly:
    ssh -t ${HOST} 'sudo tail -f ${LOG}'
  If it is genuinely unreachable: a failed wifi cutover makes the backstop restore netplan and
  reboot, which costs a second boot, and NetworkManager may come back on a different DHCP lease
  than netplan had — so check for a new address before concluding the board is down.${_why}"
fi

if [ "$HOST_ONLY" = "$_was" ]; then
    say "back after ~${WAITED}s"
else
    say "back after ~${WAITED}s, at ${HOST_ONLY}"
fi

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

    # Captured rather than tested, because `set -e` would take a bare non-zero return as a
    # failure of this script.
    _left=0
    still_provisioning || _left=$?
    if [ "$_left" = 1 ]; then
        # One more read before leaving. `provision.sh` writes its closing lines and *then*
        # removes the state file, so a loop that breaks the moment the file is gone drops the
        # last thing it said — including which token ended up where, and whether the board came
        # out a dev board. Which is the part worth reading.
        drain_log || true
        break
    fi

    # Phase 2 stopped with an error. The log already says why — it is streamed above — so this
    # ends rather than adding a diagnosis of its own, and names the two commands worth running.
    if [ "$_left" = 3 ]; then
        drain_log || true
        echo
        die "provisioning failed on ${HOST_ONLY}; the reason is the last thing in the log above.
  The board is reachable and whatever ran before the failure is in place, so this is a step to
  fix rather than a board to reflash:
    ssh ${HOST} 'systemctl status ${UNIT_NAME}'
    ssh -t ${HOST} 'sudo cat ${LOG}'"
    fi

    # The board went away mid-install. Not the end of provisioning — it carries on without this
    # watcher — and not necessarily a fault either: the wifi backstop reboots the board when a
    # cutover does not take, and NetworkManager can come back on a lease netplan never had.
    #
    # So this is the second place a new address is worth going to look for, and it reuses the same
    # race the reboot does.
    if [ "$_left" = 2 ]; then
        _was="$HOST_ONLY"
        echo
        warn "lost contact with ${HOST_ONLY} while it was still provisioning. Looking for it."
        if ! wait_for_board "$BOOT_TIMEOUT"; then
            die "${HOST_ONLY} stopped answering after ${WAITED}s of looking, and provisioning had
  not finished. The board carries on by itself, so this is a lost viewer rather than a failed
  install — but it does need finding. It is most likely on a new DHCP lease: check your router's
  leases, and then:
    ssh -t <new address> 'sudo tail -f ${LOG}'"
        fi
        if [ "$HOST_ONLY" = "$_was" ]; then
            say "back after ~${WAITED}s; resuming the log"
        else
            say "back after ~${WAITED}s at ${HOST_ONLY}; resuming the log"
            # The log is the same file on the same board, so the byte offset still holds. What can
            # differ is how it is read: the reader was chosen against a connection that has since
            # been replaced, and the group membership behind it is per-login.
            choose_log_reader || true
        fi
        _quiet=0
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
