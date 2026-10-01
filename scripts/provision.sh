#!/bin/sh
# Provision a freshly flashed board, end to end, in as few commands as a reboot allows.
#
#   export DUCK_TOKEN=...                      # only while the repository is private
#   curl -fsSL -H "Authorization: Bearer $DUCK_TOKEN" .../provision.sh -o /tmp/provision.sh
#   sudo DUCK_TOKEN="$DUCK_TOKEN" sh /tmp/provision.sh
#
# It stages the board, reboots, and finishes on its own. `robotctl health` works when you log
# back in. Everything the second half does goes to /var/lib/robot/provision.log, because there
# is nobody watching a terminal by then — and because journald persistence is configured by the
# release this is installing, so on a first boot the journal may be RAM-only.
#
# `DUCK_NO_REBOOT=1` keeps the old four-command shape: it stops and tells you what to run.
#
# `--name Ducky` names the robot at the end — `sudo sh /tmp/provision.sh --name Ducky`. Optional:
# without it the board names itself `duck-<four hex>` from its SoC serial, which is already unique
# per board.
#
# This orchestrates `setup-board.sh`, `migrate-network.sh` and `install.sh`; it does not
# duplicate them. They stay separately runnable, and they stay separate for the reasons each
# one states — different lifetimes, different risks. What they never had is anything tying
# them together, so the operator was the glue: fetch, run, fetch, run, reboot, re-run, re-run,
# fetch, run. Nine steps of clerical work with a reboot in the middle, every one of which is
# an opportunity to do them in the wrong order or forget the second half of the wifi cutover.
#
# ## Why it always asks for a reboot
#
# Not because it always needs one, but because *deciding* would mean either re-deriving what
# the two scripts already decided or parsing their output, and both drift. A reboot on a board
# being provisioned costs thirty seconds. What it buys is worth more than that:
#
#   - Phase 2 runs against live boot config, so the motor UART exists by the time `robotd`
#     starts and a bench board's health report is about its servos rather than its overlays.
#   - Your shell after the reboot is a *new login session*, which is what makes the `robot`
#     group live without `newgrp`. See `create_group`.
#
# ## Why it takes the reboot, when the scripts it calls will not
#
# `setup-board.sh` and `migrate-network.sh` each state they never reboot on their own, and that
# is right for them: they are single-purpose, they can be run on a robot that is doing something
# else, and neither can know what the reboot would interrupt. This script knows — it is only ever
# run on a board being provisioned, where the reboot is not an interruption but the next step.
#
# What that costs is that the second half runs with nobody watching. Two guards, because the
# thing that can go wrong here is a loop rather than a failure:
#
#   1. The resume unit disables itself *before* doing any work, so there is at most one
#      automatic attempt ever. A phase 2 that dies leaves a board to look at, not a board
#      retrying forever.
#   2. `migrate-network.sh` is only re-run when NetworkManager already owns wifi — i.e. the
#      cutover took and the run is just to retire the backstop. If the backstop fired and
#      restored netplan, re-cutting over is exactly what must not happen unattended: it would
#      re-arm the backstop, fail the same way, reboot, and go round again. It says so in the log
#      and leaves wifi alone.
set -eu

# ── knobs ────────────────────────────────────────────────────────────────────
#
# Same names as `install.sh`, which is where most of them end up: a fork or a pinned tag is
# one decision for the whole bring-up rather than one per script.
# Kept separately from the resolved values below, because phase 2 has to tell "the operator
# asked for this" apart from "this is the default" — the state file must not silently win over
# something typed on the phase 2 command line. See `load_state`.
ENV_REPO="${DUCK_REPO:-}"
ENV_REF="${DUCK_REF:-}"
ENV_TOKEN="${DUCK_TOKEN:-}"
ENV_DEV_KEY="${DUCK_DEV_KEY:-}"
ENV_FORCE="${DUCK_FORCE_REINSTALL:-}"
ENV_WEIRD_BLE="${DUCK_WEIRD_BLE:-}"
ENV_GSTREAMER="${DUCK_GSTREAMER:-}"
ENV_RKAIQ="${DUCK_RKAIQ:-}"

REPO="${ENV_REPO:-pollen-robotics/microduck}"
REF="${ENV_REF:-main}"
RAW="https://raw.githubusercontent.com/${REPO}/${REF}/scripts"

# For a private repository: a token with read access to contents. Carried across the reboot in
# the state file rather than asked for twice — see `save_state` for why not `~/.profile`.
#
# It is needed three times, not once, which is why it is worth carrying: fetching these scripts,
# fetching the release, and then *permanently* — `updaterd` reads `GITHUB_TOKEN` from a systemd
# drop-in on every later update check. `install.sh` writes that drop-in when it is given a token,
# so passing it through in phase 2 is what makes updates work after provisioning, not just
# during it. `finish` says where it ended up.
TOKEN="$ENV_TOKEN"

# A name for this robot, or empty to let it name itself. Set by `--name`; see `main`.
#
# Optional on purpose. Without one the robot derives `duck-7f3a` from its SoC serial, which is
# already distinguishable from every other board flashed from the same image — so this improves on
# a working default rather than supplying the only one. That is what keeps a hand-flashed board
# usable, and it is why the default is derived rather than assigned here.
#
# A flag rather than a `DUCK_*` knob, unlike everything above it. Those are environment variables
# because they are passed on — `install.sh` reads the same names, so a fork or a pinned tag is one
# decision for the whole bring-up. A name goes no further than `robotctl system set-name` at the
# end of phase 2. It is also the only thing here that is per-board rather than per-session, and an
# exported name is exactly the kind of thing that is still set when the next board is provisioned.
NAME=""

# Path to `team.dev.pub`, to make this a dev board. Usually somewhere under /tmp because it
# arrived by `scp`, which is why phase 1 copies it somewhere that survives the reboot.
DEV_KEY="$ENV_DEV_KEY"

# Passed straight through to install.sh.
FORCE_REINSTALL="$ENV_FORCE"

# Does this board need the Bluetooth workarounds? Passed to `setup-board.sh`, which is where the
# one setting they need lives.
#
# Off by default because most Radxa Zero 3W units do not need it and the workarounds have a cost:
# `Privacy = device` stops a pad forming a new bond while `btd` advertises, which is why
# a pairing session has to pause `btd` on a board that has it. See `configure_bluetooth` in
# `setup-board.sh` for the split this exists for.
WEIRD_BLE="$ENV_WEIRD_BLE"

# Install the GStreamer stack for `mediad`? Passed to `setup-gstreamer.sh`.
#
# **On** by default, since it installed cleanly and reported correctly on a Radxa Zero 3W
# (GStreamer 1.26.2 from plain Debian trixie, `webrtcbin` registered, `/dev/mpp_service` found).
# That was the agreed trigger, and it is deliberately earlier than "when `mediad` ships": waiting
# for that would leave every board provisioned in between needing a bring-up step someone has to
# remember, which is the failure this wiring exists to avoid.
#
# `DUCK_GSTREAMER=0` turns it off — `--no-gstreamer` on `provision-board.sh`. Empty is *not* off,
# because empty is what an unset environment looks like and the default has to survive that.
#
# Unlike `WEIRD_BLE` this is not a per-board quirk: every robot wants it, which is why it is a
# default rather than a flag anybody has to know about.
GSTREAMER="${ENV_GSTREAMER:-1}"

# Install the camera's 3A engine? Passed to `setup-rkaiq.sh`.
#
# **On** by default, and paired with the GStreamer stack above: that one makes the board able to
# encode a picture, this one makes the picture worth encoding. Without it the ISP has no tuning
# and no 3A loop — green, noisy, and stuck at whatever exposure `mediad` pinned — so a board
# with the stack and not the engine has a camera nobody wants to look at.
#
# `DUCK_RKAIQ=0` turns it off — `--no-rkaiq` on `provision-board.sh`. Empty is *not* off, for
# the reason spelled out above.
RKAIQ="${ENV_RKAIQ:-1}"

# The branch the operator asked for, or empty. Kept apart from `REF` because they answer different
# questions: `REF` is always set — it defaults to `main` — and says where the *scripts* come from,
# while this says whether a *branch build of the daemon* was asked for. Provisioning plainly with no
# `--ref` must install the stable release, not `main`'s dev build.
ASKED_REF="$ENV_REF"

# ── paths ────────────────────────────────────────────────────────────────────

SELF=/usr/local/sbin/robot-provision
STATE_DIR=/var/lib/robot
STATE="${STATE_DIR}/provision.env"

# Where the unattended half writes what it did. A file, not just the journal: journald
# persistence arrives with the drop-in in the release being installed, so on a board's first
# boot the journal can still be RAM-only — and this is the one record of a step nobody watched.
LOG="${STATE_DIR}/provision.log"

# The unit that resumes after the reboot. Left on disk, disabled, once provisioning is done:
# it is a record of what ran, and `ConditionPathExists` on the state file means re-enabling it
# by accident cannot re-run anything.
UNIT=/etc/systemd/system/robot-provision.service
UNIT_NAME=robot-provision.service

# Set when systemd resumed us rather than a human. Changes two things: output goes to $LOG, and
# a failed wifi cutover is reported rather than retried. See the header.
RESUMED=0

# Stop before the reboot instead of taking it, for anyone who wants the steps one at a time.
NO_REBOOT="${DUCK_NO_REBOOT:-}"
# Where a dev key is parked across the reboot. A public key, so 0644 is right; the point of
# moving it is only that /tmp does not survive the reboot this asks for.
DEV_KEY_KEPT="${STATE_DIR}/team.dev.pub"

# Persisted copies the two board scripts leave behind, which is what phase 2 should prefer:
# they are on disk, and re-fetching would be a second chance for the network to fail.
SETUP_SELF=/usr/local/sbin/robot-setup-board
MIGRATE_SELF=/usr/local/sbin/robot-migrate-network
GST_SELF=/usr/local/sbin/robot-setup-gstreamer
RKAIQ_SELF=/usr/local/sbin/robot-setup-rkaiq

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# ── helpers ──────────────────────────────────────────────────────────────────

# Fetch one sibling script into $2. Unlike the `fetch_cmd` in setup-board.sh, which prints a
# command for a human, this one runs it.
fetch() {
    # $1 script name, $2 destination
    if [ -n "$TOKEN" ]; then
        curl -fsSL -H "Authorization: Bearer ${TOKEN}" -o "$2" "${RAW}/$1" && return 0
    else
        curl -fsSL -o "$2" "${RAW}/$1" && return 0
    fi

    # A private repository answers 404, not 401, for a path with no credentials, so `curl -f`
    # reports what looks like a wrong URL. Say which of the two it is rather than leaving the
    # operator to guess at a typo that is not there.
    if [ -n "$TOKEN" ]; then
        die "could not fetch $1 from ${REPO}@${REF}.
  A token was supplied, so a 404 here means it cannot read this repository — check that it
  has Contents:Read on ${REPO}, and that any SSO authorisation was granted."
    fi
    die "could not fetch $1 from ${REPO}@${REF}.
  No DUCK_TOKEN was supplied. While the repository is private every fetch needs one, and
  GitHub answers 404 rather than 401, so this looks like a wrong URL and is not."
}

# Leave a copy at $SELF, so the command this prints for after the reboot exists.
#
# Not possible when piped (`curl | sh`): there is no file to copy, `$0` is the shell. That is
# why the documented invocation downloads first — and why this refuses rather than carrying on
# into a state whose second half cannot be reached.
persist_self() {
    case "$0" in
        */*) ;;
        *) die "run this from a file, not a pipe:
  curl -fsSL ${RAW}/provision.sh -o /tmp/provision.sh
  sudo sh /tmp/provision.sh
  Phase 2 runs after a reboot, so there has to be something left on disk to run." ;;
    esac

    # Both sides resolved, and the copy never fatal. Two ways this bites otherwise, and the
    # second one is why phase 2 died on its first line during testing:
    #
    #   - `$0` and $SELF can name the same file by different spellings — a symlinked /tmp is
    #     enough — so comparing one resolved path against one literal says "different" about a
    #     file that is not.
    #   - `install` refuses to copy a file over itself, and every resumed run is exactly that
    #     case. Under `set -eu` that exit status ends provisioning before it starts, with
    #     `install: ... are the same file` as the only clue.
    _src="$(readlink -f "$0" 2>/dev/null || printf '%s' "$0")"
    _dst="$(readlink -f "$SELF" 2>/dev/null || printf '%s' "$SELF")"
    if [ "$_src" = "$_dst" ]; then
        return 0
    fi

    install -m 755 "$_src" "$SELF" || warn "could not copy this script to ${SELF}; the
  reboot would then have nothing to resume — finish by hand if that happens."
}

# One `KEY='value'` line for the state file, with any single quote in the value escaped.
#
# Quoted because `load_state` reads that file by sourcing it, and one of the values is free text:
# `PROVISION_NAME=Ducky Two` sources as an assignment plus an attempt to run `Two`, which under
# `set -eu` ends phase 2 on its first line. Naming a robot after two words would then have broken
# provisioning rather than just the name. Applied to every value rather than to the name alone — a
# rule that holds for all of them cannot be got wrong by the next one added.
kv() {
    printf "%s='%s'\n" "$1" "$(printf '%s' "$2" | sed "s/'/'\\\\''/g")"
}

# Write what phase 2 needs to know, 0600, root-only.
#
# This is where the token lives between the two phases. Not `~/.profile`, which was the
# earlier advice: that is a file the operator keeps, readable by their own processes, and it
# outlives provisioning with a credential in it that nobody remembers putting there. This one
# is root-only, in the daemon's own state directory, and `finish` deletes it.
save_state() {
    mkdir -p "$STATE_DIR"
    # Created before it is written, and chmod'ed before anything secret goes in: a token must
    # never exist even briefly in a world-readable file.
    : > "$STATE"
    chmod 600 "$STATE"
    {
        kv DUCK_REPO "$REPO"
        kv DUCK_REF "$REF"
        kv DUCK_TOKEN "$TOKEN"
        kv DUCK_DEV_KEY "$1"
        kv DUCK_FORCE_REINSTALL "$FORCE_REINSTALL"
        kv DUCK_WEIRD_BLE "$WEIRD_BLE"
        kv DUCK_GSTREAMER "$GSTREAMER"
        kv DUCK_ASKED_REF "$ASKED_REF"
        # `PROVISION_*` for the two that are not environment knobs, so sourcing this file cannot
        # set something an operator could also have exported.
        kv PROVISION_NAME "$NAME"
        kv PROVISION_BOOT_ID "$(boot_id)"
    } > "$STATE"
}

# Read it back. Anything the operator actually typed wins over the file, so a token that was
# wrong the first time is corrected on the phase 2 command line rather than by editing a
# root-owned file — the file is a convenience for crossing the reboot, not the authority.
load_state() {
    [ -f "$STATE" ] || return 1

    # shellcheck disable=SC1090  # a generated file of KEY=value lines, written by save_state.
    . "$STATE"

    REPO="${ENV_REPO:-${DUCK_REPO:-$REPO}}"
    REF="${ENV_REF:-${DUCK_REF:-$REF}}"
    TOKEN="${ENV_TOKEN:-${DUCK_TOKEN:-}}"
    DEV_KEY="${ENV_DEV_KEY:-${DUCK_DEV_KEY:-}}"
    FORCE_REINSTALL="${ENV_FORCE:-${DUCK_FORCE_REINSTALL:-}}"
    WEIRD_BLE="${ENV_WEIRD_BLE:-${DUCK_WEIRD_BLE:-}}"
    GSTREAMER="${ENV_GSTREAMER:-${DUCK_GSTREAMER:-1}}"
    ASKED_REF="${ENV_REF:-${DUCK_ASKED_REF:-}}"
    # No `ENV_` mirror for the name, unlike its neighbours. Theirs exist because sourcing this file
    # sets the very `DUCK_*` variables the operator's environment did, so the typed value has to be
    # kept somewhere else to still win. Nothing else writes `PROVISION_NAME`, so a `--name` on the
    # phase 2 command line survives the sourcing and can simply be preferred.
    NAME="${NAME:-${PROVISION_NAME:-}}"
    RAW="https://raw.githubusercontent.com/${REPO}/${REF}/scripts"
    return 0
}

# Which boot this is, as the kernel's own opaque id — it changes on every boot and on nothing
# else. Phase 1 records it and phase 2 compares, which is how "have you actually rebooted" is
# answered without trusting anything else.
#
# Deliberately not the state file's timestamp against uptime, which was the first attempt and
# is wrong on this hardware: the board has no battery-backed RTC, starts at 1970, and NTP steps
# the clock *during* provisioning — `deploy/README.md` calls that out for TLS. File-time
# arithmetic would be comparing two clocks that disagree by decades. There is no clock in this.
boot_id() {
    cat /proc/sys/kernel/random/boot_id 2>/dev/null || true
}

# Is this still the boot that ran phase 1?
#
# Phase 2 installs the daemon, and until the reboot the overlay is staged rather than live, so
# there is no /dev/ttyS2 — `robotd` would start, see no bus, and report a hardware fault that
# is really an operator who skipped a step. Answers no when the id is unavailable, which fails
# towards letting provisioning continue: refusing on a board that cannot tell us would strand
# it with no way forward at all.
same_boot_as_phase_one() {
    _now="$(boot_id)"
    [ -n "$_now" ] || return 1
    [ -n "${PROVISION_BOOT_ID:-}" ] || return 1
    [ "$_now" = "$PROVISION_BOOT_ID" ]
}

# Create the `robot` group and put the operator in it — in phase 1, which is the whole point.
#
# `install.sh` does this too, and correctly, but it runs *last*: by then the operator's shell
# started before the group existed, a process's groups are fixed at exec, and no one can add a
# group to a running process. So provisioning ended with `newgrp robot` or a logout, on a
# board that had just told them everything was fine.
#
# Doing it here means the reboot does that work: the operator reconnects, that is a new login
# session, and it has the group already. Nothing to run, nothing to explain.
#
# Needs nothing from the release — `groupadd --system` is self-contained. install.sh still
# installs the sysusers.d file from the verified release afterwards, and still owns the
# decision; it just finds the group present and the operator already a member, and says so.
create_group() {
    if getent group robot >/dev/null; then
        say "robot group exists"
    else
        say "creating the robot group"
        groupadd --system robot \
            || die "could not create the robot group; neither daemon will start without it"
    fi

    operator="${SUDO_USER:-}"
    if [ -z "$operator" ] || [ "$operator" = root ]; then
        return 0
    fi

    if id -nG "$operator" 2>/dev/null | tr ' ' '\n' | grep -qx robot; then
        say "${operator} is in the robot group"
        return 0
    fi

    if usermod -aG robot "$operator"; then
        say "added ${operator} to the robot group — the reboot below makes it live"
    else
        warn "could not add ${operator} to the robot group; robotctl will need sudo"
    fi
}

# Park a dev key somewhere that survives the reboot, and answer with where it went.
#
# The documented way to get one onto a board is `scp` into /tmp, and /tmp is cleared by the
# reboot between the two phases. Without this, `DUCK_DEV_KEY=/tmp/team.dev.pub` produces a
# board that provisions cleanly and is silently not a dev board — the failure only shows up
# later as `--ref` being refused, which reads like a broken release.
keep_dev_key() {
    [ -n "$DEV_KEY" ] || return 0
    [ -f "$DEV_KEY" ] || die "DUCK_DEV_KEY=${DEV_KEY} is not a readable file.
  Pass the *public* half — team.dev.pub. install.sh validates it properly in phase 2; this
  only checks it is there, because finding out after a reboot is worse."

    mkdir -p "$STATE_DIR"
    install -m 644 "$DEV_KEY" "$DEV_KEY_KEPT"
    say "kept the dev key at ${DEV_KEY_KEPT} — /tmp does not survive the reboot"
}

# Install and enable the unit that finishes this after the reboot.
#
# `ConditionPathExists` on the state file rather than only `disable`, so the two ways this could
# run again — a stale enablement, someone re-enabling it by hand — both stop at a board that has
# already finished. Belt and braces on a unit whose failure mode is a reboot loop.
#
# Output goes to a file rather than the journal, which is deliberate and slightly unusual: the
# thing that configures journald persistence is the drop-in inside the release being installed,
# so during this exact window the journal can still be RAM-only. A log that a power cut erases
# is not much of a record of the one step nobody watched.
install_resume_unit() {
    # Create the log before systemd does, so it exists with a mode chosen here rather than
    # whatever the unit's umask produces. `append:` opens an existing file and leaves its
    # permissions alone, which is what makes this work.
    #
    # root:robot 0640, not 0600: the operator is in `robot` by the time they log back in, so
    # they can read it without sudo — and needing sudo is not a small inconvenience here. The
    # watcher on their laptop reads this over a non-interactive ssh, where sudo cannot prompt,
    # so a root-only log meant that watcher silently displayed nothing at all while the board
    # provisioned perfectly well behind it.
    #
    # Not world-readable either. Nothing deliberately writes the token here, but this captures
    # the whole of install.sh, and "no secret has ever appeared in this output" is a claim about
    # every line of a program that keeps changing.
    : > "$LOG"
    chmod 640 "$LOG"
    chgrp robot "$LOG" 2>/dev/null || warn "no robot group yet; ${LOG} stays root-only"

    cat > "$UNIT" <<EOF
[Unit]
Description=Finish robot provisioning after the reboot
Documentation=https://github.com/${REPO}/blob/${REF}/deploy/README.md
# Never run on a board that has already finished: \`finish\` removes this file.
ConditionPathExists=${STATE}
# install.sh downloads a release, so this needs a network that is actually up — not merely
# configured. After the wifi cutover that means NetworkManager-wait-online.
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
ExecStart=${SELF} --resumed
StandardOutput=append:${LOG}
StandardError=append:${LOG}
# Long enough for an apt install, a release download and a health gate on a slow board.
TimeoutStartSec=1800

[Install]
WantedBy=multi-user.target
EOF
    chmod 644 "$UNIT"
    systemctl daemon-reload
    systemctl enable "$UNIT_NAME" >/dev/null 2>&1 \
        || die "could not enable ${UNIT_NAME}; nothing would finish this after the reboot"
}

# Disabled *before* phase 2 does anything, so there is at most one automatic attempt ever.
#
# The unit file stays on disk as a record of what ran. What must not survive is the enablement:
# if phase 2 dies halfway, the next boot has to leave the board alone for someone to look at,
# rather than trying again into the same wall.
disable_resume_unit() {
    systemctl disable "$UNIT_NAME" >/dev/null 2>&1 || true
}

# Does NetworkManager own wlan0? Three lines of nmcli rather than a call into
# migrate-network.sh, because the question here is *whether to invoke it at all*.
nm_owns_wifi() {
    command -v nmcli >/dev/null 2>&1 || return 1
    case "$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | sed -n 's/^wlan0://p')" in
        ''|unmanaged) return 1 ;;
        *) return 0 ;;
    esac
}

# Take the reboot, having said so first.
#
# The delay is the whole courtesy: an SSH session is about to end, and ten seconds is enough to
# read why and press Ctrl-C. It is not a prompt — this has to work when stdin is not a terminal,
# which it is not under `sudo sh` from anything scripted.
reboot_now() {
    cat <<EOF

  Rebooting in 10 seconds. Ctrl-C now to stop, then finish by hand with:
      sudo ${SELF}

  After the reboot this continues on its own and logs to:
      ${LOG}
  Watch it with:  sudo tail -f ${LOG}
EOF
    sleep 10
    say "rebooting"
    systemctl reboot
}

# ── phases ───────────────────────────────────────────────────────────────────

phase_one() {
    say "phase 1: board and network"
    # Before anything is changed: a mistyped dev-key path should cost nothing at all, and this
    # is the only argument that can be wrong in a way nothing later would catch.
    keep_dev_key
    create_group

    tmp=/tmp/setup-board.sh
    fetch setup-board.sh "$tmp"
    DUCK_WEIRD_BLE="$WEIRD_BLE" sh "$tmp"

    tmp=/tmp/migrate-network.sh
    fetch migrate-network.sh "$tmp"
    sh "$tmp"

    if [ -n "$DEV_KEY" ]; then
        save_state "$DEV_KEY_KEPT"
    else
        save_state ""
    fi

    say "phase 1 done — both changes are staged, and a device-tree overlay and a network stack"
    say "cannot swap under a running kernel, so the rest happens after a reboot"

    if [ -n "$NO_REBOOT" ]; then
        cat <<EOF

  DUCK_NO_REBOOT is set, so this stops here:

      sudo reboot
      sudo ${SELF}

EOF
        return 0
    fi

    install_resume_unit
    reboot_now
}

phase_two() {
    say "phase 2: confirm the board, then install the daemon"

    # The persisted copies, which is what those scripts leave behind for exactly this moment.
    # Re-fetching would work and would also be a second chance for the network to fail.
    if [ -x "$SETUP_SELF" ]; then
        DUCK_WEIRD_BLE="$WEIRD_BLE" "$SETUP_SELF"
    else
        tmp=/tmp/setup-board.sh
        fetch setup-board.sh "$tmp"
        DUCK_WEIRD_BLE="$WEIRD_BLE" sh "$tmp"
    fi

    # GStreamer, unless turned off — see `GSTREAMER` above.
    #
    # Here rather than in phase 1 because it changes no boot config and needs no reboot: it is
    # apt packages and a report. Phase 1 exists for the two things that cannot swap under a
    # running kernel, and this is neither.
    if [ -n "$GSTREAMER" ] && [ "$GSTREAMER" != 0 ]; then
        if [ -x "$GST_SELF" ]; then
            "$GST_SELF"
        else
            tmp=/tmp/setup-gstreamer.sh
            fetch setup-gstreamer.sh "$tmp"
            sh "$tmp"
        fi
    fi

    # The camera's 3A engine, on the same terms — see `RKAIQ` above. After GStreamer because it
    # ends by restarting the camera stream, and there is no stream to restart until `mediad` has
    # a stack to run on.
    #
    # Two files, not one: the script builds an LD_PRELOAD shim from the C source beside it, and
    # fetching the script alone would leave it with nothing to compile.
    if [ -n "$RKAIQ" ] && [ "$RKAIQ" != 0 ]; then
        if [ -x "$RKAIQ_SELF" ]; then
            "$RKAIQ_SELF"
        else
            tmp=/tmp/setup-rkaiq.sh
            fetch setup-rkaiq.sh "$tmp"
            fetch rkaiq-modinfo-shim.c /tmp/rkaiq-modinfo-shim.c
            sh "$tmp"
        fi
    fi

    # This run is what retires the wifi backstop. Left armed, any later boot where wifi is
    # merely slow reverts this board to netplan.
    #
    # Conditional on the cutover having *worked*, and only when nobody is watching. If the
    # backstop fired and put netplan back, re-running the migration would re-arm it, fail the
    # same way, reboot, and go round again — a loop, unattended, on a board that is at least
    # reachable. Reported instead, and wifi left alone.
    if nm_owns_wifi; then
        if [ -x "$MIGRATE_SELF" ]; then
            "$MIGRATE_SELF"
        else
            tmp=/tmp/migrate-network.sh
            fetch migrate-network.sh "$tmp"
            sh "$tmp"
        fi
    elif [ "$RESUMED" = 1 ]; then
        warn "wifi is not NetworkManager's, so the cutover did not take — most likely the
  backstop restored netplan and rebooted. Not retrying it unattended: that would re-arm the
  backstop and loop. The board is reachable on netplan and the install below continues.
  When you can watch it:  sudo ${MIGRATE_SELF}"
    else
        # A human is here, so let the migration decide for itself — it is idempotent, and
        # retrying in front of someone is exactly the right time to retry.
        if [ -x "$MIGRATE_SELF" ]; then
            "$MIGRATE_SELF"
        else
            tmp=/tmp/migrate-network.sh
            fetch migrate-network.sh "$tmp"
            sh "$tmp"
        fi
    fi

    tmp=/tmp/install.sh
    fetch install.sh "$tmp"

    DUCK_REPO="$REPO"
    DUCK_REF="$REF"
    DUCK_TOKEN="$TOKEN"
    DUCK_FORCE_REINSTALL="$FORCE_REINSTALL"
    export DUCK_REPO DUCK_REF DUCK_TOKEN DUCK_FORCE_REINSTALL
    if [ -n "$DEV_KEY" ]; then
        DUCK_DEV_KEY="$DEV_KEY"
        export DUCK_DEV_KEY
    fi
    sh "$tmp"

    # Naming first: it needs nothing from the branch build, and a branch apply that fails is fatal
    # — which, in the other order, cost a board the one thing its operator typed.
    name_the_robot
    apply_asked_ref

    finish
}

# Wait for the restarts an apply deferred to land: the transient `systemd-run` timers `updaterd`
# schedules for itself and `btd` once its reply is on the wire (`RESTART_AFTER_REPLYING` in
# `updater/src/engine.rs`). An *active* timer is one that is pending or whose restart is still
# running, so an empty list means both have happened.
#
# Matched by what they run rather than by name, since systemd names them `run-r<random>`. Not
# fatal on a timeout: a stuck timer is worth a warning, and the apply that follows says whether
# updaterd is usable.
wait_for_deferred_restarts() {
    waited=0
    while systemctl list-units --type=timer --state=active --plain --no-legend 'run-*.timer' \
            2>/dev/null | grep -q 'systemctl restart'; do
        if [ "$waited" -ge 60 ]; then
            warn "a deferred restart is still pending after 60s; carrying on:
$(systemctl list-units --type=timer --state=active --plain --no-legend 'run-*.timer' 2>/dev/null)"
            return 0
        fi
        waited=$((waited + 1))
        sleep 1
    done
}

# Wait for `updaterd` to answer. `update status` answering means it is up and serving, which is
# the earliest point at which anything can be asked of it.
wait_for_updaterd() {
    waited=0
    until robotctl update status >/dev/null 2>&1; do
        if [ "$waited" -ge 90 ]; then
            die "updaterd did not answer within 90s. The board may be mid-rollback. Look at:
    journalctl -u updaterd -b --no-pager
    robotctl health"
        fi
        waited=$((waited + 2))
        sleep 2
    done
}

# Install the daemon that `--ref BRANCH` last built, when a branch was asked for.
#
# `install.sh` cannot do this itself: it resolves the release through GitHub`s `/releases/latest`,
# which excludes pre-releases, and every branch build is one. So provisioning brings the board up on
# the stable release and this puts the branch build on top — which is also the right arrangement
# rather than a workaround. The first release installed becomes **golden**, the boot recovery net`s
# fallback, and a branch build as golden would give a broken branch a broken fallback. This way
# `golden` stays stable and `current` is the branch.
#
# **Fatal, not a warning.** A dev board asked to run a branch and quietly running the stable release
# instead is the failure that is worst to debug: everything looks installed and the code under test
# is not there. Better to stop with the reason.
#
# The check is on what is *running*, not on the exit status, because `apply` has a health gate: a
# branch build that fails it is rolled back to stable and the apply reports that honestly — leaving
# exactly the silent board this exists to prevent.
apply_asked_ref() {
    [ -n "$ASKED_REF" ] || return 0

    say "installing the daemon that ${ASKED_REF} last built"

    # `install.sh`'s bootstrap apply ends like every apply: `updaterd` and `btd` restarted by
    # transient `systemd-run` timers "five seconds" later. Observed on a fresh board, that timer
    # fired seventeen seconds later instead — four seconds into this apply, killing the updaterd
    # performing it, so the branch build died at `Extracting` and the board stayed on stable.
    # So let the restarts land first, then wait for updaterd to answer again.
    wait_for_deferred_restarts
    wait_for_updaterd

    # **The exit status is not the verdict, and cannot be.** The release`s `hooks/postinstall`
    # restarts `updaterd`, which is the process streaming progress back — so a *successful* apply
    # ends with `updaterd closed the connection` and a non-zero exit, every time. Treating that as
    # failure is why this refused a board that had just installed correctly.
    if ! robotctl update apply daemon --ref "$ASKED_REF"; then
        say "the apply did not answer cleanly. Expected: the post-install hook restarts updaterd and
  drops the connection carrying the reply. What is actually running is the verdict, below."
    fi

    # Wait for the restarted daemons before reading anything, or this samples mid-swap. The same
    # two waits as above, for this apply's own deferred restarts: `update status` answering
    # alone can be the *old* updaterd, in the five seconds before its timer fires.
    wait_for_deferred_restarts
    wait_for_updaterd

    # And then a moment more: the health gate runs *after* the swap, so a build that fails it is
    # rolled back seconds later. Reading `current` the instant updaterd answers can catch the new
    # release still in place on a board that is about to revert.
    sleep 5

    live="$(readlink /opt/robot/daemon/current 2>/dev/null | sed 's|releases/||')"
    case "$live" in
        *-dev.*)
            say "running ${live}"
            ;;
        *)
            die "asked for ${ASKED_REF} and this board is running ${live}.
  Either the build could not be installed, or it was installed and rolled back by the health gate.
  The board is on the stable release and working; the branch is what needs looking at:
    journalctl -u updaterd -b --no-pager
    journalctl -u robotd -b --no-pager
  Common causes: CI has not published the build yet (gh run list --branch ${ASKED_REF}), the
  branch has no build at all, or this is not a dev board so a dev-signed build is refused
  (grep -c 'DEV BOARD' ${LOG})."
            ;;
    esac
}

# Give this board the name the operator asked for, if they asked for one.
#
# Through `configd`'s socket rather than by editing its file: one writer, `flock` for the write,
# the same validation an app gets, and truncation to the 24 bytes a BLE advertisement has room
# for. Editing `config.json` here would race the running daemon and skip all of it.
#
# After `install.sh`, because that is what enables `configd` — before it there is no socket to
# talk to.
name_the_robot() {
    [ -n "$NAME" ] || return 0

    if _stored="$(robotctl system set-name "$NAME" 2>&1)"; then
        say "named this robot:
${_stored}"
    else
        # Not fatal. The robot still answers to its derived default, so this costs a nicer name
        # rather than a working one — but it is the one thing the operator typed that quietly did
        # not happen, so it does not pass in silence.
        warn "could not name this robot '${NAME}':
  ${_stored}
  It keeps its derived default. Retry with:  robotctl system set-name '${NAME}'"
    fi
}

# Drop the state file, and account for the token that is *supposed* to stay.
#
# There are two copies of that credential on this board and they have opposite lifetimes. The
# state file existed only to cross the reboot, so it goes. The systemd drop-in `install.sh`
# writes is the one `updaterd` reads on every check from here on — a private repository's
# release assets are unreachable without it, so removing that one would leave a robot that is
# installed, running, and unable to fetch a single update. Which is most of what it is for.
#
# Said out loud because "provisioning complete, removed the state file" reads as *the token is
# gone from this board*, and it is not. On a developer board that is the intended outcome; it
# is also the thing you would want to know before handing the board to anyone.
finish() {
    rm -f "$STATE"
    say "provisioning complete; removed ${STATE}"

    _dropin=/etc/systemd/system/updaterd.service.d/token.conf
    if [ -n "$TOKEN" ]; then
        if [ -f "$_dropin" ]; then
            warn "this board keeps your token in ${_dropin} (mode 600). That is deliberate —
  updaterd cannot reach a private repository's release assets without it — and it means this
  board holds a credential that cannot be rotated without coming back to it. Fine for a
  developer board, never for one you ship."
        else
            # The failure this whole path exists to prevent, and it is silent: updates fail on
            # a timer, forever, with nothing in front of a human.
            warn "a token was supplied but ${_dropin} does not exist, so updaterd has no
  credential and every update check will 404 against a private repository — quietly, on a
  timer. Re-run install.sh with DUCK_TOKEN set, and check its output for that step."
        fi
    else
        say "no token on this board; updaterd can only fetch from a public repository"
    fi

    if [ "$RESUMED" = 1 ]; then
        # Nobody is reading this as it happens; it is being written to $LOG for whoever logs in
        # next. So the closing line is addressed to them, not to a terminal.
        say "nothing left to do — log in and run: robotctl health"
        return 0
    fi

    cat <<'EOF'

  robotctl health

That works in this shell: the group predates your current login session, because phase 1
created it before the reboot.
EOF
}

main() {
    [ "$(id -u)" = 0 ] || die "run as root — re-run that same command with sudo"
    command -v curl >/dev/null 2>&1 || die "curl is required"

    while [ $# -gt 0 ]; do
        case "$1" in
            --name)    NAME="${2:?--name needs a name}"; shift 2 ;;
            --resumed) RESUMED=1; shift ;;
            *) die "unknown argument: $1 (--name NAME, or --resumed, which systemd passes)" ;;
        esac
    done

    if [ "$RESUMED" = 1 ]; then
        # Before the work, not after: one automatic attempt, whatever happens next.
        disable_resume_unit
        mkdir -p "$STATE_DIR"
        say "resumed by systemd after the reboot; logging to ${LOG}"
    fi

    if load_state; then
        if same_boot_as_phase_one; then
            die "phase 1 has run but this board has not rebooted since.
  Phase 2 installs the daemon, and until the reboot the overlay is staged rather than live —
  so there is no /dev/ttyS2, and robotd would start and report a hardware fault that is
  really a missing reboot.
    sudo reboot
  Then this finishes on its own, unless DUCK_NO_REBOOT was set — in which case:
    sudo ${SELF}"
        fi
        persist_self
        phase_two
        return 0
    fi

    persist_self
    phase_one
}

main "$@"
