#!/bin/sh
# Take apt off the boot path.
#
# Every boot of a stock Armbian image runs `/usr/lib/armbian/armbian-apt-updates` from an `@reboot`
# line in `/etc/cron.d/armbian-updates`. That script's whole job is a *simulated* upgrade —
# `apt-get upgrade -s -qq` — whose output is counted into `/var/cache/apt/archives/updates.number`
# so the login banner can say "3 updates available". On a Zero 3W that simulation pins one of the
# four cores for several seconds, exactly while robotd is bringing up its control loop, and the
# number it produces is read by nothing on a robot: the daemon does not update through apt, and
# the operator does not `apt upgrade` a robot from the motd.
#
# The same reasoning removes the stock `apt-daily` and `apt-daily-upgrade` timers, which are the
# other unattended apt work a board does: a real `apt-get update` shortly after every boot, and —
# with `unattended-upgrades` present, which it is on the Ubuntu-based images — package upgrades
# nobody asked for. A kernel or a userspace library swapping under a robot between two reboots is
# how `setup-rkaiq.sh`'s shim stops matching, and it is not something a fleet of ducks should do on
# its own. Releases reach a robot through `updaterd`; apt is for a human at a shell.
#
# Nothing here touches `apt` itself. `apt-get update`/`install` by hand keep working, and so does
# `setup-board.sh`, which calls them.
#
# Idempotent, never fatal, and run on every install and every update, so a board provisioned
# before this was written gets it at its next update (`docs/design/updater-design.md` §9.1).
# `install.sh` and `hooks/postinstall` both call it; a failure there is a warning, because a
# noisy boot is not worth rolling an update back for.
set -eu

say() { printf 'setup-quiet-boot: %s\n' "$*"; }
warn() { printf 'setup-quiet-boot: warning: %s\n' "$*" >&2; }

# Overridable so the board test can exercise this against a fixture rather than the host's
# real crontab.
ARMBIAN_CRON="${ARMBIAN_CRON:-/etc/cron.d/armbian-updates}"

# The cron file is rewritten rather than deleted, and kept as a file with only comments in it.
# It is a conffile of Armbian's bsp package: dpkg keeps a locally modified conffile across a
# package upgrade, so this survives one, whereas a package postinst is free to recreate a file
# it finds missing. An empty file would work too, but one that says why it is empty is what the
# next person to open it needs.
disable_armbian_update_check() {
    if [ ! -f "$ARMBIAN_CRON" ]; then
        say "no ${ARMBIAN_CRON}; nothing runs apt at boot from cron"
        return 0
    fi
    # Anything that is not a comment or blank is a live crontab line.
    if ! grep -Eq '^[[:space:]]*[^#[:space:]]' "$ARMBIAN_CRON"; then
        say "${ARMBIAN_CRON} already holds no job"
        return 0
    fi
    say "disabling Armbian's boot-time update count in ${ARMBIAN_CRON}"
    cat > "$ARMBIAN_CRON" <<'EOF'
# Emptied by the robot daemon installer (scripts/setup-quiet-boot.sh).
#
# Armbian ships two lines here that run /usr/lib/armbian/armbian-apt-updates at every boot and
# once a day. That is a simulated `apt-get upgrade` whose only reader is the login banner, and it
# holds one core for several seconds during boot on this board. The robot updates through
# updaterd, not apt, so the count it produces is read by nothing here.
#
# Kept as a file, rather than deleted, so the package that owns it does not put its version back.
EOF
    chmod 644 "$ARMBIAN_CRON"
}

# The stock apt timers. Masked rather than disabled: `apt` re-enables them on its own upgrade,
# and a mask is the one state a package postinst does not undo.
mask_apt_timers() {
    if ! command -v systemctl >/dev/null 2>&1; then
        say "no systemctl; skipping the apt timers"
        return 0
    fi
    for unit in apt-daily.timer apt-daily-upgrade.timer; do
        if [ "$(systemctl is-enabled "$unit" 2>/dev/null)" = masked ]; then
            say "${unit} already masked"
            continue
        fi
        say "masking ${unit}"
        # `disable --now` first so a timer already armed for later today does not fire once
        # more before the mask lands; each step tolerated on its own, because an image that
        # never had the unit reports an error here that means nothing.
        systemctl disable --now "$unit" >/dev/null 2>&1 || true
        systemctl mask "$unit" >/dev/null 2>&1 \
            || warn "could not mask ${unit}; it keeps running apt in the background"
    done
}

disable_armbian_update_check
mask_apt_timers
