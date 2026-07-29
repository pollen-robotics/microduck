#!/bin/sh
#
# Install the robot daemon on a fresh board, from nothing.
#
#   curl -fsSL https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/main/scripts/install.sh | sudo sh
#
# Target: 64-bit Debian userland on aarch64 — Armbian 26.2.x on the Radxa Zero 3, and
# whatever else Debian 12/13 arm64 you point it at. Needs `curl` and coreutils and
# nothing else: tar and zstd are linked into `updaterd`, so there is no package to
# install first.
#
# Idempotent. Re-running it on an installed robot re-checks everything and changes
# nothing, and it never overwrites /etc/robot/updater.toml.
#
# ── how it works ─────────────────────────────────────────────────────────────
#
# The circularity — "an update needs the updater, which arrives in an update" — is broken
# by downloading one bare `updaterd` binary and running its `install` subcommand. That
# runs the ordinary engine: signature verification, extraction, the atomic swap, the
# journal entry. There is no bootstrap-specific install logic, so nothing here can drift
# from how every later update behaves.
#
# Notably this script never parses a manifest. It hands `updaterd` the config and lets the
# configured source resolve `latest`, because a shell script picking the version out of a
# signed JSON document would be a second, weaker reader of that document.
#
# For the same reason only two files come from the repository over raw.githubusercontent:
# the config and the public keys, both needed *before* anything can be verified. The unit
# files and the journald drop-in are taken out of the installed release instead — the same
# bytes a signature was checked against.
#
# ── chain of trust ───────────────────────────────────────────────────────────
#
#   1. TLS to raw.githubusercontent.com gets this script, the config and the public keys.
#   2. TLS to github.com gets the bootstrap `updaterd`. It is NOT yet verified.
#   3. That binary verifies the manifest and artifact against the keys from (1), and
#      refuses to install anything they do not sign.
#   4. Afterwards this script compares the bootstrap binary's sha256 against
#      `current/bin/updaterd`, which came out of the verified artifact. Equal digests
#      mean the binary from (2) was genuine after all. CI asserts the two are the same
#      bytes, so a mismatch is a real finding, not a packaging quirk.
#
# The residual trust is GitHub itself, which is also where this script came from — step
# (4) narrows it rather than removing it. An install that wants no such window should use
# `updaterd install --from <dir>` against files carried in by hand.

set -eu

# ── knobs ────────────────────────────────────────────────────────────────────

# The repository releases are published from. Override for a fork or a test repo.
REPO="${DUCK_REPO:-pollen-robotics/microduck_daemon}"

# Branch the config and trusted keys are read from. Pin to a tag for a reproducible
# provisioning run.
REF="${DUCK_REF:-main}"

# For a private repository: a token with read access to contents. Also used for the
# release download, which needs auth on a private repo.
TOKEN="${DUCK_TOKEN:-}"

RAW="https://raw.githubusercontent.com/${REPO}/${REF}"
BOOTSTRAP_URL="https://github.com/${REPO}/releases/latest/download/updaterd-bootstrap-aarch64"

CONFIG_DIR=/etc/robot
KEYS_DIR="${CONFIG_DIR}/trusted_keys"
INSTALL_DIR=/opt/robot/daemon
UNIT_DIR=/etc/systemd/system

# Public keys expected in the image. All three, not just the one that signs today: a
# robot verifies only against the set baked into it, so this is the single chance to make
# key rotation possible without re-flashing by hand.
KEYS="release-1.pub release-2.pub release-3.pub"

# ── helpers ──────────────────────────────────────────────────────────────────

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

fetch() {
    # $1 url, $2 destination
    if [ -n "$TOKEN" ]; then
        curl -fsSL -H "Authorization: Bearer ${TOKEN}" -o "$2" "$1"
    else
        curl -fsSL -o "$2" "$1"
    fi
}

# ── steps ────────────────────────────────────────────────────────────────────

check_environment() {
    if [ "$(id -u)" != 0 ]; then
        die "run as root (pipe to \`sudo sh\`, not \`sh\`)"
    fi

    arch="$(uname -m)"
    if [ "$arch" != "aarch64" ]; then
        die "this installer publishes aarch64 binaries only, and this box is ${arch}"
    fi

    for tool in curl systemctl sha256sum install; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            die "${tool} is required"
        fi
    done

    case "$REPO" in
        ORG/*)
            die "REPO is still the placeholder '${REPO}'.
  Set DUCK_REPO, or substitute the real repository in scripts/install.sh and
  deploy/updater.toml. A robot installed against a repository that does not exist
  installs fine and then never finds another update."
            ;;
    esac
}

# The board has no battery-backed RTC. A clock reading 1970 fails TLS certificate-date
# validation, and the error surfaces as an opaque handshake failure several steps later —
# `updaterd`'s own preflight checks this for the same reason. Better to say so here.
wait_for_clock() {
    if [ ! -d /run/systemd/system ]; then
        warn "not running under systemd; skipping the clock check"
        return 0
    fi
    if timedatectl show --property=NTPSynchronized --value 2>/dev/null | grep -q yes; then
        return 0
    fi

    say "waiting for the clock to sync (no RTC on this board; TLS needs a real date)"
    i=0
    while [ "$i" -lt 60 ]; do
        if timedatectl show --property=NTPSynchronized --value 2>/dev/null | grep -q yes; then
            return 0
        fi
        sleep 2
        i=$((i + 1))
    done
    warn "the clock is still unsynchronised after 2 minutes; downloads may fail with a
  certificate error. Check the network and systemd-timesyncd."
}

# The trust anchor and the one file an operator is expected to edit. Both have to come
# from the repository rather than from a release, because nothing can be verified until
# the keys are here.
install_config() {
    say "installing config and trusted keys"
    mkdir -p "$KEYS_DIR"
    chmod 755 "$CONFIG_DIR" "$KEYS_DIR"

    for key in $KEYS; do
        # Only release-1 signs today; the spares are the rotation path and may not be
        # committed yet. A missing spare is not fatal, a missing release-1 is.
        if fetch "${RAW}/deploy/trusted_keys/${key}" "${KEYS_DIR}/${key}"; then
            chmod 644 "${KEYS_DIR}/${key}"
        else
            rm -f "${KEYS_DIR}/${key}"
            if [ "$key" = "release-1.pub" ]; then
                die "cannot fetch ${key} from ${RAW}/deploy/trusted_keys/
  Without it nothing can be verified, so there is nothing safe to install."
            fi
            warn "no ${key} published yet; skipping"
        fi
    done

    # Never overwritten. This is the file an operator edits to point a bench robot at a
    # different channel or to allow dev keys, and clobbering that on a re-run is a
    # surprise nobody wants twice.
    if [ -f "${CONFIG_DIR}/updater.toml" ]; then
        warn "keeping the existing ${CONFIG_DIR}/updater.toml"
    else
        fetch "${RAW}/deploy/updater.toml" "${CONFIG_DIR}/updater.toml"
        sed -i "s|\"ORG/duck-daemon\"|\"${REPO}\"|" "${CONFIG_DIR}/updater.toml"
        chmod 644 "${CONFIG_DIR}/updater.toml"
    fi

    if grep -q '"ORG/' "${CONFIG_DIR}/updater.toml"; then
        die "${CONFIG_DIR}/updater.toml still names a placeholder repository"
    fi

    # Same rule: never overwritten, because it is where a bench robot's loop rate and
    # health thresholds get tuned. robotd runs on built-in defaults if it is missing, so
    # this is documentation as much as configuration.
    if [ -f "${CONFIG_DIR}/robotd.toml" ]; then
        warn "keeping the existing ${CONFIG_DIR}/robotd.toml"
    else
        fetch "${RAW}/deploy/robotd.toml" "${CONFIG_DIR}/robotd.toml"
        chmod 644 "${CONFIG_DIR}/robotd.toml"
    fi
}

# Land the first release through the real engine. `--config` is the config installed
# above, so there is one statement of where keys live, where state lives and which channel
# this robot tracks — rather than a copy of those values here that could disagree with the
# one the daemon reads a minute later.
bootstrap_first_release() {
    if [ -L "${INSTALL_DIR}/current" ]; then
        say "a release is already live ($(readlink "${INSTALL_DIR}/current")); skipping the bootstrap"
        return 0
    fi

    tmp="$(mktemp -d)"
    # shellcheck disable=SC2064 # expand $tmp now, deliberately
    trap "rm -rf '$tmp'" EXIT INT TERM

    say "fetching the bootstrap updaterd"
    if ! fetch "$BOOTSTRAP_URL" "${tmp}/updaterd"; then
        die "cannot fetch ${BOOTSTRAP_URL}
  A stable release must exist and carry the updaterd-bootstrap-aarch64 asset. If only
  staging releases have been published, promote one first."
    fi
    chmod +x "${tmp}/updaterd"

    say "installing the first release (verifying signatures)"
    "${tmp}/updaterd" install --config "${CONFIG_DIR}/updater.toml"

    if [ ! -L "${INSTALL_DIR}/current" ]; then
        die "the install reported success but nothing is live"
    fi

    # Close the loop on the one unverified download. The installed binary came out of a
    # signature-verified artifact; if the bootstrap binary matches it byte for byte, the
    # bootstrap binary was genuine too.
    boot_sum="$(sha256sum "${tmp}/updaterd" | cut -d' ' -f1)"
    installed_sum="$(sha256sum "${INSTALL_DIR}/current/bin/updaterd" | cut -d' ' -f1)"
    if [ "$boot_sum" != "$installed_sum" ]; then
        die "the bootstrap binary does not match bin/updaterd in the verified release.
  bootstrap: ${boot_sum}
  installed: ${installed_sum}
  The installed release is signed and safe, but the binary that installed it was not the
  one this release contains. Treat that as a compromised download and investigate."
    fi
    say "bootstrap binary verified against the signed release"

    rm -rf "$tmp"
    trap - EXIT INT TERM
}

# The `robot` group must exist before either unit starts: both declare `Group=robot`, and
# that is what makes their 0660 sockets mean "the robot group" rather than "root only".
# systemd fails the unit outright if the group is missing.
#
# Taken from the installed release rather than from the repository, so it is the copy a
# signature was checked against.
create_group() {
    say "creating the robot group"
    src="${INSTALL_DIR}/current/systemd/sysusers.d/robot.conf"
    if [ -f "$src" ]; then
        mkdir -p /usr/lib/sysusers.d
        install -m 644 "$src" /usr/lib/sysusers.d/robot.conf
        if command -v systemd-sysusers >/dev/null 2>&1; then
            systemd-sysusers
        fi
    fi
    if ! getent group robot >/dev/null; then
        groupadd --system robot
    fi
    if ! getent group robot >/dev/null; then
        die "the robot group could not be created; updaterd.service will not start without it"
    fi
}

# The units live inside the release, so this can only run after the release is installed.
# They are *copied* rather than symlinked through `current`: a unit file read through the
# symlink would change under systemd's feet on every update, and after a rollback
# systemd's view of the world would depend on which release happened to be live at the
# last daemon-reload.
install_units() {
    say "installing systemd units"
    for unit in updaterd.service robotd.service; do
        src="${INSTALL_DIR}/current/systemd/${unit}"
        if [ ! -f "$src" ]; then
            die "the installed release has no systemd/${unit}"
        fi
        install -m 644 "$src" "${UNIT_DIR}/${unit}"
    done

    # journald persistence, so the logs from an incident outlive the reboot that followed
    # it. See docs/deploy.md in the release for the Armbian tmpfs caveat this does not
    # solve on its own.
    src="${INSTALL_DIR}/current/deploy/journald.conf.d/10-robot.conf"
    if [ -f "$src" ]; then
        mkdir -p /etc/systemd/journald.conf.d
        install -m 644 "$src" /etc/systemd/journald.conf.d/10-robot.conf
        systemctl restart systemd-journald || warn "could not restart systemd-journald"
    else
        warn "the release carries no journald drop-in; logs may not survive a reboot"
    fi

    # `robotctl` on PATH, through `current` so it follows the active release. A symlink on
    # purpose here: it is a tool an operator invokes, not a file systemd caches.
    ln -sfn "${INSTALL_DIR}/current/bin/robotctl" /usr/local/bin/robotctl

    systemctl daemon-reload
    systemctl enable --now updaterd.service
    systemctl enable --now robotd.service
}

verify_install() {
    say "verifying"

    # The same list release.yml asserts on, checked here through the symlink the units
    # actually resolve — an artifact can be complete and still be installed wrong.
    for required in bin/updaterd bin/robotd bin/robotctl version.toml; do
        if [ ! -e "${INSTALL_DIR}/current/${required}" ]; then
            die "the installed release is missing ${required}"
        fi
    done

    failed=0
    for unit in updaterd robotd; do
        if systemctl is-active --quiet "$unit"; then
            printf '  %-10s active\n' "$unit"
        else
            printf '  %-10s NOT active\n' "$unit"
            failed=1
        fi
    done

    if [ "$failed" != 0 ]; then
        die "a unit did not come up. Look at:
    journalctl -u updaterd -b --no-pager
    journalctl -u robotd -b --no-pager"
    fi

    # Ask the robot what it is running, which is also the first thing to ask for in any
    # support report. Non-fatal: a daemon that is active but not yet answering is a timing
    # artefact, not a failed install.
    robotctl version || warn "robotctl could not reach the daemons yet"
}

report() {
    version="$(readlink "${INSTALL_DIR}/current" | sed 's|releases/||')"
    say "installed daemon ${version}"
    cat <<'EOF'

  robotctl version                    what is running, and what is installed
  robotctl update status              update state per component
  robotctl update check               is a newer release available
  sudo robotctl update apply daemon   update now (mutations are root-only by design)

This robot polls for updates on its own and will apply a *mandatory* release without
waiting to be asked. Ordinary releases wait for a client.
EOF
}

main() {
    check_environment
    wait_for_clock
    install_config
    bootstrap_first_release
    create_group
    install_units
    verify_install
    report
}

# Called on the last line so a truncated download — the real failure mode of
# `curl | sh` — defines functions and then does nothing, rather than running half an
# install.
main "$@"
