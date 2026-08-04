#!/bin/sh
# Get a freshly flashed board ready to run the robot, then say whether it is.
#
# Split from `install.sh` on purpose. This does OS-level bring-up — device-tree overlays,
# ONNX Runtime — which changes rarely, needs a reboot, and belongs to the *board*.
# `install.sh` installs a signed daemon release, which happens on every update and belongs
# to the *software*. Conflating them would mean every update re-litigating boot config.
#
# Idempotent, and safe to re-run. It never reboots on its own: if it changes anything that
# needs one, it says so and stops, and running it again afterwards continues.
#
#   sudo sh setup-board.sh
#   sudo reboot                     # only if it asks
#   sudo /usr/local/sbin/robot-setup-board
#
# The first run copies itself to that path. /tmp does not survive a reboot, and a script
# whose whole job is "change boot config, reboot, confirm" that then deletes itself across
# the reboot is a bad joke to play on whoever is holding the board.
#
# Radxa Zero 3W on Armbian. Nothing here is specific to a robot revision.
set -eu

ONNX_VERSION="${ONNX_VERSION:-1.20.1}"
ONNX_LIB_DIR=/usr/local/lib

# The Dynamixel bus. Every servo and the imu_to_dxl board share it, so without this there
# is no robot — just a daemon reporting that it cannot see one.
MOTOR_PORT="${MOTOR_PORT:-/dev/ttyS2}"

ENV_TXT=/boot/armbianEnv.txt

# Where this script puts itself so it is still around after the reboot it asks for.
SELF=/usr/local/sbin/robot-setup-board

# Only what `robotd` needs. The prototype also enables i2c-gpio-pihat, aic3104-pihat and a
# camera overlay; none apply here — our IMU rides the Dynamixel bus rather than I²C, and
# `robotd` owns no camera or audio.
REQUIRED_OVERLAY=uart2-m0

needs_reboot=0
# Whether we managed to leave a persistent copy, which decides what the reboot advice says.
persisted=0

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# Leave a copy somewhere that survives a reboot.
#
# Not possible when piped (`curl | sh`), because then there is no file to copy — `$0` is the
# shell. That is fine; the reboot message adapts.
persist_self() {
    case "$0" in
        sh|-sh|bash|-bash|/dev/fd/*|/proc/self/fd/*) return 0 ;;
    esac
    [ -f "$0" ] || return 0

    # Already running from the installed copy: nothing to do, and copying a file onto
    # itself would truncate it.
    if [ "$(readlink -f "$0" 2>/dev/null)" = "$(readlink -f "$SELF" 2>/dev/null)" ]; then
        persisted=1
        return 0
    fi

    if install -m 0755 "$0" "$SELF" 2>/dev/null; then
        persisted=1
    else
        warn "could not copy this script to ${SELF}; you will need to fetch it again after
  the reboot."
    fi
}

check_environment() {
    [ "$(id -u)" = 0 ] || die "run as root (sudo sh setup-board.sh)"

    arch="$(uname -m)"
    [ "$arch" = aarch64 ] || die "this targets aarch64 boards, and this box is ${arch}"

    for tool in curl tar find install; do
        command -v "$tool" >/dev/null 2>&1 || die "${tool} is required"
    done
}

# Enable the UART the Dynamixel bus lives on.
#
# Two traps here, both of which fail *silently* — which is why this is scripted rather than
# written up as a checklist:
#
#  1. Armbian ships `overlay_prefix=rk35xx`, but the RK3566 shares device-tree overlays with
#     the RK3568 and they are named `rk3568-*.dtbo`. With the wrong prefix the loader finds
#     nothing, boots happily, and there is no /dev/ttyS2.
#  2. `armbian-config`'s overlay editor crashes on this board for the same reason
#     (`Invalid overlay_prefix rk35xx`), so the file is patched directly.
#
# A kernel upgrade that repoints /boot/{Image,dtb,uInitrd} can undo this. If a board stops
# seeing its motors after an apt upgrade, re-run this.
configure_overlay() {
    if [ ! -f "$ENV_TXT" ]; then
        warn "no ${ENV_TXT}; not an Armbian image?
  Enable the UART that ${MOTOR_PORT} lives on by whatever means this image provides, then
  re-run. Everything else here will still be done."
        return 0
    fi

    changed=0

    if grep -Eq '^overlay_prefix=rk35xx$' "$ENV_TXT"; then
        say "fixing overlay_prefix: rk35xx -> rk3568"
        sed -i 's/^overlay_prefix=rk35xx$/overlay_prefix=rk3568/' "$ENV_TXT"
        changed=1
    elif ! grep -Eq '^overlay_prefix=' "$ENV_TXT"; then
        say "setting overlay_prefix=rk3568"
        echo 'overlay_prefix=rk3568' >> "$ENV_TXT"
        changed=1
    fi

    if ! grep -Eq '^overlays=' "$ENV_TXT"; then
        say "adding overlays=${REQUIRED_OVERLAY}"
        echo "overlays=${REQUIRED_OVERLAY}" >> "$ENV_TXT"
        changed=1
    elif ! grep -E '^overlays=' "$ENV_TXT" | grep -qw "$REQUIRED_OVERLAY"; then
        say "adding ${REQUIRED_OVERLAY} to overlays"
        # Appended rather than replacing the line: whatever else this image enables is not
        # ours to remove.
        sed -i "s/^overlays=\(.*\)\$/overlays=\1 ${REQUIRED_OVERLAY}/" "$ENV_TXT"
        changed=1
    fi

    if [ "$changed" = 1 ]; then
        needs_reboot=1
    else
        say "device-tree overlays already correct"
    fi
}

# ONNX Runtime, which `robotd` dlopens to run its gait policy.
#
# A board prerequisite rather than release cargo: it changes far less often than the daemon,
# so shipping ~20 MB of it in every artifact would enlarge every update for nothing. The
# consequence is that it is loaded at runtime, not linked — a board without it installs and
# starts fine and *then* cannot walk. `robotd` reports that through `robot.health` with the
# searched path in the message, so the failure names itself, but it is still a failure.
install_onnxruntime() {
    if [ -f "${ONNX_LIB_DIR}/libonnxruntime.so" ]; then
        say "ONNX Runtime already present in ${ONNX_LIB_DIR}"
        return 0
    fi

    url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-aarch64-${ONNX_VERSION}.tgz"
    tmp="$(mktemp -d)"
    say "installing ONNX Runtime ${ONNX_VERSION}"

    if ! curl -fsSL -o "${tmp}/ort.tgz" "$url"; then
        rm -rf "$tmp"
        die "cannot download ONNX Runtime from ${url}
  robotd needs it to run a policy. Install it by hand into ${ONNX_LIB_DIR}, or point
  ORT_DYLIB_PATH at wherever it lives."
    fi

    tar -xzf "${tmp}/ort.tgz" -C "$tmp" || { rm -rf "$tmp"; die "cannot unpack ONNX Runtime"; }

    found="$(find "$tmp" -name 'libonnxruntime.so*' -type f | head -1)"
    [ -n "$found" ] || { rm -rf "$tmp"; die "no libonnxruntime.so in the tarball"; }

    install -m 0644 "$found" "${ONNX_LIB_DIR}/$(basename "$found")"
    ln -sf "$(basename "$found")" "${ONNX_LIB_DIR}/libonnxruntime.so"

    # ldconfig lives in /usr/sbin, often absent from a login PATH, so `command -v` would
    # report it missing and skip the refresh — leaving the freshly copied library
    # unfindable by dlopen. Try the absolute path too.
    if command -v ldconfig >/dev/null 2>&1; then
        ldconfig
    elif [ -x /usr/sbin/ldconfig ]; then
        /usr/sbin/ldconfig
    else
        warn "no ldconfig; robotd may need ORT_DYLIB_PATH=${ONNX_LIB_DIR}/libonnxruntime.so"
    fi

    rm -rf "$tmp"
}

# Take the login console off the motor UART.
#
# UART2 is the RK3566 debug console, so Armbian runs `serial-getty@ttyS2` on it by default.
# A getty does not merely hold the port open — it *reads* from it, consuming the Dynamixel
# replies before `robotd` ever sees them. Every servo then looks absent, which is
# indistinguishable from hardware that is unpowered or unwired.
#
# That is not a hypothetical: it cost an afternoon of staring at
# `read return_delay_time on 20: Operation timed out` with a correctly wired robot attached
# and every servo visible to other tools. `fuser -v /dev/ttyS2` naming `agetty` was the
# first honest evidence.
#
# Two halves, because two things write to that UART:
#
#  1. The getty, which is masked rather than merely disabled — `getty.target` pulls it back
#     in otherwise.
#  2. The kernel's own console. Armbian's `console=both`/`console=serial` puts printk on the
#     same wires as the servos, so a kernel message mid-transaction corrupts a reply. It is
#     quiet most of the time, which makes it worse: an intermittent bus fault with no
#     pattern. `console=display` is the supported Armbian value that keeps a console on HDMI
#     and takes it off the UART.
#
# A UART cannot be both a console and a motor bus. Choosing the motor bus is the whole point
# of this script.
free_motor_port() {
    tty="$(basename "$MOTOR_PORT")"
    unit="serial-getty@${tty}.service"

    if [ "$(systemctl is-enabled "$unit" 2>/dev/null)" = masked ]; then
        say "${unit} already masked"
    else
        say "masking ${unit} so it stops eating servo replies"
        systemctl disable --now "$unit" >/dev/null 2>&1 || true
        if ! systemctl mask "$unit" >/dev/null 2>&1; then
            warn "could not mask ${unit}; it will keep consuming bytes on ${MOTOR_PORT}"
        fi
    fi

    if [ -f "$ENV_TXT" ] && grep -Eq '^console=(both|serial)$' "$ENV_TXT"; then
        say "taking the kernel console off the motor UART (console=display)"
        # Two plain substitutions rather than a BRE alternation, which differs between
        # sed dialects and would fail silently by matching nothing.
        sed -i 's/^console=both$/console=display/' "$ENV_TXT"
        sed -i 's/^console=serial$/console=display/' "$ENV_TXT"
        needs_reboot=1
    fi
}

# What the board looks like now. Printed whether or not anything was changed, because "is
# this board ready" is a question worth being able to ask on its own.
report() {
    say "board status"

    if [ -e "$MOTOR_PORT" ]; then
        printf '  %-22s %s\n' "motor bus" "$MOTOR_PORT present"
    elif [ "$needs_reboot" = 1 ]; then
        printf '  %-22s %s\n' "motor bus" "$MOTOR_PORT absent — enabled, pending reboot"
    else
        printf '  %-22s %s\n' "motor bus" "$MOTOR_PORT ABSENT"
        warn "${MOTOR_PORT} is missing and no overlay change was needed, so something else
  is wrong. Check:  dmesg | grep -iE 'ttyS|serial'
  robotd will start, fail to open the bus, and report unhealthy — which is honest, but it
  will not drive anything."
    fi

    # Named explicitly, because "the port exists" and "the port is usable" are different
    # questions and only the second one matters.
    holder=""
    if command -v fuser >/dev/null 2>&1 && [ -e "$MOTOR_PORT" ]; then
        holder="$(fuser "$MOTOR_PORT" 2>/dev/null | tr -d ' ')"
    fi
    if [ -n "$holder" ]; then
        printf '  %-22s %s\n' "motor bus owner" "IN USE by pid ${holder}"
        warn "something else has ${MOTOR_PORT} open. A reader on this port consumes servo
  replies and every motor will look absent. Identify it with:  sudo fuser -v ${MOTOR_PORT}"
    else
        printf '  %-22s %s\n' "motor bus owner" "free"
    fi

    if grep -qE '(^| )console=tty(S|AMA)' /proc/cmdline 2>/dev/null; then
        printf '  %-22s %s\n' "kernel console" "still on a serial port"
        warn "the kernel prints to a UART (see /proc/cmdline). If that is ${MOTOR_PORT},
  kernel messages will corrupt servo traffic intermittently. Set console=display in
  ${ENV_TXT} and reboot."
    fi

    if [ -f "${ONNX_LIB_DIR}/libonnxruntime.so" ]; then
        printf '  %-22s %s\n' "ONNX Runtime" "present"
    else
        printf '  %-22s %s\n' "ONNX Runtime" "ABSENT — robotd cannot load a policy"
    fi

    # A board with no battery-backed RTC reading 1970 fails TLS certificate validation, and
    # that surfaces as an opaque handshake error several steps into an install.
    if command -v timedatectl >/dev/null 2>&1; then
        if timedatectl show --property=NTPSynchronized --value 2>/dev/null | grep -q yes; then
            printf '  %-22s %s\n' "clock" "NTP-synchronised"
        else
            printf '  %-22s %s\n' "clock" "not synchronised yet"
        fi
    fi

    echo

    if [ "$needs_reboot" = 1 ]; then
        say "reboot required, then run this again"
        echo
        echo "  sudo reboot"
        if [ "$persisted" = 1 ]; then
            echo "  sudo ${SELF}"
        else
            echo "  # then fetch and run this script again — it was not copied anywhere"
            echo "  # persistent, so /tmp will have cleared it"
        fi
        cat <<'EOF'

  Boot configuration changed. Nothing else can be confirmed until the overlay is live, and
  this script is idempotent — running it again after the reboot picks up where it stopped.
EOF
        return 0
    fi

    say "board ready — install the daemon next"
    cat <<'EOF'

  URL=https://raw.githubusercontent.com/pollen-robotics/microduck_daemon/main/scripts/install.sh
  curl -fsSL -H "Authorization: Bearer $DUCK_TOKEN" "$URL" -o /tmp/install.sh
  sudo DUCK_TOKEN="$DUCK_TOKEN" sh /tmp/install.sh

  While the repository is private, both halves need the token: raw.githubusercontent.com
  404s without it, and sudo does not pass the variable through on its own. Once the
  repository is public, drop the header and the prefix.
EOF
}

main() {
    check_environment
    persist_self
    configure_overlay
    free_motor_port
    install_onnxruntime
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a setup.
main "$@"
