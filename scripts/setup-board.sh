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

# Must satisfy `ort`'s minimum, which is a hard runtime check and not a warning: ort
# 2.0.0-rc.11 requires >= 1.23.x and *panics* in `setup_api` when the dylib is older —
# killing robotd's control thread rather than returning an error. 1.20.1 was pinned here and
# every board provisioned with it could load a policy only far enough to die:
#
#   thread 'control' panicked at ort-2.0.0-rc.11/src/lib.rs:191:41:
#   Failed to load ONNX Runtime dylib: ... expected version >= '1.23.x', but got '1.20.1'
#
# Newer is safe: ort asks for *at least* its `ORT_API_VERSION`, and ONNX Runtime keeps the C
# API backward compatible, so a runtime above the floor serves an older API version happily.
# Raise this in step with `ort` in Cargo.toml — the two are one decision, and only one of them
# is checked at compile time.
ONNX_VERSION="${ONNX_VERSION:-1.28.0}"
ONNX_LIB_DIR=/usr/local/lib

# The Dynamixel bus. Every servo and the imu_to_dxl board share it, so without this there
# is no robot — just a daemon reporting that it cannot see one.
MOTOR_PORT="${MOTOR_PORT:-/dev/ttyS2}"

ENV_TXT=/boot/armbianEnv.txt

# Boot args of the *running* kernel. A variable, like MOTOR_PORT and ENV_TXT above, so the
# console check can be exercised against a fixture instead of only on a board that happens to
# be misconfigured — which is the state you least want to discover the check is wrong in.
CMDLINE="${CMDLINE:-/proc/cmdline}"

BT_CONF=/etc/bluetooth/main.conf

# A gamepad is paired with `sudo robotctl pad pair` on the installed release, with the pad held in
# pairing mode. This script's part of it is the one BlueZ setting a pad needs, which is here because
# it takes a reboot to apply — see `configure_bluetooth`.

# Where this script puts itself so it is still around after the reboot it asks for.
SELF=/usr/local/sbin/robot-setup-board

# Where the sibling scripts come from, for the commands this prints. Same override names as
# `install.sh`, so a fork or a pinned tag is one decision for the whole bring-up rather than
# per script. Nothing here is fetched by this script — see `fetch_cmd`.
REPO="${DUCK_REPO:-pollen-robotics/microduck_daemon}"
REF="${DUCK_REF:-main}"
RAW="https://raw.githubusercontent.com/${REPO}/${REF}/scripts"

# For a private repository: a token with read access to contents. Only ever interpolated into
# the commands this prints, and by name (`$DUCK_TOKEN`) rather than by value — a bring-up log
# gets pasted into chat, and a token that leaks that way cannot be rotated without touching
# every board. What it decides is *which form* to print, not what to run.
TOKEN="${DUCK_TOKEN:-}"

# Wifi migration lives in its own script — see `check_network` for why. Named here so the
# advice this prints and the thing it points at cannot drift apart.
#
# A full path, not a bare filename: the advice is copy-pasted, and `sudo sh migrate-network.sh`
# only works from whichever directory happens to hold it. /tmp is where the fetch this prints
# puts it, and where an operator following the README already has it.
MIGRATE_NAME=migrate-network.sh
MIGRATE="/tmp/${MIGRATE_NAME}"
# Where that script leaves itself once run, which is what to point at after a reboot: by then
# the copy in /tmp is gone, and telling someone to run a file that no longer exists is worse
# than telling them nothing.
MIGRATE_SELF=/usr/local/sbin/robot-migrate-network
NET_CHECK_UNIT=/etc/systemd/system/robot-net-check.service

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

# The command that puts a sibling script on this board, as a string to *print*.
#
# This script downloads exactly one thing — the ONNX Runtime tarball, from a public
# microsoft/onnxruntime release — so it never needs a token itself. Its siblings do, while
# the repository is private, and every step of bring-up that told someone to fetch one
# without a header sent them into a 404 that reads like a wrong URL.
#
# Two forms, keyed on whether this run was given a token, because the wrong one is worse than
# no advice: a private repo not told to send one 404s, and a public one told to send an unset
# or stale one gets an auth failure rather than the file. Printing the form that matches the
# situation this script is actually in is the only version an operator can paste blind.
#
# `$DUCK_TOKEN` stays unexpanded so the printed line is safe to paste into a bug report.
fetch_cmd() {
    # $1 script name, e.g. migrate-network.sh
    if [ -n "$TOKEN" ]; then
        # shellcheck disable=SC2016  # $DUCK_TOKEN must stay literal — see above.
        printf 'curl -fsSL -H "Authorization: Bearer $DUCK_TOKEN" %s/%s -o /tmp/%s' \
            "$RAW" "$1" "$1"
    else
        printf 'curl -fsSL %s/%s -o /tmp/%s' "$RAW" "$1" "$1"
    fi
}

# How to run the wifi migration *from where this board actually is*, as a string to print.
#
# Three states, and naming the wrong one wastes a round trip: once run it lives at a
# persistent path, before that it is a file in /tmp, and on a board that has not fetched it
# there is nothing to run at all — which is the state a fresh board is in, and the one the
# advice used to ignore.
migrate_advice() {
    if [ -x "$MIGRATE_SELF" ]; then
        printf 'sudo %s' "$MIGRATE_SELF"
    elif [ -f "$MIGRATE" ]; then
        printf 'sudo sh %s' "$MIGRATE"
    else
        printf '%s\n    sudo sh %s' "$(fetch_cmd "$MIGRATE_NAME")" "$MIGRATE"
    fi
}

# Which serial port the *running* kernel prints to — bare tty name, no baud — or nothing.
#
# `case` globs rather than a regex: the two substitutions in `free_motor_port` are already
# split for exactly this reason, since BRE alternation differs between sed dialects and fails
# by matching nothing. A check that silently never fires is worse here than no check.
#
# ttyFIQ* counts. It is Rockchip's FIQ debugger rather than an 8250, but it is attached to the
# SoC debug UART — uart2 on the RK3566, which is the motor bus — so a kernel printing there
# lands on the same wires. Worth naming even though the caller then hedges on the mapping.
kernel_console_tty() {
    for arg in $(cat "$CMDLINE" 2>/dev/null || true); do
        case "$arg" in
            console=ttyS*|console=ttyAMA*|console=ttyFIQ*) ;;
            *) continue ;;
        esac
        arg="${arg#console=}"
        # console=ttyS2,1500000 — the baud is not part of the device name.
        printf '%s' "${arg%%,*}"
        return 0
    done
}

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
    # No path in the message: whatever the operator just typed is what needs `sudo` in front,
    # and naming a file here is how the advice drifted from where the file actually is.
    [ "$(id -u)" = 0 ] || die "run as root — re-run that same command with sudo"

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
    # Version-aware, not merely presence-aware. The old check returned early whenever the
    # symlink existed, so a board carrying an incompatible runtime could never be fixed by
    # re-running this script — which is exactly the situation the 1.20.1 pin created.
    #
    # The tarball installs `libonnxruntime.so.<version>` with the bare name as a symlink, so
    # the resolved target names the version without needing to run anything.
    existing=""
    if [ -e "${ONNX_LIB_DIR}/libonnxruntime.so" ]; then
        resolved="$(readlink -f "${ONNX_LIB_DIR}/libonnxruntime.so" 2>/dev/null || true)"
        case "$resolved" in
            */libonnxruntime.so.*) existing="${resolved##*/libonnxruntime.so.}" ;;
            *) existing="unknown" ;;
        esac
    fi

    if [ "$existing" = "$ONNX_VERSION" ]; then
        say "ONNX Runtime ${ONNX_VERSION} already present in ${ONNX_LIB_DIR}"
        return 0
    fi

    if [ -n "$existing" ]; then
        say "replacing ONNX Runtime ${existing} with ${ONNX_VERSION}"
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

# Which stack owns wifi — checked, never changed.
#
# The netplan -> NetworkManager migration lives in `migrate-network.sh` and not here, for two
# reasons that are not about file size. It has a different *lifetime*: it exists only because
# Armbian's stock image ships netplan, and the day we build an image with NM already in it the
# whole thing is deleted, while overlays and ONNX are needed forever. And it has a different
# *risk*: it is the one step that can make a headless board unreachable, so it belongs behind
# an explicit decision rather than inside bring-up you can re-run whenever.
#
# What this does check matters because `configd` drives NetworkManager over D-Bus: a board
# still on netplan answers every `net.*` call with "no such device", which is a confusing
# failure to meet later rather than named here.
check_network() {
    migrate_cmd="$(migrate_advice)"

    if ! command -v nmcli >/dev/null 2>&1; then
        warn "wifi is still netplan's, so configd cannot manage it. Migrate first:
    ${migrate_cmd}
  Then reboot and re-run this. Everything else here is done regardless."
        return 0
    fi

    case "$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | sed -n 's/^wlan0://p')" in
        ''|unmanaged)
            warn "NetworkManager is installed but wlan0 is not its, so the migration is
  incomplete. Finish it with:  ${migrate_cmd}"
            ;;
    esac

    # A backstop left armed reboots the board on any later boot where wifi is merely slow. It
    # is `migrate-network.sh`'s to retire, so say so rather than reaching into its state.
    if [ -f "$NET_CHECK_UNIT" ]; then
        warn "the wifi cutover backstop is still armed. Re-run  ${migrate_cmd}  to retire it,
  or any later boot where wifi comes up slowly will revert this board to netplan."
    fi
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

# The one Bluetooth setting a gamepad needs from this script.
#
# `Privacy = off`, and it is the *opposite* of what this script used to set. The history matters,
# because the old value is still on every board provisioned so far:
#
#   This script set `Privacy = device`, taken from microduck_runtime's installer, whose notes
#   credit it with fixing an Xbox controller that pairs and then drops straight back out — an
#   endless connect/disconnect loop, or `disconnected with reason 3` during pairing.
#
#   With that setting, an Xbox controller cannot bond with this board at all. `btmon` shows LE
#   Secure Connections pairing reaching the last step and the *pad* rejecting it:
#
#       SMP: Pairing Public Key ×2 · Confirm · Random ×2 · DHKey Check
#       > ACL Data RX: SMP: Pairing Failed — Reason: DHKey check failed (0x0b)
#
#   The DHKey check is computed over both devices' addresses, and `Privacy = device` makes the
#   adapter pair from a resolvable private address rather than its public one. The two sides
#   compute different values, and the pad refuses — every time, with no key on either side, and
#   unaffected by retrying, by `JustWorksRepairing`, or by which side starts.
#
#   With `Privacy = off` the same pad pairs first time, is trusted, and reconnects by itself
#   across a reboot. The drop-on-connect symptom the old value was meant to prevent has not
#   returned.
#
# So this is a measurement replacing an inherited setting. If a pad ever does start dropping on
# connect, the two are in genuine tension and the answer is to pair with privacy off and then
# re-enable it — not to set `device` and lose the ability to pair at all.
#
# The change sets `needs_reboot` rather than restarting bluetooth. Restarting the daemon on
# this board left the kernel holding hci0 while bluetoothd reported "No default controller
# available", which needs a reboot to clear — so a reboot is the honest instruction, not an
# extra step.
configure_bluetooth() {
    if [ ! -f "$BT_CONF" ]; then
        warn "no ${BT_CONF}; skipping the gamepad Bluetooth settings"
        return 0
    fi

    # Written explicitly even though `off` is BlueZ's default: a board provisioned by an older
    # copy of this script has `Privacy = device` in the file, and that line has to be *corrected*
    # rather than left alone. An absent setting and a wrong one need different work.
    if grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*off' "$BT_CONF"; then
        say "bluetooth Privacy already off"
    else
        say "setting Privacy = off in ${BT_CONF} (a pad cannot bond otherwise)"
        if grep -Eq '^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=' "$BT_CONF"; then
            sed -i -E 's|^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=.*|Privacy = off|' "$BT_CONF"
        elif grep -q '^\[General\]' "$BT_CONF"; then
            sed -i '/^\[General\]/a Privacy = off' "$BT_CONF"
        else
            printf '\n[General]\nPrivacy = off\n' >> "$BT_CONF"
        fi
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

    # Gamepad readiness. This board's part of it is one setting; who may read the pad is
    # `padd.service`'s business now, and pairing one is `sudo robotctl pad pair`.
    if [ -f "$BT_CONF" ] && grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*off' "$BT_CONF"; then
        printf '  %-22s %s\n' "bluetooth privacy" "off"
    elif [ -f "$BT_CONF" ] && grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*device' "$BT_CONF"; then
        # Named rather than lumped in with "not off": this is what older boards have, and it is
        # the one value that makes pairing a pad impossible. See `configure_bluetooth`.
        printf '  %-22s %s\n' "bluetooth privacy" "device — a pad CANNOT bond; re-run this script"
    else
        printf '  %-22s %s\n' "bluetooth privacy" "not set (BlueZ defaults to off, which works)"
    fi

    # The device node is what gilrs opens, so this is the only claim that matters.
    #
    # A glob rather than `ls`: an unmatched glob stays literal in sh, so the `-e` test is what
    # distinguishes "no pad" from a device actually being there.
    pads=""
    for node in /dev/input/js*; do
        [ -e "$node" ] || continue
        pads="${pads}${node} "
    done
    if [ -n "$pads" ]; then
        printf '  %-22s %s\n' "gamepad" "$pads"
    else
        printf '  %-22s %s\n' "gamepad" "none connected"
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

    # `/proc/cmdline` is the kernel that is *running*; `free_motor_port` edits ${ENV_TXT} for
    # the kernel that will run *next*. They cannot agree until a reboot — so on the very run
    # that fixed this, an unqualified "still on a serial port / set console=display" reads as
    # "the fix did not take", and costs a round trip to disprove. Three distinct states.
    console_tty="$(kernel_console_tty)"
    if [ -n "$console_tty" ]; then
        if [ "/dev/${console_tty}" = "$MOTOR_PORT" ]; then
            console_what="${console_tty} (the motor bus)"
        else
            console_what="${console_tty}"
        fi

        if [ -f "$ENV_TXT" ] && grep -q '^console=display$' "$ENV_TXT"; then
            if [ "$needs_reboot" = 1 ]; then
                # Already handled. Say which way it is going, and do not warn.
                printf '  %-22s %s\n' "kernel console" "${console_what}, until the reboot"
            else
                printf '  %-22s %s\n' "kernel console" "${console_what} — CONFLICT"
                warn "${ENV_TXT} says console=display, yet this boot still prints to
  ${console_tty}. Something outside that line wins — an extraargs= in ${ENV_TXT}, or bootargs
  baked into U-Boot. Find it in /proc/cmdline; editing console= again will not help."
            fi
        else
            printf '  %-22s %s\n' "kernel console" "${console_what}"
            warn "the kernel prints to ${console_tty} and ${ENV_TXT} does not say
  console=display, so this script left it alone — it only rewrites console=both and
  console=serial. Kernel messages on the motor UART corrupt servo replies intermittently,
  which is an unpatterned bus fault. Set console=display in ${ENV_TXT} and reboot."
        fi
    fi

    if [ -e "${ONNX_LIB_DIR}/libonnxruntime.so" ]; then
        # The version, not just "present": an incompatible runtime is indistinguishable from
        # a correct one until robotd tries to load a policy and its control thread dies.
        have="$(readlink -f "${ONNX_LIB_DIR}/libonnxruntime.so" 2>/dev/null || true)"
        have="${have##*/libonnxruntime.so.}"
        if [ "$have" = "$ONNX_VERSION" ]; then
            printf '  %-22s %s\n' "ONNX Runtime" "$have"
        else
            printf '  %-22s %s\n' "ONNX Runtime" "${have:-unknown} (expected ${ONNX_VERSION})"
            warn "this ONNX Runtime will not load a policy. Re-run this script to replace it."
        fi
    else
        printf '  %-22s %s\n' "ONNX Runtime" "ABSENT — robotd cannot load a policy"
    fi

    # Failed units, named.
    #
    # Here rather than in board-test.sh because that runs in a container with no systemd, so
    # this is the only place it can be asked. It exists because a unit failing at boot is
    # invisible until someone thinks to look: `systemd-networkd-wait-online` failed on every
    # boot of this board for a week, costing 20s each time and delaying updaterd behind
    # network-online.target, and nothing reported it.
    if command -v systemctl >/dev/null 2>&1 && [ -d /run/systemd/system ]; then
        failed="$(systemctl list-units --state=failed --no-legend --plain 2>/dev/null \
            | awk '{print $1}' | tr '\n' ' ')"
        if [ -n "$failed" ]; then
            printf '  %-22s %s\n' "failed units" "$failed"
            warn "these units failed this boot. Even one that looks unrelated delays boot and
  can hold up network-online.target:  systemctl status ${failed%% *}"
        else
            printf '  %-22s %s\n' "failed units" "none"
        fi
    fi

    if ! command -v nmcli >/dev/null 2>&1; then
        printf '  %-22s %s\n' "wifi" "NetworkManager ABSENT — still netplan"
    else
        wifi_state="$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | sed -n 's/^wlan0://p')"
        case "$wifi_state" in
            '')          printf '  %-22s %s\n' "wifi" "no wlan0" ;;
            unmanaged)   printf '  %-22s %s\n' "wifi" "NOT NetworkManager's — still netplan" ;;
            connected)   printf '  %-22s %s\n' "wifi" "NetworkManager, connected" ;;
            *)           printf '  %-22s %s\n' "wifi" "NetworkManager, ${wifi_state}" ;;
        esac
    fi

    if [ "$(systemctl is-enabled systemd-networkd-wait-online.service 2>/dev/null)" = masked ]; then
        printf '  %-22s %s\n' "networkd wait-online" "masked"
    elif command -v nmcli >/dev/null 2>&1; then
        printf '  %-22s %s\n' "networkd wait-online" "NOT masked — expect a boot stall"
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
            # Piped in, so there was no file to persist. Print the fetch rather than a comment
            # saying one is needed — /tmp is cleared by the reboot this is asking for, and the
            # operator has no shell history to recover the command from either.
            printf '  %s\n' "$(fetch_cmd setup-board.sh)"
            echo "  sudo sh /tmp/setup-board.sh"
        fi
        cat <<'EOF'

  Boot configuration changed. Nothing else can be confirmed until the overlay is live, and
  this script is idempotent — running it again after the reboot picks up where it stopped.
EOF
        return 0
    fi

    say "board ready — install the daemon next"
    echo
    printf '  %s\n' "$(fetch_cmd install.sh)"
    if [ -n "$TOKEN" ]; then
        # Literal, not expanded: this line gets pasted around, and the value must not.
        # shellcheck disable=SC2016
        printf '  %s\n' 'sudo DUCK_TOKEN="$DUCK_TOKEN" sh /tmp/install.sh'
        cat <<'EOF'

  Both halves need the token while the repository is private: raw.githubusercontent.com 404s
  without the header, and sudo does not pass the variable through on its own. Once the
  repository is public, drop the header and the prefix.
EOF
    else
        echo "  sudo sh /tmp/install.sh"
        cat <<'EOF'

  If that 404s rather than downloading, the repository is private and needs a token — a 404
  is what GitHub returns for a private path, so it looks like a wrong URL. Export DUCK_TOKEN
  and re-run this script: it reprints these two lines with the header and the sudo prefix.
  install.sh needs the token for the release assets as well, not only for the fetch.
EOF
    fi
}

main() {
    check_environment
    persist_self
    configure_overlay
    check_network
    free_motor_port
    configure_bluetooth
    install_onnxruntime
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a setup.
main "$@"
