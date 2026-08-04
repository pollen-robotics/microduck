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

BT_CONF=/etc/bluetooth/main.conf

# Optionally pair a gamepad, e.g. PAD_MAC=78:86:2E:BB:13:28 sh setup-board.sh
#
# An environment variable rather than a flag, matching MOTOR_PORT and ONNX_VERSION above —
# this script is usually run through `curl | sh`, where flags are awkward to pass.
#
# Optional because the MAC is per-pad and most of the value here is the two settings that
# apply to every board regardless: the BlueZ privacy mode, and the `input` group.
PAD_MAC="${PAD_MAC:-}"

# Where this script puts itself so it is still around after the reboot it asks for.
SELF=/usr/local/sbin/robot-setup-board

# Wifi migration lives in its own script — see `check_network` for why. Named here so the
# advice this prints and the thing it points at cannot drift apart.
MIGRATE=migrate-network.sh
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
    # Prefer the persisted copy when it exists: after the reboot the migration asks for, the
    # one in /tmp is gone.
    if [ -x "$MIGRATE_SELF" ]; then
        migrate_cmd="sudo ${MIGRATE_SELF}"
    else
        migrate_cmd="sudo sh ${MIGRATE}"
    fi

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

# Bluetooth settings a gamepad needs, and the group that lets a human read one.
#
# `Privacy = device` is the fix for an Xbox controller that pairs and then drops straight back
# out — it presents as an endless connect/disconnect loop, or as
# `disconnected with reason 3` / `AuthenticationCanceled` during pairing. Taken from
# microduck_runtime, whose installer sets exactly this and whose notes record the same
# symptom; several hours went into rediscovering it as a supposed BR/EDR or ERTM problem,
# which it is not. BLE is fine.
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

    if grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*device' "$BT_CONF"; then
        say "bluetooth Privacy already set to device"
    else
        say "setting Privacy = device in ${BT_CONF}"
        if grep -Eq '^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=' "$BT_CONF"; then
            sed -i -E 's|^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=.*|Privacy = device|' "$BT_CONF"
        elif grep -q '^\[General\]' "$BT_CONF"; then
            sed -i '/^\[General\]/a Privacy = device' "$BT_CONF"
        else
            printf '\n[General]\nPrivacy = device\n' >> "$BT_CONF"
        fi
        needs_reboot=1
    fi

    add_operator_to_input_group
    pair_pad
}

# A gamepad is read through /dev/input/event*, which is root:input mode 0660. Without this
# `padd` starts, reports nothing, and silently sees no pad at all — the same shape of failure
# as the `robot` group, and just as hard to guess from the outside.
add_operator_to_input_group() {
    operator="${SUDO_USER:-}"
    if [ -z "$operator" ] || [ "$operator" = root ]; then
        return 0
    fi
    if id -nG "$operator" 2>/dev/null | tr ' ' '\n' | grep -qx input; then
        return 0
    fi
    if usermod -aG input "$operator"; then
        say "added ${operator} to the input group"
        warn "${operator} must log out and back in before padd can read a gamepad."
    else
        warn "could not add ${operator} to the input group; padd will see no gamepad"
    fi
}

# Trust and connect a known pad, when its MAC was supplied.
#
# Deliberately not a scan: discovery needs the pad held in pairing mode at the right moment,
# which a provisioning script cannot arrange. Supplying the MAC of a pad already in pairing
# mode is the part that can be automated; the rest stays a human at a keyboard.
pair_pad() {
    [ -n "$PAD_MAC" ] || return 0

    if ! command -v bluetoothctl >/dev/null 2>&1; then
        warn "bluetoothctl is not installed; cannot pair ${PAD_MAC}"
        return 0
    fi

    say "pairing gamepad ${PAD_MAC} (hold it in pairing mode)"
    # Discovery has to be running for a first-time connect to resolve the address.
    bluetoothctl --timeout 15 scan on >/dev/null 2>&1 || true

    # `connect` before `pair`, which is the order microduck_runtime's notes give and the one
    # that works; leading with `pair` returns AuthenticationCanceled.
    if bluetoothctl connect "$PAD_MAC" >/dev/null 2>&1; then
        bluetoothctl trust "$PAD_MAC" >/dev/null 2>&1 || true
        say "gamepad ${PAD_MAC} connected and trusted"
    else
        warn "could not connect ${PAD_MAC}. If Privacy was just changed, reboot first — it
  does not take effect until then. Otherwise pair by hand:
    sudo bluetoothctl
    scan on            (hold the pad in pairing mode until it is listed by: devices)
    connect ${PAD_MAC}
    trust ${PAD_MAC}"
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

    # Gamepad readiness, which is three separate things that each fail silently.
    if [ -f "$BT_CONF" ] && grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*device' "$BT_CONF"; then
        printf '  %-22s %s\n' "bluetooth privacy" "device"
    else
        printf '  %-22s %s\n' "bluetooth privacy" "NOT SET — a pad will drop on connect"
    fi

    operator="${SUDO_USER:-}"
    if [ -z "$operator" ] || [ "$operator" = root ]; then
        printf '  %-22s %s\n' "input group" "not applicable (no sudo user)"
    elif id -nG "$operator" 2>/dev/null | tr ' ' '\n' | grep -qx input; then
        printf '  %-22s %s\n' "input group" "${operator} is a member"
    else
        printf '  %-22s %s\n' "input group" "${operator} NOT a member — padd sees no pad"
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

    if grep -qE '(^| )console=tty(S|AMA)' /proc/cmdline 2>/dev/null; then
        printf '  %-22s %s\n' "kernel console" "still on a serial port"
        warn "the kernel prints to a UART (see /proc/cmdline). If that is ${MOTOR_PORT},
  kernel messages will corrupt servo traffic intermittently. Set console=display in
  ${ENV_TXT} and reboot."
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
    check_network
    free_motor_port
    configure_bluetooth
    install_onnxruntime
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a setup.
main "$@"
