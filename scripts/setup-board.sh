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

# BlueZ's input profile settings. One line in it decides who carries a classic (BR/EDR) gamepad's
# reports — see `configure_classic_hid`. Every board gets that change; no flag.
BT_INPUT_CONF=/etc/bluetooth/input.conf

# Does this board need `Privacy = device`? `--weird-ble` on provision-board.sh.
#
# Off by default: some Zero 3W units bond a pad under BlueZ's own default and want nothing from
# this script. See `configure_bluetooth`.
WEIRD_BLE="${DUCK_WEIRD_BLE:-}"

# Does `robotctl pad pair` have to pause `btd` on this board? `--pause-btd-on-pair`.
#
# Separate from `WEIRD_BLE` because they fix different faults, and a board can need this one
# without wanting `Privacy = device` — measured, see `configure_bluetooth`. `--weird-ble` implies
# it, so every board provisioned before this flag existed keeps behaving the same way.
PAUSE_BTD="${DUCK_PAUSE_BTD:-}"

# Where the answer is left for `robotctl`, which has to pause `btd` while a pad bonds on such a
# board. Under /var/lib rather than in a release directory: it is a fact about this board and must
# survive an update and a rollback.
#
# The name is now narrower than what it means — it marks "pause btd for a pairing", which is no
# longer tied to `Privacy = device`. Kept as-is deliberately: `robotctl` reads this exact path, and
# renaming it would make every board already carrying one stop pausing `btd` with nothing to say
# why. Rename both together, or neither.
WEIRD_BLE_MARKER=/var/lib/robot/weird-ble

# A gamepad is paired with `sudo robotctl pad pair` on the installed release, with the pad held in
# pairing mode. This script's part of it is the one BlueZ setting a pad needs, which is here because
# it takes a reboot to apply — see `configure_bluetooth`.

# Where this script puts itself so it is still around after the reboot it asks for.
SELF=/usr/local/sbin/robot-setup-board

# Where the sibling scripts come from, for the commands this prints. Same override names as
# `install.sh`, so a fork or a pinned tag is one decision for the whole bring-up rather than
# per script. Nothing here is fetched by this script — see `fetch_cmd`.
REPO="${DUCK_REPO:-pollen-robotics/microduck}"
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

# The one Bluetooth setting a gamepad needs from this script, on the boards that need it.
#
# **Only with `DUCK_WEIRD_BLE=1`** — `--weird-ble` on `provision-board.sh`. Without it this touches
# nothing, because most boards want nothing touched.
#
# The split. On a fresh Armbian with nothing installed, some Radxa Zero 3W units bond an Xbox pad
# under BlueZ`s default `Privacy = off`. Others do not bond at all under `off`, and only
# `Privacy = device` works. Roughly half of ten units in each group, and nothing measurable separates
# them: same kernel version, same BlueZ, byte-identical aic8800 firmware, and the driver build does
# not track it either — a pad bonds fine on the build that was once blamed.
#
# Why it is a flag rather than the default. `device` is not free: under it a pad cannot form a *new*
# bond while `btd` advertises, so `robotctl pad pair` has to stop `btd` for the pairing window on any
# board that has it. Setting `device` everywhere would impose that on boards that never needed it.
# `robotctl` needs to know too, since it is what pauses `btd`. It reads a marker this writes rather
# than re-deriving the answer from `main.conf`: an explicit record of the decision someone made
# cannot be confused with a `Privacy` value that arrived some other way.
#
# It also accounts for the observation that once made this script set `Privacy = off` outright:
# pairing reaching the last SMP step and the pad answering `DHKey check failed (0x0b)`. That was
# `device` with `btd` running — the interaction above, not evidence against the setting.
#
# ## Two faults, two flags
#
# `--weird-ble` used to do both halves of the workaround at once, and on `50:37:CD:16:1D:90` that
# combination cannot be made to work. Measured there on 2026-08-19, one variable at a time, daemon
# 0.6.0 throughout:
#
#   | Privacy | btd paused for the pairing | result                                          |
#   |---------|----------------------------|-------------------------------------------------|
#   | off     | no                         | pairing dies ~800us after Encryption Change     |
#   |         |                            | with Remote User Terminated (0x13)              |
#   | off     | YES                        | bonds, held 45/45 samples, real input, driving   |
#   | device  | yes                        | bonds, then flaps: 46x PIN or Key Missing (0x06) |
#
# So the two halves fix different faults and are not a package:
#
#   - **`btd` advertising breaks a NEW bond.** Pausing it for the pairing window fixes that, and it
#     is what makes the aic8800 driver build irrelevant — every earlier failure blamed on the driver
#     had `btd` advertising, which was the uncontrolled variable.
#   - **`Privacy = device` breaks RECONNECTION** on a board that does not need it. A clean bond then
#     flaps with `PIN or Key Missing`, on either driver build, whether or not the bond was made under
#     `device`. On such a board the flag is not merely unnecessary, it is harmful — and it fails in a
#     worse way than not pairing at all, because it looks like it worked.
#
# Hence `--pause-btd-on-pair` for the first alone. `--weird-ble` still implies it, so nothing that
# was provisioned before this behaves differently. A board that pairs with the pause and `off` wants
# the new flag; a board that cannot bond under `off` at all still wants `--weird-ble`.
#
# Both are workarounds for the aic8800 radio, which is not what ships. When the radio changes, both
# flags and `BtdPaused` in robotctl/src/main.rs all go.
#
# The change sets `needs_reboot` rather than restarting bluetooth. Restarting the daemon here leaves
# the kernel holding hci0 while bluetoothd reports "No default controller available", which needs a
# reboot to clear.
# ── audio: the TLV320AIC3104 codec (speaker + microphone) ─────────────────────────────────
#
# The robot's voice and ear. Ported from the prototype installer's audio bring-up, and like
# everything it did, every step here fails *soft* — a board without working audio walks
# identically, so nothing below is allowed to stop the provisioning.
#
# Five layers, each idempotent:
#   1. alsa-utils (aplay/arecord/amixer) + the DKMS toolchain + dtc.
#   2. The Armbian *vendor* (BSP 6.1) kernel: the codec's I²S clock tree only exists there,
#      and the DKMS module builds against its headers.
#   3. Device-tree overlays, compiled from the sources vendored in deploy/audio/: the
#      hardware i2c3 bus on header pins 3/5, and the codec + I²S sound card grafted onto it.
#      (The prototype also kept a bit-banged i2c-gpio fallback; it was the revert path for
#      rise-time trouble that never came back, and it is not carried here.)
#   4. The codec driver itself, out of tree via DKMS — the vendor kernel does not build
#      SND_SOC_AIC3X, which is why a stock board has no aic3104 card.
#   5. Mixer levels at boot: aic3104-init.service, running the vendored amixer script
#      before robotd so the greet is audible.
#
# The voice bank itself is NOT provisioned here — the release's postinstall renders it with
# the `sounds` binary the release carries, seeded from the SoC serial.

# Fetch a repository file (deploy/audio/...) into $2. The token, when given, rides along —
# same reasoning as fetch_cmd, done rather than printed.
fetch_repo_file() {
    if [ -n "$TOKEN" ]; then
        curl -fsSL -H "Authorization: Bearer $TOKEN" "${RAW%/scripts}/$1" -o "$2"
    else
        curl -fsSL "${RAW%/scripts}/$1" -o "$2"
    fi
}

# Add one word to armbianEnv's overlays= line, preserving order (the codec overlay must
# come after its bus overlay — it grafts the codec node onto the bus that overlay enables).
ensure_overlay_word() {
    # Same guard `configure_overlay` has, for the same reason — and here it matters more:
    # without it the `echo >>` below *creates* an armbianEnv.txt that never existed, on a
    # board that boots from something else entirely, and asks for a reboot to load it.
    if [ ! -f "$ENV_TXT" ]; then
        warn "no ${ENV_TXT}; not an Armbian image?
  Load the ${1} overlay by whatever means this image provides, then re-run.
  Everything else here will still be done."
        return 0
    fi
    if ! grep -Eq '^overlays=' "$ENV_TXT"; then
        echo "overlays=$1" >> "$ENV_TXT"
        needs_reboot=1
    elif ! grep -E '^overlays=' "$ENV_TXT" | grep -qw "$1"; then
        sed -i "s/^overlays=\(.*\)\$/overlays=\1 $1/" "$ENV_TXT"
        needs_reboot=1
    fi
}

configure_audio() {
    say "audio: TLV320AIC3104 codec bring-up"

    # 1. Packages. i2c-tools is here rather than with the ToF below because it is
    #    what *creates the `i2c` group* (its postinst does), and both the codec and
    #    the ToF sit on that bus — plus `i2cdetect` is the first thing anyone runs
    #    when a device on it goes quiet.
    audio_pkgs="alsa-utils device-tree-compiler dkms gcc make i2c-tools"
    missing=""
    for pkg in $audio_pkgs; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing="$missing $pkg"
    done
    if [ -n "$missing" ]; then
        say "installing:$missing"
        apt-get update -qq || true
        # shellcheck disable=SC2086  # word-splitting the package list is the point
        apt-get install -y -qq $missing \
            || { warn "apt failed — audio will not work on this board"; return 0; }
    fi

    # 2. The vendor kernel, with headers for the DKMS build.
    if ! dpkg -s linux-image-vendor-rk35xx >/dev/null 2>&1 \
        || ! dpkg -s linux-dtb-vendor-rk35xx >/dev/null 2>&1 \
        || ! dpkg -s linux-headers-vendor-rk35xx >/dev/null 2>&1; then
        say "installing the Armbian vendor kernel (the codec's I²S tree lives there)"
        apt-get update -qq || true
        if apt-get install -y linux-image-vendor-rk35xx linux-dtb-vendor-rk35xx \
            linux-headers-vendor-rk35xx; then
            needs_reboot=1
        else
            warn "could not install the vendor kernel — audio will not work"
            return 0
        fi
    fi
    vendor_ver=$(find /lib/modules -maxdepth 1 -name '*-vendor-rk35xx' 2>/dev/null | sort -V | tail -1 | xargs -r basename)
    if [ -z "$vendor_ver" ]; then
        warn "no vendor kernel under /lib/modules — audio will not work"
        return 0
    fi
    # Make it the active kernel. The apt postinst repoints these on a fresh install;
    # re-asserted here for idempotent re-runs where the 'current' kernel touched them last.
    for pair in "Image:vmlinuz-$vendor_ver" "uInitrd:uInitrd-$vendor_ver" "dtb:dtb-$vendor_ver"; do
        link="${pair%%:*}"; target="${pair#*:}"
        if [ -e "/boot/$target" ] && [ "$(readlink "/boot/$link")" != "$target" ]; then
            say "pointing /boot/$link -> $target"
            ln -sfn "$target" "/boot/$link"
            needs_reboot=1
        fi
    done
    depmod "$vendor_ver" 2>/dev/null || true

    # 3. Overlays: compile the vendored sources against the vendor kernel's overlay dir.
    dtbo_dir="/boot/dtb-${vendor_ver}/rockchip/overlay"
    [ -d "$dtbo_dir" ] || dtbo_dir=$(find /boot -maxdepth 3 -type d -path '*/rockchip/overlay' 2>/dev/null | head -1)
    if [ -n "$dtbo_dir" ]; then
        for ov_name in i2c3-pihat aic3104-i2c3; do
            ov_tmp=$(mktemp -d)
            if fetch_repo_file "deploy/audio/${ov_name}.dts" "$ov_tmp/src.dts" \
                && dtc -@ -I dts -O dtb -o "$ov_tmp/out.dtbo" "$ov_tmp/src.dts" 2>/dev/null; then
                if [ ! -f "$dtbo_dir/rk3568-${ov_name}.dtbo" ] \
                    || ! cmp -s "$ov_tmp/out.dtbo" "$dtbo_dir/rk3568-${ov_name}.dtbo"; then
                    say "installing overlay rk3568-${ov_name}.dtbo"
                    cp "$ov_tmp/out.dtbo" "$dtbo_dir/rk3568-${ov_name}.dtbo"
                    needs_reboot=1
                fi
                ensure_overlay_word "$ov_name"
            else
                warn "could not fetch or compile ${ov_name}.dts — audio will not work"
            fi
            rm -rf "$ov_tmp"
        done
    else
        warn "no rockchip overlay directory under /boot — audio overlays not installed"
    fi

    # 4. The codec driver, via DKMS. Versioned by the vendored dkms.conf, so a source bump
    #    upstream re-deploys cleanly.
    dkms_tmp=$(mktemp -d)
    dkms_ok=1
    for f in dkms.conf Makefile tlv320aic3x.c tlv320aic3x.h tlv320aic3x-i2c.c; do
        fetch_repo_file "deploy/audio/aic3x-dkms/$f" "$dkms_tmp/$f" || { dkms_ok=0; break; }
    done
    if [ "$dkms_ok" = 1 ]; then
        dkms_ver=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"$/\1/p' "$dkms_tmp/dkms.conf")
        dkms_src="/usr/src/aic3x-$dkms_ver"
        deploy_needed=0
        for f in dkms.conf Makefile tlv320aic3x.c tlv320aic3x.h tlv320aic3x-i2c.c; do
            cmp -s "$dkms_tmp/$f" "$dkms_src/$f" || deploy_needed=1
        done
        if [ "$deploy_needed" = 1 ]; then
            say "deploying aic3x DKMS sources to $dkms_src"
            dkms remove "aic3x/$dkms_ver" --all >/dev/null 2>&1 || true
            mkdir -p "$dkms_src"
            cp "$dkms_tmp"/* "$dkms_src/"
        fi
        if dkms status "aic3x/$dkms_ver" 2>/dev/null | grep "$vendor_ver" | grep -q installed; then
            say "aic3x DKMS module already installed"
        else
            # Armbian ships the vendor headers without built host tools; DKMS needs modpost.
            if [ -d "/usr/src/linux-headers-$vendor_ver" ] \
                && [ ! -x "/usr/src/linux-headers-$vendor_ver/scripts/mod/modpost" ]; then
                say "rebuilding the vendor headers' host tools (modpost)"
                dpkg-reconfigure linux-headers-vendor-rk35xx >/dev/null 2>&1 || true
            fi
            say "building the aic3x codec driver via DKMS (takes a minute)"
            if dkms install "aic3x/$dkms_ver" -k "$vendor_ver"; then
                say "aic3x DKMS module installed for $vendor_ver"
                needs_reboot=1
            else
                warn "DKMS build failed — audio will not work"
                warn "see /var/lib/dkms/aic3x/$dkms_ver/build/make.log"
            fi
        fi
    else
        warn "could not fetch the aic3x DKMS sources — audio will not work"
    fi
    rm -rf "$dkms_tmp"

    # 5. Mixer levels at boot. The service polls for the card (the probe is deferred until
    #    the DKMS module autoloads), sets the speaker path and routes the onboard mic.
    init_tmp=$(mktemp)
    if fetch_repo_file "deploy/audio/aic3104-init.sh" "$init_tmp"; then
        if [ ! -f /usr/local/bin/aic3104-init.sh ] \
            || ! cmp -s "$init_tmp" /usr/local/bin/aic3104-init.sh; then
            say "installing /usr/local/bin/aic3104-init.sh"
            install -m 755 "$init_tmp" /usr/local/bin/aic3104-init.sh
        fi
        svc_tmp=$(mktemp)
        cat > "$svc_tmp" <<'UNIT'
[Unit]
Description=TLV320AIC3104 mixer init
After=systemd-modules-load.service
# No ConditionPathExists: the sound card probe is deferred until the DKMS codec module
# autoloads, so the card can appear seconds into boot — the script polls for it instead.
Before=robotd.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/aic3104-init.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
        if [ ! -f /etc/systemd/system/aic3104-init.service ] \
            || ! cmp -s "$svc_tmp" /etc/systemd/system/aic3104-init.service; then
            say "installing aic3104-init.service"
            install -m 644 "$svc_tmp" /etc/systemd/system/aic3104-init.service
            systemctl daemon-reload
        fi
        systemctl is-enabled --quiet aic3104-init.service \
            || systemctl enable aic3104-init.service >/dev/null 2>&1 || true
        rm -f "$svc_tmp"
    else
        warn "could not fetch aic3104-init.sh — the mixer stays at power-on levels"
    fi
    rm -f "$init_tmp"
}

# ── the head ToF sensor's bus ──────────────────────────────────────────────────
#
# The sensor is a VL53L5CX or VL53L8CX on the *same* i2c3 bus the audio codec is
# on, so `configure_audio` has already done the expensive part: the overlay, the
# vendor kernel, and i2c-tools (which is what creates the `i2c` group `tofd`
# joins). This adds the one thing left — a stable name for the bus.
#
# `/dev/i2c-3` is what the overlay happens to produce today, and a kernel or
# overlay change can renumber it. A udev symlink keeps `tofd`'s default correct
# across that. `tofd` also falls back to `/dev/i2c-3` on a board provisioned
# before this rule existed, so neither half is load-bearing on its own.
configure_tof() {
    rule=/etc/udev/rules.d/99-robot-i2c-pihat.rules
    # Two matchers, one per bus flavour the board may end up with: the RK3566's
    # i2c3 controller by its device-tree address, and the bit-banged i2c-gpio bus
    # by name. Only one exists at a time, so the symlink follows whichever it is.
    content='SUBSYSTEM=="i2c-dev", KERNELS=="fe5c0000.i2c", SYMLINK+="i2c-pihat"
SUBSYSTEM=="i2c-dev", ATTR{name}=="i2c-gpio-pihat", SYMLINK+="i2c-pihat"'

    if [ -f "$rule" ] && [ "$(cat "$rule")" = "$content" ]; then
        say "ToF: /dev/i2c-pihat rule already in place"
        return 0
    fi
    say "ToF: installing the /dev/i2c-pihat udev rule"
    printf '%s\n' "$content" > "$rule"
    chmod 644 "$rule"
    # Applied now as well as at the next boot, so a sensor already fitted works
    # without one — `udevadm` failing is not worth stopping provisioning for.
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger --subsystem-match=i2c-dev 2>/dev/null || true
}

# ── the head camera's device-tree overlay ──────────────────────────────────────
#
# Without it the sensor is never probed: no /dev/videoN, and nothing in dmesg either, which
# reads exactly like a camera that is not plugged in.
#
# **The overlay has to be mirrored under a prefixed name first, and that is the whole trick.**
# Armbian ships it as `radxa-zero3-rpi-camera-v2.dtbo` with no `rk3568-` prefix, while this
# board runs `overlay_prefix=rk3568` — so the loader resolves the `overlays=` word to
# `rk3568-radxa-zero3-rpi-camera-v2.dtbo`, finds nothing, and boots happily with no camera.
# Same silent class of failure as the wrong prefix in `configure_overlay`. The prototype hit
# this and mirrors the file (`microduck_runtime/install.sh`); so does this.
#
# Copying rather than symlinking, matching the prototype: an Armbian package update replaces the
# unprefixed file, and a stale mirror is at least a *working* camera rather than a dangling link.
# Re-running refreshes it when the source changes.
#
# `DUCK_CAMERA_OVERLAY` selects another module — `radxa-zero3-rpi-camera-v1.3` is the Pi Cam
# v1.3 / OV5647, which nothing here has tested.
configure_camera() {
    dtbo_dir="/boot/dtb-${1}/rockchip/overlay"
    [ -d "$dtbo_dir" ] || dtbo_dir=$(find /boot -maxdepth 3 -type d -path '*/rockchip/overlay' 2>/dev/null | head -1)
    if [ -z "$dtbo_dir" ]; then
        warn "no rockchip overlay directory under /boot; camera overlay not installed"
        return 0
    fi

    cam="${DUCK_CAMERA_OVERLAY:-radxa-zero3-rpi-camera-v2}"
    src="${dtbo_dir}/${cam}.dtbo"
    dst="${dtbo_dir}/rk3568-${cam}.dtbo"

    if [ ! -f "$src" ]; then
        warn "no ${src}; the head camera will not be probed.
  Armbian ships one overlay per module — list them with
      ls ${dtbo_dir} | grep -i cam
  and set DUCK_CAMERA_OVERLAY to the right one. Everything else here is still done."
        return 0
    fi

    if [ ! -f "$dst" ] || ! cmp -s "$src" "$dst"; then
        say "camera: mirroring ${cam}.dtbo under the rk3568- prefix"
        cp "$src" "$dst"
        needs_reboot=1
    fi
    # Only after the mirror exists — a word in `overlays=` naming a dtbo that is not on disk is
    # skipped at boot with nothing said about it.
    ensure_overlay_word "$cam"
}

configure_bluetooth() {
    if [ -z "$WEIRD_BLE" ] && [ -z "$PAUSE_BTD" ]; then
        # Reported rather than silent, because a board that needs either workaround and was
        # provisioned without the flag presents as a pad that will not pair for no visible reason.
        say "leaving bluetooth alone (no --weird-ble, no --pause-btd-on-pair)"
        say "  a pad that pairs and then flaps wants neither; one that will not pair wants one"
        # The marker is deliberately *not* removed here. Without a flag this function changes
        # nothing, so a board that has `Privacy = device` still has it — and clearing the marker
        # would leave `robotctl` no longer pausing `btd` on a board that still needs it, which is
        # the silent version of the bug these flags exist for. Undoing it is a hand edit.
        return 0
    fi

    if [ ! -f "$BT_CONF" ]; then
        warn "no ${BT_CONF}; skipping the gamepad Bluetooth settings"
        return 0
    fi

    # The marker alone, for a board that pairs under `off` once `btd` is out of the way. Returns
    # before touching `Privacy`, which is the whole point of the flag: on such a board `device` is
    # what breaks it.
    if [ -z "$WEIRD_BLE" ]; then
        write_pause_marker
        say "left Privacy alone (--pause-btd-on-pair without --weird-ble)"
        return 0
    fi

    if grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*device' "$BT_CONF"; then
        say "bluetooth Privacy already device"
    else
        say "setting Privacy = device in ${BT_CONF} (--weird-ble)"
        if grep -Eq '^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=' "$BT_CONF"; then
            sed -i -E 's|^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=.*|Privacy = device|' "$BT_CONF"
        elif grep -q '^\[General\]' "$BT_CONF"; then
            sed -i '/^\[General\]/a Privacy = device' "$BT_CONF"
        else
            printf '\n[General]\nPrivacy = device\n' >> "$BT_CONF"
        fi
        needs_reboot=1
    fi

    # Written after the setting, so the marker never claims a board is configured that is not.
    write_pause_marker
}

# A classic (BR/EDR) gamepad's reports go through the kernel, not through bluetoothd.
#
# BlueZ 5.82 defaults `UserspaceHID=true`: bluetoothd reads every HID report from the L2CAP socket
# and re-injects it into the kernel through uhid. For an LE pad (Xbox) this setting is irrelevant —
# HID over GATT is bluetoothd's own profile either way, and the Xbox sends nothing while idle. For a
# classic pad it is the whole data path, and the no-name "Pro Controller" Switch clones stream IMU
# samples in every packet at ~200 packets/s whether or not anyone touches them. Measured on graphite,
# 2026-09-09, pad connected and untouched:
#
#   UserspaceHID=true   bluetoothd 15.8 % CPU, ~594 IMU reports/s on /dev/input/event5, nodes under
#                       /sys/devices/virtual/misc/uhid/
#   UserspaceHID=false  bluetoothd  0.0 %, same reports, nodes under .../hci0/hci0:128/ — the
#                       kernel's hidp owns the channel and hid-nintendo binds exactly as before
#
# The bond, the reconnect after a reboot and `padd` driving from the pad were all unchanged by the
# switch; the pad reconnected by itself on the first boot with the new value. No flag, because there
# is no board on which the userspace path is the better one: it costs a core's worth of context
# switches and buys nothing this robot uses.
#
# Applies at the next bluetoothd start, which on this board means a reboot — see the note on
# `needs_reboot` above `configure_bluetooth`.
configure_classic_hid() {
    if [ ! -f "$BT_INPUT_CONF" ]; then
        say "no ${BT_INPUT_CONF}; writing one with UserspaceHID=false"
        printf '[General]\nUserspaceHID=false\n' > "$BT_INPUT_CONF"
        needs_reboot=1
        return 0
    fi
    if grep -Eq '^[[:space:]]*UserspaceHID[[:space:]]*=[[:space:]]*false' "$BT_INPUT_CONF"; then
        say "bluetooth UserspaceHID already false (classic pads go through the kernel)"
        return 0
    fi
    say "setting UserspaceHID=false in ${BT_INPUT_CONF} (classic pads through the kernel, not bluetoothd)"
    if grep -Eq '^[[:space:]]*#?[[:space:]]*UserspaceHID[[:space:]]*=' "$BT_INPUT_CONF"; then
        sed -i -E 's|^[[:space:]]*#?[[:space:]]*UserspaceHID[[:space:]]*=.*|UserspaceHID=false|' "$BT_INPUT_CONF"
    elif grep -q '^\[General\]' "$BT_INPUT_CONF"; then
        sed -i '/^\[General\]/a UserspaceHID=false' "$BT_INPUT_CONF"
    else
        printf '\n[General]\nUserspaceHID=false\n' >> "$BT_INPUT_CONF"
    fi
    needs_reboot=1
}

# The marker `robotctl` reads to decide whether to pause `btd` for a pairing.
#
# Its own function because both flags write it and only one of them touches `Privacy` — which is the
# distinction this whole section exists to make.
write_pause_marker() {
    if [ -f "$WEIRD_BLE_MARKER" ]; then
        say "btd-pause marker already at ${WEIRD_BLE_MARKER}"
        return 0
    fi
    mkdir -p "$(dirname "$WEIRD_BLE_MARKER")"
    cat > "$WEIRD_BLE_MARKER" <<'MARKER'
# This board needs btd paused while a gamepad bonds.
#
# On the aic8800 radio a pad cannot form a NEW bond while btd advertises, so `robotctl pad pair`
# stops btd for the pairing window, power-cycles the adapter, and starts btd again afterwards. An
# existing bond is unaffected: a bonded pad connects and drives with the whole stack up.
#
# This says nothing about Privacy. A board provisioned with --pause-btd-on-pair keeps BlueZ's
# default (off); one provisioned with --weird-ble also has Privacy = device because it cannot bond
# under off at all. Setting device on a board that did not need it makes a bond flap with
# `PIN or Key Missing` instead, so the two are deliberately separate.
#
# A workaround for a radio that is not what ships. Delete this file and drop BtdPaused from
# robotctl when the radio changes.
MARKER
    chmod 644 "$WEIRD_BLE_MARKER"
    say "wrote ${WEIRD_BLE_MARKER} so robotctl pauses btd while a pad bonds"
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

    # Gamepad readiness. This board's part of it is two independent settings; who may read the pad is
    # `padd.service`'s business now, and pairing one is `sudo robotctl pad pair`.
    #
    # Both are reported, because the pair of them is what says which of the three configurations a
    # board is in — and the advice for a pad that will not bond depends on which one is missing.
    if [ -f "$WEIRD_BLE_MARKER" ]; then
        printf '  %-22s %s\n' "btd on pairing" "paused — the marker is set"
    else
        printf '  %-22s %s\n' "btd on pairing" \
            "not paused — a pad that will not bond wants --pause-btd-on-pair"
    fi

    if [ -f "$BT_CONF" ] && grep -Eq '^[[:space:]]*Privacy[[:space:]]*=[[:space:]]*device' "$BT_CONF"; then
        # Worth naming the consequence every time: `device` is what makes a bond flap on a board
        # that only needed the pause, and that failure looks like success at first.
        printf '  %-22s %s\n' "bluetooth privacy" \
            "device — drop --weird-ble if a bond flaps with PIN or Key Missing"
    else
        # Covers both `off` and absent, which behave the same. Named as a possible cause rather than
        # a fault: with btd paused, most boards bond a pad exactly like this.
        printf '  %-22s %s\n' "bluetooth privacy" \
            "off — if a pad will not bond even with btd paused, try --weird-ble"
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
    configure_classic_hid
    configure_audio
    configure_tof
    # After configure_audio, which is what installs the vendor kernel: the camera's MIPI-CSI
    # capture driver lives only on that branch, and its overlay directory is the one to mirror
    # into. `vendor_ver` is what configure_audio resolved it to, or empty.
    configure_camera "${vendor_ver:-}"
    install_onnxruntime
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a setup.
main "$@"
