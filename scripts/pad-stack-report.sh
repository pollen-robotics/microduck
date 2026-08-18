#!/bin/sh
# What stack is this board driving a pad with? One report, logged, and diffable against another
# board's.
#
# The question this answers is not "does the pad work" — `pad-link-test.sh` measures that. It is
# "are these two boards the same". Two robots built weeks apart run different kernels, different
# BlueZ, different controller firmware and pads on different pad firmware, and every one of those
# changes how the pad behaves without changing anything anyone typed. A pad that stalls on one
# board and not on its twin is that comparison, and until now making it meant eight commands run
# twice and read side by side.
#
# WHY A FINGERPRINT AS WELL AS A REPORT. The full report is for reading: it carries the evidence,
# including the volatile parts — who is connected, what the battery says, when it ran. None of that
# survives a `diff`, and a diff full of timestamps is a diff nobody reads. So `--fingerprint` prints
# only the values that *must* match between two boards running the same stack, in a fixed order,
# with no timestamps and no addresses:
#
#   diff <(ssh a sudo sh /tmp/pad-stack-report.sh --fingerprint) \
#        <(ssh b sudo sh /tmp/pad-stack-report.sh --fingerprint)
#
# Both modes are generated from the same collection pass, so they cannot disagree.
#
# WHAT IT NEEDS. Nothing, and it never fails for want of a tool: a missing `hcitool` or an
# unreadable bond directory prints as a line saying so, because a diff between "absent" and a value
# is exactly the finding this exists to surface. Root gets three more things — the adapter's HCI
# version, the bond's key type, and the pad's firmware fields when BlueZ will not say — so it is
# worth `sudo`.
#
# Usage:  sudo sh pad-stack-report.sh [--mac AA:BB:..] [--out FILE]
#         sudo sh pad-stack-report.sh --fingerprint
#
# Exit: 0 the report was written; 1 it could not run at all.
set -eu

MAC=
OUT=
FINGERPRINT=

# Mirrors `pad-link-test.sh`, and for the same reason: the parse below can then be exercised against
# a captured file on a laptop that has no pad and no /proc worth the name.
INPUT_DEVICES="${PAD_INPUT_DEVICES:-/proc/bus/input/devices}"

BT_CONF="${PAD_BT_CONF:-/etc/bluetooth/main.conf}"
BT_LIB="${PAD_BT_LIB:-/var/lib/bluetooth}"
SYS_BT="${PAD_SYS_BT:-/sys/class/bluetooth}"
OS_RELEASE="${PAD_OS_RELEASE:-/etc/os-release}"
MODULES="${PAD_PROC_MODULES:-/proc/modules}"

# Bluetooth, in /proc/bus/input/devices terms; USB is 0003. A pad on a cable is worth naming rather
# than reporting as "no pad": there is no radio stack under it to compare.
BUS_BLUETOOTH=0005
BUS_USB=0003

# The modules that decide *how* a pad reaches userspace, which is the part of the stack that changes
# behaviour without changing any version number. A fixed list, absences included, so two reports
# line up: `hidp` present means the kernel is carrying classic HID, `uhid` means BlueZ's own HoG
# plugin is carrying LE HID, and `joydev` is what puts the /dev/input/js node there for gilrs.
WATCHED_MODULES="bluetooth hci_uart btusb btrtl btbcm hidp uhid joydev"

while [ $# -gt 0 ]; do
    case "$1" in
        --mac) shift; MAC="${1:?--mac needs a value}" ;;
        --out) shift; OUT="${1:?--out needs a value}" ;;
        --fingerprint) FINGERPRINT=1 ;;
        -h|--help)
            cat <<'USAGE'
usage: pad-stack-report.sh [--mac AA:BB:CC:DD:EE:FF] [--out FILE]
       pad-stack-report.sh --fingerprint

Report the whole gamepad stack this board is running: kernel, BlueZ, adapter and controller
firmware, the bond, the transport the pad is actually connected over, and the pad's own firmware
revision. Written to a log file as well as the terminal.

  --mac ADDR      which pad, when more than one is bonded (default: connected first)
  --out FILE      where to write the log (default: /tmp/pad-stack-<host>-<when>.log)
  --fingerprint   print only the values that must match between two boards, for diff(1)

Run it on the robot, under sudo — without root the adapter version, the bond's key type and some
of the pad's firmware fields are unreadable, and print as such.
USAGE
            exit 0
            ;;
        *) echo "pad-stack-report.sh: unknown argument: $1" >&2; exit 1 ;;
    esac
    shift
done

die() {
    echo "pad-stack-report.sh: $*" >&2
    exit 1
}

# One value per line, aligned. Width fits `address type`, which is the longest label here; a label
# that outgrows it costs alignment, not correctness.
#
# Trimmed here rather than at each call site, which is where it was and where the first real run
# caught it out: `btmgmt` ends its settings line with a space, and a trailing space is invisible in
# a diff of two fingerprints — the one thing this must never be.
field() {
    printf '  %-14s %s\n' "$1" "$(trim "$2")"
}

# A captured multi-line output, indented under its section. Blank input prints nothing at all rather
# than an empty indent, because a blank line in the middle of a report reads as a bug in the report.
dump() {
    [ -n "$1" ] || return 0
    printf '%s\n' "$1" | sed 's/^/    /'
}

# Trailing whitespace is invisible in a diff and is exactly what two of these get compared with, so
# every value taken out of another tool's output loses it here.
trim() {
    printf '%s' "$1" | sed -E 's/[[:space:]]+$//'
}

have() {
    command -v "$1" >/dev/null 2>&1
}

# Every `bluetoothctl` call goes through this. Two reasons: it is run on boards where bluetoothd is
# the thing that is broken, and bluetoothctl waits for a daemon that is never going to answer; and
# `set -eu` would otherwise exit the whole report on its non-zero status. A diagnostic that hangs is
# worse than one that reports `absent`.
bt() {
    if ! have bluetoothctl; then
        return 1
    fi
    if have timeout; then
        timeout 5 bluetoothctl "$@" 2>/dev/null || true
    else
        bluetoothctl "$@" 2>/dev/null || true
    fi
}

# ---------------------------------------------------------------------------- collection

# The values `--fingerprint` prints. Set by the section functions below, which run in the current
# shell — redirecting a function's output does not fork one, so these survive. Nothing here may be
# assigned inside a pipeline, which would.
f_kernel=unknown
f_os=unknown
f_bluez=absent
f_privacy="unset"
f_modules=
f_hci=unreadable
f_settings=unreadable
f_ctrl_fw=unreadable
f_chip=unreadable
f_pad_id=none
f_pad_transport=none
f_pad_bond=none
f_pad_input=none
f_daemon=absent

# The radio as the kernel has it wired up: the driver bound to it, and for a UART-attached part the
# device-tree compatible that names the chip. No Bluetooth tool reports either, and both outlive the
# boot log — which is why this exists next to the firmware lines rather than instead of them.
chip_identity() {
    found=
    for hci in "$SYS_BT"/hci*; do
        [ -d "$hci" ] || continue
        one="$(basename "$hci")"
        if [ -L "${hci}/device/driver" ]; then
            one="${one} $(basename "$(readlink "${hci}/device/driver")")"
        fi
        # NUL-separated, most specific compatible first, so this names the part and not just the bus.
        if [ -r "${hci}/device/of_node/compatible" ]; then
            one="${one} $(tr '\0' ' ' < "${hci}/device/of_node/compatible")"
        elif [ -r "${hci}/device/modalias" ]; then
            one="${one} $(cat "${hci}/device/modalias")"
        fi
        found="${found}${found:+, }$(trim "$one")"
    done
    printf '%s' "${found:-no hci device in ${SYS_BT}}"
}

os_pretty() {
    [ -r "$OS_RELEASE" ] || { printf 'unknown'; return 0; }
    # Sourced would be shorter and would also execute whatever is in the file, on a board where the
    # file is one of the things that might be wrong.
    grep -E '^PRETTY_NAME=' "$OS_RELEASE" 2>/dev/null | head -1 | cut -d= -f2- | tr -d '"' \
        || printf 'unknown'
}

section_report() {
    echo "report"
    field script "pad-stack-report.sh"
    field when "$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo unknown)"
    field host "$(hostname 2>/dev/null || echo unknown)"
    f_os="$(os_pretty)"
    f_kernel="$(uname -r 2>/dev/null || echo unknown)"
    field os "${f_os:-unknown}"
    field kernel "$f_kernel"
    # hci0 does not exist for roughly the first 73 seconds of a boot on this board, so a report
    # taken a minute in is a report of a radio that has not arrived yet.
    up="$(cut -d' ' -f1 /proc/uptime 2>/dev/null || true)"
    field uptime "${up:-unknown}${up:+ s}"
    echo
}

section_stack() {
    echo "host stack"

    if have bluetoothctl; then
        # `bluetoothctl: 5.82`, and the daemon ships in the same package — so this one number is the
        # BlueZ userspace both halves come from.
        f_bluez="$(bt --version | awk '{print $NF}' | head -1)"
        [ -n "$f_bluez" ] || f_bluez=unreadable
    fi
    field bluez "$f_bluez"

    if have btmon; then
        field btmon "$(btmon --version 2>/dev/null | awk '{print $NF}' | head -1 || echo unreadable)"
    else
        field btmon "absent — no HCI capture on this board"
    fi

    field bluetoothd "$(systemctl is-active bluetooth 2>/dev/null || echo unknown)"

    # The one BlueZ setting that decides whether a pad can bond at all. `device` is what
    # `setup-board.sh` sets and what a board was measured bonding under; `off` is what boards
    # provisioned during the spell described there carry. Fingerprinted because it is invisible
    # everywhere else and differs between boards flashed months apart.
    if [ -r "$BT_CONF" ]; then
        if grep -Eq '^[[:space:]]*Privacy[[:space:]]*=' "$BT_CONF" 2>/dev/null; then
            f_privacy="$(grep -E '^[[:space:]]*Privacy[[:space:]]*=' "$BT_CONF" | head -1 \
                | cut -d= -f2- | tr -d ' ')"
        else
            f_privacy="unset (BlueZ defaults to off; setup-board.sh sets device)"
        fi
    else
        f_privacy="no ${BT_CONF}"
    fi
    field privacy "$f_privacy"

    # Named individually, present or not: which of `hidp` and `uhid` is loaded is the difference
    # between the kernel carrying classic HID and BlueZ carrying LE HID, and that is the single most
    # behaviour-changing difference two boards can have here.
    if [ -r "$MODULES" ]; then
        loaded=
        for mod in $WATCHED_MODULES; do
            # /proc/modules spells them with underscores, whatever the file on disk is called.
            name="$(printf '%s' "$mod" | tr '-' '_')"
            if awk -v m="$name" '$1 == m { found = 1 } END { exit !found }' "$MODULES"; then
                loaded="${loaded}${loaded:+ }${mod}"
            fi
        done
        # Empty is a real answer, not a failure: a kernel with all of this built in loads no modules
        # at all, and that board is genuinely different from one that loads seven.
        f_modules="${loaded:-none loaded (built in?)}"
    else
        f_modules="cannot read ${MODULES}"
    fi
    field modules "$f_modules"
    echo
}

section_adapter() {
    echo "adapter"

    # No root needed for this one, and it is the line that says whether there is a radio at all.
    # The word "Controller" is dropped from each line: it prefixes every one of them, and the
    # label already says what this is.
    controllers="$(bt list | sed -E 's/^[[:space:]]*(Controller )?//' || true)"
    if [ -z "$controllers" ]; then
        field controller "none — bluetoothd reports no adapter"
    elif [ "$(printf '%s\n' "$controllers" | wc -l | tr -d ' ')" = 1 ]; then
        field controller "$(trim "$controllers")"
    else
        # More than one radio on this board is unusual enough to be worth showing whole.
        echo "  controller"
        dump "$controllers"
    fi

    # `btmgmt info` is the modern one and needs root; `hciconfig` is deprecated, still present on
    # this board's image, and answers the same question. Both are tried before giving up, because a
    # report that says "unreadable" where the other tool would have answered is a wasted trip to the
    # robot.
    if have btmgmt; then
        info="$(btmgmt info 2>/dev/null || true)"
        version="$(printf '%s' "$info" | grep -E 'addr .* version' | head -1 \
            | sed -E 's/.*(version .*)/\1/' || true)"
        settings="$(printf '%s' "$info" | grep -E '^[[:space:]]*current settings:' | head -1 \
            | sed -E 's/.*current settings:[[:space:]]*//' || true)"
        [ -n "$version" ] && f_hci="$version"
        [ -n "$settings" ] && f_settings="$settings"
    fi
    if [ "$f_hci" = unreadable ] && have hciconfig; then
        f_hci="$(hciconfig -a 2>/dev/null | grep -E 'HCI Version|LMP Version|Manufacturer' \
            | sed 's/^[[:space:]]*//' | tr '\n' ' ' | sed 's/[[:space:]]*$//' || true)"
        [ -n "$f_hci" ] || f_hci=unreadable
    fi
    if [ "$f_hci" = unreadable ]; then
        f_hci="unreadable — needs root, or neither btmgmt nor hciconfig is installed"
    fi
    field hci "$f_hci"
    field settings "$f_settings"

    # Which radio this is, from sysfs rather than from the log. The first run on a board came back
    # with no kernel Bluetooth lines at all — ring buffer wrapped, or journald keeping no kmsg — and
    # left the only section that names the hardware empty. sysfs is still there an hour into a boot,
    # and needs no root.
    f_chip="$(chip_identity)"
    field chip "$f_chip"

    # The controller's own firmware, which on a UART-attached radio is a blob the kernel loads at
    # boot and names only in the log. No Bluetooth command reports it, and it is exactly the thing
    # that differs between two boards flashed from different images.
    fw=
    if have journalctl; then
        fw="$(journalctl -k -b --no-pager 2>/dev/null \
            | grep -E 'Bluetooth: hci[0-9]' \
            | grep -iE 'firmware|patch|version|chip|hci_uart|RTL|BCM' | tail -6 \
            | sed -E 's/^.*kernel: //' || true)"
    fi
    if [ -z "$fw" ] && have dmesg; then
        fw="$(dmesg 2>/dev/null | grep -E 'Bluetooth: hci[0-9]' \
            | grep -iE 'firmware|patch|version|chip|RTL|BCM' | tail -6 || true)"
    fi
    # Boot lines age out of both of those. Every boot the journal still holds is better than none,
    # and each line says plainly that it is not from this one.
    if [ -z "$fw" ] && have journalctl; then
        fw="$(journalctl -k --no-pager 2>/dev/null | grep -E 'Bluetooth: hci[0-9]' \
            | grep -iE 'firmware|patch|version|chip|RTL|BCM' | tail -4 \
            | sed -E 's/^.*kernel: /(an earlier boot) /' || true)"
    fi
    if [ -n "$fw" ]; then
        echo "  firmware"
        dump "$fw"
        # One line of it for the fingerprint: the last version-bearing line, which is the one that
        # names what was loaded rather than the steps getting there.
        f_ctrl_fw="$(printf '%s\n' "$fw" | grep -iE 'firmware|version|chip' | tail -1 \
            | sed -E 's/^[[:space:]]*//' || true)"
        [ -n "$f_ctrl_fw" ] || f_ctrl_fw="$(printf '%s\n' "$fw" | tail -1)"
    else
        # Last tier, and deliberately unfiltered: a radio driven by an out-of-tree driver does not
        # prefix its lines `Bluetooth: hci0:` at all, and every pattern above assumes it does. Six
        # lines of whatever mentions Bluetooth beats a section that says nothing on such a board.
        if have journalctl; then
            fw="$(journalctl -k -b --no-pager 2>/dev/null | grep -iE 'bluetooth|btusb|hci[0-9]' \
                | tail -6 | sed -E 's/^.*kernel: //' || true)"
        fi
        if [ -z "$fw" ] && have dmesg; then
            fw="$(dmesg 2>/dev/null | grep -iE 'bluetooth|btusb|hci[0-9]' | tail -6 || true)"
        fi
    fi

    if [ -n "$fw" ] && [ "$f_ctrl_fw" = unreadable ]; then
        echo "  firmware       (nothing matched a firmware pattern; the last Bluetooth lines)"
        dump "$fw"
        f_ctrl_fw="unnamed by the log; see chip"
    elif [ -z "$fw" ]; then
        # The places tried are named, because "unknown" here reads as a script that did not look.
        # The chip line above still identifies the radio.
        f_ctrl_fw="not in journalctl -k -b, dmesg or journalctl -k (ring buffer wrapped, or"
        f_ctrl_fw="${f_ctrl_fw} journald keeps no kmsg)"
        field firmware "$f_ctrl_fw"
    fi
    echo
}

# Every bonded device that looks like a gamepad, connected first.
#
# The heuristic mirrors `looks_like_a_gamepad` in `configd/src/pad.rs` — BlueZ's own `input-gaming`
# icon, the gamepad appearance, then the name — deliberately and not by sharing code: this script is
# read by an operator on a board where the daemon may be the broken thing, so it must not depend on
# it. If the two ever disagree about what a pad is, that disagreement is itself worth seeing.
pad_macs() {
    devices="$(bt devices Paired || true)"
    # `devices Paired` is BlueZ 5.65 and later. Older ones only have `paired-devices`, and answer
    # the newer spelling with usage text rather than an error.
    case "$devices" in
        ''|*'Usage'*|*'usage'*) devices="$(bt paired-devices || true)" ;;
    esac

    printf '%s\n' "$devices" | while read -r word mac name; do
        [ "$word" = Device ] || continue
        [ -n "$mac" ] || continue
        if [ -n "$MAC" ]; then
            # An explicit address is an instruction, not a filter to second-guess: whatever it names
            # is reported, gamepad-shaped or not.
            printf '%s\n' "$mac" | grep -iq "^${MAC}$" && printf '%s\n' "$mac"
            continue
        fi
        info="$(bt info "$mac" || true)"
        if printf '%s' "$info" | grep -q 'Icon: input-gaming'; then
            printf '%s\n' "$mac"
        elif printf '%s' "$info" | grep -qi 'Appearance: 0x03c4'; then
            printf '%s\n' "$mac"
        elif printf '%s' "$name" | grep -qiE 'controller|gamepad|joystick|dualsense|dualshock'; then
            printf '%s\n' "$mac"
        fi
    done
}

# The bond on disk, which is the only *definitive* answer to "was this pad bonded over LE or over
# BR/EDR": a LE bond stores a long-term key, a classic one stores a link key. Root-only, and worth
# the sudo — every other signal here is an inference.
bond_dir() {
    # $1 device address
    [ -d "$BT_LIB" ] || return 1
    for adapter in "$BT_LIB"/*; do
        [ -d "$adapter" ] || continue
        for device in "$adapter"/*; do
            [ -f "${device}/info" ] || continue
            base="$(basename "$device")"
            if printf '%s' "$base" | grep -iq "^${1}$"; then
                printf '%s\n' "${device}/info"
                return 0
            fi
        done
    done
    return 1
}

section_pads() {
    # A pad on a cable has no radio stack under it and looks like "no pad" to everything below, so
    # it is checked first and reported either way.
    usb="$(input_on_bus "$BUS_USB" '' js || true)"

    macs="$(pad_macs || true)"
    if [ -z "$macs" ]; then
        echo "pad"
        if [ -n "$MAC" ]; then
            field bonded "no bond for ${MAC} on this board"
        else
            field bonded "nothing bonded that looks like a gamepad"
            field next "sudo robotctl pad pair, with the pad in pairing mode"
        fi
        if [ -n "$usb" ]; then
            field usb "a pad is connected on USB — a cable has no radio stack to compare"
        fi
        echo
        return 0
    fi

    # Connected first, like `robotctl pad status` — the connected pad is the one whose stack is
    # actually in use, and it is the one the fingerprint should describe.
    ordered=
    for mac in $macs; do
        if bt info "$mac" | grep -q 'Connected: yes'; then
            ordered="${mac} ${ordered}"
        else
            ordered="${ordered} ${mac}"
        fi
    done

    first=1
    for mac in $ordered; do
        [ -n "$mac" ] || continue
        info="$(bt info "$mac" || true)"
        name="$(printf '%s' "$info" | grep -E '^[[:space:]]*Name:' | head -1 \
            | sed -E 's/.*Name:[[:space:]]*//' || true)"
        echo "pad  ${name:-unknown} ${mac}"

        # The whole of BlueZ's own answer, indented, rather than fields picked out of it. It is
        # twenty lines, every one of them a difference worth seeing between two boards, and picking
        # fields is how a report silently stops carrying the one that mattered.
        dump "$info"

        # Address type, from the first line: `Device 78:86:.. (public)`.
        addr_type="$(printf '%s' "$info" | grep -E "^Device ${mac}" | head -1 \
            | sed -E 's/.*\((.*)\).*/\1/' || true)"
        [ -n "$addr_type" ] || addr_type=unknown

        # `usb:v045Ep0B13d0509` — vendor, product, and the device revision, which is what an Xbox
        # pad bumps when its firmware is updated and the closest thing to a firmware version
        # readable from Linux at all. BlueZ builds it from the pad's PnP ID characteristic, so it is
        # absent until the pad has been queried once.
        modalias="$(printf '%s' "$info" | grep -E '^[[:space:]]*Modalias:' | head -1 \
            | sed -E 's/.*Modalias:[[:space:]]*//' || true)"

        bond="unreadable — root reads /var/lib/bluetooth"
        bond_file="$(bond_dir "$mac" || true)"
        if [ -n "$bond_file" ] && [ -r "$bond_file" ]; then
            # Every section name in the file, verbatim, rather than a lookup of the three this
            # script expected. The first run on real hardware reported `IRK` and nothing else for a
            # pad demonstrably connected over LE — so either BlueZ 5.82 stores the LE key under a
            # name not guessed here, or there is none in the file, and a curated list cannot tell
            # those two apart. Printing what is actually in the file can.
            bond="$(grep -oE '^\[[A-Za-z]+\]' "$bond_file" 2>/dev/null | tr -d '[]' \
                | grep -v '^General$' | tr '\n' ' ' || true)"
            bond="$(trim "${bond:-no sections}")"
            # `[DeviceID]` carries the same vendor/product/version BlueZ turns into Modalias, and it
            # is there when the pad has not been queried this boot and Modalias is not.
            if [ -z "$modalias" ]; then
                devid="$(sed -n '/^\[DeviceID\]/,/^\[/p' "$bond_file" 2>/dev/null \
                    | grep -E '^(Source|Vendor|Product|Version)=' | tr '\n' ' ' || true)"
                [ -n "$devid" ] && modalias="from the bond: ${devid}"
            fi
        fi
        field bond "$bond"

        # The live link. `hcitool con` is the one command that says LE or ACL for a connection that
        # exists right now, which is a different question from what the bond was made over — a pad
        # can be bonded over LE and, on a board with a classic HID stack, reconnect over BR/EDR.
        link="unknown — no hcitool; reconnect the pad under btmon to see it"
        if have hcitool; then
            con="$(hcitool con 2>/dev/null | grep -i "$mac" | sed 's/^[[:space:]]*//' || true)"
            if [ -n "$con" ]; then
                link="$(trim "$con")"
            else
                link="not connected right now"
            fi
        fi
        field link "$link"
        field "address type" "$addr_type"
        field modalias "${modalias:-not reported — BlueZ has no PnP ID for this pad yet}"

        # The verdict, with its evidence named. Every part of it is an inference except the bond,
        # which is why the inputs are printed above rather than replaced by this line.
        transport=unknown
        # A long-term key is an LE bond, a link key a BR/EDR one. Three spellings of the former:
        # BlueZ stores the peripheral's own key under its own name, and renamed that once.
        ltk=
        case " $bond " in
            *' LongTermKey '*|*' PeripheralLongTermKey '*|*' SlaveLongTermKey '*) ltk=1 ;;
        esac
        [ -n "$ltk" ] && transport="LE"
        case " $bond " in
            *' LinkKey '*)
                if [ -n "$ltk" ]; then transport="LE and BR/EDR (dual)"; else transport="BR/EDR"; fi
                ;;
        esac
        case "$link" in *' LE '*|*'< LE'*) transport="LE" ;; esac
        case "$link" in *'ACL'*) transport="BR/EDR" ;; esac
        if [ "$transport" = unknown ] \
            && printf '%s' "$info" | grep -q 'Human Interface Device' \
            && printf '%s' "$info" | grep -q 'Generic Attribute'; then
            transport="LE (inferred: HID over GATT services, no class)"
        fi
        field transport "$transport"

        # What the kernel actually put on the input bus, which is what `padd` reads. `Bus=0005` is
        # Bluetooth, and Vendor/Product/Version here are the quadruple SDL and gilrs hash into a
        # mapping GUID — so two boards differing on this line are two boards with different button
        # and axis mappings, whatever else matches.
        input="$(pad_input "$mac" || true)"
        # One Xbox pad registers two input devices — the pad, and a consumer-control keyboard for its
        # media keys. Both are shown: a board that registered only the second one is a board where
        # nothing can be driven, and that is only visible if both are there. gilrs reads the one with
        # a joystick handler, so that one is named.
        # Device nodes only. On this board's vendor kernel the handler list also carries `kbd` and
        # `dmcfreq` — the Rockchip memory-frequency governor, which listens to input events and
        # drives nothing — and neither is the node gilrs opens.
        js_line="$(input_on_bus "$BUS_BLUETOOTH" "$mac" js | grep -E '^H:' | head -1 \
            | sed -E 's/^H: Handlers=//' | tr ' ' '\n' \
            | grep -E '^(js|event)[0-9]+$' | tr '\n' ' ' || true)"
        if [ -n "$input" ]; then
            echo "  input"
            dump "$input"
            field "js node" "$(trim "${js_line:-none: joydev missing, or no joystick device}")"
        else
            field input "no input device for this pad — not connected, or joydev is missing"
        fi

        if [ "$first" = 1 ]; then
            f_pad_id="${modalias:-unknown} (${addr_type})"
            f_pad_transport="$transport"
            f_pad_bond="$bond"
            f_pad_input="$(printf '%s\n' "$input" | grep -E '^I:' | head -1 \
                | sed 's/^[[:space:]]*//' || true)"
            [ -n "$f_pad_input" ] || f_pad_input=none
            first=0
        fi
        echo
    done

    if [ -n "$usb" ]; then
        echo "also on USB"
        dump "$usb"
        echo
    fi
}

# Records from /proc/bus/input/devices on one bus, whole, optionally for one address and optionally
# only those with a joystick handler.
#
# Reported rather than parsed into fields: the `I:` line carries the ids, `N:` the name, `U:` the
# address and `H:` the handlers, and all four are worth comparing between boards.
input_on_bus() {
    # $1 bus, $2 address to insist on or empty for any, $3 non-empty for joystick handlers only
    [ -r "$INPUT_DEVICES" ] || return 0
    # Defaulted, not required: `set -u` would otherwise turn "every device on this bus" — which is
    # what one caller wants — into an unbound-variable exit halfway through a report.
    awk -v bus="$1" -v want="${2:-}" -v js_only="${3:-}" '
        BEGIN { RS = ""; FS = "\n" }
        {
            b = ""; uniq = ""; keep = ""; js = 0
            for (i = 1; i <= NF; i++) {
                if ($i ~ /^I:/ && match($i, /Bus=[0-9a-fA-F]+/))
                    b = substr($i, RSTART + 4, RLENGTH - 4)
                else if ($i ~ /^U: Uniq=/) { uniq = $i; sub(/^U: Uniq=/, "", uniq) }
                else if ($i ~ /^H: Handlers=/ && $i ~ /js[0-9]/) js = 1
                # `S:` kept: an LE pad reaches userspace through `uhid` and a classic one through
                # `hidp`, and the sysfs path says which — the answer the loaded-module list cannot
                # give when either is built into the kernel rather than loaded as a module.
                if ($i ~ /^[INUHS]:/) keep = keep $i "\n"
            }
            if (b != bus) next
            # A joystick handler, not merely an event node: every board has a USB keyboard record,
            # and none of them is a pad.
            if (js_only != "" && !js) next
            if (want != "" && tolower(uniq) != tolower(want)) next
            printf "%s", keep
        }
    ' "$INPUT_DEVICES"
}

pad_input() {
    input_on_bus "$BUS_BLUETOOTH" "$1"
}

section_daemons() {
    echo "daemons"
    for unit in padd btd configd robotd; do
        field "$unit" "$(systemctl is-active "$unit" 2>/dev/null || echo unknown)"
    done

    if have robotctl; then
        f_daemon="$(robotctl --version 2>/dev/null | awk '{print $NF}' | head -1 || true)"
        [ -n "$f_daemon" ] || f_daemon=unreadable
        field robotctl "$f_daemon"
        # The daemon's own answer, which needs its socket and so can fail on exactly the board this
        # report is being run on. Not a reason to fail the report.
        status="$(robotctl pad status 2>&1 || true)"
        dump "$status"
    else
        field robotctl "absent — no release installed, or not on PATH"
    fi
    echo
}

# Only what must match, in a fixed order, with nothing volatile. Everything here is a value some
# other board could differ on; nothing here is an address, a timestamp or a battery level.
section_fingerprint() {
    echo "fingerprint"
    field kernel "$f_kernel"
    field os "${f_os:-unknown}"
    field bluez "$f_bluez"
    field privacy "$f_privacy"
    field modules "$f_modules"
    field hci "$f_hci"
    field settings "$f_settings"
    field chip "$f_chip"
    field controller "$f_ctrl_fw"
    field pad "$f_pad_id"
    field transport "$f_pad_transport"
    field bond "$f_pad_bond"
    field input "$f_pad_input"
    field daemon "$f_daemon"
}

# ---------------------------------------------------------------------------- run

have uname || die "no uname; this is not a board this script understands"

tmp="$(mktemp)" || die "cannot create a temporary file"
trap 'rm -f "$tmp"' EXIT INT TERM

# Redirecting a function does not fork a subshell, so the `f_*` variables the fingerprint reads are
# set by this and survive it. A pipeline here would silently break that.
{
    section_report
    section_stack
    section_adapter
    section_pads
    section_daemons
} > "$tmp"

if [ -n "$FINGERPRINT" ]; then
    # The full report was still collected — that is where these values come from — and is simply not
    # printed. Nothing volatile reaches stdout, so two of these diff to nothing when two boards
    # match.
    section_fingerprint
    exit 0
fi

section_fingerprint >> "$tmp"

if [ -z "$OUT" ]; then
    stamp="$(date -u +%Y%m%dT%H%M%SZ 2>/dev/null || echo now)"
    OUT="/tmp/pad-stack-$(hostname 2>/dev/null || echo board)-${stamp}.log"
fi

if cp "$tmp" "$OUT" 2>/dev/null; then
    saved="$OUT"
else
    saved=
fi

cat "$tmp"

# The path goes to stderr, not stdout: `diff <(sh pad-stack-report.sh) other.log` has to compare
# reports, not a report and a sentence about where it was saved.
if [ -n "$saved" ]; then
    echo "pad-stack-report.sh: saved to ${saved}" >&2
else
    echo "pad-stack-report.sh: could not write ${OUT}; the report above is all of it" >&2
fi
