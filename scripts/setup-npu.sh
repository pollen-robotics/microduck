#!/bin/sh
# Install Rockchip's NPU runtime, so the duck detector can run on the board's NPU.
#
# The RK3566 has a small (0.8 TOPS) INT8 NPU. Using it needs two halves that arrive from different
# places: the **driver**, which is part of the vendor kernel and is either there or is not, and the
# **runtime** `librknnrt.so`, which is a vendor blob in no Debian suite. This installs the second and
# reports on the first.
#
#   sudo sh /tmp/setup-npu.sh
#   sudo /usr/local/sbin/robot-setup-npu          # later, to re-check
#
# Full paths, because this advice gets copy-pasted and /tmp does not survive a reboot. The first run
# leaves a copy at the second path.
#
#   --runtime TAG   which rknn-toolkit2 tag to take librknnrt.so from (default below). It has to be
#                   at least as new as the toolkit that converted the model: a runtime older than
#                   its model fails at `rknn_init` with a number and no explanation.
#   --no-enable-node  leave the device tree alone. The default is to enable the NPU node, because
#                   Armbian's rk3566-radxa-zero3.dtb ships `npu@fde40000` as `status = "disabled"`
#                   on *every* Radxa Zero 3 — so a runtime installed without it can never run
#                   anything, which is not a useful thing for this script to have done. **Takes
#                   effect on the next boot**, and this script never reboots anything. Undone by
#                   removing `npu-enable` from `overlays=` in /boot/armbianEnv.txt.
#                   `--enable-node` is still accepted, and now says out loud what already happens.
#   --help
#
# Idempotent: the runtime is only downloaded when it is missing or a different version is asked for.
#
# **Run by `hooks/preinstall` on every update**, beside `setup-gstreamer.sh` and `setup-rkaiq.sh`
# and on the same terms: never fatally, with its report in the update log. That is what the split
# between provisioning and updating is for — a board provisioned before the NPU existed is fixed by
# an ordinary update rather than by somebody remembering a command, and a release that ships a model
# should not also ship a manual step before the model can run.
#
# Running it by hand is then a retry, not the mechanism.
set -e

# Keep in step with `[workspace.metadata.rknpu]` in Cargo.toml — a test asserts they agree.
RUNTIME="v2.3.2"

# **On by default, because the node is disabled on every board this will ever run on.** Off, this
# script installs a runtime that cannot reach an NPU and reports that it cannot — which is a
# diagnosis, not a setup. The change it makes is one property, appended to a list rather than
# replacing it, compiled by `dtc` before it is installed, with the previous /boot/armbianEnv.txt
# kept beside it; and the failure mode of getting it wrong is a driver that does not probe, which
# is where an untouched board already is.
ENABLE_NODE=1

SELF=/usr/local/sbin/robot-setup-npu
LIB=/usr/lib/librknnrt.so
STAMP=/usr/lib/librknnrt.version

say()  { printf '== %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,/^set -e/{/^set -e/d;s/^# \{0,1\}//;p;}' "$0"
    exit 0
}

# `dtc`, installed rather than demanded.
#
# The sibling setup scripts install what they need — `setup-gstreamer.sh` its plugins,
# `setup-board.sh` the DKMS toolchain — and a script that stops to tell somebody to run one
# `apt install` is a round trip for nothing. Returns non-zero rather than dying: the runtime is
# still worth installing on a board where apt cannot reach a mirror.
ensure_dtc() {
    command -v dtc >/dev/null 2>&1 && return 0
    command -v apt-get >/dev/null 2>&1 || return 1
    say "installing device-tree-compiler"
    apt-get update -qq || true
    apt-get install -y -qq device-tree-compiler || return 1
    command -v dtc >/dev/null 2>&1
}

while [ $# -gt 0 ]; do
    case "$1" in
        --runtime) RUNTIME="${2:?--runtime needs a tag}"; shift 2 ;;
        --enable-node) ENABLE_NODE=1; shift ;;
        --no-enable-node) ENABLE_NODE=""; shift ;;
        --help|-h) usage ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

[ "$(id -u)" = 0 ] || die "run as root: sudo sh $0"

# ── what the board already has ────────────────────────────────────────────────
#
# Read before anything is changed, because it decides whether anything needs to be. The driver
# comes with the vendor kernel; mainline has no rknpu driver at all, so a board that booted a
# mainline kernel has no NPU as far as userspace is concerned — worth saying plainly here rather
# than discovering it through `rknn_init` returning -1.

DRIVER=""
if [ -r /sys/kernel/debug/rknpu/version ]; then
    DRIVER=$(cat /sys/kernel/debug/rknpu/version 2>/dev/null || true)
elif [ -r /proc/rknpu/version ]; then
    DRIVER=$(cat /proc/rknpu/version 2>/dev/null || true)
fi
if [ -z "$DRIVER" ]; then
    DRIVER=$(dmesg 2>/dev/null | sed -n 's/.*RKNPU driver: v\([0-9.]*\).*/\1/p' | tail -1)
fi

NODE_STATUS=""
if [ -r /proc/device-tree/npu@fde40000/status ]; then
    NODE_STATUS=$(tr -d '\0' < /proc/device-tree/npu@fde40000/status)
fi

# ── the device tree ───────────────────────────────────────────────────────────
#
# One property, `status = "okay"`, on a node whose clocks, resets, power domain, IOMMU and
# regulator are all already described. It is undone by deleting one word from armbianEnv.txt.
#
# **Only when the driver is not already bound.** A board that has an NPU has nothing to gain from
# an edit to its boot configuration, and a re-run of this script should not be a reason to touch
# /boot at all.
#
# It still never reboots: the change lands on the next boot, and rebooting somebody's robot is not
# a thing a setup script decides.
if [ -n "$DRIVER" ]; then
    say "npu driver: ${DRIVER}"
elif [ -z "$ENABLE_NODE" ]; then
    warn "--no-enable-node: leaving the device tree alone.
  On a stock Armbian image that means the NPU stays disabled and nothing can use the runtime."
elif [ "$NODE_STATUS" = enabled ] || [ "$NODE_STATUS" = okay ]; then
    warn "the device tree says the NPU node is enabled, but no driver has bound to it. That is a
  kernel question rather than a device-tree one, and this script cannot answer it."
else
    ENV=/boot/armbianEnv.txt
    OVERLAY_DIR=/boot/dtb/rockchip/overlay
    # Where the .dts might be, in the order it turns up. **Beside the script first**, because that
    # is what `scp setup-npu.sh rk3568-npu-enable.dts robot:/tmp/` produces and it is what the
    # instructions say to do — the repo layout and the installed copy are the other two.
    HERE="$(cd "$(dirname "$0")" && pwd)"
    SOURCE=""
    for candidate in \
        "${OVERLAY_DTS:-}" \
        "${HERE}/rk3568-npu-enable.dts" \
        "${HERE}/../deploy/overlays/rk3568-npu-enable.dts" \
        /usr/local/lib/rk3568-npu-enable.dts
    do
        if [ -n "$candidate" ] && [ -f "$candidate" ]; then
            SOURCE="$candidate"
            break
        fi
    done

    # **Warned about rather than fatal, and that is the point of doing this by default.** A script
    # copied to a board on its own has no overlay beside it; dying here would cost it the runtime
    # install too, which is the half it can still do.
    if [ -z "$SOURCE" ]; then
        warn "cannot find rk3568-npu-enable.dts, so the NPU node stays disabled. It lives beside
  this script:
    scp scripts/setup-npu.sh deploy/overlays/rk3568-npu-enable.dts ${USER:-microduck}@<robot>:/tmp/
  or name it: OVERLAY_DTS=/path/to/rk3568-npu-enable.dts sudo -E sh $0
  The runtime installs regardless."
    elif ! ensure_dtc; then
        warn "no dtc and apt would not install device-tree-compiler, so the NPU node stays
  disabled. The runtime installs regardless."
    elif [ ! -d "$OVERLAY_DIR" ] || [ ! -f "$ENV" ]; then
        warn "no ${ENV} or ${OVERLAY_DIR}; this is not an Armbian layout, so the NPU node stays
  disabled. Load the overlay by whatever means this image provides.
  The runtime installs regardless."
    else
        say "compiling the npu overlay"
        dtc -I dts -O dtb -o "${OVERLAY_DIR}/rk3568-npu-enable.dtbo" "$SOURCE" 2>/dev/null \
            || die "dtc could not compile ${SOURCE}"
        install -m 644 "$SOURCE" /usr/local/lib/rk3568-npu-enable.dts

        if grep -qE '^overlays=.*\bnpu-enable\b' "$ENV"; then
            say "npu-enable is already in ${ENV}"
        else
            cp "$ENV" "${ENV}.before-npu"
            # Appended to the existing list rather than replacing it: those other overlays are the
            # camera, the audio codec and the uart, and a robot without them is a brick of a
            # different kind.
            awk '/^overlays=/ { print $0 " npu-enable"; next } { print }' "${ENV}.before-npu" > "$ENV"
            say "added npu-enable to ${ENV} (previous file kept as ${ENV}.before-npu)"
        fi
        NEEDS_REBOOT=1
        printf '\n'
        warn "the NPU is enabled on the NEXT BOOT. Nothing here reboots.
  To undo: remove 'npu-enable' from overlays= in ${ENV} (or restore ${ENV}.before-npu) and reboot.
  If the board does not come back, that file is what to fix from a card reader."
        printf '\n'
    fi
fi

# ── the runtime ───────────────────────────────────────────────────────────────
#
# Rockchip publish it in the rknn-toolkit2 repository rather than as a package. Taken as a direct
# download of one file at a pinned tag, the same way `setup-gstreamer.sh` takes MPP: one artifact,
# no third-party apt repository left enabled on a robot.

URL="https://raw.githubusercontent.com/airockchip/rknn-toolkit2/${RUNTIME}/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"

installed=""
[ -r "$STAMP" ] && installed=$(cat "$STAMP")

if [ -f "$LIB" ] && [ "$installed" = "$RUNTIME" ]; then
    say "librknnrt.so ${RUNTIME} already installed"
else
    say "fetching librknnrt.so ${RUNTIME}"
    tmp=$(mktemp /tmp/librknnrt.XXXXXX)
    curl -fsSL -o "$tmp" "$URL" || die "could not download ${URL}
  Check the tag exists, and that this board has a route to the internet."
    # A truncated download is a segfault at the first inference rather than an error, so the file is
    # checked for being an aarch64 shared object before it is allowed to become the runtime.
    case "$(head -c 20 "$tmp" | od -An -tx1 | tr -d ' \n')" in
        7f454c46*) ;;
        *) rm -f "$tmp"; die "that download is not an ELF object — a proxy error page, most likely" ;;
    esac
    size=$(wc -c < "$tmp")
    [ "$size" -gt 1000000 ] || { rm -f "$tmp"; die "librknnrt.so came back as ${size} bytes"; }
    install -m 644 "$tmp" "$LIB"
    rm -f "$tmp"
    printf '%s\n' "$RUNTIME" > "$STAMP"
    ldconfig
    say "installed ${LIB} (${size} bytes)"
fi

# Leave a copy for the next person, exactly as the other setup scripts do.
if [ "$(cd "$(dirname "$0")" && pwd)/$(basename "$0")" != "$SELF" ]; then
    mkdir -p /usr/local/sbin
    install -m 755 "$0" "$SELF"
fi

# ── what a caller needs to know ───────────────────────────────────────────────

say "done"
printf '\n'
printf 'runtime  %s (%s)\n' "$LIB" "$RUNTIME"
printf 'driver   %s\n' "${DRIVER:-not found}"
printf '\nbenchmark it with a model and some frames:\n'
printf '  duck-bench --model /var/tmp/duck.rknn --frames /var/tmp/frames\n'
if [ -n "${NEEDS_REBOOT:-}" ]; then
    printf '\nREBOOT FIRST. The NPU node was just enabled and binds on the next boot:\n'
    printf '  sudo reboot\n'
    # shellcheck disable=SC2016  # the backticks are for the reader, not the shell
    printf 'Then `dmesg | grep rknpu` should say the driver initialised.\n'
elif [ -z "$DRIVER" ]; then
    # shellcheck disable=SC2016  # same, and single quotes are what keeps them literal
    printf '\nExpect `rknn_init` to fail until the NPU driver is there.\n'
fi
