#!/bin/sh
# Install Rockchip's rkaiq 3A engine and the IMX219 tuning, so the camera has white balance,
# colour and noise reduction instead of raw ISP defaults.
#
# Without this the ISP runs with no tuning and no 3A loop at all: the picture is green and noisy.
# Exposure is *not* part of what this delivers — the engine's AE converges once at stream start
# and then stops, so `mediad`'s `exposure` module owns the sensor. See the note below.
#
#   sudo sh /tmp/setup-rkaiq.sh
#   sudo /usr/local/sbin/robot-setup-rkaiq          # later, to re-check
#
# Full paths, because this advice gets copy-pasted and /tmp does not survive a reboot. The
# first run leaves a copy at the second path.
#
# **`hooks/preinstall` runs this too**, from `scripts/setup-rkaiq.sh` in the release, so a board
# provisioned before this existed gets the engine from an ordinary update rather than from
# somebody remembering a command. Same contract as `setup-gstreamer.sh`: never prompt, never be
# fatal, and say what happened.
#
#   --sensor NAME   tuning to install for (default imx219). Radxa's IQ package only ships
#                   imx219 and ov5647; anything else needs its own
#                   <sensor>_<module>_default.json in /etc/iqfiles.
#   --help
#
# Idempotent: the debs are only fetched when missing, the IQ patch is a no-op once applied, and
# the shim is rebuilt each run because it is cheap and because the kernel it probes can change
# under it.
#
# Radxa Zero 3W on the Armbian vendor kernel. Needs the camera overlay active — the ISP
# parameter and statistics nodes only exist there, and the engine has nothing to talk to
# without them.
#
# ── Why a vendor engine at all ────────────────────────────────────────────────
#
# There is no other 3A on this platform. libcamera's rkisp1 IPA drives the *mainline* rkisp1
# driver; this board runs Rockchip's vendor rkisp on the vendor kernel, which is also where the
# hardware encoder `mediad` depends on lives (`docs/project/media-bringup.md`). Choosing
# mainline to get an open 3A would cost the VPU, so the vendor engine — a prebuilt deb from
# Radxa's pool, taken as a direct download the way the MPP packages are — is the route.
#
# ── What differs from the prototype's version of this script ──────────────────
#
# **rkaiq's auto exposure is turned off here, as the prototype turned it off — because it fires
# once and `mediad` needs it to keep firing.** This script first shipped with AE left enabled, on
# the reasoning that the prototype only disabled it to stop the engine fighting a runtime that
# owned exposure itself, and that nothing owned exposure here. Measured on a robot: the engine's
# AE does write the sensor, once, at stream start — `mediad` pinned `exposure=600
# analogue_gain=1024` and the sensor was found at `1589 / 1536`, which is the engine's answer and
# not ours. But a hand-written `exposure=300 analogue_gain=256` on that same healthy boot held for
# 25 seconds with no correction. One convergence is not auto-exposure, and a robot that walks from
# a window into a corridor keeps the window's exposure.
#
# So `mediad::exposure` meters the frames and drives the sensor (the prototype's loop, ported),
# and this leaves the engine the parts it does continuously: white balance, the colour matrix,
# gamma and noise reduction. Its one-shot is disabled rather than tolerated because it lands at
# stream start, which is exactly when mediad's loop is converging from its own starting values —
# two writers, in a race, for one control.
set -e

SENSOR=imx219
SELF=/usr/local/sbin/robot-setup-rkaiq
SHIM_SO=/usr/local/lib/rkaiq_modinfo_shim.so
IQ_DIR=/etc/iqfiles
DROP_IN_DIR=/etc/systemd/system/rkaiq_3A.service.d
PIN=/usr/local/bin/rkaiq-pin-sensor-mode

# Radxa's apt pool, as direct .deb downloads rather than an entry in sources.list — the same
# route `setup-gstreamer.sh` takes for MPP, and for the same reason: one pinned artifact each,
# no third-party repository left enabled on the robot afterwards.
POOL=https://radxa-repo.github.io/bullseye/pool/main
RKAIQ_DEB="$POOL/c/camera-engine-rkaiq/camera_engine_rkaiq_rk3568_arm64-fixed.deb"
IQ_DEB="$POOL/r/rockchip-iqfiles/rockchip-iqfiles-rk356x_0.1.16_all.deb"

# The sensor mode the engine must agree with `mediad` about. See `pin_mode` below for why this
# is here at all, and keep it in step with `pin_sensor_mode` in `mediad/src/pipeline.rs`.
SENSOR_W=1920
SENSOR_H=1080

say()  { printf '== %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,/^set -e/{/^set -e/d;s/^# \{0,1\}//;p;}' "$0"
    exit 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        --sensor) SENSOR="${2:?--sensor needs a name}"; shift 2 ;;
        --help|-h) usage ;;
        *) die "unknown argument: $1 (try --help)" ;;
    esac
done

[ "$(id -u)" = 0 ] || die "run as root: sudo sh $0"

# Where this script and its shim source live, so a run from /tmp, from the release, or from a
# clone all find the C file. The release lays both out side by side under scripts/.
HERE="$(cd "$(dirname "$0")" && pwd)"
SHIM_SRC="${HERE}/rkaiq-modinfo-shim.c"

# ── the engine and its tuning ─────────────────────────────────────────────────

fetch_deb() {
    package="$1"
    url="$2"
    if dpkg -s "$package" >/dev/null 2>&1; then
        say "${package} already installed"
        return 0
    fi
    tmp="/tmp/${package}.deb"
    say "fetching ${package}"
    curl -fsSL -o "$tmp" "$url" || die "could not download ${url}
  The camera works without 3A — green, noisy and fixed-exposure — so this is not fatal to the
  robot, but it is fatal to image quality. Check the board's network and re-run:
    sudo ${SELF}"
    dpkg -i "$tmp" || die "dpkg could not install ${tmp}"
    rm -f "$tmp"
}

fetch_deb camera-engine-rkaiq-rk3568 "$RKAIQ_DEB"
fetch_deb rockchip-iqfiles-rk356x "$IQ_DEB"

say "installing ${SENSOR} tuning into ${IQ_DIR}"
mkdir -p "$IQ_DIR"
for f in /usr/share/rockchip-iqfiles-rk356x/"${SENSOR}"_*; do
    [ -e "$f" ] || continue
    [ -e "${IQ_DIR}/$(basename "$f")" ] || cp "$f" "$IQ_DIR/"
done

# `set -e` plus a glob that matches nothing would end the script here, which is the wrong
# outcome for a sensor nobody has tuning for: the engine still runs, badly, and saying so is
# more useful than stopping.
if ! ls "${IQ_DIR}/${SENSOR}"_*.json >/dev/null 2>&1; then
    warn "no IQ tuning for '${SENSOR}' in ${IQ_DIR}.
  Radxa's package ships imx219 and ov5647 only. The engine will run on raw ISP defaults, which
  looks green and noisy. Drop a ${SENSOR}_<module>_default.json in ${IQ_DIR} to fix it."
fi

# Two enum names in Radxa's IQ file are newer than the parser in Radxa's own engine deb. Left
# alone they are warnings, except that a field the parser cannot read silently falls back to a
# default — an engine that starts on half the tuning is worse than one that refuses.
#
# Only characterised for imx219: another sensor's file may not contain these names at all, and
# rewriting enums in tuning nobody has read is how you get an image that is subtly wrong.
if [ "$SENSOR" = imx219 ] && [ -f "${IQ_DIR}/imx219_rpi-camera-v2_default.json" ]; then
    say "checking IQ enum names against the engine's parser"
    python3 - <<'PY'
# Binary mode throughout: the file has CRLF line endings that text mode would rewrite, and a
# diff of the whole file is not what anyone wants out of this.
path = "/etc/iqfiles/imx219_rpi-camera-v2_default.json"
raw = open(path, "rb").read()
renames = [
    (b"AECV2_STRATEGY_MODE_LOWLIGHT_PRIOR", b"AECV2_STRATEGY_MODE_LOWLIGHT"),
    (b"CALIB_AWB_HDR_FRAME_CHOOSE_MODE_AUTO", b"CALIB_AWB_HDR_FR_CH_AUTO"),
]
changed = 0
for old, new in renames:
    n = raw.count(old)
    if n:
        raw = raw.replace(old, new)
        changed += n
if changed:
    open(path, "wb").write(raw)
    print(f"   renamed {changed} enum value(s) the parser does not know")
else:
    print("   enum names already match the parser")

# `ae_calib CommCtrl.Enable` to 0: exposure belongs to `mediad`'s software loop, for the reason at
# the top of this file — the engine's AE converges once at stream start and then stops, which is
# the same moment mediad's loop is converging. Two writers for one control is worse than one that
# keeps working, so the switch is asserted rather than assumed; an apt upgrade or a fresh IQ file
# arrives with it on.
import re
m = re.search(rb'("CommCtrl":\s*\{\s*"Enable":\s*)1', raw)
if m:
    raw = raw[: m.start()] + m.group(1) + b"0" + raw[m.end() :]
    open(path, "wb").write(raw)
    print("   rkaiq AE was enabled in this file — disabled it; mediad owns exposure")
else:
    print("   rkaiq AE is off, as mediad's exposure loop needs")
PY
fi

# ── the ioctl shim ────────────────────────────────────────────────────────────
#
# Without this, `rkaiq_3A_server` segfaults on this kernel before it ever reaches a frame; the
# reasoning is in the C file's header. Built on the board because the struct size it probes for
# belongs to the running kernel.

if [ ! -f "$SHIM_SRC" ]; then
    die "no ${SHIM_SRC} beside this script.
  The release carries scripts/rkaiq-modinfo-shim.c next to scripts/setup-rkaiq.sh; a copy of
  the script alone cannot build the shim, and the engine segfaults without it."
fi

if ! command -v gcc >/dev/null 2>&1; then
    say "installing gcc, to build the shim"
    apt-get install -y --no-install-recommends gcc \
        || die "could not install gcc, so the shim cannot be built"
fi

# Only when there is something to build. `hooks/preinstall` runs this script on every update, so
# an unconditional `gcc` is a compile on every release for ever on a board that already has the
# shim — the cost updater-design.md §9.1 says a hook step may not have. The built source is kept
# beside the object, the way `setup-npu.sh` keeps its `.dts`, and a release that changes the C
# file rebuilds because the copy no longer matches.
#
# Keyed on the source alone, and not on `uname -r`, because the shim does not depend on the
# kernel it was built against: it brute-forces the kernel's struct size at *runtime*, once, on
# the first intercepted ioctl. It is built on the board because it must be aarch64, which is the
# only reason.
SHIM_BUILT_SRC=/usr/local/lib/rkaiq-modinfo-shim.c
if [ -f "$SHIM_SO" ] && cmp -s "$SHIM_SRC" "$SHIM_BUILT_SRC"; then
    say "the ioctl shim is current"
else
    say "building the ioctl shim"
    gcc -shared -fPIC -O2 -o "$SHIM_SO" "$SHIM_SRC" -ldl \
        || die "could not build ${SHIM_SRC}"
    install -m 644 "$SHIM_SRC" "$SHIM_BUILT_SRC"
fi

# ── the sensor-mode pin ───────────────────────────────────────────────────────
#
# `rkaiq_3A_server` reads the sensor's resolution once, at startup, and programs the ISP input
# with it for every stream afterwards. The IMX219 boots in 3280x2464; `mediad` captures from
# the 1920x1080 mode (`pin_sensor_mode`, which is also what gets 30 fps rather than 21). The
# engine starts long before `mediad` does, so without this it reads the boot mode and every
# later capture dies with CIF_ISP_PIC_SIZE_ERROR — the camera delivers no frame at all, which
# reads as "the camera is broken" rather than "two components disagree by a resolution".
#
# So pin the mode before the engine starts, to the same geometry `mediad` pins. Both pinning it
# is deliberate: `mediad` cannot rely on this script having run, and this script cannot rely on
# `mediad` having started.

say "installing the sensor-mode pin at ${PIN}"
cat > "$PIN" <<PIN_HEAD
#!/bin/sh
# Pin the ${SENSOR} into its ${SENSOR_W}x${SENSOR_H} mode before rkaiq_3A_server reads it.
# Installed by scripts/setup-rkaiq.sh. Keep in step with pin_sensor_mode in
# mediad/src/pipeline.rs.
SENSOR="${SENSOR}"
WANT="${SENSOR_W}x${SENSOR_H}"
PIN_HEAD
cat >> "$PIN" <<'PIN_BODY'
# This runs at sysinit, which can be before the camera driver has probed — so wait for the
# entity rather than concluding there is no camera. Ten seconds, then give up quietly: a board
# with no camera module must still boot, and the engine's own log says the rest.
i=0
while [ "$i" -lt 40 ]; do
    for m in /dev/media*; do
        [ -e "$m" ] || continue
        entity=$(media-ctl -d "$m" -p 2>/dev/null \
            | sed -n "s/^- entity [0-9]*: \(m[0-9]*_[bf]_${SENSOR} [0-9-]*\).*/\1/p" \
            | head -1)
        if [ -n "$entity" ]; then
            if media-ctl -d "$m" --set-v4l2 "\"${entity}\":0[fmt:SRGGB10_1X10/${WANT}]"; then
                echo "pinned ${entity} to ${WANT} on ${m}"
            else
                echo "could not pin ${entity} to ${WANT} on ${m}" >&2
            fi
            exit 0
        fi
    done
    i=$((i + 1))
    sleep 0.25
done
echo "no ${SENSOR} entity appeared; leaving the sensor mode alone" >&2
exit 0
PIN_BODY
chmod 755 "$PIN"

# ── wiring, and the report ────────────────────────────────────────────────────

say "wiring the shim and the pin into rkaiq_3A.service"
mkdir -p "$DROP_IN_DIR"
cat > "${DROP_IN_DIR}/robot.conf" <<DROPIN
# Installed by scripts/setup-rkaiq.sh. Every line is load-bearing.
#
# LD_PRELOAD is what keeps the engine from segfaulting on this kernel, and the pin is what keeps it
# from programming the ISP with the sensor's boot resolution.
#
# **The ExecStartPost is an ordering invariant, and it was learned the hard way.** The engine
# attaches to the ISP and then waits for a *stream start* event — and it misses one that already
# happened. Restart it while \`mediad\` is streaming (which every \`robotctl update apply\` did, because
# the pre-install hook runs this script) and it sits on "wait stream start event..." for ever: no
# stats loop, no auto exposure, no white balance, and a green picture that a reboot fixes only
# because the reboot happens to order the two correctly. So whenever the engine starts, the camera
# stream is bounced behind it.
#
# \`try-restart\`, so a board with no \`mediad\` running is left alone, and \`--no-block\`, because a unit
# waiting on another unit's job inside its own start transaction is how you deadlock systemd.
#
# ── Why this unit is run directly rather than through its init script ─────────
#
# **The vendor unit is \`Type=forking\` over \`/etc/init.d/rkaiq_3A.sh\`, and systemd cannot supervise
# what that script starts.** The script's \`start_3A\` is \`rkaiq_3A_server 2>&1 | logger -t rkaiq &\` —
# a *pipeline*, backgrounded, and then the script exits. systemd's forking heuristic expects one
# forked child and finds none it can claim, so \`systemctl show rkaiq_3A -p MainPID\` answers
# \`MainPID=0\`: both processes end up reparented to init (they do stay in the unit's cgroup, which is
# what keeps \`systemctl stop\` honest — the cgroup kill collects them, not the unit's own
# \`ExecStop\`). What that leaves broken is the unit reading \`active\` whether the engine is alive or
# dead with nothing to restart it when it segfaults, a \`stop\` of \`killall rkaiq_3A_server\` that
# reaches any engine on the box rather than this unit's, and a \`reload\` of stop-then-start inside
# the unit that never reaches the cgroup kill and so leaks the previous run's \`logger\` every time.
#
# So: \`ExecStart\` is cleared and set to the engine itself, and the engine writes to the journal
# under this unit instead of through a \`logger\` process. systemd then knows the PID, kills the
# whole cgroup on stop, and can restart a dead engine. \`ExecStop\` is cleared too, or the vendor's
# \`killall\` would run on every stop and reach any engine on the box, not just this unit's.
#
# **The engine does not daemonise, which is what makes \`Type=simple\` right.** Measured on a board:
# the \`logger\` from the init script's pipeline was still alive alongside the server, which can only
# be true of a server holding the write end of that pipe in the foreground. A daemonising one would
# have exited its first process and closed the pipe. The report at the end of this script checks
# \`MainPID\` after a restart, so if that is ever wrong on some board, it says so there.
#
# ── Why the restart is capped ─────────────────────────────────────────────────
#
# \`Restart=\` and the \`ExecStartPost\` above are a loop waiting to happen: the known failure is a
# segfault at startup when the shim does not match the kernel, and with \`Type=simple\` the
# ExecStartPost fires as soon as the process is spawned — before the segfault is known. Uncapped,
# a board with a mismatched shim would bounce \`mediad\` every few seconds for ever, which is worse
# than the engine simply staying dead. \`StartLimitBurst\` is what makes \`Restart=\` safe here: three
# attempts, then systemd gives up and leaves the unit failed, so \`mediad\` is bounced three times
# and not indefinitely.
#
# \`on-failure\` rather than \`always\` because the segfault is the failure that has been observed and
# it exits non-zero; a clean exit from this engine has never been seen, and \`always\` would add a
# \`mediad\` bounce to a case nobody has characterised.
[Unit]
StartLimitIntervalSec=180
StartLimitBurst=3

[Service]
Type=simple
ExecStart=
ExecStart=/usr/bin/rkaiq_3A_server
ExecStop=
StandardOutput=journal
StandardError=journal
Restart=on-failure
RestartSec=10s
Environment=LD_PRELOAD=${SHIM_SO}
ExecStartPre=${PIN}
ExecStartPost=-/bin/systemctl --no-block try-restart mediad.service
DROPIN

systemctl daemon-reload
systemctl enable rkaiq_3A >/dev/null 2>&1 || warn "could not enable rkaiq_3A"

# Leave a copy for the next person, exactly as setup-gstreamer.sh does.
if [ "$(cd "$(dirname "$0")" && pwd)/$(basename "$0")" != "$SELF" ]; then
    mkdir -p /usr/local/sbin
    install -m 755 "$0" "$SELF"
    install -m 644 "$SHIM_SRC" /usr/local/lib/rkaiq-modinfo-shim.c
fi

# The ISP nodes exist only on the vendor kernel with the camera overlay active. Before the
# reboot that brings those up there is nothing to restart into, and that is a normal state
# during provisioning rather than a failure.
if [ -e /dev/video8 ] && [ -e /dev/video9 ]; then
    systemctl restart rkaiq_3A || warn "rkaiq_3A would not restart"
    # The engine either survives its own startup or segfaults in it, and which one it did is the
    # single fact worth reporting. A second is enough for the latter.
    sleep 1
    # And whether systemd is *holding* it, which is the premise of the drop-in's `Type=simple`:
    # a `MainPID` of 0 beside a running engine is the vendor unit's supervision hole reappearing,
    # and it would mean nothing restarts the engine and `systemctl stop` leaves it behind.
    main_pid=$(systemctl show rkaiq_3A -p MainPID --value 2>/dev/null || echo 0)
    if pgrep rkaiq_3A_server >/dev/null 2>&1; then
        if [ "${main_pid:-0}" -gt 0 ] 2>/dev/null; then
            say "rkaiq_3A_server is running as PID ${main_pid}, tracked by systemd, and the camera
  stream has been bounced behind it so the engine sees it start — that is the ExecStartPost in the
  drop-in above, not something to do by hand. \`journalctl -fu rkaiq_3A\` should say 'wait stream
  start event success'."
        else
            warn "rkaiq_3A_server is running, but systemd is not tracking it (MainPID=${main_pid}).
  The drop-in's Type=simple assumes the engine stays in the foreground, and on this board it
  apparently does not. Nothing will restart it if it dies, and \`systemctl stop\` will leave it
  running. Report this with:
    systemctl cat rkaiq_3A; systemctl show rkaiq_3A -p MainPID -p Type"
        fi
    else
        warn "rkaiq_3A_server is not running. The camera still works, with no 3A:
    journalctl -u rkaiq_3A -b --no-pager | tail -40
  A segfault here means the shim did not match this kernel (${SHIM_SRC}). Three attempts, then
  systemd stops trying — \`systemctl reset-failed rkaiq_3A\` after a fix, or just re-run this."
    fi
else
    say "the ISP nodes are not present yet, so nothing to start.
  That is expected before the reboot into the vendor kernel with the camera overlay; rkaiq_3A
  is enabled and starts on the next boot."
fi

say "done"
