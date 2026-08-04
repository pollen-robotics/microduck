#!/bin/sh
# Cross-compile for the board and exercise the result on real ARM64 Linux.
#
# Target: Radxa Zero 3 (RK3566, Cortex-A55 → aarch64) running Armbian 26.2.x.
#
# The intended userland is Debian 13 (Trixie). Armbian 26.2 also offers Ubuntu Noble
# and a minimal Debian Bookworm, so we build against an older glibc than any of them
# and verify against all three — Trixie first, since that is what will be flashed.
# Keeping the floor low costs nothing and means a fallback image needs no rebuild.
#
# The kernel (6.1.115 Rockchip BSP) is irrelevant to us: flock, SO_PEERCRED and
# statvfs long predate it.
#
# Not a substitute for hardware, but it catches everything that only appears off the
# dev machine: cross-linking (notably `zstd`'s C code), glibc floors, unix-socket and
# file-permission semantics, and anything that quietly depended on macOS. On an Apple
# Silicon host these containers run arm64 *natively*, so it's fast and not emulated.
#
# Usage:  scripts/board-test.sh
#
# Requires: cargo-zigbuild, zig, docker.

set -eu

TARGET_DIR=target/aarch64-unknown-linux-gnu/release

# Build floor. Below every Armbian 26.2 userland (Bookworm 2.36, Noble 2.39,
# Trixie ships glibc 2.41, so the floor only has to be at or below that. It is pinned far
# lower (2.31) because the risk is the *build host*, not the target: an unpinned build links
# against whatever glibc the CI runner happens to have, and the day that moves above the
# board's the binaries stop loading there — with nothing in the build to hint why.
GLIBC_FLOOR=2.31


# The target userland, and only that one. Armbian offers others for this board, but we
# ship Debian 13 (Trixie), and testing configurations nobody runs costs ~2x the job time
# to defend a claim we do not need. Adding one back is a word here if that changes.
#
# Overridable so a one-off check against another userland stays possible without editing
# this file: BOARD_IMAGES="debian:bookworm-slim" ./scripts/board-test.sh
IMAGES="${BOARD_IMAGES:-debian:trixie-slim}"

# Checked up front: otherwise the build succeeds and the run fails several minutes
# later with Docker's own error, which reads like a problem with the code.
if ! docker info >/dev/null 2>&1; then
    echo "error: cannot reach the Docker daemon." >&2
    echo "       The cross-build would succeed, but the binaries could not be run." >&2
    echo "       Start Docker (or Colima/OrbStack) and retry." >&2
    exit 1
fi

FIXTURE=target/board-fixture

echo "==> cross-compiling for aarch64-unknown-linux-gnu (glibc <= $GLIBC_FLOOR)"
# pkg-config has to be told it is allowed to answer for another architecture, and where
# that architecture's .pc files live. Without both, libudev-sys (via gilrs, via padd)
# either refuses outright or silently answers with the host's library.
export PKG_CONFIG_ALLOW_CROSS="${PKG_CONFIG_ALLOW_CROSS:-1}"
export PKG_CONFIG_PATH="${PKG_CONFIG_PATH:-/usr/lib/aarch64-linux-gnu/pkgconfig}"

cargo zigbuild --release --target "aarch64-unknown-linux-gnu.$GLIBC_FLOOR" --bins

# Releases for the engine to install, minted host-native: a release is signed manifests and
# tarballs, which depend on the target architecture no more than the ones GitHub serves do.
# Building this for aarch64 too would only mean a second cross-compile to produce identical
# bytes.
#
# `--prefix` is the path *inside the container*, because that is where `updater.toml` is
# read. Every version the checks need is minted in one run: one run is one signing key, and
# releases signed by two different keys cannot sit in the same tree.
echo "==> minting release fixtures"
rm -rf "$FIXTURE"
cargo run -q -p test-support --example fake-release -- "$FIXTURE" --prefix /tmp/duck \
    1.0.0 1.1.0 1.2.0:tamper 2.0.0 3.0.0 | sed 's/^./    &/'

echo
echo "==> what we built"
file "$TARGET_DIR/updaterd" | sed 's/^/    /'

# The highest glibc symbol version is what actually determines the minimum OS.
# Building against a newer glibc links cleanly here and fails on the board, so assert
# it rather than assume.
NEEDS=$(strings "$TARGET_DIR/updaterd" | grep -oE 'GLIBC_2\.[0-9]+' | sort -uV | tail -1)
echo "    needs $NEEDS"
# Sort the two and check ours isn't the larger.
if [ "$(printf '%s\n%s\n' "GLIBC_$GLIBC_FLOOR" "$NEEDS" | sort -V | tail -1)" != "GLIBC_$GLIBC_FLOOR" ]; then
    echo "    [FAIL] needs $NEEDS, above the $GLIBC_FLOOR floor"
    exit 1
fi

# Checks run identically against each userland; kept in one place so a new image is
# one word in $IMAGES.
CHECKS='
set -eu
U=/bin/robot/updaterd
R=/bin/robot/robotctl
C=/tmp/duck/updater.toml
S=/tmp/duck/d.sock
LIVE=/tmp/duck/opt/daemon/current/version.toml

echo "    $(uname -m), $(ldd --version 2>&1 | head -1)"

# Everything below drives the binaries that ship, because the point of this script is the
# binaries that ship. Nothing here is a test double except the releases themselves.

# The fixture is mounted read-only; copy it so releases can be staged as the checks walk
# forward through versions.
cp -r /bin/fixture /tmp/duck

# `local_dir` serves the newest version it can see, so what the daemon is *offered* is
# decided by what has been copied in. Staging one release at a time is what makes each
# check below unambiguous about which version it is applying.
stage() { cp /tmp/duck/r/"$1"/* /tmp/duck/published/; }

# A fresh daemon per fault configuration, so every assertion has exactly one reason it can
# fail. Cheap: startup is a config parse, a recovery pass and a bind.
start_daemon() {
    RUST_LOG=info $U --config $C --socket $S "$@" >>/tmp/d.log 2>&1 &
    DAEMON=$!
    i=0
    while [ ! -S $S ] && [ $i -lt 100 ]; do i=$((i+1)); sleep 0.1; done
    test -S $S
}

# `wait` as well as `kill`: the next step may run `updaterd` again, and two of them on one
# state directory contend for the update lock.
stop_daemon() {
    kill $DAEMON 2>/dev/null || true
    wait $DAEMON 2>/dev/null || true
}

# ── the first install, which is the path scripts/install.sh takes on a bare board ──
stage 1.0.0
RUST_LOG=info $U --config $C install --from /tmp/duck/published >/tmp/install.log 2>&1
grep -q "version=1.0.0" $LIVE
echo "    [ok] installed 1.0.0"

# ── an unhealthy release must revert, content and all ──
#
# `fail_health` rather than a robot that answers "unhealthy": there is no robotd here, and a
# socket probe with nothing to ask fails the gate on its timeout instead — which would roll
# every release back and prove nothing about the gate.
start_daemon --inject-fault fail_health

# Group-restricted, not world-writable: anyone who can write here can update firmware.
test "$(stat -c %A $S)" = "srw-rw----"
echo "    [ok] socket is srw-rw---- (0660)"

stage 1.1.0
# Progress phases go to stderr, into the daemon log rather than the terminal: this script
# prints one line per property checked, and 13 lines of `Downloading`/`Swapping` per apply
# buries them. Captured, not discarded, so a failure still has something to read.
$R --socket $S update apply daemon 2>>/tmp/d.log | grep -q rolled_back
grep -q "version=1.0.0" $LIVE
echo "    [ok] unhealthy release rolled back"

# A tampered artifact must be refused before the swap, with its own exit code so a script
# can tell "correctly rejected" from "something broke". `|| code=$?` keeps set -e from
# treating the expected failure as fatal.
stage 1.2.0
code=0
$R --socket $S update apply daemon >/dev/null 2>&1 || code=$?
test "$code" -eq 5 || { echo "    [FAIL] expected REFUSED (5), got $code"; exit 1; }
grep -q "version=1.0.0" $LIVE
echo "    [ok] tampered artifact refused (exit 5)"

# SO_PEERCRED is enforced, and the audit line is what support reads.
grep -q "mutating request" /tmp/d.log
echo "    [ok] peer credentials recorded"

# Piping must not panic: Rust ignores SIGPIPE, so `| head` used to abort.
$R --socket $S update log | head -1 >/dev/null
echo "    [ok] output survives a closed pipe"

# An unreachable daemon must be its own exit code too, so "not running" and "rejected" stay
# distinguishable.
code=0
$R --socket /tmp/nope.sock update status >/dev/null 2>&1 || code=$?
test "$code" -eq 3 || { echo "    [FAIL] expected exit 3, got $code"; exit 1; }
echo "    [ok] unreachable daemon exits 3"
stop_daemon

# ── power loss straight after the swap: the boot counter, not the gate, is what reverts ──
start_daemon --inject-fault abort_after_swap
stage 2.0.0
$R --socket $S update apply daemon >/dev/null 2>&1 || true
# The swap already happened, which is the situation the boot counter exists for.
grep -q "version=2.0.0" $LIVE
stop_daemon

# `--check-only` is what a boot does, minus serving. Two of them: the first records the
# trial, the second exhausts it (MAX_BOOT_ATTEMPTS = 2) and reverts.
RUST_LOG=info $U --config $C --check-only >/tmp/boot1.log 2>&1
grep -q "update still on trial" /tmp/boot1.log
RUST_LOG=info $U --config $C --check-only >/tmp/boot2.log 2>&1
grep -q "never confirmed healthy; reverting" /tmp/boot2.log
grep -q "version=1.0.0" $LIVE
echo "    [ok] crash after swap reverted by the boot counter"

# ── and the ordinary case, with nothing injected at all ──
start_daemon
stage 3.0.0
$R --socket $S update apply daemon 2>>/tmp/d.log | grep -q "\"outcome\": \"applied\""
$R --socket $S update status | grep -q "3.0.0"
grep -q "version=3.0.0" $LIVE
echo "    [ok] robotctl applied 3.0.0 over the socket"
stop_daemon

# ── layered access control, as the systemd unit configures it ──
# Only meaningful on Linux, so this is the only place it can be tested.
groupadd -r robot 2>/dev/null || true
useradd -r -G robot member 2>/dev/null || true
useradd -r outsider 2>/dev/null || true

# `Group=robot` in the unit is what makes mode 0660 mean "the robot group" rather than
# "root only" — the socket inherits the process primary group.
setpriv --regid robot --clear-groups \
    /bin/robot/updaterd --config /tmp/duck/updater.toml --socket /run/u.sock \
    >/tmp/d2.log 2>&1 &
DAEMON2=$!
i=0
while [ ! -S /run/u.sock ] && [ $i -lt 100 ]; do i=$((i+1)); sleep 0.1; done
test "$(stat -c %U:%G /run/u.sock)" = "root:robot"
echo "    [ok] socket is root:robot when the unit sets Group=robot"

# Layer 1: the group decides who may talk to the daemon at all.
su member -s /bin/sh -c "/bin/robot/robotctl --socket /run/u.sock update status" >/dev/null
echo "    [ok] group member can read"

code=0
su outsider -s /bin/sh -c "/bin/robot/robotctl --socket /run/u.sock update status" \
    >/dev/null 2>&1 || code=$?
test "$code" -eq 3 || { echo "    [FAIL] non-member should be blocked, got $code"; exit 1; }
echo "    [ok] non-member blocked by socket mode"

# Layer 2: talking is not the same as being allowed to change the robot.
code=0
su member -s /bin/sh -c "/bin/robot/robotctl --socket /run/u.sock update apply daemon" \
    >/dev/null 2>&1 || code=$?
test "$code" -eq 6 || { echo "    [FAIL] member should be denied (6), got $code"; exit 1; }
echo "    [ok] group member cannot mutate (exit 6, denied)"

kill $DAEMON2 2>/dev/null || true

# ── setup-board.sh, the provisioning script nothing else covered ──
#
# Added because two green-CI regressions landed here in one day, including #13 deleting the
# console fix outright: deleting a working feature breaks no test. The scripts that provision
# hardware were the least-covered surface in the repo, and the only one that touches a robot.
#
# Behavioural rather than a grep. `systemctl` is stubbed to record its arguments, so this
# asserts what the script *does*. A grep would have caught the deletion but not a masking
# call naming the wrong unit.
mkdir -p /stub /boot /usr/local/lib

# check_environment only probes with `command -v`, and the ONNX step is skipped below, so
# stubs for tools this image lacks are enough and cost no download.
cat > /stub/curl <<"STUB"
#!/bin/sh
exit 0
STUB
cp /stub/curl /stub/find
cat > /stub/systemctl <<"STUB"
#!/bin/sh
echo "$@" >> /stub/systemctl.log
exit 0
STUB
chmod +x /stub/curl /stub/find /stub/systemctl

# Already present at the version asked for, so install_onnxruntime returns early instead of
# fetching ~20 MB. Both halves matter: the check resolves the symlink and reads the version out
# of the target name, so a bare file would be treated as a mismatch and trigger a download.
#
# ONNX_VERSION is passed explicitly rather than mirroring the script default, so bumping that
# default — which happens whenever ort moves — cannot silently break this test.
touch /usr/local/lib/libonnxruntime.so.9.9.9
ln -sf libonnxruntime.so.9.9.9 /usr/local/lib/libonnxruntime.so

# A BlueZ config with [General] but no Privacy key — the insert-after-[General] branch, which
# is the one a stock Armbian image takes.
mkdir -p /etc/bluetooth
cat > /etc/bluetooth/main.conf <<"BTCONF"
[General]
Name = radxa
BTCONF

# The wrong overlay prefix and a console on the motor UART: what Armbian actually ships.
cat > /boot/armbianEnv.txt <<"ENV"
overlay_prefix=rk35xx
console=both
ENV

ONNX_VERSION=9.9.9 PATH="/stub:$PATH" sh /bin/scripts/setup-board.sh >/tmp/board.log 2>&1

# The RK3566 shares overlays with the RK3568, so the wrong prefix boots happily with no
# /dev/ttyS2 at all.
grep -q "^overlay_prefix=rk3568$" /boot/armbianEnv.txt
grep -E "^overlays=" /boot/armbianEnv.txt | grep -qw uart2-m0
echo "    [ok] setup-board fixes overlay_prefix and enables uart2-m0"

# A getty *reads* the port, consuming servo replies, so every motor looks absent —
# indistinguishable from unwired hardware and far harder to guess.
grep -q "mask serial-getty@ttyS2.service" /stub/systemctl.log
echo "    [ok] setup-board masks the getty on the motor port"

# console=both puts printk on the same wires as the servos, corrupting replies
# intermittently rather than cleanly.
grep -q "^console=display$" /boot/armbianEnv.txt
echo "    [ok] setup-board takes the kernel console off the UART"

# Idempotent: it is re-run after the reboot it asks for, and must not undo its own work or
# append a second copy of the overlay.
ONNX_VERSION=9.9.9 PATH="/stub:$PATH" sh /bin/scripts/setup-board.sh >/tmp/board2.log 2>&1
grep -q "^overlay_prefix=rk3568$" /boot/armbianEnv.txt
grep -q "^console=display$" /boot/armbianEnv.txt
test "$(grep -c uart2-m0 /boot/armbianEnv.txt)" = 1
echo "    [ok] setup-board is idempotent on a second run"

# The gamepad settings, which are the kind that fail silently: a pad that pairs and drops, or
# a padd that reads nothing because the user is not in `input`.
grep -qE "^Privacy = device$" /etc/bluetooth/main.conf
echo "    [ok] setup-board sets Privacy = device for gamepad pairing"

# Idempotent too: the second run above must not have added a duplicate key, which BlueZ
# would read as a conflicting setting.
test "$(grep -cE "^[[:space:]]*Privacy[[:space:]]*=" /etc/bluetooth/main.conf)" = 1
echo "    [ok] Privacy is set exactly once"

# ── the generated preinstall hook ──
#
# The hook that asserts a board can run the release being installed. Exercised here because
# the alternative is discovering on a robot that it rejects every update, and because the
# whole point of moving this check into a hook was to stop relying on someone remembering to
# re-run a script.
#
# Rendered from the template the way xtask does, so this covers the shipped shape rather than
# a hand-written approximation.
sed -e "s/@ONNX_FLOOR@/1.23/" -e "s/@ONNX_TARGET@/1.28.0/" \
    /bin/hooks/preinstall.in > /tmp/preinstall
chmod +x /tmp/preinstall
if grep -q "@ONNX_" /tmp/preinstall; then
    echo "    [FAIL] placeholders left in the rendered hook"
    exit 1
fi

# Satisfied: a runtime at or above the floor must pass, touching nothing.
rm -f /usr/local/lib/libonnxruntime.so*
touch /usr/local/lib/libonnxruntime.so.1.28.0
ln -sf libonnxruntime.so.1.28.0 /usr/local/lib/libonnxruntime.so
PATH="/stub:$PATH" /tmp/preinstall > /tmp/hook1.log 2>&1
grep -q "satisfies" /tmp/hook1.log
echo "    [ok] preinstall accepts a runtime at the floor"

# Too old, and unfixable: curl fails, so the hook must exit non-zero *before* the swap rather
# than let a release install that cannot load a policy. This is the case that used to reach a
# board and panic robotd control thread.
mkdir -p /stubfail
cat > /stubfail/curl <<"STUB"
#!/bin/sh
exit 22
STUB
chmod +x /stubfail/curl
rm -f /usr/local/lib/libonnxruntime.so*
touch /usr/local/lib/libonnxruntime.so.1.20.1
ln -sf libonnxruntime.so.1.20.1 /usr/local/lib/libonnxruntime.so
code=0
PATH="/stubfail:$PATH" /tmp/preinstall > /tmp/hook2.log 2>&1 || code=$?
test "$code" -ne 0 || { echo "    [FAIL] hook passed an unusable runtime"; exit 1; }
grep -q "1.20.1 is below 1.23" /tmp/hook2.log
grep -q "cannot download ONNX Runtime" /tmp/hook2.log
echo "    [ok] preinstall refuses an old runtime it cannot replace, naming the fix"
'

for image in $IMAGES; do
    echo
    echo "==> $image"
    docker run --rm --platform linux/arm64 \
        -v "$PWD/$TARGET_DIR:/bin/robot:ro" \
        -v "$PWD/$FIXTURE:/bin/fixture:ro" \
        -v "$PWD/scripts:/bin/scripts:ro" \
        -v "$PWD/hooks:/bin/hooks:ro" \
        "$image" sh -c "$CHECKS"
done

echo
echo "==> all board checks passed on: $IMAGES"
