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

# The installer checks work from a real artifact rather than a fixture release: staged binaries,
# the packaged artifact, and the tree unpacked out of it.
INSTALL_STAGED=target/board-install/staged
INSTALL_DIST=target/board-install/dist
INSTALL_RELEASE=target/board-install/release

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

# A real artifact, for the installer checks at the end.
#
# `xtask package` from the binaries just cross-compiled, with the staging and `--include` lists
# read out of the packaging workflow — so this is the artifact production builds, not an
# approximation of one. The fixture releases above deliberately carry no systemd units, and units
# are the whole subject of those checks.
#
# Unpacked here rather than in the container because the zstd *binary* is the one tool the
# target image lacks, and an apt-get in the middle of a hermetic check buys a network
# dependency to save nothing.
echo "==> packaging a release for the installer checks"
rm -rf "$INSTALL_STAGED" "$INSTALL_DIST" "$INSTALL_RELEASE"
mkdir -p "$INSTALL_STAGED" "$INSTALL_RELEASE"

# Both lists parsed from the workflow, for the reason xtask/tests/artifact.rs exists: a copy
# kept here would be a third hand-maintained list to drift from the other two.
#
# `_build-release.yml`, because that is where the recipe lives: `release.yml` decides which channel
# is being published and calls it, and holds no `cp` or `--include` line of its own. Parsing the
# entry point instead produced an empty list and a failure two hundred lines later —
# "the installed release has no systemd/updaterd.service" — so the emptiness is now checked here,
# where it can name its own cause.
PACKAGING_WORKFLOW=.github/workflows/_build-release.yml

staged_binaries="$(grep -o -- 'release/[a-z]* staged/' "$PACKAGING_WORKFLOW" \
    | sed 's|release/||; s| staged/||' | sort -u)"
if [ -z "$staged_binaries" ]; then
    echo "error: no staged binaries found in ${PACKAGING_WORKFLOW}." >&2
    echo "       The parse has stopped matching its 'cp ... staged/' lines, so the artifact" >&2
    echo "       would carry no binaries and the installer checks would fail 200 lines later" >&2
    echo "       with 'the installed release has no systemd/updaterd.service'." >&2
    exit 1
fi

for binary in $staged_binaries; do
    cp "$TARGET_DIR/$binary" "$INSTALL_STAGED/"
done

# `set --` at the top level is safe: this script takes no arguments. It is how POSIX sh builds
# an argument list without an array, and quoting each pair matters — a bare expansion splits
# `docs/deploy.md=docs/deploy.md` on nothing and a path with a space on everything.
set --
while IFS= read -r pair; do
    set -- "$@" --include "$pair"
done <<INCLUDES
$(grep -o -- '--include "[^"]*"' "$PACKAGING_WORKFLOW" | sed 's/--include //; s/"//g')
INCLUDES

if [ "$#" -eq 0 ]; then
    echo "error: no --include pairs found in ${PACKAGING_WORKFLOW}." >&2
    echo "       The artifact would carry no units, and the installer checks below are" >&2
    echo "       entirely about units." >&2
    exit 1
fi

# Level 1: this artifact is unpacked once and thrown away. The shipping default of 19 is
# single-threaded and would cost more than every check below put together.
cargo run -q -p xtask -- package \
    --version "$(grep -m1 '^version' Cargo.toml | sed 's/.*"\(.*\)".*/\1/')" \
    --bin-dir "$INSTALL_STAGED" \
    --out "$INSTALL_DIST" \
    --zstd-level 1 \
    "$@" | sed 's/^/    /'

zstd -dc "$INSTALL_DIST"/daemon-*.tar.zst | tar -x -C "$INSTALL_RELEASE"

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

# ── configd: the same two layers, plus the wifi outcomes ──
#
# configd is the service BLE provisioning goes through, so its authorisation is the thing
# standing between "a phone in the room" and the network configuration of this robot. The deny
# path needs two uids and a real unix socket, so this is the only place it can be tested at
# all — the unit tests can only reach the allow path.
#
# `--fake-net` serves an in-memory wifi stack: there is no NetworkManager in a container, and
# the point here is the socket, the authorisation and the outcome codes rather than D-Bus.
mkdir -p /var/lib/robot/config
setpriv --regid robot --clear-groups \
    /bin/robot/configd --fake-net --fake-pads --socket /run/c.sock \
    --state-dir /var/lib/robot/config --allow-user member \
    >/tmp/c.log 2>&1 &
CONFIGD=$!
i=0
while [ ! -S /run/c.sock ] && [ $i -lt 100 ]; do i=$((i+1)); sleep 0.1; done
test -S /run/c.sock

test "$(stat -c %A /run/c.sock)" = "srw-rw----"
test "$(stat -c %U:%G /run/c.sock)" = "root:robot"
echo "    [ok] configd socket is root:robot 0660"

# The name is resolved to a uid at startup, because sysusers allocates dynamically and a
# number in a unit file would be wrong on the next board.
grep -q "may change configuration" /tmp/c.log
echo "    [ok] --allow-user resolved to a uid"

# Layer 1: the group decides who may talk at all.
su member -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock net status" >/dev/null
echo "    [ok] group member can read net status"

code=0
su outsider -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock net status" \
    >/dev/null 2>&1 || code=$?
test "$code" -eq 3 || { echo "    [FAIL] non-member should be blocked, got $code"; exit 1; }
echo "    [ok] non-member blocked by socket mode"

# Layer 2, the allow side: the named user may change configuration. This is the property
# `--allow-user btd` relies on, and without it every provisioning call over BLE is refused.
su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock system set-name Ducky" >/dev/null
echo "    [ok] named user may change configuration"

# Layer 2, the deny side. It needs a peer that CAN reach the socket but is NOT the named user,
# so a second member of the robot group: reaching the daemon and being allowed to change it are
# different permissions, and this is the check that proves they are still separate.
useradd -r -G robot bystander 2>/dev/null || true
code=0
su bystander -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock system set-name Sneaky" \
    >/dev/null 2>&1 || code=$?
test "$code" -eq 6 || { echo "    [FAIL] unnamed member should be denied (6), got $code"; exit 1; }
echo "    [ok] unnamed group member cannot change configuration (exit 6)"

# And nothing changed: a refused call must not have taken effect.
su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock system info" | grep -q "Ducky"
echo "    [ok] the refused rename did not take effect"

# The outcome that made NetworkManager worth choosing: a rejected passphrase is its own
# answer, distinguishable by exit code from "could not ask".
code=0
su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock net connect Pollen --psk wrong" \
    >/dev/null 2>&1 || code=$?
test "$code" -eq 5 || { echo "    [FAIL] bad key should be refused (5), got $code"; exit 1; }
echo "    [ok] a rejected passphrase exits 5, not 1"

su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock net connect Pollen --psk correct-key" \
    | grep -q "connected to Pollen"
echo "    [ok] a correct passphrase joins"

# A PIN keeps its leading zero. It is stored as a string precisely so that "012345" does not
# become 12345, and the two sides of a pairing must agree on six characters.
su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock system set-pin 012345" >/dev/null
su member -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock system pin" \
    | grep -q "012345"
echo "    [ok] a pairing PIN keeps its leading zero"

# ── the gamepad, through the same socket ──
#
# `pad.*` is the surface that replaced pairing a controller by hand with bluetoothctl, and it is
# mutating: a bonded pad can enable the walking policy afterwards. So the authorisation matters as
# much as the outcome, and it is checked the same way — a named user may, a group member may not.
#
# `--fake-pads` serves an in-memory set: there is no bluetoothd in a container, and what is under
# test here is the socket, the gating and the answers rather than BlueZ.
su member -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock pad status" \
    | grep -q "none paired"
echo "    [ok] pad status reports no pad on a fresh robot"

code=0
su bystander -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock pad pair" \
    >/dev/null 2>&1 || code=$?
test "$code" -eq 6 || { echo "    [FAIL] pairing a pad should be denied (6), got $code"; exit 1; }
echo "    [ok] an unnamed group member cannot pair a pad (exit 6)"

su member -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock pad pair" \
    | grep -q "paired  Xbox Wireless Controller"
echo "    [ok] a named user pairs the pad that is in pairing mode"

# Trusted, not merely paired. An untrusted pad looks right and does not reconnect after a reboot,
# because approving a reconnection needs an agent and at boot there is none.
su member -s /bin/sh -c "/bin/robot/robotctl --config-socket /run/c.sock pad status --json" \
    | grep -q "\"trusted\":true"
echo "    [ok] the paired pad is trusted, so it reconnects by itself"

su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock pad forget 78:86:2E:BB:13:28" \
    | grep -q "forgot"
echo "    [ok] a pad can be forgotten"

# Forgetting again is not an error — the same contract as `net forget`, and a client must not
# present it as a failure.
su member -s /bin/sh -c \
    "/bin/robot/robotctl --config-socket /run/c.sock pad forget 78:86:2E:BB:13:28" \
    | grep -q "was not paired"
echo "    [ok] forgetting an unknown pad is not an error"

# The passphrase must not be in the journal. NetConnectParams has a hand-written Debug that
# redacts it, and this is the check that keeps it honest on the shipped binary rather than in
# a unit test of the type alone.
if grep -q "correct-key" /tmp/c.log; then
    echo "    [FAIL] a wifi passphrase reached the log"
    exit 1
fi
echo "    [ok] no wifi passphrase in the log"

kill $CONFIGD 2>/dev/null || true

# ── btd: does the vendored libdbus actually link and run on the target? ──
#
# btd is the one binary pulling C code beyond zstd: `bluer` links libdbus, built from vendored
# source by zig cc. That cross-link is exactly the class of failure this whole script exists
# for, and it cannot be seen on the build host. There is no BlueZ in a container, so this asks
# only that the binary loads and runs — which is what a dynamic-linker or glibc-floor problem
# would break.
/bin/robot/btd --version >/dev/null
echo "    [ok] btd runs on the target (vendored libdbus links)"

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

# The gamepad setting, which is the kind that fails silently — and which this script had
# BACKWARDS until a board proved it. `Privacy = device` makes an Xbox controller reject LE Secure
# Connections pairing with `DHKey check failed (0x0b)`, because that check covers both addresses
# and privacy pairs from a resolvable private one. See `configure_bluetooth`.
#
# It is the only part of gamepad readiness left in this script: reading the pad belongs to
# padd.service and its `input` group now, and pairing one is `robotctl pad pair`.
#
# No apostrophes and no single quotes anywhere in this string. It is passed to the container
# single-quoted, so one would end it early and run the rest on the host — which is exactly what
# happened once, and it presented as a grep failing on a file the fixture had just written.
grep -qE "^Privacy = off$" /etc/bluetooth/main.conf
echo "    [ok] setup-board sets Privacy = off, which is what lets a pad bond"

# Idempotent too: the second run above must not have added a duplicate key, which BlueZ
# would read as a conflicting setting.
test "$(grep -cE "^[[:space:]]*Privacy[[:space:]]*=" /etc/bluetooth/main.conf)" = 1
echo "    [ok] Privacy is set exactly once"

# The upgrade case, which is every board provisioned so far: the wrong value is already in the
# file and has to be corrected rather than left alone. An absent setting and a wrong one need
# different work, and only the first was ever tested.
sed -i "s|^Privacy = off|Privacy = device|" /etc/bluetooth/main.conf
ONNX_VERSION=9.9.9 PATH="/stub:$PATH" sh /bin/scripts/setup-board.sh >/tmp/board3.log 2>&1
grep -qE "^Privacy = off$" /etc/bluetooth/main.conf
test "$(grep -cE "^[[:space:]]*Privacy[[:space:]]*=" /etc/bluetooth/main.conf)" = 1
echo "    [ok] a board carrying Privacy = device is corrected to off"

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

# ── installing a real artifact: scripts/install.sh and hooks/postinstall ──
#
# The gap docs/project/install-path-gap.md option B names. Everything above drives the engine, which
# lands a release; nothing ran the 892 lines that turn a landed release into a working board,
# and nothing ran the hook that does the same job unattended inside the update gate. Four bugs
# reached a board through that, and the two file-placing scripts were the only part of the
# install path with no coverage at all.
#
# Behavioural, like the setup-board checks above and for the same reason: systemctl is stubbed
# to record its arguments, so this asserts what the scripts *do*. A grep over them would pass
# on a script that installed a unit into the wrong directory.
#
# The release is a real artifact, built by xtask package from the cross-compiled binaries and
# the include list read out of the packaging workflow. Not the fixture releases the engine uses:
# those carry no systemd units by design, and units are the whole subject here.

mkdir -p /stub

# install.sh reaches the network twice before it touches anything local — the releases API, for
# the bootstrap binary and the tag whose config it should pair with, and raw.githubusercontent
# for the trusted keys. Both are answered out of the checkout.
cat > /stub/curl <<"STUB"
#!/bin/sh
# Enough of curl for the two callers in install.sh: -fsSL [-H hdr] -o <dest> <url>.
dest=""
url=""
while [ $# -gt 0 ]; do
    case "$1" in
        -o) dest="$2"; shift 2 ;;
        -H) shift 2 ;;
        -*) shift ;;
        *)  url="$1"; shift ;;
    esac
done
case "$url" in
    */releases/latest)
        cp /stub/releases-latest.json "$dest" ;;
    *raw.githubusercontent.com/*)
        # Serve the repository copy of whatever was asked for. A path the checkout does not
        # have must *fail* rather than produce an empty file: install.sh treats a missing spare
        # key as a warning and a missing release-1 as fatal, and an empty file would look like
        # a successful fetch of an unusable key.
        path="${url#*raw.githubusercontent.com/}"
        path="${path#*/}"
        path="${path#*/}"
        path="${path#*/}"
        [ -f "/bin/$path" ] || exit 22
        cp "/bin/$path" "$dest" ;;
    *)
        echo "stub curl: unexpected url $url" >&2
        exit 22 ;;
esac
STUB

# `id` and `name` must survive install.sh compacting this with `tr -d " \n" | tr "{" "\n"` and
# then grepping one line for both. Hence the asset object opening its own brace with both
# fields ahead of the nested uploader — the comment on resolve_bootstrap_asset records that
# getting this shape wrong is how it failed against a real board.
cat > /stub/releases-latest.json <<"JSON"
{
  "tag_name": "v9.9.9",
  "assets": [
    { "id": 4242, "name": "updaterd-bootstrap-aarch64", "uploader": { "login": "ci" } }
  ]
}
JSON

cat > /stub/systemctl <<"STUB"
#!/bin/sh
echo "$@" >> /stub/systemctl.log
exit 0
STUB
chmod +x /stub/curl /stub/systemctl

# The release, laid out exactly as the engine leaves one: a versioned directory under
# releases/ with current pointing at it. By hand rather than through `updaterd install`,
# because what is under test is the two things that run *after* a release is live, and the
# engine install path is covered at the top of this file.
REL=/opt/robot/daemon/releases/under-test
mkdir -p "$REL"
cp -a /bin/release/. "$REL"/
ln -sfn releases/under-test /opt/robot/daemon/current

# The operator files, pre-placed. install.sh never overwrites these — the behaviour that let a
# board keep a stale on_apply list for months — so this takes the branch a re-install takes,
# and substitutes the repository the way the fetch path would.
mkdir -p /etc/robot
sed "s|\"ORG/duck-daemon\"|\"pollen-robotics/microduck_daemon\"|" \
    /bin/deploy/updater.toml > /etc/robot/updater.toml
cp /bin/deploy/robotd.toml /etc/robot/robotd.toml

# ── install.sh, the provisioning path ──
#
# The bootstrap download self-skips because a release is already live, which is the branch a
# re-install takes and the reason no signing or network is needed here.
PATH="/stub:$PATH" sh /bin/scripts/install.sh > /tmp/install.log 2>&1 || {
    echo "    [FAIL] install.sh exited non-zero"
    cat /tmp/install.log
    exit 1
}
echo "    [ok] install.sh runs to completion on a board with a release already live"

# Every unit the release ships, installed where systemd reads them. install.sh globs the
# release directory rather than naming units, so this walks the same set rather than a list
# that would need editing whenever a daemon is added.
for src in "$REL"/systemd/*.service; do
    name="$(basename "$src")"
    test -f "/etc/systemd/system/${name}" \
        || { echo "    [FAIL] ${name} was not installed"; exit 1; }
    # Byte-identical: the unit belongs to the release, and an installer that wrote its own
    # copy or an older one would still leave a plausible-looking file here.
    cmp -s "$src" "/etc/systemd/system/${name}" \
        || { echo "    [FAIL] ${name} differs from the one the release ships"; exit 1; }
    test "$(stat -c %a "/etc/systemd/system/${name}")" = "644" \
        || { echo "    [FAIL] ${name} is not mode 644"; exit 1; }
done
echo "    [ok] every unit the release ships is installed, unmodified, mode 644"

# The binary each unit execs, in the artifact that shipped the unit.
#
# This is bug 3 of install-path-gap.md, and the only class of the four with no strong test: the
# packaging step stages binaries with an explicit `cp` per binary, `btd` and `configd` were built and
# never staged, and `btd.service` failed with `203/EXEC` — which reads as a broken daemon rather than
# an incomplete artifact. `xtask/tests/artifact.rs` covers it by comparing two *source* files, which
# is the weaker form the doc criticises; here the tarball is already unpacked and `current` already
# points at it, so asking is three lines.
#
# Only `ExecStart` paths inside the release are checked. A unit may deliberately exec out of the base
# — the boot recovery net does, precisely so a broken release cannot break it — and demanding those
# be in the artifact would be wrong rather than strict.
for src in "$REL"/systemd/*.service; do
    name="$(basename "$src")"
    exec_line="$(grep -m1 "^ExecStart=" "$src")"
    exec_path="${exec_line#ExecStart=}"
    exec_path="${exec_path%% *}"
    case "$exec_path" in
        /opt/robot/daemon/current/*) ;;
        *) continue ;;
    esac
    test -x "$exec_path" \
        || { echo "    [FAIL] ${name} execs ${exec_path}, which the artifact does not contain"; exit 1; }
done
echo "    [ok] the ExecStart binary of every unit is in the release that shipped it"

# Accounts before units, or a unit naming a User= that does not exist fails to start and the
# failure reads as a broken daemon.
test -f /usr/lib/sysusers.d/robot.conf
test -f /usr/lib/sysusers.d/btd.conf
getent group robot >/dev/null \
    || { echo "    [FAIL] the robot group was not created"; exit 1; }
getent passwd btd >/dev/null \
    || { echo "    [FAIL] the btd user was not created"; exit 1; }
echo "    [ok] robot group and btd user exist, sysusers drop-ins installed"

# Through `current`, not at a versioned directory: the symlink has to follow the active
# release, or robotctl on PATH silently pins to whichever release installed it.
link="$(readlink /usr/local/bin/robotctl)"
case "$link" in
    */current/bin/robotctl) ;;
    *) echo "    [FAIL] robotctl points at ${link}, not through current"; exit 1 ;;
esac
test -e /usr/local/bin/robotctl \
    || { echo "    [FAIL] the robotctl symlink does not resolve"; exit 1; }
echo "    [ok] robotctl on PATH resolves through current"

test -f /etc/systemd/journald.conf.d/10-robot.conf \
    || { echo "    [FAIL] no journald drop-in, so logs will not survive a reboot"; exit 1; }
grep -q "restart systemd-journald" /stub/systemctl.log
echo "    [ok] journald drop-in installed and journald restarted"

# daemon-reload before anything is enabled, or systemd enables the unit it had cached.
grep -q "^daemon-reload$" /stub/systemctl.log
for unit in updaterd robotd configd btd; do
    grep -q "^enable --now ${unit}.service$" /stub/systemctl.log \
        || { echo "    [FAIL] ${unit}.service was never enabled"; exit 1; }
done
reload_at="$(grep -n "^daemon-reload$" /stub/systemctl.log | head -1 | cut -d: -f1)"
first_enable="$(grep -n "^enable --now" /stub/systemctl.log | head -1 | cut -d: -f1)"
test "$reload_at" -lt "$first_enable" \
    || { echo "    [FAIL] a unit was enabled before daemon-reload"; exit 1; }
# configd before btd, because btd asks configd for the pairing PIN and a btd that starts first
# refuses to pair until configd answers.
configd_at="$(grep -n "^enable --now configd.service$" /stub/systemctl.log | cut -d: -f1)"
btd_at="$(grep -n "^enable --now btd.service$" /stub/systemctl.log | cut -d: -f1)"
test "$configd_at" -lt "$btd_at" \
    || { echo "    [FAIL] btd was enabled before configd"; exit 1; }
echo "    [ok] daemon-reload precedes every enable, and configd precedes btd"

# The boot recovery net, whose whole job is to be armed for the *next* boot.
#
# `enable`, never `enable --now`: an `OnBootSec=` timer started past its deadline fires at once, and
# at once here is the middle of provisioning — daemons still being started by the lines above, on a
# board that has no golden release yet anyway.
grep -q "^enable robot-boot-check.timer$" /stub/systemctl.log \
    || { echo "    [FAIL] the boot recovery timer was never enabled"; exit 1; }
grep -q "^enable --now robot-boot-check" /stub/systemctl.log \
    && { echo "    [FAIL] the boot recovery check was started during provisioning"; exit 1; }
for script in robot-rescue robot-boot-check; do
    test -x "/usr/local/sbin/${script}" \
        || { echo "    [FAIL] ${script} is not installed, so a broken release has no way back"; exit 1; }
done
echo "    [ok] the recovery net is installed and armed for the next boot, not this one"

grep -q "keeping the existing /etc/robot/updater.toml" /tmp/install.log
grep -q "keeping the existing /etc/robot/robotd.toml" /tmp/install.log
echo "    [ok] the operator config files are preserved, not overwritten"

# ── a second run must change nothing ──
#
# Provisioning gets re-run: a flaky download, a fresh key, an operator repeating a step. A
# second pass that reported success while, say, appending to a config would be found on a
# board and not here.
find /etc/systemd/system /usr/lib/sysusers.d /etc/robot -type f | sort > /tmp/state1
md5sum /etc/robot/updater.toml > /tmp/conf1
PATH="/stub:$PATH" sh /bin/scripts/install.sh > /tmp/install2.log 2>&1 || {
    echo "    [FAIL] the second install.sh exited non-zero"
    cat /tmp/install2.log
    exit 1
}
find /etc/systemd/system /usr/lib/sysusers.d /etc/robot -type f | sort > /tmp/state2
cmp -s /tmp/state1 /tmp/state2 \
    || { echo "    [FAIL] a second run changed which files exist"; diff /tmp/state1 /tmp/state2; exit 1; }
md5sum -c /tmp/conf1 >/dev/null \
    || { echo "    [FAIL] a second run modified updater.toml"; exit 1; }
echo "    [ok] install.sh is idempotent"

# ── a unit the script does not know must be installed and reported ──
#
# The release is the authority on what it contains, so an unrecognised unit is installed
# anyway; but install.sh cannot know where it belongs in the start order, so it says so rather
# than starting it blindly. That warning is how adding a daemon stays one line here instead of
# a silent omission, and a refactor could drop it without any other test noticing.
cp "$REL"/systemd/robotd.service "$REL"/systemd/sentinel.service
PATH="/stub:$PATH" sh /bin/scripts/install.sh > /tmp/install3.log 2>&1
test -f /etc/systemd/system/sentinel.service \
    || { echo "    [FAIL] an unrecognised unit was not installed"; exit 1; }
grep -q "sentinel.service was installed but not enabled" /tmp/install3.log \
    || { echo "    [FAIL] an unrecognised unit was installed with no warning"; exit 1; }
echo "    [ok] a unit install.sh does not recognise is installed and reported, not started"

# ── hooks/postinstall, alone ──
#
# The other half, and the one that matters more: this runs unattended on every update, inside
# the gate, with nobody watching. Its job is that a release adding a daemon arrives working
# without anyone touching the board — so it is asserted on a box with no units at all, which is
# the state that would otherwise need install.sh to be re-run by hand.
#
# Hooks run with the release directory as the working directory; the hook exits early without
# one, so getting this wrong would make the whole check vacuous.
rm -f /etc/systemd/system/*.service /etc/systemd/system/*.timer /usr/lib/sysusers.d/*.conf \
    /stub/systemctl.log
# The journald drop-in and the robotctl symlink go too, so the asymmetry asserted at the end is
# a real absence rather than a leftover from the install above.
rm -f /etc/systemd/journald.conf.d/10-robot.conf /usr/local/bin/robotctl
( cd "$REL" && PATH="/stub:$PATH" sh "$REL"/hooks/postinstall > /tmp/hook.log 2>&1 ) || {
    echo "    [FAIL] hooks/postinstall exited non-zero, which fails an update"
    cat /tmp/hook.log
    exit 1
}
for src in "$REL"/systemd/*.service "$REL"/systemd/*.timer; do
    [ -f "$src" ] || continue
    name="$(basename "$src")"
    test -f "/etc/systemd/system/${name}" \
        || { echo "    [FAIL] postinstall did not install ${name}"; exit 1; }

    # Three outcomes, not one, and which one a unit gets is a property of the unit file:
    #
    #   - no `[Install]` section: installed and left alone. The oneshot in the boot recovery
    #     net is deliberately like this, because `enable --now` on it would run a rollback check in the
    #     middle of the update that installed it, with daemons legitimately mid-restart;
    #   - a timer: enabled, not started. An `OnBootSec=` timer started past its deadline fires at
    #     once, and this hook runs mid-update;
    #   - everything else: enabled and started, which is the whole point of the hook.
    if ! grep -q "^\[Install\]" "$src"; then
        grep -q " ${name}$" /stub/systemctl.log \
            && { echo "    [FAIL] postinstall touched ${name}, which has no [Install]"; exit 1; }
        continue
    fi
    case "$name" in
        *.timer)
            grep -q "^enable ${name}$" /stub/systemctl.log \
                || { echo "    [FAIL] postinstall did not enable ${name}"; exit 1; }
            grep -q "^enable --now ${name}$" /stub/systemctl.log \
                && { echo "    [FAIL] postinstall started ${name}; it arms at the next boot"; exit 1; }
            continue
            ;;
    esac
    grep -q "^enable --now ${name}$" /stub/systemctl.log \
        || { echo "    [FAIL] postinstall did not enable ${name}"; exit 1; }
done
test -f /usr/lib/sysusers.d/robot.conf
grep -q "^daemon-reload$" /stub/systemctl.log
echo "    [ok] postinstall alone installs every unit, and starts the ones meant to start now"

# The recovery scripts, which are what runs when a release cannot start — so they are copied into
# /usr/local/sbin rather than read through `current`, and an update has to refresh them.
for script in robot-rescue robot-boot-check; do
    test -x "/usr/local/sbin/${script}" \
        || { echo "    [FAIL] postinstall did not install ${script}"; exit 1; }
done
echo "    [ok] postinstall refreshes the recovery scripts in /usr/local/sbin"

# The asymmetry, pinned because it is easy to assume otherwise: the hook places units and
# accounts and nothing else. Both of these were deleted above and the hook did not restore
# either, so a release that added a journald drop-in or moved robotctl delivers neither on an
# update — only install.sh does those, and it runs once, by hand, at provisioning time.
#
# Asserted rather than merely documented because the reasonable next change to this hook is to
# make it place everything install.sh places, and the person making it should find a test that
# states the current contract instead of discovering it on a board.
test ! -f /etc/systemd/journald.conf.d/10-robot.conf \
    || { echo "    [FAIL] postinstall now installs the journald drop-in"; exit 1; }
test ! -e /usr/local/bin/robotctl \
    || { echo "    [FAIL] postinstall now creates the robotctl symlink"; exit 1; }
echo "    [ok] postinstall covers units and accounts only, as documented"
'

for image in $IMAGES; do
    echo
    echo "==> $image"
    docker run --rm --platform linux/arm64 \
        -v "$PWD/$TARGET_DIR:/bin/robot:ro" \
        -v "$PWD/$FIXTURE:/bin/fixture:ro" \
        -v "$PWD/scripts:/bin/scripts:ro" \
        -v "$PWD/hooks:/bin/hooks:ro" \
        -v "$PWD/deploy:/bin/deploy:ro" \
        -v "$PWD/$INSTALL_RELEASE:/bin/release:ro" \
        "$image" sh -c "$CHECKS"
done

echo
echo "==> all board checks passed on: $IMAGES"
