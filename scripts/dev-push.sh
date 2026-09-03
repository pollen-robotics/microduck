#!/bin/sh
# Build the daemon on this laptop and install it on a board, without CI.
#
# Usage:  scripts/dev-push.sh [--docker] [--dry-run] [--bootstrap] [user@host]
#         scripts/dev-push.sh --name duck-c51b       # find the board over Bluetooth
#         DUCK_ROBOT=duck-c51b scripts/dev-push.sh
#
# Requires the team dev secret key, and one of two build toolchains — `cargo-zigbuild` plus
# `zig`, or `--docker`. The board must be a dev board (`allow_dev_keys = true` and
# `team.dev.pub` in its trusted keys — `deploy/README.md`).
#
# **Two ways to build, same artifact.** The default cross-compiles here with `cargo zigbuild`:
# fastest, and what CI uses. `--docker` builds inside the board's own userland instead, where
# there is nothing to cross and libudev is an `apt-get install` — reach for it when the zig
# toolchain is not set up, when it breaks, or before you have a board to take libudev from.
#
# What this is for: the loop between "I changed a line" and "the robot is running it" was a
# push, a CI run and a `--ref` install. Everything CI does to make that artifact happens
# locally in well under a minute, so the only reason to involve CI is to publish something
# other people install.
#
# **It is an ordinary update.** The board applies this through `robotctl update apply`, so
# preflight, the signature, the artifact hash, compatibility, the health gate and auto-rollback
# all run exactly as they do for a release — a local build that does not come up is reverted and
# the board is back on what it was running. That is the reason `--from` exists as an option on
# `apply` rather than reusing `updaterd install --from`, which has to force the gate off and so
# refuses to touch a live release at all.
#
# **What it deliberately does not do** is anything a release does for provenance. The version
# carries a timestamp, not a tag; the artifact is signed with the dev key, which a customer
# robot refuses; nothing is published, so nobody else can install what you just ran. Cutting a
# release is still a tag and `release.yml`.
#
# The version is `<crate>-dev.local.<epoch>.g<sha7>`: a prerelease, so it sorts below the
# release it precedes and can never look like an upgrade for the fleet, and unique per *push*
# rather than per commit — the tree is expected to be dirty here, and two pushes of the same
# dirty tree must not collide into "already current".
set -eu

cd "$(dirname "$0")/.."

# Where the artifact lands on the board. Empty here and resolved after the board is known,
# because the default is a path on *that* machine — see the block below the argument parsing.
REMOTE_DIR="${DUCK_SIDELOAD_DIR:-}"

# The secret half of `team.dev`, the same key `dev.yml` signs branch builds with. Named apart
# from `DUCK_DEV_KEY`, which the provisioning scripts use for the *public* half.
KEY="${DUCK_DEV_SECRET_KEY:-$HOME/.duck-keys/team.dev.key}"

BOOTSTRAP=no
DRY_RUN=no
DOCKER=no
# Both empty here and filled from the environment below, after the arguments have had their say.
BOARD=""
ROBOT=""

while [ $# -gt 0 ]; do
    case "$1" in
        --bootstrap) BOOTSTRAP=yes ;;
        --dry-run) DRY_RUN=yes ;;
        --docker) DOCKER=yes ;;
        --name)
            shift
            [ $# -gt 0 ] || { echo "--name needs a robot name" >&2; exit 2; }
            ROBOT="$1"
            ;;
        -h|--help)
            sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) BOARD="$1" ;;
    esac
    shift
done

# ── which board, and why a name beats an address ───────────────────────────────────────
#
# An address moves. A reflash, a router reboot or a different network hands the board a new
# lease, and mDNS on this image is unreliable enough that a `.local` name is not the answer
# either (`provision-board.sh` says the same). So the address is the one thing about a board
# nobody can keep in a shell profile: `DUCK_BOARD=radxa@192.168.1.42` goes stale, and the way
# that reads is a push that hangs until ssh times out.
#
# A robot's *name* does not move. `duckctl` finds it over BLE by that name and `net.status`
# answers with the address it currently has — which makes the radio the way out of exactly the
# situation the network cannot help with, and needs nothing on the LAN to be known or guessed.
#
# Cached, and re-resolved only when ssh cannot reach the cached address, because BLE discovery
# costs ten to twenty seconds and this script exists to be quick. The steady state is one ssh
# probe; the reflashed-board case pays one scan and is quick again after it.
BOARD_USER="${DUCK_BOARD_USER:-radxa}"
CACHE_DIR="${DUCK_BOARD_CACHE:-$HOME/.cache/duck/boards}"

# The installed client if there is one, this clone's otherwise. `cargo install --path duckctl` is
# what puts it on a PATH, and plenty of clones have never run it.
#
# Not named `duckctl`: a function by that name would match its own `duckctl "$@"` below and
# recurse until the shell gives up.
client() {
    if command -v duckctl >/dev/null 2>&1; then
        duckctl "$@"
    else
        cargo run -q -p duckctl -- "$@"
    fi
}

# `net.status` for one robot, JSON on stdout.
#
# Only `--name` is passed. A robot with a PIN of its own needs `DUCK_PIN`, which `duckctl`
# reads for itself (`docs/robot/duckctl.md`) — repeating it here would be a second place to
# keep in step, and passing its factory default unconditionally would override the real one.
ble_status() {
    client --name "$1" wifi status
}

# A robot name in, `user@address` out. Everything else goes to stderr, so the substitution that
# calls this captures the address and nothing else.
resolve_board() {
    robot_name="$1"
    # One file per robot, named after it, so two boards do not fight over one cache. Anything a
    # filesystem would rather not see becomes `_`: a robot can be called `Ducky the Second`.
    cache="$CACHE_DIR/$(printf '%s' "$robot_name" | tr -c 'A-Za-z0-9._-' '_')"
    cached=""
    if [ -f "$cache" ]; then
        cached="$(cat "$cache")"
        # `BatchMode=yes` so an unreachable board fails instead of prompting for a password, and
        # a short timeout because this runs on the happy path of every push.
        if [ -n "$cached" ] && ssh -o ConnectTimeout=4 -o BatchMode=yes \
            "$BOARD_USER@$cached" true >/dev/null 2>&1; then
            echo "$BOARD_USER@$cached"
            return 0
        fi
    fi

    echo "==> asking $robot_name over Bluetooth where it is" >&2
    reply="$(ble_status "$robot_name")" || {
        echo "could not reach $robot_name over Bluetooth" >&2
        echo "  duckctl scan                                  # is it advertising?" >&2
        echo "  scripts/dev-push.sh $BOARD_USER@<address>        # or say where it is" >&2
        return 1
    }
    # An `error` reply is a robot that answered and refused — a wrong PIN, most often — which is a
    # different problem from a robot with no address, and says so rather than being reported as one.
    address="$(printf '%s' "$reply" | python3 -c 'import json, sys
r = json.load(sys.stdin)
e = r.get("error")
if e: sys.exit("the robot refused net.status: %s" % (e.get("message") if isinstance(e, dict) else e))
print((r.get("result") or {}).get("ip4") or "")')" || return 1

    if [ -z "$address" ]; then
        echo "$robot_name answered over Bluetooth but has no wifi address" >&2
        echo "Join it to a network first — over the same radio, so this needs no ssh:" >&2
        echo "  duckctl --name '$robot_name' wifi connect <ssid> --psk <passphrase>" >&2
        return 1
    fi

    mkdir -p "$CACHE_DIR"
    # The bare address, without the user: `DUCK_BOARD_USER` is this laptop's setting and may
    # change between pushes, and a cache holding it would answer with yesterday's.
    printf '%s\n' "$address" > "$cache"

    if [ "$address" = "$cached" ]; then
        # Worth saying rather than resolving silently: the address never moved, so whatever ssh
        # is unhappy about is not the address. A reflashed board is the usual one — new host keys,
        # same lease — and it looks identical to an unreachable board until someone says this.
        echo "    still $address, which ssh could not reach — so the address is not the problem" >&2
        echo "    a reflashed board has new host keys:" >&2
        echo "      ./scripts/provision-board.sh $BOARD_USER@$address --forget-host-key" >&2
    else
        echo "    $robot_name is at $address" >&2
    fi
    echo "$BOARD_USER@$address"
}

# The command line beats the environment, and an address beats a name: an address needs no radio.
# `DUCK_ROBOT` is the same variable `duckctl` defaults `--name` to, so one exported name serves
# both tools — and empty means unset in both, so `DUCK_ROBOT= scripts/dev-push.sh radxa@…` works.
# `--name` here is `duckctl`'s sense of it: which robot to talk to. `provision-board.sh --name`
# means the opposite way round — the name to *give* a board — because provisioning is the one place
# a name is assigned rather than used to find something.
if [ -z "$BOARD" ] && [ -z "$ROBOT" ]; then
    BOARD="${DUCK_BOARD:-}"
    [ -n "$BOARD" ] || ROBOT="${DUCK_ROBOT:-}"
fi

if [ -n "$BOARD" ] && [ -n "$ROBOT" ]; then
    echo "pass an address or --name, not both: they name the board two different ways" >&2
    exit 2
fi

if [ -n "$ROBOT" ]; then
    BOARD="$(resolve_board "$ROBOT")" || exit 1
fi

if [ -z "$BOARD" ]; then
    echo "no board: name one, or give its address" >&2
    echo "  scripts/dev-push.sh --name duck-c51b        # found over Bluetooth" >&2
    echo "  scripts/dev-push.sh radxa@192.168.1.42" >&2
    echo "or set DUCK_ROBOT or DUCK_BOARD once per shell" >&2
    exit 2
fi

# ── where the artifact lands, and why it is not /var/tmp ───────────────────────────────
#
# The board user's home, not `/var/tmp/duck-sideload`, which is where this used to put it and
# which cannot work: `updaterd.service` sets `PrivateTmp=yes`, so the unit gets a `/tmp` and a
# `/var/tmp` of its own. The files land in the ones the shell sees, `updaterd` reads the ones the
# namespace gave it, and `apply --from` fails with "no manifest for version ... in
# /var/tmp/duck-sideload" against a directory whose `ls` lists that exact manifest. It cost an
# afternoon to see, because every artifact was demonstrably where the error said it was not.
#
# The home directory has the property /var/tmp was picked for — writable by the ssh user, so no
# sudo to copy into it — and is outside that namespace. `updaterd` sets no `ProtectHome=`, and
# `xtask/tests/sideload.rs` fails if either half of that stops being true.
#
# Resolved on the board rather than written literally: `$HOME` here is this laptop's, and the
# apply needs an absolute path because `updaterd`'s working directory is not the user's.
if [ -z "$REMOTE_DIR" ]; then
    REMOTE_DIR="$(ssh "$BOARD" 'echo "$HOME/duck-sideload"')"
    [ -n "$REMOTE_DIR" ] || { echo "could not resolve a home directory on $BOARD" >&2; exit 1; }
fi

if [ "$DOCKER" = no ]; then
    # `command -v`, not `cargo zigbuild --version`: the subcommand forwards its arguments to
    # `cargo build`, which rejects `--version`, so asking it that way reports the toolchain as
    # missing on a machine where it is installed and working.
    if ! command -v cargo-zigbuild >/dev/null 2>&1; then
        echo "cargo-zigbuild is not installed; the board target has no linker without it" >&2
        echo "  cargo install cargo-zigbuild --locked" >&2
        echo "  brew install zig" >&2
        echo "or build in a container instead, which needs neither:" >&2
        echo "  scripts/dev-push.sh --docker $BOARD" >&2
        exit 1
    fi
elif ! docker version >/dev/null 2>&1; then
    echo "--docker needs a running Docker daemon" >&2
    echo "  open -a Docker" >&2
    exit 1
fi

if [ ! -f "$KEY" ]; then
    echo "no dev signing key at $KEY" >&2
    echo "The board verifies this artifact like any release, so it has to be signed." >&2
    echo "Get team.dev.key from a team member, or set DUCK_DEV_SECRET_KEY." >&2
    exit 1
fi

# ── the C dependencies, and where the target's copies come from ────────────────────────
#
# This used to `scp` libudev.so.1 off the board and hand-write a `.pc` beside it, which worked
# because `libudev-sys` asks for no particular version and the linker records the SONAME rather
# than the filename. GStreamer ended that: `mediad` needs seven pkg-config modules with real
# `Cflags` and `Requires`, and hand-writing those is not a thing to do.
#
# Two things got better rather than merely different. The sysroot comes from Debian trixie — the
# same archive the board installs from — so the versions match by construction instead of by
# having copied one file off one board. And **building no longer needs a reachable board at all**:
# the old path failed with "no libudev.so.1 on $BOARD" when the board you wanted to set up was the
# board you needed to build, which is the wrong way round.
#
# One quirk carried over from the old path, and it is inert: `libudev-sys`'s build script probes
# for `udev_hwdb_new` by linking a test binary with the *host* toolchain, which fails on a Mac and
# leaves its `hwdb` cfg off. `gilrs` calls nothing under that cfg.
#
# `--docker` needs none of this — see `scripts/dev-build.Dockerfile`.
if [ "$DOCKER" = no ]; then
    sysroot_env="$(sh "$(dirname "$0")/cross-sysroot.sh" | grep '^export')" || {
        echo "could not build the aarch64 sysroot; see scripts/cross-sysroot.sh" >&2
        exit 1
    }
    # `eval` rather than sourcing a file: the script prints the exports for a human to read first,
    # and this is the same four lines it prints.
    eval "$sysroot_env"
fi

SHA="$(git rev-parse HEAD)"
SHA7="$(git rev-parse --short=7 HEAD)"
# Read from `cargo metadata` rather than parsed out of Cargo.toml, the same way `dev.yml`
# derives it: the value lives in `[workspace.package]` and the members inherit it, so grepping
# a member's manifest finds `version.workspace = true` and grepping the root finds a line that
# is only the workspace version by convention.
CRATE="$(cargo metadata --format-version 1 --no-deps | python3 -c 'import json,sys; print(next(p["version"] for p in json.load(sys.stdin)["packages"] if p["name"] == "updater"))')"
VERSION="${CRATE}-dev.local.$(date +%s).g${SHA7}"

rm -rf staged dist

# `DUCK_REVISION` and no `DUCK_BUILD_TIME`, unlike the release workflows, and it is worth a
# measurement rather than a shrug. Both are read with `option_env!` in `duck-ipc-proto`, which
# every daemon depends on, so a value that changes invalidates it and everything above it: with
# a fresh timestamp each run, five crates rebuild on every push whether or not a line changed
# (~30s here); with only the revision, which moves when you commit rather than when you push, an
# unchanged tree rebuilds nothing at all. Nothing visible is lost — `robotctl version` reports
# the revision and not the build time, and the version string carries the push's epoch.
if [ "$DOCKER" = no ]; then
    echo "==> building $VERSION for the board (zigbuild)"
    BIN="target/aarch64-unknown-linux-gnu/release"
    DUCK_REVISION="$SHA" cargo board --bins
else
    echo "==> building $VERSION for the board (docker)"
    # A separate target directory, not the one `cargo board` writes. The two builds produce the
    # same triple through different toolchains and linkers, and cargo's fingerprints do not
    # capture all of that difference — sharing a directory risks it deciding a binary from the
    # other environment is up to date. Two directories cost disk and nothing else.
    BIN="target/docker/aarch64-unknown-linux-gnu/release"

    # Rebuilt every time and cached by Docker, so a change to the Dockerfile takes effect
    # without anyone remembering to bump a tag. The context is `scripts/`, which keeps the
    # repository (and `target/`) out of the daemon's hands.
    docker build -q -t duck-dev-build -f scripts/dev-build.Dockerfile scripts/ >/dev/null

    if [ "$(uname -m)" != arm64 ] && [ "$(uname -m)" != aarch64 ]; then
        echo "    host is $(uname -m): the arm64 container runs under emulation, expect slow" >&2
    fi

    # `--platform linux/arm64` so the binaries are aarch64 wherever this runs. On Apple Silicon
    # that is a native build and the target is the host, which is the entire point: nothing to
    # cross, and libudev came from apt.
    #
    # Registry cache in a named volume rather than the host's `~/.cargo`: crate sources are
    # re-downloaded once instead of two cargos with different ideas about locking sharing a
    # directory. `target/docker` is a bind mount so builds stay incremental across runs and the
    # binaries are here afterwards without a copy step.
    docker run --rm --platform linux/arm64 \
        -v "$PWD:/src" -w /src \
        -v duck-dev-cargo-registry:/usr/local/cargo/registry \
        -e CARGO_TARGET_DIR=/src/target/docker \
        -e DUCK_REVISION="$SHA" \
        duck-dev-build \
        cargo build --release --target aarch64-unknown-linux-gnu --bins
fi

echo "==> packaging"
mkdir -p staged
# The same list as the `cp` block in `dev.yml` and `release.yml`, deliberately: this pushes the
# same artifact a release does, and `xtask/tests/artifact.rs` packages all three lists and checks
# the tarball, so a binary added to one and not the others fails there. Only the directory
# differs, because which toolchain built these is the one thing that does.
cp "$BIN"/updaterd staged/
cp "$BIN"/robotctl staged/
cp "$BIN"/robotd staged/
cp "$BIN"/configd staged/
cp "$BIN"/btd staged/
cp "$BIN"/padd staged/
# The WebRTC gateway. Its unit ships with an `[Install]` section, so postinstall enables and starts
# it and `on_apply` restarts it, exactly as for every other daemon here.
cp "$BIN"/mediad staged/
# The voice generator (postinstall renders the per-robot bank with it) and the
# mic classifier pair — pet-detect for live listening, pet-features for training.
cp "$BIN"/sounds staged/
cp "$BIN"/pet-detect staged/
cp "$BIN"/pet-features staged/
# The head ToF daemon. Its unit is packaged below; a board with no sensor
# fitted runs it anyway and says so, which is cheaper than a special case.
cp "$BIN"/tofd staged/

# No `--base-url`: the manifest `LocalDir` reads names the artifact by bare filename, and
# `package` leaves it bare when no base is given.
#
# `--zstd-level 1` rather than the shipping default of 19. This artifact is read once, by one
# board, and thrown away; at 19 the compression alone is most of the wall-clock of this script,
# which is the one thing it exists to keep short.
cargo run -p xtask -- package \
    --version "$VERSION" \
    --channel daemon \
    --bin-dir staged \
    --out dist \
    --revision "$SHA" \
    --zstd-level 1 \
    --include "updater/systemd/updaterd.service=systemd/updaterd.service" \
    --include "updater/systemd/sysusers.d/robot.conf=systemd/sysusers.d/robot.conf" \
    --include "robotd/systemd/robotd.service=systemd/robotd.service" \
    --include "hooks/postinstall=hooks/postinstall" \
    --include "scripts/setup-gstreamer.sh=scripts/setup-gstreamer.sh" \
    --include "duck-detect/models/duck_detect.rknn=models/duck_detect.rknn" \
    --include "duck-detect/models/duck_detect.onnx=models/duck_detect.onnx" \
    --include "scripts/setup-npu.sh=scripts/setup-npu.sh" \
    --include "deploy/overlays/rk3568-npu-enable.dts=deploy/overlays/rk3568-npu-enable.dts" \
    --include "scripts/setup-rkaiq.sh=scripts/setup-rkaiq.sh" \
    --include "scripts/rkaiq-modinfo-shim.c=scripts/rkaiq-modinfo-shim.c" \
    --include "scripts/setup-login.sh=scripts/setup-login.sh" \
    --include "scripts/seed-policies.sh=scripts/seed-policies.sh" \
    --include "scripts/robot-rescue=scripts/robot-rescue" \
    --include "scripts/robot-boot-check=scripts/robot-boot-check" \
    --include "updater/systemd/robot-boot-check.service=systemd/robot-boot-check.service" \
    --include "updater/systemd/robot-boot-check.timer=systemd/robot-boot-check.timer" \
    --include "configd/systemd/configd.service=systemd/configd.service" \
    --include "btd/systemd/btd.service=systemd/btd.service" \
    --include "btd/systemd/sysusers.d/btd.conf=systemd/sysusers.d/btd.conf" \
    --include "padd/systemd/padd.service=systemd/padd.service" \
    --include "padd/systemd/sysusers.d/padd.conf=systemd/sysusers.d/padd.conf" \
    --include "mediad/systemd/mediad.service=systemd/mediad.service" \
    --include "mediad/systemd/sysusers.d/mediad.conf=systemd/sysusers.d/mediad.conf" \
    --include "tof/systemd/tofd.service=systemd/tofd.service" \
    --include "tof/systemd/sysusers.d/tofd.conf=systemd/sysusers.d/tofd.conf" \
    --include "deploy/journald.conf.d/10-robot.conf=deploy/journald.conf.d/10-robot.conf" \
    --include "docs/design/architecture.md=docs/architecture.md" \
    --include "docs/design/updater-design.md=docs/updater-design.md" \
    --include "deploy/README.md=docs/deploy.md" \
    --include "pet-detect/models/pet_detect.onnx=models/pet_detect.onnx"


echo "==> signing with $KEY"
cargo run -p xtask -- sign --dir dist --key "$KEY"

# Replaced rather than added to: a directory holding two builds makes "the newest one here"
# ambiguous to read, and nothing on the board needs yesterday's push.
echo "==> copying to $BOARD:$REMOTE_DIR"
# shellcheck disable=SC2029  # expanding the path here is the intent: it is this laptop's
# setting, and the board has no DUCK_SIDELOAD_DIR to read.
ssh "$BOARD" "rm -rf '$REMOTE_DIR' && mkdir -p '$REMOTE_DIR'"
# `scp`, not `rsync`: the artifact is a single compressed blob that changes completely every
# build, so there is no delta to exploit, and this needs nothing on the board that ssh did not
# already bring.
scp -q dist/* "$BOARD:$REMOTE_DIR/"

if [ "$BOOTSTRAP" = yes ]; then
    # For a board whose *installed* `updaterd` predates `apply --from` and therefore cannot be
    # asked to use it — including the push that first delivers it. This is the documented
    # escape hatch, and it costs the health gate for one install: `updaterd install` forces
    # `on_apply` and `health` off, which is why it refuses a live release without `--force`.
    #
    # `updaterd` then has to be restarted explicitly. It never restarts itself during an update
    # — that would kill the process performing it — so the resident daemon keeps running the old
    # binary, and the old binary is the one that does not understand `--from`.
    echo "==> bootstrap install (no health gate, robotd stopped)"
    # shellcheck disable=SC2029  # as above: $REMOTE_DIR is expanded locally on purpose.
    ssh -t "$BOARD" "set -e
        sudo systemctl stop robotd
        sudo /opt/robot/daemon/current/bin/updaterd install --from '$REMOTE_DIR' --force
        sudo systemctl restart updaterd
        sudo systemctl start robotd"
    echo "==> installed $VERSION (ungated); ordinary pushes need no --bootstrap from here"
    exit 0
fi

echo "==> applying on $BOARD"
APPLY="sudo robotctl update apply daemon --from '$REMOTE_DIR' --version '$VERSION'"
[ "$DRY_RUN" = no ] || APPLY="$APPLY --dry-run"

if ssh -t "$BOARD" "$APPLY"; then
    echo "==> $VERSION is live on $BOARD"
else
    status=$?
    echo "==> apply failed (exit $status)" >&2
    # Exit 2 from robotctl is bad usage, and the way this fails on a board that has not had a
    # build with `--from` yet: the daemon refuses the whole call on the API version, because a
    # daemon that merely ignored the option would install from its configured source instead
    # and report success for the wrong release.
    if [ "$status" -eq 2 ]; then
        echo "If robotctl and updaterd report an API mismatch, this board's installed" >&2
        echo "release predates 'apply --from'. Deliver it once, ungated:" >&2
        echo "  scripts/dev-push.sh --bootstrap $BOARD" >&2
    fi
    exit "$status"
fi

# ── did every daemon actually move? ──
#
# The apply reporting success means the swap happened and the health gate passed. It does not mean
# the seven daemons are running the release that was swapped in, and the gap between those two is
# where an afternoon goes: four wifi fixes were once verified as broken against a `configd` that
# had never restarted. `robotd`, `configd`, `padd`, `mediad` and `tofd` restart during the update;
# `updaterd` and `btd` restart five seconds after it replies, because the first cannot restart
# itself mid-update and the second may be carrying the reply (docs/design/restart-order.md).
#
# So this is the one check that observes the whole mechanism end to end, on real systemd, with real
# timing — and nothing else in the repository can. A container cannot: the transient timer, the
# `RuntimeDirectory=` that holds each identity, and the five-second delay are all systemd.
#
# Polled rather than slept: a fixed sleep is either a wrong answer or a slow one, and the interesting
# case is a restart that never happens, which no length of sleep improves.
[ "$DRY_RUN" = no ] || exit 0

echo "==> checking every daemon is running it"

# Sent on stdin, so nothing here is expanded by this laptop's shell and the quoting stays readable.
# The board derives what it expects from its own `current` symlink; the version this push built is
# compared against that separately, below.
if ssh "$BOARD" sh -s -- "$VERSION" <<'REMOTE'
set -u
pushed="${1:-}"
current="$(readlink /opt/robot/daemon/current || true)"
want="${current#releases/}"
[ -n "$want" ] || { echo "    no release is live: current -> ${current:-nothing}"; exit 1; }
echo "    current -> $want"

# That the daemons agree with `current` is not enough on its own: they could all agree on the
# release this push was supposed to replace. `apply` reported success, so this should be
# impossible — which is the reason to check it rather than the reason not to.
[ "$want" = "$pushed" ] || {
    echo "    [FAIL] current is $want, but this push built $pushed"
    exit 1
}

# The identity each daemon publishes at startup, which names the release directory it was launched
# from. A file rather than a question, so this needs no socket and no privilege — and `btd` serves
# no socket at all, so for that one it is the only answer available.
deadline=$(($(date +%s) + 30))
stale=""
for svc in robotd configd padd updaterd btd mediad tofd; do
    while :; do
        if [ ! -f "/run/${svc}/identity.json" ]; then
            state="silent"
        elif grep -q "releases/${want}/bin/${svc}" "/run/${svc}/identity.json"; then
            state="ok"
        else
            state="stale"
        fi
        # Only `updaterd` and `btd` are worth waiting for — they restart five seconds after the
        # reply. The rest restarted before it, so a mismatch there is a fault rather than a race.
        case "$state:$svc" in
            ok:*) break ;;
            stale:updaterd|stale:btd|silent:updaterd|silent:btd)
                [ "$(date +%s)" -lt "$deadline" ] || break
                sleep 1
                ;;
            *) break ;;
        esac
    done

    case "$state" in
        ok) echo "    [ok] $svc" ;;
        silent)
            # Not treated as a failure: systemd removes the runtime directory when a unit stops, so
            # this is what a deliberately disabled `padd` looks like, and what `mediad` looks like on
            # a board with no camera. `tofd` runs whether or not a sensor is fitted, so silent there
            # is a release from before it published an identity at all — one push fixes it. The GStreamer stack is not a cause any more — the preinstall
            # hook this push just ran installs it — so a silent `mediad` here is worth
            # `journalctl -u mediad -b` rather than a command to type.
            echo "    [--] $svc published nothing — stopped, or a build too old to say"
            ;;
        stale)
            echo "    [FAIL] $svc is not running $want"
            stale="${stale} ${svc}"
            ;;
    esac
done

[ -z "$stale" ] || {
    echo
    echo "  Stale:${stale}. The release is installed and those are not running it, which reads as"
    echo "  a fix that did not work. What to look at:"
    echo "    robotctl health                     # the units block names the release each one runs"
    echo "    journalctl -u updaterd -b | tail    # 'restart scheduled', or why it could not be"
    echo "    sudo systemctl restart <unit>       # and then why it needed doing by hand"
    exit 1
}

# Answerable, not healthy. A bench board with no servo power reports degraded and that is a fact
# about the bench, not about this build — the health gate draws the same distinction.
robotctl health >/dev/null 2>&1 || echo "    [--] robotctl health did not answer cleanly; worth a look"
REMOTE
then
    echo "==> every daemon on $BOARD is running $VERSION"
else
    echo "==> the release is live but not everything is running it" >&2
    exit 1
fi
