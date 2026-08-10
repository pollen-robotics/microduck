#!/bin/sh
# Install the target architecture's C libraries needed to cross-compile for the board.
#
# Only one is needed today: libudev, for `gilrs` in `padd`. Everything else that reaches
# the board is pure Rust or, like `serialport` and `ort`, configured to avoid a C
# dependency — `serialport` with `default-features = false`, `ort` with `load-dynamic`.
#
# **This script is the cost of that one exception**, and it is worth reading before adding
# another. Cross-compiling a C binding needs the target's headers and libraries, which
# means multiarch, which on Ubuntu means knowing that the main archive does not carry
# arm64 at all: those packages live on ports.ubuntu.com, and pointing apt at both without
# breaking amd64 is the bulk of what follows.
#
# Ubuntu runners only. Called by the `board` job and by the release workflows.
set -eu

ARCH="${CROSS_ARCH:-arm64}"

if ! command -v apt-get >/dev/null 2>&1; then
    echo "not a Debian-family host; nothing to do" >&2
    exit 0
fi

# shellcheck disable=SC1091  # /etc/os-release is provided by the runner image.
suite="$(. /etc/os-release && echo "${VERSION_CODENAME:-}")"
[ -n "$suite" ] || { echo "cannot determine the Ubuntu suite" >&2; exit 1; }

sudo dpkg --add-architecture "$ARCH"

# The existing sources serve amd64 only. Without pinning them, apt asks them for $ARCH
# too, gets a 404 for every index, and fails the whole update — which is exactly how this
# first went wrong.
for file in /etc/apt/sources.list.d/*.sources; do
    [ -f "$file" ] || continue
    sudo sed -i '/^Architectures:/d' "$file"
    sudo sed -i 's|^Types:|Architectures: amd64\nTypes:|' "$file"
done
if [ -f /etc/apt/sources.list ]; then
    sudo sed -i 's|^deb \(\[[^]]*\]\)\? *|deb [arch=amd64] |' /etc/apt/sources.list
fi

# ...and $ARCH comes from ports, which is a different host serving a different set.
#
# **`-security` is load-bearing, not thoroughness.** `libudev1` is `Multi-Arch: same`, so apt must
# hold both architectures at the identical version. The host's amd64 sources include
# `noble-security`, so the moment a security update lands there — 255.4-1ubuntu8.17, in the case that
# taught us this — apt intends to upgrade the amd64 copy to it, and then refuses to install any
# ${ARCH} version that would disagree:
#
#   libudev-dev:arm64 : Depends: libudev1:arm64 (= 255.4-1ubuntu8.16)
#                       but it is not going to be installed
#
# With only `${suite}` and `${suite}-updates` configured for ${ARCH}, 8.17 does not exist as far as
# this side is concerned, so there is no version both architectures can agree on and the resolver
# gives up. Nothing has drifted and no mirror is at fault: the suites were simply asymmetric, and it
# broke the first time libudev had a security fix. It is *not* enough to omit this and hope.
sudo tee "/etc/apt/sources.list.d/ports-${ARCH}.sources" >/dev/null <<EOF
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports
Suites: ${suite} ${suite}-updates ${suite}-security
Components: main universe
Architectures: ${ARCH}
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

sudo apt-get update

# What apt believes about both architectures, before it is asked for anything.
#
# Here because this step failed with a refusal that explained nothing:
#
#   libudev-dev:arm64 : Depends: libudev1:arm64 (= 255.4-1ubuntu8.16)
#                       but it is not going to be installed
#
# Kept because it is what answered the question: the candidate versions, side by side, showed the
# host being taken to a security version the ${ARCH} suites had never been told about. Guessing at
# that from the refusal alone cost several rounds; a screenful of `policy` in the log costs nothing.
echo "==> what apt believes"
apt-cache policy libudev1 "libudev1:${ARCH}" libudev-dev "libudev-dev:${ARCH}" || true
dpkg -l | grep -i libudev || true
sudo apt-get install -y "libudev-dev:${ARCH}"

# Prove it landed. A silent miss here surfaces much later as a confusing link error, or —
# worse — as pkg-config answering with the *host's* library and producing a binary that
# cannot run on the robot.
pc="/usr/lib/${ARCH}-linux-gnu/pkgconfig/libudev.pc"
case "$ARCH" in
    arm64) pc="/usr/lib/aarch64-linux-gnu/pkgconfig/libudev.pc" ;;
esac
[ -f "$pc" ] || { echo "libudev.pc for ${ARCH} not found at ${pc}" >&2; exit 1; }
echo "cross libudev ready: ${pc}"
