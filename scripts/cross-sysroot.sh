#!/bin/sh
# Build the aarch64 sysroot that `mediad` cross-compiles against.
#
# `cargo board` links one C dependency today — libudev, for `gilrs` in `padd` — and
# The multiarch script this replaced said plainly that it "is the cost of that one exception, and
# it is worth reading before adding another". GStreamer is the second, and it is much larger: the
# `gstreamer-rs` crates are pkg-config crates, so cross-compiling them needs the *target's* headers,
# `.pc` files and shared libraries on this machine.
#
# The route CI takes for libudev — Ubuntu multiarch plus ports.ubuntu.com — is not the route here,
# for one reason worth stating: it would link against *Ubuntu's* GStreamer while the robot runs
# Debian trixie's. This unpacks the robot's own packages instead, so what the compiler sees is what
# the board has, at the same version.
#
#   sh scripts/cross-sysroot.sh            build it, then print what to export
#   sh scripts/cross-sysroot.sh --check    verify an existing one and print nothing else
#
# Runs on macOS and Linux. Needs curl, ar, tar, awk and pkg-config — no dpkg, because a `.deb` is
# an `ar` archive containing a tarball and both hosts have those.
#
# It downloads and unpacks; it never installs anything, and everything lives under one directory
# you can delete.
set -eu

SYSROOT="${DUCK_SYSROOT:-${TMPDIR:-/tmp}/duck-aarch64-sysroot}"
CACHE="${SYSROOT}/.cache"

# Debian 13. The robot runs Armbian on a trixie userland and takes GStreamer from the Debian
# archive — `apt-cache policy` on a provisioned board shows deb.debian.org and security.debian.org
# with no Armbian multimedia overlay — so this is the same archive the board installs from.
MIRROR="${DUCK_DEBIAN_MIRROR:-http://deb.debian.org/debian}"
SUITE="${DUCK_DEBIAN_SUITE:-trixie}"
ARCH=arm64
TRIPLE=aarch64-linux-gnu

# The pkg-config modules `mediad` needs. `--check` verifies exactly these, so a package list that
# goes stale fails here rather than inside a build.
#
# `gstreamer-webrtc-1.0` and `gstreamer-sdp-1.0` come from plugins-bad. `mediad` drives
# `webrtcsink` mostly by setting properties and connecting signals, which is core GStreamer and
# GLib — but a data channel is a `GstWebRTCDataChannel`, so the typed bindings are wanted rather
# than reaching for untyped `glib::Object` calls.
#
# **`libudev` is here because it has to be, and the reason is worth knowing.**
# `PKG_CONFIG_LIBDIR` *replaces* pkg-config's search path rather than adding to it, so a sysroot
# that carries only GStreamer breaks `padd`, whose `gilrs` needs libudev — `cargo board` then fails
# in `libudev-sys`, nowhere near anything about media. Replacing is nonetheless the right choice:
# `PKG_CONFIG_PATH` is additive to the host's, and that costs exactly what you would fear —
# pkg-config answering with the *host's* library and producing a binary that cannot run on the
# robot. So this sysroot serves the whole workspace rather than one crate of it.
MODULES="gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 gstreamer-audio-1.0
gstreamer-webrtc-1.0 gstreamer-sdp-1.0 gstreamer-rtp-1.0 libudev"

# The packages that satisfy those modules, and nothing more.
#
# **Explicit, not resolved.** Walking Debian `Depends` from these roots pulls 543 packages —
# `libgstreamer-plugins-bad1.0-dev` declares every optional backend's dev package, so the closure
# reaches Qt, Vulkan and OpenEXR. None of that is needed to compile against one `.pc` file.
#
# Derived by unpacking the four obvious roots and then asking `pkg-config` what was missing, one
# round at a time, mapping each answer back to a package through the archive's `Contents` index:
# libpcre2-8, libffi, orc-0.4, zlib, mount, blkid, libselinux, libsepol — eight rounds, in that
# order. Re-derive the same way if `--check` starts failing; the map is
# `Contents-arm64.gz`, whose lines are `path  section/package`.
#
# Note `libglib2.0-0t64` and not `libglib2.0-0`: trixie's 64-bit `time_t` transition renamed it.
# And `libgio-2.0-dev` rather than `libglib2.0-dev`, which is a 55 KiB metapackage — GLib's headers
# and `.pc` files moved out of it.
#
# **A `-dev` package alone is not enough for anything actually linked**, and the split is not
# obvious. A `-dev` ships `libfoo.so` as a *symlink* onto the `libfoo.so.N` that lives in the
# runtime package, so `-lfoo` on the link line needs both. That is why the GStreamer, GLib and
# udev runtime packages are here.
#
# The rest are `-dev` only, and correctly so: pcre2, ffi, orc, zlib, mount, blkid, selinux and
# sepol appear in `Requires.private`, which pkg-config needs to *resolve* but which never reaches
# the link line of a dynamically linked binary. If one of them ever does, the symptom is
# `unable to find dynamic system library` and the fix is its runtime package.
PACKAGES="libgstreamer1.0-dev libgstreamer1.0-0
libgstreamer-plugins-base1.0-dev libgstreamer-plugins-base1.0-0
libgstreamer-plugins-bad1.0-dev libgstreamer-plugins-bad1.0-0
libgio-2.0-dev libglib2.0-dev libglib2.0-dev-bin libglib2.0-0t64
libsysprof-capture-4-dev libpcre2-dev libffi-dev liborc-0.4-dev
zlib1g-dev libmount-dev libblkid-dev libselinux1-dev libsepol-dev
libudev-dev libudev1"

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

check_tools() {
    for t in curl ar tar awk pkg-config; do
        command -v "$t" >/dev/null 2>&1 || die "${t} is required"
    done
}

# The environment a cross build needs. Printed rather than exported, because a script cannot
# export into its caller's shell — and `eval` on this is one line the caller can read first.
print_env() {
    cat <<EOF
export PKG_CONFIG_SYSROOT_DIR="${SYSROOT}"
export PKG_CONFIG_LIBDIR="${SYSROOT}/usr/lib/${TRIPLE}/pkgconfig:${SYSROOT}/usr/share/pkgconfig"
export PKG_CONFIG_ALLOW_CROSS=1
export RUSTFLAGS="\${RUSTFLAGS:+\$RUSTFLAGS }-L ${SYSROOT}/usr/lib/${TRIPLE}"
EOF
}

# Are all of MODULES resolvable against the sysroot?
verify() {
    PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
    PKG_CONFIG_LIBDIR="${SYSROOT}/usr/lib/${TRIPLE}/pkgconfig:${SYSROOT}/usr/share/pkgconfig"
    PKG_CONFIG_ALLOW_CROSS=1
    export PKG_CONFIG_SYSROOT_DIR PKG_CONFIG_LIBDIR PKG_CONFIG_ALLOW_CROSS

    bad=0
    for mod in $MODULES; do
        if version="$(pkg-config --modversion "$mod" 2>/dev/null)"; then
            printf '  %-24s %s\n' "$mod" "$version"
        else
            # The first line names the module pkg-config could not find, which is usually a
            # transitive one rather than $mod itself — that is the package to add above.
            printf '  %-24s MISSING: %s\n' "$mod" \
                "$(pkg-config --print-errors --exists "$mod" 2>&1 | head -1)"
            bad=1
        fi
    done
    return "$bad"
}

fetch() {
    install -d "$CACHE"
    index="${CACHE}/Packages"
    if [ ! -s "$index" ]; then
        say "fetching the ${SUITE}/${ARCH} package index"
        curl -fsSL -o "${index}.gz" \
            "${MIRROR}/dists/${SUITE}/main/binary-${ARCH}/Packages.gz" \
            || die "cannot fetch the package index from ${MIRROR}"
        gunzip -f "${index}.gz" || die "cannot gunzip the package index"
    fi

    for pkg in $PACKAGES; do
        # `Filename:` for this exact package, from its own stanza. Matched on a whole-line
        # `Package:` so `libgstreamer1.0-0` cannot be answered by `libgstreamer1.0-0-dbgsym`.
        rel="$(awk -v want="$pkg" '
            /^Package: /   { cur = ($2 == want) }
            cur && /^Filename: / { print $2; exit }
        ' "$index")"
        [ -n "$rel" ] || die "no ${pkg} in ${SUITE}/${ARCH}.
  Debian renames packages between releases — trixie moved GLib's headers out of libglib2.0-dev
  and renamed libglib2.0-0 to libglib2.0-0t64. If this is a rename, fix PACKAGES above."

        deb="${CACHE}/$(basename "$rel")"
        [ -s "$deb" ] || curl -fsSL -o "$deb" "${MIRROR}/${rel}" \
            || die "cannot download ${MIRROR}/${rel}"

        # `ar x` writes into the working directory, so each unpack gets its own.
        tmp="$(mktemp -d)"
        ( cd "$tmp" && ar x "$deb" ) || { rm -rf "$tmp"; die "cannot unpack ${deb}"; }
        data="$(find "$tmp" -maxdepth 1 -name 'data.tar*' | head -1)"
        [ -n "$data" ] || { rm -rf "$tmp"; die "no data.tar in ${deb}"; }
        tar -xf "$data" -C "$SYSROOT" || { rm -rf "$tmp"; die "cannot extract ${data}"; }
        rm -rf "$tmp"
        printf '  %s\n' "$pkg"
    done
}

main() {
    check_tools

    if [ "$CHECK_ONLY" = 1 ]; then
        [ -d "$SYSROOT" ] || die "no sysroot at ${SYSROOT}; run this without --check first"
        say "verifying ${SYSROOT}"
        verify || die "the sysroot does not satisfy every module above.
  The MISSING line names the module pkg-config could not find — look it up in the archive's
  Contents-arm64 index and add the package that ships it to PACKAGES in this script."
        exit 0
    fi

    install -d "$SYSROOT"
    say "unpacking into ${SYSROOT}"
    fetch

    printf '\n'
    say "pkg-config against the sysroot"
    verify || die "built the sysroot and it still does not satisfy every module above."

    printf '\n'
    say "ready — export these, or: eval \"\$(sh scripts/cross-sysroot.sh --check >/dev/null && \
sh scripts/cross-sysroot.sh 2>/dev/null | grep ^export)\""
    printf '\n'
    print_env
    cat <<EOF

Then \`cargo board\` links against the robot's own libraries. Delete ${SYSROOT} to start over;
the downloaded .debs are cached under it, so a rebuild costs nothing.
EOF
}

# Called on the last line so a truncated download defines functions and then does nothing, rather
# than running half a build.
main "$@"
