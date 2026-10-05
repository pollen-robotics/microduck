# Build image for `scripts/dev-push.sh --docker`: the board's userland, with the one C
# dependency installed the way CI installs it.
#
# **Why this exists at all.** The default path cross-compiles with `cargo zigbuild`, which needs
# `zig`, `cargo-zigbuild`, and — because `padd` links libudev and a Mac cannot install an aarch64
# Linux library — a copy of `libudev.so.1` taken off a board. Each of those is a way for "I want
# to try this on the robot" to turn into an afternoon of toolchain work. In here, the target *is*
# the host: `apt-get install libudev-dev` is all it takes, and there is nothing to cross.
#
# **Bookworm, not Trixie**, and this is the same reasoning as `scripts/board-test.sh`: build
# against an older glibc than any userland we might ship. Bookworm's 2.36 is below Trixie's 2.41,
# so a binary built here loads on the board we ship *and* on an older image, while one built on
# Trixie would require 2.41 and fail on anything older with a message that names no cause. This
# is the container equivalent of the `.2.31` glibc pin in `.cargo/config.toml`.
#
# **On an arm64 host this is a native build**, not an emulated one: the daemons that come out are
# aarch64 ELF because the container is. `dev-push.sh` passes `--platform linux/arm64` so that
# holds on an x86 laptop too, where it costs qemu and the script says so.
#
# `rust:1-bookworm` floats to the newest 1.x rather than pinning: the workspace declares its own
# floor (`rust-version = "1.89"`), so a toolchain below it fails with that message rather than
# with a missing method, and a pin here would be a second place to bump.
FROM rust:1-bookworm

# libudev for `gilrs` (via `padd`) — the one exception to "everything reaching the board is pure
# Rust", and the reason `scripts/ci-cross-deps.sh` exists on the CI side. pkg-config is how
# libudev-sys finds it. Nothing else: `zstd`'s C is compiled from source by the crate, and the
# rust image already carries a C compiler for that.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libudev-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*
