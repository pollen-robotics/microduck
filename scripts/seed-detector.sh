#!/bin/sh
# Install the duck detector, downloading it from the Hugging Face Hub.
#
# `mediad` reads the detector from /opt/robot/detector/current, deliberately outside the release,
# for the reason `robotd` reads its policies from /opt/robot/policies/current: a retrain should not
# need a daemon release, and a daemon fix should not re-ship fourteen megabytes of unchanged
# weights. The model is trained and published by pollen-robotics/duck_detector; this is what puts
# it on a board, and it is `seed-policies.sh` with a fixed file list — read that script for the
# reasoning behind every rule here, which is the same.
#
# Run by `hooks/postinstall` on every update and so by `scripts/install.sh` on a fresh board.
#
# THE RULE THAT MATTERS: never touch a set this script did not install. `current` pointing at
# anything but a `seed-*` directory means something else put a detector there, and replacing that
# would silently undo it on the next unrelated daemon update.
#
# Nothing here is signed, as with the policies: the model is data, `mediad` refuses one whose input
# is not a square RGB tensor, and a truncated download fails to load rather than running. And it
# is never fatal — the detector is off by default, and a robot asked to look for ducks with no model
# says so in `mediad`'s journal rather than failing an update.
#
# **The model repo shares its name with the dataset repo.** The robot only ever addresses the
# model: `huggingface.co/<repo>/resolve/<rev>/<file>` is the model's URL, and the dataset's carries
# a `datasets/` prefix this script never writes. Nothing here can fetch a frame by accident.
#
# Usage: seed-detector.sh [DETECTOR_ROOT]
# Defaults to what a robot uses; the argument exists so this can be tested off a board.
set -eu

DETECTOR_ROOT="${1:-/opt/robot/detector}"

# The pin. An xtask test asserts these literals match `[workspace.metadata.detector]` in
# Cargo.toml — this script runs from inside a release and cannot read the manifest.
#
# A floor, not a ceiling: what a board installs when it has *nothing*. A board moves past it with
# `robotctl duck-detector update`, which needs no daemon release; bump this when fresh boards should get
# a newer model, not to push one to boards that already have one.
DETECTOR_REPO="${DETECTOR_REPO:-pollen-robotics/microduck-duck-detector}"
DETECTOR_VERSION="${DETECTOR_VERSION:-duck-v1}"
DETECTOR_BASE_URL="${DETECTOR_BASE_URL:-https://huggingface.co/${DETECTOR_REPO}/resolve/${DETECTOR_VERSION}}"

# Both, always. The `.rknn` is what the detector is for; the `.onnx` is the CPU fallback for a
# board whose NPU is switched off in its device tree, which is how Armbian ships the Radxa Zero 3
# (`robotd_params::DuckDetectorParams::models`). A revision missing either is one not ready for robots,
# and `updater::policy::DETECTOR_FILES` is the same list for `robotctl duck-detector update`.
DETECTOR_FILES="duck_detect.rknn duck_detect.onnx"

# Per-file, and generous: the ONNX is ten megabytes, and `hooks/postinstall` runs inside an
# update under a 120-second hook timeout that the policies' seeder already spends up to 72 s of.
# Two files at twenty seconds is 40, which keeps the whole hook under its budget. A link that
# cannot move 10 MB in twenty seconds is one the fallback below is for, and the next update tries
# again.
CURL_OPTS="--fail --location --silent --show-error --connect-timeout 5 --max-time 20"

# Where a set came from, written beside it — the record `robotctl duck-detector check` reads to know which
# repo to ask, in the format `seed-policies.sh` writes and `updater::policy::Source` parses.
write_source() {
    [ ! -f "$1/.source" ] || return 0
    {
        echo "repo=${DETECTOR_REPO}"
        echo "version=${2}"
        echo "fetched=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    } > "$1/.source" || echo "seed-detector: cannot record where this detector came from" >&2
}

target="releases/seed-${DETECTOR_VERSION}"
live="$(readlink "${DETECTOR_ROOT}/current" 2>/dev/null || true)"

# **A set that is already installed is never replaced.** Two states: something is installed, or
# nothing is. Anything installed — whoever installed it — is left alone, with its provenance
# record back-filled if missing.
if [ -n "$live" ]; then
    case "$live" in
        releases/*)
            write_source "${DETECTOR_ROOT}/${live}" "${live#releases/seed-}" ;;
        *)
            echo "seed-detector: ${DETECTOR_ROOT}/current is not ours; leaving it alone" >&2 ;;
    esac
    exit 0
fi

staging="${DETECTOR_ROOT}/releases/.staging"
rm -rf "$staging"
mkdir -p "$staging" || { echo "seed-detector: cannot create ${staging}" >&2; exit 0; }

# Everything into staging first, so a partial download is never what `current` points at.
ok=yes
for name in $DETECTOR_FILES; do
    # shellcheck disable=SC2086  # CURL_OPTS is a deliberate word list
    if ! curl $CURL_OPTS -o "${staging}/${name}" "${DETECTOR_BASE_URL}/${name}"; then
        echo "seed-detector: could not fetch ${name} from ${DETECTOR_BASE_URL}" >&2
        ok=no
        break
    fi
done

if [ "$ok" = no ]; then
    # Nothing partial ever goes live. A board that cannot reach the Hub ends up with no detector,
    # and the pin is retried at the next update.
    rm -rf "$staging"
    echo "seed-detector: leaving the board without a detector for now" >&2
    exit 0
fi

chmod 644 "$staging"/duck_detect.* 2>/dev/null || true

write_source "$staging" "${DETECTOR_VERSION}"

rm -rf "${DETECTOR_ROOT:?}/${target}"
mv "$staging" "${DETECTOR_ROOT}/${target}" \
    || { echo "seed-detector: cannot install into ${target}" >&2; exit 0; }

# `current -> releases/<something>`, relative, swapped rather than rewritten — the policies
# seeder explains each of those; this is the same code.
ln -sfn "$target" "${DETECTOR_ROOT}/current.new" || {
    echo "seed-detector: cannot stage ${DETECTOR_ROOT}/current" >&2
    exit 0
}
if ! mv -T "${DETECTOR_ROOT}/current.new" "${DETECTOR_ROOT}/current" 2>/dev/null; then
    if ! { rm -f "${DETECTOR_ROOT}/current" \
        && mv "${DETECTOR_ROOT}/current.new" "${DETECTOR_ROOT}/current"; }; then
        echo "seed-detector: cannot point ${DETECTOR_ROOT}/current at ${target}" >&2
        exit 0
    fi
fi
echo "seed-detector: installed ${DETECTOR_REPO}@${DETECTOR_VERSION}" >&2
