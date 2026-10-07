#!/bin/sh
# Install the official policy set, downloading it from the Hugging Face Hub.
#
# `robotd` reads its policies from /opt/robot/policies/current, deliberately outside the release:
# a gait retrain should not need a daemon release, and a daemon fix should not re-ship six
# megabytes of unchanged weights (docs/design/policy-channel-design.md §9). This is what fills
# that directory, and it fetches rather than copies — the same arrangement `setup-board.sh` uses
# for ONNX Runtime and `setup-gstreamer.sh` for the plugins, which are the other two things a
# board needs and a release has no business carrying.
#
# Run by `hooks/postinstall` on every update and by `scripts/install.sh` on a fresh board, which
# between them are every way policies reach a robot.
#
# THE RULE THAT MATTERS: never touch a set that is not ours, and never move ours backwards. A
# set carries a `.source` record; one from another repo, or without a record, was put there by
# something else and replacing it would silently undo that on the next unrelated daemon update.
# One from our repo is moved up to the pin when it is *below* it — the daemon's defaults name
# files a newer set carries, and a board that updates the daemon but keeps an old set would
# otherwise fail to load its gait and roll the update back. An incomplete seed at the pin is
# retried too: a fallback download is not evidence that the revision's whole set arrived.
#
# Nothing here is signed and that is deliberate: a policy is not a binary. `robotd` holds the only
# write handle to the bus behind joint clamps, a fall reflex and an intent deadman, and refuses
# any graph that is not obs[1,61] -> actions[1,14] while the robot is standing still. That gate is
# also what catches a truncated download, which is why there are no hashes pinned here to go stale
# on every retrain.
#
# Never fatal. A robot with no policy holds its pose and reports *degraded* — a board to fix, not
# a release to roll back.
#
# A board that cannot reach the Hub on a first install ends up with no policies, and that is the
# accepted shape rather than an oversight: `robotd` holds its pose and reports *degraded*, the
# update gate passes, and the next update fetches. It is the same bargain `setup-board.sh` makes
# for ONNX Runtime — the board prerequisites need a network once.
#
# Usage: seed-policies.sh [POLICY_ROOT]
# Defaults to what a robot uses; the argument exists so this can be tested off a board.
set -eu

POLICY_ROOT="${1:-/opt/robot/policies}"

# The pin. An xtask test asserts these literals match `[workspace.metadata.policies]` in
# Cargo.toml — this script runs from inside a release and cannot read the manifest, which is the
# same reason `setup-gstreamer.sh` carries its plugin version as a literal.
#
# A minimum, not a ceiling: it decides what a board installs when it has *nothing*, and the
# oldest official set this daemon runs with. A board below it is moved up to it on update; a
# board past it — `robotctl policy update`, which needs no daemon release — is left where it is.
# Bumping this ships inside a release, so bump it when the daemon's defaults need the new set
# (a slot defaulting to a file only the new set carries), and tag the set first.
POLICY_REPO="${POLICY_REPO:-pollen-robotics/microduck-policies}"
POLICY_VERSION="${POLICY_VERSION:-v7}"
POLICY_BASE_URL="${POLICY_BASE_URL:-https://huggingface.co/${POLICY_REPO}/resolve/${POLICY_VERSION}}"

# The set says what is in it. `manifest.json` lists every policy, and that list is what gets
# downloaded — so adding a tenth policy to the set is a tag on the Hub rather than an edit here
# and a daemon release to carry it.
#
# The fallback below is what robotd's own defaults name (both modes), for a revision tagged
# before the manifest did. It goes when every tagged set carries one.
#
# The grep is over a file we generate, so its shape is ours: one `"file": "name.onnx"` per policy.
# A JSON parser is not available here — this runs from a release on a board with curl and a
# POSIX shell — and an xtask test asserts the pattern matches what the manifest actually says.
FALLBACK_FILES="velstand.onnx alpha_sitstand.onnx alpha_ground_pick.onnx ball_kick_left.onnx ball_kick_right.onnx roller.onnx roller_crouch.onnx roulade.onnx"

# Per-file, because `hooks/postinstall` runs inside an update under a 120-second hook timeout and
# a hook that times out fails the update and rolls it back. Ten files at eight seconds is 80,
# which leaves the rest of the hook room; a link that cannot move 800 KB in eight seconds is one
# the fallback below is for, and the next update tries again.
CURL_OPTS="--fail --location --silent --show-error --connect-timeout 5 --max-time 8"

# Where a set came from, written beside it.
#
# It is what lets `robotctl policy check` ask the Hub whether there is anything newer without
# anybody configuring the repo a second time. One writer, one copy, no drift — and a set that a
# different tool installs later carries its own, so "what is this and where is it from" has the
# same answer however it arrived.
#
# Only written when missing, so an update does not rewrite the file just to change a timestamp.
write_source() {
    [ ! -f "$1/.source" ] || return 0
    {
        echo "repo=${POLICY_REPO}"
        echo "version=${2}"
        echo "fetched=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    } > "$1/.source" || echo "seed-policies: cannot record where this set came from" >&2
}

target="releases/seed-${POLICY_VERSION}"
live="$(readlink "${POLICY_ROOT}/current" 2>/dev/null || true)"

# **An installed set is never moved backwards, and only ours is moved at all.** The pin is a
# minimum — what a board with nothing gets, and the oldest set this daemon's defaults load —
# and not a ceiling, which is the distinction that makes `robotctl policy update` possible.
#
# This used to replace any older `seed-*` on the reasoning that a daemon update was still how a
# retrained gait reached a board. `policy update` became how, and that rule was a trap: a board
# moved forward to v2 by hand had `current -> releases/seed-v2`, and the next unrelated daemon
# update quietly put v1 back. Then it never replaced anything, and that was the opposite trap:
# when the daemon's default gait became a file only v5 carries, a board on v4 updating the
# daemon could not load its walk, reported unhealthy, and rolled the release back.
#
# So the version decides. An official set below the pin moves up; an incomplete seed at the
# pin is repaired. A newer set, a set from another repo, or an explicitly nonnumeric version
# is left alone. A board seeded before the record existed has it
# back-filled from the directory name, because `policy check` cannot ask about a set that will
# not say where it came from — and so that this comparison can read it next time.
number_of() {
    # "v5" -> 5; anything that is not a plain vN is "" and never compares.
    case "$1" in
        v[0-9]|v[0-9][0-9]|v[0-9][0-9][0-9]) echo "${1#v}" ;;
        *) echo "" ;;
    esac
}
manifest_files() {
    # An unreadable manifest is unavailable, not a failed daemon install.
    sed -n 's/.*"file"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$1" || true
}
complete_set() {
    [ -f "$1/manifest.json" ] || return 1
    files="$(manifest_files "$1/manifest.json")"
    [ -n "$files" ] || return 1
    for name in $files; do
        case "$name" in */*|*\\*|.*) continue ;; esac
        [ -s "$1/$name" ] || return 1
    done
}
repair=no
if [ -n "$live" ]; then
    case "$live" in
        releases/*)
            seed_version="${live#releases/seed-}"
            write_source "${POLICY_ROOT}/${live}" "${seed_version%-repair}" ;;
        *)
            echo "seed-policies: ${POLICY_ROOT}/current is not ours; leaving it alone" >&2
            exit 0 ;;
    esac
    record="${POLICY_ROOT}/${live}/.source"
    have_repo="$(sed -n 's/^repo=//p' "$record" 2>/dev/null | head -n 1)"
    have_version="$(sed -n 's/^version=//p' "$record" 2>/dev/null | head -n 1)"
    # Missing versions fall back to the seed name. Tool directories retain their releases/
    # prefix, which number_of refuses; an explicit custom version still takes precedence.
    have_version="${have_version:-${seed_version%-repair}}"
    have="$(number_of "$have_version")"
    want="$(number_of "$POLICY_VERSION")"
    if [ "$have_repo" != "$POLICY_REPO" ] || [ -z "$have" ] || [ -z "$want" ] \
        || [ "$have" -gt "$want" ]; then
        exit 0
    fi
    if [ "$have" -eq "$want" ]; then
        case "$live" in
            "$target"|"$target-repair") ;;
            *) exit 0 ;;
        esac
        complete_set "${POLICY_ROOT}/${live}" && exit 0
        repair=yes
        echo "seed-policies: ${have_version} seed is incomplete; retrying its manifest" >&2
    else
        echo "seed-policies: ${have_version} is below the ${POLICY_VERSION} this daemon needs; moving up" >&2
    fi
fi

staging="${POLICY_ROOT}/releases/.staging"
rm -rf "$staging"
mkdir -p "$staging" || { echo "seed-policies: cannot create ${staging}" >&2; exit 0; }

# Everything into staging first, so a partial download is never what `current` points at.
ok=yes

# The manifest, and the file list from it. Fetched into staging like everything else, so it is
# installed beside the policies it describes and `robotd` can read what it says.
# shellcheck disable=SC2086
if manifest_status=$(curl $CURL_OPTS --write-out '%{http_code}' \
    -o "${staging}/manifest.json" "${POLICY_BASE_URL}/manifest.json"); then
    POLICY_FILES="$(manifest_files "${staging}/manifest.json")"
else
    if [ "$manifest_status" = 404 ]; then
        echo "seed-policies: ${POLICY_VERSION} has no manifest (HTTP 404)" >&2
    else
        echo "seed-policies: could not fetch the manifest for ${POLICY_VERSION}; will retry" >&2
    fi
    POLICY_FILES=""
fi
if [ -z "$POLICY_FILES" ]; then
    # A legacy tag's 404 and a temporary failure both allow a first fallback install, but
    # neither proves completeness. On a repair, keep the working fallback until a manifest
    # and every file it names arrive. No persistent marker is needed: its absence is enough.
    if [ "$repair" = yes ]; then
        rm -rf "$staging"
        echo "seed-policies: manifest unavailable; keeping the installed set for a later retry" >&2
        exit 0
    fi
    rm -f "${staging}/manifest.json"
    echo "seed-policies: taking the set this release knows" >&2
    POLICY_FILES="$FALLBACK_FILES"
fi

for name in $POLICY_FILES; do
    # A plain file name, or nothing. The manifest is fetched from a repo this script's caller
    # can point anywhere, and `${staging}/${name}` would otherwise let one naming `../../etc/…`
    # choose where the download lands. `updater::policy::files_in_manifest` applies the same rule
    # to the same field.
    case "$name" in
        */*|*\\*|.*)
            echo "seed-policies: ignoring ${name} — a policy is a file name" >&2
            continue ;;
    esac
    # shellcheck disable=SC2086  # CURL_OPTS is a deliberate word list
    if ! curl $CURL_OPTS -o "${staging}/${name}" "${POLICY_BASE_URL}/${name}"; then
        echo "seed-policies: could not fetch ${name} from ${POLICY_BASE_URL}" >&2
        ok=no
        break
    fi
done

if [ "$ok" = no ]; then
    # Nothing partial ever goes live, and nothing already installed is disturbed. A half-published
    # revision or a link that was down leaves the board exactly as it was — on the previous set if
    # it has one, with none if it does not — and the pin is retried at the next update. A board
    # below the minimum that stays there is one whose daemon cannot load its default gait; robotd
    # then reports unhealthy and the update rolls back, which is the honest outcome and the
    # message above is the reason.
    rm -rf "$staging"
    echo "seed-policies: leaving the policies already installed alone" >&2
    exit 0
fi

chmod 644 "$staging"/*.onnx 2>/dev/null || true

write_source "$staging" "${POLICY_VERSION}"

# A same-version repair must not remove the directory `current` still points at. Alternate
# between two seed destinations so the predecessor survives until the complete set is linked.
if [ "$live" = "$target" ]; then
    target="$target-repair"
fi
rm -rf "${POLICY_ROOT:?}/${target}"
mv "$staging" "${POLICY_ROOT}/${target}" \
    || { echo "seed-policies: cannot install into ${target}" >&2; exit 0; }

# `current -> releases/<something>`, relative to the directory the link is in, which is the layout
# the updater already uses and swaps (docs/design/updater-design.md §7.1). Relative and not
# absolute so the link keeps working wherever the root is — an absolute target silently resolves
# against the wrong directory the moment POLICY_ROOT is not what it was written with.
#
# Swapped rather than rewritten: a half-written `current` is one a restarting robotd could read.
# `mv -T` is the atomic form and is GNU-only; the fallback is a remove-and-relink, a smaller
# window rather than none. `rm -f` and not `rm -rf`, so a `current` that is somehow a real
# directory fails here instead of being deleted — the case above should have caught it, and if it
# did not, refusing is the right way to be wrong.
ln -sfn "$target" "${POLICY_ROOT}/current.new" || {
    echo "seed-policies: cannot stage ${POLICY_ROOT}/current" >&2
    exit 0
}
if ! mv -T "${POLICY_ROOT}/current.new" "${POLICY_ROOT}/current" 2>/dev/null; then
    if ! { rm -f "${POLICY_ROOT}/current" \
        && mv "${POLICY_ROOT}/current.new" "${POLICY_ROOT}/current"; }; then
        echo "seed-policies: cannot point ${POLICY_ROOT}/current at ${target}" >&2
        exit 0
    fi
fi
