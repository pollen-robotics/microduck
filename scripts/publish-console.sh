#!/usr/bin/env bash
#
# Publish the console page to the Hugging Face Space that serves it remotely.
#
# The page in this repository is the source (`remote-access-design.md` §5): it tracks the
# signalling protocol and the robot's method names, and a copy living in the Space would drift
# from both. This is the deploy — by hand while there is one Space, by CI when that stops being
# true.
#
# What it substitutes, and what it deliberately does not:
#
#   {{API_VERSION}}     the version this checkout speaks, read from `duck-ipc-proto`, so the page
#                       can tell a person that it and the robot disagree.
#   {{SIGNALLING_PORT}} left alone. The page reads an unsubstituted port as "no robot served me",
#                       which is exactly true here and is what selects the rendezvous transport.
#                       Substituting it would make the page try to open a WebSocket to the Space.
#
# Usage: scripts/publish-console.sh [--space <org/name>] [--dry-run]
#
# Pushing needs a Hugging Face token with write access to the Space. `hf auth login` stores one,
# and git will ask for it otherwise; the token is never read by this script.

set -euo pipefail

SPACE="pollen-robotics/microduck-console"
DRY_RUN=

while [ $# -gt 0 ]; do
    case "$1" in
        --space) SPACE="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
PAGE="$REPO_ROOT/mediad/webclient/index.html"
SPACE_DIR="$REPO_ROOT/mediad/webclient/space"
CARD="$SPACE_DIR/README.md"

for file in "$PAGE" "$CARD" "$SPACE_DIR/Dockerfile" "$SPACE_DIR/entrypoint.sh"; do
    [ -f "$file" ] || { echo "missing: $file" >&2; exit 1; }
done

# One source of truth for the wire version: the constant every daemon compiles against.
API_VERSION=$(sed -n 's/^pub const API_VERSION: u32 = \([0-9]*\);.*/\1/p' \
    "$REPO_ROOT/duck-ipc-proto/src/lib.rs")
[ -n "$API_VERSION" ] || { echo "could not read API_VERSION" >&2; exit 1; }

STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

# Which build this is, logged by the page as its first line. A static host caches and a browser
# caches harder, so "is the fix even loaded" has to be answerable without guessing.
# The revision *and* a hash of the page itself. The revision alone is not enough: a page edited
# but not yet committed publishes under its parent's revision, so two different pages can carry
# the same stamp — which is exactly the confusion this stamp exists to end.
REVISION=$(cd "$REPO_ROOT" && git rev-parse --short HEAD)
PAGE_HASH=$(shasum "$PAGE" | cut -c1-8)
STAMP=$(date -u +%Y-%m-%dT%H:%MZ)
sed -e "s/{{API_VERSION}}/$API_VERSION/g" \
    -e "s|{{CONSOLE_BUILD}}|$REVISION/$PAGE_HASH $STAMP|g" "$PAGE" > "$STAGE/index.html"
cp "$CARD" "$STAGE/README.md"
# The Space is a Docker Space: it serves the page and substitutes its own OAuth client id into it.
cp "$SPACE_DIR/Dockerfile" "$SPACE_DIR/entrypoint.sh" "$STAGE/"

# The client id is substituted by the container at start, not here, so the token must survive
# this staging — and the secret that comes with it must never appear in the page at all.
grep -q '{{OAUTH_CLIENT_ID}}' "$STAGE/index.html" || {
    echo "the OAuth token is gone from the page; the Space could not fill in its client id" >&2
    exit 1
}

grep -q '{{SIGNALLING_PORT}}' "$STAGE/index.html" || {
    echo "the port token is gone from the page; the Space copy would try to open a WebSocket" >&2
    exit 1
}

echo "page:  $(wc -c < "$STAGE/index.html") bytes, api v$API_VERSION"
echo "space: https://huggingface.co/spaces/$SPACE"

if [ -n "$DRY_RUN" ]; then
    echo "--dry-run: staged in $STAGE, nothing pushed"
    trap - EXIT
    exit 0
fi

CLONE="$STAGE/space"
git clone --depth 1 "https://huggingface.co/spaces/$SPACE" "$CLONE"
cp "$STAGE/index.html" "$STAGE/README.md" "$STAGE/Dockerfile" "$STAGE/entrypoint.sh" "$CLONE/"

cd "$CLONE"
if git diff --quiet; then
    echo "the Space already serves this page"
    exit 0
fi

git add index.html README.md Dockerfile entrypoint.sh
git commit -q -m "Console from microduck $REVISION (api v$API_VERSION)"
git push
echo "pushed. The Space rebuilds in a few seconds."
