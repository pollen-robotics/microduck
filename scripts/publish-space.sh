#!/usr/bin/env bash
#
# Publish one of `spaces/` to the Hugging Face Space that serves it.
#
# The source lives in this repository for the reason `remote-access-design.md` §5 gives about the
# console: a Space consuming a duck tracks things that live here — the rendezvous protocol, the
# robot's own method names, the camera's geometry — and a copy in a Space repo drifts from all of
# them. This is the deploy, by hand while there are two of them.
#
# Usage:
#   scripts/publish-space.sh vision-demo
#   scripts/publish-space.sh vision-demo --space pollen-robotics/other-name --dry-run
#
# Pushing needs a Hugging Face token with write access. `hf auth login` stores one and git will
# ask otherwise; this script never reads it.

set -euo pipefail

NAME="${1:-}"
[ -n "$NAME" ] || { echo "usage: $0 <directory under spaces/> [--space id] [--dry-run]" >&2; exit 2; }
shift

SPACE="pollen-robotics/microduck-$NAME"
DRY_RUN=

while [ $# -gt 0 ]; do
    case "$1" in
        --space) SPACE="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
SOURCE="$REPO_ROOT/spaces/$NAME"
[ -d "$SOURCE" ] || { echo "no such space source: $SOURCE" >&2; exit 1; }
[ -f "$SOURCE/README.md" ] || { echo "$NAME has no README.md, which is its Space card" >&2; exit 1; }

STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

# **What git has, not what the directory has**, and that is the whole of the copy below.
#
# `cp *` published whatever was lying around — and something always is: running the app from its
# own directory leaves a `__pycache__` next to it, which is a directory, which `cp` without `-r`
# refuses, which stopped the script halfway. Listing tracked files fixes both halves: bytecode and
# scratch files cannot reach a Space, and a Space always corresponds to a commit in this
# repository. A file that has not been committed does not deploy, which is a feature the first
# time somebody wonders which version is live.
FILES=$(cd "$REPO_ROOT" && git ls-files "spaces/$NAME" | sed "s|^spaces/$NAME/||")
[ -n "$FILES" ] || { echo "no tracked files under spaces/$NAME — commit them first" >&2; exit 1; }

echo "space:  https://huggingface.co/spaces/$SPACE"
echo "files:  $(echo "$FILES" | tr '\n' ' ')"

if [ -n "$DRY_RUN" ]; then
    echo "--dry-run: nothing pushed"
    exit 0
fi

CLONE="$STAGE/space"
git clone --depth 1 "https://huggingface.co/spaces/$SPACE" "$CLONE"

# Copied rather than synced: a file deleted here stays in the Space until somebody removes it
# there. Deliberate — a `--delete` that ran against the wrong Space id would remove somebody's
# work, and these are hand-run.
#
# One at a time and by name, so a subdirectory arrives as a subdirectory rather than as an error.
for file in $FILES; do
    mkdir -p "$CLONE/$(dirname "$file")"
    cp "$SOURCE/$file" "$CLONE/$file"
done

cd "$CLONE"

# **Staged first, then compared.** `git diff --quiet` ignores untracked files, so a publish whose
# only change was a *new* file reported "already serves this" and pushed nothing — which is the
# worst possible answer, because it is indistinguishable from success. `git add -A` and then a
# cached diff sees additions, deletions and modifications alike.
git add -A
if git diff --cached --quiet; then
    echo "the Space already serves this"
    exit 0
fi

REVISION=$(cd "$REPO_ROOT" && git rev-parse --short HEAD)
git commit -q -m "$NAME from microduck $REVISION"
git push
echo "pushed. The Space rebuilds: a couple of minutes for a Docker one, since aiortc and av"
echo "        are wheels worth waiting for. Its build log is on the Space page."
