#!/bin/sh
# Compose release notes: why this release exists, then what changed in it.
#
# Called by `_build-release.yml` and `_promote-release.yml`, so the two cannot drift into describing
# releases differently — and because the interesting half of the problem is the same for both.
#
# The problem it solves: a promoted release said only "Promoted from daemon-staging-v0.4.0", and the
# changelog lived on the staging release, which promotion deletes. So the stable release — the one
# anybody actually reads — ended up with no account of what was in it.
#
# The changelog comes from GitHub's own generator rather than from a file in the repo. That is
# deliberate: a hand-maintained CHANGELOG.md is a second place to forget, and the generator already
# knows the merged pull requests, which is what this project's history is made of.
#
# Reads:  TAG          the tag being published
#         PROVENANCE   a sentence or two on where these bytes came from
#         GH_TOKEN     needed by `gh`
# Writes: the composed notes, on stdout.
set -eu

: "${TAG:?TAG is required}"
: "${PROVENANCE:?PROVENANCE is required}"

# The previous *stable* release, which is the sensible left edge of a changelog for either channel: a
# candidate answers "what is new since the last release people are running", and so does the release
# it becomes.
#
# Explicit rather than letting the generator choose, because it would otherwise pick the most recent
# release of any kind — and `dev.yml` publishes a prerelease on every push to every branch, so
# "since the last release" would usually mean "since twenty minutes ago".
previous="$(gh release list --limit 100 --json tagName,isPrerelease,isDraft \
    --jq '[.[] | select(.isPrerelease == false and .isDraft == false) | .tagName]
          | map(select(startswith("daemon-v")))
          | first // empty' 2>/dev/null || true)"

# Not fatal if it is missing or fails. Notes without a changelog are worse than notes with one and
# better than a failed release: this runs after the artifact is signed and verified.
changelog=""
if [ -n "$previous" ] && [ "$previous" != "$TAG" ]; then
    changelog="$(gh api "repos/${GITHUB_REPOSITORY}/releases/generate-notes" \
        -f "tag_name=${TAG}" \
        -f "previous_tag_name=${previous}" \
        --jq .body 2>/dev/null || true)"
else
    # No previous stable release, or this is it. The generator handles that on its own.
    changelog="$(gh api "repos/${GITHUB_REPOSITORY}/releases/generate-notes" \
        -f "tag_name=${TAG}" \
        --jq .body 2>/dev/null || true)"
fi

printf '%s\n' "$PROVENANCE"
if [ -n "$changelog" ]; then
    printf '\n---\n\n%s\n' "$changelog"
else
    printf '\n_No changelog could be generated for this release._\n'
fi
