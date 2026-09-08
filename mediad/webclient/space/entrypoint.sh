#!/bin/sh
#
# Put the Space's OAuth client id in the page, then serve it.
#
# `OAUTH_CLIENT_ID` is what `hf_oauth: true` in the README puts in this container's environment.
# It is public — it identifies the app, it does not authorise anything — which is why it can be
# baked into a page that anybody can read. `OAUTH_CLIENT_SECRET` is also in this environment and
# must never go anywhere near the page: the browser flow is PKCE and needs no secret.
set -eu

if [ -z "${OAUTH_CLIENT_ID:-}" ]; then
    # Not fatal: the page says so itself, and a console that serves and explains beats one that
    # will not start. The likeliest cause is `hf_oauth: true` missing from the README.
    echo "no OAUTH_CLIENT_ID in the environment; the page will not be able to sign anybody in" >&2
fi

mkdir -p /srv
sed "s|{{OAUTH_CLIENT_ID}}|${OAUTH_CLIENT_ID:-}|g" /app/index.html > /srv/index.html

echo "serving the console on 7860 (oauth client ${OAUTH_CLIENT_ID:-none})"
exec python3 -m http.server 7860 --directory /srv
