# The console, served by a Space that can read its own OAuth client id.
#
# **Why a server for a page that needs none.** A static Space is documented to inject
# `window.huggingface.variables.OAUTH_CLIENT_ID` into the page, and for this Space it never did —
# through a rebuild, a privacy flip, a recreation, and a page given a real `<head>` to be injected
# into. A Docker Space takes the other documented path instead: `hf_oauth: true` puts the client id
# in the environment, and eight lines of `sh` put it in the page. That is the mechanism the
# `grabette-*` Spaces use, and it depends on nothing that can silently stop happening.
#
# `docs/design/remote-access-design.md` §5.
FROM python:3.13-slim

# `python -m http.server` is the whole server: one file, GET only, no upload path, no config. The
# alternative was nginx, which is a package, a config file and a user to get wrong for a page.
WORKDIR /app
COPY index.html /app/index.html
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# HF's default for a Docker Space, and `app_port` in the README says the same number.
EXPOSE 7860

ENTRYPOINT ["/app/entrypoint.sh"]
