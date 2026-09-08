# The demo, in an environment it chooses.
#
# **Docker rather than `sdk: gradio`, and not by preference.** A Gradio Space installs
# `gradio[oauth,mcp]==6.26.0` of its own accord, and that exact version cannot coexist with the
# consumer this demo is built on:
#
#     gradio 6.26   needs starlette >=1.0.1 and huggingface-hub >=1.16
#     reachy-mini   needs starlette <1.0.0            (every version, including 1.10)
#
# pip says `ResolutionImpossible` and it is right. Gradio 5.23 resolves with the same consumer in
# 117 packages, so the version is the thing to control — which a Docker Space can and a Gradio
# Space cannot. The console Space reached `sdk: docker` by a different route
# (`remote-access-design.md` §5.0); this is the second time the managed environment was the
# obstacle rather than the shortcut.
FROM python:3.12-slim

# A user with a writable home, because Gradio and `huggingface_hub` both cache under `$HOME` and
# a root-owned container on this platform does not get one.
RUN useradd --create-home --uid 1000 duck
WORKDIR /app

# Requirements first, so a change to the app does not reinstall aiortc and av — the two slow ones.
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# **Every module, not a list of them.** This said `COPY app.py filters.py ./`, and the day a third
# module appeared the image built perfectly and the app died on `import consumer` — a hand-kept
# list of files that has to match reality is the same mistake as `cp *` in the publisher and
# `git diff --quiet` in its check, three times in one afternoon. `publish-space.sh` sends only
# tracked files, so a glob here cannot pick up anything that is not meant to be deployed.
COPY *.py ./
USER duck
ENV HOME=/home/duck \
    GRADIO_SERVER_NAME=0.0.0.0 \
    GRADIO_SERVER_PORT=7860

EXPOSE 7860
CMD ["python", "app.py"]
