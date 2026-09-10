# A Docker Space, because FastAPI owns the server here: the robot's frames arrive on a WebSocket
# route of ours, with Gradio mounted underneath it. `remote-access-design.md` §5.0 has the other
# reason a Space in this project ends up as Docker.
#
# **The user is `user` at `/home/user`, and that is not a preference.** Hugging Face appends its
# own layers to a Docker Space's build — a dev-mode init process, an openvscode-server, a
# `git config --global` — and they assume the canonical uid-1000 `user` with a writable
# `/home/user`. Naming it `duck` and putting `HOME` elsewhere built fine locally and failed in
# their builder with `exit code 1` under a page of cache-miss noise, which names no step at all.
FROM python:3.12-slim

# As root, before dropping privileges.
#
# `libgl1` and `libglib2.0-0` are what `opencv-python-headless` still links against — headless
# drops the GUI toolkit, not the whole of GL.
#
# **`git` is for Hugging Face, not for us.** Their builder appends layers to every Docker Space,
# and one of them is `git config --global user.email …`. A `-slim` base has no git, so that layer
# dies with `exit code: 127` *after* the whole image has built successfully — and the error the
# API reports for it is a page of cache-miss noise naming one of their own steps, which is not
# something anybody guesses from. This line is the fix.
RUN apt-get update \
 && apt-get install -y --no-install-recommends git libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user \
    PATH=/home/user/.local/bin:$PATH \
    PYTHONUNBUFFERED=1

# Ruled out explicitly rather than left to the environment, and on its own line because a comment
# inside a line continuation is not something to gamble a build on. Gradio's server-side rendering
# starts a **Node** server of its own and this image has no Node, so a platform that sets
# `GRADIO_SSR_MODE=true` would have `mount_gradio_app` try to launch one. SSR buys SEO for a page
# that sits behind a sign-in, which is nothing worth a second process for.
ENV GRADIO_SSR_MODE=false

# **`SYSTEM=spaces`, and this one line is what four rounds of debugging were about.**
#
# Gradio decides whether it is running on a Space with `get_space()`, which is
# `os.getenv("SYSTEM") == "spaces"` and then `SPACE_ID` — *not* `SPACE_ID` alone. Hugging Face
# sets `SYSTEM=spaces` for gradio-SDK Spaces and **not** for Docker ones, so on a Docker Space
# Gradio concludes it is on somebody's laptop: `attach_oauth` installs its *mocked* login routes,
# and the mock calls `_get_mocked_oauth_info()`, which raises without a local `hf auth login`.
# A container has none, so the module fails at import, uvicorn never binds, and the platform
# reports "App process crashed" — while the UI you can see for a second is the previous container
# still serving.
#
# `SPACE_ID` and `OAUTH_CLIENT_ID` were both set the whole time. The only thing missing was this.
ENV SYSTEM=spaces
WORKDIR $HOME/app

# Requirements first, so a change to the app does not reinstall `av` and OpenCV.
COPY --chown=user requirements.txt $HOME/app/
RUN pip install --no-cache-dir --user -r requirements.txt

COPY --chown=user . $HOME/app

# `app_port` in the README metadata has to agree with this.
EXPOSE 7860

# **`boot.py`, which serves the app or serves the reason it would not import.** A container that
# exits is reported as "App process crashed" with a truncated first line, and reading a Space's own
# logs needs write access to it — so a broken import cost a round trip through a pasted traceback
# every time. This way the Space stays up and `curl /` answers the question. Its header has the
# reasoning; `app.py`'s `__main__` block stays for laptops.
CMD ["python", "boot.py"]
