"""The smallest thing that is still the shape of the real Space.

FastAPI owns the server, one WebSocket route of our own, Gradio mounted at `/`. **No OAuth**,
because that is where the real app died: a `gr.LoginButton` makes Gradio call `attach_oauth` while
the Blocks is built, which mocks when `SPACE_ID` is absent, and the mock raises without a local
`hf auth login`. A container has none.
"""

import os

import gradio as gr
from fastapi import FastAPI, WebSocket

app = FastAPI()

print(
    "environment: SPACE_ID=%s SPACE_HOST=%s OAUTH_CLIENT_ID=%s"
    % (
        os.environ.get("SPACE_ID") or "absent",
        os.environ.get("SPACE_HOST") or "absent",
        "set" if os.environ.get("OAUTH_CLIENT_ID") else "absent",
    ),
    flush=True,
)


@app.get("/env")
def env() -> dict:
    """Which of the platform's variables actually arrived.

    An endpoint rather than a log line, because reading a Space's logs needs write access to the
    Space and every question about them has cost a round trip through somebody pasting one. This
    reports presence, never a value — `OAUTH_CLIENT_SECRET` is in this environment too.
    """
    watched = (
        "SPACE_ID",
        "SPACE_HOST",
        "SPACE_AUTHOR_NAME",
        "OAUTH_CLIENT_ID",
        "OAUTH_SCOPES",
        "OPENID_PROVIDER_URL",
        "HF_TOKEN",
        "GRADIO_SSR_MODE",
        "PORT",
    )
    return {
        name: (os.environ[name] if name in ("SPACE_ID", "SPACE_HOST", "GRADIO_SSR_MODE", "PORT")
               else "set")
        if name in os.environ
        else "absent"
        for name in watched
    }


@app.websocket("/frames")
async def frames(socket: WebSocket) -> None:
    """A route of our own, which is the whole reason this is Docker and not the gradio SDK."""
    await socket.accept()
    await socket.send_text("hello from /frames")
    await socket.close()


with gr.Blocks(title="hello duck") as demo:
    gr.Markdown(
        """
        # hello duck

        A Docker Space where FastAPI owns the server and Gradio is mounted at `/`.
        `wss://<this space>/frames` is a WebSocket route of our own — it answers with one line
        and hangs up.
        """
    )
    gr.Textbox(value="it runs", label="status", interactive=False)

# `ssr_mode=False`: server-side rendering starts a Node server and this image has no Node.
app = gr.mount_gradio_app(app, demo, path="/", ssr_mode=False)
