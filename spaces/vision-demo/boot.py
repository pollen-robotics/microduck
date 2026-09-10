"""Start the app — and when it cannot start, **serve the reason instead of exiting**.

A container that exits is reported by Hugging Face as "App process crashed", and the only detail
that reaches its API is a truncated first line: four rounds of this went into the port before a
pasted traceback showed the real fault was an import-time `ValueError` from Gradio's OAuth setup.
Reading a Space's own logs needs write access to the Space, so every question cost a round trip.

So: import the app inside a `try`. If it works, serve it. If it does not, serve a page that *is*
the traceback, on the port the platform expects. The Space stays up, `curl` answers the question,
and nobody has to paste a log again.
"""

from __future__ import annotations

import os
import traceback

PORT = int(os.environ.get("PORT", 7860))

try:
    from app import app as application

    FAILURE: str | None = None
except Exception:  # noqa: BLE001 - the whole point is that any failure becomes a page
    FAILURE = traceback.format_exc()
    print("the app did not import:\n" + FAILURE, flush=True)

    from fastapi import FastAPI
    from fastapi.responses import PlainTextResponse

    application = FastAPI()

    @application.get("/", response_class=PlainTextResponse)
    @application.get("/env", response_class=PlainTextResponse)
    def why() -> str:
        watched = (
            "SPACE_ID",
            "SPACE_HOST",
            "OAUTH_CLIENT_ID",
            "OAUTH_SCOPES",
            "HF_TOKEN",
            "GRADIO_SSR_MODE",
            "PORT",
        )
        shown = ("SPACE_ID", "SPACE_HOST", "GRADIO_SSR_MODE", "PORT")
        env = "\n".join(
            f"  {name} = "
            + ((os.environ[name] if name in shown else "set") if name in os.environ else "absent")
            for name in watched
        )
        return (
            "This Space is running, and its app did not import.\n\n"
            f"environment:\n{env}\n\nthe traceback:\n\n{FAILURE}"
        )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(application, host="0.0.0.0", port=PORT)
