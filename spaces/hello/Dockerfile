FROM python:3.12-slim

# `git` because Hugging Face appends a `git config --global` layer to every Docker Space, and a
# `-slim` base has none — that layer then fails with exit 127 after the image has built fine.
RUN apt-get update \
 && apt-get install -y --no-install-recommends git \
 && rm -rf /var/lib/apt/lists/*

# The canonical uid-1000 `user` at `/home/user`: the platform's appended layers assume both.
RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user \
    PATH=/home/user/.local/bin:$PATH \
    PYTHONUNBUFFERED=1
WORKDIR $HOME/app

COPY --chown=user requirements.txt $HOME/app/
RUN pip install --no-cache-dir --user -r requirements.txt
COPY --chown=user . $HOME/app

EXPOSE 7860
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "7860"]
