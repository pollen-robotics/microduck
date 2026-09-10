---
title: hello duck
emoji: 🦆
colorFrom: yellow
colorTo: indigo
sdk: docker
app_port: 7860
pinned: false
short_description: Smallest Docker Space that has a WebSocket route and Gradio.
---

# hello duck

A deliberately minimal Docker Space, published to find out which of four things breaks the real
one: the container, a custom WebSocket route, Gradio mounted under FastAPI, or Hugging Face OAuth.

It has the first three and **not** OAuth, which is where the last crash was. Grow it back one
piece at a time rather than debugging four at once.
