"""A small vision model looks through the duck's eyes and finds the orange ball.

    pip install -e "sdk[video]" transformers torch pillow
    python examples/find_the_ball.py [duck-name]

The duck looks left, ahead and right. At each, SmolVLM (256M parameters, it runs on a laptop) is
asked what it sees and whether the ball is there, and the duck keeps looking where it said yes.
"""

import sys
import time

import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

from duck import Duck

MODEL = "HuggingFaceTB/SmolVLM-256M-Instruct"

# Trunk-frame points a metre out, metres: left, ahead and right.
GAZES = {"left": (1.0, 0.8, 0.1), "ahead": (1.0, 0.0, 0.1), "right": (1.0, -0.8, 0.1)}

device = "cuda" if torch.cuda.is_available() else "cpu"
device = "mps" if torch.backends.mps.is_available() else device
processor = AutoProcessor.from_pretrained(MODEL)
model = AutoModelForImageTextToText.from_pretrained(MODEL).to(device)


def ask(picture, question: str) -> str:
    content = [{"type": "image"}, {"type": "text", "text": question}]
    messages = [{"role": "user", "content": content}]
    prompt = processor.apply_chat_template(messages, add_generation_prompt=True)
    inputs = processor(text=prompt, images=[Image.fromarray(picture)], return_tensors="pt")
    out = model.generate(**inputs.to(device), max_new_tokens=30, do_sample=False)
    answer = out[:, inputs["input_ids"].shape[1] :]
    return processor.batch_decode(answer, skip_special_tokens=True)[0].strip()


def main(name: str | None) -> None:
    with Duck(name, video=True) as duck:
        duck.frame()
        for where, point in GAZES.items():
            duck.look(*point)
            # Long enough for the head to get there and a picture taken after it did.
            time.sleep(1.5)
            picture = duck.frame()
            seen = ask(picture, "Describe what is in front of you in one short sentence.")
            ball = ask(picture, "Is there an orange ball in this picture? Answer yes or no.")
            print(f"{where}: {seen}")
            if ball.lower().startswith("yes"):
                duck.call("robot.sound", {"tag": "chirp"})
                print(f"found it, {where}")
                return
        print("no ball in sight")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
