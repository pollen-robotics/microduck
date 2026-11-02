"""Tell the duck what to find. A small vision model looks through its eyes and says what it sees.

    pip install -e "sdk[video]" transformers torch pillow
    python examples/find_it.py "orange ball" "blue cube" [--duck name]

For each thing asked for, the duck looks left, ahead and right. At each, SmolVLM (256M
parameters, it runs on a laptop) says what it sees and whether it is the thing asked for, and the
duck chirps when it says yes.
"""

import argparse
import threading
import time

import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

from microduck import Duck

MODEL = "HuggingFaceTB/SmolVLM-256M-Instruct"
WHAT = "What object is in this picture? Answer with its colour and shape, in a few words."

# Trunk-frame points, metres: to the left, ahead and to the right.
GAZES = {"left": (1.0, 0.8, 0.1), "ahead": (1.0, 0.0, 0.1), "right": (1.0, -0.8, 0.1)}

device = "cuda" if torch.cuda.is_available() else "cpu"
device = "mps" if torch.backends.mps.is_available() else device
processor = model = None


def load() -> None:
    """Here and not on import, so `--help` does not download the model."""
    global processor, model
    processor = AutoProcessor.from_pretrained(MODEL)
    model = AutoModelForImageTextToText.from_pretrained(MODEL).to(device)


def ask(picture, question: str) -> str:
    content = [{"type": "image"}, {"type": "text", "text": question}]
    messages = [{"role": "user", "content": content}]
    prompt = processor.apply_chat_template(messages, add_generation_prompt=True)
    inputs = processor(text=prompt, images=[Image.fromarray(picture)], return_tensors="pt")
    out = model.generate(**inputs.to(device), max_new_tokens=20, do_sample=False)
    answer = out[:, inputs["input_ids"].shape[1] :]
    return processor.batch_decode(answer, skip_special_tokens=True)[0].strip()


def hold(duck, gaze: dict) -> None:
    """The robot's idle head starts looking around 2 s after the last look, and the model is
    slower than that, so the look is sent again while it thinks."""
    while not gaze["done"]:
        with gaze["lock"]:
            point = gaze["point"]
            if point and not gaze["done"]:
                try:
                    duck.look(*point)
                except Exception as e:  # noqa: BLE001, one missed look is no reason to stop
                    print(f"could not hold the look: {e}")
        time.sleep(0.5)


def turn(duck, gaze: dict, point) -> None:
    """Look somewhere new. The holder waits until the head is there, or the picture is taken
    halfway, and the lock keeps a look it still has on the way from landing after this one."""
    with gaze["lock"]:
        gaze["point"] = None
        duck.look(*point)
    # Long enough for the head to get there and a picture taken after it did.
    time.sleep(1.5)


def find(duck, gaze: dict, target: str) -> None:
    print(f"> find the {target}")
    for where, point in GAZES.items():
        turn(duck, gaze, point)
        gaze["point"] = point
        picture = duck.frame()
        if picture is None:
            print("the video went away")
            return
        what = ask(picture, WHAT)
        match = ask(picture, f"Is there a {target} in this picture? Answer yes or no.")
        print(f"{where}: {what} {target}? {match}")
        if match.lower().startswith("yes"):
            duck.call("robot.sound", {"tag": "chirp"})
            print(f"found the {target}, {where}")
            return
    print(f"no {target} in sight")


def main(targets: list[str], name: str | None) -> None:
    load()
    with Duck(name, video=True) as duck:
        duck.frame()
        gaze = {"point": None, "done": False, "lock": threading.Lock()}
        holder = threading.Thread(target=hold, args=(duck, gaze), daemon=True)
        holder.start()
        for target in targets:
            find(duck, gaze, target)
            time.sleep(2.5)
            turn(duck, gaze, GAZES["ahead"])
        gaze["done"] = True
        holder.join()  # its last look lands before the duck closes


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tell the duck what to find.")
    parser.add_argument("targets", nargs="*", default=["orange ball"], help="what to find")
    parser.add_argument("--duck", help="which duck, when the account has more than one")
    args = parser.parse_args()
    main(args.targets, args.duck)
