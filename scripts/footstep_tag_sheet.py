"""Printable A4 sheets for `robotctl robot goto tag ID`: an AprilTag and the footprints to stand on, 1:1.

    uv run --with opencv-contrib-python-headless --with matplotlib scripts/footstep_tag_sheet.py [IDS...] [--out DIR]

One page per tag id (default 0-4) in `<out>/footstep_tags.pdf`. PRINT AT 100 % (actual size): the 100 mm bar
checks it, and the tag's black square must measure TAG_SIZE_M. Lay the sheet flat on the floor; the robot walks
onto the footprints facing the tag.

The numbers are `robotctl/src/tag.rs`'s — the robot computes where to stand from them, so they must agree:
"""

import argparse
from pathlib import Path

import cv2
from matplotlib import patches
from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

TAG_SIZE_M = 0.100  # robotctl/src/tag.rs TAG_SIZE_M: the black square, edge to edge
STANCE_FROM_TAG_M = 0.130  # robotctl/src/tag.rs STANCE_FROM_TAG_M: tag centre → stance centre
FEET_SPACING_M = 0.084  # sole centres apart, standing (home-pose FK 83.6 mm)
SOLE = (0.054, 0.041)  # sole length, width (microduck_rl footsteps.py)
A4 = (210.0, 297.0)  # mm
TAG_CENTRE_MM = (A4[0] / 2, 205.0)  # on the page, from the bottom-left


def page(pdf, tag_id):
    fig = plt.figure(figsize=(A4[0] / 25.4, A4[1] / 25.4))
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, A4[0])
    ax.set_ylim(0, A4[1])
    ax.set_aspect("equal")
    ax.axis("off")

    # The tag: OpenCV's tag36h11 (= AprilTag 36h11) bitmap, black border included, scaled to TAG_SIZE_M.
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    bitmap = cv2.aruco.generateImageMarker(d, tag_id, 8 * 50)
    s = TAG_SIZE_M * 1000
    cx, cy = TAG_CENTRE_MM
    ax.imshow(bitmap, cmap="gray", vmin=0, vmax=255, interpolation="nearest",
              extent=(cx - s / 2, cx + s / 2, cy - s / 2, cy + s / 2))

    # The footprints: stance centre STANCE_FROM_TAG_M below the tag centre, toes towards it.
    sc = cy - STANCE_FROM_TAG_M * 1000
    length, width = SOLE[0] * 1000, SOLE[1] * 1000
    for side, label in ((-1, "L"), (1, "R")):
        x = cx + side * FEET_SPACING_M * 1000 / 2
        ax.add_patch(patches.Rectangle((x - width / 2, sc - length / 2), width, length, fill=False, lw=1.4,
                                       edgecolor="black", ls=(0, (3, 2))))
        ax.plot([x - width / 2, x + width / 2], [sc + length / 2] * 2, color=(0.85, 0.1, 0.1), lw=2.5)
        ax.plot([x - 3, x + 3], [sc, sc], color="black", lw=0.5)
        ax.plot([x, x], [sc - 3, sc + 3], color="black", lw=0.5)
        ax.text(x, sc - 12, label, ha="center", va="center", fontsize=10)

    ax.text(cx, cy + s / 2 + 10, f"tag {tag_id}", ha="center", fontsize=16, weight="bold")
    ax.text(12, A4[1] - 14, f"microduck — robotctl robot goto tag {tag_id}", fontsize=12, weight="bold")
    ax.text(12, A4[1] - 21, "Lay flat on the floor. The robot ends on the footprints, facing the tag "
            "(red edge = toe).", fontsize=8)
    ax.text(12, 30, f"Tag 36h11 #{tag_id}, black square {s:.0f} mm. Footprints: sole centres "
            f"{FEET_SPACING_M * 1000:.0f} mm apart, {STANCE_FROM_TAG_M * 1000:.0f} mm from the tag centre.",
            fontsize=7)
    ax.plot([12, 112], [18, 18], color="black", lw=1.5)
    for u in (12, 112):
        ax.plot([u, u], [15, 21], color="black", lw=1.0)
    ax.text(116, 16.5, "100 mm — print at 100 % (actual size); measure this bar and the tag", fontsize=7)
    pdf.savefig(fig)
    return fig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*", type=int, default=[0, 1, 2, 3, 4])
    ap.add_argument("--out", default="footstep_tags")
    ap.add_argument("--png", action="store_true", help="also a PNG preview per page")
    A = ap.parse_args()
    out = Path(A.out)
    out.mkdir(parents=True, exist_ok=True)
    with PdfPages(out / "footstep_tags.pdf") as pdf:
        for tag_id in A.ids:
            fig = page(pdf, tag_id)
            if A.png:
                fig.savefig(out / f"tag_{tag_id}.png", dpi=60)
            plt.close(fig)
    print(f"→ {out / 'footstep_tags.pdf'}: tags {A.ids} (print at 100 %)")


if __name__ == "__main__":
    main()
