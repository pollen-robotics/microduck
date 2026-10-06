#!/usr/bin/env python3
"""Bake the robot's MJCF visual model into `robotctl/assets/duck.bin`.

`robotctl monitor` draws a live 3D view of the robot, and this is where its geometry
comes from. The MJCF and its STL meshes live in the app repository — they describe the
robot the policies were trained on — but `robotctl` ships to the board as a single
binary with no assets directory, and a terminal is a ~100×100-pixel display for which
330k triangles of CAD export are pure waste. So the model is baked here, offline, into
a few thousand triangles of compact binary, and the result is committed.

Rerun when the robot's model changes:

    scripts/bake-duck-mesh.py ~/MISC/microduck_app/robot_assets/alpha/robot_walk.xml

Needs python3 + numpy, which never runs on the board — this is a dev-machine tool.

The output format is read by `robotctl/src/duck.rs`; the two must agree, and the
format version in the header is how a mismatch fails loudly instead of drawing garbage.
"""

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

FORMAT_VERSION = 1

# The wire order of `duck_ipc_proto::JOINT_NAMES`: the runtime indexes measured angles
# with the numbers stored here, so this list is a copy of that contract. `mouth` is a
# servo without an MJCF joint (the jaw is a fixed geom), so it never appears in a bake.
JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]

# Parts that exist in the CAD but are enclosed by shells: PCBs, brackets. Invisible
# from outside, and at this decimation level their triangles would poke through the
# shell that hides them — dropping them is both lighter and more correct. The battery
# (`np_f970`) is not one of them: it rides outside the trunk and is drawn.
HIDDEN = {
    "pcb__raspberry_pi_zero_2_w",
    "elec_rpi_robot_hat_pcb",
    "power_support",
    "banana_pcb_locker",
    "motor_support",
}

# Colours the MJCF gets wrong, by mesh name, as 0–1 RGB. The CAD export paints the
# battery the same light grey as the shells; the real NP-F970 pack is black, and drawn
# light it reads as part of the body rather than the battery strapped to its back.
RECOLOR = {
    "np_f970": (0.16, 0.16, 0.18),
}


def load_stl(path: Path) -> np.ndarray:
    """Binary STL → (n, 3, 3) float32 triangle corners. Meters, as exported."""
    data = path.read_bytes()
    (n,) = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + n * 50:
        raise SystemExit(f"{path}: not the binary STL this expects")
    tris = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(n, 50)
    # Each record: normal (12 bytes, ignored — recomputed at runtime), 3 corners, pad.
    return tris[:, 12:48].copy().view("<f4").reshape(n, 3, 3)


def decimate(tris: np.ndarray, budget: int) -> tuple[np.ndarray, np.ndarray]:
    """Cluster vertices on a grid until at most `budget` triangles survive.

    Vertex clustering rather than edge collapse: at the resolution of a terminal the
    silhouette is all that survives anyway, and clustering is a page of numpy instead
    of a mesh library. Each cluster's vertex is the mean of what fell into it, so
    shells shrink toward their own surface rather than toward cell corners.

    Returns (verts (v, 3) f32, faces (f, 3) int).
    """
    lo, hi = tris.min(axis=(0, 1)), tris.max(axis=(0, 1))
    diag = float(np.linalg.norm(hi - lo))
    cell = max(diag / 48.0, 1e-4)
    while True:
        keys = np.floor((tris - lo) / cell).astype(np.int64)
        flat = keys.reshape(-1, 3)
        uniq, inverse = np.unique(flat, axis=0, return_inverse=True)
        faces = inverse.reshape(-1, 3)
        # A triangle whose corners share a cell has collapsed; drop it.
        keep = (
            (faces[:, 0] != faces[:, 1])
            & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2])
        )
        faces = faces[keep]
        # Two originals can decimate to the same triangle; one is enough.
        faces = np.unique(np.sort(faces, axis=1), axis=0) if len(faces) else faces
        if len(faces) <= budget:
            break
        cell *= 1.3
    verts = np.zeros((len(uniq), 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=len(uniq)).astype(np.float64)
    for axis in range(3):
        verts[:, axis] = np.bincount(
            inverse, weights=flat_coords(tris)[:, axis], minlength=len(uniq)
        )
    verts /= counts[:, None]
    # Re-index to only the vertices faces still reference.
    used = np.unique(faces) if len(faces) else np.array([], dtype=np.int64)
    remap = np.full(len(verts), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return verts[used].astype(np.float32), remap[faces]


def flat_coords(tris: np.ndarray) -> np.ndarray:
    return tris.reshape(-1, 3).astype(np.float64)


def parse_floats(text: str | None, default: str) -> np.ndarray:
    return np.array([float(x) for x in (text or default).split()], dtype=np.float64)


def quat_matrix(q: np.ndarray) -> np.ndarray:
    """MJCF quaternion (w x y z) → rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    mjcf_path = Path(sys.argv[1]).expanduser()
    root = ET.parse(mjcf_path).getroot()
    mesh_dir = mjcf_path.parent / root.find("compiler").get("meshdir", ".")

    materials = {
        m.get("name"): parse_floats(m.get("rgba"), "0.5 0.5 0.5 1")[:3]
        for m in root.iter("material")
    }

    # Budget per unique mesh, sized by how big the part actually is: a foot shell earns
    # more triangles than a 22 mm bearing. Instanced meshes (the xl330 appears a dozen
    # times) are decimated once and referenced per instance.
    meshes: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def mesh_index(name: str) -> int:
        if name not in meshes:
            tris = load_stl(mesh_dir / f"{name}.stl")
            diag_mm = float(np.linalg.norm(tris.max(axis=(0, 1)) - tris.min(axis=(0, 1)))) * 1000
            # Sized for a ~100-pixel-tall display and a CPU that must also run the
            # robot: past this the extra triangles change nothing a cell can show.
            budget = int(np.clip(40 + diag_mm * 3.5, 60, 420))
            meshes[name] = decimate(tris, budget)
        return list(meshes).index(name)

    bodies: list[dict] = []
    parts: list[dict] = []

    def walk(body: ET.Element, parent: int) -> None:
        index = len(bodies)
        joint = body.find("joint")
        bodies.append(
            {
                "parent": parent,
                "pos": parse_floats(body.get("pos"), "0 0 0"),
                "quat": parse_floats(body.get("quat"), "1 0 0 0"),
                "joint": JOINT_NAMES.index(joint.get("name")) if joint is not None else -1,
                "axis": parse_floats(joint.get("axis"), "0 0 1") if joint is not None else np.zeros(3),
            }
        )
        for geom in body.findall("geom"):
            if geom.get("class") != "visual" or geom.get("type") != "mesh":
                continue
            name = geom.get("mesh")
            if name in HIDDEN:
                continue
            parts.append(
                {
                    "body": index,
                    "mesh": mesh_index(name),
                    "rgb": RECOLOR.get(name, materials[geom.get("material")]),
                    "pos": parse_floats(geom.get("pos"), "0 0 0"),
                    "quat": parse_floats(geom.get("quat"), "1 0 0 0"),
                }
            )
        for child in body.findall("body"):
            walk(child, index)

    for top in root.find("worldbody").findall("body"):
        walk(top, -1)

    out = bytearray()
    out += struct.pack("<4sIHHH", b"DUCK", FORMAT_VERSION, len(meshes), len(bodies), len(parts))
    for verts, faces in meshes.values():
        out += struct.pack("<HH", len(verts), len(faces))
        out += verts.astype("<f4").tobytes()
        out += faces.astype("<u2").tobytes()
    for b in bodies:
        out += struct.pack("<hh", b["parent"], b["joint"])
        out += b["pos"].astype("<f4").tobytes()
        out += b["quat"].astype("<f4").tobytes()
        out += b["axis"].astype("<f4").tobytes()
    for p in parts:
        rgb = (np.clip(p["rgb"], 0, 1) * 255).round().astype(np.uint8)
        out += struct.pack("<HHBBBB", p["body"], p["mesh"], *rgb, 0)
        out += p["pos"].astype("<f4").tobytes()
        out += p["quat"].astype("<f4").tobytes()

    dest = Path(__file__).resolve().parent.parent / "robotctl" / "assets" / "duck.bin"
    dest.parent.mkdir(exist_ok=True)
    dest.write_bytes(out)

    drawn = sum(len(meshes[list(meshes)[p["mesh"]]][1]) for p in parts)
    print(f"{dest}: {len(out) / 1024:.0f} KB")
    print(f"{len(meshes)} meshes, {len(bodies)} bodies, {len(parts)} parts, {drawn} instanced triangles")


if __name__ == "__main__":
    main()
