#!/usr/bin/env python3
"""Export the duck the policies were trained on as a glTF the app can pose.

The source is microduck_rl's `robot_walk.xml` and its STL meshes: the same MJCF the simulator and
the monitor's terminal view are built from, so the duck on a phone has the proportions, joint
axes and joint limits of the real one. Two files come out:

  duck.glb        one node per MJCF body, nested as the bodies are, each holding its parts as
                  child mesh nodes. A body's rest transform is its MJCF pos/quat, so rotating a
                  body node about its joint axis poses the duck exactly as MuJoCo would.
  duck_rig.json   which node each joint turns, about which axis, within which range. A sidecar
                  rather than glTF `extras`, because whether an importer keeps extras is a
                  version question and this file must be read the same way by every Godot.

Run it through `build_assets.sh`, which knows where microduck_rl and the venv are:

    .venv/bin/python export_model.py --rl ~/Code/microduck_rl --out ../app/duck/generated

The STL meshes are CC BY-NC-SA 4.0 (microduck_rl's LICENSE-MODELS.md), and a decimated glTF is
an adaptation of them, so the output directory gets that licence and the attribution it requires
written beside it.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import meshoptimizer
import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[2]
MJCF = Path("src/mjlab_microduck/robot/microduck/robot_walk.xml")


def load_bake_script():
    """`scripts/bake-duck-mesh.py`, imported for what the two exports must agree on.

    Which CAD parts are enclosed and dropped, which colours the MJCF gets wrong, how a binary STL
    is read, the joint order on the wire: written once there, for the monitor, and reused here so
    a fix to one is a fix to both.
    """
    path = REPO / "scripts" / "bake-duck-mesh.py"
    spec = importlib.util.spec_from_file_location("bake_duck_mesh", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bake = load_bake_script()

# Z-up MJCF to Y-up glTF: a quarter turn about X, as glTF's (x, y, z, w). Applied once, on the
# root, so every node below keeps MuJoCo's own frames and a recorded qpos poses them unconverted.
Z_UP_TO_Y_UP = [-np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]

# Faces meeting at more than this are drawn as an edge rather than smoothed over. CAD shells are
# mostly gentle curves joined at real corners, and 40° keeps both.
CREASE_DEG = 40.0

# The parts the app moves on its own, by mesh name. The jaw has no joint in the MJCF — on the robot
# the `mouth` servo opens it, and the model was exported with it shut — so it is split out as its
# own node for the app to hinge.
MOUTH_MESHES = ("jaw", "jaw_soft")


def weld(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Triangle soup → shared vertices, which decimation needs to see the surface as connected."""
    flat = tris.reshape(-1, 3).astype(np.float64)
    # 1 µm: well under the CAD's own resolution, well over float noise between coincident corners.
    keys = np.round(flat * 1e6).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return flat[first], faces[keep]


def budget_for(verts: np.ndarray, detail: float) -> int:
    """Triangles a part earns, from its size. A 22 mm bearing that the CAD draws with 21k triangles
    and the shell that hides it should not cost the same."""
    diag_mm = float(np.linalg.norm(verts.max(axis=0) - verts.min(axis=0))) * 1000
    return int(np.clip(diag_mm * 15 * detail, 120, 4000 * detail))


def simplify(verts: np.ndarray, faces: np.ndarray, budget: int) -> tuple[np.ndarray, np.ndarray]:
    """Quadric edge collapse down to `budget` triangles; the result still indexes `verts`.

    meshoptimizer rather than the more common Fast-Quadric port: the CAD tessellates in long
    fans (a median triangle is ~86 times longer than it is wide), and that port's flip test refuses
    nearly every collapse on them — the xl330 went from 4126 triangles to 3914 and stopped.
    The 5% error ceiling is relative to the part's own size; it is the triangle count, not the
    error, that stops it on every part of this duck.
    """
    if len(faces) <= budget:
        return verts, faces
    indices = faces.astype(np.uint32).reshape(-1)
    out = np.zeros_like(indices)
    count = meshoptimizer.simplify(
        out, indices, verts.astype(np.float32), target_index_count=budget * 3, target_error=0.05
    )
    return verts, out[:count].reshape(-1, 3).astype(np.int64)


def unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-20)


def crease_normals(verts: np.ndarray, faces: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Smooth normals except across creases, splitting a vertex where its faces disagree.

    Each corner takes the area-weighted normal of its vertex unless that is more than CREASE_DEG
    from its own face, in which case it takes the face's. Cheaper than grouping faces into smoothing
    regions and indistinguishable on parts this size.
    """
    tri = verts[faces]
    face_n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    vert_n = np.zeros_like(verts)
    np.add.at(vert_n, faces.reshape(-1), np.repeat(face_n, 3, axis=0))
    face_n, vert_n = unit(face_n), unit(vert_n)

    corner_n = vert_n[faces]
    sharp = (corner_n * face_n[:, None, :]).sum(axis=-1) < np.cos(np.radians(CREASE_DEG))
    corner_n = np.where(sharp[..., None], face_n[:, None, :], corner_n)

    # A corner is its position plus its normal; corners agreeing on both share a vertex.
    key = np.concatenate([faces.reshape(-1, 1), np.round(corner_n.reshape(-1, 3) * 1e4)], axis=1)
    _, first, inverse = np.unique(key.astype(np.int64), axis=0, return_index=True, return_inverse=True)
    return verts[faces.reshape(-1)[first]], corner_n.reshape(-1, 3)[first], inverse.reshape(-1, 3)


def mouth_hinge(mjcf_path: Path) -> dict:
    """Where the lower beak hinges, in the head body's frame.

    The MJCF has no mouth joint — on the robot it is a servo the model does not describe — so the
    hinge is placed from the jaw's own geometry at the zero pose, where the duck faces +X with Z up
    and the jaw runs forward under the fixed upper lip from the mouth servo at its back: near the
    jaw's rear end and upper edge, turning about the duck's left–right axis. A positive turn about
    the returned axis swings the tip down.
    """
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    points, head = [], None
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[g]
        if model.mesh(mesh).name not in MOUTH_MESHES:
            continue
        head = int(model.geom_bodyid[g])
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        verts = model.mesh_vert[start : start + count]
        points.append(verts @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g])
    world = np.concatenate(points)
    lo, hi = world.min(axis=0), world.max(axis=0)
    pivot = np.array([lo[0] + (hi[0] - lo[0]) * 0.1, (lo[1] + hi[1]) / 2, hi[2] - (hi[2] - lo[2]) * 0.25])
    rotation = data.xmat[head].reshape(3, 3)
    return {
        "pivot": (rotation.T @ (pivot - data.xpos[head])).round(6).tolist(),
        "axis": (rotation.T @ np.array([0.0, 1.0, 0.0])).round(6).tolist(),
    }


def gl_quat(mj: np.ndarray) -> list[float]:
    """MJCF (w, x, y, z) → glTF (x, y, z, w)."""
    w, x, y, z = mj / np.linalg.norm(mj)
    return [float(x), float(y), float(z), float(w)]


class Gltf:
    """The handful of glTF 2.0 a static, node-posed model needs, packed as one .glb."""

    def __init__(self) -> None:
        self.doc: dict = {
            "asset": {"version": "2.0", "generator": "microduck virtual-duck export_model.py"},
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": [],
            "meshes": [],
            "materials": [],
            "accessors": [],
            "bufferViews": [],
            "buffers": [{"byteLength": 0}],
        }
        self.bin = bytearray()

    def _view(self, data: bytes, target: int) -> int:
        while len(self.bin) % 4:
            self.bin += b"\0"
        self.doc["bufferViews"].append(
            {"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(data), "target": target}
        )
        self.bin += data
        return len(self.doc["bufferViews"]) - 1

    def _accessor(self, array: np.ndarray, kind: str, component: int, target: int, bounds: bool) -> int:
        accessor = {
            "bufferView": self._view(array.tobytes(), target),
            "componentType": component,
            "count": len(array) if kind != "SCALAR" else array.size,
            "type": kind,
        }
        if bounds:
            accessor["min"] = array.min(axis=0).tolist()
            accessor["max"] = array.max(axis=0).tolist()
        self.doc["accessors"].append(accessor)
        return len(self.doc["accessors"]) - 1

    def mesh(self, name: str, verts, normals, faces, material: int) -> int:
        ARRAY_BUFFER, ELEMENT_ARRAY_BUFFER = 34962, 34963
        FLOAT, UNSIGNED_SHORT, UNSIGNED_INT = 5126, 5123, 5125
        wide = len(verts) > 0xFFFF
        indices = faces.reshape(-1).astype("<u4" if wide else "<u2")
        primitive = {
            "attributes": {
                "POSITION": self._accessor(verts.astype("<f4"), "VEC3", FLOAT, ARRAY_BUFFER, True),
                "NORMAL": self._accessor(normals.astype("<f4"), "VEC3", FLOAT, ARRAY_BUFFER, False),
            },
            "indices": self._accessor(
                indices, "SCALAR", UNSIGNED_INT if wide else UNSIGNED_SHORT, ELEMENT_ARRAY_BUFFER, False
            ),
            "material": material,
        }
        self.doc["meshes"].append({"name": name, "primitives": [primitive]})
        return len(self.doc["meshes"]) - 1

    def material(self, name: str, rgb, roughness: float) -> int:
        # The MJCF's colours are display values — MuJoCo draws them as written — and glTF's base
        # colour is linear, so they are decoded from sRGB here or every part renders washed out.
        srgb = np.clip(np.asarray(rgb, dtype=np.float64), 0, 1)
        linear = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
        self.doc["materials"].append(
            {
                "name": name,
                "pbrMetallicRoughness": {
                    "baseColorFactor": [*map(float, linear.round(5)), 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": roughness,
                },
            }
        )
        return len(self.doc["materials"]) - 1

    def node(self, name: str, translation=None, rotation=None, mesh=None) -> int:
        node: dict = {"name": name}
        if translation is not None and np.any(translation):
            node["translation"] = [float(t) for t in translation]
        if rotation is not None and rotation != [0.0, 0.0, 0.0, 1.0]:
            node["rotation"] = rotation
        if mesh is not None:
            node["mesh"] = mesh
        self.doc["nodes"].append(node)
        return len(self.doc["nodes"]) - 1

    def child(self, parent: int, child: int) -> None:
        self.doc["nodes"][parent].setdefault("children", []).append(child)

    def write(self, path: Path) -> None:
        while len(self.bin) % 4:
            self.bin += b"\0"
        self.doc["buffers"][0]["byteLength"] = len(self.bin)
        text = json.dumps(self.doc, separators=(",", ":")).encode()
        text += b" " * (-len(text) % 4)
        glb = struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(text) + 8 + len(self.bin))
        glb += struct.pack("<I4s", len(text), b"JSON") + text
        glb += struct.pack("<I4s", len(self.bin), b"BIN\0") + bytes(self.bin)
        path.write_bytes(glb)


def source_revision(rl: Path) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(rl), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--rl", type=Path, required=True, help="a microduck_rl checkout")
    parser.add_argument("--out", type=Path, required=True, help="directory to write duck.glb into")
    parser.add_argument("--detail", type=float, default=1.0, help="triangle budget multiplier")
    args = parser.parse_args()

    mjcf_path = args.rl.expanduser() / MJCF
    root = ET.parse(mjcf_path).getroot()
    mesh_dir = mjcf_path.parent / root.find("compiler").get("meshdir", ".")
    colours = {
        m.get("name"): bake.parse_floats(m.get("rgba"), "0.5 0.5 0.5 1")[:3] for m in root.iter("material")
    }

    gltf = Gltf()
    duck = gltf.node("Microduck", rotation=Z_UP_TO_Y_UP)
    materials: dict[str, int] = {}
    meshes: dict[tuple[str, str], tuple[int, int]] = {}
    triangles = 0
    joints = []
    mouth = []

    def material_for(name: str, mesh: str) -> int:
        if name not in materials:
            rgb = bake.RECOLOR.get(mesh, colours[name])
            # The camera lens is glass; everything else on the duck is printed or moulded plastic.
            materials[name] = gltf.material(name, rgb, 0.15 if mesh == "lens" else 0.6)
        return materials[name]

    def mesh_for(name: str, material: str) -> tuple[int, int]:
        """The glTF mesh for one STL in one material, and its triangle count. Instanced parts — the
        xl330 appears fifteen times — are decimated once and referenced by every node."""
        key = (name, material)
        if key not in meshes:
            verts, faces = weld(bake.load_stl(mesh_dir / f"{name}.stl"))
            verts, faces = simplify(verts, faces, budget_for(verts, args.detail))
            positions, normals, faces = crease_normals(verts, faces)
            index = gltf.mesh(name, positions, normals, faces, material_for(material, name))
            meshes[key] = (index, len(faces))
        return meshes[key]

    def walk(body: ET.Element, parent: int) -> None:
        nonlocal triangles
        node = gltf.node(
            body.get("name"),
            translation=bake.parse_floats(body.get("pos"), "0 0 0"),
            rotation=gl_quat(bake.parse_floats(body.get("quat"), "1 0 0 0")),
        )
        gltf.child(parent, node)
        for joint in body.findall("joint"):
            if joint.get("pos") not in (None, "0 0 0"):
                # Every joint in the export sits on its body's origin, and the app hinges the
                # body node there. One that does not would need a pivot node the app lacks.
                raise SystemExit(f"{joint.get('name')}: joint off its body origin, not supported")
            joints.append(
                {
                    "name": joint.get("name"),
                    "node": body.get("name"),
                    "axis": bake.parse_floats(joint.get("axis"), "0 0 1").tolist(),
                    "range": bake.parse_floats(joint.get("range"), "0 0").tolist(),
                }
            )
        seen: dict[str, int] = {}
        for geom in body.findall("geom"):
            name = geom.get("mesh")
            if geom.get("class") != "visual" or geom.get("type") != "mesh" or name in bake.HIDDEN:
                continue
            if any(geom.get(a) for a in ("euler", "axisangle", "xyaxes", "zaxis")):
                raise SystemExit(f"{name}: geom orientation other than quat is not supported")
            mesh, count = mesh_for(name, geom.get("material"))
            triangles += count
            seen[name] = seen.get(name, 0) + 1
            # Suffixed, because the CAD reuses names across kinds: the head's *body* is `jaw_soft`
            # and so is one of its meshes, and a lookup by name must not find the head when it
            # asks for the lip.
            part = gltf.node(
                f"{name}_geom" if seen[name] == 1 else f"{name}_geom{seen[name]}",
                translation=bake.parse_floats(geom.get("pos"), "0 0 0"),
                rotation=gl_quat(bake.parse_floats(geom.get("quat"), "1 0 0 0")),
                mesh=mesh,
            )
            gltf.child(node, part)
            if name in MOUTH_MESHES:
                mouth.append(gltf.doc["nodes"][part]["name"])
        for child in body.findall("body"):
            walk(child, node)

    for top in root.find("worldbody").findall("body"):
        walk(top, duck)

    known = set(bake.JOINT_NAMES)
    unknown = [j["name"] for j in joints if j["name"] not in known]
    if unknown:
        raise SystemExit(f"joints the robot does not have: {unknown}")

    out = args.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    gltf.write(out / "duck.glb")
    revision = source_revision(args.rl.expanduser())
    rig = {
        "source": {"repo": "pollen-robotics/microduck_rl", "revision": revision, "mjcf": str(MJCF)},
        # The order of `duck_ipc_proto::JOINT_NAMES`, which the recorded motions use too.
        "joint_order": bake.JOINT_NAMES,
        "joints": joints,
        "trunk": root.find("worldbody").find("body").get("name"),
        "mouth": {"parts": mouth, **mouth_hinge(mjcf_path)},
    }
    (out / "duck_rig.json").write_text(json.dumps(rig, indent=2) + "\n")
    (out / "LICENSE-MODELS.md").write_text(
        "# Licence for the files in this directory\n\n"
        "`duck.glb` is adapted from the 3D model files of the Microduck by **Pollen Robotics**,\n"
        "https://github.com/pollen-robotics/microduck_rl (revision "
        f"`{revision}`), which are licensed under\n"
        "CC BY-NC-SA 4.0: https://creativecommons.org/licenses/by-nc-sa/4.0/\n\n"
        "Changes: hidden internal parts removed, meshes decimated, normals recomputed, and the\n"
        "result converted from STL + MJCF to glTF. This adaptation is licensed under the same\n"
        "CC BY-NC-SA 4.0 licence.\n\n"
        "`duck_rig.json` is derived from the MJCF, which microduck_rl licenses under Apache-2.0.\n"
    )
    size = (out / "duck.glb").stat().st_size
    print(f"{out / 'duck.glb'}: {size / 1e6:.1f} MB, {len(meshes)} meshes, {triangles} triangles drawn")
    print(f"{len(joints)} joints, mouth parts {mouth}, from microduck_rl {revision[:12]}")


if __name__ == "__main__":
    sys.exit(main())
