#!/usr/bin/env python3
"""Record the duck's motions from the real policies, for the app to play back.

A phone does not run the physics. The policies were trained against MuJoCo and the BAM model of
the XL330, and under any other engine's physics they would not hold the duck up. So this runs them
here, under exactly that physics, and the app replays what they did: every walk, sit, peck, kick and
fall the virtual duck makes is a frame-for-frame copy of what the real policies drove the simulated
body to do.

The harness is microduck_rl's own `scripts/infer_policy.py`, imported rather than copied. Its
`PolicyInference` builds the 61-wide observation the policies are rehearsed against, and its BAM
loader builds the actuator they were trained with, so a policy that walks there walks here and a
change upstream is picked up by re-running this.

Each take is a short scripted session — settle, command, wait — recorded at the policy's 50 Hz and
cut into clips. Locomotion is cut into a start, a seamless loop and a stop, the stop beginning on
the loop's own first pose so the app can leave the loop on any wrap. Every clip's trunk is stored
relative to where the duck stood when it began (its x, y and heading), which is what lets the app
chain clips anywhere in its room.

    .venv/bin/python record_motions.py --rl ~/Code/microduck_rl --out ../app/duck/generated
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import math
import sys
import tomllib
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[2]
SCENES = Path("src/mjlab_microduck/robot/microduck")
RATE_HZ = 50
DT = 1.0 / RATE_HZ
# The actuated joints in recording order, set once a session has said what they are.
JOINT_ORDER: list[str] = []

# The battery voltage infer_policy defaults to, and the middle of the range the policies were
# randomised over in training (6.5–8.2 V): a duck on a healthy charge.
VIN = 7.4
VIN_DROP_GAIN = 0.1


def policy_pin() -> tuple[str, str]:
    """The official policy set this repository's daemon ships against — `[workspace.metadata.
    policies]` in Cargo.toml, read rather than repeated, so the virtual duck moves like a real one
    on the same release."""
    with open(REPO / "Cargo.toml", "rb") as f:
        pin = tomllib.load(f)["workspace"]["metadata"]["policies"]
    return pin["repo"], pin["version"]


def fetch_policies(cache: Path) -> tuple[Path, dict]:
    repo, version = policy_pin()
    dest = cache / repo.replace("/", "--") / version
    dest.mkdir(parents=True, exist_ok=True)

    def get(name: str) -> Path:
        path = dest / name
        if not path.exists():
            url = f"https://huggingface.co/{repo}/resolve/{version}/{name}"
            with urllib.request.urlopen(url, timeout=60) as response:
                path.write_bytes(response.read())
        return path

    manifest = json.loads(get("manifest.json").read_text())
    for entry in manifest["policies"]:
        get(entry["file"])
    return dest, manifest


def load_harness(rl: Path):
    spec = importlib.util.spec_from_file_location("infer_policy", rl / "scripts" / "infer_policy.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # One thread per session. The policies are small MLPs, so a pool buys nothing but contention —
    # and by default ONNX Runtime sizes it to every core, on a machine that may be busy with other
    # things. A single thread is also what makes the deterministic re-run in `locomotion` hold.
    ort = module.ort
    session = ort.InferenceSession

    def single_threaded(path, *args, **kwargs):
        options = kwargs.pop("sess_options", None) or ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        return session(path, options, *args, **kwargs)

    module.ort = type("ort", (), {**{k: getattr(ort, k) for k in dir(ort) if not k.startswith("__")},
                                  "InferenceSession": staticmethod(single_threaded)})
    return module


def episode(manifest: dict, name: str) -> float:
    """How long the robot runs an episodic policy before handing back to walking — the manifest's
    `duration_s`, which is what robotd honours, rather than infer_policy's keyboard defaults."""
    for entry in manifest["policies"]:
        if entry.get("name", Path(entry["file"]).stem) == name:
            return float(entry["duration_s"])
    raise SystemExit(f"{name}: not in the policy manifest")


@dataclass
class Clip:
    frames: np.ndarray  # (n, 7 + joints): trunk xyz, trunk quat wxyz, joint angles
    loop: bool = False
    end: np.ndarray | None = None  # loops: the frame one period on, for the cycle's displacement
    ball: np.ndarray | None = None  # kicks: ball xyz per frame
    notes: dict = field(default_factory=dict)


class Session:
    """One simulated duck under the policies, stepped at 50 Hz and recorded every step."""

    def __init__(self, harness, rl: Path, policies: Path, manifest: dict, scene: str = "scene.xml"):
        self.h = harness
        files = {Path(p["file"]).stem: str(policies / p["file"]) for p in manifest["policies"]}
        with contextlib.redirect_stdout(io.StringIO()):
            bam_model = harness.load_bam_model(harness.BAM_KP_FW, VIN, None)
            self.model, self.data, self.bam, _ = harness.load_mujoco_with_bam(
                str(rl / SCENES / scene), bam_model, 0.005, VIN_DROP_GAIN, harness.BAM_VIN_MIN
            )
            self.policy = harness.PolicyInference(
                self.model,
                self.data,
                walking_onnx_path=files["velstand"],
                bam_ctrl=self.bam,
                use_projected_gravity=True,
                new_cmd_obs=True,
                ground_pick_onnx_path=files["alpha_ground_pick"],
                ground_pick_period=4.0,
                sitstand_onnx_path=files["alpha_sitstand"],
                kick_left_onnx_path=files["ball_kick_left"],
                kick_right_onnx_path=files["ball_kick_right"],
                roulade_onnx_path=files["roulade"],
                kick_duration=episode(manifest, "kick_left"),
                roulade_duration=episode(manifest, "roulade"),
            )
        p, m, d = self.policy, self.model, self.data
        free = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
        self.trunk_qpos = int(m.jnt_qposadr[free])
        self.trunk_qvel = int(m.jnt_dofadr[free])
        self.joint_names = [
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(m.actuator_trnid[i, 0])) for i in range(m.nu)
        ]
        # The start infer_policy uses: standing pose, trunk at its standing height.
        d.qpos[self.trunk_qpos : self.trunk_qpos + 3] = [0.0, 0.0, 0.125]
        d.qpos[self.trunk_qpos + 3 : self.trunk_qpos + 7] = [1, 0, 0, 0]
        for i, adr in enumerate(p.joint_qpos_indices):
            d.qpos[adr] = p.default_pose[i]
        self.bam.reset(d.qpos)
        p.set_position_targets(p.default_pose)
        mujoco.mj_forward(m, d)
        self.frames: list[np.ndarray] = []
        self.ball: list[np.ndarray] = []

    @property
    def now(self) -> int:
        """Frames recorded so far — the index the next frame will have."""
        return len(self.frames)

    def run(self, seconds: float) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(round(seconds * RATE_HZ)):
                self.policy.update_ground_pick_phase(DT)
                self.policy.update_behavior(DT)
                self.policy.apply_action(self.policy.infer())
                for _ in range(4):
                    self.bam.update()
                    mujoco.mj_step(self.model, self.data)
                q = self.data.qpos
                self.frames.append(
                    np.concatenate(
                        [q[self.trunk_qpos : self.trunk_qpos + 7], q[self.policy.joint_qpos_indices]]
                    ).copy()
                )
                if self.policy.ball_qpos_adr is not None:
                    self.ball.append(q[self.policy.ball_qpos_adr : self.policy.ball_qpos_adr + 3].copy())

    def quietly(self, fn, *args) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            fn(*args)

    def walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        self.quietly(self.policy.set_vel_cmd, vx, vy, wz)

    def head(self, neck_pitch=0.0, head_pitch=0.0, head_yaw=0.0, head_roll=0.0) -> None:
        self.policy.head_offset[:] = [neck_pitch, head_pitch, head_yaw, head_roll]
        self.quietly(self.policy._update_command)

    def body(self, roll=0.0, pitch=0.0, yaw=0.0, z=0.0) -> None:
        self.policy.body_cmd[:] = [0.0, 0.0, z, roll, pitch, yaw]
        self.quietly(self.policy._update_command)

    def back_to_walking(self) -> None:
        """Hands control from sitstand back to velstand. infer_policy only switches sessions on a
        velocity command when it has a separate standing policy, which v5 onwards does not."""
        p = self.policy
        p.current_policy, p.ort_session = "walking", p.walking_session
        self.quietly(p._update_command)

    def push(self, vx: float, vy: float) -> None:
        """Sets the trunk's horizontal velocity, as infer_policy's push does — hard enough here to
        knock the duck over, so it has to get back up."""
        self.data.qvel[self.trunk_qvel : self.trunk_qvel + 2] = [vx, vy]

    def clip(self, start: int, stop: int, **kwargs) -> Clip:
        ball = np.array(self.ball[start:stop]) if self.ball else None
        return Clip(np.array(self.frames[start:stop]), ball=ball, **kwargs)


def heading(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = quat_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def to_ground_frame(clip: Clip) -> Clip:
    """Re-expresses a clip relative to where the duck stood at its first frame: that frame's trunk
    x, y and heading become the origin, so the app can play the clip from wherever its duck is."""
    x0, y0 = clip.frames[0, 0], clip.frames[0, 1]
    yaw0 = heading(clip.frames[0, 3:7])
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    turn = np.array([math.cos(-yaw0 / 2), 0.0, 0.0, math.sin(-yaw0 / 2)])

    def local(frames: np.ndarray) -> np.ndarray:
        out = frames.copy()
        dx, dy = frames[:, 0] - x0, frames[:, 1] - y0
        out[:, 0], out[:, 1] = c * dx - s * dy, s * dx + c * dy
        out[:, 3:7] = quat_mul(turn, frames[:, 3:7])
        return out

    def local_points(points: np.ndarray) -> np.ndarray:
        out = points.copy()
        dx, dy = points[:, 0] - x0, points[:, 1] - y0
        out[:, 0], out[:, 1] = c * dx - s * dy, s * dx + c * dy
        return out

    return Clip(
        local(clip.frames),
        loop=clip.loop,
        end=local(clip.end[None])[0] if clip.end is not None else None,
        ball=local_points(clip.ball) if clip.ball is not None else None,
        notes=clip.notes,
    )


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a ⊗ b for a single quaternion a and an (n, 4) array b, both (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


def find_cycle(
    frames: np.ndarray, steady: tuple[int, int], cut: tuple[int, int], min_period: int, max_period: int
) -> tuple[int, int, float]:
    """The gait's period, from the `steady` window, and the best place in `cut` to start one cycle.

    The period is the shortest lag at which the joint angles repeat — the shortest, so a walk is
    cut as one stride and not two. The cut is then the start whose pose and velocity best match one
    period later, which is what makes the wrap invisible. `cut` is kept early so the start clip
    leading into the loop is short.
    """
    joints = frames[:, 7:]
    lo, hi = steady
    errors = []
    for period in range(min_period, max_period + 1):
        a, b = joints[lo : hi - period], joints[lo + period : hi]
        errors.append(float(np.sqrt(((a - b) ** 2).sum(axis=1)).mean()))
    errors = np.array(errors)
    # First lag within 25% of the best: the fundamental, not a multiple of it.
    period = min_period + int(np.argmax(errors <= errors.min() * 1.25 + 1e-4))
    starts = np.arange(max(cut[0], 1), cut[1])
    seams = np.array(
        [
            np.linalg.norm(joints[t] - joints[t + period])
            + np.linalg.norm((joints[t + 1] - joints[t - 1]) - (joints[t + period + 1] - joints[t + period - 1]))
            for t in starts
        ]
    )
    # The earliest cut nearly as clean as the cleanest: every frame before it is start clip, and a
    # turn's start clip is a turn the duck cannot stop part-way through.
    first = int(np.argmax(seams <= seams.min() * 1.5 + 0.005))
    return int(starts[first]), period, float(seams[first])


def settled_after(frames: np.ndarray, start: int, upright: bool = True, still_rad_s: float = 0.15) -> int:
    """The first frame from `start` after which the duck has been still for half a second — where a
    transition clip can end without cutting off its own settling.

    Still means the joints together moving slower than `still_rad_s`; a standing duck is never quite
    motionless, because the policy is balancing it. `upright` also asks for the trunk at standing
    height, so a duck lying quiet on its back before it gets up is not mistaken for one that has.
    """
    speed = np.linalg.norm(np.diff(frames[:, 7:], axis=0), axis=1) * RATE_HZ
    still = speed < still_rad_s
    if upright:
        still &= frames[1:, 2] > 0.105
    window = RATE_HZ // 2
    for t in range(start, len(still) - window):
        if still[t : t + window].all():
            return t
    return len(frames) - 1


class Recorder:
    def __init__(self, harness, rl: Path, policies: Path, manifest: dict):
        self.args = (harness, rl, policies, manifest)
        self.manifest = manifest
        self.clips: dict[str, Clip] = {}
        self.joint_names: list[str] = []

    def session(self, scene: str = "scene.xml") -> Session:
        s = Session(*self.args, scene=scene)
        self.joint_names = s.joint_names
        return s

    def add(self, name: str, clip: Clip) -> None:
        self.clips[name] = to_ground_frame(clip)

    def stand(self) -> None:
        s = self.session()
        s.run(2.0)
        start = s.now
        s.run(4.0)
        # Standing is nearly still; the app crossfades this clip into itself to loop it.
        self.add("stand", s.clip(start, s.now))

    def locomotion(self, name: str, vx: float = 0.0, wz: float = 0.0) -> None:
        """A start, a loop and a stop, from one run and a deterministic re-run.

        The first run finds where the gait settles into its cycle. The second runs the same commands
        up to the end of that cycle and stops there, so the stop clip begins on the loop's own first
        pose and the app can go from loop to stop on any wrap. MuJoCo, BAM and ONNX Runtime on one
        CPU thread are deterministic, so the re-run reaches exactly the same state.
        """
        s = self.session()
        s.run(2.0)
        command_at = s.now
        s.walk(vx, 0.0, wz)
        s.run(7.0)
        start, period, error = find_cycle(
            np.array(s.frames),
            steady=(command_at + 2 * RATE_HZ, s.now),
            cut=(command_at + RATE_HZ, command_at + 3 * RATE_HZ),
            min_period=RATE_HZ // 4,
            max_period=int(1.5 * RATE_HZ),
        )
        loop = s.clip(start, start + period, loop=True, notes={"closure": round(error, 4)})
        loop.end = np.array(s.frames[start + period])
        self.add(f"{name}_start", s.clip(command_at, start))
        self.add(name, loop)

        first = np.array(s.frames)
        s = self.session()
        s.run(2.0)
        s.walk(vx, 0.0, wz)
        s.run((start + period - command_at) / RATE_HZ)
        stop_at = s.now
        if not np.array_equal(np.array(s.frames), first[:stop_at]):
            raise SystemExit(f"{name}: the re-run diverged from the first run; the stop would not join the loop")
        s.walk(0.0, 0.0, 0.0)
        s.run(3.0)
        end = settled_after(np.array(s.frames), stop_at + RATE_HZ // 2)
        self.add(f"{name}_stop", s.clip(stop_at, end + 1))

    def sit(self) -> None:
        s = self.session()
        s.run(2.0)
        down_at = s.now
        s.quietly(s.policy.toggle_sit)
        s.run(4.0)
        seated = settled_after(np.array(s.frames), down_at + RATE_HZ, upright=False)
        s.run(max(0, seated + 3 * RATE_HZ - s.now) / RATE_HZ)
        self.add("sit_down", s.clip(down_at, seated + 1))
        self.add("sit_idle", s.clip(seated, seated + 3 * RATE_HZ))
        up_at = s.now
        s.quietly(s.policy.toggle_sit)
        s.run(4.0)
        s.back_to_walking()
        s.run(2.0)
        stood = settled_after(np.array(s.frames), up_at + 4 * RATE_HZ)
        self.add("sit_up", s.clip(up_at, stood + 1))

    def peck(self) -> None:
        s = self.session()
        s.run(2.0)
        at = s.now
        s.quietly(s.policy.trigger_ground_pick)
        s.run(5.0)
        end = settled_after(np.array(s.frames), at + int(episode(self.manifest, "ground_pick") * RATE_HZ))
        self.add("peck", s.clip(at, end + 1))

    def episodic(self, name: str, scene: str = "scene.xml") -> None:
        s = self.session(scene)
        s.run(2.0)
        at = s.now
        s.quietly(s.policy.trigger_behavior, name)
        s.run(6.0)
        end = settled_after(np.array(s.frames), at + int(episode(self.manifest, name) * RATE_HZ) + RATE_HZ)
        self.add(name, s.clip(at, end + 1))

    def fall(self, name: str, vx: float, vy: float) -> None:
        s = self.session()
        s.run(2.0)
        at = s.now
        s.push(vx, vy)
        s.run(8.0)
        end = settled_after(np.array(s.frames), at + 2 * RATE_HZ)
        self.add(name, s.clip(at, end + 1))

    def dance(self) -> None:
        """Not a policy of its own: velstand standing still while its head and body commands are
        swung to a beat. The policy keeps its balance under them, so the sway is physical — the hips
        give where the body leans. Every command repeats each 2 s, so the response does too."""
        s = self.session()
        s.run(2.0)
        start = s.now
        for i in range(10 * RATE_HZ):
            t = i * DT
            s.body(roll=0.15 * math.sin(2 * math.pi * t / 2.0), pitch=0.06 * math.sin(2 * math.pi * t))
            s.head(
                neck_pitch=0.15 * math.sin(2 * math.pi * t),
                head_yaw=0.45 * math.sin(2 * math.pi * t / 2.0),
                head_roll=0.2 * math.sin(2 * math.pi * t / 2.0),
            )
            s.run(DT)
        period = 2 * RATE_HZ
        begin = start + 4 * RATE_HZ
        loop = s.clip(begin, begin + period, loop=True)
        loop.end = np.array(s.frames[begin + period])
        self.add("dance", loop)


def peck_events(rl: Path, clip: Clip) -> dict:
    """When the beak is lowest during a peck, and how far ahead of the duck's start it is then.

    That is the bite: where a seed has to be for the duck to eat it. It is not when the trunk is
    lowest — the trunk keeps sinking for most of a second after the beak has come back up — so it
    is measured on the beak itself, by posing the visual model with each recorded frame.
    """
    model = mujoco.MjModel.from_xml_path(str(rl / SCENES / "robot_walk.xml"))
    data = mujoco.MjData(model)
    jaw = next(
        g for g in range(model.ngeom)
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and model.mesh(model.geom_dataid[g]).name == "jaw"
    )
    mesh = model.geom_dataid[jaw]
    verts = model.mesh_vert[model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    best = (math.inf, 0, 0.0)
    for i, frame in enumerate(clip.frames):
        data.qpos[0:7] = frame[0:7]
        for k, joint in enumerate(JOINT_ORDER):
            data.qpos[model.jnt_qposadr[names.index(joint)]] = frame[7 + k]
        mujoco.mj_forward(model, data)
        world = verts @ data.geom_xmat[jaw].reshape(3, 3).T + data.geom_xpos[jaw]
        low = world[np.argmin(world[:, 2])]
        if low[2] < best[0]:
            best = (float(low[2]), i, float(low[0]))
    _, frame, reach = best
    return {"bite_s": round(frame / RATE_HZ, 3), "reach_m": round(reach, 4)}


def report(name: str, clip: Clip) -> str:
    f = clip.frames
    height = f[:, 2] * 1000
    travel = math.hypot(*(f[-1, :2] - f[0, :2])) * 1000
    yaw = np.unwrap([heading(q) for q in f[:, 3:7]])
    turned = math.degrees(yaw[-1] - yaw[0])
    line = (
        f"{name:16s} {len(f) / RATE_HZ:5.2f} s  trunk {height[0]:5.1f}→{height[-1]:5.1f} mm "
        f"(min {height.min():5.1f})  moved {travel:5.1f} mm  turned {turned:+6.1f}°"
    )
    if clip.loop and clip.end is not None:
        line += f"  loop: per cycle {math.hypot(*clip.end[:2]) * 1000:5.1f} mm, {math.degrees(heading(clip.end[3:7])):+5.1f}°"
    if clip.notes:
        line += f"  {clip.notes}"
    return line


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--rl", type=Path, required=True, help="a microduck_rl checkout")
    parser.add_argument("--out", type=Path, required=True, help="directory to write motions.json into")
    parser.add_argument("--cache", type=Path, default=Path.home() / ".cache" / "virtual-duck")
    args = parser.parse_args()

    rl = args.rl.expanduser()
    policies, manifest = fetch_policies(args.cache.expanduser())
    recorder = Recorder(load_harness(rl), rl, policies, manifest)

    recorder.stand()
    # A full stick on the real duck: robotd's `pad_drive` defaults, which say these are what the v8
    # velstand follows well. It stands still below about 0.25 m/s, so a slower walk is not on offer.
    recorder.locomotion("walk", vx=0.4)
    recorder.locomotion("turn_left", wz=1.0)
    recorder.locomotion("turn_right", wz=-1.0)
    recorder.sit()
    recorder.peck()
    recorder.episodic("kick_left", scene="scene_ball.xml")
    recorder.episodic("kick_right", scene="scene_ball.xml")
    recorder.episodic("roulade")
    # Pushed from the front it falls backward, and so on.
    recorder.fall("fall_backward", -1.2, 0.0)
    recorder.fall("fall_forward", 1.2, 0.0)
    recorder.fall("fall_left", 0.0, 1.2)
    recorder.fall("fall_right", 0.0, -1.2)
    recorder.dance()

    repo, version = policy_pin()
    frame_size = 7 + len(recorder.joint_names)
    index = {
        "source": {"policies": f"{repo}@{version}", "rate_hz": RATE_HZ},
        "joints": recorder.joint_names,
        # Per frame: trunk x y z (m), trunk quaternion w x y z, then one angle per joint (rad),
        # relative to where the duck stood when the clip began.
        "frame_size": frame_size,
        # Offsets count floats into motions.bin; -1 where a clip has no such block.
        "clips": {},
    }
    # The frames themselves are one flat block of little-endian float32. A phone reads that with
    # one bulk copy; the same numbers as JSON took longer to parse than the app took to start.
    blocks: list[np.ndarray] = []
    offset = 0

    def block(values: np.ndarray) -> int:
        nonlocal offset
        at = offset
        flat = np.ascontiguousarray(values, dtype="<f4").reshape(-1)
        blocks.append(flat)
        offset += flat.size
        return at

    global JOINT_ORDER
    JOINT_ORDER = recorder.joint_names
    for name, clip in recorder.clips.items():
        index["clips"][name] = {
            "loop": clip.loop,
            "frames": len(clip.frames),
            "offset": block(clip.frames),
            "end": block(clip.end) if clip.end is not None else -1,
            "ball": block(clip.ball) if clip.ball is not None else -1,
        }
        if name == "peck":
            index["clips"][name]["events"] = peck_events(rl, clip)
        print(report(name, clip))
    print(f"peck bite: {index['clips']['peck']['events']}")

    dest = args.out.expanduser()
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "motions.bin").write_bytes(np.concatenate(blocks).tobytes())
    (dest / "motions.json").write_text(json.dumps(index, indent=1) + "\n")
    print(f"{dest / 'motions.bin'}: {(dest / 'motions.bin').stat().st_size / 1e6:.2f} MB, {len(index['clips'])} clips")


if __name__ == "__main__":
    sys.exit(main())
