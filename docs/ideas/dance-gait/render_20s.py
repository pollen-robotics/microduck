import os, torch
from dataclasses import asdict
from pathlib import Path
import mjlab.tasks  # noqa
import mjlab_microduck.tasks  # noqa
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.utils.wrappers import VideoRecorder

task = "Mjlab-Dance-Loco-Flat-MicroDuck"
ckpt = Path(os.environ.get(
    "DANCE_LOCO_CKPT",
    "/home/mstaub/jobs/feat-dancing-skill-loco-1bf7770a/microduck_rl/logs/rsl_rl/microduck_dance_loco/2026-09-05_15-44-58_microduck_dance_loco/model_23997.pt",
))
device = os.environ.get("DANCE_LOCO_DEVICE", "cuda:0")
try:
    env_cfg = load_env_cfg(task, play=True)
except TypeError:
    env_cfg = load_env_cfg(task)
    print("WARN: load_env_cfg has no play=; using train cfg")
agent_cfg = load_rl_cfg(task)
env_cfg.scene.num_envs = 1
env_cfg.episode_length_s = 25.0
if hasattr(env_cfg, "viewer") and env_cfg.viewer is not None:
    env_cfg.viewer.height = 720
    env_cfg.viewer.width = 1280
    print("viewer", env_cfg.viewer.height, env_cfg.viewer.width, "dist", getattr(env_cfg.viewer, "distance", None))

env = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode="rgb_array")
print("step_dt", getattr(env, "step_dt", None), "ckpt", ckpt)
env.metadata["render_fps"] = 50
video_dir = Path(os.environ.get("DANCE_LOCO_VIDEO_DIR", "/tmp/dance_loco20s"))
video_dir.mkdir(parents=True, exist_ok=True)
env = VideoRecorder(env, video_folder=video_dir, step_trigger=lambda s: s == 0, video_length=1000, disable_logger=True)
env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
runner_cls = load_runner_cls(task)
runner = runner_cls(env, asdict(agent_cfg), device=device)
runner.load(str(ckpt), load_cfg={"actor": True}, strict=True, map_location=device)
policy = runner.get_inference_policy(device=device)

try:
    raw = env.unwrapped
    term = raw.command_manager.get_term("body_pose")
    term.phase[:] = 0.0
    if hasattr(term, "_write_command"):
        term._write_command()
    print("play bpm", float(term.bpm[0]), "energy", float(term.energy[0]), "no pin")
    twist = raw.command_manager.get_term("twist")
    print(
        "twist mix",
        twist.cfg.inplace,
        twist.cfg.translate,
        twist.cfg.sidestep,
        "phrase",
        twist.cfg.phrase_beats_min,
        twist.cfg.phrase_beats_max,
        "slew",
        getattr(twist.cfg, "slew_tau", 0.0),
        "wander",
        getattr(twist.cfg, "wander", 0.0),
    )
    print(
        "organic",
        getattr(term.cfg, "organic", False),
        "groove",
        getattr(term.cfg, "groove", 0.0),
        "hesitate",
        getattr(term.cfg, "hesitate_prob", 0.0),
    )
except Exception as e:
    print("could not inspect commands:", type(e).__name__, e)

obs = env.get_observations()
for i in range(1000):
    with torch.no_grad():
        actions = policy(obs)
    obs, _, dones, _ = env.step(actions)
env.close()
for root, _, files in os.walk(video_dir):
    for f in files:
        p = os.path.join(root, f)
        print("VIDEO", p, os.path.getsize(p))
