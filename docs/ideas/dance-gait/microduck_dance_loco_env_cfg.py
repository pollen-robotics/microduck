"""Microduck locomoting hop-dance — travel + alternate support on the beat.

Must match the mix table in docs/ideas/dance-gait/locomote-and-hop.md and
(when wired) pet-detect DanceDirector. Do not sample uniform lin_vel_*
independently.

    body_x, body_y, body_yaw = sin(2πφ), cos(2πφ), energy   # unchanged
    twist = phrase mix, resampled every 1–3 phase wraps (play 2–6, slewed)
    obs stays 61-D
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.manager_based_rl_env import ManagerBasedRlEnv
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers import CommandTermCfg, ObservationTermCfg, RewardTermCfg
from mjlab.managers.command_manager import CommandTerm
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg
from mjlab.sensor import ContactSensor
from mjlab.tasks.velocity import mdp as vel_mdp
from mjlab.tasks.velocity.mdp.rewards import feet_air_time

from mjlab_microduck.tasks.microduck_dance_env_cfg import (
    AZ_MAX,
    BPM_MAX,
    BPM_MIN,
    BeatPhaseCommand,
    BeatPhaseCommandCfg,
    ZERO_ENERGY_PROB,
    dance_z_tracking,
)
from mjlab_microduck.tasks.microduck_standup_env_cfg import STAND_Z
from mjlab_microduck.tasks.microduck_velocity_env_cfg import (
    make_microduck_velocity_env_cfg,
)
from mjlab_microduck.tasks.symmetry import PpoWithSymmetryCfg

# Train mix — keep the box the hop2 policy saw.
MIX_INPLACE = 0.15
MIX_TRANSLATE = 0.55
MIX_SIDESTEP = 0.20
MIX_SPIN = 0.10
# Play / DanceDirector — livelier. Policy tracks twist, so mix can change
# without a retrain (locomote-and-hop.md locked decision 2).
PLAY_INPLACE = 0.20
PLAY_TRANSLATE = 0.28
PLAY_SIDESTEP = 0.27
PLAY_SPIN = 0.25
VX_RANGE = (-0.10, 0.20)
VY_RANGE = (-0.12, 0.12)
VYAW_SMALL = 0.25
VYAW_SPIN = (0.40, 0.80)
PHRASE_BEATS_MIN = 2
PHRASE_BEATS_MAX = 4
PLAY_PHRASE_BEATS_MIN = 2
PLAY_PHRASE_BEATS_MAX = 6
PLAY_SLEW_TAU = 0.45
PLAY_TWIST_WANDER = 0.018
SWAP_WINDOW = 0.08
EPISODE_LENGTH_S = 20.0
KIND_INPLACE = 0
KIND_TRANSLATE = 1
KIND_SIDESTEP = 2
KIND_SPIN = 3


def sample_mix_table(
    n: int,
    *,
    generator: torch.Generator | None = None,
    device: torch.device | str = "cpu",
    inplace: float = MIX_INPLACE,
    translate: float = MIX_TRANSLATE,
    sidestep: float = MIX_SIDESTEP,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Draw n phrase twists. Returns (twist [N,3], kind [N] long)."""
    device = torch.device(device)
    u = torch.rand(n, generator=generator, device=device)
    kind = torch.empty(n, dtype=torch.long, device=device)
    kind = torch.where(u < inplace, torch.full_like(kind, KIND_INPLACE), kind)
    kind = torch.where(
        (u >= inplace) & (u < inplace + translate),
        torch.full_like(kind, KIND_TRANSLATE),
        kind,
    )
    kind = torch.where(
        (u >= inplace + translate) & (u < inplace + translate + sidestep),
        torch.full_like(kind, KIND_SIDESTEP),
        kind,
    )
    kind = torch.where(
        u >= inplace + translate + sidestep,
        torch.full_like(kind, KIND_SPIN),
        kind,
    )
    vx = torch.zeros(n, device=device)
    vy = torch.zeros(n, device=device)
    yaw = torch.zeros(n, device=device)

    def _u(lo: float, hi: float, k: int) -> torch.Tensor:
        return lo + (hi - lo) * torch.rand(k, generator=generator, device=device)

    def _signed(lo: float, hi: float, k: int) -> torch.Tensor:
        sign = torch.where(
            torch.rand(k, generator=generator, device=device) < 0.5,
            torch.full((k,), -1.0, device=device),
            torch.ones(k, device=device),
        )
        return sign * _u(lo, hi, k)

    m = kind == KIND_TRANSLATE
    if bool(m.any()):
        k = int(m.sum().item())
        vx[m] = _u(*VX_RANGE, k)
        vy[m] = _u(*VY_RANGE, k)
        yaw[m] = _u(-VYAW_SMALL, VYAW_SMALL, k)
    m = kind == KIND_SIDESTEP
    if bool(m.any()):
        k = int(m.sum().item())
        vy[m] = _u(*VY_RANGE, k)
        yaw[m] = _u(-VYAW_SMALL, VYAW_SMALL, k)
    m = kind == KIND_SPIN
    if bool(m.any()):
        k = int(m.sum().item())
        yaw[m] = _signed(*VYAW_SPIN, k)
    return torch.stack([vx, vy, yaw], dim=1), kind


class DanceTwistCommand(CommandTerm):
    """3-D twist from the mix table; resample after a random number of φ wraps."""

    cfg: "DanceTwistCommandCfg"

    def __init__(self, cfg: "DanceTwistCommandCfg", env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(self.num_envs, 3, device=self.device)
        self._prev_phase = torch.zeros(self.num_envs, device=self.device)
        self._beats = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._phrase_len = torch.full(
            (self.num_envs,),
            int(self.cfg.phrase_beats_max),
            dtype=torch.long,
            device=self.device,
        )
        self._kind = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._target = torch.zeros(self.num_envs, 3, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self) -> None:
        self.metrics["mix_inplace"] = (self._kind == KIND_INPLACE).float()
        self.metrics["mix_translate"] = (self._kind == KIND_TRANSLATE).float()
        self.metrics["mix_sidestep"] = (self._kind == KIND_SIDESTEP).float()
        self.metrics["mix_spin"] = (self._kind == KIND_SPIN).float()
        self.metrics["twist_speed_xy"] = torch.linalg.norm(self._command[:, :2], dim=1)

    def _beat_term(self) -> BeatPhaseCommand:
        term = self._env.command_manager.get_term("body_pose")
        assert isinstance(term, BeatPhaseCommand)
        return term

    def _apply_sample(self, env_ids: torch.Tensor, *, snap: bool) -> None:
        n = len(env_ids)
        if n == 0:
            return
        twist, kind = sample_mix_table(
            n,
            device=self.device,
            inplace=self.cfg.inplace,
            translate=self.cfg.translate,
            sidestep=self.cfg.sidestep,
        )
        energy = self._beat_term().energy[env_ids]
        silent = energy <= 1e-4
        twist[silent] = 0.0
        kind[silent] = KIND_INPLACE
        self._target[env_ids] = twist
        if snap or float(self.cfg.slew_tau) <= 0.0:
            self._command[env_ids] = twist
        self._kind[env_ids] = kind
        self._beats[env_ids] = 0
        lo = int(self.cfg.phrase_beats_min)
        hi = int(self.cfg.phrase_beats_max)
        self._phrase_len[env_ids] = torch.randint(
            lo, hi + 1, (n,), device=self.device
        )

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        beat = self._beat_term()
        self._prev_phase[env_ids] = beat.phase[env_ids]
        self._apply_sample(env_ids, snap=True)

    def _update_command(self) -> None:
        beat = self._beat_term()
        wrapped = (beat.phase < (self._prev_phase - 0.5)) & (beat.energy > 0.0)
        self._beats = torch.where(wrapped, self._beats + 1, self._beats)
        self._prev_phase = beat.phase.clone()
        due = (self._beats >= self._phrase_len).nonzero().flatten()
        if len(due) > 0:
            self._apply_sample(due, snap=False)
        tau = float(self.cfg.slew_tau)
        if tau > 0.0:
            dt = float(self._env.step_dt)
            alpha = 1.0 - math.exp(-dt / tau)
            self._command = self._command + alpha * (self._target - self._command)
            wander = float(self.cfg.wander)
            if wander > 0.0:
                self._command = (
                    self._command
                    + wander
                    * math.sqrt(dt)
                    * torch.randn_like(self._command)
                )
            self._command[:, 0].clamp_(*VX_RANGE)
            self._command[:, 1].clamp_(*VY_RANGE)
            self._command[:, 2].clamp_(-VYAW_SPIN[1], VYAW_SPIN[1])
        silent = beat.energy <= 1e-4
        self._command[silent] = 0.0
        self._target[silent] = 0.0
        self._kind[silent] = KIND_INPLACE


@dataclass(kw_only=True)
class DanceTwistCommandCfg(CommandTermCfg):
    inplace: float = MIX_INPLACE
    translate: float = MIX_TRANSLATE
    sidestep: float = MIX_SIDESTEP
    phrase_beats_min: int = PHRASE_BEATS_MIN
    phrase_beats_max: int = PHRASE_BEATS_MAX
    slew_tau: float = 0.0
    wander: float = 0.0

    def build(self, env: ManagerBasedRlEnv) -> DanceTwistCommand:
        return DanceTwistCommand(self, env)


def _phase_from_body(cmd: torch.Tensor) -> torch.Tensor:
    return (torch.atan2(cmd[:, 0], cmd[:, 1]) / (2.0 * math.pi)) % 1.0


def _swap_window(phase: torch.Tensor) -> torch.Tensor:
    near_zero = (phase < SWAP_WINDOW) | (phase > (1.0 - SWAP_WINDOW))
    near_half = (phase - 0.5).abs() < SWAP_WINDOW
    return near_zero | near_half


def dance_foot_phase(
    env: ManagerBasedRlEnv,
    body_command_name: str = "body_pose",
    sensor_name: str = "feet_ground_contact",
) -> torch.Tensor:
    """Left stance on φ∈[0,0.5), right on [0.5,1). Skip the swap window."""
    sensor: ContactSensor = env.scene[sensor_name]
    found = sensor.data.found
    assert found is not None
    left = found[:, 0] > 0
    right = found[:, 1] > 0
    cmd = env.command_manager.get_command(body_command_name)
    energy = cmd[:, 5].clamp(0.0, 1.0)
    phase = _phase_from_body(cmd)
    silent = energy <= 1e-4
    want_left = phase < 0.5
    left_ok = torch.where(want_left, left, ~left)
    right_ok = torch.where(want_left, ~right, right)
    reward = 0.5 * (left_ok.float() + right_ok.float())
    reward = torch.where(_swap_window(phase), torch.ones_like(reward), reward)
    both_down = (left & right).float()
    return torch.where(silent, both_down, reward)


def dance_air_time(
    env: ManagerBasedRlEnv,
    sensor_name: str = "feet_ground_contact",
    threshold_min: float = 0.08,
    threshold_max: float = 0.28,
    body_command_name: str = "body_pose",
) -> torch.Tensor:
    """Swing-foot air time while energy > 0, including in-place phrases.

    The walk `feet_air_time` term zeros itself when |twist| is below
    `command_threshold`, which hid hopping on the 15% in-place mix.
    """
    reward = feet_air_time(
        env,
        sensor_name=sensor_name,
        threshold_min=threshold_min,
        threshold_max=threshold_max,
        command_name=None,
    )
    energy = env.command_manager.get_command(body_command_name)[:, 5]
    return reward * (energy > 1e-4).float()


def dance_both_airborne(
    env: ManagerBasedRlEnv,
    sensor_name: str = "feet_ground_contact",
) -> torch.Tensor:
    sensor: ContactSensor = env.scene[sensor_name]
    found = sensor.data.found
    assert found is not None
    left = found[:, 0] > 0
    right = found[:, 1] > 0
    return ((~left) & (~right)).float()


def make_microduck_dance_loco_env_cfg(
    play: bool = False,
    rough: bool = False,
) -> ManagerBasedRlEnvCfg:
    """Locomoting hop-dance: small twist box + beat in unbound body slots."""
    cfg = make_microduck_velocity_env_cfg(play=play, rough=rough)
    cfg.episode_length_s = EPISODE_LENGTH_S

    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg)
    joint_pos_action.scale = 1.0

    for name in ["foot_clearance", "foot_swing_height"]:
        cfg.rewards.pop(name, None)

    body_pose = BeatPhaseCommandCfg(
        resampling_time_range=(4.0, 8.0) if not play else (1e6, 1e6),
        bpm_min=88.0 if play else BPM_MIN,
        bpm_max=124.0 if play else BPM_MAX,
        energy_min=0.55 if play else 0.35,
        energy_max=1.0,
        zero_energy_prob=0.0 if play else ZERO_ENERGY_PROB,
        resample_phase=not play,
        organic=play,
        groove=0.18 if play else 0.0,
        bpm_wander=14.0 if play else 0.0,
        energy_wander=0.12 if play else 0.0,
        hesitate_prob=0.12 if play else 0.0,
        lead_flip_prob=0.10 if play else 0.0,
    )
    twist = DanceTwistCommandCfg(
        resampling_time_range=(1e6, 1e6),
        debug_vis=True,
        inplace=PLAY_INPLACE if play else MIX_INPLACE,
        translate=PLAY_TRANSLATE if play else MIX_TRANSLATE,
        sidestep=PLAY_SIDESTEP if play else MIX_SIDESTEP,
        phrase_beats_min=PLAY_PHRASE_BEATS_MIN if play else PHRASE_BEATS_MIN,
        phrase_beats_max=PLAY_PHRASE_BEATS_MAX if play else PHRASE_BEATS_MAX,
        slew_tau=PLAY_SLEW_TAU if play else 0.0,
        wander=PLAY_TWIST_WANDER if play else 0.0,
    )
    rest = {k: v for k, v in cfg.commands.items() if k not in ("twist", "body_pose")}
    cfg.commands = {"body_pose": body_pose, "twist": twist, **rest}

    for group in ("actor", "critic"):
        cfg.observations[group].terms["body_command"] = ObservationTermCfg(
            func=vel_mdp.generated_commands,
            params={"command_name": "body_pose"},
        )

    cfg.rewards["dance_z_tracking"] = RewardTermCfg(
        func=dance_z_tracking,
        weight=1.5,
        params={
            "command_name": "body_pose",
            "stand_z": STAND_Z,
            "az_max": AZ_MAX,
            "std": 0.012,
        },
    )
    cfg.rewards["dance_foot_phase"] = RewardTermCfg(
        func=dance_foot_phase,
        weight=5.0,
        params={
            "body_command_name": "body_pose",
            "sensor_name": "feet_ground_contact",
        },
    )
    cfg.rewards["dance_both_airborne"] = RewardTermCfg(
        func=dance_both_airborne,
        weight=-2.0,
        params={"sensor_name": "feet_ground_contact"},
    )
    if "body_pose_tracking" in cfg.rewards:
        cfg.rewards["body_pose_tracking"].weight = 0.0

    cfg.rewards["track_linear_velocity"].weight = 1.0
    cfg.rewards["track_angular_velocity"].weight = 0.5
    cfg.rewards["air_time"] = RewardTermCfg(
        func=dance_air_time,
        weight=2.0,
        params={
            "sensor_name": "feet_ground_contact",
            "threshold_min": 0.08,
            "threshold_max": 0.28,
            "body_command_name": "body_pose",
        },
    )
    cfg.rewards["foot_slip"].weight = -0.4
    cfg.rewards["foot_slip"].params["command_threshold"] = 0.02
    cfg.rewards["foot_slip"].params["sensor_name"] = "feet_ground_contact"

    cfg.curriculum.pop("standing_envs", None)
    cfg.curriculum.pop("command_vel", None)
    cfg.curriculum.pop("body_pose_range", None)
    cfg.curriculum.pop("terrain_levels", None)

    if play:
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.viewer.distance = 0.55
        cfg.viewer.elevation = -18.0
        cfg.viewer.azimuth = 50.0
        cfg.viewer.height = 720
        cfg.viewer.width = 1280

    return cfg


MicroduckDanceLocoRlCfg = RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
        hidden_dims=(512, 256, 128),
        activation="elu",
        obs_normalization=True,
        distribution_cfg={
            "class_name": "GaussianDistribution",
            "init_std": 1.0,
            "std_type": "scalar",
        },
    ),
    critic=RslRlModelCfg(
        hidden_dims=(512, 256, 128),
        activation="elu",
        obs_normalization=True,
    ),
    algorithm=PpoWithSymmetryCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        symmetry_cfg=None,
    ),
    wandb_project="mjlab_microduck",
    experiment_name="microduck_dance_loco",
    run_name="microduck_dance_loco",
    save_interval=250,
    num_steps_per_env=24,
    max_iterations=15_000,
)


if __name__ == "__main__":
    g = torch.Generator().manual_seed(1)
    _, kind = sample_mix_table(10_000, generator=g)
    counts = torch.bincount(kind, minlength=4).float() / 10_000
    print("mix histogram", counts.tolist())
    expected = [MIX_INPLACE, MIX_TRANSLATE, MIX_SIDESTEP, MIX_SPIN]
    err = (counts - torch.tensor(expected)).abs().max().item()
    assert err < 0.02, err
    _, play_kind = sample_mix_table(
        10_000,
        generator=torch.Generator().manual_seed(1),
        inplace=PLAY_INPLACE,
        translate=PLAY_TRANSLATE,
        sidestep=PLAY_SIDESTEP,
    )
    play_counts = torch.bincount(play_kind, minlength=4).float() / 10_000
    play_expected = [PLAY_INPLACE, PLAY_TRANSLATE, PLAY_SIDESTEP, PLAY_SPIN]
    play_err = (play_counts - torch.tensor(play_expected)).abs().max().item()
    assert play_err < 0.02, play_err
    print("mix table ok")
