"""Microduck dance gait — standing bob conditioned on beat phase.

The observation stays 61-D. Beat phase and energy occupy the three unbound
command slots the runtime currently hardcodes to zero:

    body_x   = sin(2π φ)
    body_y   = cos(2π φ)
    body_yaw = energy
    body_z / roll / pitch = 0   # those axes belong to the *stand* policy's pose box

A newly trained network may read those three; stuffing them into today's stand
weights would be out of distribution.

Reward: trunk height tracks STAND_Z plus the crouch waveform (most negative at
phase 0, zero at energy 0), plus upright / feet-down / smoothness. BPM is
randomized in 60–140.
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
from mjlab.tasks.velocity import mdp as vel_mdp

from mjlab_microduck.tasks import mdp as microduck_mdp
from mjlab_microduck.tasks.microduck_standup_env_cfg import STAND_Z
from mjlab_microduck.tasks.microduck_velocity_env_cfg import (
    make_microduck_velocity_env_cfg,
)
from mjlab_microduck.tasks.symmetry import PpoWithSymmetryCfg

BPM_MIN = 60.0
BPM_MAX = 140.0
# Crouch amplitude at energy 1 — the runtime pose box, not standup's wider one.
AZ_MAX = 0.025
EPISODE_LENGTH_S = 12.0
ZERO_ENERGY_PROB = 0.3


class BeatPhaseCommand(CommandTerm):
    """6-D body_pose command: [sin, cos, 0, 0, 0, energy]. Phase advances online."""

    cfg: "BeatPhaseCommandCfg"

    def __init__(self, cfg: "BeatPhaseCommandCfg", env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(self.num_envs, 6, device=self.device)
        self.phase = torch.zeros(self.num_envs, device=self.device)
        self.bpm = torch.full((self.num_envs,), 100.0, device=self.device)
        self.energy = torch.ones(self.num_envs, device=self.device)
        self._bpm_center = self.bpm.clone()
        self._energy_center = self.energy.clone()
        self._groove = torch.zeros(self.num_envs, device=self.device)
        self._lead = torch.zeros(self.num_envs, device=self.device)
        self._hold = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._prev_phase = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return self._command

    def _update_metrics(self) -> None:
        pass

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        n = len(env_ids)
        if n == 0:
            return
        self.bpm[env_ids] = torch.empty(n, device=self.device).uniform_(
            self.cfg.bpm_min, self.cfg.bpm_max
        )
        self.energy[env_ids] = torch.empty(n, device=self.device).uniform_(
            self.cfg.energy_min, self.cfg.energy_max
        )
        self._bpm_center[env_ids] = self.bpm[env_ids]
        self._energy_center[env_ids] = self.energy[env_ids]
        if self.cfg.resample_phase:
            self.phase[env_ids] = torch.empty(n, device=self.device).uniform_(0.0, 1.0)
        if self.cfg.organic:
            g = float(self.cfg.groove)
            self._groove[env_ids] = g * (
                0.65 + 0.35 * torch.rand(n, device=self.device)
            )
            self._lead[env_ids] = torch.where(
                torch.rand(n, device=self.device) < 0.5,
                torch.zeros(n, device=self.device),
                torch.full((n,), 0.5, device=self.device),
            )
            self._hold[env_ids] = 0
        silent = torch.rand(n, device=self.device) < self.cfg.zero_energy_prob
        self.energy[env_ids[silent]] = 0.0
        self._write_command()

    def _update_command(self) -> None:
        dt = float(self._env.step_dt)
        active = self.energy > 0.0
        if self.cfg.organic:
            self._organic_step(dt, active)
        else:
            self.phase = torch.where(
                active,
                (self.phase + self.bpm / 60.0 * dt) % 1.0,
                self.phase,
            )
        self._write_command()

    def _organic_step(self, dt: float, active: torch.Tensor) -> None:
        """Wander tempo/energy, swing the displayed beat, hesitate now and then."""
        n = self.num_envs
        device = self.device
        sqrt_dt = math.sqrt(dt)
        self.bpm = (
            self.bpm
            + 1.4 * (self._bpm_center - self.bpm) * dt
            + float(self.cfg.bpm_wander) * sqrt_dt * torch.randn(n, device=device)
        )
        self.bpm.clamp_(self.cfg.bpm_min, self.cfg.bpm_max)
        self.energy = (
            self.energy
            + 0.9 * (self._energy_center - self.energy) * dt
            + float(self.cfg.energy_wander) * sqrt_dt * torch.randn(n, device=device)
        )
        self.energy.clamp_(self.cfg.energy_min, self.cfg.energy_max)

        self._groove = (
            self._groove + 0.05 * sqrt_dt * torch.randn(n, device=device)
        ).clamp(0.05, max(0.08, 1.4 * float(self.cfg.groove)))
        wrapped = (self.phase < (self._prev_phase - 0.5)) & active
        flip = wrapped & (torch.rand(n, device=device) < float(self.cfg.lead_flip_prob))
        self._lead = torch.where(flip, 0.5 - self._lead, self._lead)
        start_hold = wrapped & (
            torch.rand(n, device=device) < float(self.cfg.hesitate_prob)
        )
        if bool(start_hold.any()):
            k = int(start_hold.sum().item())
            # ~80–220 ms at 50 Hz control.
            self._hold[start_hold] = torch.randint(4, 12, (k,), device=device)
        holding = self._hold > 0
        self._hold = torch.where(holding, self._hold - 1, self._hold)
        moving = active & ~holding
        self._prev_phase = self.phase.clone()
        self.phase = torch.where(
            moving,
            (self.phase + self.bpm / 60.0 * dt) % 1.0,
            self.phase,
        )

    def _write_command(self) -> None:
        two_pi = 2.0 * math.pi
        active = self.energy > 0.0
        phi = self.phase
        if self.cfg.organic:
            # Late backbeat so foot swaps are not a metronome click.
            phi = (phi + self._lead + self._groove * torch.sin(two_pi * phi)) % 1.0
        s = torch.sin(two_pi * phi)
        c = torch.cos(two_pi * phi)
        zeros = torch.zeros_like(s)
        energy = self.energy
        if self.cfg.organic:
            energy = energy * (0.86 + 0.14 * (0.5 + 0.5 * torch.sin(two_pi * phi)))
        self._command[:, 0] = torch.where(active, s, zeros)
        self._command[:, 1] = torch.where(active, c, zeros)
        self._command[:, 2] = 0.0
        self._command[:, 3] = 0.0
        self._command[:, 4] = 0.0
        self._command[:, 5] = torch.where(active, energy, zeros)


@dataclass(kw_only=True)
class BeatPhaseCommandCfg(CommandTermCfg):
    bpm_min: float = BPM_MIN
    bpm_max: float = BPM_MAX
    energy_min: float = 0.35
    energy_max: float = 1.0
    zero_energy_prob: float = ZERO_ENERGY_PROB
    resample_phase: bool = True
    organic: bool = False
    groove: float = 0.0
    bpm_wander: float = 0.0
    energy_wander: float = 0.0
    hesitate_prob: float = 0.0
    lead_flip_prob: float = 0.0

    def build(self, env: ManagerBasedRlEnv) -> BeatPhaseCommand:
        return BeatPhaseCommand(self, env)


def dance_z_tracking(
    env: ManagerBasedRlEnv,
    command_name: str = "body_pose",
    stand_z: float = STAND_Z,
    az_max: float = AZ_MAX,
    std: float = 0.012,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Gaussian on trunk z vs the crouch waveform implied by the beat command."""
    asset = env.scene[asset_cfg.name]
    z = asset.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]
    z = torch.nan_to_num(z, nan=stand_z)
    cmd = env.command_manager.get_command(command_name)
    energy = cmd[:, 5].clamp(0.0, 1.0)
    # atan2(sin, cos) recovers φ even if the vector was scaled.
    phase = (torch.atan2(cmd[:, 0], cmd[:, 1]) / (2.0 * math.pi)) % 1.0
    silent = energy <= 1e-4
    target = stand_z - az_max * energy * 0.5 * (1.0 + torch.cos(2.0 * math.pi * phase))
    target = torch.where(silent, torch.full_like(target, stand_z), target)
    err = z - target
    return torch.exp(-((err / std) ** 2))


def make_microduck_dance_env_cfg(
    play: bool = False,
    rough: bool = False,
) -> ManagerBasedRlEnvCfg:
    """Standing dance: velocity base, twist ~ 0, beat in unbound body slots."""
    cfg = make_microduck_velocity_env_cfg(play=play, rough=rough)
    cfg.episode_length_s = EPISODE_LENGTH_S

    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg)
    # Match the stand policy's scale so export is drop-in for a standing net.
    joint_pos_action.scale = 1.0

    for name in [
        "track_linear_velocity",
        "track_angular_velocity",
        "air_time",
        "foot_clearance",
        "foot_swing_height",
        "foot_slip",
    ]:
        cfg.rewards.pop(name, None)

    cfg.commands["body_pose"] = BeatPhaseCommandCfg(
        resampling_time_range=(4.0, 8.0) if not play else (1e6, 1e6),
        bpm_min=100.0 if play else BPM_MIN,
        bpm_max=100.0 if play else BPM_MAX,
        zero_energy_prob=0.0 if play else ZERO_ENERGY_PROB,
    )
    for group in ("actor", "critic"):
        cfg.observations[group].terms["body_command"] = ObservationTermCfg(
            func=vel_mdp.generated_commands,
            params={"command_name": "body_pose"},
        )

    cfg.rewards["dance_z_tracking"] = RewardTermCfg(
        func=dance_z_tracking,
        weight=2.0,
        params={
            "command_name": "body_pose",
            "stand_z": STAND_Z,
            "az_max": AZ_MAX,
            "std": 0.012,
        },
    )
    # Disable the unused 6-D pose tracker (weight 0 on velocity); the z term above
    # is the dance-specific one.
    if "body_pose_tracking" in cfg.rewards:
        cfg.rewards["body_pose_tracking"].weight = 0.0

    command = cfg.commands["twist"]
    command.rel_standing_envs = 1.0
    command.heading_command = False
    command.ranges.heading = None
    command.resampling_time_range = (EPISODE_LENGTH_S, EPISODE_LENGTH_S * 2)
    command.ranges.lin_vel_x = (0.0, 0.0)
    command.ranges.lin_vel_y = (0.0, 0.0)
    command.ranges.ang_vel_z = (0.0, 0.0)
    cfg.commands["twist"] = microduck_mdp.VelocityCommandCommandOnlyCfg(**vars(command))

    cfg.curriculum.pop("standing_envs", None)
    cfg.curriculum.pop("command_vel", None)
    cfg.curriculum.pop("body_pose_range", None)
    cfg.curriculum.pop("terrain_levels", None)

    if play:
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)

    return cfg


MicroduckDanceRlCfg = RslRlOnPolicyRunnerCfg(
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
    experiment_name="microduck_dance",
    run_name="microduck_dance",
    save_interval=250,
    num_steps_per_env=24,
    max_iterations=15_000,
)
