"""Bounded high-level Twist action for the twelve Carbot wheel joints."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass


class CarbotTwistAction(ActionTerm):
    """Map physical-unit ``[linear.x, angular.z]`` to wheel velocities.

    Velocity, acceleration, coupled wheel saturation, joint-coordinate sign,
    and stale-command handling are applied here so a policy cannot bypass the
    same safety boundary used by ``/cmd_vel``. PWM and torque are deliberately
    outside this action space.
    """

    cfg: "CarbotTwistActionCfg"
    _asset: Articulation

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        left_ids, left_names = self._asset.find_joints(cfg.left_joint_expr)
        right_ids, right_names = self._asset.find_joints(cfg.right_joint_expr)
        if len(left_ids) != 6 or len(right_ids) != 6:
            raise ValueError(
                "CarbotTwistAction requires six left and six right wheel "
                f"joints; found {left_names} and {right_names}"
            )
        self._left_joint_ids = left_ids
        self._right_joint_ids = right_ids
        self._raw_actions = torch.zeros(
            self.num_envs, self.action_dim, device=self.device
        )
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._target_actions = torch.zeros_like(self._raw_actions)
        self._command_age_s = torch.zeros(
            self.num_envs, 1, device=self.device
        )
        self._left_targets = torch.zeros(
            self.num_envs, len(left_ids), device=self.device
        )
        self._right_targets = torch.zeros(
            self.num_envs, len(right_ids), device=self.device
        )

    @property
    def action_dim(self):
        return 2

    @property
    def raw_actions(self):
        return self._raw_actions

    @property
    def processed_actions(self):
        return self._processed_actions

    def process_actions(self, actions):
        if actions.shape != self._raw_actions.shape:
            raise ValueError(
                f"Expected action shape {self._raw_actions.shape}, got "
                f"{actions.shape}"
            )
        self._raw_actions[:] = actions
        self._target_actions[:, 0] = torch.clamp(
            actions[:, 0],
            -self.cfg.max_linear_velocity_mps,
            self.cfg.max_linear_velocity_mps,
        )
        self._target_actions[:, 1] = torch.clamp(
            actions[:, 1],
            -self.cfg.max_angular_velocity_rad_s,
            self.cfg.max_angular_velocity_rad_s,
        )
        self._command_age_s.zero_()
        self._approach_target(self._env.step_dt)

    def _approach_target(self, dt_s):
        maximum_change = torch.tensor(
            [
                self.cfg.max_linear_acceleration_mps2 * dt_s,
                self.cfg.max_angular_acceleration_rad_s2 * dt_s,
            ],
            device=self.device,
        )
        delta = torch.clamp(
            self._target_actions - self._processed_actions,
            min=-maximum_change,
            max=maximum_change,
        )
        self._processed_actions.add_(delta)

    def _wheel_targets(self):
        linear = self._processed_actions[:, 0]
        angular = self._processed_actions[:, 1]
        half_separation = self.cfg.effective_track_separation_m / 2.0
        radius = self.cfg.effective_sprocket_radius_m
        left = (linear - angular * half_separation) / radius
        right = (linear + angular * half_separation) / radius
        peak = torch.maximum(torch.abs(left), torch.abs(right))
        scale = torch.clamp(
            self.cfg.max_wheel_velocity_rad_s / torch.clamp(peak, min=1e-9),
            max=1.0,
        )
        sign = self.cfg.wheel_joint_coordinate_sign
        self._left_targets[:] = (sign * left * scale).unsqueeze(1)
        self._right_targets[:] = (sign * right * scale).unsqueeze(1)

    def apply_actions(self):
        physics_dt = self._env.cfg.sim.dt
        self._command_age_s.add_(physics_dt)
        stale = self._command_age_s[:, 0] > self.cfg.watchdog_timeout_s
        if torch.any(stale):
            self._target_actions[stale] = 0.0
            self._approach_target(physics_dt)
        self._wheel_targets()
        self._asset.set_joint_velocity_target(
            self._left_targets, joint_ids=self._left_joint_ids
        )
        self._asset.set_joint_velocity_target(
            self._right_targets, joint_ids=self._right_joint_ids
        )

    def reset(self, env_ids: Sequence[int] | None = None):
        self._raw_actions[env_ids] = 0.0
        self._processed_actions[env_ids] = 0.0
        self._target_actions[env_ids] = 0.0
        self._command_age_s[env_ids] = 0.0


@configclass
class CarbotTwistActionCfg(ActionTermCfg):
    """Configuration for :class:`CarbotTwistAction`."""

    class_type: type[ActionTerm] = CarbotTwistAction
    left_joint_expr: str = "left_.*_wheel_joint"
    right_joint_expr: str = "right_.*_wheel_joint"
    max_linear_velocity_mps: float = MISSING
    max_angular_velocity_rad_s: float = MISSING
    max_linear_acceleration_mps2: float = MISSING
    max_angular_acceleration_rad_s2: float = MISSING
    max_wheel_velocity_rad_s: float = MISSING
    effective_track_separation_m: float = MISSING
    effective_sprocket_radius_m: float = MISSING
    wheel_joint_coordinate_sign: float = MISSING
    watchdog_timeout_s: float = MISSING
