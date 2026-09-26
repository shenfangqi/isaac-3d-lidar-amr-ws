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
        self._applied_actions = torch.zeros_like(self._raw_actions)
        self._wheel_scale = torch.ones(self.num_envs, device=self.device)
        self._command_age_s = torch.zeros(
            self.num_envs, 1, device=self.device
        )
        self._watchdog_active = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._watchdog_trigger_age_s = torch.full(
            (self.num_envs,), float("nan"), device=self.device
        )
        self._ideal_position_xy = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._ideal_yaw = torch.zeros(self.num_envs, device=self.device)
        self._ideal_initialized = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._left_targets = torch.zeros(
            self.num_envs, len(left_ids), device=self.device
        )
        self._right_targets = torch.zeros(
            self.num_envs, len(right_ids), device=self.device
        )
        self._left_wheel_travel_rad = torch.zeros_like(self._left_targets)
        self._right_wheel_travel_rad = torch.zeros_like(self._right_targets)

    @property
    def action_dim(self):
        return 2

    @property
    def raw_actions(self):
        return self._raw_actions

    @property
    def processed_actions(self):
        return self._processed_actions

    @property
    def applied_actions(self):
        return self._applied_actions

    @property
    def wheel_scale(self):
        return self._wheel_scale

    @property
    def left_wheel_travel_rad(self):
        return self._left_wheel_travel_rad

    @property
    def right_wheel_travel_rad(self):
        return self._right_wheel_travel_rad

    @property
    def watchdog_active(self):
        return self._watchdog_active

    @property
    def watchdog_trigger_age_s(self):
        return self._watchdog_trigger_age_s

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
        self._watchdog_active.zero_()
        self._watchdog_trigger_age_s.fill_(float("nan"))
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
        right_turn = angular < 0.0
        angular = torch.where(
            right_turn,
            angular
            * self.cfg.right_turn_command_scale
            * self.cfg.right_turn_response_gain_vs_left,
            angular,
        )
        half_separation = self.cfg.effective_track_separation_m / 2.0
        radius = self.cfg.effective_sprocket_radius_m
        left = (linear - angular * half_separation) / radius
        right = (linear + angular * half_separation) / radius
        peak = torch.maximum(torch.abs(left), torch.abs(right))
        scale = torch.clamp(
            self.cfg.max_wheel_velocity_rad_s / torch.clamp(peak, min=1e-9),
            max=1.0,
        )
        self._wheel_scale[:] = scale
        left = self._apply_track_deadband(left * scale)
        right = self._apply_track_deadband(right * scale)
        self._applied_actions[:, 0] = radius * (left + right) / 2.0
        self._applied_actions[:, 1] = (
            radius * (right - left) / self.cfg.effective_track_separation_m
        )
        sign = self.cfg.wheel_joint_coordinate_sign
        self._left_targets[:] = (sign * left).unsqueeze(1)
        self._right_targets[:] = (sign * right).unsqueeze(1)

    def _apply_track_deadband(self, wheel_velocity_rad_s):
        """Zero per-track targets below the measured directional threshold."""
        radius = self.cfg.effective_sprocket_radius_m
        forward = self.cfg.forward_track_deadband_mps / radius
        reverse = self.cfg.reverse_track_deadband_mps / radius
        threshold = torch.where(
            wheel_velocity_rad_s >= 0.0,
            torch.full_like(wheel_velocity_rad_s, forward),
            torch.full_like(wheel_velocity_rad_s, reverse),
        )
        return torch.where(
            torch.abs(wheel_velocity_rad_s) >= threshold,
            wheel_velocity_rad_s,
            torch.zeros_like(wheel_velocity_rad_s),
        )

    def _apply_ideal_planar_kinematics(self, dt_s):
        """Enforce the bounded planar velocity while calibration is pending.

        This mirrors the existing Isaac Sim ``ideal_kinematic`` runtime.  The
        wheel targets are still applied for joint animation and observations,
        but uncalibrated isotropic wheel contact is not allowed to redefine the
        commanded body twist.
        """
        pose = self._asset.data.root_pose_w.clone()
        quaternion = pose[:, 3:7]
        w, x, y, z = quaternion.unbind(dim=1)
        measured_yaw = torch.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        initialize = ~self._ideal_initialized
        self._ideal_position_xy[:] = torch.where(
            initialize.unsqueeze(1),
            pose[:, :2],
            self._ideal_position_xy,
        )
        self._ideal_yaw[:] = torch.where(
            initialize, measured_yaw, self._ideal_yaw
        )
        self._ideal_initialized[:] = True

        linear = self._applied_actions[:, 0]
        angular = self._applied_actions[:, 1]
        yaw = self._ideal_yaw
        pose[:, :2] = self._ideal_position_xy
        half_yaw = yaw / 2.0
        pose[:, 3] = torch.cos(half_yaw)
        pose[:, 4:6] = 0.0
        pose[:, 6] = torch.sin(half_yaw)
        velocity = self._asset.data.root_vel_w.clone()
        velocity[:, 0] = linear * torch.cos(yaw)
        velocity[:, 1] = linear * torch.sin(yaw)
        # Preserve vertical velocity so gravity/contact still settle the model.
        velocity[:, 3:5] = 0.0
        velocity[:, 5] = angular
        self._asset.write_root_pose_to_sim(pose)
        self._asset.write_root_velocity_to_sim(velocity)

        midpoint_yaw = yaw + angular * dt_s / 2.0
        self._ideal_position_xy[:, 0] += (
            linear * torch.cos(midpoint_yaw) * dt_s
        )
        self._ideal_position_xy[:, 1] += (
            linear * torch.sin(midpoint_yaw) * dt_s
        )
        self._ideal_yaw.add_(angular * dt_s)

    def apply_actions(self):
        physics_dt = self._env.cfg.sim.dt
        self._command_age_s.add_(physics_dt)
        stale = self._command_age_s[:, 0] > self.cfg.watchdog_timeout_s
        newly_stale = torch.logical_and(stale, ~self._watchdog_active)
        self._watchdog_trigger_age_s[:] = torch.where(
            newly_stale,
            self._command_age_s[:, 0],
            self._watchdog_trigger_age_s,
        )
        self._watchdog_active[:] = stale
        stale_mask = stale.unsqueeze(1)
        self._target_actions[:] = torch.where(
            stale_mask,
            torch.zeros_like(self._target_actions),
            self._target_actions,
        )
        maximum_change = torch.tensor(
            [
                self.cfg.max_linear_acceleration_mps2 * physics_dt,
                self.cfg.max_angular_acceleration_rad_s2 * physics_dt,
            ],
            device=self.device,
        )
        stale_delta = torch.clamp(
            -self._processed_actions,
            min=-maximum_change,
            max=maximum_change,
        )
        self._processed_actions[:] = torch.where(
            stale_mask,
            self._processed_actions + stale_delta,
            self._processed_actions,
        )
        self._wheel_targets()
        self._left_wheel_travel_rad.add_(self._left_targets * physics_dt)
        self._right_wheel_travel_rad.add_(self._right_targets * physics_dt)
        self._asset.set_joint_velocity_target(
            self._left_targets, joint_ids=self._left_joint_ids
        )
        self._asset.set_joint_velocity_target(
            self._right_targets, joint_ids=self._right_joint_ids
        )
        if self.cfg.ideal_kinematic:
            # Match the existing Isaac Sim ideal runtime: make joint state and
            # wheel animation obey the same bounded targets while the
            # high-fidelity torque/contact model remains uncalibrated.
            self._asset.write_joint_velocity_to_sim(
                self._left_targets, joint_ids=self._left_joint_ids
            )
            self._asset.write_joint_velocity_to_sim(
                self._right_targets, joint_ids=self._right_joint_ids
            )
            self._apply_ideal_planar_kinematics(physics_dt)

    def reset(self, env_ids: Sequence[int] | None = None):
        self._raw_actions[env_ids] = 0.0
        self._processed_actions[env_ids] = 0.0
        self._target_actions[env_ids] = 0.0
        self._applied_actions[env_ids] = 0.0
        self._wheel_scale[env_ids] = 1.0
        self._command_age_s[env_ids] = 0.0
        self._watchdog_active[env_ids] = False
        self._watchdog_trigger_age_s[env_ids] = float("nan")
        self._ideal_initialized[env_ids] = False
        self._left_wheel_travel_rad[env_ids] = 0.0
        self._right_wheel_travel_rad[env_ids] = 0.0


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
    forward_track_deadband_mps: float = MISSING
    reverse_track_deadband_mps: float = MISSING
    right_turn_command_scale: float = 1.0
    right_turn_response_gain_vs_left: float = 1.0
    ideal_kinematic: bool = False
