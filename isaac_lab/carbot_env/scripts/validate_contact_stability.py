#!/usr/bin/env python3
"""Check Carbot ground contact, stability, and ideal-track consistency."""

import argparse
import math
from pathlib import Path
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--start-trigger-file")
parser.add_argument("--hold-seconds", type=float, default=3600.0)
parser.add_argument("--render-stride", type=int, default=10)
parser.add_argument("--realtime", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402

from isaac_lab.carbot_env.carbot_env_cfg import (  # noqa: E402
    ROBOT,
    CarbotEnvCfgPlay,
)


def _rpy_from_wxyz(quaternion):
    w, x, y, z = quaternion.tolist()
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_argument)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class StabilityMonitor:
    def __init__(self, env, robot, action_term):
        self.env = env
        self.robot = robot
        self.action_term = action_term
        self.body_index = {name: index for index, name in enumerate(robot.body_names)}
        wheel_groups = ROBOT["geometry"]["wheel_groups"]
        self.wheels = []
        for side in ("left", "right"):
            for group, geometry in wheel_groups.items():
                name = f"{side}_{group}_wheel_link"
                self.wheels.append(
                    (name, self.body_index[name], geometry["radius_m"], "road" in group)
                )
        self.root_z_min = float("inf")
        self.root_z_max = -float("inf")
        self.chassis_bottom_min = float("inf")
        self.road_bottom_min = float("inf")
        self.road_bottom_max = -float("inf")
        self.end_bottom_min = float("inf")
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.max_vertical_speed = 0.0
        self.max_side_velocity_spread = 0.0
        self.reset_count = 0
        self.start_position = None
        self.start_yaw = None
        self.shadow_xy = None
        self.shadow_yaw = None

    def begin_motion_reference(self):
        position = self.robot.data.root_pos_w[0, :2]
        _, _, yaw = _rpy_from_wxyz(self.robot.data.root_quat_w[0])
        self.start_position = position.clone()
        self.start_yaw = yaw
        self.shadow_xy = position.clone()
        self.shadow_yaw = yaw

    def sample(self):
        root_position = self.robot.data.root_pos_w[0]
        roll, pitch, _ = _rpy_from_wxyz(self.robot.data.root_quat_w[0])
        root_z = root_position[2].item()
        self.root_z_min = min(self.root_z_min, root_z)
        self.root_z_max = max(self.root_z_max, root_z)
        self.max_roll = max(self.max_roll, abs(roll))
        self.max_pitch = max(self.max_pitch, abs(pitch))
        self.max_vertical_speed = max(
            self.max_vertical_speed, abs(self.robot.data.root_lin_vel_w[0, 2].item())
        )

        body_collision = ROBOT["geometry"]["body_collision"]
        chassis_bottom = (
            root_z
            + ROBOT["geometry"]["base_link_height_m"]
            + body_collision["position_from_base_link_m"][2]
            - body_collision["size_m"][2] / 2.0
        )
        self.chassis_bottom_min = min(self.chassis_bottom_min, chassis_bottom)
        for _, body_id, radius, is_road_wheel in self.wheels:
            bottom = self.robot.data.body_pos_w[0, body_id, 2].item() - radius
            if is_road_wheel:
                self.road_bottom_min = min(self.road_bottom_min, bottom)
                self.road_bottom_max = max(self.road_bottom_max, bottom)
            else:
                self.end_bottom_min = min(self.end_bottom_min, bottom)

        left = self.robot.data.joint_vel[0, self.action_term._left_joint_ids]
        right = self.robot.data.joint_vel[0, self.action_term._right_joint_ids]
        side_spread = max(
            (left.max() - left.min()).item(),
            (right.max() - right.min()).item(),
        )
        self.max_side_velocity_spread = max(self.max_side_velocity_spread, side_spread)

    def step(self, actions):
        started = time.perf_counter()
        _, _, terminated, time_out, _ = self.env.step(actions)
        self.reset_count += int(torch.logical_or(terminated, time_out).sum().item())
        if self.shadow_xy is not None:
            linear = self.action_term.applied_actions[0, 0].item()
            angular = self.action_term.applied_actions[0, 1].item()
            midpoint_yaw = self.shadow_yaw + angular * self.env.step_dt / 2.0
            self.shadow_xy[0] += linear * math.cos(midpoint_yaw) * self.env.step_dt
            self.shadow_xy[1] += linear * math.sin(midpoint_yaw) * self.env.step_dt
            self.shadow_yaw += angular * self.env.step_dt
        self.sample()
        if args.realtime:
            remaining = self.env.step_dt - (time.perf_counter() - started)
            if remaining > 0.0:
                time.sleep(remaining)

    def run(self, actions, seconds, label):
        print(f"[G7] {label} for {seconds:.1f} s", flush=True)
        for _ in range(max(1, round(seconds / self.env.step_dt))):
            self.step(actions)
        self.env.sim.render()


def main():
    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
    cfg.viewer.eye = (2.4, -3.6, 2.4)
    cfg.viewer.lookat = (0.0, 0.0, 0.08)
    cfg.viewer.origin_type = "world"
    cfg.viewer.asset_name = None
    cfg.events.reset_base.params["pose_range"] = {
        "x": (0.0, 0.0),
        "y": (0.0, 0.0),
        "yaw": (0.0, 0.0),
    }
    cfg.observations.policy.lidar_ranges = None
    cfg.observations.policy.height_scan = None
    cfg.scene.lidar = None
    cfg.scene.height_scanner = None
    cfg.terminations.goal_reached = None
    cfg.sim.render_interval = args.render_stride

    env = ManagerBasedRLEnv(cfg=cfg)
    env.reset()
    robot = env.scene["robot"]
    action_term = env.action_manager.get_term("twist")
    monitor = StabilityMonitor(env, robot, action_term)
    zeros = torch.zeros((env.num_envs, 2), device=env.device)

    def command(linear=0.0, angular=0.0):
        value = zeros.clone()
        value[:, 0] = linear
        value[:, 1] = angular
        return value

    print(
        "[G7] Sequence: settle -> forward -> stop -> reverse -> stop -> "
        "spin -> stop -> arc -> final stop.",
        flush=True,
    )
    print(
        "[G7] Collision support uses the eight road-wheel collision cylinders; "
        "front/rear idler-drive wheels are intentionally raised.",
        flush=True,
    )
    env.sim.render()
    if args.start_trigger_file:
        trigger = Path(args.start_trigger_file)
        print(f"[G7] Waiting for operator trigger: {trigger}", flush=True)
        while simulation_app.is_running() and not trigger.exists():
            env.sim.render()
            time.sleep(0.05)
        if not simulation_app.is_running():
            env.close()
            return
        trigger.unlink(missing_ok=True)
    print(f"[G7] Operator ready; starting in {args.startup_delay:.0f} s.", flush=True)
    deadline = time.monotonic() + args.startup_delay
    while simulation_app.is_running() and time.monotonic() < deadline:
        env.sim.render()
        time.sleep(0.05)

    monitor.run(zeros, 3.0, "SETTLE")
    monitor.begin_motion_reference()
    left_start = action_term.left_wheel_travel_rad[0].clone()
    right_start = action_term.right_wheel_travel_rad[0].clone()
    monitor.run(command(linear=0.10), 3.0, "FORWARD 0.10 m/s")
    monitor.run(zeros, 2.0, "STOP-1")
    monitor.run(command(linear=-0.10), 3.0, "REVERSE 0.10 m/s")
    monitor.run(zeros, 2.0, "STOP-2")
    monitor.run(command(angular=0.50), 5.0, "IN-PLACE TURN 0.50 rad/s")
    monitor.run(zeros, 2.0, "STOP-3")
    monitor.run(command(linear=0.10, angular=-0.50), 6.0, "RIGHT ARC")
    monitor.run(zeros, 3.0, "FINAL STOP")

    final_position = robot.data.root_pos_w[0, :2]
    _, _, final_yaw = _rpy_from_wxyz(robot.data.root_quat_w[0])
    pose_error = torch.linalg.vector_norm(final_position - monitor.shadow_xy).item()
    yaw_error = abs(
        math.atan2(
            math.sin(final_yaw - monitor.shadow_yaw),
            math.cos(final_yaw - monitor.shadow_yaw),
        )
    )
    left_delta = action_term.left_wheel_travel_rad[0] - left_start
    right_delta = action_term.right_wheel_travel_rad[0] - right_start
    wheel_spread = max(
        (left_delta.max() - left_delta.min()).item(),
        (right_delta.max() - right_delta.min()).item(),
    )
    final_linear_speed = torch.linalg.vector_norm(
        robot.data.root_lin_vel_b[0, :2]
    ).item()
    final_angular_speed = abs(robot.data.root_ang_vel_b[0, 2].item())
    root_z_span = monitor.root_z_max - monitor.root_z_min

    contact_ok = (
        monitor.road_bottom_min >= -0.006
        and monitor.road_bottom_max <= 0.008
        and monitor.chassis_bottom_min >= 0.003
        and monitor.end_bottom_min >= 0.015
    )
    stable_ok = (
        monitor.max_roll <= math.radians(2.0)
        and monitor.max_pitch <= math.radians(2.0)
        and root_z_span <= 0.008
        and monitor.max_vertical_speed <= 0.08
    )
    track_ok = (
        pose_error <= 0.02
        and yaw_error <= math.radians(2.0)
        and wheel_spread <= 0.01
    )
    stop_ok = final_linear_speed <= 0.05 and final_angular_speed <= 0.05
    reset_ok = monitor.reset_count == 0

    print(
        "[G7][CONTACT] "
        f"road-wheel bottoms=[{monitor.road_bottom_min:.4f}, "
        f"{monitor.road_bottom_max:.4f}] m, "
        f"raised end-wheel minimum={monitor.end_bottom_min:.4f} m, "
        f"chassis bottom minimum={monitor.chassis_bottom_min:.4f} m",
        flush=True,
    )
    print(
        "[G7][STABILITY] "
        f"root_z=[{monitor.root_z_min:.4f}, {monitor.root_z_max:.4f}] m, "
        f"span={root_z_span:.4f} m, "
        f"max_roll/pitch=({math.degrees(monitor.max_roll):.2f}, "
        f"{math.degrees(monitor.max_pitch):.2f}) deg, "
        f"max_vertical_speed={monitor.max_vertical_speed:.4f} m/s",
        flush=True,
    )
    print(
        "[G7][TRACK] "
        f"ideal-path position/yaw error=({pose_error:.4f} m, "
        f"{math.degrees(yaw_error):.2f} deg), "
        f"wheel-travel spread={wheel_spread:.5f} rad, "
        f"contact-resolved instantaneous same-side spread="
        f"{monitor.max_side_velocity_spread:.5f} rad/s (informational), "
        f"final_speeds=({final_linear_speed:.4f} m/s, "
        f"{final_angular_speed:.4f} rad/s), resets={monitor.reset_count}",
        flush=True,
    )
    if not (contact_ok and stable_ok and track_ok and stop_ok and reset_ok):
        raise RuntimeError(
            "G7 failed: "
            f"contact_ok={contact_ok}, stable_ok={stable_ok}, "
            f"track_ok={track_ok}, stop_ok={stop_ok}, reset_ok={reset_ok}"
        )

    print(
        "[G7] PASS: no collision penetration, unsupported road wheels, "
        "rollover, or ideal-track inconsistency detected. Physical skid/slip "
        "calibration remains a hardware/high-fidelity backlog item.",
        flush=True,
    )
    if args.hold_seconds > 0.0:
        print(
            "[G7] Holding the stopped robot for visual confirmation "
            f"({args.hold_seconds:.0f} s maximum).",
            flush=True,
        )
        deadline = time.monotonic() + args.hold_seconds
        while simulation_app.is_running() and time.monotonic() < deadline:
            env.sim.render()
            time.sleep(0.05)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
