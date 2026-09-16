#!/usr/bin/env python3
"""Visually and quantitatively validate zero hold and the action watchdog."""

import argparse
import math
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--zero-seconds", type=float, default=3.0)
parser.add_argument("--command-speed", type=float, default=0.20)
parser.add_argument("--command-seconds", type=float, default=3.0)
parser.add_argument("--watchdog-observe-seconds", type=float, default=3.0)
parser.add_argument("--hold-seconds", type=float, default=3600.0)
parser.add_argument("--render-stride", type=int, default=10)
parser.add_argument("--trace-steps", action="store_true")
parser.add_argument("--realtime", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402

from isaac_lab.carbot_env.carbot_env_cfg import CarbotEnvCfgPlay  # noqa: E402


def _yaw_from_wxyz(quaternion):
    w, x, y, z = quaternion.tolist()
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose(robot):
    position = robot.data.root_pos_w[0].clone()
    yaw = _yaw_from_wxyz(robot.data.root_quat_w[0])
    return position, yaw


def _run_actions(env, actions, seconds, label):
    if seconds <= 0.0:
        return 0
    steps = max(1, round(seconds / env.step_dt))
    started = time.perf_counter()
    termination_count = 0
    for index in range(steps):
        step_started = time.perf_counter()
        _, _, terminated, time_out, _ = env.step(actions)
        termination_count += int(
            torch.logical_or(terminated, time_out).sum().item()
        )
        if args.realtime:
            remaining = env.step_dt - (time.perf_counter() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)
        if args.trace_steps:
            print(
                f"[G1][TRACE] {label} step {index + 1}/{steps}: "
                f"{time.perf_counter() - step_started:.3f} s wall",
                flush=True,
            )
    env.sim.render()
    if str(env.device).startswith("cuda"):
        torch.cuda.synchronize()
    print(
        f"[G1][TIMING] {label}: {steps} control steps in "
        f"{time.perf_counter() - started:.3f} s wall; "
        f"resets={termination_count}",
        flush=True,
    )
    return termination_count


def main():
    if args.render_stride < 1:
        raise ValueError("--render-stride must be at least 1")
    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
    # Keep the camera fixed in the world so translation is visible during the
    # WebRTC gate.  A robot-relative camera would follow the chassis and make
    # a correct forward pulse look almost stationary.
    cfg.viewer.eye = (0.6, -3.0, 1.0)
    cfg.viewer.lookat = (0.6, 0.0, 0.12)
    cfg.viewer.origin_type = "world"
    cfg.viewer.asset_name = None
    # Use a deterministic origin and heading for a repeatable visual check.
    cfg.events.reset_base.params["pose_range"] = {
        "x": (0.0, 0.0),
        "y": (0.0, 0.0),
        "yaw": (0.0, 0.0),
    }
    # This gate validates chassis motion only. The normal environment retains
    # both scanners; this one-off visual gate removes their observation terms
    # so a long control step never waits for an unused sensor update.
    cfg.observations.policy.lidar_ranges = None
    cfg.observations.policy.height_scan = None
    cfg.scene.lidar = None
    cfg.scene.height_scanner = None
    # Goal completion is unrelated to this action-safety gate and can cause a
    # random episode reset before the zero-hold measurement finishes.
    cfg.terminations.goal_reached = None
    cfg.sim.render_interval = args.render_stride
    env = ManagerBasedRLEnv(cfg=cfg)
    env.reset()
    robot = env.scene["robot"]
    action_term = env.action_manager.get_term("twist")
    zeros = torch.zeros((env.num_envs, 2), device=env.device)
    command = zeros.clone()
    command[:, 0] = args.command_speed

    print(
        "[G1] WebRTC inspection starts after "
        f"{args.startup_delay:.0f} s; zero hold -> visible forward pulse -> "
        "command loss/watchdog stop.",
        flush=True,
    )
    env.sim.render()
    time.sleep(args.startup_delay)

    zero_start, zero_start_yaw = _pose(robot)
    print(f"[G1] ZERO HOLD for {args.zero_seconds:.1f} s", flush=True)
    zero_resets = _run_actions(env, zeros, args.zero_seconds, "zero-hold")
    zero_end, zero_end_yaw = _pose(robot)
    zero_drift = torch.linalg.vector_norm(zero_end[:2] - zero_start[:2]).item()
    zero_yaw_drift = abs(zero_end_yaw - zero_start_yaw)

    print(
        f"[G1] FORWARD PULSE {args.command_speed:.2f} m/s for "
        f"{args.command_seconds:.1f} s",
        flush=True,
    )
    motion_start, _ = _pose(robot)
    forward_resets = _run_actions(
        env, command, args.command_seconds, "forward"
    )
    print(
        "[G1] COMMAND SOURCE LOST: normal 20 ms physics continues but no "
        "new action reaches the action term.",
        flush=True,
    )
    process_actions = action_term.process_actions
    action_term.process_actions = lambda _actions: None
    try:
        watchdog_resets = _run_actions(
            env, zeros, args.watchdog_observe_seconds, "watchdog-loss"
        )
    finally:
        action_term.process_actions = process_actions
    motion_end, _ = _pose(robot)
    travel = torch.linalg.vector_norm(motion_end[:2] - motion_start[:2]).item()
    final_speed = abs(robot.data.root_lin_vel_b[0, 0].item())
    wheel_targets_zero = torch.allclose(
        action_term._left_targets, torch.zeros_like(action_term._left_targets),
        atol=1e-6,
    ) and torch.allclose(
        action_term._right_targets,
        torch.zeros_like(action_term._right_targets),
        atol=1e-6,
    )
    timeout = action_term.cfg.watchdog_timeout_s
    trigger_age = action_term.watchdog_trigger_age_s[0].item()
    zero_ok = (
        zero_resets == 0
        and zero_drift <= 0.02
        and zero_yaw_drift <= math.radians(2.0)
    )
    watchdog_ok = (
        forward_resets == 0
        and watchdog_resets == 0
        and action_term.watchdog_active[0].item()
        and timeout < trigger_age <= timeout + 2.0 * env.physics_dt
        and torch.allclose(action_term.processed_actions, zeros, atol=1e-6)
        and wheel_targets_zero
        and final_speed <= 0.05
        and travel >= 0.02
    )
    print(
        "[G1][METRICS] "
        f"zero_drift={zero_drift:.4f} m, "
        f"zero_yaw_drift={math.degrees(zero_yaw_drift):.2f} deg, "
        f"travel={travel:.4f} m, final_speed={final_speed:.4f} m/s, "
        f"trigger_age={trigger_age:.4f} s, "
        f"watchdog_active={action_term.watchdog_active[0].item()}, "
        f"processed={action_term.processed_actions[0].tolist()}, "
        f"wheels_zero={wheel_targets_zero}, "
        f"resets=({zero_resets},{forward_resets},{watchdog_resets}), "
        f"zero_ok={zero_ok}, "
        f"watchdog_ok={watchdog_ok}",
        flush=True,
    )
    if not zero_ok or not watchdog_ok:
        raise RuntimeError(
            "[G1] FAIL: "
            f"zero_drift={zero_drift:.4f} m, "
            f"zero_yaw_drift={math.degrees(zero_yaw_drift):.2f} deg, "
            f"trigger_age={trigger_age:.3f} s, travel={travel:.3f} m, "
            f"final_speed={final_speed:.3f} m/s, wheels_zero={wheel_targets_zero}"
        )

    print(
        "[G1] PASS: "
        f"zero drift={zero_drift:.4f} m/{math.degrees(zero_yaw_drift):.2f} deg; "
        f"pulse travel={travel:.3f} m; "
        f"watchdog trigger={trigger_age:.2f} s; "
        f"final speed={final_speed:.3f} m/s.",
        flush=True,
    )
    print(
        f"[G1] Holding the stopped robot for visual confirmation "
        f"({args.hold_seconds:.0f} s maximum).",
        flush=True,
    )
    _run_actions(env, zeros, args.hold_seconds, "confirmation-hold")
    env.close()


try:
    main()
finally:
    simulation_app.close()
