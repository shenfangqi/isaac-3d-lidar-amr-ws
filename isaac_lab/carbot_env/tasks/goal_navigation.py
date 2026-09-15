"""Manager terms for the initial Carbot goal-navigation task."""

import math
from pathlib import Path

import yaml

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from .. import mdp


TASK_PATH = Path(__file__).with_suffix(".yaml")
with TASK_PATH.open(encoding="utf-8") as stream:
    TASK = yaml.safe_load(stream)


@configclass
class CommandsCfg:
    """Sample relative planar goals, independently of the robot asset."""

    pose_command = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=False,
        resampling_time_range=(
            TASK["goal_sampling"]["resampling_time_s"],
            TASK["goal_sampling"]["resampling_time_s"],
        ),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(
            pos_x=tuple(TASK["goal_sampling"]["x_m"]),
            pos_y=tuple(TASK["goal_sampling"]["y_m"]),
            heading=tuple(TASK["goal_sampling"]["heading_rad"]),
        ),
    )


@configclass
class RewardsCfg:
    """Initial conservative task shaping; intentionally not final training."""

    position_tracking = RewTerm(
        func=mdp.position_tracking,
        weight=TASK["rewards"]["position_tracking"],
        params={"command_name": "pose_command", "standard_deviation_m": 1.0},
    )
    heading_error = RewTerm(
        func=mdp.heading_error,
        weight=TASK["rewards"]["heading_error"],
        params={"command_name": "pose_command"},
    )
    planar_speed = RewTerm(
        func=mdp.planar_speed,
        weight=TASK["rewards"]["planar_speed"],
    )
    action_rate = RewTerm(
        func=mdp.action_rate_l2,
        weight=TASK["rewards"]["action_rate"],
    )
    goal_reached = RewTerm(
        func=mdp.goal_reached_reward,
        weight=TASK["rewards"]["goal_reached"],
        params={
            "command_name": "pose_command",
            "position_tolerance_m": TASK["success"]["position_tolerance_m"],
            "heading_tolerance_rad": TASK["success"]["heading_tolerance_rad"],
        },
    )


@configclass
class TerminationsCfg:
    """Reset on timeout, goal completion, or rollover."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    goal_reached = DoneTerm(
        func=mdp.goal_reached_termination,
        params={
            "command_name": "pose_command",
            "position_tolerance_m": TASK["success"]["position_tolerance_m"],
            "heading_tolerance_rad": TASK["success"]["heading_tolerance_rad"],
        },
    )
    excessive_tilt = DoneTerm(
        func=mdp.excessive_tilt,
        params={"minimum_up_z": math.cos(math.radians(45.0))},
    )
