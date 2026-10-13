"""
Submap evidence for the 3D decision after short moves (issue #53).

Pure logic, no ROS.  The robot moves in short segments and stops; no
localization frame is taken while it moves.  At every stop two independent
time windows of deskewed clouds are captured, as for the stationary
decision.  ``submap_windows`` puts all stops of one odometry session in
the base frame of the LAST stop: TRAIN is the union of the first windows
and HOLDOUT the union of the second ones, so the two stay independent, and
every point keeps the sensor position it was seen from for the see-through
check (``WindowOrigins``).  The result feeds ``decide_surface`` unchanged:
map-wide 3D search, fair refinement, see-through exclusion, nearby
competitors, support extent, independent windows, variable regions and
surface additions all apply to the submap as they do to one view.

Odometry drift grows with the path, so the path between the first and the
last stop and the number of stops are bounded (``SubmapDecisionConfig``);
beyond them the submap is refused, never truncated.
"""

from dataclasses import dataclass
import math

import numpy as np

from .localization_contracts import ContractError
from .localization_surface_validation import WindowOrigins


@dataclass(frozen=True)
class SubmapDecisionConfig:
    """Provisional until the linear motion profile is accepted on the robot."""

    max_path_m: float = 3.0
    max_stops: int = 4
    max_points_per_cloud: int = 6000
    min_range_m: float = .5

    def __post_init__(self):
        if (not math.isfinite(self.max_path_m) or self.max_path_m <= 0
                or not math.isfinite(self.min_range_m) or self.min_range_m < 0
                or type(self.max_stops) is not int or self.max_stops < 1
                or type(self.max_points_per_cloud) is not int
                or self.max_points_per_cloud < 1):
            raise ContractError('invalid submap decision limits')


@dataclass(frozen=True)
class Stop:
    """Two independent capture windows (``SubmapFrame`` tuples) at one stop."""

    first: tuple
    second: tuple


def _window(frames, reference_T_odom, config):
    points, origins = [], []
    for frame in frames:
        cloud = np.asarray(frame.points, float).reshape(-1, 3)
        cloud = cloud[np.isfinite(cloud).all(1)]
        if len(cloud) > config.max_points_per_cloud:
            cloud = cloud[np.linspace(0, len(cloud) - 1,
                                      config.max_points_per_cloud).astype(int)]
        base = (np.asarray(frame.base_T_sensor, float)
                @ np.c_[cloud, np.ones(len(cloud))].T).T[:, :3]
        cloud = cloud[np.hypot(base[:, 0], base[:, 1]) >= config.min_range_m]
        if not len(cloud):
            continue
        matrix = reference_T_odom @ np.asarray(frame.odom_T_base, float) @ frame.base_T_sensor
        points.append((matrix @ np.c_[cloud, np.ones(len(cloud))].T).T[:, :3])
        origins.append(np.broadcast_to(matrix[:3, 3], (len(cloud), 3)))
    if not points:
        raise ContractError('a capture window has no points')
    return np.vstack(points), np.vstack(origins)


def path_length(stops):
    """Odometry path through the stops (first frame of each), metres."""
    positions = [np.asarray(stop.first[0].odom_T_base, float)[:2, 3] for stop in stops]
    return float(sum(np.linalg.norm(b - a) for a, b in zip(positions, positions[1:])))


def submap_windows(stops, config=SubmapDecisionConfig()):
    """
    Return ``(train, holdout, WindowOrigins, path_m)`` in the last stop's base frame.

    The reference is the odometry pose of the last stop's first frame, so a
    decided pose is the robot's map pose there.
    """
    stops = tuple(stops)
    if not stops or any(not stop.first or not stop.second for stop in stops):
        raise ContractError('every stop needs two capture windows')
    if len(stops) > config.max_stops:
        raise ContractError(f'{len(stops)} stops exceed the limit of {config.max_stops}')
    path = path_length(stops)
    if path > config.max_path_m:
        raise ContractError(f'odometry path {path:.2f} m exceeds {config.max_path_m} m')
    reference_T_odom = np.linalg.inv(np.asarray(stops[-1].first[0].odom_T_base, float))
    train = [_window(stop.first, reference_T_odom, config) for stop in stops]
    holdout = [_window(stop.second, reference_T_odom, config) for stop in stops]
    return (np.vstack([p for p, _ in train]), np.vstack([p for p, _ in holdout]),
            WindowOrigins(np.vstack([o for _, o in train]), np.vstack([o for _, o in holdout])),
            path)
