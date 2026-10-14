"""
Issue #53: the translation guard's own static-map check of the remaining path.

Pure logic (numpy/scipy), no ROS.  The manager sends a
``localization_translation_contracts.RoutePlan`` once per MOVE: every
plausible candidate at the stopped start pose.  The guard moves each
candidate by its own odometry since that start and checks the remaining
forward path under all of them, every cycle, as the manager used to do and
send as route evidence.  2026-10-14 that evidence reached the guard too late
under load (SWEEP_STALE twice before the robot moved); checked here, it
always names the guard's newest odometry sample.
"""

import math

from .localization_contracts import SE2, normalize_angle
from .localization_route_planner import (
    check_route, ClearanceField, FORWARD, MotionModel, PoseBounds, static_map_from_grid)
from .localization_translation_policy import TranslationPreview

# The robot is stopped at MOVE: the guard's start pose and the plan's start
# differ only by odometry jitter (about 1 mm at rest, 2026-10-13).
START_TOLERANCE_M = 0.02
START_TOLERANCE_RAD = 0.02


def route_fields(static_map):
    """Clearance fields of the route check (map, then occupied only)."""
    return (ClearanceField(static_map),
            ClearanceField(static_map, blocked=static_map.data == 100,
                           outside_blocked=False))


def guard_model(footprint, padding_m, lateral_error_m, stop_extension_m, plan):
    """Return the forward envelope: guard geometry and profile, the plan's drift."""
    return MotionModel(footprint=tuple(footprint), padding_m=padding_m,
                       forward_stop_extension_m=stop_extension_m,
                       forward_lateral_error_m=lateral_error_m,
                       forward_heading_error_rad=plan.heading_error_rad, calibrated=True)


class RouteChecker:
    """The remaining forward path of one map, under a plan's candidates."""

    def __init__(self, grid, map_hash):
        self.map_hash = map_hash
        self.fields = route_fields(static_map_from_grid(grid, map_hash))

    def preview(self, plan, model, guard_start, odom, remaining_m, stop_extension_m):
        """
        Return the TranslationPreview for the guard at odometry ``odom``.

        ``guard_start`` is the guard's odometry pose at MOVE and ``odom`` its
        newest sample (``stamp_ns``, ``x``, ``y``, ``yaw``); ``remaining_m``
        the distance still to drive, checked with the plan's margin.  A plan
        for another map or another start pose, or any candidate whose path
        is not clear, refuses.
        """
        if remaining_m > 0:
            remaining_m += plan.margin_m
        here = SE2(odom.x, odom.y, odom.yaw)
        reason = ''
        start = plan.start
        if plan.map_hash != self.map_hash:
            reason = 'ROUTE_PLAN_MAP_MISMATCH'
        elif (math.hypot(guard_start.x - start.x, guard_start.y - start.y)
              > START_TOLERANCE_M
              or abs(normalize_angle(guard_start.yaw - start.yaw)) > START_TOLERANCE_RAD):
            reason = 'ROUTE_PLAN_START_MISMATCH'
        elif remaining_m > 1e-3:
            delta = start.inverse().compose(here)
            bounds = PoseBounds(plan.bounds_xy_m, plan.bounds_yaw_rad)
            for x, y, yaw in plan.candidates:
                pose = SE2(x, y, yaw).compose(delta)
                check = check_route(self.fields, model, (pose.x, pose.y, pose.yaw),
                                    bounds, ((FORWARD, remaining_m),))
                if not check.safe:
                    reason = check.reason or 'UNKNOWN_SWEEP'
                    break
        distance = max(remaining_m, 1e-3)
        return TranslationPreview(not reason, reason, distance, stop_extension_m, here,
                                  here.compose(SE2(distance, 0.0, 0.0)), odom.stamp_ns)
