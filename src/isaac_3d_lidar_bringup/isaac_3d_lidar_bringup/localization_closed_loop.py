"""
Issue #13 phase 4: offline closed-loop simulation of probe-driven localization.

Not used at runtime.  The production search, candidate re-check, validation
(with per-view gates after a translation) and the phase-1 route planner run
unchanged; only the sensor and the robot are simulated: scans are ray cast
in a *world* grid from the true pose, and primitives move the true pose
exactly (optionally with noise).  Decisions never read the true pose; it is
only compared with the final answer.

Each step records the candidate count, the hardest pair's predicted and
measured margins, the chosen primitive, unsafe reasons and the verdict, so
failures are reproducible instead of looping.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from .localization_contracts import FrameRole, Keyframe, SE2
from .localization_hypotheses import (
    recheck_hypotheses, search_multiview_cached, seed_pose_at_current_time,
    validate_hypotheses,
)
from .localization_observations import ScanSnapshot
from .localization_route_planner import (
    check_route, ClearanceField, FORWARD, NO_COMMON_SAFE_ACTION, ObservationModel,
    plan_route, PlannerConfig, PoseBounds, ROTATE, ROUTE_FOUND,
)

READY = 'READY'
WRONG_READY = 'WRONG_READY'
GAVE_UP = 'GAVE_UP'


@dataclass(frozen=True)
class SimulationConfig:
    """Budgets of one simulated session."""

    max_steps: int = 6
    max_travel_m: float = 1.5
    candidate_window: float = 0.25
    max_candidates: int = 6
    frames_per_role: int = 3
    range_noise_m: float = 0.005
    truth_xy_m: float = 0.15
    truth_yaw_rad: float = math.radians(5.0)
    seed: int = 0


@dataclass
class SimulationResult:
    """Outcome and step history of one simulated session."""

    outcome: str
    reason: str
    final_pose: object
    history: list = field(default_factory=list)
    travel_m: float = 0.0


def grid_from_static_map(static_map):
    """OccupancyGrid-like snapshot of a StaticMap for the production search."""
    from types import SimpleNamespace as NS
    from array import array
    return NS(frame_id='map', data=array('b', static_map.data.astype(np.int8).ravel()),
              info=NS(width=static_map.width, height=static_map.height,
                      resolution=static_map.resolution,
                      origin=NS(position=NS(x=static_map.origin_x, y=static_map.origin_y),
                                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))


class ClosedLoopSimulator:
    """
    Simulated session over a known map and a (possibly different) world.

    ``world_layers`` are the observation layers the *sensor* sees; the
    planner predicts with ``planner_layers`` (by default the same).  The
    production matcher always scores against the navigation map.
    """

    def __init__(self, static_map, search_config, thresholds, motion_model,
                 planner_config=PlannerConfig(), sim_config=SimulationConfig(),
                 world_layers=None, planner_layers=None):
        self.map = static_map
        self.grid = grid_from_static_map(static_map)
        self.search_config = search_config
        self.thresholds = thresholds
        self.model = motion_model
        self.planner_config = planner_config
        self.config = sim_config
        occupied = static_map.data == 100
        self.world = ObservationModel(static_map, world_layers or {'nav': (occupied, occupied)},
                                      beams=360, max_range_m=8.0)
        self.planner_layers = planner_layers or self.world.layers
        self.prediction = ObservationModel(static_map, self.planner_layers)
        self.fields = (ClearanceField(static_map),
                       ClearanceField(static_map, blocked=occupied, outside_blocked=False))
        self._rng = np.random.default_rng(sim_config.seed)
        self._frame_id = 0
        self._stamp = 1_000_000_000

    # -- sensor -------------------------------------------------------------

    def _scan(self, true_pose):
        layer = next(iter(self.world.layers))
        ranges = self.world.ranges(layer, (true_pose.x, true_pose.y, true_pose.yaw))
        noisy = np.where(np.isfinite(ranges),
                         ranges + self._rng.normal(0, self.config.range_noise_m, ranges.shape),
                         np.inf)
        self._stamp += 100_000_000
        increment = 2.0 * math.pi / self.world.beams
        # Sensor offset folded into the frame: the scan is in base_footprint.
        return ScanSnapshot('base_footprint', self._stamp, -math.pi, increment,
                            self.world.min_range_m, self.world.max_range_m,
                            tuple(float(r) for r in noisy))

    def _frames(self, true_pose, odom_pose, view, role, session):
        frames = []
        for _ in range(self.config.frames_per_role):
            scan = self._scan(true_pose)
            frames.append(Keyframe(self._frame_id, session, scan.stamp_ns, view, scan,
                                   odom_pose, SE2(*self.world.sensor_offset, 0.0),
                                   scan.stamp_ns / 1e9, FrameRole(role)))
            self._frame_id += 1
        return tuple(frames)

    # -- session --------------------------------------------------------------

    def run(self, true_start, session='simsession00001'):
        """Simulate one session from ``true_start`` (map pose, never read)."""
        true_pose, odom = true_start, SE2(0.0, 0.0, 0.0)
        train = self._frames(true_pose, odom, 0, 'TRAIN', session)
        holdout = self._frames(true_pose, odom, 0, 'HOLDOUT', session)
        output = search_multiview_cached(self.grid, train, self.search_config)
        result, cache = output.result, output.coarse_cache
        history, travel, translated, gates = [], 0.0, False, ()
        reference = train[0].T_odom_base
        for step in range(self.config.max_steps + 1):
            decision = (validate_hypotheses(self.grid, result, train, holdout,
                                            self.search_config, self.thresholds,
                                            gate_frames=gates)
                        if result.complete else None)
            entry = {'step': step, 'search_complete': result.complete,
                     'candidates': len(result.hypotheses),
                     'verdict': (decision.reason or 'ACCEPTED') if decision else result.reason,
                     'measured_margin': self._margin(decision)}
            history.append(entry)
            if decision is not None and decision.accepted:
                pose = seed_pose_at_current_time(
                    SE2(decision.winner.x, decision.winner.y, decision.winner.yaw),
                    reference, odom)
                correct = (math.hypot(pose.x - true_pose.x, pose.y - true_pose.y)
                           <= self.config.truth_xy_m
                           and abs(math.atan2(math.sin(pose.yaw - true_pose.yaw),
                                              math.cos(pose.yaw - true_pose.yaw)))
                           <= self.config.truth_yaw_rad)
                return SimulationResult(READY if correct else WRONG_READY, '', pose,
                                        history, travel)
            if step == self.config.max_steps:
                return SimulationResult(GAVE_UP, 'STEP_BUDGET', None, history, travel)
            candidates, focus = self._plausible(result, decision, reference, odom)
            entry['plausible'] = len(candidates)
            entry['contending'] = len(focus)
            if len(candidates) < 2:
                return SimulationResult(GAVE_UP, entry['verdict'], None, history, travel)
            primitive, plan = self._choose(candidates, travel,
                                           focus if len(focus) >= 2 else None)
            entry['plan_status'] = plan.status if plan else NO_COMMON_SAFE_ACTION
            entry['predicted_margin'] = round(plan.objective, 3) if plan else None
            entry['unsafe_reasons'] = dict(plan.unsafe_reasons) if plan else {}
            if primitive is None:
                return SimulationResult(GAVE_UP, entry['plan_status'], None, history, travel)
            entry['primitive'] = primitive
            kind, value = primitive
            move = SE2(0.0, 0.0, value) if kind == ROTATE else SE2(value, 0.0, 0.0)
            true_pose, odom = true_pose.compose(move), odom.compose(move)
            if kind == FORWARD:
                travel += value
                translated = True
            view = step + 1
            new_train = self._frames(true_pose, odom, view, 'TRAIN', session)
            new_holdout = self._frames(true_pose, odom, view, 'HOLDOUT', session)
            train = train + new_train
            # Margin on the newest view; earlier HOLDOUT only gate per view.
            gates = (gates + holdout) if translated else ()
            holdout = new_holdout
            if result.complete and result.hypotheses:
                result = recheck_hypotheses(self.grid, result, reference, train,
                                            self.search_config)
                if result.complete and not any(
                        h.score >= self.thresholds.min_score for h in result.hypotheses):
                    entry['refuted'] = True        # every candidate refuted
                    output = search_multiview_cached(self.grid, train, self.search_config,
                                                     coarse_cache=cache)
                    result, cache = output.result, output.coarse_cache
            else:
                output = search_multiview_cached(self.grid, train, self.search_config,
                                                 coarse_cache=cache)
                result, cache = output.result, output.coarse_cache
        return SimulationResult(GAVE_UP, 'STEP_BUDGET', None, history, travel)

    @staticmethod
    def _margin(decision):
        if decision is None or not decision.holdout or len(decision.holdout) < 2:
            return None
        return round(decision.holdout[0][1] - decision.holdout[1][1], 3)

    def _plausible(self, result, decision, reference, odom):
        """
        Return the plausible candidates and the contending indices.

        Candidates within the window are given at the current odometry
        pose; contenders are those close enough to the leader to keep the
        verdict ambiguous.
        """
        if not result.hypotheses:
            return (), ()
        by_id = {h.cluster_id: h for h in result.hypotheses}
        ranked = ([(by_id[c], s) for c, s in decision.holdout if c in by_id]
                  if decision is not None and decision.holdout
                  else [(h, h.score) for h in sorted(result.hypotheses, key=lambda h: -h.score)])
        best = ranked[0][1]
        kept = [(h, s) for h, s in ranked
                if s >= best - self.config.candidate_window][:self.config.max_candidates]
        contend = self.thresholds.min_margin + 0.05
        focus = tuple(i for i, (_h, s) in enumerate(kept) if s >= best - contend)
        return (tuple(seed_pose_at_current_time(SE2(h.x, h.y, h.yaw), reference, odom)
                      for h, _s in kept), focus)

    def _choose(self, candidates, travel, focus=None):
        """First primitive of a planned route, else a commonly safe rotation."""
        bounds = PoseBounds()
        remaining = self.config.max_travel_m - travel
        config = PlannerConfig(
            rotations=self.planner_config.rotations,
            forwards=tuple(d for d in self.planner_config.forwards if d <= remaining + 1e-9),
            max_depth=self.planner_config.max_depth,
            max_total_forward_m=remaining,
            min_difference=self.planner_config.min_difference,
            time_budget_s=self.planner_config.time_budget_s,
            layers=self.planner_config.layers)
        plan = plan_route(self.fields, self.model, self.prediction,
                          [((p.x, p.y, p.yaw), bounds) for p in candidates], config, focus)
        if plan.status == ROUTE_FOUND and plan.route:
            return plan.route[0], plan
        for angle in self.planner_config.rotations:     # rotation fallback
            if all(check_route(self.fields, self.model, (p.x, p.y, p.yaw), bounds,
                               ((ROTATE, angle),)).safe for p in candidates):
                return (ROTATE, angle), plan
        return None, plan
