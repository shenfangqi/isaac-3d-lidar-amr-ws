"""Phase 4: offline closed-loop simulation with the production pipeline."""

import math

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_closed_loop import (
    ClosedLoopSimulator, GAVE_UP, READY, SimulationConfig,
)
from isaac_3d_lidar_bringup.localization_contracts import SE2
from isaac_3d_lidar_bringup.localization_hypotheses import (
    SearchConfig, ValidationThresholds,
)
from isaac_3d_lidar_bringup.localization_route_planner import (
    MotionModel, ObservationModel, PlannerConfig, StaticMap,
)

pytest.importorskip('scipy.ndimage')
RES = 0.05
SEARCH = SearchConfig(coarse_step_m=0.2, coarse_yaw_step_rad=math.radians(15),
                      coarse_beams=36, refine_beams=90, refine_evaluations=60,
                      tolerance_cells=3)
# Mechanism thresholds: with the production hit-share matcher a 1 m stub in
# a long corridor moves the margin by a few percent only (the site analysis
# found the same for 2D), so the closed loop is exercised at a 0.03 margin;
# the planner's target equals the validation margin.  Production thresholds
# are checked separately for never producing a wrong READY.
MECHANISM = ValidationThresholds(min_margin=0.03)
PLANNER = PlannerConfig(rotations=(math.pi / 2, -math.pi / 2), forwards=(0.4, 0.6),
                        max_depth=2, min_difference=0.03, time_budget_s=120)
MODEL = MotionModel(forward_stop_extension_m=0.08)


def _corridor(post=True, extra=()):
    """8 x 1.2 m closed corridor, symmetric under a half turn about (4, 0.6)."""
    w, h = round(8.0 / RES), round(1.2 / RES)
    data = np.zeros((h, w), np.int16)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    # A 1 m stub on the lower wall only; its half-turn image is free.  It is
    # beyond the 2 m sensor range of both twins at the start.
    boxes = [(3.5, 0.0, 4.5, 0.35)] if post else []
    for x0, y0, x1, y1 in list(boxes) + list(extra):
        data[round(y0 / RES):round(y1 / RES), round(x0 / RES):round(x1 / RES)] = 100
    return StaticMap(data, RES, 0.0, 0.0)


def _simulator(static_map, world=None, steps=4, thresholds=MECHANISM):
    layers = None
    if world is not None:
        occupied = world.data == 100
        layers = {'world': (occupied, occupied)}
    simulator = ClosedLoopSimulator(
        static_map, SEARCH, thresholds, MODEL, PLANNER,
        SimulationConfig(max_steps=steps, max_travel_m=1.2),
        world_layers=layers)
    # The sensor reaches 2.5 m: from either end the stub is at the edge of
    # the view and too few beams hit it, so the ends are twins until the
    # robot drives towards it.
    simulator.world = ObservationModel(static_map, simulator.world.layers,
                                       beams=360, max_range_m=2.5)
    occupied = static_map.data == 100
    simulator.prediction = ObservationModel(static_map, {'map': (occupied, occupied)},
                                            beams=180, max_range_m=2.5)
    return simulator


@pytest.mark.parametrize('start', [SE2(1.5, 0.6, 0.0), SE2(6.5, 0.6, math.pi)])
def test_driving_resolves_twins_that_are_ambiguous_in_place(start):
    simulator = _simulator(_corridor())
    result = simulator.run(start)
    assert result.outcome == READY, result.history
    first = result.history[0]
    assert first['verdict'] == 'AMBIGUOUS_LOCATION' and first['plausible'] >= 2
    assert any(step.get('primitive', ('', 0))[0] == 'FORWARD' for step in result.history)
    assert result.travel_m > 0


def test_a_fully_symmetric_corridor_never_ends_in_a_wrong_ready():
    simulator = _simulator(_corridor(post=False), steps=3)
    result = simulator.run(SE2(1.5, 0.6, 0.0))
    assert result.outcome == GAVE_UP
    assert result.reason in ('STEP_BUDGET', 'INDISTINGUISHABLE_IN_MODEL',
                             'NO_COMMON_SAFE_ACTION', 'AMBIGUOUS_LOCATION')
    assert all(step['verdict'] != 'ACCEPTED' for step in result.history)


def test_a_world_that_contradicts_the_map_refutes_and_searches_again():
    static_map = _corridor()
    # A crate the map does not know sits right where the robot drives to.
    world = _corridor(extra=[(2.45, 0.0, 2.75, 0.45), (2.45, 0.75, 2.75, 1.2)])
    simulator = _simulator(static_map, world=world)
    result = simulator.run(SE2(1.5, 0.6, 0.0))
    assert result.outcome != 'WRONG_READY'
    assert len(result.history) >= 1


def test_production_thresholds_never_produce_a_wrong_ready():
    for start in (SE2(1.5, 0.6, 0.0), SE2(6.5, 0.6, math.pi)):
        simulator = _simulator(_corridor(), thresholds=ValidationThresholds(), steps=3)
        result = simulator.run(start)
        assert result.outcome in (READY, GAVE_UP), result.history


def test_history_records_progress_metrics_for_every_step():
    simulator = _simulator(_corridor())
    result = simulator.run(SE2(1.5, 0.6, 0.0))
    for step in result.history[:-1]:
        assert {'candidates', 'verdict', 'measured_margin', 'plan_status',
                'predicted_margin', 'primitive'} <= set(step)
