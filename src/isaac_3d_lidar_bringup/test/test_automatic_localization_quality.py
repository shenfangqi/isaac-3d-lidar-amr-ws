"""Unit tests for pure automatic-localization quality metrics."""

from collections import deque
import math
from types import SimpleNamespace

import pytest

from isaac_3d_lidar_bringup.automatic_localization_quality import (
    angular_difference,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    covariance_quality,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    particle_concentration,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    planar_window_span,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    scan_map_score,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    trim_time_window,
)


def _quaternion(yaw=0.0):
    return SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def _pose_with_covariance(x_variance, y_variance, yaw_variance):
    covariance = [0.0] * 36
    covariance[0] = x_variance
    covariance[7] = y_variance
    covariance[35] = yaw_variance
    return SimpleNamespace(pose=SimpleNamespace(covariance=covariance))


def test_angular_difference_wraps_across_pi():
    """Angles on opposite sides of pi must remain close."""
    difference = angular_difference(-math.pi + 0.1, math.pi - 0.1)
    assert difference == pytest.approx(0.2)


def test_planar_window_span_unwraps_heading():
    """A stable heading crossing pi must have a small span."""
    samples = [
        (1.0, 2.0, math.pi - 0.02),
        (1.03, 1.98, -math.pi + 0.03),
    ]
    assert planar_window_span(samples) == pytest.approx((0.03, 0.02, 0.05))


def test_time_window_retains_sample_bracketing_boundary():
    """Discrete pruning must retain enough history to cover the window."""
    samples = deque((stamp, None) for stamp in (6.8, 6.9, 7.0, 9.9))
    trim_time_window(samples, now=10.0, window=3.0)
    assert [entry[0] for entry in samples] == [7.0, 9.9]
    assert 10.0 - samples[0][0] >= 3.0


def test_covariance_quality_rejects_large_or_invalid_uncertainty():
    """Covariance gates must reject high and nonsensical uncertainty."""
    good = _pose_with_covariance(0.01, 0.02, 0.01)
    large = _pose_with_covariance(0.01, 0.09, 0.01)
    invalid = _pose_with_covariance(0.01, 0.02, -1.0)
    assert covariance_quality(good, 0.2, 0.15)
    assert not covariance_quality(large, 0.2, 0.15)
    assert not covariance_quality(invalid, 0.2, 0.15)


def test_particle_concentration_rejects_distant_second_mode():
    """Weight outside the selected pose cluster must lower confidence."""
    def particle(x, weight):
        return SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.0),
                orientation=_quaternion(),
            ),
            weight=weight,
        )

    center = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=_quaternion(),
    )
    particles = [particle(0.1, 0.6), particle(3.0, 0.4)]
    assert particle_concentration(
        particles, center, 0.75, 0.5
    ) == pytest.approx(0.6)


def test_scan_map_score_counts_occupied_endpoint_neighbors():
    """Only scan endpoints landing on occupied cells should match."""
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=_quaternion(),
    )
    data = [0] * 25
    data[2 * 5 + 3] = 100
    grid = SimpleNamespace(
        info=SimpleNamespace(
            width=5,
            height=5,
            resolution=1.0,
            origin=origin,
        ),
        data=data,
    )
    scan = SimpleNamespace(
        ranges=[1.0, 1.0],
        range_min=0.1,
        range_max=5.0,
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
    )
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=2.0, y=2.0),
        rotation=_quaternion(),
    )

    score, valid, sampled = scan_map_score(
        grid,
        scan,
        transform,
        occupied_threshold=65,
        tolerance_cells=0,
        max_beams=180,
    )
    assert valid == 2
    assert sampled == 2
    assert score == pytest.approx(0.5)


def test_scan_map_score_counts_unknown_endpoints_in_denominator():
    """Unknown map cells remain in the score denominator."""
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=_quaternion(),
    )
    grid = SimpleNamespace(
        info=SimpleNamespace(
            width=3,
            height=3,
            resolution=1.0,
            origin=origin,
        ),
        data=[-1] * 9,
    )
    scan = SimpleNamespace(
        ranges=[1.0],
        range_min=0.1,
        range_max=5.0,
        angle_min=0.0,
        angle_increment=1.0,
    )
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=1.0),
        rotation=_quaternion(),
    )
    assert scan_map_score(
        grid, scan, transform, 65, 1, 180
    ) == (0.0, 0, 1)


def test_scan_map_score_rejects_ray_that_crosses_wall():
    """An endpoint match is invalid if the measured free ray crosses a wall."""
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=_quaternion(),
    )
    data = [0] * 21
    data[1 * 7 + 3] = 100
    data[1 * 7 + 5] = 100
    grid = SimpleNamespace(
        info=SimpleNamespace(
            width=7,
            height=3,
            resolution=1.0,
            origin=origin,
        ),
        data=data,
    )
    scan = SimpleNamespace(
        ranges=[4.0],
        range_min=0.1,
        range_max=5.0,
        angle_min=0.0,
        angle_increment=1.0,
    )
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=1.0),
        rotation=_quaternion(),
    )
    assert scan_map_score(
        grid, scan, transform, 65, 0, 180
    ) == (0.0, 1, 1)


def test_diagnostics_expose_legacy_false_positive_on_identical_scan():
    """Synthetic regression only: same scan accepts truth, rejects wrong room."""
    from isaac_3d_lidar_bringup.automatic_localization_quality import scan_map_metrics
    data = [0] * 40
    # Correct ray starts at x=1 and reaches x=3; wrong starts x=3,
    # crosses occupied origin x=3 before a coincidental endpoint x=5.
    data[1 * 10 + 3] = 100
    data[1 * 10 + 5] = 100
    grid = SimpleNamespace(info=SimpleNamespace(
        width=10, height=4, resolution=1., origin=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0.), orientation=_quaternion())), data=data)
    scan = SimpleNamespace(ranges=[2.], range_min=.1, range_max=20.,
                           angle_min=0., angle_increment=1.)
    def at(x):
        return scan_map_metrics(grid, scan, SimpleNamespace(
            translation=SimpleNamespace(x=x, y=1.), rotation=_quaternion()), 65, 0, 120)
    correct, wrong, outside = at(1.), at(3.), at(9.)
    assert correct['score'] == 1.
    assert correct['wall_conflict_ratio'] == 0.
    assert wrong['legacy_score'] == 1.
    assert wrong['score'] == 0.
    assert wrong['wall_conflict_ratio'] == 1.
    assert outside['score'] == 0.
    assert outside['outside'] == 1
    assert outside['sampled'] == 1


def test_scan_metrics_limit_applies_after_invalid_ranges_are_removed():
    """A 40-beam budget evaluates 40 real returns, not 20 interleaved ones."""
    from isaac_3d_lidar_bringup.automatic_localization_quality import scan_map_metrics
    data = [0] * (50 * 50)
    grid = SimpleNamespace(info=SimpleNamespace(
        width=50, height=50, resolution=0.1, origin=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0.), orientation=_quaternion())),
        data=data)
    scan = SimpleNamespace(
        ranges=[math.inf if index % 2 == 0 else 1.0
                for index in range(200)],
        range_min=.1, range_max=20., angle_min=-.4,
        angle_increment=.004)
    metrics = scan_map_metrics(
        grid, scan, SimpleNamespace(
            translation=SimpleNamespace(x=2., y=2.),
            rotation=_quaternion()),
        occupied_threshold=65, tolerance_cells=0, max_beams=40)

    assert metrics['sampled'] == 40


def test_global_search_adds_bounded_second_stage_refinement(monkeypatch):
    """The final local grid resolves offsets smaller than the first grid."""
    import isaac_3d_lidar_bringup.automatic_localization_quality as quality

    target = (0.72, 0.50, 0.12)

    def metric(_grid, _scan, pose, *_args):
        x = pose.translation.x
        y = pose.translation.y
        yaw = 2.0 * math.atan2(pose.rotation.z, pose.rotation.w)
        error = abs(x - target[0]) + abs(y - target[1]) + abs(yaw - target[2])
        return {
            'score': 1.0 - error,
            'coverage': 1.0,
            'wall_conflict_ratio': 0.0,
            'known': 100,
        }

    monkeypatch.setattr(quality, 'scan_map_metrics', metric)
    grid = SimpleNamespace(info=SimpleNamespace(
        width=2, height=1, resolution=1.0,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))),
        data=[0, 0])
    candidates = quality.deterministic_global_search(
        grid, [object()], occupied_threshold=65, tolerance_cells=0,
        position_step_m=1.0, yaw_step_rad=math.pi,
        coarse_beams=1, refine_beams=1, refine_count=2,
        fine_position_step_m=0.1, fine_yaw_step_rad=0.1,
        fine_position_radius_m=0.2, fine_yaw_radius_rad=0.2,
        fine_seed_count=1, final_position_step_m=0.02,
        final_yaw_step_rad=0.02, final_position_radius_m=0.04,
        final_yaw_radius_rad=0.04)

    assert candidates[0]['x'] == pytest.approx(target[0])
    assert candidates[0]['y'] == pytest.approx(target[1])
    assert candidates[0]['yaw'] == pytest.approx(target[2])
