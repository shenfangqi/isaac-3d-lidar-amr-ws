"""
Issue #13 PR2: read-only rotation sweep gate (spec S01-S06).

Scans are ray-cast analytically against wall segments in the odom frame;
evidence grids are otherwise built cell by cell.  No ROS graph is used.
"""

import json
import math
from types import SimpleNamespace as NS

import pytest

from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError,
    MotionProfile,
    RotationAttestation,
    SE2,
)
from isaac_3d_lidar_bringup.localization_rotation_policy import (
    add_obstacle_points,
    braking_extension,
    classify_sweep,
    evaluate_localization_rotation,
    evidence_from_scan,
    footprint_geometry_hash,
    OCCUPIED,
    preview_payload,
    RotationGateConfig,
    SweepEvidence,
)


FOOTPRINT = ((0.155, 0.133), (0.155, -0.133),
             (-0.130, -0.133), (-0.130, 0.133))
ORIGIN = SE2(0.0, 0.0, 0.0)
EXTRINSICS = 'a' * 64
CONTROL = 'b' * 64
# Pure geometry: no padding, drift or braking unless a test adds them.
EXACT = RotationGateConfig(padding_m=0.0, yaw_margin_rad=0.0,
                           unmeasured_braking_rad=0.0,
                           unmeasured_center_drift_m=0.0,
                           chassis_watchdog_s=0.0)


def _profile(config, stop_tail=0.0, latency=0.0, drift=0.0,
             status='ACCEPTED'):
    return MotionProfile(
        schema_version=1,
        geometry_hash=footprint_geometry_hash(FOOTPRINT, config.padding_m),
        extrinsics_hash=EXTRINSICS, control_chain_hash=CONTROL,
        evidence_ids=('bag-1',), stop_tail_rad=stop_tail,
        center_drift_m=drift, latency_s=latency,
        externally_reviewed=True, status=status)


def _hashes(config):
    return (footprint_geometry_hash(FOOTPRINT, config.padding_m),
            EXTRINSICS, CONTROL)


def _evaluate(evidence, delta, config, profile='accepted', **kwargs):
    if profile == 'accepted':
        profile = _profile(config)
    return evaluate_localization_rotation(
        evidence, FOOTPRINT, ORIGIN, delta, profile=profile,
        hashes=_hashes(config), config=config, **kwargs)


def _grid(free=True, resolution=0.01, half=0.5, occupied_points=(),
          unknown=lambda x, y: False):
    """Evidence centred on the origin; ``unknown(x, y)`` clears free."""
    cells = round(2 * half / resolution)
    origin = -cells * resolution / 2.0
    observed, occupied = [], set()
    for my in range(cells):
        for mx in range(cells):
            cx = origin + (mx + 0.5) * resolution
            cy = origin + (my + 0.5) * resolution
            observed.append(free and not unknown(cx, cy))
    for x, y in occupied_points:
        index = (math.floor((y - origin) / resolution) * cells
                 + math.floor((x - origin) / resolution))
        occupied.add(index)
        observed[index] = False
    return SweepEvidence(cells, cells, resolution, origin, origin,
                         tuple(observed), frozenset(occupied), 'odom', 1000)


def _polar(radius, degrees):
    angle = math.radians(degrees)
    return (radius * math.cos(angle), radius * math.sin(angle))


def _square(cx, cy, half):
    corners = [(cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def _scan(walls, range_min, increment=0.01, range_max=8.0):
    ranges = []
    count = round(2 * math.pi / increment)
    for index in range(count):
        angle = -math.pi + index * increment
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        for (ax, ay), (bx, by) in walls:
            ex, ey = bx - ax, by - ay
            denominator = dx * ey - dy * ex
            if abs(denominator) < 1e-12:
                continue
            t = (ax * ey - ay * ex) / denominator
            s = (ax * dy - ay * dx) / denominator
            if t > 0.0 and 0.0 <= s <= 1.0:
                best = min(best, t)
        ranges.append(best if range_min <= best <= range_max else math.inf)
    return NS(header=NS(stamp=NS(sec=5, nanosec=0)), angle_min=-math.pi,
              angle_increment=increment, range_min=range_min,
              range_max=range_max, ranges=tuple(ranges))


ROOM = _square(0.0, 0.0, 1.2)


def test_near_but_outside_sweep():
    # S01: a post 0.45 m away is inside the old 0.55 m all-round gate but
    # outside the swept footprint; every swept cell has ray evidence.
    config = RotationGateConfig()
    walls = ROOM + _square(0.47, 0.0, 0.02)
    evidence = evidence_from_scan(_scan(walls, range_min=0.05), ORIGIN,
                                  (0.0, 0.0), resolution=0.025)

    result = _evaluate(evidence, math.radians(30), config)

    assert result.decision.allowed, result.decision
    assert result.decision.min_clearance_m < 0.55
    assert result.decision.unknown_cells == 0


def test_corner_collision_mid_arc():
    # S02: neither the start nor the end footprint touches the post, but
    # the corner (radius 0.204 m) passes over it at about 44 deg.
    post = _polar(0.19, 85.0)
    evidence = _grid(occupied_points=[post])
    for yaw in (0.0, math.pi / 2):
        cells, _ = classify_sweep(evidence, FOOTPRINT, SE2(0, 0, yaw),
                                  1e-6, 0.0)
        assert cells[OCCUPIED] == []

    result = _evaluate(evidence, math.pi / 2, EXACT)

    assert not result.decision.allowed
    assert result.decision.reason == 'OBSTACLE_IN_SWEEP'


def test_blind_zone_is_unknown():
    # S03: /scan range_min 0.5 m leaves the whole sweep band unobserved.
    # A static map is not an input, so map free space cannot clear it.
    config = RotationGateConfig()
    evidence = evidence_from_scan(_scan(ROOM, range_min=0.5), ORIGIN,
                                  (0.0, 0.0), resolution=0.025)

    result = _evaluate(evidence, math.radians(30), config)

    assert not result.decision.allowed
    assert result.decision.reason == 'UNKNOWN_SWEEP'
    assert result.decision.unknown_cells > 0
    assert result.cells[OCCUPIED] == []


def _inside_body(x, y, half_cell=0.005):
    return (-0.130 + half_cell <= x <= 0.155 - half_cell
            and -0.133 + half_cell <= y <= 0.133 - half_cell)


def test_self_mask_not_padding():
    # S04: the body interior can never be observed and is masked; an
    # unobserved cell in the padding ring outside the body still rejects.
    config = RotationGateConfig(yaw_margin_rad=0.0,
                                unmeasured_braking_rad=0.0,
                                unmeasured_center_drift_m=0.0)
    interior = _grid(unknown=_inside_body)
    result = _evaluate(interior, math.radians(30), config)
    assert result.decision.allowed, result.decision
    assert result.self_cell_count > 0

    ring_cell = (0.175, 0.005)          # a cell centre in the padding ring

    def unknown(x, y):
        return (_inside_body(x, y) or (
            abs(x - ring_cell[0]) < 0.005 and abs(y - ring_cell[1]) < 0.005))

    result = _evaluate(_grid(unknown=unknown), math.radians(30), config)
    assert result.decision.reason == 'UNKNOWN_SWEEP'
    assert result.decision.unknown_cells == 1


def test_braking_sweep():
    # S05: the 30 deg target is clear, but the post sits in the 0.3 rad
    # stop tail beyond it, so the rotation must be refused before it starts.
    evidence = _grid(occupied_points=[_polar(0.195, 80.0)])
    stopped_at_target = _evaluate(
        evidence, math.radians(30), EXACT, profile=_profile(EXACT))
    assert stopped_at_target.decision.allowed, stopped_at_target.decision

    with_tail = _evaluate(evidence, math.radians(30), EXACT,
                          profile=_profile(EXACT, stop_tail=0.3))
    assert with_tail.decision.reason == 'OBSTACLE_IN_SWEEP'
    assert with_tail.braking_measured
    assert with_tail.swept_angle_rad == pytest.approx(math.radians(30) + 0.3)

    # Without measured values the conservative default tail applies.
    unmeasured = _evaluate(
        evidence, math.radians(30),
        RotationGateConfig(padding_m=0.0, unmeasured_center_drift_m=0.0),
        profile=None)
    assert unmeasured.decision.reason == 'OBSTACLE_IN_SWEEP'
    assert not unmeasured.braking_measured


def test_sweep_covers_guard_crash_until_chassis_watchdog():
    # 2026-10-06: with the guard dead the chassis kept the last command for
    # 0.49-0.57 s.  A post beyond the normal stop (latency 0.171 s) but
    # within the watchdog hold must refuse the probe.
    import dataclasses

    evidence = _grid(occupied_points=[_polar(0.195, 85.0)])
    measured = _profile(EXACT, stop_tail=0.001, latency=0.171)
    no_crash = _evaluate(evidence, math.radians(30), EXACT, profile=measured)
    assert no_crash.decision.allowed, no_crash.decision

    crash = dataclasses.replace(EXACT, chassis_watchdog_s=0.6)
    result = _evaluate(evidence, math.radians(30), crash,
                       profile=_profile(crash, stop_tail=0.001,
                                        latency=0.171))
    assert result.decision.reason == 'OBSTACLE_IN_SWEEP'
    assert result.braking_rad == pytest.approx(0.4 * 0.6 + 0.001)


def test_braking_extension_uses_the_longer_hold():
    config = RotationGateConfig(yaw_margin_rad=0.0, chassis_watchdog_s=0.6)
    short = _profile(config, stop_tail=0.01, latency=0.2)
    long = _profile(config, stop_tail=0.01, latency=0.9)
    assert braking_extension(short, config)[0] == pytest.approx(
        0.4 * 0.6 + 0.01)
    assert braking_extension(long, config)[0] == pytest.approx(
        0.4 * 0.9 + 0.01)
    unmeasured, _, flag = braking_extension(None, config)
    assert unmeasured == pytest.approx(0.4 * 0.6 + 0.40) and not flag


def test_sparse_3d_not_free():
    # S06: only returns above the collision band, nothing low: points add
    # obstacles at most and never turn the gaps between them into free.
    blank = _grid(free=False)
    high = [(_polar(0.2, angle) + (0.6,)) for angle in range(0, 360, 15)]
    evidence = add_obstacle_points(blank, high, z_min=0.01, z_max=0.24)
    assert evidence.occupied == frozenset()
    assert not any(evidence.observed_free)
    assert _evaluate(evidence, math.radians(30),
                     RotationGateConfig()).decision.reason == 'UNKNOWN_SWEEP'

    low = add_obstacle_points(_grid(), [(0.4, 0.0, 0.1)], 0.01, 0.24)
    mx, my = low.cell(0.4, 0.0)
    assert low.state(mx, my) == OCCUPIED


def test_cells_across_the_scan_seam_use_real_rays():
    # A full-circle /scan has its seam behind the robot (angle_min = -pi);
    # those cells must still be cleared by the rays on both sides.
    evidence = evidence_from_scan(_scan(ROOM, range_min=0.05), ORIGIN,
                                  (0.0, 0.0), resolution=0.025)
    mx, my = evidence.cell(-0.25, 0.0)
    assert evidence.state(mx, my) == 'OBSERVED_FREE'

    partial = _scan(ROOM, range_min=0.05)
    keep = len(partial.ranges) - 40         # 0.4 rad gap behind the robot
    partial = NS(header=partial.header, angle_min=partial.angle_min + 0.2,
                 angle_increment=partial.angle_increment,
                 range_min=0.05, range_max=8.0,
                 ranges=partial.ranges[20:20 + keep])
    evidence = evidence_from_scan(partial, ORIGIN, (0.0, 0.0),
                                  resolution=0.025)
    assert evidence.state(*evidence.cell(-0.25, 0.0)) == 'UNKNOWN'


def test_observed_obstacle_is_never_attested_away():
    session = 'abcdef0123456789'
    attestation = RotationAttestation(session, ORIGIN, 10.0, 0.05, 60.0)
    blind = evidence_from_scan(_scan(ROOM, range_min=0.5), ORIGIN,
                               (0.0, 0.0), resolution=0.025)
    config = RotationGateConfig()

    covered = _evaluate(blind, math.radians(30), config,
                        attestation=attestation, session=session,
                        now_mono=20.0)
    assert covered.decision.allowed, covered.decision
    assert covered.decision.unknown_cells == 0
    assert covered.decision.attested_cells > 0

    blocked = add_obstacle_points(blind, [(0.0, 0.2, 0.1)], 0.01, 0.24)
    result = _evaluate(blocked, math.radians(30), config,
                       attestation=attestation, session=session,
                       now_mono=20.0)
    assert result.decision.reason == 'OBSTACLE_IN_SWEEP'

    moved = evaluate_localization_rotation(
        blind, FOOTPRINT, SE2(0.2, 0.0, 0.0), math.radians(30),
        profile=_profile(config), hashes=_hashes(config), config=config,
        attestation=attestation, session=session, now_mono=20.0)
    assert moved.decision.reason == 'UNKNOWN_SWEEP'


@pytest.mark.parametrize('profile, hashes', [
    (None, 'match'),
    ('estimated', 'match'),
    ('accepted', ('c' * 64, EXTRINSICS, CONTROL)),
    ('accepted', None),
])
def test_profile_must_be_accepted_and_match(profile, hashes):
    config = EXACT
    profile = {None: None,
               'estimated': _profile(config, status='ESTIMATED'),
               'accepted': _profile(config)}[profile]
    result = evaluate_localization_rotation(
        _grid(), FOOTPRINT, ORIGIN, math.radians(30), profile=profile,
        hashes=_hashes(config) if hashes == 'match' else hashes,
        config=config)
    assert result.decision.reason == 'PROFILE_INVALID'


def test_cells_outside_evidence_window_are_unknown():
    tiny = _grid(half=0.1)
    result = _evaluate(tiny, math.radians(30), EXACT)
    assert result.decision.reason == 'UNKNOWN_SWEEP'


@pytest.mark.parametrize('delta', [0.0, math.pi, math.nan])
def test_invalid_probe_angle_is_rejected(delta):
    with pytest.raises(ContractError):
        _evaluate(_grid(), delta, EXACT)


def test_geometry_hash_binds_padding():
    assert (footprint_geometry_hash(FOOTPRINT, 0.05)
            != footprint_geometry_hash(FOOTPRINT, 0.06))


def test_preview_payload_is_strict_json_and_never_commands_motion():
    config = RotationGateConfig()
    evidence = evidence_from_scan(_scan(ROOM, range_min=0.5), ORIGIN,
                                  (0.0, 0.0), resolution=0.025)
    evaluations = [_evaluate(evidence, angle, config, profile=None)
                   for angle in (math.radians(30), -math.radians(30))]
    payload = preview_payload(evaluations, geometry_hash='0' * 64,
                              profile_state='none')

    text = json.dumps(payload, allow_nan=False)
    assert json.loads(text)['motion_commanded'] is False
    assert [p['reason'] for p in payload['probes']] == ['UNKNOWN_SWEEP'] * 2
    assert payload['probes'][0]['reason_text']


# --- Read-only preview node -------------------------------------------------

PACKAGE_DIR = __import__('pathlib').Path(__file__).resolve().parents[1]


def test_preview_node_has_no_velocity_output():
    source = (PACKAGE_DIR / 'isaac_3d_lidar_bringup'
              / 'localization_rotation_preview.py').read_text()
    assert 'Twist' not in source
    assert 'cmd_vel' not in source
    publishers = [line for line in source.splitlines()
                  if 'create_publisher(' in line]
    assert len(publishers) == 2
    assert 'MarkerArray, self.get_parameter' in source
    assert 'String, self.get_parameter' in source


def test_preview_is_opt_in_in_launch_and_registered():
    launch = (PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'
              ).read_text()
    assert "executable='localization_rotation_preview'" in launch
    assert 'condition=IfCondition(rotation_preview)' in launch
    block = launch.split("'rotation_preview',", 1)[1]
    assert block.lstrip().startswith("default_value='false'")
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    assert ('isaac_3d_lidar_bringup.localization_rotation_preview:main'
            in setup)


def _preview_node(tf_ok=True):
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformException

    import isaac_3d_lidar_bringup.localization_rotation_preview as module

    node = object.__new__(module.RotationPreview)
    node._footprint = FOOTPRINT
    node._config = RotationGateConfig()
    node._angles = [math.pi / 6, -math.pi / 6, math.pi / 3, -math.pi / 3,
                    math.pi / 2, -math.pi / 2]
    node._geometry_hash = footprint_geometry_hash(FOOTPRINT, 0.05)
    node._profile, node._profile_state, node._hashes = None, 'none', None
    node._odom_frame, node._base_frame = 'odom', 'base_footprint'
    parameters = {'compute_budget_sec': 2.0, 'evidence_half_extent_m': 1.0,
                  'resolution': 0.05}
    node.get_parameter = lambda name: NS(value=parameters[name])

    def lookup(target, source, stamp):
        if not tf_ok:
            raise TransformException('no transform at source time')
        return NS(transform=NS(translation=NS(x=0.0, y=0.0, z=0.0),
                               rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))

    node._tf_buffer = NS(lookup_transform=lookup)
    synthetic = _scan(ROOM, range_min=0.5, increment=0.0175)
    scan = LaserScan()
    scan.header.frame_id = 'base_footprint'
    scan.header.stamp.sec = 5
    scan.angle_min = synthetic.angle_min
    scan.angle_increment = synthetic.angle_increment
    scan.range_min, scan.range_max = 0.5, 8.0
    scan.ranges = list(synthetic.ranges)
    node._scan = scan
    node._scan_received = __import__('time').monotonic()
    node.status, node.markers = [], []
    node._status = NS(publish=lambda msg: node.status.append(msg.data))
    node._markers = NS(publish=lambda msg: node.markers.append(msg))
    return node


def test_preview_reports_real_blind_zone_without_commanding_motion():
    node = _preview_node()
    node._evaluate()

    payload = json.loads(node.status[-1])
    assert payload['motion_commanded'] is False
    assert payload['profile_state'] == 'none'
    assert [p['reason'] for p in payload['probes']] == ['UNKNOWN_SWEEP'] * 6
    assert all(p['unknown_cells'] > 0 for p in payload['probes'])
    namespaces = {m.ns for m in node.markers[-1].markers}
    assert {'rotation_sweep_cells', 'rotation_footprint',
            'rotation_probe_verdicts'} <= namespaces


def test_preview_refuses_without_source_time_tf():
    node = _preview_node(tf_ok=False)
    node._evaluate()

    payload = json.loads(node.status[-1])
    assert payload['error'] == 'TF_AT_SOURCE_MISSING'
    assert payload['motion_commanded'] is False
    assert node.markers == []
