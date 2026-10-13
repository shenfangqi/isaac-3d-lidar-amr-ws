import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from isaac_3d_lidar_bringup.localization_linear_profile_analysis import (
    along_and_lateral, analyze_linear, build_linear_profile, Command, linear_segments,
    Pose, Segment, summarize)
from isaac_3d_lidar_bringup.localization_translation_contracts import (
    decode_linear_profile, encode_linear_profile)

SCRIPTS = Path(__file__).resolve().parents[3] / 'scripts'
HASHES = ('a' * 64, 'b' * 64, 'c' * 64)
HEADING = math.radians(30)


def _run(start_t, command, seconds, *, delay=.07, coast=.0, decel_s=.4, tracking=.9,
         drift=.01, cut=False, origin=(0., 0.)):
    """
    Commands and odometry of one straight run.

    The robot starts ``delay`` after the first command at ``tracking`` times
    the command, keeps going ``coast`` after the last one, then decelerates
    linearly over ``decel_s``; it drifts sideways by ``drift`` per metre.
    """
    commands = [Command(start_t + k * .05, command, 0.) for k in range(int(seconds / .05))]
    t_off = commands[-1].t + .05
    if not cut:
        commands += [Command(t_off + k * .05, 0., 0.) for k in range(60)]
    speed = command * tracking
    poses, travelled = [], 0.
    t = start_t - .5
    while t < t_off + 4.:
        if t < start_t + delay:
            v = 0.
        elif t < t_off + coast:
            v = speed
        else:
            v = speed * max(0., 1 - (t - t_off - coast) / decel_s)
        travelled += v * .05
        x = origin[0] + travelled * math.cos(HEADING) - drift * abs(travelled) * math.sin(HEADING)
        y = origin[1] + travelled * math.sin(HEADING) + drift * abs(travelled) * math.cos(HEADING)
        poses.append(Pose(t, x, y, HEADING))
        t += .05
    return commands, poses


def test_a_stop_gives_reaction_tail_and_lateral_error():
    commands, poses = _run(10., .08, 5., coast=.1)
    (segment,) = linear_segments(commands)
    assert not segment.cut and segment.speed == pytest.approx(.08)
    sample = analyze_linear(segment, poses)
    assert sample.valid, sample.note
    # The 0.07 s delay plus 1 cm at 0.072 m/s.
    assert sample.start_latency_s == pytest.approx(.07 + .01 / .072, abs=.06)
    assert .05 <= sample.reaction_s <= .25
    # Decelerating from 0.072 m/s over 0.4 s covers about 0.0144 m.
    assert sample.stop_tail_m == pytest.approx(.0144, abs=.006)
    assert sample.lateral_error_m == pytest.approx(.01 * sample.travel_m, abs=.002)
    assert sample.speed_at_stop / .08 == pytest.approx(.9, abs=.05)
    assert sample.watchdog_s is None


def test_commands_that_cease_are_a_watchdog_trial():
    commands, poses = _run(10., -.08, 2., coast=.5, cut=True)
    (segment,) = linear_segments(commands)
    assert segment.cut and segment.speed < 0
    sample = analyze_linear(segment, poses)
    assert sample.valid and sample.direction == -1
    assert sample.watchdog_s == pytest.approx(.5 + .4, abs=.1)     # coast, then decelerate
    # A pause inside a command stream also ends the run as a cut.
    gap = [Command(0., .08, 0.), Command(.05, .08, 0.), Command(1., .08, 0.),
           Command(1.05, 0., 0.)]
    assert [s.cut for s in linear_segments(gap)] == [True, False]


def test_turning_commands_and_bad_odometry_are_not_counted():
    turning = [Command(k * .05, .08, .2) for k in range(20)] + [Command(1., 0., 0.)]
    assert linear_segments(turning) == []
    commands, poses = _run(10., .08, 3.)
    segment = linear_segments(commands)[0]
    assert analyze_linear(segment, poses[:3]).note == 'no odometry'
    assert analyze_linear(segment, poses[:40] + poses[50:]).note == 'odometry gap'
    still = [Pose(p.t, 0., 0., HEADING) for p in poses]
    assert analyze_linear(segment, still).note == 'no motion'
    fast = analyze_linear(Segment(segment.t_on, segment.t_off, .05, False), poses)
    assert fast.note == 'overspeed'
    backwards = analyze_linear(Segment(segment.t_on, segment.t_off, -.08, False), poses)
    assert backwards.note == 'wrong direction'


def _samples(forward=3, backward=3, trials=3):
    samples, t = [], 0.
    for direction, count, cut in ((1, forward, False), (-1, backward, False),
                                  (1, trials, True)):
        for _ in range(count):
            commands, poses = _run(t, direction * .08, 4., coast=.5 if cut else .1, cut=cut)
            samples.append(analyze_linear(linear_segments(commands)[0], poses))
            t += 20.
    return samples


def test_the_profile_needs_both_directions_and_watchdog_trials():
    summary = summarize(_samples())
    assert summary['sufficient'] and summary['forward'] == summary['backward'] == 3
    profile = build_linear_profile(summary, .08, HASHES, ['bag_a'])
    assert profile.status == 'ESTIMATED' and profile.max_speed_mps == .08
    assert profile.watchdog_s == pytest.approx(summary['watchdog_s']['max'])
    assert profile.stop_tail_m == pytest.approx(summary['stop_tail_m']['max'])
    assert decode_linear_profile(encode_linear_profile(profile)) == profile
    assert build_linear_profile(summarize(_samples(backward=2)), .08, HASHES, ['b']) is None
    assert build_linear_profile(summarize(_samples(trials=2)), .08, HASHES, ['b']) is None


def test_along_and_lateral_follow_the_start_heading():
    start = Pose(0., 1., 2., math.pi / 2)
    assert along_and_lateral(start, 1., 2.5) == pytest.approx((.5, 0.))
    assert along_and_lateral(start, .9, 2.) == pytest.approx((0., .1))


def _capture_module():
    spec = importlib.util.spec_from_file_location(
        'jetson_linear_profile_capture', SCRIPTS / 'jetson_linear_profile_capture.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_capture_refuses_outside_the_supervised_envelope():
    capture = _capture_module()
    args = capture.parse(['--mode', 'stops', '--operator-present'])
    assert args.speed == .08 and args.distance == .4
    for extra in ([], ['--speed', '0.12'], ['--distance', '0.7'], ['--hold', '3', '--speed',
                  '0.1'], ['--max-excursion', '0.5'], ['--max-lateral', '0.3'],
                  ['--odom-timeout', '1.0']):
        argv = ['--mode', 'watchdog'] + extra
        if extra:
            argv.append('--operator-present')
        with pytest.raises(SystemExit):
            capture.parse(argv)
    origin = (0., 1., 2., math.pi / 2)
    assert capture.along_and_lateral(origin, 1., 2.5) == pytest.approx((.5, 0.))
    assert capture.turned(0., math.radians(350)) == pytest.approx(math.radians(10))
    assert capture.check_envelope(SimpleNamespace(
        operator_present=True, speed=.08, distance=.4, pairs=4, trials=4, hold=2.,
        safety_net=1.5, max_lateral=.1, max_yaw_deg=10., max_excursion=.8,
        max_path=5., odom_timeout=.5)) is None
