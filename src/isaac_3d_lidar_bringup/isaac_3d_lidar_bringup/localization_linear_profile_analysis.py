"""
Linear (translation) motion-profile analysis from command/odometry (Issue #53).

Each straight command run is one segment.  From odometry positions only
(the FAST-LIO twist can stay biased after motion, 2026-10-08), every
segment gives:

* start latency: command on -> first 1 cm of travel (report only);
* reaction: command off -> onset of deceleration;
* stop tail: travel from deceleration onset until settled;
* lateral error: largest distance from the start line (start position along
  the start heading) during the segment and its tail;
* tracking: speed just before the stop over the commanded speed.

A segment whose commands simply cease (no zero command within ``cut_gap_s``)
is a watchdog trial: its ``watchdog_s`` is the time from the last command
until settled.  Profile values are the worst valid sample.  The profile is
ESTIMATED only with ``min_per_direction`` valid stops forward and backward
and ``min_watchdog`` watchdog trials; this code never produces REVIEWED or
ACCEPTED, which need an external cross-check and a human decision.
"""

from dataclasses import dataclass
import math
import statistics

from .localization_translation_guard import LinearProfile


@dataclass(frozen=True)
class Command:
    t: float
    linear: float
    angular: float


@dataclass(frozen=True)
class Pose:
    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Segment:
    t_on: float
    t_off: float
    speed: float
    cut: bool                  # commands ceased without a zero (watchdog trial)


@dataclass(frozen=True)
class LinearSample:
    """One analysed straight run; an invalid one keeps its reason."""

    direction: int
    commanded: float
    cut: bool
    valid: bool
    note: str = ''
    start_latency_s: float = None
    reaction_s: float = None
    stop_tail_m: float = None
    watchdog_s: float = None
    travel_m: float = None
    lateral_error_m: float = None
    yaw_change_rad: float = None
    speed_at_stop: float = None


def along_and_lateral(start, x, y):
    """Signed travel along the start heading and lateral offset from that line."""
    dx, dy = x - start.x, y - start.y
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return dx * c + dy * s, -dx * s + dy * c


def linear_segments(commands, epsilon=1e-4, max_angular=1e-3, cut_gap_s=.3):
    """Return the :class:`Segment` of each pure straight command run."""
    segments, start, speed, last = [], None, 0., None
    for sample in commands:
        moving = abs(sample.linear) > epsilon and abs(sample.angular) <= max_angular
        if start is not None and last is not None and sample.t - last.t > cut_gap_s:
            segments.append(Segment(start, last.t, speed, True))
            start = None
        if moving and start is None:
            start, speed = sample.t, sample.linear
        elif start is not None and not moving:
            segments.append(Segment(start, sample.t, speed, False))
            start = None
        last = sample
    if start is not None:
        segments.append(Segment(start, last.t, speed, True))
    return segments


def _speeds(window, span_s):
    """Position-derived planar speed at every pose (centred difference)."""
    speeds = []
    for i, point in enumerate(window):
        a = next((p for p in reversed(window[:i + 1]) if point.t - p.t >= span_s / 2),
                 window[0])
        b = next((p for p in window[i:] if p.t - point.t >= span_s / 2), window[-1])
        speeds.append(math.hypot(b.x - a.x, b.y - a.y) / max(b.t - a.t, 1e-6))
    return speeds


def analyze_linear(segment, odometry, *, stop_linear=.02, settle_s=.5, max_gap_s=.2,
                   tail_window_s=3., motion_m=.01, span_s=.2, max_overspeed=1.1):
    """
    Analyse one :class:`Segment` against time-sorted :class:`Pose` odometry.

    Settled means below ``stop_linear`` for ``settle_s``: 0.02 m/s, the
    project's stationarity limit, since FAST-LIO odometry at rest shows
    0.007-0.013 m/s of position noise (2026-10-06); with 0.01 m/s one of the
    four 2026-10-13 watchdog trials never settled.
    """
    direction = 1 if segment.speed > 0 else -1
    commanded = abs(segment.speed)

    def invalid(note):
        return LinearSample(direction, segment.speed, segment.cut, False, note)
    window = [p for p in odometry
              if segment.t_on - .5 <= p.t <= segment.t_off + tail_window_s]
    if len(window) < 5:
        return invalid('no odometry')
    if any(b.t - a.t > max_gap_s for a, b in zip(window, window[1:])):
        return invalid('odometry gap')
    index_on = min(range(len(window)), key=lambda i: abs(window[i].t - segment.t_on))
    start = window[index_on]
    moved = next((i for i in range(index_on, len(window))
                  if math.hypot(window[i].x - start.x, window[i].y - start.y) > motion_m),
                 None)
    if moved is None or window[moved].t > segment.t_off:
        return invalid('no motion')
    speeds = _speeds(window, span_s)
    before = [v for p, v in zip(window, speeds) if segment.t_off - .5 <= p.t < segment.t_off]
    if not before:
        return invalid('no odometry at stop')
    at_stop = statistics.median(before)
    if at_stop > commanded * max_overspeed:
        return invalid('overspeed')
    after = [i for i, p in enumerate(window) if p.t >= segment.t_off]
    onset = next((i for i in after if speeds[i] < .9 * at_stop), None)
    if onset is None:
        return invalid('no deceleration')
    settled = None
    for i in range(onset, len(window)):
        if speeds[i] >= stop_linear:
            continue
        end = next((j for j in range(i, len(window))
                    if window[j].t - window[i].t >= settle_s), None)
        if end is None:
            break
        if all(speeds[k] < stop_linear for k in range(i, end + 1)):
            settled = i
            break
    if settled is None:
        return invalid('did not settle')
    travel, _ = along_and_lateral(start, window[settled].x, window[settled].y)
    if travel * direction <= 0:
        return invalid('wrong direction')
    lateral = max(abs(along_and_lateral(start, p.x, p.y)[1])
                  for p in window[index_on:settled + 1])
    yaw = window[settled].yaw - start.yaw
    return LinearSample(
        direction, segment.speed, segment.cut, True,
        start_latency_s=window[moved].t - segment.t_on,
        reaction_s=window[onset].t - segment.t_off,
        stop_tail_m=math.hypot(window[settled].x - window[onset].x,
                               window[settled].y - window[onset].y),
        watchdog_s=window[settled].t - segment.t_off if segment.cut else None,
        travel_m=abs(travel), lateral_error_m=lateral,
        yaw_change_rad=abs(math.atan2(math.sin(yaw), math.cos(yaw))),
        speed_at_stop=at_stop)


def summarize(samples, min_per_direction=3, min_watchdog=3):
    """Aggregate samples; the worst valid values become profile measurements."""
    valid = [s for s in samples if s.valid]
    stops = [s for s in valid if not s.cut]
    trials = [s for s in valid if s.cut]
    counts = {d: sum(1 for s in stops if s.direction == d) for d in (1, -1)}

    def stat(name, group):
        values = [getattr(s, name) for s in group if getattr(s, name) is not None]
        if not values:
            return None
        return {'max': max(values), 'median': statistics.median(values), 'n': len(values)}
    return {
        'samples': len(samples), 'valid': len(valid),
        'forward': counts[1], 'backward': counts[-1], 'watchdog_trials': len(trials),
        'invalid_notes': sorted({s.note for s in samples if not s.valid}),
        'sufficient': (all(c >= min_per_direction for c in counts.values())
                       and len(trials) >= min_watchdog),
        'start_latency_s': stat('start_latency_s', valid),
        'reaction_s': stat('reaction_s', stops),
        'stop_tail_m': stat('stop_tail_m', valid),
        'watchdog_s': stat('watchdog_s', trials),
        'lateral_error_m': stat('lateral_error_m', valid),
        'yaw_change_rad': stat('yaw_change_rad', valid),
        'tracking': (None if not valid else {
            'min': min(s.speed_at_stop / abs(s.commanded) for s in valid),
            'median': statistics.median(s.speed_at_stop / abs(s.commanded) for s in valid)}),
    }


def build_linear_profile(summary, probe_speed, hashes, evidence_ids):
    """
    Return an ESTIMATED :class:`LinearProfile`, or None if data are insufficient.

    A worst value below 1 ms (one odometry step can show an immediate
    deceleration) is recorded as 1 ms: the profile needs positive values.
    """
    if not summary['sufficient']:
        return None

    def worst(name):
        return max(summary[name]['max'], 1e-3)
    return LinearProfile(
        *hashes, tuple(evidence_ids), max_speed_mps=probe_speed,
        reaction_s=worst('reaction_s'), stop_tail_m=worst('stop_tail_m'),
        watchdog_s=worst('watchdog_s'), lateral_error_m=worst('lateral_error_m'),
        status='ESTIMATED')
