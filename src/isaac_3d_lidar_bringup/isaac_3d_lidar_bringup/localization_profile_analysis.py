"""
Rotation motion-profile analysis from recorded command/odometry (Issue #13).

Each in-place rotation command is one segment.  For every segment this
measures, from odometry only:

* start latency: command on -> first yaw change (report only);
* stop latency: command off -> onset of deceleration;
* stop tail: yaw from deceleration onset until settled;
* centre drift: largest planar displacement during the segment and tail.

Stop latency and stop tail are measured separately so the guard's
``rate * latency + tail`` does not count the same angle twice.  Profile
values are the worst valid sample.  The result is ESTIMATED only with at
least ``min_per_direction`` valid stops in each direction, otherwise
INSUFFICIENT with null measurements.  This code never produces REVIEWED or
ACCEPTED.
"""

from dataclasses import dataclass
import hashlib
import json
import math
import statistics

from .localization_contracts import (
    MotionProfile,
    ProfileStatus,
    SCHEMA_VERSION,
)


@dataclass(frozen=True)
class TwistSample:
    t: float
    linear: float
    angular: float


@dataclass(frozen=True)
class OdomPoint:
    t: float
    x: float
    y: float
    yaw: float
    angular: float


@dataclass(frozen=True)
class StopSample:
    """One analysed rotation; invalid samples keep a reason."""

    direction: int
    commanded_rate: float
    valid: bool
    note: str = ''
    start_latency_s: float = None
    stop_latency_s: float = None
    stop_tail_rad: float = None
    center_drift_m: float = None
    rate_at_stop: float = None


def section_hash(section):
    """Stable hash of a parameter subtree (extrinsics, control chain)."""
    text = json.dumps(section, separators=(',', ':'), sort_keys=True)
    return hashlib.sha256(text.encode()).hexdigest()


def command_segments(commands, epsilon=1e-4, max_linear=1e-3):
    """Return ``(t_on, t_off, rate)`` for each pure in-place rotation."""
    segments = []
    start = None
    rate = 0.0
    for sample in commands:
        turning = (abs(sample.angular) > epsilon
                   and abs(sample.linear) <= max_linear)
        if turning and start is None:
            start, rate = sample.t, sample.angular
        elif start is not None and not turning:
            segments.append((start, sample.t, rate))
            start = None
    return segments


def _unwrap(points):
    yaws = [points[0].yaw]
    for previous, point in zip(points, points[1:]):
        step = math.atan2(math.sin(point.yaw - previous.yaw),
                          math.cos(point.yaw - previous.yaw))
        yaws.append(yaws[-1] + step)
    return yaws


def analyze_segment(segment, odometry, *, stop_angular=0.03, settle_s=0.5,
                    max_gap_s=0.2, tail_window_s=3.0, motion_rad=0.01):
    """Analyse one ``(t_on, t_off, rate)`` command segment."""
    t_on, t_off, rate = segment
    direction = 1 if rate > 0.0 else -1
    window = [p for p in odometry
              if t_on - 0.5 <= p.t <= t_off + tail_window_s]
    if len(window) < 5:
        return StopSample(direction, rate, False, 'no odometry')
    if any(b.t - a.t > max_gap_s for a, b in zip(window, window[1:])):
        return StopSample(direction, rate, False, 'odometry gap')
    yaws = _unwrap(window)
    index_on = min(range(len(window)),
                   key=lambda i: abs(window[i].t - t_on))
    moved = next((i for i in range(index_on, len(window))
                  if abs(yaws[i] - yaws[index_on]) > motion_rad), None)
    if moved is None or window[moved].t > t_off:
        return StopSample(direction, rate, False, 'no motion')
    before_off = [p.angular for p in window if t_off - 0.3 <= p.t < t_off]
    if not before_off:
        return StopSample(direction, rate, False, 'no odometry at stop')
    rate_at_stop = abs(statistics.median(before_off))
    after = [i for i in range(len(window)) if window[i].t >= t_off]
    onset = next((i for i in after
                  if abs(window[i].angular) < 0.9 * rate_at_stop), None)
    if onset is None:
        return StopSample(direction, rate, False, 'no deceleration')
    settled = None
    for i in range(onset, len(window)):
        if abs(window[i].angular) >= stop_angular:
            continue
        end = next((j for j in range(i, len(window))
                    if window[j].t - window[i].t >= settle_s), None)
        if end is None:
            break
        if all(abs(window[k].angular) < stop_angular
               for k in range(i, end + 1)):
            settled = i
            break
    if settled is None:
        return StopSample(direction, rate, False, 'did not settle')
    origin = window[index_on]
    drift = max(math.hypot(p.x - origin.x, p.y - origin.y)
                for p in window[index_on:settled + 1])
    return StopSample(
        direction, rate, True,
        start_latency_s=window[moved].t - t_on,
        stop_latency_s=window[onset].t - t_off,
        stop_tail_rad=abs(yaws[settled] - yaws[onset]),
        center_drift_m=drift, rate_at_stop=rate_at_stop)


def summarize(samples, min_per_direction=3):
    """Aggregate samples; worst valid values become profile measurements."""
    valid = [s for s in samples if s.valid]
    counts = {d: sum(1 for s in valid if s.direction == d) for d in (1, -1)}
    sufficient = all(count >= min_per_direction for count in counts.values())

    def stat(name):
        values = [getattr(s, name) for s in valid]
        if not values:
            return None
        return {'max': max(values), 'median': statistics.median(values),
                'n': len(values)}

    return {
        'samples': len(samples),
        'valid': len(valid),
        'valid_left': counts[1],
        'valid_right': counts[-1],
        'invalid_notes': sorted({s.note for s in samples if not s.valid}),
        'sufficient': sufficient,
        'start_latency_s': stat('start_latency_s'),
        'stop_latency_s': stat('stop_latency_s'),
        'stop_tail_rad': stat('stop_tail_rad'),
        'center_drift_m': stat('center_drift_m'),
        'rate_at_stop': stat('rate_at_stop'),
    }


def build_profile(summary, geometry_hash, extrinsics_hash,
                  control_chain_hash, evidence_ids):
    """ESTIMATED with worst-case values, or INSUFFICIENT with nulls."""
    if summary['sufficient']:
        status = ProfileStatus.ESTIMATED
        values = {name: summary[name]['max']
                  for name in ('stop_tail_rad', 'center_drift_m')}
        values['latency_s'] = summary['stop_latency_s']['max']
    else:
        status = ProfileStatus.INSUFFICIENT
        values = {'stop_tail_rad': None, 'center_drift_m': None,
                  'latency_s': None}
    return MotionProfile(
        SCHEMA_VERSION, geometry_hash, extrinsics_hash, control_chain_hash,
        tuple(evidence_ids), values['stop_tail_rad'],
        values['center_drift_m'], values['latency_s'], False, status)
