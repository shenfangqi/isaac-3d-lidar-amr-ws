"""Offline summaries for Issue #12 validation-only advisory evidence."""

from collections import Counter
import math
from statistics import median


def _finite(values):
    return [float(value) for value in values
            if isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value)]


def summarize_advisories(records):
    """Summarize status reports or saved snapshot documents."""
    normalized = []
    for item in records:
        if not isinstance(item, dict):
            raise ValueError('evidence records must be JSON objects')
        if item.get('schema') == 'carbot_complex_route_snapshot_v1':
            item = item.get('report')
        if (not isinstance(item, dict)
                or item.get('schema') != 'carbot_complex_route_advisory_v1'):
            raise ValueError('unsupported complex-route evidence schema')
        if not isinstance(item.get('reason'), str):
            raise ValueError('advisory reason is missing')
        normalized.append(item)
    if not normalized:
        raise ValueError('no advisory records')

    reasons = Counter(item['reason'] for item in normalized)
    recommended = _finite(
        item.get('recommended_speed_mps') for item in normalized)
    current = _finite(item.get('current_speed_mps') for item in normalized)
    collision_distances = _finite(
        item.get('risk', {}).get('distance_to_unsafe_m')
        for item in normalized)
    curvatures = _finite(
        item.get('risk', {}).get('maximum_curvature_inv_m')
        for item in normalized)
    hashes = sorted({item.get('navigation_config_sha256', '')
                     for item in normalized
                     if item.get('navigation_config_sha256')})
    return {
        'schema': 'carbot_complex_route_evidence_summary_v1',
        'records': len(normalized),
        'reasons': dict(sorted(reasons.items())),
        'calibrated_records': sum(
            item.get('braking_calibrated') is True for item in normalized),
        'speed_reduction_records': sum(
            item.get('speed_reduction_required') is True
            for item in normalized),
        'recommended_speed_mps': _summary(recommended),
        'current_speed_mps': _summary(current),
        'distance_to_unsafe_m': _summary(collision_distances),
        'maximum_curvature_inv_m': _summary(curvatures),
        'navigation_config_sha256': hashes,
        'all_validation_only': all(
            item.get('validation_only') is True for item in normalized),
        'any_motion_eligible': any(
            item.get('motion_eligible') is True for item in normalized),
        'any_command_applied': any(
            item.get('command_applied') is True for item in normalized),
    }


def _summary(values):
    if not values:
        return None
    return {
        'minimum': min(values),
        'median': median(values),
        'maximum': max(values),
    }
