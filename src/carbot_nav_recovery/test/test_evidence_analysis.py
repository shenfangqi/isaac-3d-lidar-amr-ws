import pytest

from carbot_nav_recovery.evidence_analysis import summarize_advisories


def report(reason='OK', speed=0.08):
    return {
        'schema': 'carbot_complex_route_advisory_v1',
        'reason': reason,
        'recommended_speed_mps': speed,
        'current_speed_mps': 0.10,
        'braking_calibrated': True,
        'speed_reduction_required': speed < 0.10,
        'validation_only': True,
        'motion_eligible': False,
        'command_applied': False,
        'navigation_config_sha256': 'abc',
        'risk': {
            'distance_to_unsafe_m': 0.25,
            'maximum_curvature_inv_m': 1.0,
        },
    }


def test_summary_accepts_reports_and_saved_snapshots():
    summary = summarize_advisories([
        report(),
        {'schema': 'carbot_complex_route_snapshot_v1',
         'report': report('CURVATURE_SPEED_UNSAFE', 0.05)},
    ])
    assert summary['records'] == 2
    assert summary['reasons'] == {
        'CURVATURE_SPEED_UNSAFE': 1, 'OK': 1}
    assert summary['recommended_speed_mps']['median'] == pytest.approx(0.065)
    assert summary['speed_reduction_records'] == 2
    assert summary['all_validation_only'] is True
    assert summary['any_motion_eligible'] is False
    assert summary['any_command_applied'] is False
    assert summary['navigation_config_sha256'] == ['abc']


def test_summary_rejects_unknown_schema_and_empty_input():
    with pytest.raises(ValueError, match='no advisory'):
        summarize_advisories([])
    with pytest.raises(ValueError, match='unsupported'):
        summarize_advisories([{'schema': 'other', 'reason': 'OK'}])
