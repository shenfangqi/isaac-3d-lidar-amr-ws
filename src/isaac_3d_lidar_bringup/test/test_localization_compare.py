"""End-to-end offline CLI regression; entirely synthetic, not robot acceptance."""
import json
from pathlib import Path
import subprocess
import sys

import yaml


def test_cli_compares_same_scan_without_claiming_navigation_acceptance(tmp_path):
    root = Path(__file__).resolve().parents[3]
    quaternion = dict(x=0., y=0., z=0., w=1.)
    cells = [0] * 40
    cells[13] = cells[15] = 100
    dataset = {
        'map': {'info': {'width': 10, 'height': 4, 'resolution': 1.,
                         'origin': {'position': {'x': 0., 'y': 0.},
                                    'orientation': quaternion}},
                'data': cells},
        'scans': [{'scan': {'header': {'frame_id': 'base_footprint'},
                            'ranges': [2., None], 'range_min': .1,
                            'range_max': 20., 'angle_min': 0.,
                            'angle_increment': 1.}}]}
    poses = [dict(name='synthetic_correct', x=1., y=1., yaw=0.),
             dict(name='synthetic_wrong', x=3., y=1., yaw=0.)]
    params = dict(occupied_threshold=65, scan_match_tolerance_m=0.,
                  scan_score_max_beams=120, min_scan_map_score=.65,
                  min_scan_map_coverage=.65, min_valid_scan_beams=1)
    scan_file, pose_file = tmp_path / 'scans.json', tmp_path / 'poses.json'
    config_file, report_file = tmp_path / 'config.yaml', tmp_path / 'report.json'
    scan_file.write_text(json.dumps(dataset))
    pose_file.write_text(json.dumps(poses))
    config_file.write_text(yaml.safe_dump(
        {'automatic_localization_manager': {'ros__parameters': params}}))
    subprocess.run([sys.executable, str(root / 'scripts/localization_compare.py'),
                    str(scan_file), str(pose_file), '--config', str(config_file),
                    '--output', str(report_file)], check=True, capture_output=True)
    report = json.loads(report_file.read_text())
    assert report['navigation_accepted'] is False
    assert [row['pass_fraction'] for row in report['results']] == [1., 0.]
    assert report['results'][1]['frames'][0]['legacy_score'] == 1.
    assert len(report['sha256']) == 3
