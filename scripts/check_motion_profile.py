#!/usr/bin/env python3
"""
Check an Issue #13 rotation motion profile before guarded startup.

Runs on the workstation without ROS.  The profile must decode strictly, be
ACCEPTED with an external review, and match the geometry, extrinsics and
control-chain hashes recomputed from the canonical parameters.  On success
prints a JSON object with the hashes and the request profile_hash; on
failure prints the reason (Chinese and English) and exits 1.
"""

import argparse
import json
from pathlib import Path
import sys

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(WORKSPACE / 'src/isaac_3d_lidar_bringup'),
                str(WORKSPACE / 'src/carbot_nav_recovery')]

import yaml  # noqa: E402

from isaac_3d_lidar_bringup.localization_contracts import (  # noqa: E402
    ContractError,
    decode_motion_profile,
    ProfileStatus,
)
from isaac_3d_lidar_bringup.localization_motion_guard import (  # noqa: E402
    profile_hash,
)
from isaac_3d_lidar_bringup.localization_profile_analysis import (  # noqa: E402
    section_hash,
)
from isaac_3d_lidar_bringup.localization_rotation_policy import (  # noqa: E402
    footprint_geometry_hash,
)


def canonical_hashes(parameters, padding_m):
    footprint = tuple(tuple(point) for point in
                      parameters['geometry']['footprint_m'])
    return (footprint_geometry_hash(footprint, padding_m),
            section_hash(parameters['sensors']['mid360']),
            section_hash(parameters['control']))


def check(profile_text, parameters, padding_m=0.05):
    """Return the result mapping, or raise ValueError with the reason."""
    try:
        profile = decode_motion_profile(profile_text)
    except ContractError as error:
        raise ValueError(f'配置格式无效 / invalid profile: {error}')
    if profile.status != ProfileStatus.ACCEPTED or (
            not profile.externally_reviewed):
        raise ValueError(
            f'配置未验收（{profile.status.value}）/ profile is not '
            'ACCEPTED with an external review')
    geometry, extrinsics, control = canonical_hashes(parameters, padding_m)
    mismatched = [name for name, ours, theirs in (
        ('geometry_hash', geometry, profile.geometry_hash),
        ('extrinsics_hash', extrinsics, profile.extrinsics_hash),
        ('control_chain_hash', control, profile.control_chain_hash))
        if ours != theirs]
    if mismatched:
        raise ValueError(
            f'车体/外参/控制链已变化，配置失效：{", ".join(mismatched)} / '
            'profile no longer matches the canonical parameters')
    return {'geometry_hash': geometry, 'extrinsics_hash': extrinsics,
            'control_chain_hash': control,
            'profile_hash': profile_hash(profile)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', type=Path)
    parser.add_argument('--parameters', type=Path, default=WORKSPACE / (
        'src/carbot_description/config/carbot_parameters.yaml'))
    parser.add_argument('--padding', type=float, default=0.05)
    args = parser.parse_args()
    try:
        parameters = yaml.safe_load(args.parameters.read_text('utf-8'))
        result = check(args.profile.read_text('utf-8'), parameters,
                       args.padding)
    except (OSError, ValueError) as error:
        print(f'MOTION_PROFILE_REJECTED: {error}', file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
