#!/usr/bin/env python3
"""
Check an Issue #13 linear (translation) profile before guarded startup.

Runs on the workstation without ROS.  The profile must decode strictly, be
ACCEPTED, and match the geometry, extrinsics and control-chain hashes
recomputed from the canonical parameters (the same hashes as the rotation
profile).  On success prints a JSON object with the profile digest and its
stopping extension; on failure prints the reason and exits 1.
"""

import argparse
import json
from pathlib import Path
import sys

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(WORKSPACE / 'src/isaac_3d_lidar_bringup'),
                str(WORKSPACE / 'src/carbot_nav_recovery'),
                str(WORKSPACE / 'scripts')]

import yaml  # noqa: E402

from check_motion_profile import canonical_hashes  # noqa: E402
from isaac_3d_lidar_bringup.localization_contracts import ContractError  # noqa: E402
from isaac_3d_lidar_bringup.localization_translation_contracts import (  # noqa: E402
    decode_linear_profile,
)
from isaac_3d_lidar_bringup.localization_translation_guard import (  # noqa: E402
    LinearProbeGuard,
)


def check(profile_text, parameters, padding_m=0.05):
    """Return the result mapping, or raise ValueError with the reason."""
    try:
        profile = decode_linear_profile(profile_text)
    except ContractError as error:
        raise ValueError(f'平移配置格式无效 / invalid linear profile: {error}')
    hashes = canonical_hashes(parameters, padding_m)
    guard = LinearProbeGuard(profile, expected_hashes=hashes)
    if not guard.permitted:
        raise ValueError(
            f'平移配置未验收或与本车不匹配（{profile.status}）/ linear profile is '
            'not ACCEPTED for this geometry, extrinsics and control chain')
    return {'profile_digest': profile.digest,
            'stop_extension_m': round(guard.stop_extension_m, 4),
            'max_speed_mps': profile.max_speed_mps}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', type=Path)
    parser.add_argument('--parameters', type=Path, default=WORKSPACE / (
        'src/carbot_description/config/carbot_parameters.yaml'))
    parser.add_argument('--padding', type=float, default=0.05)
    args = parser.parse_args()
    try:
        parameters = yaml.safe_load(args.parameters.read_text('utf-8'))
        result = check(args.profile.read_text('utf-8'), parameters, args.padding)
    except (OSError, ValueError) as error:
        print(f'LINEAR_PROFILE_REJECTED: {error}', file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
