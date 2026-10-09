"""
Issue #13 saved-pose prior: the last trusted map pose, kept across restarts.

While navigation is READY the manager periodically saves the AMCL pose.  At
the next start the saved pose is used only when the operator attests for
that launch that the robot has not been moved since (``robot_not_moved``):
it may then pick, among validated candidates that are too close to tell
apart, the single one at the saved pose.  It never lowers a gate.

A near-symmetric twin of a placement (2026-10-09: map (-0.7, 0.05) and
(4.7, 6.2), holdout scores 0.03-0.07 apart in either order) cannot be told
apart by rotating in place; the attestation is the extra information.  If
the robot was carried to the twin, the attestation is false and the pose
would be wrong, so the prior is opt-in per launch.
"""

from dataclasses import dataclass
import json
import math
import os
import tempfile

from isaac_3d_lidar_bringup.localization_contracts import ContractError

SCHEMA_VERSION = 1


@dataclass(frozen=True)
class SavedPose:
    """A trusted map pose and the map it belongs to."""

    map_hash: str
    x: float
    y: float
    yaw: float
    saved_unix: float
    xy_std: float
    yaw_std: float

    def __post_init__(self):
        if not (isinstance(self.map_hash, str) and len(self.map_hash) == 64
                and all(c in '0123456789abcdef' for c in self.map_hash)):
            raise ContractError('map_hash must be 64 lowercase hex digits')
        for name in ('x', 'y', 'yaw', 'saved_unix', 'xy_std', 'yaw_std'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(
                    value, (int, float)) or not math.isfinite(value):
                raise ContractError(f'{name} must be a finite number')
        if self.xy_std < 0.0 or self.yaw_std < 0.0:
            raise ContractError('standard deviations cannot be negative')


def encode_saved_pose(pose):
    """Canonical JSON text of a SavedPose."""
    return json.dumps({
        'schema_version': SCHEMA_VERSION, 'map_hash': pose.map_hash,
        'x': pose.x, 'y': pose.y, 'yaw': pose.yaw,
        'saved_unix': pose.saved_unix, 'xy_std': pose.xy_std,
        'yaw_std': pose.yaw_std}, sort_keys=True)


def decode_saved_pose(text):
    """Strictly decode a SavedPose; ContractError on anything unexpected."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError) as error:
        raise ContractError(f'saved pose is not JSON: {error}') from error
    expected = {'schema_version', 'map_hash', 'x', 'y', 'yaw', 'saved_unix',
                'xy_std', 'yaw_std'}
    if not isinstance(data, dict) or set(data) != expected:
        raise ContractError('saved pose has unexpected fields')
    if data.pop('schema_version') != SCHEMA_VERSION:
        raise ContractError('unsupported saved pose schema')
    return SavedPose(**data)


def save_pose(path, pose):
    """Write atomically: a crash leaves the old file or the new one."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    handle, temporary = tempfile.mkstemp(dir=directory, prefix='.pose-')
    try:
        with os.fdopen(handle, 'w', encoding='utf-8') as stream:
            stream.write(encode_saved_pose(pose))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def load_pose(path, map_hash, now_unix, max_age_s):
    """
    Return ``(SavedPose, '')`` if usable, else ``(None, reason)``.

    The pose must exist, decode strictly, belong to this map and be no older
    than ``max_age_s``.
    """
    try:
        with open(path, encoding='utf-8') as stream:
            pose = decode_saved_pose(stream.read())
    except FileNotFoundError:
        return None, 'no saved pose'
    except (OSError, ContractError) as error:
        return None, f'saved pose unreadable: {error}'
    if pose.map_hash != map_hash:
        return None, 'saved pose belongs to another map'
    age = now_unix - pose.saved_unix
    if age < 0.0 or age > max_age_s:
        return None, f'saved pose age {age:.0f} s is outside 0..{max_age_s:.0f} s'
    return pose, ''
