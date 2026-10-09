"""
Offline multi-height candidate comparison; never authorizes navigation.

Each layer needs its own registered occupancy map, including observed free
and unknown space, and a matching height-filtered scan. A PLY projection
with every empty cell called free is not a valid substitute. No runtime
manager integration is provided until real held-out data calibrates this
additional evidence. Scores are descriptive, not posterior probabilities.
"""

from dataclasses import dataclass, replace
import math

from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError, FrameRole, SE2,
)
from isaac_3d_lidar_bringup.localization_hypotheses import map_hash, score_pose


def _digest(value):
    return (isinstance(value, str) and len(value) == 64
            and all(c in '0123456789abcdef' for c in value))


@dataclass(frozen=True)
class HeightLayer:
    """
    A registered map slice in metres above the common navigation plane.

    The grid and hash belong to this height, not to the navigation obstacle
    projection. source_map_sha256 binds all slices to the same 3D artifact.
    Creation/export of observed-free layers is a separate verified step.
    """

    name: str
    min_z: float
    max_z: float
    grid: object
    grid_hash: str
    navigation_map_hash: str
    source_map_sha256: str

    def __post_init__(self):
        if (not self.name or not all(math.isfinite(z) for z in (self.min_z, self.max_z))
                or self.min_z >= self.max_z):
            raise ContractError('invalid height interval/name')
        if not all(_digest(d) for d in (
                self.grid_hash, self.navigation_map_hash, self.source_map_sha256)):
            raise ContractError('height layers require full SHA256 provenance')
        if self.grid.frame_id != 'map' or map_hash(self.grid) != self.grid_hash:
            raise ContractError('layer grid/frame does not match its manifest')


@dataclass(frozen=True)
class LayerObservation:
    """
    Height filter metadata accompanies stopped HOLDOUT keyframes.

    Frames must already be projected in base_footprint at source time, using
    full-attitude TF before slicing. This scorer cannot infer missing z or
    repair a flattened IMU/body transform.
    """

    name: str
    min_z: float
    max_z: float
    frames: tuple


def compare_height_layers(layers, observations, candidates, reference_odom,
                          navigation_map_hash, source_map_sha256, config,
                          thresholds, train_stamps=(), tolerance_m=0.15):
    """
    Compare all candidates without dropping poorly covered alternatives.

    A layer is comparable only when *every* candidate has enough known
    evidence. Missing/unknown geometry is therefore not a false vote for a
    well-covered candidate. Frame medians and equal view/layer weights avoid
    counting high-density/repeated scans as independent confirmations.
    """
    layers, observations, candidates = tuple(layers), tuple(observations), tuple(candidates)
    if not math.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ContractError('match tolerance must be finite and positive')
    if len(layers) < 2 or len(candidates) < 2:
        raise ContractError('comparison needs at least two layers and candidates')
    if len({h.cluster_id for h in candidates}) != len(candidates):
        raise ContractError('candidate cluster IDs must be unique')
    names = [layer.name for layer in layers]
    if len(set(names)) != len(names):
        raise ContractError('layer names must be unique')
    ordered = sorted(layers, key=lambda layer: layer.min_z)
    if any(a.max_z > b.min_z for a, b in zip(ordered, ordered[1:])):
        raise ContractError('overlapping layers would double-count geometry')
    by_name = {obs.name: obs for obs in observations}
    if len(by_name) != len(observations) or set(by_name) != set(names):
        raise ContractError('each map layer needs exactly one matching observation')
    training = set(train_stamps)
    sessions, frame_signatures = set(), []
    rows, layer_reports = [], []
    for candidate in candidates:
        rows.append({'cluster_id': candidate.cluster_id,
                     'pose_at_reference': {
                         'x': candidate.x, 'y': candidate.y,
                         'yaw': candidate.yaw},
                     'layers': {}, 'balanced_score': None})
    for layer in layers:
        if (layer.navigation_map_hash != navigation_map_hash
                or layer.source_map_sha256 != source_map_sha256
                or map_hash(layer.grid) != layer.grid_hash):
            raise ContractError('layer provenance changed or belongs to another map')
        obs = by_name[layer.name]
        if obs.min_z != layer.min_z or obs.max_z != layer.max_z or not obs.frames:
            raise ContractError('scan/map height interval mismatch or empty observations')
        stamps = set()
        for frame in obs.frames:
            if (frame.role != FrameRole.HOLDOUT or frame.stamp_ns in training
                    or frame.stamp_ns in stamps):
                raise ContractError('need unique HOLDOUT stamps disjoint from TRAIN')
            if (frame.scan.frame_id != 'base_footprint'
                    or frame.T_base_scan != SE2(0.0, 0.0, 0.0)):
                raise ContractError('height scans must be projected in base_footprint')
            stamps.add(frame.stamp_ns)
            sessions.add(frame.session)
        frame_signatures.append({(f.stamp_ns, f.view_id, f.T_odom_base) for f in obs.frames})
        comparable = True
        layer_config = replace(config, tolerance_cells=math.ceil(
            tolerance_m / layer.grid.info.resolution))
        for row, candidate in zip(rows, candidates):
            metrics = score_pose(layer.grid, SE2(candidate.x, candidate.y, candidate.yaw),
                                 obs.frames, reference_odom, layer_config, config.refine_beams)
            row['layers'][layer.name] = metrics
            comparable &= (metrics['known'] >= thresholds.min_known
                           and metrics['coverage'] >= thresholds.min_coverage)
        layer_reports.append({'name': layer.name, 'height_m': [layer.min_z, layer.max_z],
                              'grid_hash': layer.grid_hash, 'comparable': bool(comparable),
                              'reason': '' if comparable else 'INSUFFICIENT_COMMON_COVERAGE'})
    if len(sessions) != 1 or any(s != frame_signatures[0] for s in frame_signatures[1:]):
        raise ContractError('layers must share session, source stamps, view IDs and odometry')
    comparable = all(layer['comparable'] for layer in layer_reports)
    if comparable:
        for row in rows:
            row['balanced_score'] = sum(m['score'] for m in row['layers'].values()) / len(layers)
        rows.sort(key=lambda row: (-row['balanced_score'], row['cluster_id']))
    return {
        'schema': 1, 'navigation_accepted': False, 'offline_only': True,
        'navigation_map_hash': navigation_map_hash, 'source_map_sha256': source_map_sha256,
        'comparable': comparable, 'layers': layer_reports, 'candidates': rows,
        'score_margin': (rows[0]['balanced_score'] - rows[1]['balanced_score']
                         if comparable else None),
        'warning': 'Uncalibrated evidence comparison; no acceptance, pruning or motion decision.',
    }
