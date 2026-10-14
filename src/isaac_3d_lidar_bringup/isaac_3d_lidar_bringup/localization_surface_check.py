"""
Issue #13 static 3D surface re-check of 2D localization candidates.

Pure logic (numpy/scipy), no ROS.  Stationary deskewed clouds, already in
the base frame of the reference keyframe, are placed at each candidate pose
and scored per height band against the map surface: the share of points
within ``tolerance_m`` of a surface point.  The surface points are the
mesh vertices or, by default in the manager, dense samples of its triangles
(``localization_surface_model``).  Bands with too few points are excluded
for every candidate; the composite is point-weighted.

These are the scoring primitives.  The localization decision built on
them (independent cloud windows, refined candidates, bounded support and a
2D sanity check) is ``localization_surface_validation.decide_surface``.
"""

from dataclasses import dataclass
import math

import numpy as np

from .localization_contracts import ContractError

DEFAULT_BANDS = ((0.0, 0.22), (0.22, 0.35), (0.35, 0.6), (0.6, 1.0),
                 (1.0, 1.5), (1.5, 2.0), (2.0, 3.0))


@dataclass(frozen=True)
class SurfaceCheckConfig:
    """
    Thresholds of the 3D re-check.

    Initial values from three operator-labelled captures on 2026-10-10,
    where the true pose led with composite gaps of 0.21-0.25 and composites
    of 0.75-0.81; they are provisional, not a statistical calibration.
    """

    tolerance_m: float = 0.10
    min_band_points: int = 2000
    min_range_m: float = 0.5
    min_composite_gap: float = 0.15
    min_leader_composite: float = 0.70
    min_points: int = 20000
    bands: tuple = DEFAULT_BANDS
    # Points above the mapped structure cannot fit any candidate; they only
    # dilute the composite.  A planar pose keeps a point's height, so this
    # drops the same points for every candidate (see map_top_m).
    max_height_m: float = math.inf

    def __post_init__(self):
        for name in ('tolerance_m', 'min_range_m', 'min_composite_gap',
                     'min_leader_composite'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0):
                raise ContractError(f'{name} must be positive')
        if math.isnan(self.max_height_m) or self.max_height_m <= 0:
            raise ContractError('max_height_m must be positive')
        if type(self.min_band_points) is not int or self.min_band_points < 1:
            raise ContractError('min_band_points must be a positive integer')
        if type(self.min_points) is not int or self.min_points < 1:
            raise ContractError('min_points must be a positive integer')


@dataclass(frozen=True)
class SurfaceResult:
    """Per-candidate composites and the provisional nomination."""

    resolved: bool
    reason: str
    leader: int
    composite: tuple
    composite_gap: float
    band_points: tuple
    bands_used: tuple
    points: int


def quaternion_matrix(x, y, z, w):
    """Rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform(translation, quaternion):
    """4x4 homogeneous transform."""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(*quaternion)
    matrix[:3, 3] = translation
    return matrix


def planar(x, y, yaw):
    """4x4 transform of a planar pose."""
    matrix = np.eye(4)
    matrix[:2, :2] = [[math.cos(yaw), -math.sin(yaw)],
                      [math.sin(yaw), math.cos(yaw)]]
    matrix[:2, 3] = (x, y)
    return matrix


def load_ply_vertices(path):
    """Vertices (N x 3) of an ASCII PLY mesh."""
    with open(path, encoding='ascii', errors='strict') as stream:
        if stream.readline().strip() != 'ply':
            raise ValueError('not a PLY file')
        count, properties, in_vertex = None, [], False
        for line in stream:
            words = line.split()
            if words[:2] == ['format', 'ascii'] or not words:
                continue
            if words[0] == 'format':
                raise ValueError('only ASCII PLY is supported')
            if words[0] == 'element':
                in_vertex = words[1] == 'vertex'
                if in_vertex:
                    count = int(words[2])
            elif words[0] == 'property' and in_vertex:
                properties.append(words[-1])
            elif words[0] == 'end_header':
                break
        if count is None or properties[:3] != ['x', 'y', 'z']:
            raise ValueError('PLY vertices must start with x y z')
        vertices = np.loadtxt(stream, max_rows=count, usecols=(0, 1, 2), ndmin=2)
    if len(vertices) != count:
        raise ValueError('truncated PLY vertex list')
    return vertices


_FLOAT32 = 7


def pointcloud2_xyz(message, max_points=None):
    """
    Finite x/y/z of a sensor_msgs/PointCloud2 as an (N x 3) float array.

    Reads the buffer directly using the field offsets (float32 only), so no
    ROS helper is needed; ``max_points`` keeps an even subsample.
    """
    offsets = {f.name: f.offset for f in message.fields}
    types = {f.name: f.datatype for f in message.fields}
    if any(name not in offsets or types[name] != _FLOAT32 for name in 'xyz'):
        raise ContractError('point cloud needs float32 x, y and z fields')
    if message.is_bigendian:
        raise ContractError('big-endian point clouds are not supported')
    count = message.width * message.height
    if count == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if raw.size < count * message.point_step:
        raise ContractError('truncated point cloud')
    records = raw[:count * message.point_step].reshape(count, message.point_step)
    xyz = np.stack([records[:, offsets[n]:offsets[n] + 4].copy().view('<f4')[:, 0]
                    for n in 'xyz'], 1).astype(float)
    xyz = xyz[np.isfinite(xyz).all(1)]
    if max_points is not None and len(xyz) > max_points:
        xyz = xyz[np.linspace(0, len(xyz) - 1, max_points).astype(int)]
    return xyz


# Scored height above the map top: the top is a quantile, and returns just
# above it can still meet the surface within the inlier tolerance.
MAP_TOP_MARGIN_M = 0.05


def map_top_m(surface_points, share=0.99):
    """
    Height below which ``share`` of the map surface lies.

    The mapper stops at a height limit (carbot_map_20260928_215841: 99% of
    the surface below 2.31 m) while the LiDAR also sees the wall tops and
    ceiling above it; those returns have no surface to fit.  The quantile
    ignores a few stray high samples.
    """
    return float(np.quantile(np.asarray(surface_points, float)[:, 2], share))


def resolve_max_height(setting, surface_points):
    """
    Return ``max_height_m`` for a height-limit setting.

    < 0: the map top plus MAP_TOP_MARGIN_M; 0: no limit; > 0: that height.
    """
    setting = float(setting)
    if math.isnan(setting):
        raise ContractError('the height limit must be a number')
    if setting < 0:
        return map_top_m(surface_points) + MAP_TOP_MARGIN_M
    return math.inf if setting == 0 else setting


def scored_points(points, config):
    """Points the check scores: beyond ``min_range_m``, below ``max_height_m``."""
    points = np.asarray(points, float).reshape(-1, 3)
    keep = np.hypot(points[:, 0], points[:, 1]) >= config.min_range_m
    if math.isfinite(config.max_height_m):
        keep &= points[:, 2] <= config.max_height_m
    return points[keep]


def outside_variable(tree, world):
    """Points not in a variable region of ``tree`` (all, if it has none)."""
    regions = getattr(tree, 'regions', None)
    return np.ones(len(world), bool) if regions is None else ~regions.contains(world)


def band_inliers(base_points, pose, tree, bands, tolerance):
    """
    [(points, inliers)] per band for ``base_points`` placed at ``pose``.

    Points that land in a variable region of ``tree`` are not counted.
    """
    world = (planar(*pose) @ np.c_[base_points, np.ones(len(base_points))].T).T[:, :3]
    distance, _ = tree.query(world, distance_upper_bound=tolerance * 2.0)
    scored = outside_variable(tree, world)
    near = (distance <= tolerance) & scored
    heights = base_points[:, 2]
    return [(int((mask & scored).sum()), int((near & mask).sum()))
            for mask in ((heights >= lo) & (heights < hi) for lo, hi in bands)]


def compare(per_candidate, min_points):
    """
    Composite and margins from per-band counts.

    ``per_candidate`` is a list of per-band ``(points, inliers)`` lists over
    the same points.  Bands with fewer than ``min_points`` points are
    excluded for every candidate.
    """
    counts = np.array([[points for points, _ in bands] for bands in per_candidate])
    hits = np.array([[inliers for _, inliers in bands] for bands in per_candidate],
                    dtype=float)
    used = counts[0] >= min_points
    shares = np.divide(hits, counts, out=np.zeros_like(hits), where=counts > 0)
    # Each candidate weights the bands by its own scored points; they differ
    # only when variable regions drop points at some poses.
    weights = counts * used
    composite = (shares * weights).sum(1) / np.maximum(weights.sum(1), 1)
    order = list(np.argsort(-composite, kind='stable'))
    best = order[0]
    runner = order[1] if len(order) > 1 else None
    margins = (shares[best] - shares[runner]) if runner is not None else None
    return {
        'bands_used': used.tolist(),
        'shares': shares.round(4).tolist(),
        'composite': composite.round(4).tolist(),
        'order': [int(i) for i in order],
        'composite_gap': (None if runner is None else
                          round(float(composite[best] - composite[runner]), 4)),
        'band_margins': (None if margins is None else
                         [round(float(m), 4) if u else None
                          for m, u in zip(margins, used)]),
    }


class MaskedSurface:
    """
    A surface KD-tree without its variable regions, which it remembers.

    ``additions`` (``localization_surface_additions``) join the fit surface
    but not ``solid``, the points the see-through check treats as solid.
    """

    def __init__(self, points, regions, additions=None):
        from scipy.spatial import cKDTree
        points = np.asarray(points, float).reshape(-1, 3)
        extra = (np.zeros((0, 3)) if additions is None
                 else np.asarray(additions, float).reshape(-1, 3))
        self.regions = regions
        if regions is None:
            self.solid = points
            self.tree = cKDTree(np.vstack([points, extra]))
        else:
            self.solid = points[~regions.contains(points)]
            self.tree = cKDTree(np.vstack([self.solid, extra[~regions.contains(extra)]]))
            # The whole surface, to check that the regions do not change the
            # leader (localization_surface_validation).
            self.full = cKDTree(np.vstack([points, extra]))
            self.full_solid = points
        self.data = self.tree.data

    def query(self, points, **kwargs):
        return self.tree.query(points, **kwargs)


def solid_points(tree):
    """Return the surface points the see-through check may treat as solid."""
    return getattr(tree, 'solid', tree.data)


def surface_tree(surface, regions=None, additions=None):
    """
    KD-tree of the map surface points; a prebuilt tree is returned as is.

    ``surface`` is either the (N, 3) surface points (mesh vertices or the
    dense samples of ``localization_surface_model``) or a ``cKDTree`` of
    them, so one decision builds the tree once.  With ``regions``
    (``localization_variable_regions``) the surface inside them is dropped
    and scoring ignores observed points that land there.  ``additions`` are
    fitted but never solid (``localization_surface_additions``).
    """
    if hasattr(surface, 'query'):
        return surface
    if regions is not None or additions is not None:
        return MaskedSurface(surface, regions, additions)
    from scipy.spatial import cKDTree
    return cKDTree(np.asarray(surface, float))


def _map(function, items, workers):
    """``map`` on a thread pool; KD-tree queries release the GIL."""
    if workers <= 1 or len(items) < 2:
        return [function(item) for item in items]
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(min(workers, len(items))) as pool:
        return list(pool.map(function, items))


def check_surfaces(vertices, base_points, poses, config=SurfaceCheckConfig(), workers=1):
    """
    Score candidate poses (at the reference keyframe) against the map surface.

    ``vertices`` are the surface points or their tree (see
    :func:`surface_tree`).  ``base_points`` are stationary cloud points in
    the reference base frame; points nearer than ``min_range_m`` or above
    ``max_height_m`` are not scored.  Returns
    a :class:`SurfaceResult`.
    """
    poses = [tuple(float(v) for v in pose) for pose in poses]
    if len(poses) < 2:
        raise ContractError('the 3D re-check needs at least two candidates')
    points = scored_points(base_points, config)
    if len(points) < config.min_points:
        return SurfaceResult(False, 'TOO_FEW_POINTS', -1, (), 0.0, (), (), len(points))
    tree = surface_tree(vertices)
    per_candidate = _map(
        lambda pose: band_inliers(points, pose, tree, config.bands, config.tolerance_m),
        poses, workers)
    result = compare(per_candidate, config.min_band_points)
    if not any(result['bands_used']):
        return SurfaceResult(False, 'NO_USABLE_BAND', -1, tuple(result['composite']),
                             0.0, tuple(n for n, _ in per_candidate[0]),
                             tuple(result['bands_used']), len(points))
    leader = result['order'][0]
    gap = result['composite_gap']
    composite = result['composite'][leader]
    reason = ('' if gap >= config.min_composite_gap
              and composite >= config.min_leader_composite
              else 'LEADER_GAP_TOO_SMALL' if gap < config.min_composite_gap
              else 'LEADER_FIT_TOO_LOW')
    return SurfaceResult(not reason, reason, leader, tuple(result['composite']), gap,
                         tuple(n for n, _ in per_candidate[0]),
                         tuple(result['bands_used']), len(points))


def run_surface_check_job(vertices, base_points, poses, config):
    """Worker entry point for :func:`check_surfaces`."""
    return check_surfaces(vertices, base_points, poses, config)
