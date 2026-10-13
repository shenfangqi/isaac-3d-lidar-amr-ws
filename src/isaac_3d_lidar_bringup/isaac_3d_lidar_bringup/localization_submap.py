"""
Local submap: several LiDAR frames in one frame, compared with the map.

The shared base of issue #54 (map updates: where does the scene differ from
the map, pose known) and issue #53 (relocalization while moving: where does
a submap fit the map best, pose unknown).  Pure logic, no ROS.

``build_submap`` puts deskewed clouds (FAST-LIO ``cloud_registered_body`` is
already corrected to the scan end) in the base frame of one reference
odometry pose, voxelises them and drops transient voxels: hit in too few
frames, or passed through by the rays of other frames as often as they are
hit (a person walking by).  Voxels near the floor or ceiling are never
dropped by rays, which graze them on the way to distant floor or ceiling.

``compare_submap`` places the submap at a map pose and classifies it:

- added: kept voxels farther than ``added_m`` from the map surface (a new
  object, a closed curtain or door, structure the mesh missed);
- see-through: rays that cross mapped structure before their last
  ``end_margin_m`` (a wrong pose, or structure that is gone);
- removed: mapped voxels crossed by rays of at least ``min_removed_frames``
  frames with no submap voxel within ``hit_m`` (an opened door, an object
  taken away, a mapping artefact the sensor sees through).

Each kind is grouped into connected components so that a decision can be
made per object.  The pose is an input: a wrong pose shows up as change
everywhere, so callers must establish it independently first.
"""

from dataclasses import dataclass
import math

import numpy as np

from .localization_contracts import ContractError
from .localization_surface_check import planar, solid_points, surface_tree
from .localization_surface_validation import _voxel_keys


@dataclass(frozen=True)
class SubmapConfig:
    """
    Submap building limits.

    ``min_range_m`` is horizontal, in each frame's own base frame (the robot
    body, as in the stationary captures).  Carving (the transient filter)
    uses every ``carve_every``-th frame and at most ``carve_rays`` rays of it.
    """

    voxel_m: float = .05
    min_range_m: float = .5
    max_range_m: float = 12.
    max_points_per_frame: int = 6000
    min_frames: int = 3
    max_free_share: float = .5
    carve_every: int = 4
    carve_rays: int = 1500
    carve_voxel_m: float = .1
    carve_start_m: float = .3
    carve_end_margin_m: float = .3
    carve_min_z_m: float = .1
    carve_max_z_m: float = 2.0

    def __post_init__(self):
        values = (self.voxel_m, self.min_range_m, self.max_range_m, self.max_free_share,
                  self.carve_start_m, self.carve_end_margin_m, self.carve_voxel_m)
        counts = (self.max_points_per_frame, self.min_frames, self.carve_every,
                  self.carve_rays)
        if (any(not math.isfinite(v) or v <= 0 for v in values)
                or any(type(v) is not int or v < 1 for v in counts)
                or self.min_range_m >= self.max_range_m or self.max_free_share > 1
                or not self.carve_min_z_m < self.carve_max_z_m):
            raise ContractError('invalid submap limits')


@dataclass(frozen=True)
class SubmapFrame:
    """One deskewed cloud: points in the sensor frame and where it was."""

    stamp_ns: int
    odom_T_base: np.ndarray        # 4x4, odometry pose at the cloud stamp
    base_T_sensor: np.ndarray      # 4x4, static
    points: np.ndarray             # (n, 3), sensor frame


@dataclass(frozen=True)
class Submap:
    """
    Voxelised frames in the reference base frame.

    ``points`` are the kept voxel centroids, ``frames`` how many frames hit
    each and ``free_share`` the share of carving frames that passed through
    it rather than hit it.  ``dropped`` are the centroids of transient
    voxels.  ``ray_origins``/``ray_ends``/``ray_frames`` are the carving
    rays (all of them, a ray to a passer-by still proves the space before it
    is free).
    """

    points: np.ndarray
    frames: np.ndarray
    free_share: np.ndarray
    dropped: np.ndarray
    origins: np.ndarray
    ray_origins: np.ndarray
    ray_ends: np.ndarray
    ray_frames: np.ndarray
    frame_count: int


def _subsample(points, count):
    if len(points) > count:
        points = points[np.linspace(0, len(points) - 1, count).astype(int)]
    return points


def _homogeneous(points):
    return np.c_[points, np.ones(len(points))]


def _ray_samples(origins, ends, start_m, end_margin_m, step_m):
    """Points along each ray from ``start_m`` to ``end_margin_m`` short of its end."""
    delta = ends - origins
    length = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(length, 1e-9)[:, None]
    end = length - end_margin_m
    if not len(end) or float(end.max()) <= start_m:
        return np.zeros((len(ends), 0, 3)), np.zeros((len(ends), 0), bool)
    steps = np.arange(start_m, float(end.max()), step_m)
    samples = origins[:, None, :] + direction[:, None, :] * steps[None, :, None]
    return samples, steps[None, :] < end[:, None]


def _member(sorted_keys, keys):
    """Index into ``sorted_keys`` of each key, -1 where absent."""
    if not len(sorted_keys):
        return np.full(keys.shape, -1)
    slot = np.minimum(np.searchsorted(sorted_keys, keys), len(sorted_keys) - 1)
    return np.where(sorted_keys[slot] == keys, slot, -1)


def build_submap(frames, odom_T_reference, config=SubmapConfig()):
    """
    Return the :class:`Submap` of ``frames`` in the base frame at ``odom_T_reference``.

    Frames without points after the range limits are skipped; at least one
    must remain.
    """
    reference_T_odom = np.linalg.inv(np.asarray(odom_T_reference, float))
    clouds, origins = [], []
    for frame in frames:
        points = np.asarray(frame.points, float).reshape(-1, 3)
        points = points[np.isfinite(points).all(1)]
        points = points[np.linalg.norm(points, axis=1) <= config.max_range_m]
        base = (np.asarray(frame.base_T_sensor, float) @ _homogeneous(points).T).T
        points = points[np.hypot(base[:, 0], base[:, 1]) >= config.min_range_m]
        if not len(points):
            continue
        points = _subsample(points, config.max_points_per_frame)
        matrix = reference_T_odom @ np.asarray(frame.odom_T_base, float) @ frame.base_T_sensor
        clouds.append((matrix @ _homogeneous(points).T).T[:, :3])
        origins.append(matrix[:3, 3])
    if not clouds:
        raise ContractError('no frame has points within the range limits')

    # Voxel hits, counted once per frame.
    keys = [_voxel_keys(cloud, config.voxel_m) for cloud in clouds]
    all_keys = np.concatenate(keys)
    voxels, inverse = np.unique(all_keys, return_inverse=True)
    points = np.vstack(clouds)
    count = np.bincount(inverse, minlength=len(voxels))
    centroids = np.stack([np.bincount(inverse, points[:, i], len(voxels))
                          for i in range(3)], 1) / count[:, None]
    hits = np.zeros(len(voxels), int)
    for frame_keys in keys:
        hits[_member(voxels, np.unique(frame_keys))] += 1

    # Carving: how often rays of other frames pass through each voxel, on a
    # coarser grid so that a few thousand rays per frame cover it.
    coarse = np.unique(_voxel_keys(centroids, config.carve_voxel_m), return_inverse=True)
    cells, cell_of = coarse
    carve_hits = np.zeros(len(cells), int)
    carve_free = np.zeros(len(cells), int)
    ray_origins, ray_ends, ray_frames = [], [], []
    for index in range(0, len(clouds), config.carve_every):
        ends = _subsample(clouds[index], config.carve_rays)
        origin = np.broadcast_to(origins[index], ends.shape)
        ray_origins.append(origin)
        ray_ends.append(ends)
        ray_frames.append(np.full(len(ends), index))
        hit_cells = np.unique(_voxel_keys(clouds[index], config.carve_voxel_m))
        carve_hits[_member(cells, hit_cells)[_member(cells, hit_cells) >= 0]] += 1
        samples, inside = _ray_samples(origin, ends, config.carve_start_m,
                                       config.carve_end_margin_m, config.carve_voxel_m / 2)
        inside &= ((samples[..., 2] >= config.carve_min_z_m)
                   & (samples[..., 2] <= config.carve_max_z_m))
        slot = _member(cells, _voxel_keys(samples[inside], config.carve_voxel_m))
        crossed = np.setdiff1d(np.unique(slot[slot >= 0]), _member(cells, hit_cells))
        carve_free[crossed] += 1
    carve_hits, carve_free = carve_hits[cell_of], carve_free[cell_of]
    seen = carve_hits + carve_free
    free_share = np.where(seen > 0, carve_free / np.maximum(seen, 1), 0.)
    keep = (hits >= config.min_frames) & (free_share < config.max_free_share)
    return Submap(
        points=centroids[keep], frames=hits[keep], free_share=free_share[keep],
        dropped=centroids[~keep], origins=np.asarray(origins),
        ray_origins=np.vstack(ray_origins), ray_ends=np.vstack(ray_ends),
        ray_frames=np.concatenate(ray_frames), frame_count=len(clouds))


@dataclass(frozen=True)
class CompareConfig:
    """
    Map comparison limits.

    Only mapped structure between ``min_z_m`` and ``max_z_m`` (map frame)
    can be crossed, so rays to distant floor and ceiling do not count, and
    only submap voxels in that band can be added.
    ``min_points`` is the smallest component reported per kind.
    """

    added_m: float = .15
    hit_m: float = .10
    voxel_m: float = .05
    start_m: float = .3
    end_margin_m: float = .3
    max_range_m: float = 8.
    max_rays: int = 40000
    min_z_m: float = .1
    max_z_m: float = 2.0
    min_removed_frames: int = 2
    cluster_m: float = .15
    min_added_points: int = 20
    min_removed_points: int = 5

    def __post_init__(self):
        values = (self.added_m, self.hit_m, self.voxel_m, self.start_m, self.end_margin_m,
                  self.max_range_m, self.cluster_m)
        counts = (self.max_rays, self.min_removed_frames, self.min_added_points,
                  self.min_removed_points)
        if (any(not math.isfinite(v) or v <= 0 for v in values)
                or any(type(v) is not int or v < 1 for v in counts)
                or not self.min_z_m < self.max_z_m):
            raise ContractError('invalid submap comparison limits')


@dataclass(frozen=True)
class Change:
    """One connected component of added or removed structure, map frame."""

    kind: str
    points: int
    low: tuple
    high: tuple
    centroid: tuple


@dataclass(frozen=True)
class SubmapComparison:
    """
    The submap at one map pose.

    ``distance`` is each submap voxel's distance to the map surface (capped
    at 1 m), ``added`` its mask.  ``see_through`` is the share of tested
    rays that cross mapped structure, ``crossings`` the first crossing of
    each such ray and ``removed`` the mapped voxels classed as removed (all
    map frame).
    """

    world: np.ndarray
    distance: np.ndarray
    added: np.ndarray
    see_through: float
    crossings: np.ndarray
    removed: np.ndarray
    changes: tuple


def components(points, cluster_m):
    """Labels of ``points`` connected within ``cluster_m``."""
    points = np.asarray(points, float).reshape(-1, 3)
    if len(points) < 2:
        return np.zeros(len(points), int)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import cKDTree
    pairs = cKDTree(points).query_pairs(cluster_m, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(points), len(points)))
    return connected_components(graph, directed=False)[1]


def _changes(kind, points, cluster_m, min_points):
    labels = components(points, cluster_m)
    out = []
    for label in np.unique(labels):
        part = points[labels == label]
        if len(part) < min_points:
            continue
        out.append(Change(kind, int(len(part)),
                          tuple(round(float(v), 3) for v in part.min(0)),
                          tuple(round(float(v), 3) for v in part.max(0)),
                          tuple(round(float(v), 3) for v in part.mean(0))))
    return sorted(out, key=lambda change: -change.points)


def compare_submap(submap, pose, surface, config=CompareConfig()):
    """
    Return the :class:`SubmapComparison` of ``submap`` at map ``pose``.

    ``surface`` is the map surface (points or a tree from ``surface_tree``,
    possibly with variable regions and surface additions).  Additions count
    for ``added`` but not for crossings, as in the see-through check.
    """
    tree = surface_tree(surface)
    matrix = planar(*pose)
    world = (matrix @ _homogeneous(submap.points).T).T[:, :3]
    distance = (tree.query(world, distance_upper_bound=1.)[0] if len(world)
                else np.zeros(0))
    distance = np.minimum(distance, 1.)
    # Outside the height band the map is not expected to be complete (floor
    # holes, an unmapped ceiling), so nothing there counts as added.
    added = ((distance > config.added_m) & (world[:, 2] >= config.min_z_m)
             & (world[:, 2] <= config.max_z_m))

    solid = np.asarray(solid_points(tree), float)
    solid = solid[(solid[:, 2] >= config.min_z_m) & (solid[:, 2] <= config.max_z_m)]
    occupancy = np.unique(_voxel_keys(solid, config.voxel_m))
    rays = np.linspace(0, len(submap.ray_ends) - 1,
                       min(len(submap.ray_ends), config.max_rays)).astype(int)
    origins = (matrix @ _homogeneous(submap.ray_origins[rays]).T).T[:, :3]
    ends = (matrix @ _homogeneous(submap.ray_ends[rays]).T).T[:, :3]
    # Rays longer than max_range_m are tested up to it only.
    delta = ends - origins
    length = np.linalg.norm(delta, axis=1)
    ends = origins + delta * (np.minimum(length, config.max_range_m + config.end_margin_m)
                              / np.maximum(length, 1e-9))[:, None]
    tested, crossings, pairs = 0, [], []
    for chunk in range(0, len(rays), 2000):
        part = slice(chunk, chunk + 2000)
        samples, inside = _ray_samples(origins[part], ends[part], config.start_m,
                                       config.end_margin_m, config.voxel_m / 2)
        slot = _member(occupancy, _voxel_keys(samples, config.voxel_m))
        hit = (slot >= 0) & inside
        tested += int(inside.any(1).sum())
        crossed = hit.any(1)
        first = hit.argmax(1)
        crossings.append(samples[crossed, first[crossed]])
        rows, steps = np.nonzero(hit)
        pairs.append(np.c_[slot[rows, steps], submap.ray_frames[rays[part]][rows]])
    crossings = np.vstack(crossings) if crossings else np.zeros((0, 3))
    see_through = len(crossings) / tested if tested else float('nan')

    # Removed: mapped voxels crossed by rays of enough frames, and not hit.
    pairs = np.unique(np.vstack(pairs), axis=0) if pairs else np.zeros((0, 2), int)
    voxel, frame_count = (np.unique(pairs[:, 0], return_counts=True) if len(pairs)
                          else (np.zeros(0, int), np.zeros(0, int)))
    candidates = voxel[frame_count >= config.min_removed_frames]
    centres = (np.zeros((0, 3)) if not len(candidates)
               else _key_centres(occupancy[candidates], config.voxel_m))
    if len(centres) and len(world):
        from scipy.spatial import cKDTree
        near = cKDTree(world).query(centres, distance_upper_bound=config.hit_m)[0]
        centres = centres[~np.isfinite(near)]
    changes = (_changes('added', world[added], config.cluster_m, config.min_added_points)
               + _changes('removed', centres, config.cluster_m, config.min_removed_points))
    return SubmapComparison(world, distance, added, see_through, crossings, centres,
                            tuple(changes))


def _key_centres(keys, voxel):
    mask = (1 << 21) - 1
    cells = np.stack([(keys >> 42) & mask, (keys >> 21) & mask, keys & mask], 1)
    return (cells - (1 << 20) + .5) * voxel
