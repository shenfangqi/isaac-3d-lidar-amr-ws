"""
Issue #13 stage A: the 3D map surface used by localization scoring.

Pure logic (numpy), no ROS.  The map mesh (nvblox PLY) is turned into dense
points on its triangles, so that a scan point lying on a mapped surface is
close to some surface point wherever it lies on that surface.  Scoring
against the mesh vertices alone does not have that property: on the
2026-09-28 map a point on a triangle is 1.9 cm from the nearest vertex at
the median and 3.3 cm at p99, and that error changes as a scan slides along
a wall, which invents position constraints the surface does not have.

Every triangle is subdivided until no sub-triangle edge exceeds
``spacing_m``; the sub-triangle corners are the samples.  Any point on the
surface is then within ``spacing_m / sqrt(3)`` of a sample (the largest
point-to-nearest-corner distance in a triangle is at most its longest edge
over sqrt(3)).

The samples are map-derived data: they are rebuilt from the mesh content,
the algorithm version and the spacing, and a cache is keyed by all three so
a rebuilt or different map never reuses old samples.
"""

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import tempfile

import numpy as np

ALGORITHM_VERSION = 1
DEFAULT_SPACING_M = 0.01

_PLY_TYPES = {
    'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1',
    'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
    'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4',
    'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8',
}


@dataclass(frozen=True)
class SurfaceModel:
    """Dense surface samples of a map mesh and how they were made."""

    points: np.ndarray          # (N, 3) float32, map frame
    mesh_sha256: str
    spacing_m: float
    version: int
    report: dict


def _read_header(stream):
    if stream.readline().strip() != b'ply':
        raise ValueError('not a PLY file')
    fmt, elements = None, []
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('PLY header has no end_header')
        words = line.decode('ascii', 'strict').split()
        if not words or words[0] in ('comment', 'obj_info'):
            continue
        if words[0] == 'format':
            fmt = words[1]
        elif words[0] == 'element':
            elements.append([words[1], int(words[2]), []])
        elif words[0] == 'property':
            if not elements:
                raise ValueError('PLY property before any element')
            elements[-1][2].append(tuple(words[1:]))
        elif words[0] == 'end_header':
            break
    if fmt not in ('ascii', 'binary_little_endian'):
        raise ValueError(f'unsupported PLY format {fmt!r}')
    return fmt, elements


def load_ply_mesh(path):
    """
    ``(vertices (N, 3), faces (M, 3))`` of an ASCII or little-endian PLY.

    Polygons with more than three corners are fanned into triangles; faces
    referring to missing vertices are refused.
    """
    with open(path, 'rb') as stream:
        fmt, elements = _read_header(stream)
        vertices, faces = None, np.zeros((0, 3), np.int64)
        for name, count, properties in elements:
            if fmt == 'ascii':
                rows = _read_ascii_element(stream, count, properties)
            else:
                rows = _read_binary_element(stream, count, properties)
            if name == 'vertex':
                names = [p[-1] for p in properties]
                if names[:3] != ['x', 'y', 'z']:
                    raise ValueError('PLY vertices must start with x y z')
                vertices = np.asarray([row[:3] for row in rows], float).reshape(-1, 3)
            elif name == 'face':
                faces = _triangulate(rows)
    if vertices is None:
        raise ValueError('PLY has no vertex element')
    if len(faces) and (faces.min() < 0 or faces.max() >= len(vertices)):
        raise ValueError('PLY face refers to a missing vertex')
    return vertices, faces


def _read_ascii_element(stream, count, properties):
    rows = []
    for _ in range(count):
        line = stream.readline()
        if not line:
            raise ValueError('truncated PLY')
        words = line.split()
        row, index = [], 0
        for prop in properties:
            if prop[0] == 'list':
                length = int(words[index])
                row.append([int(w) for w in words[index + 1:index + 1 + length]])
                index += 1 + length
            else:
                row.append(float(words[index]))
                index += 1
        rows.append(row)
    return rows


def _read_binary_element(stream, count, properties):
    if not any(prop[0] == 'list' for prop in properties):
        dtype = np.dtype([(prop[-1], '<' + _PLY_TYPES[prop[0]]) for prop in properties])
        data = np.frombuffer(stream.read(dtype.itemsize * count), dtype, count)
        if len(data) != count:
            raise ValueError('truncated PLY')
        return [list(row) for row in data.tolist()]
    rows = []
    for _ in range(count):
        row = []
        for prop in properties:
            if prop[0] == 'list':
                size = np.dtype('<' + _PLY_TYPES[prop[1]])
                item = np.dtype('<' + _PLY_TYPES[prop[2]])
                length = int(np.frombuffer(stream.read(size.itemsize), size)[0])
                row.append(np.frombuffer(stream.read(item.itemsize * length), item).tolist())
            else:
                kind = np.dtype('<' + _PLY_TYPES[prop[0]])
                row.append(float(np.frombuffer(stream.read(kind.itemsize), kind)[0]))
        rows.append(row)
    return rows


def _triangulate(rows):
    triangles = []
    for row in rows:
        corners = row[0]
        if len(corners) < 3:
            continue
        for k in range(1, len(corners) - 1):
            triangles.append((corners[0], corners[k], corners[k + 1]))
    return np.asarray(triangles, np.int64).reshape(-1, 3)


def sample_surface(vertices, faces, spacing_m=DEFAULT_SPACING_M):
    """
    Corners of every triangle subdivided to edges <= ``spacing_m``.

    Returns ``(points (N, 3) float32, report)``.  Degenerate triangles
    contribute their corners only.  Every vertex is kept, including those
    of no face.
    """
    if not (math.isfinite(spacing_m) and spacing_m > 0):
        raise ValueError('spacing_m must be positive')
    vertices = np.asarray(vertices, float).reshape(-1, 3)
    faces = np.asarray(faces, np.int64).reshape(-1, 3)
    if not np.isfinite(vertices).all():
        raise ValueError('mesh vertices must be finite')
    a, b, c = (vertices[faces[:, k]] for k in range(3))
    longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                      np.linalg.norm(a - c, axis=1)], axis=0)
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    divisions = np.maximum(1, np.ceil(longest / spacing_m - 1e-9)).astype(np.int64)
    parts = [vertices]
    for n in np.unique(divisions):
        if n == 1:
            continue
        chosen = divisions == n
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing='ij')
        keep = (i + j) <= n
        u, v = i[keep] / n, j[keep] / n
        # Interior and edge points; corners are already in ``vertices``.
        inner = ~(((u == 0) & (v == 0)) | ((u == 1) & (v == 0)) | ((u == 0) & (v == 1)))
        u, v = u[inner], v[inner]
        ta, tb, tc = a[chosen], b[chosen], c[chosen]
        points = (ta[:, None, :] + u[None, :, None] * (tb - ta)[:, None, :]
                  + v[None, :, None] * (tc - ta)[:, None, :])
        parts.append(points.reshape(-1, 3))
    points = np.concatenate(parts)
    # Shared edges are sampled by both triangles: drop exact repeats
    # (rounded to 0.1 mm, far below the spacing).
    keys = np.round(points / 1e-4).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    points = points[np.sort(first)].astype(np.float32)
    report = {
        'vertices': int(len(vertices)), 'faces': int(len(faces)),
        'degenerate_faces': int((area < 1e-10).sum()),
        'area_m2': round(float(area.sum()), 3),
        'longest_edge_m': round(float(longest.max()), 4) if len(faces) else 0.0,
        'samples': int(len(points)),
        'max_surface_to_sample_m': round(spacing_m / math.sqrt(3), 4),
        'bounds_min': np.round(vertices.min(0), 3).tolist() if len(vertices) else [],
        'bounds_max': np.round(vertices.max(0), 3).tolist() if len(vertices) else [],
    }
    return points, report


def file_sha256(path):
    """SHA-256 of a file, streamed."""
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def cache_path(mesh_path, mesh_sha256, spacing_m, cache_dir=None):
    """Cache file name bound to mesh content, algorithm version and spacing."""
    directory = Path(cache_dir) if cache_dir else Path(mesh_path).parent
    name = (f'{Path(mesh_path).stem}.surface-{mesh_sha256[:16]}'
            f'-v{ALGORITHM_VERSION}-{round(spacing_m * 1000)}mm.npz')
    return directory / name


def load_surface_model(mesh_path, spacing_m=DEFAULT_SPACING_M, cache_dir=None):
    """
    Surface samples of ``mesh_path``, from a matching cache when present.

    A cache is used only if its stored mesh hash, version and spacing match;
    otherwise the samples are rebuilt and the cache rewritten (best effort:
    an unwritable directory only costs the rebuild next time).
    """
    digest = file_sha256(mesh_path)
    path = cache_path(mesh_path, digest, spacing_m, cache_dir)
    try:
        with np.load(path, allow_pickle=False) as cached:
            if (str(cached['mesh_sha256']) == digest
                    and int(cached['version']) == ALGORITHM_VERSION
                    and float(cached['spacing_m']) == float(spacing_m)):
                report = {k[len('report_'):]: cached[k].tolist()
                          for k in cached.files if k.startswith('report_')}
                report['cache'] = 'hit'
                return SurfaceModel(cached['points'], digest, float(spacing_m),
                                    ALGORITHM_VERSION, report)
    except (OSError, ValueError, KeyError):
        pass
    vertices, faces = load_ply_mesh(mesh_path)
    points, report = sample_surface(vertices, faces, spacing_m)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=path.parent, suffix='.npz',
                                         delete=False) as handle:
            np.savez(handle, points=points, mesh_sha256=digest,
                     version=ALGORITHM_VERSION, spacing_m=float(spacing_m),
                     **{f'report_{k}': np.asarray(v) for k, v in report.items()})
        os.replace(handle.name, path)
        report['cache'] = 'written'
    except OSError:
        report['cache'] = 'unwritable'
    return SurfaceModel(points, digest, float(spacing_m), ALGORITHM_VERSION, report)
