"""Stage A: dense surface samples of the map mesh."""

import math
import struct

import numpy as np
import pytest
from scipy.spatial import cKDTree

from isaac_3d_lidar_bringup import localization_surface_model as model
from isaac_3d_lidar_bringup.localization_surface_check import planar


HEADER = ('ply\nformat {fmt} 1.0\nelement vertex {nv}\nproperty float x\n'
          'property float y\nproperty float z\nproperty float nx\n'
          'element face {nf}\nproperty list uchar int vertex_indices\nend_header\n')


def _ascii(path, vertices, faces):
    lines = [' '.join(map(str, v)) + ' 0' for v in vertices]
    lines += [' '.join(map(str, [len(f), *f])) for f in faces]
    path.write_text(HEADER.format(fmt='ascii', nv=len(vertices), nf=len(faces))
                    + '\n'.join(lines) + '\n')
    return path


def _binary(path, vertices, faces):
    body = b''.join(struct.pack('<4f', *v, 0.0) for v in vertices)
    body += b''.join(struct.pack('<B', len(f)) + struct.pack(f'<{len(f)}i', *f)
                     for f in faces)
    path.write_bytes(HEADER.format(fmt='binary_little_endian', nv=len(vertices),
                                   nf=len(faces)).encode() + body)
    return path


SQUARE = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]


def test_ascii_and_binary_meshes_read_alike_and_polygons_are_fanned(tmp_path):
    a = model.load_ply_mesh(_ascii(tmp_path / 'a.ply', SQUARE, [(0, 1, 2, 3)]))
    b = model.load_ply_mesh(_binary(tmp_path / 'b.ply', SQUARE, [(0, 1, 2, 3)]))
    for vertices, faces in (a, b):
        assert vertices.tolist() == [list(map(float, v)) for v in SQUARE]
        assert faces.tolist() == [[0, 1, 2], [0, 2, 3]]


def test_bad_meshes_are_refused(tmp_path):
    with pytest.raises(ValueError, match='missing vertex'):
        model.load_ply_mesh(_ascii(tmp_path / 'a.ply', SQUARE, [(0, 1, 9)]))
    big = tmp_path / 'b.ply'
    big.write_text('ply\nformat binary_big_endian 1.0\nend_header\n')
    with pytest.raises(ValueError, match='unsupported'):
        model.load_ply_mesh(big)


def test_every_surface_point_is_within_the_bound_of_a_sample():
    rng = np.random.default_rng(3)
    vertices = rng.uniform(-1, 1, (30, 3))
    faces = rng.integers(0, 30, (40, 3))
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    points, report = model.sample_surface(vertices, faces, 0.05)
    a, b, c = (vertices[faces[:, k]] for k in range(3))
    u, v = rng.random((2, 5000, len(faces)))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    surface = (a + u[..., None] * (b - a) + v[..., None] * (c - a)).reshape(-1, 3)
    distance = cKDTree(points).query(surface)[0]
    assert distance.max() <= 0.05 / math.sqrt(3) + 1e-6
    assert report['max_surface_to_sample_m'] == round(0.05 / math.sqrt(3), 4)
    # Every vertex is kept and shared edges are not sampled twice.
    assert cKDTree(points).query(vertices)[0].max() < 1e-6
    assert len(np.unique(np.round(points / 1e-4), axis=0)) == len(points)


def test_cache_is_bound_to_mesh_content_version_and_spacing(tmp_path):
    mesh = _ascii(tmp_path / 'm.ply', SQUARE, [(0, 1, 2, 3)])
    first = model.load_surface_model(mesh, 0.1)
    assert first.report['cache'] == 'written'
    again = model.load_surface_model(mesh, 0.1)
    assert again.report['cache'] == 'hit'
    assert np.array_equal(first.points, again.points)
    assert model.load_surface_model(mesh, 0.2).report['cache'] == 'written'
    # Rebuilt map: new content, new cache entry; the old samples are not used.
    _ascii(mesh, [(0, 0, 0), (2, 0, 0), (2, 2, 0), (0, 2, 0)], [(0, 1, 2, 3)])
    rebuilt = model.load_surface_model(mesh, 0.1)
    assert rebuilt.report['cache'] == 'written'
    assert rebuilt.mesh_sha256 != first.mesh_sha256
    assert rebuilt.points.max() == pytest.approx(2.0)


def test_a_corrupt_cache_is_rebuilt(tmp_path):
    mesh = _ascii(tmp_path / 'm.ply', SQUARE, [(0, 1, 2, 3)])
    built = model.load_surface_model(mesh, 0.1)
    model.cache_path(mesh, built.mesh_sha256, 0.1).write_bytes(b'not a cache')
    assert model.load_surface_model(mesh, 0.1).report['cache'] == 'written'


def test_sliding_along_a_coarse_wall_is_not_a_position_constraint():
    # A wall mesh with 0.5 m triangles.  Scored against its vertices, a scan
    # of the wall fits only where its points happen to sit near a vertex, so
    # the fit changes as the scan slides along the wall: a constraint the
    # surface does not have.  Against the surface samples it does not.
    xs, zs = np.arange(0, 6.01, .5), np.arange(0, 2.01, .5)
    vertices = np.array([(x, 2.0, z) for x in xs for z in zs])
    index = {(i, k): i * len(zs) + k for i in range(len(xs)) for k in range(len(zs))}
    faces = [f for i in range(len(xs) - 1) for k in range(len(zs) - 1)
             for f in ((index[i, k], index[i + 1, k], index[i + 1, k + 1]),
                       (index[i, k], index[i + 1, k + 1], index[i, k + 1]))]
    # Regular rows and columns, like the rings and azimuth steps of a lidar.
    sx, sz = np.meshgrid(np.arange(1.5, 4.51, .25), np.arange(.5, 1.51, .25))
    scan = np.c_[sx.ravel(), np.full(sx.size, 2.0), sz.ravel()]

    def fits(points):
        tree = cKDTree(points)
        result = []
        for shift in np.arange(0, .5, .05):
            world = (planar(shift, 0, 0) @ np.c_[scan, np.ones(len(scan))].T).T[:, :3]
            result.append(float((tree.query(world)[0] <= .05).mean()))
        return np.array(result)
    on_vertices = fits(vertices)
    on_surface = fits(model.sample_surface(vertices, faces, .02)[0])
    assert on_vertices.max() - on_vertices.min() > .3      # spurious constraint
    assert on_surface.min() == 1.0                         # flat along the wall
