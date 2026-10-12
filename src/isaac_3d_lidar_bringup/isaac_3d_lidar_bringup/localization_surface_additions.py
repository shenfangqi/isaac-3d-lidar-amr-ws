"""
Static map structure the mesh is missing, added to the 3D fit (pure logic).

The MID-360 sits low and sees up to 52 deg, so mapping from a distance can
miss low horizontal structure such as a bed frame; a robot next to it then
sees surface the map lacks and the true pose loses fit.  Surface additions
are map-frame points measured from stationary captures at operator-confirmed
poses (scripts/build_surface_additions.py).

They count as surface for the fit, the ranking and the 3D search, but not as
solid for the see-through check: partial or slatted structure seen from one
side lets rays pass from another, and a see-through crossing would wrongly
exclude the true pose.  Every candidate is scored against the same surface.

Like variable regions, additions belong to one mesh: the file names its
SHA-256 and a different mesh refuses it.  The file is a NumPy ``.npz`` with
``points`` (n, 3), ``mesh_sha256`` and a free-text ``source``.
"""

import numpy as np

from .localization_contracts import ContractError


def load_surface_additions(path, mesh_sha256):
    """Points of ``path`` for the mesh with ``mesh_sha256``; refuse a mismatch."""
    with np.load(path, allow_pickle=False) as document:
        sha = str(document['mesh_sha256']) if 'mesh_sha256' in document else ''
        if sha != mesh_sha256:
            raise ContractError(f'surface additions are for mesh {sha[:12]}, '
                                f'not {mesh_sha256[:12]}')
        points = np.asarray(document['points'], float) if 'points' in document else None
    if points is None or points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ContractError('surface-additions file holds no (n, 3) points')
    if not np.isfinite(points).all():
        raise ContractError('surface additions contain non-finite points')
    return points


def save_surface_additions(path, points, mesh_sha256, source=''):
    """Write additions for one mesh (see the module docstring)."""
    np.savez_compressed(path, points=np.asarray(points, np.float32),
                        mesh_sha256=np.str_(mesh_sha256), source=np.str_(source))
