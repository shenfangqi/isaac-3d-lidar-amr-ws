"""
Map regions whose appearance changes in normal use (curtains, doors).

Pure logic, no ROS.  A variable region is an axis-aligned box in the map
frame.  Localization ignores it on both sides: mapped surface inside it is
dropped, and an observed point that lands inside it at a candidate pose is
not scored for that candidate.  Every candidate is treated by the same rule.

The regions belong to one map: the file names the SHA-256 of the mesh it
was made for, and a different mesh refuses it, so a rebuilt map never
inherits regions drawn for another.  Example (YAML)::

    mesh_sha256: 9eb7d035...
    regions:
      - name: bottom-wall curtains
        min: [-1.25, -1.10, 0.30]
        max: [1.30, -0.30, 2.00]
        reason: curtains open when mapped, closed in use (2026-10-10)
"""

from dataclasses import dataclass
import math

import numpy as np
import yaml

from .localization_contracts import ContractError


@dataclass(frozen=True)
class VariableRegions:
    """Axis-aligned map-frame boxes, ``(n, 6)``: min xyz then max xyz."""

    boxes: np.ndarray
    names: tuple = ()

    def contains(self, points):
        """Whether each map-frame point lies in any region."""
        points = np.asarray(points, float).reshape(-1, 3)
        inside = np.zeros(len(points), bool)
        for box in self.boxes:
            inside |= ((points >= box[:3]) & (points <= box[3:])).all(1)
        return inside


def load_variable_regions(path, mesh_sha256):
    """Regions of ``path`` for the mesh with ``mesh_sha256``; refuse a mismatch."""
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    if str(document.get('mesh_sha256', '')) != mesh_sha256:
        raise ContractError(
            f'variable regions are for mesh {str(document.get("mesh_sha256"))[:12]}, '
            f'not {mesh_sha256[:12]}')
    boxes, names = [], []
    for region in document.get('regions') or ():
        low = [float(v) for v in region['min']]
        high = [float(v) for v in region['max']]
        if (len(low) != 3 or len(high) != 3
                or not all(math.isfinite(v) for v in low + high)
                or any(a >= b for a, b in zip(low, high))):
            raise ContractError(f'invalid variable region {region.get("name")!r}')
        boxes.append(low + high)
        names.append(str(region.get('name', '')))
    if not boxes:
        raise ContractError('variable-region file lists no region')
    return VariableRegions(np.asarray(boxes, float), tuple(names))


def mask_grid(grid, regions, z_low, z_high):
    """
    Return a copy of ``grid`` with the cells of overlapping regions unknown.

    For 2D localization scoring only: a region whose height range overlaps
    the localization scan band [z_low, z_high] marks cells where that scan
    may see something other than the map, e.g. under a bed rail that the
    height-aware navigation map rightly blocks (2026-10-12: the bed's south
    edge made every bottom-left true pose look like it saw through a wall).
    Unknown cells are neither hits nor wall crossings.  The navigation map,
    motion safety and route planning keep the original grid.  The grid
    needs ``info`` (width, height, resolution, origin with an axis-aligned
    orientation) and a flat row-major ``data`` sequence.
    """
    if regions is None:
        return grid
    from array import array
    from types import SimpleNamespace
    info = grid.info
    width, height, resolution = int(info.width), int(info.height), float(info.resolution)
    ox, oy = float(info.origin.position.x), float(info.origin.position.y)
    data = np.frombuffer(array('b', grid.data).tobytes(), np.int8).reshape(height, width).copy()
    changed = 0
    for low_x, low_y, low_z, high_x, high_y, high_z in regions.boxes:
        if high_z < z_low or low_z > z_high:
            continue
        i0 = max(0, int(math.floor((low_x - ox) / resolution)))
        i1 = min(width, int(math.ceil((high_x - ox) / resolution)))
        j0 = max(0, int(math.floor((low_y - oy) / resolution)))
        j1 = min(height, int(math.ceil((high_y - oy) / resolution)))
        if i0 < i1 and j0 < j1:
            data[j0:j1, i0:i1] = -1
            changed += (i1 - i0) * (j1 - j0)
    if not changed:
        return grid
    return SimpleNamespace(frame_id=getattr(grid, 'frame_id', 'map'), info=info,
                           data=array('b', data.ravel().tobytes()))
