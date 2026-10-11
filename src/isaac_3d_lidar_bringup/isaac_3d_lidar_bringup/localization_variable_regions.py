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
