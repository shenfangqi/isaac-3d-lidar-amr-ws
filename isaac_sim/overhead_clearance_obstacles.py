"""Persistent Isaac Sim obstacles for overhead-clearance validation."""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class OverheadObstacle:
    name: str
    map_x_m: float
    map_y_m: float
    clearance_m: float
    size_x_m: float = 0.40
    size_y_m: float = 1.00
    thickness_m: float = 0.10
    color_rgb: tuple = (0.95, 0.65, 0.05)

    @property
    def center_z_m(self):
        return self.clearance_m + self.thickness_m / 2.0

    @property
    def prim_path(self):
        clearance_cm = round(self.clearance_m * 100.0)
        label = self.name.title()
        return f"/World/CarbotOverhead{label}Clearance{clearance_cm:02d}cm"


def _load_obstacles():
    config_path = (
        Path(__file__).resolve().parents[1] / "configs/carbot/common.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    obstacles = []
    for record in config["simulation_overhead_obstacles"]:
        values = dict(record)
        values["color_rgb"] = tuple(values["color_rgb"])
        obstacles.append(OverheadObstacle(**values))
    return tuple(obstacles)


OVERHEAD_OBSTACLES = _load_obstacles()


def create_overhead_clearance_obstacles(stage, robot_prim_path):
    """Create static collision beams at map coordinates for every startup."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Carbot prim is missing: {robot_prim_path}")
    robot_world = UsdGeom.Xformable(
        robot_prim
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    map_origin = robot_world.ExtractTranslation()

    created = []
    for obstacle in OVERHEAD_OBSTACLES:
        cube = UsdGeom.Cube.Define(stage, obstacle.prim_path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*obstacle.color_rgb)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        transform = UsdGeom.XformCommonAPI(cube.GetPrim())
        transform.SetTranslate(
            Gf.Vec3d(
                float(map_origin[0]) + obstacle.map_x_m,
                float(map_origin[1]) + obstacle.map_y_m,
                float(map_origin[2]) + obstacle.center_z_m,
            )
        )
        transform.SetScale(
            Gf.Vec3f(
                obstacle.size_x_m,
                obstacle.size_y_m,
                obstacle.thickness_m,
            )
        )
        created.append(obstacle)
    return created
