#!/usr/bin/env python3
"""Import the expanded Carbot URDF and compose a Carter-free warehouse USD."""

import argparse
import hashlib
import os
import shutil
import sys
import tempfile
import traceback
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_URDF = WORKSPACE / ".codex_tmp/carbot_build/carbot.urdf"
DEFAULT_XACRO = WORKSPACE / "src/carbot_description/urdf/carbot.urdf.xacro"
DEFAULT_PARAMETERS = (
    WORKSPACE / "src/carbot_description/config/carbot_parameters.yaml"
)
DEFAULT_OUTPUT = WORKSPACE / "isaac_sim/usd/carbot.usd"
DEFAULT_WAREHOUSE = WORKSPACE / "isaac_sim/usd/warehouse_3d_nav.usd"
DEFAULT_SCENE_OUTPUT = (
    WORKSPACE / "isaac_sim/usd/warehouse_3d_nav_origin_carbot.usd"
)
CARTER_ORIGIN_Y_M = 0.9844150670532934
LEGACY_LIDAR_GRAPH_PATH = "/World/ROS2_LidarRTX"


def is_legacy_carter_path(path):
    lowered = str(path).lower()
    return (
        "carter" in lowered
        or "nova_" in lowered
        or str(path) == LEGACY_LIDAR_GRAPH_PATH
    )


def deactivate_legacy_carter_roots(stage):
    paths = [prim.GetPath() for prim in stage.TraverseAll()]
    root_paths = [
        path
        for path in paths
        if is_legacy_carter_path(path)
        and not is_legacy_carter_path(path.GetParentPath())
    ]
    for path in root_paths:
        stage.GetPrimAtPath(path).SetActive(False)
    return [str(path) for path in root_paths]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--xacro", type=Path, default=DEFAULT_XACRO)
    parser.add_argument("--parameters", type=Path, default=DEFAULT_PARAMETERS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--warehouse", type=Path, default=DEFAULT_WAREHOUSE)
    parser.add_argument("--scene-output", type=Path, default=DEFAULT_SCENE_OUTPUT)
    return parser.parse_args()


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_inputs(paths):
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("Missing model input(s): " + ", ".join(missing))


args = parse_args()
require_inputs([args.urdf, args.xacro, args.parameters, args.warehouse])
args.output.parent.mkdir(parents=True, exist_ok=True)
args.scene_output.parent.mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402
import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402


def update(count=10):
    for _ in range(count):
        simulation_app.update()


def import_robot():
    enable_extension("isaacsim.asset.importer.urdf")
    update(20)

    with tempfile.TemporaryDirectory(
        prefix=".carbot_usd_", dir=args.output.parent
    ) as temporary_directory:
        temporary_output = Path(temporary_directory) / args.output.name
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("Isaac Sim failed to create a URDF import config")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.collision_from_visuals = False
        import_config.create_physics_scene = False
        import_config.make_default_prim = True

        status, imported_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(args.urdf),
            import_config=import_config,
            dest_path=str(temporary_output),
        )
        if not status:
            raise RuntimeError("Isaac Sim URDF import failed")
        update(20)

        stage = Usd.Stage.Open(str(temporary_output))
        if stage is None:
            raise RuntimeError(f"Cannot reopen generated USD: {temporary_output}")
        robot = stage.GetDefaultPrim()
        if not robot.IsValid():
            raise RuntimeError("Generated Carbot USD has no default robot prim")
        articulation_roots = [
            prim
            for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        if not articulation_roots:
            UsdPhysics.ArticulationRootAPI.Apply(robot)
            articulation_roots = [robot]
        if len(articulation_roots) != 1:
            paths = [str(prim.GetPath()) for prim in articulation_roots]
            raise RuntimeError(f"Expected one articulation root, found {paths}")

        prims = list(stage.Traverse())
        revolute_joints = [
            prim for prim in prims if prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        collision_prims = [
            prim for prim in prims if prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if len(revolute_joints) != 12:
            raise RuntimeError(
                f"Expected 12 wheel joints, found {len(revolute_joints)}"
            )
        if len(collision_prims) != 13:
            raise RuntimeError(
                f"Expected 13 collision bodies, found {len(collision_prims)}"
            )
        prim_paths = [str(prim.GetPath()) for prim in prims]
        for required_name in (
            "base_link",
            "lidar_link",
            "livox_frame",
            "front_3d_lidar",
            "imu_link",
        ):
            if not any(required_name in path for path in prim_paths):
                raise RuntimeError(f"Generated USD is missing {required_name}")
        if any(
            token in path.lower()
            for path in prim_paths
            for token in ("ackermann", "steer")
        ):
            raise RuntimeError("Generated USD unexpectedly contains steering geometry")

        layer = stage.GetRootLayer()
        metadata = dict(layer.customLayerData)
        metadata.update(
            {
                "carbot:generator": "isaac_sim/scripts/build_carbot_usd.py",
                "carbot:parameterSha256": sha256(args.parameters),
                "carbot:xacroSha256": sha256(args.xacro),
                "carbot:sourceUrdf": str(args.urdf),
                "carbot:temporaryDynamics": "TEMP_ESTIMATE_NOT_CALIBRATED",
            }
        )
        layer.customLayerData = metadata
        layer.Save()

        generated_configuration = Path(temporary_directory) / "configuration"
        output_configuration = args.output.parent / "configuration"
        output_configuration.mkdir(exist_ok=True)
        for generated_file in generated_configuration.iterdir():
            shutil.copy2(generated_file, output_configuration / generated_file.name)
        os.replace(temporary_output, args.output)
    print(
        f"Generated Carbot articulation: {args.output} ({imported_path}); "
        f"root={articulation_roots[0].GetPath()}, "
        "12 wheel joints, 13 collision bodies",
        flush=True,
    )


def compose_warehouse():
    temporary_scene = args.scene_output.with_suffix(".building.usd")
    if temporary_scene.exists():
        temporary_scene.unlink()

    stage = Usd.Stage.CreateNew(str(temporary_scene))
    stage.GetRootLayer().subLayerPaths = [
        os.path.relpath(args.warehouse, args.scene_output.parent)
    ]
    carbot = UsdGeom.Xform.Define(stage, Sdf.Path("/Carbot"))
    carbot.GetPrim().GetReferences().AddReference(
        os.path.relpath(args.output, args.scene_output.parent)
    )
    carbot.GetPrim().GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(0.0, CARTER_ORIGIN_Y_M, 0.0)
    )
    deactivated_legacy_roots = deactivate_legacy_carter_roots(stage)

    world = stage.GetPrimAtPath("/World")
    if world.IsValid():
        stage.SetDefaultPrim(world)
    layer = stage.GetRootLayer()
    metadata = dict(layer.customLayerData)
    metadata.update(
        {
            "carbot:baseWarehouse": str(args.warehouse),
            "carbot:localRobotAsset": str(args.output),
            "carbot:replaces": "Nova Carter",
            "carbot:deactivatedLegacyPrims": ",".join(
                deactivated_legacy_roots
            ),
        }
    )
    layer.customLayerData = metadata
    layer.Save()
    os.replace(temporary_scene, args.scene_output)

    composed = Usd.Stage.Open(str(args.scene_output))
    if composed is None or not composed.GetPrimAtPath("/Carbot").IsValid():
        raise RuntimeError("Generated warehouse does not contain /Carbot")
    active_legacy_prims = [
        str(prim.GetPath())
        for prim in composed.Traverse()
        if is_legacy_carter_path(prim.GetPath())
    ]
    if active_legacy_prims:
        raise RuntimeError(
            f"Generated warehouse still contains active Carter prims: {active_legacy_prims}"
        )
    print(
        f"Generated Carter-free warehouse: {args.scene_output}; "
        f"disabled={deactivated_legacy_roots}",
        flush=True,
    )


try:
    import_robot()
    compose_warehouse()
except BaseException:
    traceback.print_exc()
    sys.stderr.flush()
    raise
finally:
    simulation_app.close()
