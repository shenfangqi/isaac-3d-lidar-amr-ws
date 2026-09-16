import importlib.util
from pathlib import Path

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = DESCRIPTION_ROOT.parents[1]
MID360_PATH = WORKSPACE / "isaac_sim" / "carbot_mid360.py"
PARAMETER_PATH = DESCRIPTION_ROOT / "config" / "carbot_parameters.yaml"

spec = importlib.util.spec_from_file_location("carbot_mid360", MID360_PATH)
carbot_mid360 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(carbot_mid360)


def test_runtime_config_comes_from_canonical_parameters():
    parameters = yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))
    config = carbot_mid360.mid360_runtime_config(parameters)

    assert config == {
        "profile_name": "Livox_Mid360_Approx",
        "pointcloud_topic": "/livox/lidar",
        "pointcloud_rate_hz": 10,
        "frame_skip_count": 4,
        "frame_id": "front_3d_lidar",
        "origin_from_housing_bottom_m": (0.0, 0.0, 0.0),
    }


class _Path:
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return self.value


class _Prim:
    def __init__(self, name, path, active=True, valid=True):
        self._name = name
        self._path = _Path(path)
        self._active = active
        self._valid = valid

    def GetName(self):
        return self._name

    def GetPath(self):
        return self._path

    def IsActive(self):
        return self._active

    def IsValid(self):
        return self._valid


class _Stage:
    def __init__(self, prims):
        self._prims = prims

    def Traverse(self):
        return iter(self._prims)

    def GetPrimAtPath(self, path):
        return next(
            (prim for prim in self._prims if str(prim.GetPath()) == str(path)),
            _Prim("", str(path), valid=False),
        )


def test_lidar_link_is_selected_only_below_configured_articulation():
    root = _Prim("base_footprint", "/World/Carbot/base_footprint")
    expected = _Prim(
        "lidar_link", "/World/Carbot/base_footprint/imported/path/lidar_link"
    )
    duplicate = _Prim("lidar_link", "/Carbot/base_link/lidar_link")
    stage = _Stage([root, duplicate, expected])
    assert (
        carbot_mid360.find_unique_link_prim(
            stage, "/World/Carbot/base_footprint", "lidar_link"
        )
        is expected
    )


def test_duplicate_active_carbot_root_is_rejected():
    stage = _Stage(
        [
            _Prim("Carbot", "/World/Carbot"),
            _Prim("base_footprint", "/World/Carbot/base_footprint"),
            _Prim("Carbot", "/Carbot"),
        ]
    )
    try:
        carbot_mid360.validate_carbot_hierarchy(
            stage, "/World/Carbot/base_footprint"
        )
    except RuntimeError as error:
        assert "/Carbot" in str(error)
    else:
        raise AssertionError("duplicate active Carbot root was accepted")
