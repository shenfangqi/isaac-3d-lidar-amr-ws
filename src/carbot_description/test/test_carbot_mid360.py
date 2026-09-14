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
        "frame_id": "front_3d_lidar",
        "origin_from_housing_bottom_m": (0.0, 0.0, 0.0),
    }


class _Path:
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return self.value


class _Prim:
    def __init__(self, name, path):
        self._name = name
        self._path = _Path(path)

    def GetName(self):
        return self._name

    def GetPath(self):
        return self._path


class _Stage:
    def __init__(self, prims):
        self._prims = prims

    def Traverse(self):
        return iter(self._prims)


def test_unique_lidar_link_is_selected_without_hard_coded_import_path():
    expected = _Prim("lidar_link", "/Carbot/imported/path/lidar_link")
    stage = _Stage([_Prim("base_link", "/Carbot/base_link"), expected])
    assert carbot_mid360.find_unique_link_prim(stage, "lidar_link") is expected
