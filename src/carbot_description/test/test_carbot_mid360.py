import importlib.util
import json
import math
from pathlib import Path

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = DESCRIPTION_ROOT.parents[1]
MID360_PATH = WORKSPACE / "isaac_sim" / "carbot_mid360.py"
PARAMETER_PATH = DESCRIPTION_ROOT / "config" / "carbot_parameters.yaml"
RTX_PROFILE_PATH = (
    WORKSPACE / "isaac_sim" / "lidar_configs" / "Livox_Mid360_Approx.json"
)
HARDWARE_ROOT = WORKSPACE / "src" / "carbot_hardware"
DRIVER_CONFIG_PATH = HARDWARE_ROOT / "config" / "MID360_config.json"
DRIVER_LAUNCH_PATH = HARDWARE_ROOT / "launch" / "mid360_stack.launch.py"
NVBLOX_CONFIG_ROOT = (
    WORKSPACE / "src" / "isaac_3d_lidar_bringup" / "config" / "nvblox"
)

spec = importlib.util.spec_from_file_location("carbot_mid360", MID360_PATH)
carbot_mid360 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(carbot_mid360)


def test_runtime_config_comes_from_canonical_parameters():
    parameters = yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))
    config = carbot_mid360.mid360_runtime_config(parameters)

    assert config == {
        "profile_name": "Livox_Mid360_Approx",
        "pointcloud_topic": "/livox/lidar",
        "pointcloud_type": "sensor_msgs/msg/PointCloud2",
        "pointcloud_rate_hz": 10,
        "frame_skip_count": 4,
        "full_scan": True,
        "frame_id": "front_3d_lidar",
        "origin_from_housing_bottom_m": (0.0, 0.0, 0.047),
    }


def _parameters():
    return yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))


def _json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_canonical_manufacturer_spec_is_complete_and_traceable():
    specs = _parameters()["sensors"]["mid360"]["manufacturer_specs"]

    assert specs["product_source"].startswith("https://www.livoxtech.com/")
    assert "Livox-SDK/livox_wiki_en" in specs["protocol_source"]
    assert specs["model"] == "MID-360"
    assert specs["minimum_detection_range_m"] == 0.1
    assert specs["detection_range_m_at_10_percent_reflectivity"] == 40.0
    assert specs["detection_range_m_at_80_percent_reflectivity"] == 70.0
    assert specs["horizontal_fov_deg"] == 360.0
    assert specs["vertical_fov_deg"] == [-7.0, 52.0]
    assert specs[
        "range_precision_1sigma_m_at_10m_80_percent_reflectivity"
    ] == 0.02
    assert specs[
        "range_precision_1sigma_m_at_0p2m_80_percent_reflectivity"
    ] == 0.03
    assert specs["angular_precision_1sigma_upper_bound_deg"] == 0.15
    assert specs["first_return_point_rate_hz"] == 200000
    assert specs["typical_frame_rate_hz"] == 10
    assert specs["equivalent_lines"] == 40
    assert specs["maximum_returns"] == 1
    assert specs["wavelength_nm"] == 905


def test_rtx_profile_is_consistent_and_does_not_mislabel_assumptions():
    specs = _parameters()["sensors"]["mid360"]["manufacturer_specs"]
    profile_document = _json(RTX_PROFILE_PATH)
    profile = profile_document["profile"]

    assert profile["nearRangeM"] == specs["minimum_detection_range_m"]
    assert profile["farRangeM"] == specs[
        "detection_range_m_at_10_percent_reflectivity"
    ]
    assert profile["wavelengthNm"] == specs["wavelength_nm"]
    assert profile["scanRateBaseHz"] == specs["typical_frame_rate_hz"]
    assert profile["numberOfEmitters"] == specs["equivalent_lines"]
    assert profile["maxReturns"] == specs["maximum_returns"]
    elevations = profile["emitterStates"][0]["elevationDeg"]
    assert [min(elevations), max(elevations)] == specs["vertical_fov_deg"]
    assert (
        profile["reportRateBaseHz"] * profile["numberOfEmitters"]
        == specs["first_return_point_rate_hz"]
    )
    assert profile["rangeAccuracyM"] == specs[
        "range_precision_1sigma_m_at_0p2m_80_percent_reflectivity"
    ]
    angular_limit = specs["angular_precision_1sigma_upper_bound_deg"]
    assert 0.0 < profile["azimuthErrorStd"] < angular_limit
    assert 0.0 < profile["elevationErrorStd"] < angular_limit
    review = profile_document["_error_model_review"]
    assert "simulation assumption" in review["rangeResolutionM"]
    assert "simulation assumption" in review["azimuthErrorStd"]
    assert "simulation assumption" in review["elevationErrorStd"]


def test_nvblox_lidar_limits_are_compatible_with_manufacturer_spec():
    specs = _parameters()["sensors"]["mid360"]["manufacturer_specs"]
    for name in ("mid360_nvblox_sim.yaml", "mid360_nvblox_real.yaml"):
        document = yaml.safe_load(
            (NVBLOX_CONFIG_ROOT / name).read_text(encoding="utf-8")
        )["/**"]["ros__parameters"]
        assert document["lidar_min_valid_range_m"] >= specs[
            "minimum_detection_range_m"
        ]
        assert document["static_mapper"][
            "lidar_projective_integrator_max_integration_distance_m"
        ] <= specs["detection_range_m_at_10_percent_reflectivity"]

    real = yaml.safe_load(
        (NVBLOX_CONFIG_ROOT / "mid360_nvblox_real.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    assert real["min_angle_below_zero_elevation_rad"] >= math.radians(
        abs(specs["vertical_fov_deg"][0])
    )
    assert real["max_angle_above_zero_elevation_rad"] >= math.radians(
        specs["vertical_fov_deg"][1]
    )


def test_repository_owned_driver_config_and_fast_lio_contract():
    lidar = _parameters()["sensors"]["mid360"]
    config = _json(DRIVER_CONFIG_PATH)
    assert config["lidar_summary_info"]["lidar_type"] == 8
    assert config["MID360"]["host_net_info"] == {
        "cmd_data_ip": "192.168.2.100",
        "cmd_data_port": 56101,
        "push_msg_ip": "192.168.2.100",
        "push_msg_port": 56201,
        "point_data_ip": "192.168.2.100",
        "point_data_port": 56301,
        "imu_data_ip": "192.168.2.100",
        "imu_data_port": 56401,
        "log_data_ip": "",
        "log_data_port": 56501,
    }
    sensor = config["lidar_configs"]
    assert len(sensor) == 1
    assert sensor[0]["ip"] == "192.168.2.202"
    assert sensor[0]["pcl_data_type"] == 1
    assert sensor[0]["pattern_mode"] == 0
    assert set(sensor[0]["extrinsic_parameter"].values()) == {0, 0.0}
    assert lidar["full_pointcloud_fields"] == [
        "x", "y", "z", "intensity", "tag", "line", "timestamp"
    ]
    assert lidar["compact_pointcloud_topic"] == "/mid360/points_xyz"
    assert lidar["compact_pointcloud_fields"] == ["x", "y", "z"]

    launch_source = DRIVER_LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'FindPackageShare("carbot_hardware")' in launch_source
    assert '"xfer_format": 1' in launch_source
    assert 'executable="pointcloud_xyz_relay"' not in launch_source


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
