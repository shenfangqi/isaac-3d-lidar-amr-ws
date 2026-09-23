import yaml

from isaac_sim.carbot_mid360 import mid360_runtime_config


def test_mid360_ground_truth_topic_can_be_isolated():
    with open(
        "src/carbot_description/config/carbot_parameters.yaml",
        encoding="utf-8",
    ) as stream:
        parameters = yaml.safe_load(stream)
    config = mid360_runtime_config(
        parameters, "/livox/lidar_ground_truth"
    )
    assert config["pointcloud_topic"] == "/livox/lidar_ground_truth"
    assert parameters["sensors"]["mid360"]["pointcloud_topic"] == (
        "/livox/lidar"
    )
