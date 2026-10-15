import math
import torch
from isaac_lab.carbot_env.sensor_geometry import lidar_pattern, tof_pattern, encode_distances
from isaac_lab.carbot_env.spec import load_robot_parameters, load_environment_spec


def test_lidar_origin_and_unique_azimuth_samples():
    robot = load_robot_parameters()
    spec = load_environment_spec()['observations']['lidar']
    starts, directions = lidar_pattern(robot, spec, 'cpu')
    assert directions.shape == (576, 3)
    assert torch.allclose(directions.norm(dim=-1), torch.ones(576), atol=1e-6)
    assert torch.unique(directions, dim=0).shape[0] == 576
    # Optical origin is at 209 mm above ground, not the housing bottom.
    assert math.isclose(starts[0, 2].item()+robot['geometry']['base_link_height_m'], .209, abs_tol=1e-7)


def test_tof_origin_and_fov_zone_order():
    robot = load_robot_parameters()
    spec = load_environment_spec()['observations']['tof']
    starts, directions = tof_pattern(robot, spec, 'cpu')
    assert starts.shape == directions.shape == (512, 3)
    assert torch.allclose(directions.norm(dim=-1), torch.ones(512), atol=1e-6)
    assert torch.unique(starts, dim=0).shape[0] == 8
    rays = directions.reshape(8, 8, 8, 3)
    assert (rays[:, 0, :, 2] > 0).all()  # top row sees upward
    assert (rays[:, -1, :, 2] < 0).all()
    # Forward-left sensor points outward 12.5 degrees, with symmetric zones.
    mean = rays[0].mean(dim=(0, 1))
    assert math.isclose(math.degrees(math.atan2(mean[1], mean[0])), 12.5, abs_tol=1e-5)


def test_invalid_values_cannot_masquerade_as_clear_space():
    raw = torch.tensor([float('nan'), float('inf'), -1., 0., .01, .02, 1., 4., 4.1])
    distances, valid = encode_distances(raw, .02, 4.)
    assert torch.isfinite(distances).all()
    assert valid.tolist() == [0,0,0,0,0,1,1,1,0]
    assert distances[valid == 0].eq(0).all()
    assert torch.equal(distances[valid == 1], raw[valid == 1])
