import math

from isaac_3d_lidar_exploration.frontier import build_frontier_candidates
from isaac_3d_lidar_exploration.frontier import cell_to_world
from isaac_3d_lidar_exploration.frontier import find_frontier_clusters
from isaac_3d_lidar_exploration.frontier import obstacle_clearance_cells
from isaac_3d_lidar_exploration.frontier import unknown_clearance_cells
from isaac_3d_lidar_exploration.frontier import world_to_cell


def test_finds_single_frontier_ring():
    width = 7
    height = 7
    data = [-1] * (width * height)
    for row in range(2, 5):
        for col in range(2, 5):
            data[row * width + col] = 0

    clusters = find_frontier_clusters(data, width, height)

    assert len(clusters) == 1
    assert len(clusters[0]) == 8


def test_clearance_distance_uses_diagonals():
    width = 5
    height = 5
    data = [0] * (width * height)
    data[2 * width + 2] = 100

    distances = obstacle_clearance_cells(data, width, height)

    assert distances[2 * width + 2] == 0.0
    assert distances[2 * width + 3] == 1.0
    assert math.isclose(distances[3 * width + 3], math.sqrt(2.0))


def test_candidate_respects_obstacle_and_boundary_clearance():
    width = 15
    height = 15
    data = [-1] * (width * height)
    for row in range(3, 12):
        for col in range(3, 12):
            data[row * width + col] = 0
    data[7 * width + 10] = 100

    clusters, candidates = build_frontier_candidates(
        data=data,
        width=width,
        height=height,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        robot_x=0.75,
        robot_y=0.75,
        min_frontier_length_m=0.2,
        goal_clearance_m=0.2,
        boundary_margin_m=0.2,
        min_goal_distance_m=0.1,
        max_goal_distance_m=5.0,
    )

    assert clusters
    assert candidates
    assert all(candidate.clearance_m >= 0.2 for candidate in candidates)
    assert all(
        candidate.unknown_clearance_m >= 0.4
        for candidate in candidates
    )


def test_candidate_stands_back_from_unknown_space():
    width = 21
    height = 21
    data = [-1] * (width * height)
    for row in range(3, 18):
        for col in range(3, 18):
            data[row * width + col] = 0

    _, candidates = build_frontier_candidates(
        data=data,
        width=width,
        height=height,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        robot_x=1.05,
        robot_y=1.05,
        min_frontier_length_m=0.2,
        goal_clearance_m=0.2,
        goal_unknown_clearance_m=0.4,
        goal_search_radius_m=0.8,
        boundary_margin_m=0.2,
        min_goal_distance_m=0.1,
        max_goal_distance_m=5.0,
    )

    unknown_distances = unknown_clearance_cells(data, width, height)
    assert candidates
    for candidate in candidates:
        index = candidate.row * width + candidate.col
        assert data[index] == 0
        assert math.isclose(
            candidate.unknown_clearance_m,
            unknown_distances[index] * 0.1,
        )
        assert candidate.unknown_clearance_m >= 0.4


def test_rotated_map_coordinate_round_trip():
    x, y = cell_to_world(
        row=4,
        col=7,
        resolution=0.05,
        origin_x=-2.0,
        origin_y=3.0,
        origin_yaw=0.6,
    )

    row, col = world_to_cell(
        x=x,
        y=y,
        resolution=0.05,
        origin_x=-2.0,
        origin_y=3.0,
        origin_yaw=0.6,
    )

    assert (row, col) == (4, 7)
