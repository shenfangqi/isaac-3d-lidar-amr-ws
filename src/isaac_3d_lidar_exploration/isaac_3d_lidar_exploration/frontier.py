from dataclasses import dataclass
import heapq
import math


UNKNOWN = -1


@dataclass(frozen=True)
class FrontierCandidate:
    row: int
    col: int
    x: float
    y: float
    cluster_cells: int
    clearance_m: float
    unknown_clearance_m: float
    robot_distance_m: float
    score: float


def _neighbors4(row, col, width, height):
    if row > 0:
        yield row - 1, col
    if row + 1 < height:
        yield row + 1, col
    if col > 0:
        yield row, col - 1
    if col + 1 < width:
        yield row, col + 1


def _neighbors8(row, col, width, height):
    for row_offset in (-1, 0, 1):
        for col_offset in (-1, 0, 1):
            if row_offset == 0 and col_offset == 0:
                continue
            neighbor_row = row + row_offset
            neighbor_col = col + col_offset
            if 0 <= neighbor_row < height and 0 <= neighbor_col < width:
                yield neighbor_row, neighbor_col


def find_frontier_clusters(data, width, height, free_threshold=20):
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('Occupancy grid dimensions do not match its data')

    frontier_indices = set()
    for row in range(height):
        for col in range(width):
            index = row * width + col
            value = data[index]
            if value < 0 or value > free_threshold:
                continue
            if any(
                data[neighbor_row * width + neighbor_col] == UNKNOWN
                for neighbor_row, neighbor_col in _neighbors4(
                    row, col, width, height
                )
            ):
                frontier_indices.add(index)

    clusters = []
    while frontier_indices:
        seed = frontier_indices.pop()
        cluster = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            row, col = divmod(current, width)
            for neighbor_row, neighbor_col in _neighbors8(
                row, col, width, height
            ):
                neighbor = neighbor_row * width + neighbor_col
                if neighbor in frontier_indices:
                    frontier_indices.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)
        clusters.append(cluster)
    return clusters


def _clearance_cells(data, width, height, source_predicate):
    """Return an 8-connected distance transform in grid-cell units."""
    distances = [math.inf] * (width * height)
    queue = []
    for index, value in enumerate(data):
        if source_predicate(value):
            distances[index] = 0.0
            heapq.heappush(queue, (0.0, index))

    while queue:
        distance, index = heapq.heappop(queue)
        if distance != distances[index]:
            continue
        row, col = divmod(index, width)
        for neighbor_row, neighbor_col in _neighbors8(
            row, col, width, height
        ):
            neighbor = neighbor_row * width + neighbor_col
            diagonal = neighbor_row != row and neighbor_col != col
            step_distance = math.sqrt(2.0) if diagonal else 1.0
            candidate_distance = distance + step_distance
            if candidate_distance < distances[neighbor]:
                distances[neighbor] = candidate_distance
                heapq.heappush(queue, (candidate_distance, neighbor))
    return distances


def _limited_distance_cells(width, height, source_indices, maximum_distance):
    distances = {index: 0.0 for index in source_indices}
    queue = [(0.0, index) for index in source_indices]
    heapq.heapify(queue)
    while queue:
        distance, index = heapq.heappop(queue)
        if distance != distances[index]:
            continue
        row, col = divmod(index, width)
        for neighbor_row, neighbor_col in _neighbors8(
            row, col, width, height
        ):
            neighbor = neighbor_row * width + neighbor_col
            diagonal = neighbor_row != row and neighbor_col != col
            step_distance = math.sqrt(2.0) if diagonal else 1.0
            candidate_distance = distance + step_distance
            if candidate_distance > maximum_distance:
                continue
            if candidate_distance < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate_distance
                heapq.heappush(queue, (candidate_distance, neighbor))
    return distances


def obstacle_clearance_cells(data, width, height, occupied_threshold=65):
    return _clearance_cells(
        data,
        width,
        height,
        lambda value: value >= occupied_threshold,
    )


def unknown_clearance_cells(data, width, height):
    return _clearance_cells(
        data,
        width,
        height,
        lambda value: value == UNKNOWN,
    )


def cell_to_world(row, col, resolution, origin_x, origin_y, origin_yaw=0.0):
    local_x = (col + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        origin_x + cosine * local_x - sine * local_y,
        origin_y + sine * local_x + cosine * local_y,
    )


def world_to_cell(x, y, resolution, origin_x, origin_y, origin_yaw=0.0):
    delta_x = x - origin_x
    delta_y = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return math.floor(local_y / resolution), math.floor(local_x / resolution)


def build_frontier_candidates(
    data,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    origin_yaw,
    robot_x,
    robot_y,
    free_threshold=20,
    occupied_threshold=65,
    min_frontier_length_m=0.30,
    goal_clearance_m=0.55,
    goal_unknown_clearance_m=0.40,
    goal_search_radius_m=1.00,
    boundary_margin_m=0.60,
    min_goal_distance_m=0.80,
    max_goal_distance_m=15.0,
    information_gain_weight=1.0,
    distance_weight=0.35,
):
    clusters = find_frontier_clusters(data, width, height, free_threshold)
    clearance_cells = obstacle_clearance_cells(
        data, width, height, occupied_threshold
    )
    unknown_cells = unknown_clearance_cells(data, width, height)
    minimum_cells = max(1, math.ceil(min_frontier_length_m / resolution))
    search_radius_cells = max(
        1, math.ceil(goal_search_radius_m / resolution)
    )
    candidates = []

    for cluster in clusters:
        if len(cluster) < minimum_cells:
            continue
        centroid_row = sum(index // width for index in cluster) / len(cluster)
        centroid_col = sum(index % width for index in cluster) / len(cluster)
        frontier_distances = _limited_distance_cells(
            width,
            height,
            cluster,
            search_radius_cells,
        )

        safe_cells = []
        for index, frontier_distance in frontier_distances.items():
            row, col = divmod(index, width)
            value = data[index]
            if value < 0 or value > free_threshold:
                continue
            boundary_cells = min(
                row,
                col,
                height - 1 - row,
                width - 1 - col,
            )
            clearance_m = clearance_cells[index] * resolution
            unknown_clearance_m = unknown_cells[index] * resolution
            if boundary_cells * resolution < boundary_margin_m:
                continue
            if clearance_m < goal_clearance_m:
                continue
            if unknown_clearance_m < goal_unknown_clearance_m:
                continue
            x, y = cell_to_world(
                row,
                col,
                resolution,
                origin_x,
                origin_y,
                origin_yaw,
            )
            robot_distance_m = math.hypot(x - robot_x, y - robot_y)
            if robot_distance_m < min_goal_distance_m:
                continue
            outside_goal_radius = (
                max_goal_distance_m > 0.0
                and robot_distance_m > max_goal_distance_m
            )
            if outside_goal_radius:
                continue
            safe_cells.append(
                (
                    index,
                    clearance_m,
                    unknown_clearance_m,
                    frontier_distance,
                    x,
                    y,
                    robot_distance_m,
                )
            )

        if not safe_cells:
            continue

        (
            index,
            clearance_m,
            unknown_clearance_m,
            _,
            x,
            y,
            robot_distance_m,
        ) = min(
            safe_cells,
            key=lambda item: (
                item[3]
                + 0.05 * (
                    (item[0] // width - centroid_row) ** 2
                    + (item[0] % width - centroid_col) ** 2
                ),
                (item[0] // width - centroid_row) ** 2
                + (item[0] % width - centroid_col) ** 2
                - 0.02 * (item[1] / resolution) ** 2,
            ),
        )
        row, col = divmod(index, width)

        frontier_length_m = len(cluster) * resolution
        score = (
            information_gain_weight * frontier_length_m
            - distance_weight * robot_distance_m
        )
        candidates.append(
            FrontierCandidate(
                row=row,
                col=col,
                x=x,
                y=y,
                cluster_cells=len(cluster),
                clearance_m=clearance_m,
                unknown_clearance_m=unknown_clearance_m,
                robot_distance_m=robot_distance_m,
                score=score,
            )
        )

    candidates.sort(key=lambda candidate: candidate.score, reverse=True)
    return clusters, candidates
