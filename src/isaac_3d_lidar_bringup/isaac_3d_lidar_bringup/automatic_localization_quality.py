"""Pure quality metrics for automatic saved-map localization."""

import math
from types import SimpleNamespace


def angular_difference(first, second):
    """Return the signed shortest angular distance from second to first."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


def quaternion_yaw(quaternion):
    """Return planar yaw from a geometry quaternion-like object."""
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def planar_window_span(samples):
    """Return x, y, and unwrapped yaw peak-to-peak spans."""
    if not samples:
        return math.inf, math.inf, math.inf
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    reference = samples[0][2]
    yaws = [angular_difference(sample[2], reference) for sample in samples]
    return (
        max(xs) - min(xs),
        max(ys) - min(ys),
        max(yaws) - min(yaws),
    )


def trim_time_window(samples, now, window):
    """Keep the oldest sample bracketing a requested time window."""
    while len(samples) > 1 and now - samples[1][0] >= window:
        samples.popleft()
    return samples


def covariance_quality(pose, max_xy_std, max_yaw_std):
    """Return whether AMCL covariance is finite and below limits."""
    covariance = pose.pose.covariance
    values = (covariance[0], covariance[7], covariance[35])
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        return False
    return (
        math.sqrt(values[0]) <= max_xy_std
        and math.sqrt(values[1]) <= max_xy_std
        and math.sqrt(values[2]) <= max_yaw_std
    )


def particle_concentration(particles, center_pose, max_xy, max_yaw):
    """Return normalized particle weight near the selected AMCL pose."""
    if center_pose is None or not particles:
        return 0.0
    center_yaw = quaternion_yaw(center_pose.orientation)
    total_weight = 0.0
    close_weight = 0.0
    for particle in particles:
        weight = particle.weight
        if not math.isfinite(weight) or weight < 0.0:
            continue
        total_weight += weight
        pose = particle.pose
        distance = math.hypot(
            pose.position.x - center_pose.position.x,
            pose.position.y - center_pose.position.y,
        )
        yaw_error = abs(angular_difference(
            quaternion_yaw(pose.orientation), center_yaw
        ))
        if distance <= max_xy and yaw_error <= max_yaw:
            close_weight += weight
    if total_weight <= 0.0:
        return 0.0
    return close_weight / total_weight


def scan_map_metrics(grid, scan, transform, occupied_threshold,
                     tolerance_cells, max_beams):
    """Score scan endpoints and free rays against an OccupancyGrid."""
    metrics = dict(score=0.0, known=0, sampled=0, endpoint_hits=0,
                   wall_conflicts=0, unknown=0, outside=0, coverage=0.0,
                   wall_conflict_ratio=0.0, legacy_score=0.0)
    if grid is None or scan is None or transform is None:
        return metrics
    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return metrics

    origin = grid.info.origin
    origin_yaw = quaternion_yaw(origin.orientation)
    cos_origin = math.cos(-origin_yaw)
    sin_origin = math.sin(-origin_yaw)
    translation = transform.translation
    transform_yaw = quaternion_yaw(transform.rotation)
    cos_tf = math.cos(transform_yaw)
    sin_tf = math.sin(transform_yaw)

    valid_indices = [
        index for index, distance in enumerate(scan.ranges)
        if (
            math.isfinite(distance)
            and scan.range_min <= distance <= scan.range_max
        )
    ]
    beam_limit = max(1, int(max_beams))
    if len(valid_indices) > beam_limit:
        if beam_limit == 1:
            valid_indices = [valid_indices[len(valid_indices) // 2]]
        else:
            valid_indices = [
                valid_indices[round(
                    sample * (len(valid_indices) - 1) / (beam_limit - 1)
                )]
                for sample in range(beam_limit)
            ]
    matched = 0
    valid = 0
    sampled = 0
    endpoint_hits = 0
    conflicts = 0

    def grid_coordinates(map_x, map_y):
        shifted_x = map_x - origin.position.x
        shifted_y = map_y - origin.position.y
        return (
            int(math.floor(
                (cos_origin * shifted_x - sin_origin * shifted_y)
                / resolution
            )),
            int(math.floor(
                (sin_origin * shifted_x + cos_origin * shifted_y)
                / resolution
            )),
        )

    def ray_crosses_wall(start_x, start_y, end_x, end_y):
        """Return true when a measured free ray crosses a mapped wall."""
        delta_x = abs(end_x - start_x)
        delta_y = -abs(end_y - start_y)
        step_x = 1 if start_x < end_x else -1
        step_y = 1 if start_y < end_y else -1
        error = delta_x + delta_y
        cells = []
        x_cell = start_x
        y_cell = start_y
        while True:
            cells.append((x_cell, y_cell))
            if x_cell == end_x and y_cell == end_y:
                break
            doubled = 2 * error
            if doubled >= delta_y:
                error += delta_y
                x_cell += step_x
            if doubled <= delta_x:
                error += delta_x
                y_cell += step_y

        protected = max(1, tolerance_cells + 1)
        for cell_x, cell_y in cells[:-protected]:
            if not (0 <= cell_x < width and 0 <= cell_y < height):
                continue
            if grid.data[cell_y * width + cell_x] >= occupied_threshold:
                return True
        return False

    sensor_grid_x, sensor_grid_y = grid_coordinates(
        translation.x, translation.y
    )
    for index in valid_indices:
        distance = scan.ranges[index]
        sampled += 1
        angle = scan.angle_min + index * scan.angle_increment
        local_x = distance * math.cos(angle)
        local_y = distance * math.sin(angle)
        map_x = translation.x + cos_tf * local_x - sin_tf * local_y
        map_y = translation.y + sin_tf * local_x + cos_tf * local_y

        grid_x, grid_y = grid_coordinates(map_x, map_y)
        crosses_wall = ray_crosses_wall(
            sensor_grid_x, sensor_grid_y, grid_x, grid_y)
        conflicts += int(crosses_wall)
        if not (0 <= grid_x < width and 0 <= grid_y < height):
            metrics['outside'] += 1
            continue

        cell = grid.data[grid_y * width + grid_x]
        if cell < 0:
            metrics['unknown'] += 1
            continue
        valid += 1
        endpoint_matched = False
        for offset_y in range(-tolerance_cells, tolerance_cells + 1):
            candidate_y = grid_y + offset_y
            if candidate_y < 0 or candidate_y >= height:
                continue
            for offset_x in range(-tolerance_cells,
                                  tolerance_cells + 1):
                candidate_x = grid_x + offset_x
                if candidate_x < 0 or candidate_x >= width:
                    continue
                candidate = grid.data[candidate_y * width + candidate_x]
                if candidate >= occupied_threshold:
                    endpoint_matched = True
                    break
            if endpoint_matched:
                break
        endpoint_hits += int(endpoint_matched)
        if endpoint_matched and not crosses_wall:
            matched += 1
    metrics.update(
        score=matched / sampled if sampled else 0.0,
        known=valid, sampled=sampled, endpoint_hits=endpoint_hits,
        wall_conflicts=conflicts,
        coverage=valid / sampled if sampled else 0.0,
        wall_conflict_ratio=conflicts / sampled if sampled else 0.0,
        legacy_score=endpoint_hits / valid if valid else 0.0,
    )
    return metrics


def scan_map_score(grid, scan, transform, occupied_threshold,
                   tolerance_cells, max_beams):
    """Compatibility tuple; full diagnostics use scan_map_metrics."""
    metrics = scan_map_metrics(grid, scan, transform, occupied_threshold,
                               tolerance_cells, max_beams)
    return metrics['score'], metrics['known'], metrics['sampled']


def deterministic_global_search(
        grid, scans, occupied_threshold, tolerance_cells,
        position_step_m=0.20, yaw_step_rad=math.radians(10.0),
        coarse_beams=40, refine_beams=120, refine_count=120,
        fine_position_step_m=0.05, fine_yaw_step_rad=math.radians(2.0),
        fine_position_radius_m=0.20, fine_yaw_radius_rad=math.radians(10.0),
        fine_seed_count=1, final_position_step_m=0.01,
        final_yaw_step_rad=math.radians(0.5),
        final_position_radius_m=0.04,
        final_yaw_radius_rad=math.radians(2.0)):
    """
    Return ranked map-wide poses using endpoint and free-ray evidence.

    This deliberately uses the same interpretable metric as final validation,
    unlike AMCL's endpoint-only likelihood field.  Call it off the ROS executor
    because a complete map search is CPU intensive.
    """
    if grid is None or not scans:
        return []
    resolution = grid.info.resolution
    if resolution <= 0.0 or position_step_m <= 0.0 or yaw_step_rad <= 0.0:
        return []
    cell_step = max(1, round(position_step_m / resolution))
    yaw_count = max(1, math.ceil(2.0 * math.pi / yaw_step_rad))
    origin = grid.info.origin.position
    coarse_scan = scans[len(scans) // 2]
    candidates = []

    def planar_transform(x, y, yaw):
        return SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
        )

    for cell_y in range(0, grid.info.height, cell_step):
        for cell_x in range(0, grid.info.width, cell_step):
            if grid.data[cell_y * grid.info.width + cell_x] != 0:
                continue
            x = origin.x + (cell_x + 0.5) * resolution
            y = origin.y + (cell_y + 0.5) * resolution
            for yaw_index in range(yaw_count):
                yaw = -math.pi + yaw_index * 2.0 * math.pi / yaw_count
                metrics = scan_map_metrics(
                    grid, coarse_scan, planar_transform(x, y, yaw),
                    occupied_threshold, tolerance_cells, coarse_beams)
                candidates.append((metrics['score'], x, y, yaw))

    candidates.sort(reverse=True)
    refined = []
    for _coarse_score, x, y, yaw in candidates[:refine_count]:
        transform = planar_transform(x, y, yaw)
        metrics = [
            scan_map_metrics(
                grid, scan, transform, occupied_threshold,
                tolerance_cells, refine_beams)
            for scan in scans
        ]
        refined.append({
            'x': x,
            'y': y,
            'yaw': yaw,
            'score': sum(item['score'] for item in metrics) / len(metrics),
            'coverage': sum(item['coverage'] for item in metrics) / len(metrics),
            'wall_conflict_ratio': sum(
                item['wall_conflict_ratio'] for item in metrics
            ) / len(metrics),
            'known': min(item['known'] for item in metrics),
        })

    def ranking_key(item):
        return (
            item['score'], -item['wall_conflict_ratio'],
            item['coverage'], item['known'],
        )

    refined.sort(key=ranking_key, reverse=True)
    if not refined or fine_seed_count <= 0:
        return refined

    fine_poses = set()
    position_steps = round(fine_position_radius_m / fine_position_step_m)
    yaw_steps = round(fine_yaw_radius_rad / fine_yaw_step_rad)
    for seed in refined[:fine_seed_count]:
        for delta_x in range(-position_steps, position_steps + 1):
            for delta_y in range(-position_steps, position_steps + 1):
                x = seed['x'] + delta_x * fine_position_step_m
                y = seed['y'] + delta_y * fine_position_step_m
                for delta_yaw in range(-yaw_steps, yaw_steps + 1):
                    yaw = angular_difference(
                        seed['yaw'] + delta_yaw * fine_yaw_step_rad, 0.0)
                    fine_poses.add((round(x, 6), round(y, 6), round(yaw, 8)))

    fine = []
    for x, y, yaw in fine_poses:
        transform = planar_transform(x, y, yaw)
        metrics = [
            scan_map_metrics(
                grid, scan, transform, occupied_threshold,
                tolerance_cells, refine_beams)
            for scan in scans
        ]
        fine.append({
            'x': x,
            'y': y,
            'yaw': yaw,
            'score': sum(item['score'] for item in metrics) / len(metrics),
            'coverage': sum(item['coverage'] for item in metrics) / len(metrics),
            'wall_conflict_ratio': sum(
                item['wall_conflict_ratio'] for item in metrics
            ) / len(metrics),
            'known': min(item['known'] for item in metrics),
        })
    # Retain coarse/refined spatial alternatives so ambiguity checks still see
    # distant rooms even though only the global winner receives fine search.
    fine.sort(key=ranking_key, reverse=True)
    final = []
    if (
        fine and final_position_step_m > 0.0 and final_yaw_step_rad > 0.0
        and final_position_radius_m >= 0.0 and final_yaw_radius_rad >= 0.0
    ):
        seed = fine[0]
        position_steps = round(
            final_position_radius_m / final_position_step_m)
        yaw_steps = round(final_yaw_radius_rad / final_yaw_step_rad)
        for delta_x in range(-position_steps, position_steps + 1):
            for delta_y in range(-position_steps, position_steps + 1):
                x = seed['x'] + delta_x * final_position_step_m
                y = seed['y'] + delta_y * final_position_step_m
                for delta_yaw in range(-yaw_steps, yaw_steps + 1):
                    yaw = angular_difference(
                        seed['yaw'] + delta_yaw * final_yaw_step_rad, 0.0)
                    transform = planar_transform(x, y, yaw)
                    metrics = [
                        scan_map_metrics(
                            grid, scan, transform, occupied_threshold,
                            tolerance_cells, refine_beams)
                        for scan in scans
                    ]
                    final.append({
                        'x': x, 'y': y, 'yaw': yaw,
                        'score': sum(
                            item['score'] for item in metrics
                        ) / len(metrics),
                        'coverage': sum(
                            item['coverage'] for item in metrics
                        ) / len(metrics),
                        'wall_conflict_ratio': sum(
                            item['wall_conflict_ratio'] for item in metrics
                        ) / len(metrics),
                        'known': min(item['known'] for item in metrics),
                    })
        final.sort(key=ranking_key, reverse=True)

    combined = final + fine + refined
    combined.sort(key=ranking_key, reverse=True)
    return combined
