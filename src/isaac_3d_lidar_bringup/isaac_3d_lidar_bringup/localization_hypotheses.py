"""
Multi-view global pose hypotheses for Issue #13 automatic localization.

A hypothesis ``H`` is the map->base pose at the reference keyframe ``t0``.
Every other frame is placed with its own source-time odometry:

    T_map_scan(ti) = H * inv(T_odom_base(t0)) * T_odom_base(ti)
                     * T_base_scan(ti)

Scoring reuses ``scan_map_metrics``.  The search never accepts a
provisional leader: a canceled or budget-limited search is incomplete, and
an alternative cluster that was not refined but could still compete makes
the result incomplete as well.  Acceptance is decided only on HOLDOUT frames
that the search never saw.
"""

from array import array
from dataclasses import dataclass, field
import hashlib
import math
import multiprocessing
import struct
import time
from types import SimpleNamespace

from isaac_3d_lidar_bringup.automatic_localization_quality import (
    quaternion_yaw,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    scan_map_metrics,
)
from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError,
    FrameRole,
    Hypothesis,
    normalize_angle,
    RejectReason,
    SE2,
    SearchResult,
)


@dataclass(frozen=True)
class SearchConfig:
    """Search budget.  Cluster radii are search settings, not tolerances."""

    occupied_threshold: int = 65
    tolerance_cells: int = 3
    coarse_step_m: float = 0.20
    coarse_yaw_step_rad: float = math.radians(10.0)
    coarse_beams: int = 40
    refine_beams: int = 120
    coarse_keep: int = 600
    cluster_xy_m: float = 0.30
    cluster_yaw_rad: float = math.pi / 12.0
    max_refined_clusters: int = 8
    # Beyond max_refined_clusters, clusters are refined one at a time only
    # while an unrefined one can still compete, up to this many more
    # (2026-10-09: a wrongly leading basin lowered the floor so 11-30 clusters
    # competed and every search at one placement was incomplete).
    max_extra_refined_clusters: int = 16
    # A cluster left unrefined still competes when its coarse score is this
    # close to the best coarse score among clusters that refined into the
    # winner's basin.
    coarse_competition_margin: float = 0.15
    refine_evaluations: int = 160
    refine_initial_step_m: float = 0.10
    refine_initial_yaw_rad: float = math.radians(5.0)
    refine_min_step_m: float = 0.01
    refine_min_yaw_rad: float = math.radians(0.5)
    # The match tolerance makes the hit score flat near the optimum.  Every
    # cluster then gets the same polishing budget on a smooth distance-field
    # fit; acceptance gates still use the standard scan_map_metrics.
    fit_sigma_m: float = 0.05
    polish_evaluations: int = 80
    polish_initial_step_m: float = 0.08
    polish_initial_yaw_rad: float = math.radians(3.0)
    polish_min_step_m: float = 0.005
    polish_min_yaw_rad: float = math.radians(0.25)


@dataclass(frozen=True)
class ValidationThresholds:
    """Initial holdout gates from the specification; not relaxed here."""

    min_score: float = 0.65
    min_coverage: float = 0.65
    min_known: int = 30
    max_conflict: float = 0.25
    min_margin: float = 0.12
    # Loss on the smooth fit score that still counts as near-optimal.
    support_score_loss: float = 0.03
    support_step_m: float = 0.05
    support_range_m: float = 0.60
    support_step_yaw_rad: float = math.radians(1.0)
    support_range_yaw_rad: float = math.radians(15.0)
    max_support_xy_m: float = 0.20
    max_support_yaw_rad: float = math.radians(5.0)


@dataclass(frozen=True)
class QualityDecision:
    """Holdout verdict; ``winner`` is set only when accepted."""

    accepted: bool
    reason: str
    winner: object = None
    runner_up_score: object = None
    holdout: tuple = field(default_factory=tuple)


def grid_snapshot(message):
    """Copy a nav_msgs/OccupancyGrid into a picklable structure."""
    info = message.info
    origin = info.origin
    return SimpleNamespace(
        frame_id=message.header.frame_id,
        info=SimpleNamespace(
            width=int(info.width), height=int(info.height),
            resolution=float(info.resolution),
            origin=SimpleNamespace(
                position=SimpleNamespace(
                    x=float(origin.position.x), y=float(origin.position.y)),
                orientation=SimpleNamespace(
                    x=float(origin.orientation.x),
                    y=float(origin.orientation.y),
                    z=float(origin.orientation.z),
                    w=float(origin.orientation.w)))),
        data=array('b', message.data),
    )


def map_hash(grid):
    """Content hash over geometry, origin and cells."""
    info = grid.info
    origin = info.origin
    header = struct.pack(
        '<IIdddd', info.width, info.height, info.resolution,
        origin.position.x, origin.position.y,
        quaternion_yaw(origin.orientation))
    data = grid.data
    payload = data.tobytes() if hasattr(data, 'tobytes') else bytes(
        value & 0xFF for value in data)
    return hashlib.sha256(header + payload).hexdigest()


def cell_center_to_world(grid, cell_x, cell_y):
    """World coordinates of a cell centre, honouring the origin yaw."""
    info = grid.info
    yaw = quaternion_yaw(info.origin.orientation)
    local_x = (cell_x + 0.5) * info.resolution
    local_y = (cell_y + 0.5) * info.resolution
    return (info.origin.position.x + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y,
            info.origin.position.y + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y)


def world_to_cell(grid, x, y):
    """Integer cell containing a world point, honouring the origin yaw."""
    info = grid.info
    yaw = quaternion_yaw(info.origin.orientation)
    dx, dy = x - info.origin.position.x, y - info.origin.position.y
    return (math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy)
                       / info.resolution),
            math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy)
                       / info.resolution))


def scan_pose_in_map(hypothesis, reference_odom, frame):
    """T_map_scan for one keyframe under hypothesis ``H`` (an SE2)."""
    return (hypothesis.compose(reference_odom.inverse())
            .compose(frame.T_odom_base).compose(frame.T_base_scan))


def seed_pose_at_current_time(winner, reference_odom, current_odom):
    """Map->base now: H * inv(T_odom_base(t0)) * T_odom_base(now)."""
    return winner.compose(reference_odom.inverse()).compose(current_odom)


def _transform(pose):
    return SimpleNamespace(
        translation=SimpleNamespace(x=pose.x, y=pose.y),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(pose.yaw / 2.0),
                                 w=math.cos(pose.yaw / 2.0)))


def _median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def score_pose(grid, pose, frames, reference_odom, config, beams):
    """
    Robust per-view aggregation, then equal weight per view.

    Repeated frames of one view are combined by median so that resampling a
    single viewpoint cannot inflate confidence.
    """
    views = {}
    for frame in frames:
        metrics = scan_map_metrics(
            grid, frame.scan,
            _transform(scan_pose_in_map(pose, reference_odom, frame)),
            config.occupied_threshold, config.tolerance_cells, beams)
        views.setdefault(frame.view_id, []).append(metrics)
    per_view = []
    for view_id in sorted(views):
        items = views[view_id]
        per_view.append({
            'score': _median([m['score'] for m in items]),
            'coverage': _median([m['coverage'] for m in items]),
            'conflict': _median([m['wall_conflict_ratio'] for m in items]),
            'known': min(m['known'] for m in items),
        })
    count = len(per_view)
    return {
        'score': sum(v['score'] for v in per_view) / count,
        'coverage': sum(v['coverage'] for v in per_view) / count,
        'conflict': sum(v['conflict'] for v in per_view) / count,
        'known': min(v['known'] for v in per_view),
        'per_view': tuple(v['score'] for v in per_view),
    }


def distance_field(grid, occupied_threshold):
    """
    Two-pass chamfer distance (m) to the nearest occupied cell.

    Cacheable by map_hash; computed in the worker, never on the executor.
    """
    info = grid.info
    width, height = info.width, info.height
    big = float(width + height) * 2.0
    field = array('d', [0.0 if value >= occupied_threshold else big
                        for value in grid.data])
    diagonal = math.sqrt(2.0)
    for y in range(height):
        row = y * width
        for x in range(width):
            index = row + x
            best = field[index]
            if x > 0:
                best = min(best, field[index - 1] + 1.0)
            if y > 0:
                best = min(best, field[index - width] + 1.0)
                if x > 0:
                    best = min(best, field[index - width - 1] + diagonal)
                if x + 1 < width:
                    best = min(best, field[index - width + 1] + diagonal)
            field[index] = best
    for y in range(height - 1, -1, -1):
        row = y * width
        for x in range(width - 1, -1, -1):
            index = row + x
            best = field[index]
            if x + 1 < width:
                best = min(best, field[index + 1] + 1.0)
            if y + 1 < height:
                best = min(best, field[index + width] + 1.0)
                if x + 1 < width:
                    best = min(best, field[index + width + 1] + diagonal)
                if x > 0:
                    best = min(best, field[index + width - 1] + diagonal)
            field[index] = best
    for index, value in enumerate(field):
        field[index] = value * info.resolution
    return field


def _frame_fit(grid, field, frame, sensor, beams, sigma):
    scan = frame.scan
    valid = [index for index, distance in enumerate(scan.ranges)
             if math.isfinite(distance)
             and scan.range_min <= distance <= scan.range_max]
    if not valid:
        return 0.0
    if len(valid) > beams:
        valid = [valid[round(i * (len(valid) - 1) / (beams - 1))]
                 for i in range(beams)]
    info = grid.info
    cosine, sine = math.cos(sensor.yaw), math.sin(sensor.yaw)
    total = 0.0
    for index in valid:
        distance = scan.ranges[index]
        angle = scan.angle_min + index * scan.angle_increment
        lx, ly = distance * math.cos(angle), distance * math.sin(angle)
        cell_x, cell_y = world_to_cell(
            grid, sensor.x + cosine * lx - sine * ly,
            sensor.y + sine * lx + cosine * ly)
        if 0 <= cell_x < info.width and 0 <= cell_y < info.height:
            d = field[cell_y * info.width + cell_x]
            total += math.exp(-0.5 * (d / sigma) ** 2)
    return total / len(valid)


def fit_score(grid, field, pose, frames, reference_odom, config):
    """Smooth endpoint fit in [0, 1]; median per view, views equal weight."""
    views = {}
    for frame in frames:
        sensor = scan_pose_in_map(pose, reference_odom, frame)
        views.setdefault(frame.view_id, []).append(_frame_fit(
            grid, field, frame, sensor, config.refine_beams,
            config.fit_sigma_m))
    return sum(_median(v) for v in views.values()) / len(views)


def _ranking(item):
    return (item['score'], -item['conflict'], item['coverage'], item['known'])


def _angle_distance(first, second):
    return abs(normalize_angle(first - second))


def cluster_hypotheses(candidates, xy_radius, yaw_radius):
    """
    Greedy clustering on position AND circular yaw distance.

    ``candidates`` are ``(score, x, y, yaw)`` tuples.  Membership is tested
    against each cluster's seed only, so chains of neighbours cannot let one
    cluster swallow a nearby distinct solution; the same position with an
    opposite heading always forms a separate cluster.
    """
    clusters = []
    for candidate in sorted(candidates, key=lambda c: c[0], reverse=True):
        _score, x, y, yaw = candidate
        for cluster in clusters:
            _seed_score, sx, sy, syaw = cluster['seed']
            if (math.hypot(x - sx, y - sy) <= xy_radius
                    and _angle_distance(yaw, syaw) <= yaw_radius):
                cluster['members'] += 1
                break
        else:
            clusters.append({'cluster_id': len(clusters),
                             'seed': candidate, 'members': 1})
    return tuple(clusters)


def _expired(deadline, cancel_token):
    if cancel_token is not None and cancel_token():
        return RejectReason.CANCELED
    if deadline is not None and time.monotonic() > deadline:
        return RejectReason.SEARCH_INCOMPLETE
    return None


def _pattern_search(start_pose, evaluate, key, budget, step, yaw_step,
                    min_step, min_yaw, deadline, cancel_token):
    best_pose = start_pose
    best = evaluate(best_pose)
    evaluations = 1
    while evaluations < budget:
        stop = _expired(deadline, cancel_token)
        if stop is not None:
            return None, stop
        improved = False
        for dx, dy, dyaw in ((step, 0, 0), (-step, 0, 0), (0, step, 0),
                             (0, -step, 0), (0, 0, yaw_step),
                             (0, 0, -yaw_step)):
            pose = SE2(best_pose.x + dx, best_pose.y + dy,
                       normalize_angle(best_pose.yaw + dyaw))
            candidate = evaluate(pose)
            evaluations += 1
            if key(candidate) > key(best):
                best_pose, best, improved = pose, candidate, True
            if evaluations >= budget:
                break
        if not improved:
            if step <= min_step and yaw_step <= min_yaw:
                break
            step = max(min_step, step / 2.0)
            yaw_step = max(min_yaw, yaw_step / 2.0)
    return (best_pose, best), None


def refine_cluster(grid, seed, frames, reference_odom, config,
                   deadline=None, cancel_token=None, field=None):
    """Pattern search, then smooth-fit polishing, on a fixed budget."""
    def standard(pose):
        return score_pose(grid, pose, frames, reference_odom, config,
                          config.refine_beams)

    outcome, stop = _pattern_search(
        SE2(seed[1], seed[2], normalize_angle(seed[3])), standard, _ranking,
        config.refine_evaluations, config.refine_initial_step_m,
        config.refine_initial_yaw_rad, config.refine_min_step_m,
        config.refine_min_yaw_rad, deadline, cancel_token)
    if stop is not None:
        return None, stop
    pose, metrics = outcome
    if field is None or config.polish_evaluations <= 0:
        return outcome, None
    outcome, stop = _pattern_search(
        pose, lambda candidate: fit_score(grid, field, candidate, frames,
                                          reference_odom, config),
        lambda value: value, config.polish_evaluations,
        config.polish_initial_step_m, config.polish_initial_yaw_rad,
        config.polish_min_step_m, config.polish_min_yaw_rad, deadline,
        cancel_token)
    if stop is not None:
        return None, stop
    # Gates and ranking stay on the standard, interpretable metric.
    return (outcome[0], standard(outcome[0])), None


def _incomplete(session, digest, evaluated, started, reason, hypotheses=()):
    return SearchResult(session, digest, False, tuple(hypotheses), evaluated,
                        time.monotonic() - started, reason.value)


def _train_frames(train_frames):
    frames = tuple(train_frames)
    if not frames:
        raise ContractError('search needs TRAIN frames')
    if any(frame.role != FrameRole.TRAIN for frame in frames):
        raise ContractError('search accepts TRAIN frames only')
    session = frames[0].session
    if any(frame.session != session for frame in frames):
        raise ContractError('frames from different sessions')
    return frames, session


def _same_basin(first, second, config):
    return (math.hypot(first.x - second.x, first.y - second.y)
            <= config.cluster_xy_m
            and _angle_distance(first.yaw, second.yaw)
            <= config.cluster_yaw_rad)


@dataclass(frozen=True)
class CoarseCache:
    """
    Per-view coarse scores of one session's map-wide searches.

    The multi-view coarse score is the mean of the single-view scores of one
    representative frame per view, so the scores of an earlier view can be
    reused verbatim while the map, the search settings, the reference frame
    and that view's representative are unchanged.  ``views`` maps a view id
    to ``(representative keyframe id, array of scores)`` in pose order.
    """

    key: tuple
    views: dict


@dataclass(frozen=True)
class SearchOutput:
    """A map-wide search result and the coarse cache for the next one."""

    result: SearchResult
    coarse_cache: CoarseCache


def _coarse_key(digest, config, reference_id):
    return (digest, reference_id, config.coarse_step_m,
            config.coarse_yaw_step_rad, config.coarse_beams,
            config.occupied_threshold, config.tolerance_cells)


def search_multiview(grid, train_frames, config, deadline=None,
                     cancel_token=None):
    """Map-wide search over every free cell centre using TRAIN frames."""
    return search_multiview_cached(grid, train_frames, config, deadline,
                                   cancel_token).result


def search_multiview_cached(grid, train_frames, config, deadline=None,
                            cancel_token=None, coarse_cache=None):
    """
    ``search_multiview`` that reuses and returns per-view coarse scores.

    The result is identical to a search without the cache; only views
    without valid cached scores are evaluated in the coarse pass.
    """
    started = time.monotonic()
    frames, session = _train_frames(train_frames)
    digest = map_hash(grid)
    reference = frames[0].T_odom_base
    views = {}
    for frame in frames:
        views.setdefault(frame.view_id, []).append(frame)
    # One representative per view keeps the coarse pass at single-view cost
    # per viewpoint; all TRAIN frames are used during refinement.
    view_ids = sorted(views)
    representative = {view: views[view][len(views[view]) // 2]
                      for view in view_ids}

    info = grid.info
    cell_step = max(1, round(config.coarse_step_m / info.resolution))
    yaw_count = max(1, math.ceil(2.0 * math.pi / config.coarse_yaw_step_rad))
    key = _coarse_key(digest, config, frames[0].id)
    cached = coarse_cache.views if (
        coarse_cache is not None and coarse_cache.key == key) else {}
    scores = {view: cached[view][1] for view in view_ids
              if view in cached
              and cached[view][0] == representative[view].id}
    fresh = {view: array('d') for view in view_ids if view not in scores}
    pose_count = sum(
        1 for cell_y in range(0, info.height, cell_step)
        for cell_x in range(0, info.width, cell_step)
        if grid.data[cell_y * info.width + cell_x] == 0) * yaw_count
    if any(len(values) != pose_count for values in scores.values()):
        fresh.update({view: array('d') for view in scores})
        scores = {}

    def incomplete(stop):
        return SearchOutput(_incomplete(session, digest, evaluated, started,
                                        stop), coarse_cache)

    coarse = []
    evaluated = 0
    index = 0
    for cell_y in range(0, info.height, cell_step):
        stop = _expired(deadline, cancel_token)
        if stop is not None:
            return incomplete(stop)
        for cell_x in range(0, info.width, cell_step):
            # A free cell is a candidate centre, not a rotation-safety proof.
            if grid.data[cell_y * info.width + cell_x] != 0:
                continue
            x, y = cell_center_to_world(grid, cell_x, cell_y)
            for yaw_index in range(yaw_count):
                yaw = -math.pi + yaw_index * 2.0 * math.pi / yaw_count
                pose = SE2(x, y, yaw)
                for view, values in fresh.items():
                    values.append(score_pose(
                        grid, pose, (representative[view],), reference,
                        config, config.coarse_beams)['score'])
                # Same order and arithmetic as score_pose over all views.
                score = sum(
                    (scores[view] if view in scores else fresh[view])[index]
                    for view in view_ids) / len(view_ids)
                index += 1
                evaluated += 1
                if score > 0.0:
                    coarse.append((score, x, y, yaw))
    cache = CoarseCache(key, {
        view: (representative[view].id,
               scores[view] if view in scores else fresh[view])
        for view in view_ids})
    coarse.sort(reverse=True)
    clusters = cluster_hypotheses(coarse[:config.coarse_keep],
                                  config.cluster_xy_m, config.cluster_yaw_rad)
    if not clusters:
        return SearchOutput(SearchResult(session, digest, True, (), evaluated,
                                         time.monotonic() - started, ''),
                            cache)

    field = distance_field(grid, config.occupied_threshold)
    refined = []
    refined_ids = set()

    def same_basin(first, second):
        return _same_basin(first, second, config)

    def summarize():
        ranked = sorted(refined, key=lambda item: _ranking(item[2]),
                        reverse=True)
        # Neighbouring coarse clusters may refine into the same optimum.
        # That is one solution, not an independent alternative; keep the
        # best copy.
        distinct = []
        for item in ranked:
            if not any(same_basin(item[1], kept[1]) for kept in distinct):
                distinct.append(item)
        # The winner's coarse evidence is the best coarse score of every
        # cluster that refined into its basin.  Taking only the copy that
        # happened to refine highest let frame noise pick a weak duplicate
        # and lower the competition floor below unrelated clusters
        # (2026-10-05 real replay).
        winner_coarse = max(cluster['seed'][0] for cluster, pose, _metrics
                            in refined if same_basin(pose, distinct[0][1]))

        def represented(seed):
            # Inside the basin of an already refined solution: not
            # independent.
            return any(
                math.hypot(seed[1] - item[1].x, seed[2] - item[1].y)
                <= config.cluster_xy_m
                and _angle_distance(seed[3], item[1].yaw)
                <= config.cluster_yaw_rad
                for item in distinct)

        competitors = [
            cluster for cluster in clusters
            if cluster['cluster_id'] not in refined_ids
            and cluster['seed'][0] >= winner_coarse
            - config.coarse_competition_margin
            and not represented(cluster['seed'])]
        return distinct, competitors

    queue = list(clusters[:config.max_refined_clusters])
    limit = config.max_refined_clusters + config.max_extra_refined_clusters
    while True:
        for cluster in queue:
            outcome, stop = refine_cluster(
                grid, cluster['seed'], frames, reference, config, deadline,
                cancel_token, field)
            if stop is not None:
                return SearchOutput(_incomplete(session, digest, evaluated,
                                                started, stop), cache)
            pose, metrics = outcome
            evaluated += config.refine_evaluations + config.polish_evaluations
            refined.append((cluster, pose, metrics))
            refined_ids.add(cluster['cluster_id'])
        distinct, competitors = summarize()
        if not competitors or len(refined) >= limit:
            break
        queue = competitors[:1]           # strongest competitor next

    hypotheses = tuple(
        Hypothesis(
            x=pose.x, y=pose.y, yaw=pose.yaw,
            score=metrics['score'], coverage=metrics['coverage'],
            conflict=metrics['conflict'], cluster_id=cluster['cluster_id'],
            per_view=metrics['per_view'], support_bounds=())
        for cluster, pose, metrics in distinct)

    if competitors:
        return SearchOutput(_incomplete(
            session, digest, evaluated, started,
            RejectReason.SEARCH_INCOMPLETE, hypotheses), cache)
    return SearchOutput(SearchResult(session, digest, True, hypotheses,
                                     evaluated, time.monotonic() - started,
                                     ''), cache)


def recheck_hypotheses(grid, previous, previous_reference, train_frames,
                       config, deadline=None, cancel_token=None):
    """
    Re-refine the hypotheses of a complete search with every TRAIN view.

    Spec 5.3: the map-wide search runs once and later views re-check its
    candidates.  An incomplete search may have dropped alternatives, so it
    is refused here and needs a new map-wide search.  Each hypothesis is
    moved from ``previous_reference`` (odom pose of the previous search's
    reference frame) to this request's reference frame first.  Nothing is
    pruned: a candidate the new view weakens still competes in validation.
    """
    started = time.monotonic()
    frames, session = _train_frames(train_frames)
    if not previous.complete or not previous.hypotheses:
        raise ContractError('re-check needs a complete search with hypotheses')
    if previous.session != session:
        raise ContractError('re-check frames are from another session')
    digest = map_hash(grid)
    if digest != previous.map_hash:
        return _incomplete(session, digest, 0, started,
                           RejectReason.MAP_CHANGED)
    reference = frames[0].T_odom_base
    shift = previous_reference.inverse().compose(reference)
    field = distance_field(grid, config.occupied_threshold)
    refined = []
    evaluated = 0
    for hypothesis in previous.hypotheses:
        start = SE2(hypothesis.x, hypothesis.y, hypothesis.yaw).compose(shift)
        outcome, stop = refine_cluster(
            grid, (hypothesis.score, start.x, start.y, start.yaw), frames,
            reference, config, deadline, cancel_token, field)
        if stop is not None:
            return _incomplete(session, digest, evaluated, started, stop)
        evaluated += config.refine_evaluations + config.polish_evaluations
        refined.append((hypothesis.cluster_id,) + outcome)
    refined.sort(key=lambda item: _ranking(item[2]), reverse=True)
    distinct = []
    for item in refined:
        if not any(_same_basin(item[1], kept[1], config) for kept in distinct):
            distinct.append(item)
    hypotheses = tuple(
        Hypothesis(
            x=pose.x, y=pose.y, yaw=pose.yaw,
            score=metrics['score'], coverage=metrics['coverage'],
            conflict=metrics['conflict'], cluster_id=cluster_id,
            per_view=metrics['per_view'], support_bounds=())
        for cluster_id, pose, metrics in distinct)
    return SearchResult(session, digest, True, hypotheses, evaluated,
                        time.monotonic() - started, '')


def _support_extent(evaluate, center, floor, direction, step, limit):
    """Measure the connected distance along one axis above ``floor``."""
    extent = 0.0
    for sign in (1.0, -1.0):
        distance = 0.0
        while True:
            candidate = distance + step
            if candidate > limit + 1e-9:
                return None  # Still near-optimal at the edge: unbounded.
            if evaluate(direction(center, sign * candidate)) < floor:
                break
            distance = candidate
        extent += distance
    return extent


def validate_hypotheses(grid, result, train_frames, holdout_frames, config,
                        thresholds, deadline=None, cancel_token=None):
    """Re-score every refined cluster on independent HOLDOUT frames."""
    if not result.complete:
        return QualityDecision(False, result.reason or
                               RejectReason.SEARCH_INCOMPLETE.value)
    holdout = tuple(holdout_frames)
    if not holdout or any(f.role != FrameRole.HOLDOUT for f in holdout):
        raise ContractError('validation needs HOLDOUT frames')
    train = tuple(train_frames)
    train_ids = {frame.id for frame in train}
    train_stamps = {frame.stamp_ns for frame in train}
    if any(frame.id in train_ids or frame.stamp_ns in train_stamps
           for frame in holdout):
        raise ContractError('a TRAIN frame cannot serve as HOLDOUT')
    if any(frame.session != result.session for frame in holdout):
        raise ContractError('HOLDOUT frames are from another session')
    if map_hash(grid) != result.map_hash:
        return QualityDecision(False, RejectReason.MAP_CHANGED.value)
    if not result.hypotheses:
        return QualityDecision(False, RejectReason.NO_VALID_CANDIDATE.value)

    reference = train[0].T_odom_base
    rescored = []
    for hypothesis in result.hypotheses:
        stop = _expired(deadline, cancel_token)
        if stop is not None:
            return QualityDecision(False, stop.value)
        pose = SE2(hypothesis.x, hypothesis.y, hypothesis.yaw)
        rescored.append((hypothesis, pose, score_pose(
            grid, pose, holdout, reference, config, config.refine_beams)))
    rescored.sort(key=lambda item: _ranking(item[2]), reverse=True)
    summary = tuple(
        (item[0].cluster_id, round(item[2]['score'], 4)) for item in rescored)
    best_hypothesis, best_pose, best = rescored[0]
    runner_up = rescored[1][2]['score'] if len(rescored) > 1 else None
    if (best['score'] < thresholds.min_score
            or best['coverage'] < thresholds.min_coverage
            or best['known'] < thresholds.min_known
            or best['conflict'] > thresholds.max_conflict):
        return QualityDecision(False, RejectReason.NO_VALID_CANDIDATE.value,
                               runner_up_score=runner_up, holdout=summary)
    if (runner_up is not None
            and best['score'] - runner_up < thresholds.min_margin):
        return QualityDecision(False, RejectReason.AMBIGUOUS_LOCATION.value,
                               runner_up_score=runner_up, holdout=summary)

    # Corridor check: the near-optimal region of the smooth fit must be
    # bounded in every planar direction and in yaw; otherwise that axis is
    # unobservable.  (The standard hit score is flat over its own tolerance
    # width and would mask real observability.)
    field = distance_field(grid, config.occupied_threshold)

    def fit(pose):
        return fit_score(grid, field, pose, holdout, reference, config)

    floor = fit(best_pose) - thresholds.support_score_loss
    extents = []
    for angle in (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0):
        stop = _expired(deadline, cancel_token)
        if stop is not None:
            return QualityDecision(False, stop.value)
        cosine, sine = math.cos(angle), math.sin(angle)
        extent = _support_extent(
            fit, best_pose, floor,
            lambda c, d, cs=cosine, sn=sine: SE2(c.x + cs * d, c.y + sn * d,
                                                 c.yaw),
            thresholds.support_step_m, thresholds.support_range_m)
        if extent is None or extent > thresholds.max_support_xy_m:
            return QualityDecision(
                False, RejectReason.UNOBSERVABLE_AXIS.value,
                runner_up_score=runner_up, holdout=summary)
        extents.append((angle, extent))
    yaw_extent = _support_extent(
        fit, best_pose, floor,
        lambda c, d: SE2(c.x, c.y, normalize_angle(c.yaw + d)),
        thresholds.support_step_yaw_rad, thresholds.support_range_yaw_rad)
    if yaw_extent is None or yaw_extent > thresholds.max_support_yaw_rad:
        return QualityDecision(False, RejectReason.UNOBSERVABLE_AXIS.value,
                               runner_up_score=runner_up, holdout=summary)
    support = (max(abs(math.cos(a)) * e for a, e in extents),
               max(abs(math.sin(a)) * e for a, e in extents), yaw_extent)
    winner = Hypothesis(
        x=best_pose.x, y=best_pose.y, yaw=best_pose.yaw,
        score=best['score'], coverage=best['coverage'],
        conflict=best['conflict'], cluster_id=best_hypothesis.cluster_id,
        per_view=best['per_view'], support_bounds=support)
    return QualityDecision(True, '', winner=winner,
                           runner_up_score=runner_up, holdout=summary)


def run_search_job(grid, train_frames, config, timeout_s, coarse_cache=None):
    """Worker entry point: a SearchOutput with the updated coarse cache."""
    return search_multiview_cached(grid, train_frames, config,
                                   deadline=time.monotonic() + timeout_s,
                                   coarse_cache=coarse_cache)


def run_recheck_job(grid, previous, previous_reference, train_frames, config,
                    timeout_s):
    """Worker entry point for recheck_hypotheses."""
    return recheck_hypotheses(grid, previous, previous_reference,
                              train_frames, config,
                              deadline=time.monotonic() + timeout_s)


def run_validation_job(grid, result, train_frames, holdout_frames, config,
                       thresholds, timeout_s):
    """Worker entry point for validate_hypotheses."""
    return validate_hypotheses(grid, result, train_frames, holdout_frames,
                               config, thresholds,
                               deadline=time.monotonic() + timeout_s)


def _worker_main(connection, function, arguments):
    try:
        outcome = ('ok', function(*arguments))
    except Exception as error:  # Reported to the parent, never swallowed.
        outcome = ('error', f'{type(error).__name__}: {error}')
    connection.send(outcome)
    connection.close()


class SearchWorker:
    """
    One in-flight CPU job in a separate process.

    The ROS executor never runs the search.  ``cancel`` terminates and reaps
    the process, and every job carries a token so that a late result from an
    invalidated session or map can be recognised and dropped by the caller.
    """

    def __init__(self, context='spawn'):
        """Use a spawned process so no ROS/DDS threads are forked."""
        self._context = multiprocessing.get_context(context)
        self._process = None
        self._connection = None
        self.token = None

    @property
    def busy(self):
        """Whether a job is in flight."""
        return self._process is not None

    def submit(self, token, function, *arguments):
        """Start one job; refuses to queue behind another."""
        if self.busy:
            raise RuntimeError('a search job is already in flight')
        # A one-way pipe needs no POSIX semaphore, unlike Queue.
        receiver, sender = self._context.Pipe(duplex=False)
        self._process = self._context.Process(
            target=_worker_main, args=(sender, function, arguments),
            daemon=True)
        self._process.start()
        sender.close()
        self._connection = receiver
        self.token = token

    def poll(self):
        """Return None while running, else ``(token, status, payload)``."""
        if self._process is None:
            return None
        try:
            ready = self._connection.poll()
            status, payload = (self._connection.recv() if ready
                               else (None, None))
        except EOFError:
            ready, status, payload = True, 'error', (
                f'worker exited with code {self._process.exitcode}')
        if not ready:
            if self._process.is_alive():
                return None
            # The child may have sent its result and exited between the two
            # checks above (#17); read the pipe before calling it an error.
            try:
                ready = self._connection.poll()
                if ready:
                    status, payload = self._connection.recv()
            except EOFError:
                ready = False
            if not ready:
                status, payload = 'error', (
                    f'worker exited with code {self._process.exitcode}')
        token = self.token
        self._reap()
        return token, status, payload

    def cancel(self, timeout_s=1.0):
        """Terminate any job; idempotent."""
        if self._process is None:
            return
        if self._process.is_alive():
            self._process.terminate()
        self._process.join(timeout_s)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout_s)
        self._reap()

    def _reap(self):
        if self._process is not None:
            self._process.join(0.1)
        if self._connection is not None:
            self._connection.close()
        self._process = None
        self._connection = None
        self.token = None
