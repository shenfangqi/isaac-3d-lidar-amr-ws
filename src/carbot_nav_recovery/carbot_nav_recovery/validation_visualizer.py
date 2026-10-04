"""Read-only RViz preview for rotation sweeps on the live local costmap."""

import math
import time
import json
from dataclasses import asdict
from pathlib import Path

from geometry_msgs.msg import Point
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .swept_footprint import (
    CostmapSnapshot,
    Pose2D,
    SnapshotFreshness,
    check_snapshot_data_freshness,
    check_observed_free_path,
    check_swept_path,
)
from .sensor_visibility import scan_visibility


class RecoveryValidationVisualizer(Node):
    """Show costmap rotation candidates without publishing motion commands."""

    def __init__(self):
        super().__init__('carbot_nav_recovery_validation')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic',
                               '/carbot_nav_recovery/markers')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_costmap_age_sec', 0.5)
        self.declare_parameter('max_costmap_receive_age_sec', 0.5)
        self.declare_parameter('max_tf_age_sec', 0.5)
        self.declare_parameter('safety_margin_m', 0.0)
        # Kept as an explicit diagnostic override for offline/synthetic use.
        # The real-robot launch leaves it false and supplies the guarded
        # automatic-localization status instead.
        self.declare_parameter('localization_valid', False)
        self.declare_parameter(
            'localization_status_topic', '/automatic_localization/status')
        self.declare_parameter('max_localization_age_sec', 0.75)
        self.declare_parameter('evaluation_budget_sec', 0.20)
        self.declare_parameter('snapshot_directory', '/tmp/carbot_recovery')
        self.declare_parameter(
            'candidate_angles_deg', [15.0, -15.0, 30.0, -30.0,
                                     60.0, -60.0, 90.0, -90.0])
        self.declare_parameter(
            'footprint_xy', [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133])

        footprint_values = list(self.get_parameter('footprint_xy').value)
        if len(footprint_values) < 6 or len(footprint_values) % 2:
            raise ValueError('footprint_xy must contain at least 3 x/y pairs')
        self._footprint = tuple(
            (float(footprint_values[index]),
             float(footprint_values[index + 1]))
            for index in range(0, len(footprint_values), 2))
        self._costmap = None
        self._costmap_received_monotonic = None
        self._scan = None
        self._scan_received_monotonic = None
        self._localization_ready = False
        self._localization_received_monotonic = None
        self._localization_reason = 'NO_LOCALIZATION_STATUS'
        self._last_snapshot = None

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Costmap,
            str(self.get_parameter('costmap_topic').value),
            self._on_costmap,
            qos,
        )
        scan_qos = QoSProfile(
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            scan_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('localization_status_topic').value),
            self._on_localization_status,
            10,
        )
        self._markers = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )
        self._status = self.create_publisher(
            String, '/carbot_nav_recovery/status', 10)
        self.create_service(Trigger, '~/save_snapshot', self._save_snapshot)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.5, self._evaluate)
        self.get_logger().info(
            'validation-only preview started; this node has no cmd_vel '
            'publisher and short translations remain disabled')

    def _on_costmap(self, message):
        self._costmap = message
        self._costmap_received_monotonic = time.monotonic()

    def _on_scan(self, message):
        self._scan = message
        self._scan_received_monotonic = time.monotonic()

    def _on_localization_status(self, message):
        self._localization_received_monotonic = time.monotonic()
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self._localization_ready = False
            self._localization_reason = 'INVALID_LOCALIZATION_STATUS'
            return
        if not isinstance(status, dict):
            self._localization_ready = False
            self._localization_reason = 'INVALID_LOCALIZATION_STATUS'
            return
        self._localization_ready = bool(
            status.get('ready') is True
            and status.get('state') == 'READY'
            and status.get('navigation_activated') is True
            and status.get('validation_only') is False
            and not status.get('failure_reason'))
        self._localization_reason = (
            'OK' if self._localization_ready else 'LOCALIZATION_NOT_READY')

    def _localization_gate(self, now_monotonic):
        if bool(self.get_parameter('localization_valid').value):
            return True, 'PARAMETER_OVERRIDE'
        received = self._localization_received_monotonic
        maximum_age = float(
            self.get_parameter('max_localization_age_sec').value)
        if received is None:
            return False, self._localization_reason
        if not math.isfinite(maximum_age) or not 0 < maximum_age <= 5.0:
            return False, 'INVALID_LOCALIZATION_MAX_AGE'
        if now_monotonic - received > maximum_age:
            return False, 'LOCALIZATION_STATUS_STALE'
        return self._localization_ready, self._localization_reason

    def _evaluate(self):
        self._last_snapshot = None
        message = self._costmap
        if message is None:
            self._publish_status('NO_COSTMAP')
            return
        try:
            snapshot = self._snapshot(message)
        except (ValueError, TypeError, OverflowError) as error:
            self._publish_status('INVALID_COSTMAP: ' + str(error))
            return
        try:
            stamp = Time.from_msg(message.header.stamp)
            transform = self._tf_buffer.lookup_transform(
                message.header.frame_id,
                str(self.get_parameter('base_frame').value),
                stamp,
                # Never block the executor that must also receive pending TF.
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            self._publish_status('TF_INVALID: ' + str(error))
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        tf_stamp = Time.from_msg(transform.header.stamp).nanoseconds * 1.0e-9
        tf_age = now_sec - tf_stamp
        tf_valid = 0.0 <= tf_age <= float(
            self.get_parameter('max_tf_age_sec').value)
        now_monotonic = time.monotonic()
        localization_valid, localization_reason = self._localization_gate(
            now_monotonic)
        freshness = SnapshotFreshness(
            now_ros_sec=now_sec,
            now_monotonic_sec=now_monotonic,
            max_source_age_sec=float(
                self.get_parameter('max_costmap_age_sec').value),
            max_receive_age_sec=float(
                self.get_parameter('max_costmap_receive_age_sec').value),
            localization_valid=localization_valid,
            tf_valid=tf_valid,
        )
        fresh = check_snapshot_data_freshness(snapshot, freshness)
        try:
            base_pose = _transform_pose(transform)
        except ValueError as error:
            self._publish_status('TF_INVALID: ' + str(error))
            return
        angles = [math.radians(float(value)) for value in
                  self.get_parameter('candidate_angles_deg').value]
        margin = float(self.get_parameter('safety_margin_m').value)
        budget = float(self.get_parameter('evaluation_budget_sec').value)
        if (not 0 < len(angles) <= 16
                or any(not math.isfinite(a) or abs(a) > math.pi
                       for a in angles)
                or not math.isfinite(margin) or not 0 <= margin <= 0.5
                or not math.isfinite(budget) or not 0 < budget <= 0.5):
            self._publish_status('INVALID_EVALUATION_PARAMETERS')
            return
        started = time.monotonic()
        deadline = started + budget
        candidates = []
        candidate_details = []
        if fresh.safe:
            scan = self._scan
            if scan is None or self._scan_received_monotonic is None:
                self._publish_status('NO_SCAN')
                return
            try:
                scan_transform = self._tf_buffer.lookup_transform(
                    message.header.frame_id, scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                    timeout=Duration(seconds=0.0))
                visibility = scan_visibility(
                    scan, _transform_pose(scan_transform), snapshot,
                    self._scan_received_monotonic, deadline=deadline,
                    world_bounds=(
                        base_pose.x-max(math.hypot(x, y)
                                        for x, y in self._footprint)-margin,
                        base_pose.y-max(math.hypot(x, y)
                                        for x, y in self._footprint)-margin,
                        base_pose.x+max(math.hypot(x, y)
                                        for x, y in self._footprint)+margin,
                        base_pose.y+max(math.hypot(x, y)
                                        for x, y in self._footprint)+margin))
            except (TransformException, ValueError) as error:
                self._publish_status(
                    'SCAN_VISIBILITY_INVALID: ' + str(error))
                return
            for angle in angles:
                path = (base_pose, Pose2D(
                    base_pose.x, base_pose.y, base_pose.yaw + angle))
                map_result = check_swept_path(
                    snapshot, self._footprint, path, margin,
                    deadline_monotonic=deadline)
                observed_result = check_observed_free_path(
                    visibility, self._footprint, path, freshness, margin,
                    deadline_monotonic=deadline)
                result = map_result if not map_result.safe else observed_result
                candidates.append((angle, result))
                candidate_details.append({
                    'angle_rad': angle,
                    'costmap_reason': map_result.reason,
                    'observed_free_reason': observed_result.reason,
                    **asdict(result),
                })
        report = {
            'schema': 'carbot_recovery_preview_v1',
            'motion_eligible': False,
            'costmap_update_verified': False,
            'source_attribution': 'FUSED_COSTMAP_LAYER_UNKNOWN',
            'publication_age_sec': now_sec - snapshot.stamp_sec,
            'tf_age_sec': tf_age,
            'localization_valid': localization_valid,
            'localization_reason': localization_reason,
            'gate_reason': fresh.reason,
            'departure_verified': False,
            'evaluation_ms': (time.monotonic() - started) * 1000.0,
            'candidates': candidate_details,
        }
        self._status.publish(String(data=json.dumps(report, allow_nan=False)))
        self._last_snapshot = {
            'schema': 'carbot_recovery_snapshot_v1',
            'costmap': asdict(snapshot), 'pose': asdict(base_pose),
            'footprint': self._footprint, 'angles_rad': angles,
            'safety_margin_m': margin, 'report': report,
        }
        self._publish_candidates(
            message.header.frame_id, now.to_msg(), base_pose, candidates,
            fresh.reason, tf_valid, localization_valid, localization_reason)

    def _snapshot(self, message):
        metadata = message.metadata
        origin = metadata.origin
        origin_yaw = _yaw(origin.orientation)
        quaternion = origin.orientation
        if (not all(math.isfinite(v) for v in (
                quaternion.x, quaternion.y, quaternion.z, quaternion.w))
                or abs(quaternion.x) > 1e-5 or abs(quaternion.y) > 1e-5
                or abs(quaternion.z ** 2 + quaternion.w ** 2 - 1) > 1e-5):
            raise ValueError('costmap origin needs a unit planar quaternion')
        if abs(origin_yaw) > 1.0e-5:
            raise ValueError('rotated costmap origins are not supported')
        if (len(message.data) > 1000000
                or len(message.data) != metadata.size_x * metadata.size_y):
            raise ValueError('costmap data length does not match metadata')
        stamp_sec = (message.header.stamp.sec
                     + message.header.stamp.nanosec * 1.0e-9)
        return CostmapSnapshot(
            width=int(metadata.size_x),
            height=int(metadata.size_y),
            resolution=float(metadata.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            costs=tuple(message.data),
            frame_id=message.header.frame_id,
            stamp_sec=stamp_sec,
            received_monotonic_sec=self._costmap_received_monotonic,
        )

    def _publish_candidates(
        self, frame_id, stamp, pose, candidates, gate_reason, tf_valid,
        localization_valid, localization_reason,
    ):
        output = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        output.markers.append(_footprint_marker(
            frame_id, stamp, self._footprint, pose, 0, (0.2, 0.8, 1.0)))

        summary = [
            'VALIDATION ONLY — NO MOTION',
            'rotation geometry: ' + ('fresh' if gate_reason == 'OK'
                                     else gate_reason),
            'localization gate: ' + (
                'VALID' if localization_valid else localization_reason),
            'TF: ' + ('VALID' if tf_valid else 'TF_INVALID'),
            'translation: disabled (near-field evidence unavailable)',
            'departure-path feasibility: not evaluated',
            'costmap update time / obstacle source: UNVERIFIED',
            'retrace: requires measured history + fresh rear visibility',
        ]
        for index, (angle, result) in enumerate(candidates):
            color = ((1.0, 0.7, 0.1) if result.safe else (1.0, 0.15, 0.1))
            output.markers.append(_sweep_marker(
                frame_id, stamp, self._footprint, pose, angle,
                10 + index, color))
            degrees = math.degrees(angle)
            summary.append(
                f'{degrees:+.0f} deg: {result.reason}'
                + (f' at ({result.blocked_cell[0]:.2f}, '
                   f'{result.blocked_cell[1]:.2f})'
                   if result.blocked_cell else ''))
            if result.blocked_cell:
                output.markers.append(_blocker_marker(
                    frame_id, stamp, result.blocked_cell,
                    100 + index))
        output.markers.append(_status_marker(
            frame_id, stamp, pose, '\n'.join(summary), 1000))
        for marker in output.markers:
            marker.lifetime = Duration(seconds=1.5).to_msg()
        self._markers.publish(output)

    def _publish_status(self, text):
        self._status.publish(String(data=json.dumps({
            'schema': 'carbot_recovery_preview_v1',
            'motion_eligible': False, 'gate_reason': text,
            'costmap_update_verified': False, 'candidates': [],
        })))
        stamp = self.get_clock().now().to_msg()
        marker = _status_marker(
            str(self.get_parameter('base_frame').value), stamp,
            Pose2D(0.0, 0.0, 0.0),
            'VALIDATION ONLY — NO MOTION\n' + text, 1000)
        clear = Marker()
        clear.header.frame_id = marker.header.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker.lifetime = Duration(seconds=1.5).to_msg()
        self._markers.publish(MarkerArray(markers=[clear, marker]))

    def _save_snapshot(self, request, response):
        if self._last_snapshot is None:
            response.success = False
            response.message = 'No coherent costmap/TF evaluation available'
            return response
        try:
            directory = Path(self.get_parameter('snapshot_directory').value)
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f'snapshot_{time.time_ns()}.json'
            with path.open('x', encoding='utf-8') as stream:
                json.dump(self._last_snapshot, stream, allow_nan=False)
            response.success = True
            response.message = str(path)
        except (OSError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response


def _transform_pose(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    if (not all(math.isfinite(v) for v in (
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w))
            or abs(sum(v * v for v in (
                rotation.x, rotation.y, rotation.z, rotation.w)) - 1) > 1e-5):
        raise ValueError('nonfinite translation or invalid quaternion')
    return Pose2D(translation.x, translation.y, _yaw(rotation))


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def _footprint_marker(frame_id, stamp, footprint, pose, marker_id, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'carbot_recovery'
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.015
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    points = _transform_footprint(footprint, pose)
    points.append(points[0])
    marker.points = [_point(x, y) for x, y in points]
    return marker


def _sweep_marker(frame_id, stamp, footprint, pose, angle, marker_id, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'carbot_recovery_sweeps'
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.008
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 0.9
    radius = max(math.hypot(x, y) for x, y in footprint)
    samples = max(2, int(math.ceil(radius * abs(angle) / 0.025)))
    for sample in range(samples + 1):
        yaw = pose.yaw + angle * sample / samples
        corners = _transform_footprint(
            footprint, Pose2D(pose.x, pose.y, yaw))
        for index, start in enumerate(corners):
            end = corners[(index + 1) % len(corners)]
            marker.points.extend((_point(*start), _point(*end)))
    return marker


def _transform_footprint(footprint, pose):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return [(pose.x + x * cosine - y * sine,
             pose.y + x * sine + y * cosine)
            for x, y in footprint]


def _blocker_marker(frame_id, stamp, location, marker_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'carbot_recovery_blockers'
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y = location
    marker.pose.position.z = 0.05
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.07
    marker.color.r = 1.0
    marker.color.g = 0.05
    marker.color.b = 0.05
    marker.color.a = 1.0
    return marker


def _status_marker(frame_id, stamp, pose, text, marker_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'carbot_recovery_status'
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = pose.x + 0.45
    marker.pose.position.y = pose.y + 0.35
    marker.pose.position.z = 0.6
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.11
    marker.color.r = marker.color.g = marker.color.b = 1.0
    marker.color.a = 1.0
    marker.text = text
    return marker


def _point(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0.025
    return point


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryValidationVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
