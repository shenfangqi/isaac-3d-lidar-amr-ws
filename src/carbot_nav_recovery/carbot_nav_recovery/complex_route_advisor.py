"""Validation-only ROS adapter for Issue #12 complex-route speed advice."""

from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path as FilePath
import time

from geometry_msgs.msg import Point
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .ros_snapshots import raw_costmap
from .speed_advisor import BrakingProfile, advise_speed
from .swept_footprint import Pose2D, SnapshotFreshness


def _yaw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non-finite quaternion')
    norm = sum(value * value for value in values)
    if not 0.99 <= norm <= 1.01:
        raise ValueError('path quaternion is not normalized')
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z))


def _pose_from_transform(transform):
    translation = transform.transform.translation
    return Pose2D(translation.x, translation.y,
                  _yaw(transform.transform.rotation))


def local_path(path_message, current_pose, horizon_m, max_join_distance_m):
    """Build a bounded forward path beginning at the current robot pose."""
    if (not math.isfinite(horizon_m) or horizon_m <= 0.0
            or not math.isfinite(max_join_distance_m)
            or max_join_distance_m <= 0.0):
        raise ValueError('invalid path selection limits')
    points = []
    for stamped in path_message.poses:
        position = stamped.pose.position
        if not all(math.isfinite(value) for value in
                   (position.x, position.y, position.z)):
            raise ValueError('path position is not finite')
        points.append(Pose2D(position.x, position.y,
                             _yaw(stamped.pose.orientation)))
    if len(points) < 2:
        raise ValueError('path has fewer than two poses')
    nearest = min(range(len(points)), key=lambda index: math.hypot(
        points[index].x - current_pose.x,
        points[index].y - current_pose.y))
    join_distance = math.hypot(points[nearest].x - current_pose.x,
                               points[nearest].y - current_pose.y)
    if join_distance > max_join_distance_m:
        raise ValueError('PATH_NOT_NEAR_ROBOT')
    selected = [current_pose]
    if join_distance > 1.0e-4:
        selected.append(points[nearest])
    travelled = join_distance
    previous = points[nearest]
    for point in points[nearest + 1:]:
        segment = math.hypot(point.x - previous.x, point.y - previous.y)
        if travelled + segment <= horizon_m:
            selected.append(point)
            travelled += segment
            previous = point
            continue
        remaining = horizon_m - travelled
        if segment > 1.0e-9 and remaining > 1.0e-6:
            fraction = remaining / segment
            yaw_delta = math.atan2(math.sin(point.yaw - previous.yaw),
                                   math.cos(point.yaw - previous.yaw))
            selected.append(Pose2D(
                previous.x + (point.x - previous.x) * fraction,
                previous.y + (point.y - previous.y) * fraction,
                previous.yaw + yaw_delta * fraction))
        break
    if len(selected) < 2:
        raise ValueError('NO_FORWARD_PATH')
    return tuple(selected)


class ComplexRouteAdvisor(Node):
    """Publish diagnostics and markers; never create a velocity publisher."""

    def __init__(self):
        """Create subscriptions and read-only diagnostic publishers."""
        super().__init__('carbot_complex_route_advisor')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('status_topic',
                               '/carbot_nav_recovery/complex_route_advisory')
        self.declare_parameter('marker_topic',
                               '/carbot_nav_recovery/complex_route_markers')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('braking_profile', '')
        self.declare_parameter('navigation_config_path', '')
        self.declare_parameter('snapshot_directory',
                               '/tmp/carbot_complex_route')
        self.declare_parameter('horizon_m', 1.5)
        self.declare_parameter('max_path_join_distance_m', 0.50)
        self.declare_parameter('max_costmap_age_sec', 0.5)
        self.declare_parameter('max_odom_age_sec', 0.5)
        self.declare_parameter('max_path_receive_age_sec', 5.0)
        self.declare_parameter('max_localization_age_sec', 0.75)
        self.declare_parameter('evaluation_budget_sec', 0.08)
        self.declare_parameter('safety_margin_m', 0.0)
        self.declare_parameter('localization_valid', False)
        self.declare_parameter('localization_status_topic',
                               '/automatic_localization/status')
        self.declare_parameter(
            'footprint_xy', [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133])
        values = list(self.get_parameter('footprint_xy').value)
        if len(values) < 6 or len(values) % 2:
            raise ValueError('footprint_xy must contain at least 3 x/y pairs')
        self._footprint = tuple((float(values[index]),
                                 float(values[index + 1]))
                                for index in range(0, len(values), 2))
        self._profile, self._profile_reason = self._load_profile()
        self._navigation_config_sha256 = self._hash_navigation_config()
        self._path = None
        self._path_received = None
        self._costmap = None
        self._costmap_received = None
        self._odom = None
        self._odom_received = None
        self._localization_ready = False
        self._localization_received = None
        self._localization_reason = 'NO_LOCALIZATION_STATUS'
        self._last_snapshot = None

        reliable = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(
            depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            Path, str(self.get_parameter('path_topic').value),
            self._on_path, reliable)
        self.create_subscription(
            Costmap, str(self.get_parameter('costmap_topic').value),
            self._on_costmap, reliable)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, sensor)
        self.create_subscription(
            String,
            str(self.get_parameter('localization_status_topic').value),
            self._on_localization, 10)
        self._status = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self._markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_service(Trigger, '~/save_snapshot', self._save_snapshot)
        self.create_timer(0.5, self._evaluate)
        self.get_logger().info(
            'Issue #12 validation-only advisor started; no velocity publisher')

    def _load_profile(self):
        path = str(self.get_parameter('braking_profile').value)
        if not path:
            return None, 'CALIBRATION_REQUIRED'
        if not os.path.isabs(path) or not os.path.isfile(path):
            return None, 'INVALID_BRAKING_PROFILE_PATH'
        try:
            with open(path, encoding='utf-8') as stream:
                values = json.load(stream)
            profile = BrakingProfile.from_mapping(values)
            if not profile.physical_acceptance_complete:
                return None, 'CALIBRATION_REQUIRED'
            if not os.path.isdir(profile.evidence_directory):
                return None, 'EVIDENCE_DIRECTORY_MISSING'
            return profile, 'OK'
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None, 'INVALID_BRAKING_PROFILE'

    def _on_path(self, message):
        self._path = message
        self._path_received = time.monotonic()

    def _on_costmap(self, message):
        self._costmap = message
        self._costmap_received = time.monotonic()

    def _on_odom(self, message):
        self._odom = message
        self._odom_received = time.monotonic()

    def _on_localization(self, message):
        self._localization_received = time.monotonic()
        try:
            status = json.loads(message.data)
            self._localization_ready = bool(
                isinstance(status, dict)
                and status.get('ready') is True
                and status.get('state') == 'READY'
                and status.get('navigation_activated') is True
                and status.get('validation_only') is False
                and not status.get('failure_reason'))
            self._localization_reason = (
                'OK' if self._localization_ready
                else 'LOCALIZATION_NOT_READY')
        except (json.JSONDecodeError, TypeError):
            self._localization_ready = False
            self._localization_reason = 'INVALID_LOCALIZATION_STATUS'

    def _localization_gate(self, now):
        if bool(self.get_parameter('localization_valid').value):
            return True, 'PARAMETER_OVERRIDE'
        age_limit = float(
            self.get_parameter('max_localization_age_sec').value)
        if (self._localization_received is None
                or now - self._localization_received > age_limit):
            return False, 'LOCALIZATION_STATUS_STALE'
        return self._localization_ready, self._localization_reason

    def _publish_reason(self, reason, **details):
        report = {
            'schema': 'carbot_complex_route_advisory_v1',
            'reason': reason,
            'validation_only': True,
            'motion_eligible': False,
            'braking_calibrated': self._profile is not None,
            'profile_status': self._profile_reason,
            'navigation_config_sha256': self._navigation_config_sha256,
            **details,
        }
        self._status.publish(String(data=json.dumps(report, allow_nan=False)))
        self._publish_markers((), reason, None)

    def _evaluate(self):
        now_monotonic = time.monotonic()
        inputs = (self._path, self._costmap, self._odom)
        if any(value is None for value in inputs):
            self._publish_reason('MISSING_RUNTIME_INPUT')
            return
        path_age = now_monotonic - self._path_received
        odom_age = now_monotonic - self._odom_received
        if path_age > float(
                self.get_parameter('max_path_receive_age_sec').value):
            self._publish_reason('PATH_RECEIVE_STALE')
            return
        if odom_age > float(self.get_parameter('max_odom_age_sec').value):
            self._publish_reason('ODOMETRY_STALE')
            return
        global_frame = str(self.get_parameter('global_frame').value)
        if (self._path.header.frame_id != global_frame
                or self._costmap.header.frame_id != global_frame):
            self._publish_reason('FRAME_MISMATCH')
            return
        try:
            snapshot = raw_costmap(
                self._costmap, self._costmap_received)
            costmap_stamp = Time.from_msg(self._costmap.header.stamp)
            transform = self._tf_buffer.lookup_transform(
                global_frame,
                str(self.get_parameter('base_frame').value),
                costmap_stamp, timeout=Duration(seconds=0.0))
            current_pose = _pose_from_transform(transform)
            path = local_path(
                self._path, current_pose,
                float(self.get_parameter('horizon_m').value),
                float(self.get_parameter('max_path_join_distance_m').value))
        except TransformException:
            self._publish_reason('TF_INVALID')
            return
        except (ValueError, TypeError, OverflowError) as error:
            self._publish_reason(str(error))
            return

        now_ros = self.get_clock().now()
        odom_stamp = Time.from_msg(self._odom.header.stamp)
        odom_source_age = (now_ros - odom_stamp).nanoseconds * 1.0e-9
        max_odom_age = float(self.get_parameter('max_odom_age_sec').value)
        if not 0.0 <= odom_source_age <= max_odom_age:
            self._publish_reason('ODOMETRY_SOURCE_STALE')
            return
        localization_valid, localization_reason = self._localization_gate(
            now_monotonic)
        if not localization_valid:
            self._publish_reason(localization_reason)
            return
        freshness = SnapshotFreshness(
            now_ros_sec=now_ros.nanoseconds * 1.0e-9,
            now_monotonic_sec=now_monotonic,
            max_source_age_sec=float(
                self.get_parameter('max_costmap_age_sec').value),
            max_receive_age_sec=float(
                self.get_parameter('max_costmap_age_sec').value),
            localization_valid=True,
            tf_valid=True)
        twist = self._odom.twist.twist
        current_speed = math.hypot(twist.linear.x, twist.linear.y)
        budget = float(self.get_parameter('evaluation_budget_sec').value)
        try:
            advice = advise_speed(
                snapshot, self._footprint, path, freshness, current_speed,
                self._profile,
                float(self.get_parameter('safety_margin_m').value),
                deadline_monotonic=time.monotonic() + budget)
        except (ValueError, TypeError, OverflowError) as error:
            self._publish_reason('INVALID_ADVISORY_INPUT', detail=str(error))
            return
        report = asdict(advice)
        report.update({
            'schema': 'carbot_complex_route_advisory_v1',
            'profile_status': self._profile_reason,
            # Raw costmap publication time does not prove every layer updated.
            'costmap_update_verified': False,
            'command_applied': False,
            'navigation_config_sha256': self._navigation_config_sha256,
        })
        self._status.publish(String(data=json.dumps(report, allow_nan=False)))
        self._last_snapshot = {
            'schema': 'carbot_complex_route_snapshot_v1',
            'captured_unix_ns': time.time_ns(),
            'report': report,
            'path': [asdict(pose) for pose in path],
            'costmap': asdict(snapshot),
            'footprint': self._footprint,
        }
        self._publish_markers(path, advice.reason,
                              advice.recommended_speed_mps)

    def _hash_navigation_config(self):
        path = str(self.get_parameter('navigation_config_path').value)
        if not path:
            return ''
        if not os.path.isabs(path) or not os.path.isfile(path):
            return 'INVALID_PATH'
        digest = hashlib.sha256()
        try:
            with open(path, 'rb') as stream:
                for block in iter(lambda: stream.read(65536), b''):
                    digest.update(block)
        except OSError:
            return 'READ_ERROR'
        return digest.hexdigest()

    def _save_snapshot(self, request, response):
        if self._last_snapshot is None:
            response.success = False
            response.message = 'No coherent path/costmap evaluation available'
            return response
        try:
            directory = FilePath(
                str(self.get_parameter('snapshot_directory').value))
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f'advisory_{time.time_ns()}.json'
            with path.open('x', encoding='utf-8') as stream:
                json.dump(self._last_snapshot, stream, allow_nan=False)
            response.success = True
            response.message = str(path)
        except (OSError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _publish_markers(self, path, reason, speed):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        if path:
            marker = Marker()
            marker.header.frame_id = str(
                self.get_parameter('global_frame').value)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'complex_route_validation_only'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02
            marker.color.a = 0.9
            if reason == 'OK':
                marker.color.g = 1.0
            elif reason == 'CALIBRATION_REQUIRED':
                marker.color.r, marker.color.g = 1.0, 0.65
            else:
                marker.color.r = 1.0
            marker.lifetime = Duration(seconds=0.75).to_msg()
            marker.points = [Point(x=pose.x, y=pose.y, z=0.04)
                             for pose in path]
            output.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = marker.ns
            label.id = 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = path[0].x
            label.pose.position.y = path[0].y
            label.pose.position.z = 0.20
            label.pose.orientation.w = 1.0
            label.scale.z = 0.08
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = reason if speed is None else (
                f'{reason}: advisory {speed:.3f} m/s')
            label.lifetime = marker.lifetime
            output.markers.append(label)
        self._markers.publish(output)


def main(args=None):
    """Run the validation-only ROS node."""
    rclpy.init(args=args)
    node = ComplexRouteAdvisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
