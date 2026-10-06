"""
Read-only Issue #13 rotation sweep preview.

Evaluates the configured probe rotations against the latest /scan at its
source time and shows observed free, observed occupied and unknown sweep
cells in RViz.  It has no velocity publisher and never commands motion.
"""

import json
import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .localization_contracts import (
    ContractError,
    decode_motion_profile,
    SE2,
)
from .localization_rotation_policy import (
    evaluate_localization_rotation,
    evidence_from_scan,
    footprint_geometry_hash,
    FREE,
    OCCUPIED,
    preview_payload,
    RotationGateConfig,
    UNKNOWN,
)


CANONICAL_FOOTPRINT = [0.155, 0.133, 0.155, -0.133,
                       -0.130, -0.133, -0.130, 0.133]
CELL_COLORS = {
    FREE: (0.1, 0.8, 0.2, 0.25),
    UNKNOWN: (1.0, 0.85, 0.0, 0.6),
    OCCUPIED: (0.9, 0.1, 0.1, 0.9),
}


class RotationPreview(Node):
    """Publish probe rotation verdicts and sweep evidence markers."""

    def __init__(self):
        super().__init__('localization_rotation_preview')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('footprint_xy', CANONICAL_FOOTPRINT)
        self.declare_parameter('padding_m', 0.05)
        self.declare_parameter('probe_angles_rad', [
            math.pi / 6, -math.pi / 6, math.pi / 3, -math.pi / 3,
            math.pi / 2, -math.pi / 2])
        self.declare_parameter('evidence_half_extent_m', 1.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('period_sec', 1.0)
        self.declare_parameter('compute_budget_sec', 0.5)
        self.declare_parameter('motion_profile_path', '')
        self.declare_parameter('extrinsics_hash', '')
        self.declare_parameter('control_chain_hash', '')
        self.declare_parameter('marker_topic',
                               '/automatic_localization/markers')
        self.declare_parameter('status_topic',
                               '/automatic_localization/rotation_preview')

        flat = list(self.get_parameter('footprint_xy').value)
        if len(flat) < 6 or len(flat) % 2:
            raise ValueError('footprint_xy needs at least three x,y pairs')
        self._footprint = tuple(zip(flat[0::2], flat[1::2]))
        self._config = RotationGateConfig(
            padding_m=float(self.get_parameter('padding_m').value))
        self._angles = [float(a) for a in
                        self.get_parameter('probe_angles_rad').value]
        self._geometry_hash = footprint_geometry_hash(
            self._footprint, self._config.padding_m)
        self._profile, self._profile_state = self._load_profile()
        extrinsics = self.get_parameter('extrinsics_hash').value
        control = self.get_parameter('control_chain_hash').value
        self._hashes = ((self._geometry_hash, extrinsics, control)
                        if extrinsics and control else None)

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._scan = None
        self._scan_received = None
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self._markers = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 1)
        self._status = self.create_publisher(
            String, self.get_parameter('status_topic').value, 1)
        self.create_timer(float(self.get_parameter('period_sec').value),
                          self._evaluate)
        self.get_logger().info(
            'read-only rotation preview: no velocity output; '
            f'profile={self._profile_state}')

    def _load_profile(self):
        path = self.get_parameter('motion_profile_path').value
        if not path:
            return None, 'none'
        try:
            with open(path, encoding='utf-8') as stream:
                profile = decode_motion_profile(stream.read())
        except (OSError, ContractError) as error:
            self.get_logger().error(f'motion profile rejected: {error}')
            return None, 'invalid'
        return profile, profile.status.value

    def _on_scan(self, message):
        self._scan = message
        self._scan_received = time.monotonic()

    def _lookup(self, target, source, stamp):
        transform = self._tf_buffer.lookup_transform(
            target, source, Time.from_msg(stamp)).transform
        rotation = transform.rotation
        yaw = math.atan2(2.0 * (rotation.w * rotation.z
                                + rotation.x * rotation.y),
                         1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
        return SE2(transform.translation.x, transform.translation.y, yaw)

    def _evaluate(self):
        scan = self._scan
        if scan is None:
            self._publish_error('SENSOR_STALE', 'no scan received')
            return
        try:
            # Source-time transforms only; the latest TF is not a substitute.
            sensor = self._lookup(self._odom_frame, scan.header.frame_id,
                                  scan.header.stamp)
            base = self._lookup(self._odom_frame, self._base_frame,
                                scan.header.stamp)
        except TransformException as error:
            self._publish_error('TF_AT_SOURCE_MISSING', str(error))
            return
        deadline = time.monotonic() + float(
            self.get_parameter('compute_budget_sec').value)
        try:
            evidence = evidence_from_scan(
                scan, sensor, (base.x, base.y),
                half_extent_m=float(
                    self.get_parameter('evidence_half_extent_m').value),
                resolution=float(self.get_parameter('resolution').value),
                frame_id=self._odom_frame,
                received_mono=self._scan_received, deadline=deadline)
            evaluations = [
                evaluate_localization_rotation(
                    evidence, self._footprint, base, angle,
                    profile=self._profile, hashes=self._hashes,
                    config=self._config)
                for angle in self._angles]
        except ContractError as error:
            # Includes the compute deadline and unsupported scan geometry.
            self._publish_error('EVALUATION_FAILED', str(error))
            return
        payload = preview_payload(evaluations,
                                  geometry_hash=self._geometry_hash,
                                  profile_state=self._profile_state)
        payload['scan_age_sec'] = round(
            time.monotonic() - self._scan_received, 3)
        self._status.publish(String(data=json.dumps(
            payload, allow_nan=False, sort_keys=True)))
        self._markers.publish(self._marker_array(evidence, base,
                                                 evaluations, scan))

    def _publish_error(self, reason, detail):
        self._status.publish(String(data=json.dumps({
            'motion_commanded': False, 'error': reason, 'detail': detail,
            'profile_state': self._profile_state}, sort_keys=True)))

    def _marker_array(self, evidence, base, evaluations, scan):
        stamp = scan.header.stamp
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        # One state per cell across all probes: occupied > unknown > free.
        rank = {FREE: 0, UNKNOWN: 1, OCCUPIED: 2}
        cells = {}
        for item in evaluations:
            for state, members in item.cells.items():
                for cell in members:
                    if rank[state] >= rank[cells.get(cell, FREE)]:
                        cells[cell] = state
        for marker_id, state in enumerate((FREE, UNKNOWN, OCCUPIED), 1):
            marker = self._marker(stamp, 'rotation_sweep_cells', marker_id,
                                  Marker.CUBE_LIST, CELL_COLORS[state])
            marker.scale.x = marker.scale.y = evidence.resolution * 0.95
            marker.scale.z = 0.01
            for (mx, my), cell_state in cells.items():
                if cell_state == state:
                    x, y = evidence.center(mx, my)
                    marker.points.append(Point(x=x, y=y, z=0.0))
            output.markers.append(marker)
        body = self._marker(stamp, 'rotation_footprint', 10,
                            Marker.LINE_STRIP, (0.2, 0.6, 1.0, 1.0))
        body.scale.x = 0.012
        cosine, sine = math.cos(base.yaw), math.sin(base.yaw)
        for px, py in self._footprint + self._footprint[:1]:
            body.points.append(Point(x=base.x + cosine * px - sine * py,
                                     y=base.y + sine * px + cosine * py,
                                     z=0.02))
        output.markers.append(body)
        for index, item in enumerate(evaluations):
            decision = item.decision
            label = self._marker(stamp, 'rotation_probe_verdicts', 20 + index,
                                 Marker.TEXT_VIEW_FACING,
                                 (0.2, 1.0, 0.2, 1.0) if decision.allowed
                                 else (1.0, 0.5, 0.2, 1.0))
            heading = base.yaw + decision.delta_yaw
            label.pose.position.x = base.x + 0.45 * math.cos(heading)
            label.pose.position.y = base.y + 0.45 * math.sin(heading)
            label.pose.position.z = 0.15
            label.scale.z = 0.05
            label.text = (
                f'{math.degrees(decision.delta_yaw):+.0f} deg: '
                f'{decision.reason or "ALLOWED"} '
                f'(unknown {decision.unknown_cells})')
            output.markers.append(label)
        return output

    def _marker(self, stamp, namespace, marker_id, kind, rgba):
        marker = Marker()
        marker.header.frame_id = self._odom_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RotationPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
