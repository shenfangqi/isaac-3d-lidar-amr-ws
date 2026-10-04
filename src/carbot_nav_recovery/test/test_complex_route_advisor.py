"""Synthetic validation for the non-actuating Issue #12 ROS adapter."""

import json
import os
import time

import pytest


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '73',
                    reason='requires dedicated synthetic ROS_DOMAIN_ID=73')
def test_validation_node_reports_calibration_gate_and_has_no_twist_publisher():
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Odometry, Path
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    from carbot_nav_recovery.complex_route_advisor import ComplexRouteAdvisor

    rclpy.init()
    advisor = ComplexRouteAdvisor()
    source = rclpy.create_node('synthetic_complex_route_source')
    path_publisher = source.create_publisher(Path, '/plan', 10)
    costmap_publisher = source.create_publisher(
        Costmap, '/local_costmap/costmap_raw', 10)
    odom_publisher = source.create_publisher(Odometry, '/odom', 10)
    broadcaster = TransformBroadcaster(source)
    reports = []
    source.create_subscription(
        String, '/carbot_nav_recovery/complex_route_advisory',
        lambda message: reports.append(json.loads(message.data)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(advisor)
    executor.add_node(source)

    def publish_inputs():
        stamp = source.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        for index in range(21):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = index * 0.05
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        path_publisher.publish(path)

        costmap = Costmap()
        costmap.header.stamp = stamp
        costmap.header.frame_id = 'map'
        costmap.metadata.size_x = costmap.metadata.size_y = 80
        costmap.metadata.resolution = 0.05
        costmap.metadata.origin.position.x = -2.0
        costmap.metadata.origin.position.y = -2.0
        costmap.metadata.origin.orientation.w = 1.0
        costmap.data = [0] * 6400
        costmap_publisher.publish(costmap)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom_publisher.publish(odom)

    timer = source.create_timer(0.1, publish_inputs)
    try:
        advisor.set_parameters([
            rclpy.parameter.Parameter(
                'localization_valid',
                rclpy.Parameter.Type.BOOL, True),
        ])
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if any(report.get('reason') == 'CALIBRATION_REQUIRED'
                   for report in reports):
                break
        accepted = [report for report in reports
                    if report.get('reason') == 'CALIBRATION_REQUIRED']
        assert accepted, reports
        assert accepted[-1]['recommended_speed_mps'] is None
        assert accepted[-1]['validation_only'] is True
        assert accepted[-1]['motion_eligible'] is False
        assert accepted[-1]['command_applied'] is False
        topics = source.get_publisher_names_and_types_by_node(
            advisor.get_name(), advisor.get_namespace())
        assert all('cmd_vel' not in name for name, _ in topics)
        assert all('geometry_msgs/msg/Twist' not in types
                   for _, types in topics)
    finally:
        timer.cancel()
        executor.shutdown()
        source.destroy_node()
        advisor.destroy_node()
        rclpy.shutdown()
