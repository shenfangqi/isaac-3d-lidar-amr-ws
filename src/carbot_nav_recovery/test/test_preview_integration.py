"""Synthetic ROS smoke check, only enabled in a dedicated test DDS domain."""

import json
import os
import time

import pytest


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '73',
                    reason='requires dedicated synthetic ROS_DOMAIN_ID=73')
def test_raw_costmap_preview_expires_and_never_publishes_velocity():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav2_msgs.msg import Costmap
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from carbot_nav_recovery.validation_visualizer import (
        RecoveryValidationVisualizer,
    )

    rclpy.init()
    preview = RecoveryValidationVisualizer()
    source = rclpy.create_node('synthetic_recovery_source')
    publisher = source.create_publisher(
        Costmap, '/local_costmap/costmap_raw', 10)
    broadcaster = TransformBroadcaster(source)
    statuses = []
    source.create_subscription(
        String, '/carbot_nav_recovery/status',
        lambda msg: statuses.append(json.loads(msg.data)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(preview)
    executor.add_node(source)

    def publish():
        stamp = source.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        grid = Costmap()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.metadata.size_x = grid.metadata.size_y = 40
        grid.metadata.resolution = 0.05
        grid.metadata.origin.position.x = -1.0
        grid.metadata.origin.position.y = -1.0
        grid.metadata.origin.orientation.w = 1.0
        grid.data = [0] * 1600
        publisher.publish(grid)

    timer = source.create_timer(0.1, publish)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if any(len(item.get('candidates', [])) == 8 for item in statuses):
                break
        complete = [item for item in statuses
                    if len(item.get('candidates', [])) == 8]
        assert complete, statuses
        assert complete[-1]['motion_eligible'] is False
        assert complete[-1]['costmap_update_verified'] is False
        assert complete[-1]['departure_verified'] is False
        assert any(item['safe'] for item in complete[-1]['candidates'])
        topics = source.get_publisher_names_and_types_by_node(
            preview.get_name(), preview.get_namespace())
        assert all('cmd_vel' not in name for name, _ in topics)
        assert all('geometry_msgs/msg/Twist' not in types
                   for _, types in topics)

        timer.cancel()
        statuses.clear()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert statuses
        assert statuses[-1]['candidates'] == []
        assert statuses[-1]['motion_eligible'] is False
    finally:
        executor.shutdown()
        source.destroy_node()
        preview.destroy_node()
        rclpy.shutdown()
