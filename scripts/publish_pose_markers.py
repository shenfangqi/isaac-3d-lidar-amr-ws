#!/usr/bin/env python3
"""
Show candidate poses as RViz arrows for an operator check (Jetson host).

usage: publish_pose_markers.py X,Y,YAW_DEG,R,G,B,LABEL [...]

Publishes a latched (TRANSIENT_LOCAL) MarkerArray on
/overhead_clearance_markers, which the project RViz configuration already
displays, for 30 minutes.  The arrow root is base_footprint and the arrow
points along the heading.  Display only: it never publishes a pose, a
goal or a velocity.
"""

import math
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def markers(specs):
    array = MarkerArray()
    for index, spec in enumerate(specs):
        x, y, yaw, r, g, b, label = spec.split(',')
        x, y, yaw = float(x), float(y), math.radians(float(yaw))
        for kind in ('arrow', 'text'):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = f'pose_check_{kind}'
            marker.id = index
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = x, y
            marker.pose.orientation.z = math.sin(yaw / 2)
            marker.pose.orientation.w = math.cos(yaw / 2)
            marker.color.r, marker.color.g, marker.color.b = float(r), float(g), float(b)
            marker.color.a = 1.0
            if kind == 'arrow':
                marker.type = Marker.ARROW
                marker.scale.x, marker.scale.y, marker.scale.z = .6, .12, .12
                marker.pose.position.z = .3
            else:
                marker.type = Marker.TEXT_VIEW_FACING
                marker.text = label
                marker.scale.z = .35
                marker.pose.position.z = .9
            array.markers.append(marker)
    return array


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    rclpy.init()
    node = rclpy.create_node('pose_check_markers')
    publisher = node.create_publisher(MarkerArray, '/overhead_clearance_markers', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE))
    array = markers(sys.argv[1:])
    end = time.monotonic() + 1800
    try:
        while time.monotonic() < end:
            publisher.publish(array)
            rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
