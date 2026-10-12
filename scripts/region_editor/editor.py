#!/usr/bin/env python3
"""
Draw variable regions in RViz: drag two corner handles, right-click to save.

Shows the regions already in use (orange), the box being drawn (blue) and
the boxes saved in this session (green).  Saving writes every saved box to
--output as a variable-region YAML bound to the mesh SHA-256, ready for
scripts/check_variable_regions.py.  Boxes saved before a restart are kept.
Publishes only markers on the workstation; nothing reaches the robot.
"""

import argparse
import hashlib
import time

import rclpy
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import MarkerArray
import yaml

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def box(ns, mid, low, high, rgba, text):
    cube = Marker()
    cube.header.frame_id = 'map'
    cube.ns, cube.id, cube.type = ns, mid, Marker.CUBE
    cube.pose.orientation.w = 1.0
    cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = [
        (a + b) / 2 for a, b in zip(low, high)]
    cube.scale.x, cube.scale.y, cube.scale.z = [max(abs(b - a), .01)
                                                for a, b in zip(low, high)]
    cube.color.r, cube.color.g, cube.color.b, cube.color.a = rgba
    label = Marker()
    label.header.frame_id = 'map'
    label.ns, label.id, label.type = ns + '_label', mid, Marker.TEXT_VIEW_FACING
    label.pose.orientation.w = 1.0
    label.pose.position.x = (low[0] + high[0]) / 2
    label.pose.position.y = (low[1] + high[1]) / 2
    label.pose.position.z = max(low[2], high[2]) + .3
    label.scale.z, label.text = .25, text
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0
    return [cube, label]


def handle(name, position, color, menu_text):
    marker = InteractiveMarker()
    marker.header.frame_id = 'map'
    marker.name = name
    marker.description = f'corner {name} ({menu_text})'
    marker.scale = .5
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.w = 1.0
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.scale.x = sphere.scale.y = sphere.scale.z = .15
    sphere.color.r, sphere.color.g, sphere.color.b = color
    sphere.color.a = 1.0
    body = InteractiveMarkerControl()
    body.always_visible = True
    body.interaction_mode = InteractiveMarkerControl.MENU
    body.markers.append(sphere)
    marker.controls.append(body)
    plane = InteractiveMarkerControl()
    plane.name = 'move_xy'
    plane.orientation.w = plane.orientation.y = 0.70710678
    plane.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    marker.controls.append(plane)
    for axis, (qx, qy, qz) in (('x', (1, 0, 0)), ('y', (0, 0, 1)), ('z', (0, 1, 0))):
        control = InteractiveMarkerControl()
        control.name = 'move_' + axis
        control.orientation.w = 0.70710678
        control.orientation.x, control.orientation.y, control.orientation.z = [
            v * 0.70710678 for v in (qx, qy, qz)]
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
    return marker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', required=True, help='map mesh (.ply) the regions belong to')
    parser.add_argument('--existing', help='variable-region YAML in use (shown, not edited)')
    parser.add_argument('--output', required=True, help='YAML the saved boxes are written to')
    parser.add_argument('--start', type=float, nargs=6, default=(1., 3., 0., 1.5, 3.2, 1.8),
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'),
                        help='initial box')
    args = parser.parse_args()
    with open(args.mesh, 'rb') as stream:
        digest = hashlib.sha256(stream.read()).hexdigest()
    existing = []
    if args.existing:
        with open(args.existing, encoding='utf-8') as stream:
            existing = [(r['name'], r['min'], r['max'])
                        for r in yaml.safe_load(stream)['regions']]
    saved = []
    try:
        with open(args.output, encoding='utf-8') as stream:
            for region in (yaml.safe_load(stream) or {}).get('regions') or ():
                entry = ([float(v) for v in region['min']], [float(v) for v in region['max']])
                if entry not in saved:
                    saved.append(entry)
    except OSError:
        pass
    corners = {'low': list(args.start[:3]), 'high': list(args.start[3:])}

    rclpy.init()
    node = rclpy.create_node('region_editor_node')
    publisher = node.create_publisher(MarkerArray, '/overhead_clearance_markers', LATCHED)
    server = InteractiveMarkerServer(node, 'region_editor')
    menu = MenuHandler()

    def bounds():
        low = [min(a, b) for a, b in zip(corners['low'], corners['high'])]
        high = [max(a, b) for a, b in zip(corners['low'], corners['high'])]
        return low, high

    def redraw():
        array = MarkerArray()
        for i, (name, low, high) in enumerate(existing):
            array.markers += box('existing', i, low, high, (1., .5, 0., .5), f'in use: {name}')
        low, high = bounds()
        text = ('new region  x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f'
                % (low[0], high[0], low[1], high[1], low[2], high[2]))
        array.markers += box('new', 0, low, high, (0., .6, 1., .45), text)
        for i, (low, high) in enumerate(saved, start=1):
            array.markers += box('saved', i, low, high, (0., 1., 0., .35), f'saved {i}')
        publisher.publish(array)

    def write():
        with open(args.output, 'w', encoding='utf-8') as stream:
            stream.write(f'# saved {time.strftime("%Y-%m-%d %H:%M:%S")} in RViz\n')
            stream.write(f'mesh_sha256: {digest}\nregions:\n')
            for i, (low, high) in enumerate(saved, start=1):
                stream.write(f'  - name: operator region {i}\n'
                             f'    min: [{low[0]:.2f}, {low[1]:.2f}, {low[2]:.2f}]\n'
                             f'    max: [{high[0]:.2f}, {high[1]:.2f}, {high[2]:.2f}]\n')

    def feedback(message):
        p = message.pose.position
        corners[message.marker_name] = [p.x, p.y, p.z]
        redraw()

    def save(_message):
        saved.append(bounds())
        write()
        print(f'saved region {len(saved)}: {saved[-1]}', flush=True)
        redraw()

    def clear(_message):
        saved.clear()
        write()
        print('cleared saved regions', flush=True)
        redraw()

    menu.insert('save region', callback=save)
    menu.insert('clear saved regions', callback=clear)
    for name, color in (('low', (1., 1., 0.)), ('high', (1., 0., 1.))):
        server.insert(handle(name, corners[name], color, 'right-click: save'),
                      feedback_callback=feedback)
        menu.apply(server, name)
    server.applyChanges()
    redraw()
    node.create_timer(2.0, redraw)
    print(f'region editor ready; saving to {args.output}', flush=True)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
