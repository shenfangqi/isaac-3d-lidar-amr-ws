#!/usr/bin/env python3
"""
Wait until the automatic-localization stack can be armed; print its settings.

Runs once inside the navigation container (rclpy), so one DDS participant
discovers the graph once.  The startup script used to call ``ros2 param get
--no-daemon`` up to eight times, each a fresh process rediscovering the
graph; on a Jetson just booted (load 6-8) those calls failed after their ten
tries and aborted the startup (2026-10-14, twice).

Stages, all under one deadline, with progress every 10 s:

1. the manager's latched ``/automatic_localization/status`` arrives (its
   constructor finished and it is spinning) and shows WAIT_FOR_START;
2. the manager's parameter service answers ``--params``;
3. with ``--guard``: the motion guard's status topic arrives (it is
   spinning) and its node name is in the graph.  Node names are discovered
   separately from topic data and can lag it; the startup checks the name
   (2026-10-14: the status had arrived, the name had not, startup refused).

Prints one JSON line ``{"state", "params", "nodes", "seconds"}`` and exits 0, or
names the stage it was waiting for and exits 2.  Never publishes anything.
"""

import argparse
import json
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

MANAGER = '/automatic_localization_manager'
STATUS = '/automatic_localization/status'
GUARD_STATUS = '/automatic_localization/motion_status'
GUARD_NODE = '/localization_motion_guard'


def parse_args():
    """Parse the deadline, parameter names and the guard requirement."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--params', nargs='*', default=[])
    parser.add_argument('--guard', action='store_true',
                        help='also wait for the motion guard to publish its status')
    return parser.parse_args()


def parameter_value(value):
    """Return the Python value of an rcl_interfaces ParameterValue."""
    return {
        ParameterType.PARAMETER_BOOL: value.bool_value,
        ParameterType.PARAMETER_INTEGER: value.integer_value,
        ParameterType.PARAMETER_DOUBLE: value.double_value,
        ParameterType.PARAMETER_STRING: value.string_value,
    }.get(value.type)


def node_names(node):
    """Fully qualified names of the nodes in the graph, sorted."""
    return sorted(f'{namespace.rstrip("/")}/{name}'
                  for name, namespace in node.get_node_names_and_namespaces())


def main():
    """Wait for the stages in order; print the result."""
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node('automatic_localization_ready_check')
    seen = {'state': None, 'guard': False}

    def on_status(message):
        try:
            seen['state'] = json.loads(message.data).get('state')
        except (TypeError, ValueError):
            pass

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, STATUS, on_status, latched)
    node.create_subscription(String, GUARD_STATUS,
                             lambda message: seen.update(guard=True), 10)
    client = node.create_client(GetParameters, f'{MANAGER}/get_parameters')
    started = time.monotonic()
    deadline = started + args.timeout
    reported = started
    stage = 'manager status'
    params, future, code = None, None, 2
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if now - reported >= 10.0:
            print(f'READY_CHECK waiting for {stage} ({now - started:.0f} s)', flush=True)
            reported = now
        if seen['state'] is None:
            continue
        if seen['state'] != 'WAIT_FOR_START':
            stage = f'WAIT_FOR_START (manager is in {seen["state"]})'
            continue
        if params is None:
            stage = 'manager parameters'
            if future is None:
                if not client.service_is_ready():
                    continue
                request = GetParameters.Request()
                request.names = list(args.params)
                future = client.call_async(request)
            if not future.done():
                continue
            values = future.result().values if future.result() else []
            if len(values) != len(args.params):
                future = None       # not answered completely; ask again
                continue
            params = {name: parameter_value(value)
                      for name, value in zip(args.params, values)}
        if args.guard and not seen['guard']:
            stage = 'motion guard status'
            continue
        if args.guard and GUARD_NODE not in node_names(node):
            stage = 'motion guard node name'
            continue
        code = 0
        break
    if code == 0:
        print(json.dumps({'state': seen['state'], 'params': params,
                          'nodes': node_names(node),
                          'seconds': round(time.monotonic() - started, 1)},
                         sort_keys=True), flush=True)
    else:
        print(f'READY_CHECK_TIMEOUT after {args.timeout:.0f} s waiting for {stage}',
              flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
