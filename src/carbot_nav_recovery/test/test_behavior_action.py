"""C++ behavior fault injection, isolated DDS and a disconnected output topic."""

import os
import subprocess
import tempfile
import time

import pytest


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '73',
                    reason='requires isolated synthetic domain 73')
@pytest.mark.parametrize('mode', ['success', 'cancel', 'timeout', 'bad_command'])
def test_behavior_action_always_stops(mode):
    import rclpy
    from rclpy.action import ActionClient
    from lifecycle_msgs.srv import ChangeState
    from geometry_msgs.msg import Twist
    from carbot_recovery_interfaces.action import BoundedRecovery
    from carbot_recovery_interfaces.srv import RecoveryStep

    rclpy.init()
    node = rclpy.create_node('synthetic_behavior_driver')
    commands = []
    calls = []
    node.create_subscription(Twist, '/issue9_test/cmd_vel',
                             lambda msg: commands.append((msg.linear.x, msg.angular.z)), 100)

    def step(request, response):
        calls.append(request.operation)
        if request.operation == RecoveryStep.Request.STOP:
            response.status = RecoveryStep.Response.FAILED
            response.reason = 'STOPPED'
            return response
        ticks = calls.count(RecoveryStep.Request.TICK)
        if mode == 'timeout' and ticks >= 1:
            time.sleep(0.30)
        if mode == 'success' and ticks >= 2:
            response.status = RecoveryStep.Response.SUCCEEDED
            response.reason = 'REPLAN_ORIGINAL_GOAL'
        else:
            response.status = RecoveryStep.Response.RUNNING
            response.command.linear.x = 0.2 if mode == 'bad_command' else 0.04
        return response

    node.create_service(RecoveryStep, '/carbot_nav_recovery/step', step)
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([
        '/opt/ros/humble/lib/nav2_behaviors/behavior_server', '--ros-args',
        '-p', 'behavior_plugins:=[bounded_recovery]',
        '-p', 'bounded_recovery.plugin:=carbot_recovery_plugins/BoundedRecovery',
        '-p', 'bounded_recovery.execution_enabled:=true',
        '-r', 'cmd_vel:=/issue9_test/cmd_vel',
    ], stdout=log, stderr=log)

    def spin_until(predicate, seconds=10):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline and not predicate():
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate()

    try:
        lifecycle = node.create_client(ChangeState, '/behavior_server/change_state')
        spin_until(lifecycle.service_is_ready)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            spin_until(future.done)
            assert future.result().success
        client = ActionClient(node, BoundedRecovery, '/bounded_recovery')
        spin_until(client.server_is_ready)
        future = client.send_goal_async(BoundedRecovery.Goal())
        spin_until(future.done)
        handle = future.result()
        assert handle.accepted
        result_future = handle.get_result_async()
        if mode == 'cancel':
            spin_until(lambda: any(v != (0, 0) for v in commands))
            cancel = handle.cancel_goal_async()
            spin_until(cancel.done)
        spin_until(result_future.done)
        result = result_future.result()
        assert result.result.recovered is (mode == 'success')
        assert result.status == (4 if mode == 'success' else 5 if mode == 'cancel' else 6)
        spin_until(lambda: commands and commands[-1] == (0, 0))
        assert RecoveryStep.Request.STOP in calls
        if mode == 'bad_command':
            assert all(v == (0, 0) for v in commands)
        else:
            assert any(v == (0.04, 0) for v in commands)
    except Exception:
        log.seek(0)
        print(log.read())
        raise
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
