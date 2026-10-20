"""Exercise the actual bridge command handlers without requiring a ROS install.

Only the handlers are loaded from the source AST; message and clock dependencies
are replaced so the JSON -> ROS payload and session gates remain under test.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Optional
import unittest


class JointCommand:
    def __init__(self):
        self.context = SimpleNamespace()


class CartesianJog(JointCommand):
    pass


source = Path(__file__).resolve().parents[1] / 'eng26_web_bridge/web_bridge_node.py'
tree = ast.parse(source.read_text(encoding='utf-8-sig'))
original_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ArmWebBridge')
handler_names = {'_on_servo_json', '_validate_servo', '_publish_rejected'}
handlers = [n for n in original_class.body if isinstance(n, ast.FunctionDef) and n.name in handler_names]
test_class = ast.ClassDef(name='BridgeHandlers', bases=[], keywords=[], body=handlers, decorator_list=[])
module = ast.fix_missing_locations(ast.Module(body=[test_class], type_ignores=[]))
scope = dict(json=json, math=math, Optional=Optional, String=SimpleNamespace,
             JointCommand=JointCommand, CartesianJog=CartesianJog,
             COMMAND_SERVO=0x20, COMMAND_CARTESIAN=0x22,
             OUTCOME_REJECTED=1, MCU_RESULT_UNAVAILABLE=255)
exec(compile(module, str(source), 'exec'), scope)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class WebTargetsTest(unittest.TestCase):
    def setUp(self):
        self.bridge = scope['BridgeHandlers']()
        self.now = 1_000_000_000
        self.bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=self.now, to_msg=lambda: 'test_stamp'))
        self.bridge.get_logger = lambda: SimpleNamespace(warn=lambda _: None, debug=lambda _: None)
        self.bridge._driver_state = SimpleNamespace(
            connected=True, rearm_required=False, control_blocked=False,
            snapshot_valid=True, motion_ready=True, mode=2, trajectory_reserved=False,
            session_id=42, mode_epoch=7)
        self.bridge._driver_state_received_ns = self.now
        self.bridge._state_fresh_ms = 300
        self.bridge._command_min_duration_ms = 20
        self.bridge._command_max_duration_ms = 60000
        self.bridge._servo_publish_limit_hz = 50
        self.bridge._last_servo_publish_ns = 0
        self.bridge._servo_pub = Publisher()
        self.bridge._cartesian_pub = Publisher()
        self.bridge._result_pub = Publisher()
        self.bridge._publish_json = lambda pub, data: pub.publish(data)

    def send(self, **overrides):
        payload = dict(client_id='10', arm_mask=1, duration_ms=100,
                       right=[0.25, 0, 0.5, 0, 0, 0])
        payload.update(overrides)
        self.bridge._on_servo_json(SimpleNamespace(data=json.dumps(payload)))
        return payload

    def test_existing_joint_command_is_unchanged(self):
        payload = self.send()
        target = self.bridge._servo_pub.messages[0]
        self.assertIs(type(target), JointCommand)
        self.assertEqual(target.right, payload['right'])
        self.assertEqual(target.context.session_id, 42)
        self.assertEqual(target.context.mode_epoch, 7)
        self.assertEqual(self.bridge._cartesian_pub.messages, [])

    def test_pose_passes_meters_radians_to_existing_cartesian_driver(self):
        payload = self.send(space='CartesianSpace', reference_frame=0)
        target = self.bridge._cartesian_pub.messages[0]
        self.assertIs(type(target), CartesianJog)
        self.assertEqual(target.right, payload['right'])  # no m -> mm conversion in ROS
        self.assertEqual(target.reference_frame, 0)
        self.assertEqual(target.duration_ms, 100)
        self.assertEqual(target.context.session_id, 42)
        self.assertEqual(target.context.mode_epoch, 7)
        self.assertEqual(target.context.client_id, 10)
        self.assertEqual(self.bridge._servo_pub.messages, [])

    def test_rejects_invalid_pose_and_space(self):
        for invalid in [dict(space='bad'), dict(space='CartesianSpace', reference_frame=1),
                        dict(space='CartesianSpace', right=[0] * 7),
                        dict(space='CartesianSpace', right=[float('nan')] * 6),
                        dict(space='CartesianSpace', client_id='bad')]:
            with self.subTest(invalid=invalid):
                self.send(**invalid)
                self.assertEqual(self.bridge._servo_pub.messages, [])
                self.assertEqual(self.bridge._cartesian_pub.messages, [])
                self.assertEqual(self.bridge._result_pub.messages[-1]['outcome'], 1)

    def test_pose_keeps_existing_authorization_and_freshness_gates(self):
        for field, value in [('connected', False), ('rearm_required', True),
                             ('control_blocked', True), ('snapshot_valid', False),
                             ('motion_ready', False), ('mode', 0), ('trajectory_reserved', True)]:
            with self.subTest(field=field):
                original = getattr(self.bridge._driver_state, field)
                setattr(self.bridge._driver_state, field, value)
                self.send(space='CartesianSpace')
                setattr(self.bridge._driver_state, field, original)
                self.assertEqual(self.bridge._cartesian_pub.messages, [])
                self.assertEqual(self.bridge._result_pub.messages[-1]['request_command'], 0x22)
        self.now += 301_000_000
        self.send(space='CartesianSpace')
        self.assertEqual(self.bridge._cartesian_pub.messages, [])

    def test_joint_and_pose_commands_share_rate_limit(self):
        self.send(space='CartesianSpace')
        self.send()
        self.assertEqual(len(self.bridge._cartesian_pub.messages), 1)
        self.assertEqual(self.bridge._servo_pub.messages, [])
        self.assertEqual(self.bridge._result_pub.messages[-1]['request_command'], 0x20)
        self.now += 20_000_000
        self.send()
        self.assertEqual(len(self.bridge._servo_pub.messages), 1)


if __name__ == '__main__':
    unittest.main()
