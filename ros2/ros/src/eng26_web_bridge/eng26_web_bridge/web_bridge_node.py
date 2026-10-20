"""Guard and JSON bridge between the WebView and eng26_arm_driver.

Browser-facing contract:
  * /arm_web/joint_state      sensor_msgs/JointState (reliable relay)
  * /arm_web/driver_state     std_msgs/String JSON
  * /arm_web/command_result   std_msgs/String JSON
  * /arm_web/event            std_msgs/String JSON
  * /arm_web/servo_target     std_msgs/String JSON -> joint servo or Cartesian jog
  * /arm_web/stop             std_srvs/Trigger   -> /arm_driver/stop
  * /arm_web/hold             std_srvs/Trigger   -> /arm_driver/command HOLD

The node deliberately only exposes standard ROS message types to rosbridge.
That avoids custom-IDL lookup in the bridge process and fixes the
sensor_msgs/JointState best-effort -> reliable QoS transformation.
"""

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from eng26_arm_interfaces.msg import CommandResult
from eng26_arm_interfaces.msg import DriverState
from eng26_arm_interfaces.msg import JointCommand
from eng26_arm_interfaces.msg import CartesianJog
from eng26_arm_interfaces.msg import RobotEvent
from eng26_arm_interfaces.srv import Command
from eng26_arm_interfaces.srv import Stop

MODE_NAMES = {
    0: 'IDLE',
    1: 'REMOTE',
    2: 'PC_DEBUG',
    3: 'ROS_TRAJECTORY',
    4: 'VLA',
    5: 'HOMING',
    6: 'FAULT',
    7: 'ESTOP',
}

COMMAND_SERVO = 0x20
COMMAND_CARTESIAN = 0x22
COMMAND_HOLD = 48
OUTCOME_REJECTED = 1
MCU_RESULT_UNAVAILABLE = 255

SERVO_COMMAND_NAME = 'Servo'


class ArmWebBridge(Node):
    def __init__(self) -> None:
        super().__init__('eng26_web_bridge')

        self._driver_state_topic = self.declare_parameter(
            'driver_state_topic', '/arm_driver/state').value
        self._joint_state_topic = self.declare_parameter(
            'joint_state_topic', '/joint_states').value
        self._servo_topic = self.declare_parameter(
            'servo_topic', '/arm_driver/servo_target').value
        self._cartesian_topic = self.declare_parameter(
            'cartesian_topic', '/arm_driver/cartesian_jog').value
        self._command_result_topic = self.declare_parameter(
            'command_result_topic', '/arm_driver/command_results').value
        self._event_topic = self.declare_parameter(
            'event_topic', '/arm_driver/events').value
        self._command_service = self.declare_parameter(
            'command_service', '/arm_driver/command').value
        self._stop_service = self.declare_parameter(
            'stop_service', '/arm_driver/stop').value

        self._joint_out_topic = self.declare_parameter(
            'joint_out_topic', '/arm_web/joint_state').value
        self._state_out_topic = self.declare_parameter(
            'state_out_topic', '/arm_web/driver_state').value
        self._result_out_topic = self.declare_parameter(
            'result_out_topic', '/arm_web/command_result').value
        self._event_out_topic = self.declare_parameter(
            'event_out_topic', '/arm_web/event').value
        self._servo_in_topic = self.declare_parameter(
            'servo_in_topic', '/arm_web/servo_target').value

        self._state_fresh_ms = int(self.declare_parameter('state_fresh_ms', 300).value)
        self._joint_publish_hz = max(1.0, float(self.declare_parameter('joint_publish_hz', 30.0).value))
        self._command_min_duration_ms = int(self.declare_parameter('command_min_duration_ms', 20).value)
        self._command_max_duration_ms = int(self.declare_parameter('command_max_duration_ms', 60000).value)
        self._servo_publish_limit_hz = max(1.0, float(self.declare_parameter('servo_publish_limit_hz', 50.0).value))

        reliable_depth_1 = self._qos(1, DurabilityPolicy.TRANSIENT_LOCAL)
        reliable_depth_10 = self._qos(10)
        reliable_depth_64 = self._qos(64)
        best_effort_depth_1 = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._joint_pub = self.create_publisher(JointState, self._joint_out_topic, reliable_depth_10)
        self._state_pub = self.create_publisher(String, self._state_out_topic, reliable_depth_1)
        self._result_pub = self.create_publisher(String, self._result_out_topic, reliable_depth_64)
        self._event_pub = self.create_publisher(String, self._event_out_topic, reliable_depth_64)
        self._servo_pub = self.create_publisher(JointCommand, self._servo_topic, best_effort_depth_1)
        self._cartesian_pub = self.create_publisher(CartesianJog, self._cartesian_topic, best_effort_depth_1)

        self._stop_client = self.create_client(Stop, self._stop_service)
        self._command_client = self.create_client(Command, self._command_service)

        self.create_subscription(
            DriverState, self._driver_state_topic, self._on_driver_state, reliable_depth_1)
        self.create_subscription(
            JointState, self._joint_state_topic, self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(
            CommandResult, self._command_result_topic, self._on_command_result, reliable_depth_64)
        self.create_subscription(
            RobotEvent, self._event_topic, self._on_event, reliable_depth_64)
        self.create_subscription(
            String, self._servo_in_topic, self._on_servo_json, reliable_depth_10)

        self.create_service(Trigger, '/arm_web/stop', self._on_stop)
        self.create_service(Trigger, '/arm_web/hold', self._on_hold)

        self._driver_state: Optional[DriverState] = None
        self._driver_state_received_ns = 0
        self._joint_state: Optional[JointState] = None
        self._joint_dirty = False
        self._last_servo_publish_ns = 0
        self._stop_client_id = 1

        self.create_timer(1.0 / self._joint_publish_hz, self._publish_joint_tick)

        self.get_logger().info(
            'eng26_web_bridge ready: %s -> %s, %s -> %s'
            % (self._servo_in_topic, self._servo_topic,
               self._joint_state_topic, self._joint_out_topic))

    @staticmethod
    def _qos(depth: int, durability: DurabilityPolicy = DurabilityPolicy.VOLATILE) -> QoSProfile:
        return QoSProfile(
            depth=depth,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=durability,
        )

    # ------------------------------------------------------------------
    # Driver -> Web
    # ------------------------------------------------------------------
    def _on_driver_state(self, message: DriverState) -> None:
        self._driver_state = message
        self._driver_state_received_ns = self.get_clock().now().nanoseconds
        payload = {
            'stamp_ns': self._stamp_to_ns(message.header.stamp),
            'received_at_ns': str(self._driver_state_received_ns),
            'connected': bool(message.connected),
            'mode': int(message.mode),
            'mode_name': MODE_NAMES.get(int(message.mode), 'UNKNOWN'),
            'session_id': str(message.session_id),
            'mode_epoch': int(message.mode_epoch),
            'heartbeat_fresh': bool(message.heartbeat_fresh),
            'mode_fresh': bool(message.mode_fresh),
            'peer_online': bool(message.peer_online),
            'control_blocked': bool(message.control_blocked),
            'snapshot_valid': bool(message.snapshot_valid),
            'joint_state_fresh': bool(message.joint_state_fresh),
            'motion_ready': bool(message.motion_ready),
            'rearm_required': bool(message.rearm_required),
            'safety_flags': int(message.safety_flags),
            'fault_code': int(message.fault_code),
            'heartbeat_age_ms': int(message.heartbeat_age_ms),
            'mode_age_ms': int(message.mode_age_ms),
            'joint_state_age_ms': int(message.joint_state_age_ms),
            'right_gripper_state': int(message.right_gripper_state),
            'trajectory_reserved': bool(message.trajectory_reserved),
            'trajectory_id': int(message.trajectory_id),
            'last_error': str(message.last_error),
        }
        self._publish_json(self._state_pub, payload)

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_state = message
        self._joint_dirty = True

    def _publish_joint_tick(self) -> None:
        if not self._joint_dirty or self._joint_state is None:
            return
        source = self._joint_state
        output = JointState()
        output.header = source.header
        output.name = list(source.name)
        output.position = [float(value) for value in source.position]
        output.velocity = [float(value) for value in source.velocity]
        output.effort = [float(value) for value in source.effort]
        self._joint_pub.publish(output)
        self._joint_dirty = False

    def _on_command_result(self, message: CommandResult) -> None:
        payload = {
            'stamp_ns': self._stamp_to_ns(message.header.stamp),
            'ticket': str(message.ticket),
            'client_id': str(message.client_id),
            'session_id': str(message.session_id),
            'request_command': int(message.request_command),
            'sequence': int(message.sequence),
            'mode_epoch': int(message.mode_epoch),
            'outcome': int(message.outcome),
            'mcu_result': int(message.mcu_result),
            'detail': int(message.detail),
            'message': str(message.message),
        }
        self._publish_json(self._result_pub, payload)

    def _on_event(self, message: RobotEvent) -> None:
        payload = {
            'stamp_ns': self._stamp_to_ns(message.header.stamp),
            'session_id': str(message.session_id),
            'type': int(message.type),
            'mcu_timestamp_ms': int(message.mcu_timestamp_ms),
            'mode': int(message.mode),
            'mode_name': MODE_NAMES.get(int(message.mode), 'UNKNOWN'),
            'mode_epoch': int(message.mode_epoch),
            'detail': int(message.detail),
            'matches_current_epoch': bool(message.matches_current_epoch),
        }
        self._publish_json(self._event_pub, payload)

    # ------------------------------------------------------------------
    # Web -> Driver
    # ------------------------------------------------------------------
    def _on_servo_json(self, message: String) -> None:
        try:
            command = json.loads(message.data)
        except (TypeError, ValueError):
            self.get_logger().warn('Ignoring servo_target with invalid JSON')
            return

        error = self._validate_servo(command)
        request_command = COMMAND_CARTESIAN if isinstance(command, dict) and command.get('space') == 'CartesianSpace' else COMMAND_SERVO
        if error:
            client_id = str(command.get('client_id', '0')) if isinstance(command, dict) else '0'
            self._publish_rejected(client_id, error, request_command)
            return
        client_id = str(command.get('client_id', '0'))

        state = self._driver_state
        assert state is not None
        now_ns = self.get_clock().now().nanoseconds
        rate_limit_ns = int(1e9 / self._servo_publish_limit_hz)
        if now_ns - self._last_servo_publish_ns < rate_limit_ns:
            self._publish_rejected(client_id, 'servo_target 发送频率超过桥接节点限制', request_command)
            return

        try:
            client_id_value = int(client_id)
            if client_id_value < 0 or client_id_value > 0xFFFFFFFFFFFFFFFF:
                raise ValueError('client_id out of uint64 range')
        except (TypeError, ValueError):
            self._publish_rejected(client_id, 'client_id 不是有效的 uint64', request_command)
            return

        is_pose = request_command == COMMAND_CARTESIAN
        target = CartesianJog() if is_pose else JointCommand()
        target.context.stamp = self.get_clock().now().to_msg()
        target.context.session_id = state.session_id
        target.context.mode_epoch = state.mode_epoch
        target.context.client_id = client_id_value
        target.arm_mask = int(command['arm_mask'])
        target.duration_ms = int(command['duration_ms'])
        target.right = [float(value) for value in command['right']]

        if is_pose:
            target.reference_frame = 0
            self._cartesian_pub.publish(target)
        else:
            self._servo_pub.publish(target)
        self._last_servo_publish_ns = now_ns
        self.get_logger().debug(
            'Forwarded servo_target client_id=%s duration_ms=%d' % (client_id, target.duration_ms))

    def _validate_servo(self, command: dict) -> Optional[str]:
        if not isinstance(command, dict):
            return 'servo_target JSON 必须是对象'

        space = command.get('space', 'JointAngleSpace')
        if space not in ('JointAngleSpace', 'CartesianSpace'):
            return 'space 必须是 JointAngleSpace 或 CartesianSpace'
        if space == 'CartesianSpace' and command.get('reference_frame', 0) != 0:
            return 'CartesianSpace 只接受基座系绝对位姿 (reference_frame=0)'

        required = ('client_id', 'arm_mask', 'duration_ms', 'right',)
        for key in required:
            if key not in command:
                return 'servo_target 缺少字段 %s' % key

        try:
            arm_mask = int(command['arm_mask'])
            duration_ms = int(command['duration_ms'])
        except (TypeError, ValueError):
            return 'arm_mask/duration_ms 不是整数'

        if arm_mask != 1:
            return 'arm_mask 必须是 1'
        if duration_ms < self._command_min_duration_ms or duration_ms > self._command_max_duration_ms:
            return 'duration_ms 超出范围 [%d, %d]' % (
                self._command_min_duration_ms, self._command_max_duration_ms)

        for key in ('right',):
            values = command[key]
            if not isinstance(values, list) or len(values) != 6:
                return '%s 必须包含 6 个关节角或位姿分量' % key
            if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values):
                return '%s 包含非有限数值' % key

        state = self._driver_state
        if state is None:
            return '尚未收到 DriverState'
        age_ns = self.get_clock().now().nanoseconds - self._driver_state_received_ns
        if age_ns > self._state_fresh_ms * 1_000_000:
            return 'DriverState 已过期，拒绝下发'
        if not state.connected:
            return 'driver 串口未连接'
        if state.rearm_required:
            return 'MCU 需要重新授权 (rearm_required)'
        if state.control_blocked:
            return 'MCU control_blocked'
        if not state.snapshot_valid:
            return 'MCU 快照无效'
        if not state.motion_ready:
            return 'motion_ready=false'
        if int(state.mode) not in (2, 4):
            return '当前 MCU 模式不允许 servo_target（需要 PC_DEBUG/VLA）'
        if state.trajectory_reserved:
            return 'driver 正在处理轨迹上传'

        return None

    def _publish_rejected(self, client_id: str, reason: str, request_command: int = COMMAND_SERVO) -> None:
        state = self._driver_state
        payload = {
            'stamp_ns': str(self.get_clock().now().nanoseconds),
            'ticket': '0',
            'client_id': str(client_id),
            'session_id': str(state.session_id) if state is not None else '0',
            'request_command': request_command,
            'sequence': 0,
            'mode_epoch': int(state.mode_epoch) if state is not None else 0,
            'outcome': OUTCOME_REJECTED,
            'mcu_result': MCU_RESULT_UNAVAILABLE,
            'detail': 0,
            'message': 'web_bridge: ' + reason,
        }
        self._publish_json(self._result_pub, payload)
        self.get_logger().warn('Rejected servo_target client_id=%s: %s' % (client_id, reason))

    # ------------------------------------------------------------------
    # Triggers from Web
    # ------------------------------------------------------------------
    def _on_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._driver_state is None:
            response.success = False
            response.message = '尚未收到 DriverState，STOP 未发送'
            return response

        if not self._stop_client.service_is_ready():
            response.success = False
            response.message = 'STOP 服务不可用'
            return response

        stop_request = Stop.Request()
        stop_request.client_id = self._stop_client_id
        self._stop_client_id += 1
        future = self._stop_client.call_async(stop_request)
        future.add_done_callback(self._on_stop_future)
        response.success = True
        response.message = 'STOP 已发送；MCU 后续需重新 rearm'
        return response

    def _on_hold(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._driver_state is None:
            response.success = False
            response.message = '尚未收到 DriverState，HOLD 未发送'
            return response

        if not self._command_client.service_is_ready():
            response.success = False
            response.message = 'HOLD 服务不可用'
            return response

        state = self._driver_state
        command = Command.Request()
        command.context.stamp = self.get_clock().now().to_msg()
        command.context.session_id = state.session_id
        command.context.mode_epoch = state.mode_epoch
        command.context.client_id = int(time.time_ns() & 0xFFFFFFFFFFFFFFFF)
        command.command = COMMAND_HOLD
        command.trajectory_id = 0

        future = self._command_client.call_async(command)
        future.add_done_callback(self._on_hold_future)
        response.success = True
        response.message = 'HOLD 已发送'
        return response

    def _on_stop_future(self, future) -> None:
        try:
            result = future.result()
        except Exception as error:  # pragma: no cover - defensive logging only
            self.get_logger().warn('STOP call failed: %s' % error)
            return
        self.get_logger().info('STOP response: queued=%s message=%s' % (
            bool(result.queued), str(result.message)))

    def _on_hold_future(self, future) -> None:
        try:
            result = future.result()
        except Exception as error:  # pragma: no cover - defensive logging only
            self.get_logger().warn('HOLD call failed: %s' % error)
            return
        self.get_logger().info('HOLD response: accepted=%s message=%s' % (
            bool(result.accepted), str(result.message)))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_json(self, publisher, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        publisher.publish(message)

    @staticmethod
    def _stamp_to_ns(stamp) -> str:
        return str(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmWebBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
