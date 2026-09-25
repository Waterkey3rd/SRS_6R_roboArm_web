# eng26_web_bridge

浏览器与 `eng26_arm_driver` 之间的薄桥接节点，只向 rosbridge 暴露标准消息：

- `/arm_web/joint_state` (`sensor_msgs/JointState`，可靠转发)
- `/arm_web/driver_state`、`/arm_web/command_result`、`/arm_web/event` (`std_msgs/String` JSON)
- `/arm_web/servo_target` (`std_msgs/String` JSON -> `/arm_driver/servo_target`)
- `/arm_web/stop`、`/arm_web/hold` (`std_srvs/Trigger`)

## 运行

```bash
colcon build --symlink-install --packages-up-to eng26_arm_driver eng26_web_bridge
source install/setup.bash

ros2 launch eng26_arm_driver driver.launch.py device:=/dev/serial/by-id/your_device
ros2 launch eng26_web_bridge web_bridge.launch.py

sudo apt install ros-$ROS_DISTRO-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

WebView 默认连接 `ws://<页面host>:9090`，也可通过 `?rosbridge=ws://host:port`
或页面右上角齿轮设置。

## 注意

- 本节点只做校验、JSON 编解码和 QoS 转换；`servo_target` 的最终安全门禁仍在
  `eng26_arm_driver` 内。
- `/arm_web/servo_target` JSON 字段：`client_id`、`arm_mask`、`duration_ms`、
  `right`、`left`。
- STOP 会通过 `eng26_arm_driver` 触发 session 轮换，之后 MCU 需要重新 rearm。