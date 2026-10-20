# Engineer2026 6 轴 ROS2 工作区

当前节点为 `eng26_arm_driver`，匹配 `App/Devices/HostComputer` 的单臂 6 轴协议。`eng26_usb_bridge` 旧实现已退役。默认关节名 `joint_1`..`joint_6`，依次对应 MCU J0..J5；mask=1，字段 `right` 为唯一机械臂。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to eng26_arm_driver eng26_web_bridge dataset_recorder
source install/setup.bash
SERIAL_PORT=/dev/ttyUSB0 BAUD=921600 bash start_web_stack.sh
```

启动前在 `../../web` 安装 npm 依赖，并安装 ROS rosbridge-suite。完整部署、单位与模式授权规则见 [Web README](../../README.md)。串口节点首次连接要求 STOP + MCU 新 epoch 授权；UI 不负责切换 MCU 模式。

`/joint_states` 为 6 个 rad 位置和 rad/s 速度；`/gripper_state` 为一个夹爪枚举；Web 使用 `/arm_web/*` 合同。轨迹上传仅接受 6 关节位置序列，轨迹时间单位 us。四字节 uptime 心跳、55 字节遥测均与 MCU 对齐。
