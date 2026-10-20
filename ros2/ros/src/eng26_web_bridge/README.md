# 六轴 Web bridge

节点 `eng26_web_bridge` 对接当前 `eng26_arm_driver`，使用标准 ROS 类型供 rosbridge 访问。

- `/arm_web/joint_state`：6 关节 `sensor_msgs/JointState`。
- `/arm_web/driver_state`、`/arm_web/command_result`、`/arm_web/event`：`std_msgs/String` JSON。
- `/arm_web/servo_target`：JSON `{"client_id":"1","arm_mask":1,"duration_ms":800,"right":[0,0,0,0,0,0]}`，角度为 rad。仅支持单臂 6 个有限值。
- 同一 `/arm_web/servo_target` JSON 可加 `"space":"CartesianSpace","reference_frame":0`，此时 `right=[x,y,z,yaw,pitch,roll]` 为基座系绝对 TCP 位姿（m/rad），节点改发既有 `/arm_driver/cartesian_jog`（`CartesianJog`）。省略 space 或 `JointAngleSpace` 时保持原关节目标路径。`cartesian_topic` 参数可配置对应驱动 topic，默认值见 config。
- `/arm_web/stop` 与 `/arm_web/hold`：`std_srvs/Trigger` 服务，转交会话驱动执行，不绕过其授权。

节点补充当前 session/epoch 和本地命令时间，拒绝过期状态、未授权模式及错误维度；ACK 为通信层接受结果。启动方式和完整合同见 [总 README](../../../../README.md)。

无 ROS 环境下运行 `python test/test_web_targets.py`，覆盖关节/位姿分流、m/rad 保持、session/epoch、拒绝条件和共享限频。
