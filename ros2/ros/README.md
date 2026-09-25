# ENG26 ROS 2 双臂通信驱动

基于当前 `App/Devices/HostComputer` 实际线协议的 C++ 实现。架构、接口语义、并发与恢复策略见 [ARCHITECTURE.md](ARCHITECTURE.md)。**本次按要求未编译，C++ 测试尚未运行。**

源码以 Humble 公共 API 和 C++17 为基线，供 Ubuntu 22.04 实机与 26.04 开发环境分别编译。使用各环境支持的 ROS 2 发行版；不复用另一个系统编出的 `build/install`，不需要 `ros1_bridge`。

## 目录

```text
ros2/
  src/eng26_arm_interfaces/    msg、srv、UploadTrajectory.action
  src/eng26_arm_driver/
    include/eng26_arm_driver/  独立协议、Transport、Session、ROS 节点入口
    src/                      C++17 实现，组件 + 独立可执行程序
    config/driver.yaml        串口、超时与关节名称
    launch/driver.launch.py
    test/                     GTest 测试源码
```

## 在目标 Linux 环境构建

以下是后续操作说明，本次未执行。先 source 该环境已安装的 ROS 2 setup.bash，再进入本目录。rosdep 安装步骤会修改目标系统的依赖环境。

```bash
export ROS_DISTRO=humble  # 22.04 示例；26.04 使用其实际安装的发行版
source /opt/ros/$ROS_DISTRO/setup.bash
cd /path/to/eng2026_-da_-arms/ros2
export XDG_CACHE_HOME="$PWD/.cache"
export PYTHONPYCACHEPREFIX="$PWD/.cache/pycache"
export ROS_LOG_DIR="$PWD/log/runtime"
rosdep install --from-paths src --ignore-src -r -y --rosdistro "$ROS_DISTRO"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select eng26_arm_driver --event-handlers console_direct+
colcon test-result --verbose
```

两个包通过 colcon 依赖排序构建。不要把固件项目根目录加入 ROS 构建；此工作区不改变根 CMake。缓存、日志、构建和安装输出留在 `ros2/`，已配置忽略，不应提交。

## 启动与配置

配置 `config/driver.yaml` 的 `joint_names` 必须与 URDF 一致，顺序固定为右臂物理轴 1..7、左臂物理轴 1..7。driver 不做轴符号、减速比或零位变换，输入输出均假设与 MCU 的 rad 定义一致。

```bash
ros2 launch eng26_arm_driver driver.launch.py device:=/dev/serial/by-id/你的设备
# 多机器人可以使用 namespace:=robot1
# 自定义配置可以使用 params_file:=/absolute/path/driver.yaml
ros2 topic echo /arm_driver/state
ros2 topic echo /arm_driver/command_results
ros2 topic echo /joint_states --qos-reliability best_effort
```

默认 921600 / 8N1 / 无流控。使用设备稳定路径，确保当前用户有串口访问权限，避免串口终端、其他 driver 或 ModemManager 同时占用设备。端口由 `TIOCEXCL` 独占。

**打开连接会发送 STOP，并等待 MCU 提供新 epoch。** 不会主动切换机器人模式。当前固件已在 PerformanceState 接入右拨杆 middle 回零、down 进入 Vla 并消费 servo_target；RoboArmThread 持续更新双臂关节快照。先启动驱动，再完成保护/回零并切 down；若 STOP 打断保护，切 middle 重新完成保护后再切 down。轨迹执行 FSM 仍需接入。

参数在启动时读取，除 ROS 自带参数外均为只读，运行时也拒绝打开 `use_sim_time`；修改串口或时序配置需重新启动节点并重新授权。心跳默认 50 ms，MCU 超时默认 300 ms；改变主机参数不会同步修改 MCU。默认 ACK 窗口为 1，支持高频率多少取决于串口和 MCU 调度，不能把输入 topic 频率理解为实际送达频率。

组件名为 `eng26::ArmDriverNode`，插件位于 `eng26_arm_driver`；可通过标准 `LoadComposableNodes` 装载。所有私有实体随节点名/namespace 变化。`joint_states` 和 `diagnostics` 是相对名称，可 remap 到业务图需要的位置。

## 上层接入

建议 Motion Manager 独占控制输出，业务节点不要直接处理 wire seq、checksum 或分块。接口使用方式：

1. 订阅 `~/state`（可靠、transient local）和 `~/command_results`（可靠、volatile），校验状态接收新鲜度。
2. 只有 `motion_ready=true` 且模式符合当前业务时才产生新运动命令。将 state 的 `session_id`、`mode_epoch` 复制到 context，`stamp` 填实际创建时间，`client_id` 填应用关联 ID。
3. 实时 topic 每条消息只发送一次。servo 为 rad、joint_jog 为 rad/s；Cartesian 为 m/s 与 rad/s。duration_ms 非零；不在 driver 内执行 200 ms 插值。
4. session/epoch 改变时废弃业务旧目标，基于最新反馈重新规划；不要把旧目标换上新 token 重发。
5. service 的 accepted 只表示本地入队；通过 result 的 ticket/client_id 关联 ACK。ACK(Ok) 也不代表动作完成。

在上层 rclcpp 节点中的实时目标示例（发布者成员应长期保留）：

```cpp
// publisher: create_publisher<eng26_arm_interfaces::msg::JointCommand>(
//   "/arm_driver/servo_target", rclcpp::QoS(1).best_effort());
// state、q_right、q_left 应来自业务节点自己的已校验状态/规划结果。
eng26_arm_interfaces::msg::JointCommand target;
target.context.stamp = now();
target.context.session_id = state.session_id;
target.context.mode_epoch = state.mode_epoch;
target.context.client_id = ++request_id;
target.arm_mask = target.BOTH;
target.duration_ms = 200;
target.right = q_right;
target.left = q_left;
publisher->publish(target);
```

发轨迹时使用 `rclcpp_action::Client<eng26_arm_interfaces::action::UploadTrajectory>`，服务器 `/arm_driver/upload_trajectory`。goal 填 context、arm_mask 和 `trajectory_msgs/JointTrajectory`；关节名字可任意顺序，点时间必须是整数微秒，velocity/acceleration/effort 留空。原生 MoveIt 轨迹常带导数，需要上层明确转换，driver 不静默删除约束。

Action 返回 `READY` 后保存返回的 trajectory_id，并以**新创建的 context 时间戳**调用 `/arm_driver/command`，command=TRAJECTORY_START、trajectory_id=返回值。不要改变返回关联的 session/epoch；若已变化应重新规划上传。观察 Start ACK 后，再通过业务明确的 MCU 事件/反馈判断执行状态。

Hold/ClearFault 也通过 command service。Action 上传过程中若需打断，调用 Action cancel 或 STOP。以下 STOP 无需模式、心跳、session 参数：

```bash
ros2 service call /arm_driver/stop eng26_arm_interfaces/srv/Stop '{client_id: 1}'
```

STOP 返回 queued 或 WRITTEN_UNCONFIRMED 都不能代替硬件急停/停止确认。上传 Action 的 canceled 只确认本地事务取消，结果文本说明 MCU 清理仍待确认。停止进程时串口 RAII 关闭，MCU 应通过心跳超时停止运动；本节点不在退出析构中阻塞等待 STOP。

完整字段、常量直接见 [接口包](src/eng26_arm_interfaces)。`DriverState.left_gripper_state` 和 `right_gripper_state` 来自 MCU 心跳，取值 Close=0、Open=1、Free=2、Suspend=3；尚未收到有效心跳或断线后为 Unknown=255，使用前同时检查 `heartbeat_fresh`。它们是 MCU 当前夹爪参考状态，不表示夹爪已经到达目标位置。`JointTelemetry` 保留 MCU 时间戳；`JointState.header.stamp` 是 ROS 收到新快照的时间，无伪造同步。事件可能重复或来自旧 epoch，必须检查 `matches_current_epoch` 并按业务幂等处理。

## 实机联调顺序

1. 按 [架构第 6 节](ARCHITECTURE.md#6-部署前必须接上的固件业务)接好 MCU 模式、快照、STOP、命令消费和轨迹执行逻辑。
2. 首先只观察心跳、模式和 14 轴反馈，核对轴序、单位、正方向、零位与时间戳推进。
3. 验证启动 STOP、新 epoch 授权、主动 STOP、串口拔插、MCU 重启及主机进程重启后旧指令失效。
4. 在受控工况分别检查单臂/双臂 servo、jog、Hold/ClearFault 的 ACK 和实际执行。
5. 用短轨迹验证上传、显式 Start、取消与实际储存释放；再测 256 点、掉 ACK、换模式和超时。

当前固件没有规定可关联的轨迹执行完成合同，因此此包不冒充 `FollowJointTrajectory` 控制器；该标准接口留给 Motion Manager/Controller Adapter。在补齐实际开始、完成、失败、取消事件后，可利用现有上传 Action、command、events、joint_states 实现完整适配。

## WebView 实机接入

`src/eng26_web_bridge/` 是给 `7RArm-WebView` 使用的薄桥接节点：浏览器只通过
rosbridge 使用 `std_msgs/String` JSON 和 `sensor_msgs/JointState`，由该节点
完成 `/arm_driver/servo_target` 的 context 填充与安全门禁。部署命令见
`src/eng26_web_bridge/README.md` 和 Web 仓库根目录 README。
