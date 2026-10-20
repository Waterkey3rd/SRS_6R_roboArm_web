# Engineer2026 单臂 6R Web / ROS2 上位机

目录已由原 7R 名称迁移为 `SRS_6R_roboArm_web`，当前模型、协议、界面和默认启动链路均适配本仓库的单臂 6 轴项目。关节顺序是 MCU J0..J5，ROS 默认名为 `joint_1`..`joint_6`。协议及动作 JSON 中的历史字段 `right` 表示当前唯一机械臂，mask 仅允许 1。

## 运行 Web

```powershell
cd SRS_6R_roboArm_web/web
npm ci --cache ./.npm-cache
npm run dev
```

打开 `http://localhost:5173`。纯仿真模式可离线使用 3D 视口、6 个关节控件、笛卡尔/工具系增量移动、公式轨迹、历史编辑和动作 JSON 导入导出。示例 `actionsequence.json` 与 `web/public/demo_dance.json` 为新的 6 轴序列。旧 7 轴/双臂序列需依据当前模型重新生成，导入会明确拒绝。

```powershell
npm test
npm run build
```

生产输出为 `web/dist/`。WASM 的 ABI 模型版本为 2；前端拒绝旧版 7R 二进制。浏览器模板使用独立的 `engineer2026.arm6` 存储键，避免载入旧模型预设。

## 同构控制器 USB：跟随 / 主动 / 从动仿真

网页顶部增加“同构控制器”一栏。点击“选择 USB 串口”，在浏览器选择接入本机的控制板串口，连接参数为 **921600、8N1、无流控**。新连接始终从跟随开始，模式不写入本地存储。USB 使用桌面 Chrome/Edge 的 Web Serial API，请通过 `http://localhost` 或 HTTPS 打开网页；直接访问远程机器的普通 HTTP 页面不能使用此功能。参见 [Chrome Web Serial 文档](https://developer.chrome.com/docs/capabilities/serial)。同构控制器串口由浏览器独占，主机械臂串口继续由现有 ROS 驱动管理。

当前同构控制器为 **STM32G474 原生 USB CDC 虚拟串口**。上面的 921600/8N1 是 Web Serial 打开端口时的主机参数；直接 USB CDC 固件通常可忽略其 line coding，不需要配置 MCU 硬件 UART，也不会把 USB 实际传输速度限制为 921600。网页现有收发实现支持该虚拟 COM 设备，应用层仍按 A5/5A 二进制帧解析。

| 网页运行模式 | USB 模式 | 心跳 mode | 数据流向 |
| --- | --- | --- | --- |
| 纯仿真 | 跟随（默认） | 0 | 视口当前虚拟臂关节角 / TCP 位姿 → USB 控制板 |
| 纯仿真 | 从动仿真（点击按钮） | 1 | USB 控制板关节角 / 位姿 → 本地虚拟臂 |
| 实机接入 | 跟随（默认） | 0 | ROS 实机反馈 → Web → USB 控制板 |
| 实机接入 | 主动（点击按钮） | 1 | USB 控制板关节角 / 位姿 → Web → 现有 ROS 目标入口 → 实机 |

**纯仿真可在 ROS 未连接时直接使用 USB**，即使后台 ROS 已连接，纯仿真也只走本地数据源。连接默认“跟随”，控制板声明类型后，最多 20 Hz 收到视口中虚拟臂当前显示的关节角或 TCP 位姿，包含动作播放中的插值姿态。点击“跟随 · 点击从动仿真”后，虚拟臂跟着控制板运动：`0x0002` 直接应用 6 个 rad 关节角，`0x0003` 将 m 转成内部 mm 后，使用当前 6R WASM IK、以虚拟臂当前关节角作种子逆解。超限角度、不可达位姿保留上一有效姿态并提示。实时数据不追加历史点；进入从动仿真时停止网页动画/播放，网页执行、回退、重置和时间轴预览暂停，切回跟随后恢复。断开 USB 后保持最后姿态。纯仿真连接 USB 后可点击 STOP，在本地停止并恢复跟随，不调用 ROS。

实机接入的跟随反馈格式匹配控制板最近发来的有效数据类型：`0x0002` 回传真实关节角（rad）；`0x0003` 则用真实关节角做当前 6R 模型 FK，回传实际 TCP 位姿 `[x,y,z,yaw,pitch,roll]`（m/rad，位置由 mm 转 m）。来源为最近 300 ms 内的 ROS 实机反馈，默认 `/arm_web/joint_state`，`ros2_real` profile 为 `/joint_states`，最多 20 Hz。没有新鲜反馈时只发送心跳；实机接入不会用浏览器仿真值代替实机反馈。

首次连接未确定类型时只发心跳。控制板收到首次心跳后，应主动发送一份当前关节角或位姿声明类型，即使当前是跟随模式；本地切换类型时再上传一帧即可，网页不请求切换。重新识别或主动切回跟随会保留已知反馈类型，USB 重连后重新识别。界面会显示当前回传的是关节角还是 TCP 位姿。

主动前请切换“实机接入”，连接 ROS 并等待实机就绪，结束网页正在执行的动作，再点击“跟随 · 点击主动”。控制板可以用 `0x0002` 上传 6 个绝对关节角（rad），或用 `0x0003` 上传基座系绝对 TCP 位姿 `[x,y,z,yaw,pitch,roll]`（m/rad）。两者都复用 `/arm_web/servo_target`：位姿 JSON 标识 `space=CartesianSpace`，仍由现有 `eng26_web_bridge` 补充 session/epoch，并分发到现有驱动的关节/CartesianJog 入口。目标 `duration_ms=100`，合计最多 20 Hz。`ros2_real` profile 的位姿经当前 Web 6R IK 转成关节角，仍发原 `/joint_command`。主动时停止向控制板回传 ROS 关节角，网页手动动作与时间轴下发暂停。再次点击按钮恢复跟随。

“识别数据类型”按钮只等待下一份有效 USB 上传，显示关节角或位姿，2 秒未收到提示重试。控制板自行选择上传类型，网页不发送类型选择/`changeMode` 信息，原跟随/主动心跳保持不变。控制板后续切换类型时界面也会自动更新；识别可独立于 ROS 连接使用，跟随时只识别、不转发为运动目标。

USB 重连、切换纯仿真/实机接入、点击 STOP 都会回到跟随；实机接入还会在 ROS 未就绪或 MCU 事件时恢复跟随，实机 STOP 服务仍走原链路。纯仿真不受 ROS 连接状态或 MCU 事件影响。USB 断开后，对端依靠心跳超时退出主动授权。数据只取当前包，不缓存离线指令。协议帧、字节序、校验和与主控板伪代码见 [USB 同构控制器协议](docs/controller-usb-protocol.md)。

## 网页固定间隔录包

顶部“离散点录包”选择采样间隔（50 / 100 / 200 / 500 / 1000 ms，默认 100 ms），点击“开始录包”，结束时点击“停止录包并保存”。无需停用同构控制器主动模式；实机主动、从动仿真、普通仿真播放都可录制。纯仿真记录视口当前显示的关节角（包括插值或 USB 位姿 IK 结果），实机接入记录新鲜的 ROS 实际关节反馈，避免把控制器目标误当成电机实际到位状态。

录制开始立即记录起点，此后按固定间隔采样。保存时统一生成关节角空间的离散点（rad），每段时长为所选间隔，追加到现有时间轴，不改变实时姿态。空时间轴以录制首点作为初始帧；已有序列会先追加一个相同时长的入场点，再连接后续采样。少于两个采样点不生成动作。通过“动作序列 IO”导出 `performance-action-sequence-v2` JSON，可再次导入、编辑或回放；实际回放仍遵守原有速度/加速度限制，必要时会延长运动时长。

切换纯仿真/实机模式、退出录制开始时的 USB 主动模式（含 USB 断开）、STOP、实机反馈过期（300 ms）或 MCU 故障/模式事件时，自动结束并保存已有有效点。浏览器错过一个完整采样周期时停止录制，避免后台恢复后用重复姿态补齐不存在的运动；请保持页面在前台。单次最多 10000 个采样点，录制中不发送额外 USB 包，也不改变现有 ROS 指令链路。

## 动作序列范围选择、删除与复制粘贴

在底部时间轴，第一次 **Ctrl+点击** 选定范围起点，再 **Ctrl+点击** 终点，选中包含两端的全部动作点；支持从右向左选择。普通点击保持原来的姿态预览行为，同时清除范围，不能作为范围起点。范围高亮与播放进度分别保存，播放更新不会改变已选范围。按 Esc 或“取消选择”清除范围后，可重新选择一组端点。

- **Delete**：一次删除选中的全部动作点。
- **Ctrl+C**：复制所选范围，保留顺序、目标类型、时长、超时与标签；复制本身不修改动作序列。
- **Ctrl+V**：插入当前选中范围之后；也可先普通点击目标动作点，再粘贴到该点之后。空序列可直接粘贴，无选中范围也无当前点时追加到末尾。粘贴后自动选中新插入的范围，支持重复粘贴。

剪贴板使用现有 `performance-action-sequence-v2` JSON，可跨页面复制粘贴或在动作序列 IO 使用；同一页面内还保留编辑器的函数来源信息。删除与粘贴会重算后续动作的起始状态和笛卡尔逆解，失败时保留完整原序列。输入框、文本框及弹窗继续使用原生复制/粘贴/Delete。实机模式、USB 从动仿真和正在录制时沿用动作编辑限制，复制仍可使用；请切回可编辑的纯仿真状态后删除或粘贴。

## 模型与单位

- WASM 直接编译主工程 `App/Arm/Lib/myfkine26pro`、`smartikine26pro` 和 `myikine26pro`；smart IK 优先、普通 IK 回退并折叠到最近等价解。
- 3D 连杆使用相同 Modified DH：300 mm 上臂、266 mm 前臂段、174 mm 末端段；关节偏移为 `[0, 3π/4, -π/4, 0, 0, 0]`，基座 +X 向前、+Y 向左、+Z 向上。
- `deploy_arm_model.hpp` 的关节限位与 `App/Arm/Src/ArmMiddleware.cpp` 对齐；限位更改时需同步检查并重建 WASM。
- UI 的角度为 degree，内部/ROS/串口关节角为 rad；网页位置为 mm，串口 CartesianJog 位置为 m，姿态均采用 yaw/pitch/roll。
- 浏览器采用五次曲线做离线预览，速度上限与当前固件对齐。MCU 实际跟踪仍采用反馈驱动的分段对数速度规划；网页预览不承诺与实机逐时刻一致。

WASM 源码依赖本仓库根目录的 `App/Arm/Lib`，请从完整仓库构建：

```powershell
# emsdk 可安装到主仓库 .cache/emsdk，缓存全部留在项目目录。
# 使用已有 Emscripten 时显式传入 em++.bat：
./wasm/build_wasm.ps1 -Emxx /path/to/emsdk/upstream/emscripten/em++.bat
```

生成 `web/public/wasm/deploy_ik.js` 和 `deploy_ik.wasm`，更新模型后必须同步部署二者。脚本的临时文件与编译缓存写入主仓库 `.cache/`。

## ROS2 实机部署

推荐 Linux + ROS2 Humble。首次安装 rosbridge 后，在此目录运行：

```bash
sudo apt install ros-humble-rosbridge-suite
cd ros2/ros
source /opt/ros/humble/setup.bash
colcon build --packages-up-to eng26_arm_driver eng26_web_bridge dataset_recorder
source install/setup.bash
SERIAL_PORT=/dev/ttyUSB0 BAUD=921600 bash start_web_stack.sh
```

启动链路为：

```text
浏览器 → rosbridge WebSocket :9090 → eng26_web_bridge
        → eng26_arm_driver → 串口 → MCU HostComputer
```

默认使用 `web_bridge` profile，网页地址 `http://localhost:5173/?rosProfile=web_bridge`。Web bridge 提供 `/arm_web/joint_state`、driver 状态、命令结果与事件；目标通过 `/arm_web/servo_target` JSON 发布，仅包含 `right[6]`、`arm_mask=1`、`duration_ms`、`client_id`。STOP/Hold 分别通过 `/arm_web/stop`、`/arm_web/hold` Trigger 服务转发。可用 `?rosbridge=ws://主机:9090` 指定连接地址。

驱动首次连接会发送 STOP，等待 MCU FSM 用新 epoch 重新授权；心跳恢复不会自动解锁。需要有效模式/epoch、实时反馈和 MCU 的 PcDebug/Vla 授权后才允许运动。完整协议见 [MCU 通信文档](../App/Devices/HostComputer/README.md)。主固件应用层已提供 UART10（921600、8N1）收发和 PerformanceState 的 PcDebug 消费。Ctrl+Shift+F 进入 debug，Ctrl+Shift+E 返回 Keyboard，机械臂参考姿态连续交接。STOP/断线后通过退出、重新进入 debug 建立新 epoch。

ROS `/joint_states` 仅含 6 个关节；`/gripper_state` 为长度 1 的 `UInt8MultiArray`，与遥测最后一个夹爪枚举一致。轨迹上传 Action 要求 6 个有效且不重复的关节名。原 `eng26_usb_bridge` 已退役，默认启动脚本不会启动它。`ros2_real/` 的采集启动脚本与 LeRobot 转换器也采用 6 关节 + 1 夹爪。

`?rosProfile=ros2_real` 仅保留给外部自有 `/joint_command` 节点使用，采用 6 轴 `sensor_msgs/JointState`，不提供 MCU ACK/STOP 合同；本仓库默认会话驱动使用前述 `web_bridge` profile。

## Docker 静态站点部署

```bash
docker network create web_gateway  # 网络尚不存在时执行
docker compose up -d --build
```

容器 `roboarm-web-view` 在 8080 提供静态页面，宿主机端口可用 `WEB_PORT` 修改。它使用已经生成的 6 轴 WASM，不在镜像内编译 C++，也不启动 ROS/串口驱动。网页实机模式仍需能连接 ROS 主机的 rosbridge WebSocket。生产构建输出导出到 `web/dist/`；本次未在 Docker/真实 ROS2 环境运行部署。

## 回归检查

主仓库 `tests/host_computer` 提供不依赖 ROS 的真实 MCU/PC 协议互通测试，使用测试专用的 RTOS 锁与队列替身：

```bash
cmake -S tests/host_computer -B .cache/host-tests
cmake --build .cache/host-tests
ctest --test-dir .cache/host-tests --output-on-failure
```

从主仓库根目录运行这些命令。Web 测试覆盖 6R FK/IK、连杆末端一致性、轨迹规划、历史编辑、JSON 往返、6 轴 ROS 映射和同构 USB 的帧解析、模式心跳、两类目标识别/转发、断开重连。缓存和构建输出不提交。

Web bridge 的关节/位姿分流和授权检查可独立于 ROS 环境验证：

```bash
python ros2/ros/src/eng26_web_bridge/test/test_web_targets.py
```

验证记录：Windows 原生 MCU/PC 协议互通、HostLink DMA 传输及 Debug 无缝切换回归通过，当前整机固件已生成 ELF/HEX/BIN。Web 51 项测试、Web bridge 5 项处理器测试、TypeScript 检查、应用和库生产构建通过。回归包含真实 6R WASM FK/IK 的离线关节/位姿跟随、动画中间姿态反馈、固定间隔主动录包、范围删除/复制粘贴、JSON 往返、编辑失败回滚、输入框快捷键隔离、不可达/超限目标、ROS 离线/事件隔离和模式切换。同构 USB 使用流模拟测试，尚未连接真实控制板；Linux ROS2、Docker 及真实电机联调未在此环境验证。

Git 忽略本地缓存、Node 依赖、固件/原生编译输出、Web `dist`、ROS build/install/log、录包数据和工具私有配置。`web/public/wasm/deploy_ik.js` 与 `.wasm` 是网页启动必需的运行资源，继续随源码版本管理；依赖锁文件、示例动作和模型源码也保留。Web 保留现有独立 Git 仓库，主工程提交不自动包含 Web，更新时分别保存/提交两个仓库。现有 `ros2_real` 独立仓库已在 Web 内登记为子模块，发布时先推送该子仓库，再推 Web 的对应提交。
