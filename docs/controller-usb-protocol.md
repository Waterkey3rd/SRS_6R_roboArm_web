# 同构控制器 USB 串口协议 v1.1

这条链路连接 **浏览器 ↔ STM32G474 同构控制器主控板**，使用主控原生 USB CDC 虚拟串口。浏览器通过 Web Serial 打开系统枚举的 COM/tty 设备，以字节流收发本协议。主机械臂仍走原 ROS 驱动与 HostComputer 协议。关节顺序统一为 J0..J5 / `joint_1`..`joint_6`，绝对逻辑关节角，单位 rad，与 ROS 反馈使用相同的零位、正方向和减速比换算。

Web Serial 的 `open()` 要求传入 `baudRate`，当前调用使用 921600、8N1、无流控。对于直接使用 USB CDC 端点收发、未桥接硬件 UART 的 G474 固件，这些是主机接口/CDC line coding 参数，通常不参与实际数据传输，不要求配置一个 921600 的 MCU UART，也不表示 USB 总线速率为 921600。固件可接受并保存 line coding；只有固件主动用这些参数配置 UART 或限制收发时才需要匹配。若使用 USB 转 UART 硬件，实际 UART 参数才必须匹配。参见 [Web Serial 文档](https://developer.chrome.com/docs/capabilities/serial) 和 [ST 的 CDC line coding 说明](https://community.st.com/stm32-mcus-60/how-to-set-usb-cdc-line-coding-33)。

v1.1 在原协议上新增 `0x0003` 位姿包，原 `0x0001` 心跳与 `0x0002` 关节角包保持不变。

网页支持纯仿真和实机接入，USB 帧格式相同，控制板无需识别网页运行环境。**mode 始终从控制板角度定义**：`0` 控制板跟随网页发来的目标；`1` 控制板主动上传。纯仿真中，mode=0 的数据源是视口当前虚拟臂（含动画插值姿态），mode=1 在界面叫“从动仿真”，由本地虚拟臂跟着控制板运动。两种情况都不依赖 ROS，也不向 ROS 发运动指令。实机接入则仍使用 ROS 实机反馈/目标入口。每次 USB 连接或网页运行环境切换都从 mode=0 开始。

| command | 包类型 | payload | 总帧长 | 方向 |
| --- | --- | --- | --- | --- |
| `0x0001` | 心跳 | `uint32 uptime_ms` + `uint8 mode` | 12 | Web → 控制板 |
| `0x0002` | 关节角 | `float32 q[6]`，rad | 31 | 跟随时 Web → 控制板；主动时 控制板 → Web |
| `0x0003` | 末端位姿 | `float32 [x,y,z,yaw,pitch,roll]`，m/rad | 31 | 跟随时 Web → 控制板；主动时 控制板 → Web |

## 帧格式

复用现有 HostComputer 帧的封装方式。所有多字节整数和 IEEE754 float32 均为 **小端**。

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 1 | 帧头 `0xA5` |
| 1 | 2 | 总帧长 uint16，包含帧头帧尾，即 payload 长度 + 7 |
| 3 | 2 | command uint16 |
| 5 | N | payload |
| 5+N | 1 | sum8 = payload 各字节之和 `& 0xFF`，只计算 payload |
| 6+N | 1 | 帧尾 `0x5A` |

仅有上面三种包。串口本身保证字节顺序；不设 ACK、重传、序号、session 或并发事务。数据错包丢弃，下一帧继续。接收端应处理半包/粘包、校验失败后逐字节重新找帧头；半帧超过 100 ms 无新字节则丢弃。

## 0x0001：Web 心跳及模式（Web → 控制板）

总帧长 **12**，payload **5** 字节，每 **100 ms** 发送，连接后和切模式时立即发送。

| payload 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | uint32 | 页面单调运行时间 ms，允许自然回绕，仅用于观察 |
| 4 | uint8 | `0` 跟随 / `1` 主动 |

控制板以最新有效心跳里的 mode 为准。上电默认跟随；非法 mode 丢弃。**超过 500 ms 未收到有效心跳**，清除本次控制授权、回到默认跟随、停止上传，并丢弃旧跟随目标；需重新收到跟随心跳与新目标后才继续跟随。跟随目标超过 300 ms 未更新时，保持当前位置，不能继续执行缓存旧目标。页面后台计时可能被浏览器限速，按超时处理即可。

示例：`uptime_ms=0x12345678, mode=1`：

```text
A5 0C 00 01 00 78 56 34 12 01 15 5A
```

## 0x0002：6 关节角（方向由模式确定）

总帧长 **31**，payload **24** 字节：

```c
// 逐个小端序写入，不依赖 C 结构体对齐。
float q[6];  // q[0]..q[5], rad
```

- **跟随 mode=0**：Web → 控制板。最近收到控制板的 `0x0002` 后，Web 最多每 50 ms 发送一份关节角：纯仿真来自当前显示的虚拟臂，实机接入来自新鲜 ROS 反馈。控制板使用此目标使同构机械臂跟随。跟随时控制板上传仅声明类型，不驱动网页/主机械臂。
- **主动 mode=1**：控制板 → Web。收到主动心跳后，控制板释放跟随位置控制，按自身项目的示教/手动策略运行，每 50 ms 上传实际电机关节角。Web 停止向控制板发送角度；纯仿真直接更新虚拟臂，实机接入转发到 ROS 当前节点的目标入口。只传 6 个角度，不包含夹爪。

上电或链路失效时不要直接执行零角度；只使用有效授权下新收到的目标。非有限数值（NaN/Inf）或长度不匹配的包丢弃。

## 0x0003：末端位姿（方向由模式确定）

总帧长 **31**，payload **24** 字节，每 50 ms 一包。6 个小端 float32 的定义：

| payload 偏移 | 字段 | 单位 |
| --- | --- | --- |
| 0 | x | m |
| 4 | y | m |
| 8 | z | m |
| 12 | yaw | rad |
| 16 | pitch | rad |
| 20 | roll | rad |

使用**机械臂基座系下的绝对 TCP 位姿**，基座 +X 向前、+Y 向左、+Z 向上。旋转矩阵为 `Rz(yaw) * Ry(pitch) * Rx(roll)`；不是增量、工具坐标增量或四元数。例 `[0.25, 0, 0.5, 0, 0, 0]` 的完整帧：

```text
A5 1F 00 03 00 00 00 80 3E 00 00 00 00 00 00 00 3F 00 00 00 00 00 00 00 00 00 00 00 00 FD 5A
```

- **跟随 mode=0**：最近收到控制板的 `0x0003` 后，Web → 控制板的反馈也使用 `0x0003`。纯仿真取当前显示的虚拟臂关节角，实机接入取最近 300 ms 内的 ROS 实际关节角，通过当前 6R 模型做 FK，位置从 mm 转成 m，姿态转为 yaw/pitch/roll（rad）。实机接入不以仿真值代替真实反馈。控制板用此绝对位姿驱动本机跟随。
- **主动 mode=1**：控制板 → Web，上传本机实际 TCP 位姿。纯仿真在本地将位置 m 转成 mm，使用当前 6R WASM IK 解出关节角并更新虚拟臂；实机接入转发到 ROS 位姿目标入口。

从动仿真以虚拟臂当前角度作 IK 种子，不记录实时上传为历史动作。超出关节限位或 IK 无解时保持上一有效姿态并提示。点击切回跟随、STOP 或 USB 断开后不再应用上传。ROS 断线或 MCU 事件不改变纯仿真的 USB 模式；实机接入仍要求 ROS 就绪。模式切换不会增加任何 USB 类型切换包。

## 数据类型识别与转发

**跟随/主动模式与关节角/位姿类型分别处理**：Web 心跳中的 mode 仍只表示跟随或主动。上传数据类型由控制板本地选择，发送 `0x0002` 或 `0x0003` 即可；Web 不发送任何 `changeMode`、类型选择或类型探测包，心跳里不增加类型字段。

**反馈类型与控制板最近有效包匹配**：收到 `0x0002`，跟随反馈发 `0x0002`；收到 `0x0003`，跟随反馈发 `0x0003`。首次连接尚未收到类型时只发心跳，不默认发送关节角。控制板收到首次心跳后，即使处于跟随，也应先发一份当前关节角或当前位姿声明类型；本地切换类型时再主动发送一帧即可。可在跟随时低频重复声明，以便网页重新识别。此类声明在跟随模式只用于选取反馈格式，不使主机械臂运动。

网页点击“识别数据类型”后，等待下一份校验、长度、有限值检查均通过的数据包；显示“关节角（rad）”或“位姿（m/rad）”。2 秒未收到则提示重试。心跳、乱码和坏包不参与识别。此按钮可以在 ROS 未连接或跟随时使用，但控制板必须自行发送至少一份数据才有识别结果；识别本身不会授权运动。后续控制板切换上传类型，网页自动更新显示，并始终按每个包自己的 command 路由。跟随时即使收到了上传包，也只识别、不下发 ROS。

重新识别等待期间及识别超时后，已知的反馈类型保持，不回退为关节角。主动切回跟随时也保留最近类型；USB 断开/重连则清除，等待新控制板重新声明。实机接入下，ROS 反馈过期或未连接时两种跟随反馈都停止，只保留心跳；纯仿真使用本地当前姿态，无需 ROS。

跟随/主动切换顺序由 Web 保证：先发送模式心跳，再按新方向发送/接收目标。Web 接收主动数据最多合计转发 20 Hz，超过频率的包直接丢弃，无队列/追赶。关节角与位姿共享限频。默认 ROS `web_bridge` 入口如下：

```text
topic: /arm_web/servo_target (std_msgs/msg/String)
data: {"client_id":"递增数字", "arm_mask":1, "duration_ms":100,
       "right":[q0,q1,q2,q3,q4,q5], "source":"controller_usb"}

// 位姿使用同一 topic，在 JSON 中标识类型，right 数组保持 m/rad：
data: {"client_id":"递增数字", "arm_mask":1, "duration_ms":100,
       "right":[x,y,z,yaw,pitch,roll], "space":"CartesianSpace",
       "reference_frame":0, "source":"controller_usb"}
```

实机接入时，现有 `eng26_web_bridge` 节点依据 `space` 发布到 `/arm_driver/servo_target`（省略或 `JointAngleSpace`）或已有的 `/arm_driver/cartesian_jog`（`CartesianSpace`），补充同一套 session/epoch。驱动转成 MCU `0x20` / `0x22`；位姿到达 MCU 时才转换 m → mm。仍使用既有状态检查，ROS 未就绪时不下发，界面恢复跟随。

外部 `ros2_real` profile 使用原 `/joint_command`（sensor_msgs/msg/JointState），名称为 `joint_1`..`joint_6`。该入口仅接受关节角，位姿由网页当前 6R 模型以实机反馈作种子逆解后发布，和网页原有笛卡尔操作采用相同路径。

## 控制板发送打包示例

下面的函数可直接用于 C 主控代码，`float` 需为 IEEE754 32 位。调用处先检查 6 个值均有限，再选择 `cmd=0x0002` 或 `cmd=0x0003`；数据包类型由控制板自行决定。

```c
#include <stdint.h>
#include <string.h>

static void pack_target(uint8_t out[31], uint16_t cmd, const float values[6]) {
    out[0] = 0xA5;
    out[1] = 31; out[2] = 0;
    out[3] = (uint8_t)cmd; out[4] = (uint8_t)(cmd >> 8);
    uint8_t sum = 0;
    for (unsigned i = 0; i < 6; ++i) {
        uint32_t bits;
        memcpy(&bits, &values[i], 4);
        for (unsigned j = 0; j < 4; ++j) {
            uint8_t b = (uint8_t)(bits >> (8 * j));
            out[5 + 4 * i + j] = b;
            sum = (uint8_t)(sum + b);
        }
    }
    out[29] = sum;
    out[30] = 0x5A;
}

// uint8_t tx[31];
// pack_target(tx, 0x0002, joint_angles_rad);
// 或 pack_target(tx, 0x0003, tcp_pose_m_rad);
// usb_write(tx, 31);  // 保证 tx 在异步发送完成前有效
```

## 对端主控实现参考

下面只说明协议状态逻辑。`apply_follow_target`、`hold_current`、`leave_follow_control` 接入同构控制板自己的控制流程。

```c
// tick: 本机单调毫秒。超时判断用 uint32 差值，兼容回绕。
uint8_t mode = 0;
uint16_t upload_command = 0x0002; // 控制板本地切换成 0x0003 即上传位姿
bool authorized = false, have_target = false;
bool announce_pending = true; // 本地切换 upload_command 时也置 true
uint32_t heartbeat_at, target_at, upload_at;

on_heartbeat(uint32_t tick, uint8_t new_mode) {
    if (new_mode > 1) return;
    if (!authorized) announce_pending = true;
    if (!authorized || mode != new_mode) {
        have_target = false;
        leave_follow_control();
    }
    mode = new_mode;
    authorized = true;
    heartbeat_at = tick;
}

on_target_packet(uint32_t tick, uint16_t command, const float values[6]) {
    if (!authorized || mode != 0 || command != upload_command || !all_finite(values, 6)) return;
    if (command == 0x0003) apply_follow_pose_m_rad(values);
    else if (command == 0x0002) apply_follow_joint_rad(values);
    else return;
    target_at = tick;
    have_target = true;
}

periodic(uint32_t tick) {
    if (authorized && (uint32_t)(tick - heartbeat_at) > 500) {
        authorized = false;
        mode = 0;
        have_target = false;
        hold_current();
    }
    if (!authorized) return;
    if (mode == 0 && have_target && (uint32_t)(tick - target_at) > 300) {
        have_target = false;
        hold_current();
    }
    if (announce_pending || (mode == 1 && (uint32_t)(tick - upload_at) >= 50)) {
        if (upload_command == 0x0003)
            send_pose_packet(actual_tcp_pose_m_rad());
        else
            send_joint_packet(actual_joint_angles_rad());
        upload_at = tick;
        announce_pending = false;
    }
}
```

编码/解码的网页实现位于 `web/src/serial/controller-protocol.ts`，串口与转发逻辑位于 `web/src/serial/homologous-controller.ts`。协议使用独立串口，不能把此处的 5 字节模式心跳发到主机械臂现有 4 字节心跳入口。
