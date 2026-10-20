# 单臂 6 轴链路

`Browser -> rosbridge -> eng26_web_bridge -> eng26_arm_driver -> HostComputer`。

`eng26_arm_driver` 独占串口，Session 负责非阻塞收发、心跳、序号、ACK、重连及 epoch 门禁；ROS 适配器负责 6 关节消息映射与轨迹上传。`eng26_web_bridge` 在标准 String JSON 与自定义 ROS 消息之间转换，并校验目标、模式及反馈新鲜度。MCU FSM 保持模式权威，HostComputer 接收目标与轨迹供 FSM 消费。

协议、部署命令与接入边界见 [README](README.md) 和 [总部署文档](../../README.md)。旧 7 轴/双臂帧及旧 USB bridge 不在当前链路中。
