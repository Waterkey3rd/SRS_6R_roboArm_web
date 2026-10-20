# 旧 USB bridge 已退役

7 轴/双臂 `usb_bridge_node` 已移除。当前 6 轴链路使用 `eng26_arm_driver` 的 `arm_driver_node`，并通过 `eng26_web_bridge` 接入网页。请使用 ros2/ros/start_web_stack.sh 启动，避免旧工作区 install/ 中遗留的二进制混用。
