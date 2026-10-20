cmake_minimum_required(VERSION 3.16)
project(eng26_usb_bridge)
find_package(ament_cmake REQUIRED)
# Compatibility package only. The six-axis protocol is owned by eng26_arm_driver.
ament_package()
