cmake_minimum_required(VERSION 3.8)

project(eng26_usb_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================
# ROS2
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================
# libusb
# ============================================================

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ============================================================
# executable
# ============================================================

add_executable(
  usb_bridge_node
  src/ros2_usb_bridge.cpp
)

target_include_directories(
  usb_bridge_node
  PRIVATE
    include
    ${LIBUSB_INCLUDE_DIRS}
)

target_link_directories(
  usb_bridge_node
  PRIVATE
    ${LIBUSB_LIBRARY_DIRS}
)

target_link_libraries(
  usb_bridge_node
  ${LIBUSB_LIBRARIES}
)

target_compile_options(
  usb_bridge_node
  PRIVATE
    ${LIBUSB_CFLAGS_OTHER}
)

ament_target_dependencies(
  usb_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ============================================================
# install
# ============================================================

install(
  TARGETS
    usb_bridge_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)

ament_package()