cmake_minimum_required(VERSION 3.16)
project(eng26_arm_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandContext.msg"
  "msg/JointCommand.msg"
  "msg/CartesianJog.msg"
  "msg/DriverState.msg"
  "msg/JointTelemetry.msg"
  "msg/RobotEvent.msg"
  "msg/CommandResult.msg"
  "srv/Command.srv"
  "srv/Stop.srv"
  "action/UploadTrajectory.action"
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
