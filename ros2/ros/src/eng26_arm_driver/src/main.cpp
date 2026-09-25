#include "eng26_arm_driver/arm_driver_node.hpp"
#include <exception>
#include <cstdio>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<eng26::ArmDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "arm_driver_node: %s\n", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
