#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace eng26 {
class ArmDriverNode final : public rclcpp::Node {
public:
  explicit ArmDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ArmDriverNode() override;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace eng26
