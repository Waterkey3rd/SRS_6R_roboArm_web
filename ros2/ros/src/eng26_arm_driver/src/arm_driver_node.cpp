#include "eng26_arm_driver/arm_driver_node.hpp"
#include "eng26_arm_driver/session.hpp"
#include <eng26_arm_interfaces/action/upload_trajectory.hpp>
#include <eng26_arm_interfaces/msg/cartesian_jog.hpp>
#include <eng26_arm_interfaces/msg/command_result.hpp>
#include <eng26_arm_interfaces/msg/driver_state.hpp>
#include <eng26_arm_interfaces/msg/joint_command.hpp>
#include <eng26_arm_interfaces/msg/joint_telemetry.hpp>
#include <eng26_arm_interfaces/msg/robot_event.hpp>
#include <eng26_arm_interfaces/srv/command.hpp>
#include <eng26_arm_interfaces/srv/stop.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>

namespace eng26 {
namespace msg = eng26_arm_interfaces::msg;
namespace srv = eng26_arm_interfaces::srv;
using Upload = eng26_arm_interfaces::action::UploadTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<Upload>;
using wire::Command;
namespace {
uint64_t steady_ms() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
uint64_t session_seed() {
  std::random_device random;
  return (uint64_t{random()} << 32) ^ random() ^ steady_ms();
}
template<class Values> bool finite(const Values& values) {
  return std::all_of(values.begin(), values.end(), [](auto value) { return std::isfinite(value); });
}
bool valid_mask(uint8_t mask) { return mask == 1; }
struct Point { uint32_t time_us{}; std::array<float, wire::kJoints> right{}; };
}

class ArmDriverNode::Impl {
public:
  explicit Impl(ArmDriverNode& node);
  ~Impl() = default;
private:
  template<class T> T parameter(const std::string& name, const T& value, const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description; descriptor.read_only = true;
    return node_.declare_parameter<T>(name, value, descriptor);
  }
  uint64_t milliseconds(const std::string& name, int64_t value, int64_t low, int64_t high) {
    const auto result = parameter<int64_t>(name, value, "Steady-clock milliseconds; restart required to change");
    if (result < low || result > high) throw std::invalid_argument(name + " outside supported range");
    return static_cast<uint64_t>(result);
  }
  void tick();
  void telemetry(const wire::Frame& frame);
  void drain_results();
  void publish_result(const CommandResult& result);
  void reject(const msg::CommandContext& context, Command command, const std::string& reason);
  void publish_state(uint64_t now);
  void publish_diagnostics(uint64_t now);
  std::string context_error(const msg::CommandContext& context, Command command, uint64_t now) const;
  std::optional<uint64_t> command_ttl(const msg::CommandContext& context) const;
  bool joint_fresh(uint64_t now) const;
  void joint_command(const msg::JointCommand& message, Command command);
  void cartesian_command(const msg::CartesianJog& message);
  void command_service(const srv::Command::Request& request, srv::Command::Response& response);
  std::string validate_trajectory(const Upload::Goal& goal, std::vector<Point>& points) const;
  void accepted(const std::shared_ptr<GoalHandle>& goal);
  void drive_upload(uint64_t now);
  void upload_result(const CommandResult& result);
  void end_upload(uint8_t code, const std::string& text);
  void stop_upload_if_current(uint64_t now);
  void feedback();

  ArmDriverNode& node_;
  std::unique_ptr<Session> session_;
  std::vector<std::string> names_;
  std::string frame_id_, device_;
  uint64_t command_age_ms_{}, joint_timeout_ms_{};
  uint64_t last_joint_rx_{}, joint_session_{}, last_state_tx_{}, last_diagnostics_tx_{};
  uint32_t last_joint_timestamp_{};
  bool joint_seen_{};
  uint64_t observed_session_{};
  std::array<uint32_t, 7> mcu_diagnostics_{};
  bool diagnostics_seen_{};
  uint64_t diagnostics_rx_{};
  uint16_t next_trajectory_id_{};
  bool trajectory_reserved_{};
  uint16_t trajectory_id_{};
  uint64_t trajectory_session_{};
  bool trajectory_started_{};
  bool goal_reserved_{};
  std::shared_ptr<GoalHandle> goal_;
  std::vector<Point> points_;
  uint8_t phase_{};
  uint16_t uploaded_{}, chunk_index_{}, chunk_count_{}, upload_id_{};
  uint64_t upload_session_{}, upload_ticket_{}, upload_deadline_{};
  uint16_t upload_epoch_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<msg::JointTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Publisher<msg::DriverState>::SharedPtr state_pub_;
  rclcpp::Publisher<msg::RobotEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<msg::CommandResult>::SharedPtr result_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<msg::JointCommand>::SharedPtr servo_sub_, jog_sub_;
  rclcpp::Subscription<msg::CartesianJog>::SharedPtr cartesian_sub_;
  rclcpp::Service<srv::Command>::SharedPtr command_srv_;
  rclcpp::Service<srv::Stop>::SharedPtr stop_srv_;
  rclcpp_action::Server<Upload>::SharedPtr upload_server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

ArmDriverNode::Impl::Impl(ArmDriverNode& node) : node_(node) {
  device_ = parameter<std::string>("device", "/dev/ttyUSB0", "Prefer a stable /dev/serial/by-id path");
  const auto baud = parameter<int64_t>("baud_rate", 921600, "Must match MCU UART10, 8N1, no flow control");
  const std::set<int64_t> bauds{115200, 230400, 460800, 921600, 1000000, 2000000};
  if (bauds.count(baud) == 0) throw std::invalid_argument("unsupported baud_rate");
  frame_id_ = parameter<std::string>("frame_id", "base_link", "Telemetry frame label, no TF is invented");
  std::vector<std::string> defaults;
  for (std::size_t i = 1; i <= wire::kJoints; ++i) defaults.push_back("joint_" + std::to_string(i));
  names_ = parameter<std::vector<std::string>>("joint_names", defaults, "Exactly six joints in J0..J5 order, matching URDF");
  if (names_.size() != wire::kJoints || std::set<std::string>(names_.begin(), names_.end()).size() != wire::kJoints ||
      std::any_of(names_.begin(), names_.end(), [](const auto& name) { return name.empty(); }))
    throw std::invalid_argument("joint_names must contain six unique nonempty names");
  SessionConfig config;
  config.heartbeat_period_ms = milliseconds("heartbeat_period_ms", 50, 10, 99);
  config.heartbeat_timeout_ms = milliseconds("heartbeat_timeout_ms", 300, 100, 1000);
  config.mode_timeout_ms = milliseconds("mode_timeout_ms", 300, 100, 1000);
  config.ack_timeout_ms = milliseconds("ack_timeout_ms", 200, 20, 1000);
  config.write_timeout_ms = milliseconds("write_timeout_ms", 100, 10, 250);
  config.reconnect_ms = milliseconds("reconnect_ms", 1000, 100, 10000);
  command_age_ms_ = milliseconds("command_max_age_ms", 200, 10, 1000);
  joint_timeout_ms_ = milliseconds("joint_state_timeout_ms", 200, 20, 1000);
  const auto tick_ms = milliseconds("io_period_ms", 2, 1, 10);
  if (node_.get_parameter("use_sim_time").as_bool())
    throw std::invalid_argument("physical serial driver requires use_sim_time=false");
  parameter_callback_ = node_.add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& value : parameters) {
      if (value.get_name() == "use_sim_time" && value.as_bool()) {
        result.successful = false; result.reason = "physical serial driver requires use_sim_time=false";
      }
    }
    return result;
  });
  session_ = std::make_unique<Session>(std::make_unique<SerialTransport>(device_, static_cast<int>(baud)), config, session_seed());
  session_->on_telemetry = [this](const auto& frame) { telemetry(frame); };
  auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  joint_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  telemetry_pub_ = node_.create_publisher<msg::JointTelemetry>("~/joint_telemetry", rclcpp::SensorDataQoS());
  state_pub_ = node_.create_publisher<msg::DriverState>("~/state", state_qos);
  gripper_pub_ = node_.create_publisher<std_msgs::msg::UInt8MultiArray>("/gripper_state", rclcpp::QoS(10).reliable());
  event_pub_ = node_.create_publisher<msg::RobotEvent>("~/events", rclcpp::QoS(64).reliable());
  result_pub_ = node_.create_publisher<msg::CommandResult>("~/command_results", rclcpp::QoS(64).reliable());
  diagnostics_pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(10));
  servo_sub_ = node_.create_subscription<msg::JointCommand>("~/servo_target", stream_qos,
    [this](msg::JointCommand::ConstSharedPtr message) { joint_command(*message, Command::Servo); });
  jog_sub_ = node_.create_subscription<msg::JointCommand>("~/joint_jog", stream_qos,
    [this](msg::JointCommand::ConstSharedPtr message) { joint_command(*message, Command::JointJog); });
  cartesian_sub_ = node_.create_subscription<msg::CartesianJog>("~/cartesian_jog", stream_qos,
    [this](msg::CartesianJog::ConstSharedPtr message) { cartesian_command(*message); });
  command_srv_ = node_.create_service<srv::Command>("~/command",
    [this](srv::Command::Request::SharedPtr request, srv::Command::Response::SharedPtr response) {
      command_service(*request, *response);
    });
  stop_srv_ = node_.create_service<srv::Stop>("~/stop",
    [this](srv::Stop::Request::SharedPtr request, srv::Stop::Response::SharedPtr response) {
      const auto result = session_->stop(request->client_id, steady_ms());
      response->queued = result.accepted; response->ticket = result.ticket; response->message = result.message;
      if (goal_) end_upload(Upload::Result::UNKNOWN, "STOP requested; MCU trajectory state is unconfirmed");
      drain_results(); publish_state(steady_ms());
    });
  // All callbacks use the node's default MutuallyExclusive group, including
  // action/timer callbacks in a multithreaded component container. No detached
  // threads, nested spin, blocking service waits or shared state races.
  upload_server_ = rclcpp_action::create_server<Upload>(&node_, "~/upload_trajectory",
    [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Upload::Goal> goal) {
      std::vector<Point> points;
      auto error = context_error(goal->context, Command::Begin, steady_ms());
      if (goal_reserved_ || goal_) error = "another upload is active";
      if (error.empty()) error = validate_trajectory(*goal, points);
      if (!error.empty()) {
        reject(goal->context, Command::Begin, error);
        return rclcpp_action::GoalResponse::REJECT;
      }
      goal_reserved_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](std::shared_ptr<GoalHandle> goal) {
      return goal_ == goal ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
    },
    [this](std::shared_ptr<GoalHandle> goal) { accepted(goal); });
  timer_ = node_.create_wall_timer(std::chrono::milliseconds(tick_ms), [this] { tick(); });
  RCLCPP_INFO(node_.get_logger(), "HostComputer v1 on %s at %ld baud; initial STOP requires MCU rearm",
    device_.c_str(), static_cast<long>(baud));
}

std::optional<uint64_t> ArmDriverNode::Impl::command_ttl(const msg::CommandContext& context) const {
  if (context.stamp.sec < 0 || context.stamp.nanosec >= 1000000000U) return std::nullopt;
  const int64_t stamp = int64_t{context.stamp.sec} * 1000000000LL + context.stamp.nanosec;
  const auto age = node_.now().nanoseconds() - stamp;
  if (stamp == 0 || age < 0 || age >= static_cast<int64_t>(command_age_ms_ * 1000000)) return std::nullopt;
  return command_age_ms_ - static_cast<uint64_t>(age / 1000000);
}
bool ArmDriverNode::Impl::joint_fresh(uint64_t now) const {
  return joint_seen_ && joint_session_ == session_->state(now).session_id && now - last_joint_rx_ < joint_timeout_ms_;
}
std::string ArmDriverNode::Impl::context_error(const msg::CommandContext& context, Command command, uint64_t now) const {
  if (!command_ttl(context)) return "missing, future or expired command timestamp (shared ROS clock required)";
  auto error = session_->gate(command, context.session_id, context.mode_epoch, now);
  if (error.empty() && command != Command::Hold && command != Command::ClearFault && !joint_fresh(now))
    error = "joint telemetry is stale";
  return error;
}
void ArmDriverNode::Impl::reject(const msg::CommandContext& context, Command command, const std::string& reason) {
  CommandResult result;
  result.client_id = context.client_id; result.session_id = context.session_id;
  result.epoch = context.mode_epoch; result.command = command; result.message = reason;
  publish_result(result);
}
void ArmDriverNode::Impl::joint_command(const msg::JointCommand& message, Command command) {
  const auto now = steady_ms();
  auto error = context_error(message.context, command, now);
  if (!valid_mask(message.arm_mask) || message.duration_ms == 0 || !finite(message.right))
    error = "invalid arm mask, duration or joint values";
  if (!error.empty()) { reject(message.context, command, error); return; }
  wire::Writer body;
  body.u8(message.arm_mask); body.u8(0); body.u16(message.duration_ms);
  body.floats(message.right);
  session_->submit(command, std::move(body.bytes), message.context.session_id, message.context.mode_epoch,
    message.context.client_id, now, command_ttl(message.context).value_or(0), true);
  drain_results();
}
void ArmDriverNode::Impl::cartesian_command(const msg::CartesianJog& message) {
  const auto now = steady_ms();
  auto error = context_error(message.context, Command::CartesianJog, now);
  if (!valid_mask(message.arm_mask) || message.reference_frame != 0 || message.duration_ms == 0 ||
      !finite(message.right)) error = "invalid Cartesian jog fields";
  if (!error.empty()) { reject(message.context, Command::CartesianJog, error); return; }
  wire::Writer body;
  body.u8(message.arm_mask); body.u8(message.reference_frame); body.u16(message.duration_ms);
  body.floats(message.right);
  session_->submit(Command::CartesianJog, std::move(body.bytes), message.context.session_id, message.context.mode_epoch,
    message.context.client_id, now, command_ttl(message.context).value_or(0), true);
  drain_results();
}
void ArmDriverNode::Impl::command_service(const srv::Command::Request& request, srv::Command::Response& response) {
  const auto command = static_cast<Command>(request.command);
  const auto now = steady_ms();
  auto error = context_error(request.context, command, now);
  if (command < Command::Hold || command > Command::Cancel) error = "unsupported discrete command";
  if ((command == Command::Start || command == Command::Cancel) &&
      (!trajectory_reserved_ || trajectory_session_ != request.context.session_id || trajectory_id_ != request.trajectory_id))
    error = "trajectory ID does not match this driver's committed trajectory";
  if (command == Command::Start && trajectory_started_) error = "Start already accepted; no automatic restart";
  if (goal_ || goal_reserved_) error = "upload active: cancel its Action or use STOP first";
  if (!error.empty()) { response.message = error; reject(request.context, command, error); return; }
  wire::Writer body;
  if (command == Command::Start || command == Command::Cancel) body.u16(request.trajectory_id);
  const auto result = session_->submit(command, std::move(body.bytes), request.context.session_id,
    request.context.mode_epoch, request.context.client_id, now, command_ttl(request.context).value_or(0));
  response.accepted = result.accepted; response.ticket = result.ticket; response.message = result.message;
  // Reserve Start at admission to prevent two queued starts; rejection restores it.
  if (command == Command::Start && result.accepted) trajectory_started_ = true;
  drain_results();
}

void ArmDriverNode::Impl::telemetry(const wire::Frame& frame) {
  wire::Reader reader(frame.payload);
  const auto now = steady_ms(); const auto state = session_->state(now);
  if (frame.command == Command::JointState) {
    msg::JointTelemetry raw;
    raw.header.stamp = node_.now(); raw.header.frame_id = frame_id_;
    raw.session_id = state.session_id;
    raw.mcu_timestamp_ms = reader.u32(); raw.mode_epoch = reader.u16();
    if (joint_seen_ && joint_session_ == state.session_id) {
      const auto delta = static_cast<uint32_t>(raw.mcu_timestamp_ms - last_joint_timestamp_);
      if (delta == 0 || delta >= 0x80000000U) return; // Repeated snapshot is not new feedback.
    }
    reader.floats(raw.right_position); reader.floats(raw.right_velocity);
    sensor_msgs::msg::JointState joints;
    joints.header = raw.header; joints.name = names_;
    joints.position.insert(joints.position.end(), raw.right_position.begin(), raw.right_position.end());
    joints.velocity.insert(joints.velocity.end(), raw.right_velocity.begin(), raw.right_velocity.end());
    joint_seen_ = true; joint_session_ = state.session_id; last_joint_rx_ = now;
    last_joint_timestamp_ = raw.mcu_timestamp_ms;
    std_msgs::msg::UInt8MultiArray gripper; gripper.data = {reader.u8()};
    joint_pub_->publish(joints); telemetry_pub_->publish(raw); gripper_pub_->publish(gripper);
  } else if (frame.command == Command::Event) {
    msg::RobotEvent event;
    event.header.stamp = node_.now(); event.header.frame_id = frame_id_;
    event.session_id = state.session_id; event.type = reader.u8(); event.mcu_timestamp_ms = reader.u32();
    event.mode = reader.u8(); event.mode_epoch = reader.u16(); event.detail = reader.u16();
    event.matches_current_epoch = state.mode_known && state.epoch == event.mode_epoch;
    event_pub_->publish(event);
  } else if (frame.command == Command::Diagnostics) {
    for (auto& value : mcu_diagnostics_) value = reader.u32();
    diagnostics_seen_ = true; diagnostics_rx_ = now;
  }
}
void ArmDriverNode::Impl::publish_result(const CommandResult& result) {
  msg::CommandResult message;
  message.header.stamp = node_.now(); message.ticket = result.ticket;
  message.client_id = result.client_id; message.session_id = result.session_id;
  message.request_command = static_cast<uint16_t>(result.command);
  message.sequence = result.sequence; message.mode_epoch = result.epoch;
  message.outcome = static_cast<uint8_t>(result.outcome); message.mcu_result = result.mcu_result;
  message.detail = result.detail; message.message = result.message;
  result_pub_->publish(message);
}
void ArmDriverNode::Impl::drain_results() {
  CommandResult result;
  while (session_->pop_result(result)) {
    publish_result(result);
    if (result.command == Command::Start && result.session_id == trajectory_session_ &&
        (result.outcome == Outcome::Rejected || (result.outcome == Outcome::Ack && result.mcu_result != 0)))
      trajectory_started_ = false;
    if (goal_ && result.ticket == upload_ticket_) upload_result(result);
  }
}
void ArmDriverNode::Impl::publish_state(uint64_t now) {
  const auto state = session_->state(now);
  msg::DriverState message;
  message.header.stamp = node_.now(); message.header.frame_id = frame_id_;
  message.session_id = state.session_id; message.mode = static_cast<uint8_t>(state.mode); message.mode_epoch = state.epoch;
  message.connected = state.connected; message.mode_known = state.mode_known;
  message.heartbeat_fresh = state.heartbeat_fresh; message.mode_fresh = state.mode_fresh;
  message.peer_online = state.peer_online; message.control_blocked = state.control_blocked;
  message.snapshot_valid = state.snapshot_valid; message.rearm_required = state.rearm_required;
  message.joint_state_fresh = joint_fresh(now);
  message.motion_ready = state.connected && state.heartbeat_fresh && state.mode_fresh && state.peer_online &&
    !state.control_blocked && !state.rearm_required && state.snapshot_valid && message.joint_state_fresh &&
    (state.mode == wire::Mode::PcDebug || state.mode == wire::Mode::Vla || state.mode == wire::Mode::RosTrajectory);
  message.safety_flags = state.safety_flags; message.fault_code = state.fault_code;
  message.mcu_uptime_ms = state.mcu_uptime_ms;
  message.right_gripper_state = state.right_gripper_state;
  message.heartbeat_age_ms = state.heartbeat_age_ms;
  message.mode_age_ms = state.mode_age_ms;
  message.joint_state_age_ms = joint_seen_ && joint_session_ == state.session_id ? now - last_joint_rx_ : std::numeric_limits<uint64_t>::max();
  message.trajectory_reserved = trajectory_reserved_ && trajectory_session_ == state.session_id;
  message.trajectory_id = message.trajectory_reserved ? trajectory_id_ : 0;
  message.last_error = state.last_error;
  state_pub_->publish(message); last_state_tx_ = now;
}
void ArmDriverNode::Impl::publish_diagnostics(uint64_t now) {
  const auto state = session_->state(now);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_.now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(node_.get_fully_qualified_name()) + "/host_link"; status.hardware_id = device_;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK; status.message = "communication healthy";
  if (!state.connected || !state.heartbeat_fresh || !state.mode_fresh) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; status.message = "link or mode telemetry unavailable";
  } else if (state.snapshot_valid && state.fault_code != 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; status.message = "MCU reports a fault";
  } else if (!state.peer_online || state.rearm_required || state.control_blocked || !state.snapshot_valid || !joint_fresh(now)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; status.message = "waiting for MCU rearm or valid feedback";
  }
  auto add = [&status](const std::string& name, uint64_t value) {
    diagnostic_msgs::msg::KeyValue entry; entry.key = name; entry.value = std::to_string(value);
    status.values.push_back(std::move(entry));
  };
  add("host.framing_errors", state.framing_errors); add("host.payload_errors", state.payload_errors);
  add("host.opens", state.reconnects); add("host.ack_timeouts", state.ack_timeouts);
  add("host.unmatched_acks", state.unmatched_acks); add("host.heartbeat_age_ms", state.heartbeat_age_ms);
  add("host.right_gripper_state", state.right_gripper_state);
  add("host.mode_age_ms", state.mode_age_ms); add("host.session_id", state.session_id);
  const std::array<const char*, 7> labels{"valid_frames", "framing_errors", "rejected_commands", "rx_drops", "queue_full", "tx_errors", "transport_errors"};
  add("mcu.diagnostics_seen", diagnostics_seen_);
  if (diagnostics_seen_) {
    add("mcu.diagnostics_age_ms", now - diagnostics_rx_);
    for (std::size_t i = 0; i < labels.size(); ++i) add(std::string("mcu.") + labels[i], mcu_diagnostics_[i]);
  }
  array.status.push_back(std::move(status)); diagnostics_pub_->publish(array); last_diagnostics_tx_ = now;
}

std::string ArmDriverNode::Impl::validate_trajectory(const Upload::Goal& goal, std::vector<Point>& points) const {
  const auto& trajectory = goal.trajectory;
  if (!valid_mask(goal.arm_mask) || trajectory.points.empty() || trajectory.points.size() > 256)
    return "arm_mask must be 1 and trajectory must contain 1..256 points";
  if (trajectory.header.stamp.sec != 0 || trajectory.header.stamp.nanosec != 0)
    return "trajectory.header.stamp must be zero; explicit Start controls execution";
  std::vector<std::size_t> mapping;
  std::set<std::string> supplied;
  for (const auto& name : trajectory.joint_names) {
    const auto found = std::find(names_.begin(), names_.end(), name);
    if (found == names_.end() || !supplied.insert(name).second) return "unknown or duplicate trajectory joint name";
    mapping.push_back(static_cast<std::size_t>(found - names_.begin()));
  }
  if (mapping.size() != wire::kJoints) return "provide exactly six joint names";
  points.clear(); points.reserve(trajectory.points.size());
  for (const auto& input : trajectory.points) {
    if (input.positions.size() != mapping.size() || !input.velocities.empty() || !input.accelerations.empty() || !input.effort.empty())
      return "wire format supports positions only: explicitly retime/resample and remove unsupported derivatives upstream";
    if (!finite(input.positions) || input.time_from_start.sec < 0 || input.time_from_start.nanosec >= 1000000000U)
      return "nonfinite position or invalid time_from_start";
    const auto nanoseconds = uint64_t{static_cast<uint32_t>(input.time_from_start.sec)} * 1000000000ULL + input.time_from_start.nanosec;
    if (nanoseconds % 1000 != 0 || nanoseconds / 1000 > std::numeric_limits<uint32_t>::max())
      return "time_from_start must be exact microseconds within uint32 range";
    Point point; point.time_us = static_cast<uint32_t>(nanoseconds / 1000);
    if (!points.empty() && point.time_us <= points.back().time_us) return "trajectory times must strictly increase";
    for (std::size_t i = 0; i < mapping.size(); ++i) {
      if (std::abs(input.positions[i]) > std::numeric_limits<float>::max()) return "position exceeds binary32 range";
      const auto index = mapping[i];
      point.right[index] = static_cast<float>(input.positions[i]);
    }
    points.push_back(point);
  }
  return {};
}
void ArmDriverNode::Impl::accepted(const std::shared_ptr<GoalHandle>& goal) {
  goal_reserved_ = false; goal_ = goal;
  upload_session_ = goal->get_goal()->context.session_id; upload_epoch_ = goal->get_goal()->context.mode_epoch;
  upload_id_ = ++next_trajectory_id_; phase_ = 0; uploaded_ = chunk_index_ = chunk_count_ = 0; upload_ticket_ = 0;
  const auto error = validate_trajectory(*goal->get_goal(), points_);
  if (!error.empty()) { end_upload(Upload::Result::FAILED, error); return; }
  upload_deadline_ = steady_ms() + 120000;  // bounded even under unrelated traffic
  feedback();
}
void ArmDriverNode::Impl::feedback() {
  if (!goal_) return;
  auto value = std::make_shared<Upload::Feedback>();
  value->phase = phase_; value->trajectory_id = upload_id_; value->acknowledged_points = uploaded_;
  value->total_points = static_cast<uint16_t>(points_.size()); goal_->publish_feedback(value);
}
void ArmDriverNode::Impl::drive_upload(uint64_t now) {
  if (!goal_) return;
  if (goal_->is_canceling()) {
    // Cancel cannot acknowledge storage release in v1. STOP is the only
    // unconditionally accepted cleanup request; next epoch establishes release.
    stop_upload_if_current(now);
    end_upload(Upload::Result::CANCELED, "upload canceled locally; STOP queued, await MCU rearm before reuse");
    return;
  }
  const auto error = session_->gate(Command::Begin, upload_session_, upload_epoch_, now);
  if (!error.empty() || !joint_fresh(now) || now >= upload_deadline_) {
    stop_upload_if_current(now);
    end_upload(Upload::Result::UNKNOWN, "upload interrupted: " + (error.empty() ? std::string("feedback stale or overall timeout") : error));
    return;
  }
  if (upload_ticket_ != 0) return;
  wire::Writer body; body.u16(upload_id_);
  Command command = Command::Begin;
  if (phase_ == 0) { body.u16(static_cast<uint16_t>(points_.size())); body.u8(goal_->get_goal()->arm_mask); }
  else if (phase_ == 1) {
    command = Command::Chunk; body.u16(chunk_index_);
    chunk_count_ = static_cast<uint16_t>(std::min<std::size_t>(4, points_.size() - uploaded_));
    body.u8(static_cast<uint8_t>(chunk_count_));
    for (uint16_t i = 0; i < chunk_count_; ++i) {
      const auto& point = points_[uploaded_ + i]; body.u32(point.time_us); body.floats(point.right);
    }
  } else command = Command::Commit;
  const auto result = session_->submit(command, std::move(body.bytes), upload_session_, upload_epoch_,
    goal_->get_goal()->context.client_id, now, 500);
  upload_ticket_ = result.ticket;
}
void ArmDriverNode::Impl::upload_result(const CommandResult& result) {
  upload_ticket_ = 0;
  if (goal_->is_canceling()) {
    stop_upload_if_current(steady_ms());
    end_upload(Upload::Result::CANCELED, "upload canceled locally; STOP queued, MCU release unconfirmed");
    return;
  }
  const auto current = session_->state(steady_ms());
  if (current.session_id != upload_session_ || current.epoch != upload_epoch_) {
    end_upload(Upload::Result::UNKNOWN, "MCU session changed after the upload reply; old trajectory is no longer usable");
    return;
  }
  if (result.outcome != Outcome::Ack || result.mcu_result != 0 || result.detail != upload_id_) {
    const bool unknown = result.outcome == Outcome::Timeout || result.outcome == Outcome::Invalidated;
    const bool begin_rejected = phase_ == 0 && result.outcome == Outcome::Ack && result.mcu_result != 0;
    if (!begin_rejected) stop_upload_if_current(steady_ms());
    end_upload(unknown ? Upload::Result::UNKNOWN : Upload::Result::FAILED, result.message + "; MCU result=" + std::to_string(result.mcu_result));
    return;
  }
  if (phase_ == 0) { phase_ = 1; }
  else if (phase_ == 1) {
    uploaded_ = static_cast<uint16_t>(uploaded_ + chunk_count_); ++chunk_index_;
    if (uploaded_ == points_.size()) phase_ = 2;
  } else {
    trajectory_reserved_ = true; trajectory_id_ = upload_id_; trajectory_session_ = upload_session_; trajectory_started_ = false;
    end_upload(Upload::Result::READY, "Commit ACK(Ok): trajectory stored on MCU; use command service to Start");
    return;
  }
  feedback();
}
void ArmDriverNode::Impl::stop_upload_if_current(uint64_t now) {
  const auto state = session_->state(now);
  // A newer MCU mode/epoch has already invalidated the old transfer. Do not
  // disturb that mode with a STOP intended to clean up the previous upload.
  if (state.connected && state.session_id == upload_session_ && state.epoch == upload_epoch_)
    session_->stop(goal_->get_goal()->context.client_id, now);
}
void ArmDriverNode::Impl::end_upload(uint8_t code, const std::string& text) {
  if (!goal_) return;
  auto result = std::make_shared<Upload::Result>();
  result->code = code; result->session_id = upload_session_; result->mode_epoch = upload_epoch_;
  result->trajectory_id = upload_id_; result->message = text;
  if (goal_->is_canceling()) { result->code = Upload::Result::CANCELED; goal_->canceled(result); }
  else if (code == Upload::Result::READY) goal_->succeed(result);
  else goal_->abort(result);
  goal_.reset(); points_.clear(); upload_ticket_ = 0;
}
void ArmDriverNode::Impl::tick() {
  const auto now = steady_ms();
  const auto before = session_->state(now);
  // Invalidate queued targets before I/O if once-live feedback has expired.
  if (joint_seen_ && joint_session_ == before.session_id && !joint_fresh(now) && !before.rearm_required &&
      (before.mode == wire::Mode::PcDebug || before.mode == wire::Mode::Vla || before.mode == wire::Mode::RosTrajectory))
    session_->stop(0, now);
  if (goal_ && goal_->is_canceling()) drive_upload(now);
  session_->tick(now);
  const auto state = session_->state(now);
  if (state.session_id != observed_session_) {
    trajectory_reserved_ = false; trajectory_started_ = false;
    // Trajectory IDs intentionally continue across ROS session changes: STOP
    // and events can change the session without resetting the MCU's ID filter.
    observed_session_ = state.session_id;
    publish_state(now);
  }
  drain_results(); drive_upload(now); drain_results();
  if (now - last_state_tx_ >= 100) publish_state(now);
  if (now - last_diagnostics_tx_ >= 500) publish_diagnostics(now);
}

ArmDriverNode::ArmDriverNode(const rclcpp::NodeOptions& options) : Node("arm_driver", options), impl_(std::make_unique<Impl>(*this)) {}
ArmDriverNode::~ArmDriverNode() = default;
}  // namespace eng26
RCLCPP_COMPONENTS_REGISTER_NODE(eng26::ArmDriverNode)
