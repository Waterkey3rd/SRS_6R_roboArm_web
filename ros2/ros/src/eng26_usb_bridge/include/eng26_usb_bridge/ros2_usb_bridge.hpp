#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Ros2UsbBridge final : public rclcpp::Node
{
public:
    Ros2UsbBridge();
    ~Ros2UsbBridge() override;

private:
    static constexpr std::size_t kArmDof = 7;
    static constexpr std::size_t kJointCount = 14;
    static constexpr std::size_t kMaxFrameSize = 1024;
    static constexpr std::size_t kMaxRxStreamSize =
        kMaxFrameSize * 4;

    // ============================================================
    // Protocol command IDs
    // ============================================================

    static constexpr uint16_t CMD_HEARTBEAT = 0x10;
    static constexpr uint16_t CMD_JOINT_SERVO_TARGET = 0x20;

    static constexpr uint16_t CMD_JOINT_STATE = 0x80;
    static constexpr uint16_t CMD_MODE_STATE = 0x81;
    static constexpr uint16_t CMD_ACK = 0x83;

    static constexpr uint8_t MODE_PC_DEBUG = 2;
    static constexpr uint8_t MODE_VLA = 4;

    static constexpr uint8_t ARM_RIGHT = 1;
    static constexpr uint8_t ARM_LEFT = 2;
    static constexpr uint8_t ARM_BOTH = 3;

    struct TxFrame
    {
        uint16_t cmd{0};
        std::vector<uint8_t> bytes;
    };

    // ============================================================
    // ROS2
    // ============================================================

    void jointCommandCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg);

    void gripperCommandCallback(
        const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    void heartbeatTimerCallback();

    void publishJointState(
        const std::array<float, kArmDof>& right_q,
        const std::array<float, kArmDof>& right_dq,
        const std::array<float, kArmDof>& left_q,
        const std::array<float, kArmDof>& left_dq);

    void publishGripperState(
        uint8_t left,
        uint8_t right);

    bool extractJointTarget(
        const sensor_msgs::msg::JointState& msg,
        std::array<float, kArmDof>& right,
        std::array<float, kArmDof>& left) const;

    static std::vector<std::string> defaultJointNames();

    // ============================================================
    // Serial
    // ============================================================

    bool openSerial();
    void closeSerial();
    void serialLoop();

    bool drainTxQueue(std::size_t budget);

    void enqueueFrame(
        uint16_t cmd,
        std::vector<uint8_t>&& frame);

    // ============================================================
    // Protocol TX
    // ============================================================

    void sendHeartbeat();

    void sendJointServoTarget(
        uint8_t arm_mask,
        uint16_t duration_ms,
        const std::array<float, kArmDof>& right,
        const std::array<float, kArmDof>& left);

    // Reserved: 底层夹爪下行协议尚未确定
    void sendGripperTarget(
        uint8_t left,
        uint8_t right);

    // ============================================================
    // Protocol RX
    // ============================================================

    void consumeRxBytes(
        const uint8_t* data,
        std::size_t size);

    bool processFrame(
        const uint8_t* frame,
        std::size_t size);

    void handleHeartbeat(
        const uint8_t* payload,
        std::size_t size);

    void handleJointState(
        const uint8_t* payload,
        std::size_t size);

    void handleModeState(
        const uint8_t* payload,
        std::size_t size);

    void handleAck(
        const uint8_t* payload,
        std::size_t size);

    // ============================================================
    // Codec
    // ============================================================

    static std::vector<uint8_t> buildFrame(
        uint16_t cmd,
        const std::vector<uint8_t>& payload);

    static uint16_t readU16(const uint8_t* p);
    static uint32_t readU32(const uint8_t* p);
    static float readFloat(const uint8_t* p);

    static void appendU16(
        std::vector<uint8_t>& buffer,
        uint16_t value);

    static void appendU32(
        std::vector<uint8_t>& buffer,
        uint32_t value);

    static void appendFloat(
        std::vector<uint8_t>& buffer,
        float value);

    // ============================================================
    // ROS objects
    // ============================================================

    rclcpp::Publisher<
        sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;

    rclcpp::Publisher<
        std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_state_pub_;

    rclcpp::Subscription<
        std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_command_sub_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // ============================================================
    // Serial config
    // ============================================================

    std::string serial_port_{"/dev/ttyUSB0"};
    int baud_rate_{921600};
    int reconnect_interval_ms_{1000};

    std::size_t tx_queue_limit_{32};

    uint16_t command_duration_ms_{100};

    // ============================================================
    // Serial runtime
    // ============================================================

    int serial_fd_{-1};

    std::atomic<bool> running_{false};
    std::atomic<bool> serial_connected_{false};

    std::thread serial_thread_;

    std::mutex tx_mutex_;
    std::deque<TxFrame> tx_queue_;

    std::vector<uint8_t> rx_stream_;

    // ============================================================
    // MCU state
    // ============================================================

    std::atomic<bool> mcu_online_{false};
    std::atomic<bool> control_blocked_{true};
    std::atomic<bool> snapshot_valid_{false};

    std::atomic<uint8_t> current_mode_{0};
    std::atomic<uint16_t> current_epoch_{0};

    std::atomic<uint16_t> joint_seq_{0};

    // reserved gripper target
    std::atomic<uint8_t> desired_left_gripper_{0};
    std::atomic<uint8_t> desired_right_gripper_{0};

    // ============================================================
    // Config
    // ============================================================

    std::vector<std::string> joint_names_;

    std::chrono::steady_clock::time_point start_time_;
};
