#include "eng26_usb_bridge/ros2_usb_bridge.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <unordered_map>
#include <unistd.h>

using namespace std::chrono_literals;

// ============================================================
// Constructor / Destructor
// ============================================================

Ros2UsbBridge::Ros2UsbBridge()
    : Node("ros2_usb_bridge"),
      start_time_(std::chrono::steady_clock::now())
{
    serial_port_ =
        declare_parameter<std::string>(
            "serial_port",
            "/dev/ttyUSB0");

    baud_rate_ =
        declare_parameter<int>(
            "baud_rate",
            921600);

    reconnect_interval_ms_ =
        declare_parameter<int>(
            "reconnect_interval_ms",
            1000);

    tx_queue_limit_ =
        static_cast<std::size_t>(
            declare_parameter<int>(
                "tx_queue_limit",
                32));

    const int duration =
        declare_parameter<int>(
            "command_duration_ms",
            100);

    command_duration_ms_ =
        static_cast<uint16_t>(
            std::clamp(duration, 1, 65535));

    joint_names_ =
        declare_parameter<std::vector<std::string>>(
            "joint_names",
            defaultJointNames());

    if (joint_names_.size() != kJointCount)
    {
        throw std::runtime_error(
            "joint_names must contain exactly 14 names");
    }

    // ============================================================
    // ROS interfaces
    // ============================================================

    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::QoS(
                rclcpp::KeepLast(10))
                .reliable()
                .durability_volatile());

    joint_command_sub_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_command",
            rclcpp::QoS(10),
            std::bind(
                &Ros2UsbBridge::jointCommandCallback,
                this,
                std::placeholders::_1));

    gripper_state_pub_ =
        create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/gripper_state",
            rclcpp::QoS(
                rclcpp::KeepLast(10))
                .reliable()
                .durability_volatile());

    gripper_command_sub_ =
        create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/gripper_command",
            rclcpp::QoS(10),
            std::bind(
                &Ros2UsbBridge::gripperCommandCallback,
                this,
                std::placeholders::_1));

    // PC heartbeat
    heartbeat_timer_ =
        create_wall_timer(
            50ms,
            std::bind(
                &Ros2UsbBridge::heartbeatTimerCallback,
                this));

    running_.store(true);

    serial_thread_ =
        std::thread(
            &Ros2UsbBridge::serialLoop,
            this);

    RCLCPP_INFO(
        get_logger(),
        "ENG26 serial bridge started");

    RCLCPP_INFO(
        get_logger(),
        "Serial: %s @ %d",
        serial_port_.c_str(),
        baud_rate_);

    RCLCPP_INFO(
        get_logger(),
        "TX topic: /joint_command");

    RCLCPP_INFO(
        get_logger(),
        "RX topic: /joint_states");
}

Ros2UsbBridge::~Ros2UsbBridge()
{
    running_.store(false);

    if (serial_thread_.joinable())
    {
        serial_thread_.join();
    }

    closeSerial();
}

// ============================================================
// Joint names
// ============================================================

std::vector<std::string>
Ros2UsbBridge::defaultJointNames()
{
    return {
        "right_joint_1",
        "right_joint_2",
        "right_joint_3",
        "right_joint_4",
        "right_joint_5",
        "right_joint_6",
        "right_joint_7",

        "left_joint_1",
        "left_joint_2",
        "left_joint_3",
        "left_joint_4",
        "left_joint_5",
        "left_joint_6",
        "left_joint_7"
    };
}

// ============================================================
// ROS Joint Command
// ============================================================

void Ros2UsbBridge::jointCommandCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!serial_connected_.load())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Joint command ignored: serial disconnected");

        return;
    }

    if (!mcu_online_.load())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Joint command ignored: MCU offline");

        return;
    }

    const uint8_t mode =
        current_mode_.load();

    if (mode != MODE_PC_DEBUG &&
        mode != MODE_VLA)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Joint command ignored: MCU mode=%u",
            static_cast<unsigned>(mode));

        return;
    }

    if (control_blocked_.load())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Joint command ignored: controlBlocked=1");

        return;
    }

    std::array<float, kArmDof> right{};
    std::array<float, kArmDof> left{};

    if (!extractJointTarget(
            *msg,
            right,
            left))
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Invalid /joint_command; expected 14 finite positions");

        return;
    }

    sendJointServoTarget(
        ARM_BOTH,
        command_duration_ms_,
        right,
        left);
}

bool Ros2UsbBridge::extractJointTarget(
    const sensor_msgs::msg::JointState& msg,
    std::array<float, kArmDof>& right,
    std::array<float, kArmDof>& left) const
{
    if (msg.position.size() != kJointCount)
    {
        return false;
    }

    if (msg.name.empty())
    {
        for (std::size_t i = 0;
             i < kArmDof;
             ++i)
        {
            const double rq =
                msg.position[i];

            const double lq =
                msg.position[i + kArmDof];

            if (!std::isfinite(rq) ||
                !std::isfinite(lq))
            {
                return false;
            }

            right[i] =
                static_cast<float>(rq);

            left[i] =
                static_cast<float>(lq);
        }

        return true;
    }

    if (msg.name.size() !=
        msg.position.size())
    {
        return false;
    }

    std::unordered_map<std::string, double> values;

    for (std::size_t i = 0;
         i < msg.name.size();
         ++i)
    {
        if (!std::isfinite(
                msg.position[i]))
        {
            return false;
        }

        values[msg.name[i]] =
            msg.position[i];
    }

    for (std::size_t i = 0;
         i < kArmDof;
         ++i)
    {
        const auto right_it =
            values.find(
                joint_names_[i]);

        const auto left_it =
            values.find(
                joint_names_[i + kArmDof]);

        if (right_it == values.end() ||
            left_it == values.end())
        {
            return false;
        }

        right[i] =
            static_cast<float>(
                right_it->second);

        left[i] =
            static_cast<float>(
                left_it->second);
    }

    return true;
}

// ============================================================
// Gripper reserved
// ============================================================

void Ros2UsbBridge::gripperCommandCallback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 2)
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "/gripper_command must be [left, right]");

        return;
    }

    desired_left_gripper_.store(
        msg->data[0]);

    desired_right_gripper_.store(
        msg->data[1]);

    sendGripperTarget(
        msg->data[0],
        msg->data[1]);
}

void Ros2UsbBridge::sendGripperTarget(
    uint8_t left,
    uint8_t right)
{
    (void)left;
    (void)right;

    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Gripper downlink protocol not implemented");
}

// ============================================================
// Heartbeat
// ============================================================

void Ros2UsbBridge::heartbeatTimerCallback()
{
    if (serial_connected_.load())
    {
        sendHeartbeat();
    }
}

void Ros2UsbBridge::sendHeartbeat()
{
    const auto now =
        std::chrono::steady_clock::now();

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            now - start_time_);

    const uint32_t uptime_ms =
        static_cast<uint32_t>(
            elapsed.count());

    std::vector<uint8_t> payload;
    payload.reserve(4);

    appendU32(
        payload,
        uptime_ms);

    enqueueFrame(
        CMD_HEARTBEAT,
        buildFrame(
            CMD_HEARTBEAT,
            payload));
}

// ============================================================
// JointServoTarget
// ============================================================

void Ros2UsbBridge::sendJointServoTarget(
    uint8_t arm_mask,
    uint16_t duration_ms,
    const std::array<float, kArmDof>& right,
    const std::array<float, kArmDof>& left)
{
    std::vector<uint8_t> payload;
    payload.reserve(64);

    const uint16_t seq =
        joint_seq_.fetch_add(1);

    const uint16_t epoch =
        current_epoch_.load();

    appendU16(
        payload,
        seq);

    appendU16(
        payload,
        epoch);

    payload.push_back(
        arm_mask);

    // flags
    payload.push_back(
        0);

    appendU16(
        payload,
        duration_ms);

    for (const float q : right)
    {
        appendFloat(
            payload,
            q);
    }

    for (const float q : left)
    {
        appendFloat(
            payload,
            q);
    }

    if (payload.size() != 64)
    {
        RCLCPP_ERROR(
            get_logger(),
            "JointServoTarget payload invalid size=%zu",
            payload.size());

        return;
    }

    enqueueFrame(
        CMD_JOINT_SERVO_TARGET,
        buildFrame(
            CMD_JOINT_SERVO_TARGET,
            payload));
}

// ============================================================
// Serial
// ============================================================

bool Ros2UsbBridge::openSerial()
{
    serial_fd_ =
        ::open(
            serial_port_.c_str(),
            O_RDWR |
            O_NOCTTY |
            O_NONBLOCK |
            O_CLOEXEC);

    if (serial_fd_ < 0)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Failed to open %s: %s",
            serial_port_.c_str(),
            std::strerror(errno));

        return false;
    }

    if (::ioctl(serial_fd_, TIOCEXCL) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to exclusively lock %s: %s",
            serial_port_.c_str(),
            std::strerror(errno));

        ::close(serial_fd_);
        serial_fd_ = -1;

        return false;
    }

    termios tty{};

    if (tcgetattr(
            serial_fd_,
            &tty) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "tcgetattr failed: %s",
            std::strerror(errno));

        ::close(
            serial_fd_);

        serial_fd_ = -1;

        return false;
    }

    cfmakeraw(
        &tty);

    tty.c_cflag |=
        CLOCAL | CREAD;

    // 8 data bits
    tty.c_cflag &=
        ~CSIZE;

    tty.c_cflag |=
        CS8;

    // no parity
    tty.c_cflag &=
        ~PARENB;

    // 1 stop bit
    tty.c_cflag &=
        ~CSTOPB;

    // no hardware flow control
    tty.c_cflag &=
        ~CRTSCTS;

    // no software flow control
    tty.c_iflag &=
        ~(IXON |
          IXOFF |
          IXANY);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    speed_t speed;

    switch (baud_rate_)
    {
        case 921600:
            speed = B921600;
            break;

        case 460800:
            speed = B460800;
            break;

        case 115200:
            speed = B115200;
            break;

        default:
            RCLCPP_ERROR(
                get_logger(),
                "Unsupported baud rate: %d",
                baud_rate_);

            ::close(
                serial_fd_);

            serial_fd_ = -1;

            return false;
    }

    if (cfsetispeed(
            &tty,
            speed) != 0 ||
        cfsetospeed(
            &tty,
            speed) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to set baud rate");

        ::close(
            serial_fd_);

        serial_fd_ = -1;

        return false;
    }

    if (tcsetattr(
            serial_fd_,
            TCSANOW,
            &tty) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "tcsetattr failed: %s",
            std::strerror(errno));

        ::close(
            serial_fd_);

        serial_fd_ = -1;

        return false;
    }

    tcflush(
        serial_fd_,
        TCIOFLUSH);

    rx_stream_.clear();

    serial_connected_.store(
        true);

    RCLCPP_INFO(
        get_logger(),
        "Serial connected: %s @ %d baud",
        serial_port_.c_str(),
        baud_rate_);

    return true;
}

void Ros2UsbBridge::closeSerial()
{
    serial_connected_.store(
        false);

    mcu_online_.store(
        false);

    control_blocked_.store(
        true);

    if (serial_fd_ >= 0)
    {
        ::close(
            serial_fd_);

        serial_fd_ = -1;
    }

    rx_stream_.clear();
}

void Ros2UsbBridge::serialLoop()
{
    std::array<uint8_t, 4096> rx_buffer{};

    while (running_.load())
    {
        if (serial_fd_ < 0)
        {
            if (!openSerial())
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(
                        reconnect_interval_ms_));

                continue;
            }
        }

        if (!drainTxQueue(4))
        {
            RCLCPP_WARN(
                get_logger(),
                "Serial TX failed, reconnecting");

            closeSerial();

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    reconnect_interval_ms_));

            continue;
        }

        const ssize_t count =
            ::read(
                serial_fd_,
                rx_buffer.data(),
                rx_buffer.size());

        if (count > 0)
        {
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
                "Serial RX alive: %ld bytes",
                static_cast<long>(count));

            consumeRxBytes(
                rx_buffer.data(),
                static_cast<std::size_t>(count));
        }
        else if (
            count < 0 &&
            errno != EAGAIN &&
            errno != EWOULDBLOCK)
        {
            RCLCPP_WARN(
                get_logger(),
                "Serial RX error: %s",
                std::strerror(errno));

            closeSerial();

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    reconnect_interval_ms_));

            continue;
        }

        std::this_thread::sleep_for(
            1ms);
    }
}

// ============================================================
// TX queue
// ============================================================

bool Ros2UsbBridge::drainTxQueue(
    std::size_t budget)
{
    for (std::size_t i = 0;
         i < budget;
         ++i)
    {
        TxFrame frame;

        {
            std::lock_guard<std::mutex> lock(
                tx_mutex_);

            if (tx_queue_.empty())
            {
                return true;
            }

            frame =
                std::move(
                    tx_queue_.front());

            tx_queue_.pop_front();
        }

        std::size_t offset = 0;

        while (offset <
               frame.bytes.size())
        {
            const ssize_t written =
                ::write(
                    serial_fd_,
                    frame.bytes.data() +
                        offset,
                    frame.bytes.size() -
                        offset);

            if (written > 0)
            {
                offset +=
                    static_cast<std::size_t>(
                        written);

                continue;
            }

            if (written < 0 &&
                (errno == EAGAIN ||
                 errno == EWOULDBLOCK))
            {
                std::this_thread::sleep_for(
                    200us);

                continue;
            }

            RCLCPP_WARN(
                get_logger(),
                "Serial TX failed cmd=0x%04x: %s",
                frame.cmd,
                std::strerror(errno));

            return false;
        }
    }

    return true;
}

void Ros2UsbBridge::enqueueFrame(
    uint16_t cmd,
    std::vector<uint8_t>&& frame)
{
    std::lock_guard<std::mutex> lock(
        tx_mutex_);

    // 实时位置命令只保留最新值
    if (cmd ==
        CMD_JOINT_SERVO_TARGET)
    {
        tx_queue_.erase(
            std::remove_if(
                tx_queue_.begin(),
                tx_queue_.end(),
                [](const TxFrame& queued)
                {
                    return queued.cmd ==
                        CMD_JOINT_SERVO_TARGET;
                }),
            tx_queue_.end());
    }

    // heartbeat 不积压
    if (cmd ==
        CMD_HEARTBEAT)
    {
        tx_queue_.erase(
            std::remove_if(
                tx_queue_.begin(),
                tx_queue_.end(),
                [](const TxFrame& queued)
                {
                    return queued.cmd ==
                        CMD_HEARTBEAT;
                }),
            tx_queue_.end());
    }

    if (tx_queue_.size() >=
        tx_queue_limit_)
    {
        tx_queue_.pop_front();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "TX queue full; dropping oldest frame");
    }

    tx_queue_.push_back(
        TxFrame{
            cmd,
            std::move(frame)
        });
}

// ============================================================
// Stream parser
// ============================================================

void Ros2UsbBridge::consumeRxBytes(
    const uint8_t* data,
    std::size_t size)
{
    if (size > kMaxRxStreamSize)
    {
        data += size - kMaxRxStreamSize;
        size = kMaxRxStreamSize;
        rx_stream_.clear();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "RX burst exceeded %zu bytes; resynchronizing stream",
            kMaxRxStreamSize);
    }

    if (rx_stream_.size() + size > kMaxRxStreamSize)
    {
        rx_stream_.clear();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "RX stream exceeded %zu bytes; resynchronizing stream",
            kMaxRxStreamSize);
    }

    rx_stream_.insert(
        rx_stream_.end(),
        data,
        data + size);

    while (true)
    {
        const auto sof =
            std::find(
                rx_stream_.begin(),
                rx_stream_.end(),
                static_cast<uint8_t>(0xA5));

        if (sof ==
            rx_stream_.end())
        {
            rx_stream_.clear();
            return;
        }

        if (sof !=
            rx_stream_.begin())
        {
            rx_stream_.erase(
                rx_stream_.begin(),
                sof);
        }

        if (rx_stream_.size() < 3)
        {
            return;
        }

        const uint16_t frame_len =
            readU16(
                &rx_stream_[1]);

        if (frame_len < 7 ||
            frame_len >
                kMaxFrameSize)
        {
            rx_stream_.erase(
                rx_stream_.begin());

            continue;
        }

        if (rx_stream_.size() <
            frame_len)
        {
            return;
        }

        if (!processFrame(
                rx_stream_.data(),
                frame_len))
        {
            // A corrupted length/checksum must not consume an entire
            // candidate frame: it may contain the SOF of the next valid one.
            rx_stream_.erase(
                rx_stream_.begin());

            continue;
        }

        rx_stream_.erase(
            rx_stream_.begin(),
            rx_stream_.begin() +
                frame_len);
    }
}

// ============================================================
// Frame validation
// ============================================================

bool Ros2UsbBridge::processFrame(
    const uint8_t* frame,
    std::size_t size)
{
    if (size < 7)
    {
        return false;
    }

    if (frame[0] != 0xA5 ||
        frame[size - 1] != 0x5A)
    {
        return false;
    }

    const uint16_t wire_size =
        readU16(
            frame + 1);

    if (wire_size != size)
    {
        return false;
    }

    const uint16_t cmd =
        readU16(
            frame + 3);

    const uint8_t* payload =
        frame + 5;

    const std::size_t payload_size =
        size - 7;

    uint8_t checksum = 0;

    for (std::size_t i = 0;
         i < payload_size;
         ++i)
    {
        checksum =
            static_cast<uint8_t>(
                checksum +
                payload[i]);
    }

    const uint8_t wire_checksum =
        frame[size - 2];

    if (checksum !=
        wire_checksum)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Checksum error cmd=0x%04x",
            cmd);

        return false;
    }

    RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "Valid frame: cmd=0x%04X payload=%zu",
    static_cast<unsigned>(cmd),
        payload_size);

    switch (cmd)
    {
        case CMD_HEARTBEAT:
            handleHeartbeat(
                payload,
                payload_size);
            break;

        case CMD_JOINT_STATE:
            handleJointState(
                payload,
                payload_size);
            break;

        case CMD_MODE_STATE:
            handleModeState(
                payload,
                payload_size);
            break;

        case CMD_ACK:
            handleAck(
                payload,
                payload_size);
            break;

        default:
            break;
    }

    return true;
}

// ============================================================
// Heartbeat RX
// ============================================================

void Ros2UsbBridge::handleHeartbeat(
    const uint8_t* payload,
    std::size_t size)
{
    // In idle mode the current MCU firmware sends uptime only.  The agreed
    // idle gripper state is Free=2 for both sides.  In control mode firmware
    // sends the extended heartbeat (uptime + left/right state), which is
    // decoded below without replacing the dynamic values.
    if (size == 4)
    {
        publishGripperState(2, 2);
        return;
    }

    if (size != 6)
    {
        return;
    }

    // uptimeMs = payload[0..3]

    const uint8_t left =
        payload[4];

    const uint8_t right =
        payload[5];

    publishGripperState(
        left,
        right);
}

// ============================================================
// JointState RX
// ============================================================

void Ros2UsbBridge::handleJointState(
    const uint8_t* payload,
    std::size_t size)
{
    // 兼容两种固件变体：
    //   118 = 6(timestamp+epoch) + 28 float，夹爪状态由心跳帧携带
    //   120 = 同上 + 2 字节左右夹爪状态（旧固件，附在末尾）
    const bool has_gripper = (size == 120);

    if (size != 118 && !has_gripper)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Invalid JointState size=%zu",
            size);

        return;
    }

    const uint32_t timestamp_ms =
        readU32(
            payload);

    (void)timestamp_ms;

    const uint16_t epoch =
        readU16(
            payload + 4);

    (void)epoch;

    std::array<float, kArmDof>
        right_q{};

    std::array<float, kArmDof>
        right_dq{};

    std::array<float, kArmDof>
        left_q{};

    std::array<float, kArmDof>
        left_dq{};

    std::size_t offset = 6;

    for (auto& value : right_q)
    {
        value =
            readFloat(
                payload +
                offset);

        offset += 4;
    }

    for (auto& value : right_dq)
    {
        value =
            readFloat(
                payload +
                offset);

        offset += 4;
    }

    for (auto& value : left_q)
    {
        value =
            readFloat(
                payload +
                offset);

        offset += 4;
    }

    for (auto& value : left_dq)
    {
        value =
            readFloat(
                payload +
                offset);

        offset += 4;
    }

    const auto finite = [](const auto& values)
    {
        return std::all_of(
            values.begin(),
            values.end(),
            [](float value)
            {
                return std::isfinite(value);
            });
    };

    if (!finite(right_q) ||
        !finite(right_dq) ||
        !finite(left_q) ||
        !finite(left_dq))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Discarding JointState with non-finite values (epoch=%u)",
            static_cast<unsigned>(epoch));

        return;
    }

    RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "JointState published: timestamp=%u epoch=%u "
    "R0=%.3f R1=%.3f L0=%.3f L1=%.3f",
    timestamp_ms,
    static_cast<unsigned>(epoch),
    right_q[0],
    right_q[1],
    left_q[0],
    left_q[1]);

    if (has_gripper)
    {
        const uint8_t left_gripper = payload[118];
        const uint8_t right_gripper = payload[119];

        // Close/Open/Free/Suspend = 0/1/2/3；非法值不影响关节数据，仅跳过夹爪发布
        if (left_gripper > 3 || right_gripper > 3)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Invalid gripper state in JointState (epoch=%u), skipping gripper publish",
                static_cast<unsigned>(epoch));
        }
        else
        {
            publishGripperState(
                left_gripper,
                right_gripper);
        }
    }

    publishJointState(
        right_q,
        right_dq,
        left_q,
        left_dq);
}

// ============================================================
// ModeState RX
// ============================================================

void Ros2UsbBridge::handleModeState(
    const uint8_t* payload,
    std::size_t size)
{
    if (size != 10)
    {
        return;
    }

    const uint8_t mode =
        payload[0];

    const uint16_t epoch =
        readU16(
            payload + 1);

    const uint16_t safety_flags =
        readU16(
            payload + 3);

    const uint16_t fault_code =
        readU16(
            payload + 5);

    const bool online =
        payload[7] != 0;

    const bool control_blocked =
        payload[8] != 0;

    const bool snapshot_valid =
        payload[9] != 0;

    const uint8_t old_mode =
        current_mode_.exchange(
            mode);

    const uint16_t old_epoch =
        current_epoch_.exchange(
            epoch);

    mcu_online_.store(
        online);

    control_blocked_.store(
        control_blocked);

    snapshot_valid_.store(
        snapshot_valid);

    if (old_mode != mode ||
        old_epoch != epoch)
    {
        RCLCPP_INFO(
            get_logger(),
            "MCU mode=%u epoch=%u "
            "online=%d blocked=%d snapshot=%d "
            "safety=0x%04x fault=0x%04x",
            static_cast<unsigned>(mode),
            static_cast<unsigned>(epoch),
            online,
            control_blocked,
            snapshot_valid,
            safety_flags,
            fault_code);
    }
}

// ============================================================
// ACK RX
// ============================================================

void Ros2UsbBridge::handleAck(
    const uint8_t* payload,
    std::size_t size)
{
    if (size != 8)
    {
        return;
    }

    const uint16_t seq =
        readU16(
            payload);

    const uint8_t request_cmd =
        payload[2];

    const uint8_t result =
        payload[3];

    const uint16_t current_epoch =
        readU16(
            payload + 4);

    const uint16_t detail =
        readU16(
            payload + 6);

    current_epoch_.store(
        current_epoch);

    if (result != 0)
    {
        RCLCPP_WARN(
            get_logger(),
            "ACK reject: seq=%u cmd=0x%02x "
            "result=%u epoch=%u detail=%u",
            seq,
            request_cmd,
            result,
            current_epoch,
            detail);
    }
}

// ============================================================
// ROS state publishers
// ============================================================

void Ros2UsbBridge::publishJointState(
    const std::array<float, kArmDof>& right_q,
    const std::array<float, kArmDof>& right_dq,
    const std::array<float, kArmDof>& left_q,
    const std::array<float, kArmDof>& left_dq)
{
    sensor_msgs::msg::JointState msg;

    msg.header.stamp =
        now();

    msg.name =
        joint_names_;

    msg.position.resize(
        kJointCount);

    msg.velocity.resize(
        kJointCount);

    for (std::size_t i = 0;
         i < kArmDof;
         ++i)
    {
        msg.position[i] =
            right_q[i];

        msg.velocity[i] =
            right_dq[i];

        msg.position[
            i + kArmDof] =
            left_q[i];

        msg.velocity[
            i + kArmDof] =
            left_dq[i];
    }

    joint_state_pub_->publish(
        msg);
}

void Ros2UsbBridge::publishGripperState(
    uint8_t left,
    uint8_t right)
{
    std_msgs::msg::UInt8MultiArray msg;

    msg.data = {
        left,
        right
    };

    gripper_state_pub_->publish(
        msg);
}

// ============================================================
// Frame builder
// ============================================================

std::vector<uint8_t>
Ros2UsbBridge::buildFrame(
    uint16_t cmd,
    const std::vector<uint8_t>& payload)
{
    const std::size_t total_size =
        payload.size() + 7;

    if (total_size >
        kMaxFrameSize)
    {
        throw std::runtime_error(
            "Frame too large");
    }

    std::vector<uint8_t> frame;

    frame.reserve(
        total_size);

    frame.push_back(
        0xA5);

    appendU16(
        frame,
        static_cast<uint16_t>(
            total_size));

    appendU16(
        frame,
        cmd);

    frame.insert(
        frame.end(),
        payload.begin(),
        payload.end());

    uint8_t checksum = 0;

    for (const uint8_t byte :
         payload)
    {
        checksum =
            static_cast<uint8_t>(
                checksum +
                byte);
    }

    frame.push_back(
        checksum);

    frame.push_back(
        0x5A);

    return frame;
}

// ============================================================
// Little endian helpers
// ============================================================

uint16_t Ros2UsbBridge::readU16(
    const uint8_t* p)
{
    return
        static_cast<uint16_t>(
            p[0]) |
        (static_cast<uint16_t>(
             p[1]) << 8);
}

uint32_t Ros2UsbBridge::readU32(
    const uint8_t* p)
{
    return
        static_cast<uint32_t>(
            p[0]) |
        (static_cast<uint32_t>(
             p[1]) << 8) |
        (static_cast<uint32_t>(
             p[2]) << 16) |
        (static_cast<uint32_t>(
             p[3]) << 24);
}

float Ros2UsbBridge::readFloat(
    const uint8_t* p)
{
    const uint32_t bits =
        readU32(
            p);

    float value = 0.0f;

    std::memcpy(
        &value,
        &bits,
        sizeof(value));

    return value;
}

void Ros2UsbBridge::appendU16(
    std::vector<uint8_t>& buffer,
    uint16_t value)
{
    buffer.push_back(
        static_cast<uint8_t>(
            value &
            0xFF));

    buffer.push_back(
        static_cast<uint8_t>(
            (value >> 8) &
            0xFF));
}

void Ros2UsbBridge::appendU32(
    std::vector<uint8_t>& buffer,
    uint32_t value)
{
    buffer.push_back(
        static_cast<uint8_t>(
            value &
            0xFF));

    buffer.push_back(
        static_cast<uint8_t>(
            (value >> 8) &
            0xFF));

    buffer.push_back(
        static_cast<uint8_t>(
            (value >> 16) &
            0xFF));

    buffer.push_back(
        static_cast<uint8_t>(
            (value >> 24) &
            0xFF));
}

void Ros2UsbBridge::appendFloat(
    std::vector<uint8_t>& buffer,
    float value)
{
    uint32_t bits = 0;

    static_assert(
        sizeof(float) ==
        sizeof(uint32_t));

    std::memcpy(
        &bits,
        &value,
        sizeof(bits));

    appendU32(
        buffer,
        bits);
}

// ============================================================
// main
// ============================================================

int main(
    int argc,
    char** argv)
{
    rclcpp::init(
        argc,
        argv);

    try
    {
        auto node =
            std::make_shared<
                Ros2UsbBridge>();

        rclcpp::spin(
            node);
    }
    catch (const std::exception& e)
    {
        fprintf(
            stderr,
            "eng26_usb_bridge fatal: %s\n",
            e.what());

        rclcpp::shutdown();

        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
