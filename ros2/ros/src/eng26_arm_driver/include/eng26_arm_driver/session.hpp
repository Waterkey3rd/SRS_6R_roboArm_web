#pragma once
#include "eng26_arm_driver/protocol.hpp"
#include "eng26_arm_driver/transport.hpp"
#include <array>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace eng26 {
struct SessionConfig {
  uint64_t heartbeat_period_ms{50}, heartbeat_timeout_ms{300};
  uint64_t mode_timeout_ms{300}, ack_timeout_ms{200}, write_timeout_ms{100};
  uint64_t reconnect_ms{1000};
};
enum class Outcome : uint8_t { Ack, Rejected, Timeout, Invalidated, Superseded, WrittenUnconfirmed };
struct CommandResult {
  uint64_t ticket{}, session_id{}, client_id{};
  wire::Command command{};
  uint16_t sequence{}, epoch{}, detail{};
  Outcome outcome{Outcome::Rejected};
  uint8_t mcu_result{255};
  std::string message;
};
struct Submission { bool accepted{}; uint64_t ticket{}; std::string message; };
struct SessionState {
  bool connected{}, mode_known{}, heartbeat_fresh{}, mode_fresh{};
  bool peer_online{}, control_blocked{true}, snapshot_valid{}, rearm_required{true};
  wire::Mode mode{wire::Mode::Idle};
  uint16_t epoch{}, safety_flags{}, fault_code{};
  uint8_t left_gripper_state{255}, right_gripper_state{255};
  uint64_t session_id{}, heartbeat_age_ms{}, mode_age_ms{};
  uint64_t framing_errors{}, payload_errors{}, reconnects{}, ack_timeouts{}, unmatched_acks{};
  uint32_t mcu_uptime_ms{};
  std::string last_error;
};

// No ROS or thread dependencies. All methods belong to ONE execution context.
// tick() performs at most four 1024-byte reads and one write; never sleeps.
class Session {
public:
  Session(std::unique_ptr<Transport> transport, SessionConfig config, uint64_t session_seed);
  ~Session();
  void tick(uint64_t now_ms);
  SessionState state(uint64_t now_ms) const;
  std::string gate(wire::Command command, uint64_t session_id, uint16_t epoch, uint64_t now_ms) const;
  Submission submit(wire::Command command, wire::Bytes body, uint64_t session_id,
                    uint16_t epoch, uint64_t client_id, uint64_t now_ms,
                    uint64_t ttl_ms = 200, bool latest = false);
  Submission stop(uint64_t client_id, uint64_t now_ms);
  bool pop_result(CommandResult& result);
  // Called synchronously during tick after protocol/session validation.
  std::function<void(const wire::Frame&)> on_telemetry;
private:
  struct Request {
    CommandResult result;
    wire::Bytes body;
    uint64_t expires_ms{};
  };
  struct Pending { Request request; uint64_t sent_ms{}; };
  void receive(const wire::Frame& frame, uint64_t now_ms);
  void invalidate(const std::string& reason);
  void disconnect(const std::string& reason, uint64_t now_ms);
  void finish(Request request, Outcome outcome, const std::string& message, uint8_t result = 255, uint16_t detail = 0);
  void rotate_session();
  void select_transmit(uint64_t now_ms);
  uint16_t next_sequence(wire::Command command);
  std::unique_ptr<Transport> transport_;
  SessionConfig config_;
  SessionState state_;
  wire::Parser parser_;
  std::deque<Request> commands_;
  std::array<std::optional<Request>, 3> latest_;
  std::optional<Pending> pending_;
  std::optional<Request> sending_;
  std::optional<Request> stop_request_;
  std::deque<CommandResult> results_;
  wire::Bytes tx_;
  wire::Command tx_command_{wire::Command::Heartbeat};
  std::size_t tx_offset_{};
  uint64_t tx_started_ms_{}, last_heartbeat_tx_{}, last_heartbeat_rx_{}, last_mode_rx_{}, retry_at_ms_{};
  uint64_t next_ticket_{1};
  std::array<uint16_t, 5> sequences_{};
  std::size_t realtime_cursor_{};
  bool heartbeat_seen_{}, initialized_{}, stop_written_{}, awaiting_mode_{};
  std::optional<uint16_t> expected_mode_epoch_;
  uint16_t stop_epoch_{};
};
}  // namespace eng26
