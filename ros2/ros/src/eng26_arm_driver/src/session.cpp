#include "eng26_arm_driver/session.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace eng26 {
using wire::Command;
Session::Session(std::unique_ptr<Transport> transport, SessionConfig config, uint64_t seed)
: transport_(std::move(transport)), config_(config) {
  if (!transport_ || config.heartbeat_period_ms == 0 || config.heartbeat_period_ms >= 100 ||
      config.heartbeat_timeout_ms == 0 || config.mode_timeout_ms == 0 ||
      config.ack_timeout_ms == 0 || config.write_timeout_ms == 0 || config.reconnect_ms == 0)
    throw std::invalid_argument("invalid transport/session timing configuration");
  state_.session_id = seed == 0 ? 1 : seed;
}
Session::~Session() { transport_->close(); }
void Session::rotate_session() { if (++state_.session_id == 0) ++state_.session_id; }
SessionState Session::state(uint64_t now) const {
  auto out = state_;
  out.connected = transport_->is_open();
  out.heartbeat_age_ms = heartbeat_seen_ ? now - last_heartbeat_rx_ : std::numeric_limits<uint64_t>::max();
  out.mode_age_ms = initialized_ ? now - last_mode_rx_ : std::numeric_limits<uint64_t>::max();
  out.heartbeat_fresh = out.connected && heartbeat_seen_ && out.heartbeat_age_ms < config_.heartbeat_timeout_ms;
  out.mode_fresh = out.connected && initialized_ && !awaiting_mode_ && out.mode_age_ms < config_.mode_timeout_ms;
  out.framing_errors = parser_.errors();
  return out;
}
std::string Session::gate(Command cmd, uint64_t session, uint16_t epoch, uint64_t now) const {
  const auto s = state(now);
  if (!s.connected || !s.heartbeat_fresh || !s.mode_fresh || !s.peer_online) return "link/mode state is not fresh and online";
  if (session != s.session_id || epoch != s.epoch) return "stale ROS session or MCU epoch";
  if (cmd == Command::Hold || cmd == Command::ClearFault) return {};
  if (s.rearm_required || s.control_blocked) return "MCU must rearm with a new epoch";
  if (!s.snapshot_valid) return "MCU has no valid snapshot for this epoch";
  if (cmd == Command::Servo && (s.mode == wire::Mode::PcDebug || s.mode == wire::Mode::Vla)) return {};
  if ((cmd == Command::JointJog || cmd == Command::CartesianJog) && s.mode == wire::Mode::PcDebug) return {};
  if ((cmd == Command::Begin || cmd == Command::Chunk || cmd == Command::Commit ||
       cmd == Command::Start || cmd == Command::Cancel) && s.mode == wire::Mode::RosTrajectory) return {};
  return "command is not permitted in the current MCU mode";
}
void Session::finish(Request request, Outcome outcome, const std::string& message, uint8_t result, uint16_t detail) {
  request.result.outcome = outcome; request.result.message = message;
  request.result.mcu_result = result; request.result.detail = detail;
  // ROS adapter drains on every callback/tick. Bounded even if another adapter fails to drain.
  if (results_.size() == 256) results_.pop_front();
  results_.push_back(std::move(request.result));
}
bool Session::pop_result(CommandResult& result) {
  if (results_.empty()) return false;
  result = std::move(results_.front()); results_.pop_front(); return true;
}
Submission Session::submit(Command cmd, wire::Bytes body, uint64_t session, uint16_t epoch,
                           uint64_t client_id, uint64_t now, uint64_t ttl, bool latest) {
  Request request;
  request.result.ticket = next_ticket_++;
  request.result.client_id = client_id; request.result.session_id = session;
  request.result.command = cmd; request.result.epoch = epoch;
  request.body = std::move(body); request.expires_ms = now + ttl;
  auto error = gate(cmd, session, epoch, now);
  if (ttl == 0 || ttl > 2000 || request.body.size() > wire::kMaxFrame - 11) error = "invalid request size or TTL";
  const int index = static_cast<int>(cmd) - static_cast<int>(Command::Servo);
  if (latest && (index < 0 || index >= 3)) error = "invalid realtime command";
  if (!latest && commands_.size() >= 8) error = "driver command queue full";
  const auto ticket = request.result.ticket;
  if (!error.empty()) { finish(std::move(request), Outcome::Rejected, error); return {false, ticket, error}; }
  if (latest) {
    auto& slot = latest_[static_cast<std::size_t>(index)];
    if (slot) finish(std::move(*slot), Outcome::Superseded, "replaced by a newer realtime target");
    slot = std::move(request);
  } else commands_.push_back(std::move(request));
  return {true, ticket, "queued locally; observe command_results for MCU ACK"};
}
void Session::invalidate(const std::string& reason) {
  if (pending_) { finish(std::move(pending_->request), Outcome::Invalidated, reason); pending_.reset(); }
  if (sending_) { finish(std::move(*sending_), Outcome::Invalidated, reason); sending_.reset(); }
  // A partially written frame must finish before STOP; cutting it would corrupt the stream.
  if (tx_offset_ == 0 && tx_command_ != Command::Stop) tx_.clear();
  for (auto& request : commands_) finish(std::move(request), Outcome::Invalidated, reason);
  commands_.clear();
  for (auto& request : latest_) {
    if (request) finish(std::move(*request), Outcome::Invalidated, reason);
    request.reset();
  }
  rotate_session();
}
Submission Session::stop(uint64_t client_id, uint64_t now) {
  invalidate("STOP requested; accepted/sent commands may already have reached MCU");
  state_.rearm_required = true; stop_written_ = false;
  stop_epoch_ = state_.epoch;
  if (stop_request_) finish(std::move(*stop_request_), Outcome::Superseded, "STOP coalesced");
  Request request;
  request.result.ticket = next_ticket_++; request.result.client_id = client_id;
  request.result.command = Command::Stop; request.result.session_id = state_.session_id;
  request.result.epoch = state_.epoch; request.expires_ms = now + 1000;
  const auto ticket = request.result.ticket;
  if (!transport_->is_open()) {
    finish(std::move(request), Outcome::Rejected, "serial port is closed; STOP delivery unavailable");
    return {false, ticket, "serial port is closed"};
  }
  stop_request_ = std::move(request);
  return {true, ticket, "STOP queued; protocol has no STOP ACK"};
}
void Session::disconnect(const std::string& reason, uint64_t now) {
  invalidate(reason);
  if (stop_request_) { finish(std::move(*stop_request_), Outcome::Invalidated, reason); stop_request_.reset(); }
  transport_->close(); parser_.reset(); tx_.clear(); tx_offset_ = 0;
  heartbeat_seen_ = initialized_ = stop_written_ = awaiting_mode_ = false;
  expected_mode_epoch_.reset();
  state_.mode_known = false; state_.peer_online = false; state_.snapshot_valid = false;
  state_.left_gripper_state = state_.right_gripper_state = 255;
  state_.rearm_required = true; state_.control_blocked = true; state_.last_error = reason;
  retry_at_ms_ = now + config_.reconnect_ms;
}
void Session::receive(const wire::Frame& frame, uint64_t now) {
  wire::Reader r(frame.payload);
  const auto size = frame.payload.size();
  switch (frame.command) {
    case Command::Heartbeat: {
      if (size != 6) break;
      const auto uptime = r.u32();
      const auto left_gripper = r.u8(); const auto right_gripper = r.u8();
      if (left_gripper > 3 || right_gripper > 3) break;
      if (heartbeat_seen_ && static_cast<uint32_t>(uptime - state_.mcu_uptime_ms) >= 0x80000000U) {
        disconnect("MCU uptime moved backwards; restart handshake", now); return;
      }
      state_.mcu_uptime_ms = uptime;
      state_.left_gripper_state = left_gripper;
      state_.right_gripper_state = right_gripper;
      heartbeat_seen_ = true; last_heartbeat_rx_ = now;
      return;
    }
    case Command::ModeState: {
      if (size != 10) break;
      const auto mode = r.u8(); const auto epoch = r.u16();
      const auto safety = r.u16(); const auto fault = r.u16();
      const auto online = r.u8(); const auto blocked = r.u8(); const auto snapshot = r.u8();
      if (mode > 7 || online > 1 || blocked > 1 || snapshot > 1) break;
      if (expected_mode_epoch_ && epoch != *expected_mode_epoch_ && !wire::newer(epoch, *expected_mode_epoch_)) return;
      if (initialized_ && epoch != state_.epoch && !wire::newer(epoch, state_.epoch)) return;
      if (initialized_ && epoch == state_.epoch && mode != static_cast<uint8_t>(state_.mode)) {
        disconnect("MCU changed mode without changing epoch", now); return;
      }
      const bool first = !initialized_;
      const bool changed = !first && epoch != state_.epoch;
      if (changed) { invalidate("MCU epoch changed"); sequences_.fill(0); }
      if (!first && epoch == state_.epoch && (!online || blocked) && !state_.rearm_required) {
        stop(0, now);
      }
      state_.mode = static_cast<wire::Mode>(mode); state_.epoch = epoch;
      state_.safety_flags = safety; state_.fault_code = fault;
      state_.peer_online = online != 0; state_.control_blocked = blocked != 0;
      state_.snapshot_valid = snapshot != 0; state_.mode_known = true;
      initialized_ = true; awaiting_mode_ = false; last_mode_rx_ = now;
      expected_mode_epoch_.reset();
      if (first) {
        sequences_.fill(0);
        // Reopening a serial port does not reset the MCU's sequence filters.
        // Establish a clean epoch via STOP and external MCU rearm, never guess seq.
        stop(0, now);
      } else if (stop_written_ && wire::newer(epoch, stop_epoch_) && !blocked && online) {
        state_.rearm_required = false;
      }
      if (on_telemetry) on_telemetry(frame);
      return;
    }
    case Command::Ack: {
      if (size != 8) break;
      const auto sequence = r.u16(); const auto command = r.u8(); const auto result = r.u8();
      const auto epoch = r.u16(); const auto detail = r.u16();
      if (result > 12) break;
      if (!pending_ || pending_->request.result.sequence != sequence ||
          static_cast<uint16_t>(pending_->request.result.command) != command ||
          (epoch != pending_->request.result.epoch &&
           !((result == 4 || result == 5) && wire::newer(epoch, pending_->request.result.epoch)))) {
        ++state_.unmatched_acks; return;
      }
      auto request = std::move(pending_->request); pending_.reset();
      const bool newer_context = epoch != request.result.epoch;
      finish(std::move(request), Outcome::Ack, result == 0 ? "MCU communication layer accepted" : "MCU rejected command", result, detail);
      if (result == 4 || result == 5) {
        if (newer_context) {
          invalidate("ACK reports a newer MCU epoch; waiting for ModeState");
          expected_mode_epoch_ = epoch;
        } else stop(0, now);
        awaiting_mode_ = true;
      }
      return;
    }
    case Command::JointState: {
      if (size != 118) break;
      r.u32(); const auto epoch = r.u16();
      bool finite = true;
      for (int i = 0; i < 28; ++i) finite = std::isfinite(r.f32()) && finite;
      if (!finite) break;
      const auto s = state(now);
      if (!s.mode_fresh || !s.heartbeat_fresh || !s.snapshot_valid || epoch != s.epoch) return;
      if (on_telemetry) on_telemetry(frame);
      return;
    }
    case Command::Diagnostics:
      if (size != 28) break;
      if (on_telemetry) on_telemetry(frame);
      return;
    case Command::Event: {
      if (size != 10) break;
      const auto type = r.u8(); r.u32(); const auto mode = r.u8(); const auto epoch = r.u16(); r.u16();
      if (type > 5 || mode > 7) break;
      if (initialized_ && type == 0 && wire::newer(epoch, state_.epoch)) {
        if (!awaiting_mode_) invalidate("mode-change event; waiting for authoritative ModeState");
        awaiting_mode_ = true;
        if (!expected_mode_epoch_ || wire::newer(epoch, *expected_mode_epoch_)) expected_mode_epoch_ = epoch;
      }
      if (initialized_ && epoch == state_.epoch && (type == 1 || type == 4 || type == 5) && !state_.rearm_required)
        stop(0, now);
      if (on_telemetry) on_telemetry(frame);
      return;
    }
    default: break;
  }
  ++state_.payload_errors;
}
uint16_t Session::next_sequence(Command command) {
  // Mailboxes are independent; discrete commands share ONE counter. Upload seq
  // is only an ACK correlation ID. A fast servo stream cannot age out Hold seq.
  std::size_t index = 4;
  if (command >= Command::Servo && command <= Command::CartesianJog)
    index = static_cast<std::size_t>(command) - static_cast<std::size_t>(Command::Servo);
  else if (command >= Command::Hold && command <= Command::Cancel) index = 3;
  return ++sequences_[index];
}
void Session::select_transmit(uint64_t now) {
  if (!tx_.empty()) return;
  sending_.reset(); tx_offset_ = 0; tx_started_ms_ = now;
  if (stop_request_) {
    tx_command_ = Command::Stop; sending_ = std::move(stop_request_); stop_request_.reset();
    tx_ = wire::encode(Command::Stop, {}); return;
  }
  if (now - last_heartbeat_tx_ >= config_.heartbeat_period_ms) {
    wire::Writer body; body.u32(static_cast<uint32_t>(now));
    tx_command_ = Command::Heartbeat; tx_ = wire::encode(tx_command_, body.bytes); return;
  }
  if (pending_) return;  // one command in flight; independent heartbeat/STOP bypass
  // Maximum eleven candidates, so stale queues cannot monopolize the executor.
  for (int attempt = 0; attempt < 11; ++attempt) {
    std::optional<Request> candidate;
    if (!commands_.empty()) { candidate = std::move(commands_.front()); commands_.pop_front(); }
    else {
      for (std::size_t i = 0; i < 3; ++i) {
        const auto index = (realtime_cursor_ + i) % 3;
        if (latest_[index]) {
          candidate = std::move(latest_[index]); latest_[index].reset();
          realtime_cursor_ = (index + 1) % 3; break;
        }
      }
    }
    if (!candidate) return;
    auto error = gate(candidate->result.command, candidate->result.session_id, candidate->result.epoch, now);
    if (now >= candidate->expires_ms) error = "request expired before transmission";
    if (!error.empty()) { finish(std::move(*candidate), Outcome::Rejected, error); continue; }
    candidate->result.sequence = next_sequence(candidate->result.command);
    wire::Writer payload; payload.u16(candidate->result.sequence); payload.u16(candidate->result.epoch);
    payload.bytes.insert(payload.bytes.end(), candidate->body.begin(), candidate->body.end());
    tx_command_ = candidate->result.command; tx_ = wire::encode(tx_command_, payload.bytes);
    sending_ = std::move(candidate); return;
  }
}
void Session::tick(uint64_t now) {
  if (!transport_->is_open()) {
    if (now < retry_at_ms_) return;
    std::string error;
    if (!transport_->open(error)) { state_.last_error = error; retry_at_ms_ = now + config_.reconnect_ms; return; }
    ++state_.reconnects; rotate_session();
    last_heartbeat_tx_ = now >= config_.heartbeat_period_ms ? now - config_.heartbeat_period_ms : 0;
    retry_at_ms_ = now + config_.heartbeat_timeout_ms;
  }
  // Check before reading: queued old input after an executor stall must not
  // refresh a lapsed session. On reopen, wait for fresh heartbeat + ModeState.
  if ((heartbeat_seen_ && now - last_heartbeat_rx_ >= config_.heartbeat_timeout_ms) ||
      (!heartbeat_seen_ && now >= retry_at_ms_) ||
      (initialized_ && now - last_mode_rx_ >= config_.mode_timeout_ms)) {
    disconnect("heartbeat/ModeState timeout; commands invalidated", now); return;
  }
  std::array<uint8_t, 1024> bytes{};
  for (int i = 0; i < 4; ++i) {
    const auto count = transport_->read(bytes.data(), bytes.size());
    if (count < 0) { disconnect("serial read failed", now); return; }
    if (count > 0) parser_.feed(bytes.data(), static_cast<std::size_t>(count), now);
    wire::Frame frame;
    while (parser_.next(frame, now)) {
      receive(frame, now);
      if (!transport_->is_open()) return;
    }
    if (count == 0) break;
  }
  if (pending_ && now - pending_->sent_ms >= config_.ack_timeout_ms) {
    ++state_.ack_timeouts;
    auto request = std::move(pending_->request); pending_.reset();
    finish(std::move(request), Outcome::Timeout, "ACK timeout: execution/acceptance is unknown; no automatic retry");
    stop(0, now);
  }
  select_transmit(now);
  if (tx_.empty()) return;
  if (now - tx_started_ms_ >= config_.write_timeout_ms) { disconnect("serial write timeout", now); return; }
  const auto count = transport_->write(tx_.data() + tx_offset_, tx_.size() - tx_offset_);
  if (count < 0) { disconnect("serial write failed", now); return; }
  tx_offset_ += static_cast<std::size_t>(count);
  if (tx_offset_ != tx_.size()) return;
  if (tx_command_ == Command::Heartbeat) last_heartbeat_tx_ = now;
  if (tx_command_ == Command::Stop) {
    stop_written_ = true; stop_epoch_ = state_.epoch;
    if (sending_) finish(std::move(*sending_), Outcome::WrittenUnconfirmed, "STOP bytes handed to OS; physical stop is unconfirmed");
  } else if (sending_) pending_ = Pending{std::move(*sending_), now};
  sending_.reset(); tx_.clear(); tx_offset_ = 0;
}
}  // namespace eng26
