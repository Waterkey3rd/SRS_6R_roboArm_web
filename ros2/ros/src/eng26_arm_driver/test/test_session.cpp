#include "eng26_arm_driver/session.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <deque>

using namespace eng26;
using wire::Command;
class FakeTransport final : public Transport {
public:
  bool opened{};
  std::size_t write_limit{1024};
  wire::Bytes input, output;
  bool open(std::string&) override { opened = true; return true; }
  void close() override { opened = false; }
  bool is_open() const override { return opened; }
  std::ptrdiff_t read(uint8_t* bytes, std::size_t size) override {
    size = std::min(size, input.size()); std::copy_n(input.begin(), size, bytes);
    input.erase(input.begin(), input.begin() + size); return static_cast<std::ptrdiff_t>(size);
  }
  std::ptrdiff_t write(const uint8_t* bytes, std::size_t size) override {
    size = std::min(size, write_limit); output.insert(output.end(), bytes, bytes + size);
    return static_cast<std::ptrdiff_t>(size);
  }
  void feed(Command command, const wire::Writer& payload) {
    const auto bytes = wire::encode(command, payload.bytes); input.insert(input.end(), bytes.begin(), bytes.end());
  }
};
class SessionTest : public ::testing::Test {
protected:
  FakeTransport* link{};
  std::unique_ptr<Session> session;
  uint64_t now{1000};
  void SetUp() override {
    auto transport = std::make_unique<FakeTransport>(); link = transport.get();
    session = std::make_unique<Session>(std::move(transport), SessionConfig{}, 100);
  }
  void heartbeat(uint32_t uptime = 1000, uint8_t left_gripper = 2, uint8_t right_gripper = 2) {
    wire::Writer w; w.u32(uptime); w.u8(left_gripper); w.u8(right_gripper);
    link->feed(Command::Heartbeat, w);
  }
  void mode(uint16_t epoch, bool blocked = false, uint8_t value = 2) {
    wire::Writer w; w.u8(value); w.u16(epoch); w.u16(0); w.u16(0); w.u8(1); w.u8(blocked); w.u8(1);
    link->feed(Command::ModeState, w);
  }
  void online() {
    heartbeat(); mode(10); session->tick(now);
    ASSERT_TRUE(session->state(now).rearm_required);
    ++now; mode(11); heartbeat(1001); session->tick(now);
    ASSERT_FALSE(session->state(now).rearm_required);
    link->output.clear(); CommandResult result; while (session->pop_result(result)) {}
  }
  Submission send(Command command, bool latest = false, uint64_t ttl = 200) {
    const auto state = session->state(now);
    return session->submit(command, {}, state.session_id, state.epoch, 99, now, ttl, latest);
  }
  void ack(uint16_t seq, Command command, uint8_t result = 0, uint16_t epoch = 11) {
    wire::Writer w; w.u16(seq); w.u8(static_cast<uint8_t>(command)); w.u8(result); w.u16(epoch); w.u16(0);
    link->feed(Command::Ack, w);
  }
};
TEST_F(SessionTest, StartupStopAndExternalRearmAreRequired) {
  heartbeat(); mode(10); session->tick(now);
  EXPECT_EQ(link->output, wire::encode(Command::Stop, {}));
  EXPECT_FALSE(send(Command::Servo, true).accepted);
  mode(10, false); session->tick(++now);
  EXPECT_TRUE(session->state(now).rearm_required);
  mode(11); session->tick(++now);
  EXPECT_FALSE(session->state(now).rearm_required);
  EXPECT_TRUE(send(Command::Servo, true).accepted);
}
TEST_F(SessionTest, HeartbeatCarriesBothGripperStates) {
  heartbeat(1234, 1, 0); mode(10); session->tick(now);
  auto state = session->state(now);
  EXPECT_TRUE(state.heartbeat_fresh);
  EXPECT_EQ(state.mcu_uptime_ms, 1234U);
  EXPECT_EQ(state.left_gripper_state, 1U);
  EXPECT_EQ(state.right_gripper_state, 0U);

  wire::Writer invalid; invalid.u32(1235); invalid.u8(4); invalid.u8(2);
  link->feed(Command::Heartbeat, invalid); session->tick(++now);
  state = session->state(now);
  EXPECT_EQ(state.mcu_uptime_ms, 1234U);
  EXPECT_EQ(state.left_gripper_state, 1U);
  EXPECT_EQ(state.payload_errors, 1U);
}
TEST_F(SessionTest, LatestTargetAndAckCorrelation) {
  online(); const auto old = send(Command::Servo, true); const auto current = send(Command::Servo, true);
  CommandResult result; ASSERT_TRUE(session->pop_result(result));
  EXPECT_EQ(result.ticket, old.ticket); EXPECT_EQ(result.outcome, Outcome::Superseded);
  session->tick(++now);
  ack(1, Command::JointJog); session->tick(++now);
  EXPECT_FALSE(session->pop_result(result)); EXPECT_EQ(session->state(now).unmatched_acks, 1U);
  ack(1, Command::Servo); session->tick(++now);
  ASSERT_TRUE(session->pop_result(result)); EXPECT_EQ(result.ticket, current.ticket);
  EXPECT_EQ(result.outcome, Outcome::Ack); EXPECT_EQ(result.mcu_result, 0);
}
TEST_F(SessionTest, ModeChangeInvalidatesQueuedCommandsAndOldRosSession) {
  online(); const auto state = session->state(now); const auto request = send(Command::Hold);
  mode(12); session->tick(++now);
  CommandResult result; ASSERT_TRUE(session->pop_result(result));
  EXPECT_EQ(result.ticket, request.ticket); EXPECT_EQ(result.outcome, Outcome::Invalidated);
  EXPECT_FALSE(session->submit(Command::Hold, {}, state.session_id, state.epoch, 1, now).accepted);
}
TEST_F(SessionTest, OldModeStateCannotUndoNewerModeEvent) {
  online();
  wire::Writer event; event.u8(0); event.u32(1002); event.u8(2); event.u16(12); event.u16(0);
  link->feed(Command::Event, event); session->tick(++now);
  EXPECT_FALSE(session->state(now).mode_fresh);
  mode(11); session->tick(++now);
  EXPECT_FALSE(session->state(now).mode_fresh);
  mode(12); session->tick(++now);
  EXPECT_TRUE(session->state(now).mode_fresh);
  EXPECT_EQ(session->state(now).epoch, 12);
}
TEST_F(SessionTest, StaleEpochAckDoesNotStopNewMcuMode) {
  online(); send(Command::Hold); session->tick(++now);
  link->output.clear();
  ack(1, Command::Hold, 4, 12); session->tick(++now);
  EXPECT_TRUE(link->output.empty());
  EXPECT_FALSE(session->state(now).mode_fresh);
  mode(11); session->tick(++now);
  EXPECT_FALSE(session->state(now).mode_fresh);
  mode(12, false, 1); session->tick(++now);
  EXPECT_TRUE(session->state(now).mode_fresh);
  EXPECT_EQ(session->state(now).mode, wire::Mode::Remote);
  EXPECT_TRUE(link->output.empty());
}
TEST_F(SessionTest, AckTimeoutDoesNotRetransmitAndRequestsStop) {
  online(); const auto request = send(Command::Hold); session->tick(++now);
  heartbeat(1201); mode(11); now += 201; session->tick(now);
  CommandResult result; ASSERT_TRUE(session->pop_result(result));
  EXPECT_EQ(result.ticket, request.ticket); EXPECT_EQ(result.outcome, Outcome::Timeout);
  EXPECT_TRUE(session->state(now).rearm_required);
  wire::Parser parser; parser.feed(link->output.data(), link->output.size(), now);
  wire::Frame frame; int holds = 0, stops = 0;
  while (parser.next(frame, now)) { holds += frame.command == Command::Hold; stops += frame.command == Command::Stop; }
  EXPECT_EQ(holds, 1); EXPECT_EQ(stops, 1);
}
TEST_F(SessionTest, StopNeverInterleavesWithPartiallyWrittenFrame) {
  online(); link->write_limit = 2; send(Command::Hold); session->tick(++now);
  session->stop(1, now);
  for (int i = 0; i < 12; ++i) session->tick(++now);
  wire::Parser parser; parser.feed(link->output.data(), link->output.size(), now);
  wire::Frame frame;
  ASSERT_TRUE(parser.next(frame, now)); EXPECT_EQ(frame.command, Command::Hold);
  ASSERT_TRUE(parser.next(frame, now)); EXPECT_EQ(frame.command, Command::Stop);
  EXPECT_EQ(parser.errors(), 0U);
}
TEST_F(SessionTest, ADelayedHeartbeatCannotHideExecutorStall) {
  online(); const auto request = send(Command::Hold);
  now += 301; heartbeat(1002); session->tick(now);
  EXPECT_FALSE(session->state(now).connected);
  CommandResult result; ASSERT_TRUE(session->pop_result(result));
  EXPECT_EQ(result.ticket, request.ticket); EXPECT_EQ(result.outcome, Outcome::Invalidated);
}
TEST_F(SessionTest, BackpressureAndExpiry) {
  online();
  for (int i = 0; i < 8; ++i) EXPECT_TRUE(send(Command::Hold, false, 1).accepted);
  EXPECT_FALSE(send(Command::Hold).accepted);
  now += 2; session->tick(now);
  EXPECT_TRUE(link->output.empty());
}
TEST_F(SessionTest, MailboxTrafficDoesNotAdvanceDiscreteSequence) {
  online(); send(Command::Hold); session->tick(++now); ack(1, Command::Hold); session->tick(++now);
  for (uint16_t i = 1; i <= 20; ++i) {
    send(Command::Servo, true); session->tick(++now); ack(i, Command::Servo); session->tick(++now);
  }
  const auto request = send(Command::Hold); session->tick(++now); ack(2, Command::Hold); session->tick(++now);
  CommandResult result; bool found = false;
  while (session->pop_result(result)) if (result.ticket == request.ticket) {
    EXPECT_EQ(result.sequence, 2); EXPECT_EQ(result.outcome, Outcome::Ack); found = true;
  }
  EXPECT_TRUE(found);
}
