#include "eng26_arm_driver/protocol.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace eng26::wire;
TEST(Protocol, FirmwareGoldenVectors) {
  EXPECT_EQ(encode(Command::Stop, {}), (Bytes{0xA5, 7, 0, 0x11, 0, 0, 0x5A}));
  EXPECT_EQ(encode(Command::Heartbeat, {0x78, 0x56, 0x34, 0x12}),
    (Bytes{0xA5, 11, 0, 0x10, 0, 0x78, 0x56, 0x34, 0x12, 0x14, 0x5A}));
  Writer writer; writer.u16(0x1234); writer.u32(0x12345678); writer.f32(1.0F);
  EXPECT_EQ(writer.bytes, (Bytes{0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0, 0, 0x80, 0x3F}));
}
TEST(Protocol, BytewiseFragmentationAndCoalescing) {
  const auto first = encode(Command::Heartbeat, {0, 0, 0, 0});
  Parser parser; Frame frame;
  for (std::size_t i = 0; i < first.size(); ++i) {
    parser.feed(&first[i], 1, i);
    EXPECT_EQ(parser.next(frame, i), i + 1 == first.size());
  }
  const auto second = encode(Command::ModeState, Bytes(10, 0));
  auto coalesced = first; coalesced.insert(coalesced.end(), second.begin(), second.end());
  parser.feed(coalesced.data(), coalesced.size(), 20);
  ASSERT_TRUE(parser.next(frame, 20)); EXPECT_EQ(frame.command, Command::Heartbeat);
  ASSERT_TRUE(parser.next(frame, 20)); EXPECT_EQ(frame.command, Command::ModeState);
  EXPECT_FALSE(parser.next(frame, 20));
}
TEST(Protocol, NoiseBadChecksumAndPartialTimeout) {
  Parser parser; Frame frame;
  Bytes input{0, 0xA5, 0xFF, 0xFF};
  auto bad = encode(Command::Heartbeat, {1, 0, 0, 0}); bad[9] ^= 1;
  input.insert(input.end(), bad.begin(), bad.end());
  const auto good = encode(Command::Stop, {}); input.insert(input.end(), good.begin(), good.end());
  parser.feed(input.data(), input.size(), 10);
  ASSERT_TRUE(parser.next(frame, 10)); EXPECT_EQ(frame.command, Command::Stop);
  EXPECT_GT(parser.errors(), 0U);
  const Bytes partial{0xA5, 0, 4, 0};
  parser.feed(partial.data(), partial.size(), 20);
  EXPECT_FALSE(parser.next(frame, 121));
  parser.feed(good.data(), good.size(), 122);
  ASSERT_TRUE(parser.next(frame, 122)); EXPECT_EQ(frame.command, Command::Stop);
}
TEST(Protocol, ExactPayloadLengthsAndCounterWrap) {
  Writer servo; servo.u16(1); servo.u16(2); servo.u8(1); servo.u8(0); servo.u16(200);
  for (int i = 0; i < 6; ++i) servo.f32(0);
  EXPECT_EQ(servo.bytes.size(), 32U); EXPECT_EQ(encode(Command::Servo, servo.bytes).size(), 39U);
  Writer chunk; chunk.u16(1); chunk.u16(2); chunk.u16(3); chunk.u16(0); chunk.u8(4);
  for (int i = 0; i < 4; ++i) { chunk.u32(i); for (int j = 0; j < 6; ++j) chunk.f32(0); }
  EXPECT_EQ(chunk.bytes.size(), 121U); EXPECT_EQ(encode(Command::Chunk, chunk.bytes).size(), 128U);
  EXPECT_TRUE(newer(0, 65535)); EXPECT_FALSE(newer(65535, 0));
  EXPECT_FALSE(newer(12, 12)); EXPECT_FALSE(newer(32768, 0));
  Reader reader(servo.bytes);
  EXPECT_EQ(reader.u16(), 1); EXPECT_EQ(reader.u16(), 2); EXPECT_FALSE(reader.done());
}
