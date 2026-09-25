#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace eng26::wire {
using Bytes = std::vector<uint8_t>;
constexpr std::size_t kMaxFrame = 1024;
constexpr std::size_t kJoints = 7;
enum class Command : uint16_t {
  Heartbeat = 0x10, Stop = 0x11, Servo = 0x20, JointJog = 0x21,
  CartesianJog = 0x22, Hold = 0x30, ClearFault = 0x31, Start = 0x32,
  Cancel = 0x33, Begin = 0x40, Chunk = 0x41, Commit = 0x42,
  JointState = 0x80, ModeState = 0x81, Diagnostics = 0x82, Ack = 0x83, Event = 0x84
};
enum class Mode : uint8_t { Idle, Remote, PcDebug, RosTrajectory, Vla, Homing, Fault, EStop };
struct Frame { Command command{}; Bytes payload; };
struct Writer {
  Bytes bytes;
  void u8(uint8_t value);
  void u16(uint16_t value);
  void u32(uint32_t value);
  void f32(float value);
  template<std::size_t N> void floats(const std::array<float, N>& values) {
    for (const auto value : values) f32(value);
  }
};
class Reader {
public:
  explicit Reader(const Bytes& bytes) : bytes_(bytes) {}
  uint8_t u8();
  uint16_t u16();
  uint32_t u32();
  float f32();
  template<std::size_t N> void floats(std::array<float, N>& values) {
    for (auto& value : values) value = f32();
  }
  bool done() const { return valid_ && offset_ == bytes_.size(); }
private:
  const Bytes& bytes_;
  std::size_t offset_{};
  bool valid_{true};
};
Bytes encode(Command command, const Bytes& payload);
bool newer(uint16_t value, uint16_t previous);

// Bounded byte-stream parser. A bad candidate advances by one byte, not its
// untrusted length. A stalled partial frame is discarded after 100 ms.
class Parser {
public:
  void feed(const uint8_t* bytes, std::size_t size, uint64_t now_ms);
  bool next(Frame& frame, uint64_t now_ms);
  void reset();
  uint64_t errors() const { return errors_; }
private:
  Bytes buffer_;
  uint64_t last_byte_ms_{};
  uint64_t errors_{};
};
}  // namespace eng26::wire
