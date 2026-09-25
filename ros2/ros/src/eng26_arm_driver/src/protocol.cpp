#include "eng26_arm_driver/protocol.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace eng26::wire {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
void Writer::u8(uint8_t value) { bytes.push_back(value); }
void Writer::u16(uint16_t value) { u8(static_cast<uint8_t>(value)); u8(static_cast<uint8_t>(value >> 8)); }
void Writer::u32(uint32_t value) { u16(static_cast<uint16_t>(value)); u16(static_cast<uint16_t>(value >> 16)); }
void Writer::f32(float value) { uint32_t bits; std::memcpy(&bits, &value, 4); u32(bits); }
uint8_t Reader::u8() {
  if (offset_ == bytes_.size()) { valid_ = false; return 0; }
  return bytes_[offset_++];
}
uint16_t Reader::u16() { const auto lo = u8(); return static_cast<uint16_t>(lo | (uint16_t{u8()} << 8)); }
uint32_t Reader::u32() { const auto lo = u16(); return uint32_t{lo} | (uint32_t{u16()} << 16); }
float Reader::f32() { const auto bits = u32(); float value; std::memcpy(&value, &bits, 4); return value; }
Bytes encode(Command command, const Bytes& payload) {
  if (payload.size() > kMaxFrame - 7) throw std::length_error("HostComputer payload exceeds 1017 bytes");
  Writer out;
  out.u8(0xA5); out.u16(static_cast<uint16_t>(payload.size() + 7));
  out.u16(static_cast<uint16_t>(command));
  uint8_t checksum = 0;
  for (auto byte : payload) { out.u8(byte); checksum = static_cast<uint8_t>(checksum + byte); }
  out.u8(checksum); out.u8(0x5A);
  return out.bytes;
}
bool newer(uint16_t value, uint16_t previous) {
  const auto difference = static_cast<uint16_t>(value - previous);
  return difference != 0 && difference < 0x8000;
}
void Parser::reset() { buffer_.clear(); last_byte_ms_ = 0; }
void Parser::feed(const uint8_t* bytes, std::size_t size, uint64_t now_ms) {
  if (size == 0) return;
  if (!buffer_.empty() && now_ms - last_byte_ms_ >= 100) { buffer_.clear(); ++errors_; }
  if (size > 4096) { bytes += size - 4096; size = 4096; ++errors_; }
  if (buffer_.size() + size > 4096) { buffer_.clear(); ++errors_; }
  buffer_.insert(buffer_.end(), bytes, bytes + size);
  last_byte_ms_ = now_ms;
}
bool Parser::next(Frame& frame, uint64_t now_ms) {
  while (!buffer_.empty()) {
    const auto start = std::find(buffer_.begin(), buffer_.end(), 0xA5);
    if (start != buffer_.begin()) { buffer_.erase(buffer_.begin(), start); ++errors_; }
    if (buffer_.size() < 3) break;
    const std::size_t length = buffer_[1] | (uint16_t{buffer_[2]} << 8);
    if (length < 7 || length > kMaxFrame) { buffer_.erase(buffer_.begin()); ++errors_; continue; }
    if (buffer_.size() < length) break;
    uint8_t checksum = 0;
    for (std::size_t i = 5; i < length - 2; ++i) checksum = static_cast<uint8_t>(checksum + buffer_[i]);
    if (buffer_[length - 1] != 0x5A || buffer_[length - 2] != checksum) {
      buffer_.erase(buffer_.begin()); ++errors_; continue;
    }
    frame.command = static_cast<Command>(buffer_[3] | (uint16_t{buffer_[4]} << 8));
    frame.payload.assign(buffer_.begin() + 5, buffer_.begin() + length - 2);
    buffer_.erase(buffer_.begin(), buffer_.begin() + length);
    return true;
  }
  if (!buffer_.empty() && now_ms - last_byte_ms_ >= 100) { buffer_.clear(); ++errors_; }
  return false;
}
}  // namespace eng26::wire
