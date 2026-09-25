#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace eng26 {
// Single-owner, nonblocking I/O; positive = byte count, zero = would block,
// negative = disconnected/error. Implement this interface for a simulator.
class Transport {
public:
  virtual ~Transport() = default;
  virtual bool open(std::string& error) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual std::ptrdiff_t read(uint8_t* data, std::size_t size) = 0;
  virtual std::ptrdiff_t write(const uint8_t* data, std::size_t size) = 0;
};
class SerialTransport final : public Transport {
public:
  SerialTransport(std::string device, int baud);
  ~SerialTransport() override;
  bool open(std::string& error) override;
  void close() override;
  bool is_open() const override { return fd_ >= 0; }
  std::ptrdiff_t read(uint8_t* data, std::size_t size) override;
  std::ptrdiff_t write(const uint8_t* data, std::size_t size) override;
private:
  std::string device_;
  int baud_;
  int fd_{-1};
};
}  // namespace eng26
