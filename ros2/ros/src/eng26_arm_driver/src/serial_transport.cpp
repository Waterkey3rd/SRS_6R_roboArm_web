#include "eng26_arm_driver/transport.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace eng26 {
namespace {
speed_t baud_constant(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return 0;
  }
}
bool healthy(int fd) {
  pollfd item{fd, static_cast<short>(POLLIN | POLLOUT), 0};
  const auto rc = ::poll(&item, 1, 0);
  return (rc >= 0 || errno == EINTR) && !(item.revents & (POLLERR | POLLHUP | POLLNVAL));
}
std::ptrdiff_t result(ssize_t count) {
  if (count >= 0) return count;
  return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ? 0 : -1;
}
}
SerialTransport::SerialTransport(std::string device, int baud) : device_(std::move(device)), baud_(baud) {}
SerialTransport::~SerialTransport() { close(); }
bool SerialTransport::open(std::string& error) {
  close();
  const auto speed = baud_constant(baud_);
  if (speed == 0) { error = "unsupported baud rate"; return false; }
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) { error = std::strerror(errno); return false; }
  termios config{};
  if (::ioctl(fd_, TIOCEXCL) < 0 || ::tcgetattr(fd_, &config) < 0) {
    error = std::strerror(errno); close(); return false;
  }
  ::cfmakeraw(&config);
  config.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  config.c_cflag |= CS8 | CLOCAL | CREAD;
  config.c_cc[VMIN] = 0; config.c_cc[VTIME] = 0;
  if (::cfsetispeed(&config, speed) < 0 || ::cfsetospeed(&config, speed) < 0 ||
      ::tcsetattr(fd_, TCSANOW, &config) < 0 || ::tcflush(fd_, TCIOFLUSH) < 0) {
    error = std::strerror(errno); close(); return false;
  }
  return true;
}
void SerialTransport::close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
std::ptrdiff_t SerialTransport::read(uint8_t* data, std::size_t size) {
  if (fd_ < 0 || !healthy(fd_)) return -1;
  return result(::read(fd_, data, size));
}
std::ptrdiff_t SerialTransport::write(const uint8_t* data, std::size_t size) {
  if (fd_ < 0 || !healthy(fd_)) return -1;
  return result(::write(fd_, data, size));
}
}  // namespace eng26
