#include "learm_bridge_cpp/serial_port.hpp"

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace learm_bridge_cpp {
namespace {
speed_t baud_value(unsigned int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw std::invalid_argument("unsupported baud rate");
  }
}
[[noreturn]] void fail(const char* operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}
}  // namespace

SerialPort::~SerialPort() { close(); }
void SerialPort::close() noexcept {
  // On Linux, do not retry close() after EINTR: the descriptor may be reused.
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}
void SerialPort::open(const std::string& path, unsigned int baud) {
  if (is_open()) throw std::logic_error("serial port is already open");
  const auto speed = baud_value(baud);
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) fail("open serial port");
  try {
    // Prevent subsequent opens; existing users must still be stopped first.
    if (::ioctl(fd_, TIOCEXCL) < 0) fail("TIOCEXCL");
    termios options{};
    if (::tcgetattr(fd_, &options) < 0) fail("tcgetattr");
    ::cfmakeraw(&options);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::cfsetispeed(&options, speed) < 0 || ::cfsetospeed(&options, speed) < 0) {
      fail("set baud rate");
    }
    if (::tcsetattr(fd_, TCSANOW, &options) < 0) fail("tcsetattr");
  } catch (...) {
    close();
    throw;
  }
}
std::size_t SerialPort::read_some(std::uint8_t* data, std::size_t capacity) {
  if (!is_open()) throw std::logic_error("serial port is closed");
  if (capacity == 0) return 0;
  if (!data) throw std::invalid_argument("null serial read buffer");
  pollfd readiness{fd_, POLLIN, 0};
  const int result = ::poll(&readiness, 1, 0);
  if (result < 0) {
    if (errno == EINTR) return 0;
    fail("poll serial port");
  }
  if (readiness.revents & (POLLHUP | POLLERR | POLLNVAL)) {
    throw std::system_error(EIO, std::generic_category(), "serial port disconnected");
  }
  if (result == 0 || !(readiness.revents & POLLIN)) return 0;
  const auto count = ::read(fd_, data, capacity);
  if (count >= 0) return static_cast<std::size_t>(count);
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
  fail("read serial port");
}
}  // namespace learm_bridge_cpp
