#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace learm_bridge_cpp {

// Linux userspace UART adapter. The object exclusively owns its descriptor.
// Read-only API: this migration stage cannot send motor commands.
class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  SerialPort(SerialPort&&) = delete;
  SerialPort& operator=(SerialPort&&) = delete;

  void open(const std::string& path, unsigned int baud = 115200);
  void close() noexcept;
  bool is_open() const noexcept { return fd_ >= 0; }
  // Nonblocking: zero means no bytes available; errors throw system_error.
  std::size_t read_some(std::uint8_t* data, std::size_t capacity);

 private:
  int fd_{-1};
};

}  // namespace learm_bridge_cpp
