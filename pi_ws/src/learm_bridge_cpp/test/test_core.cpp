#include "learm_bridge_cpp/odometry.hpp"
#include "learm_bridge_cpp/protocol.hpp"
#include "learm_bridge_cpp/serial_port.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <pty.h>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace b = learm_bridge_cpp;
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected) {
  check(std::abs(actual - expected) < 1e-10, "odometry mismatch");
}
const std::vector<std::uint8_t> captured{
  0x03,0x01,0x81,0x02,0x0c,0x01,0x02,0x20,0x03,0x01,0x04,0x01,0x03,0x01,0x04,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x10,0x02,0x07,0x01,0x01,0x05,0x6f,0xf5,0x3d,0x3b,0x00};

void protocol_tests() {
  const std::array<std::uint8_t, 9> digits{'1','2','3','4','5','6','7','8','9'};
  check(b::crc32c(digits.data(), digits.size()) == 0xe3069283U, "CRC32C known vector");
  b::ProtocolDecoder decoder;
  check(decoder.feed(captured.data(), 17).empty(), "fragment emitted early");
  auto frames = decoder.feed(captured.data() + 17, captured.size() - 17);
  check(frames.size() == 1 && frames[0].timestamp_ms == 1025, "captured frame");
  const auto state = b::decode_chassis_state(frames[0]);
  check(state.encoders == std::array<std::int32_t, 4>{0,0,0,0}, "captured counts");
  check(state.timestamp_ms == 1025 && state.fault_flags == 16 && state.safety_state == 7,
        "captured state");
  decoder.feed(captured);
  check(decoder.stats().duplicate_frames == 1, "duplicates");
  auto corrupt = captured;
  corrupt[corrupt.size() - 3] ^= 1;
  check(decoder.feed(corrupt).empty() && decoder.stats().crc_errors == 1, "CRC reject");
  std::vector<std::uint8_t> noise(300, 0xff);
  noise.push_back(0);
  decoder.feed(noise);
  check(decoder.stats().overflow_errors == 1, "overflow bounded");
  check(decoder.feed(captured).size() == 1, "overflow recovery");
  b::ProtocolDecoder noisy;
  noise = {0xff};
  noise.insert(noise.end(), captured.begin(), captured.end());
  noise.insert(noise.end(), captured.begin(), captured.end());
  check(noisy.feed(noise).size() == 1 && noisy.stats().cobs_errors == 1, "reset noise recovery");
  b::Frame wrong;
  try { b::decode_chassis_state(wrong); throw std::runtime_error("accepted bad payload"); }
  catch (const std::invalid_argument&) {}
}

void odometry_tests() {
  b::MecanumOdometry odom;
  near(odom.update({100,200,300,400}, 1).x_m, 0);
  odom.reset({0,0,0,0}, 1);
  const auto f = odom.update({100,-100,-100,100}, 1.1);
  near(f.x_m, 100 * odom.meters_per_tick());
  near(f.vx_m_s, 1000 * odom.meters_per_tick());
  near(f.y_m, 0);
  odom.reset({0,0,0,0}, 1);
  const auto l = odom.update({100,100,100,100}, 1.1);
  near(l.y_m, 100 * odom.meters_per_tick());
  near(l.x_m, 0);
  odom.reset({0,0,0,0}, 1);
  check(odom.update({100,100,-100,-100}, 1.1).wz_rad_s > 0, "CCW mapping");
  odom.reset({0,0,0,0}, 1);
  near(odom.update({100,100,100,100}, 1).y_m, 0);
  near(odom.update({200,200,200,200}, 1.1).y_m, 100 * odom.meters_per_tick());
  // Widen before subtracting int32 boundaries; this is not a rollover correction.
  odom.reset({std::numeric_limits<std::int32_t>::min(),0,0,0}, 1);
  check(std::isfinite(odom.update({std::numeric_limits<std::int32_t>::max(),0,0,0}, 2).x_m),
        "int32 subtraction overflow");
  try { b::MecanumOdometry invalid(0); throw std::runtime_error("invalid geometry accepted"); }
  catch (const std::invalid_argument&) {}
}

struct Pty {
  int master{-1}, slave{-1};
  ~Pty() { if (master >= 0) ::close(master); if (slave >= 0) ::close(slave); }
};
void serial_tests() {
  Pty pty;
  std::array<char, 128> name{};
  check(::openpty(&pty.master, &pty.slave, name.data(), nullptr, nullptr) == 0, "openpty");
  b::SerialPort port;
  port.open(name.data());
  termios config{};
  check(::tcgetattr(pty.slave, &config) == 0, "inspect termios");
  check((config.c_cflag & CSIZE) == CS8 && !(config.c_cflag & (PARENB | CSTOPB)), "8N1");
  check(!(config.c_lflag & (ICANON | ECHO)) && ::cfgetispeed(&config) == B115200, "raw baud");
  std::array<std::uint8_t, 128> buffer{};
  check(port.read_some(buffer.data(), buffer.size()) == 0, "nonblocking empty");
  check(::write(pty.master, captured.data(), captured.size()) == static_cast<ssize_t>(captured.size()),
        "PTY write");
  // PTY delivery is scheduled; bounded retries allow a local kernel to deliver bytes.
  b::ProtocolDecoder decoder;
  std::size_t received = 0;
  for (int attempt = 0; attempt < 100 && received == 0; ++attempt) {
    const auto count = port.read_some(buffer.data(), buffer.size());
    received += decoder.feed(buffer.data(), count).size();
    if (received == 0) ::usleep(1000);
  }
  check(received == 1, "PTY to serial to decoder");
  port.close();
  check(!port.is_open(), "descriptor close");
  try { port.open("/definitely/nonexistent/learm", 123); throw std::runtime_error("bad baud accepted"); }
  catch (const std::invalid_argument&) {}
  try { port.open("/definitely/nonexistent/learm"); throw std::runtime_error("bad path accepted"); }
  catch (const std::system_error&) { check(!port.is_open(), "failed open leaked fd"); }
}
int main() {
  try {
    protocol_tests(); odometry_tests(); serial_tests();
    std::cout << "PASS: protocol, odometry, Linux PTY serial\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
