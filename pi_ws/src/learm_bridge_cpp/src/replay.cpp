#include "learm_bridge_cpp/odometry.hpp"
#include "learm_bridge_cpp/protocol.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: learm_replay captured_uart.bin\n";
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) { std::cerr << "Cannot open input file\n"; return 2; }
  namespace bridge = learm_bridge_cpp;
  bridge::ProtocolDecoder decoder;
  bridge::MecanumOdometry odometry;
  std::optional<std::uint32_t> last_timestamp;
  std::array<char, 512> buffer{};
  std::size_t chassis_frames = 0;
  std::cout << "timestamp_ms,FR,RR,FL,RL,x_m,y_m,yaw_rad,vx_m_s,vy_m_s,wz_rad_s\n"
            << std::setprecision(17);
  while (input.read(buffer.data(), buffer.size()) || input.gcount() != 0) {
    const auto frames = decoder.feed(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                                     static_cast<std::size_t>(input.gcount()));
    for (const auto& frame : frames) {
      if (frame.message_type != bridge::kMsgChassisState) continue;
      try {
        const auto state = bridge::decode_chassis_state(frame);
        if (last_timestamp && state.timestamp_ms < *last_timestamp) {
          odometry.reset(state.encoders, state.timestamp_ms * 0.001);
        }
        last_timestamp = state.timestamp_ms;
        const auto value = odometry.update(state.encoders, state.timestamp_ms * 0.001);
        std::cout << state.timestamp_ms;
        for (auto count : state.encoders) std::cout << ',' << count;
        std::cout << ',' << value.x_m << ',' << value.y_m << ',' << value.yaw_rad
                  << ',' << value.vx_m_s << ',' << value.vy_m_s << ',' << value.wz_rad_s << '\n';
        ++chassis_frames;
      } catch (const std::invalid_argument& error) {
        std::cerr << "Rejected chassis payload: " << error.what() << '\n';
      }
    }
  }
  if (input.bad()) { std::cerr << "Input read failed\n"; return 2; }
  const auto& stats = decoder.stats();
  std::cerr << "valid=" << stats.frames_valid << " dropped=" << stats.frames_dropped
            << " crc=" << stats.crc_errors << " cobs=" << stats.cobs_errors
            << " overflow=" << stats.overflow_errors << " format=" << stats.format_errors
            << " sequence_drops=" << stats.sequence_drops
            << " duplicates=" << stats.duplicate_frames
            << " chassis=" << chassis_frames << '\n';
  return chassis_frames == 0 ? 1 : 0;
}
