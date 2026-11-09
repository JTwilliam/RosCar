#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace learm_bridge_cpp {

inline constexpr std::uint8_t kProtocolVersion = 1;
inline constexpr std::size_t kHeaderLength = 12;
inline constexpr std::size_t kCrcLength = 4;
inline constexpr std::size_t kMaxPayloadLength = 128;
inline constexpr std::size_t kMaxDecodedLength = 256;
inline constexpr std::size_t kMaxEncodedLength = 258;

inline constexpr std::uint8_t kMsgHelloState = 0x80;
inline constexpr std::uint8_t kMsgChassisState = 0x81;
inline constexpr std::uint8_t kMsgArmState = 0x82;
inline constexpr std::uint8_t kMsgHealthState = 0x83;

struct Frame {
  std::uint8_t version{};
  std::uint8_t message_type{};
  std::uint8_t flags{};
  std::uint16_t sequence{};
  std::uint32_t timestamp_ms{};
  std::vector<std::uint8_t> payload;
};

struct ChassisState {
  std::uint32_t timestamp_ms{};
  // Raw wheel order: front right, rear right, front left, rear left.
  std::array<std::int32_t, 4> encoders{};
  std::array<std::int8_t, 4> motor_commands{};
  std::uint16_t battery_mv{};
  std::uint16_t fault_flags{};
  std::uint8_t safety_state{};
  std::uint16_t sequence{};
};

struct DecoderStats {
  std::uint64_t bytes_received{};
  std::uint64_t frames_valid{};
  std::uint64_t frames_dropped{};
  std::uint64_t cobs_errors{};
  std::uint64_t crc_errors{};
  std::uint64_t format_errors{};
  std::uint64_t overflow_errors{};
  std::uint64_t sequence_drops{};
  std::uint64_t duplicate_frames{};
};

// A null pointer is permitted only when size is zero.
std::uint32_t crc32c(const std::uint8_t* data, std::size_t size);
ChassisState decode_chassis_state(const Frame& frame);

class ProtocolDecoder {
 public:
  // Incremental COBS input, including trailing zero delimiters.
  std::vector<Frame> feed(const std::uint8_t* data, std::size_t size);
  std::vector<Frame> feed(const std::vector<std::uint8_t>& data) {
    return feed(data.data(), data.size());
  }
  const DecoderStats& stats() const noexcept { return stats_; }

 private:
  void track_sequence(const Frame& frame);

  DecoderStats stats_{};
  std::vector<std::uint8_t> candidate_;
  bool discard_until_delimiter_{false};
  bool has_last_sequence_{false};
  std::uint16_t last_sequence_{};
  std::uint32_t last_timestamp_ms_{};
};

}  // namespace learm_bridge_cpp
