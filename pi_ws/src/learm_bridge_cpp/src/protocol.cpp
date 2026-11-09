#include "learm_bridge_cpp/protocol.hpp"

#include <stdexcept>
#include <utility>

namespace learm_bridge_cpp {
namespace {
enum class DecodeError { cobs, crc, format };

std::uint16_t u16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}
std::uint32_t u32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) |
         (static_cast<std::uint32_t>(p[3]) << 24);
}
// Convert the wire's two's-complement value without implementation-defined casts.
std::int32_t i32(const std::uint8_t* p) {
  const auto value = u32(p);
  const std::int64_t signed_value = value <= 0x7fffffffU
      ? static_cast<std::int64_t>(value)
      : static_cast<std::int64_t>(value) - 0x100000000LL;
  return static_cast<std::int32_t>(signed_value);
}

Frame decode(const std::vector<std::uint8_t>& encoded) {
  std::vector<std::uint8_t> raw;
  raw.reserve(kMaxDecodedLength);
  std::size_t index = 0;
  while (index < encoded.size()) {
    const auto code = encoded[index++];
    if (code == 0 || index + code - 1 > encoded.size()) throw DecodeError::cobs;
    const auto end = index + code - 1;
    raw.insert(raw.end(), encoded.begin() + index, encoded.begin() + end);
    index = end;
    if (code != 0xff && index < encoded.size()) raw.push_back(0);
    if (raw.size() > kMaxDecodedLength) throw DecodeError::cobs;
  }
  if (raw.size() < kHeaderLength + kCrcLength) throw DecodeError::format;
  const auto payload_length = u16(raw.data() + 6);
  if (raw[0] != kProtocolVersion || raw[3] != kHeaderLength ||
      payload_length > kMaxPayloadLength ||
      raw.size() != kHeaderLength + payload_length + kCrcLength) {
    throw DecodeError::format;
  }
  if (u32(raw.data() + kHeaderLength + payload_length) !=
      crc32c(raw.data(), kHeaderLength + payload_length)) throw DecodeError::crc;
  return Frame{raw[0], raw[1], raw[2], u16(raw.data() + 4), u32(raw.data() + 8),
               {raw.begin() + kHeaderLength, raw.end() - kCrcLength}};
}
}  // namespace

std::uint32_t crc32c(const std::uint8_t* data, std::size_t size) {
  if (!data && size != 0) throw std::invalid_argument("null CRC input");
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1U) ? 0x82f63b78U : 0U);
    }
  }
  return crc ^ 0xffffffffU;
}

ChassisState decode_chassis_state(const Frame& frame) {
  if (frame.message_type != kMsgChassisState || frame.payload.size() != 32) {
    throw std::invalid_argument("expected CHASSIS_STATE with 32-byte payload");
  }
  const auto* p = frame.payload.data();
  ChassisState state;
  state.timestamp_ms = u32(p);
  for (std::size_t i = 0; i < 4; ++i) {
    state.encoders[i] = i32(p + 4 + i * 4);
    const int value = p[20 + i];
    state.motor_commands[i] = static_cast<std::int8_t>(value < 128 ? value : value - 256);
  }
  state.battery_mv = u16(p + 24);
  state.fault_flags = u16(p + 26);
  state.safety_state = p[28];
  state.sequence = u16(p + 30);
  return state;
}

std::vector<Frame> ProtocolDecoder::feed(const std::uint8_t* data, std::size_t size) {
  if (!data && size != 0) throw std::invalid_argument("null decoder input");
  std::vector<Frame> frames;
  for (std::size_t i = 0; i < size; ++i) {
    const auto byte = data[i];
    ++stats_.bytes_received;
    if (discard_until_delimiter_) {
      if (byte == 0) {
        discard_until_delimiter_ = false;
        ++stats_.frames_dropped;
      }
      continue;
    }
    if (byte != 0) {
      if (candidate_.size() >= kMaxEncodedLength) {
        candidate_.clear();
        discard_until_delimiter_ = true;
        ++stats_.overflow_errors;
      } else {
        candidate_.push_back(byte);
      }
      continue;
    }
    if (candidate_.empty()) continue;
    try {
      auto frame = decode(candidate_);
      track_sequence(frame);
      ++stats_.frames_valid;
      frames.push_back(std::move(frame));
    } catch (DecodeError error) {
      ++stats_.frames_dropped;
      switch (error) {
        case DecodeError::cobs: ++stats_.cobs_errors; break;
        case DecodeError::crc: ++stats_.crc_errors; break;
        case DecodeError::format: ++stats_.format_errors; break;
      }
    }
    candidate_.clear();
  }
  return frames;
}

void ProtocolDecoder::track_sequence(const Frame& frame) {
  if (has_last_sequence_ && frame.timestamp_ms >= last_timestamp_ms_) {
    const auto difference = static_cast<std::uint16_t>(frame.sequence - last_sequence_);
    if (difference == 0) ++stats_.duplicate_frames;
    else if (difference > 1) stats_.sequence_drops += difference - 1;
  }
  has_last_sequence_ = true;
  last_sequence_ = frame.sequence;
  last_timestamp_ms_ = frame.timestamp_ms;
}
}  // namespace learm_bridge_cpp
