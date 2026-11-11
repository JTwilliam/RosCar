#pragma once

#include "learm_bridge_cpp/protocol.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace learm_bridge_cpp {
inline constexpr std::uint16_t kAgeUnknown = 65535;
inline constexpr std::uint16_t kAgeSaturated = 65534;
// Wire order: DEFAULT, CHASSIS, SERIAL, ARM, HEALTH.
struct HealthState {
  std::uint32_t timestamp_ms{}, minimum_heap_bytes{};
  std::array<std::uint16_t, 5> stack_free_words{};
  std::array<std::uint16_t, 11> counters{};
  std::uint16_t latched_fault_flags{};
  std::uint8_t safety_state{};
  bool extended{false};
  std::uint8_t schema{}, heartbeat_valid_mask{};
  std::uint16_t active_faults{}, historical_faults{}, last_fault_bits{};
  std::uint32_t last_fault_timestamp_ms{};
  std::array<std::uint16_t, 5> heartbeat_age_ms{};
  std::uint16_t chassis_age_ms{kAgeUnknown}, i2c_consecutive_failures{}, command_age_ms{kAgeUnknown};
  std::uint8_t command_validity{}, arm_status{}, arm_query{}, owner{};
  std::uint16_t last_event{};
  std::uint32_t last_event_timestamp_ms{}, uptime_ms{}, tx_drops{};
  std::string firmware_id;
};
HealthState decode_health_state(const Frame& frame);

struct HealthThresholds {
  double health_stale_after_s{0.5}, offline_after_s{2.0};
  std::array<std::uint16_t, 5> task_deadlines_ms{100, 50, 100, 500, 250};
  std::uint32_t minimum_heap_bytes{1024};
  std::array<std::uint16_t, 5> stack_warning_words{32, 64, 80, 80, 40};
  std::uint16_t chassis_max_age_ms{100};
  void validate() const;
};

// Numeric levels intentionally match diagnostic_msgs: OK/WARN/ERROR/STALE.
enum class HealthLevel : std::uint8_t { ok = 0, warn = 1, error = 2, stale = 3 };
struct HealthDiagnostic {
  std::string name, message;
  HealthLevel level{HealthLevel::warn};
  std::vector<std::pair<std::string, std::string>> values;
};
struct HealthSnapshot {
  std::optional<HealthState> state;
  bool updated_this_connection{false};
  double health_age_s{}, receive_age_s{};
  std::uint32_t boot_count{};
};

// Host times are steady-clock seconds supplied by the caller, never ROS time.
// Only a new HEALTH_STATE snapshot advances freshness, not chassis or duplicates.
class HealthTracker {
 public:
  void connected();
  void update(const HealthState& state, double received_at_s);
  HealthSnapshot snapshot(double now_s) const;
 private:
  std::optional<HealthState> state_;
  std::optional<double> received_at_s_, progressed_at_s_;
  bool updated_this_connection_{false};
  std::uint32_t boot_count_{};
};
std::vector<HealthDiagnostic> assess_health(const HealthSnapshot& snapshot,
                                          const HealthThresholds& thresholds, bool serial_open);
}  // namespace learm_bridge_cpp
