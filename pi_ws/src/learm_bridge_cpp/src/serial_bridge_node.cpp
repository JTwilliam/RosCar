#include "learm_bridge_cpp/health.hpp"
#include "learm_bridge_cpp/odometry.hpp"
#include "learm_bridge_cpp/protocol.hpp"
#include "learm_bridge_cpp/serial_port.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace learm_bridge_cpp {
namespace {
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

// Preserve all counters when a new connection discards a partially received frame.
DecoderStats sum_stats(const DecoderStats& a, const DecoderStats& b) {
  return DecoderStats{
      a.bytes_received + b.bytes_received,
      a.frames_valid + b.frames_valid,
      a.frames_dropped + b.frames_dropped,
      a.cobs_errors + b.cobs_errors,
      a.crc_errors + b.crc_errors,
      a.format_errors + b.format_errors,
      a.overflow_errors + b.overflow_errors,
      a.sequence_drops + b.sequence_drops,
      a.duplicate_frames + b.duplicate_frames};
}

std::string fixed(double value, int precision) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string fault_hex(std::uint16_t flags) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << flags;
  return stream.str();
}

// The same two-second receive-time window as Python ChassisTelemetryTracker.
// A stale stream is reported by frame age; its historical rate remains available.
class TelemetryTracker {
 public:
  void update(TimePoint received_at) {
    last_received_at_ = received_at;
    receive_times_.push_back(received_at);
    const auto cutoff = received_at - std::chrono::seconds(2);
    while (receive_times_.size() > 1 && receive_times_.front() < cutoff) {
      receive_times_.pop_front();
    }
  }

  double rate_hz() const {
    if (receive_times_.size() < 2) return 0.0;
    const double duration = std::chrono::duration<double>(
        receive_times_.back() - receive_times_.front()).count();
    return duration > 0.0 ? static_cast<double>(receive_times_.size() - 1) / duration : 0.0;
  }

  double frame_age_s(TimePoint now) const {
    if (!last_received_at_) return std::numeric_limits<double>::infinity();
    return std::max(0.0, std::chrono::duration<double>(now - *last_received_at_).count());
  }

 private:
  std::deque<TimePoint> receive_times_;
  std::optional<TimePoint> last_received_at_;
};
}  // namespace

class SerialBridgeNode : public rclcpp::Node {
 public:
  SerialBridgeNode() : Node("learm_serial_bridge") {
    port_ = declare_parameter<std::string>(
        "port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    baud_ = declare_parameter<int>("baud", 115200);
    const auto chunk_size = declare_parameter<std::int64_t>("read_chunk_size", 512);
    const double diagnostics_period = declare_parameter<double>("diagnostics_period_s", 0.2);
    const double reconnect_period = declare_parameter<double>("reconnect_period_s", 2.0);
    const double radius = declare_parameter<double>("wheel_radius_m", 0.048);
    const double base = declare_parameter<double>("wheel_base_m", 0.190);
    const double track = declare_parameter<double>("track_width_m", 0.205);
    const double counts = declare_parameter<double>(
        "encoder_counts_per_revolution", 5740.25304043815);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto variances = declare_parameter<std::vector<double>>(
        "twist_covariance_diagonal", {0.0025, 0.01, 1e6, 1e6, 1e6, 0.04});
    if (variances.size() != 6 || std::any_of(variances.begin(), variances.end(),
        [](double value) { return !std::isfinite(value) || value <= 0.0; })) {
      throw std::invalid_argument("twist_covariance_diagonal needs six finite positive variances");
    }
    if (!std::isfinite(diagnostics_period) || !std::isfinite(reconnect_period)) {
      throw std::invalid_argument("diagnostics_period_s and reconnect_period_s must be finite");
    }
    if (baud_ <= 0) throw std::invalid_argument("baud must be positive");
    read_buffer_.resize(static_cast<std::size_t>(std::max<std::int64_t>(1, chunk_size)));
    reconnect_period_ = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(std::max(0.2, reconnect_period)));
    for (std::size_t index = 0; index < variances.size(); ++index) {
      twist_covariance_[index * 7] = variances[index];
    }
    health_thresholds_.health_stale_after_s = declare_parameter<double>("health_stale_after_s", 0.5);
    health_thresholds_.offline_after_s = declare_parameter<double>("health_offline_after_s", 2.0);
    const auto deadlines = declare_parameter<std::vector<std::int64_t>>(
        "task_deadlines_ms", {100, 50, 100, 500, 250});
    const auto stacks = declare_parameter<std::vector<std::int64_t>>(
        "stack_warning_words", {32, 64, 80, 80, 40});
    const auto heap = declare_parameter<std::int64_t>("minimum_heap_bytes", 1024);
    const auto chassis_age = declare_parameter<std::int64_t>("chassis_max_age_ms", 100);
    if (deadlines.size() != 5 || stacks.size() != 5 || heap <= 0 ||
        heap > std::numeric_limits<std::uint32_t>::max() || chassis_age <= 0 || chassis_age >= kAgeSaturated) {
      throw std::invalid_argument("health arrays need five entries and thresholds must fit wire units");
    }
    for (std::size_t i = 0; i < 5; ++i) {
      if (deadlines[i] <= 0 || deadlines[i] >= kAgeSaturated || stacks[i] <= 0 || stacks[i] > 65535)
        throw std::invalid_argument("health deadline/stack thresholds outside valid range");
      health_thresholds_.task_deadlines_ms[i] = static_cast<std::uint16_t>(deadlines[i]);
      health_thresholds_.stack_warning_words[i] = static_cast<std::uint16_t>(stacks[i]);
    }
    health_thresholds_.minimum_heap_bytes = static_cast<std::uint32_t>(heap);
    health_thresholds_.chassis_max_age_ms = static_cast<std::uint16_t>(chassis_age);
    health_thresholds_.validate();
    odometry_ = std::make_unique<MecanumOdometry>(radius, base, track, counts);
    encoder_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
        "/learm/wheel_encoders", 10);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(5),
                                   [this]() { poll_serial(); });
    diagnostic_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.1, diagnostics_period))),
        [this]() { publish_diagnostics(); });
    RCLCPP_INFO(get_logger(), "LeArm serial bridge configured for %s at %d 8N1",
                port_.c_str(), baud_);
  }

 private:
  bool ensure_serial_open(TimePoint now) {
    if (serial_.is_open()) return true;
    if (now < next_reconnect_at_) return false;
    try {
      serial_.open(port_, static_cast<unsigned int>(baud_));
      // A fresh physical connection must not inherit a stale half-frame or sequence
      // baseline. Counters remain cumulative across every connection.
      cumulative_stats_ = sum_stats(cumulative_stats_, decoder_.stats());
      decoder_ = ProtocolDecoder{};
      health_tracker_.connected();
      last_serial_error_.clear();
      RCLCPP_INFO(get_logger(), "Opened serial port %s", port_.c_str());
      return true;
    } catch (const std::exception& error) {
      const std::string message = error.what();
      if (message != last_serial_error_) {
        RCLCPP_ERROR(get_logger(), "Cannot open %s: %s", port_.c_str(), message.c_str());
        last_serial_error_ = message;
      }
      serial_.close();
      next_reconnect_at_ = now + reconnect_period_;
      return false;
    }
  }

  void close_serial_after_error(const std::exception& error) {
    const std::string message = error.what();
    if (message != last_serial_error_) {
      RCLCPP_ERROR(get_logger(), "Serial read failed: %s", message.c_str());
      last_serial_error_ = message;
    }
    serial_.close();
    next_reconnect_at_ = SteadyClock::now() + reconnect_period_;
  }

  void poll_serial() {
    if (!ensure_serial_open(SteadyClock::now())) return;
    std::size_t count = 0;
    try {
      count = serial_.read_some(read_buffer_.data(), read_buffer_.size());
    } catch (const std::exception& error) {
      close_serial_after_error(error);
      return;
    }
    if (count == 0) return;
    for (const auto& frame : decoder_.feed(read_buffer_.data(), count)) {
      if (frame.message_type == kMsgHealthState) {
        try {
          health_tracker_.update(decode_health_state(frame),
              std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count());
        } catch (const std::invalid_argument&) {
          ++payload_format_errors_;
        }
        continue;
      }
      if (frame.message_type != kMsgChassisState) continue;
      ChassisState state;
      try {
        state = decode_chassis_state(frame);
      } catch (const std::invalid_argument&) {
        // Python adds these to format_errors and frames_dropped while retaining
        // the decoder's frames_valid count for the valid protocol envelope.
        ++payload_format_errors_;
        continue;
      }
      if (last_firmware_timestamp_ms_ && state.timestamp_ms < *last_firmware_timestamp_ms_) {
        // Match the existing Python reset: clear pose and establish a new baseline.
        odometry_->reset(state.encoders, state.timestamp_ms * 0.001);
      }
      last_firmware_timestamp_ms_ = state.timestamp_ms;
      last_state_ = state;
      tracker_.update(SteadyClock::now());
      publish_odometry(state);
      std_msgs::msg::Int32MultiArray encoders;
      encoders.data.assign(state.encoders.begin(), state.encoders.end());
      encoder_publisher_->publish(encoders);
    }
  }

  void publish_odometry(const ChassisState& state) {
    // Firmware sampling time supplies dt, independently of batched USB arrivals.
    const auto estimate = odometry_->update(state.encoders, state.timestamp_ms * 0.001);
    const auto stamp = get_clock()->now();
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = estimate.x_m;
    message.pose.pose.position.y = estimate.y_m;
    message.pose.pose.orientation.z = std::sin(estimate.yaw_rad * 0.5);
    message.pose.pose.orientation.w = std::cos(estimate.yaw_rad * 0.5);
    message.twist.twist.linear.x = estimate.vx_m_s;
    message.twist.twist.linear.y = estimate.vy_m_s;
    message.twist.twist.angular.z = estimate.wz_rad_s;
    message.twist.covariance = twist_covariance_;
    odom_publisher_->publish(message);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = message.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = estimate.x_m;
      transform.transform.translation.y = estimate.y_m;
      transform.transform.rotation = message.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_diagnostics() {
    const double age = tracker_.frame_age_s(SteadyClock::now());
    auto stats = sum_stats(cumulative_stats_, decoder_.stats());
    stats.format_errors += payload_format_errors_;
    stats.frames_dropped += payload_format_errors_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "learm_serial_bridge";
    status.hardware_id = "stm32f103rbt6";
    const auto fault_flags = last_state_ ? last_state_->fault_flags : 0;
    const auto safety_state = last_state_ ? last_state_->safety_state : 0;
    const auto health_snapshot = health_tracker_.snapshot(
        std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count());
    const bool extended_health = health_snapshot.updated_this_connection && health_snapshot.state &&
        health_snapshot.state->extended && health_snapshot.health_age_s <= health_thresholds_.health_stale_after_s;
    if (!serial_.is_open()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = last_serial_error_.empty() ? "serial port is closed" : last_serial_error_;
    } else if (!std::isfinite(age) || age > 0.5) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "no recent CHASSIS_STATE frame";
    } else if (extended_health ? ((health_snapshot.state->active_faults & ~std::uint16_t(8)) != 0 ||
                                  health_snapshot.state->safety_state == 7)
                               : (fault_flags != 0 || safety_state == 7)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "STM32 reports a controller fault";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "receiving chassis telemetry";
    }
    const auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
    add("firmware_timestamp_ms", std::to_string(last_state_ ? last_state_->timestamp_ms : 0));
    add("sample_sequence", std::to_string(last_state_ ? last_state_->sequence : 0));
    add("receive_rate_hz", fixed(tracker_.rate_hz(), 3));
    add("frame_age_s", fixed(age, 6));
    add("fault_flags", fault_hex(static_cast<std::uint16_t>(fault_flags)));
    add("safety_state", std::to_string(safety_state));
    add("frames_valid", std::to_string(stats.frames_valid));
    add("frames_dropped", std::to_string(stats.frames_dropped));
    add("crc_errors", std::to_string(stats.crc_errors));
    add("sequence_drops", std::to_string(stats.sequence_drops));
    add("serial_port", port_);
    add("baud_rate", std::to_string(baud_));
    add("bytes_received", std::to_string(stats.bytes_received));
    add("cobs_errors", std::to_string(stats.cobs_errors));
    add("format_errors", std::to_string(stats.format_errors));
    add("overflow_errors", std::to_string(stats.overflow_errors));
    add("duplicate_frames", std::to_string(stats.duplicate_frames));
    if (last_state_) {
      add("battery_mv", std::to_string(last_state_->battery_mv));
      std::string commands;
      for (const auto command : last_state_->motor_commands) {
        if (!commands.empty()) commands += ',';
        commands += std::to_string(static_cast<int>(command));
      }
      add("motor_commands", commands);
    }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = get_clock()->now();
    message.status.push_back(status);
    for (const auto& item : assess_health(health_snapshot, health_thresholds_, serial_.is_open())) {
      diagnostic_msgs::msg::DiagnosticStatus health;
      health.name = item.name;
      health.hardware_id = "stm32f103rbt6";
      health.level = static_cast<std::uint8_t>(item.level);
      health.message = item.message;
      for (const auto& entry : item.values) {
        diagnostic_msgs::msg::KeyValue value; value.key = entry.first; value.value = entry.second;
        health.values.push_back(value);
      }
      if (item.name == "learm/health/communication") {
        const auto host_counter = [&health](const char* key, std::uint64_t count) {
          diagnostic_msgs::msg::KeyValue value;
          value.key = key; value.value = std::to_string(count); health.values.push_back(value);
        };
        host_counter("host_crc_errors", stats.crc_errors);
        host_counter("host_cobs_errors", stats.cobs_errors);
        host_counter("host_frames_dropped", stats.frames_dropped);
        for (auto& value : health.values) {
          if (value.key == "detail") value.value += "; host RX CRC=" + std::to_string(stats.crc_errors) +
              ", dropped=" + std::to_string(stats.frames_dropped);
        }
      }
      message.status.push_back(health);
    }
    diagnostic_publisher_->publish(message);
  }

  std::string port_, odom_frame_, base_frame_, last_serial_error_;
  int baud_{};
  bool publish_tf_{};
  std::vector<std::uint8_t> read_buffer_;
  std::array<double, 36> twist_covariance_{};
  SteadyClock::duration reconnect_period_{};
  TimePoint next_reconnect_at_{};
  SerialPort serial_;
  ProtocolDecoder decoder_;
  DecoderStats cumulative_stats_{};
  std::uint64_t payload_format_errors_{};
  TelemetryTracker tracker_;
  HealthTracker health_tracker_;
  HealthThresholds health_thresholds_;
  std::optional<ChassisState> last_state_;
  std::optional<std::uint32_t> last_firmware_timestamp_ms_;
  std::unique_ptr<MecanumOdometry> odometry_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr poll_timer_, diagnostic_timer_;
};
}  // namespace learm_bridge_cpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<learm_bridge_cpp::SerialBridgeNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("learm_serial_bridge"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
