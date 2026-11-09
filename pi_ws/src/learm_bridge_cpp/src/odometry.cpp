#include "learm_bridge_cpp/odometry.hpp"

#include <cmath>
#include <stdexcept>

namespace learm_bridge_cpp {
namespace { constexpr double kPi = 3.14159265358979323846; }
MecanumOdometry::MecanumOdometry(double radius, double base, double track, double counts)
    : meters_per_tick_(0), wheel_base_m_(base), track_width_m_(track) {
  for (double value : {radius, base, track, counts}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("geometry and counts must be finite and positive");
    }
  }
  meters_per_tick_ = 2 * kPi * radius / counts;
}
OdometryEstimate MecanumOdometry::reset() {
  estimate_ = {};
  last_counts_.reset();
  last_time_s_.reset();
  return estimate_;
}
OdometryEstimate MecanumOdometry::reset(const std::array<std::int32_t, 4>& counts,
                                       double timestamp_s) {
  reset();
  last_counts_ = counts;
  last_time_s_ = timestamp_s;
  return estimate_;
}
OdometryEstimate MecanumOdometry::update(const std::array<std::int32_t, 4>& counts,
                                        double timestamp_s) {
  if (!std::isfinite(timestamp_s)) throw std::invalid_argument("timestamp must be finite");
  if (!last_counts_ || !last_time_s_) {
    last_counts_ = counts;
    last_time_s_ = timestamp_s;
    return estimate_;
  }
  const auto previous = *last_counts_;
  const double dt = timestamp_s - *last_time_s_;
  last_counts_ = counts;
  last_time_s_ = timestamp_s;
  if (dt <= 0) return estimate_;
  std::array<std::int64_t, 4> d{};
  for (std::size_t i = 0; i < 4; ++i) {
    d[i] = static_cast<std::int64_t>(counts[i]) - previous[i];
  }
  // Preserve the Armrobot FR, RR, FL, RL signed mapping.
  const double forward = (d[0] - d[1] - d[2] + d[3]) * 0.25 * meters_per_tick_;
  const double left = (d[0] + d[1] + d[2] + d[3]) * 0.25 * meters_per_tick_;
  const double yaw = (d[0] + d[1] - d[2] - d[3]) * meters_per_tick_ /
                     (2 * (wheel_base_m_ + track_width_m_));
  const double old_yaw = estimate_.yaw_rad;
  estimate_.x_m += forward * std::cos(old_yaw) - left * std::sin(old_yaw);
  estimate_.y_m += forward * std::sin(old_yaw) + left * std::cos(old_yaw);
  double wrapped = std::fmod(old_yaw + yaw + kPi, 2 * kPi);
  if (wrapped < 0) wrapped += 2 * kPi;
  estimate_.yaw_rad = wrapped - kPi;
  estimate_.vx_m_s = forward / dt;
  estimate_.vy_m_s = left / dt;
  estimate_.wz_rad_s = yaw / dt;
  return estimate_;
}
}  // namespace learm_bridge_cpp
