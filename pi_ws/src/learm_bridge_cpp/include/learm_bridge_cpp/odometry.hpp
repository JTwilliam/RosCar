#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace learm_bridge_cpp {
struct OdometryEstimate {
  double x_m{}, y_m{}, yaw_rad{}, vx_m_s{}, vy_m_s{}, wz_rad_s{};
};

class MecanumOdometry {
 public:
  explicit MecanumOdometry(double wheel_radius_m = 0.048,
                           double wheel_base_m = 0.190,
                           double track_width_m = 0.205,
                           double encoder_counts_per_revolution = 5740.25304043815);
  OdometryEstimate update(const std::array<std::int32_t, 4>& encoders,
                          double timestamp_s);
  OdometryEstimate reset();
  OdometryEstimate reset(const std::array<std::int32_t, 4>& encoders,
                         double timestamp_s);
  double meters_per_tick() const noexcept { return meters_per_tick_; }
 private:
  double meters_per_tick_, wheel_base_m_, track_width_m_;
  std::optional<std::array<std::int32_t, 4>> last_counts_;
  std::optional<double> last_time_s_;
  OdometryEstimate estimate_;
};
}  // namespace learm_bridge_cpp
