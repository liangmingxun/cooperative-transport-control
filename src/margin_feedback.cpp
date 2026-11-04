#include "paper_control/margin_feedback.hpp"
#include <algorithm>
#include <cmath>

namespace paper_control {
// Causal feedback margins; previous-cycle inputs are supplied by the caller.
double normalizedMargin(double value, double scale) {
  if (!std::isfinite(value) || !std::isfinite(scale) || scale <= 0.0) return 0.0;
  return std::clamp(value / scale, 0.0, 1.0);
}

std::array<double, 5> causalMargins(const MarginScales& s, const MarginFeedback& f) {
  const double capability = normalizedMargin(
      f.mapped_upper_capability - s.capability_reserve - std::abs(f.candidate_velocity),
      s.capability_safe_margin);
  const double input = normalizedMargin(
      std::min(f.available_acceleration, f.available_deceleration) - std::abs(f.previous_input_raw),
      s.input_safe_margin);
  const double wheel = normalizedMargin(
      std::min(f.wheel_velocity_limits[0] - std::abs(f.previous_wheel_raw[0]),
               f.wheel_velocity_limits[1] - std::abs(f.previous_wheel_raw[1])),
      s.wheel_safe_margin);
  const double position = normalizedMargin(s.position_limit - f.previous_position_error,
                                           s.position_safe_margin);
  return {{capability, input, wheel, position, f.failure_margin}};
}
}  // namespace paper_control

