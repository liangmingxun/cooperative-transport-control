#pragma once
#include <array>

namespace paper_control {
struct MarginScales {
  double capability_reserve{0.005};
  double capability_safe_margin{0.010};
  double input_safe_margin{0.40};
  double wheel_safe_margin{0.010};
  double position_limit{0.12};
  double position_safe_margin{0.06};
};

struct MarginFeedback {
  double candidate_velocity{0.10};
  double mapped_upper_capability{0.16};
  double available_acceleration{1.0};
  double available_deceleration{2.0};
  std::array<double, 2> wheel_velocity_limits{{0.16, 0.16}};
  double previous_input_raw{0.0};
  std::array<double, 2> previous_wheel_raw{{0.0, 0.0}};
  double previous_position_error{0.0};
  double failure_margin{1.0};
};

// previous_* inputs must be taken from the most recently completed lower
// cycle. Current capability is supplied separately by the caller.
double normalizedMargin(double value, double scale);
std::array<double, 5> causalMargins(const MarginScales&, const MarginFeedback&);
}  // namespace paper_control

