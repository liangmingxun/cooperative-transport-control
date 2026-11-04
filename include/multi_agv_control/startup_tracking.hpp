#pragma once
#include <algorithm>

namespace multi_agv_control {
inline double startupSmoothStep(double tau) {
  tau = std::clamp(tau, 0.0, 1.0);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}
}  // namespace multi_agv_control
