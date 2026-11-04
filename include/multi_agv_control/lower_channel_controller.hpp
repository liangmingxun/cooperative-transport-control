#pragma once

#include <array>
#include <limits>
#include <cstddef>
#include <string>

namespace multi_agv_control {

enum class LowerMode {
  kR1,
  kPaperNM = 2,
  kPDPenalty,
};

struct LowerChannelConfig {
  LowerMode mode{LowerMode::kR1};
  // Default gains for the proposed lower-layer controller.
  double composite_gain{1.2};
  double feedback_gain{2.4};
  std::array<double, 2> adaptation_gain{{0.12, 0.12}};
  double disturbance_adaptation_gain{1.0};
  double parameter_leakage{0.025};
  double disturbance_leakage{0.08};
  double robust_boundary_layer{0.055};
  std::array<double, 2> parameter_min{{-0.8, -0.8}};
  std::array<double, 2> parameter_max{{0.8, 0.8}};
  double disturbance_estimate_max{0.55};
  double constraint_margin{2.0e-5};
  double sustained_saturation_seconds{0.5};
  // External lower-layer control laws. PaperNM forwards its native input
  // and integrated velocity to the common downstream execution interface.
  double paper_nm_k1{1.25};
  double paper_nm_k2{2.75};
  double pd_penalty_kp1{4.0};
  double pd_penalty_kp2{2.0};
  double pd_penalty_kpd{0.24};
  double pd_penalty_denominator_guard{2.0e-5};
};

struct LowerChannelInput {
  double position_actual{0.0};
  double velocity_actual{0.0};
  double position_reference{0.0};
  double velocity_reference{0.0};
  double acceleration_reference{0.0};
  double velocity_lower_bound{-0.5};
  double velocity_upper_bound{0.5};
  double available_acceleration{1.0};
  double available_deceleration{1.0};
  std::array<double, 2> regressor{{0.0, 0.0}};
  double injected_matched_disturbance{0.0};
  bool physical_wheel_saturation{false};
  double dt_seconds{0.01};
  // Platform command bounds, separate from the nonlinear-mapping bounds.
  // Leave both NaN to use velocity_lower_bound / velocity_upper_bound.
  double execution_velocity_lower_bound{std::numeric_limits<double>::quiet_NaN()};
  double execution_velocity_upper_bound{std::numeric_limits<double>::quiet_NaN()};
};

struct LowerChannelOutput {
  double position_error{0.0};
  double velocity_error{0.0};
  // Paper notation: psi, Gamma and Gamma^{-1}Upsilon.
  double transformed_error{0.0};
  double transformation_gain{0.0};
  double inverse_gain_term{0.0};
  double composite_error{0.0};
  double input_raw{0.0};
  double input_plant{0.0};
  // Control input after platform acceleration/deceleration limiting.
  double input_limited{0.0};
  double channel_velocity_command{0.0};
  std::array<double, 2> parameter_estimate{{0.0, 0.0}};
  double disturbance_estimate{0.0};
  bool adaptation_enabled{false};
  bool disturbance_compensation_enabled{false};
  bool input_limit_active{false};
  bool velocity_projection_active{false};
  bool sustained_physical_saturation{false};
  bool valid{false};
};

class LowerChannelController {
 public:
  explicit LowerChannelController(const LowerChannelConfig& config);

  LowerChannelOutput step(const LowerChannelInput& input);
  void reset(
      const std::array<double, 2>& initial_parameter = {{0.0, 0.0}},
      double initial_disturbance = 0.0);
  const LowerChannelOutput& output() const noexcept { return output_; }

 private:
  LowerChannelConfig config_;
  std::array<double, 2> parameter_estimate_{{0.0, 0.0}};
  double disturbance_estimate_{0.0};
  double physical_saturation_duration_{0.0};
  LowerChannelOutput output_;
};

LowerMode lowerModeFromString(const std::string& value);
const char* lowerModeName(LowerMode mode);
bool lowerAdaptationEnabled(LowerMode mode);
bool lowerDisturbanceCompensationEnabled(LowerMode mode);

}  // namespace multi_agv_control
