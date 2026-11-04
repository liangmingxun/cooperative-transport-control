#include "paper_control/nominal_speed_baseline.hpp"
#include "multi_agv_control/startup_tracking.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace paper_control {
NominalFleetLimitResult limitNominalFleetCommands(
    std::array<multi_agv_control::PlanarTrackingResult, 3>* tracking,
    double maximum_wheel_command, double minimum_command_scale) {
  NominalFleetLimitResult out;
  if (!tracking || !std::isfinite(maximum_wheel_command) || maximum_wheel_command <= 0.0 ||
      !std::isfinite(minimum_command_scale) || minimum_command_scale <= 0.0 || minimum_command_scale > 1.0)
    return out;
  double peak = 0.0;
  for (const auto& value : *tracking) {
    if (!value.valid || !std::isfinite(value.wheel_linear_velocity_left_raw) ||
        !std::isfinite(value.wheel_linear_velocity_right_raw)) return out;
    peak = std::max({peak, std::abs(value.wheel_linear_velocity_left_raw),
                    std::abs(value.wheel_linear_velocity_right_raw)});
  }
  if (peak > maximum_wheel_command) {
    out.scale = 0.999 * maximum_wheel_command / peak;
    if (!std::isfinite(out.scale) || out.scale < minimum_command_scale) return out;
    for (auto& value : *tracking) {
      value.linear_velocity_feedforward *= out.scale;
      value.angular_velocity_feedforward *= out.scale;
      value.linear_velocity_raw *= out.scale;
      value.angular_velocity_raw *= out.scale;
      value.wheel_linear_velocity_left_raw *= out.scale;
      value.wheel_linear_velocity_right_raw *= out.scale;
    }
  }
  out.valid = true;
  return out;
}

NominalSpeedBaseline::NominalSpeedBaseline(
    const multi_agv_control::PlanarSupportTracker& tracker,
    const multi_agv_control::FleetPlanarExecutionConfig& execution,
    const NominalSpeedBaselineConfig& config)
    : tracker_(tracker), execution_(tracker, execution), config_(config) {
  if (!std::isfinite(config_.nominal_velocity) || config_.nominal_velocity <= 0.0 ||
      !std::isfinite(config_.target_progress) || config_.target_progress <= 0.0 ||
      config_.target_progress > tracker_.geometry().path().length() + 1e-8 ||
      !std::isfinite(config_.startup_ramp_seconds) || config_.startup_ramp_seconds < 0.0 ||
      !std::isfinite(config_.maximum_dt_seconds) || config_.maximum_dt_seconds <= 0.0 ||
      !std::isfinite(config_.maximum_wheel_command) || config_.maximum_wheel_command <= 0.0 ||
      !std::isfinite(config_.minimum_command_scale) || config_.minimum_command_scale <= 0.0 ||
      config_.minimum_command_scale > 1.0 ||
      (config_.startup_profile != "quintic" && config_.startup_profile != "linear"))
    throw std::invalid_argument("invalid nominal-speed baseline configuration");
}

void NominalSpeedBaseline::reset(double initial_progress) {
  if (!std::isfinite(initial_progress) || initial_progress < 0.0 || initial_progress > config_.target_progress)
    throw std::invalid_argument("invalid nominal-speed initial progress");
  progress_ = initial_progress;
}

NominalSpeedBaselineOutput NominalSpeedBaseline::step(double elapsed, double dt,
    const std::array<multi_agv_control::PlanarPose, 3>& robot_poses,
    const std::array<multi_agv_control::PlanarPose, 3>& support_poses) {
  NominalSpeedBaselineOutput out;
  out.reference_progress = out.next_reference_progress = progress_;
  if (!std::isfinite(elapsed) || elapsed < 0.0 || !std::isfinite(dt) || dt <= 0.0 || dt > config_.maximum_dt_seconds)
    return out;
  if (progress_ >= config_.target_progress) {
    out.valid = out.completed = true;
    return out;
  }
  double ramp = config_.startup_ramp_seconds > 0.0
      ? std::clamp(elapsed / config_.startup_ramp_seconds, 0.0, 1.0) : 1.0;
  if (config_.startup_profile == "quintic") ramp = multi_agv_control::startupSmoothStep(ramp);
  out.requested_velocity = config_.nominal_velocity * ramp;
  out.tracking = tracker_.trackFleet(progress_, out.requested_velocity, robot_poses, support_poses);
  execution_.adapt(progress_, {{out.requested_velocity, out.requested_velocity, out.requested_velocity}},
                   robot_poses, support_poses, &out.tracking);
  const auto limit = limitNominalFleetCommands(&out.tracking, config_.maximum_wheel_command,
                                              config_.minimum_command_scale);
  if (!limit.valid) return out;
  out.command_scale = limit.scale;
  out.reference_velocity = out.requested_velocity * out.command_scale;
  progress_ = std::min(config_.target_progress, progress_ + out.reference_velocity * dt);
  out.next_reference_progress = progress_;
  out.completed = progress_ >= config_.target_progress;
  out.valid = true;
  return out;
}
}  // namespace paper_control

