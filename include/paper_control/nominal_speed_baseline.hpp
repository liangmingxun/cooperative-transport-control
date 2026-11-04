#pragma once
#include <array>
#include <string>
#include "multi_agv_control/planar_support_tracker.hpp"

namespace paper_control {
struct NominalSpeedBaselineConfig {
  double nominal_velocity{0.10};
  double target_progress{5.178229715025710};
  double startup_ramp_seconds{2.5};
  std::string startup_profile{"quintic"};
  double maximum_dt_seconds{0.05};
  double maximum_wheel_command{0.15};
  double minimum_command_scale{0.70};
};

struct NominalFleetLimitResult {
  bool valid{false};
  double scale{1.0};
};

NominalFleetLimitResult limitNominalFleetCommands(
    std::array<multi_agv_control::PlanarTrackingResult, 3>* tracking,
    double maximum_wheel_command, double minimum_command_scale);

struct NominalSpeedBaselineOutput {
  bool valid{false};
  bool completed{false};
  double reference_progress{0.0};
  double next_reference_progress{0.0};
  double requested_velocity{0.0};
  double reference_velocity{0.0};
  double command_scale{1.0};
  std::array<multi_agv_control::PlanarTrackingResult, 3> tracking;
};

// The caller owns the immutable tracker and supplies actual robot/support poses.
// This interface computes references and wheel demands without transport I/O.
class NominalSpeedBaseline {
 public:
  NominalSpeedBaseline(const multi_agv_control::PlanarSupportTracker& tracker,
      const multi_agv_control::FleetPlanarExecutionConfig& execution,
      const NominalSpeedBaselineConfig& config);
  void reset(double initial_progress = 0.0);
  NominalSpeedBaselineOutput step(double elapsed, double dt,
      const std::array<multi_agv_control::PlanarPose, 3>& robot_poses,
      const std::array<multi_agv_control::PlanarPose, 3>& support_poses);
 private:
  const multi_agv_control::PlanarSupportTracker& tracker_;
  multi_agv_control::FleetPlanarExecutionAdapter execution_;
  NominalSpeedBaselineConfig config_;
  double progress_{0.0};
};
}  // namespace paper_control

