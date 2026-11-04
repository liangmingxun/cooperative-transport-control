#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include "chassis_controller/wheel_command_limiter.hpp"
#include "paper_control/scenarios.hpp"

namespace pc = paper_control;
namespace mac = multi_agv_control;

void describeCrA(const std::string& scene) {
  const auto config = pc::crAForScenario(scene);
  mac::UpperReferenceGenerator upper(config.upper);
  std::array<mac::UpperAgentInput, 3> input;
  for (auto& member : input) {
    member.candidate_velocity = pc::nominalVelocity;
    member.mapped_upper_capability = pc::nominalWheelSpeed;
  }
  const auto reference = upper.step(input, pc::lowerDt);
  if (!reference.valid) throw std::runtime_error("invalid CR-A reference");
  std::cout << "CR-A controller interface: " << mac::upperModeName(config.upper.mode)
            << " + " << mac::lowerModeName(config.lower.mode)
            << "; fixed common upper bound: " << reference.common_upper << " m/s\n";
}

void demonstrateDerating(double target) {
  pc::DeratingScenario schedule(target);
  for (int step = 0; step <= 3200; ++step) {
    const double time = step * pc::lowerDt;
    const double progress = time * pc::nominalVelocity;
    const auto ratios = schedule.update(progress, pc::lowerDt);
    if (step % 500 == 0 || step == 1600 || step == 3100)
      std::cout << target << ',' << time << ',' << progress << ',' << ratios.speed_left
                << ',' << pc::nominalWheelSpeed * ratios.speed_left << '\n';
  }
}

#ifdef PAPER_WITH_GEOMETRY
double wheelPeak(const std::array<mac::PlanarTrackingResult, 3>& values) {
  double peak = 0.0;
  for (const auto& value : values) {
    if (!value.valid) throw std::runtime_error("invalid normal-condition planar tracking result");
    peak = std::max({peak, std::abs(value.wheel_linear_velocity_left_raw),
                     std::abs(value.wheel_linear_velocity_right_raw)});
  }
  return peak;
}

void demonstrateNormalConditions() {
  const mac::SCurvePath path(pc::circlePath());
  const mac::SupportGeometry geometry(path, pc::supportFormation());
  const mac::PlanarSupportTracker tracker(geometry, pc::planarTracker());
  const mac::FleetPlanarExecutionAdapter execution(tracker, pc::executionAdapter());
  const mac::PlanarSupportTracker baseline_tracker(geometry, pc::nsBTracker());
  const auto baseline_config = pc::nominalSpeedBaselineConfig();
  pc::NominalSpeedBaseline baseline(baseline_tracker, pc::nsBExecutionAdapter(), baseline_config);
  // One steady-motion API evaluation at an explicitly supplied example state.
  const double progress = .5;
  std::array<mac::PlanarPose, 3> robots, supports;
  for (std::size_t i = 0; i < 3; ++i) {
    mac::PlanarTrackingInput seed;
    seed.support_index = i; seed.load_progress_reference = progress;
    seed.channel_velocity_command = pc::normalVelocity;
    const auto reference = tracker.track(seed);
    if (!reference.valid) throw std::runtime_error("invalid example reference pose");
    robots[i] = reference.chassis_pose_reference;
    supports[i] = reference.support_pose_reference;
  }
  std::array<mac::UpperAgentInput, 3> upper_input;
  for (auto& member : upper_input) {
    member.candidate_velocity = pc::normalVelocity;
    member.mapped_upper_capability = pc::nominalWheelSpeed;
  }
  mac::UpperReferenceGenerator upper(pc::proposedUpper());
  const auto reference = upper.step(upper_input, pc::normalDt);
  if (!reference.valid) throw std::runtime_error("invalid proposed normal-condition reference");
  mac::DistributedReferenceState candidates;
  candidates.position.fill(progress); candidates.velocity.fill(pc::normalVelocity);
  mac::DistributedReferenceInput distributed_input;
  distributed_input.leader_position = progress;
  distributed_input.leader_velocity = pc::normalVelocity;
  distributed_input.dt_seconds = pc::normalDt;
  distributed_input.inner_margin = pc::proposedUpper().agents[0].inner_margin;
  for (std::size_t i = 0; i < 3; ++i) distributed_input.boundary[i] = reference.agents[i].boundary;
  const auto distributed = mac::stepDistributedReference({}, candidates, distributed_input);
  if (!distributed.valid) throw std::runtime_error("invalid normal-condition candidate update");
  std::array<double, 3> proposed_channels;
  for (std::size_t i = 0; i < 3; ++i) {
    mac::LowerChannelController lower(pc::proposedLower());
    lower.reset({{0., 0.}}, pc::normalInitialDisturbanceEstimate);
    mac::LowerChannelInput input;
    input.position_actual = input.position_reference = progress;
    input.velocity_actual = pc::normalVelocity;
    input.velocity_reference = distributed.common_velocity;
    input.velocity_lower_bound = pc::normalLowerVelocityBound;
    input.velocity_upper_bound = pc::normalUpperVelocityBound;
    input.execution_velocity_lower_bound = pc::normalLowerVelocityBound;
    input.execution_velocity_upper_bound = pc::normalUpperVelocityBound;
    input.available_acceleration = pc::nominalWheelAcceleration;
    input.available_deceleration = pc::nominalWheelDeceleration;
    input.regressor = {{std::sin(progress), pc::normalVelocity * pc::normalVelocity}};
    input.dt_seconds = pc::normalDt;
    const auto control = lower.step(input);
    if (!control.valid) throw std::runtime_error("invalid proposed normal-condition lower control");
    proposed_channels[i] = control.channel_velocity_command;
  }
  std::array<mac::PlanarTrackingResult, 3> proposed_tracking;
  for (std::size_t i = 0; i < 3; ++i)
    proposed_tracking[i] = tracker.track({i, progress, proposed_channels[i], robots[i], supports[i]});
  execution.adapt(progress, proposed_channels, robots, supports, &proposed_tracking);

  mac::M2bController lrnm(pc::cooperativeLrNm());
  mac::M2bGeneratorState local;
  local.position.fill(progress); local.velocity.fill(pc::normalVelocity); local.auxiliary.fill(0.0);
  lrnm.reset(local);
  mac::M2bInput input;
  input.leader_position = progress; input.leader_velocity = pc::normalVelocity;
  input.position_actual.fill(progress); input.velocity_actual.fill(pc::normalVelocity);
  input.reported_capability.fill(pc::nominalWheelSpeed); input.dt_seconds = pc::normalDt;
  input.available_acceleration.fill(pc::nominalWheelAcceleration);
  input.available_deceleration.fill(pc::nominalWheelDeceleration);
  input.execution_velocity_lower_bound.fill(pc::normalLowerVelocityBound);
  input.execution_velocity_upper_bound.fill(pc::normalUpperVelocityBound);
  const auto local_control = lrnm.step(input);
  if (!local_control.valid) throw std::runtime_error("invalid normal-condition LR-NM control");
  std::array<mac::PlanarTrackingResult, 3> local_tracking;
  for (std::size_t i = 0; i < 3; ++i)
    local_tracking[i] = tracker.track({i, progress, local_control.channel_velocity_command[i], robots[i], supports[i]});
  execution.adapt(progress, local_control.channel_velocity_command, robots, supports, &local_tracking);

  baseline.reset(progress);
  const auto nominal = baseline.step(baseline_config.startup_ramp_seconds, pc::normalDt, robots, supports);
  if (!nominal.valid) throw std::runtime_error("invalid normal-condition NS-B control");
  const auto methods = pc::normalMethods();
  std::cout << "Manuscript section: " << pc::normalSection << "\n"
            << "method,reference_velocity_mps,peak_wheel_demand_mps\n"
            << methods[0] << ',' << distributed.common_velocity << ',' << wheelPeak(proposed_tracking) << '\n'
            << methods[1] << ',' << local_control.load_velocity_reference << ',' << wheelPeak(local_tracking) << '\n'
            << methods[2] << ',' << nominal.reference_velocity << ',' << wheelPeak(nominal.tracking) << '\n';
}
#endif

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      std::cerr << "Usage: scenario_example normal_conditions|derating_recovery|severe_derating|compound_disturbances|lower_layer_comparison\n";
      return 2;
    }
    std::cout << "Offline scenario demonstration (example inputs)\n" << std::setprecision(9);
    const std::string scene = argv[1];
    if (scene == "normal_conditions") {
#ifdef PAPER_WITH_GEOMETRY
      demonstrateNormalConditions();
#else
      throw std::runtime_error("normal_conditions requires the geometry-enabled build");
#endif
    } else if (scene == "derating_recovery") {
      describeCrA(scene);
      std::cout << "Manuscript section: " << pc::deratingRecoverySection << '\n';
      const auto methods = pc::recoveryMethods();
      std::cout << "Methods: " << methods[0] << ", " << methods[1] << ", " << methods[2] << '\n';
      std::cout << "target_ratio,example_time_s,example_progress_m,current_ratio,wheel_limit_mps\n";
      demonstrateDerating(pc::deratingRecoveryRatio);
    } else if (scene == "severe_derating") {
      std::cout << "Manuscript section: " << pc::severeDeratingSection << '\n';
      describeCrA(scene);
      std::cout << "target_ratio,example_time_s,example_progress_m,current_ratio,wheel_limit_mps\n";
      for (double target : {.75, .80}) demonstrateDerating(target);
    } else if (scene == "compound_disturbances") {
      std::cout << "Manuscript section: " << pc::compoundDisturbanceSection << '\n';
      describeCrA(scene);
      const auto config = pc::compoundDisturbance();
      std::cout << "robot,time_s,progress_m,effectiveness,longitudinal_disturbance_mps,yaw_disturbance_radps,applied_left_mps,applied_right_mps\n";
      for (int robot = 1; robot <= 3; ++robot) {
        mac::CandidateBSpatialCompositeDisturbance disturbance;
        chassis_controller::WheelCommandLimiter limiter;
        const chassis_controller::WheelLimits limits{.16, .16, 1., 1., 2., 2.};
        for (int step = 0; step <= 1000; ++step) {
          const double time = step * pc::lowerDt;
          const double progress = 1.9 + pc::nominalVelocity * time;
          const auto out = disturbance.evaluate(config, robot, progress, time, .10, .10);
          const auto executed = limiter.update({out.left_post_disturbance, out.right_post_disturbance}, limits, pc::lowerDt);
          if (!executed.valid) return 1;
          if (step % 200 == 0)
            std::cout << robot << ',' << time << ',' << progress << ',' << out.effectiveness << ','
                      << out.longitudinal_disturbance << ',' << out.yaw_disturbance << ','
                      << executed.applied.left << ',' << executed.applied.right << '\n';
        }
      }
    } else if (scene == "lower_layer_comparison") {
      std::cout << "Manuscript section: " << pc::lowerComparisonSection << '\n';
      std::cout << "Common reference velocity: " << pc::nominalVelocity << " m/s\n"
                << "Progress-velocity state interval: [" << pc::lowerVelocityBound << ", "
                << pc::upperVelocityBound << "] m/s\n"
                << "Controllers: Proposed (R1), LR-NM (PaperNM), VP-PD (PDPenalty)\n"
                << "VP-PD penalty coefficient: " << pc::vpPdLower().pd_penalty_kpd << '\n';
    } else { std::cerr << "Unknown scenario\n"; return 2; }
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

