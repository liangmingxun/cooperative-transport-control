#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "paper_control/scenarios.hpp"

namespace mac = multi_agv_control;
namespace pc = paper_control;

int main() {
  try {
    std::cout << "Offline API demonstration (deterministic example inputs)\n" << std::setprecision(9);
    std::array<mac::UpperAgentInput, 3> input;
    for (auto& agent : input) {
      agent.candidate_velocity = pc::nominalVelocity;
      agent.mapped_upper_capability = pc::nominalWheelSpeed;
      pc::MarginFeedback feedback;
      feedback.previous_wheel_raw = {{0.155, 0.155}};
      agent.normalised_causal_margins = pc::causalMargins(pc::marginScales(), feedback);
    }
    mac::UpperReferenceGenerator proposed(pc::proposedUpper());
    const auto cra_config = pc::crAForScenario("severe_derating");
    mac::UpperReferenceGenerator cra(cra_config.upper);
    const auto dynamic = proposed.step(input, pc::lowerDt);
    const auto fixed = cra.step(input, pc::lowerDt);
    if (!dynamic.valid || !fixed.valid) return 1;
    std::cout << "Proposed common upper bound: " << dynamic.common_upper << " m/s\n"
              << "CR-A common upper bound: " << fixed.common_upper << " m/s\n";

    mac::M2bController cooperative(pc::cooperativeLrNm());
    mac::M2bGeneratorState state;
    state.position.fill(0.0); state.velocity.fill(pc::nominalVelocity); state.auxiliary.fill(0.0);
    cooperative.reset(state);
    mac::M2bInput coupled;
    coupled.leader_velocity = pc::nominalVelocity;
    coupled.velocity_actual.fill(pc::nominalVelocity);
    coupled.dt_seconds = pc::lowerDt;
    coupled.available_acceleration.fill(pc::nominalWheelAcceleration);
    coupled.available_deceleration.fill(pc::nominalWheelDeceleration);
    coupled.execution_velocity_lower_bound.fill(pc::normalLowerVelocityBound);
    coupled.execution_velocity_upper_bound.fill(pc::normalUpperVelocityBound);
    const auto local = cooperative.step(coupled);
    if (!local.valid) return 1;
    std::cout << "LR-NM cooperative mean reference: " << local.load_velocity_reference << " m/s\n";

    for (const std::string method : {"Proposed", "LR-NM", "VP-PD"}) {
      const auto config = method == "Proposed" ? pc::proposedLower() :
                          method == "LR-NM" ? pc::lrNmLower() : pc::vpPdLower();
      mac::LowerChannelController controller(config);
      controller.reset({{0.0, 0.0}}, pc::initialDisturbanceEstimate);
      mac::LowerChannelInput lower;
      lower.position_actual = 0.002; lower.velocity_actual = 0.098;
      lower.velocity_reference = pc::nominalVelocity;
      lower.velocity_lower_bound = pc::lowerVelocityBound;
      lower.velocity_upper_bound = pc::upperVelocityBound;
      lower.execution_velocity_lower_bound = pc::lowerVelocityBound;
      lower.execution_velocity_upper_bound = pc::upperVelocityBound;
      lower.available_acceleration = pc::nominalWheelAcceleration;
      lower.available_deceleration = pc::nominalWheelDeceleration;
      lower.regressor = {{std::sin(lower.position_actual), lower.velocity_actual * lower.velocity_actual}};
      lower.dt_seconds = pc::lowerDt;
      const auto output = controller.step(lower);
      if (!output.valid) return 1;
      std::cout << method << " progress control input: " << output.input_raw << " m/s^2\n";
    }
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

