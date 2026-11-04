#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include "paper_control/parameters.hpp"

namespace paper_control {
struct CrAControllerConfig {
  multi_agv_control::UpperReferenceConfig upper;
  multi_agv_control::LowerChannelConfig lower;
};

inline CrAControllerConfig crAForScenario(const std::string& scenario) {
  if (scenario != "derating_recovery" && scenario != "severe_derating" && scenario != "compound_disturbances")
    throw std::invalid_argument("CR-A scenario must be derating_recovery, severe_derating or compound_disturbances");
  return {crAUpper(), crALower()};
}

class DeratingScenario {
 public:
  explicit DeratingScenario(double ratio) : config_(deratingScene(ratio)) {}
  chassis_controller::DeratingRatios update(double actual_load_progress, double dt) {
    if (!std::isfinite(actual_load_progress) || !std::isfinite(dt) || dt <= 0)
      throw std::invalid_argument("invalid derating progress or time step");
    if (!activated_ && actual_load_progress >= config_.activation_progress) {
      profile_.accept(1, config_.target, config_.ramp_down);
      activated_ = true;
    }
    if (activated_ && !restored_ && actual_load_progress >= config_.restoration_progress) {
      profile_.accept(2, chassis_controller::DeratingRatios::nominal(), config_.ramp_up);
      restored_ = true;
    }
    return profile_.update(dt);
  }
 private:
  DeratingSceneConfig config_;
  chassis_controller::DeratingProfile profile_;
  bool activated_{false}, restored_{false};
};
}  // namespace paper_control

