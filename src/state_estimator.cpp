#include "multi_agv_control/state_estimator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace multi_agv_control {
namespace {

Eigen::Vector2d rotate(const Eigen::Vector2d& value, double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * value.x() - s * value.y(),
          s * value.x() + c * value.y()};
}

}  // namespace

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector2d supportPointVelocity(const PlanarPose& base,
    const PlanarPose& offset, const Eigen::Vector2d& velocity, double omega) {
  const Eigen::Vector2d lever_velocity(-omega*offset.position.y(),
                                      omega*offset.position.x());
  return rotate(velocity+lever_velocity, base.yaw);
}

PlanarPose composePose(const PlanarPose& parent, const PlanarPose& child) {
  return {parent.position + rotate(child.position, parent.yaw),
          normalizeAngle(parent.yaw + child.yaw)};
}

StateEstimator::StateEstimator(PathProjector projector,
                               const StateEstimatorConfig& config)
    : projector_(std::move(projector)), config_(config) {
  if (!std::isfinite(config_.filter_alpha) || config_.filter_alpha <= 0.0 ||
      config_.filter_alpha >= 1.0 ||
      !std::isfinite(config_.projection_half_window) ||
      config_.projection_half_window <= 0.0 ||
      !std::isfinite(config_.initial_progress) || config_.initial_progress < 0.0 ||
      config_.initial_progress > projector_.pathLength() ||
      !std::isfinite(config_.minimum_measurement_interval) ||
      config_.minimum_measurement_interval <= 0.0 ||
      !std::isfinite(config_.maximum_measurement_interval) ||
      config_.maximum_measurement_interval <= config_.minimum_measurement_interval ||
      !std::isfinite(config_.maximum_absolute_speed) ||
      config_.maximum_absolute_speed <= 0.0) {
    throw std::invalid_argument("invalid state estimator configuration");
  }
  if (!std::isfinite(config_.maximum_position_correction) || config_.maximum_position_correction < 0.0)
    throw std::invalid_argument("invalid position correction envelope");
  reset(config_.initial_progress);
}

StateEstimate StateEstimator::invalidEstimate(
    double measurement_stamp, const ProjectionResult& projection) const {
  StateEstimate result;
  result.measurement_stamp = measurement_stamp;
  result.projection = projection;
  return result;
}

StateEstimate StateEstimator::update(const Eigen::Vector2d& measured_position,
                                     double measurement_stamp) {
  return updateImpl(measured_position, measurement_stamp, nullptr);
}

StateEstimate StateEstimator::updateWithVelocity(
    const Eigen::Vector2d& position, double stamp,
    const Eigen::Vector2d& velocity) {
  return updateImpl(position, stamp, &velocity);
}

StateEstimate StateEstimator::updateImpl(const Eigen::Vector2d& measured_position,
    double measurement_stamp, const Eigen::Vector2d* measured_velocity) {
  if (!std::isfinite(measurement_stamp)) {
    return invalidEstimate(measurement_stamp, {});
  }
  const double seed = initialized_ ? last_estimate_.progress
                                   : config_.initial_progress;
  const auto projection = projector_.project(
      measured_position, seed, config_.projection_half_window);
  if (!projection.valid) {
    return invalidEstimate(measurement_stamp, projection);
  }
  const double motion_speed = measured_velocity ?
      projector_.projectedVelocity(measured_position, projection, *measured_velocity) : 0.0;
  if (measured_velocity && (!std::isfinite(motion_speed) ||
      std::abs(motion_speed) > config_.maximum_absolute_speed))
    return invalidEstimate(measurement_stamp, projection);

  if (!initialized_) {
    last_estimate_.valid = true;
    last_estimate_.speed_initialized = false;
    last_estimate_.measurement_stamp = measurement_stamp;
    last_estimate_.progress = projection.s;
    last_estimate_.speed = 0.0;
    last_estimate_.projection = projection;
    initialized_ = true;
    return last_estimate_;
  }

  const double dt = measurement_stamp - last_estimate_.measurement_stamp;
  if (!std::isfinite(dt) || dt <= 0.0) {
    return invalidEstimate(measurement_stamp, projection);
  }
  // A synchronized camera snapshot can legitimately select a source stamp
  // only a few microseconds newer than the preceding one.  It contains no
  // useful new time baseline for differentiation, so retain the last accepted
  // estimate without declaring a temporal resynchronization.  Once the source
  // stamp has advanced by the configured interval, the normal update below
  // consumes the accumulated displacement.  Do not lower the interval: doing
  // so would turn camera position noise into very large velocity derivatives.
  if (dt < config_.minimum_measurement_interval) {
    return last_estimate_;
  }
  if (measured_velocity && config_.enforce_progress_correction &&
      dt <= config_.maximum_measurement_interval &&
      std::abs(projection.s-last_estimate_.progress) >
      config_.maximum_absolute_speed*dt+config_.maximum_position_correction)
    return invalidEstimate(measurement_stamp, projection);
  const double raw_speed = measured_velocity ? motion_speed :
      (projection.s - last_estimate_.progress) / dt;
  if (dt > config_.maximum_measurement_interval ||
      !std::isfinite(raw_speed) ||
      std::abs(raw_speed) > config_.maximum_absolute_speed) {
    last_estimate_.valid = true;
    last_estimate_.speed_initialized = false;
    last_estimate_.measurement_stamp = measurement_stamp;
    last_estimate_.progress = projection.s;
    last_estimate_.speed = 0.0;
    last_estimate_.projection = projection;
    return invalidEstimate(measurement_stamp, projection);
  }

  const double previous_speed = last_estimate_.speed_initialized
                                    ? last_estimate_.speed : 0.0;
  last_estimate_.valid = true;
  last_estimate_.speed_initialized = true;
  last_estimate_.measurement_stamp = measurement_stamp;
  last_estimate_.progress = projection.s;
  last_estimate_.speed = config_.filter_alpha * previous_speed +
                         (1.0 - config_.filter_alpha) * raw_speed;
  last_estimate_.projection = projection;
  return last_estimate_;
}

void StateEstimator::reset(double initial_progress) {
  if (!std::isfinite(initial_progress) || initial_progress < 0.0 ||
      initial_progress > projector_.pathLength()) {
    throw std::invalid_argument("initial progress lies outside the path");
  }
  config_.initial_progress = initial_progress;
  initialized_ = false;
  last_estimate_ = StateEstimate{};
  last_estimate_.progress = initial_progress;
}

}  // namespace multi_agv_control
