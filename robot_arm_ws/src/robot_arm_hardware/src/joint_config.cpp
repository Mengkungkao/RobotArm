// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/joint_config.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "robot_arm_hardware/parameter_utils.hpp"

namespace robot_arm_hardware
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

/// Head-room factor for the plausibility check of a raw encoder reading: a
/// value far outside the reachable range means a corrupt frame, a wrong motor
/// id or a dead encoder - never a real pose.
constexpr double kPlausibilityMargin = 1.5;
}  // namespace

double JointConfig::counts_per_joint_radian() const
{
  const double reduction = apply_gear_ratio ? gear_ratio : 1.0;
  return static_cast<double>(encoder_resolution) * reduction / kTwoPi;
}

double JointConfig::counts_to_position(int64_t counts) const
{
  const double raw = static_cast<double>(counts) * static_cast<double>(encoder_direction) /
    counts_per_joint_radian();
  return static_cast<double>(direction) * (raw - zero_offset);
}

int64_t JointConfig::position_to_counts(double position) const
{
  // `direction` is +-1, so dividing by it is the same as multiplying.
  const double raw = static_cast<double>(direction) * position + zero_offset;
  const double counts = raw * counts_per_joint_radian() * static_cast<double>(encoder_direction);
  return static_cast<int64_t>(std::llround(counts));
}

double JointConfig::counts_to_velocity(double counts_per_second) const
{
  return static_cast<double>(direction) * static_cast<double>(encoder_direction) *
         counts_per_second / counts_per_joint_radian();
}

double JointConfig::velocity_to_counts(double velocity) const
{
  return static_cast<double>(direction) * static_cast<double>(encoder_direction) * velocity *
         counts_per_joint_radian();
}

double JointConfig::current_to_effort(double current) const
{
  const double reduction = apply_gear_ratio ? gear_ratio : 1.0;
  return static_cast<double>(direction) * static_cast<double>(encoder_direction) * current *
         torque_constant * reduction;
}

double JointConfig::effort_to_current(double effort) const
{
  const double reduction = apply_gear_ratio ? gear_ratio : 1.0;
  const double denominator = torque_constant * reduction;
  if (std::abs(denominator) < 1e-9) {
    return 0.0;
  }
  return static_cast<double>(direction) * static_cast<double>(encoder_direction) * effort /
         denominator;
}

bool JointConfig::is_plausible_counts(int64_t counts) const
{
  const double position = counts_to_position(counts);
  if (!std::isfinite(position)) {
    return false;
  }
  const double span = max_position - min_position;
  const double lower = min_position - kPlausibilityMargin * span;
  const double upper = max_position + kPlausibilityMargin * span;
  return position >= lower && position <= upper;
}

void JointConfig::validate() const
{
  if (encoder_resolution <= 0) {
    throw std::invalid_argument(name + ": encoder_resolution must be > 0");
  }
  if (std::abs(gear_ratio) < 1e-9) {
    throw std::invalid_argument(name + ": gear_ratio must not be 0");
  }
  if (encoder_direction != 1 && encoder_direction != -1) {
    throw std::invalid_argument(name + ": encoder_direction must be +1 or -1");
  }
  if (direction != 1 && direction != -1) {
    throw std::invalid_argument(name + ": direction must be +1 or -1");
  }
  if (min_position >= max_position) {
    throw std::invalid_argument(name + ": min_position must be < max_position");
  }
  if (max_velocity <= 0.0) {
    throw std::invalid_argument(name + ": max_velocity must be > 0");
  }
  if (max_effort <= 0.0) {
    throw std::invalid_argument(name + ": max_effort must be > 0");
  }
  if (!std::isfinite(zero_offset)) {
    throw std::invalid_argument(name + ": zero_offset must be finite");
  }
}

JointConfig joint_config_from_parameters(
  const std::string & joint_name,
  const std::unordered_map<std::string, std::string> & parameters,
  bool apply_gear_ratio)
{
  JointConfig config;
  config.name = joint_name;

  // Anything that scales or bounds the motion must be stated explicitly: a
  // defaulted gear ratio or joint limit would move a real machine by the wrong
  // amount, or past its stops, with nothing in the logs to say why.
  config.motor_id = require_int(parameters, "motor_id", joint_name);
  config.encoder_resolution = require_int64(parameters, "encoder_resolution", joint_name);
  config.gear_ratio = require_double(parameters, "gear_ratio", joint_name);
  // These have safe defaults: an unstated encoder sign is "as wired", an
  // unstated zero offset is "uncalibrated", and an unstated thermal limit only
  // affects when a warning is raised.
  config.encoder_direction = get_int(parameters, "encoder_direction", 1);
  config.torque_constant = get_double(parameters, "torque_constant", 0.0);
  config.max_current = get_double(parameters, "max_current", 0.0);
  config.max_temperature = get_double(parameters, "max_temperature", 80.0);

  config.zero_offset = get_double(parameters, "zero_offset", 0.0);
  config.direction = get_int(parameters, "direction", 1);
  config.home_position = get_double(parameters, "home_position", 0.0);

  config.min_position = require_double(parameters, "min_position", joint_name);
  config.max_position = require_double(parameters, "max_position", joint_name);
  config.max_velocity = require_double(parameters, "max_velocity", joint_name);
  config.max_effort = require_double(parameters, "max_effort", joint_name);

  config.apply_gear_ratio = apply_gear_ratio;

  config.validate();
  return config;
}

}  // namespace robot_arm_hardware
