// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__PARAMETER_UTILS_HPP_
#define ROBOT_ARM_HARDWARE__PARAMETER_UTILS_HPP_

#include <string>
#include <unordered_map>

namespace robot_arm_hardware
{

/// Helpers for reading ros2_control `<param>` values, which always arrive as
/// strings.  Xacro writes Python booleans ("True"/"False") while a hand-written
/// URDF usually says "true"/"1", so every spelling is accepted.
///
/// All of them raise std::invalid_argument on a malformed value instead of
/// silently substituting a default - a typo in a gear ratio must never be
/// swallowed by a driver that then drives a real machine.
using ParameterMap = std::unordered_map<std::string, std::string>;

bool has_parameter(const ParameterMap & params, const std::string & key);

std::string get_string(
  const ParameterMap & params, const std::string & key, const std::string & fallback);

double get_double(const ParameterMap & params, const std::string & key, double fallback);

int get_int(const ParameterMap & params, const std::string & key, int fallback);

int64_t get_int64(const ParameterMap & params, const std::string & key, int64_t fallback);

bool get_bool(const ParameterMap & params, const std::string & key, bool fallback);

/// Parse "true"/"True"/"1"/"yes"/"on" and their negatives.
bool parse_bool(const std::string & value, const std::string & context);

/// Accessors for values that MUST be present.
///
/// A driver that quietly substitutes a default gear ratio or encoder
/// resolution will move a real machine by the wrong amount, so a missing value
/// is an error here, never a default.
double require_double(
  const ParameterMap & params, const std::string & key, const std::string & context);

int64_t require_int64(
  const ParameterMap & params, const std::string & key, const std::string & context);

int require_int(
  const ParameterMap & params, const std::string & key, const std::string & context);

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__PARAMETER_UTILS_HPP_
