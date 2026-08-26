// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/parameter_utils.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace robot_arm_hardware
{
namespace
{

std::string to_lower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\n\r");
  return value.substr(begin, end - begin + 1);
}

}  // namespace

bool has_parameter(const ParameterMap & params, const std::string & key)
{
  return params.find(key) != params.end();
}

std::string get_string(
  const ParameterMap & params, const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : trim(it->second);
}

double get_double(const ParameterMap & params, const std::string & key, double fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || trim(it->second).empty()) {
    return fallback;
  }
  try {
    return std::stod(trim(it->second));
  } catch (const std::exception &) {
    throw std::invalid_argument("parameter '" + key + "' is not a number: '" + it->second + "'");
  }
}

int get_int(const ParameterMap & params, const std::string & key, int fallback)
{
  return static_cast<int>(get_int64(params, key, fallback));
}

int64_t get_int64(const ParameterMap & params, const std::string & key, int64_t fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || trim(it->second).empty()) {
    return fallback;
  }
  try {
    return std::stoll(trim(it->second));
  } catch (const std::exception &) {
    throw std::invalid_argument("parameter '" + key + "' is not an integer: '" + it->second + "'");
  }
}

bool get_bool(const ParameterMap & params, const std::string & key, bool fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || trim(it->second).empty()) {
    return fallback;
  }
  return parse_bool(it->second, key);
}

namespace
{
void ensure_present(
  const ParameterMap & params, const std::string & key, const std::string & context)
{
  const auto it = params.find(key);
  if (it == params.end() || trim(it->second).empty()) {
    throw std::invalid_argument(
      context + ": required parameter '" + key + "' is missing. It is normally injected "
      "from hardware.yaml/calibration.yaml by robot_arm_description/urdf/ros2_control.xacro.");
  }
}
}  // namespace

double require_double(
  const ParameterMap & params, const std::string & key, const std::string & context)
{
  ensure_present(params, key, context);
  return get_double(params, key, 0.0);
}

int64_t require_int64(
  const ParameterMap & params, const std::string & key, const std::string & context)
{
  ensure_present(params, key, context);
  return get_int64(params, key, 0);
}

int require_int(
  const ParameterMap & params, const std::string & key, const std::string & context)
{
  return static_cast<int>(require_int64(params, key, context));
}

bool parse_bool(const std::string & value, const std::string & context)
{
  const std::string normalised = to_lower(trim(value));
  if (normalised == "true" || normalised == "1" || normalised == "yes" || normalised == "on") {
    return true;
  }
  if (normalised == "false" || normalised == "0" || normalised == "no" || normalised == "off") {
    return false;
  }
  throw std::invalid_argument("parameter '" + context + "' is not a boolean: '" + value + "'");
}

}  // namespace robot_arm_hardware
