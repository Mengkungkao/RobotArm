// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/safety_checker.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace robot_arm_hardware
{
namespace
{
/// A measured position this far outside the configured range can only be a
/// corrupt reading, not a real pose.
constexpr double kFeedbackPlausibilityMargin = 1.5;

/// Warn (but do not clamp) once a command comes this close to a limit.
constexpr double kWarnBand = 0.05;   // [rad]

void add_joint(std::vector<std::string> & joints, const std::string & name)
{
  if (std::find(joints.begin(), joints.end(), name) == joints.end()) {
    joints.push_back(name);
  }
}
}  // namespace

std::string to_string(SafetyReport::Level level)
{
  switch (level) {
    case SafetyReport::Level::kOk: return "OK";
    case SafetyReport::Level::kWarn: return "WARN";
    case SafetyReport::Level::kViolation: return "VIOLATION";
    case SafetyReport::Level::kEStop: return "E-STOP";
  }
  return "UNKNOWN";
}

void SafetyChecker::configure(std::vector<JointConfig> joints, const SafetyLimits & limits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  joints_ = std::move(joints);
  limits_ = limits;
  last_command_time_ = -1.0;
  last_feedback_time_ = -1.0;
  consecutive_errors_ = 0;
}

void SafetyChecker::set_e_stop(bool active, const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  e_stop_ = active;
  e_stop_reason_ = active ? reason : "";
}

bool SafetyChecker::e_stop_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return e_stop_;
}

std::string SafetyChecker::e_stop_reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return e_stop_reason_;
}

void SafetyChecker::notify_command(double now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_command_time_ = now;
}

void SafetyChecker::notify_feedback(double now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_feedback_time_ = now;
}

void SafetyChecker::notify_error()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++consecutive_errors_;
}

void SafetyChecker::notify_success()
{
  std::lock_guard<std::mutex> lock(mutex_);
  consecutive_errors_ = 0;
}

int SafetyChecker::consecutive_errors() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return consecutive_errors_;
}

bool SafetyChecker::validate_feedback(
  const std::vector<double> & positions, const std::vector<double> & velocities,
  SafetyReport & report) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool valid = true;

  for (std::size_t i = 0; i < joints_.size() && i < positions.size(); ++i) {
    const auto & joint = joints_[i];
    const double span = joint.max_position - joint.min_position;
    const double lower = joint.min_position - kFeedbackPlausibilityMargin * span;
    const double upper = joint.max_position + kFeedbackPlausibilityMargin * span;

    if (!std::isfinite(positions[i]) || positions[i] < lower || positions[i] > upper) {
      report.invalid_feedback = true;
      add_joint(report.violating_joints, joint.name);
      valid = false;
      continue;
    }
    if (i < velocities.size() && !std::isfinite(velocities[i])) {
      report.invalid_feedback = true;
      add_joint(report.violating_joints, joint.name);
      valid = false;
    }
  }

  if (!valid) {
    report.level = SafetyReport::Level::kViolation;
    report.motion_allowed = false;
    report.message = "implausible or non-finite encoder feedback";
  }
  return valid;
}

SafetyReport SafetyChecker::check_commands(
  std::vector<double> & position_commands, std::vector<double> & velocity_commands,
  const std::vector<double> & measured_positions, double period, double now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  SafetyReport report;

  // ---- unconditional stops ------------------------------------------------
  if (e_stop_) {
    report.level = SafetyReport::Level::kEStop;
    report.e_stop_active = true;
    report.motion_allowed = false;
    report.message = e_stop_reason_.empty() ?
      "emergency stop engaged" : "emergency stop engaged: " + e_stop_reason_;
  }

  if (last_command_time_ >= 0.0 && (now - last_command_time_) > limits_.command_timeout) {
    report.command_timeout = true;
    report.motion_allowed = false;
    if (report.level < SafetyReport::Level::kViolation) {
      report.level = SafetyReport::Level::kViolation;
    }
    report.message = "no fresh command for " +
      std::to_string(now - last_command_time_) + " s";
  }

  if (last_feedback_time_ >= 0.0 && (now - last_feedback_time_) > limits_.comm_timeout) {
    report.communication_timeout = true;
    report.motion_allowed = false;
    if (report.level < SafetyReport::Level::kViolation) {
      report.level = SafetyReport::Level::kViolation;
    }
    report.message = "no valid feedback for " +
      std::to_string(now - last_feedback_time_) + " s";
  }

  // Even when motion is forbidden the commands are neutralised, so that a
  // stale setpoint can never reach the drives once motion resumes.
  if (!report.motion_allowed) {
    for (std::size_t i = 0; i < position_commands.size(); ++i) {
      if (i < measured_positions.size() && std::isfinite(measured_positions[i])) {
        position_commands[i] = measured_positions[i];   // hold the current pose
      }
    }
    std::fill(velocity_commands.begin(), velocity_commands.end(), 0.0);
    return report;
  }

  // ---- per-joint limits ---------------------------------------------------
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = joints_[i];
    const double lower = joint.min_position + limits_.position_margin;
    const double upper = joint.max_position - limits_.position_margin;
    const double max_velocity = joint.max_velocity * limits_.velocity_scale;

    if (i < position_commands.size()) {
      double & command = position_commands[i];

      if (!std::isfinite(command)) {
        // A NaN setpoint must never be forwarded; hold the measured pose.
        command = (i < measured_positions.size() && std::isfinite(measured_positions[i])) ?
          measured_positions[i] : 0.0;
        report.position_limit_violation = true;
        add_joint(report.violating_joints, joint.name);
      }

      if (command < lower || command > upper) {
        report.position_limit_violation = true;
        add_joint(report.violating_joints, joint.name);
        if (limits_.clamp_commands) {
          command = std::clamp(command, lower, upper);
        } else {
          report.motion_allowed = false;
        }
      } else if (command < lower + kWarnBand || command > upper - kWarnBand) {
        if (report.level == SafetyReport::Level::kOk) {
          report.level = SafetyReport::Level::kWarn;
        }
      }

      // Velocity limit across cycles: cap how far the setpoint may move.
      if (period > 0.0 && i < measured_positions.size() &&
        std::isfinite(measured_positions[i]))
      {
        const double max_step = max_velocity * period;
        const double step = command - measured_positions[i];
        if (std::abs(step) > max_step) {
          report.velocity_limit_violation = true;
          add_joint(report.violating_joints, joint.name);
          command = measured_positions[i] + std::copysign(max_step, step);
        }
      }
    }

    if (i < velocity_commands.size()) {
      double & command = velocity_commands[i];
      if (!std::isfinite(command)) {
        command = 0.0;
        report.velocity_limit_violation = true;
        add_joint(report.violating_joints, joint.name);
      } else if (std::abs(command) > max_velocity) {
        report.velocity_limit_violation = true;
        add_joint(report.violating_joints, joint.name);
        command = std::clamp(command, -max_velocity, max_velocity);
      }
    }
  }

  if ((report.position_limit_violation || report.velocity_limit_violation) &&
    report.level < SafetyReport::Level::kViolation)
  {
    report.level = SafetyReport::Level::kViolation;
    report.message = "command clamped to the configured joint limits";
  }
  return report;
}

SafetyReport SafetyChecker::stopped_report(const std::string & reason) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  SafetyReport report;
  report.level = e_stop_ ? SafetyReport::Level::kEStop : SafetyReport::Level::kViolation;
  report.e_stop_active = e_stop_;
  report.motion_allowed = false;
  report.message = reason;
  return report;
}

}  // namespace robot_arm_hardware
