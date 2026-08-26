// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__SAFETY_CHECKER_HPP_
#define ROBOT_ARM_HARDWARE__SAFETY_CHECKER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "robot_arm_hardware/joint_config.hpp"

namespace robot_arm_hardware
{

/// Tunables of the software safety layer, read from safety.yaml / hardware.yaml.
struct SafetyLimits
{
  double command_timeout{0.25};     ///< [s] no fresh command -> stop the arm
  double comm_timeout{0.20};        ///< [s] no valid feedback -> stop the arm
  int max_consecutive_errors{5};    ///< bus errors tolerated in a row
  double velocity_scale{1.0};       ///< 0..1, fraction of the configured max velocity
  double position_margin{0.0};      ///< [rad] extra distance kept from the limits
  bool clamp_commands{true};        ///< clamp (true) or reject (false) out-of-range commands
};

/// Outcome of one safety evaluation.
struct SafetyReport
{
  enum class Level : uint8_t
  {
    kOk = 0,
    kWarn = 1,
    kViolation = 2,
    kEStop = 3,
  };

  Level level{Level::kOk};
  bool e_stop_active{false};
  bool position_limit_violation{false};
  bool velocity_limit_violation{false};
  bool effort_limit_violation{false};
  bool command_timeout{false};
  bool communication_timeout{false};
  bool invalid_feedback{false};

  /// False means the caller must not send motion commands this cycle.
  bool motion_allowed{true};

  std::vector<std::string> violating_joints;
  std::string message;
};

/// Software safety layer of the driver.
///
/// It is deliberately free of ROS types and of any clock: every entry point
/// takes the current time as an argument.  That makes every rule - limits,
/// timeouts, e-stop latching - directly unit-testable, which is the only way
/// to trust a safety layer that will eventually gate a physical machine.
///
/// The class does not merely report: `check_commands` rewrites the command
/// vectors in place, so a violated limit cannot reach the drives.
class SafetyChecker
{
public:
  SafetyChecker() = default;

  void configure(std::vector<JointConfig> joints, const SafetyLimits & limits);

  /// Engage/release the emergency stop.  Engaging takes effect immediately and
  /// is latched until it is explicitly released.
  void set_e_stop(bool active, const std::string & reason);
  bool e_stop_active() const;
  std::string e_stop_reason() const;

  /// Timestamps, in seconds on a monotonic clock.
  void notify_command(double now);
  void notify_feedback(double now);
  void notify_error();
  void notify_success();
  int consecutive_errors() const;

  /// Validate one cycle of measured state.  Non-finite values, or positions
  /// far outside the mechanical range, mark the feedback invalid so the caller
  /// can stop instead of acting on a corrupt encoder reading.
  bool validate_feedback(
    const std::vector<double> & positions, const std::vector<double> & velocities,
    SafetyReport & report) const;

  /// Clamp the commands to what is safe and report what had to be changed.
  ///
  /// `measured_positions` is used to enforce the velocity limit across cycles;
  /// `period` is the control period in seconds.
  SafetyReport check_commands(
    std::vector<double> & position_commands, std::vector<double> & velocity_commands,
    const std::vector<double> & measured_positions, double period, double now) const;

  /// Report for a cycle in which no motion may be commanded (e-stop, timeout).
  SafetyReport stopped_report(const std::string & reason) const;

  const std::vector<JointConfig> & joints() const {return joints_;}
  const SafetyLimits & limits() const {return limits_;}

private:
  mutable std::mutex mutex_;
  std::vector<JointConfig> joints_;
  SafetyLimits limits_;

  bool e_stop_{false};
  std::string e_stop_reason_;
  double last_command_time_{-1.0};
  double last_feedback_time_{-1.0};
  int consecutive_errors_{0};
};

/// "OK" / "WARN" / "VIOLATION" / "E-STOP", for logs and diagnostics.
std::string to_string(SafetyReport::Level level);

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__SAFETY_CHECKER_HPP_
