// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__JOINT_CONFIG_HPP_
#define ROBOT_ARM_HARDWARE__JOINT_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace robot_arm_hardware
{

/// Per-joint drive, encoder and calibration data.
///
/// Every field is populated from the ros2_control `<param>` entries that
/// robot_arm_description injects from hardware.yaml and calibration.yaml, so
/// nothing here is ever hard-coded and no two joints have to be alike.
struct JointConfig
{
  std::string name;

  // --- drive / encoder -----------------------------------------------------
  int motor_id{0};
  int64_t encoder_resolution{4096};   ///< counts per *motor* revolution
  double gear_ratio{1.0};             ///< motor revolutions per joint revolution
  int encoder_direction{1};           ///< +1/-1, encoder sign w.r.t. the motor
  double torque_constant{0.0};        ///< [Nm/A] motor-side
  double max_current{0.0};            ///< [A]
  double max_temperature{80.0};       ///< [degC] diagnostics warning threshold

  // --- calibration ---------------------------------------------------------
  double zero_offset{0.0};            ///< [rad] raw angle at the mechanical zero
  int direction{1};                   ///< +1/-1, joint sign w.r.t. the URDF axis
  double home_position{0.0};          ///< [rad] target of the homing routine

  // --- limits enforced by the driver --------------------------------------
  double min_position{-3.1416};
  double max_position{3.1416};
  double max_velocity{3.14};
  double max_effort{100.0};

  /// When false the gearbox reduction is expected to be handled by a
  /// transmission_interface plugin instead of by this driver.
  bool apply_gear_ratio{true};

  /// Counts of the encoder per radian of *joint* motion.
  double counts_per_joint_radian() const;

  /// Raw encoder counts -> calibrated joint position [rad].
  double counts_to_position(int64_t counts) const;

  /// Calibrated joint position [rad] -> raw encoder counts.
  int64_t position_to_counts(double position) const;

  /// Encoder counts/s -> calibrated joint velocity [rad/s].
  double counts_to_velocity(double counts_per_second) const;

  /// Joint velocity [rad/s] -> encoder counts/s.
  double velocity_to_counts(double velocity) const;

  /// Motor current [A] -> joint effort [Nm] (sign follows the joint axis).
  double current_to_effort(double current) const;

  /// Joint effort [Nm] -> motor current [A].
  double effort_to_current(double effort) const;

  /// True when the value is finite and inside the mechanically possible range.
  bool is_plausible_counts(int64_t counts) const;

  /// Throws std::invalid_argument when the configuration cannot work
  /// (zero resolution, zero gear ratio, inverted limits, ...).
  void validate() const;
};

/// Build a JointConfig from a ros2_control parameter map.
/// `joint_name` is used for error messages only.  Missing entries fall back to
/// the struct defaults; malformed entries raise std::invalid_argument.
JointConfig joint_config_from_parameters(
  const std::string & joint_name,
  const std::unordered_map<std::string, std::string> & parameters,
  bool apply_gear_ratio = true);

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__JOINT_CONFIG_HPP_
