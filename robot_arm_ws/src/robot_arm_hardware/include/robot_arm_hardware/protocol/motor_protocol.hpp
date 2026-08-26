// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__PROTOCOL__MOTOR_PROTOCOL_HPP_
#define ROBOT_ARM_HARDWARE__PROTOCOL__MOTOR_PROTOCOL_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "robot_arm_hardware/joint_config.hpp"
#include "robot_arm_hardware/transport/transport.hpp"

namespace robot_arm_hardware
{

/// A setpoint for one drive, expressed in MOTOR units.
///
/// The conversion between joint radians and encoder counts belongs to
/// JointConfig, not here: a protocol implementation should only have to worry
/// about the wire format.
struct MotorCommand
{
  int motor_id{0};
  double target_counts{0.0};          ///< position setpoint [counts]
  double target_counts_per_s{0.0};    ///< velocity setpoint [counts/s]
  bool velocity_mode{false};          ///< false -> position mode
};

/// One drive's reply, in MOTOR units.  `valid` is false when the drive did not
/// answer or answered with a corrupt frame; the caller must then not use the
/// numbers at all.
struct MotorFeedback
{
  int motor_id{0};
  int64_t counts{0};
  double counts_per_s{0.0};
  double current{0.0};                ///< [A], NaN when the drive cannot report it
  double temperature{0.0};            ///< [degC], NaN when the drive cannot report it
  uint8_t fault_code{0};
  bool valid{false};
};

struct ProtocolConfig
{
  std::string type{"loopback"};       ///< loopback | simple_ascii
  bool checksum{true};
  int can_base_id{0x100};             ///< used when the transport is CAN
};

/// Abstract motor-controller protocol.
///
/// Everything above this class works in joint space and knows nothing about
/// frames, checksums or register maps.  Supporting a new controller means
/// implementing this interface - ros2_control, MoveIt and the user APIs stay
/// untouched.
class MotorProtocol
{
public:
  virtual ~MotorProtocol() = default;

  /// `transport` stays owned by the caller and must outlive the protocol.
  virtual bool initialize(
    Transport * transport, const std::vector<JointConfig> & joints,
    const ProtocolConfig & config, std::string & error) = 0;

  /// Energise / de-energise the drives.
  virtual bool enable(bool enabled, std::string & error) = 0;

  /// Immediate controlled halt.  Power is kept so the arm holds its pose.
  virtual bool stop(std::string & error) = 0;

  /// Send one setpoint per joint, in the order given to initialize().
  virtual bool write_commands(const std::vector<MotorCommand> & commands, std::string & error) = 0;

  /// Fetch one feedback per joint, in the same order.
  virtual bool read_feedback(std::vector<MotorFeedback> & feedback, std::string & error) = 0;

  virtual std::string name() const = 0;
};

using MotorProtocolPtr = std::unique_ptr<MotorProtocol>;

/// Factory.  Returns nullptr and fills `error` for an unknown type.
MotorProtocolPtr create_protocol(const ProtocolConfig & config, std::string & error);

std::vector<std::string> available_protocols();

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__PROTOCOL__MOTOR_PROTOCOL_HPP_
