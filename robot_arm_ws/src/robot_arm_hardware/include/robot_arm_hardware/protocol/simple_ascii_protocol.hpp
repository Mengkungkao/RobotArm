// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__PROTOCOL__SIMPLE_ASCII_PROTOCOL_HPP_
#define ROBOT_ARM_HARDWARE__PROTOCOL__SIMPLE_ASCII_PROTOCOL_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "robot_arm_hardware/protocol/motor_protocol.hpp"

namespace robot_arm_hardware
{

/// Reference line protocol for a multi-axis motor controller.
///
/// It is deliberately simple, human readable and easy to implement in
/// firmware, and it is complete: this class is what a real deployment either
/// uses directly or copies as the template for a vendor protocol.
///
/// WIRE FORMAT
/// ===========
/// Every frame is one line, starts with '#', and ends with an optional
/// checksum "*HH" (XOR of every character between '#' and '*', two uppercase
/// hex digits).  Set `protocol.checksum: false` in hardware.yaml to omit it.
///
///   host -> controller
///     #C <id>:<mode><value> [<id>:<mode><value> ...]*HH
///         setpoints for all axes in one frame; mode 'P' = position [counts],
///         mode 'V' = velocity [counts/s]
///     #Q*HH                        request feedback for all axes
///     #E <0|1>*HH                  disable / enable the drives
///     #S*HH                        controlled stop, keep power
///
///   controller -> host
///     #F <id>:<counts>,<counts_per_s>,<milliamps>,<decidegC>,<fault> ...*HH
///     #A OK*HH                     acknowledge
///     #A ERR <text>*HH             refusal, with a reason
///
/// Example exchange at 100 Hz (checksums are the real ones for these frames):
///     -> #C 1:P409600 2:P-102400 3:P0 4:P0 5:P51200 6:P0*63
///     -> #Q*51
///     <- #F 1:409580,120,850,310,0 2:-102390,-45,900,305,0 3:5,0,120,300,0
///        4:0,0,90,295,0 5:51190,-8,110,298,0 6:0,0,80,294,0*55
///        (one line on the wire; wrapped here for readability)
class SimpleAsciiProtocol : public MotorProtocol
{
public:
  bool initialize(
    Transport * transport, const std::vector<JointConfig> & joints,
    const ProtocolConfig & config, std::string & error) override;
  bool enable(bool enabled, std::string & error) override;
  bool stop(std::string & error) override;
  bool write_commands(const std::vector<MotorCommand> & commands, std::string & error) override;
  bool read_feedback(std::vector<MotorFeedback> & feedback, std::string & error) override;
  std::string name() const override;

  /// Append "*HH" when checksums are enabled.  Public for the unit tests.
  std::string frame(const std::string & body) const;

  /// Verify and strip the checksum.  Returns false on a corrupt frame.
  bool unframe(const std::string & line, std::string & body, std::string & error) const;

  /// XOR checksum of `body` as two uppercase hex digits.
  static std::string checksum(const std::string & body);

private:
  bool transact(const std::string & body, std::string & reply, std::string & error);

  mutable std::mutex mutex_;
  Transport * transport_{nullptr};
  ProtocolConfig config_;
  std::vector<int> motor_ids_;
  bool initialised_{false};
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__PROTOCOL__SIMPLE_ASCII_PROTOCOL_HPP_
