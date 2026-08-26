// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__PROTOCOL__LOOPBACK_PROTOCOL_HPP_
#define ROBOT_ARM_HARDWARE__PROTOCOL__LOOPBACK_PROTOCOL_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "robot_arm_hardware/protocol/motor_protocol.hpp"

namespace robot_arm_hardware
{

/// A deterministic motor simulation that answers like a real controller.
///
/// This is NOT a second control system: it sits at the very bottom of the
/// stack, below the same hardware interface the physical robot uses, so
/// `hardware_type:=real` with `transport.type: loopback` exercises the real
/// driver - encoder conversion, watchdogs, safety, diagnostics, e-stop - with
/// no bus attached.  Extremely useful for CI and for bringing up a new
/// application before the machine exists.
///
/// The model is a rate-limited first-order tracker per joint, which is enough
/// to produce plausible positions, velocities, currents and temperatures.
class LoopbackProtocol : public MotorProtocol
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

private:
  struct SimulatedMotor
  {
    int motor_id{0};
    double counts{0.0};
    double counts_per_s{0.0};
    double target_counts{0.0};
    double target_counts_per_s{0.0};
    bool velocity_mode{false};
    double max_counts_per_s{0.0};
    double temperature{25.0};
    double current{0.0};
  };

  void integrate(double dt);

  mutable std::mutex mutex_;
  Transport * transport_{nullptr};
  ProtocolConfig config_;
  std::vector<SimulatedMotor> motors_;
  bool enabled_{false};
  std::chrono::steady_clock::time_point last_update_{};
  bool initialised_{false};
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__PROTOCOL__LOOPBACK_PROTOCOL_HPP_
