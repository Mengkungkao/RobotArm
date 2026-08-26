// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/protocol/loopback_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace robot_arm_hardware
{
namespace
{
/// Closed-loop bandwidth of the simulated drive [1/s].  Fast enough to follow
/// a trajectory closely, slow enough to look like a real servo.
constexpr double kTrackingGain = 25.0;
constexpr double kAmbientTemperature = 25.0;
constexpr double kHeatingPerAmp = 4.0;
constexpr double kCooling = 0.05;
}  // namespace

bool LoopbackProtocol::initialize(
  Transport * transport, const std::vector<JointConfig> & joints,
  const ProtocolConfig & config, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (joints.empty()) {
    error = "loopback protocol: no joints configured";
    return false;
  }

  transport_ = transport;
  config_ = config;
  motors_.clear();
  motors_.reserve(joints.size());
  for (const auto & joint : joints) {
    SimulatedMotor motor;
    motor.motor_id = joint.motor_id;
    // Start at the calibrated zero of the joint, not at raw count 0, so the
    // first feedback is a pose the arm could actually be in.
    motor.counts = static_cast<double>(joint.position_to_counts(0.0));
    motor.target_counts = motor.counts;
    motor.max_counts_per_s = std::abs(joint.velocity_to_counts(joint.max_velocity));
    motor.temperature = kAmbientTemperature;
    motors_.push_back(motor);
  }

  last_update_ = std::chrono::steady_clock::now();
  enabled_ = false;
  initialised_ = true;
  return true;
}

bool LoopbackProtocol::enable(bool enabled, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "loopback protocol is not initialised";
    return false;
  }
  enabled_ = enabled;
  if (!enabled) {
    for (auto & motor : motors_) {
      motor.counts_per_s = 0.0;
      motor.target_counts = motor.counts;
    }
  }
  return true;
}

bool LoopbackProtocol::stop(std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "loopback protocol is not initialised";
    return false;
  }
  for (auto & motor : motors_) {
    motor.target_counts = motor.counts;
    motor.target_counts_per_s = 0.0;
    motor.counts_per_s = 0.0;
    motor.velocity_mode = false;
  }
  return true;
}

bool LoopbackProtocol::write_commands(
  const std::vector<MotorCommand> & commands, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "loopback protocol is not initialised";
    return false;
  }
  if (commands.size() != motors_.size()) {
    error = "loopback protocol: expected " + std::to_string(motors_.size()) +
      " commands, got " + std::to_string(commands.size());
    return false;
  }

  for (std::size_t i = 0; i < commands.size(); ++i) {
    motors_[i].target_counts = commands[i].target_counts;
    motors_[i].target_counts_per_s = commands[i].target_counts_per_s;
    motors_[i].velocity_mode = commands[i].velocity_mode;
  }

  // Push the frame through the transport as well, so the transport layer is
  // exercised (and can inject failures) even in this mode.
  if (transport_ != nullptr && transport_->is_open()) {
    std::string ignored;
    transport_->write(Frame::from_string("#C loopback"), ignored);
  }
  return true;
}

void LoopbackProtocol::integrate(double dt)
{
  for (auto & motor : motors_) {
    if (!enabled_) {
      motor.counts_per_s = 0.0;
      motor.current = 0.0;
    } else if (motor.velocity_mode) {
      motor.counts_per_s = std::clamp(
        motor.target_counts_per_s, -motor.max_counts_per_s, motor.max_counts_per_s);
      motor.counts += motor.counts_per_s * dt;
    } else {
      const double error_counts = motor.target_counts - motor.counts;
      motor.counts_per_s = std::clamp(
        kTrackingGain * error_counts, -motor.max_counts_per_s, motor.max_counts_per_s);
      motor.counts += motor.counts_per_s * dt;
    }

    // A crude but monotone load model: current follows the commanded speed,
    // temperature integrates the current.  Enough to make the diagnostics and
    // the effort state interface meaningful.
    const double speed_ratio = motor.max_counts_per_s > 0.0 ?
      std::abs(motor.counts_per_s) / motor.max_counts_per_s : 0.0;
    motor.current = enabled_ ? 0.2 + 2.0 * speed_ratio : 0.0;
    motor.temperature += dt * (kHeatingPerAmp * motor.current * 0.01 -
      kCooling * (motor.temperature - kAmbientTemperature));
  }
}

bool LoopbackProtocol::read_feedback(std::vector<MotorFeedback> & feedback, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "loopback protocol is not initialised";
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_update_).count();
  last_update_ = now;
  // Guard against a stalled or jumping clock.
  dt = std::clamp(dt, 0.0, 0.1);
  integrate(dt);

  feedback.clear();
  feedback.reserve(motors_.size());
  for (const auto & motor : motors_) {
    MotorFeedback item;
    item.motor_id = motor.motor_id;
    item.counts = static_cast<int64_t>(std::llround(motor.counts));
    item.counts_per_s = motor.counts_per_s;
    item.current = motor.current;
    item.temperature = motor.temperature;
    item.fault_code = 0;
    item.valid = true;
    feedback.push_back(item);
  }

  // Drain whatever write_commands() echoed back, so the queue cannot grow.
  if (transport_ != nullptr && transport_->is_open()) {
    Frame frame;
    std::string ignored;
    while (transport_->read(frame, ignored)) {
    }
  }
  return true;
}

std::string LoopbackProtocol::name() const
{
  return "loopback";
}

}  // namespace robot_arm_hardware
