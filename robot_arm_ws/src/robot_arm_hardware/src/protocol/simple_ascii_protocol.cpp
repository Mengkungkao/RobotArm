// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/protocol/simple_ascii_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace robot_arm_hardware
{
namespace
{

/// Split "a,b,c" into its fields.
std::vector<std::string> split(const std::string & text, char separator)
{
  std::vector<std::string> parts;
  std::string item;
  std::istringstream stream(text);
  while (std::getline(stream, item, separator)) {
    parts.push_back(item);
  }
  return parts;
}

}  // namespace

std::string SimpleAsciiProtocol::checksum(const std::string & body)
{
  unsigned char value = 0;
  for (const char character : body) {
    value = static_cast<unsigned char>(value ^ static_cast<unsigned char>(character));
  }
  char buffer[3];
  std::snprintf(buffer, sizeof(buffer), "%02X", value);
  return std::string(buffer);
}

std::string SimpleAsciiProtocol::frame(const std::string & body) const
{
  if (!config_.checksum) {
    return "#" + body;
  }
  return "#" + body + "*" + checksum(body);
}

bool SimpleAsciiProtocol::unframe(
  const std::string & line, std::string & body, std::string & error) const
{
  if (line.empty() || line.front() != '#') {
    error = "malformed frame (no '#'): '" + line + "'";
    return false;
  }

  const auto star = line.find('*');
  if (star == std::string::npos) {
    if (config_.checksum) {
      error = "frame without checksum while checksums are enabled: '" + line + "'";
      return false;
    }
    body = line.substr(1);
    return true;
  }

  body = line.substr(1, star - 1);
  const std::string received = line.substr(star + 1, 2);
  const std::string expected = checksum(body);
  if (received != expected) {
    error = "checksum mismatch (got " + received + ", expected " + expected + ") in '" +
      line + "'";
    return false;
  }
  return true;
}

bool SimpleAsciiProtocol::initialize(
  Transport * transport, const std::vector<JointConfig> & joints,
  const ProtocolConfig & config, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (transport == nullptr) {
    error = "simple_ascii protocol needs a transport";
    return false;
  }
  if (joints.empty()) {
    error = "simple_ascii protocol: no joints configured";
    return false;
  }

  transport_ = transport;
  config_ = config;
  motor_ids_.clear();
  for (const auto & joint : joints) {
    motor_ids_.push_back(joint.motor_id);
  }
  initialised_ = true;
  return true;
}

bool SimpleAsciiProtocol::transact(
  const std::string & body, std::string & reply, std::string & error)
{
  // Caller holds mutex_.
  if (!transport_->write(Frame::from_string(frame(body)), error)) {
    return false;
  }

  Frame response;
  if (!transport_->read(response, error)) {
    return false;
  }
  return unframe(response.as_string(), reply, error);
}

bool SimpleAsciiProtocol::enable(bool enabled, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "simple_ascii protocol is not initialised";
    return false;
  }

  std::string reply;
  if (!transact(std::string("E ") + (enabled ? "1" : "0"), reply, error)) {
    return false;
  }
  if (reply.rfind("A OK", 0) != 0) {
    error = "drive refused the enable command: '" + reply + "'";
    return false;
  }
  return true;
}

bool SimpleAsciiProtocol::stop(std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "simple_ascii protocol is not initialised";
    return false;
  }

  std::string reply;
  if (!transact("S", reply, error)) {
    return false;
  }
  if (reply.rfind("A OK", 0) != 0) {
    error = "drive refused the stop command: '" + reply + "'";
    return false;
  }
  return true;
}

bool SimpleAsciiProtocol::write_commands(
  const std::vector<MotorCommand> & commands, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "simple_ascii protocol is not initialised";
    return false;
  }

  std::ostringstream body;
  body << "C";
  for (const auto & command : commands) {
    const double value = command.velocity_mode ?
      command.target_counts_per_s : command.target_counts;
    if (!std::isfinite(value)) {
      error = "non-finite setpoint for motor " + std::to_string(command.motor_id);
      return false;
    }
    body << ' ' << command.motor_id << ':' << (command.velocity_mode ? 'V' : 'P')
         << static_cast<int64_t>(std::llround(value));
  }

  // Setpoints are fire-and-forget: the feedback poll in read_feedback() is the
  // acknowledgement, which keeps one round trip per control cycle.
  return transport_->write(Frame::from_string(frame(body.str())), error);
}

bool SimpleAsciiProtocol::read_feedback(std::vector<MotorFeedback> & feedback, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_) {
    error = "simple_ascii protocol is not initialised";
    return false;
  }

  std::string reply;
  if (!transact("Q", reply, error)) {
    return false;
  }
  if (reply.rfind("F ", 0) != 0) {
    error = "unexpected reply to a feedback request: '" + reply + "'";
    return false;
  }

  // Start from "invalid" for every configured motor: an axis missing from the
  // reply must be reported as missing, never as a stale value.
  feedback.assign(motor_ids_.size(), MotorFeedback{});
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    feedback[i].motor_id = motor_ids_[i];
    feedback[i].valid = false;
  }

  std::istringstream stream(reply.substr(2));
  std::string token;
  while (stream >> token) {
    const auto colon = token.find(':');
    if (colon == std::string::npos) {
      error = "malformed feedback token '" + token + "'";
      return false;
    }

    int motor_id = 0;
    try {
      motor_id = std::stoi(token.substr(0, colon));
    } catch (const std::exception &) {
      error = "malformed motor id in '" + token + "'";
      return false;
    }

    const auto position = std::find(motor_ids_.begin(), motor_ids_.end(), motor_id);
    if (position == motor_ids_.end()) {
      continue;   // a drive we do not control shares the bus: ignore it
    }
    const auto index = static_cast<std::size_t>(std::distance(motor_ids_.begin(), position));

    const auto fields = split(token.substr(colon + 1), ',');
    if (fields.size() < 5) {
      error = "feedback for motor " + std::to_string(motor_id) + " has " +
        std::to_string(fields.size()) + " fields, expected 5";
      return false;
    }

    try {
      feedback[index].counts = std::stoll(fields[0]);
      feedback[index].counts_per_s = std::stod(fields[1]);
      feedback[index].current = std::stod(fields[2]) / 1000.0;      // mA -> A
      feedback[index].temperature = std::stod(fields[3]) / 10.0;    // 0.1 degC -> degC
      feedback[index].fault_code = static_cast<uint8_t>(std::stoi(fields[4]));
      feedback[index].valid = true;
    } catch (const std::exception &) {
      error = "non-numeric feedback field for motor " + std::to_string(motor_id);
      return false;
    }
  }

  return true;
}

std::string SimpleAsciiProtocol::name() const
{
  return config_.checksum ? "simple_ascii(checksum)" : "simple_ascii";
}

}  // namespace robot_arm_hardware
