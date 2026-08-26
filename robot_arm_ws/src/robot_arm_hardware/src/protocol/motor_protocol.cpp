// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/protocol/motor_protocol.hpp"

#include <memory>
#include <string>
#include <vector>

#include "robot_arm_hardware/protocol/loopback_protocol.hpp"
#include "robot_arm_hardware/protocol/simple_ascii_protocol.hpp"

namespace robot_arm_hardware
{

std::vector<std::string> available_protocols()
{
  return {"loopback", "simple_ascii"};
}

MotorProtocolPtr create_protocol(const ProtocolConfig & config, std::string & error)
{
  if (config.type == "loopback") {
    return std::make_unique<LoopbackProtocol>();
  }
  if (config.type == "simple_ascii") {
    return std::make_unique<SimpleAsciiProtocol>();
  }

  error = "unknown protocol type '" + config.type + "'; available:";
  for (const auto & name : available_protocols()) {
    error += " " + name;
  }
  return nullptr;
}

}  // namespace robot_arm_hardware
