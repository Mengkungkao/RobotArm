// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/transport/transport.hpp"

#include <memory>
#include <string>
#include <vector>

#include "robot_arm_hardware/transport/can_transport.hpp"
#include "robot_arm_hardware/transport/loopback_transport.hpp"
#include "robot_arm_hardware/transport/serial_transport.hpp"
#include "robot_arm_hardware/transport/tcp_transport.hpp"

namespace robot_arm_hardware
{

std::vector<std::string> available_transports()
{
  return {"loopback", "serial", "rs485", "can", "tcp"};
}

TransportPtr create_transport(const TransportConfig & config, std::string & error)
{
  if (config.type == "loopback") {
    return std::make_unique<LoopbackTransport>(config);
  }
  if (config.type == "serial" || config.type == "rs485") {
    // RS485 is a serial port with optional direction control; the flag lives
    // in the config, so no separate class is needed.
    TransportConfig serial_config = config;
    if (config.type == "rs485") {
      serial_config.rs485_rts_toggle = config.rs485_rts_toggle;
    }
    return std::make_unique<SerialTransport>(serial_config);
  }
  if (config.type == "can") {
    return std::make_unique<CanTransport>(config);
  }
  if (config.type == "tcp") {
    return std::make_unique<TcpTransport>(config);
  }

  error = "unknown transport type '" + config.type + "'; available:";
  for (const auto & name : available_transports()) {
    error += " " + name;
  }
  return nullptr;
}

}  // namespace robot_arm_hardware
