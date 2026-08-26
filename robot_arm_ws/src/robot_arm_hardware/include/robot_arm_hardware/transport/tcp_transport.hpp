// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__TRANSPORT__TCP_TRANSPORT_HPP_
#define ROBOT_ARM_HARDWARE__TRANSPORT__TCP_TRANSPORT_HPP_

#include <mutex>
#include <string>

#include "robot_arm_hardware/transport/transport.hpp"

namespace robot_arm_hardware
{

/// Raw TCP transport for Ethernet based motor controllers.
///
/// Nagle's algorithm is disabled: a control loop sends small frames at a fixed
/// rate and cannot afford the coalescing delay.
class TcpTransport : public Transport
{
public:
  explicit TcpTransport(const TransportConfig & config);
  ~TcpTransport() override;

  bool open(std::string & error) override;
  void close() override;
  bool is_open() const override;
  bool write(const Frame & frame, std::string & error) override;
  bool read(Frame & frame, std::string & error) override;
  void flush() override;
  std::string name() const override;

private:
  TransportConfig config_;
  mutable std::mutex mutex_;
  int socket_{-1};
  std::string rx_buffer_;
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__TRANSPORT__TCP_TRANSPORT_HPP_
