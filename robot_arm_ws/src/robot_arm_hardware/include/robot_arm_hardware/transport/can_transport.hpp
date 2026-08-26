// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__TRANSPORT__CAN_TRANSPORT_HPP_
#define ROBOT_ARM_HARDWARE__TRANSPORT__CAN_TRANSPORT_HPP_

#include <mutex>
#include <string>

#include "robot_arm_hardware/transport/transport.hpp"

namespace robot_arm_hardware
{

/// SocketCAN transport (`can0`, `vcan0`, ...).
///
/// Frames carry an id and up to 8 payload bytes, which is the natural unit for
/// a CAN motor controller.  A CANopen or a vendor specific stack can be built
/// on top of this class as another MotorProtocol - the bus access stays here.
///
/// Test the whole stack without hardware with a virtual bus:
///   sudo modprobe vcan
///   sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
class CanTransport : public Transport
{
public:
  explicit CanTransport(const TransportConfig & config);
  ~CanTransport() override;

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
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__TRANSPORT__CAN_TRANSPORT_HPP_
