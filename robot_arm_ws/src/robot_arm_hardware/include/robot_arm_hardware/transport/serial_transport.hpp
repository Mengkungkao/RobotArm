// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__TRANSPORT__SERIAL_TRANSPORT_HPP_
#define ROBOT_ARM_HARDWARE__TRANSPORT__SERIAL_TRANSPORT_HPP_

#include <mutex>
#include <string>

#include "robot_arm_hardware/transport/transport.hpp"

namespace robot_arm_hardware
{

/// POSIX serial transport (USB-serial, UART and RS485).
///
/// RS485 half-duplex transceivers that need manual direction control are
/// handled by `rs485_rts_toggle`: RTS is asserted around the write and
/// released before listening.  Transceivers with automatic direction control
/// need no special handling - leave the flag false.
class SerialTransport : public Transport
{
public:
  explicit SerialTransport(const TransportConfig & config);
  ~SerialTransport() override;

  bool open(std::string & error) override;
  void close() override;
  bool is_open() const override;
  bool write(const Frame & frame, std::string & error) override;
  bool read(Frame & frame, std::string & error) override;
  void flush() override;
  std::string name() const override;

private:
  bool configure_port(std::string & error);
  void set_rts(bool asserted);

  TransportConfig config_;
  mutable std::mutex mutex_;
  int fd_{-1};
  std::string rx_buffer_;
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__TRANSPORT__SERIAL_TRANSPORT_HPP_
