// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__TRANSPORT__TRANSPORT_HPP_
#define ROBOT_ARM_HARDWARE__TRANSPORT__TRANSPORT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robot_arm_hardware
{

/// One unit of communication.
///
/// `id` is the CAN identifier; stream transports (serial, RS485, TCP) ignore
/// it and frame the payload with a terminator instead.  Modelling both in one
/// struct is what lets a protocol run unchanged over a bus or over a wire.
struct Frame
{
  uint32_t id{0};
  std::vector<uint8_t> data;

  std::string as_string() const {return std::string(data.begin(), data.end());}

  static Frame from_string(const std::string & text, uint32_t frame_id = 0)
  {
    Frame frame;
    frame.id = frame_id;
    frame.data.assign(text.begin(), text.end());
    return frame;
  }
};

/// Everything the transports need, parsed from hardware.yaml.
struct TransportConfig
{
  std::string type{"loopback"};       ///< loopback | serial | rs485 | can | tcp

  // serial / rs485
  std::string serial_port{"/dev/ttyUSB0"};
  int baudrate{921600};
  std::string parity{"none"};         ///< none | even | odd
  int data_bits{8};
  int stop_bits{1};
  bool rs485_rts_toggle{false};

  // can
  std::string can_interface{"can0"};
  int can_base_id{0x100};

  // tcp
  std::string tcp_host{"127.0.0.1"};
  int tcp_port{5000};

  int read_timeout_ms{8};
  int write_timeout_ms{8};

  /// Terminator used by the stream transports to delimit a frame.
  char terminator{'\n'};
};

/// Abstract byte pipe to the motor controller.
///
/// This is the ONLY layer that knows about file descriptors, sockets or
/// termios.  Adding CANopen, Modbus or EtherCAT means adding a class here (and
/// possibly a protocol), and changes nothing in the hardware interface, in
/// ros2_control, or in MoveIt.
///
/// Implementations must be safe to call from one thread at a time; the
/// hardware interface serialises access, and each implementation additionally
/// guards its own file descriptor.
class Transport
{
public:
  virtual ~Transport() = default;

  /// Open the underlying device.  Returns false and fills `error` on failure.
  virtual bool open(std::string & error) = 0;

  virtual void close() = 0;

  virtual bool is_open() const = 0;

  /// Send one frame.  Blocks at most write_timeout_ms.
  virtual bool write(const Frame & frame, std::string & error) = 0;

  /// Receive one frame.  Blocks at most read_timeout_ms; a timeout is a
  /// failure (returns false) so the caller's watchdog can react.
  virtual bool read(Frame & frame, std::string & error) = 0;

  /// Drop anything buffered - used after an error to resynchronise.
  virtual void flush() = 0;

  /// Human readable name, e.g. "serial(/dev/ttyUSB0@921600)".
  virtual std::string name() const = 0;
};

using TransportPtr = std::unique_ptr<Transport>;

/// Factory.  Returns nullptr and fills `error` for an unknown type.
TransportPtr create_transport(const TransportConfig & config, std::string & error);

/// Names accepted by create_transport, for error messages and tests.
std::vector<std::string> available_transports();

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__TRANSPORT__TRANSPORT_HPP_
