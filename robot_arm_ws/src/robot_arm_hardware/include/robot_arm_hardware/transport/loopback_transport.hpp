// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#ifndef ROBOT_ARM_HARDWARE__TRANSPORT__LOOPBACK_TRANSPORT_HPP_
#define ROBOT_ARM_HARDWARE__TRANSPORT__LOOPBACK_TRANSPORT_HPP_

#include <deque>
#include <mutex>
#include <string>

#include "robot_arm_hardware/transport/transport.hpp"

namespace robot_arm_hardware
{

/// In-process transport: whatever is written can be read back.
///
/// It exists so the *real* driver code path - framing, checksums, timeouts,
/// watchdogs, encoder conversion - can be exercised in unit tests and on a
/// developer machine with no bus attached.  It is also the default in
/// hardware.yaml, so a fresh clone cannot accidentally drive a machine.
class LoopbackTransport : public Transport
{
public:
  explicit LoopbackTransport(const TransportConfig & config);

  bool open(std::string & error) override;
  void close() override;
  bool is_open() const override;
  bool write(const Frame & frame, std::string & error) override;
  bool read(Frame & frame, std::string & error) override;
  void flush() override;
  std::string name() const override;

  /// Test hook: queue a frame to be returned by the next read().
  void inject(const Frame & frame);

  /// Test hook: make the next `count` operations fail, to exercise the
  /// error/watchdog paths.
  void fail_next(std::size_t count);

private:
  TransportConfig config_;
  mutable std::mutex mutex_;
  std::deque<Frame> queue_;
  bool open_{false};
  std::size_t failures_left_{0};
};

}  // namespace robot_arm_hardware

#endif  // ROBOT_ARM_HARDWARE__TRANSPORT__LOOPBACK_TRANSPORT_HPP_
