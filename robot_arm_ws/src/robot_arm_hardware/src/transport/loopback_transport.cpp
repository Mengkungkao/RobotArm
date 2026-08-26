// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/transport/loopback_transport.hpp"

#include <mutex>
#include <string>

namespace robot_arm_hardware
{

LoopbackTransport::LoopbackTransport(const TransportConfig & config)
: config_(config)
{
}

bool LoopbackTransport::open(std::string & /*error*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  open_ = true;
  queue_.clear();
  return true;
}

void LoopbackTransport::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  open_ = false;
  queue_.clear();
}

bool LoopbackTransport::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_;
}

bool LoopbackTransport::write(const Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    error = "loopback transport is not open";
    return false;
  }
  if (failures_left_ > 0) {
    --failures_left_;
    error = "injected write failure";
    return false;
  }
  queue_.push_back(frame);
  return true;
}

bool LoopbackTransport::read(Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    error = "loopback transport is not open";
    return false;
  }
  if (failures_left_ > 0) {
    --failures_left_;
    error = "injected read failure";
    return false;
  }
  if (queue_.empty()) {
    error = "loopback transport: nothing to read (timeout)";
    return false;
  }
  frame = queue_.front();
  queue_.pop_front();
  return true;
}

void LoopbackTransport::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

std::string LoopbackTransport::name() const
{
  return "loopback";
}

void LoopbackTransport::inject(const Frame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push_back(frame);
}

void LoopbackTransport::fail_next(std::size_t count)
{
  std::lock_guard<std::mutex> lock(mutex_);
  failures_left_ = count;
}

}  // namespace robot_arm_hardware
