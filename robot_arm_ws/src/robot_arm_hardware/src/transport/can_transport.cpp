// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/transport/can_transport.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#if defined(__linux__)
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#define ROBOT_ARM_HAS_SOCKETCAN 1
#else
#define ROBOT_ARM_HAS_SOCKETCAN 0
#endif

namespace robot_arm_hardware
{

CanTransport::CanTransport(const TransportConfig & config)
: config_(config)
{
}

CanTransport::~CanTransport()
{
  close();
}

#if ROBOT_ARM_HAS_SOCKETCAN

bool CanTransport::open(std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ >= 0) {
    return true;
  }

  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    error = std::string("cannot create CAN socket: ") + std::strerror(errno);
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, config_.can_interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    error = "CAN interface '" + config_.can_interface + "' not found: " + std::strerror(errno);
    ::close(socket_);
    socket_ = -1;
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    error = "cannot bind CAN socket to " + config_.can_interface + ": " + std::strerror(errno);
    ::close(socket_);
    socket_ = -1;
    return false;
  }
  return true;
}

void CanTransport::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool CanTransport::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return socket_ >= 0;
}

bool CanTransport::write(const Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) {
    error = "CAN socket is not open";
    return false;
  }
  if (frame.data.size() > CAN_MAX_DLEN) {
    error = "CAN payload of " + std::to_string(frame.data.size()) + " bytes exceeds 8";
    return false;
  }

  struct can_frame raw {};
  raw.can_id = frame.id;
  raw.can_dlc = static_cast<__u8>(frame.data.size());
  std::memcpy(raw.data, frame.data.data(), frame.data.size());

  struct pollfd pfd {socket_, POLLOUT, 0};
  if (::poll(&pfd, 1, config_.write_timeout_ms) <= 0) {
    error = "CAN write timeout after " + std::to_string(config_.write_timeout_ms) + " ms";
    return false;
  }
  if (::write(socket_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    error = std::string("CAN write failed: ") + std::strerror(errno);
    return false;
  }
  return true;
}

bool CanTransport::read(Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) {
    error = "CAN socket is not open";
    return false;
  }

  struct pollfd pfd {socket_, POLLIN, 0};
  const int ready = ::poll(&pfd, 1, config_.read_timeout_ms);
  if (ready < 0) {
    error = std::string("CAN poll failed: ") + std::strerror(errno);
    return false;
  }
  if (ready == 0) {
    error = "CAN read timeout after " + std::to_string(config_.read_timeout_ms) + " ms";
    return false;
  }

  struct can_frame raw {};
  const ssize_t count = ::read(socket_, &raw, sizeof(raw));
  if (count != static_cast<ssize_t>(sizeof(raw))) {
    error = std::string("CAN read failed: ") + std::strerror(errno);
    return false;
  }

  frame.id = raw.can_id & CAN_EFF_MASK;
  frame.data.assign(raw.data, raw.data + raw.can_dlc);
  return true;
}

void CanTransport::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) {
    return;
  }
  // Drain whatever is queued so a resynchronisation starts from a clean bus.
  struct can_frame raw {};
  struct pollfd pfd {socket_, POLLIN, 0};
  while (::poll(&pfd, 1, 0) > 0) {
    if (::read(socket_, &raw, sizeof(raw)) <= 0) {
      break;
    }
  }
}

#else   // not Linux: SocketCAN is unavailable, fail loudly instead of silently

bool CanTransport::open(std::string & error)
{
  error = "SocketCAN is only available on Linux";
  return false;
}
void CanTransport::close() {}
bool CanTransport::is_open() const {return false;}
bool CanTransport::write(const Frame &, std::string & error)
{
  error = "SocketCAN is only available on Linux";
  return false;
}
bool CanTransport::read(Frame &, std::string & error)
{
  error = "SocketCAN is only available on Linux";
  return false;
}
void CanTransport::flush() {}

#endif

std::string CanTransport::name() const
{
  char base_id[16];
  std::snprintf(base_id, sizeof(base_id), "0x%X", config_.can_base_id);
  return "can(" + config_.can_interface + ", base_id=" + std::string(base_id) + ")";
}

}  // namespace robot_arm_hardware
