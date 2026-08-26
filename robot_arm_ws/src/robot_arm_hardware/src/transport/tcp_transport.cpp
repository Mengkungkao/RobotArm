// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/transport/tcp_transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>

namespace robot_arm_hardware
{
namespace
{
int64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
constexpr int kConnectTimeoutMs = 2000;
}  // namespace

TcpTransport::TcpTransport(const TransportConfig & config)
: config_(config)
{
}

TcpTransport::~TcpTransport()
{
  close();
}

bool TcpTransport::open(std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ >= 0) {
    return true;
  }

  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * results = nullptr;
  const std::string port = std::to_string(config_.tcp_port);
  const int status = ::getaddrinfo(config_.tcp_host.c_str(), port.c_str(), &hints, &results);
  if (status != 0) {
    error = "cannot resolve " + config_.tcp_host + ": " + ::gai_strerror(status);
    return false;
  }

  for (struct addrinfo * it = results; it != nullptr; it = it->ai_next) {
    const int fd = ::socket(it->ai_family, it->ai_socktype | SOCK_NONBLOCK, it->ai_protocol);
    if (fd < 0) {
      continue;
    }
    int connected = ::connect(fd, it->ai_addr, it->ai_addrlen);
    if (connected < 0 && errno == EINPROGRESS) {
      struct pollfd pfd {fd, POLLOUT, 0};
      if (::poll(&pfd, 1, kConnectTimeoutMs) > 0) {
        int sockerr = 0;
        socklen_t length = sizeof(sockerr);
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &sockerr, &length);
        connected = (sockerr == 0) ? 0 : -1;
      }
    }
    if (connected == 0) {
      socket_ = fd;
      break;
    }
    ::close(fd);
  }
  ::freeaddrinfo(results);

  if (socket_ < 0) {
    error = "cannot connect to " + config_.tcp_host + ":" + port + ": " + std::strerror(errno);
    return false;
  }

  // A control loop sends small frames at a fixed rate: coalescing them would
  // add tens of milliseconds of latency.
  int flag = 1;
  ::setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
  rx_buffer_.clear();
  return true;
}

void TcpTransport::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ >= 0) {
    ::shutdown(socket_, SHUT_RDWR);
    ::close(socket_);
    socket_ = -1;
  }
  rx_buffer_.clear();
}

bool TcpTransport::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return socket_ >= 0;
}

bool TcpTransport::write(const Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) {
    error = "TCP socket is not open";
    return false;
  }

  std::string payload(frame.data.begin(), frame.data.end());
  if (payload.empty() || payload.back() != config_.terminator) {
    payload.push_back(config_.terminator);
  }

  const int64_t deadline = now_ms() + config_.write_timeout_ms;
  std::size_t sent = 0;
  while (sent < payload.size()) {
    const ssize_t count = ::send(socket_, payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
    if (count > 0) {
      sent += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (now_ms() > deadline) {
        error = "TCP write timeout after " + std::to_string(config_.write_timeout_ms) + " ms";
        return false;
      }
      struct pollfd pfd {socket_, POLLOUT, 0};
      ::poll(&pfd, 1, 1);
      continue;
    }
    error = std::string("TCP write failed: ") + std::strerror(errno);
    return false;
  }
  return true;
}

bool TcpTransport::read(Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) {
    error = "TCP socket is not open";
    return false;
  }

  const int64_t deadline = now_ms() + config_.read_timeout_ms;
  while (true) {
    const auto position = rx_buffer_.find(config_.terminator);
    if (position != std::string::npos) {
      frame = Frame::from_string(rx_buffer_.substr(0, position));
      rx_buffer_.erase(0, position + 1);
      return true;
    }

    const int64_t remaining = deadline - now_ms();
    if (remaining <= 0) {
      error = "TCP read timeout after " + std::to_string(config_.read_timeout_ms) + " ms";
      return false;
    }

    struct pollfd pfd {socket_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      error = std::string("TCP poll failed: ") + std::strerror(errno);
      return false;
    }
    if (ready == 0) {
      continue;
    }

    char buffer[512];
    const ssize_t count = ::recv(socket_, buffer, sizeof(buffer), 0);
    if (count > 0) {
      rx_buffer_.append(buffer, static_cast<std::size_t>(count));
      if (rx_buffer_.size() > 16384) {
        rx_buffer_.clear();
        error = "TCP receive buffer overflow, resynchronising";
        return false;
      }
    } else if (count == 0) {
      error = "connection closed by " + config_.tcp_host;
      return false;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
      error = std::string("TCP read failed: ") + std::strerror(errno);
      return false;
    }
  }
}

void TcpTransport::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  rx_buffer_.clear();
}

std::string TcpTransport::name() const
{
  return "tcp(" + config_.tcp_host + ":" + std::to_string(config_.tcp_port) + ")";
}

}  // namespace robot_arm_hardware
