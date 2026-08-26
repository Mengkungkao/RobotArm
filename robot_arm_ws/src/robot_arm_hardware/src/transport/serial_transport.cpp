// Copyright (c) 2026 robot_arm_ws contributors
// SPDX-License-Identifier: MIT
#include "robot_arm_hardware/transport/serial_transport.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
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

/// Map a numeric baud rate to the termios constant.  Returns B0 when the rate
/// is not supported, which the caller reports as a configuration error rather
/// than silently running at the wrong speed.
speed_t to_speed(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return B0;
  }
}

int64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

SerialTransport::SerialTransport(const TransportConfig & config)
: config_(config)
{
}

SerialTransport::~SerialTransport()
{
  close();
}

bool SerialTransport::open(std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(config_.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = "cannot open " + config_.serial_port + ": " + std::strerror(errno);
    return false;
  }
  if (!configure_port(error)) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  rx_buffer_.clear();
  return true;
}

bool SerialTransport::configure_port(std::string & error)
{
  struct termios tty {};
  if (::tcgetattr(fd_, &tty) != 0) {
    error = "tcgetattr failed on " + config_.serial_port + ": " + std::strerror(errno);
    return false;
  }

  const speed_t speed = to_speed(config_.baudrate);
  if (speed == B0) {
    error = "unsupported baudrate " + std::to_string(config_.baudrate);
    return false;
  }
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);

  // 8N1 by default, raw mode, no flow control, local receiver enabled.
  tty.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
  switch (config_.data_bits) {
    case 7: tty.c_cflag |= CS7; break;
    case 8: tty.c_cflag |= CS8; break;
    default:
      error = "unsupported data_bits " + std::to_string(config_.data_bits);
      return false;
  }

  if (config_.parity == "none") {
    tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);
  } else if (config_.parity == "even") {
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~static_cast<tcflag_t>(PARODD);
  } else if (config_.parity == "odd") {
    tty.c_cflag |= PARENB | PARODD;
  } else {
    error = "unsupported parity '" + config_.parity + "' (none|even|odd)";
    return false;
  }

  if (config_.stop_bits == 2) {
    tty.c_cflag |= CSTOPB;
  } else {
    tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
  }

  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);

  tty.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  tty.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
  tty.c_oflag &= ~static_cast<tcflag_t>(OPOST | ONLCR);

  // Non-blocking reads; timing is handled with poll() so the control loop
  // never blocks for longer than read_timeout_ms.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error = "tcsetattr failed on " + config_.serial_port + ": " + std::strerror(errno);
    return false;
  }
  ::tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialTransport::set_rts(bool asserted)
{
  if (fd_ < 0) {
    return;
  }
  int status = 0;
  if (::ioctl(fd_, TIOCMGET, &status) != 0) {
    return;
  }
  if (asserted) {
    status |= TIOCM_RTS;
  } else {
    status &= ~TIOCM_RTS;
  }
  ::ioctl(fd_, TIOCMSET, &status);
}

void SerialTransport::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rx_buffer_.clear();
}

bool SerialTransport::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

bool SerialTransport::write(const Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    error = "serial port is not open";
    return false;
  }

  std::string payload(frame.data.begin(), frame.data.end());
  if (payload.empty() || payload.back() != config_.terminator) {
    payload.push_back(config_.terminator);
  }

  if (config_.rs485_rts_toggle) {
    set_rts(true);           // drive the RS485 bus
  }

  const int64_t deadline = now_ms() + config_.write_timeout_ms;
  std::size_t written = 0;
  bool ok = true;
  while (written < payload.size()) {
    const ssize_t count = ::write(fd_, payload.data() + written, payload.size() - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (now_ms() > deadline) {
        error = "serial write timeout after " + std::to_string(config_.write_timeout_ms) + " ms";
        ok = false;
        break;
      }
      struct pollfd pfd {fd_, POLLOUT, 0};
      ::poll(&pfd, 1, 1);
      continue;
    }
    error = std::string("serial write failed: ") + std::strerror(errno);
    ok = false;
    break;
  }

  if (ok) {
    // Wait for the UART FIFO to drain before releasing the bus, otherwise the
    // tail of the frame is cut off on half-duplex RS485.
    ::tcdrain(fd_);
  }
  if (config_.rs485_rts_toggle) {
    set_rts(false);          // release the bus so the drive can answer
  }
  return ok;
}

bool SerialTransport::read(Frame & frame, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    error = "serial port is not open";
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
      error = "serial read timeout after " + std::to_string(config_.read_timeout_ms) + " ms";
      return false;
    }

    struct pollfd pfd {fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      error = std::string("serial poll failed: ") + std::strerror(errno);
      return false;
    }
    if (ready == 0) {
      continue;   // let the deadline check above decide
    }

    char buffer[256];
    const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
    if (count > 0) {
      rx_buffer_.append(buffer, static_cast<std::size_t>(count));
      // A runaway peer must not grow the buffer without bound.
      if (rx_buffer_.size() > 8192) {
        rx_buffer_.clear();
        error = "serial receive buffer overflow, resynchronising";
        return false;
      }
    } else if (count == 0) {
      error = "serial port closed by peer";
      return false;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
      error = std::string("serial read failed: ") + std::strerror(errno);
      return false;
    }
  }
}

void SerialTransport::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  rx_buffer_.clear();
  if (fd_ >= 0) {
    ::tcflush(fd_, TCIOFLUSH);
  }
}

std::string SerialTransport::name() const
{
  return (config_.rs485_rts_toggle ? "rs485(" : "serial(") + config_.serial_port + "@" +
         std::to_string(config_.baudrate) + ")";
}

}  // namespace robot_arm_hardware
