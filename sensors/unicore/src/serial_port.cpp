// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include "mowgli_unicore_gnss/serial_port.hpp"

#include <cerrno>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace mowgli_unicore_gnss
{

SerialPort::SerialPort(std::string device, int baudrate)
    : device_(std::move(device)), baudrate_(baudrate)
{
}

SerialPort::~SerialPort()
{
  close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : device_(std::move(other.device_)), baudrate_(other.baudrate_), fd_(other.fd_)
{
  other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept
{
  if (this != &other)
  {
    close();
    device_ = std::move(other.device_);
    baudrate_ = other.baudrate_;
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void SerialPort::configure(std::string device, int baudrate)
{
  close();
  device_ = std::move(device);
  baudrate_ = baudrate;
}

bool SerialPort::open()
{
  if (fd_ >= 0)
  {
    return true;
  }

  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    return false;
  }

  struct termios tty{};

  if (::tcgetattr(fd_, &tty) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tty.c_iflag &= ~(
      static_cast<tcflag_t>(IXON) | static_cast<tcflag_t>(IXOFF) | static_cast<tcflag_t>(IXANY) |
      static_cast<tcflag_t>(ICRNL) | static_cast<tcflag_t>(INLCR) | static_cast<tcflag_t>(IGNCR) |
      static_cast<tcflag_t>(ISTRIP) | static_cast<tcflag_t>(INPCK) | static_cast<tcflag_t>(IGNBRK));
  tty.c_oflag = 0;
  tty.c_cflag &= ~(static_cast<tcflag_t>(PARENB) | static_cast<tcflag_t>(CSTOPB) |
                   static_cast<tcflag_t>(CSIZE) | static_cast<tcflag_t>(CRTSCTS));
  tty.c_cflag |=
      static_cast<tcflag_t>(CS8) | static_cast<tcflag_t>(CREAD) | static_cast<tcflag_t>(CLOCAL);
  tty.c_lflag &= ~(static_cast<tcflag_t>(ECHO) | static_cast<tcflag_t>(ECHOE) |
                   static_cast<tcflag_t>(ECHONL) | static_cast<tcflag_t>(ICANON) |
                   static_cast<tcflag_t>(ISIG) | static_cast<tcflag_t>(IEXTEN));
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const speed_t speed = static_cast<speed_t>(to_termios_baud(baudrate_));
  if (speed == B0)
  {
    ::close(fd_);
    fd_ = -1;
    errno = EINVAL;
    return false;
  }

  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  ::tcflush(fd_, TCIFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const noexcept
{
  return fd_ >= 0;
}

const std::string& SerialPort::device() const noexcept
{
  return device_;
}

int SerialPort::baudrate() const noexcept
{
  return baudrate_;
}

ssize_t SerialPort::read(uint8_t* buffer, std::size_t max_len)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  return ::read(fd_, buffer, max_len);
}

ssize_t SerialPort::write(const uint8_t* buffer, std::size_t len)
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }

  std::size_t total_written = 0U;
  while (total_written < len)
  {
    const ssize_t rc = ::write(fd_, buffer + total_written, len - total_written);
    if (rc > 0)
    {
      total_written += static_cast<std::size_t>(rc);
      continue;
    }

    if (rc < 0 && errno == EINTR)
    {
      continue;
    }

    if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      if (!wait_writable(100))
      {
        errno = EAGAIN;
        return -1;
      }
      continue;
    }

    return -1;
  }

  return static_cast<ssize_t>(total_written);
}

bool SerialPort::wait_writable(int timeout_ms) const
{
  if (fd_ < 0)
  {
    errno = EBADF;
    return false;
  }

  struct pollfd poll_fd{};
  poll_fd.fd = fd_;
  poll_fd.events = POLLOUT;

  while (true)
  {
    const int rc = ::poll(&poll_fd, 1, timeout_ms);
    if (rc > 0)
    {
      return (poll_fd.revents & POLLOUT) != 0;
    }
    if (rc == 0)
    {
      return false;
    }
    if (errno == EINTR)
    {
      continue;
    }
    return false;
  }
}

int SerialPort::to_termios_baud(int baudrate) noexcept
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B0;
  }
}

}  // namespace mowgli_unicore_gnss
