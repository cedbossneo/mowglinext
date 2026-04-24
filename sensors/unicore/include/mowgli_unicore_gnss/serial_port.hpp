// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <sys/types.h>

namespace mowgli_unicore_gnss
{

class SerialPort
{
public:
  explicit SerialPort(std::string device = "", int baudrate = 115200);
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  SerialPort(SerialPort&& other) noexcept;
  SerialPort& operator=(SerialPort&& other) noexcept;

  bool open();
  void close();

  [[nodiscard]] bool is_open() const noexcept;
  [[nodiscard]] const std::string& device() const noexcept;
  [[nodiscard]] int baudrate() const noexcept;

  void configure(std::string device, int baudrate);
  ssize_t read(uint8_t* buffer, std::size_t max_len);

private:
  [[nodiscard]] static int to_termios_baud(int baudrate) noexcept;

  std::string device_;
  int baudrate_;
  int fd_{-1};
};

}  // namespace mowgli_unicore_gnss
