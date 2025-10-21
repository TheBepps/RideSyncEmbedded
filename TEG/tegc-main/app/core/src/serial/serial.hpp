#pragma once

#include <termios.h>

#include <string>

#include "errors/error.hpp"

class SerialPort {
 public:
  SerialPort() = default;

  err::Error Open(const std::string &port, speed_t speed);
  void Close();

  bool IsOpen() const;

  int ReadBytes(uint8_t *bytes, size_t nBytes);
  void Send(const std::string &data);

 private:
  int m_fd = 0;
  bool m_open = false;
};
