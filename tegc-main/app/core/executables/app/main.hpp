#pragma once

#include "serial/serial.hpp"

bool readNextChar(SerialPort &port,
                  std::string &prevBuffer);  // return true when new line
