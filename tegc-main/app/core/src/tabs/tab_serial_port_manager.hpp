#pragma once

#include <fstream>
#include <vector>

#include "serial/serial.hpp"
#include "tabs/tab.hpp"

class TabSerialPortManager : public TabBase {
 public:
  TabSerialPortManager() : TabBase("Tab Serial Port Manager") {};
  void parseLine(const std::string &line);
  void drawImpl() override;

  bool needToSendCommand() const { return !m_toSendCommand.empty(); }
  std::string toSendCommand() {
    std::string ret = m_toSendCommand;
    m_toSendCommand.clear();
    return ret;
  };

 private:
  std::vector<std::string> csvHeader;
  std::vector<std::vector<std::string>> csvRowsString;
  std::string notParsedText;

  std::string m_toSendCommand = "";

  std::ofstream csvOutFile;
};
