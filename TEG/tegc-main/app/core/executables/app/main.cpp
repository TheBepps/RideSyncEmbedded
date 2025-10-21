#include "main.hpp"

#include <cctype>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

#include "imgui.h"
#include "modals/serial_port_modal.hpp"
#include "serial/serial.hpp"
#include "tabs/tab_serial_port_manager.hpp"
#include "window/window.hpp"

int main(int argc, char **argv) {
  GLFWwindow *window = OpenWindow();

  SerialPort stm;
  TabSerialPortManager tabSerialPortManager;

  std::atomic<bool> killThread{false};
  std::mutex mtxForLine;

  std::string line;
  std::thread readerThread([&]() {
    std::string tmpLine;
    while (!killThread.load()) {
      if (!stm.IsOpen()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if (readNextChar(stm, tmpLine)) {
        line = tmpLine;
        tmpLine.clear();
        std::unique_lock<std::mutex> lck(mtxForLine);
        try {
          tabSerialPortManager.parseLine(line);
        } catch (std::exception &e) {
          printf("got exception: %s\n", e.what());
        }
      }
    }
  });

  if (argc > 1) {
    std::string line;
    std::ifstream csv(argv[1]);
    int lineCount = 0;
    if (csv.is_open()) {
      while (std::getline(csv, line)) {
        if (lineCount == 0) {
          line = "csv_header:" + line;
        } else {
          line = "csv_data:" + line;
        }
        tabSerialPortManager.parseLine(line);
        lineCount++;
      }
      csv.close();
    }
    tabSerialPortManager.parseLine(
        ">temp <cold_side> <hot_side>\n"
        ">pid <kp> <ki> <kd>\n"
        ">duty <duty cycle>\n"
        "clear\n"
        "pid\n"
        "setpoint\n");
    tabSerialPortManager.parseLine(
        "\n\n"
        "pid kp 0.25\n"
        "pid ki 0.01\n");
  }

  while (!glfwWindowShouldClose(window)) {
    NewFrame();

    Dockspace();

    if (argc == 1) {
      SerialPortPopup(stm);
    }

    // ImGui::ShowDemoWindow();

    {
      std::unique_lock<std::mutex> lck(mtxForLine);
      tabSerialPortManager.draw();
    }
    if (tabSerialPortManager.needToSendCommand()) {
      stm.Send(tabSerialPortManager.toSendCommand());
    }

    Render(window, ImVec4(0.14, 0.14, 0.14, 0.0));
    line.clear();
  }
  killThread = true;
  readerThread.join();

  return 0;
}

bool readNextChar(SerialPort &port, std::string &prevBuffer) {
  if (!port.IsOpen()) {
    return false;
  }
  char c;
  int bytesRead = port.ReadBytes((uint8_t *)&c, 1);
  if (bytesRead <= 0) {
    printf("Error reading from serial port\n");
    printf("%d\n", bytesRead);
    printf("errno: %d\n", errno);
    port.Close();
  }
  if (c == '\r') {
    return false;
  }

  for (char newLineChar : "\n") {
    if (c == newLineChar) return true;
  }

  prevBuffer.push_back(c);

  return false;
}
