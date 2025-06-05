#include "serial_port_modal.hpp"

#include "errors/error.hpp"
#include "imgui.h"
#include "imgui_stdlib.h"
#include "utils/utils.hpp"

static std::string address = "/dev/ttyACM0";
static double openTimestamp;
static std::string errorText;

const char *baudratesStr[] = {"9600", "115200", "1000000", "2000000"};
const speed_t baudrates[] = {B9600, B115200, B1000000, B2000000};

void SerialPortPopup(SerialPort &serialPort) {
  if (!serialPort.IsOpen()) {
    ImGui::OpenPopup("Serial Port Popup");
  }
  if (ImGui::BeginPopupModal("Serial Port Popup")) {
    ImGui::InputTextWithHint("Serial port address", "path to port", &address);
    static int baudSelected = 2;
    ImGui::Combo("Baudrate", &baudSelected, baudratesStr,
                 IM_ARRAYSIZE(baudratesStr));
    if (ImGui::Button("Open")) {
      err::Error openError = serialPort.Open(address, baudrates[baudSelected]);
      if (openError) {
        errorText = err::errorStr(openError.get());
      } else {
        errorText.clear();
        openTimestamp = getTimestampSeconds();
      }
    }
    if (!errorText.empty()) {
      ImGui::SameLine();
      ImGui::Text("%s", errorText.c_str());
    }
    if (serialPort.IsOpen()) {
      ImGui::Text("Opened");
      if (getTimestampSeconds() - openTimestamp > 0.5) {
        ImGui::CloseCurrentPopup();
      }
    }
    ImGui::EndPopup();
  }
}
