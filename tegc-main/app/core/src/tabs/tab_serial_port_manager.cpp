#include "tab_serial_port_manager.hpp"

#include <cctype>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_stdlib.h"
#include "implot.h"
#include "utils/shared_data.hpp"
#include "utils/utils.hpp"

#define CSV_DELIM ','

const char *commands[] = {">setpoint", ">pid kp",  ">pid ki",   ">pid kd",
                          ">pid",      ">duty",    "clear",     "reset",
                          "pid",       "setpoint", "csv_header"};
const char *commandsExtraParams[] = {"<temperature>",
                                     "<kp value>",
                                     "<ki value>",
                                     "<kd value>",
                                     "<kp> <ki> <kd>",
                                     "<duty cycle value>",
                                     "",
                                     "",
                                     "",
                                     "",
                                     ""};
static_assert(sizeof(commands) == sizeof(commandsExtraParams));

struct plotXY {
  std::string xLabel;
  std::string yLabel;
};

void TabSerialPortManager::parseLine(const std::string &line) {
  if (line.empty()) {
    return;
  }
  if (cmd_compare("csv_header:", line)) {
    csvHeader.clear();
    csvRowsString.clear();
    csvHeader.resize(1);
    char c;
    size_t idx = line.find(':') + 1;
    for (; idx < line.size() && (c = line[idx]); idx++) {
      if (c == CSV_DELIM) {
        csvHeader.push_back("");
      } else {
        csvHeader.back().push_back(c);
      }
    }
    if (csvOutFile.is_open()) {
      csvOutFile << line << "\n";
    }
    notParsedText.clear();
  } else if (cmd_compare("csv_data:", line) && !csvHeader.empty()) {
    std::vector<std::string> newRow(1);
    std::vector<double> newRowDouble;
    char c;
    size_t idx = line.find(':') + 1;
    std::string data_line = line.substr(idx);
    bool validRow = true;
    for (; idx < line.size() && (c = line[idx]); idx++) {
      if (c == CSV_DELIM) {
        if (newRow.size() == csvHeader.size()) {
          validRow = false;
          break;
        }
        const char *strStart = newRow.back().c_str();
        char *strEnd;
        double value = strtod(strStart, &strEnd);
        if (strEnd == strStart + newRow.back().size()) {
          if (newRowDouble.empty()) {
            value /= 1000.0f;
          }
          newRowDouble.push_back(value);
        } else {
          validRow = false;
          break;
        }
        newRow.push_back("");
      } else {
        newRow.back().push_back(c);
      }
    }
    if (validRow && newRow.size() == csvHeader.size()) {
      csvRowsString.push_back(std::move(newRow));
      for (size_t i = 0; i < csvHeader.size(); i++) {
        csvData[csvHeader[i]].push_back(newRowDouble[i]);
      }
      if (csvOutFile.is_open()) {
        csvOutFile << data_line << "\n";
      }
    }
  } else {
    notParsedText.append(line + "\n");
  }
  if (csvHeader.empty()) {
    static double lastRequestT = -100.0;
    if (getTimestampSeconds() - lastRequestT > 0.5) {
      m_toSendCommand = "csv_header\n";
      lastRequestT = getTimestampSeconds();
    }
  }
}

void cleanDataUI() {
  if (ImGui::Button("Clear all data")) {
    csvData.clear();
  }
  static int keepNSamples = 3000;
  ImGui::SetNextItemWidth(80);
  ImGui::InputInt("samples to keep", &keepNSamples);

  ImGui::BeginGroup();
  ImGui::Text("Cleanup strategy");
  static int selectedStrategy = 0;
  ImGui::RadioButton("Remove uniformly (one every n samples)",
                     &selectedStrategy, 0);
  ImGui::RadioButton("Remove first samples", &selectedStrategy, 1);
  ImGui::EndGroup();
  if (ImGui::Button("Cleanup")) {
    if (selectedStrategy == 0) {
      removeSamplesUniformly(keepNSamples);
    } else if (selectedStrategy == 1) {
      keepFirstNSamples(keepNSamples);
    }
  }
}

void TabSerialPortManager::drawImpl() {
  ImVec2 space = ImGui::GetContentRegionAvail();

  ImGui::BeginGroup();
  ImGui::Text("Write csv output to file");
  if (ImGui::Button("Clear table")) {
    csvRowsString.clear();
  }
  ImGui::EndGroup();
  ImGui::SameLine();
  {
    ImGui::BeginGroup();
    if (csvOutFile.is_open()) {
      ImGui::Text("Currently writing on file");
      if (ImGui::Button("Close")) {
        csvOutFile.close();
      }
    } else {
      static std::string outFilePath = "teg_data.csv";
      const char *openMode[] = {"Overwrite", "Append"};
      ImGui::SetNextItemWidth(220);
      ImGui::InputText("Path of file", &outFilePath);
      static int currentOpenModeIndex = 0;
      ImGui::SetNextItemWidth(220);
      ImGui::Combo("Open mode", &currentOpenModeIndex, openMode,
                   IM_ARRAYSIZE(openMode));
      if (ImGui::Button("Open")) {
        csvOutFile.open(outFilePath, [&]() {
          if (currentOpenModeIndex == 0) {
            return std::ofstream::out;
          } else {
            return std::ofstream::out | std::ofstream::app;
          }
        }());
        for (const auto &col : csvHeader) {
          if (col == "") continue;
          csvOutFile << col << ",";
        }
        csvOutFile << "\n";
      }
    }
    ImGui::EndGroup();
  }
  if (csvHeader.empty()) {
    ImGui::Text("No csv data yet");
  } else {
    if (ImGui::BeginTable("CSV", csvHeader.size(),
                          ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY,
                          ImVec2(0.0f, space.y / 3.0f))) {
      for (const auto &column : csvHeader) {
        ImGui::TableSetupColumn(column.c_str());
      }
      ImGui::TableHeadersRow();
      for (auto it = csvRowsString.rbegin(); it != csvRowsString.rend(); ++it) {
        ImGui::TableNextRow();
        for (const auto &cell : *it) {
          ImGui::TableNextColumn();
          ImGui::Text("%s", cell.c_str());
        }
      }
      ImGui::EndTable();
    }
  }

  ImGui::SeparatorText("Send commands");

  static std::string currentCommand = "";
  static int currentCommandIndex = -1;
  ImGui::SetNextItemWidth(220);
  if (ImGui::Combo("Command Preset", &currentCommandIndex, commands,
                   IM_ARRAYSIZE(commands))) {
    currentCommand = commands[currentCommandIndex];
  }
  if (currentCommandIndex >= 0) {
    ImGui::TextDisabled("Command format: %s %s", commands[currentCommandIndex],
                        commandsExtraParams[currentCommandIndex]);
  }

  ImGui::SetNextItemWidth(220);
  if (ImGui::InputText("Command", &currentCommand,
                       ImGuiInputTextFlags_EnterReturnsTrue)) {
    m_toSendCommand = currentCommand + "\n";
  }

  ImGui::SeparatorText("Lines not parsed");
  if (ImGui::Button("Clear")) {
    notParsedText.clear();
  }
  ImGui::InputTextMultiline("##text", &notParsedText, ImVec2(),
                            ImGuiInputTextFlags_ReadOnly);

  ImGui::SeparatorText("Plot");
  cleanDataUI();
  ImGui::Separator();
  static std::vector<std::vector<plotXY>> customPlots = {
      {
          plotXY{.xLabel = "time", .yLabel = "temp_cold_side"},
          plotXY{.xLabel = "time", .yLabel = "temp_hot_side"},
          plotXY{.xLabel = "time", .yLabel = "hot_side_set_point"},
      },
      {
          plotXY{.xLabel = "time", .yLabel = "temp_delta"},
          plotXY{.xLabel = "time", .yLabel = "teg_oc_voltage"},
      },
      {
          plotXY{.xLabel = "time", .yLabel = "temp_delta"},
          plotXY{.xLabel = "time", .yLabel = "teg_current"},
      },
      {
          plotXY{.xLabel = "temp_delta", .yLabel = "teg_oc_voltage"},
      },
      {
          plotXY{.xLabel = "temp_delta", .yLabel = "teg_current"},
      }};
  int addToIndex = -1, toRemovePlot = -1;
  if (ImGui::Button("AddPlot")) {
    addToIndex = 0;
  }
  for (size_t i = 0; i < customPlots.size(); i++) {
    ImGui::PushID(i);
    ImGui::BeginGroup();
    ImGui::Text("Setup Plot");
    if (ImGui::Button("Delete")) {
      toRemovePlot = i;
    }
    if (ImGui::SmallButton("add line")) {
      customPlots[i].push_back({"", ""});
    }
    for (size_t j = 0; j < customPlots[i].size(); j++) {
      auto &[x, y] = customPlots[i][j];
      ImGui::PushID((i * customPlots.size()) + j);
      ImGui::SetNextItemWidth(80);
      ImGui::InputText("##xLabel", &x);
      ImGui::SetNextItemWidth(80);
      ImGui::InputText("##yLabel", &y);
      ImGui::PopID();
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    if (ImPlot::BeginPlot(std::to_string(i).c_str(), ImVec2(-1.0, 0.0),
                          ImPlotFlags_NoTitle)) {
      ImPlot::SetupAxes("##x", "##y", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);
      for (auto &[x, y] : customPlots[i]) {
        if (csvData.find(x) != csvData.end() &&
            csvData.find(y) != csvData.end()) {
          ImPlot::PlotLine((x + " - " + y).c_str(), &(csvData.at(x).begin()[0]),
                           &(csvData.at(y).begin()[0]), csvData.at(x).size());
        }
      }
      ImPlot::EndPlot();
    }
    if (ImGui::Button("AddPlot")) {
      addToIndex = i + 1;
    }
    ImGui::PopID();
  }
  if (addToIndex != -1) {
    customPlots.insert(customPlots.begin() + addToIndex, std::vector<plotXY>());
  }
  if (toRemovePlot != -1) {
    customPlots.erase(customPlots.begin() + toRemovePlot);
  }
}
