#pragma once

#include <string>

#include "imgui.h"
class TabBase {
 public:
  TabBase(const std::string &name) : m_name(name) {}
  const std::string &name() const { return m_name; }

  void draw() {
    if (ImGui::Begin(m_name.c_str())) {
      drawImpl();
    }
    ImGui::End();
  }

  virtual void drawImpl() = 0;

 protected:
  std::string m_name;
};
