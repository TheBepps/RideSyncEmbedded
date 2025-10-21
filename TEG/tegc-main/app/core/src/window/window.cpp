#include "window.hpp"

#include <cstdio>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "imgui_internal.h"
#include "imgui_stdlib.h"
#include "implot.h"

void GlwfErrorCallback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void InitImgui(GLFWwindow *window);

GLFWwindow *OpenWindow() {
  // Setup window
  glfwSetErrorCallback(GlwfErrorCallback);
  glewExperimental = GL_TRUE;
  if (!glfwInit()) {
    return nullptr;
  }

  GLFWmonitor *monitor = glfwGetPrimaryMonitor();

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
#ifndef __APPLE__
  glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
#endif

  int x, y, w, h;
  glfwGetMonitorWorkarea(monitor, &x, &y, &w, &h);

  glfwWindowHint(GLFW_MAXIMIZED, GL_TRUE);
  GLFWwindow *window =
      glfwCreateWindow(w, h, "Thermoelectric Generator", nullptr, nullptr);

  if (window == nullptr) {
    return nullptr;
  }

  // GLFWimage images[2];
  // for (auto itr = EnvPaths::Iterator(); itr != EnvPaths::End(); itr++) {
  //   std::filesystem::path path_ =
  //       *itr / EnvPaths::Retrieve(EnvPaths::ExtraPaths::ICONS) /
  //       "IconEagle.png";
  //   if (std::filesystem::exists(path_) &&
  //       std::filesystem::is_regular_file(path_)) {
  //     images[0].pixels =
  //         stbi_load(path_.string().c_str(), &images[0].width,
  //         &images[0].height,
  //                   nullptr, 4);  // rgba channels
  //   }
  // }
  //
  // if (images[0].width > 0 && images[0].height > 0) {
  //   glfwSetWindowIcon(window, 1, images);
  //   stbi_image_free(images[0].pixels);
  // }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // Enable vsync

  if (glewInit() != GLEW_OK) {
    return nullptr;
  }

  glEnable(GL_MULTISAMPLE);
  glfwShowWindow(window);

  InitImgui(window);

  return window;
}

void InitImgui(GLFWwindow *window) {
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;  // Enable Docking

  ImPlot::CreateContext();

  float scaling_x, scaling_y;
  glfwGetWindowContentScale(window, &scaling_x, &scaling_y);
#if defined(__APPLE__)
  s_app.scaling = 1.0f;
#endif

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();

  ImPlot::GetStyle().FitPadding = {0.1, 0.1};
}

void NewFrame() {
  glfwPollEvents();
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Dockspace() {
  float width = ImGui::GetMainViewport()->WorkSize.x;
  float height = ImGui::GetMainViewport()->WorkSize.y;
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->WorkPos);
  ImGui::SetNextWindowSize(ImVec2(width, height));

  ImGuiWindowFlags host_window_flags = 0;
  host_window_flags |= ImGuiWindowFlags_NoTitleBar |
                       ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                       ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking;
  host_window_flags |=
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

  ImGuiDockNodeFlags dockspace_flags =
      ImGuiDockNodeFlags_None | ImGuiDockNodeFlags_PassthruCentralNode;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::Begin("##MainDockspace", nullptr, host_window_flags);
  ImGui::PopStyleVar(3);

  ImGuiID dockspace_id = ImGui::GetID("DockSpace");
  ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags, nullptr);
  ImGui::End();
}

void Render(GLFWwindow *window, const ImVec4 &clearColor) {
  static int display_w, display_h;

  ImGui::Render();
  glfwGetFramebufferSize(window, &display_w, &display_h);

  glViewport(0, 0, display_w, display_h);
  glClearColor(clearColor.x * clearColor.w, clearColor.y * clearColor.w,
               clearColor.z * clearColor.w, clearColor.w);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
  if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
  }
  glfwMakeContextCurrent(window);

  glfwSwapBuffers(window);
}
