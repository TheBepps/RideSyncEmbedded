#pragma once

// clang-format off
#include <GL/glew.h>
#include <GLFW/glfw3.h>
// clang-format on

#include "imgui.h"

GLFWwindow *OpenWindow();
void NewFrame();
void Dockspace();
void Render(GLFWwindow *window, const ImVec4 &clearColor);
