/*
 * lightwindow.cpp
 *
 * Created on: May 08, 2020 09:41
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "lightview/lightwindow.hpp"

#include <iostream>

namespace {
void GlfwWinErrorCallback(int error, const char* description) {
  std::cerr << "GLFW error " << error << ": " << description << std::endl;
}

void GlfwKeyCallback(GLFWwindow* window, int key, int scancode, int action,
                     int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}
}  // namespace

namespace rav {
LightWindow::LightWindow(int width, int height, const char* title) {
  if (!glfwInit()) return;

    // Decide GL+GLSL versions
#if __APPLE__
  // GL 3.2 + GLSL 150
  glsl_version_ = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on Mac
#else
  // GL 3.0 + GLSL 130
  glsl_version_ = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
  // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
#endif

  // setup glfw window
  glfwSetErrorCallback(GlfwWinErrorCallback);
  window_ = glfwCreateWindow(width, height, title, NULL, NULL);
  if (!window_) return;

  // setup OpenGL context
  glfwMakeContextCurrent(window_);
  if (!gladLoadGL()) return;
  glfwSwapInterval(1);

  // additional tweaks
  glfwSetKeyCallback(window_, GlfwKeyCallback);
  //   glClearColor(clear_color_.x, clear_color_.y, clear_color_.z,
  //   clear_color_.w);

  win_initialized_ = true;
}

LightWindow::~LightWindow() {
  glfwDestroyWindow(window_);
  glfwTerminate();
}

void LightWindow::SetBackgroundColor(float x, float y, float z, float w) {
  clear_color_.x = x;
  clear_color_.y = y;
  clear_color_.z = z;
  clear_color_.w = w;
  glClearColor(clear_color_.x, clear_color_.y, clear_color_.z, clear_color_.w);
}

void LightWindow::SetErrorCallback(WinowErrorCallbackFunc cb) {
  glfwSetErrorCallback(cb);
}

void LightWindow::SetKeyCallback(WindowKeyCallbackFunc cb) {
  glfwSetKeyCallback(window_, cb);
};

void LightWindow::GetWindowSize(int& width, int& height) {
  int w, h;
  glfwGetWindowSize(window_, &w, &h);
  width = w;
  height = h;
}

void LightWindow::Draw() {
  // call user specified iterative procedure
  if (Iteration) Iteration();
}

void LightWindow::Run() {
  if (!win_initialized_) {
    std::cerr << "FATAL: GLFW not properly initialized" << std::endl;
    return;
  };

  while (!glfwWindowShouldClose(window_)) {
    // glClear(GL_COLOR_BUFFER_BIT);

    Draw();

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }
}

}  // namespace rav