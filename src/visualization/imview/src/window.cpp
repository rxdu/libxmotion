/*
 * window.cpp
 *
 * Created on: Mar 04, 2021 16:01
 * Description:
 *
 * Note: how to generate compressed data from ttf file
 *  $ cd src/fonts && g++ binary_to_compressed_c.cpp
 *  $ ./a.out OpenSans-Regular.ttf OpenSansRegular > opensans_regular.hpp
 *
 * In the above command, "xxx.ttf" is the original ttf file, "OpenSansRegular"
 * will be part of the variable name, e.g. OpenSansRegular_compressed_data,
 * "xxx.hpp" will be the source file you can include to load compressed data
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/window.hpp"

#include <stdexcept>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "fonts/opensans_regular.hpp"
#include "fonts/opensans_semibold.hpp"
#include "fonts/opensans_bold.hpp"

namespace xmotion {
namespace swviz {
void Init() {
  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW library");
  }
}

void Terminate() { glfwTerminate(); }

//-------------------------------------------------------------------//

Window::Window(std::string title, uint32_t width, uint32_t height,
               uint32_t window_hints) {
  // setup GLFW window
  ApplyWindowHints(window_hints);
  glfw_win_ = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
  if (glfw_win_ == NULL) {
    throw std::runtime_error("Failed to create GLFW window");
  }
  glfwMakeContextCurrent(glfw_win_);
  glfwSwapInterval(1);

  // setup ImGUI context
  const char *glsl_version = "#version 130";
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  // connect GLFW and ImGUI
  ImGui_ImplGlfw_InitForOpenGL(glfw_win_, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  EnableDocking(true);

  LoadDefaultStyle();
}

Window::~Window() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(glfw_win_);
}

void Window::ApplyWindowHints(uint32_t window_hints) {
  // default hints
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, glfw_context_version_major);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, glfw_context_version_minor);

  // optional hints
  if (window_hints & WIN_FOCUSED) {
    glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
  }
  if (window_hints & WIN_RESIZABLE) {
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  }
  if (window_hints & WIN_DECORATED) {
    glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
  }
  if (window_hints & WIN_AUTO_ICONIFY) {
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
  }
  if (window_hints & WIN_FLOATING) {
    glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_FLOATING, GLFW_FALSE);
  }
  if (window_hints & WIN_MAXIMIZED) {
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_FALSE);
  }
}

void Window::LoadDefaultStyle() {
  // additional variable initialization
  background_color_ = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);

  ImGuiIO &io = ImGui::GetIO();
  // default font (first loaded font)
  font_normal_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 20.f);
  // additional fonts
  font_tiny_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 16.f);
  font_small_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 18.f);
  font_big_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 28.f);
  font_large_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 32.f);
  font_extra_large_ = io.Fonts->AddFontFromMemoryCompressedTTF(
      OpenSansRegular_compressed_data, OpenSansRegular_compressed_size, 40.f);

  // disable roundings
  ImGui::GetStyle().WindowRounding = 0.0f;
  ImGui::GetStyle().ChildRounding = 0.0f;
  ImGui::GetStyle().FrameRounding = 0.0f;
  ImGui::GetStyle().GrabRounding = 0.0f;
  ImGui::GetStyle().PopupRounding = 0.0f;
  ImGui::GetStyle().ScrollbarRounding = 0.0f;
  ImGui::GetStyle().WindowPadding = {0.0f, 0.0f};
}

void Window::ApplyDarkStyle() {
  background_color_ = ImVec4(0.4f, 0.4f, 0.4f, 1.00f);
  ImGui::StyleColorsDark();

  auto &colors = ImGui::GetStyle().Colors;
  colors[ImGuiCol_WindowBg] = ImVec4{0.1f, 0.105f, 0.11f, 1.0f};

  // Headers
  colors[ImGuiCol_Header] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_HeaderHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_HeaderActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Buttons
  colors[ImGuiCol_Button] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_ButtonHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_ButtonActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Frame BG
  colors[ImGuiCol_FrameBg] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_FrameBgHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_FrameBgActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Tabs
  colors[ImGuiCol_Tab] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TabHovered] = ImVec4{0.38f, 0.3805f, 0.381f, 1.0f};
  colors[ImGuiCol_TabActive] = ImVec4{0.28f, 0.2805f, 0.281f, 1.0f};
  colors[ImGuiCol_TabUnfocused] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TabUnfocusedActive] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};

  // Title
  colors[ImGuiCol_TitleBg] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TitleBgActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
}

void Window::ApplyLightStyle() {
  background_color_ = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
  ImGui::StyleColorsClassic();
}

void Window::EnableDocking(bool enable) {
  if (enable) {
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  } else {
    ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
  }
}

void Window::SetBackgroundColor(ImVec4 color) { background_color_ = color; }

void Window::EnableKeyboardNav() {
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
}

void Window::EnableGamepadNav() {
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
}

uint32_t Window::GetWidth() const {
  int width, height;
  glfwGetWindowSize(glfw_win_, &width, &height);
  return width;
  //   return ImGui::GetIO().DisplaySize.x;
}

uint32_t Window::GetHeight() const {
  int width, height;
  glfwGetWindowSize(glfw_win_, &width, &height);
  return height;
  // return ImGui::GetIO().DisplaySize.y;
}

void Window::PollEvents() {
  // Poll and handle events (inputs, window resize, etc.)
  glfwPollEvents();
}

void Window::CloseWindow() { glfwSetWindowShouldClose(glfw_win_, 1); }

bool Window::WindowShouldClose() { return glfwWindowShouldClose(glfw_win_); }

void Window::StartNewFrame() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Window::RenderFrame() {
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(glfw_win_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(background_color_.x, background_color_.y, background_color_.z,
               background_color_.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(glfw_win_);
}

ImFont *Window::GetFont(FontSize size) {
  if (size == FontSize::Tiny) {
    return font_tiny_;
  } else if (size == FontSize::Small) {
    return font_small_;
  } else if (size == FontSize::Big) {
    return font_big_;
  } else if (size == FontSize::Large) {
    return font_large_;
  } else if (size == FontSize::ExtraLarge) {
    return font_extra_large_;
  } else {
    return font_normal_;
  }
}

void Window::Show() {
  while (!glfwWindowShouldClose(glfw_win_)) {
    // Poll and handle events (inputs, window resize, etc.)
    glfwPollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) break;

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // // call draw()
    // Draw();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(glfw_win_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(background_color_.x, background_color_.y, background_color_.z,
                 background_color_.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(glfw_win_);
  }
}
}  // namespace swviz
}  // namespace xmotion