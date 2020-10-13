/*
 * imgui_base.cpp
 *
 * Created on: May 08, 2020 14:12
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "lightview/imgui_base.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace rav {

ImGuiBase::ImGuiBase(int width, int height, const char* title)
    : LightWindow(width, height, title) {
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  // Enable Keyboard Controls
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  // Enable Gamepad Controls
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init(glsl_version_.c_str());
}

ImGuiBase::~ImGuiBase() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

void ImGuiBase::Draw() {
  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  int width, height;
  GetWindowSize(width, height);
  ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);

  // no padding, no border
  //   ImGui::SetNextWindowBgAlpha(0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0);

  ImGuiWindowFlags window_flags = 0;
  window_flags |= ImGuiWindowFlags_NoTitleBar;
  window_flags |= ImGuiWindowFlags_NoResize;
  window_flags |= ImGuiWindowFlags_NoCollapse;

  bool show_another_window = true;
  {
    static float f = 0.0f;
    static int counter = 0;

    ImGui::Begin("Hello, world!",
                 &show_another_window, window_flags);  // Create a window called "Hello,
                                         // world!" and append into it.

    ImGui::Text("This is some useful text.");  // Display some text (you can use
                                               // a format strings too)
    // ImGui::Checkbox(
    //     "Demo Window",
    //     &show_demo_window);  // Edit bools storing our window open/close
    //     state
    ImGui::Checkbox("Another Window", &show_another_window);

    ImGui::SliderFloat("float", &f, 0.0f,
                       1.0f);  // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3(
        "clear color",
        (float*)&clear_color_);  // Edit 3 floats representing a color

    if (ImGui::Button("Button"))  // Buttons return true when clicked (most
                                  // widgets return true when edited/activated)
      counter++;
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
  }

  //   if (show_another_window) {
  //     ImGui::Begin(
  //         "Another Window",
  //         &show_another_window);  // Pass a pointer to our bool variable (the
  //                                 // window will have a closing button that
  //                                 will
  //                                 // clear the bool when clicked)
  //     ImGui::Text("Hello from another window!");
  //     if (ImGui::Button("Close Me")) show_another_window = false;
  //     ImGui::End();
  //   }

  // Rendering
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(window_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  //   glClearColor(clear_color_.x, clear_color_.y, clear_color_.z,
  //   clear_color_.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

}  // namespace rav