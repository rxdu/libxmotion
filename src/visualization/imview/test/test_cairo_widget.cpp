/*
 * test_imgui.cpp
 *
 * Created on: Mar 04, 2021 15:02
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"
#include "imview/cairo_widget.hpp"

using namespace xmotion::swviz;

const std::string img_file = "../data/screenshots/sampling/rrts.png";

class MyWin : public Viewer {
 public:
  MyWin(std::string title = "Canvas", uint32_t width = 1080,
        uint32_t height = 720)
      : Viewer(title, width, height), cairo_panel_{width, height} {
    cairo_panel_.LoadImage(img_file);
  }

  bool show_demo_window = true;
  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  double b = 0.0001;

  void Update() override {
    ShowCairoPanel();
    ShowImPanel();
    // ShowImPanel2();
  }

  void Paint(cairo_t* cr) {
    cairo_set_source_rgba(cr, 1, 1, 1, 0.8);
    cairo_paint(cr);

    double x = 25.6, y = 128.0;
    double x1 = 102.4, y1 = 230.4, x2 = 153.6, y2 = 25.6, x3 = 230.4,
           y3 = 128.0;

    cairo_set_source_rgba(cr, 0.2, 0.2, 0.2, 0.6);

    cairo_move_to(cr, x, y);
    cairo_curve_to(cr, x1, y1, x2, y2, x3, y3);

    cairo_set_line_width(cr, 10.0);
    cairo_stroke(cr);

    cairo_set_source_rgba(cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width(cr, 6.0);
    cairo_move_to(cr, x, y);
    cairo_line_to(cr, x1, y1);
    cairo_move_to(cr, x2, y2);
    cairo_line_to(cr, x3, y3);
    cairo_stroke(cr);

    b = b + 0.0005;
    std::string change;
    change = std::to_string(b);
    // cairo_panel_.Draw(change.c_str(), 20, 320, 240, 1 - b, 1, b);
  }

  void ShowCairoPanel() {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(window_->GetWidth(), window_->GetHeight()));

    ImGui::Begin("Cairo Canvas", NULL,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBackground);

    int width, height;
    glfwGetWindowSize(window_->GetGlfwWindow(), &width, &height);
    cairo_panel_.Resize(width, height);

    cairo_panel_.Fill();
    cairo_panel_.Draw(img_file, 0, 0, M_PI / 4.0f);
    cairo_panel_.Draw(std::bind(&MyWin::Paint, this, std::placeholders::_1));
    cairo_panel_.DrawText("Hello World, Canvas", 100, 100);

    cairo_panel_.Render();

    ImGui::End();
  }

  void ShowImPanel() {
    static float f = 0.0f;
    static int counter = 0;

    ImGui::Begin("Hello, world!");  // Create a window called "Hello, world!"
                                    // and append into it.

    // AddText("DrawImGuiText Function", ImGui::GetWindowPos().x + 100,
    //         ImGui::GetWindowPos().y + 100, ItemAlignment::TopLeft);
    // ImGui::SetWindowFontScale(1.2);

    ImGui::Text("This is some useful text.");  // Display some text (you can
                                               // use a format strings too)
    ImGui::Checkbox(
        "Demo Window",
        &show_demo_window);  // Edit bools storing our window open/close state
    ImGui::Checkbox("Another Window", &show_another_window);

    ImGui::SliderFloat("float", &f, 0.0f,
                       1.0f);  // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3(
        "clear color",
        (float*)&clear_color);  // Edit 3 floats representing a color

    if (ImGui::Button("Button"))  // Buttons return true when clicked (most
                                  // widgets return true when edited/activated)
      counter++;
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
  }

  void ShowImPanel2() {
    static float f = 0.0f;
    static int counter = 0;

    ImGui::Begin("Hello, world2!");  // Create a window called "Hello, world!"
                                     // and append into it.

    // ImGui::SetWindowFontScale(1.2);

    ImGui::Text("This is some useful text.");  // Display some text (you can
                                               // use a format strings too)
    ImGui::Checkbox(
        "Demo Window",
        &show_demo_window);  // Edit bools storing our window open/close state
    ImGui::Checkbox("Another Window", &show_another_window);

    ImGui::SliderFloat("float", &f, 0.0f,
                       1.0f);  // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3(
        "clear color",
        (float*)&clear_color);  // Edit 3 floats representing a color

    if (ImGui::Button("Button"))  // Buttons return true when clicked (most
                                  // widgets return true when edited/activated)
      counter++;
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
  }

 private:
  CairoWidget cairo_panel_;
};

int main(int argc, char* argv[]) {
  //   MyWin win(1280, 720);
  MyWin win("cairo_widget", 1920, 1080);
  //   MyWin win(1080, 720);

  win.Show();
  return 0;
}
