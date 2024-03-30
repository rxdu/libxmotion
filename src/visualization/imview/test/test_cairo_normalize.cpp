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
#include "imview/cairo_draw.hpp"

using namespace xmotion::swviz;

const std::string img_file = "../image/fish.png";

class MyWin : public Viewer {
 public:
  MyWin(std::string title, uint32_t width, uint32_t height)
      : Viewer(title, width, height), cairo_panel_{width, height, true} {
    cairo_panel_.LoadImage(img_file);
  }

  bool show_demo_window = true;
  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  double b = 0.0001;

  void Update() override {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(window_->GetWidth(), window_->GetHeight()));

    ImGui::Begin("Cairo Canvas", NULL,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBackground);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    int width, height;
    glfwGetWindowSize(window_->GetGlfwWindow(), &width, &height);
    cairo_panel_.Resize(window_->GetWidth(), window_->GetHeight());

    cairo_panel_.Fill();
    cairo_panel_.Draw(std::bind(&MyWin::Paint, this, std::placeholders::_1));

    cairo_panel_.DrawText("1.5 m/s", 0.5, 0.5, 0.0, {0.5, 0.5, 0.5, 1}, 0.0140,
                          0.05);

    cairo_panel_.Render();

    ImGui::End();
  }

  void Paint(cairo_t* cr) {
    float ratio =
        window_->GetWidth() / static_cast<float>(window_->GetHeight());
    float pos_x = 0.5 * cairo_panel_.GetAspectRatio();
    float pos_y = 0.5;

    // std::cout << "first: " << ratio
    //           << " , second: " << cairo_panel_.GetAspectRatio() << std::endl;

    DrawPoint(cr, {pos_x, pos_y}, 0.01, {1, 0.2, 0.2, 0.6});
    DrawCircle(cr, {pos_x, pos_y}, 0.3, 0.002, {1, 0.2, 0.2, 0.6});
    DrawRectangle(cr, {pos_x - 0.2f, pos_y - 0.2f},
                  {pos_x + 0.2f, pos_y + 0.2f}, 0.002);
  }

 private:
  CairoWidget cairo_panel_;
};

int main(int argc, char* argv[]) {
  // MyWin win(1280, 720);
  //   MyWin win(1920, 1080);
  MyWin win("cairo_normalize", 1080, 720);

  win.Show();
  return 0;
}
