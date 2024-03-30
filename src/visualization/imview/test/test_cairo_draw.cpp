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
  }

  void Paint(cairo_t* cr) {
    DrawPoint(cr, {860, 200});
    DrawPoint(cr, {1060, 200}, 10, {1, 0.2, 0.2, 0.6});

    DrawLine(cr, {860, 150}, {1060, 150});
    DrawLine(cr, {860, 250}, {1060, 250}, 2, {1, 0.2, 0.2, 0.6});

    DrawCircle(cr, {960, 540}, 30);
    DrawCircle(cr, {960, 540}, 50, 5, {1, 0.2, 0.2, 0.6});

    DrawRing(cr, {960, 540}, 100, 130, 0, M_PI / 4.0);
    DrawRing(cr, {960, 540}, 140, 180, 0, M_PI / 4.0, 5, colors[GREEN]);

    DrawRing(cr, {960, 540}, 100, 200, M_PI, M_PI * 1.5f, 5, colors[YELLOW],
             true);
    DrawRing(cr, {960, 540}, 140, 220, M_PI / 2.0, 2 * M_PI / 3.0, 5,
             colors[GREEN], true);
    DrawRing(cr, {960, 540}, 140, 220, M_PI / 2.0, 5 * M_PI / 6.0, 5,
             colors[PURPLE], false);

    DrawArc(cr, {600, 800}, 60, M_PI, 2 * M_PI / 4.0);
    DrawArc(cr, {600, 800}, 70, M_PI / 4.0, M_PI, 5, {1, 0.2, 0.2, 0.6});

    DrawArcSector(cr, {1060, 800}, 60, 0, -M_PI / 4.0);
    DrawArcSector(cr, {1060, 800}, 80, M_PI / 4.0, M_PI, 5, {1, 0.2, 0.2, 0.6});
    DrawArcSector(cr, {1060, 800}, 85, 5.0 * M_PI / 4.0, 6.0 * M_PI / 4.0, 5,
                  {1, 0.2, 0.2, 0.3}, true);

    DrawRectangle(cr, {400, 400}, {500, 600});
    DrawRectangle(cr, {550, 400}, {600, 600}, 5, colors[MAGENTA]);
    DrawRectangle(cr, {650, 400}, {700, 600}, 5, colors[CYAN], true);

    for (int i = 0; i < COLOR_LAST; ++i) {
      DrawLine(cr, {200.0f, 300.0f + 20 * i}, {250.0f, 300.0f + 20 * i}, 10,
               colors[i]);
    }
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
    // cairo_panel_.Draw(img_file, 0, 0, M_PI / 4.0f);
    cairo_panel_.Draw(std::bind(&MyWin::Paint, this, std::placeholders::_1));
    // cairo_panel_.DrawText("Hello World, Canvas", 100, 100);

    cairo_panel_.Render();

    ImGui::End();
  }

  void ShowImPanel() {
    ImGui::Begin("ImPanel");
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
  }

 private:
  CairoWidget cairo_panel_;
};

int main(int argc, char* argv[]) {
  //   MyWin win(1280, 720);
  MyWin win("cairo_draw", 1920, 1080);
  //   MyWin win(1080, 720);

  win.Show();
  return 0;
}
