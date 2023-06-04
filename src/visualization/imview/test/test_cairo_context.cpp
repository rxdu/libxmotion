/*
 * test_cairo_canvas.cpp
 *
 * Created on: Dec 03, 2020 21:27
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du
 */

#include <memory>

#include "imview/viewer.hpp"
#include "imview/cairo_widget.hpp"

using namespace xmotion::swviz;

void Paint(cairo_t* cr);

struct DrawArc : public Viewer {
  DrawArc() { ctx1_ = std::make_shared<CairoWidget>(320, 240); }

  std::shared_ptr<CairoWidget> ctx1_;

  void Update() override {
    ImVec2 panel_size = {ImGui::GetIO().DisplaySize.x / 2.0f,
                         ImGui::GetIO().DisplaySize.y / 2.0f};

    // show on imgui
    {
      ImGui::SetNextWindowPos(ImVec2(0, 0));
      ImGui::SetNextWindowSize(panel_size);

      ImGui::Begin("Cairo Canvas 1", NULL,
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                       ImGuiWindowFlags_NoBringToFrontOnFocus |
                       ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                       ImGuiWindowFlags_NoResize |
                       ImGuiWindowFlags_NoScrollbar);

      // do paint with cairo
      ctx1_->Draw(Paint);

      //   GLuint image = ctx1_->RenderToGlTexture();
      //   ImGui::Image((void*)(intptr_t)image, ImGui::GetContentRegionAvail());

      ImGui::End();
    }
  }
};

void Paint(cairo_t* cr) {
  cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 0.6);
  cairo_paint(cr);

  double x = 25.6, y = 128.0;
  double x1 = 102.4, y1 = 230.4, x2 = 153.6, y2 = 25.6, x3 = 230.4, y3 = 128.0;

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
}

int main(int argc, const char* argv[]) {
  DrawArc cc;
  cc.Show();

  return 0;
}