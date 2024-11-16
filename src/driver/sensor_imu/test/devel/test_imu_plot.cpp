/*
 * test_plot_buffer.cpp
 *
 * Created on: Mar 25, 2021 17:41
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"
#include "imview/data_buffer.hpp"

using namespace xmotion::swviz;

struct ImDraw : public Viewer {
  void Update() override {
    // do nothing
    ImGui::BulletText("Move your mouse to change the data!");
    ImGui::BulletText(
        "This example assumes 60 FPS. Higher FPS requires larger buffer "
        "size.");
    static DataBuffer sdata1;
    static DataBuffer sdata2;

    ImVec2 mouse = ImGui::GetMousePos();
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    sdata1.AddPoint(t, mouse.x * 0.0005f);
    sdata2.AddPoint(t, mouse.y * 0.0005f);

    static float history = 10.0f;
    ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

    static ImPlotAxisFlags rt_axis = ImPlotAxisFlags_NoTickLabels;
    // ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);

    if (ImPlot::BeginPlot("##Scrolling")) {
      ImPlot::PlotShaded("Data 1", &(sdata1[0].x), &(sdata1[0].y),
                         sdata1.GetSize(), 0, sdata1.GetOffset(),
                         2 * sizeof(float));
      ImPlot::PlotLine("Data 2", &(sdata1[0].x), &(sdata1[0].y),
                       sdata2.GetSize(), sdata2.GetOffset(), 2 * sizeof(float));
      ImPlot::EndPlot();

      //   std::cout << "buffer size: " << sdata1.GetSize() << std::endl;
    }
  }
};

int main(int argc, char *argv[]) {
  ImDraw canvas;
  canvas.Show();
  return 0;
}