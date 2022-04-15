/* 
 * plot_panel.cpp
 *
 * Created on 4/4/22 9:25 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/plot_panel.hpp"

#include "implot/implot.h"

namespace robosw {
namespace {
template<typename T>
void PlotDataBufferLine(T &var_buff, const std::string &name) {
  if (!var_buff.GetData().empty()) {
    ImPlot::PlotLine(name.c_str(), &var_buff[0].x, &var_buff[0].y,
                     var_buff.GetSize(), var_buff.GetOffset(),
                     2 * sizeof(float));
  }
}
}

PlotPanel::PlotPanel(swviz::Viewer *parent, TbotContext &ctx) :
    Panel("PlotPanel", parent), ctx_(ctx) {

}

void PlotPanel::Draw() {
  auto &raw_rpm_left = ctx_.msger->GetDataBuffer(Messenger::DataBufferIndex::kRawRpmLeft);
  auto &raw_rpm_right = ctx_.msger->GetDataBuffer(Messenger::DataBufferIndex::kRawRpmRight);
  auto &filtered_rpm_left = ctx_.msger->GetDataBuffer(Messenger::DataBufferIndex::kFilteredRpmLeft);
  auto &filtered_rpm_right = ctx_.msger->GetDataBuffer(Messenger::DataBufferIndex::kFilteredRpmRight);

  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

  static float t = 0;
  if (ctx_.msger->IsStarted()) {
    t = std::chrono::duration_cast<std::chrono::milliseconds>(RSClock::now() -
        ctx_.time_of_start).count() / 1000.0f;
  }
  float history = ctx_.plot_history;

  static ImPlotAxisFlags flags =
      ImPlotAxisFlags_None;  // ImPlotAxisFlags_NoTickLabels;

  if (ImPlot::BeginPlot("RPM", ImVec2(-1, -1))) {
    ImPlot::SetupAxes(NULL, NULL, flags, flags);
    ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -600, 600);
    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

    PlotDataBufferLine(raw_rpm_left, "RPM Raw - Left");
    PlotDataBufferLine(raw_rpm_right, "RPM Raw - Right");
    PlotDataBufferLine(filtered_rpm_left, "RPM Filtered - Left");
    PlotDataBufferLine(filtered_rpm_right, "RPM Filtered - Right");

    ImPlot::EndPlot();
  }

  End();
}
}