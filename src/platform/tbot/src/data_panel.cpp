/* 
 * data_panel.cpp
 *
 * Created on 4/28/22 9:42 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/data_panel.hpp"

namespace xmotion {
DataPanel::DataPanel(swviz::Viewer *parent, TbotContext &ctx)
    : Panel("DataPanel", parent), ctx_(ctx) {}

void DataPanel::Draw() {
  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

  ImGui::Indent(10);

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("Left:");
    ImGui::SameLine(75);
    ImGui::SetNextItemWidth(305);
    ImGui::SliderFloat("##slider_pwm_left", &ctx_.plot_history, 0, 60, "%.0f s");
  }

  ImGui::Unindent(10);

  End();
}
}