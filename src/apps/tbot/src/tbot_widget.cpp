/* 
 * tbot_widget.cpp
 *
 * Created on 4/3/22 10:48 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/tbot_widget.hpp"

namespace robosw {
using namespace swviz;
TbotWidget::TbotWidget(uint32_t width, uint32_t height,
                       std::string title, uint32_t window_hints) :
    swviz::Viewer(width, height, title, window_hints) {
  InitUI();
}

void TbotWidget::InitUI() {
  control_panel_ = std::make_unique<ControlPanel>(this, context_);
  plot_panel_ = std::make_unique<PlotPanel>(this, context_);
  canbus_panel_ = std::make_unique<CanbusPanel>(this, context_);

  ImGuiStyle &style = ImGui::GetStyle();
  float win_min_size_y = style.WindowMinSize.y;
  style.WindowMinSize.x = panel_min_size_x;
  style.WindowMinSize.y = panel_min_size_y;
}

void TbotWidget::Update() {
  DockSpaceOverMainViewport();

  control_panel_->Draw();
  plot_panel_->Draw();

  canbus_panel_->Draw();
}
}