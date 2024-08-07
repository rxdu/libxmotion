/*
 * tbot_widget.cpp
 *
 * Created on 4/3/22 10:48 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/tbot_widget.hpp"

namespace xmotion {
using namespace swviz;
TbotWidget::TbotWidget(std::string title, uint32_t width, uint32_t height,
                       uint32_t window_hints)
    : swviz::Viewer(title, width, height, window_hints) {
  InitUI();
}

TbotWidget::~TbotWidget() {
  if (context_.msger && context_.msger->IsStarted()) {
    context_.msger->Stop();
  }
}

void TbotWidget::InitUI() {
  // init context
  context_.time_of_start = RSClock::now();
  context_.msger = std::make_shared<Messenger>(context_.time_of_start);
  context_.speed_ctrl_ = std::make_shared<SpeedController>();

  // init ui components
  control_panel_ = std::make_unique<ControlPanel>(this, context_);
  plot_panel_ = std::make_unique<PlotPanel>(this, context_);
  canbus_panel_ = std::make_unique<CanbusPanel>(this, context_);
  data_panel_ = std::make_unique<DataPanel>(this, context_);

  // set ui styles
  ImGuiStyle &style = ImGui::GetStyle();
  float win_min_size_y = style.WindowMinSize.y;
  style.WindowMinSize.x = panel_min_size_x;
  style.WindowMinSize.y = panel_min_size_y;
}

void TbotWidget::Update() {
  DockSpaceOverMainViewport();

  control_panel_->Draw();
  plot_panel_->Draw();
  data_panel_->Draw();
  canbus_panel_->Draw();
}
}  // namespace xmotion