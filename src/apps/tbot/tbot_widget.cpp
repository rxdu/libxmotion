/* 
 * tbot_widget.cpp
 *
 * Created on 4/3/22 10:48 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot_widget.hpp"

namespace robosw {
using namespace swviz;
TbotWidget::TbotWidget(uint32_t width, uint32_t height,
                       std::string title, uint32_t window_hints) :
    swviz::Viewer(width, height, title, window_hints) {
  InitUI();
}

void TbotWidget::InitUI() {
  control_panel_ = std::make_unique<ControlPanel>(this);
}

void TbotWidget::Update() {
  DockSpaceOverMainViewport();

  control_panel_->Draw();
}
}