/* 
 * tbot_widget.hpp
 *
 * Created on 4/3/22 10:48 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_TBOT_WIDGET_HPP
#define ROBOSW_SRC_APPS_TBOT_TBOT_WIDGET_HPP

#include <memory>

#include "imview/viewer.hpp"
#include "control_panel.hpp"
#include "plot_panel.hpp"
#include "canbus_panel.hpp"

namespace robosw {
class TbotWidget : public swviz::Viewer {
 public:
  TbotWidget(uint32_t width = 1280, uint32_t height = 800,
             std::string title = "Tbot Widget",
             uint32_t window_hints = swviz::Window::WIN_RESIZABLE |
                 swviz::Window::WIN_DECORATED);
  ~TbotWidget();

 private:
  TbotContext context_;

  std::unique_ptr<ControlPanel> control_panel_;
  std::unique_ptr<PlotPanel> plot_panel_;
  std::unique_ptr<CanbusPanel> canbus_panel_;

  const float panel_min_size_x = 350;
  const float panel_min_size_y = 80;

  void InitUI();
  void Update();
};
}

#endif //ROBOSW_SRC_APPS_TBOT_TBOT_WIDGET_HPP
