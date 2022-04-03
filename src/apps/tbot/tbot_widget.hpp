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

namespace robosw {
class TbotWidget : public swviz::Viewer {
 public:
  TbotWidget(uint32_t width = 640, uint32_t height = 480,
             std::string title = "Viewer",
             uint32_t window_hints = swviz::Window::WIN_RESIZABLE |
                 swviz::Window::WIN_DECORATED);

 private:
  std::unique_ptr<ControlPanel> control_panel_;

  void InitUI();
  void Update();
};
}

#endif //ROBOSW_SRC_APPS_TBOT_TBOT_WIDGET_HPP
