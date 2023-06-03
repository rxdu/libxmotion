/* 
 * plot_panel.hpp
 *
 * Created on 4/4/22 9:25 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_PLOT_PANEL_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_PLOT_PANEL_HPP

#include "imview/panel.hpp"
#include "tbot_context.hpp"

namespace xmotion {
class PlotPanel : public swviz::Panel {
 public:
  PlotPanel(swviz::Viewer *parent, TbotContext &ctx);

  void Draw() override;

 private:
  TbotContext &ctx_;
};
}

#endif //ROBOSW_SRC_APPS_TBOT_INCLUDE_PLOT_PANEL_HPP
