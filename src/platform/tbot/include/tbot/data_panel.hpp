/* 
 * data_panel.hpp
 *
 * Created on 4/28/22 9:42 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_DATA_PANEL_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_DATA_PANEL_HPP

#include "imview/panel.hpp"
#include "tbot_context.hpp"

namespace xmotion {
class DataPanel : public swviz::Panel {
 public:
  DataPanel(swviz::Viewer *parent, TbotContext &ctx);

  void Draw() override;

 private:
  TbotContext &ctx_;
};
}

#endif //ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_DATA_PANEL_HPP
