/* 
 * canbus_panel.hpp
 *
 * Created on 4/4/22 9:56 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_CANBUS_PANEL_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_CANBUS_PANEL_HPP

#include <memory>

#include "imview/panel.hpp"
#include "tbot/tbot_context.hpp"

namespace robosw {
class CanbusPanel : public swviz::Panel {
 public:
  CanbusPanel(swviz::Viewer *parent, TbotContext &ctx);

  void Draw() override;

 private:
  TbotContext &ctx_;
};
}

#endif //ROBOSW_SRC_APPS_TBOT_INCLUDE_CANBUS_PANEL_HPP
