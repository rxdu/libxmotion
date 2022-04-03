/* 
 * control_panel.hpp
 *
 * Created on 4/3/22 11:12 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_CONTROL_PANEL_HPP
#define ROBOSW_SRC_APPS_TBOT_CONTROL_PANEL_HPP

#include "imview/panel.hpp"

namespace robosw {
class ControlPanel : public swviz::Panel {
 public:
  ControlPanel(swviz::Viewer *parent);

  void Draw() override;

 private:

};
}

#endif //ROBOSW_SRC_APPS_TBOT_CONTROL_PANEL_HPP
