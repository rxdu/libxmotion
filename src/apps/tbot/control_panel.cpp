/* 
 * control_panel.cpp
 *
 * Created on 4/3/22 11:12 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "control_panel.hpp"

namespace robosw {
ControlPanel::ControlPanel(swviz::Viewer *parent) :
    Panel("ControlPanel", parent) {

}

void ControlPanel::Draw() {
  Begin();
  End();
}
}