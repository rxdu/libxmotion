/* 
 * plot_panel.cpp
 *
 * Created on 4/4/22 9:25 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/plot_panel.hpp"

namespace robosw {
PlotPanel::PlotPanel(swviz::Viewer *parent, TbotContext &ctx) :
    Panel("PlotPanel", parent), ctx_(ctx) {

}

void PlotPanel::Draw() {
  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);
  End();
}
}