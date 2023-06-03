/* 
 * panel.cpp
 *
 * Created on 4/3/22 11:07 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "imview/panel.hpp"

namespace xmotion {
namespace swviz {
Panel::Panel(std::string name, Viewer *parent) :
    name_(name), parent_(parent) {}

void Panel::Begin(bool *p_open, ImGuiWindowFlags flags) {
  ImGui::Begin(name_.c_str(), p_open, flags);
}

void Panel::End() {
  ImGui::End();
}
}
}