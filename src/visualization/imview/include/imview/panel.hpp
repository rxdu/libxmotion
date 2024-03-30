/*
 * panel.hpp
 *
 * Created on 4/3/22 11:07 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_PANEL_HPP
#define ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_PANEL_HPP

#include <string>

#include "imgui.h"

namespace xmotion {
namespace swviz {
class Viewer;

class Panel {
 public:
  Panel(std::string name, Viewer *parent);

  virtual void Draw() = 0;

 protected:
  void Begin(bool *p_open = NULL, ImGuiWindowFlags flags = 0);
  void End();

  std::string name_;
  Viewer *parent_;
};
}  // namespace swviz
}  // namespace xmotion

#endif  // ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_PANEL_HPP
