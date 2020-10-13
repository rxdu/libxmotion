/*
 * imgui_base.hpp
 *
 * Created on: May 08, 2020 14:07
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef IMGUI_BASE_HPP
#define IMGUI_BASE_HPP

#include "lightview/lightwindow.hpp"

namespace rav {
class ImGuiBase : public LightWindow {
 public:
  ImGuiBase(int width = 640, int height = 480, const char* title = "ImGUI");
  virtual ~ImGuiBase();

//   virtual void DrawGUI();

 protected:
  void Draw() override;
};
}  // namespace rav

#endif /* IMGUI_BASE_HPP */
