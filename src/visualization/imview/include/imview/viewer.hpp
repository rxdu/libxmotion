/*
 * viewer_base.hpp
 *
 * Created on: Jul 27, 2021 08:56
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef VIEWER_BASE_HPP
#define VIEWER_BASE_HPP

#include <memory>

#include "imview/window.hpp"

namespace xmotion {
namespace swviz {
struct DisplayRegion {
  ImVec2 pos;
  ImVec2 size;
};

class Viewer {
 public:
  Viewer(std::string title = "Viewer", uint32_t width = 640,
         uint32_t height = 480,
         uint32_t window_hints = Window::WIN_RESIZABLE | Window::WIN_DECORATED);
  virtual ~Viewer();

  // public API
  uint32_t GetWidth();
  uint32_t GetHeight();
  ImFont *GetFont(FontSize size);

  void DockSpaceOverMainViewport();

  // start viewer loop
  void Show();

 protected:
  std::unique_ptr<swviz::Window> window_;

  // draw function (to be implemented in derived classes)
  virtual void Update() {}
};
}  // namespace swviz
}  // namespace xmotion

#endif /* VIEWER_BASE_HPP */
