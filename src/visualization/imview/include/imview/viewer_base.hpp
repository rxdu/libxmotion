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

namespace robosw {
namespace viewer {
struct DisplayRegion {
  ImVec2 pos;
  ImVec2 size;
};

class ViewerBase {
 public:
  ViewerBase(uint32_t width = 640, uint32_t height = 480,
             std::string title = "Viewer",
             uint32_t window_hints = Window::WIN_RESIZABLE |
                 Window::WIN_DECORATED);
  virtual ~ViewerBase();

  // start viewer loop
  void Show();

 protected:
  std::unique_ptr<viewer::Window> window_;

  // draw function (to be implemented in derived classes)
  virtual void Update() {}
};
}  // namespace viewer
}  // namespace robosw

#endif /* VIEWER_BASE_HPP */
