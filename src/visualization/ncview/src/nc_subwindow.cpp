/*
 * nc_subwindow.cpp
 *
 * Created on: Jul 11, 2022 15:01
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/nc_viewer.hpp"

namespace robosw {
namespace swviz {
NcSubWindow::NcSubWindow(const Specs &specs)
    : specs_(specs) {
  window_ = derwin(stdscr, specs_.size.y, specs_.size.x,
                   specs_.pos.y, specs_.pos.x);
  bounding_box_ = {{specs_.pos.x, specs_.pos.x + specs_.size.x},
                   {specs_.pos.y, specs_.pos.y + specs_.size.y}};
}

NcSubWindow::~NcSubWindow() {
  delwin(window_);
}

void NcSubWindow::OnResize(int rows, int cols, int y, int x) {
  wresize(window_, rows, cols);
  mvwin(window_, y, x);
};

void NcSubWindow::Update() {
  // pre-draw

  // user-defined draw
  OnDraw();

  // post-draw
  if (specs_.with_border) {
    box(window_, 0, 0);
  }
}
}  // namespace swviz
}  // namespace robosw