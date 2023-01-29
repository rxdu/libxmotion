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
//  window_ = newwin(specs_.size.y, specs_.size.x,
//                   specs_.pos.y, specs_.pos.x);
//  int sy, sx;
//  getmaxyx(stdscr, sy, sx);
//  window_ = newwin(sy, sx, 0, 0);
//  bounding_box_ = {{specs_.pos.x, specs_.pos.x + specs_.size.x},
//                   {specs_.pos.y, specs_.pos.y + specs_.size.y}};
}

NcSubWindow::~NcSubWindow() {
  delwin(window_);
}

void NcSubWindow::OnResize() {
//  getmaxyx(stdscr, term_size_y_, term_size_x_);
//  wresize(window_, term_size_y_, term_size_x_);
//  mvwin(window_, 0, 0);
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