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
  window_ = derwin(stdscr, 1, 1, 0, 0);
  OnResize(1, 1, 0, 0);
}

NcSubWindow::~NcSubWindow() {
  delwin(window_);
}

void NcSubWindow::OnResize(int rows, int cols, int y, int x) {
  wresize(window_, rows, cols);
  mvderwin(window_, y, x);
  bounding_box_ = {{x, x + cols}, {y, y + rows}};
  if (specs_.with_border) {
    disp_region_ = {1, 1, cols - 2, rows - 2};
  } else {
    disp_region_ = {0, 0, cols, rows};
  }
};

void NcSubWindow::ShowTitle() {
  if (specs_.with_border) {
    box(window_, 0, 0);
    mvwprintw(window_, disp_region_.pos.y - 1, disp_region_.pos.x + 1,
              specs_.name.c_str(), NULL);
  }
}

void NcSubWindow::Update() {
  // pre-draw

  // user-defined draw
  OnDraw();

  // post-draw
  if (specs_.with_border) {
    box(window_, 0, 0);
    disp_region_ = {bounding_box_.x.min + 1, bounding_box_.y.min + 1,
                    bounding_box_.x.max - bounding_box_.x.min + 1 - 2,
                    bounding_box_.y.max - bounding_box_.y.min + 1 - 2};
  }
}
}  // namespace swviz
}  // namespace robosw