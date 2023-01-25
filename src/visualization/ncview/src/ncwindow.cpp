/*
 * ncwindow.cpp
 *
 * Created on: Jul 11, 2022 15:01
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/ncwindow.hpp"

namespace robosw {
namespace swviz {
NcWindow::NcWindow(std::string name, NcViewer* parent)
    : name_(name), parent_(parent) {
  int sy, sx;
  getmaxyx(stdscr, sy, sx);
  window_ = newwin(sy, sx, 0, 0);
}

NcWindow::~NcWindow() {
  if (window_ != NULL) delwin(window_);
}

void NcWindow::Clear() {
  werase(window_);

  // update window size
  getmaxyx(stdscr, term_size_y_, term_size_x_);
}

void NcWindow::Refresh() { wnoutrefresh(window_); }
}  // namespace swviz
}  // namespace robosw