/*
 * nc_window.cpp
 *
 * Created on: Jul 11, 2022 15:01
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/nc_window.hpp"

namespace robosw {
namespace swviz {
NcWindow::NcWindow(std::string name, Terminal* parent)
    : name_(name), parent_(parent) {
  int sy, sx;
  getmaxyx(stdscr, sy, sx);
  window_ = newwin(sy, sx, 0, 0);
}

NcWindow::~NcWindow() {
  if (window_ != NULL) delwin(window_);
}

void NcWindow::Clear() { wclear(window_); }

void NcWindow::Refresh() { wrefresh(window_); }
}  // namespace swviz
}  // namespace robosw