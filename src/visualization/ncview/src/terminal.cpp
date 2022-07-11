/*
 * terminal.cpp
 *
 * Created on: Jul 11, 2022 14:45
 * Description:
 *
 * Reference:
 * [1] https://pubs.opengroup.org/onlinepubs/7908799/xcurses/curses.h.html
 * [2] https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/index.html
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/terminal.hpp"

#include <ncurses.h>

#include <thread>
#include <chrono>
#include <iostream>

namespace robosw {
namespace swviz {
Terminal::Terminal() {
  initscr();
  // raw();
  cbreak();
  noecho();
  nonl();
  curs_set(FALSE);
  intrflush(stdscr, FALSE);
  keypad(stdscr, TRUE);
}

Terminal::~Terminal() { endwin(); }

void Terminal::AddWindow(std::shared_ptr<NcWindow> win) {
  windows_[win->GetName()] = win;
}

void Terminal::Show(uint32_t fps) {
  uint32_t period_ms = 1000 / fps;
  keep_running_ = true;
  while (keep_running_) {
    for (auto& win : windows_) {
      win.second->Clear();
      win.second->Draw();
      win.second->Refresh();
    }
    refresh();
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}
}  // namespace swviz
}  // namespace robosw