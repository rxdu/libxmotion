/*
 * terminal.cpp
 *
 * Created on: Jul 11, 2022 14:45
 * Description:
 *
 * Reference:
 * [1] https://pubs.opengroup.org/onlinepubs/7908799/xcurses/curses.h.html
 * [2] https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/index.html
 * [3] https://invisible-island.net/ncurses/man/resizeterm.3x.html#h2-NOTES
 * [4]
 * https://stackoverflow.com/questions/53822353/where-does-curses-inject-key-resize-on-endwin-and-refresh
 * [5] https://github.com/superjamie/snippets/blob/master/curses-window-resize.c
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/ncviewer.hpp"

#include <ncurses.h>

#include <csignal>
#include <thread>
#include <chrono>
#include <iostream>

namespace robosw {
namespace swviz {
NcViewer::NcViewer() { Init(); }

NcViewer::~NcViewer() { Deinit(); }

void NcViewer::Init() {
  initscr();
  // raw();
  cbreak();
  noecho();
  nonl();
  curs_set(FALSE);
  intrflush(stdscr, FALSE);
  keypad(stdscr, TRUE);
}

void NcViewer::Deinit() { endwin(); }

void NcViewer::AddWindow(std::shared_ptr<NcWindow> win) {
  windows_[win->GetName()] = win;
}

void NcViewer::Show(uint32_t fps) {
  uint32_t period_ms = 1000 / fps;
  keep_running_ = true;
  while (keep_running_) {
    // refresh sub-windows
    for (auto& win : windows_) {
      win.second->Clear();
      win.second->Draw();
      win.second->Refresh();
    }
    doupdate();

    // handle input
    int input_ch = getch();
    switch (input_ch) {
      case KEY_RESIZE: {
        resize_triggered_ = true;
        break;
      }
    }

    // handle window resize
    if (resize_triggered_) {
      Deinit();
      Init();
      resize_triggered_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}
}  // namespace swviz
}  // namespace robosw