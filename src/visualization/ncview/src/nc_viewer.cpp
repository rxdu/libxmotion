/*
 * nc_viewer.cpp
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

#include "ncview/nc_viewer.hpp"

#include <ncurses.h>

#include <csignal>
#include <thread>
#include <chrono>
#include <iostream>

namespace robosw {
namespace swviz {
NcViewer::NcViewer(const std::string &title)
    : title_(title) {
  Init();
}

NcViewer::~NcViewer() {
  keep_running_ = false;
  Deinit();
}

void NcViewer::Init() {
  // setup ncurses mode
  initscr();
  // raw();
  cbreak();
  noecho();
  nonl();
  curs_set(FALSE);
  intrflush(stdscr, FALSE);
  keypad(stdscr, TRUE);

  // create main window
  getmaxyx(stdscr, term_size_y_, term_size_x_);
  window_ = newwin(term_size_y_, term_size_x_, 0, 0);
//  wrefresh(window_);
}

void NcViewer::Deinit() {
  delwin(window_);
  endwin();
}

void NcViewer::AddSubWindow(std::shared_ptr<NcSubWindow> win) {
  sub_wins_[win->GetName()] = win;
}

void NcViewer::Show(uint32_t fps) {
  uint32_t period_ms = 1000 / fps;
  keep_running_ = true;
  while (keep_running_) {
    getmaxyx(stdscr, term_size_y_, term_size_x_);

    // refresh sub-windows
    werase(window_);
    for (auto &win : sub_wins_) {
      win.second->Update();
    }
    mvwprintw(window_, 1, (term_size_x_ - title_.size()) / 2, title_.c_str(), NULL);
    box(window_, 0, 0);
    touchwin(window_);
    wrefresh(window_);

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