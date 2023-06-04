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

#include "ncview/nc_text.hpp"

namespace xmotion {
namespace swviz {
NcViewer::NcViewer(const std::string &title, bool has_border)
    : title_(title), has_border_(has_border) {
  Init();

  bool has_title = !title.empty();
  if (has_title && !has_border_) {
    title_option_ = TitleOption::kWithTitleOnly;
  } else if (!has_title && has_border_) {
    title_option_ = TitleOption::kWithBorderOnly;
  } else if (has_title && has_border_) {
    title_option_ = TitleOption::kWithBorderAndTitle;
  } else {
    title_option_ = TitleOption::kNone;
  }
  CalcDisplayRegion();
}

NcViewer::~NcViewer() {
  keep_running_ = false;
  Deinit();
}

void NcViewer::Init() {
  // setup ncurses mode
  initscr();
  NcText::InitColor();

  refresh();
  cbreak();
  noecho();
  nonl();
  curs_set(FALSE);
  intrflush(stdscr, FALSE);
  keypad(stdscr, TRUE);
}

void NcViewer::Deinit() {
  //  delete win and exit ncurses mode
  endwin();
}

// void NcViewer::AddSubWindow(std::shared_ptr<NcSubWindow> win) {
//   sub_wins_[win->GetName()] = win;
// }

void NcViewer::AddElement(std::shared_ptr<NcElement> element) {
  elements_.push_back(element);
}

void NcViewer::CalcDisplayRegion() {
  getmaxyx(stdscr, term_size_y_, term_size_x_);
  switch (title_option_) {
    case TitleOption::kNone: {
      disp_region_ = {{0, 0}, {0, 0}};
      break;
    }
    case TitleOption::kWithBorderOnly: {
      disp_region_ = {{1, 1}, {term_size_x_ - 2, term_size_y_ - 2}};
      break;
    }
    case TitleOption::kWithTitleOnly: {
      disp_region_ = {{0, 1}, {term_size_x_, term_size_y_ - 1}};
      break;
    }
    case TitleOption::kWithBorderAndTitle: {
      disp_region_ = {{1, 2}, {term_size_x_ - 2, term_size_y_ - 3}};
      break;
    }
  }
}

void NcViewer::Show(uint32_t fps) {
  uint32_t period_ms = 1000 / fps;
  keep_running_ = true;
  while (keep_running_) {
    // update sub-windows
    werase(stdscr);

    for (auto &element : elements_) {
      element->OnResize(disp_region_.size.y, disp_region_.size.x,
                        disp_region_.pos.y, disp_region_.pos.x);
      element->OnDraw();
    }

    // show title and border
    if (has_border_) {
      box(stdscr, 0, 0);
    }
    if (!title_.empty()) {
      int row = 0;
      if (has_border_) row = 1;
      mvwprintw(stdscr, row, (term_size_x_ - title_.size()) / 2, title_.c_str(),
                NULL);
    }

    touchwin(stdscr);
    wrefresh(stdscr);

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
      CalcDisplayRegion();
      resize_triggered_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}
}  // namespace swviz
}  // namespace xmotion