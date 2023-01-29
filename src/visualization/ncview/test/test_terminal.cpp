/*
 * test_terminal.cpp
 *
 * Created on: Jul 11, 2022 14:43
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include <csignal>
#include <iostream>

#include "ncview/nc_viewer.hpp"
#include "ncview/nc_subwindow.hpp"
#include "ncview/nc_hbox.hpp"

using namespace robosw::swviz;

class SampleWindow : public NcSubWindow {
 public:
  SampleWindow(const NcSubWindow::Specs &specs) :
      NcSubWindow(specs) {
    //  NcHbox hbox;
    //  hbox.AddElement(win);
  }

 private:
  void OnDraw() override {
//    mvprintw(1, 1, "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz");

    mvwprintw(window_, 1, 1, "abcdefghijklmnopqrstuvwxyz");

//    mvwprintw(window_, 1, 1, "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz");
//    mvwprintw(window_, 2,
//              0,
//              "01234567890123456789012345678901234567890123456789012345678901234567890123456789");
  }
};

int main(int argc, char *argv[]) {
  NcSubWindow::Specs specs;
  specs.name = "mywin";
  specs.pos.x = 1;
  specs.pos.y = 2;
  specs.size.x = 70;
  specs.size.y = 15;

  NcViewer viewer;
  auto win =
      std::make_shared<SampleWindow>(specs);
  viewer.AddSubWindow(win);
  viewer.Show();

  return 0;
}