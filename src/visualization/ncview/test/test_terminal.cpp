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
  }

 private:
  void OnDraw() override {
//    mvwprintw(window_, 1, 1, "abcdefghijklmnopqrstuvwxyz");
//    mvwprintw(window_, 1, 1, "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz");
    mvwprintw(window_, disp_region_.pos.y, disp_region_.pos.x,
              "01234567890123456789012345678901234567890123456789012345678901234567890123456789");

    if (specs_.with_border) {
      box(window_, 0, 0);
    }
  }
};

int main(int argc, char *argv[]) {
  NcViewer viewer("NcView", true);
  auto disp = viewer.GetDisplayRegion();

  NcSubWindow::Specs specs;
  specs.name = "mywin";
//  specs.pos = disp.pos;
//  specs.size = disp.size;
  //  specs.pos.x = 1;
//  specs.pos.y = 2;
//  specs.size.x = 78;
//  specs.size.y = 21;

  auto win =
      std::make_shared<SampleWindow>(specs);

  NcHbox hbox;
  hbox.AddElement(win);

//  viewer.AddSubWindow(win);
  viewer.AddElement(win);
  viewer.Show();

  return 0;
}