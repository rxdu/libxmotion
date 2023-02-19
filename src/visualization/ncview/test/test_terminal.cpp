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
#include "ncview/nc_vbox.hpp"

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

    ShowTitle();
  }
};

int main(int argc, char *argv[]) {
  NcViewer viewer("NcView", true);
  auto disp = viewer.GetDisplayRegion();

  NcSubWindow::Specs specs;
  specs.name = "mywin";
  specs.with_border = true;

  auto win =
      std::make_shared<SampleWindow>(specs);
  auto win2 =
      std::make_shared<SampleWindow>(specs);
  auto win3 =
      std::make_shared<SampleWindow>(specs);

  auto hbox = std::make_shared<NcHbox>();
  hbox->AddElement(win, NcConstraint{NcConstraint::Type::kFixed, 0.6});
  hbox->AddElement(win2);

  auto hbox2 = std::make_shared<NcHbox>();
  hbox2->AddElement(win3);

  auto vbox = std::make_shared<NcVbox>();
  vbox->AddElement(hbox, NcConstraint{NcConstraint::Type::kFixed, 0.7});
  vbox->AddElement(hbox2);

  viewer.AddElement(vbox);
  viewer.Show();

  return 0;
}