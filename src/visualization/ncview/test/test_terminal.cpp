/*
 * test_terminal.cpp
 *
 * Created on: Jul 11, 2022 14:43
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/terminal.hpp"
#include "ncview/nc_window.hpp"

using namespace robosw::swviz;

class MyWin : public NcWindow {
 public:
  MyWin(std::string name, Terminal* parent) : NcWindow(name, parent) {}

  void Draw() override {
    box(window_, 0, 0);
    // printw("Hello World !!!"); /* Print Hello World		  */
  }

 private:
};

int main(int argc, char* argv[]) {
  Terminal term;

  std::shared_ptr<MyWin> win = std::make_shared<MyWin>("mywin", &term);
  term.AddWindow(win);

  term.Show();
  return 0;
}