/*
 * test_terminal.cpp
 *
 * Created on: Jul 11, 2022 14:43
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "ncview/ncviewer.hpp"
#include "ncview/ncsubwindow.hpp"

#include <csignal>
#include <iostream>

using namespace xmotion::swviz;

class MyWin : public NcWindow {
 public:
  MyWin(std::string name, NcViewer* parent) : NcWindow(name, parent) {}

  void OnDraw() override {
    box(window_, 0, 0);
    // printw("Hello World !!!"); /* Print Hello World		  */
  }

 private:
};

// void signal_handler(int signal) {
//   std::cout << "signal received: " << signal << std::endl;
// }

int main(int argc, char* argv[]) {
  NcViewer term;

//   std::signal(SIGWINCH, signal_handler);

  auto win = std::make_shared<MyWin>("mywin", &term);
  term.AddWindow(win);

  term.Show();
  return 0;
}