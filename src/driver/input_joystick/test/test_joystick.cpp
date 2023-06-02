/*
 * test_joystick.cpp
 *
 * Created on 6/1/23 8:46 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "input_joystick/joystick.hpp"

#include <thread>
#include <chrono>
#include <iostream>

using namespace robosw;

int main(int argc, char* argv[]) {
  auto jds = Joystick::EnumberateJoysticks();
  for (auto jd : jds) {
    std::cout << "Joystick index: " << jd.index << std::endl;
    std::cout << "Joystick name: " << jd.name << std::endl;
  }
  if (jds.empty()) {
    std::cout << "No joystick found" << std::endl;
    return -1;
  }

  Joystick js(jds.front());
  if (!js.Open()) {
    std::cout << "Failed to open joystick" << std::endl;
    return -1;
  }

  while (js.IsOpened()) {
    //    auto axes = js.GetAxes();
    //    std::cout << "Axes: ";
    //    for (auto a : axes) std::cout << a << " ";
    //    std::cout << std::endl;
    //
    //    auto buttons = js.GetButtons();
    //    std::cout << "Buttons: ";
    //    for (auto b : buttons) std::cout << b << " ";
    //    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return 0;
}