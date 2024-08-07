/*
 * test_joystick.cpp
 *
 * Created on 6/1/23 8:46 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "input_hid/joystick.hpp"

#include <thread>
#include <chrono>
#include <iostream>

using namespace xmotion;

int main(int argc, char* argv[]) {
  int option = 0;
  int index = 0;
  if(argc == 2) {
        option = std::atoi(argv[1]);
  }
  else if(argc == 3) {
        option = std::atoi(argv[1]);
        index = std::atoi(argv[2]);
  } else {
        std::cout << "Usage: ./test_joystick <option>" << std::endl;
        std::cout << "Option: 0 - list joysticks" << std::endl;
        std::cout << "        1 - connect to joystick" << std::endl;
        return -1;
  }

  auto jds = Joystick::EnumberateJoysticks();
  for (auto jd : jds) {
    std::cout << "Joystick index: " << jd.index << std::endl;
    std::cout << "Joystick name: " << jd.name << std::endl;
  }
  if (jds.empty()) {
    std::cout << "No joystick found" << std::endl;
    return -1;
  }

  if(option == 0) return 0;

  Joystick js(index);
  if (!js.Open()) {
    std::cout << "Failed to open joystick" << std::endl;
    return -1;
  }

  while (js.IsOpened()) {
    std::cout << "Axes X: " << js.GetAxisState(JsAxis::kX).value
              << ", Axes Y: " << js.GetAxisState(JsAxis::kY).value
              << ", Axes Z: " << js.GetAxisState(JsAxis::kZ).value
              << ", Axes RX: " << js.GetAxisState(JsAxis::kRX).value
              << ", Axes RY: " << js.GetAxisState(JsAxis::kRY).value
              << ", Axes RZ: " << js.GetAxisState(JsAxis::kRZ).value
              << std::endl;

    if (js.GetButtonState(JsButton::kMode)) {
      js.SetJoystickRumble(0.1 * 0xFFFF, 0.5 * 0xFFFF);
    } else {
      js.SetJoystickRumble(0.0 * 0xFFFF, 0.0 * 0xFFFF);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}