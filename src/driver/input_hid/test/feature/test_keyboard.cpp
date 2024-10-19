/*
 * test_keyboard.cpp
 *
 * Created on 7/5/24 10:23 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <string>

#include "input_hid/keyboard.hpp"

using namespace xmotion;

void OnKeyEvent(KeyboardCode code, KeyboardEvent event) {
  std::cout << "Key " << KeyboardMapping::GetKeyName(code) << " "
            << (event == KeyboardEvent::kPress ? "pressed" : "released")
            << std::endl;
}

int main(int argc, char** argv) {
  int event_num = 8;

  if (argc == 2) {
    event_num = std::atoi(argv[1]);
  } else {
    std::cout << "Usage: " << argv[0] << " <event_num>" << std::endl;
    std::cout << "event_num: 0 - /dev/input/event0" << std::endl;
    return -1;
  }

  Keyboard kb;
  kb.RegisterKeyEventCallback(OnKeyEvent);
  kb.StartMonitoring("/dev/input/event" + std::to_string(event_num));
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
