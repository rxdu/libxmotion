/*
 * test_hid_event_listener.cpp
 *
 * Created on 7/15/24 9:20 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "input_hid/keyboard_handler.hpp"
#include "input_hid/hid_event_listener.hpp"

using namespace xmotion;

void OnKeyEvent(KeyboardCode code, KeyboardEvent event) {
  std::cout << "Key " << KeyboardMapping::GetKeyName(code) << " "
            << (event == KeyboardEvent::kPress ? "pressed" : "released")
            << std::endl;
}

int main(int argc, char* argv[]) {
  KeyboardHandler keyboard_handler("/dev/input/event3");
  if (!keyboard_handler.Open()) {
    std::cerr << "Failed to open keyboard device" << std::endl;
    return -1;
  }
  keyboard_handler.RegisterKeyEventCallback(OnKeyEvent);

  HidEventListener hid_event_listener;
  if (!hid_event_listener.AddHidHandler(&keyboard_handler)) {
    std::cerr << "Failed to add keyboard handler to event listener"
              << std::endl;
    return -1;
  }
  hid_event_listener.StartListening();

  uint32_t count = 0;
  while (count++ < 20) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}