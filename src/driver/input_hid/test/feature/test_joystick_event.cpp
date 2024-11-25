/*
 * test_keyboard_event.cpp
 *
 * Created on 7/15/24 9:20 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "input_hid/joystick_handler.hpp"
#include "input_hid/hid_event_listener.hpp"

using namespace xmotion;

void OnJsButtonEvent(const JsButton& btn, const JxButtonEvent& event) {
  std::cout << "Button " << (int)(btn) << " "
            << (event == JxButtonEvent::kPress ? "pressed" : "released")
            << std::endl;
}

void OnJsAxisEvent(const JsAxis& axis, const float& value) {
  std::cout << "Axis " << (int)(axis) << " value: " << value << std::endl;
}

int main(int argc, char* argv[]) {
  int event_num = 8;

  if (argc == 2) {
    event_num = std::atoi(argv[1]);
  } else {
    std::cout << "Usage: " << argv[0] << " <event_num>" << std::endl;
    std::cout << "event_num: 0 - /dev/input/event0" << std::endl;
    return -1;
  }

  JoystickHandler joystick("/dev/input/event" + std::to_string(event_num));
  if (!joystick.Open()) {
    std::cerr << "Failed to open keyboard device" << std::endl;
    return -1;
  }
  joystick.RegisterJoystickButtonEventCallback(OnJsButtonEvent);
  joystick.RegisterJoystickAxisEventCallback(OnJsAxisEvent);

  HidEventListener hid_event_listener;
  if (!hid_event_listener.AddHidHandler(&joystick)) {
    std::cerr << "Failed to add joystick handler to event listener"
              << std::endl;
    return -1;
  }
  hid_event_listener.StartListening();

  uint32_t count = 0;
  while (count++ < 20) {
    std::cout << "running..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}