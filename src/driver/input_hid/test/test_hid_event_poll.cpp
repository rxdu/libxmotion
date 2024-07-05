/*
 * test_hid_event_poll.cpp
 *
 * Created on 7/6/24 12:20 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "input_hid/hid_event_poll.hpp"
#include "input_hid/keyboard.hpp"
#include "input_hid/joystick.hpp"

using namespace xmotion;

void OnKeyEvent(KeyboardCode code, KeyboardEvent event) {
  std::cout << "Key " << Keyboard::GetKeyName(code) << " "
            << (event == KeyboardEvent::kPress ? "pressed" : "released")
            << std::endl;
}

int main(int argc, char** argv) {
  // setup keyboard
  Keyboard kb(false);
  kb.StartMonitoring("/dev/input/event3");
  kb.RegisterKeyEventCallback(OnKeyEvent);

  // setup joystick
  Joystick js(22, false);
  if (!js.Open()) {
    std::cout << "Failed to open joystick" << std::endl;
    return -1;
  }

  HidEventPoll poll;
  poll.RegisterDevice(&kb);
  poll.RegisterDevice(&js);
  poll.StartPolling();

  while (true) {
    std::cout << "Axes X: " << js.GetAxisState(JsAxis::kX).value
              << ", Axes Y: " << js.GetAxisState(JsAxis::kY).value
              << ", Axes Z: " << js.GetAxisState(JsAxis::kZ).value
              << ", Axes RX: " << js.GetAxisState(JsAxis::kRX).value
              << ", Axes RY: " << js.GetAxisState(JsAxis::kRY).value
              << ", Axes RZ: " << js.GetAxisState(JsAxis::kRZ).value
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  poll.StopPolling();

  return 0;
}