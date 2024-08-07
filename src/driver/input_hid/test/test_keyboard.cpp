/*
 * test_keyboard.cpp
 *
 * Created on 7/5/24 10:23 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_hid/keyboard.hpp"

#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <fstream>
#include <string>
#include <regex>

using namespace xmotion;

void OnKeyEvent(KeyboardCode code, KeyboardEvent event) {
  std::cout << "Key " << KeyboardMapping::GetKeyName(code) << " "
            << (event == KeyboardEvent::kPress ? "pressed" : "released")
            << std::endl;
}

int main(int argc, char** argv) {
  Keyboard kb;
  kb.StartMonitoring("/dev/input/event3");
  kb.RegisterKeyEventCallback(OnKeyEvent);
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}

// #include <iostream>
// #include <fcntl.h>
// #include <unistd.h>
// #include <linux/input.h>
// #include <cstring>
//
// int main(int argc, char* argv[]) {
//   if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " <keyboard event number>"
//               << std::endl;
//     return 1;
//   }
//
//   const char* device_path = argv[1];
//
//   // Open the device file
//   int fd = open(device_path, O_RDONLY);
//   if (fd < 0) {
//     std::cerr << "Failed to open input device: " << device_path << std::endl;
//     return 1;
//   }
//
//   struct input_event ev;
//   ssize_t n;
//
//   std::cout << "Reading keyboard events, press Ctrl+C to quit..." <<
//   std::endl;
//
//   while (true) {
//     n = read(fd, &ev, sizeof(ev));
//     if (n == (ssize_t)-1) {
//       std::cerr << "Failed to read input event" << std::endl;
//       close(fd);
//       return 1;
//     }
//
//     if (ev.type == EV_KEY) {
//       std::cout << "Key " << ev.code << (ev.value ? " pressed" : " released")
//                 << std::endl;
//     }
//
//     // Small delay to avoid busy loop
//     usleep(10000);
//   }
//
//   close(fd);
//   return 0;
// }
