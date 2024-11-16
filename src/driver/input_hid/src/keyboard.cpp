/*
 * keyboard.cpp
 *
 * Created on 7/5/24 10:17 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_hid/keyboard.hpp"

#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>

#include "logging/xlogger.hpp"

#include "input_hid/keyboard_mapping.hpp"

namespace xmotion {
Keyboard::~Keyboard() {
  keep_running_ = false;
  if (io_thread_.joinable()) {
    keep_running_ = false;
    io_thread_.join();
  }
  if (fd_ > 0) close(fd_);
}

bool Keyboard::StartMonitoring(const std::string& event_name) {
  fd_ = open(event_name.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    XLOG_ERROR_STREAM("Failed to open input device: " << event_name);
    return false;
  }

  keep_running_ = true;
  io_thread_ = std::thread([this]() {
    while (keep_running_) {
      this->PollEvent();
      usleep(10000);
    }
  });

  return true;
}

void Keyboard::PollEvent() {
  struct input_event ev;
  while (ssize_t n = read(fd_, &ev, sizeof(ev)) > 0) {
    if (ev.type == EV_KEY) {
      XLOG_DEBUG_STREAM("Key " << ev.code
                               << (ev.value ? " pressed" : " released"));
      if (key_event_callback_ != nullptr) {
        key_event_callback_(
            KeyboardMapping::keycode_map[ev.code],
            (ev.value ? KeyboardEvent::kPress : KeyboardEvent::kRelease));
      }
    }
  }
}

void Keyboard::RegisterKeyEventCallback(KeyEventCallback callback) {
  key_event_callback_ = callback;
}
}  // namespace xmotion