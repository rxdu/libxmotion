/*
 * keyboard_handler.cpp
 *
 * Created on 7/15/24 9:51 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_hid/keyboard_handler.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

#include "logging/xlogger.hpp"

namespace xmotion {
KeyboardHandler::KeyboardHandler(const std::string &input_event)
    : device_(input_event) {}

KeyboardHandler::~KeyboardHandler() { Close(); }

void KeyboardHandler::RegisterKeyEventCallback(KeyEventCallback callback) {
  key_event_callbacks_.push_back(callback);
}

bool KeyboardHandler::Open() {
  fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    XLOG_ERROR("Could not open device: {0}", device_);
    return false;
  }
  return true;
}

void KeyboardHandler::Close() {
  if (fd_ > 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool KeyboardHandler::IsOpened() const { return fd_ > 0; }

void KeyboardHandler::OnInputEvent() {
  struct input_event ev;
  ssize_t bytes_read = read(fd_, &ev, sizeof(ev));
  if (bytes_read <= 0) {
    XLOG_ERROR("No input event read from device: {0}", device_);
  } else if (bytes_read == sizeof(ev)) {
    if (ev.type == EV_KEY) {
      XLOG_DEBUG_STREAM("Key " << ev.code
                               << (ev.value ? " pressed" : " released"));
      for (auto &key_event_callback_ : key_event_callbacks_) {
        key_event_callback_(
            KeyboardMapping::keycode_map[ev.code],
            (ev.value ? KeyboardEvent::kPress : KeyboardEvent::kRelease));
      }
    }
  } else {
    XLOG_ERROR("Unexpected bytes read: {0}", bytes_read);
  }
}
}  // namespace xmotion