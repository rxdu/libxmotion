/*
 * @file joystick_handler.cpp
 * @date 10/19/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_hid/joystick_handler.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

#include "logging/xlogger.hpp"

namespace xmotion {
JoystickHandler::JoystickHandler(const std::string& input_event)
    : device_(input_event) {}

JoystickHandler::~JoystickHandler() { Close(); }

void JoystickHandler::RegisterJoystickButtonEventCallback(
    JoystickHandler::JoystickButtonEventCallback callback) {
  button_event_callbacks_.push_back(callback);
}

void JoystickHandler::RegisterJoystickAxisEventCallback(
    JoystickHandler::JoystickAxisEventCallback callback) {
  axis_event_callbacks_.push_back(callback);
}

bool JoystickHandler::Open() {
  fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    XLOG_ERROR("Could not open device: {0}", device_);
    return false;
  }
  // read joystick info
  for (unsigned int i = 0; i < axis_info_.size(); ++i) {
    input_absinfo axisInfo;
    if (ioctl(fd_, EVIOCGABS(i), &axisInfo) != -1) {
      axis_info_[i].min = axisInfo.minimum;
      axis_info_[i].max = axisInfo.maximum;
    }
  }

  return true;
}

void JoystickHandler::Close() {
  if (fd_ > 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool JoystickHandler::IsOpened() const { return fd_ > 0; }

void JoystickHandler::OnInputEvent() {
  struct input_event ev;
  ssize_t bytes_read = read(fd_, &ev, sizeof(ev));
  if (bytes_read <= 0) {
    XLOG_ERROR("No input event read from device: {0}", device_);
  } else if (bytes_read == sizeof(ev)) {
    // only handle button and axis events
    if (ev.type == EV_KEY && ev.code >= BTN_JOYSTICK && ev.code <= BTN_THUMBR) {
      XLOG_DEBUG_STREAM("Joystick button "
                        << ev.code << (ev.value ? " pressed" : " released"));
      for (auto& button_event_callback_ : button_event_callbacks_) {
        button_event_callback_(
            static_cast<JsButton>(ev.code - 0x120),
            (ev.value ? JxButtonEvent::kPress : JxButtonEvent::kRelease));
      }
    } else if (ev.type == EV_ABS && ev.code < ABS_TOOL_WIDTH) {
      int axis_index = ev.code;
      float normalized = (ev.value - axis_info_[axis_index].min) /
                             static_cast<float>(axis_info_[axis_index].max -
                                                axis_info_[axis_index].min) *
                             2 -
                         1;
      XLOG_DEBUG_STREAM("Joystick axis " << ev.code
                                         << " value: " << normalized);
      for (auto& axis_event_callback_ : axis_event_callbacks_) {
        axis_event_callback_(static_cast<JsAxis>(ev.code), normalized);
      }
    }
  } else {
    XLOG_ERROR("Unexpected bytes read: {0}", bytes_read);
  }
}
}  // namespace xmotion