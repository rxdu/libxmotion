/*
 * hid_event.hpp
 *
 * Created on 7/8/24 9:43 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_HID_EVENT_HPP
#define QUADRUPED_MOTION_HID_EVENT_HPP

#include "interface/driver/keyboard_interface.hpp"
#include "interface/driver/joystick_interface.hpp"
#include "quadruped/system_config.hpp"

namespace xmotion {
class HidEvent {
 public:
  enum class EventType { kKeyPressed, kButtonPressed, kAxisChanged };

  struct AxisChangedEventData {
    JsAxis axis;
    JsAxisValue value;
  };

  HidEvent(KeyboardCode code)
      : type_(EventType::kKeyPressed), pressed_key_(code) {}

  HidEvent(JsButton button)
      : type_(EventType::kButtonPressed), pressed_button_(button) {}

  HidEvent(AxisChangedEventData axis)
      : type_(EventType::kAxisChanged), changed_axis_(axis) {}

  KeyboardCode GetKeyCode() const { return pressed_key_; }

  JsButton GetJsButton() const { return pressed_button_; }

  AxisChangedEventData GetJsAxis() const { return changed_axis_; }

 private:
  EventType type_;
  KeyboardCode pressed_key_ = KeyboardCode::kUnknown;

  JsButton pressed_button_;
  AxisChangedEventData changed_axis_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_HID_EVENT_HPP
