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

namespace xmotion {
class HidEvent {
 public:
  enum class EventType { kButtonEvent, kAxisEvent };

  HidEvent(EventType type, int id, int value)
      : type_(type), id_(id), value_(value) {}

  int GetId() const { return id_; }

  int GetValue() const { return value_; }

 private:
  EventType type_;
  int id_;
  int value_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_HID_EVENT_HPP
