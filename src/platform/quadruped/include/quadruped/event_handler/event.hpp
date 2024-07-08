/*
 * event.hpp
 *
 * Created on 7/8/24 9:37 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_EVENT_HPP
#define QUADRUPED_MOTION_EVENT_HPP

#include <string>

namespace xmotion {
class Event {
 public:
  enum class EventType { kUserEvent, kSystemEvent };

  Event(EventType type, std::string name) : type_(type), name_(name) {}

  std::string GetName() const { return name_; }

 private:
  EventType type_;
  std::string name_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_EVENT_HPP
