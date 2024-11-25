/*
 * @file event_dispatcher.hpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EVENT_DISPATCHER_HPP
#define QUICKVIZ_EVENT_DISPATCHER_HPP

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include "event/event.hpp"

namespace xmotion {
class EventDispatcher {
 public:
  using EventPtr = std::shared_ptr<BaseEvent>;
  using HandlerFunc = std::function<void(std::shared_ptr<BaseEvent>)>;

  // public interface
  static EventDispatcher& GetInstance();
  void RegisterHandler(const std::string& event_name, HandlerFunc handler);
  void Dispatch(std::shared_ptr<BaseEvent> event) const;

 private:
  EventDispatcher() = default;

  // do not allow copy or move
  EventDispatcher(const EventDispatcher&) = delete;
  EventDispatcher(EventDispatcher&&) = delete;
  EventDispatcher& operator=(const EventDispatcher&) = delete;
  EventDispatcher& operator=(EventDispatcher&&) = delete;

  std::unordered_map<std::string, std::vector<HandlerFunc>> handlers_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_DISPATCHER_HPP