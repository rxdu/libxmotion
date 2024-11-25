/*
 * @file event_emitter.hpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_EVENT_EMITTER_HPP
#define QUICKVIZ_EVENT_EMITTER_HPP

#include "event/event_dispatcher.hpp"

namespace xmotion {
class EventEmitter {
 public:
  template <typename EventT, typename... Args>
  void Emit(EventSource type, const std::string& event_name, Args... args) {
    auto event = std::make_shared<EventT>(type, event_name, args...);
    EventDispatcher::GetInstance().Dispatch(event);
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_EMITTER_HPP
