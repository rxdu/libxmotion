/*
 * @file event_dispatcher.cpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "event/event_dispatcher.hpp"

namespace xmotion {
EventDispatcher& EventDispatcher::GetInstance() {
  static EventDispatcher instance;
  return instance;
}

void EventDispatcher::RegisterHandler(const std::string& event_name,
                                      HandlerFunc handler) {
  handlers_[event_name].push_back(handler);
}

void EventDispatcher::Dispatch(std::shared_ptr<BaseEvent> event) const {
  if (event == nullptr) return;
  if (handlers_.find(event->GetName()) != handlers_.end()) {
    for (const auto& handler : handlers_.at(event->GetName())) {
      handler(event);
    }
  }
}
}  // namespace quickviz