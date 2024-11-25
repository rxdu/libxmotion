/*
 * @file async_event_dispatcher.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "event/async_event_dispatcher.hpp"

namespace xmotion {
AsyncEventDispatcher& AsyncEventDispatcher::GetInstance() {
  static AsyncEventDispatcher instance;
  return instance;
}

void AsyncEventDispatcher::RegisterHandler(const std::string& event_name,
                                           HandlerFunc handler) {
  std::lock_guard<std::mutex> lock(handler_mutex_);
  handlers_[event_name].push_back(handler);
}

void AsyncEventDispatcher::Dispatch(std::shared_ptr<BaseEvent> event) {
  // validate that Dispatch is called from a consistent thread
  if (dispatch_thread_id_ == std::thread::id()) {
    dispatch_thread_id_ = std::this_thread::get_id();
  } else if (std::this_thread::get_id() != dispatch_thread_id_) {
    throw std::runtime_error("Error: Dispatch called from multiple threads!");
  }

  // ensure Dispatch is not called from the same thread as HandleEvents
  if (std::this_thread::get_id() == handle_events_thread_id_) {
    throw std::runtime_error(
        "Error: Dispatch called from the same thread as HandleEvents!");
  }

  // push the event to the queue
  event_queue_.Push(event);
}

void AsyncEventDispatcher::HandleEvents() {
  // validate that HandleEvents is called from a consistent thread
  if (handle_events_thread_id_ == std::thread::id()) {
    handle_events_thread_id_ = std::this_thread::get_id();
  } else if (std::this_thread::get_id() != handle_events_thread_id_) {
    throw std::runtime_error(
        "Error: HandleEvents called from multiple threads!");
  }

  // ensure HandleEvents is not called from the same thread as Dispatch
  if (std::this_thread::get_id() == dispatch_thread_id_) {
    throw std::runtime_error(
        "Error: HandleEvents called from the same thread as Dispatch!");
  }

  std::shared_ptr<BaseEvent> event;
  while (event_queue_.TryPop(event)) {
    std::vector<HandlerFunc> event_handlers;
    // lock only while accessing the handlers_ map
    {
      std::lock_guard<std::mutex> hlock(handler_mutex_);
      if (handlers_.find(event->GetName()) != handlers_.end()) {
        event_handlers =
            handlers_[event->GetName()];  // Copy the list of handlers
      }
    }
    // execute handlers outside the lock
    for (const auto& handler : event_handlers) {
      handler(event);
    }
  }
}
}  // namespace quickviz