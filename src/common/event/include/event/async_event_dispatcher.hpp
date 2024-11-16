/*
 * @file async_event_dispatcher.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP
#define QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <string>
#include <queue>
#include <atomic>

#include "event/event.hpp"
#include "event/thread_safe_queue.hpp"

namespace xmotion {
class AsyncEventDispatcher {
 public:
  using HandlerFunc = std::function<void(std::shared_ptr<BaseEvent>)>;

  // public interface
  static AsyncEventDispatcher& GetInstance();
  void RegisterHandler(const std::string& event_name, HandlerFunc handler);
  void Dispatch(std::shared_ptr<BaseEvent> event);
  void HandleEvents();

 private:
  AsyncEventDispatcher() = default;

  std::mutex handler_mutex_;
  std::unordered_map<std::string, std::vector<HandlerFunc>> handlers_;
  ThreadSafeQueue<std::shared_ptr<BaseEvent>> event_queue_;

  std::atomic<std::thread::id> dispatch_thread_id_{};
  std::atomic<std::thread::id> handle_events_thread_id_{};
};
}  // namespace quickviz

#endif  // QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP