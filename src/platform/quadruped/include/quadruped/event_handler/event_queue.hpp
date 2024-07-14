/*
 * event_queue.hpp
 *
 * Created on 7/8/24 9:24 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_EVENT_QUEUE_HPP
#define QUADRUPED_MOTION_EVENT_QUEUE_HPP

#include <string>
#include <queue>
#include <mutex>
#include <optional>

namespace xmotion {
template <typename T>
class EventQueue {
 public:
  void Push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
  }

  std::optional<T> TryPop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_EVENT_QUEUE_HPP
