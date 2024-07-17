/*
 * @file data_queue.hpp
 * @date 7/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_DATA_QUEUE_HPP
#define QUADRUPED_MOTION_DATA_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class DataQueue {
 public:
  // Add an element to the queue
  void Push(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
    condition_.notify_one();
  }

  // Remove and get the front element from the queue
  bool Pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !queue_.empty(); });
    if (queue_.empty()) {
      return false;
    }
    value = queue_.front();
    queue_.pop();
    return true;
  }

 private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

#endif  // QUADRUPED_MOTION_DATA_QUEUE_HPP
