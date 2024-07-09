/*
 * @file hid_event_poll.hpp
 * @date 7/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_HID_EVENT_POLL_HPP
#define QUADRUPED_MOTION_HID_EVENT_POLL_HPP

#include <set>
#include <thread>
#include <atomic>

#include "interface/driver/hid_poll_interface.hpp"

namespace xmotion {
class HidEventPoll {
 public:
  HidEventPoll() = default;
  ~HidEventPoll();

  void RegisterDevice(HidPollInterface* device);

  void StartPolling();
  void StopPolling();

 private:
  std::set<HidPollInterface*> devices_;

  std::thread io_thread_;
  std::atomic<bool> keep_running_{false};
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_HID_EVENT_POLL_HPP
