/*
 * @file hid_event_listener.hpp
 * @date 7/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_HID_EVENT_LISTENER_HPP
#define XMOTION_HID_EVENT_LISTENER_HPP

#include <event2/event.h>

#include <memory>
#include <vector>
#include <thread>

#include "interface/driver/hid_handler_interface.hpp"

namespace xmotion {
class HidEventListener {
 public:
  HidEventListener();
  ~HidEventListener();

  // do not allow copy
  HidEventListener(const HidEventListener&) = delete;
  HidEventListener& operator=(const HidEventListener&) = delete;

  // public methods
  bool AddHidHandler(HidInputInterface* handler);
  void StartListening();

 private:
  static void EventCallback(evutil_socket_t fd, short events, void* arg);
  struct event_base* base_;
  std::vector<HidInputInterface*> handlers_;
  std::vector<struct event*> hid_events_;

  std::thread event_thread_;
};
}  // namespace xmotion

#endif  // XMOTION_HID_EVENT_LISTENER_HPP
