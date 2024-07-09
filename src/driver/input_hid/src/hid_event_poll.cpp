/*
 * hid_event_poll.cpp
 *
 * Created on 7/6/24 12:22 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_hid/hid_event_poll.hpp"

#include <unistd.h>

namespace xmotion {
HidEventPoll::~HidEventPoll() { StopPolling(); }

void HidEventPoll::RegisterDevice(HidPollInterface *device) {
  devices_.insert(device);
}

void HidEventPoll::StartPolling() {
  keep_running_ = true;
  io_thread_ = std::thread([this]() {
    while (keep_running_) {
      for (auto device : devices_) {
        if (device != nullptr) {
          device->PollEvent();
          usleep(10000);
        }
      }
    }
  });
}

void HidEventPoll::StopPolling() {
  keep_running_ = false;
  if (io_thread_.joinable()) io_thread_.join();
}
}  // namespace xmotion