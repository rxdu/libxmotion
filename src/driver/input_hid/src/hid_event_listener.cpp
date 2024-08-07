/*
 * hid_event_listener.cpp
 *
 * Created on 7/15/24 9:19 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <stdexcept>

#include <linux/input.h>

#include "input_hid/hid_event_listener.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
HidEventListener::HidEventListener() {
  base_ = event_base_new();
  if (base_ == nullptr) {
    throw std::runtime_error("HidEventListener: failed to create event base");
  }
}

HidEventListener::~HidEventListener() {
  event_base_loopbreak(base_);
  if (event_thread_.joinable()) {
    event_thread_.join();
  }

  for (auto event : hid_events_) {
    event_free(event);
  }
  event_base_free(base_);
}

void HidEventListener::EventCallback(evutil_socket_t fd, short events,
                                     void *arg) {
  HidInputInterface *handler = static_cast<HidInputInterface *>(arg);
  handler->OnInputEvent();
}

bool HidEventListener::AddHidHandler(HidInputInterface *handler) {
  auto fd = handler->GetFd();
  if (fd < 0) {
    XLOG_ERROR("HidEventListener: invalid file descriptor");
    return false;
  }
  struct event *hid_event =
      event_new(base_, fd, EV_READ | EV_PERSIST, EventCallback, handler);
  if (!hid_event) {
    XLOG_ERROR("HidEventListener: failed to create event");
    return false;
  }

  if (event_add(hid_event, NULL) < 0) {
    XLOG_ERROR("HidEventListener: failed to add event");
    event_free(hid_event);
    return false;
  }
  hid_events_.push_back(hid_event);

  return true;
}

void HidEventListener::StartListening() {
  event_thread_ = std::thread([this]() { event_base_dispatch(base_); });
}
}  // namespace xmotion