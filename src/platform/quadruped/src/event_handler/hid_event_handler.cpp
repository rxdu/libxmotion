/*
 * hid_event_handler.cpp
 *
 * Created on 7/8/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/event_handler/hid_event_handler.hpp"

namespace xmotion {
HidEventHandler::HidEventHandler(const HidConfig& config) : config_(config) {
  if (config_.keyboard.enable) {
    keyboard_ = std::make_unique<Keyboard>(false);
  }
  if (config_.joystick.enable) {
    joystick_ = std::make_unique<Joystick>(config_.joystick.device, false);
  }
}

bool HidEventHandler::Initialize() { return false; }

void HidEventHandler::Start() {}

void HidEventHandler::PollEvents() {}
}  // namespace xmotion