/*
 * hid_event_handler.cpp
 *
 * Created on 7/8/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/event_handler/hid_event_handler.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
HidEventHandler::HidEventHandler(const HidSettings& config) : config_(config) {
  if (config_.keyboard.enable) {
    keyboard_ = std::make_unique<Keyboard>(false);
    keyboard_->RegisterKeyEventCallback(std::bind(&HidEventHandler::OnKeyEvent,
                                                  this, std::placeholders::_1,
                                                  std::placeholders::_2));
    XLOG_INFO("Keyboard enabled: {}", config_.keyboard.device);
  }
  if (config_.joystick.enable) {
    joystick_ = std::make_unique<Joystick>(config_.joystick.device, false);
    XLOG_INFO("Joystick enabled: {}", config_.joystick.device);
  }
}

bool HidEventHandler::Initialize() {
  if (config_.keyboard.enable) {
    if (!keyboard_->StartMonitoring(config_.keyboard.device)) {
      XLOG_ERROR("Failed to start monitoring keyboard");
      return false;
    }
    hid_poll_.RegisterDevice(keyboard_.get());
  }
  if (config_.joystick.enable) {
    if (!joystick_->Open()) {
      XLOG_ERROR("Failed to open joystick");
      return false;
    }
    hid_poll_.RegisterDevice(joystick_.get());
  }
  return true;
}

void HidEventHandler::Start() { hid_poll_.StartPolling(); }

void HidEventHandler::Stop() { hid_poll_.StopPolling(); }

void HidEventHandler::PollEvents() {
  if (config_.joystick.enable) {
    std::cout << "Axes X: " << joystick_->GetAxisState(JsAxis::kX).value
              << ", Axes Y: " << joystick_->GetAxisState(JsAxis::kY).value
              << ", Axes Z: " << joystick_->GetAxisState(JsAxis::kZ).value
              << ", Axes RX: " << joystick_->GetAxisState(JsAxis::kRX).value
              << ", Axes RY: " << joystick_->GetAxisState(JsAxis::kRY).value
              << ", Axes RZ: " << joystick_->GetAxisState(JsAxis::kRZ).value
              << std::endl;
  }
}

void HidEventHandler::OnKeyEvent(KeyboardCode code, KeyboardEvent event) {
  //  XLOG_INFO("Key {} {}", Keyboard::GetKeyName(code),
  //            (event == KeyboardEvent::kPress ? "pressed" : "released"));
  kb_event_queue_.Push(HidEvent{code});
//  XLOG_INFO("Key event pushed to queue");
}

std::optional<HidEvent> HidEventHandler::TryPopJoystickEvent() {
  return js_event_queue_.TryPop();
}

std::optional<HidEvent> HidEventHandler::TryPopKeyboardEvent() {
  return kb_event_queue_.TryPop();
}
}  // namespace xmotion