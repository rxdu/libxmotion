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
HidEventHandler::HidEventHandler(const HidSettings& config)
    : config_(config), keyboard_handler_(config.keyboard.device) {
  if (config_.keyboard.enable) {
    keyboard_handler_.RegisterKeyEventCallback(
        std::bind(&HidEventHandler::OnKeyEvent, this, std::placeholders::_1,
                  std::placeholders::_2));
    XLOG_INFO("Keyboard enabled: {}", config_.keyboard.device);
  }
  if (config_.joystick.enable) {
    //    joystick_ = std::make_unique<Joystick>(config_.joystick.device,
    //    false); XLOG_INFO("Joystick enabled: {}", config_.joystick.device);
  }
}

bool HidEventHandler::Initialize() {
  if (config_.keyboard.enable) {
    if (!keyboard_handler_.Open()) {
      XLOG_ERROR("Failed to start monitoring keyboard");
      return false;
    }
    if (!hid_event_listener_.AddHidHandler(&keyboard_handler_)) {
      XLOG_ERROR("Failed to add keyboard handler to event listener");
      return false;
    }
  }
  if (config_.joystick.enable) {
    //    if (!joystick_->Open()) {
    //      XLOG_ERROR("Failed to open joystick");
    //      return false;
    //    }
    //    hid_poll_.RegisterDevice(joystick_.get());
  }
  return true;
}

void HidEventHandler::Start() { hid_event_listener_.StartListening(); }

// void HidEventHandler::PollEvents() {
//   if (config_.joystick.enable) {
//     std::cout << "Axes X: " << joystick_->GetAxisState(JsAxis::kX).value
//               << ", Axes Y: " << joystick_->GetAxisState(JsAxis::kY).value
//               << ", Axes Z: " << joystick_->GetAxisState(JsAxis::kZ).value
//               << ", Axes RX: " << joystick_->GetAxisState(JsAxis::kRX).value
//               << ", Axes RY: " << joystick_->GetAxisState(JsAxis::kRY).value
//               << ", Axes RZ: " << joystick_->GetAxisState(JsAxis::kRZ).value
//               << std::endl;
//   }
// }

void HidEventHandler::OnKeyEvent(KeyboardCode code, KeyboardEvent event) {
  //  XLOG_INFO("Key {} {}", KeyboardMapping::GetKeyName(code),
  //            (event == KeyboardEvent::kPress ? "pressed" : "released"));
  if (event == KeyboardEvent::kPress) {
    //    XLOG_INFO("Key pressed: {}", KeyboardMapping::GetKeyName(code));
    if (config_.keyboard.keyboard_mappings.find(code) !=
        config_.keyboard.keyboard_mappings.end()) {
      auto key_func = config_.keyboard.keyboard_mappings.at(code);
      if (key_func > HidSettings::KeyFunction::kFirstModeKey &&
          key_func < HidSettings::KeyFunction::kLastModeKey) {
        //        XLOG_INFO("Key pressed: {}",
        //        KeyboardMapping::GetKeyName(code));
        kb_mode_switch_queue_.Push(HidEvent{code});
      }
      if (key_func > HidSettings::KeyFunction::kFirstControlKey &&
          key_func < HidSettings::KeyFunction::kLastControlKey) {
        //        XLOG_INFO("Key pressed: {}",
        //        KeyboardMapping::GetKeyName(code));
        kb_control_input_queue_.Push(HidEvent{code});
      }
    }
  }
  //  XLOG_INFO("Key event pushed to queue");
}

std::optional<HidEvent> HidEventHandler::TryPopJoystickEvent() {
  return js_event_queue_.TryPop();
}

std::optional<HidEvent> HidEventHandler::TryPopKeyboardEvent(
    KeyboardEventType type) {
  if (type == KeyboardEventType::kModeSwitch) {
    return kb_mode_switch_queue_.TryPop();
  } else if (type == KeyboardEventType::kControlInput) {
    return kb_control_input_queue_.TryPop();
  }
  return std::nullopt;
}
}  // namespace xmotion