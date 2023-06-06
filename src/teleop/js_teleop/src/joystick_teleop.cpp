/*
 * joystick_teleop.cpp
 *
 * Created on 6/6/23 8:54 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "js_teleop/joystick_teleop.hpp"

#include <iostream>

#include "input_joystick/joystick.hpp"

namespace xmotion {
JoystickTeleop::JoystickTeleop(const Config& config) : config_(config) {
  joystick_ = std::make_unique<Joystick>(config_.jd);
}

JoystickTeleop::~JoystickTeleop() {
  if (joystick_->IsOpened()) {
    joystick_->Close();
  }
}

bool JoystickTeleop::Initialize() {
  if (!joystick_->Open()) {
    std::cout << "Failed to open joystick" << std::endl;
    return false;
  }
  return true;
}

void JoystickTeleop::Run(int rate_hz) {
  int rate_ms = 1000 / rate_hz;

  while (joystick_->IsOpened()) {
    std::cout << "Axes X: " << joystick_->GetAxisState(JsAxis::kX).value
              << ", Axes Y: " << joystick_->GetAxisState(JsAxis::kY).value
              << ", Axes Z: " << joystick_->GetAxisState(JsAxis::kZ).value
              << ", Axes RX: " << joystick_->GetAxisState(JsAxis::kRX).value
              << ", Axes RY: " << joystick_->GetAxisState(JsAxis::kRY).value
              << ", Axes RZ: " << joystick_->GetAxisState(JsAxis::kRZ).value
              << std::endl;

    if (joystick_->GetButtonState(JsButton::kMode)) {
      joystick_->SetJoystickRumble(0.1 * 0xFFFF, 0.5 * 0xFFFF);
    } else {
      joystick_->SetJoystickRumble(0.0 * 0xFFFF, 0.0 * 0xFFFF);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(rate_ms));
  }
}
}  // namespace xmotion