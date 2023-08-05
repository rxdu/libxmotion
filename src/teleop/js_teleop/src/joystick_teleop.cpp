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
#include <functional>

#include "input_joystick/joystick.hpp"

namespace xmotion {
JoystickTeleop::JoystickTeleop(const Config& config) : config_(config) {
  joystick_ = std::make_unique<Joystick>(config_.jd);
  vesc_ = std::make_unique<VescCanInterface>();
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
  vesc_->SetStateUpdatedCallback(std::bind(
      &JoystickTeleop::VescStateUpdatedCallback, this, std::placeholders::_1));
  if (!vesc_->Connect(config_.can_if_name, config_.vesc_id)) {
    std::cerr << "Failed to connect to " << config_.can_if_name << std::endl;
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

    // set control source

    // send command to vesc
    if (control_source_ != ControlSource::kNone) {
      //    vesc_->SetServo(0.5);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(rate_ms));
  }
}

void JoystickTeleop::VescStateUpdatedCallback(const StampedVescState& state) {
  std::cout << "voltage input: " << state.state.voltage_input << ", "
            << "temp pcb: " << state.state.temperature_pcb << ", "
            << "current motor: " << state.state.current_motor << ", "
            << "current input: " << state.state.current_input << ", "
            << "speed: " << state.state.speed << ", "
            << "duty cycle: " << state.state.duty_cycle << ", "
            << "charge drawn: " << state.state.charge_drawn << ", "
            << "charge regen: " << state.state.charge_regen << ", "
            << "charge drawn: " << state.state.energy_drawn << ", "
            << "charge regen: " << state.state.energy_regen << ", "
            << "displacement: " << state.state.displacement << ", "
            << "dist traveled: " << state.state.distance_traveled << std::endl;
}
}  // namespace xmotion