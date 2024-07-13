/*
 * swing_test_mode.cpp
 *
 * Created on 7/12/24 10:16 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/swing_test_mode.hpp"

#include "quadruped/utils.hpp"
#include "logging/xlogger.hpp"

namespace xmotion {
SwingTestMode::SwingTestMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to SwingTestMode");

  context.robot_model->SetJointGains(
      context.system_config.ctrl_settings.fixed_stand_mode.default_joint_gains);

  auto current_state = context.robot_model->GetEstimatedState();
}

void SwingTestMode::HandleKeyboardInput(ControlContext& context) {
  auto key_func = Utils::PollKeyFunction(
      context, HidEventHandler::KeyboardEventType::kControlInput);
  if (key_func.has_value()) {
    if (key_func.value() == HidSettings::KeyFunction::kLeftStickUp) {
      XLOG_INFO("SwingTestMode: LeftStickUp");
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickDown) {
      XLOG_INFO("SwingTestMode: LeftStickDown");
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickLeft) {
      XLOG_INFO("SwingTestMode: LeftStickLeft");
    } else if (key_func.value() == HidSettings::KeyFunction::kLeftStickRight) {
      XLOG_INFO("SwingTestMode: LeftStickRight");
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickUp) {
      XLOG_INFO("SwingTestMode: RightStickUp");
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickDown) {
      XLOG_INFO("SwingTestMode: RightStickDown");
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickLeft) {
      XLOG_INFO("SwingTestMode: RightStickLeft");
    } else if (key_func.value() == HidSettings::KeyFunction::kRightStickRight) {
      XLOG_INFO("SwingTestMode: RightStickRight");
    }
  }
}

void SwingTestMode::Update(ControlContext& context) {
  HandleKeyboardInput(context);
}
}  // namespace xmotion