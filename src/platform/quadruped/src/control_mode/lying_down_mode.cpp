/*
 * lying_down_mode.cpp
 *
 * Created on 7/11/24 10:58 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/lying_down_mode.hpp"

#include <cmath>

#include "logging/xlogger.hpp"

namespace xmotion {
LyingDownMode::LyingDownMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to LyingDownMode");

  context.robot_model->SetJointGains(
      context.system_config.ctrl_settings.lying_down_mode.default_joint_gains);

  joint_cmd_.q_dot = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.tau = QuadrupedModel::AllJointVar::Zero();

  auto current_state = context.robot_model->GetEstimatedState();
  for (int i = 0; i < 12; i++) {
    initial_state_.q[i] = current_state.q[i];
  }
  target_state_.q = context.system_config.ctrl_settings.lying_down_mode
                        .desired_joint_position;

  sw_.tic();
}

void LyingDownMode::Update(ControlContext& context) {
  //  XLOG_INFO("LyingDownMode::Update");

  double elapsed_ms = sw_.mtoc();
  double phase = std::tanh(
      elapsed_ms /
      context.system_config.ctrl_settings.lying_down_mode.duration_ms);

  QuadrupedModel::State desired_state = initial_state_;
  for (int i = 0; i < 12; i++) {
    desired_state.q[i] = initial_state_.q[i] +
                         phase * (target_state_.q[i] - initial_state_.q[i]);
  }
  joint_cmd_.q = desired_state.q;
  context.robot_model->SetJointCommand(joint_cmd_);

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();

  //  XLOG_INFO("LyingDownMode: Update end");
}
}  // namespace xmotion