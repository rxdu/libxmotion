/*
 * fixed_stand_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/fixed_stand_mode.hpp"

#include <cmath>

#include "logging/xlogger.hpp"

namespace xmotion {
FixedStandMode::FixedStandMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to FixedStandMode");

  context.robot_model->SetJointGains(
      context.system_config.ctrl_settings.fixed_stand_mode.default_joint_gains);

  joint_cmd_.q_dot = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.tau = QuadrupedModel::AllJointVar::Zero();

  auto current_state = context.robot_model->GetEstimatedState();
  for (int i = 0; i < 12; i++) {
    initial_state_.q[i] = current_state.q[i];
  }
  target_state_.q = context.system_config.ctrl_settings.fixed_stand_mode
                        .desired_joint_position;

  sw_.tic();
}

void FixedStandMode::Update(ControlContext& context) {
  //  XLOG_INFO("FixedStandMode::Update");

  double elapsed_ms = sw_.mtoc();
  double phase = std::tanh(
      elapsed_ms /
      context.system_config.ctrl_settings.fixed_stand_mode.duration_ms);

  auto desired_state = initial_state_;
  for (int i = 0; i < 12; i++) {
    desired_state.q[i] = initial_state_.q[i] +
                         phase * (target_state_.q[i] - initial_state_.q[i]);
    desired_state.q_dot[i] = 0;
    desired_state.tau[i] = 0;
  }
  joint_cmd_.q = desired_state.q;
  context.robot_model->SetJointCommand(joint_cmd_);

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();

  //  XLOG_INFO("FixedStandMode: Update end");
}
}  // namespace xmotion