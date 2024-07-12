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

  auto target_pos = context.system_config.ctrl_settings.fixed_stand_mode
                        .desired_joint_position;
  target_state_.q = target_pos;
  target_state_.q_dot = QuadrupedModel::JointVar::Zero();
  target_state_.tau = QuadrupedModel::JointVar::Zero();

  auto current_state = context.robot_model->GetEstimatedState();
  for (int i = 0; i < 12; i++) {
    initial_state_.q[i] = current_state.q[i];
  }

  sw_.tic();
}

void FixedStandMode::Update(ControlContext& context) {
  //  XLOG_INFO("FixedStandMode::Update");

  double elapsed_ms = sw_.mtoc();
  double phase = std::tanh(
      elapsed_ms /
      context.system_config.ctrl_settings.fixed_stand_mode.duration_ms);

  QuadrupedModel::State desired_state = initial_state_;
  for (int i = 0; i < 12; i++) {
    desired_state.q[i] = initial_state_.q[i] +
                         phase * (target_state_.q[i] - initial_state_.q[i]);
    desired_state.q_dot[i] = 0;
    desired_state.tau[i] = 0;

    //    desired_gains_.kp[i] = phase * 100.0 + (1 - phase) * 50.0;
    //    desired_gains_.kd[i] = 3.5;

    //    context.robot_model->SetJointGains(desired_gains_);
    context.robot_model->SetTargetState(desired_state);
  }

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();

  //  XLOG_INFO("FixedStandMode: Update end");
}
}  // namespace xmotion