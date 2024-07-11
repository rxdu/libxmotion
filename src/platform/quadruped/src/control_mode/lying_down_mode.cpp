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

  auto desired_gains =
      context.system_config.ctrl_settings.lying_down_mode.joint_gains;
  context.robot_model->SetJointGains(desired_gains);

  //  std::cout << "kp: " << desired_gains.kp.transpose() << std::endl;
  //  std::cout << "kd: " << desired_gains.kd.transpose() << std::endl;

  auto target_pos = context.system_config.ctrl_settings.lying_down_mode
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
    desired_state.q_dot[i] = 0;
    desired_state.tau[i] = 0;

    context.robot_model->SetTargetState(desired_state);
  }

  // finally send the command to the robot
  context.robot_model->SendCommandToRobot();

  //  XLOG_INFO("LyingDownMode: Update end");
}
}  // namespace xmotion