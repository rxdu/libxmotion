/*
 * passive_mode.cpp
 *
 * Created on 7/4/24 11:30 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/control_mode/passive_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
PassiveMode::PassiveMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to PassiveMode");
  context.robot_model->SetJointGains(
      context.system_config.ctrl_settings.passive_mode.default_joint_gains);

  joint_cmd_.q = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.q_dot = QuadrupedModel::AllJointVar::Zero();
  joint_cmd_.tau = QuadrupedModel::AllJointVar::Zero();
  context.robot_model->SetJointCommand(joint_cmd_);
}

void PassiveMode::Update(ControlContext& context) {
  //  XLOG_INFO("PassiveMode::Update");

  context.robot_model->SendCommandToRobot();

  //  XLOG_INFO("PassiveMode: Update end");
}
}  // namespace xmotion