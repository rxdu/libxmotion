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
  QuadrupedModel::JointGains joint_gains;
  auto passive_mode_gains =
      context.system_config.ctrl_settings.passive_mode.joint_gains;
  for (int i = 0; i < 12; ++i) {
    joint_gains.kp[i] = passive_mode_gains.kp[0];
    joint_gains.kd[i] = passive_mode_gains.kd[0];
    //    XLOG_INFO("Joint {} kp: {}, kd: {}", i, joint_gains.kp[i],
    //              joint_gains.kd[i]);

    target_state_.q[i] = 0;
    target_state_.q_dot[i] = 0;
    target_state_.tau[i] = 0;
  }
  context.robot_model->SetJointGains(joint_gains);
  context.robot_model->SetTargetState(target_state_);
}

void PassiveMode::Update(ControlContext& context) {
  //  XLOG_INFO("PassiveMode::Update");

  context.robot_model->SendCommandToRobot();

  //  XLOG_INFO("PassiveMode: Update end");
}
}  // namespace xmotion