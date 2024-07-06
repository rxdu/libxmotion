/*
 * unitree_motor.cpp
 *
 * Created on 7/6/24 9:09 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/robot_model/unitree_motor.hpp"

namespace xmotion {
void UnitreeMotor::SetMode(Mode mode) {
  mode_ = mode;
  if (mode_ == Mode::kIdle) {
    state_.q = 0;
    state_.dq = 0;
  } else {
    state_.q = q_idle_target;
    state_.dq = qd_idle_target;
  }
  state_.tau = 0;
  state_.kp = 0;
  state_.kd = 0;
}

void UnitreeMotor::SetGains(float kp, float kd) {
  state_.kp = kp;
  state_.kd = kd;
}

void UnitreeMotor::SetTarget(float q, float dq, float tau) {
  state_.q = q;
  state_.dq = dq;
  state_.tau = tau;
}

UnitreeMotor::CmdMsg UnitreeMotor::ToCmdMsg() {
  CmdMsg msg;
  msg.mode() = static_cast<uint8_t>(mode_);
  msg.q() = state_.q;
  msg.dq() = state_.dq;
  msg.tau() = state_.tau;
  msg.kp() = state_.kp;
  msg.kd() = state_.kd;
  return msg;
}
}  // namespace xmotion