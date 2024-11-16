/* 
 * pid_controller.cpp
 *
 * Created on 4/11/22 11:33 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "pid/pid_controller.hpp"

#include <cmath>

namespace xmotion {
PidController::PidController(float kp, float ki, float kd, float umax, float ts)
    : kp_(kp), ki_(ki), kd_(kd), u_max_(umax), ts_(ts) {}

float PidController::Update(float reference, float measurement) {
  float error = reference - measurement;
  float derivative = (error - error_last_) / ts_;
  float new_integral = integral_ + error * ts_;

  float u = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // anti-windup
  if (u > u_max_) {
    u = u_max_;
  } else if (u < -u_max_) {
    u = -u_max_;
  } else {
    integral_ = new_integral;
  }

  error_last_ = error;

  return u;
}
}